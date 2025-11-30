// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


//Nes stuff wants to define this as well...
#undef false
#undef true
#undef bool


#include <math.h>
#include <string.h>
#include <stdint.h>

#include <Runtime/Kernel.h>
#include <Runtime/Video.h>

#include <noftypes.h>
#include <bitmap.h>
#include <nofconfig.h>
#include <event.h>
#include <gui.h>
#include <log.h>
#include <nes.h>
#include <nes_pal.h>
#include <nesinput.h>
#include <osd.h>
#include <psxcontroller.h>

#define  DEFAULT_SAMPLERATE   22100
#define  DEFAULT_FRAGSIZE     128

#define  DEFAULT_WIDTH        256
#define  DEFAULT_HEIGHT       NES_VISIBLE_HEIGHT


// TimerHandle_t timer;

static void (*s_timer_isr)(void) = 0;

static void rd_thread_timer()
{
	for (;;)
	{
		if (s_timer_isr)
			(*s_timer_isr)();

		rt_kernel_sleep(16);
	}
}


//Seemingly, this will be called only once. Should call func with a freq of frequency,
int osd_installtimer(int frequency, void *func, int funcsize, void *counter, int countersize)
{
	printf("Timer install, freq=%d\n", frequency);

	s_timer_isr = func;
	rt_kernel_create_thread(rd_thread_timer);

	// timer=xTimerCreate("nes",configTICK_RATE_HZ/frequency, pdTRUE, NULL, func);
	// xTimerStart(timer, 0);
	return 0;
}


/*
** Audio
*/
static void (*audio_callback)(void *buffer, int length) = NULL;
#if CONFIG_SOUND_ENA
QueueHandle_t queue;
static uint16_t *audio_frame;
#endif

static void do_audio_frame() {

#if CONFIG_SOUND_ENA
	int left=DEFAULT_SAMPLERATE/NES_REFRESH_RATE;
	while(left) {
		int n=DEFAULT_FRAGSIZE;
		if (n>left) n=left;
		audio_callback(audio_frame, n); //get more data
		//16 bit mono -> 32-bit (16 bit r+l)
		for (int i=n-1; i>=0; i--) {
			audio_frame[i*2+1]=audio_frame[i];
			audio_frame[i*2]=audio_frame[i];
		}
		i2s_write_bytes(0, audio_frame, 4*n, portMAX_DELAY);
		left-=n;
	}
#endif
}

void osd_setsound(void (*playfunc)(void *buffer, int length))
{
   //Indicates we should call playfunc() to get more data.
   audio_callback = playfunc;
}

static void osd_stopsound(void)
{
   audio_callback = NULL;
}


static int osd_init_sound(void)
{
#if CONFIG_SOUND_ENA
	audio_frame=malloc(4*DEFAULT_FRAGSIZE);
	i2s_config_t cfg={
		.mode=I2S_MODE_DAC_BUILT_IN|I2S_MODE_TX|I2S_MODE_MASTER,
		.sample_rate=DEFAULT_SAMPLERATE,
		.bits_per_sample=I2S_BITS_PER_SAMPLE_16BIT,
		.channel_format=I2S_CHANNEL_FMT_RIGHT_LEFT,
		.communication_format=I2S_COMM_FORMAT_I2S_MSB,
		.intr_alloc_flags=0,
		.dma_buf_count=4,
		.dma_buf_len=512
	};
	i2s_driver_install(0, &cfg, 4, &queue);
	i2s_set_pin(0, NULL);
	i2s_set_dac_mode(I2S_DAC_CHANNEL_LEFT_EN); 

	//I2S enables *both* DAC channels; we only need DAC1.
	//ToDo: still needed now I2S supports set_dac_mode?
	CLEAR_PERI_REG_MASK(RTC_IO_PAD_DAC1_REG, RTC_IO_PDAC1_DAC_XPD_FORCE_M);
	CLEAR_PERI_REG_MASK(RTC_IO_PAD_DAC1_REG, RTC_IO_PDAC1_XPD_DAC_M);

#endif

	audio_callback = NULL;

	return 0;
}

void osd_getsoundinfo(sndinfo_t *info)
{
   info->sample_rate = DEFAULT_SAMPLERATE;
   info->bps = 16;
}

/*
** Video
*/

static int init(int width, int height);
static void shutdown(void);
static int set_mode(int width, int height);
static void set_palette(rgb_t *pal);
static void clear(uint8 color);
static bitmap_t *lock_write(void);
static void free_write(int num_dirties, rect_t *dirty_rects);
static void custom_blit(bitmap_t *bmp, int num_dirties, rect_t *dirty_rects);

static uint8_t* frameBuffer[2];
bitmap_t *myBitmap;

viddriver_t sdlDriver =
{
   "Simple DirectMedia Layer",         /* name */
   init,          /* init */
   shutdown,      /* shutdown */
   set_mode,      /* set_mode */
   set_palette,   /* set_palette */
   clear,         /* clear */
   lock_write,    /* lock_write */
   free_write,    /* free_write */
   custom_blit,   /* custom_blit */
   false          /* invalidate flag */
};

void osd_getvideoinfo(vidinfo_t *info)
{
   info->default_width = DEFAULT_WIDTH;
   info->default_height = DEFAULT_HEIGHT;
   info->driver = &sdlDriver;
}

/* flip between full screen and windowed */
void osd_togglefullscreen(int code)
{
}

/* initialise video */
static int init(int width, int height)
{
	frameBuffer[0] = rt_video_create_target();
	frameBuffer[1] = rt_video_create_target();

	memset(frameBuffer[0], 0, rt_video_get_resolution_width() * rt_video_get_resolution_height());
	memset(frameBuffer[1], 0, rt_video_get_resolution_width() * rt_video_get_resolution_height());

	myBitmap = bmp_createhw(
		frameBuffer[0],
		DEFAULT_WIDTH,
		256,
		rt_video_get_resolution_width()
	);
	return 0;
}

static void shutdown(void)
{
   bmp_destroy(&myBitmap);
}

/* set a video mode */
static int set_mode(int width, int height)
{
   return 0;
}

// uint16 myPalette[256];

/* copy nes palette over to hardware */
static void set_palette(rgb_t *pal)
{
	// uint16 c;

   int i;

   for (i = 0; i < 256; i++)
   {
    //   c=(pal[i].b>>3)+((pal[i].g>>2)<<5)+((pal[i].r>>3)<<11);
    //   //myPalette[i]=(c>>8)|((c&0xff)<<8);
    //   myPalette[i]=c;

	  rt_video_set_palette(
		i,
		(pal[i].r << 16) | (pal[i].g << 8) | (pal[i].b)
	  );
   }

}

/* clear all frames to a particular color */
static void clear(uint8 color)
{
//   SDL_FillRect(mySurface, 0, color);
}



/* acquire the directbuffer for writing */
static bitmap_t *lock_write(void)
{
	return myBitmap;
}

/* release the resource */
static void free_write(int num_dirties, rect_t *dirty_rects)
{
}


static void custom_blit(bitmap_t *bmp, int num_dirties, rect_t *dirty_rects)
{
	rt_video_wait();
	rt_video_present(0);

	if (bmp->data == frameBuffer[0])
	{
		rt_video_blit(frameBuffer[0]);
		bmp->data = frameBuffer[1];
	}
	else
	{
		rt_video_blit(frameBuffer[1]);
		bmp->data = frameBuffer[0];
	}

	bmp->line[0] = bmp->data;
	for (int i = 1; i < bmp->height; i++)
		bmp->line[i] = bmp->line[i - 1] + bmp->pitch;
}

/*
** Input
*/

static void osd_initinput()
{
	psxcontrollerInit();
}

void osd_getinput(void)
{
// 	const int ev[16]={
// 			event_joypad1_select,0,0,event_joypad1_start,event_joypad1_up,event_joypad1_right,event_joypad1_down,event_joypad1_left,
// 			0,0,0,0,event_soft_reset,event_joypad1_a,event_joypad1_b,event_hard_reset
// 		};
// 	static int oldb=0xffff;
// 	int b=psxReadInput();
// 	int chg=b^oldb;
// 	int x;
// 	oldb=b;
// 	event_t evh;
// //	printf("Input: %x\n", b);
// 	for (x=0; x<16; x++) {
// 		if (chg&1) {
// 			evh=event_get(ev[x]);
// 			if (evh) evh((b&1)?INP_STATE_BREAK:INP_STATE_MAKE);
// 		}
// 		chg>>=1;
// 		b>>=1;
// 	}
}

static void osd_freeinput(void)
{
}

void osd_getmouse(int *x, int *y, int *button)
{
}

/*
** Shutdown
*/

/* this is at the bottom, to eliminate warnings */
void osd_shutdown()
{
	osd_stopsound();
	osd_freeinput();
}

static int logprint(const char *string)
{
   return printf("%s", string);
}

/*
** Startup
*/

int osd_init()
{
	log_chain_logfunc(logprint);

	if (osd_init_sound())
		return -1;

	rt_video_set_mode(VMODE_360_360_8);
    // for (int i = 0; i < sizeof(c_paletteColors) / sizeof(c_paletteColors[0]); ++i)
    // {
    //     const uint8_t* p = (const uint8_t*)&c_paletteColors[i];
    //     const uint32_t rgb = (p[3] << 16) | (p[2] << 8) | (p[1] << 0);
    //     rt_video_set_palette(i, rgb);
    // }

	// ili9341_init();
	// ili9341_write_frame(0,0,320,240,NULL);
	// vidQueue=xQueueCreate(1, sizeof(bitmap_t *));
	// xTaskCreatePinnedToCore(&videoTask, "videoTask", 2048, NULL, 5, NULL, 1);

	osd_initinput();
	return 0;
}
