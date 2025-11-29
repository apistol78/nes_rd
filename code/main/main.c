#include <stdio.h>
#include <stdlib.h>

#include <Runtime/Runtime.h>

#include "nofrendo.h"

char *osd_getromdata()
{
	FILE* file;
	uint8_t* mod_data;

	if (!(file = fopen("mario.nes", "rb")))
		return 0;

	fseek(file, 0, SEEK_END);
	const int32_t mod_size = ftell(file);
	rewind(file);

	if (!(mod_data = malloc(mod_size)))
		return 0;
	else if (!fread(mod_data, mod_size, 1, file))
		return 0;

	fclose(file);

	return mod_data;
}

int main(void)
{
	runtime_init();
	nofrendo_main(0, 0);
    return 0;
}

