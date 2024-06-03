#ifndef __ETCH_A_SKETCH_
#define __ETCH_A_SKETCH_

#include <inttypes.h>

#include "freertos/semphr.h"

#include "esp32-spi-ssd1327.h"


struct game_state {
	int16_t cursor_x; /* These are stored in pixels (px) and range from 0 <= x <= 127 */
	int16_t cursor_y;
};


#endif
