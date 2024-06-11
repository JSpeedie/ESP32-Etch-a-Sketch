#ifndef __ETCH_A_SKETCH_H_
#define __ETCH_A_SKETCH_H_

#include <inttypes.h>


struct game_state {
	int16_t cursor_x; /* These are stored in pixels (px) and range from 0 <= x <= 127 */
	int16_t cursor_y;
};


#endif
