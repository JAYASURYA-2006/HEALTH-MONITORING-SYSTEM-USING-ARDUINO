// 3 x 3.7 V battery pack (11.1 V nominal): positive terminal = logic high, negative = ground.
#include "wokwi-api.h"

void chip_init(void) {
  pin_init("POS", OUTPUT_HIGH);
  pin_init("NEG", OUTPUT_LOW);
}