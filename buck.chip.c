// Buck converter model: VOUT (regulated 5 V rail) is high only while VIN (from the switch) is high.
#include "wokwi-api.h"
#include <stdlib.h>

typedef struct {
  pin_t vin;
  pin_t vout;
} chip_state_t;

static void on_vin_change(void *user_data, pin_t pin, uint32_t value) {
  chip_state_t *c = (chip_state_t *)user_data;
  pin_write(c->vout, value ? HIGH : LOW);
}

void chip_init(void) {
  chip_state_t *c = (chip_state_t *)malloc(sizeof(chip_state_t));
  c->vin  = pin_init("VIN", INPUT_PULLDOWN);
  c->vout = pin_init("VOUT", OUTPUT_LOW);
  pin_init("GND", INPUT);

  pin_watch_config_t cfg = {
    .edge = BOTH,
    .pin_change = on_vin_change,
    .user_data = c
  };
  pin_watch(c->vin, &cfg);
  pin_write(c->vout, pin_read(c->vin));
}