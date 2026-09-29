// Simulated MAX30102 pulse oximeter: I2C slave at 0x57.
// Generates representative RED/IR PPG samples at 100 Hz into a 32-entry FIFO.
// This is NOT a physical optical simulation.

#include "wokwi-api.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#define I2C_ADDR          0x57
#define FIFO_DEPTH        32
#define FINGER_DELAY_TICK 500      /* 5 s after measurement starts */

#define REG_FIFO_WR   0x04
#define REG_FIFO_OVF  0x05
#define REG_FIFO_RD   0x06
#define REG_FIFO_DATA 0x07
#define REG_MODE      0x09
#define REG_REV_ID    0xFE
#define REG_PART_ID   0xFF

#define IR_DC   50000.0f
#define RED_DC  42000.0f
#define IR_AC   2000.0f
#define W_MEAN  0.53333f           /* mean of ((4x(1-x))^2) over one cycle */

typedef struct {
  pin_t pin_scl, pin_sda, pin_vcc, pin_int;
  uint8_t regs[256];
  uint8_t reg_ptr;
  bool expect_reg;
  uint32_t fifo_red[FIFO_DEPTH];
  uint32_t fifo_ir[FIFO_DEPTH];
  uint8_t wr_ptr, rd_ptr, byte_idx;
  uint32_t rng;
  uint32_t tick;
  float phase, bpm_cur, bpm_tgt, spo2_cur, spo2_tgt;
} chip_state_t;

static uint32_t rnd(chip_state_t *c) {
  uint32_t x = c->rng;
  x ^= x << 13;
  x ^= x >> 17;
  x ^= x << 5;
  c->rng = x;
  return x;
}

static void reset_device(chip_state_t *c) {
  for (int i = 0; i < 256; i++) c->regs[i] = 0;
  for (int i = 0; i < FIFO_DEPTH; i++) { c->fifo_red[i] = 0; c->fifo_ir[i] = 0; }
  c->reg_ptr = 0;
  c->expect_reg = true;
  c->wr_ptr = 0;
  c->rd_ptr = 0;
  c->byte_idx = 0;
  c->tick = 0;
  c->phase = 0.0f;
  c->bpm_cur = 72.0f;
  c->bpm_tgt = 72.0f;
  c->spo2_cur = 98.0f;
  c->spo2_tgt = 98.0f;
}

/* ---------- 100 Hz sample generator ---------- */
static void on_sample_timer(void *user_data) {
  chip_state_t *c = (chip_state_t *)user_data;

  if (pin_read(c->pin_vcc) == LOW) {   /* no power -> device loses state */
    reset_device(c);
    return;
  }
  if ((c->regs[REG_MODE] & 0x07) == 0) return;   /* not measuring */

  c->tick++;

  /* every 4 s pick a new small target (realistic drift, no big jumps) */
  if (c->tick % 400 == 0) {
    c->bpm_tgt += (float)((int)(rnd(c) % 7) - 3);     /* -3 .. +3 */
    if (c->bpm_tgt < 66.0f) c->bpm_tgt = 66.0f;
    if (c->bpm_tgt > 79.0f) c->bpm_tgt = 79.0f;
    c->spo2_tgt += (float)((int)(rnd(c) % 3) - 1);    /* -1 .. +1 */
    if (c->spo2_tgt < 96.0f) c->spo2_tgt = 96.0f;
    if (c->spo2_tgt > 99.0f) c->spo2_tgt = 99.0f;
  }
  /* slew smoothly toward the targets */
  c->bpm_cur  += (c->bpm_tgt  - c->bpm_cur)  * 0.004f;
  c->spo2_cur += (c->spo2_tgt - c->spo2_cur) * 0.003f;

  c->phase += c->bpm_cur / 6000.0f;    /* bpm / 60 / 100 Hz */
  if (c->phase >= 1.0f) c->phase -= 1.0f;

  int noise_r = (int)(rnd(c) % 7) - 3;
  int noise_i = (int)(rnd(c) % 7) - 3;
  float red_f, ir_f;

  if (c->tick < FINGER_DELAY_TICK) {
    /* startup period: no finger on sensor -> very low reflected light */
    ir_f  = 2500.0f + noise_i;
    red_f = 2000.0f + noise_r;
  } else {
    float x = c->phase;
    float b = 4.0f * x * (1.0f - x);
    float w = b * b;                    /* pulse-shaped waveform 0..1 */

    float R  = (110.0f - c->spo2_cur) / 25.0f;          /* ratio of ratios */
    float k  = IR_AC / (IR_DC + W_MEAN * IR_AC);
    float a  = (R * k * RED_DC) / (1.0f - W_MEAN * R * k);

    ir_f  = IR_DC  + IR_AC * w + noise_i;
    red_f = RED_DC + a * w     + noise_r;
  }

  uint32_t red = (uint32_t)(red_f + 0.5f) & 0x3FFFF;
  uint32_t ir  = (uint32_t)(ir_f  + 0.5f) & 0x3FFFF;

  c->fifo_red[c->wr_ptr] = red;
  c->fifo_ir[c->wr_ptr]  = ir;
  c->wr_ptr = (c->wr_ptr + 1) & (FIFO_DEPTH - 1);
  if (c->wr_ptr == c->rd_ptr) c->rd_ptr = (c->rd_ptr + 1) & (FIFO_DEPTH - 1);
}

/* ---------- I2C slave ---------- */
static bool on_i2c_connect(void *user_data, uint32_t address, bool read) {
  chip_state_t *c = (chip_state_t *)user_data;
  if (pin_read(c->pin_vcc) == LOW) return false;   /* unpowered -> NACK */
  if (!read) c->expect_reg = true;
  return true;
}

static uint8_t on_i2c_read(void *user_data) {
  chip_state_t *c = (chip_state_t *)user_data;
  uint8_t r = c->reg_ptr;
  uint8_t v = 0;

  switch (r) {
    case REG_FIFO_WR:  v = c->wr_ptr; break;
    case REG_FIFO_OVF: v = 0; break;
    case REG_FIFO_RD:  v = c->rd_ptr; break;
    case REG_FIFO_DATA: {
      uint8_t idx = c->rd_ptr;
      uint32_t red = c->fifo_red[idx];
      uint32_t ir  = c->fifo_ir[idx];
      switch (c->byte_idx) {
        case 0: v = (red >> 16) & 0x03; break;
        case 1: v = (red >> 8) & 0xFF;  break;
        case 2: v = red & 0xFF;         break;
        case 3: v = (ir >> 16) & 0x03;  break;
        case 4: v = (ir >> 8) & 0xFF;   break;
        default: v = ir & 0xFF;         break;
      }
      c->byte_idx++;
      if (c->byte_idx >= 6) {
        c->byte_idx = 0;
        if (c->rd_ptr != c->wr_ptr) c->rd_ptr = (c->rd_ptr + 1) & (FIFO_DEPTH - 1);
      }
      break;
    }
    case REG_REV_ID:  v = 0x03; break;
    case REG_PART_ID: v = 0x15; break;
    default: v = c->regs[r]; break;
  }
  if (r != REG_FIFO_DATA) c->reg_ptr = (uint8_t)(r + 1);
  return v;
}

static bool on_i2c_write(void *user_data, uint8_t data) {
  chip_state_t *c = (chip_state_t *)user_data;
  if (pin_read(c->pin_vcc) == LOW) return false;

  if (c->expect_reg) {
    c->reg_ptr = data;
    c->expect_reg = false;
    return true;
  }

  uint8_t r = c->reg_ptr;
  switch (r) {
    case REG_FIFO_WR:  c->wr_ptr = data & 0x1F; break;
    case REG_FIFO_OVF: break;
    case REG_FIFO_RD:  c->rd_ptr = data & 0x1F; break;
    case REG_FIFO_DATA:
    case REG_REV_ID:
    case REG_PART_ID:  break;
    case REG_MODE:
      if (data & 0x40) {               /* soft reset */
        reset_device(c);
        c->expect_reg = false;
        return true;
      }
      if ((data & 0x07) && !(c->regs[REG_MODE] & 0x07)) {
        c->tick = 0;                   /* measurement starts now */
        c->phase = 0.0f;
      }
      c->regs[r] = data & 0x07;
      break;
    default:
      c->regs[r] = data;
      break;
  }
  if (r != REG_FIFO_DATA) c->reg_ptr = (uint8_t)(r + 1);
  return true;
}

static void on_i2c_disconnect(void *user_data) {
  chip_state_t *c = (chip_state_t *)user_data;
  c->byte_idx = 0;
}

void chip_init(void) {
  chip_state_t *c = (chip_state_t *)malloc(sizeof(chip_state_t));

  c->pin_scl = pin_init("SCL", INPUT_PULLUP);
  c->pin_sda = pin_init("SDA", INPUT_PULLUP);
  c->pin_vcc = pin_init("VCC", INPUT);
  pin_init("GND", INPUT);
  c->pin_int = pin_init("INT", OUTPUT_HIGH);   /* active-low, idle high */

  c->rng = 0x1234ABCDu;
  reset_device(c);

  i2c_config_t i2c = {
    .address = I2C_ADDR,
    .scl = c->pin_scl,
    .sda = c->pin_sda,
    .connect = on_i2c_connect,
    .read = on_i2c_read,
    .write = on_i2c_write,
    .disconnect = on_i2c_disconnect,
    .user_data = c
  };
  i2c_init(&i2c);

  timer_config_t tcfg = {
    .callback = on_sample_timer,
    .user_data = c
  };
  timer_t t = timer_init(&tcfg);
  timer_start(t, 10000, true);         /* 10 ms -> 100 samples/s */
}