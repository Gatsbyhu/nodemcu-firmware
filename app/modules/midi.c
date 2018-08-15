#include "module.h"
#include "lauxlib.h"
#include "platform.h"
#include "c_stdlib.h"

#include "ws2812.h"
#include "color_utils.h"

extern bool uart0_echo;
extern bool run_input;
extern uint16_t need_len;
extern int16_t end_char;

#define RIPPLE_SIZE 10

uint8_t *data = NULL;
uint8_t keys[88];
uint8_t colors[88][3];
uint8_t ripple[88][3];

int inactiv_cb = LUA_NOREF;
int activ_cb = LUA_NOREF;

typedef enum channel_modes {
    NONE = 0,
    VELOCITY,
    COLOR_CODED,
    OCTAVE_COLOR_CODED,
    RANDOM,
    FROZEN,
    RIPPLE
} channel_mode_t;

channel_mode_t channel_mode[16];

static const uint8_t octave_key_colors[12][3] = {   //G R B
    {0,   255, 0},
    {0,   80,  200},
    {127, 255, 0},
    {0,   80,  200},
    {200, 255, 0},
    {255, 0,   0},
    {0,   80,  200},
    {255, 0,   255},
    {0,   80,  200},
    {0,   0,   255},
    {0,   80,  200},
    {0,   255, 255},
};

static const uint8_t octave_colors[9][3] = {   //G R B
    {0,   255, 0},
    {127, 255, 0},
    {200, 255, 0},
    {255, 0,   0},
    {255, 0,   255},
    {0,   0,   255},
    {0,   255, 255},
    {0,   80,  200},
    {255, 255, 255},
};

typedef enum midi_com {
    NOTE_OFF = 0,
    NOTE_ON,
    AFTERTOUCH,
    CONTROLLER,
    PATCH_CHANGE,
    CHANNEL_PRESSURE,
    PITCH_BEND,
    SYSTEM,
    SUSTAIN
} midi_com_t;

midi_com_t midi_cmd = SYSTEM;
uint8_t midi_ch = 0;
uint8_t midi_arg = 0;
uint8_t midi_key = 0;
uint8_t midi_val = 0;
uint8_t active_ch = 0;
uint8_t sustain = 0;
uint8_t started = FALSE;
uint32_t last_key = 0;
uint32_t tmr_cntr = 0;
os_timer_t refresh_tmr;

static inline void register_lua_cb(lua_State* L,int* cb_ref){
  int ref=luaL_ref(L, LUA_REGISTRYINDEX);
  if( *cb_ref != LUA_NOREF){
    luaL_unref(L, LUA_REGISTRYINDEX, *cb_ref);
  }
  *cb_ref = ref;
}

void note_on(uint8_t key, uint8_t val, uint8_t midi_ch) {
  last_key = tmr_cntr;
  if (!started && activ_cb != LUA_NOREF) {
    lua_State* L = lua_getstate();
    lua_rawgeti(L, LUA_REGISTRYINDEX, activ_cb);
    lua_call(L, 0, 0);
  }

  int i = (int) val * 3;
  uint8_t k = key - 21;
  keys[k] = 255;
  active_ch = midi_ch;

  channel_mode_t mode = channel_mode[active_ch];
  if (mode == VELOCITY) {
    uint8_t div = 1;

    uint8_t v = (uint8_t) (i > 255 ? 255 : i);
    uint8_t r = (uint8_t) ((v >= 128 ? (v - 128) * 2 : 0) / div);
    uint8_t g = (uint8_t) ((v < 128 ? v * 2 : (255 - v) * 2) / div);
    uint8_t b = (uint8_t) ((v / 30) / div);

    colors[k][0] = g;
    colors[k][1] = r;
    colors[k][2] = b;
  } else if (mode == COLOR_CODED) {
    uint8_t octave_k = key % 12;
    colors[k][0] = octave_key_colors[octave_k][0];
    colors[k][1] = octave_key_colors[octave_k][1];
    colors[k][2] = octave_key_colors[octave_k][2];
  } else if (mode == OCTAVE_COLOR_CODED) {
    uint8_t octave = 0;
    if (k > 2) {
      octave = ((k - 3) / 12) + 1;
    }
    colors[k][0] = octave_colors[octave][0];
    colors[k][1] = octave_colors[octave][1];
    colors[k][2] = octave_colors[octave][2];
  } else if (mode == RANDOM) {
    colors[k][0] = os_random();
    colors[k][1] = os_random();
    colors[k][2] = os_random();
  } else if (mode == FROZEN) {
    colors[k][0] = 200;
    colors[k][1] = 0;
    colors[k][2] = 255;
  } else if (mode == RIPPLE) {
    uint8_t div = 1;

    uint8_t v = (uint8_t) (i > 255 ? 255 : i);
    uint8_t r = (uint8_t) ((v >= 128 ? (v - 128) * 2 : 0) / div);
    uint8_t g = (uint8_t) ((v < 128 ? v * 2 : (255 - v) * 2) / div);
    uint8_t b = (uint8_t) ((v / 30) / div);

    ripple[k][0] = g;
    ripple[k][1] = r;
    ripple[k][2] = b;

    ripple[k][3] = RIPPLE_SIZE;
  }
}

void note_off(uint8_t key, uint8_t val) {
  channel_mode_t mode = channel_mode[active_ch];
  if (mode != RIPPLE) {
    keys[key - 21] = 0;
    if (!sustain) {
      colors[key - 21][0] = 0;
      colors[key - 21][1] = 0;
      colors[key - 21][2] = 0;
    }
  }
}

void sustain_on() {

}

void sustain_off() {
  channel_mode_t mode = channel_mode[active_ch];
  if (active_ch != RIPPLE) {
    for (int k = 0; k < 88; k++) {
      if (keys[k] == 0) {
        keys[k] = 0;
        colors[k][0] = 0;
        colors[k][1] = 0;
        colors[k][2] = 0;
      }
    }
  }
}

void uart_on_char_cb(const uint8_t c) {
  if (c & 0x80) {
    midi_cmd = (midi_com_t) ((c & 0x70) >> 4);
    midi_ch = c & 0xf;
    midi_arg = 0;
  } else {
    if (midi_cmd == NOTE_ON) {
      if (midi_arg % 2 == 0) {
        midi_key = c;
      } else if (midi_arg % 2 == 1) {
        midi_val = c;
        if (data != NULL) {
          note_on(midi_key, midi_val, midi_ch);
        }
      }
    } else if (midi_cmd == NOTE_OFF) {
      if (midi_arg % 2 == 0) {
        midi_key = c;
      } else if (midi_arg % 2 == 1) {
        midi_val = c;
        if (data != NULL) {
          note_off(midi_key, midi_val);
        }
      }
    } else if (midi_cmd == CONTROLLER) {
      if (midi_arg == 0 && c == 64) {
        midi_cmd = SUSTAIN;
      }
    } else if (midi_cmd == SUSTAIN) {
      if (c < 63 && sustain == 1) {
        sustain = 0;
        if (data != NULL) {
          sustain_off();
        }
      } else if (c > 65 && sustain == 0) {
        sustain = 1;
        if (data != NULL) {
          sustain_on();
        }
      }
    }
    midi_arg++;
  }
}

static uint8_t col(int val) {
  if (val > 255) {
    return 255;
  } else if (val < 0) {
    return 0;
  } else {
    return val;
  }
}

static uint8_t add(uint8_t col1, uint8_t col2) {
  return col(((uint16) col1) + col2);
}

static void on_refresh_tmr(void *arg) {
  tmr_cntr++;

  if (last_key != 0 && tmr_cntr - last_key > 50 * 60) {
    last_key = 0;

    lua_State* L = lua_getstate();
    lua_rawgeti(L, LUA_REGISTRYINDEX, inactiv_cb);
    lua_call(L, 0, 0);
  }

  channel_mode_t mode = channel_mode[active_ch];
  if (mode == RIPPLE && tmr_cntr % 2 == 0) {
    uint8 diff = tmr_cntr % 200 / 10;
    for (int i = 0; i < 88; i++){
      colors[i][1] = 0;
      colors[i][2] = 0;
      if (diff >= 10) {
        colors[i][0] = 40 - diff;
      } else {
        colors[i][0] = 20 + diff;
      }
    }


    for (int i = 0; i < 88; i++) {
      int v = ripple[i][3];
      if (v > 0) {
        ripple[i][3]--;
        for (int j = 0; j < RIPPLE_SIZE - v; j++) {
          int e1 = i - j;
          int e2 = i + j;
          if (e1 > 0 && e1 < 88) {
            colors[e1][0] = add(colors[e1][0], ripple[i][0] / (j + 1));
            colors[e1][1] = add(colors[e1][1], ripple[i][1] / (j + 1));
            colors[e1][2] = add(colors[e1][2], ripple[i][2] / (j + 1));
          }
          if (e2 > 0 && e2 < 88) {
            colors[e2][0] = add(colors[e2][0], ripple[i][0] / (j + 1));
            colors[e2][1] = add(colors[e2][1], ripple[i][1] / (j + 1));
            colors[e2][2] = add(colors[e2][2], ripple[i][2] / (j + 1));
          }
        }
      }
    }
  }

  if (mode == FROZEN) {
    uint8 diff = tmr_cntr % 200 / 10;
    for (int i = 0; i < 88; i++) {
      if (colors[i][2] != 255) {
        if (diff >= 10) {
          colors[i][0] = ((uint8)os_random())/30;
          colors[i][2] = 40 - diff;
        } else {
          colors[i][0] = ((uint8)os_random())/30;
          colors[i][2] = 20 + diff;
        }
      }
    }
  }

  for (int k = 0; k < 88; k++) {
    int i = k * 6;

    data[i] = colors[k][0];
    data[i + 1] = colors[k][1];
    data[i + 2] = colors[k][2];
    data[i + 3] = colors[k][0];
    data[i + 4] = colors[k][1];
    data[i + 5] = colors[k][2];
  }

  ws2812_write_data(data, 3 * 176, NULL, 0);

  for (int k = 0; k < 88; k++) {
    uint8_t v = keys[k];
    if (v > 2) {
      v -= 3;
    } else {
      v = 0;
    }
    keys[k] = v;
  }
}

static int midi_init(lua_State *L) {
  platform_gpio_mode(1, PLATFORM_GPIO_OUTPUT, PLATFORM_GPIO_FLOAT);
  platform_gpio_mode(2, PLATFORM_GPIO_OUTPUT, PLATFORM_GPIO_FLOAT);
  platform_gpio_write(1, PLATFORM_GPIO_HIGH);
  platform_gpio_write(2, PLATFORM_GPIO_HIGH);
  ws2812_buffer *buffer = (ws2812_buffer *) luaL_checkudata(L, 1, "ws2812.buffer");
  luaL_argcheck(L, buffer != NULL, 1, "no valid buffer provided");
  // get rid of old state
  data = buffer->values;

  uart0_echo = false;
  run_input = false;
  platform_uart_setup(0, 31250, 8, 0, 1);
  need_len = 1;
  end_char = -1;

  return 0;
}

static int midi_start(lua_State *L) {
  os_timer_disarm(&refresh_tmr);
  os_timer_setfn(&refresh_tmr, on_refresh_tmr, NULL);
  os_timer_arm(&refresh_tmr, 20, 1);
  started = TRUE;
  return 0;
}

static int midi_stop(lua_State *L) {
  os_timer_disarm(&refresh_tmr);
  started = FALSE;
  return 0;
}

static int midi_inactiv_register(lua_State *L) {
  lua_pushvalue(L, 1);  // copy argument (func) to the top of stack
  register_lua_cb(L, &inactiv_cb);
  return 0;
}

static int midi_activ_register(lua_State *L) {
  lua_pushvalue(L, 1);  // copy argument (func) to the top of stack
  register_lua_cb(L, &activ_cb);
  return 0;
}

static int midi_set_mode(lua_State *L) {
  uint8 channel = luaL_checkinteger(L, 1);
  uint8 mode = luaL_checkinteger(L, 2);
  channel_mode[channel] = mode;
  return 0;
}


static const LUA_REG_TYPE midi_map[] = {
    {LSTRKEY("init"),  LFUNCVAL(midi_init)},
    {LSTRKEY("start"), LFUNCVAL(midi_start)},
    {LSTRKEY("stop"),  LFUNCVAL(midi_stop)},
    {LSTRKEY("setChannelMode"),  LFUNCVAL(midi_set_mode)},
    {LSTRKEY("onInactivity"),  LFUNCVAL(midi_inactiv_register)},
    {LSTRKEY("onActivity"),  LFUNCVAL(midi_activ_register)},
    {LSTRKEY("NONE"),               LNUMVAL(NONE)},
    {LSTRKEY("VELOCITY"),           LNUMVAL(VELOCITY)},
    {LSTRKEY("COLOR_CODED"),        LNUMVAL(COLOR_CODED)},
    {LSTRKEY("OCTAVE_COLOR_CODED"), LNUMVAL(OCTAVE_COLOR_CODED)},
    {LSTRKEY("RANDOM"),             LNUMVAL(RANDOM)},
    {LSTRKEY("FROZEN"),             LNUMVAL(FROZEN)},
    {LSTRKEY("RIPPLE"),             LNUMVAL(RIPPLE)},
    {LNILKEY,          LNILVAL}
};

NODEMCU_MODULE(MIDI, "midi", midi_map, NULL);
