
#include "module.h"
#include "lauxlib.h"
#include "platform.h"
#include "c_stdlib.h"

#include "SX1278.h"

static int ICACHE_FLASH_ATTR lora_init(lua_State* L) {
  uint32_t freq = luaL_optinteger( L, 1, 915000000);
  uint8_t sf = luaL_optinteger( L, 2, 8);
  uint8_t bw = luaL_optinteger( L, 3, 7);

  SX1278LoraInit(freq, sf, bw);

  return 0;
}

static int ICACHE_FLASH_ATTR lora_cad(lua_State* L) {
  SX1278LoRaStartCAD();

  return 0;
}

static int ICACHE_FLASH_ATTR lora_rx(lua_State* L) {
  SX1278LoRaStartRx();

  return 0;
}

static int ICACHE_FLASH_ATTR lora_get(lua_State* L) {
  uint8_t rxBuffer[8];
  SX1278LoRaGetRxPacket();
  SX1278LoRaReadRxBuffer(rxBuffer);
  lua_pushlstring(L, (const char*) rxBuffer, 8);
  return 1;
}

static int ICACHE_FLASH_ATTR lora_tx(lua_State* L) {
  size_t length;
  SX1278LoRaSetTxMessage((const uint8_t*) lua_tolstring(L, 1, &length));
  SX1278LoRaStartTx();

  return 0;
}

static int ICACHE_FLASH_ATTR lora_flags(lua_State* L) {
  lua_pushinteger(L, SX1278LoRaGetFlags());

  return 1;
}

static int ICACHE_FLASH_ATTR lora_clear_flags(lua_State* L) {
  SX1278LoRaClearFlags();

  return 0;
}

static const LUA_REG_TYPE lora_map[] = {
    { LSTRKEY( "init" ),         LFUNCVAL( lora_init )},
    { LSTRKEY( "cad" ),         LFUNCVAL( lora_cad )},
    { LSTRKEY( "rx" ),         LFUNCVAL( lora_rx )},
    { LSTRKEY( "get" ),         LFUNCVAL( lora_get )},
    { LSTRKEY( "tx" ),         LFUNCVAL( lora_tx )},
    { LSTRKEY( "flags" ),         LFUNCVAL( lora_flags )},
    { LSTRKEY( "clear_flags" ),         LFUNCVAL( lora_clear_flags )},
    { LNILKEY, LNILVAL}
};

NODEMCU_MODULE(LORA, "lora", lora_map, NULL);
