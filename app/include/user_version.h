#ifndef __USER_VERSION_H__
#define __USER_VERSION_H__

#include "version.h"	/* ESP firmware header */

#define NODE_VERSION_MAJOR	ESP_SDK_VERSION_MAJOR
#define NODE_VERSION_MINOR	ESP_SDK_VERSION_MINOR
#define NODE_VERSION_REVISION	ESP_SDK_VERSION_PATCH
#define NODE_VERSION_INTERNAL   0

#define NODE_VERSION_STR(x)	#x
#define NODE_VERSION_XSTR(x)	NODE_VERSION_STR(x)

#define NODE_VERSION		"NodeMCU " ESP_SDK_VERSION_STRING "." NODE_VERSION_XSTR(NODE_VERSION_INTERNAL) " built with Docker provided by frightanic.com\n\tbranch: newLFS\n\tcommit: d3642d2fc55a81343e9b6a45f9677e3c49b36397\n\tSSL: true\n\tmodules: adc,bh1750,bit,bmp085,cron,crypto,devapi,dht,ds18b20,file,gpio,hcsr04,http,htu21,i2c,ir,lora,mdns,mqtt,net,node,ow,pwm,rgb,rtcfifo,rtcmem,rtctime,sigma_delta,sjson,sntp,spi,struct,tmr,uart,u8g2,vl53l0x,websocket,wifi,ws2812,ws2812_effects,rboot\n"

#ifndef BUILD_DATE
#define BUILD_DATE		"created on 2018-08-25 23:05\n"
#endif

extern char SDK_VERSION[];

#endif	/* __USER_VERSION_H__ */
