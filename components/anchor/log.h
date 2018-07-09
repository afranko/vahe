#ifndef __LOG_H
#define __LOG_H

#include <stdio.h>
#include "printf/tinyprintf.h"

#define DEBUG
#ifdef DEBUG
#define LOGT(tag,x,...)  //tfp_printf("[T," tag "] - " x "\n",##__VA_ARGS__)
#define LOGD(tag,x,...)  //tfp_printf("[D," tag "] - " x "\n",##__VA_ARGS__)
#define LOGW(tag,x,...)  //tfp_printf("[W," tag "] - " x "\n",##__VA_ARGS__)
#define LOGE(tag,x,...)  //tfp_printf("[E," tag "] - " x "\n",##__VA_ARGS__)

#define LOG_SH(tag,x,...) //printf("[S] - " x "\n",##__VA_ARGS__)
#else
#define LOGD(tag,x,...)
#define LOGW(tag,x,...)
#define LOGE(tag,x,...)
#endif

void log_init();

#endif
