#ifndef LV_CONF_H
#define LV_CONF_H

#define LV_COLOR_DEPTH 16
#define LV_COLOR_16_SWAP 1

/* Base de tempo do LVGL (sem isto nenhum timer/refresh executa) */
#define LV_TICK_CUSTOM 1
#define LV_TICK_CUSTOM_INCLUDE "Arduino.h"
#define LV_TICK_CUSTOM_SYS_TIME_EXPR (millis())

/* Memória do LVGL na PSRAM */
#define LV_MEM_CUSTOM 1
#define LV_MEM_CUSTOM_INCLUDE <esp_heap_caps.h>
#define LV_MEM_CUSTOM_ALLOC(size)   heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)
#define LV_MEM_CUSTOM_FREE(ptr)     free(ptr)
#define LV_MEM_CUSTOM_REALLOC(ptr, size) heap_caps_realloc(ptr, size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)

#define LV_FONT_MONTSERRAT_20 1
#define LV_FONT_MONTSERRAT_40 1
#define LV_FONT_MONTSERRAT_48 1
#define LV_USE_CHART 1
#define LV_USE_ARC 1
#define LV_SPRINTF_USE_FLOAT 1
#define LV_USE_LINE 1
#define LV_USE_IMG 1
#define LV_USE_CANVAS 1

#endif /* LV_CONF_H */
