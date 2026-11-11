#ifndef CONFIG_UI_H
#define CONFIG_UI_H

#include "drivers/drv_st7789.h"

#define APP_DISPLAY_ORIENTATION DRV_ST7789_ORIENTATION_90

#define APP_UI_TASK_PERIOD_MS      100u
#define APP_UI_RENDER_PERIOD_MS    500u
#define APP_UI_RETRY_MS            1000u

#endif
