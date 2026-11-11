#ifndef DRV_ST7789_H
#define DRV_ST7789_H

#include "interfaces/if_display_io.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DRV_ST7789_OK                 0
#define DRV_ST7789_ERR               -1
#define DRV_ST7789_NATIVE_WIDTH       240u
#define DRV_ST7789_NATIVE_HEIGHT      320u
#define DRV_ST7789_PIXEL_SCALE        2u
#define DRV_ST7789_BYTES_PER_PIXEL    2u
#define DRV_ST7789_CANVAS_MAX_WIDTH   160u
#define DRV_ST7789_CANVAS_MAX_HEIGHT  160u
#define DRV_ST7789_FRAMEBUFFER_SIZE   38400u
#define DRV_ST7789_TRANSFER_LINE_SIZE 1280u

typedef enum
{
    DRV_ST7789_ORIENTATION_0 = 0,
    DRV_ST7789_ORIENTATION_90,
    DRV_ST7789_ORIENTATION_180,
    DRV_ST7789_ORIENTATION_270
} drv_st7789_orientation_t;

typedef struct
{
    drv_st7789_orientation_t orientation;
} drv_st7789_config_t;

typedef struct
{
    if_display_io_t io;
    drv_st7789_config_t config;
    uint8_t *framebuffer;
    size_t framebuffer_size;
    uint16_t width;
    uint16_t height;
    _Alignas(4) uint8_t transfer_line[DRV_ST7789_TRANSFER_LINE_SIZE];
    bool initialized;
} drv_st7789_t;

drv_st7789_config_t drv_st7789_default_config(void);

int drv_st7789_init(drv_st7789_t *display,
                    const if_display_io_t *io,
                    const drv_st7789_config_t *config,
                    uint8_t *framebuffer,
                    size_t framebuffer_size);

uint16_t drv_st7789_width(const drv_st7789_t *display);
uint16_t drv_st7789_height(const drv_st7789_t *display);

void drv_st7789_clear(drv_st7789_t *display, uint16_t color);

void drv_st7789_draw_pixel(drv_st7789_t *display,
                           uint16_t x,
                           uint16_t y,
                           uint16_t color);

void drv_st7789_fill_rect(drv_st7789_t *display,
                          uint16_t x,
                          uint16_t y,
                          uint16_t width,
                          uint16_t height,
                          uint16_t color);

int drv_st7789_update(drv_st7789_t *display);

#ifdef __cplusplus
}
#endif

#endif
