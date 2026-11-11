#include "drivers/drv_st7789.h"

#include <stddef.h>
#include <string.h>

#define ST7789_CMD_SOFTWARE_RESET 0x01u
#define ST7789_CMD_SLEEP_OUT      0x11u
#define ST7789_CMD_NORMAL_ON      0x13u
#define ST7789_CMD_INVERSION_ON   0x21u
#define ST7789_CMD_DISPLAY_ON     0x29u
#define ST7789_CMD_SET_COLUMN     0x2Au
#define ST7789_CMD_SET_ROW        0x2Bu
#define ST7789_CMD_WRITE_RAM      0x2Cu
#define ST7789_CMD_MEMORY_ACCESS  0x36u
#define ST7789_CMD_PIXEL_FORMAT   0x3Au
#define ST7789_CMD_PORCH_CONTROL  0xB2u
#define ST7789_CMD_GATE_CONTROL   0xB7u
#define ST7789_CMD_VCOM           0xBBu
#define ST7789_CMD_LCM_CONTROL    0xC0u
#define ST7789_CMD_VDV_VRH_ENABLE 0xC2u
#define ST7789_CMD_VRH_SET        0xC3u
#define ST7789_CMD_VDV_SET        0xC4u
#define ST7789_CMD_FRAME_RATE     0xC6u
#define ST7789_CMD_POWER_CONTROL  0xD0u
#define ST7789_CMD_GAMMA_POSITIVE 0xE0u
#define ST7789_CMD_GAMMA_NEGATIVE 0xE1u

#define ST7789_MADCTL_ROW_REVERSE    0x80u
#define ST7789_MADCTL_COLUMN_REVERSE 0x40u
#define ST7789_MADCTL_ROW_COLUMN_SWAP 0x20u

static int drv_st7789_command(drv_st7789_t *display,
                              uint8_t command,
                              const uint8_t *data,
                              size_t data_length);
static int drv_st7789_write_init_sequence(drv_st7789_t *display);
static uint8_t drv_st7789_memory_access(
    drv_st7789_orientation_t orientation);
static void drv_st7789_set_dimensions(drv_st7789_t *display);
static int drv_st7789_set_window(drv_st7789_t *display);
static void drv_st7789_write_pixel(uint8_t *destination,
                                   uint16_t color);

drv_st7789_config_t drv_st7789_default_config(void)
{
    const drv_st7789_config_t config =
    {
        .orientation = DRV_ST7789_ORIENTATION_90,
    };

    return config;
}

int drv_st7789_init(drv_st7789_t *display,
                    const if_display_io_t *io,
                    const drv_st7789_config_t *config,
                    uint8_t *framebuffer,
                    size_t framebuffer_size)
{
    if ((display == NULL) || (io == NULL) || (config == NULL) ||
        (framebuffer == NULL) ||
        (framebuffer_size < DRV_ST7789_FRAMEBUFFER_SIZE) ||
        (io->write_command == NULL) || (io->write_data == NULL) ||
        (io->set_reset == NULL) || (io->delay_ms == NULL) ||
        (config->orientation > DRV_ST7789_ORIENTATION_270))
    {
        return DRV_ST7789_ERR;
    }

    memset(display, 0, sizeof(*display));
    display->io = *io;
    display->config = *config;
    display->framebuffer = framebuffer;
    display->framebuffer_size = framebuffer_size;
    drv_st7789_set_dimensions(display);

    display->io.set_reset(display->io.context, true);
    display->io.delay_ms(display->io.context, 20u);
    display->io.set_reset(display->io.context, false);
    display->io.delay_ms(display->io.context, 20u);
    display->io.set_reset(display->io.context, true);
    display->io.delay_ms(display->io.context, 120u);

    if (drv_st7789_write_init_sequence(display) != DRV_ST7789_OK)
    {
        return DRV_ST7789_ERR;
    }

    display->initialized = true;
    drv_st7789_clear(display, 0x0000u);

    if (drv_st7789_update(display) != DRV_ST7789_OK)
    {
        display->initialized = false;
        return DRV_ST7789_ERR;
    }

    return DRV_ST7789_OK;
}

uint16_t drv_st7789_width(const drv_st7789_t *display)
{
    return (display != NULL) ? display->width : 0u;
}

uint16_t drv_st7789_height(const drv_st7789_t *display)
{
    return (display != NULL) ? display->height : 0u;
}

void drv_st7789_clear(drv_st7789_t *display, uint16_t color)
{
    if (display == NULL)
    {
        return;
    }

    drv_st7789_fill_rect(display,
                         0u,
                         0u,
                         display->width,
                         display->height,
                         color);
}

void drv_st7789_draw_pixel(drv_st7789_t *display,
                           uint16_t x,
                           uint16_t y,
                           uint16_t color)
{
    if ((display == NULL) || (display->framebuffer == NULL) ||
        (x >= display->width) || (y >= display->height))
    {
        return;
    }

    const size_t offset =
        (((size_t)y * display->width) + x) *
        DRV_ST7789_BYTES_PER_PIXEL;

    drv_st7789_write_pixel(&display->framebuffer[offset], color);
}

void drv_st7789_fill_rect(drv_st7789_t *display,
                          uint16_t x,
                          uint16_t y,
                          uint16_t width,
                          uint16_t height,
                          uint16_t color)
{
    if ((display == NULL) || (display->framebuffer == NULL) ||
        (width == 0u) || (height == 0u) ||
        (x >= display->width) || (y >= display->height))
    {
        return;
    }

    uint32_t end_x = (uint32_t)x + width;
    uint32_t end_y = (uint32_t)y + height;

    if (end_x > display->width)
    {
        end_x = display->width;
    }

    if (end_y > display->height)
    {
        end_y = display->height;
    }

    for (uint32_t row = y; row < end_y; row++)
    {
        for (uint32_t column = x; column < end_x; column++)
        {
            const size_t offset =
                (((size_t)row * display->width) + column) *
                DRV_ST7789_BYTES_PER_PIXEL;
            drv_st7789_write_pixel(&display->framebuffer[offset],
                                   color);
        }
    }
}

int drv_st7789_update(drv_st7789_t *display)
{
    if ((display == NULL) || !display->initialized ||
        (display->framebuffer == NULL) ||
        (drv_st7789_set_window(display) != DRV_ST7789_OK))
    {
        return DRV_ST7789_ERR;
    }

    for (uint32_t row = 0u; row < display->height; row++)
    {
        const uint8_t *source = &display->framebuffer[
            (size_t)row * display->width * DRV_ST7789_BYTES_PER_PIXEL];

        for (uint32_t column = 0u; column < display->width; column++)
        {
            const uint8_t high = source[column * 2u];
            const uint8_t low = source[(column * 2u) + 1u];
            const size_t destination = column * 4u;

            display->transfer_line[destination] = high;
            display->transfer_line[destination + 1u] = low;
            display->transfer_line[destination + 2u] = high;
            display->transfer_line[destination + 3u] = low;
        }

        const size_t line_size =
            (size_t)display->width * DRV_ST7789_PIXEL_SCALE *
            DRV_ST7789_BYTES_PER_PIXEL;

        for (uint32_t repeat = 1u;
             repeat < DRV_ST7789_PIXEL_SCALE;
             repeat++)
        {
            memcpy(&display->transfer_line[repeat * line_size],
                   display->transfer_line,
                   line_size);
        }

        if (display->io.write_data(
                display->io.context,
                display->transfer_line,
                line_size * DRV_ST7789_PIXEL_SCALE) !=
            IF_DISPLAY_IO_OK)
        {
            return DRV_ST7789_ERR;
        }
    }

    return DRV_ST7789_OK;
}

static int drv_st7789_command(drv_st7789_t *display,
                              uint8_t command,
                              const uint8_t *data,
                              size_t data_length)
{
    if ((display == NULL) || ((data == NULL) && (data_length != 0u)))
    {
        return DRV_ST7789_ERR;
    }

    if (display->io.write_command(display->io.context, command) !=
        IF_DISPLAY_IO_OK)
    {
        return DRV_ST7789_ERR;
    }

    if ((data_length != 0u) &&
        (display->io.write_data(display->io.context,
                                data,
                                data_length) != IF_DISPLAY_IO_OK))
    {
        return DRV_ST7789_ERR;
    }

    return DRV_ST7789_OK;
}

static int drv_st7789_write_init_sequence(drv_st7789_t *display)
{
    const uint8_t memory_access =
        drv_st7789_memory_access(display->config.orientation);
    const uint8_t pixel_format = 0x55u;
    const uint8_t porch_control[] =
        {0x0Cu, 0x0Cu, 0x00u, 0x33u, 0x33u};
    const uint8_t gate_control = 0x35u;
    const uint8_t vcom = 0x19u;
    const uint8_t lcm_control = 0x2Cu;
    const uint8_t vdv_vrh_enable = 0x01u;
    const uint8_t vrh = 0x12u;
    const uint8_t vdv = 0x20u;
    const uint8_t frame_rate = 0x0Fu;
    const uint8_t power_control[] = {0xA4u, 0xA1u};
    const uint8_t gamma_positive[] =
    {
        0xD0u, 0x04u, 0x0Du, 0x11u, 0x13u, 0x2Bu, 0x3Fu,
        0x54u, 0x4Cu, 0x18u, 0x0Du, 0x0Bu, 0x1Fu, 0x23u,
    };
    const uint8_t gamma_negative[] =
    {
        0xD0u, 0x04u, 0x0Cu, 0x11u, 0x13u, 0x2Cu, 0x3Fu,
        0x44u, 0x51u, 0x2Fu, 0x1Fu, 0x1Fu, 0x20u, 0x23u,
    };

    if (drv_st7789_command(display,
                           ST7789_CMD_SOFTWARE_RESET,
                           NULL,
                           0u) != DRV_ST7789_OK)
    {
        return DRV_ST7789_ERR;
    }

    display->io.delay_ms(display->io.context, 150u);

    if (drv_st7789_command(display,
                           ST7789_CMD_SLEEP_OUT,
                           NULL,
                           0u) != DRV_ST7789_OK)
    {
        return DRV_ST7789_ERR;
    }

    display->io.delay_ms(display->io.context, 120u);

    if ((drv_st7789_command(display, ST7789_CMD_MEMORY_ACCESS,
                            &memory_access, 1u) != DRV_ST7789_OK) ||
        (drv_st7789_command(display, ST7789_CMD_PIXEL_FORMAT,
                            &pixel_format, 1u) != DRV_ST7789_OK) ||
        (drv_st7789_command(display, ST7789_CMD_PORCH_CONTROL,
                            porch_control,
                            sizeof(porch_control)) != DRV_ST7789_OK) ||
        (drv_st7789_command(display, ST7789_CMD_GATE_CONTROL,
                            &gate_control, 1u) != DRV_ST7789_OK) ||
        (drv_st7789_command(display, ST7789_CMD_VCOM,
                            &vcom, 1u) != DRV_ST7789_OK) ||
        (drv_st7789_command(display, ST7789_CMD_LCM_CONTROL,
                            &lcm_control, 1u) != DRV_ST7789_OK) ||
        (drv_st7789_command(display, ST7789_CMD_VDV_VRH_ENABLE,
                            &vdv_vrh_enable, 1u) != DRV_ST7789_OK) ||
        (drv_st7789_command(display, ST7789_CMD_VRH_SET,
                            &vrh, 1u) != DRV_ST7789_OK) ||
        (drv_st7789_command(display, ST7789_CMD_VDV_SET,
                            &vdv, 1u) != DRV_ST7789_OK) ||
        (drv_st7789_command(display, ST7789_CMD_FRAME_RATE,
                            &frame_rate, 1u) != DRV_ST7789_OK) ||
        (drv_st7789_command(display, ST7789_CMD_POWER_CONTROL,
                            power_control,
                            sizeof(power_control)) != DRV_ST7789_OK) ||
        (drv_st7789_command(display, ST7789_CMD_GAMMA_POSITIVE,
                            gamma_positive,
                            sizeof(gamma_positive)) != DRV_ST7789_OK) ||
        (drv_st7789_command(display, ST7789_CMD_GAMMA_NEGATIVE,
                            gamma_negative,
                            sizeof(gamma_negative)) != DRV_ST7789_OK) ||
        (drv_st7789_command(display, ST7789_CMD_INVERSION_ON,
                            NULL, 0u) != DRV_ST7789_OK) ||
        (drv_st7789_command(display, ST7789_CMD_NORMAL_ON,
                            NULL, 0u) != DRV_ST7789_OK) ||
        (drv_st7789_command(display, ST7789_CMD_DISPLAY_ON,
                            NULL, 0u) != DRV_ST7789_OK))
    {
        return DRV_ST7789_ERR;
    }

    display->io.delay_ms(display->io.context, 100u);
    return DRV_ST7789_OK;
}

static uint8_t drv_st7789_memory_access(
    drv_st7789_orientation_t orientation)
{
    switch (orientation)
    {
        case DRV_ST7789_ORIENTATION_90:
            return ST7789_MADCTL_COLUMN_REVERSE |
                   ST7789_MADCTL_ROW_COLUMN_SWAP;

        case DRV_ST7789_ORIENTATION_180:
            return ST7789_MADCTL_ROW_REVERSE |
                   ST7789_MADCTL_COLUMN_REVERSE;

        case DRV_ST7789_ORIENTATION_270:
            return ST7789_MADCTL_ROW_REVERSE |
                   ST7789_MADCTL_ROW_COLUMN_SWAP;

        case DRV_ST7789_ORIENTATION_0:
        default:
            return 0u;
    }
}

static void drv_st7789_set_dimensions(drv_st7789_t *display)
{
    const bool landscape =
        (display->config.orientation == DRV_ST7789_ORIENTATION_90) ||
        (display->config.orientation == DRV_ST7789_ORIENTATION_270);

    display->width = (landscape ? DRV_ST7789_NATIVE_HEIGHT :
                                  DRV_ST7789_NATIVE_WIDTH) /
                     DRV_ST7789_PIXEL_SCALE;
    display->height = (landscape ? DRV_ST7789_NATIVE_WIDTH :
                                   DRV_ST7789_NATIVE_HEIGHT) /
                      DRV_ST7789_PIXEL_SCALE;
}

static int drv_st7789_set_window(drv_st7789_t *display)
{
    const uint16_t physical_width =
        (uint16_t)(display->width * DRV_ST7789_PIXEL_SCALE);
    const uint16_t physical_height =
        (uint16_t)(display->height * DRV_ST7789_PIXEL_SCALE);
    const uint16_t end_x = (uint16_t)(physical_width - 1u);
    const uint16_t end_y = (uint16_t)(physical_height - 1u);
    const uint8_t column[] =
    {
        0u, 0u, (uint8_t)(end_x >> 8u), (uint8_t)end_x,
    };
    const uint8_t row[] =
    {
        0u, 0u, (uint8_t)(end_y >> 8u), (uint8_t)end_y,
    };

    if ((drv_st7789_command(display, ST7789_CMD_SET_COLUMN,
                             column, sizeof(column)) != DRV_ST7789_OK) ||
        (drv_st7789_command(display, ST7789_CMD_SET_ROW,
                             row, sizeof(row)) != DRV_ST7789_OK) ||
        (drv_st7789_command(display, ST7789_CMD_WRITE_RAM,
                             NULL, 0u) != DRV_ST7789_OK))
    {
        return DRV_ST7789_ERR;
    }

    return DRV_ST7789_OK;
}

static void drv_st7789_write_pixel(uint8_t *destination,
                                   uint16_t color)
{
    destination[0] = (uint8_t)(color >> 8u);
    destination[1] = (uint8_t)color;
}
