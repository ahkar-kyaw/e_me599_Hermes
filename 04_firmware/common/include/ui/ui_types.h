#ifndef UI_TYPES_H
#define UI_TYPES_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    UI_PAGE_STATUS = 0,
    UI_PAGE_RC,
    UI_PAGE_IMU,
    UI_PAGE_CONTROL,
    UI_PAGE_CAN,
    UI_PAGE_COUNT
} ui_page_t;

typedef enum
{
    UI_MODE_BROWSE = 0,
    UI_MODE_INTERACT
} ui_mode_t;

typedef enum
{
    UI_EVENT_NONE = 0u,
    UI_EVENT_LEFT = 1u << 0,
    UI_EVENT_RIGHT = 1u << 1,
    UI_EVENT_UP = 1u << 2,
    UI_EVENT_DOWN = 1u << 3,
    UI_EVENT_INTERACT_ON = 1u << 4,
    UI_EVENT_INTERACT_OFF = 1u << 5,
    UI_EVENT_ENTER = 1u << 6,
    UI_EVENT_INPUT_ENABLED = 1u << 7,
    UI_EVENT_INPUT_DISABLED = 1u << 8,
    UI_EVENT_LINK_LOST = 1u << 9
} ui_event_t;

typedef uint32_t ui_event_flags_t;

typedef struct
{
    ui_page_t page;
    ui_mode_t mode;
    uint8_t selection;
    bool input_enabled;
    uint32_t enter_count;
} ui_state_t;

#ifdef __cplusplus
}
#endif

#endif
