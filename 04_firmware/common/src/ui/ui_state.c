#include "ui/ui_state.h"

#include "app/app_actuator_types.h"

#include <stddef.h>
#include <string.h>

void ui_state_init(ui_state_t *state)
{
    if (state == NULL)
    {
        return;
    }

    memset(state, 0, sizeof(*state));
    state->page = UI_PAGE_STATUS;
    state->mode = UI_MODE_BROWSE;
    state->input_enabled = false;
}

bool ui_state_update(ui_state_t *state, ui_event_flags_t events)
{
    if (state == NULL)
    {
        return false;
    }

    const ui_state_t previous = *state;
    if ((events & UI_EVENT_LINK_LOST) != 0u)
    {
        state->input_enabled = false;
        state->mode = UI_MODE_BROWSE;
        state->selection = 0u;
        return memcmp(&previous, state, sizeof(*state)) != 0;
    }

    if ((events & UI_EVENT_INPUT_DISABLED) != 0u)
    {
        state->input_enabled = false;
        state->mode = UI_MODE_BROWSE;
        state->selection = 0u;
        return memcmp(&previous, state, sizeof(*state)) != 0;
    }

    if (!state->input_enabled)
    {
        if ((events & UI_EVENT_INPUT_ENABLED) != 0u)
        {
            state->input_enabled = true;
            state->mode = UI_MODE_BROWSE;
            state->selection = 0u;
        }

        return memcmp(&previous, state, sizeof(*state)) != 0;
    }

    if ((events & UI_EVENT_INTERACT_OFF) != 0u)
    {
        state->mode = UI_MODE_BROWSE;

        if ((state->page != UI_PAGE_CAN) &&
            (state->page != UI_PAGE_CONTROL))
        {
            state->selection = 0u;
        }

        return memcmp(&previous, state, sizeof(*state)) != 0;
    }

    if ((events & UI_EVENT_INTERACT_ON) != 0u)
    {
        state->mode = UI_MODE_INTERACT;

        if ((state->page != UI_PAGE_CAN) &&
            (state->page != UI_PAGE_CONTROL))
        {
            state->selection = 0u;
        }

        return memcmp(&previous, state, sizeof(*state)) != 0;
    }

    if (((events & UI_EVENT_ENTER) != 0u) &&
        (state->mode == UI_MODE_INTERACT))
    {
        state->enter_count++;
    }

    if (state->mode == UI_MODE_BROWSE)
    {
        if ((events & UI_EVENT_LEFT) != 0u)
        {
            state->page = (state->page == UI_PAGE_STATUS)
                ? (ui_page_t)(UI_PAGE_COUNT - 1)
                : (ui_page_t)(state->page - 1);
            state->selection = 0u;
        }
        else if ((events & UI_EVENT_RIGHT) != 0u)
        {
            state->page = (ui_page_t)((state->page + 1) % UI_PAGE_COUNT);
            state->selection = 0u;
        }
        else if ((state->page == UI_PAGE_CAN) ||
                 (state->page == UI_PAGE_CONTROL))
        {
            const uint8_t selection_count =
                ui_state_selection_count(state->page);

            if ((events & UI_EVENT_UP) != 0u)
            {
                state->selection = (state->selection == 0u)
                    ? (uint8_t)(selection_count - 1u)
                    : (uint8_t)(state->selection - 1u);
            }
            else if ((events & UI_EVENT_DOWN) != 0u)
            {
                state->selection =
                    (uint8_t)((state->selection + 1u) % selection_count);
            }
        }
    }
    else
    {
        if ((state->page == UI_PAGE_CAN) ||
            (state->page == UI_PAGE_CONTROL))
        {
            return memcmp(&previous, state, sizeof(*state)) != 0;
        }

        const uint8_t selection_count =
            ui_state_selection_count(state->page);

        if ((events & UI_EVENT_UP) != 0u)
        {
            state->selection = (state->selection == 0u)
                ? (uint8_t)(selection_count - 1u)
                : (uint8_t)(state->selection - 1u);
        }
        else if ((events & UI_EVENT_DOWN) != 0u)
        {
            state->selection =
                (uint8_t)((state->selection + 1u) % selection_count);
        }
    }

    return memcmp(&previous, state, sizeof(*state)) != 0;
}

uint8_t ui_state_selection_count(ui_page_t page)
{
    switch (page)
    {
        case UI_PAGE_RC:
            return 2u;

        case UI_PAGE_IMU:
            return 2u;

        case UI_PAGE_CAN:
            return APP_ACTUATOR_COUNT;

        case UI_PAGE_CONTROL:
            return 4u;

        case UI_PAGE_STATUS:
        default:
            return 1u;
    }
}
