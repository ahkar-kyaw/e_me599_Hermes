#ifndef APP_BALANCE_CONTROL_TYPES_H
#define APP_BALANCE_CONTROL_TYPES_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    uint64_t timestamp_us;
    uint32_t zero_lean_request_count;
    bool master_enabled;
    bool balance_enabled;
} app_balance_control_request_t;

#ifdef __cplusplus
}
#endif

#endif
