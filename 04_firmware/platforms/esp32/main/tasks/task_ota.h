#ifndef TASK_OTA_H
#define TASK_OTA_H

#include "app/app_ota_types.h"

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void task_ota_start(void);
void task_ota_get_snapshot(app_ota_snapshot_t *snapshot);
bool task_ota_is_active(void);

#ifdef __cplusplus
}
#endif

#endif
