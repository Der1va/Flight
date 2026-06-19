#ifndef _INT_VL53L1X_H_
#define _INT_VL53L1X_H_

#include "vl53l1_platform.h"
#include "Vl53l1X_api.h"
#include "VL53l1X_calibration.h"
#include "FreeRTOS.h"
#include "task.h"

void Int_VL53L1X_Init(void);

uint16_t Int_VL53L1X_Read_Distance(void);

#endif // _INT_VL53L1X_H_