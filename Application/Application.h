#ifndef _APPLICATION_H
#define _APPLICATION_H

#include "FreeRTOS.h"
#include "task.h"
#include "common_debug.h"
#include "Int_IP5305T.h"
#include "Int_SI24R1.h"
#include "Application_process_data.h"
#include "Application_transmit_data.h"
#include "Application_display.h"

void App_FreeRTOS_Start(void);


#endif /* _APPLICATION_H */
