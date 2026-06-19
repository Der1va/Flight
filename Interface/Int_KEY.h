#ifndef _INT_KEY_H
#define _INT_KEY_H

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "common_debug.h"

typedef enum
{
    KEY_NONE = 0,
    KEY_UP,
    KEY_DOWN,
    KEY_LEFT,
    KEY_RIGHT,
    KEY_LEFT_X,
    KEY_LEFT_X_LONG,
    KEY_RIGHT_X,
    KEY_RIGHT_X_LONG
} Key_TypeDef;
/** 
    * @brief 获取按键值
    * @return Key_TypeDef
**/
Key_TypeDef Int_KEY_get(void);

#endif /* _INT_KEY_H */