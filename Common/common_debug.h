#ifndef __COMMON_DEBUG_H
#define __COMMON_DEBUG_H

#include "usart.h"
#include "stdio.h"

int __io_putchar(int ch);
int fputc(int ch, FILE *f);

#define Debug_Enable 1



#if Debug_Enable
#define Debug_Printf(format, ...) printf("[%s:%d] "  format, __FILE_NAME__, __LINE__, ##__VA_ARGS__)
#else
#define Debug_Printf(...)
#endif

#endif /* __COMMON_DEBUG_H */
