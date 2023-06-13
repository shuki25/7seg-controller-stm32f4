/*
* misc.h
* Created on: 2023-06-03
* Author: Josh Butler, MD, MHI
*
* Miscellaneous functions
*
*/

#include "misc.h"

void *rtos_malloc(size_t size) {
    return pvPortMalloc(size);
}

void rtos_free(void *ptr) {
    vPortFree(ptr);
}
