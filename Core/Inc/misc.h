/*
* misc.h
* Created on: 2023-06-03
* Author: Josh Butler, MD, MHI
*
* Miscellaneous functions
*
*/

#ifndef MISC_H_
#define MISC_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "cmsis_os.h"

void *rtos_malloc(size_t size);
void rtos_free(void *ptr);

#ifdef __cplusplus
}
#endif

#endif /* MISC_H_ */