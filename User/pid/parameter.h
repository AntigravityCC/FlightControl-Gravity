#ifndef __PARAMETER_H
#define __PARAMETER_H

#include "include.h"



void  parameter_init(float set);
extern uint16_t parameter_buffer[9];
void parameter_set(uint8_t * parameter_buffer,uint8_t len);
#endif





