/**
 * @defgroup   ACTUATORS actuators.cpp
 *
 * @brief      Flywheel helper functions. 
 * @details    This file implements helper functions for initializing actuator objects such as the flywheel.
 *
 * @author     Garrett Wells, Parker Piedmont
 * @date       2022
 */
#ifndef __ACTUATORS_H__
#define __ACTUATORS_H__

#include "global_definitions.h"
#include "DRV10970.h"
#include"ZXMB5210.h"


extern DRV10970 flywhl;

void initFlyWhl(void);
int RPS(void); 
void initMtx(void);

#endif