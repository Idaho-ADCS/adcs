#include "actuators.h"

// DRV10970 motor driver object
DRV10970 flywhl(MEN_PIN, FG_PIN, FR_PIN, 0, PWM_PIN, RD_PIN);  // pin 0 needs to be something else
ZXMB5210 Mtx1(MTX1_F_PIN,MTX1_R_PIN,BEN_PIN);
ZXMB5210 Mtx2(MTX2_F_PIN,MTX2_R_PIN,BEN_PIN);

/**
 * @brief      Initialize pins for flywheel motor driver, DRV10970
 */
void initFlyWhl(void)
{
	flywhl.init();
	#if DEBUG
		SERCOM_USB.print("[system init]\tFlywheel initialized\r\n");
	#endif
}

int RPS(void)
{
	int RPS =0; 
	RPS = int(flywhl.readRPS(false));
	return RPS; 
}

void initMtx(void)
{
	Mtx1.init(); 
	Mtx2.init();
	#if DEBUG
		SERCOM_USB.print("[system init]\tMTx1, MTx2 initalized\r\n");
	#endif
}