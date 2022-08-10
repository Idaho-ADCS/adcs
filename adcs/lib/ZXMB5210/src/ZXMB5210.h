#ifndef ZXMB5210_MAGNETORQUER_H
#define ZXMB5210_MAGNETORQUER_H

#include <Arduino.h>
#include <global_definitions.h>

class ZXMB5210 {
private:
	uint8_t fwd_pin, rev_pin, buck_enable=255;

public:
	ZXMB5210(uint8_t fwd, uint8_t rev, uint8_t buck);
	ZXMB5210(uint8_t fwd, uint8_t rev);

	void init();
	void fwd(void);
	void rev(void);
	void standby(void);
	void stop(void);
	

};

#endif