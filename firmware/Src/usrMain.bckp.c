#include "usrMain.h"
#include "main.h"
//#include "AMIS30543"
#include "handles.h"
uint32_t c = 0;


uint8_t buffer[100] = "asdfghij";
uint8_t size = 5;

void writeReg(uint8_t address, uint8_t value)
{
	uint8_t addr = 0x80 | (address & 0b11111);
	uint8_t buf;
	HAL_SPI_TransmitReceive(spi1Handle, &addr, &buf, 1, HAL_MAX_DELAY);
	HAL_SPI_TransmitReceive(spi1Handle, &value, &buf, 1, HAL_MAX_DELAY);
}


struct Motor
{
	GPIO_TypeDef *port;
	uint16_t pin;
	uint8_t wr, cr0, cr1, cr2, cr3;
} motors[3];

void applySettings(struct Motor motor)
{
	HAL_GPIO_WritePin(motor.port, motor.pin, 0);
	HAL_Delay(1);
	writeReg(0x3, motor.cr2);
    writeReg(0x0, motor.wr);
    writeReg(0x1, motor.cr0);
    writeReg(0x2, motor.cr1);
    writeReg(0x9, motor.cr3);
	HAL_GPIO_WritePin(motor.port, motor.pin, 1);
	HAL_Delay(1);
}

void setStepMode(struct Motor motor, uint8_t mode)
{
    uint8_t esm = 0b000;
    uint8_t sm = 0b000;
    switch(mode)
    {
        case 1: esm = 0b011; break;
        case 2: sm = 0b100; break;
        case 4: sm = 0b011; break;
        case 8: sm = 0b010; break;
        case 16: sm = 0b001; break;
        case 32: sm = 0b000; break;
        case 64: esm = 0b010; break;
        case 128: esm = 0b001; break;
    }
    motor.cr0 = (motor.cr0 & ~0b11100000) | (sm << 5);
    motor.cr3 = (motor.cr3 & ~0b111) | esm;
    applySettings(motor);
}

void enableMotor(struct Motor motor)
{
	motor.cr2 |= 0b10000000;
    applySettings(motor);
}

void timer_tick() // timer6 interrupt
{
	c++;
	if (c > 450000)
		HAL_GPIO_WritePin(NXT1_GPIO_Port, NXT1_Pin, ((c / (125000 / 700)) % 2));
}

int usrMain()
{
	motors[0].port = CS1_GPIO_Port;
	motors[0].pin = CS1_Pin;
	HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, 0);
	HAL_Delay(1);
	writeReg(0x03, 0x80);
	writeReg(0x01, 0b01000011);
	//HAL_Delay(1);
	//HAL_GPIO_WritePin(motors[0].port, motors[0].pin, 1);

    while(1)
    {
	    HAL_Delay(122);
	    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);
	    HAL_Delay(122);
	    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 1);
	}
	return 0;
}
