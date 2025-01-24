#include "States.h"
#include "main.h"

double TempDif;
static unsigned short heater_state,fan_state,ld1ex_state,ld2ex_state,ld3ex_state;

/**
 *
 * @brief States_Change is a main function in our program,
 * its job is to measure difference between TempRef and temp,
 * then react with heater(if difference is positive and bigger than 0.04 degree),
 * or with fan(if difference is negative and less than -0.10).
 *
 * @param TempRef is our set temperature received from a user via UART
 * @param temp is our current temperature
 *
 */

void States_Change (double TempRef, double temp)
{
	 TempDif=TempRef-temp;

	 if (TempDif > 0.04)
	 {
		 heater_state = 1;
		 fan_state = 0;
		 ld1ex_state = 0;
		 ld2ex_state = 0;
		 ld3ex_state = 1;
	 }
	 else if (TempDif < -0.10)
	 {
		 heater_state = 0;
		 fan_state = 1;
		 ld1ex_state = 0;
		 ld2ex_state = 1;
		 ld3ex_state = 0;
	 }
	 else
	 {
		 heater_state = 0;
		 fan_state = 0;
		 ld1ex_state = 1;
		 ld2ex_state = 0;
		 ld3ex_state = 0;
	 }

}

/**
 *
 * @brief States_Run function in our program turns on GPIO Ports,
 * which our external control mechanism is connected to STM.
 * It's launching each port according to present state of pin,
 * depending on current states provided by States_Change function.
 *
 * @param None
 *
 */

void States_Run()
{
	HAL_GPIO_WritePin(Heater_GPIO_Port, Heater_Pin, heater_state);  	// Heater control
	HAL_GPIO_WritePin(Fan_GPIO_Port, Fan_Pin, fan_state);			    // Fan control
	HAL_GPIO_WritePin(LD1EX_GPIO_Port, LD1EX_Pin, ld1ex_state);		    // Control system insensitivity LED control
	HAL_GPIO_WritePin(LD2EX_GPIO_Port, LD2EX_Pin, ld2ex_state);		    // Fan LED control
	HAL_GPIO_WritePin(LD3EX_GPIO_Port, LD3EX_Pin, ld3ex_state);		    // Heater LED control
}
