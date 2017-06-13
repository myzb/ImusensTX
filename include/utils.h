// Only modify this file to include
// - function definitions (prototypes)
// - include files
// - extern variable definitions
// In the appropriate section

#ifndef _UTILS_H_
#define _UTILS_H_
#include "Arduino.h"
//add your includes for the project imusens_tx here


//end of add your includes here


//add your function definitions for the project imusens_tx here
void I2Cscan();
void getEulers();
float getTimeDelta(uint32_t *now, uint32_t *lastUpdate);

//Do not add code below this line
#endif /* _UTILS_H_ */
