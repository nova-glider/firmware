#include "main.h"
#include "sensors.h"
#include "stm32f1xx_hal.h"

void writeandsend(SENSORDATA *sensordata);
void initsd(void);
int write(char *filename, char *data);
void writeheaders(char *filename);