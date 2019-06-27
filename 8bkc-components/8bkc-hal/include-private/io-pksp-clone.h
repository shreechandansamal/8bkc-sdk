#ifndef IO_CLONE_H
#define IO_CLONE_H

#define IO_CHG_NOCHARGER 0
#define IO_CHG_CHARGING 1
#define IO_CHG_FULL 2

int ioJoyReadInput();
void ioAmplifierOn();
void ioAmplifierOff();
void ioInit();
void ioPowerDown();
void ioOledPowerDown();

/* not implemented yet
int ioGetChgStatus();
*/
int ioGetVbatAdcVal();
void ioVbatForceMeasure();

#endif