#include "types.h"

void DIO_Init();
void DIO_WritePin(vulong32_ptr port, uint8 pin, uint8 value);
void DIO_WritePort(vulong32_ptr port, uint32 value);
uint32 DIO_ReadPin(vulong32_ptr port, uint8 pin);
uint32 DIO_ReadPort(vulong32_ptr port);
void DIO_TogglePin(vulong32_ptr port,uint8 pin);