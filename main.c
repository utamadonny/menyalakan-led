#include "DrvGPIO.h"
#include "DrvSYS.h"
void Init();
int main(void)
{
    Init();
    while(1)
    {
    	DrvGPIO_ClrBit(E_GPC,15);
    	DrvSYS_Delay(500000);
    	DrvGPIO_SetBit(E_GPC,15);
    	DrvSYS_Delay(500000);
    }
}
