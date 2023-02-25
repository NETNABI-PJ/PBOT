//=================================================================================================
//- File: stm32g031xx_core.c
//- Creation Date: 22_0211A / Modify : 22_0207A
//- Encoding: UTF-8
//=================================================================================================
//- Code Style
//- 01. a : a_Data[] (Array Variable)
//- 02. v : v_Data (Variable)
//- 03. e : e_Data (Enum)
//- 04. m : m_Data (Enum Member)
//- 05. d : d_Data (Define)
//- 06. u : u_Data (Union)
//- 07. p : p_Data (Pointer)
//- 08. t : t_Data (TypeDef)
//- 09. s : s_Data (Structuer)
//- 10. g : gv_Data (Global Variable)
//=================================================================================================
//=================================================== [Define] ========================================================

//=================================================== [Include] ========================================================

#include "../Inc/stm32g031xx_interrupt.h"
#include "../Inc/stm32g031xx_rcc.h"
#include "../Inc/stm32g031xx_gpio.h"
#include "../Inc/stm32g031xx_timer.h"
#include "../Inc/stm32g031xx_uart.h"
#include "../Inc/stm32g031xx_i2c.h"
#include "../Inc/stm32g031xx_spi.h"
#include "../Inc/stm32g031xx_core.h"

tu32 gvSYSLOOP = 0u;     //- 시스템 루프
tu32 gvLoopSpeed = 0;    //- 시스템 루프 스피드

//=====================================================================================================================
//= Name  : _fDEVICE_INIT
//= Cycle : one
//= Date  : 19.1202A
//=====================================================================================================================
void _fDEVICE_INIT(void)
{
    _fRCC_INIT();   //- 기본 클럭 소스 외부 설정
    _fGPIO_INIT();  //- GPIO 입출력 설정.
    _fUART_INIT();  //- UART 설정
    _fI2C_INIT();   //- I2C 설정
    _fSPI_INIT();   //- SPI 
    _fTIM_INIT();   //- TIM
    _fNVIC_INIT();  //- 인터럽트 설정
   // _fDMA_INIT(); // - DMA
}

void f_DWT_ENABLE(void)
{

}

void f_Core_DWT_Disable(void)
{

}
