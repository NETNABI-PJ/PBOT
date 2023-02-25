//=================================================================================================
//- File: stm32g031xx_rcc.c
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
#include "../Inc/stm32g031xx_reg.h"
#include "../Inc/stm32g031xx_rcc.h"




//=====================================================================================================================
//= Name        : fDEV_RCC_INIT
//= explanation : System colock init.
//= Cycle       : One
//= Date        : 21.0112A
//=====================================================================================================================
void _fRCC_INIT(void)
{
    //- 주변 클력 공급 설정
    //- RCC_AHBENR = 22[IOPF EN], 20[IOPD EN], 19[IOPC EN], 18[IOPB EN], 17[IOPA EN]
    //- 6[CRC EN], 4[FLITF EN] 2[SRAM EN], 0[DMA EN]
    RCC->AHBENR = 0x00460014u;  //- PORT A,B,F, FLTF, SRAM EN

    //- RCC_APB1ENR = 28[PWR EN], 23[USB EN], 22[I2C2 EN], 21[I2C1 EN], 20[USART5 EN], 19[USART4 EN] 18[USART3 EN]
    //- 17[USART2 EN], 14[SPI2 EN], 11[WWDG EN], 8[TIM14 EN], 5[TIM7 EN], 4[TIM6 EN], 1[TIM3 EN]
    RCC->APB1ENR = 0x00200100u;    //- I2C1 EN

    //- RCC_APB2ENR = 22[DBGMCU EN], 18[TIM17 EN], 17[TIM16 EN], 16[TIM15 EN]
    //- 14[USART1 EN], 12[SPI1 EN], 11[TIM1 EN], 9[ADC EN], 5[USART6 EN], 0[SYSCFG COMPEN]
    RCC->APB2ENR = 0x00005000u;    //- EN = UART1, SPI1
}

void RCC_IRQHandler(void)
{
    
}

//=====================================================================================================================
//= Name        : fDEV_RCC_STARTUP_INIT
//= explanation : Startup Code colock init.
//= Cycle       : One
//= Date        : 21.0112A
//=====================================================================================================================
void f_RCC_Startup(void)
{
    //- External 16Mhz Crystal Setting
    //- ### 외부16Mhz 크리스탈 동작을 위한 설정 ###
    //- 메모리 접근 딜레이를 1클럭 딜레이로 바꾼다.
    FLASH->ACR |= 0x11u;                    //- FLASH_ACR = 4[PRFEBE]1, 2~0[LATENCY]1 (One Wait)

    //- 외부 클럭 발진
    RCC->CR = (tu32) 0x00010081u;           //- RCC_CR = 16[HESON]1, 7-3[HSITRIM]16, 0[HSION]1 (HSE ON)
    while( (RCC->CR & 0x02u) != 0x02u );    //- RCC_CR = 1[HSIRDY]R1 (clock Ready ?)

    //-  PLL 설정
    RCC->CFGR = (tu32) 0x00050000u;         //- RCC_CFGR = 21-18[PLLMUL]1(x3), 16[PLLSRC]1 (16Mhz x 3 = 48Mhz)
    RCC->CFGR2 = (tu32) 0x00000000u;        //- RCC_CFGR2 = 3-0[PREDIV]0(/1)

    //-  PLL 동작 및  클럭 안정화를 확인 (고장일 일경우 무한 루프 이므로 적절한 처리가 필요)
    RCC->CR |= (tu32) 0x01000000u;           //- RCC_CR = 24[PLL_ON]1 (Start PLL)
    while( (RCC->CR & 0x02000000u) != 0x02000000u );    //- RCC_CR = 25[PLLRDY]R1 (PLL Ready ?)

    //- (HSI내부클럭 -> HSE외부클럭) 로 클럭 소스를 변경한다.
    //- 내부 HSI 클럭발진을 중단
    RCC->CFGR |= (tu32) 0x00000002u;         //- RCC_CFGR = 1-0[SW]2 (Select HSE PLL Source.)
    while( (RCC->CFGR & 0x0000000Cu) != 0x08u);
    RCC->CR &= (volatile tu32)0xFFFFFFFEu;           //- RCC_CR = 0[HSION]0 (Disable HSI)

    //- ### 주변 장치에 사용되는 클럭 소스 설정 ###
    //- USB는 사용안하고 기본 Disabled 라서 설정 불필요 건너뜀.
    //- I2C1 클럭 소스를 HSI -> SYSCLK 로 변경. (나중에 사용안하므로 설정 삭제)
    RCC->CFGR3 = (tu32) 0x00000010u;         //- RCC_CFGR3 = 4[I2C1SW]1 (I2c1 HSI -> SYSCLK select clk source )

    //- ADC용 HSI14는 비활성 외부클럭 에서 조정된 PCLK소스를 사용하기 위해서.
    RCC->CR2 &= (volatile tu32)0xFFFFFFFEu;          //- RCC_CR2 = 0[HSI14ON]0 (Oscillator OFF)
    
    //- 현재 인터럽트 
    RCC->CIR = 0x00000000u;                  //- Disable all interrupts

    //- SysTick(System Tick Timer) 설정 
    //- 자세한 SYSTICK 레지스터는 CORTEX-M0 코어  ARMv6-M ARM 메뉴얼 참조
    //- 16[COUNTFLAG], 2[CLKSOURCE], 1[TICKINT], 0[ENABLE]
    SysTick->LOAD = 0x00FFFFFFu;    //- Reload Val 24bit MAX
    SysTick->CTRL = 0x00000001u;    //- 외부 8분주 클럭 사용 및 SysTick 활성화
    SysTick->VAL = 0x00000000u;    //- 카운터값 리셋
}
