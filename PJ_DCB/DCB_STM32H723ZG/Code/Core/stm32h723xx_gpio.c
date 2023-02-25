//=================================================================================================
//- File: stm32g031xx_gpio.c
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

#include "stm32f030f4p6_gpio.h"    //- deivce gpio header file

//=====================================================================================================================
//= Name  : _fGPIO_INIT (GPIO INIT)
//= Cycle : One
//= Date  : 19.1202A
//=====================================================================================================================
void _fGPIO_INIT(void)
{
    //- RCC 초기화에서 PORTA,B 블럭에 클럭공급 설정 선행

    //- [PORT A]--------------------------------------------------
    //- 31-30[P15],....1-0[P0]
    //- 00 = Input. 01 = OUTPUT, 10 = AF, 11 = Analog
    //- P0,1=Analog, P2,3,4=IN, P5=OUT, P6,7,9,10=AF
    //- Reset value : 0x28000000
    //GPIOA->MODER = 0x2828A9A5u;    //- PORTA2 Mode Set +  Reset value

    //- GPIOA_MODER = 31-30[P15],....1-0[P0]
    //- Input = 00. OUTPUT = 01 , AF = 10, Analog = 11
    //- Reset value : 0x28000000
    //- Analog=x, IN=x, OUT=PA4,6, AF = PA2,3,5,7,9,10
    GPIOA->MODER = 0x282899A0u;    //- PORTA2 Mode Set +  Reset value

    //- 15[P15] ~...0[P0]
    GPIOA->OTYPER = 0x00000000u;    //- PORTA Output push-pull

    //- 31-30[P15],....1-0[P0]
    //- Speed Low = x0, Medium = 01, High = 11
    //- Reset value : 0x0C000000
    GPIOA->OSPEEDR = 0x0C3CFFFFu;    //0x0C14FD55u;  //- PORTA High SPEED

    //- 31-30[P15],....1-0[P0]
    //- Reset value : 0x24000000
    GPIOA->PUPDR = 0x24140000u;     //- Pull Up I2c(PA9, 10) +  Reset value

    //- 31-28[AF_P15], 27-24[AF_P14], 23-20[AF_P13], 19-16[AF_P12]
    //- 15-12[AF_P11], 11-8[AF_P10], 7-4[AF_P9], 3-0[AF_P8]
    //- AF0 ~ AF7
    GPIOA->AFR[1] = 0x00000440u;       //- I2C(PA9,10)AF4,

    //- 31-28[AF_P7], 27-24[AF_P6], 23-20[AF_P5], 19-16[AF_P4]
    //- 15-12[AF_P3], 11-8[AF_P2], 7-4[AF_P1], 3-0[AF_P0]
    //- AF0 ~ AF7
    GPIOA->AFR[0] = 0x00001100u;       //- PORTA UART1(PA2,3)AF1, SPI1(PA5,7)AF0

    //- [PORT B]--------------------------------------------------
    //- 31-30[P15],....1-0[P0]
    //- 00 = Input. 01 = OUTPUT, 10 = AF, 11 = Analog
    //- P1=OUT
    GPIOB->MODER = 0x00000004u;    //- PORTB Mode Set PB1 10

    //- 15[P15] ~...0[P0]
    GPIOB->OTYPER = 0x00000000u;    //- PORTB Output push-pull

    //- 31-30[P15],....1-0[P0]
    GPIOB->OSPEEDR = 0x0000000Cu;  //- PORTB High SPEED P1

    //- 31-30[P15],....1-0[P0]
    GPIOB->PUPDR = 0x00000000U;     //- PORTB No Pull-up,Down

    //- 31-28[AF7], 27-24[AF6], 23-20[AF5], 19-16[AF4]
    //- 15-12[AF3], 11-8[AF2], 7-4[AF1], 3-0[AF0]
    GPIOB->AFR[1] = 0x00000000u;       //- PORTB
    GPIOB->AFR[0] = 0x00000000u;       //- PORTB

}
