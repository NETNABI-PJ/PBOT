//=================================================================================================
//- File: stm32g031xx_i2c.c
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
#include "stm32f030f4p6_i2c.h"

//=====================================================================================================================
//= Name  : IRQHandler_I2C1 (Interrupt)
//= Cycle : One
//= Date  : 21.0427A
//=====================================================================================================================
void IRQHandler_I2C1(void)
{
    
}

//=====================================================================================================================
//= Name  : D_fGPIO_INIT (GPIO INIT)
//= Cycle : One
//= Date  : 19.1202A
//=====================================================================================================================
void _fI2C_INIT(void)
{
    //- I2C_CR1 = 23[PECEN], 22[ALERTEN], 21[SMBDEN], 20[SMBHEN], 19[GCEN], 17[NOSTRETCH], 16[SBC]
    //- 15[RXDMAEN], 14[TXDMAEN], 12[ANFOFF], 11-8[DNF], 7[ERRIE], 6[TCIE], 5[STOP], 4[NACKIE], 3[ADDRIE]
    //- 2[RXIE], 1[TXIE], 0[PE]
    I2C1->CR1 = 0x00000000; //- RC1 기본값으로 설정.
    //- I2C_CR2 = 26[PECBYTE], 25[AUTOEND], 24[RELOAD], 23-16[NBYTES], 15[NACK], 14[START], 12[HEAD10R]
    //- 11[ADD10], 10[RE_WRN], 9-0[SADD]
    I2C1->CR2 = 0x00000000; //- CR2 기본값으로 설정.
    // - I2C_TIMINGR = 31-28[PRESC], 23-20[SCLDEL], 19-16[SDADEL], 15-8[SCLH], 7-0[SCLL]
    //- PRESC= 2, (48,000,000 / (2+1) = 62.5ns)
    //- SCLL= 23 (62.5ns x (23+1) = 1500ns)
    //- SCLH= 11 (62.5ns x (9+1) = 625ns)
    //- tSCL = 2150ns
    I2C1->TIMINGR = 0x20310917u;
    I2C1->CR2 = 0x0100003Cu;    //- 24[RELOAD], 9-0[SADD]0011 1100
    I2C1->CR1 = 0x00000001u;    //- 0[PE]1

}

tu32 _fI2C_SEND(tu8 vBuff)
{
    static tu32 vErr = 0;
    if(0u != (_dI2C_ISR_NACKF & I2C1->ISR))
    {
        if(0u != (_dI2C_ISR_TXIS & I2C1->ISR))
        {
            I2C1->TXDR = vBuff;
        }
        else
        {
            vErr++;
        }
    }
    else
    {
        vErr++;
    }
    return(vErr);
}
