//=================================================================================================
//- File: stm32g031xx_spi.c
//- Creation Date: 22_0211A / Modify : 22_0211A
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

#include "stm32f030f4p6_spi.h"

//=====================================================================================================================
//= Name  : IRQHandler_SPI1 (interrupt)
//= Cycle : One
//= Date  : 21.0427A
//=====================================================================================================================
void SPI1_IRQHandler(void)
{

}

void SPI2_IRQHandler(void)
{

}

//=====================================================================================================================
//= Name  : _fSPI_INIT
//= Cycle : One
//= Date  : 21.0427A
//=====================================================================================================================
void _fSPI_INIT(void)
{
    //- [ SPI1_CR1 (Reset value: 0x0000) ]
    //- 15[BIDI MODE], 14[BIDI OE], 13[CRC EN], 12[CRC NEXT], 11[CRCL], 10[RX ONLY]
    //- 9[SSM], 8[SSI], 7[LSB FIRST], 6[SPE], 5-3[BR], 2[MSTR], 1[CPOL], 0[CPHA]

    //- Set: 5-3[BR](48Mhz/32 = 1.5Mhz), 2[MSTR] 
    SPI1->CR1 = (_dSPI_CR1_SSM | _dSPI_CR1_SSI | _dSPI_CR1_MASTER | _dSPI_CR1_BR32 | _dSPI_CR1_CPOL | _dSPI_CR1_CPHA); 

    //- [ SPI1_CR2 (Reset Value: 0x0700) ]
    //- 14[LDMA_TX], 13[LDMA_RX], 12[FRXTH], 11-8[DS], 
    //- 7[TXEIE], 6[RXNEIE], 5[ERRIE], 4[FRF], 3[NSSP], 2[SSOE], 1[TXDMAEN], 0[RXDMAEN]

    //- SPI1_CR2: Reset Value
    SPI1->CR2 |= (0x0700 | _dSPI_CR2_FRXTH);    

    //- SPI1_CR1: 6[SPE]SPI Enable
    SPI1->CR1 |= _dSPI_CR1_SPE;
}

//=====================================================================================================================
//= Name  : _fSPI1_SEND
//= Cycle : One
//= Date  : 21.0427A
//=====================================================================================================================
tu32 _fSPI1_SEND(tu8 vData)
{
    static tu32 vErr;
    vErr = 0;
    if(0u == (SPI1->SR & _dSPI_SR_BSY))
    {
        if(0u != (SPI1->SR & _dSPI_SR_TXE))
        {
            *(volatile tu8 *)(&SPI1->DR) = vData;    //- 8bit Data send
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

//=====================================================================================================================
//= Name  : _fSPI1_BSY
//= Cycle : One
//= Date  : 21.0427A
//=====================================================================================================================
tu32 _fSPI1_BSY(void)
{
    static tu32 vErr;
    vErr = 0;
    if(0u == (SPI1->SR & _dSPI_SR_BSY))
    {
        if(0u == (SPI1->SR & _dSPI_SR_TXE))
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


