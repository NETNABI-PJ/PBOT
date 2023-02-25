
//=================================================================================================
//- File: stm32g031xx_dma.c
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
#include "stm32f030f4p6_dma.h"
#include "stm32f030f4p6_uart.h"
#include "stm32f030f4p6_spi.h"

//- SPI_TX(CH3), SPI_RX(CH2)
//- I2C1_TX(CH2), I2C1_RX(CH3)
//- USART1_TX*(CH4), USART1_RX*(CH5)

void DMA1_Channel1_IRQHandler(void)
{

}

void DMA1_Channel2_3_IRQHandler(void)
{

}

void DMA1_Ch4_5_DMAMUX1_OVR_IRQHandler(void)
{
    
}

void _fDMA_INIT(void)
{

    //- DMA_IFCR: 19[CTEIF5], 18[CHTIF5], 17[CTCIF5], 16[CGIF5]
    //-           15[CTEIF4], 14[CHTIF4], 13[CTCIF4], 12[CGIF4]
    //-           11[CTEIF3], 10[CHTIF3],  9[CTCIF3],  8[CGIF3]
    //-            7[CTEIF2],  6[CHTIF2],  5[CTCIF2],  4[CGIF2]
    //-            3[CTEIF1],  2[CHTIF1],  1[CTCIF1],  0[CGIF1]
    DMA1->IFCR = 0x000FFFFF;    //- Interrupt Flag All clear
    //- DMA_CCR: 14[MEM2MEM], 13-12[PL], 11-10[MSIZE], 9-8[PSIZE]
    //-           7[MINC], 6[PINC], 5[CIRC], 4[DIR], 3[TEIE], 2[HTIE], 1[TCIE], 0[EN]

    
    DMA1_Channel3->CCR = DMA_CCR_CIRC | DMA_CCR_PL_0 | DMA_CCR_DIR | DMA_CCR_PINC;
    DMA1_Channel3->CNDTR = _dU1TX_BUFF_SIZE;
    DMA1_Channel3->CMAR = _gaU1TxBuff;
    DMA1_Channel3->CPAR = &SPI1->DR;

    DMA1_Channel5->CCR = DMA_CCR_CIRC | DMA_CCR_PL_0 | DMA_CCR_MINC;
    DMA1_Channel5->CNDTR = _dU1RX_BUFF_SIZE;
    DMA1_Channel5->CMAR = _gaU1RxBuff;
    DMA1_Channel5->CPAR = &USART1->RDR;
    
    DMA1_Channel3->CCR |= DMA_CCR_EN;
    DMA1_Channel5->CCR |= DMA_CCR_EN;


}