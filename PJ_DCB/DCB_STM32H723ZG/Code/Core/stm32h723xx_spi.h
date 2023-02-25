//=================================================================================================
//- File: stm32h723xx_spi.h
//- Creation Date: 22_0207A
//- Encoding: UTF-8
//- Modify : 22_0207A
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
#ifndef _D_STM32H723XX_SPI_H_
#define _D_STM32H723XX_SPI_H_

#include "stm32f030x6_reg.h"

#define _dSPI_CR1_CPHA      ((tu32) 0x00000001u)
#define _dSPI_CR1_CPOL      ((tu32) 0x00000002u)

#define _dSPI_CR1_SPE       ((tu32) 0x00000040u)
#define _dSPI_CR1_SSM       ((tu32) 0x00000200u)
#define _dSPI_CR1_SSI       ((tu32) 0x00000100u)

#define _dSPI_CR1_MASTER    ((tu32) 0x00000004u)
#define _dSPI_CR1_SLAVE     ((tu32) 0x00000000u)

#define _dSPI_CR1_CPOL0     ((tu32) 0x00000000u)
#define _dSPI_CR1_CPOL1     ((tu32) 0x00000002u)

#define _dSPI_CR1_CPHA0     ((tu32) 0x00000000u)
#define _dSPI_CR1_CPHA1     ((tu32) 0x00000001u)

#define _dSPI_CR1_BR2       ((tu32) 0x00000000u)
#define _dSPI_CR1_BR4       ((tu32) 0x00000008u)
#define _dSPI_CR1_BR8       ((tu32) 0x00000010u)
#define _dSPI_CR1_BR16      ((tu32) 0x00000018u)
#define _dSPI_CR1_BR32      ((tu32) 0x00000020u)
#define _dSPI_CR1_BR64      ((tu32) 0x00000028u)
#define _dSPI_CR1_BR128     ((tu32) 0x00000030u)
#define _dSPI_CR1_BR256     ((tu32) 0x00000038u)

#define _dSPI_CR1_BIDIOE    ((tu32) 0x40000000u)

#define _dSPI_CR2_RXNEIE    ((tu32) 0x00000040u)
#define _dSPI_CR2_FRXTH     ((tu32) 0x00001000u)
#define _dSPI_CR2_SSOE      ((tu32) 0x00000004u)
#define _dSPI_CR2_NSSP      ((tu32) 0x00000008u)

#define _dSPI_SR_BSY        ((tu32) 0x00000080u)
#define _dSPI_SR_TXE        ((tu32) 0x00000002u)
#define _dSPI_SR_RXNE       ((tu32) 0x00000001u)


//= [FUNCTION] ========================================================================================================
void _fSPI_INIT(void);
tu32 _fSPI1_SEND(tu8 vData);
tu32 _fSPI1_BSY(void);

#endif