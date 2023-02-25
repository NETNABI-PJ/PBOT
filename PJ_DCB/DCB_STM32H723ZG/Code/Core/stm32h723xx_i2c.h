//=================================================================================================
//- File: stm32h723xx_i2c.h
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
#ifndef _D_STM32H723XX_I2C_H_
#define _D_STM32H723XX_I2C_H_

#include "stm32f030x6_reg.h"

#define _dI2C_ISR_NACKF 0x10
#define _dI2C_ISR_TXIS  0x02

//= [FUNCTION] ========================================================================================================
void _fI2C_INIT(void);

#endif //- D_STM32F030F4P6_DEVICE_I2C_H_20_1102A


