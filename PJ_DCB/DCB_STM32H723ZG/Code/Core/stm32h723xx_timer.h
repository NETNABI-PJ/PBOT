//=================================================================================================
//- File: stm32h723xx_timer.h
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
#ifndef _D_STM32H723XX_TIMER_H_
#define _D_STM32H723XX_TIMER_H_

#include "stm32f030x6_reg.h"

#define _dTIM01_ENABLE
#define _dTIM30_ENABLE
#define _dTIM14_ENABLE

#define _dSYSTICK (SysTick->VAL)
#define _dTIME_GET ((tu16)TIM14->CNT)    //- 타이머

typedef struct  
{
    tu32 OT;     //- Old time
    tu32 CT;     //- current time
    tu32 TT;     //- Totla time
    tu32 ST;     //- Set Time
}_tsTimeInfo_Type;

//= [FUNCTION] ========================================================================================================
tu32 _fTIME_CHECK (_tsTimeInfo_Type* sTime);
void _fTIM_INIT(void);
void _fTIM14_INIT(void);

#endif
