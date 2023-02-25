//=================================================================================================
//- File: stm32h723xx_core.h
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
#ifndef _D_STM32H723XX_CORE_H_
#define _D_STM32H723XX_CORE_H_

#include "stm32f030x6_reg.h"

//= [INCLUDE] =====================================================================

//= [DEFINE] ======================================================================

//= [ENUM] ======================================================================== 
typedef enum
{
    _mMSG_CODE_NONE,
    _mMSG_CODE_OK,
    _mMSG_CODE_CANCEL,
    _mMSG_CODE_ERR,
    _mMSG_CODE_PASS,
    _mMSG_CODE_FAIL
}_teMsgCode_Type;

//= [STRUCT] ======================================================================
typedef struct
{
    tu32 CPU_Load;          //- 퍼센트
    tu32 CPU_Load_us;       //- us 루프 측정
    tu32 UART1_BuffLoad;    //- 퍼센트
}gtsDEV_System_Info;

//= [EXTERN] ======================================================================
extern tu32 gvSYSLOOP;     //- 시스템 루프
extern tu32 gvLoopSpeed;    //- 시스템 루프 스피드

//= [FUNCTION] ========================================================================================================
void _fDEVICE_INIT(void);

#endif