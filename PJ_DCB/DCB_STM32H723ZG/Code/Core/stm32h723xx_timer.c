//=================================================================================================
//- File: stm32g031xx_timer.c
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

#include "stm32f030f4p6_timer.h"


//=====================================================================================================================
//= Name  : _fTIM_INIT (timer init)
//= Cycle : One
//= Date  : 19.1202A
//=====================================================================================================================
void _fTIM_INIT(void)
{
    _fTIM14_INIT();
}

void TIM1_BRK_UP_TRG_COM_IRQHandler(void)
{

}
void TIM1_CC_IRQHandler(void)
{

}
void TIM2_IRQHandler(void)
{

}
void TIM3_IRQHandler(void)
{

}
void LPTIM1_IRQHandler(void)
{

}
void LPTIM2_IRQHandler(void)
{

}
//=====================================================================================================================
//= Name  : _fTIM14_INIT (Timer14 init)
//= Cycle : One
//= Date  : 19.1202A
//=====================================================================================================================
void _fTIM14_INIT(void)
{
    //- [SYSTEM TIMER] --------------------------------------------------------------------------------
    //- Output Mode(CCIS = 00일때) 6-4[OC1M], 3[OC1PE], 2[OC1FE], 1-0[CC1S]
    //- Input Mode  7-4[IC1F], 3-2[IC1OSC], 1-0[CC1S]
    TIM14->CCMR1 = 0x0000;  //- input mode fDTS/32 분주후 8번 검사 설정
    //- 3[CC1NP], 1[CC1P], 0[CC1E]
    TIM14->CCER = 0x0000;   //- 캡쳐 동작
    //- 1[CC1IE], 0[UIE]
    TIM14->DIER = 0x0000u;  //- CC1IE 인터럽트 활성
    //- 15-0[PSC]
    TIM14->PSC = 4799u;     //- 클럭/4799 마다 타이머 클럭발생
    //- 15-0[ARR]
    TIM14->ARR = 0xFFFFu;   //- 오토 로드 설정 값
    //- 9-8[CKD], 7[ARPE], 2[URS], 1[UDIS], 0[CNE]
    TIM14->CR1 = 0x0081u;   //- TIM16 동작. 및 버퍼 (ARPE1, CEN1)

    while( (TIM14->SR & 0x00000001u) == 0u);  //- wait 카운터 오버플러 이벤트 발생후 설정값 적용됨
}
//=====================================================================================================================
//= Name  : _fTIME_CHECK (time flag check, down counter)
//= Cycle : One
//= Date  : 19.1202A
//=====================================================================================================================
tu32 _fTIME_CHECK (_tsTimeInfo_Type* sTime)
{
    static tu32 vErr;
    vErr = 1u;

    sTime->CT = _dTIME_GET;
    if(sTime->OT <= sTime->CT)
    {
        sTime->TT += (sTime->CT - sTime->OT);
    }
    else
    {
        sTime->TT += (sTime->CT + (0x0000FFFFu - sTime->OT));
    }
    sTime->OT = sTime->CT;

    if(sTime->TT >= sTime->ST)
    {
        sTime->TT -= sTime->ST;
        vErr = 0;
    }
    return(vErr);
}
