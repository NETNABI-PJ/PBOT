/**************************************************************************************************
//  FileName : core_time.c
//  Author   : Netnabi
//  Date     : 2024.05.03 New!
//  Modify   : 2023.05.03 
//-------------------------------------------------------------------------------------------------
//  coding info
01. a = array variable         e.g. a_Data[]
02. b = bit fields             e.g. b_Data
03. c = const                  e.g, c_Data
04. d = define                 e.g. d_Data
05. e = enum Type              e.g. e_Data
06. f = Function               e.g. f_Data
07. g = source global          e.g. gv_Data, ga_Data...
08. i = Inline                 e.g. i_Data
09. l = local variable         e.g. lv_Data
10. m = enum member            e.g. m_Data
11. p = Pointer                e.g. p_Data
12. s = struct                 e.g. s_Data
13. t = typedef                e.g. t_Data
14. u = Union                  e.g. u_Data
15. v = variable               e.g. v_Data
16. x = Extern                 e.g. xv_Data
**************************************************************************************************/
#include "core_time.h"

#define d_CORETIME_TIME_CNT 0

ts_CoreTime ls_CoreTime_10ms;
ts_CoreTime ls_CoreTime_20ms;
ts_CoreTime ls_CoreTime_100ms;
ts_CoreTime ls_CoreTime_500ms;
ts_CoreTime ls_CoreTime_1s;

void f_CoreTime_Init(void)
{

}

void f_CoreTime_Module(void)
{

}

void f_CoreTime_Polling_Set(ts_CoreTime *s_Time, uint32 v_TimeUS)
{
    s_Time->v_PT = d_CORETIME_TIME_CNT;
    s_Time->v_CT = d_CORETIME_TIME_CNT;
    s_Time->v_ST = d_CORE_TIME_SET_US(v_TimeUS);
    s_Time->v_TT = 0;

}

//=================================================================================================
//- Core Time Check
//=================================================================================================
uint32 f_CoreTime_Check(ts_CoreTime* s_Time)
{
    static uint32 v_OK;
    v_OK = 0;
    s_Time->v_CT = d_CORETIME_TIME_CNT;
    if(s_Time->v_CT != s_Time->v_PT)
    {
        if(s_Time->v_CT >= s_Time->v_PT)
            { s_Time->v_TT += (s_Time->v_CT - s_Time->v_PT); }
        else
            { s_Time->v_TT += (s_Time->v_CT + (0xFFFFFFFF - s_Time->v_PT)); }
        s_Time->v_CT = s_Time->v_CT;
        if(s_Time->v_TT >= s_Time->v_ST)
        {
            s_Time->v_TT -= s_Time->v_ST;
            v_OK++;
        }
    }
    return(v_OK);
}