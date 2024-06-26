/**************************************************************************************************
//  FileName : core_time_type.h
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
#ifndef _CORE_TIME_TYPE_H_
#define _CORE_TIME_TYPE_H_
#include "core__setting.h"

typedef struct _ts_CoreTime
{
    uint32 v_CT;     //- Current time.  (현재시간)
    uint32 v_PT;     //- Previous time. (이전시간)
    uint32 v_ST;     //- Setting time.  (설정시간)
    uint32 v_TT;     //- Total time.    (총시간)
}ts_CoreTime;  

#endif