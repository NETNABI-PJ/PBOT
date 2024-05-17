/**************************************************************************************************
//  FileName : typedef.h
//  Author   : Netnabi
//  Date     : 2024.05.11 New!
//  Modify   : 2023.05.11 
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
#ifndef _TYPEDEF_H_
#define _TYPEDEF_H_
//===================================================================================== [ TYPEDEF ]
typedef unsigned char           uint8;
typedef unsigned short          uint16;
typedef unsigned long           uint32;
typedef unsigned long long      uint64;

typedef signed char             sint8;
typedef short                   sint16;
typedef long                    sint32;
typedef long long               sint64;

typedef float                   f32;
typedef double                  f64;
//====================================================================================== [ DEFINE ]
#define d_YES       1
#define d_NO        0
#define d_ON        1
#define d_OFF       0
#define d_OK        1
#define d_NOK       0
#define d_NULL      0
#define d_DONE      0
#define d_FAULT     1

#endif