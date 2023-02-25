
//=================================================================================================
//- File: error_msg.h
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
#ifndef _D_ERROR_MSG_H_
#define _D_ERROR_MSG_H_

typedef enum
{
    emMSGCODE_NONE,
    meMSGCODE_OK,
    msMSGCODE_ERR,
    emMSGCODE_NOK
}te_MsgCode;

#endif