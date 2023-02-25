//=================================================================================================
//- File: stm32h723xx_uart.h
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
#ifndef _D_STM32H723XX_UART_H_
#define _D_STM32H723XX_UART_H_

//= [INCLUDE] =====================================================================
#include "stm32f030x6_reg.h"
#include "stm32f030f4p6_core.h"
//= [DEFINE] ======================================================================
//= UART1 Buff
#define _dU1TX_BUFF_SIZE 256u
#define _dU1RX_BUFF_SIZE 512u

#define _dUART_9600
#define _ENABLE_ 1u
#define _DISABLE_ 0u

#define _NONE_ 0
#define _POL_  1    //- Polling
#define _INT_  2    //- Interrupt
#define _DMA_  3    //- DMA

//= [ UART Setting ]  Set 1 = Enable, 0 = Disable
#define _dUart1_Run     _DISABLE_
#define _dUart1_Speed   250000u
#define _dUart1_TX_Buff 256
#define _dUart1_TX_Ctrl _POL_
#define _dUart1_RX_Buff 256
#define _dUart1_     _Dis
#define _dUart2_Enable _Disable
//= [ENUM] ======================================================================== 
//= [STRUCT] ======================================================================
typedef struct
{
    volatile const tu32 vMaxSize;        //- Max Buff Size. 
    volatile tu32 vPCT;             //- Buff percentage (사용률 퍼센트)
    volatile tu32 vCNT;             //- Buff use count (사용중인 값)
    volatile tu32 vOVF;             //- Buff overflow (오버플로 발생 횟수)
    volatile tu32 vSavePos;         //- Save position (현재 까지 전송한 데이터 위치)
    volatile tu32 vReadPos;         //- Writ position (현재 까지 저장한 데이터 위치)
    volatile tu8* pData;            //- Data Buff (버퍼 포인터)
}_tsUartBuff_Type;
//= [EXTERN] ======================================================================
extern _tsUartBuff_Type _sUART1_TxInfo, _sUART1_RxInfo;
extern tu8 _gaU1TxBuff[_dU1TX_BUFF_SIZE];
extern tu8 _gaU1RxBuff[_dU1RX_BUFF_SIZE];
//= [FUNCTION] ========================================================================================================
_teMsgCode_Type _fUART1_SEND(tu8 D_vData);
_teMsgCode_Type _fUART1_TX_BUFF(tu8 vData);
void _fUART1_TX_STRING(tu8 *pString);
void _fUART1_BUFF_INIT(void);
void _fUART1_REG_INIT(void);
void _fUART_INIT(void);

void USART1_IRQHandler(void);

#endif
