/********************************************************************************************************************
 * Device Target : STM32F030F4P6
 * File Name     : device_uart.c
 * Update ver    : 20.1030A
 * Encoding      : UTF-8
 *******************************************************************************************************************/

#include "stm32f030f4p6_uart.h"

#define _dUART_RXNE  0x00000020u
#define _dUART_TXE   0x00000080u
#define _dUART_TC    0x00000040u

//- Uart1 Buff
tu8 _gaU1TxBuff[_dU1TX_BUFF_SIZE] = {0u,};
tu8 _gaU1RxBuff[_dU1RX_BUFF_SIZE] = {0u,};
_tsUartBuff_Type _sUART1_TxInfo = {_dU1TX_BUFF_SIZE, 0u, 0u, 0u, 0u, 0u, _gaU1TxBuff};
_tsUartBuff_Type _sUART1_RxInfo = {_dU1RX_BUFF_SIZE, 0u, 0u, 0u, 0u, 0u, _gaU1RxBuff};

//=====================================================================================================================
//= Name  : USART1_IRQHandler (Interrupt)
//= Cycle : One
//= Date  : 19.1202A
//=====================================================================================================================
void USART1_IRQHandler(void)
{
    if( (USART1->ISR & _dUART_RXNE) != 0u )
    {
        //- Check Ring Buff Position Reset
        if( _sUART1_RxInfo.vSavePos == 0u ) _sUART1_RxInfo.vSavePos = _dU1RX_BUFF_SIZE;
        //- Next Buff Position
        _sUART1_RxInfo.vSavePos--;
        //- Buff full Check
        if(_sUART1_RxInfo.vSavePos == _sUART1_RxInfo.vReadPos) _sUART1_RxInfo.vOVF++; //- Overflow count.
        else _sUART1_RxInfo.pData[_sUART1_RxInfo.vSavePos] = (tu8)USART1->RDR; //- Data read and interrupt auto clear.
    }
}

void USART1_IRQHandler(void)
{

}

void LPUART1_IRQHandler(void)
{

}

//=====================================================================================================================
//= Name  : _fUART_INIT
//= Cycle : One
//= Date  : 19.1202A
//=====================================================================================================================
void _fUART_INIT(void)
{
    _fUART1_BUFF_INIT();
    _fUART1_REG_INIT();
}


//====================================================================================================================
//= Name  : _fUART1_REG_INIT (Register Initializing)
//= Cycle : One
//= Date  : 19.1202A
//=====================================================================================================================
void _fUART1_REG_INIT(void)
{
    //- RCC UART Clock Enable!!!
    USART1->CR3 = 0x00001000u;
    USART1->CR1 |= 0x0000002Cu;     //- 수신인터럽트 송수신 블럭 활성 RXNEIE(1), TE(1), RE(1) 설정
    USART1->CR2 = 0x00;             //- Reset
    USART1->BRR = 0x00000068u;      //- x16 Sampling 48Mhz/460800bps = 100d = 68h
    USART1->CR1 |= 0x00000001u;     //- UART Enable
    //- Ring buff reset
    _sUART1_TxInfo.vSavePos = 0u;
    _sUART1_TxInfo.vReadPos = 0u;
    _sUART1_RxInfo.vSavePos = 0u;
    _sUART1_RxInfo.vReadPos = 0u;
}

//====================================================================================================================
//= Name  : _fUART1_BUFF_INIT (Buff Data Initializing)
//= Cycle : One
//= Date  : 19.1202A
//=====================================================================================================================
void _fUART1_BUFF_INIT(void)
{
    tu32 vDEV_Cnt = 0u;
    while(vDEV_Cnt < _dU1RX_BUFF_SIZE)
    {
        _sUART1_RxInfo.pData[vDEV_Cnt] = 0u;
        vDEV_Cnt++;
    }
    vDEV_Cnt = 0u;
    while(vDEV_Cnt < _dU1TX_BUFF_SIZE)
    {
        _sUART1_TxInfo.pData[vDEV_Cnt] = 0u;
        vDEV_Cnt++;
    }
}

//=====================================================================================================================
//= Name  : _fUART1_SEND (UART Send Control)
//= Cycle : One
//= Date  : 19.1202A
//=====================================================================================================================
_teMsgCode_Type _fUART1_SEND(tu8 D_vData)
{
    _teMsgCode_Type eRetrun = _mMSG_CODE_ERR;
    if(0u != (USART1->ISR & _dUART_TC)) //- Transmission complete
    {
        if(0u != (USART1->ISR & _dUART_TXE))  //- TX Buff empty
        {
            USART1->TDR = D_vData;    //- Data Send
            eRetrun = _mMSG_CODE_OK;
        }
    }
    return(eRetrun);
}


//=====================================================================================================================
//= Name  : _fUART1_TX_BUFF
//= Cycle : One
//= Date  : 19.1202A
//=====================================================================================================================
_teMsgCode_Type _fUART1_TX_BUFF(tu8 vData)
{
    _teMsgCode_Type _eRetrun = _mMSG_CODE_ERR;

    //- Check Ring Buff Position Reset
    if( _sUART1_TxInfo.vSavePos == 0u ) _sUART1_TxInfo.vSavePos = _dU1TX_BUFF_SIZE;  
    //- Next Buff Position
    _sUART1_TxInfo.vSavePos--;

    //- 현재 쓰기 위치와 읽기 위치가 1개 버퍼일때 어버플러 발생.
    //- 오버 플로 발생시 기존데이터를 덥어 쓰기를 안함.
    if(  _sUART1_TxInfo.vSavePos == _sUART1_TxInfo.vReadPos )
    {
        _sUART1_TxInfo.vOVF++;             //- 전송 버퍼가 꽉찰경우 오버 카운터 증가
        _sUART1_TxInfo.vSavePos++;    //- 위치 다시 원래대로
        if(_sUART1_TxInfo.vSavePos == _dU1TX_BUFF_SIZE ) _sUART1_TxInfo.vSavePos = 0;
    }
    else
    {
         _sUART1_TxInfo.pData[_sUART1_TxInfo.vSavePos] = vData; //- 전송 버퍼에저장
         _eRetrun = _mMSG_CODE_OK;
    }
    return(_eRetrun);
}

//=====================================================================================================================
//= Name  : fUART1_TX_STRING
//= Cycle : One
//= Date  : 19.1202A
//=====================================================================================================================
void _fUART1_TX_STRING(tu8 *pString)
{
    static tu32 vPos;
    vPos = 0u;

    while( pString[vPos] != 0u )  //- Null Check
    {
        _fUART1_TX_BUFF(pString[vPos]);   //- 문자 저장
        vPos++;
    }
}

