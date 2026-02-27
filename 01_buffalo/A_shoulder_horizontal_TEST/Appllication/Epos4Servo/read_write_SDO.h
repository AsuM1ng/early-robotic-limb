
#ifndef _READ_WRITE_SDO_H__
#define _READ_WRITE_SDO_H__
#include "data.h"

#define SUCCEED     0x00
#define FAIL        0xFF

#define CAN_READ_FAIL       0xFF
#define CAN_READ_SUCCEED    0x00

typedef enum
{
    ReadProcess     = 1,
    ReadSuccess     = 2,
    ReadFail        = 3,
    
    WriteProcess    = 4,
    WriteSuccess    = 5,
    WriteFail       = 6
}SDOreadStatus_en;

typedef struct{
    _Bool ProcessEnable;                /*SDO读取使能*/
    SDOreadStatus_en Sdo_Status;        /*SDO读取状态*/

    OD_DATATYPE_ENUM dataType;          /*要操作的数据类型*/
    uint16_t WriteLostCount;
    uint16_t ReadLostCount;
    uint8_t NodeId;         /*要操作节点号*/
    uint16_t Index;         /*要操作的对象*/
    uint8_t subIndex;       /*子对象*/
    int32_t Data;          /*数据*/
}SDORW_t;

extern SDORW_t Sdo_R_Para;
extern SDORW_t Sdo_W_Para;

uint8_t SdoReadPowerSupply(SDORW_t *Sdo_R_Para);

void sdoWriteNetworkDictCallBackAI(CO_Data* d, UNS8 nodeId, UNS16 index,UNS8 subIndex, UNS32 count, UNS8 dataType, 
                                   void *data, SDOCallback_t Callback, UNS8 endianize, UNS8 useBlockMode);

void sdoReadNetworkDictCallBackAI(CO_Data* d, UNS8 nodeId, UNS16 index,UNS8 subIndex, UNS32 *size, UNS8 dataType, 
                                   void *data, SDOCallback_t Callback, UNS8 useBlockMode);

uint8_t SdoReadProcess(SDORW_t *Sdo_R_Para);
void SdoReadAnalysis(SDORW_t *Sdo_R_Para, Message *m);

uint8_t SdoWriteProcess(SDORW_t *Sdo_W_Para);
void SdoWriteAck(SDORW_t *Sdo_W_Para, Message *m);

#endif

