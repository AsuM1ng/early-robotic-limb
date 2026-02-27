
//#include "TestSlave.h"
#include "CANopenMaster.h"
#include "read_write_SDO.h"
#include "timer.h"
#include "SDO.h"
#include "chipHAL_delay.h"

#include "canfestival.h"


int sdoWritelost1=0;
int sdoWritelost2=0;
int sdoWritelost3=0;

int sdoWriteNetworkDictCallBackAI_error[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
/*******************************************************************************
 函数功能：SDO写
 参数说明：d             CAN_open结构体指针
           nodeId       与之通信的设备的节点号
           index        主索引
           subIndex     子索引
           count        数据大小
           dataType     数据类型
           data         数据源
           Callback     超时或中止回调函数
           endianize    数据源大小端格式设置选择
           useBlockMode 块模式选择
*******************************************************************************/
// this function may do a endless loop when sdo communication error, need to be modify
void sdoWriteNetworkDictCallBackAI(CO_Data* d, UNS8 nodeId, UNS16 index,UNS8 subIndex, UNS32 count, UNS8 dataType, 
                                   void *data, SDOCallback_t Callback, UNS8 endianize, UNS8 useBlockMode)
{
	UNS8  ret=0;
    UNS8  nowline = 0;	
	UNS8  result=0;	
	UNS32 abortCode=0;

	// white a sdo msg
	ret = writeNetworkDictCallBackAI (d,  nodeId, index, subIndex, count, dataType, data,  Callback,  endianize,  useBlockMode);

	
    // this part may trapped in endless loop when sdo communication err	
	while(ret == 0xFE || ret == 0xFF)
	{
		sdoWritelost1++;
		ret = writeNetworkDictCallBackAI (d,  nodeId, index, subIndex, count, dataType, data,  Callback,  endianize,  useBlockMode);
	}

/*
	while ((result=getWriteResultNetworkDict (d, &nowline, nodeId, &abortCode)) == SDO_DOWNLOAD_IN_PROGRESS);

    for(int i = 0;i<16;i++)
    {
        if(d->transfers[i].state == SDO_FINISHED && d->transfers[i].index == index && d->transfers[i].subIndex == subIndex)
        {
            resetSDOline(d, i);
            sdoWriteNetworkDictCallBackAI_error[i]++;
        }
    }

		
	while(result == SDO_ABORTED_INTERNAL || result == SDO_ABORTED_RCV )
	{
		sdoWritelost2++;
	
        resetSDOline(d, (UNS8)nowline);

        delay_100us(1000);

		sdoWritelost2++;
		ret = writeNetworkDictCallBackAI (d,  nodeId, index, subIndex, count, dataType, data,  Callback,  endianize,  useBlockMode);
		while(ret == 0xFE || ret == 0xFF)
		{
			sdoWritelost3++;
			ret = writeNetworkDictCallBackAI (d,  nodeId, index, subIndex, count, dataType, data,  Callback,  endianize,  useBlockMode);
		}
		while ((result=getWriteResultNetworkDict (d, &nowline, nodeId, &abortCode)) == SDO_DOWNLOAD_IN_PROGRESS);
        
        for(int i = 0;i<16;i++)
        {
            if(d->transfers[i].state == SDO_FINISHED && d->transfers[i].index == index && d->transfers[i].subIndex == subIndex)
            {
                resetSDOline(d, i);
                sdoWriteNetworkDictCallBackAI_error[i]++;
            }
        }
		
	}
*/

}


int sdoReadlost1=0;
int sdoReadlost2=0;
int sdoReadlost3=0;

int sdoReadNetworkDictCallBackAI_error[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
/*******************************************************************************
 函数功能：SDO读
 参数说明：d             CAN_open结构体指针
           nodeId       与之通信的设备的节点号
           index        主索引
           subIndex     子索引
           size         数据大小
           dataType     数据类型
           data         存放读取回来的数据
           Callback     超时或中止回调函数
           useBlockMode 块模式选择
*******************************************************************************/
void sdoReadNetworkDictCallBackAI(CO_Data* d, UNS8 nodeId, UNS16 index,UNS8 subIndex, UNS32 *size, UNS8 dataType, 
                                   void *data, SDOCallback_t Callback, UNS8 useBlockMode)
{
	UNS8  ret=0;	
    UNS8  nowline = 0;
	UNS8  result=0;	
	UNS32 abortCode=0;

	ret = readNetworkDictCallbackAI (d,  nodeId, index, subIndex, dataType,  Callback,  useBlockMode);
	
	while(ret >= 0xFE)
	{
		sdoReadlost1++;
		ret = readNetworkDictCallbackAI (d,  nodeId, index, subIndex, dataType,  Callback,  useBlockMode);
	}
	
	while ((result=getReadResultNetworkDict (d, &nowline,  nodeId, data, size, &abortCode)) == SDO_UPLOAD_IN_PROGRESS);

    
    for(int i = 0;i<16;i++)
    {
        if(d->transfers[i].state == SDO_FINISHED && d->transfers[i].index == index && d->transfers[i].subIndex == subIndex)
        {
            resetSDOline(d, i);
            sdoReadNetworkDictCallBackAI_error[i]++;
        }
    }
       
	while(result == SDO_ABORTED_INTERNAL || result == SDO_ABORTED_RCV )
	{
        resetSDOline(d, (UNS8)nowline);

        delay_ms(100);
        
		sdoReadlost2++;
		ret = readNetworkDictCallbackAI (d,  nodeId, index, subIndex, dataType,  Callback,  useBlockMode);
		while(ret >= 0xFE)
		{
			sdoReadlost3++;
			ret = readNetworkDictCallbackAI (d,  nodeId, index, subIndex, dataType,  Callback,  useBlockMode);
		}
		while ((result=getReadResultNetworkDict (d, &nowline,  nodeId, data, size, &abortCode)) == SDO_UPLOAD_IN_PROGRESS);
        
        for(int i = 0;i<16;i++)
        {
            if(d->transfers[i].state == SDO_FINISHED && d->transfers[i].index == index && d->transfers[i].subIndex == subIndex)
            {
                resetSDOline(d, i);
                sdoReadNetworkDictCallBackAI_error[i]++;
            }
        }
	}
}


/*SDO 读取*/
/*
SDO读取节点对象时，需要设置以下参数：
1. 要读取的节点NodeID
2. 要读取的节点对象的Index和subindex
*/
uint8_t SdoReadProcess(SDORW_t *Sdo_R_Para)
{
    uint8_t SendError = 0;
	uint16_t index_high = 0;
	uint16_t index_low = 0;
    uint8_t dataBuff[4] = {0};
	
	index_low = Sdo_R_Para->Index & 0xFF;
	index_high = (Sdo_R_Para->Index & 0xFF00)>>8;
    
	Message can_frame;
	can_frame.cob_id = 0x600 + (Sdo_R_Para->NodeId & 0x7F);
	can_frame.rtr = 0;
	can_frame.len = 8;
    can_frame.data[0] = 0x40;
    can_frame.data[1] = index_low;
    can_frame.data[2] = index_high;
    can_frame.data[3] = (0xFF & Sdo_R_Para->subIndex);
    can_frame.data[4] = dataBuff[0];
    can_frame.data[5] = dataBuff[1];
    can_frame.data[6] = dataBuff[2];
    can_frame.data[7] = dataBuff[3];
    
    Sdo_R_Para->Sdo_Status = ReadProcess;
    Sdo_R_Para->ReadLostCount = 0;
    
	SendError = canSend(&CANopenMaster_Data, &can_frame);
    if(SendError)
    {
#if(CANOPENMONITOR)
    printf("CAN Send Fail!\r\n");
#endif
        return CAN_READ_FAIL;
    }
    while(ReadProcess == Sdo_R_Para->Sdo_Status)
    {
        if(Sdo_R_Para->ReadLostCount >= 5) /*5mS 读取超时*/
        {
            Sdo_R_Para->ReadLostCount = 0;
            Sdo_R_Para->Sdo_Status = ReadFail;
#if(CANOPENMONITOR)
            printf("0xFB: SDO Read Fail!\r\n");
#endif
            return CAN_READ_FAIL;
        }
        else if(ReadFail == Sdo_R_Para->Sdo_Status)
        {
            return CAN_READ_FAIL;
        }
    }
    return CAN_READ_SUCCEED;
}

/*SDO 写入*/
uint8_t SdoWriteProcess(SDORW_t *Sdo_W_Para)
{
    uint8_t errorCode = 0;
	uint16_t index_high = 0;
	uint16_t index_low = 0;
    uint8_t dataBuff[4] = {0};
	
	index_low = Sdo_W_Para->Index & 0xFF;
	index_high = (Sdo_W_Para->Index & 0xFF00)>>8;
    
	Message can_frame;
	can_frame.cob_id = 0x600 + (Sdo_W_Para->NodeId & 0x7F);
	can_frame.rtr = 0;
	can_frame.len = 8;
    
    switch(Sdo_W_Para->dataType)
    {
        case OD_DATATYPE_I8:
        case OD_DATATYPE_U8:
            can_frame.data[0] = 0x2F;
            dataBuff[0] = (Sdo_W_Para->Data & 0xFF);
        break;
        case OD_DATATYPE_I16:
        case OD_DATATYPE_U16:
            can_frame.data[0] = 0x2B;
            dataBuff[0] = (Sdo_W_Para->Data & 0xFF);
            dataBuff[1] = ((Sdo_W_Para->Data >> 8) & 0xFF);
        break;
        case OD_DATATYPE_Int24:
        case OD_DATATYPE_Uint24:
            can_frame.data[0] = 0x27;
            dataBuff[0] = (Sdo_W_Para->Data & 0xFF);
            dataBuff[1] = ((Sdo_W_Para->Data >> 8) & 0xFF);
            dataBuff[2] = ((Sdo_W_Para->Data >> 16) & 0xFF);
        break;
        case OD_DATATYPE_I32:
        case OD_DATATYPE_U32:
            can_frame.data[0] = 0x23;
            dataBuff[0] = (Sdo_W_Para->Data & 0xFF);
            dataBuff[1] = ((Sdo_W_Para->Data >> 8) & 0xFF);
            dataBuff[2] = ((Sdo_W_Para->Data >> 16) & 0xFF);
            dataBuff[3] = ((Sdo_W_Para->Data >> 24) & 0xFF);
        break;
        default :   /*数据类型错误*/
            Sdo_W_Para->Sdo_Status = WriteFail;
            return 0xFF;
    }
    can_frame.data[1] = index_low;
    can_frame.data[2] = index_high;
    can_frame.data[3] = (0xFF & Sdo_W_Para->subIndex);
    can_frame.data[4] = dataBuff[0];
    can_frame.data[5] = dataBuff[1];
    can_frame.data[6] = dataBuff[2];
    can_frame.data[7] = dataBuff[3];
    Sdo_W_Para->Sdo_Status = WriteProcess;
    Sdo_W_Para->WriteLostCount = 0;
    
	errorCode = canSend(&CANopenMaster_Data, &can_frame);
    if(errorCode)
    {
        return FAIL;
    }
    
//    /*SDO 写超时判断，超时门限10mS*/
//    while(WriteSuccess != Sdo_W_Para->Sdo_Status)
//    {
//        if(Sdo_W_Para->WriteLostCount >= 5) /*5mS 写超时*/
//        {
//            Sdo_W_Para->Sdo_Status = WriteFail;
//            Sdo_W_Para->WriteLostCount = 0;
//            return FAIL;
//        }
//        else if(WriteFail == Sdo_W_Para->Sdo_Status)
//        {
//            return FAIL;
//        }
//    }
    return SUCCEED;
}


/*SDO 解析读取的数据*/
void SdoReadAnalysis(SDORW_t *Sdo_R_Para, Message *m)
{
    uint16_t index;
    uint8_t subIndex;
    uint8_t dataLen;
    if(0x580 == (m->cob_id & 0x580))
    {
        if(Sdo_R_Para->NodeId == (m->cob_id & 0x7F))
        {
            if(m->len != 8)
            {
                return;
            }
            else
            {
                if(0x80 == m->data[0])
                {
                    Sdo_R_Para->ReadLostCount = 0;
                    Sdo_R_Para->Sdo_Status = ReadFail;
                    return;
                }
                else
                {
                    index = (m->data[2] << 8) + m->data[1];
                    subIndex = m->data[3];
                    if((index == Sdo_R_Para->Index) && (subIndex == Sdo_R_Para->subIndex))
                    {
                        dataLen = m->data[0];
                        switch(dataLen)
                        {
                            case 0x4F:
                                Sdo_R_Para->Data = m->data[4];
                                Sdo_R_Para->Sdo_Status = ReadSuccess;
                            break;
                            
                            case 0x4B:
                                Sdo_R_Para->Data = (m->data[5] << 8) + m->data[4];
                                Sdo_R_Para->Sdo_Status = ReadSuccess;
                            break;
                            
                            case 0x47:
                                Sdo_R_Para->Data = (m->data[6] << 16) + (m->data[5] << 8) + m->data[4];
                                Sdo_R_Para->Sdo_Status = ReadSuccess;
                            break;
                            
                            case 0x43:
                                Sdo_R_Para->Data = (m->data[7] << 24) +(m->data[6] << 16) + (m->data[5] << 8) + m->data[4];
                                Sdo_R_Para->Sdo_Status = ReadSuccess;
                            break;
                            
                            default:
                                Sdo_R_Para->Data = 0;
                                Sdo_R_Para->Sdo_Status = ReadFail;
                                break;
                        }
                    }
                }
            }
        }
    }
}

/*SDO 处理写入ACK数据*/
void SdoWriteAck(SDORW_t *Sdo_W_Para, Message *m)
{
    uint16_t index;
    uint8_t subIndex;
    
    if(0x580 == (m->cob_id & 0x580))
    {
        if(Sdo_W_Para->NodeId == (m->cob_id & 0x7F))
        {
            index = (m->data[2] << 8) + m->data[1];
            subIndex = m->data[3];
            if((index == Sdo_W_Para->Index) && (subIndex == Sdo_W_Para->subIndex))
            {
                if(0x60 == m->data[0])
                {
                    Sdo_W_Para->Sdo_Status = WriteSuccess;
                    Sdo_W_Para->WriteLostCount = 0;
                }
                else
                {
                    Sdo_W_Para->Sdo_Status = WriteFail;
                    Sdo_W_Para->WriteLostCount = 0;
                }
            }
        }
    }
}
