#ifndef OD_CALLBACK_PROCESS_H__
#define OD_CALLBACK_PROCESS_H__


#include "data.h"
void CANopenMasterCallbackRegister(void);
void CANopenSlaveCallbackRegister(void);

// Slave
UNS32 OperationModeCallback(CO_Data* d, const OD_OI_typeDef * unsused_indextable, UNS8 unsused_bSubindex);
UNS32 NewAngleCallback(CO_Data* d, const OD_OI_typeDef * unsused_indextable, UNS8 unsused_bSubindex);
UNS32 NewSpeedCallback(CO_Data* d, const OD_OI_typeDef * unsused_indextable, UNS8 unsused_bSubindex);

UNS32 interpolation_data_record_callback(CO_Data* d, const OD_OI_typeDef * unsused_indextable, UNS8 unsused_bSubindex);

UNS32 TPDO5Callback(CO_Data* d, const OD_OI_typeDef * unsused_indextable, UNS8 unsused_bSubindex);

// Master 
UNS32 servoErrorCodeCallback(CO_Data* d, const OD_OI_typeDef * unsused_indextable, UNS8 unsused_bSubindex);
UNS32 servoStatuswordCallback(CO_Data* d, const OD_OI_typeDef * unsused_indextable, UNS8 unsused_bSubindex);
UNS32 servoControlwordCallback(CO_Data* d, const OD_OI_typeDef * unsused_indextable, UNS8 unsused_bSubindex);
UNS32 servoModesOperationCallback(CO_Data* d, const OD_OI_typeDef * unsused_indextable, UNS8 unsused_bSubindex);
UNS32 servoActualPositionChangeCallback(CO_Data* d, const OD_OI_typeDef * unsused_indextable, UNS8 unsused_bSubindex);
UNS32 servoTargetPositionChangeCallback(CO_Data* d, const OD_OI_typeDef * unsused_indextable, UNS8 unsused_bSubindex);
UNS32 servoProfileVelocityChangeCallback(CO_Data* d, const OD_OI_typeDef * unsused_indextable, UNS8 unsused_bSubindex);
UNS32 servoProfileAccelerationCallback(CO_Data* d, const OD_OI_typeDef * unsused_indextable, UNS8 unsused_bSubindex);
UNS32 servoProfileDecelerationCallback(CO_Data* d, const OD_OI_typeDef * unsused_indextable, UNS8 unsused_bSubindex);

#endif
