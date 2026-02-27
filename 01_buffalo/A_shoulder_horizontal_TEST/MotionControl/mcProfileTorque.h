#ifndef __mcProfileTorque_H__
#define __mcProfileTorque_H__

#include "MC_ProfileTorque.h"

void profileFluxAndTorqueInit(PrflTrq_Typedef *PrflTrq_struct, int32_t velloop_frq);
void profileFluxAndTorqueSetValue(PrflTrq_Typedef *that ,int32_t flux, int32_t trq, int32_t flux_inc, int32_t trq_inc);
void profileFluxAndTorqueInput(PrflTrq_Typedef *that, int32_t flux_fdbk, int32_t torque_fdbk);
void profileFluxAndTorqueClearOutput(PrflTrq_Typedef* that);
int32_t profileFluxAndTorqueGenerator(PrflTrq_Typedef* that);

#endif