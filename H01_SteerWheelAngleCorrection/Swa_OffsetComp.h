/* ===========================================================================
 *
 *                   Human Horizons Operation Center Shanghai
 *
 *   Human Horizons Operation Center Shanghai (HRYT) owns all rights to this work and
 *   intends to maintain it in confidence to preserve its trade secret status.
 *   HRYT reserves its rights under all copyright laws to protect this work
 *   as a published work, when appropriate.Those having access to this work may
 *   not copy it, use it, modify it, or disclose the information contained in
 *   it without the written authorization of HRYT.
 *
 * ========================================================================= */
#ifndef Swa_OffsetComp_h
#define Swa_OffsetComp_h
/* ===========================================================================
 *
 *  Include Files
 *
 * ========================================================================= */
#include "SASOffset.h"
#include "Swa_OffsetPara.h"


#define fabsf_hryt(a)     ((a)>0?(a):(a)*-1)
#define max_hryt(a,b)    ((a)>(b)?(a):(b))
 /* ===========================================================================
  *
  *  Public Function Prototypes
  *
  * ========================================================================= */


void fSASRackOffsetProcess(void);
extern  uint16 u16_SASRackEnableCnt ;
extern boolean flag ;

//Rack Data
extern uint16 u16_RackDeltaN ;
extern float32 f32_RackPosOffset ;
extern uint16 u16_RackEnableCnt ;
extern  boolean b_SwaOffsetValid_Rack ;
extern boolean b_NotNewFireCycle;
extern boolean b_InitFlag;


#endif /* __SASRackOffsetComp_h_ */
/* ===========================================================================
 *
 * Project Name:   
 * File Name:      SASRackOffsetComp.h
 * Archive:        
 * Author:         
 * Date:           
 * (c) Copyright   2022, Human Horizons Operation Center Shanghai
 *-----------------------------------------------------------------------------
 *              V E R S I O N 	T R A C K I N G
 *-----------------------------------------------------------------------------
 * Version       Name                 Date                      Log
 * --------   -----------       ----------------     --------------------------
 *   v1.1		Kaifeng Qu				0615				Rack Cal added
 *
 * ========================================================================= */


