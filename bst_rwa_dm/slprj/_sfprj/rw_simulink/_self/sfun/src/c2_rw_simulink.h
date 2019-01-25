#ifndef __c2_rw_simulink_h__
#define __c2_rw_simulink_h__

/* Include files */
#include "sf_runtime/sfc_sf.h"
#include "sf_runtime/sfc_mex.h"
#include "rtwtypes.h"
#include "multiword_types.h"

/* Type Definitions */
#ifndef struct_sL6LJlPlxhdTxZzXh5NTaQC
#define struct_sL6LJlPlxhdTxZzXh5NTaQC

struct sL6LJlPlxhdTxZzXh5NTaQC
{
  int32_T intNumBits;
};

#endif                                 /*struct_sL6LJlPlxhdTxZzXh5NTaQC*/

#ifndef typedef_c2_sL6LJlPlxhdTxZzXh5NTaQC
#define typedef_c2_sL6LJlPlxhdTxZzXh5NTaQC

typedef struct sL6LJlPlxhdTxZzXh5NTaQC c2_sL6LJlPlxhdTxZzXh5NTaQC;

#endif                                 /*typedef_c2_sL6LJlPlxhdTxZzXh5NTaQC*/

#ifndef struct_svlYwr1a83R4ShH2bxiqAFE
#define struct_svlYwr1a83R4ShH2bxiqAFE

struct svlYwr1a83R4ShH2bxiqAFE
{
  real_T trq_m;
  real_T spd_m;
  real_T trq;
  real_T spd;
  real_T cur_motor;
  real_T cur_cmd;
};

#endif                                 /*struct_svlYwr1a83R4ShH2bxiqAFE*/

#ifndef typedef_c2_svlYwr1a83R4ShH2bxiqAFE
#define typedef_c2_svlYwr1a83R4ShH2bxiqAFE

typedef struct svlYwr1a83R4ShH2bxiqAFE c2_svlYwr1a83R4ShH2bxiqAFE;

#endif                                 /*typedef_c2_svlYwr1a83R4ShH2bxiqAFE*/

#ifndef typedef_SFc2_rw_simulinkInstanceStruct
#define typedef_SFc2_rw_simulinkInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c2_sfEvent;
  boolean_T c2_isStable;
  boolean_T c2_doneDoubleBufferReInit;
  uint8_T c2_is_active_c2_rw_simulink;
  c2_svlYwr1a83R4ShH2bxiqAFE c2_rw_dat;
  boolean_T c2_rw_dat_not_empty;
  c2_svlYwr1a83R4ShH2bxiqAFE c2_dat;
  boolean_T c2_dat_not_empty;
  real_T *c2_ctrl_mode;
  real_T *c2_trq_m;
  real_T *c2_cur_tgt;
  real_T *c2_spd_tgt;
  real_T *c2_trq_tgt;
  real_T *c2_vol_in;
  real_T *c2_spd_init;
  real_T *c2_dt;
  real_T *c2_time;
  real_T *c2_spd_m;
  real_T *c2_trq;
  real_T *c2_spd;
  real_T *c2_cur_motor;
  real_T *c2_cur_cmd;
} SFc2_rw_simulinkInstanceStruct;

#endif                                 /*typedef_SFc2_rw_simulinkInstanceStruct*/

/* Named Constants */

/* Variable Declarations */
extern struct SfDebugInstanceStruct *sfGlobalDebugInstanceStruct;

/* Variable Definitions */

/* Function Declarations */
extern const mxArray *sf_c2_rw_simulink_get_eml_resolved_functions_info(void);

/* Function Definitions */
extern void sf_c2_rw_simulink_get_check_sum(mxArray *plhs[]);
extern void c2_rw_simulink_method_dispatcher(SimStruct *S, int_T method, void
  *data);

#endif
