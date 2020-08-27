#ifndef _MBTASK_H
#define _MBTASK_H


#ifdef __cplusplus
PR_BEGIN_EXTERN_C
#endif

#include "port.h"

#define MB_SLAVE_ADDRESS        2

/*      NOT USED        */
#define REG_INPUT_START         1 
#define REG_INPUT_NREGS         2 
/*      NOT USED        */

#define REG_HOLDING_START       1
#define REG_HOLDING_NREGS       64


#define REG_COILS_START         1
#define REG_COILS_SIZE          16      /* 16/8=2 */

#define REG_DISC_START          1
#define REG_DISC_SIZE           8       /* 8/8=1 */

extern USHORT usRegInputStart; 
extern USHORT usRegInputBuf[REG_INPUT_NREGS];  

extern USHORT   usRegHoldingStart;
extern USHORT   usRegHoldingBuf[REG_HOLDING_NREGS];

extern unsigned char ucRegCoilsBuf[REG_COILS_SIZE / 8]; /* 16/8=2 */
extern unsigned char ucRegDiscBuf[REG_DISC_SIZE / 8];   /* 8/8=1 */  

#ifndef M_PI
  #define M_PI 3.14159265358979323846
#endif

#ifdef __cplusplus
PR_END_EXTERN_C
#endif
#endif