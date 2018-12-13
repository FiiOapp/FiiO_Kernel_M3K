#include "ivDefine.h"
#include "ivIvwDefine.h"
#include "ivIvwErrorCode.h"
#include "ivIVW.h"
#include "ivPlatform.h"

#include "jz_dma.h"
#include "circ_buf.h"
#include "dmic_config.h"
#include "dmic_ops.h"
#include "interface.h"
#include "print.h"
#include <tcsm_layout.h>
#include <common.h>
#include "mem_buffer.h"
#include "recognization.h"

/* unsigned char __attribute__((aligned(32))) pIvwObjBuf[SR_IVWOBJ_SIZE]; */
/* unsigned char __attribute__((aligned(32))) nReisdentBuf[SR_RESIDENT_RAM_SIZE]; */

static int send_data_to_process(ivPointer pIvwObj, unsigned char *pdata, unsigned int size)
{
	unsigned int nSize_PCM = size;
	unsigned int tmp;
	int i;
	ivInt16 CMScore  =  0;
	ivInt16 ResID = 0;
	ivInt16 KeywordID = 0;
	ivUInt32 StartMS = 0;
	ivUInt32 EndMS = 0;
	unsigned int Samples;
	unsigned int BytesPerSample;
	unsigned char *pData = pdata;
	unsigned int nSamples_count;
	ivStatus iStatus1;
	ivStatus iStatus2;
	Samples = 110;
#if 0
	//serial_put_hex(size);
	//TCSM_PCHAR('D');
//	serial_put_hex(*(volatile unsigned int*)pData);
	BytesPerSample = 2;
	nSamples_count = nSize_PCM / (Samples * BytesPerSample);

	/*process 16k, 16bit samples*/
	for(i = 0; i <= nSamples_count; i++ )
	{
		if(i == nSamples_count) {
			tmp = nSize_PCM % (Samples*BytesPerSample);
			if(!tmp) {
				break;
			}
			Samples = tmp / BytesPerSample;
		}
		iStatus1 = IvwAppendAudioData(pIvwObj,(ivCPointer)pData, Samples);
		pData = pData + Samples * BytesPerSample;
		if( iStatus1 != IvwErrID_OK )
		{
			vtw_print(LOG_ERROR, "IvwErr iStatus1: ");
			vtw_print_hex(LOG_ERROR, iStatus1);
			vtw_print(LOG_ERROR, "\r\n");
#if 0
			if( iStatus1 == IvwErr_BufferFull ){
			}else if( iStatus1 ==  IvwErr_InvArg ){
			}else if( iStatus1 ==  IvwErr_InvCal ){
			}
#endif
		} else {
			//printf("IvwAppendAudioData Ok!!!!!!!!\n");
		}
		iStatus2 = IvwRunStepEx( pIvwObj, &CMScore,  &ResID, &KeywordID,  &StartMS, &EndMS );
		if( IvwErr_WakeUp == iStatus2 ){
			vtw_print(LOG_INFO, "WKUPOK\r\n");
			return IvwErr_WakeUp;
		}

		vtw_print(LOG_VERBOSE, "IvwErr iStatus2: ");
		vtw_print_hex(LOG_VERBOSE, iStatus2);
		vtw_print(LOG_VERBOSE, "\r\n");

	}
#endif
	return 0;
}


int wakeup_open(void)
{
	ivSize nIvwObjSize = SR_IVWOBJ_SIZE;
	ivUInt16 nResidentRAMSize = SR_RESIDENT_RAM_SIZE;
	ivCPointer pResKey;
	ivUInt16 nWakeupNetworkID = 0;

	ivStatus iStatus;

	//printf("wakeu module init#####\n");
	//printf("pIvwObj:%x, pResidentRAM:%x, pResKey:%x\n", pIvwObj, pResidentRAM, pResKey);
	ivPointer pIvwObj;
	ivPointer pResidentRAM;

	pIvwObj = (ivPointer)get_pIvwObjBuf();
	pResidentRAM = (ivPointer)get_nReisdentBuf();

	pResKey = (void *)get_wakeup_key_resource_address();

#if 0

	iStatus = IvwCreate(pIvwObj, &nIvwObjSize, pResidentRAM, &nResidentRAMSize, pResKey, nWakeupNetworkID);
	if( IvwErrID_OK != iStatus ) {
		//printf("IvwVreate Error: %d\n", iStatus);
		return ivFalse;
	}
	//printf("[voice wakeup] OBJECT create ok\n");
	IvwSetParam( pIvwObj, IVW_CM_THRESHOLD, 20, 0 ,0);
#if 0
	IvwSetParam( pIvwObj, IVW_CM_THRESHOLD, 20, 1 ,0);
	IvwSetParam( pIvwObj, IVW_CM_THRESHOLD, 15, 2 ,0);
#endif

#endif	/* if 0 */

	return 0;
}

int wakeup_close(void)
{

	return 0;
}


int process_buffer_data(unsigned char *buf, unsigned long len)
{
	int ret;
	unsigned char *a_buf = (unsigned char *)((unsigned int)buf | 0xA0000000);
	ivPointer pIvwObj;
	if ( !buf )
		return SYS_NEED_DATA;
		
	pIvwObj = (ivPointer)get_pIvwObjBuf();

	ret = send_data_to_process(pIvwObj, a_buf, len);
	if(ret == IvwErr_WakeUp) {
		return SYS_WAKEUP_OK;
	}
	return SYS_NEED_DATA;
}

int recognizer_process_voice_data(unsigned char *buf, unsigned long len)
{
	return process_buffer_data(buf, len);
}
