#ifndef __VOICE_WAKEUP_H__
#define __VOICE_WAKEUP_H__



#include "ivDefine.h"
#include "ivIvwDefine.h"
#include "ivIvwErrorCode.h"
#include "ivIVW.h"
#include "ivPlatform.h"


int wakeup_open(void);
int wakeup_close(void);

int recognizer_process_voice_data(unsigned char *buf, unsigned long len);
int process_buffer_data(unsigned char *buffer, unsigned long len);

#endif

