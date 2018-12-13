
#include "common.h"
#include "interface.h"
#include "irq.h"
#include "print.h"
#include "pm_ops.h"
#include "rtc_ops.h"
#include "dmic_ops.h"
#include "tcu_timer.h"
#include "recognization.h"

#define MAX_PROCESS_TIMES	(4) /* do voice trigger duration max 4 seconds */
#define MAX_PROCESS_BYTES	(MAX_PROCESS_TIMES * (16000*2))

static int total_process_len = 0;


int voice_data_is_enough_to_recognizition(int length)
{

#define PROCESS_BYTES_PER_TIME	(1024)	/* 1 KB */

	if ( length > PROCESS_BYTES_PER_TIME)
		return 1;
	else 
		return 0;
}

/*
 * return value:
 * 	SYS_NEED_DATA
 * 	SYS_WAKEUP_OK
 */
static int check_voice_data_and_do_recognize(int suspending)
{
	int length;
	int ret;

	vtw_print(LOG_DEBUG, "check_voice_data_and_do_recognize()\r\n");
	ret = SYS_NEED_DATA;

	/* dmic check and receive data */
	length = dmic_check_and_receive_data();
	vtw_print(LOG_DEBUG, " length= ");
	vtw_print_hex(LOG_DEBUG, length); vtw_print(LOG_DEBUG, "\r\n");

	if ( voice_data_is_enough_to_recognizition(length)) {

		while (length) {
			int nbytes;
			unsigned char * buf;
			buf = NULL;
			nbytes = dmic_get_voice_data(&buf, length);

			vtw_print(LOG_DEBUG, " nbytes= ");
			vtw_print_hex(LOG_DEBUG, nbytes);
			vtw_print(LOG_DEBUG, " buf= ");
			vtw_print_hex(LOG_DEBUG, (unsigned int)buf);
			vtw_print(LOG_DEBUG, "\r\n");

			if ( buf && nbytes ) {
				/* recognize sample data */
				ret = recognizer_process_voice_data(buf, nbytes);
			}

			if( 1 || ret == SYS_NEED_DATA) {
				/* check max process time */
				total_process_len += nbytes;
			}

			if(ret == SYS_WAKEUP_OK) {
				break;
			}

			length -= nbytes;
		};
	}

	vtw_print(LOG_DEBUG, " ret= ");
	vtw_print_hex(LOG_DEBUG, ret);
	vtw_print(LOG_DEBUG, "\r\n");

	return ret;
}



/* call from jz_wakeup_v13.c wakeup_module_process_data() every 30ms.
 */
int do_voice_recognization(void)
{
	int ret;
	vtw_print(LOG_VERBOSE, __FUNCTION__);
	vtw_print(LOG_VERBOSE, "\r\n");

	set_debug_print_level(LOG_INFO);
	//set_debug_print_level(LOG_WARN);
	ret = check_voice_data_and_do_recognize(0);
	//set_debug_print_level(LOG_VERBOSE);

	return ret;
}

/* 
[2016-08-12 18:18:01.810] irs 00000001 00000000 
[2016-08-12 18:18:01.811] irp 04000001 00000000 wakeup_handler_internal()
[2016-08-12 18:18:01.815] do_irq_handler()
[2016-08-12 18:18:01.817] pending:      04000001
[2016-08-12 18:18:01.818] do_irq(): 0000000000000000
[2016-08-12 18:18:01.821] do irq handler: 00000000 dmic irq
[2016-08-12 18:18:01.824] DMIC_ICR      00000011 DMIC_IMR       0000002E        TRGR
[2016-08-12 18:18:01.828] do_irq(): 000000000000001A
[2016-08-12 18:18:01.830] do irq handler: 0000001A tcu irq
[2016-08-12 18:18:01.833] tcu_irq_handler() TFR:        00218021
[2016-08-12 18:18:01.836] pending:      00000000
[2016-08-12 18:18:01.838] 
[2016-08-12 18:18:01.838] is_dmic_trigger_irq_arrival() continue...
[2016-08-12 18:18:01.843] do_irq_handler()
[2016-08-12 18:18:01.843] pending:      00000000
[2016-08-12 18:18:01.845] pending:      00000000
[2016-08-12 18:18:01.847] 
[2016-08-12 18:18:01.847] not tcu timer irq, or dmic trigger irq, continue...
[2016-08-12 18:18:01.852]  WAKEUP_FAILED
 */
static int wakeup_handler_internal(int par)
{
	//int irq_val;
	int ret;

	vtw_print(LOG_DEBUG, "wakeup_handler_internal()\r\n");

	total_process_len = 0;
	set_wakeup_source_type(WAKEUP_BY_OTHERS);

do_voice_wakeup:
	tcu_timer_clear_irq_state();

	ret = do_irq_handler(0);

	vtw_print(LOG_VERBOSE, "do_irq_handler ret= "); vtw_print_hex(LOG_VERBOSE, ret); vtw_print(LOG_DEBUG, "\r\n");


	/* if there are interrupts that not handled by irq_handler, then wakeup OS. */
	if (ret != 0) {
		/* wakeup by other devices */
		//set_wakeup_source_type(WAKEUP_BY_OTHERS);
		ret = SYS_WAKEUP_BY_OTHERS;
		return ret;
	}

	/* if os rtc alarm occur, then wakeup OS. */
	if (is_os_rtc_alarm_occur()) {
		/* wakeup by other devices */
		ret = SYS_WAKEUP_BY_OTHERS;
		return ret;
	}

	if( is_dmic_trigger_irq_arrival()
	    || !is_tcu_timer_irq_arrival() ) {
		vtw_print(LOG_DEBUG, "is_dmic_trigger_irq_arrival() continue...\r\n");

		reset_dmic_trigger_irq_arrival_state();

		/* do cpu sleeping, waiting tcu timer(40ms?). */
		cpu_idle();
		goto do_voice_wakeup;
	}

	/* if not tcu timer irq, or dmic trigger irq return FAILED */

	if ( !is_dmic_waiting_data()
	     || !is_tcu_timer_irq_arrival() ) {
		vtw_print(LOG_DEBUG, "not tcu timer irq, or dmic trigger irq, continue...\r\n");

		ret = SYS_WAKEUP_FAILED;
		return ret;
	}

	tcu_timer_clear_irq_state();


	/* do voice tigger */
	ret = check_voice_data_and_do_recognize(1);

	if(total_process_len >= MAX_PROCESS_BYTES) {
		ret =  SYS_WAKEUP_FAILED;
	}


	if(ret == SYS_NEED_DATA) {
		vtw_print(LOG_VERBOSE, "SYS_NEED_DATA continue...\r\n");

		/* do cpu sleeping, waiting tcu timer(40ms?). */
		cpu_idle();

		goto do_voice_wakeup;
	}

	return ret;
}

int wakeup_handler_impl(int par)
{
	int ret;

	//set_debug_print_level(LOG_VERBOSE);
	//set_debug_print_level(LOG_DEBUG);
	set_debug_print_level(LOG_INFO);

	ret = wakeup_handler_internal(par);

	if(ret == SYS_WAKEUP_OK) {
		vtw_print(LOG_INFO, " WAKEUP_OK\r\n");
		set_wakeup_source_type(WAKEUP_BY_DMIC);
	}
	else if(ret == SYS_WAKEUP_FAILED) {
		vtw_print(LOG_INFO, " WAKEUP_FAILED\r\n");
	}
	else if(ret == SYS_WAKEUP_BY_OTHERS) {
		vtw_print(LOG_INFO, " WAKEUP_BY_OTHERS\r\n");
		set_wakeup_source_type(WAKEUP_BY_OTHERS);
		ret = SYS_WAKEUP_OK;
	} else {
		vtw_print(LOG_WARN, " warning voice wakeup not handled\t");
		vtw_print_hex(LOG_WARN, ret);
		vtw_print(LOG_WARN, "\r\n");
	}

	//set_debug_print_level(LOG_INFO);

	return ret;
}
