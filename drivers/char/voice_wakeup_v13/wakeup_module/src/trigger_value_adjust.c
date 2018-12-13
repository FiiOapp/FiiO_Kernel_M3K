#include <common.h>
#include "interface.h"
#include "dmic_config.h"
#include "trigger_value_adjust.h"

int thr_table[TRIGGER_CNTS] = {2000, 5000};
int tri_cnt[TRIGGER_CNTS] = {2, 6};

#if 0
static int quantity_thr(int thr)
{
	int i;
	/* find the place thr value is in thr_table */
	for(i = 0; i < TRIGGER_CNTS; i++) {
		if(thr_table[i] == thr)
			break;
	}
	return i;
}
#endif
int quantity_tri(int times)
{
	if(times < 2) {
		return 0; /* UP */
	} else {
		return 1; /* DOWN */
	}
}

/*
 * @return, adjusted value.
 * */
int adjust_trigger_value(int times_per_unit, int cur_thr)
{
	int result;
	int down = quantity_tri(times_per_unit);  

	if(down) {
		result = thr_table[0];
	} else {
		result = thr_table[1];
	}

	return result;

}


