#ifndef __CPM_OPS_H__
#define __CPM_OPS_H__

int cpm_init_save(int ch);
int cpm_exit_restore(int ch);

int cpm_start_tcu_clock(int ch);
int cpm_stop_tcu_clock(int ch);

int cpm_start_dma_clock(int ch);
int cpm_stop_dma_clock(int ch);

int cpm_start_dmic_clock(int ch);
int cpm_stop_dmic_clock(int ch);
void cpm_config_dmic_clk(void);
void cpm_config_suspend_dmic_clk(void);


#endif	/* __CPM_OPS_H__ */
