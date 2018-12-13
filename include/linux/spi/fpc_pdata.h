#ifndef __LINUX_SPI_FPC_FP_H
#define __LINUX_SPI_FPC_FP_H

struct fpc_platform_data {
    int en_pin;
    int int_pin;
    int reset_pin;
    int cs_pin;
    int wkup_ring_sel_pin;
    int encrypt_ic_rst_pin;
#ifdef CONFIG_MCLK_PROVIDED_TO_ENCRYPT_IC
    int encrypt_ic_clk_pin;
#endif
};

#endif
