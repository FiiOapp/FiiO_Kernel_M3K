#ifndef __DWC2_JZ_PLATFORM_H__
#define __DWC2_JZ_PLATFORM_H__
struct jz_otg_info {
	struct dwc2_hsotg	hsotg;
	struct jzdwc_pin 	*drvvbus_pin;
	struct jzdwc_pin 	*dete_pin;
	struct jzdwc_pin 	*id_pin;
	struct clk		*otg_gate_clk;
	struct clk		*otg_div_clk;
	struct delayed_work	work;
	int			dete_irq_enable;
	int			id_irq_enable;
};
#endif /* __DWC2_JZ_PLATFORM_H__ */
