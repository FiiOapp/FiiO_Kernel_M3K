#ifndef __LINUX_FT6X06_TS_H__
#define __LINUX_FT6X06_TS_H__

/* -- dirver configure -- */
#define CFG_MAX_TOUCH_POINTS	1//2

#define PRESS_MAX	0xFF
#define FT_PRESS		0x7F

#define FT6X06_NAME 	"ft6x06_ts"

#define FT_MAX_ID	0x0F
#define FT_TOUCH_STEP	6
#define FT_TOUCH_X_H_POS		3
#define FT_TOUCH_X_L_POS		4
#define FT_TOUCH_Y_H_POS		5
#define FT_TOUCH_Y_L_POS		6
#define FT_TOUCH_EVENT_POS		3
#define FT_TOUCH_ID_POS			5
#define FT_TOUCH_WEIGHT_POS			7

#define POINT_READ_BUF	(3 + FT_TOUCH_STEP * CFG_MAX_TOUCH_POINTS)

/*register address*/
#define FT6x06_REG_FW_VER		0xA6
#define FT6x06_REG_POINT_RATE	0x88
#define FT6x06_REG_THGROUP	0x80
#define FT6x06_REG_D_MODE	0x00
#define FT6x06_REG_G_MODE	0xA4
#define FT6x06_REG_P_MODE	0xA5
#define FT6x06_REG_CTRL	0x86
#define FT6x06_REG_STATE	0xBC
int ft6x06_i2c_Read(struct i2c_client *client, char *writebuf, int writelen,
		    char *readbuf, int readlen);
int ft6x06_i2c_Write(struct i2c_client *client, char *writebuf, int writelen);

/* The platform data for the Focaltech ft5x0x touchscreen driver */
struct ft6x06_platform_data {
	unsigned int x_max;
	unsigned int y_max;
	unsigned int va_x_max;
	unsigned int va_y_max;
	unsigned long irqflags;
	unsigned int irq;
	unsigned int reset;
};
struct ts_event {
        u16 au16_x[CFG_MAX_TOUCH_POINTS];       /*x coordinate */
        u16 au16_y[CFG_MAX_TOUCH_POINTS];       /*y coordinate */
        u8 au8_touch_event[CFG_MAX_TOUCH_POINTS];       /*touch event:
                                        0 -- down; 1-- contact; 2 -- contact */
        u8 au8_finger_id[CFG_MAX_TOUCH_POINTS]; /*touch ID */
        u8 au8_touch_wight[CFG_MAX_TOUCH_POINTS];       /*touch Wight */
        u16 pressure;
        u8 touch_point;
};

struct ft6x06_ts_data {
        unsigned int irq;
        unsigned int irq_pin;
        unsigned int x_max;
        unsigned int y_max;
        unsigned int va_x_max;
        unsigned int va_y_max;
        struct i2c_client *client;
        struct input_dev *input_dev;
        struct ts_event event;
        struct ft6x06_platform_data *pdata;
        struct work_struct  work;
        struct timer_list timer;
        bool touch_pressed;
        unsigned int up_touch_count;
        unsigned int down_touch_count;
        unsigned int same_touch_count;
	unsigned int touch_count;
	unsigned int long_touch_status;
        struct workqueue_struct *workqueue;
        struct regulator *vcc_reg;

#ifdef DEBUG_SKIP_REPORT_POINT
        unsigned int report_count;
#endif
};

extern struct ft6x06_ts_data *ft6x06_ts_close;
extern void ft6x06_ts_reset(struct ft6x06_ts_data *ts);

#endif
