#ifndef __MACH_JZSCC_H
#define __MACH_JZSCC_H

/* SCC registers */
#define SCCDR        0x00
#define SCCFDR       0x04
#define SCCCR        0x08
#define SCCSR        0x0C
#define SCCTFR       0x10
#define SCCEGTR      0x14
#define SCCECR       0x18
#define SCCRTOR      0x1C


#define ENABLE_SCC   (1<<31)
#define TRANSMODE    (1<<30)
#define EMPTYFIFO    (1<<23)
#define TRANSEND     (1<<7)
#define ECNTO        (1<<0)


/* Exported constants --------------------------------------------------------*/
#define T0_PROTOCOL        0x00  /* T0 protocol */
#define DIRECT             0x3B  /* Direct bit convention */
#define INDIRECT           0x3F  /* Indirect bit convention */
#define SETUP_LENGTH       20
#define HIST_LENGTH        20
#define LCMAX              255

/* ATR structure - Answer To Reset -------------------------------------------*/
struct atr {
	unsigned char ts;               /* Bit Convention */
	unsigned char t0;               /* High nibble = Number of setup byte; low nibble = Number of historical byte */
	unsigned char t[SETUP_LENGTH];  /* Setup array */
	unsigned char h[HIST_LENGTH];   /* Historical array */
	unsigned char t_len;            /* Setup array dimension */
	unsigned char h_len;            /* Historical array dimension */
};

/* ADPU-Header command structure ---------------------------------------------*/
struct adpu_header {
	unsigned char cla;  /* Command class */
	unsigned char ins;  /* Operation code */
	unsigned char p1;   /* Selection Mode */
	unsigned char p2;   /* Selection Option */
};

/* ADPU-Body command structure -----------------------------------------------*/
struct adpu_body {
	unsigned char lc;           /* Data field length */
	unsigned char data[LCMAX];  /* Command parameters */
	unsigned char le;           /* Expected length of data to be returned */
};

/* ADPU Command structure ----------------------------------------------------*/
struct adpu_cmd {
	struct adpu_header header;
	struct adpu_body body;
};

/* SC response structure -----------------------------------------------------*/
struct adpu_response {
	unsigned char data[LCMAX];  /* Data returned from the card */
	unsigned char sw1;          /* Command Processing status */
	unsigned char sw2;          /* Command Processing qualification */
};


struct jz_scc_platform_data {
	int reset_pin;
	int power_pin;

	unsigned char pwr_en_level;
};

#endif /* __MACH_JZSCC_H */
