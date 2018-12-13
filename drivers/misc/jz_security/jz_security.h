#ifndef __JZ_SECURITY_H__
#define __JZ_SECURITY_H__

/*#define DEBUG*/
#ifdef CONFIG_AES_DEBUG
#define debug_printk printk
#else
#define debug_printk(fmt,arg...) do{}while(0)
#endif

#define DRV_NAME "jz-security"
#define SECURITY_INTERNAL_CHANGE_KEY  (0xffff0010)
#define SECURITY_INTERNAL_AES         (0xffff0020)
#define SECURITY_RSA                  (0xffff0030)
#define SECURITY_INTERNAL_AES_KEY		(0xffff0040)

#define RSA_BIT	(992)
#define RSA_BYTE_LEN	(RSA_BIT/8)
#define RSA_WORD_LEN	(RSA_BYTE_LEN/4)

struct rsa_aes_packet {
	unsigned short oklen; //old security key len in bits
	unsigned short nklen;// new security key len in bits
	unsigned	int * okey;// old security key
	unsigned int * nkey;// new security key
};

struct change_key_param {
	int len;			//rsa_enc_data len in bytes.
	int *rsa_enc_data;  //okey_len,nkey_len,okey,nkey
	int *nku_kr;
	int init_mode;//init key 1;init,0:not init
};

struct aes_param {
	unsigned int crypt;
	unsigned int *input;
	unsigned int *output;
	unsigned int *iv;
	unsigned int tc;
	unsigned int len; /*word len*/
};

struct rsa_param {
	unsigned int input_len;
	unsigned int key_len;
	unsigned int n_len;
	unsigned int out_len;

	unsigned int *input;
	unsigned int *key;      /*KU*/
	unsigned int *n;        /*N*/
	unsigned int *output;
	unsigned int mode;
};

#define AES_DECRYPT			0x1  //0:en  de:1
#define AES_CRYPT			0x0  //0:en  de:1

#define AES_MODE			0x2   //0:cpu   1:dma
#define AES_CBC				0x4

#define AES_BY_RKEY (0x1 << 4)
#define AES_BY_CKEY (0x1 << 5)
#define AES_BY_UKEY (0x1 << 6)

#define AES_MAX_DATA_SIZE		512 //bytes

struct aes_internal_key {
	unsigned int crypt;
	unsigned int *input;
	unsigned int *output;
	unsigned int anykey;
	unsigned int len; /*bytes len*/
};
struct security_info{
	unsigned int ker_rsa_enc_data[31];
	unsigned int ker_nku_kr[62];
	unsigned int ker_input[31];
	unsigned int ker_output[31];
	unsigned int ker_k[31];
	unsigned int ker_n[31];

};

#endif
