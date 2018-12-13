#ifndef __JZ_AES_V12_H__
#define __JZ_AES_V12_H__

/*-----------------------------------------------------------------------------
 *  example procedure for app
 *  1. open, when success continue 2.
 *  2. ioctl, AES_LOAD_KEY.
 *
 *  3. loop ioctl, AES_DO_CRYPT, if(encrypt data), block to wait encrypt done, else decrypt
 *
 *  4. close.
 *-----------------------------------------------------------------------------*/

/* struct for user ioctl args. */
struct aes_key {
	char *key;
	unsigned int keylen;

	unsigned int bitmode;	/* indicate the keybits, 128,192,256. */
	unsigned int aesmode;	/* indicate the aes work mode, CBC or ECB */

	/* optional iv if in cbc mode */
	char *iv;
	unsigned int ivlen;
};


struct aes_data {
	char *input;	/* user input */
	unsigned int input_len;

	char  *output;	/* user output */

	int encrypt;
};

#define MAX_KEY_LEN_INWORD	(8)
#define IV_LEN_INWORD		(4)

/*-----------------------------------------------------------------------------
 *  User Space Ioctl Define.
 *-----------------------------------------------------------------------------*/
#define AES_IOCTL_MAGIC	'A'
#define AES_LOAD_KEY	_IOW(AES_IOCTL_MAGIC, 0x01, struct aes_key)
#define AES_DO_CRYPT	_IOWR(AES_IOCTL_MAGIC, 0x03, struct aes_data)


#define AES_KEY_128BIT		(0x00000000)
#define AES_KEY_192BIT		(0x00000001)
#define AES_KEY_256BIT		(0x00000002)

#define AES_MODE_NONE		(0x00000000)
#define AES_MODE_ECB		(0x00000001)
#define AES_MODE_CBC		(0x00000002)


#endif

