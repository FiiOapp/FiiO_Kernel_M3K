#ifndef __AES_TEST_H__
#define __AES_TEST_H__

#define SECURITY_INTERNAL_CHANGE_KEY    (0xffff0010)
#define SECURITY_INTERNAL_AES           (0xffff0020)
#define SECURITY_RSA                    (0xffff0030)
#define SECURITY_INTERNAL_AES_KEY       (0xffff0040)


struct aes_internal_key {
	unsigned int crypt;
	unsigned int *input;
	unsigned int *output;
	unsigned int anykey;
	unsigned int len; /*word len*/
};

#endif
