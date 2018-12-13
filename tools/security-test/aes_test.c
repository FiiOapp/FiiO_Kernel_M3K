#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <errno.h>
#include "aes_test.h"

#define DRV_NAME "/dev/jz-security"

#define AES_MAX_DATA_SIZE       512 //bytes

#define AES_DECRYPT         0x1  //0:en  de:1
#define AES_CRYPT           0x0  //0:en  de:1

int main()
{
	int ret = 0;
	int fd,test_fd,test_fd2;

	unsigned char aes_inbuf[AES_MAX_DATA_SIZE] = {0};
	unsigned char aes_outbuf[AES_MAX_DATA_SIZE] = {0};

	unsigned int cp_len = 0;

	struct aes_internal_key aes;
	struct stat src_stat;

	fd = open(DRV_NAME,O_RDWR);
	if(fd < 0) {
		printf("error! Can't open jz_security node, err = %d \n", -errno);
		return -errno;
	}
	if(stat("./testfile",&src_stat))
	{
		printf("stat testfile fail !\n %s \n",strerror(ENOENT));
		return -ENOENT;
	}
	printf("testfile size = %d\n",src_stat.st_size);

	/* crypt by aes userkey */
	test_fd = open("./testfile",O_RDWR); //testfile: src file signature by signature tool with "other file" mode
	test_fd2 = open("./testfile_src",O_RDWR | O_CREAT, S_IRUSR | S_IWUSR); // after aes decrypt , get the src file

	while(cp_len = read(test_fd,aes_inbuf,AES_MAX_DATA_SIZE)){

		memset(aes_outbuf,0,AES_MAX_DATA_SIZE);

		aes.crypt = AES_DECRYPT; //crypt or decrypt
		aes.input = aes_inbuf;
		aes.output = aes_outbuf;
		aes.anykey = 1; // just support uk to crypt or decrypt
		aes.len = cp_len;	// ops len
		ret = ioctl(fd, SECURITY_INTERNAL_AES_KEY, &aes);

		write(test_fd2,aes_outbuf,cp_len);
		memset(aes_inbuf,0,AES_MAX_DATA_SIZE);
	}

	close(fd);
	close(test_fd);
	close(test_fd2);
	return 0;
}
