#include "string.h"

/*
 * Compare strings.
 */
int strcmp(const char *s1, const char *s2)
{
	while (*s1 == *s2++)
		if (*s1++ == 0)
			return (0);
	return (*(unsigned char *)s1 - *(unsigned char *)--s2);
}



void* memcpy(void *dst0, const void *src0, size_t length)
{
	char * d;
	char * s;

	while (length--) {
		*d++ = *s++;
	}

	return dst0;
}


void* memset(void*  dst, int c, size_t n)
{
    char*  q   = dst;
    char*  end = q + n;

    for (;;) {
        if (q >= end) break; *q++ = (char) c;
        if (q >= end) break; *q++ = (char) c;
        if (q >= end) break; *q++ = (char) c;
        if (q >= end) break; *q++ = (char) c;
    }

  return dst;
}
