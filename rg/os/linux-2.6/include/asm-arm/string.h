#ifndef __ASM_ARM_STRING_H
#define __ASM_ARM_STRING_H

/*
 * We don't do inline string functions, since the
 * optimised inline asm versions are not small.
 */

#define __HAVE_ARCH_STRRCHR
extern char * strrchr(const char * s, int c);

#define __HAVE_ARCH_STRCHR
extern char * strchr(const char * s, int c);

#define __HAVE_ARCH_MEMCPY

#ifdef CONFIG_MV_XOR_MEMCOPY
extern void * xor_memcpy(void *, const void*, __kernel_size_t);
extern void * asm_memcpy(void *, const void*, __kernel_size_t);

static inline void* memcpy(void * dest, const void * src, size_t n)
{
	if (n < CONFIG_MV_XOR_MEMCOPY_THRESHOLD) 
		return asm_memcpy(dest, src, n);
	return xor_memcpy(dest, src, n);
}

#elif defined (CONFIG_MV_IDMA_MEMCOPY)

extern void * dma_memcpy(void *, const void*, __kernel_size_t);
extern void * asm_memcpy(void *, const void *, __kernel_size_t);

static inline void* memcpy(void * dest, const void * src, size_t n)
{
	if (n < CONFIG_MV_IDMA_MEMCOPY_THRESHOLD) 
		return asm_memcpy(dest, src, n);
	return dma_memcpy(dest, src, n);
}
#else
extern void * memcpy(void *, const void *, __kernel_size_t);
#endif

#define __HAVE_ARCH_MEMMOVE
extern void * memmove(void *, const void *, __kernel_size_t);

#define __HAVE_ARCH_MEMCHR
extern void * memchr(const void *, int, __kernel_size_t);

#define __HAVE_ARCH_MEMSET
extern void * memset(void *, int, __kernel_size_t);

extern void __memzero(void *ptr, __kernel_size_t n);
#define memzero __memzero
#define memset(p,v,n)							\
	({								\
	 	void *__p = (p); size_t __n = n;			\
		if ((__n) != 0) {					\
			if (__builtin_constant_p((v)) && (v) == 0)	\
				__memzero((__p),(__n));			\
			else						\
				memset((__p),(v),(__n));		\
		}							\
		(__p);							\
	})

#endif
