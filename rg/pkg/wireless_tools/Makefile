JMK_ROOT=../../
include $(JMK_ROOT)/envir.mak

JMK_TARGET+=iwconfig iwlist iwpriv iwspy iwgetid iwevent ifrename

JMK_INTERNAL_HEADERS=iwlib.h iwcommon.h wireless.h
JMK_RAMDISK_LIB_FILES+=$(JMK_SO_TARGET)

JMK_O_OBJS_iwconfig+=iwconfig.o
JMK_O_OBJS_iwlist+=iwlist.o
JMK_O_OBJS_iwpriv+=iwpriv.o
JMK_O_OBJS_iwspy+=iwspy.o
JMK_O_OBJS_iwgetid+=iwgetid.o
JMK_O_OBJS_iwevent+=iwevent.o
JMK_O_OBJS_ifrename+=ifrename.o

ifndef CONFIG_DYN_LINK
  LIBIW=libiw.a
  JMK_A_TARGET+=libiw.a
else
  LIBIW=libiw.so
  JMK_SO_TARGET+=libiw.so
endif

JMK_O_OBJS_$(LIBIW)+=iwlib.o

JMK_LDLIBS:=-liw -lm $(JMK_LDLIBS)
JMK_LDFLAGS+=-L.

JMK_RAMDISK_BIN_FILES+=$(JMK_TARGET)

JMK_INTERNAL_HEADERS+=iwlib.h wireless.h
include $(JMKE_MK)
