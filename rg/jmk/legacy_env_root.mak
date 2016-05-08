# directories that use old-version makefiles

# Please Use JMK_LEGACY_SUBDIRS where possible - 
# See vendor/atheros/Makefile

legacy_subdirs = \
  os/ \
  vendor/abaxis/ \
  vendor/airgo/ \
  vendor/cavium/ \
  vendor/intel/ \
  vendor/jstream/ \
  vendor/lexra/ \
  vendor/mango/ \
  vendor/msystems/ \
  vendor/ralink/ \
  vendor/st/ \
  vendor/tdsoft/ \
  vendor/welltech/ \
  vendor/adi/ \
  vendor/centillium/ \
  vendor/cybertan/ \
  vendor/freescale/ \
  vendor/ikanos/ \
  vendor/intersil/ \
  vendor/jungo/ \
  vendor/lsi/ \
  vendor/micrel/ \
  vendor/pci/ \
  vendor/samsung/ \
  vendor/storlink/ \
  vendor/ti/ \
  vendor/zcom/ \
  vendor/admtek/ \
  vendor/broadcom/ \
  vendor/conexant/ \
  vendor/gemtek/ \
  vendor/infineon/danube/ \
  vendor/infineon/incaip/ \
  vendor/infineon/vinax/ \
  vendor/infineon/vinetic/ \
  vendor/lantiq/xway/modules/dsp_firmware \
  vendor/jabil/ \
  vendor/kinpo/ \
  vendor/mindspeed/ \
  vendor/mototech/mvg3420n/ \
  vendor/radvision/ \
  vendor/sohoware/ \
  vendor/synopsys/ \
  vendor/us_robotics/ \
  vendor/zoom/ \
  pkg/802.11b/ \
  pkg/802.1x/ \
  pkg/acl/ \
  pkg/attr/ \
  pkg/auto_conf/ \
  pkg/autotest/ \
  pkg/av/ \
  pkg/backup/ \
  pkg/bluetooth/ \
  pkg/bootldr/ \
  pkg/boot/ \
  pkg/bridge/ \
  pkg/bridge-utils/ \
  pkg/busybox/ \
  pkg/cablehome/ \
  pkg/classpath/ \
  pkg/csg/ \
  pkg/disk_mng/ \
  pkg/disktype/ \
  pkg/dns/ \
  pkg/doc/ \
  pkg/dosfstools/ \
  pkg/e2fsprogs/ \
  pkg/ElectricFence/ \
  pkg/external/ \
  pkg/file_server/ \
  pkg/flash/ \
  pkg/freeswan/ \
  pkg/ftp/ \
  pkg/fuse/ \
  pkg/gdb/ \
  pkg/genromfs/ \
  pkg/gmp/ \
  pkg/gnudip/ \
  pkg/heimdal/ \
  pkg/home-automation/ \
  pkg/hostapd/ \
  pkg/hotplug/ \
  pkg/hping/ \
  pkg/igmp/ \
  pkg/install/ \
  pkg/ipfilter/ \
  pkg/iproute2/ \
  pkg/iptables/ \
  pkg/java/ \
  pkg/kaffe/ \
  pkg/kernel/common/ \
  pkg/kernel/linux/ \
  pkg/kos/ \
  pkg/l2tp/ \
  pkg/libpcap/ \
  pkg/libxml/ \
  pkg/libtool/ \
  pkg/lilo/ \
  pkg/lirc/ \
  pkg/lpd/ \
  pkg/lzma/ \
  pkg/macintosh/ \
  pkg/mail_client/ \
  pkg/mail/ \
  pkg/mdadm/ \
  pkg/mgt/lib/tests/ \
  pkg/mkisofs/ \
  pkg/mpatrol/ \
  pkg/mss/ \
  pkg/netkit/ \
  pkg/net-tools/ \
  pkg/ntfs-3g/ \
  pkg/ntfsprogs/ \
  pkg/osgi/ \
  pkg/pcmcia/ \
  pkg/pktgen/ \
  pkg/popt/ \
  pkg/poptop/ \
  pkg/pppoe-server/ \
  pkg/pptp-client/ \
  pkg/qos/ \
  pkg/rgloader/ \
  pkg/rip/ \
  pkg/samba/ \
  pkg/sqlite/ \
  pkg/star/ \
  pkg/stp/ \
  pkg/mtd-utils/ \
  pkg/strace/ \
  pkg/sysstat/ \
  pkg/tcpdump/ \
  pkg/telnet/ \
  pkg/termcap/ \
  pkg/testing/ \
  pkg/test_tools/ \
  pkg/tftps/ \
  pkg/tod/ \
  pkg/ucd-snmp/ \
  pkg/uclibc++/ \
  pkg/ulibc/ \
  pkg/umsdos_progs/ \
  pkg/usagi/ \
  pkg/usb_slave/ \
  pkg/util-linux/ \
  pkg/util/eresolv/test/ \
  pkg/valgrind/ \
  pkg/vendor/ \
  pkg/vlan/ \
  pkg/voip/hwemu/ \
  pkg/voip/jasterisk/ \
  pkg/voip/main/ \
  pkg/voip/openh323/ \
  pkg/voip/pwlib/ \
  pkg/voip/tr_104/ \
  pkg/voip/zaptel/ \
  pkg/voip/common/ \
  pkg/voip/dsp/ \
  pkg/voip/exosip/ \
  pkg/voip/ixj/ \
  pkg/voip/jata_osip/ \
  pkg/voip/jrtp/ \
  pkg/voip/osip/ \
  pkg/voip/sox/ \
  pkg/voip/wbm/ \
  pkg/webcam/ \
  pkg/web_cifs/ \
  pkg/web_mng/rg_cgi/ \
  pkg/wireless_tools/ \
  pkg/wsc/ \
  pkg/x509/ \
  pkg/xmlsec/ \
  pkg/zziplib/ \

legacy_subdirs_filter := $(addsuffix %,$(legacy_subdirs))

# Add $1 to legacy subdirs list
Legacy_Dir_Add = $(eval legacy_subdirs_filter+=$(addsuffix %,$1)) 

# legacy dir depends on a file from a directory with new makefiles
# $1 - dependent
# $2 - prerequisites
Dir_Dep_File = $(eval $(JMKE_BUILDDIR)/$1/__subdir: $2)

# Dir_Dep is in legacy_root_end.mak
