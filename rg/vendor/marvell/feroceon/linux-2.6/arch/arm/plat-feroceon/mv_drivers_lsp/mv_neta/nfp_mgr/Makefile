#
# Makefile for the Marvell Gigabit Ethernet driver
#
ifeq ($(CONFIG_ARCH_FEROCEON_KW2),y)
	include $(CURDIR)/arch/arm/mach-feroceon-kw2/config/mvRules.mk
endif

obj-$(CONFIG_MV_ETH_NFP)    += mv_nfp_mgr.o nfp_sysfs.o nfp_fib_arp_sysfs.o ipv6_parsing.o nfp_fib6_arp_sysfs.o mv_nfp_hooks.o

obj-$(CONFIG_MV_ETH_NFP_CT) += nfp_ct_sysfs.o nfp_ct6_sysfs.o

obj-$(CONFIG_MV_ETH_NFP_CLASSIFY) += nfp_classification_sysfs.o

obj-$(CONFIG_MV_ETH_NFP_BRIDGE) += nfp_bridge_sysfs.o

obj-$(CONFIG_MV_ETH_NFP_VLAN) += nfp_vlan_sysfs.o

obj-$(CONFIG_MV_ETH_NFP_PPP) += nfp_ppp_sysfs.o


