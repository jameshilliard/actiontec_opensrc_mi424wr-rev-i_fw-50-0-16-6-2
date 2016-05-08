/*******************************************************************************
Copyright (C) Marvell International Ltd. and its affiliates

This software file (the "File") is owned and distributed by Marvell
International Ltd. and/or its affiliates ("Marvell") under the following
alternative licensing terms.  Once you have made an election to distribute the
File under one of the following license alternatives, please (i) delete this
introductory statement regarding license alternatives, (ii) delete the two
license alternatives that you have not elected to use and (iii) preserve the
Marvell copyright notice above.

********************************************************************************
Marvell GPL License Option

If you received this File from Marvell, you may opt to use, redistribute and/or
modify this File in accordance with the terms and conditions of the General
Public License Version 2, June 1991 (the "GPL License"), a copy of which is
available along with the File in the license.txt file or by writing to the Free
Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 or
on the worldwide web at http://www.gnu.org/licenses/gpl.txt.

THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE IMPLIED
WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY
DISCLAIMED.  The GPL License provides additional details about this warranty
disclaimer.
*******************************************************************************/

#include "mvCommon.h"
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/platform_device.h>

#include "mvOs.h"
#include "mvDebug.h"
#include "mvSysHwConfig.h"
#include "boardEnv/mvBoardEnvLib.h"
#include "ctrlEnv/mvCtrlEnvLib.h"
#include "ctrlEnv/mvCtrlEthCompLib.h"
#include "eth-phy/mvEthPhy.h"
#include "mv_switch.h"

#include "../net_dev/mv_netdev.h"
#include "mv_wan_swap.h"

#define SWITCH_PHY_ADDR	0x8
#define RGMII_PHY_ADDR	0x10
#define GEPHY_PHY_ADDR	0x9

/* WAN swap feature */
static int mv_eth_ctrl_wan_mode(int wan_mode)
{
	/* WAN mode: 0 - MoCA, 1 - GbE */
	printk(KERN_CONT "WAN swap requested: new WAN mode is ");
	if (wan_mode == MV_WAN_MODE_MOCA)
		printk(KERN_CONT "MoCA\n");
	else
		printk(KERN_CONT "GbE\n");

#ifdef CONFIG_MV_ETH_SWITCH
	if ((mvBoardSwitchConnectedPortGet(0) != -1) || (mvBoardSwitchConnectedPortGet(1) != -1)) {
		if (mv_switch_unload(SWITCH_CONNECTED_PORTS_MASK))
			printk(KERN_ERR "Warning: Switch unload failed\n");
	}

	mv_eth_switch_swap_config_str();
#endif

	if (mv_eth_init_module())
		printk(KERN_ERR "Error: re-initialization of Marvell Ethernet Driver failed\n");

	return 0;
}

int mv_eth_wan_swap(int wan_mode)
{
	MV_U32 oldCfg;
	MV_U32 newCfg;
	MV_U32 irq_mask;
	MV_U32 portMode;
	static int prev_wan_mode = MV_WAN_MODE_MOCA; /* new mode: 0 - MoCA, 1 - GbE */
	static MV_U32 firstInit = 1;
	MV_STATUS status = 0;

	portMode = mvBoardEthPortsModeGet();

	if (prev_wan_mode == wan_mode) {
		/* do nothing */
		printk(KERN_ERR "Requested WAN mode is identical to the current mode - doing nothing...\n");
		return 0;
	}

	/* Disable TWSI interrupt/s */
	irq_mask = MV_REG_READ(MV_IRQ_MASK_HIGH_REG);
	if (irq_mask & BIT18)
		disable_irq(TWSI_IRQ_NUM(0));
	if (irq_mask & BIT25)
		disable_irq(TWSI_IRQ_NUM(1));

	if (mv_eth_check_all_ports_down()) {
		status = -1;
		goto out;
	}

	if (mv_eth_all_ports_cleanup())
		printk(KERN_ERR "Error: mv_eth_all_ports_cleanup failed\n");

	oldCfg = mvBoardEthComplexConfigGet();
	if (oldCfg & ESC_OPT_MAC1_2_SW_P5) {
		/* sanity check */
		if (prev_wan_mode != MV_WAN_MODE_MOCA)
			printk(KERN_ERR "Error: prev_wan_mode is GbE but configuration is MoCA\n");

		/* GbE mode */
		newCfg = oldCfg & ~(ESC_OPT_MAC1_2_SW_P5 | ESC_OPT_RGMIIB_MAC0);
		newCfg |= ESC_OPT_MAC0_2_SW_P4;
		newCfg |= ESC_OPT_GEPHY_MAC1;
		mvEthGmacRgmiiSet(0, 0);
		mvEthGmacRgmiiSet(1, 0);
		mvBoardMacSpeedSet(0, BOARD_MAC_SPEED_1000M);
		mvBoardMacSpeedSet(1, BOARD_MAC_SPEED_AUTO);
		mvBoardPhyAddrSet(0, SWITCH_PHY_ADDR);
		mvBoardPhyAddrSet(1, GEPHY_PHY_ADDR);
		mvCtrlEthComplexMppUpdate(ESC_OPT_RGMIIB_MAC0, MV_FALSE);
	} else {
		/* sanity check */
		if (prev_wan_mode != MV_WAN_MODE_GBE)
			printk(KERN_ERR "Error: prev_wan_mode is MoCA but configuration is GbE\n");

		/* MoCA mode */
		newCfg = oldCfg & ~(ESC_OPT_MAC0_2_SW_P4 | ESC_OPT_GEPHY_MAC1);
		newCfg |= ESC_OPT_MAC1_2_SW_P5;
		newCfg |= ESC_OPT_RGMIIB_MAC0;

		if (portMode & EPM_MAC0_MII)
		{
		    /* Rev-I MoCA is in TMII mode */
		    mvEthGmacRgmiiSet(0, 0); 
		    mvBoardMacSpeedSet(0, BOARD_MAC_SPEED_100M); 
		}
		else
		{
		    /* Rev-J MoCA is in RGMII mode */
		    mvEthGmacRgmiiSet(0, 1); 
		    mvBoardMacSpeedSet(0, BOARD_MAC_SPEED_1000M); 
		}
		mvEthGmacRgmiiSet(1, 0);
		mvBoardMacSpeedSet(1, BOARD_MAC_SPEED_1000M);
		mvBoardPhyAddrSet(0, RGMII_PHY_ADDR);
		mvBoardPhyAddrSet(1, SWITCH_PHY_ADDR);
		mvCtrlEthComplexMppUpdate(ESC_OPT_RGMIIB_MAC0, MV_TRUE);
	}

	mvBoardEthComplexConfigSet(newCfg);
	mvBoardSwitchInfoUpdate();
	mvEthernetComplexChangeMode(oldCfg, newCfg);

	if ((firstInit) && (newCfg & ESC_OPT_GEPHY_MAC1)) {
		firstInit = 0;
		mvOsDelay(1);
		mvEthInternalGEPhyBasicInit(mvBoardPhyAddrGet(1), 1);
	}

	mvBoardMppModuleTypePrint();

	status = mv_eth_ctrl_wan_mode(wan_mode);
	if (status == MV_OK)
		prev_wan_mode = wan_mode;

out:
	/* Enable TWSI interrupt/s */
	if (irq_mask & BIT18)
		enable_irq(TWSI_IRQ_NUM(0));
	if (irq_mask & BIT25)
		enable_irq(TWSI_IRQ_NUM(1));

	return status;
}

static ssize_t wan_swap_help(char *buf)
{
	int off = 0;

	off += scnprintf(buf + off, PAGE_SIZE - off, "cat              help   - Show this help\n");
	off += scnprintf(buf + off, PAGE_SIZE - off, "cat              config - Show WAN port mode\n");
	off += scnprintf(buf + off, PAGE_SIZE - off, "echo moca|gbe  > config - Set WAN port mode\n");

	return off;
}

static ssize_t wan_swap_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int off = 0;
	MV_U32 ethComp;
	const char  *name = attr->attr.name;

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;

	if (!strcmp(name, "help"))
		return wan_swap_help(buf);

	if (!strcmp(name, "config")) {
		ethComp = mvBoardEthComplexConfigGet();
		if (ethComp & ESC_OPT_MAC0_2_SW_P4)
			off += scnprintf(buf + off, PAGE_SIZE - off, "GbE WAN Mode: Switch on MAC0, GE-PHY on MAC1.\n");
		else if (ethComp & ESC_OPT_MAC1_2_SW_P5)
			off += scnprintf(buf + off, PAGE_SIZE - off, "MoCA WAN Mode: Switch on MAC1, RGMII-B on MAC0.\n");
	} else {
		printk(KERN_ERR "%s: illegal operation <%s>\n", __func__, attr->attr.name);
		return -EINVAL;
	}
	return off;
}

static ssize_t wan_swap_store(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t len)
{
	const char    *name = attr->attr.name;
	unsigned int  err = 0;

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;

	if (!strcmp(name, "config")) {
		/* Switch ethernet complex configuration. */
		if (!strncmp(buf, "moca", 4))
			err = mv_eth_wan_swap(0);
		else if (!strncmp(buf, "gbe", 3))
			err = mv_eth_wan_swap(1);
		else {
			printk(KERN_ERR "'moca' | 'gbe' should be used as WAN mode\n");
			err = 1;
		}
	} else {
		printk(KERN_ERR "%s: illegal operation <%s>\n", __func__, name);
		err = 1;
	}
	return err ? -EINVAL : len;
}

static DEVICE_ATTR(help,	          S_IRUSR, wan_swap_show,  NULL);
static DEVICE_ATTR(config,	S_IWUSR | S_IRUSR, wan_swap_show, wan_swap_store);


static struct attribute *wan_swap_attrs[] = {
	&dev_attr_help.attr,
	&dev_attr_config.attr,
	NULL
};

static struct attribute_group wan_swap_group = {
	.name = "wan_swap",
	.attrs = wan_swap_attrs,
};

int __devinit mv_wan_swap_sysfs_init(void)
{
	int err;
	struct device *pd;

	pd = bus_find_device_by_name(&platform_bus_type, NULL, "neta");
	if (!pd) {
		platform_device_register_simple("neta", -1, NULL, 0);
		pd = bus_find_device_by_name(&platform_bus_type, NULL, "neta");
	}

	if (!pd) {
		printk(KERN_ERR"%s: cannot find neta device\n", __func__);
		pd = &platform_bus;
	}

	err = sysfs_create_group(&pd->kobj, &wan_swap_group);
	if (err) {
		printk(KERN_INFO "sysfs group failed %d\n", err);
		goto out;
	}
out:
	return err;
}

module_init(mv_wan_swap_sysfs_init);

MODULE_AUTHOR("Dmitri Epshtein");
MODULE_DESCRIPTION("sysfs for marvell wan swap support");
MODULE_LICENSE("GPL");

