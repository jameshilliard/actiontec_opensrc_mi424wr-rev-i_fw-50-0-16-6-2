/* l2fw_sysfs.c */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/capability.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>

#include "mvTypes.h"
#include "mv_eth_l2fw.h"
#include "linux/inet.h"

static ssize_t l2fw_help(char *buf)
{
	int off = 0;
	off += sprintf(buf+off, "help\n");
	off += sprintf(buf+off, "echo mode rxp txp > l2fw - set l2f <rxp>->");
	off += sprintf(buf+off, "<txp><mode> 0-dis,1-as_is,2-swap,3-copy\n");
	off += sprintf(buf+off, "echo threshold > l2fw_xor: set threshold\n");
	off += sprintf(buf+off, "echo 1 > esp   - enable ESP\n");
	off += sprintf(buf+off, "cat dump - display L2fw rules DB\n");
	off += sprintf(buf+off, "echo 1 > flush - flush L2fw rules DB\n");
	return off;
}

static ssize_t l2fw_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
    const char	*name = attr->attr.name;
    int             off = 0;

    if (!capable(CAP_NET_ADMIN))
	return -EPERM;

	if (!strcmp(name, "help")) {
	    off = l2fw_help(buf);
		return off;
	}
	return off;
}




static ssize_t l2fw_hex_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t len)
{
	/* const char      *name = attr->attr.name; */
	int             err;
	unsigned int    addr1, addr2, port;
	unsigned long   flags;

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;

	err = addr1 = addr2 = port = 0;

	local_irq_save(flags);
	/* for future use. */
	local_irq_restore(flags);

	return err ? -EINVAL : len;
}

static ssize_t l2fw_store(struct device *dev,
				   struct device_attribute *attr, const char *buf, size_t len)
{
	const char	*name = attr->attr.name;
	int             err;

	unsigned int    p, txp, txq, v;
	unsigned long   flags;

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;

	err = p = txp = txq = v = 0;
	sscanf(buf, "%d %d %d %d", &p, &txp, &txq, &v);

	local_irq_save(flags);

	if (!strcmp(name, "l2fw_xor"))
		l2fw_xor(p);

	else if (!strcmp(name, "l2fw"))
		l2fw(p, txp, txq);

	local_irq_restore(flags);

	if (err)
		mvOsPrintf("%s: error %d\n", __func__, err);

	return err ? -EINVAL : len;

}


static DEVICE_ATTR(l2fw,		S_IWUSR, l2fw_show, l2fw_store);
static DEVICE_ATTR(l2fw_xor,	S_IWUSR, l2fw_show, l2fw_store);
static DEVICE_ATTR(l2fw_add,	S_IWUSR, l2fw_show, l2fw_hex_store);
static DEVICE_ATTR(l2fw_add_ip,	S_IWUSR, l2fw_show, l2fw_hex_store);
static DEVICE_ATTR(help,		S_IRUSR, l2fw_show,  NULL);
static DEVICE_ATTR(dump,		S_IRUSR, l2fw_show,  NULL);
static DEVICE_ATTR(flush,		S_IRUSR, l2fw_show,  NULL);
static DEVICE_ATTR(esp,			S_IWUSR, l2fw_show,  l2fw_store);


static struct attribute *l2fw_attrs[] = {
	&dev_attr_l2fw.attr,
	&dev_attr_l2fw_xor.attr,
	&dev_attr_l2fw_add.attr,
	&dev_attr_l2fw_add_ip.attr,
	&dev_attr_help.attr,
	&dev_attr_dump.attr,
	&dev_attr_flush.attr,
	&dev_attr_esp.attr,
	NULL
};

static struct attribute_group l2fw_group = {
	.name = "l2fw",
	.attrs = l2fw_attrs,
};

#ifdef CONFIG_MV_ETH_L2FW
int __devinit mv_l2fw_sysfs_init(void)
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

	err = sysfs_create_group(&pd->kobj, &l2fw_group);
	if (err) {
		printk(KERN_INFO "sysfs group failed %d\n", err);
		goto out;
	}
out:
	return err;
}
#endif

module_init(mv_l2fw_sysfs_init);

MODULE_AUTHOR("Rami Rosen");
MODULE_DESCRIPTION("sysfs for marvell l2fw");
MODULE_LICENSE("GPL");

