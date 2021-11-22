/******************************************************************************
 * Copyright (c) 2020 Boxing
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 *
*******************************************************************************/

#define DRIVER_VERSION      "0.2"

#include <linux/module.h>      /* Needed by all modules */
#include <linux/version.h>
#include <linux/io.h>
#include <linux/radix-tree.h>
#include <linux/irq.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_irq.h>
#include <linux/uio_driver.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/types.h>
#include <linux/printk.h>
#include <linux/slab.h>
#include <asm/io.h>
#include <asm/octeon/octeon.h>
#include "kernel_compat.h"

#define OCT_MBOX_IRQ_UIO_DISABLE	0
#define OCT_MBOX_IRQ_UIO_ENABLE		1
#define UIO_NAME_SIZE			16
#define REDUN_NOTIFY_BITPOS		12

static DEFINE_SPINLOCK(oct_mbox_irq_lock);
static LIST_HEAD(oct_mbox_irq_list);
static RADIX_TREE(irq_desc_tree, GFP_KERNEL);
static spinlock_t irq_lock; /* lock for irq_desc_tree */

struct oct_mbox_irq_info {
	struct list_head list;
	struct kobject kobj;
	raw_spinlock_t lock;
	struct platform_device *pdev;
	unsigned int rx_bitmask;
	unsigned int rx_irq_bitpos;
	unsigned int tx_irq_bitpos;
	const char *uio_name;
	struct uio_info *uio;
	unsigned long int uio_irq_rx_count;
	unsigned long int uio_irq_tx_count;
	unsigned int irq;
	unsigned int state; /* to control state by sysfs */
	struct irq_chip	*irq_chip; /* The chip that handles the IRQ */
	unsigned int uio_registered:2;
	unsigned int irq_registered:2;
	unsigned int kobj_registered:2;
	unsigned long masked; /* to simulate irq mask/umask(0=unmasked, 1=masked) */
	unsigned long flags; /* to manage irq state(0=enabled, 1=disabled) */
	unsigned int ipi_handler_registered:2;
	unsigned int dst_cpu;
};

static int debug = 0;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Enable debug logging");

#define DEV_DBG(dev,fmt,...) \
do { \
        if(debug) \
                dev_info(dev, "[%s(%d)] " fmt, __func__, \
			 __LINE__, ##__VA_ARGS__); \
} while(0)

static irqreturn_t uio_handler(int irq, struct uio_info *uio)
{
	struct oct_mbox_irq_info *info = uio->priv;

	/* 
	 * dont't disable irq before sent to userspace
	 * because userspace don't enable it any more.
	 * Fixme: disable it if userspace need.
	 * if (!test_and_set_bit(0, &info->flags))
	 *	disable_irq_nosync(info->irq);
	 */

	if (info->state == OCT_MBOX_IRQ_UIO_DISABLE)
		return IRQ_NONE;

	info->uio_irq_rx_count++;

	return IRQ_HANDLED;
}

static int oct_mbox_irq_control(struct uio_info *uio, s32 irq_on)
{
	struct oct_mbox_irq_info *info = uio->priv;
	struct device *dev = &info->pdev->dev;

	DEV_DBG(dev, "irq %d,%s\n", info->irq, irq_on ? "enabled." : "disabled.");

	if (irq_on) {
		if (test_and_clear_bit(0, &info->flags))
			enable_irq(info->irq); /* not yet enabled so enable now */
	} else {
		if ((info->tx_irq_bitpos == REDUN_NOTIFY_BITPOS) && (info->state == OCT_MBOX_IRQ_UIO_ENABLE)) {
			unsigned int irq_index;
			DEV_DBG(dev, "trig,tx_irq_bitpos=%d\n", info->tx_irq_bitpos);
			irq_index = 1 << info->tx_irq_bitpos;
			cvmx_write_csr(CVMX_CIU_MBOX_SETX(info->dst_cpu), irq_index);
			info->uio_irq_tx_count++;
			DEV_DBG(dev, "send redun notify to core-%d,reg_val=0x%x,tx_count=%lu\n", info->dst_cpu, irq_index, info->uio_irq_tx_count);
		}
		if (!test_and_set_bit(0, &info->flags))
			disable_irq(info->irq); /* not yet disabled so disable now */
	}

	return 0;
}

static int oct_mbox_irq_register_uiodev(struct platform_device *pdev, struct oct_mbox_irq_info *info)
{
	struct device *dev = &pdev->dev;
	struct uio_info *uio;
	int ret = 0;

	uio = devm_kzalloc(dev, sizeof(*uio), GFP_KERNEL);
	if (!uio) {
		dev_err(dev, "cannot alloc uio memory\n");
		return -ENOMEM;
	}

	uio->name = info->uio_name;
	uio->version = "0.0.1";
	uio->irq = info->irq;
	uio->irqcontrol = oct_mbox_irq_control;
	uio->handler = uio_handler;
	uio->irq_flags = IRQF_SHARED;
	uio->priv = info;

	info->uio = uio;

	ret = uio_register_device(dev, uio);
	if (ret) {
		dev_err(dev, "register uio device fail");
		devm_kfree(dev, uio);
	}
	return ret;
}

static int oct_mbox_irq_set_state(struct oct_mbox_irq_info *info, const char *buf)
{
	unsigned int state = (__be32)simple_strtoul(buf, NULL, 10);
	struct device *dev = &info->pdev->dev;
	unsigned long flags;

	if ((state != OCT_MBOX_IRQ_UIO_ENABLE) && (state != OCT_MBOX_IRQ_UIO_DISABLE)) {
		DEV_DBG(dev, "bad input. enable=1, disable=0\n");
		return -EINVAL;
	}

	raw_spin_lock_irqsave(&info->lock, flags);
	if (state == OCT_MBOX_IRQ_UIO_ENABLE) {
		info->state = OCT_MBOX_IRQ_UIO_ENABLE;
	} else {
		info->state = OCT_MBOX_IRQ_UIO_DISABLE;
		info->uio_irq_rx_count = 0;
	}
	raw_spin_unlock_irqrestore(&info->lock, flags);

	DEV_DBG(dev, "irq %d %s.\n", info->irq,
		(state == OCT_MBOX_IRQ_UIO_ENABLE) ? "enabled" : "disabled");

	return 0;
}

static ssize_t oct_mbox_irq_show(struct kobject *kobj, struct attribute *attr, char *buf)
{
	struct oct_mbox_irq_info *info;
	int ret;

	info = container_of(kobj, struct oct_mbox_irq_info, kobj);

	if (strcmp(attr->name, "irq") == 0)
		ret = sprintf(buf, "%d\n", info->irq);
	else if (strcmp(attr->name, "state") == 0)
		ret = sprintf(buf, "%d\n", info->state);
	else if (strcmp(attr->name, "uio_irq_rx_count") == 0)
		ret = sprintf(buf, "%lu\n", info->uio_irq_rx_count);
	else if (strcmp(attr->name, "uio_irq_tx_count") == 0)
		ret = sprintf(buf, "%lu\n", info->uio_irq_tx_count);	else
		ret = -EIO;

	return ret;
}

static ssize_t oct_mbox_irq_store(struct kobject *kobj, struct attribute * attr, const char *buf, size_t count)
{
	struct oct_mbox_irq_info *info;
	struct device *dev;
	int ret = 0;

	info = container_of(kobj, struct oct_mbox_irq_info, kobj);
	dev = &info->pdev->dev;

	if (strncmp(attr->name, "state", 5) == 0) {
		ret = oct_mbox_irq_set_state(info, buf);
	} else if (strncmp(attr->name, "dst_cpu", 7) == 0) {
		info->dst_cpu = (__be32)simple_strtoul(buf, NULL, 10);
		DEV_DBG(dev, "dst_cpu=%u\n", info->dst_cpu);
	} else if (strncmp(attr->name, "trig", 4) == 0) {
		DEV_DBG(dev, "trig,tx_irq_bitpos=%d\n", info->tx_irq_bitpos);
		if (!info->tx_irq_bitpos)
			return count;
		if (info->state == OCT_MBOX_IRQ_UIO_ENABLE) {
			unsigned int irq_index;
			irq_index = 1 << info->tx_irq_bitpos;
			cvmx_write_csr(CVMX_CIU_MBOX_SETX(info->dst_cpu), irq_index);
		} else {
			printk("Irq not enabled!\n");
			ret =  -EINVAL;
		}
	} else {
		ret = -EINVAL;
	}

	return ret < 0 ? -EINVAL : count;
}

static struct attribute irq_attr = SYSFS_ATTR(irq, S_IRUGO);
static struct attribute state_attr = SYSFS_ATTR(state, S_IRUGO | S_IWUSR);
static struct attribute uio_irq_rx_count_attr = SYSFS_ATTR(uio_irq_rx_count, S_IRUGO);
static struct attribute uio_irq_tx_count_attr = SYSFS_ATTR(uio_irq_tx_count, S_IRUGO);
static struct attribute dst_cpu_attr = SYSFS_ATTR(dst_cpu, S_IRUGO | S_IWUSR);
static struct attribute trig_attr = SYSFS_ATTR(trig, S_IWUSR);

static struct attribute *oct_mbox_irq_attrs[] = {
	&irq_attr,
	&state_attr,
	&uio_irq_rx_count_attr,
	&uio_irq_tx_count_attr,
	&trig_attr,
	&dst_cpu_attr,
	NULL
};

static struct sysfs_ops oct_mbox_irq_sysfs_ops = {
	.show = oct_mbox_irq_show,
	.store = oct_mbox_irq_store
};

static struct kobj_type oct_mbox_irq_kobj_type = {
	.sysfs_ops = &oct_mbox_irq_sysfs_ops,
	.default_attrs = oct_mbox_irq_attrs
};

static struct oct_mbox_irq_info *irq_to_info(unsigned int irq)
{
	unsigned long flags;
	struct oct_mbox_irq_info *info;

	spin_lock_irqsave(&irq_lock, flags);
	info = radix_tree_lookup(&irq_desc_tree, irq);
	spin_unlock_irqrestore(&irq_lock, flags);

	return info;
}

static void oct_mbox_irq_data_mask_ack(struct irq_data *data)
{
	struct oct_mbox_irq_info *info = irq_to_info(data->irq);

	test_and_set_bit(0, &info->masked);
}

static void oct_mbox_irq_data_unmask(struct irq_data *data)
{
	struct oct_mbox_irq_info *info = irq_to_info(data->irq);

	test_and_clear_bit(0, &info->masked);
}

static struct irq_chip oct_mbox_irq_chip = {
	.name		= "OCT-MBOX",
	.irq_mask_ack	= oct_mbox_irq_data_mask_ack,
	.irq_unmask	= oct_mbox_irq_data_unmask,
};

static int register_irq(struct oct_mbox_irq_info *info)
{
	int start;
	unsigned long flags;

	start = irq_alloc_desc_from(1, 0);
	if (start < 0)
		return start;

	info->irq = start;

	spin_lock_irqsave(&irq_lock, flags);
	radix_tree_insert(&irq_desc_tree, start, info);
	spin_unlock_irqrestore(&irq_lock, flags);

	irq_clear_status_flags(start, IRQ_NOREQUEST);
	irq_set_status_flags(start, IRQ_LEVEL);

	return start;
}

static void unregister_irq(struct oct_mbox_irq_info *info)
{
	unsigned long	flags;

	irq_free_descs(info->uio->irq, 1);

	spin_lock_irqsave(&irq_lock, flags);
	radix_tree_delete(&irq_desc_tree, info->uio->irq);
	spin_unlock_irqrestore(&irq_lock, flags);
}

static int oct_mbox_register_irq(struct oct_mbox_irq_info *info)
{
	int ret;
	struct device *dev = &info->pdev->dev;

	/* Sanity checks (should never occur even with device tree errors) */
	if (info->irq_registered)
		return -EINVAL;

	ret = register_irq(info);
	if (ret < 0) {
		dev_err(dev, "cannot initialize new irq (errno %d)\n", ret);
		return ret;
	}

	info->irq_chip = &oct_mbox_irq_chip;
	if (info->irq_chip->irq_disable == NULL)
		info->irq_chip->irq_disable = info->irq_chip->irq_mask;

	ret = irq_set_chip(info->irq, info->irq_chip);
	if (ret < 0) {
		dev_err(dev, "cannot set chip for irq %d (errno %d)\n",
			info->irq, ret);
		unregister_irq(info);
		return ret;
	}

	irq_set_handler(info->irq, handle_level_irq);

	info->irq_registered = 1;

	dev_info(dev, "registered interrupt %d\n", info->irq);
	return 0;
}

static void bitpos_handler(void)
{
	struct oct_mbox_irq_info *info = NULL;
	u32 coreid = cvmx_get_core_num();
	u32 ciu_mbox_clr = octeon_get_message_csr(coreid);

	list_for_each_entry(info, &oct_mbox_irq_list, list) {
		if (info->rx_bitmask & ciu_mbox_clr) {
			if (test_bit(0, &info->masked))
				goto clear_csr;
			generic_handle_irq(info->irq);
		}
	}

clear_csr:
	octeon_clear_message_csr(coreid);
}

static int oct_mbox_irq_remove(struct platform_device *pdev);

static int oct_mbox_irq_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *of_node = dev->of_node;
	struct oct_mbox_irq_info *info = NULL;
	int ret;
	int i, irq_count = 0;

	spin_lock_init(&irq_lock);

	irq_count = of_property_count_u32_elems(of_node, "rx-irq-bitpos");
	if (irq_count < 0) {
        	dev_err(dev, "No 'rx-irq-bitpos' property found\n");
        	return -EINVAL;
    	}

	info = devm_kcalloc(&pdev->dev, irq_count,
			    sizeof(*info), GFP_KERNEL);
	if (!info) {
		dev_err(dev, "cannot allocate oct_mbox_irq resource\n");
		return -ENOMEM;
	}

	for (i = 0; i < irq_count; i++, info++) {
		INIT_LIST_HEAD(&info->list);
		info->state = OCT_MBOX_IRQ_UIO_DISABLE;
		raw_spin_lock_init(&info->lock);
		info->pdev = pdev;

		info->dst_cpu = 1; /* set dst_cpu to core1 by default */

		spin_lock(&oct_mbox_irq_lock);
		list_add(&info->list, &oct_mbox_irq_list);
		spin_unlock(&oct_mbox_irq_lock);

		of_property_read_u32_index(of_node, "rx-irq-bitpos", i, &info->rx_irq_bitpos);

		ret = of_property_read_u32_index(of_node, "tx-irq-bitpos", i, &info->tx_irq_bitpos);
		if (ret) {
        		dev_err(dev, "No 'tx-irq-bitpos' property found\n");
			goto fail_out;
		}

		ret = of_property_read_string_index(of_node, "uio-names",
					   	    i, &info->uio_name);
		if (ret) {
			dev_err(dev, "no uio-names property not found\n");
			goto fail_out;
		}
		dev_info(dev, "uio_name=%s.\n", info->uio_name);

		if (info->rx_irq_bitpos) {
			ret = oct_mbox_register_irq(info);
			if (ret < 0) {
				dev_err(dev, "cannot register irq (errno %d)\n", ret);
				goto fail_out;
			}
			info->irq_registered = 1;

			info->rx_bitmask =
			octeon_request_ipi_handler_with_pos((octeon_message_fn_t)bitpos_handler, info->rx_irq_bitpos);
			if (info->rx_bitmask < 0) {
				dev_err(dev, "request ipi handler failed (errno %d)\n",
					info->rx_bitmask);
				ret = info->rx_bitmask;
				goto fail_out;
			}
			dev_info(dev, "info->rx_bitmask=0x%x\n", info->rx_bitmask);
			info->ipi_handler_registered = 1;
		}

		dev_info(dev, "Register uio device.\n");
		ret = oct_mbox_irq_register_uiodev(pdev, info);
		if (ret) {
			dev_err(dev, "cannot init irq %d, %s\n",
				info->irq, of_node->full_name);
			goto fail_out;
		}
		info->uio_registered = 1;

		ret = kobject_init_and_add(&info->kobj,
					   &oct_mbox_irq_kobj_type, &dev->kobj,
					   info->uio_name,
					   info->rx_irq_bitpos);
		if (ret) {
			dev_err(dev, "cannot add kobject resource\n");
			goto fail_out;
		}
		info->kobj_registered = 1;
	}

	platform_set_drvdata(pdev, &oct_mbox_irq_list);

        return 0;
fail_out:
	oct_mbox_irq_remove(pdev);
	return ret;
}

static int oct_mbox_irq_remove(struct platform_device *pdev)
{
	struct oct_mbox_irq_info *info;
	struct list_head *irq_list = platform_get_drvdata(pdev);

	list_for_each_entry(info, irq_list, list) {
		if (info->ipi_handler_registered)
			octeon_release_ipi_handler(info->rx_bitmask);
		if (info->uio_registered)
			unregister_irq(info);
		if (info->kobj_registered)
			kobject_put(&info->kobj);
		if (info->uio_registered) {
			uio_unregister_device(info->uio);
			devm_kfree(&pdev->dev, info->uio);
		}
		devm_kfree(&pdev->dev, info);
	}

	return 0;
}

static const struct of_device_id oct_mbox_irq_ids[] = {
	{
	 	.compatible = "boxing,octeon-mailbox-irq",
	},
	{},
};

static struct platform_driver oct_mbox_irq_driver = {
	.driver = {
			.owner = THIS_MODULE,
			.name = "Boxing octeon mailbox irq",
			.of_match_table = oct_mbox_irq_ids,
		},
	.probe = oct_mbox_irq_probe,
	.remove = oct_mbox_irq_remove,
};

static int __init oct_mbox_irq_init(void)
{
	printk(KERN_INFO "Boxing octeon mailbox irq: module init version %s\n", DRIVER_VERSION);
	return platform_driver_register(&oct_mbox_irq_driver);
}

static void __exit oct_mbox_irq_exit(void)
{
	platform_driver_unregister(&oct_mbox_irq_driver);
}

module_init(oct_mbox_irq_init);
module_exit(oct_mbox_irq_exit);

MODULE_AUTHOR("Sun Jianguo <jianguo.sun@boxing.com>");
MODULE_DESCRIPTION("Boxing octeon mailbox irq driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);
