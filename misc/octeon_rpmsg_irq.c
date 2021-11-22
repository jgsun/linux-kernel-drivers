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
#include <linux/err.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/printk.h>
#include <linux/slab.h>
#include <asm/octeon/octeon.h>
#include <linux/version.h>
#include "kernel_compat.h"
#include <linux/rpmsg.h>
#include "rpmsg_interface.h"

#define DRIVER_VERSION      "0.0.1"

static int debug = 0;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Enable debug logging");

#define DBG(fmt,...) \
do { \
        if(debug) \
                pr_info("[%s(%d)] " fmt, __func__, \
			 __LINE__, ##__VA_ARGS__); \
} while(0)

struct octeon_rpmsg_irq_info {
	struct kobject kobj;
	unsigned char descpu;
	unsigned char rx_irq_pos;
	unsigned char tx_irq_pos;
	unsigned long int rx_irq_count;
	unsigned long int tx_irq_count;
};

static struct rpmsg_data *octeon_rpmsg_data;

static void octeon_rpmsg_irq_handler(void)
{
	struct rpmsg_channel_data *rc_info = NULL;
	struct rpmsg_data *rd_info = octeon_rpmsg_data;

	list_for_each_entry(rc_info, &rd_info->delay_work_list, list_node)
		if (rc_info)
			schedule_delayed_work(rc_info->p_delay_work, 0);
}

int rpmsg_irq_trigger(unsigned int index, unsigned int channel_num)
{
	struct rpmsg_data *rd_info = octeon_rpmsg_data;
	struct octeon_rpmsg_irq_info *irq_info = NULL;
	struct rpmsg_channel_data *rc_info = NULL;
	unsigned int irq_index;

	if (!rd_info) {
		DBG("RPMSG work info %u does not init!\n", RPMSG_IRQ_TX);
		return 1;
	}

	rc_info = get_channel_data(rd_info, channel_num);
	if (!rc_info) {
		DBG("There is no delay work for channel %d\n", channel_num);
		return 1;
	}
	irq_info = (struct octeon_rpmsg_irq_info *)(rc_info->channel_irq_info);
	irq_index = 1 << irq_info->tx_irq_pos;

	DBG("%s-%d:descpu=%d, tx-irq-pos=0x%x\n", __func__,
		__LINE__, irq_info->descpu, irq_info->tx_irq_pos);
	DBG("%s-%d:irq_index=0x%x\n", __func__, __LINE__, irq_index);
	cvmx_write_csr(CVMX_CIU_MBOX_SETX(irq_info->descpu), irq_index);
	irq_info->tx_irq_count++;

	return 0;
}
EXPORT_SYMBOL_GPL(rpmsg_irq_trigger);

/*****************************************************************************/
ssize_t rpmsg_irq_show(struct kobject *kobj,
		       struct attribute *attr, char *buf)
{
	struct octeon_rpmsg_irq_info *irq_info =
		container_of(kobj, struct octeon_rpmsg_irq_info, kobj);
	int ret;

	if (strcmp(attr->name, "interrupts") == 0) {
		ret = sprintf(buf, "rx: %lu\ntx: %lu\n",
			      octeon_get_ipi_count(irq_info->rx_irq_pos),
			      irq_info->tx_irq_count);
	} else {
		ret = -EIO;
	}

	return ret;
}

/*****************************************************************************/
static struct attribute interrupts_attr = SYSFS_ATTR(interrupts, S_IRUGO);

static struct attribute *rpmsg_irq_attrs[] = {
	&interrupts_attr,
	NULL};

static struct sysfs_ops rpmsg_irq_sysfs_ops = {
	.show = rpmsg_irq_show,
};

static struct kobj_type rpmsg_irq_kobj_type = {
	.sysfs_ops = &rpmsg_irq_sysfs_ops,
	.default_attrs = rpmsg_irq_attrs,
};

void rpmsg_irq_setup(struct rpmsg_irq_data *irq_data)
{
	struct rpmsg_data *rd_info = octeon_rpmsg_data;
	struct octeon_rpmsg_irq_info *irq_info = NULL;
	struct rpmsg_channel_data *rc_info = NULL;
	int ret;
	int cvm_oct_enable_one_message;

	printk(KERN_DEBUG "%s: channel_num = %d, descpu = %d, dw = %p\n",
	       __func__, irq_data->channel_num,
	       irq_data->descpu, irq_data->pDwork);

	irq_info = kzalloc(sizeof(struct octeon_rpmsg_irq_info), GFP_KERNEL);
	if (!irq_info) {
		printk("No memory for octeon_rpmsg_irq_info.\n");
		return;
	}

	rc_info = kzalloc(sizeof(struct rpmsg_channel_data), GFP_KERNEL);
	if (!rc_info) {
		kfree(irq_info);
		printk("No memory for rpmsg_channel_data\n");
		return;
	}
	rc_info->channel_irq_info = irq_info;
	list_add(&rc_info->list_node, &rd_info->delay_work_list);

	rc_info->channel_num = irq_data->channel_num;
	if (irq_data->pDwork)
		rc_info->p_delay_work = irq_data->pDwork;

	irq_info->descpu = irq_data->descpu;
	irq_info->rx_irq_pos = irq_data->input_irq_num;
	irq_info->tx_irq_pos = irq_data->output_irq_num;

	ret = kobject_init_and_add(&irq_info->kobj, &rpmsg_irq_kobj_type,
			NULL, "rpmsg%d", rc_info->channel_num);
	if (ret)
		printk("cannot add kobject resource\n");

	cvm_oct_enable_one_message =
	octeon_request_ipi_handler((octeon_message_fn_t)octeon_rpmsg_irq_handler);
	printk(KERN_DEBUG "%s(%d)cvm_oct_enable_one_message=0x%x\n", __func__,
		__LINE__, cvm_oct_enable_one_message);
}
EXPORT_SYMBOL_GPL(rpmsg_irq_setup);

static int __init octeon_rpmsg_irq_init(void)
{
	struct rpmsg_data *rd_info;

	printk("%s\n", __func__);

	rd_info = kzalloc(sizeof(struct rpmsg_data), GFP_KERNEL);
	if (!rd_info) {
		printk("No memory for rpmsg_data\n");
		return -ENOMEM;
	}
	INIT_LIST_HEAD(&rd_info->delay_work_list);
	octeon_rpmsg_data = rd_info;

	return 0;
}

static void __exit octeon_rpmsg_irq_exit(void)
{
	if(octeon_rpmsg_data)
		kfree(octeon_rpmsg_data);
}

module_init(octeon_rpmsg_irq_init);
module_exit(octeon_rpmsg_irq_exit);
/*****************************************************************************/
MODULE_AUTHOR("Sun Jianguo <jianguo.sun@boxing.com>");
MODULE_DESCRIPTION("octeon rpmsg irq driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);
