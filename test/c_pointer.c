#include <linux/module.h>
#include <linux/device.h>

#define VERSION			"0.3"

typedef struct {
	u32 *irs; /* interrupt status register offset */
	u32 *ire; /* interrupt enable register offset */
	u32 *ina; /* interrupt non-ackable bits */
} c_pointer_t;

static int c_pointer_init(void)
{
	int ret, i, j;
	printk(KERN_INFO "%s: driver init\n", __func__);
	int count = 3；

	struct of_get_u8_from_dts {
		char *name;
		u32 **value;
		bool mandatory;
	} u32_prop[] = {
		{ "irs", &cpld_ic->irs, true },
		{ "ire", &cpld_ic->ire, false },
		{ "ina", &cpld_ic->ina, false }
	};

	printk("value@%p\n", value)

	//for (i = 0; i < ARRAY_SIZE(u32_prop); i++) {
		printk("value@%p\n", u32_prop[0].value);

		*u32_prop[0]->value = kcalloc(dev, count, sizeof(u32), GFP_KERNEL);

		printk("*value@%p\n", *u32_prop[0].value);

		for (j = 0; j < count; j++) {
			*u32_prop[0]->value[i] = 0xaa;
			printk("irs[%d]@0x%x\n", i, &cpld_ic->irs[i]);
			printk("irs[%d]=0x%x\n", i, cpld_ic->irs[i]);
		}
	//}

	return 0;
}

static void c_pointer_exit(void)
{
	printk(KERN_INFO "c_pointer: module exit version %s\n", DRIVER_VERSION);
}

module_init(c_pointer_init);
module_exit(c_pointer_exit);