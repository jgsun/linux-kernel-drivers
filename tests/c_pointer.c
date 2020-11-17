#include <linux/module.h>
#include <linux/device.h>
#include <linux/slab.h>

#define DRIVER_VERSION			"0.1"

typedef struct {
	u32 *irs; /* interrupt status register offset */
	u32 *ire; /* interrupt enable register offset */
	u32 *ina; /* interrupt non-ackable bits */
} c_pointer_t;

c_pointer_t cp;

static int c_pointer_init(void)
{
	int i, j;
	int count = 3;

	struct of_get_u32_from_dts {
		char *name;
		u32 **value;
		bool mandatory;
	} u32_prop[] = {
		{ "irs", &cp.irs, true },
		{ "ire", &cp.ire, false },
		{ "ina", &cp.ina, false }
	};
	printk(KERN_INFO "%s: driver init\n", __func__);
	for (i = 0; i < ARRAY_SIZE(u32_prop); i++) {
		//u32 *value = kcalloc(count, sizeof(u32), GFP_KERNEL);
		//*u32_prop[i].value = value;
		*u32_prop[i].value = kcalloc(count, sizeof(u32), GFP_KERNEL);

		printk("u32_prop[%d].value@%px\n", i, u32_prop[i].value);
		printk("*u32_prop[%d].value@%px\n", i, *u32_prop[i].value);

		for (j = 0; j < count; j++)
			//value[j] = 0xaa; //OK
			*(*u32_prop[i].value + j) = 0xaa; //OK
			//*u32_prop[i].value[j] = 0xaa; //NOK
	}

	for (j = 0; j < count; j++) {
		printk("irs[%d]@0x%px\n", j, &cp.irs[j]);
		printk("irs[%d]=0x%x\n", j, cp.irs[j]);
	}	

	return 0;
}

static void c_pointer_exit(void)
{
	printk(KERN_INFO "c_pointer: module exit version %s\n", DRIVER_VERSION);
}

module_init(c_pointer_init);
MODULE_AUTHOR("Jianguo Sun <jianguo_sun@hotmail.com>");
module_exit(c_pointer_exit);
MODULE_LICENSE("GPL");
