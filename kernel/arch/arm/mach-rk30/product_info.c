#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <mach/product_info.h>

struct __product_info_proc {
	struct proc_dir_entry *sn;
	struct proc_dir_entry *hw_version;
	struct proc_dir_entry *user_info;
	struct proc_dir_entry *pcba;
	struct proc_dir_entry *curef;
	struct proc_dir_entry *voltage;
};
static struct __product_info_proc *product_info_proc;

extern char GetSNSectorInfo(char *pbuf);
static int get_sn(char *page, char **start, off_t off,
		int count, int *eof, void *data)
{
	int len;
	int ret;
	int i;
	char buf[512] = {0};

	ret = GetSNSectorInfo(buf);
	/* SN从第2个字节开始，总共30字节长，不足部分补0xd */
	i = 2;
	while (1) {
		if (i > 31)
			break;

		if (buf[i] == 0xd || buf[i] == '\0') {
			buf[i] = '\0';
			break;
		}
		i++;
	}
	len = sprintf(page, "%s\n", buf+2);

	return len;
}

static int get_curef(char *page, char **start, off_t off,
		int count, int *eof, void *data)
{
	int len;
	int ret;
	char buf[512] = {0};

	ret = GetSNSectorInfo(buf);

	len = sprintf(page, "%s\n", buf+54);

	return len;
}

static int get_hw_version(char *page, char **start, off_t off,
		int count, int *eof, void *data)
{
	int len;
	int ret;
	char buf[512] = {0};

	ret = GetSNSectorInfo(buf);

	/* 从第38个字节开始为硬件版本号，硬件版本号15字节+1个'\0' */
	len = sprintf(page, "%s\n", buf+38);

	return len;
}

battery_type get_battery_type(void)
{
	char buf[512] = {0};
	char *hw_version;

	GetSNSectorInfo(buf);
	hw_version = buf+38;
	printk(KERN_INFO "________hw_version=%s_________\n", buf+38);
	printk(KERN_INFO "________hw_version[0]=%c______\n", hw_version[0]);

	if (hw_version[0] == '1')
		return JN_BATTERY;
	else if (hw_version[0] == '2')
		return BYD_BATTERY;
	else
		return JN_BATTERY;
}
EXPORT_SYMBOL(get_battery_type);

static int get_voltage_test_result(char *page, char **start, off_t off,
		int count, int *eof, void *data)
{
	int len;
	int ret;
	char buf[512] = {0};
	char c;

	ret = GetSNSectorInfo(buf);
	c = buf[86];
	if (c != 1 && c != 2)
		c = 0;

	c += '0';
	len = sprintf(page, "%c\n", c);

	return len;
}

static int get_pcba(char *page, char **start, off_t off,
		int count, int *eof, void *data)
{
	int len;
	int ret;
	char buf[512] = {0};

	ret = GetSNSectorInfo(buf);

	len = sprintf(page, "%s\n", buf+87);

	return len;
}
static int store_user_info(struct file *file, const char __user *buffer,
				    unsigned long count, void *data)
{
	return count;
}

static int __init product_info_module_init(void)
{
	product_info_proc = kmalloc(sizeof(struct __product_info_proc), GFP_KERNEL);
	if (!product_info_proc)
		return -ENOMEM;

	product_info_proc->sn = create_proc_entry("get_sn", 0644, NULL);
	if (product_info_proc->sn) {
		product_info_proc->sn->read_proc = get_sn;
		product_info_proc->sn->write_proc = NULL;
	}

	product_info_proc->hw_version = create_proc_entry("get_hw_version", 0644, NULL);
	if (product_info_proc->hw_version) {
		product_info_proc->hw_version->read_proc = get_hw_version;
		product_info_proc->hw_version->write_proc = NULL;
	}

	product_info_proc->voltage = create_proc_entry("get_voltage_test_result", 0644, NULL);
	if (product_info_proc->voltage) {
		product_info_proc->voltage->read_proc = get_voltage_test_result;
		product_info_proc->voltage->write_proc = NULL;
	}

	product_info_proc->pcba = create_proc_entry("get_pcba", 0644, NULL);
	if (product_info_proc->pcba) {
		product_info_proc->pcba->read_proc = get_pcba;
		product_info_proc->pcba->write_proc = NULL;
	}

	product_info_proc->user_info = create_proc_entry("store_user_info", 0644, NULL);
	if (product_info_proc->user_info) {
		product_info_proc->user_info->read_proc = NULL;
		product_info_proc->user_info->write_proc = store_user_info;
	}

	product_info_proc->curef = create_proc_entry("get_curef", 0644, NULL);
	if (product_info_proc->curef) {
		product_info_proc->curef->read_proc = get_curef;
		product_info_proc->curef->write_proc = NULL;
	}
	return 0;
}

static void __exit product_info_module_exit(void)
{
	if (product_info_proc) {
		remove_proc_entry("get_sn", NULL);
		remove_proc_entry("get_hw_version", NULL);
		remove_proc_entry("get_voltage_test_result", NULL);
		remove_proc_entry("get_pcba", NULL);
		remove_proc_entry("store_user_info", NULL);
		remove_proc_entry("get_curef", NULL);

		kfree(product_info_proc);
		product_info_proc = NULL;
	}
}

module_init(product_info_module_init);
module_exit(product_info_module_exit);
