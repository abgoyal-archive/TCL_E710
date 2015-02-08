#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/syscalls.h>
#include <asm/uaccess.h>
#include <linux/wakelock.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <mach/board.h>

#define SET_KERNEL_FS() \
    mm_segment_t old_fs; \
    do { \
        old_fs = get_fs(); \
        set_fs(KERNEL_DS); \
    } while (0)

#define SET_OLD_FS() do { \
    set_fs(old_fs); \
} while (0)

#define ACCESS_PWD "*#09#"
static char password[10];
static int password__read_proc(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	int len;
    
	len = sprintf(page, "%s\n", password);
	return len;
}

static int password__write_proc(struct file *file, const char __user *buffer, unsigned long count, void *data)
{	
	memset(password, 0, sizeof(password));
	
	if (copy_from_user(password, buffer, strlen(ACCESS_PWD)))
		return -EFAULT;
	
	return count;
}

static inline unsigned char have_access_password(void)
{
	return strcmp(password, ACCESS_PWD) == 0 ? 1 : 0;
}

#define BASE_DIR "/mnt/.factory/"

/* boot_flag */
static int boot_flag__read_proc(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	int len;
	int val;
	
	if (!have_access_password())
		return -EPERM;
	
	len = sprintf(page, "%d\n", android_booted());
	return len;
}

static int boot_flag__write_proc(struct file *file, const char __user *buffer, unsigned long count, void *data)
{
	if (!have_access_password())
		return -EPERM;
	
	return count;
}

/* pcba */
#define PCBA__FILE BASE_DIR "pcba__file"
#define MAX_LEN 512
static int pcba__read_proc(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	int len;
	long fd;
	char pcba[MAX_LEN + 1] = {0};
	
	if (!have_access_password())
		return -EPERM;
	
	{
		SET_KERNEL_FS();
		
		fd = sys_open(PCBA__FILE, O_RDONLY, 0);
		if (fd < 0) {
			len = sprintf(page, "\n");
			return len;
		}
		
		sys_read(fd, pcba, MAX_LEN);
		sys_close(fd);
		
		SET_OLD_FS();
	}
	
	len = sprintf(page, "%s\n", pcba);
	return len;
}

static int pcba__write_proc(struct file *file, const char __user *buffer, unsigned long count, void *data)
{
    long fd;
	
	if (!have_access_password())
		return -EPERM;

	{
		SET_KERNEL_FS();
		
		fd = sys_open(PCBA__FILE, O_CREAT | O_WRONLY | O_TRUNC, 0644);
		if (fd < 0) {
			pr_err("open %s failed\n", PCBA__FILE);
			return count;
		}
		
		/* trim the new line character */
		sys_write(fd, buffer, count - 1);
		sys_close(fd);
		
		SET_OLD_FS();
	}

	return count;
}

/* pt_result */
#define PT_RESULT__FILE BASE_DIR "pt_result__file"
static int pt_result__read_proc(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	int len;
	long fd;
	int result;
	
	if (!have_access_password())
		return -EPERM;
	
	{
		SET_KERNEL_FS();
		
		fd = sys_open(PT_RESULT__FILE, O_RDONLY, 0);
		if (fd < 0) {
			len = sprintf(page, "\n");
			return len;
		}
		
		sys_read(fd, (char *)&result, 4);
		sys_close(fd);
		
		SET_OLD_FS();
	}
	
	len = sprintf(page, "%d\n", result);
	return len;
}

static int pt_result__write_proc(struct file *file, const char __user *buffer, unsigned long count, void *data)
{
	long fd;
	int result;
	
	if (!have_access_password())
		return -EPERM;
	
	{
		SET_KERNEL_FS();
		
		fd = sys_open(PT_RESULT__FILE, O_CREAT | O_WRONLY, 0644);
		if (fd < 0) {
			pr_err("open %s failed\n", PT_RESULT__FILE);
			return count;
		}

		result = simple_strtol(buffer, NULL, 10);
		sys_write(fd, (char *)&result, 4);
		sys_close(fd);
		
		SET_OLD_FS();
	}

	return count;
}

/* wifi_result */
#define WIFI_RESULT__FILE BASE_DIR "wifi_result__file"
static int wifi_result__read_proc(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	int len;
	long fd;
	int result;
	
	if (!have_access_password())
		return -EPERM;
	
	{
		SET_KERNEL_FS();
		
		fd = sys_open(WIFI_RESULT__FILE, O_RDONLY, 0);
		if (fd < 0) {
			len = sprintf(page, "\n");
			return len;
		}
		
		sys_read(fd, (char *)&result, 4);
		sys_close(fd);
		
		SET_OLD_FS();
	}
	
	len = sprintf(page, "%d\n", result);
	return len;

}

static int wifi_result__write_proc(struct file *file, const char __user *buffer, unsigned long count, void *data)
{
	long fd;
	int result;
	
	if (!have_access_password())
		return -EPERM;
	
	{
		SET_KERNEL_FS();
		
		fd = sys_open(WIFI_RESULT__FILE, O_CREAT | O_WRONLY, 0644);
		if (fd < 0) {
			pr_err("open %s failed\n", WIFI_RESULT__FILE);
			return count;
		}

		result = simple_strtol(buffer, NULL, 10);
		sys_write(fd, (char *)&result, 4);
		sys_close(fd);
		
		SET_OLD_FS();
	}

	return count;
}

/* mmi_result */
#define MMI_RESULT__FILE BASE_DIR ".mmirst/test.txt"
static int mmi_result__read_proc(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	int len;
	long fd;
	int result = '0';
	
	if (!have_access_password())
		return -EPERM;
	
	{
		SET_KERNEL_FS();
		
		fd = sys_open(MMI_RESULT__FILE, O_RDONLY, 0);
		if (fd < 0) {
			len = sprintf(page, "0\n");
			return len;
		}
		
		sys_read(fd, (char *)&result, 1);
		sys_close(fd);
		
		SET_OLD_FS();
	}
    
	len = sprintf(page, "%c\n", result);
	return len;
}

static int mmi_result__write_proc(struct file *file, const char __user *buffer, unsigned long count, void *data)
{
	long fd;
	int result;

	return -EPERM;

	if (!have_access_password())
		return -EPERM;
	
	{
		SET_KERNEL_FS();
		
		fd = sys_open(MMI_RESULT__FILE, O_CREAT | O_WRONLY, 0644);
		if (fd < 0) {
			pr_err("open %s failed\n", MMI_RESULT__FILE);
			return count;
		}

		result = simple_strtol(buffer, NULL, 10);
		sys_write(fd, (char *)&result, 4);
		sys_close(fd);

		SET_OLD_FS();
	}
    
	return count;
}

static struct wake_lock smt_keep_wake;
static unsigned char smt_keep_wake_stat;
static int keep_wake__read_proc(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	int len;
	
	if (!have_access_password())
		return -EPERM;
    
	len = sprintf(page, "%d\n", smt_keep_wake_stat);
	return len;
}

static int keep_wake__write_proc(struct file *file, const char __user *buffer, unsigned long count, void *data)
{
	int val;
	
	if (!have_access_password())
		return -EPERM;
	
	val = simple_strtol(buffer, NULL, 10);
	if (val == 1 && smt_keep_wake_stat == 0) {
		pr_info("keep wake!!!\n");
		wake_lock(&smt_keep_wake);
		smt_keep_wake_stat = 1;
	} else if (val == 0 && smt_keep_wake_stat == 1) {
		pr_info("don't keep wake!\n");
		wake_unlock(&smt_keep_wake);
		smt_keep_wake_stat = 0;
	}
	
	return count;
}


static struct proc_dir_entry *smt;

static int __init smt_test_init(void)
{
	smt = proc_mkdir("smt", NULL);
	if (smt) {
		struct proc_dir_entry *boot_flag;
		struct proc_dir_entry *pcba;
		struct proc_dir_entry *pt_result;
		struct proc_dir_entry *wifi_result;
		struct proc_dir_entry *mmi_result;
		struct proc_dir_entry *keep_wake;
		struct proc_dir_entry *password;
		
		
		CREATE_PROC_ENTRY(boot_flag, smt);
		CREATE_PROC_ENTRY(pcba, smt);
		CREATE_PROC_ENTRY(pt_result, smt);
		CREATE_PROC_ENTRY(wifi_result, smt);
		CREATE_PROC_ENTRY(mmi_result, smt);

		wake_lock_init(&smt_keep_wake, WAKE_LOCK_SUSPEND, "smt_keep_wake");
		CREATE_PROC_ENTRY(keep_wake, smt);

		CREATE_PROC_ENTRY_PASSWORD(password, smt);

	}

	return 0;
}


static void __exit smt_test_exit(void)
{
	if (smt) {
		REMOVE_PROC_ENTRY(boot_flag, smt);
		REMOVE_PROC_ENTRY(pcba, smt);
		REMOVE_PROC_ENTRY(pt_result, smt);
		REMOVE_PROC_ENTRY(wifi_result, smt);
		REMOVE_PROC_ENTRY(mmi_result, smt);
		REMOVE_PROC_ENTRY(keep_wake, smt);
		REMOVE_PROC_ENTRY(password, smt);

		REMOVE_PROC_ENTRY(smt, NULL);
	}
}

MODULE_LICENSE("GPL");

module_init(smt_test_init);
module_exit(smt_test_exit);
