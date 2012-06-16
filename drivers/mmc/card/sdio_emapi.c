#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>

#include <asm/mach-types.h>

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/proc_fs.h>

#include <linux/delay.h>

#include <linux/miscdevice.h>
/*#include <linux/cdev.h>*/
#include <asm/uaccess.h>

/* sdio function */
#include <linux/mmc/core.h>
#include <linux/mmc/card.h>
#include <linux/mmc/sdio_func.h>
#include <linux/mmc/sdio_ids.h>
#include <linux/mmc/debug.h>

#include "sdio_emapi.h"

#include <linux/platform_device.h>

#define DEV_NAME "emapi"

#if 1 /*HTC_CSP_START*/
#define MAX_NVS_SIZE	0x800U
#endif /*HTC_CSP_END*/

struct _emapi {
	struct class		*emapi_class;
	struct device		*device;
	dev_t			emapi_cdevno;
	/*struct cdev		emapi_cdev;*/
	struct proc_dir_entry	*calibration;
	int			sdio_status;
};
static struct _emapi emapi;


/* sdio driver */
static const struct sdio_device_id emapi_sdio_ids[] = {
	{	SDIO_DEVICE_CLASS(SDIO_CLASS_WLAN)	},
	{					},
};

MODULE_DEVICE_TABLE(sdio, emapi_sdio_ids);


/* calibration read func */
static int emapi_calibration_read(char *page, char **start, off_t off,
			  int count, int *eof, void *data)
{
	unsigned char *nvs;
	unsigned long len;

	nvs = get_wifi_nvs_ram();
	if (nvs) {
		/* max size is 2048, we read max size by default*/
	len = 2048;
		memcpy((void *)page, (void *) nvs, len);
		LOGI("%s - read %d bytes\n", __func__, (unsigned int)len);
		return len;
	} else {
		LOGI("%s - no calibration data\n", __func__);
		return -EIO;
	}

	return 0;
}

static int emapi_calibration_write(struct file *file, const char __user *buffer,
			   unsigned long count, void *data)
{
	LOGI("%s do nothing\n", __func__);
	return 0;
}

/* most content happens here */
static int emapi_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
/*Raymond- static int emapi_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg)*/
{
	int error = 0;
	int retval = 0;
	struct sdio_request req;

	if (_IOC_TYPE(cmd) != MAGIC)
		return -ENOTTY;
	if (_IOC_NR(cmd) > EMAPI_IOC_MAXNR)
		return -ENOTTY;

	if (_IOC_DIR(cmd) & _IOC_READ)
		error = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	if (_IOC_DIR(cmd) & _IOC_WRITE)
		error = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	if (error)
		return -ENOTTY;

	if ((cmd != EMAPI_IOC_OPEN) && (cmd != EMAPI_IOC_CLOSE)) {
		if (copy_from_user(&req, (void __user *) arg, sizeof(struct sdio_request)))
			return -EFAULT;
	}

	switch (cmd) {
	case EMAPI_IOC_OPEN:
		LOGI("sdio open\n");
		break;
	case EMAPI_IOC_CLOSE:
		LOGI("sdio close\n");
		break;
	case EMAPI_IOC_READ:
		/* LOGI("sdio read, addr:%x, len:%d\n", req.addr, req.len); */
		retval = emapi_sdio_read(req.buf, req.addr, req.len);
		break;
	case EMAPI_IOC_WRITE:
		/* LOGI("sdio write, addr:%x, len:%d\n", req.addr, req.len); */
		retval = emapi_sdio_write(req.buf, req.addr, req.len);
		break;
	default:
		LOGI("emapi: unknown command\n");
		return -ENOTTY;
	};

	return retval;
}

static int emapi_open(struct inode *inode, struct file *filp)
{
	LOGI("emapi is open\n");
	return nonseekable_open(inode, filp);
}

static int emapi_release(struct inode *inode, struct file *filp)
{
	LOGI("emapi is close\n");
	return 0;
}

static struct file_operations emapi_fops = {
	.owner		= THIS_MODULE,
	.unlocked_ioctl	= emapi_ioctl,
/*Raymond-	.ioctl		= emapi_ioctl,*/
	.open		= emapi_open,
	.release	= emapi_release,
};

static struct miscdevice emapi_device = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "emapi",
	.fops	= &emapi_fops,
};

static int emapi_init_module(void)
{
	int rc;

#if 1
	rc = misc_register(&emapi_device);
	if (rc) {
		LOGE("emapi: register fail\n");
		return rc;
	}
	emapi.calibration = create_proc_entry("emapi_calibration", 0644, NULL);
	if (!emapi.calibration) {
		LOGE("emapi: alloc calibration error\n");
		rc = PTR_ERR(emapi.calibration);
		if (misc_deregister(&emapi_device))
			LOGE("emapi: deregister fail\n");
		return rc;
	}

	emapi.calibration->read_proc = emapi_calibration_read;
	emapi.calibration->write_proc = emapi_calibration_write;
	emapi.calibration->size = WIFI_NVS_MAX_SIZE;
	emapi.sdio_status = 0;
	return 0;
#else
	/* create device node */
	emapi.emapi_class = class_create(THIS_MODULE, DEV_NAME);
	if (IS_ERR(emapi.emapi_class)) {
		rc = PTR_ERR(emapi.emapi_class);
		emapi.emapi_class = NULL;
		return rc;
	}

	/* register char drv */
	rc = alloc_chrdev_region(&emapi.emapi_cdevno, 0, 1, DEV_NAME);
	if (rc) {
		class_destroy(emapi.emapi_class);
		device_destroy(emapi.emapi_class, 0);
		emapi.device = NULL;
		emapi.emapi_class = NULL;
		return rc;
	}

	emapi.device = device_create(emapi.emapi_class, NULL, 0, "%s", "emapi");

	if (unlikely(IS_ERR(emapi.device))) {
		rc = PTR_ERR(emapi.device);
		emapi.device = NULL;
		class_destroy(emapi.emapi_class);
		emapi.emapi_class = NULL;
		return rc;
	}

	cdev_init(&emapi.emapi_cdev, &emapi_cdev_fops);
	emapi.emapi_cdev.owner = THIS_MODULE;
	cdev_add(&emapi.emapi_cdev, emapi.emapi_cdevno, 1);

	emapi.calibration = create_proc_entry("emapi_calibration", 0644, NULL);
	if (!emapi.calibration) {
		LOGE("emapi calibration proc error\n");
		goto init_err;
	}

	emapi.calibration->read_proc = emapi_calibration_read;
	emapi.calibration->write_proc = emapi_calibration_write;
#if 1 /*HTC_CSP_START*/
	emapi.calibration->size = WIFI_NVS_MAX_SIZE;
#endif /*HTC_CSP_END*/
	emapi.sdio_status = 0;

	return 0;
init_err1:
	remove_proc_entry("emapi_calibration", NULL);
init_err:
	class_destroy(emapi.emapi_class);
	device_destroy(emapi.emapi_class, 0);
	emapi.device = NULL;
	emapi.emapi_class = NULL;
	unregister_chrdev_region(emapi.emapi_cdevno, 1);
	return rc;
#endif
}

static void emapi_exit_module(void)
{
	/*if (emapi.sdio_status)
	sdio_unregister_driver(&emapi_sdio_drv);*/

#if 1
	if (misc_deregister(&emapi_device))
		LOGE("emapi: deregister fail\n");
#else
	class_destroy(emapi.emapi_class);
	device_destroy(emapi.emapi_class, 0);
	unregister_chrdev_region(emapi.emapi_cdevno, 1);
#endif
	remove_proc_entry("emapi_calibration", NULL);
}

module_init(emapi_init_module);
module_exit(emapi_exit_module);

MODULE_AUTHOR("bowgo_tsai@htc.com");
MODULE_LICENSE("GPL");
