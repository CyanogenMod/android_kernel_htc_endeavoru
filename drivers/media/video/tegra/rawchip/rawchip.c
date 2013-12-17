/* Copyright (c) 2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <media/rawchip/rawchip.h>
#include <media/rawchip/Yushan_HTC_Functions.h>
#include <media/camera_mfg.h>

#ifdef YUSHAN_API_DEBUG
#define CDBG(fmt, args...) pr_info(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif

#define MSM_RAWCHIP_NAME "rawchip"

static struct rawchip_id_info_t yushan_id_info = {
	.rawchip_id_reg_addr = 0x5c04,
	.rawchip_id = 0x02030200,
};

static struct rawchip_info_t rawchip_info = {
	.rawchip_id_info = &yushan_id_info,
};

static struct rawchip_ctrl *rawchipCtrl = NULL;

static struct class *rawchip_class;
static dev_t rawchip_devno;


int rawchip_intr0, rawchip_intr1;
atomic_t interrupt, interrupt2;
struct yushan_int_t yushan_int;



static irqreturn_t yushan_irq_handler(int irq, void *dev_id){

	unsigned long flags;


	//smp_mb();
	spin_lock_irqsave(&yushan_int.yushan_spin_lock,flags);
	//CDBG("[CAM] %s detect INT0, interrupt:%d \n",__func__, interrupt);
	//if (atomic_read(&start_counting))
	//atomic_set(&yushan_int.frame_count, 1);
	//smp_mb();
	atomic_set(&interrupt, 1);
	CDBG("[CAM] %s after detect INT0, interrupt:%d \n",__func__, atomic_read(&interrupt));
	//interrupt = 1;
	//Yushan_Interrupt_Manager_Pad0();
	//CDBG("[CAM] %s atomic_set\n",__func__);
	//Yushan_ClearInterruptEvent(1);
	wake_up(&yushan_int.yushan_wait);
	spin_unlock_irqrestore(&yushan_int.yushan_spin_lock,flags);

	return IRQ_HANDLED;
}

static irqreturn_t yushan_irq_handler2(int irq, void *dev_id){

	unsigned long flags;


	spin_lock_irqsave(&yushan_int.yushan_spin_lock,flags);
	atomic_set(&interrupt2, 1);
	CDBG("[CAM] %s after detect INT1, interrupt:%d \n", __func__, atomic_read(&interrupt2));
	wake_up(&yushan_int.yushan_wait);
	spin_unlock_irqrestore(&yushan_int.yushan_spin_lock,flags);

	return IRQ_HANDLED;
}

int rawchip_set_size(struct rawchip_sensor_data data, bool *clock_init_done)
{
	int rc = 0;
	struct tegra_camera_rawchip_info *pdata = rawchipCtrl->pdata;
	struct rawchip_sensor_init_data rawchip_init_data;
	Yushan_New_Context_Config_t sYushanNewContextConfig;
	Yushan_ImageChar_t	sImageChar_context;
	int bit_cnt = 1;
	static uint32_t pre_pixel_clk = 0;
	uint8_t orientation;
	CDBG("%s", __func__);

	if (data.mirror_flip == CAMERA_SENSOR_MIRROR_FLIP)
		orientation = YUSHAN_ORIENTATION_MIRROR_FLIP;
	else if (data.mirror_flip == CAMERA_SENSOR_MIRROR)
		orientation = YUSHAN_ORIENTATION_MIRROR;
	else if (data.mirror_flip == CAMERA_SENSOR_FLIP)
		orientation = YUSHAN_ORIENTATION_FLIP;
	else
		orientation = YUSHAN_ORIENTATION_NONE;

	if (rawchipCtrl->rawchip_init == 0 || pre_pixel_clk != data.pixel_clk) {
		pre_pixel_clk = data.pixel_clk;
		switch (data.datatype) {
		case CSI_RAW8:
			bit_cnt = 8;
			break;
		case CSI_RAW10:
			bit_cnt = 10;
			break;
		case CSI_RAW12:
			bit_cnt = 12;
			break;
		}
		rawchip_init_data.sensor_name = data.sensor_name;
		rawchip_init_data.spi_clk = pdata->rawchip_spi_freq;
		rawchip_init_data.ext_clk = pdata->rawchip_mclk_freq;
		rawchip_init_data.lane_cnt = data.lane_cnt;
		rawchip_init_data.orientation = orientation;
		rawchip_init_data.use_ext_1v2 = pdata->rawchip_use_ext_1v2();
		rawchip_init_data.bitrate = (data.pixel_clk * bit_cnt / data.lane_cnt) / 1000000;
		rawchip_init_data.width = data.width;
		rawchip_init_data.height = data.height;
		rawchip_init_data.blk_pixels = data.line_length_pclk - data.width;
		rawchip_init_data.blk_lines = data.frame_length_lines - data.height;
		rawchip_init_data.x_addr_start = data.x_addr_start;
		rawchip_init_data.y_addr_start = data.y_addr_start;
		rawchip_init_data.x_addr_end = data.x_addr_end;
		rawchip_init_data.y_addr_end = data.y_addr_end;
		rawchip_init_data.x_even_inc = data.x_even_inc;
		rawchip_init_data.x_odd_inc = data.x_odd_inc;
		rawchip_init_data.y_even_inc = data.y_even_inc;
		rawchip_init_data.y_odd_inc = data.y_odd_inc;
		rawchip_init_data.binning_rawchip = data.binning_rawchip;

		pr_info("[CAM] rawchip init spi_clk=%d ext_clk=%d lane_cnt=%d bitrate=%d %d %d %d %d\n",
			rawchip_init_data.spi_clk, rawchip_init_data.ext_clk,
			rawchip_init_data.lane_cnt, rawchip_init_data.bitrate,
			rawchip_init_data.width, rawchip_init_data.height,
			rawchip_init_data.blk_pixels, rawchip_init_data.blk_lines);
		if (rawchipCtrl->rawchip_init) {
			//rc = gpio_request(pdata->rawchip_reset, "rawchip");
			if (rc < 0) {
				pr_err("GPIO(%d) request failed\n", pdata->rawchip_reset);
				return rc;
			}
			gpio_direction_output(pdata->rawchip_reset, 0);
			mdelay(1);
			gpio_direction_output(pdata->rawchip_reset, 1);
			//gpio_free(pdata->rawchip_reset);
		}
		rawchip_init_data.use_rawchip = data.use_rawchip;
		rc = Yushan_sensor_open_init(rawchip_init_data, clock_init_done);
		if (rc >= 0)
			rawchipCtrl->rawchip_init = 1;
		return rc;
	}

	pr_info("[CAM] rawchip set size %d %d %d %d\n",
		data.width, data.height, data.line_length_pclk, data.frame_length_lines);

	sYushanNewContextConfig.uwActivePixels = data.width;
	sYushanNewContextConfig.uwLineBlank = data.line_length_pclk - data.width;
	sYushanNewContextConfig.uwActiveFrameLength = data.height;
	sYushanNewContextConfig.uwPixelFormat = 0x0A0A;

	sImageChar_context.bImageOrientation = orientation;
	sImageChar_context.uwXAddrStart = data.x_addr_start;
	sImageChar_context.uwYAddrStart = data.y_addr_start;
	sImageChar_context.uwXAddrEnd= data.x_addr_end;
	sImageChar_context.uwYAddrEnd= data.y_addr_end;
	sImageChar_context.uwXEvenInc = data.x_even_inc;
	sImageChar_context.uwXOddInc = data.x_odd_inc;
	sImageChar_context.uwYEvenInc = data.y_even_inc;
	sImageChar_context.uwYOddInc = data.y_odd_inc;
	sImageChar_context.bBinning = data.binning_rawchip;

	rc = Yushan_ContextUpdate_Wrapper(sYushanNewContextConfig, sImageChar_context);
	return rc ? 0: -1;
}

static int rawchip_get_interrupt(struct rawchip_ctrl *raw_dev, void __user *arg)
{
	int rc = 0;
	struct rawchip_stats_event_ctrl se;
	int timeout;
	uint8_t interrupt_type;
	uint8_t interrupt0_type = 0;
	uint8_t interrupt1_type = 0;

	CDBG("%s\n", __func__);

	if (copy_from_user(&se, arg,
			sizeof(struct rawchip_stats_event_ctrl))) {
		pr_err("[CAM] %s, ERR_COPY_FROM_USER\n", __func__);
		rc = -EFAULT;
		goto end;
	}

	timeout = (int)se.timeout_ms;

	CDBG("[CAM] %s: timeout %d\n", __func__, timeout);

	interrupt0_type = Yushan_parse_interrupt(INTERRUPT_PAD_0, rawchipCtrl->error_interrupt_times);
	interrupt1_type = Yushan_parse_interrupt(INTERRUPT_PAD_1, rawchipCtrl->error_interrupt_times);
	interrupt_type = interrupt0_type | interrupt1_type;
	if (interrupt_type & RAWCHIP_INT_TYPE_ERROR) {
		rawchipCtrl->total_error_interrupt_times++;
		if (rawchipCtrl->total_error_interrupt_times <= 10 || rawchipCtrl->total_error_interrupt_times % 1000 == 0) {
			Yushan_Status_Snapshot();
			//Yushan_dump_Dxo();
		}
	}
	se.type = 10;
	se.length = sizeof(interrupt_type);
	if (copy_to_user((void *)(se.data),
			&interrupt_type,
			se.length)) {
			pr_err("[CAM] %s, ERR_COPY_TO_USER 1\n", __func__);
		rc = -EFAULT;
		goto end;
	}

	if (copy_to_user((void *)arg, &se, sizeof(se))) {
		pr_err("[CAM] %s, ERR_COPY_TO_USER 2\n", __func__);
		rc = -EFAULT;
		goto end;
	}
end:
	return rc;
}

static int rawchip_get_af_status(struct rawchip_ctrl *raw_dev, void __user *arg)
{
	int rc = 0;
	struct rawchip_stats_event_ctrl se;
	int timeout;
	rawchip_af_stats af_stats;
	CDBG("%s\n", __func__);

	if (copy_from_user(&se, arg,
			sizeof(struct rawchip_stats_event_ctrl))) {
		pr_err("[CAM] %s, ERR_COPY_FROM_USER\n", __func__);
		rc = -EFAULT;
		goto end;
	}

	timeout = (int)se.timeout_ms;

	CDBG("[CAM] %s: timeout %d\n", __func__, timeout);
	rc = Yushan_get_AFSU(&af_stats);

	if (rc < 0) {
		pr_err("[CAM] %s, Yushan_get_AFSU failed\n", __func__);
		rc = -EFAULT;
		goto end;
	}
	se.type = 5;
	se.length = sizeof(af_stats);

	if (copy_to_user((void *)(se.data),
			&af_stats,
			se.length)) {
			pr_err("[CAM] %s, ERR_COPY_TO_USER 1\n", __func__);
		rc = -EFAULT;
		goto end;
	}

	if (copy_to_user((void *)arg, &se, sizeof(se))) {
		pr_err("[CAM] %s, ERR_COPY_TO_USER 2\n", __func__);
		rc = -EFAULT;
		goto end;
	}

end:
	return rc;
}

static int rawchip_update_aec_awb_params(struct rawchip_ctrl *raw_dev, void __user *arg)
{
	struct rawchip_stats_event_ctrl se;
	rawchip_update_aec_awb_params_t *update_aec_awb_params;

	CDBG("%s\n", __func__);
	if (copy_from_user(&se, arg,
			sizeof(struct rawchip_stats_event_ctrl))) {
		pr_err("[CAM] %s, ERR_COPY_FROM_USER\n", __func__);
		return -EFAULT;
	}

	update_aec_awb_params = kmalloc(se.length, GFP_ATOMIC);
	if (!update_aec_awb_params) {
		pr_err("[CAM] %s %d: kmalloc failed\n", __func__,
			__LINE__);
		return -ENOMEM;
	}
	if (copy_from_user(update_aec_awb_params,
		(void __user *)(se.data),
		se.length)) {
		pr_err("[CAM] %s %d: copy_from_user failed\n", __func__,
			__LINE__);
		kfree(update_aec_awb_params);
		return -EFAULT;
	}

	CDBG("%s gain=%d line=%d\n", __func__,
		update_aec_awb_params->aec_params.gain, update_aec_awb_params->aec_params.line);
	CDBG("%s rg_ratio=%d bg_ratio=%d\n", __func__,
		update_aec_awb_params->awb_params.rg_ratio, update_aec_awb_params->awb_params.bg_ratio);

	Yushan_Update_AEC_AWB_Params(update_aec_awb_params);

	kfree(update_aec_awb_params);
	return 0;
}

static int rawchip_update_af_params(struct rawchip_ctrl *raw_dev, void __user *arg)
{
	struct rawchip_stats_event_ctrl se;
	rawchip_update_af_params_t *update_af_params;

	CDBG("%s\n", __func__);
	if (copy_from_user(&se, arg,
			sizeof(struct rawchip_stats_event_ctrl))) {
		pr_err("[CAM] %s, ERR_COPY_FROM_USER\n", __func__);
		return -EFAULT;
	}

	update_af_params = kmalloc(se.length, GFP_ATOMIC);
	if (!update_af_params) {
		pr_err("[CAM] %s %d: kmalloc failed\n", __func__,
			__LINE__);
		return -ENOMEM;
	}
	if (copy_from_user(update_af_params,
		(void __user *)(se.data),
		se.length)) {
		pr_err("[CAM] %s %d: copy_from_user failed\n", __func__,
			__LINE__);
		kfree(update_af_params);
		return -EFAULT;
	}

	CDBG("active_number=%d\n", update_af_params->af_params.active_number);
	CDBG("sYushanAfRoi[0] %d %d %d %d\n",
		update_af_params->af_params.sYushanAfRoi[0].bXStart,
		update_af_params->af_params.sYushanAfRoi[0].bXEnd,
		update_af_params->af_params.sYushanAfRoi[0].bYStart,
		update_af_params->af_params.sYushanAfRoi[0].bYEnd);

	Yushan_Update_AF_Params(update_af_params);

	kfree(update_af_params);
	return 0;
}

static int rawchip_update_3a_params(struct rawchip_ctrl *raw_dev, void __user *arg)
{
	struct rawchip_stats_event_ctrl se;
	uint8_t *enable_newframe_ack;

	CDBG("%s\n", __func__);
	if (copy_from_user(&se, arg,
			sizeof(struct rawchip_stats_event_ctrl))) {
		pr_err("[CAM] %s, ERR_COPY_FROM_USER\n", __func__);
		return -EFAULT;
	}

	enable_newframe_ack = kmalloc(se.length, GFP_ATOMIC);
	if (!enable_newframe_ack) {
		pr_err("[CAM] %s %d: kmalloc failed\n", __func__,
			__LINE__);
		return -ENOMEM;
	}
	if (copy_from_user(enable_newframe_ack,
		(void __user *)(se.data),
		se.length)) {
		pr_err("[CAM] %s %d: copy_from_user failed\n", __func__,
			__LINE__);
		kfree(enable_newframe_ack);
		return -EFAULT;
	}

	CDBG("enable_newframe_ack=%d\n", *enable_newframe_ack);
	Yushan_Update_3A_Params(*enable_newframe_ack);
	CDBG("rawchip_update_3a_params done\n");

	kfree(enable_newframe_ack);
	return 0;
}



uint32_t rawchip_id;

int rawchip_match_id(void)
{
	int rc = 0;
	uint32_t chipid = 0;
	int retry_spi_cnt = 0, retry_readid_cnt = 0;
	pr_info("%s\n", __func__);

	for (retry_spi_cnt = 0, retry_readid_cnt = 0; (retry_spi_cnt < 3 && retry_readid_cnt < 3); ) {
		rc = SPI_Read(rawchip_info.rawchip_id_info->rawchip_id_reg_addr, 4, (uint8_t*)(&chipid));
		if (rc < 0) {
			pr_err("%s: read id failed\n", __func__);
			retry_spi_cnt++;
			pr_info("%s: retry: %d\n", __func__, retry_spi_cnt);
			mdelay(5);
			continue;
		} else
			rawchip_id = chipid;

		pr_info("rawchip id: 0x%x requested id: 0x%x\n", chipid, rawchip_info.rawchip_id_info->rawchip_id);
		if (chipid != rawchip_info.rawchip_id_info->rawchip_id) {
			pr_err("rawchip_match_id chip id does not match\n");
			retry_readid_cnt++;
			pr_info("%s: retry: %d\n", __func__, retry_readid_cnt);
			mdelay(5);
			rc = -ENODEV;
			continue;
		} else
			break;
	}

	return rc;
}

void rawchip_release(void)
{
	struct tegra_camera_rawchip_info *pdata = rawchipCtrl->pdata;

	pr_info("[CAM] %s\n", __func__);

	CDBG("[CAM] rawchip free irq");
	free_irq(TEGRA_GPIO_TO_IRQ(pdata->rawchip_intr0), 0);
	free_irq(TEGRA_GPIO_TO_IRQ(pdata->rawchip_intr1), 0);

}

int rawchip_open_init(void)
{
	int rc = 0;
	struct tegra_camera_rawchip_info *pdata = rawchipCtrl->pdata;
	int i;

	pr_info("[CAM] %s\n", __func__);

	//XXX Yushan_common_init();

	init_waitqueue_head(&yushan_int.yushan_wait);
	spin_lock_init(&yushan_int.yushan_spin_lock);
	atomic_set(&interrupt, 0);
	atomic_set(&interrupt2, 0);

	/*create irq*/
	rc = request_irq(TEGRA_GPIO_TO_IRQ(pdata->rawchip_intr0), yushan_irq_handler,
		IRQF_TRIGGER_RISING, "yushan_irq", 0);
	if (rc < 0) {
		pr_err("request irq intr0 failed\n");
		goto open_init_failed;
	}

	rc = request_irq(TEGRA_GPIO_TO_IRQ(pdata->rawchip_intr1), yushan_irq_handler2,
		IRQF_TRIGGER_RISING, "yushan_irq2", 0);
	if (rc < 0) {
		pr_err("request irq intr1 failed\n");
		free_irq(TEGRA_GPIO_TO_IRQ(pdata->rawchip_intr0), 0);
		goto open_init_failed;
	}

	rawchipCtrl->rawchip_init = 0;

	rawchipCtrl->total_error_interrupt_times = 0;
	for (i = 0; i < TOTAL_INTERRUPT_COUNT; i++)
	       rawchipCtrl->error_interrupt_times[i] = 0;
	rawchip_intr0 = pdata->rawchip_intr0;
	rawchip_intr1 = pdata->rawchip_intr1;

	return rc;

open_init_failed:
	pr_err("%s: rawchip_open_init failed\n", __func__);

	return rc;
}

/* read rawchip id and create kobject file for store it */




static int rawchip_fops_open(struct inode *inode, struct file *filp)
{
	struct rawchip_ctrl *raw_dev = container_of(inode->i_cdev,
		struct rawchip_ctrl, cdev);

	filp->private_data = raw_dev;

	return 0;
}

static unsigned int rawchip_fops_poll(struct file *filp,
	struct poll_table_struct *pll_table)
{
	int rc = 0;
	unsigned long flags;

	poll_wait(filp, &yushan_int.yushan_wait, pll_table);

	spin_lock_irqsave(&yushan_int.yushan_spin_lock, flags);
	if (atomic_read(&interrupt)) {
		atomic_set(&interrupt, 0);
		rc = POLLIN | POLLRDNORM;
	}
	if (atomic_read(&interrupt2)) {
		atomic_set(&interrupt2, 0);
		rc = POLLIN | POLLRDNORM;
	}
	spin_unlock_irqrestore(&yushan_int.yushan_spin_lock, flags);

	return rc;
}
bool_t block_iotcl = false;

void tegra_rawchip_block_iotcl(bool_t blocked)
{
	block_iotcl = blocked;
}
static long rawchip_fops_ioctl(struct file *filp, unsigned int cmd,
	unsigned long arg)
{
	int rc = 0;
	struct rawchip_ctrl *raw_dev = filp->private_data;
	void __user *argp = (void __user *)arg;

	if(block_iotcl)
	{
		pr_info("[CAM] block rawchip ioctl (%d)", _IOC_NR(cmd));
		return rc;
	}
	CDBG("%s:%d cmd = %d\n", __func__, __LINE__, _IOC_NR(cmd));
	mutex_lock(&raw_dev->raw_ioctl_lock);
	switch (cmd) {
	case RAWCHIP_IOCTL_GET_INT:
		CDBG("RAWCHIP_IOCTL_GET_INT\n");
		rawchip_get_interrupt(raw_dev, argp);
		break;
	case RAWCHIP_IOCTL_GET_AF_STATUS:
		CDBG("RAWCHIP_IOCTL_GET_AF_STATUS\n");
		rawchip_get_af_status(raw_dev, argp);
		break;
	case RAWCHIP_IOCTL_UPDATE_AEC_AWB:
		CDBG("RAWCHIP_IOCTL_UPDATE_AEC\n");
		rawchip_update_aec_awb_params(raw_dev, argp);
		break;
	case RAWCHIP_IOCTL_UPDATE_AF:
		CDBG("RAWCHIP_IOCTL_UPDATE_AF\n");
		rawchip_update_af_params(raw_dev, argp);
		break;
	case RAWCHIP_IOCTL_UPDATE_3A:
		CDBG("RAWCHIP_IOCTL_UPDATE_3A\n");
		rawchip_update_3a_params(raw_dev, argp);
		break;
	}
	mutex_unlock(&raw_dev->raw_ioctl_lock);
	return rc;
}

static  const struct  file_operations rawchip_fops = {
	.owner	  = THIS_MODULE,
	.open	   = rawchip_fops_open,
	.unlocked_ioctl = rawchip_fops_ioctl,
	.poll  = rawchip_fops_poll,
};

static int setup_rawchip_cdev(void)
{
	int rc = 0;
	struct device *dev;

	pr_info("%s\n", __func__);

	rc = rawchip_spi_init();
	if (rc < 0) {
		pr_err("%s: failed to register spi driver\n", __func__);
		goto fail;
	}


	rc = alloc_chrdev_region(&rawchip_devno, 0, 1, MSM_RAWCHIP_NAME);
	if (rc < 0) {
		pr_err("%s: failed to allocate chrdev\n", __func__);
		goto alloc_chrdev_region_failed;
	}

	if (!rawchip_class) {
		rawchip_class = class_create(THIS_MODULE, MSM_RAWCHIP_NAME);
		if (IS_ERR(rawchip_class)) {
			rc = PTR_ERR(rawchip_class);
			pr_err("%s: create device class failed\n",
				__func__);
			goto class_create_failed;
		}
	}

	dev = device_create(rawchip_class, NULL,
		MKDEV(MAJOR(rawchip_devno), MINOR(rawchip_devno)), NULL,
		"%s%d", MSM_RAWCHIP_NAME, 0);
	if (IS_ERR(dev)) {
		pr_err("%s: error creating device\n", __func__);
		rc = -ENODEV;
		goto device_create_failed;
	}

	cdev_init(&rawchipCtrl->cdev, &rawchip_fops);
	rawchipCtrl->cdev.owner = THIS_MODULE;
	rawchipCtrl->cdev.ops   =
		(const struct file_operations *) &rawchip_fops;
	rc = cdev_add(&rawchipCtrl->cdev, rawchip_devno, 1);
	if (rc < 0) {
		pr_err("%s: error adding cdev\n", __func__);
		rc = -ENODEV;
		goto cdev_add_failed;
	}
	printk(KERN_INFO "%s %s: success\n", __func__, MSM_RAWCHIP_NAME);

	return rc;

cdev_add_failed:
	device_destroy(rawchip_class, rawchip_devno);
device_create_failed:
	class_destroy(rawchip_class);
class_create_failed:
	unregister_chrdev_region(rawchip_devno, 1);
alloc_chrdev_region_failed:

fail:
	return rc;
}

static void rawchip_tear_down_cdev(void)
{
	cdev_del(&rawchipCtrl->cdev);
	device_destroy(rawchip_class, rawchip_devno);
	class_destroy(rawchip_class);
	unregister_chrdev_region(rawchip_devno, 1);
}

static int rawchip_driver_probe(struct platform_device *pdev)
{
	int rc = 0;
	pr_info("%s\n", __func__);

	rawchipCtrl = kzalloc(sizeof(struct rawchip_ctrl), GFP_ATOMIC);
	if (!rawchipCtrl) {
		pr_err("%s: could not allocate mem for rawchip_dev\n", __func__);
		return -ENOMEM;
	}

	rawchipCtrl->pdata = pdev->dev.platform_data;
	if (!rawchipCtrl->pdata) {
		pr_err("%s: rawchip platform_data is NULL\n", __func__);
		kfree(rawchipCtrl);
		return -EFAULT;
	}
	rc = setup_rawchip_cdev();
	if (rc < 0) {
		kfree(rawchipCtrl);
		return rc;
	}

	mutex_init(&rawchipCtrl->raw_ioctl_lock);
	
	MFG_READ_RAWCHIP_ID();

	return rc;
}

static int rawchip_driver_remove(struct platform_device *pdev)
{
	rawchip_tear_down_cdev();

	mutex_destroy(&rawchipCtrl->raw_ioctl_lock);

	kfree(rawchipCtrl);

	return 0;
}

static struct  platform_driver rawchip_driver = {
	.probe  = rawchip_driver_probe,
	.remove = rawchip_driver_remove,
	.driver = {
		.name = "rawchip",
		.owner = THIS_MODULE,
	},
};

static int __init rawchip_driver_init(void)
{
	int rc;
	rc = platform_driver_register(&rawchip_driver);
	return rc;
}

static void __exit rawchip_driver_exit(void)
{
	platform_driver_unregister(&rawchip_driver);
}

MODULE_DESCRIPTION("rawchip driver");
MODULE_VERSION("rawchip 0.1");

module_init(rawchip_driver_init);
module_exit(rawchip_driver_exit);
