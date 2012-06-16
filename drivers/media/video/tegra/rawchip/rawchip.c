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

#ifdef YUSHAN_API_DEBUG
#define CDBG(fmt, args...) pr_info(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif

#define MSM_RAWCHIP_NAME "rawchip"

struct tegra_rawchip_device {
	struct platform_device *pdev;
	struct resource        *mem;
	int                     irq;
	void                   *base;

	struct device *device;
	struct cdev   cdev;
	struct mutex  lock;
	char	  open_count;
	uint8_t       op_mode;
	wait_queue_head_t wait;
};

static struct rawchip_ctrl *rawchipCtrl;

static struct mutex raw_ioctl_lock;
static struct class *tegra_rawchip_class;
static dev_t tegra_rawchip_devno;
static struct tegra_rawchip_device *tegra_rawchip_device_p;

struct platform_device *yushan_pdev;

int rawchip_intr0, rawchip_intr1;
atomic_t interrupt, interrupt2;
struct yushan_int_t yushan_int;
struct yushan_int_t {
	spinlock_t yushan_spin_lock;
	wait_queue_head_t yushan_wait;
};

int rawchip_init;

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
	//Yushan_ISR();
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

static int yushan_create_irq(void)
{
	struct tegra_camera_rawchip_info *pdata = rawchipCtrl->pdata;

	CDBG("[CAM] yushan_create_irq");
	//udwListOfInterrupts	= (uint32_t *) kmalloc(96, GFP_KERNEL);
	return request_irq(TEGRA_GPIO_TO_IRQ(pdata->rawchip_intr0), yushan_irq_handler, IRQF_TRIGGER_RISING, "yushan_irq", 0);
}

static int yushan_create_irq2(void)
{
	struct tegra_camera_rawchip_info *pdata = rawchipCtrl->pdata;

	CDBG("[CAM]yushan_create_irq2");
	return  request_irq(TEGRA_GPIO_TO_IRQ(pdata->rawchip_intr1), yushan_irq_handler2, IRQF_TRIGGER_RISING, "yushan_irq2", 0);
}


int rawchip_set_size(struct rawchip_sensor_data data)
{
	struct tegra_camera_rawchip_info *sdata = yushan_pdev->dev.platform_data;
	struct rawchip_sensor_init_data rawchip_init_data;
	Yushan_New_Context_Config_t sYushanNewContextConfig;
	struct timespec ts_start, ts_end;
	int bit_cnt = 1;
	static uint32_t pre_pixel_clk = 0;
	uint8_t orientation;
	pr_info("%s", __func__);

	if (data.mirror_flip == CAMERA_SENSOR_MIRROR_FLIP)
		orientation = YUSHAN_ORIENTATION_MIRROR_FLIP;
	else if (data.mirror_flip == CAMERA_SENSOR_MIRROR)
		orientation = YUSHAN_ORIENTATION_MIRROR;
	else if (data.mirror_flip == CAMERA_SENSOR_FLIP)
		orientation = YUSHAN_ORIENTATION_FLIP;
	else
		orientation = YUSHAN_ORIENTATION_NONE;

	if (rawchip_init == 0 || pre_pixel_clk != data.pixel_clk) {
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
		rawchip_init_data.spi_clk = sdata->rawchip_spi_freq;
		rawchip_init_data.ext_clk = sdata->rawchip_mclk_freq;
		rawchip_init_data.lane_cnt = data.lane_cnt;
		rawchip_init_data.orientation = orientation;
		rawchip_init_data.use_ext_1v2 = sdata->rawchip_use_ext_1v2();
		rawchip_init_data.bitrate = (data.pixel_clk * bit_cnt / data.lane_cnt) / 1000000;
		rawchip_init_data.width = data.fullsize_width;
		rawchip_init_data.height = data.fullsize_height;
		rawchip_init_data.blk_pixels = data.fullsize_line_length_pclk - data.fullsize_width;
		rawchip_init_data.blk_lines = data.fullsize_frame_length_lines - data.fullsize_height;

		pr_info("[CAM] rawchip init spi_clk=%d ext_clk=%d lane_cnt=%d bitrate=%d %d %d %d %d\n",
			rawchip_init_data.spi_clk, rawchip_init_data.ext_clk,
			rawchip_init_data.lane_cnt, rawchip_init_data.bitrate,
			rawchip_init_data.width, rawchip_init_data.height,
			rawchip_init_data.blk_pixels, rawchip_init_data.blk_lines);
		if (rawchip_init) {
			//XXX Yushan_common_deinit();
			//XXX Yushan_common_init();
			/*Reset_Yushan();*/
		}
		ktime_get_ts(&ts_start);
		Yushan_sensor_open_init(rawchip_init_data);
		ktime_get_ts(&ts_end);
		pr_info("%s: %ld ms\n", __func__,
			(ts_end.tv_sec-ts_start.tv_sec)*1000+(ts_end.tv_nsec-ts_start.tv_nsec)/1000000);
		mdelay(100);
		rawchip_init = 1;
	}


	pr_info("[CAM] rawchip set size %d %d %d %d\n",
		data.width, data.height, data.line_length_pclk, data.frame_length_lines);

	sYushanNewContextConfig.uwActivePixels = data.width;
	sYushanNewContextConfig.uwLineBlank = data.line_length_pclk - data.width;
	sYushanNewContextConfig.uwActiveFrameLength = data.height;
	sYushanNewContextConfig.bSelectStillVfMode = YUSHAN_FRAME_FORMAT_VF_MODE;
	sYushanNewContextConfig.uwPixelFormat = 0x0A0A;
	sYushanNewContextConfig.orientation = orientation;

	Yushan_ContextUpdate_Wrapper(&sYushanNewContextConfig);
	return 0;
}

static int rawchip_get_interrupt(struct tegra_rawchip_device *pgmn_dev, void __user *arg)
{
	int rc = 0;
	struct rawchip_stats_event_ctrl se;
	int timeout;
	uint8_t interrupt_type;

	CDBG("%s\n", __func__);

	if (copy_from_user(&se, arg,
			sizeof(struct rawchip_stats_event_ctrl))) {
		pr_err("[CAM] %s, ERR_COPY_FROM_USER\n", __func__);
		rc = -EFAULT;
		goto end;
	}

	timeout = (int)se.timeout_ms;

	CDBG("[CAM] %s: timeout %d\n", __func__, timeout);

	interrupt_type = Yushan_parse_interrupt();
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

static int rawchip_get_af_status(struct tegra_rawchip_device *pgmn_dev, void __user *arg)
{
	int rc = 0;
	struct rawchip_stats_event_ctrl se;
	int timeout;
	uint32_t pAfStatsGreen[20];

	CDBG("%s\n", __func__);

	if (copy_from_user(&se, arg,
			sizeof(struct rawchip_stats_event_ctrl))) {
		pr_err("[CAM] %s, ERR_COPY_FROM_USER\n", __func__);
		rc = -EFAULT;
		goto end;
	}

	timeout = (int)se.timeout_ms;

	CDBG("[CAM] %s: timeout %d\n", __func__, timeout);

	rc = Yushan_get_AFSU(pAfStatsGreen);
	if (rc < 0) {
		pr_err("[CAM] %s, Yushan_get_AFSU failed\n", __func__);
		rc = -EFAULT;
		goto end;
	}
	se.type = 5;
	se.length = sizeof(pAfStatsGreen);

	if (copy_to_user((void *)(se.data),
			pAfStatsGreen,
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

static int rawchip_update_aec_awb_params(struct tegra_rawchip_device *pgmn_dev, void __user *arg)
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

static int rawchip_update_af_params(struct tegra_rawchip_device *pgmn_dev, void __user *arg)
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

static int rawchip_update_3a_params(struct tegra_rawchip_device *pgmn_dev, void __user *arg)
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

void rawchip_release(void)
{
	struct tegra_camera_rawchip_info *pdata = rawchipCtrl->pdata;

	pr_info("[CAM] %s\n", __func__);

	//XXX Yushan_common_deinit();

	CDBG("[CAM] rawchip free irq");
	free_irq(TEGRA_GPIO_TO_IRQ(pdata->rawchip_intr0), 0);
	free_irq(TEGRA_GPIO_TO_IRQ(pdata->rawchip_intr1), 0);
}

int rawchip_open_init(void)
{
	int32_t rc = 0;
	struct tegra_camera_rawchip_info *pdata = rawchipCtrl->pdata;

	pr_info("[CAM] %s\n", __func__);

	//XXX Yushan_common_init();

	init_waitqueue_head(&yushan_int.yushan_wait);
	spin_lock_init(&yushan_int.yushan_spin_lock);
	atomic_set(&interrupt, 0);
	atomic_set(&interrupt2, 0);

	/*create irq*/
	rc = yushan_create_irq();
	if (rc < 0)
		pr_err("[CAM] request GPIO irq error");

	rc = yushan_create_irq2();
	if (rc < 0)
		pr_err("[CAM] request GPIO irq 2 error");

	rawchip_init = 0;
	rawchip_intr0 = pdata->rawchip_intr0;
	rawchip_intr1 = pdata->rawchip_intr1;

	return 0;
}

#if 0
int rawchip_vreg_enable(void)
{
	struct tegra_camera_rawchip_info *sdata = yushan_pdev->dev.platform_data;
	int rc;
	pr_info("[CAM]%s camera vreg on\n", __func__);

	if (tegra_rawchip_device_p == NULL) {
		pr_err("already failed in __tegra_rawchip_probe\n");
		return -EINVAL;
	}

	if (sdata->camera_rawchip_power_on == NULL) {
		pr_err("[CAM]sensor platform_data didnt register\n");
		return -EIO;
	}
	rc = sdata->camera_rawchip_power_on();
	return rc;
}

int Yushan_common_init(void)
{
	int32_t rc = 0;
	uint32_t check_yushan_id;
	struct msm_camera_rawchip_info *sdata = yushan_pdev->dev.platform_data;

	pr_info("%s\n", __func__);

	//afsu_info.active_number = 0;

	/*RESET*/
	pr_info("request RAW_RSTN\n");
	rc = gpio_request(sdata->rawchip_reset, "yushan");
	if (!rc) {
		gpio_direction_output(sdata->rawchip_reset, 1);
	} else {
		pr_err("GPIO (%d) request failed\n", sdata->rawchip_reset);
		goto init_fail;
	}
	gpio_free(sdata->rawchip_reset);

	yushan_spi_write(0x0008, 0x7f);

	//mdelay(10);
	msleep(1);

	SPI_Read(0x5c04, 4, (uint8_t*)(&check_yushan_id));
	pr_info("[CAM]Yushan power on id: 0x%x \n", check_yushan_id);

	if (check_yushan_id == 0x02030200)
	{
		pr_info("[CAM]Yushan read ID correct\n");
		goto init_done;
	}
	init_fail:
		pr_err("[CAM]Yushan power on failed\n");
	init_done:
		return rc;
}

int rawchip_vreg_disable(void)
{
	struct tegra_camera_rawchip_info *sdata = yushan_pdev->dev.platform_data;
	int rc;
	pr_info("%s camera vreg off\n", __func__);

	if (sdata->camera_rawchip_power_off == NULL) {
		pr_err("sensor platform_data didnt register\n");
		return -EIO;
	}
	rc = sdata->camera_rawchip_power_off();
	return rc;
}

int Yushan_common_deinit(void)
{
	int32_t rc = 0;
	struct tegra_camera_rawchip_info *sdata = yushan_pdev->dev.platform_data;

	pr_info("%s\n", __func__);
	/*FPGA's RESET to low*/
	rc = gpio_request(sdata->rawchip_reset, "yushan");
	if (!rc) {
		gpio_direction_output(sdata->rawchip_reset, 0);
	} else {
		pr_err("GPIO (%d) request failed\n", sdata->rawchip_reset);
		goto init_fail;
	}
	gpio_free(sdata->rawchip_reset);

	msleep(1);
	//rawchip_vreg_disable(yushan_pdev);
	goto init_done;

	init_fail:
		pr_err("Yushan power off failed\n");
	init_done:
		return rc;
}
#endif

static int tegra_rawchip_open(struct inode *inode, struct file *filp)
{
	int rc;

	struct tegra_rawchip_device *pgmn_dev = container_of(inode->i_cdev,
		struct tegra_rawchip_device, cdev);
	filp->private_data = pgmn_dev;

	pr_info("%s:%d]\n", __func__, __LINE__);

	rc = 0;//Yushan_sensor_open_init();//__tegra_rawchip_open(pgmn_dev);

	//pr_err(KERN_INFO "%s:%d] %s open_count = %d\n", __func__, __LINE__,
	//	filp->f_path.dentry->d_name.name, pgmn_dev->open_count);

	return rc;
}

#if 0
static int tegra_rawchip_release(struct inode *inode, struct file *filp)
{
	int rc;

	struct tegra_rawchip_device *pgmn_dev = filp->private_data;

	pr_err(KERN_INFO "%s:%d]\n", __func__, __LINE__);

	rc = __tegra_rawchip_release(pgmn_dev);

	pr_err(KERN_INFO "%s:%d] %s open_count = %d\n", __func__, __LINE__,
		filp->f_path.dentry->d_name.name, pgmn_dev->open_count);
	return rc;
}
#endif

static unsigned int tegra_rawchip_poll(struct file *filp,
	struct poll_table_struct *pll_table)
{
	int rc = 0;
	unsigned long flags;

	poll_wait(filp, &yushan_int.yushan_wait, pll_table);

	spin_lock_irqsave(&yushan_int.yushan_spin_lock, flags);
	if (atomic_read(&interrupt) || atomic_read(&interrupt2)) {
		atomic_set(&interrupt, 0);
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
static long tegra_rawchip_ioctl(struct file *filp, unsigned int cmd,
	unsigned long arg)
{
	int rc = 0;
	struct tegra_rawchip_device *pgmn_dev = filp->private_data;
	void __user *argp = (void __user *)arg;

	if(block_iotcl)
	{
		pr_info("[CAM] block rawchip ioctl (%d)", _IOC_NR(cmd));
		return rc;
	}
	CDBG("%s:%d cmd = %d\n", __func__, __LINE__, _IOC_NR(cmd));
	mutex_lock(&raw_ioctl_lock);
	switch (cmd) {
	case RAWCHIP_IOCTL_GET_INT:
		CDBG("RAWCHIP_IOCTL_GET_INT\n");
		rawchip_get_interrupt(pgmn_dev, argp);
		break;
	case RAWCHIP_IOCTL_GET_AF_STATUS:
		CDBG("RAWCHIP_IOCTL_GET_AF_STATUS\n");
		rawchip_get_af_status(pgmn_dev, argp);
		break;
	case RAWCHIP_IOCTL_UPDATE_AEC_AWB:
		CDBG("RAWCHIP_IOCTL_UPDATE_AEC\n");
		rawchip_update_aec_awb_params(pgmn_dev, argp);
		break;
	case RAWCHIP_IOCTL_UPDATE_AF:
		CDBG("RAWCHIP_IOCTL_UPDATE_AF\n");
		rawchip_update_af_params(pgmn_dev, argp);
		break;
	case RAWCHIP_IOCTL_UPDATE_3A:
		CDBG("RAWCHIP_IOCTL_UPDATE_3A\n");
		rawchip_update_3a_params(pgmn_dev, argp);
		break;
	}
	mutex_unlock(&raw_ioctl_lock);
	return rc;
}

#if 0
static long tegra_rawchip_ioctl(struct file *filp, unsigned int cmd,
	unsigned long arg)
{
	int rc;
	void __user *argp = (void __user *)arg;
	//struct tegra_rawchip_device *pgmn_dev = filp->private_data;

	//pr_err("%s:%d cmd = %d\n", __func__, __LINE__, _IOC_NR(cmd));

	//rc = __tegra_rawchip_ioctl(pgmn_dev, cmd, arg);
	switch (cmd) {
		case MSM_CAM_IOCTL_SET_AFSU: {
			if (copy_from_user(&afsu_info, argp, sizeof(struct tegra_AFSU_info))) {
				pr_info("<ChenC>copy_from_user error\n");
				rc = -EFAULT;
			}
			pr_info("<ChenC>tegra_rawchip_ioctl :AFSU:active number:%d, xstart:%d %d %d %d \n", afsu_info.active_number, afsu_info.sYushanAfRoi[0].bXStart, afsu_info.sYushanAfRoi[0].bYStart, afsu_info.sYushanAfRoi[0].bXEnd, afsu_info.sYushanAfRoi[0].bYEnd); 		
			if(afsu_info.active_number == 0)
			{
				pr_info("<ChenC>stop AFSU\n");
				//break;
			}
			Yushan_Dxo_Dop_Af_Run((Yushan_AF_ROI_t*)&afsu_info.sYushanAfRoi, (uint32_t *)&pAfStatsGreen, afsu_info.active_number);
			AF_STAT_INT = 0;
			break;
		}
		case MSM_CAM_IOCTL_GET_AFSU_DATA: {
			uint8_t		bStatus = SUCCESS;

			bStatus = WaitForInterruptEvent2(EVENT_DXODOP7_NEWFRAMEPROC_ACK, TIME_50MS);
			if(bStatus)
			{
				Yushan_Read_AF_Statistics(pAfStatsGreen);
				pr_info("<ChenC>kernal:GET_AFSU:G:%d, R:%d, B:%d, confi:%d\n",  pAfStatsGreen[0], pAfStatsGreen[1], pAfStatsGreen[2], pAfStatsGreen[3]);  		
					if (copy_to_user((void *)argp,
						pAfStatsGreen,
						sizeof(uint32_t)*20))
					{
						pr_info("<ChenC>copy_from_user error\n");
						rc = -EFAULT;
					}
			}
			else
				pr_info("<ChenC>kernal:%s:Get AFSU statistic data fail\n", __func__);
			break;
		}
		case MSM_CAM_IOCTL_RAWCHIP_ISR: {
				Yushan_ISR();
				//pr_info("<ChenC>kernal:%s:MSM_CAM_IOCTL_RAWCHIP_ISR\n", __func__);
			break;
		}
	}
	//pr_err("%s:%d\n", __func__, __LINE__);
	return rc;
}
#endif

static  const struct  file_operations tegra_rawchip_fops = {
	.owner	  = THIS_MODULE,
	.open	   = tegra_rawchip_open,
//	.release	= tegra_rawchip_release,
	.unlocked_ioctl = tegra_rawchip_ioctl,
	.poll  = tegra_rawchip_poll,
};

int setup_rawchip_dev(int node, char *device_name)
{
	return 0;
}

struct tegra_rawchip_device *__tegra_rawchip_init(struct platform_device *pdev)
{
	struct tegra_rawchip_device *pgmn_dev;

	pgmn_dev = kzalloc(sizeof(struct tegra_rawchip_device), GFP_ATOMIC);
	if (!pgmn_dev) {
		pr_err("%s:%d]no mem\n", __func__, __LINE__);
		return NULL;
	}

	mutex_init(&pgmn_dev->lock);
       mutex_init(&raw_ioctl_lock);
	pgmn_dev->pdev = pdev;

	init_waitqueue_head(&pgmn_dev->wait);

	return pgmn_dev;
}

int __tegra_rawchip_exit(struct tegra_rawchip_device *pgmn_dev)
{
	pr_info("%s:%d]\n", __func__, __LINE__);
	mutex_destroy(&raw_ioctl_lock);
	mutex_destroy(&pgmn_dev->lock);
	kfree(pgmn_dev);
	return 0;
}

static int tegra_rawchip_init(struct platform_device *pdev)
{
	int rc = -1;
	struct device *dev;

	pr_info("%s:%d]\n", __func__, __LINE__);

	rc = rawchip_spi_init();
	if (rc < 0) {
		pr_err("%s: failed to register spi driver\n", __func__);
		goto fail;
	}

	tegra_rawchip_device_p = __tegra_rawchip_init(pdev);
	if (tegra_rawchip_device_p == NULL) {
		pr_err("%s: initialization failed\n", __func__);
		goto fail;
	}

	rc = alloc_chrdev_region(&tegra_rawchip_devno, 0, 1, MSM_RAWCHIP_NAME);
	if (rc < 0) {
		pr_err("%s: failed to allocate chrdev\n", __func__);
		goto fail_1;
	}

	if (!tegra_rawchip_class) {
		tegra_rawchip_class = class_create(THIS_MODULE, MSM_RAWCHIP_NAME);
		if (IS_ERR(tegra_rawchip_class)) {
			rc = PTR_ERR(tegra_rawchip_class);
			pr_err("%s: create device class failed\n",
				__func__);
			goto fail_2;
		}
	}

	dev = device_create(tegra_rawchip_class, NULL,
		MKDEV(MAJOR(tegra_rawchip_devno), MINOR(tegra_rawchip_devno)), NULL,
		"%s%d", MSM_RAWCHIP_NAME, 0);
	if (IS_ERR(dev)) {
		pr_err("%s: error creating device\n", __func__);
		rc = -ENODEV;
		goto fail_3;
	}
	cdev_init(&tegra_rawchip_device_p->cdev, &tegra_rawchip_fops);
	tegra_rawchip_device_p->cdev.owner = THIS_MODULE;
	tegra_rawchip_device_p->cdev.ops   =
		(const struct file_operations *) &tegra_rawchip_fops;
	rc = cdev_add(&tegra_rawchip_device_p->cdev, tegra_rawchip_devno, 1);
	if (rc < 0) {
		pr_err("%s: error adding cdev\n", __func__);
		rc = -ENODEV;
		goto fail_4;
	}
	printk(KERN_INFO "%s %s: success\n", __func__, MSM_RAWCHIP_NAME);

      rawchip_open_init();

	return rc;

fail_4:
	device_destroy(tegra_rawchip_class, tegra_rawchip_devno);

fail_3:
	class_destroy(tegra_rawchip_class);

fail_2:
	unregister_chrdev_region(tegra_rawchip_devno, 1);

fail_1:
	__tegra_rawchip_exit(tegra_rawchip_device_p);

fail:
	return rc;
}

static void tegra_rawchip_exit(void)
{
	cdev_del(&tegra_rawchip_device_p->cdev);
	device_destroy(tegra_rawchip_class, tegra_rawchip_devno);
	class_destroy(tegra_rawchip_class);
	unregister_chrdev_region(tegra_rawchip_devno, 1);
	__tegra_rawchip_exit(tegra_rawchip_device_p);
}

static int __tegra_rawchip_probe(struct platform_device *pdev)
{
	int rc;
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

	yushan_pdev = pdev;
	rc = tegra_rawchip_init(pdev);

	if (rc < 0) {
		kfree(rawchipCtrl);
		return rc;
	}


	return rc;
}

static int __tegra_rawchip_remove(struct platform_device *pdev)
{
	pr_info("%s:%d]\n", __func__, __LINE__);
	tegra_rawchip_exit();
	kfree(rawchipCtrl);
	return 0;
}

static struct  platform_driver tegra_rawchip_driver = {
	.probe  = __tegra_rawchip_probe,
	.remove = __tegra_rawchip_remove,
	.driver = {
		.name = "rawchip",
		.owner = THIS_MODULE,
	},
};

static int __init tegra_rawchip_driver_init(void)
{
	int rc;
	rc = platform_driver_register(&tegra_rawchip_driver);
	return rc;
}

static void __exit tegra_rawchip_driver_exit(void)
{
	platform_driver_unregister(&tegra_rawchip_driver);
}

MODULE_DESCRIPTION("tegra rawchip driver");
MODULE_VERSION("tegra rawchip 0.1");

module_init(tegra_rawchip_driver_init);
module_exit(tegra_rawchip_driver_exit);

