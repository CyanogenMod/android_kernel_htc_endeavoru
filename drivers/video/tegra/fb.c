/*
 * drivers/video/tegra/fb.c
 *
 * Copyright (C) 2010 Google, Inc.
 * Author: Erik Gilling <konkers@android.com>
 *         Colin Cross <ccross@android.com>
 *         Travis Geiselbrecht <travis@palm.com>
 *
 * Copyright (C) 2010-2011 NVIDIA Corporation
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/fb.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/file.h>
#include <linux/workqueue.h>

#include <asm/atomic.h>

#include <video/tegrafb.h>

#include <mach/dc.h>
#include <mach/fb.h>
#include <mach/board_htc.h>
#include <linux/nvhost.h>
#include <mach/nvmap.h>

#include "host/dev.h"
#include "nvmap/nvmap.h"
#include "dc/dc_priv.h"

#define ONMODE_CHARGE() ((board_mfg_mode() == 0) && \
							(board_zchg_mode() & 0x2) && \
							(dc == tegra_fb->win->dc))


/* Pad pitch to 16-byte boundary. */
#define TEGRA_LINEAR_PITCH_ALIGNMENT 16

struct tegra_usb_projector_info usb_pjt_info;
struct tegra_fb_info {
	struct tegra_dc_win	*win;
	struct nvhost_device	*ndev;
	struct fb_info		*info;
	bool			valid;

	struct resource		*fb_mem;

	int			xres;
	int			yres;
};

/* palette array used by the fbcon */
static u32 pseudo_palette[16];

static int tegra_fb_check_var(struct fb_var_screeninfo *var,
			      struct fb_info *info)
{
	if ((var->yres * var->xres * var->bits_per_pixel / 8 * 2) >
	    info->screen_size)
		return -EINVAL;

	/* double yres_virtual to allow double buffering through pan_display */
	var->yres_virtual = var->yres * 2;

	return 0;
}

static int tegra_fb_set_par(struct fb_info *info)
{
	struct tegra_fb_info *tegra_fb = info->par;
	struct fb_var_screeninfo *var = &info->var;

	if (var->bits_per_pixel) {
		/* we only support RGB ordering for now */
		switch (var->bits_per_pixel) {
		case 32:
			var->red.offset = 0;
			var->red.length = 8;
			var->green.offset = 8;
			var->green.length = 8;
			var->blue.offset = 16;
			var->blue.length = 8;
			var->transp.offset = 24;
			var->transp.length = 8;
			tegra_fb->win->fmt = TEGRA_WIN_FMT_R8G8B8A8;
			break;
		case 16:
			var->red.offset = 11;
			var->red.length = 5;
			var->green.offset = 5;
			var->green.length = 6;
			var->blue.offset = 0;
			var->blue.length = 5;
			tegra_fb->win->fmt = TEGRA_WIN_FMT_B5G6R5;
			break;

		default:
			return -EINVAL;
		}
		info->fix.line_length = var->xres * var->bits_per_pixel / 8;
		/* Pad the stride to 16-byte boundary. */
		info->fix.line_length = round_up(info->fix.line_length,
						TEGRA_LINEAR_PITCH_ALIGNMENT);
		tegra_fb->win->stride = info->fix.line_length;
		tegra_fb->win->stride_uv = 0;
		tegra_fb->win->phys_addr_u = 0;
		tegra_fb->win->phys_addr_v = 0;
	}

	if (var->pixclock) {
		bool stereo;
		struct fb_videomode m;

		fb_var_to_videomode(&m, var);

		info->mode = (struct fb_videomode *)
			fb_find_nearest_mode(&m, &info->modelist);
		if (!info->mode) {
			dev_warn(&tegra_fb->ndev->dev, "can't match video mode\n");
			return -EINVAL;
		}

		/*
		 * only enable stereo if the mode supports it and
		 * client requests it
		 */
		stereo = !!(var->vmode & info->mode->vmode &
					FB_VMODE_STEREO_FRAME_PACK);

		tegra_dc_set_fb_mode(tegra_fb->win->dc, info->mode, stereo);

		tegra_fb->win->w.full = dfixed_const(info->mode->xres);
		tegra_fb->win->h.full = dfixed_const(info->mode->yres);
		tegra_fb->win->out_w = info->mode->xres;
		tegra_fb->win->out_h = info->mode->yres;
	}
	return 0;
}

static int tegra_fb_setcmap(struct fb_cmap *cmap, struct fb_info *info)
{
	struct tegra_fb_info *tegra_fb = info->par;
	struct tegra_dc *dc = tegra_fb->win->dc;
	int i;
	u16 *red = cmap->red;
	u16 *green = cmap->green;
	u16 *blue = cmap->blue;
	int start = cmap->start;

	if (((unsigned)start > 255) || ((start + cmap->len) > 256))
		return -EINVAL;

	if (info->fix.visual == FB_VISUAL_TRUECOLOR ||
	    info->fix.visual == FB_VISUAL_DIRECTCOLOR) {
		for (i = 0; i < cmap->len; i++) {
			dc->fb_lut.r[start+i] = *red++ >> 8;
			dc->fb_lut.g[start+i] = *green++ >> 8;
			dc->fb_lut.b[start+i] = *blue++ >> 8;
		}

		tegra_dc_update_lut(dc, -1, -1);
	}

	return 0;
}

static int tegra_fb_blank(int blank, struct fb_info *info)
{
	struct tegra_fb_info *tegra_fb = info->par;

	switch (blank) {
	case FB_BLANK_UNBLANK:
		dev_dbg(&tegra_fb->ndev->dev, "unblank\n");
		tegra_fb->win->flags = TEGRA_WIN_FLAG_ENABLED;
		tegra_dc_enable(tegra_fb->win->dc);
		return 0;

	case FB_BLANK_NORMAL:
		dev_dbg(&tegra_fb->ndev->dev, "blank - normal\n");
		tegra_dc_blank(tegra_fb->win->dc);
		return 0;

	case FB_BLANK_VSYNC_SUSPEND:
	case FB_BLANK_HSYNC_SUSPEND:
	case FB_BLANK_POWERDOWN:
		dev_dbg(&tegra_fb->ndev->dev, "blank - powerdown\n");
		tegra_dc_disable(tegra_fb->win->dc);
		return 0;

	default:
		return -ENOTTY;
	}
}

static int tegra_fb_pan_display(struct fb_var_screeninfo *var,
				struct fb_info *info)
{
	struct tegra_fb_info *tegra_fb = info->par;
	char __iomem *flush_start;
	char __iomem *flush_end;
	u32 addr;

	int i;
	struct tegra_dc *dc = tegra_dc_get_dc(0);
	/*
	This is only for china sku suspend/resume battery update and only for DC0
	Initialize window. It wouldn't support yuv in framebuffer.
	Therefore, we set RGBX as default and disable the other windows.
	*/
	struct tegra_dc_win *dcwins[DC_N_WINDOWS];
	if (ONMODE_CHARGE()) {
		for (i = 0; i < DC_N_WINDOWS; i++) {
			dcwins[i] = tegra_dc_get_window(tegra_fb->win->dc, i);
			dcwins[i]->fmt = TEGRA_WIN_FMT_R8G8B8A8;
			if (tegra_fb->win != dcwins[i])
				dcwins[i]->flags &= ~TEGRA_WIN_FLAG_ENABLED;
			else
				dcwins[i]->flags |= TEGRA_WIN_FLAG_ENABLED;
		}
	}
	if (!tegra_fb->win->cur_handle) {
		flush_start = info->screen_base + (var->yoffset * info->fix.line_length);
		flush_end = flush_start + (var->yres * info->fix.line_length);

		info->var.xoffset = var->xoffset;
		info->var.yoffset = var->yoffset;

		addr = info->fix.smem_start + (var->yoffset * info->fix.line_length) +
			(var->xoffset * (var->bits_per_pixel/8));

		tegra_fb->win->phys_addr = addr;
		/* TODO: update virt_addr */
		tegra_fb->win->x.full = dfixed_const(0);
		tegra_fb->win->y.full = dfixed_const(0);
		tegra_fb->win->w.full = dfixed_const(var->xres);
		tegra_fb->win->h.full = dfixed_const(var->yres);
		tegra_fb->win->out_x = 0;
		tegra_fb->win->out_y = 0;
		tegra_fb->win->out_w = var->xres;
		tegra_fb->win->out_h = var->yres;
		tegra_fb->win->z = 0;
		tegra_fb->win->stride = info->fix.line_length;
		if (ONMODE_CHARGE()) {
			/*Update all of windows, not only window a, b or c*/
			tegra_dc_update_windows(dcwins, DC_N_WINDOWS);
			tegra_dc_sync_windows(dcwins, DC_N_WINDOWS);
		}
		else {
			tegra_dc_update_windows(&tegra_fb->win, 1);
			tegra_dc_sync_windows(&tegra_fb->win, 1);
		}
	}
	return 0;
}

static void tegra_fb_fillrect(struct fb_info *info,
			      const struct fb_fillrect *rect)
{
	cfb_fillrect(info, rect);
}

static void tegra_fb_copyarea(struct fb_info *info,
			      const struct fb_copyarea *region)
{
	cfb_copyarea(info, region);
}

static void tegra_fb_imageblit(struct fb_info *info,
			       const struct fb_image *image)
{
	cfb_imageblit(info, image);
}

static int tegra_fb_ioctl(struct fb_info *info, unsigned int cmd, unsigned long arg)
{
	struct tegra_fb_modedb modedb;
	void __user *argp = (void __user *)arg;
	struct fb_modelist *modelist;
	int i;

	int ret = 0;
	struct tegra_usb_projector_info tmp_info;
	switch (cmd) {
	case FBIO_TEGRA_GET_MODEDB:
		if (copy_from_user(&modedb, (void __user *)arg, sizeof(modedb)))
			return -EFAULT;

		i = 0;
		list_for_each_entry(modelist, &info->modelist, list) {
			struct fb_var_screeninfo var;

			if (i >= modedb.modedb_len)
				break;

			/* fb_videomode_to_var doesn't fill out all the members
			   of fb_var_screeninfo */
			memset(&var, 0x0, sizeof(var));

			fb_videomode_to_var(&var, &modelist->mode);

			if (copy_to_user((void __user *)&modedb.modedb[i],
					 &var, sizeof(var)))
				return -EFAULT;
			i++;

			if (var.vmode & FB_VMODE_STEREO_MASK) {
				if (i >= modedb.modedb_len)
					break;
				var.vmode &= ~FB_VMODE_STEREO_MASK;
				if (copy_to_user(
					(void __user *)&modedb.modedb[i],
					 &var, sizeof(var)))
					return -EFAULT;
				i++;
			}
		}
		modedb.modedb_len = i;

		if (copy_to_user((void __user *)arg, &modedb, sizeof(modedb)))
			return -EFAULT;
		break;
       case FBIO_TEGRA_GET_USB_PROJECTOR_INFO:
               ret = copy_to_user(argp, &usb_pjt_info, sizeof(usb_pjt_info));
               if (ret)
                       return ret;
               break;
       case FBIO_TEGRA_SET_USB_PROJECTOR_INFO:
               ret = copy_from_user(&tmp_info, argp, sizeof(tmp_info));
               usb_pjt_info.latest_offset = tmp_info.latest_offset;
               if (ret)
                       return ret;
               break;
	default:
		return -ENOTTY;
	}

	return 0;
}

static struct fb_ops tegra_fb_ops = {
	.owner = THIS_MODULE,
	.fb_check_var = tegra_fb_check_var,
	.fb_set_par = tegra_fb_set_par,
	.fb_setcmap = tegra_fb_setcmap,
	.fb_blank = tegra_fb_blank,
	.fb_pan_display = tegra_fb_pan_display,
	.fb_fillrect = tegra_fb_fillrect,
	.fb_copyarea = tegra_fb_copyarea,
	.fb_imageblit = tegra_fb_imageblit,
	.fb_ioctl = tegra_fb_ioctl,
};

void tegra_fb_update_monspecs(struct tegra_fb_info *fb_info,
			      struct fb_monspecs *specs,
			      bool (*mode_filter)(const struct tegra_dc *dc,
						  struct fb_videomode *mode))
{
	struct fb_event event;
	int i;

	mutex_lock(&fb_info->info->lock);
	fb_destroy_modedb(fb_info->info->monspecs.modedb);

	fb_destroy_modelist(&fb_info->info->modelist);

	if (specs == NULL) {
		struct tegra_dc_mode mode;
		memset(&fb_info->info->monspecs, 0x0,
		       sizeof(fb_info->info->monspecs));
		memset(&mode, 0x0, sizeof(mode));

		/*
		 * reset video mode properties to prevent garbage being displayed on 'mode' device.
		 */
		fb_info->info->mode = (struct fb_videomode*) NULL;

		tegra_dc_set_mode(fb_info->win->dc, &mode);
		mutex_unlock(&fb_info->info->lock);
		return;
	}

	memcpy(&fb_info->info->monspecs, specs,
	       sizeof(fb_info->info->monspecs));

	for (i = 0; i < specs->modedb_len; i++) {
		if (mode_filter) {
			if (mode_filter(fb_info->win->dc, &specs->modedb[i]))
				fb_add_videomode(&specs->modedb[i],
						 &fb_info->info->modelist);
		} else {
			fb_add_videomode(&specs->modedb[i],
					 &fb_info->info->modelist);
		}
	}

	event.info = fb_info->info;
	fb_notifier_call_chain(FB_EVENT_NEW_MODELIST, &event);
	mutex_unlock(&fb_info->info->lock);
}

struct tegra_fb_info *tegra_fb_register(struct nvhost_device *ndev,
					struct tegra_dc *dc,
					struct tegra_fb_data *fb_data,
					struct resource *fb_mem)
{
	struct tegra_dc_win *win;
	struct fb_info *info;
	struct tegra_fb_info *tegra_fb;
	void __iomem *fb_base = NULL;
	unsigned long fb_size = 0;
	unsigned long fb_phys = 0;
	int ret = 0;

	win = tegra_dc_get_window(dc, fb_data->win);
	if (!win) {
		dev_err(&ndev->dev, "dc does not have a window at index %d\n",
			fb_data->win);
		return ERR_PTR(-ENOENT);
	}

	info = framebuffer_alloc(sizeof(struct tegra_fb_info), &ndev->dev);
	if (!info) {
		ret = -ENOMEM;
		goto err;
	}

	tegra_fb = info->par;
	tegra_fb->win = win;
	tegra_fb->ndev = ndev;
	tegra_fb->fb_mem = fb_mem;
	tegra_fb->xres = fb_data->xres;
	tegra_fb->yres = fb_data->yres;

	if (fb_mem) {
		fb_size = resource_size(fb_mem);
		fb_phys = fb_mem->start;
		fb_base = ioremap_nocache(fb_phys, fb_size);
		if (!fb_base) {
			dev_err(&ndev->dev, "fb can't be mapped\n");
			ret = -EBUSY;
			goto err_free;
		}
		tegra_fb->valid = true;
	}

	info->fbops = &tegra_fb_ops;
	info->pseudo_palette = pseudo_palette;
	info->screen_base = fb_base;
	info->screen_size = fb_size;

	strlcpy(info->fix.id, "tegra_fb", sizeof(info->fix.id));
	info->fix.type		= FB_TYPE_PACKED_PIXELS;
	info->fix.visual	= FB_VISUAL_TRUECOLOR;
	info->fix.xpanstep	= 1;
	info->fix.ypanstep	= 1;
	info->fix.accel		= FB_ACCEL_NONE;
	info->fix.smem_start	= fb_phys;
	info->fix.smem_len	= fb_size;
	info->fix.line_length = fb_data->xres * fb_data->bits_per_pixel / 8;
	/* Pad the stride to 16-byte boundary. */
	info->fix.line_length = round_up(info->fix.line_length,
					TEGRA_LINEAR_PITCH_ALIGNMENT);

	info->var.xres			= fb_data->xres;
	info->var.yres			= fb_data->yres;
	info->var.xres_virtual		= fb_data->xres;
	info->var.yres_virtual		= fb_data->yres * 2;
	info->var.bits_per_pixel	= fb_data->bits_per_pixel;
	info->var.activate		= FB_ACTIVATE_VBL;
	info->var.height		= tegra_dc_get_out_height(dc);
	info->var.width			= tegra_dc_get_out_width(dc);
	info->var.pixclock		= 0;
	info->var.left_margin		= 0;
	info->var.right_margin		= 0;
	info->var.upper_margin		= 0;
	info->var.lower_margin		= 0;
	info->var.hsync_len		= 0;
	info->var.vsync_len		= 0;
	info->var.vmode			= FB_VMODE_NONINTERLACED;

	win->x.full = dfixed_const(0);
	win->y.full = dfixed_const(0);
	win->w.full = dfixed_const(fb_data->xres);
	win->h.full = dfixed_const(fb_data->yres);
	/* TODO: set to output res dc */
	win->out_x = 0;
	win->out_y = 0;
	win->out_w = fb_data->xres;
	win->out_h = fb_data->yres;
	win->z = 0;
	win->phys_addr = fb_phys;
	win->virt_addr = fb_base;
	win->phys_addr_u = 0;
	win->phys_addr_v = 0;
	win->stride = info->fix.line_length;
	win->stride_uv = 0;
	win->flags = TEGRA_WIN_FLAG_ENABLED;

	if (fb_mem)
		tegra_fb_set_par(info);

	if (register_framebuffer(info)) {
		dev_err(&ndev->dev, "failed to register framebuffer\n");
		ret = -ENODEV;
		goto err_iounmap_fb;
	}

	tegra_fb->info = info;

	dev_info(&ndev->dev, "probed\n");

	if (fb_data->flags & TEGRA_FB_FLIP_ON_PROBE) {
		tegra_dc_update_windows(&tegra_fb->win, 1);
		tegra_dc_sync_windows(&tegra_fb->win, 1);
	}

	return tegra_fb;

err_iounmap_fb:
	if (fb_base)
		iounmap(fb_base);
err_free:
	framebuffer_release(info);
err:
	return ERR_PTR(ret);
}

void tegra_fb_unregister(struct tegra_fb_info *fb_info)
{
	struct fb_info *info = fb_info->info;

	unregister_framebuffer(info);

	iounmap(info->screen_base);
	framebuffer_release(info);
}
