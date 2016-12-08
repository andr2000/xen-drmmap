/*
 *  Xen para-virtual DRM device
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 * Copyright (C) 2016 EPAM Systems Inc.
 */

#include <drm/drmP.h>

#include <drm/drm_fb_cma_helper.h>
#include <drm/drm_gem_cma_helper.h>

#include <linux/dma-buf.h>
#include <linux/platform_device.h>

#include "xen-drm-map.h"
#include "xen-drm-logs.h"


struct xendrmmap_info {
	struct drm_device *drm_dev;
};

static const struct file_operations xendrm_fops = {
	.owner          = THIS_MODULE,
	.open           = drm_open,
	.release        = drm_release,
	.unlocked_ioctl = drm_ioctl,
};

static int memset_page(void *addr, int color)
{
	if (!addr)
		return -ENOMEM;
	memset(addr, color, PAGE_SIZE);
	return 0;
}

static int xendrmmap_ioctl_map(struct drm_device *dev, void *data,
	struct drm_file *file_priv)
{
	struct xendrmmap_ioctl_map *m = (struct xendrmmap_ioctl_map *)data;

	DRM_DEBUG("m %p\n", m);
	if (!m)
		return -EINVAL;
	DRM_DEBUG("Prime fd %u\n", m->fd);
	return 0;
}

struct drm_gem_object *xendrmmap_gem_prime_import_sg_table(
	struct drm_device *dev, struct dma_buf_attachment *attach,
	struct sg_table *sgt)
{
	struct sg_page_iter piter;
	int i;

	DRM_DEBUG("Number of segments in the sg table: %d, is contiguous %d\n",
		sgt->nents, sgt->nents == 1);

	for_each_sg_page(sgt->sgl, &piter, sgt->nents, 0) {
		struct page *page;
		dma_addr_t dma_addr;

		page = sg_page_iter_page(&piter);
		dma_addr = sg_page_iter_dma_address(&piter);
		memset_page(page_to_virt(page), i);
		i++;
	}
	return NULL;
}

void xendrmmap_gem_close_object(struct drm_gem_object *gem_obj, struct drm_file *file_priv)
{
	/* unmap here */
}

static const struct drm_ioctl_desc xendrmmap_ioctls[] = {
	DRM_IOCTL_DEF_DRV(XENDRM_MAP, xendrmmap_ioctl_map, DRM_AUTH |
		DRM_UNLOCKED | DRM_RENDER_ALLOW),
};

static struct drm_driver xendrmmap_driver = {
	.driver_features           = DRIVER_GEM | DRIVER_PRIME,
	.prime_fd_to_handle        = drm_gem_prime_fd_to_handle,
	.gem_prime_import          = drm_gem_prime_import,
	.gem_prime_import_sg_table = xendrmmap_gem_prime_import_sg_table,
	.gem_close_object          = xendrmmap_gem_close_object,
	.fops                      = &xendrm_fops,
	.ioctls                    = xendrmmap_ioctls,
	.num_ioctls                = ARRAY_SIZE(xendrmmap_ioctls),
	.name                      = XENDRMMAP_DRIVER_NAME,
	.desc                      = "Xen PV DRM mapper",
	.date                      = "20161207",
	.major                     = 1,
	.minor                     = 0,
};

static int xendrmmap_remove(struct platform_device *pdev)
{
	struct xendrmmap_info *info = platform_get_drvdata(pdev);
	struct drm_device *drm_dev = info->drm_dev;

	drm_dev_unregister(drm_dev);
	drm_dev_unref(drm_dev);
	return 0;
}

static int xendrmmap_probe(struct platform_device *pdev)
{
	struct xendrmmap_info *info;
	struct drm_device *drm_dev;
	int ret;

	LOG0("Creating %s", xendrmmap_driver.desc);
	info = devm_kzalloc(&pdev->dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	drm_dev = drm_dev_alloc(&xendrmmap_driver, &pdev->dev);
	if (!drm_dev)
		return -ENOMEM;

	info->drm_dev = drm_dev;
	drm_dev->dev_private = info;
	platform_set_drvdata(pdev, info);

	ret = drm_dev_register(drm_dev, 0);
	if (ret)
		goto fail;

	DRM_INFO("Initialized %s %d.%d.%d %s on minor %d\n",
		xendrmmap_driver.name, xendrmmap_driver.major,
		xendrmmap_driver.minor, xendrmmap_driver.patchlevel,
		xendrmmap_driver.date, drm_dev->primary->index);
	return 0;
fail:
	/* TODO: remove doesn't check if any part of the driver was created
	 * this needs to be fixed
	 */
	xendrmmap_remove(pdev);
	return ret;
}

static struct platform_driver xendrmmap_ddrv_info = {
	.probe		= xendrmmap_probe,
	.remove		= xendrmmap_remove,
	.driver		= {
		.name	= XENDRMMAP_DRIVER_NAME,
	},
};

static struct platform_device *xendrmmap_pdev;

static int __init xendrmmap_init(void)
{
	int ret;

	xendrmmap_pdev = platform_device_alloc(XENDRMMAP_DRIVER_NAME, -1);
	if (!xendrmmap_pdev) {
		LOG0("Failed to allocate " XENDRMMAP_DRIVER_NAME \
			" device");
		return -ENODEV;
	}
	ret = platform_device_add(xendrmmap_pdev);
	if (ret != 0) {
		LOG0("Failed to register " XENDRMMAP_DRIVER_NAME \
			" device: %d\n", ret);
		platform_device_put(xendrmmap_pdev);
		return -ENODEV;
	}
	ret = platform_driver_register(&xendrmmap_ddrv_info);
	if (ret != 0) {
		LOG0("Failed to register " XENDRMMAP_DRIVER_NAME \
			" driver: %d\n", ret);
		platform_device_unregister(xendrmmap_pdev);
	}
	return 0;
}

static void __exit xendrmmap_cleanup(void)
{
	platform_driver_unregister(&xendrmmap_ddrv_info);
	if (xendrmmap_pdev)
		platform_device_unregister(xendrmmap_pdev);
}

module_init(xendrmmap_init);
module_exit(xendrmmap_cleanup);

MODULE_DESCRIPTION("Xen DRM buffer mapper");
MODULE_LICENSE("GPL");
MODULE_ALIAS("xen:" XENDRMMAP_DRIVER_NAME);
