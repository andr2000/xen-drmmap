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

#define XENDRMMAP_DRIVER_NAME	"xen-drmmap"

static const struct file_operations xendrm_fops = {
	.owner          = THIS_MODULE,
	.open           = drm_open,
	.release        = drm_release,
	.unlocked_ioctl = drm_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl   = drm_compat_ioctl,
#endif
	.poll           = drm_poll,
	.read           = drm_read,
	.llseek         = no_llseek,
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
	struct dma_buf *dma_buf;
	struct dma_buf_attachment *attach;
	struct sg_table *sg;
	struct sg_page_iter piter;
	int ret, i;

	DRM_DEBUG("m %p\n", m);
	if (!m)
		return -EINVAL;
	DRM_DEBUG("Prime fd %u\n", m->fd);
	dma_buf = dma_buf_get(m->fd);
	if (IS_ERR(dma_buf)) {
		DRM_ERROR("Prime fd %u not found, ret %lu\n",
			m->fd, PTR_ERR(dma_buf));
		return PTR_ERR(dma_buf);
	}
	DRM_DEBUG("Prime fd %u found, dma_buf at %p size %zu\n",
		m->fd, dma_buf, dma_buf->size);

	attach = dma_buf_attach(dma_buf, dev->dev);
	if (!attach) {
		DRM_ERROR("Failed to attach DMA buffer for fd %u, ret %lu\n",
			m->fd, PTR_ERR(attach));
		return PTR_ERR(attach);
	}
	DRM_DEBUG("Attached %p, prime fd %u, dma_buf at %p size %zu\n",
		attach, m->fd, dma_buf, dma_buf->size);
	sg = dma_buf_map_attachment(attach, DMA_BIDIRECTIONAL);
	if (!sg) {
		DRM_ERROR("Failed to map attachment, ret %lu\n",
			PTR_ERR(attach));
		ret = PTR_ERR(attach);
		goto out_detach;
	}
	i = 0;
	DRM_DEBUG("Number of segments in the table: %d is contiguous %d\n",
		sg->nents, sg->nents == 1);

	for_each_sg_page(sg->sgl, &piter, sg->nents, 0) {
		struct page *page;
		dma_addr_t dma_addr;

		page = sg_page_iter_page(&piter);
		dma_addr = sg_page_iter_dma_address(&piter);
		memset_page(page_to_virt(page), 0x7f + i);
		i++;
	}
	DRM_DEBUG("Filled %d pages\n", i);
	DRM_DEBUG("Unmapping attachment\n");
	dma_buf_unmap_attachment(attach, sg, DMA_BIDIRECTIONAL);
	ret = 0;
out_detach:
#if 0
	DRM_DEBUG("Detaching dma_buf\n");
	dma_buf_detach(dma_buf, attach);
	dma_buf_put(dma_buf);
#endif
	return ret;
}

static int xendrmmap_ioctl_unmap(struct drm_device *dev, void *data,
	struct drm_file *file_priv)
{
	struct xendrmmap_ioctl_unmap *m = (struct xendrmmap_ioctl_unmap *)data;
	struct dma_buf *dma_buf;
	struct dma_buf_attachment *attach;
	struct sg_table *sg;
	struct sg_page_iter piter;
	int ret, i;

	DRM_DEBUG("m %p\n", m);
	if (!m)
		return -EINVAL;
	DRM_DEBUG("Prime fd %u\n", m->fd);
	dma_buf = dma_buf_get(m->fd);
	if (IS_ERR(dma_buf)) {
		DRM_ERROR("Prime fd %u not found, ret %lu\n",
			m->fd, PTR_ERR(dma_buf));
		return PTR_ERR(dma_buf);
	}
	DRM_DEBUG("Detaching dma_buf\n");
	dma_buf_detach(dma_buf, attach);
	dma_buf_put(dma_buf);
	return ret;
}

static const struct drm_ioctl_desc xendrmmap_ioctls[] = {
	DRM_IOCTL_DEF_DRV(XENDRM_MAP, xendrmmap_ioctl_map, DRM_AUTH |
		DRM_UNLOCKED | DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(XENDRM_UNMAP, xendrmmap_ioctl_unmap, DRM_AUTH |
		DRM_UNLOCKED | DRM_RENDER_ALLOW),
};

static struct drm_driver xendrmmap_driver = {
	.driver_features           = DRIVER_GEM | DRIVER_PRIME,
	.prime_handle_to_fd        = drm_gem_prime_handle_to_fd,
	.prime_fd_to_handle        = drm_gem_prime_fd_to_handle,
	.gem_prime_import          = drm_gem_prime_import,
	.gem_prime_export          = drm_gem_prime_export,
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
