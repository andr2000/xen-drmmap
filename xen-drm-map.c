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

#include <xen/grant_table.h>

#include "xen-drm-map.h"
#include "xen-drm-logs.h"


struct xendrmmap_info {
	struct drm_device *drm_dev;
};

struct xendrmmap_gem_object {
	struct drm_gem_object base;
	struct page **pages;
	bool is_contiguos;
	struct xendrmmap_ioctl_create_dumb dumb_obj;
};

static inline struct xendrmmap_gem_object *
to_xendrmmap_gem_obj(struct drm_gem_object *gem_obj)
{
	return container_of(gem_obj, struct xendrmmap_gem_object, base);
}

static int xendrmmap_do_map(struct xendrmmap_gem_object *xen_obj)
{
#if 0
	struct sg_page_iter piter;

	for_each_sg_page(xen_obj->sgt->sgl, &piter,
			xen_obj->sgt->nents, 0) {
		struct page *page;
		dma_addr_t dma_addr;

		page = sg_page_iter_page(&piter);
		dma_addr = sg_page_iter_dma_address(&piter);
	}
#endif
	return 0;
}

static int xendrmmap_do_unmap(struct xendrmmap_gem_object *xen_obj)
{
#if 0
	struct sg_page_iter piter;

	for_each_sg_page(xen_obj->sgt->sgl, &piter,
			xen_obj->sgt->nents, 0) {
		struct page *page;
		dma_addr_t dma_addr;

		page = sg_page_iter_page(&piter);
		dma_addr = sg_page_iter_dma_address(&piter);
	}
#endif
	return 0;
}


static void xendrmmap_gem_close_object(struct drm_gem_object *gem_obj,
	struct drm_file *file_priv)
{
	struct xendrmmap_gem_object *xen_obj = to_xendrmmap_gem_obj(gem_obj);

	DRM_DEBUG("++++++++++++ Closing GEM object\n");
	xendrmmap_do_unmap(xen_obj);
}

static void xendrmmap_gem_free_object(struct drm_gem_object *gem_obj)
{
	struct xendrmmap_gem_object *xen_obj = to_xendrmmap_gem_obj(gem_obj);

	DRM_DEBUG("++++++++++++ Freeing GEM object\n");
	drm_gem_object_release(gem_obj);
	if (xen_obj->dumb_obj.grefs)
		kfree(xen_obj->dumb_obj.grefs);
	if (xen_obj->pages)
		kfree(xen_obj->pages);
	kfree(xen_obj);
}

static int xendrmmap_gem_create_with_handle(
	struct xendrmmap_gem_object *xen_obj, struct drm_file *file_priv,
	struct drm_device *dev, size_t size)
{
	struct drm_gem_object *gem_obj;
	int ret;

	ret = drm_gem_object_init(dev, &xen_obj->base, size);
	if (ret < 0) {
		DRM_DEBUG("++++++++++++ Failed to initialize GEM, ret %d\n",
			ret);
		return ret;
	}
	ret = drm_gem_create_mmap_offset(gem_obj);
	if (ret < 0) {
		drm_gem_object_release(gem_obj);
		return ret;
	}
	gem_obj = &xen_obj->base;
	/*
	 * allocate a id of idr table where the obj is registered
	 * and handle has the id that user can see.
	 */
	ret = drm_gem_handle_create(file_priv, gem_obj,
		&xen_obj->dumb_obj.dumb.handle);
	/* drop reference from allocate - handle holds it now. */
	drm_gem_object_unreference_unlocked(gem_obj);
	return ret;
}

static int xendrm_create_dumb_obj(struct xendrmmap_gem_object *xen_obj,
	struct drm_device *dev, void *data, struct drm_file *file_priv)
{
	struct xendrmmap_ioctl_create_dumb *xen_args =
		(struct xendrmmap_ioctl_create_dumb *)data;
	struct drm_mode_create_dumb *args = &xen_args->dumb;
	struct drm_gem_object *gem_obj;
	int ret;

	ret = xendrmmap_gem_create_with_handle(xen_obj, file_priv, dev,
		args->size);
	if (ret < 0)
		goto fail;
	gem_obj = drm_gem_object_lookup(file_priv, args->handle);
	if (!gem_obj) {
		ret = -EINVAL;
		goto fail_destroy;
	}
	drm_gem_object_unreference_unlocked(gem_obj);
	return 0;

fail_destroy:
	drm_gem_dumb_destroy(file_priv, dev, args->handle);
fail:
	DRM_ERROR("++++++++++++ Failed to create dumb buffer, ret %d\n", ret);
	return ret;
}

static int xendrm_do_dumb_create(struct drm_device *dev, void *data,
	struct drm_file *file_priv)
{
	struct xendrmmap_ioctl_create_dumb *args =
		(struct xendrmmap_ioctl_create_dumb *)data;
	struct xendrmmap_gem_object *xen_obj;
	int sz, ret;

	DRM_DEBUG("++++++++++++ args at %p\n", args);
	if (!args)
		return -EINVAL;
	if (args->num_grefs > DIV_ROUND_UP(args->dumb.size, PAGE_SIZE)) {
		DRM_ERROR("++++++++++++ Provided %d pages, required %d\n",
			args->num_grefs,
			(int)DIV_ROUND_UP(args->dumb.size, PAGE_SIZE));
		return -EINVAL;
	}
	xen_obj = kzalloc(sizeof(*xen_obj), GFP_KERNEL);
	if (!xen_obj)
		return -ENOMEM;
	DRM_DEBUG("++++++++++++ Creating DUMB\n");
	xen_obj->dumb_obj.dumb = args->dumb;
	sz = args->num_grefs * sizeof(grant_ref_t);
	xen_obj->dumb_obj.grefs = kmalloc(sz, GFP_KERNEL);
	if (!xen_obj->dumb_obj.grefs) {
		ret = -ENOMEM;
		goto fail;
	}
	if (copy_from_user(xen_obj->dumb_obj.grefs, args->grefs, sz) != sz) {
		ret = -EINVAL;
		goto fail;
	}
	sz = args->num_grefs * sizeof(struct page *);
	xen_obj->pages = kmalloc(sz, GFP_KERNEL);
	if (!xen_obj->pages) {
		ret = -ENOMEM;
		goto fail;
	}
	ret = xendrmmap_do_map(xen_obj);
	if (ret < 0)
		goto fail;
	return xendrm_create_dumb_obj(xen_obj, dev, data, file_priv);

fail:
	if (xen_obj->dumb_obj.grefs)
		kfree(xen_obj->dumb_obj.grefs);
	xen_obj->dumb_obj.grefs = NULL;
	if (xen_obj->pages)
		kfree(xen_obj->pages);
	xen_obj->pages = NULL;
	return ret;
}

static int xendrmmap_create_dumb_ioctl(struct drm_device *dev,
	void *data, struct drm_file *file_priv)
{
	struct xendrmmap_ioctl_create_dumb *xen_args =
		(struct xendrmmap_ioctl_create_dumb *)data;
	struct drm_mode_create_dumb *args = &xen_args->dumb;
	u32 cpp, stride, size;

	if (!dev->driver->dumb_create)
		return -ENOSYS;
	if (!args->width || !args->height || !args->bpp)
		return -EINVAL;

	/* overflow checks for 32bit size calculations */
	/* NOTE: DIV_ROUND_UP() can overflow */
	cpp = DIV_ROUND_UP(args->bpp, 8);
	if (!cpp || cpp > 0xffffffffU / args->width)
		return -EINVAL;
	stride = cpp * args->width;
	if (args->height > 0xffffffffU / stride)
		return -EINVAL;

	/* test for wrap-around */
	size = args->height * stride;
	if (PAGE_ALIGN(size) == 0)
		return -EINVAL;

	/* this are the output parameters */
	args->pitch = DIV_ROUND_UP(args->width * args->bpp, 8);
	args->size = args->pitch * args->height;
	args->handle = 0;
	return xendrm_do_dumb_create(dev, data, file_priv);
}

static struct sg_table *xendrmmap_gem_prime_get_sg_table_cont(
	struct xendrmmap_gem_object *xen_obj)
{
	return NULL;
}

struct sg_table *xendrmmap_gem_prime_get_sg_table(
	struct drm_gem_object *gem_obj)
{
	struct xendrmmap_gem_object *xen_obj = to_xendrmmap_gem_obj(gem_obj);

	DRM_DEBUG("++++++++++++ Exporting sgt\n");
	/* FIXME: drivers relying on CMA will
	 * not accept the buffer otherwise
	 */
	if (xen_obj->is_contiguos)
		return xendrmmap_gem_prime_get_sg_table_cont(xen_obj);
	return drm_prime_pages_to_sg(xen_obj->pages,
		xen_obj->dumb_obj.num_grefs);
}

static const struct drm_ioctl_desc xendrmmap_ioctls[] = {
	DRM_IOCTL_DEF_DRV(XENDRM_CREATE_DUMB, xendrmmap_create_dumb_ioctl,
		DRM_CONTROL_ALLOW | DRM_UNLOCKED),
};

static const struct file_operations xendrmmap_fops = {
	.owner          = THIS_MODULE,
	.open           = drm_open,
	.release        = drm_release,
	.unlocked_ioctl = drm_ioctl,
};

static struct drm_driver xendrmmap_driver = {
	.driver_features           = DRIVER_GEM | DRIVER_PRIME,
	.prime_handle_to_fd        = drm_gem_prime_handle_to_fd,
	.gem_prime_export          = drm_gem_prime_export,
	.gem_prime_get_sg_table    = xendrmmap_gem_prime_get_sg_table,
	.gem_close_object          = xendrmmap_gem_close_object,
	.gem_free_object_unlocked  = xendrmmap_gem_free_object,
	.fops                      = &xendrmmap_fops,
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
