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

#include <xen/balloon.h>
#include <xen/grant_table.h>

#include "xen-drm-map.h"
#include "xen-drm-logs.h"


struct xen_info {
	struct drm_device *drm_dev;
};

struct xen_gem_object {
	struct drm_gem_object base;
	uint32_t dumb_handle;
	int size;
	/* these are pages from Xen balloon */
	struct page **pages;
	/* and their map grant handles */
	grant_handle_t *map_handles;
	/* Xen */
	uint32_t num_pages;
	grant_ref_t *grefs;
	int otherend_id;
};

static const char *gnttabop_error_msgs[] = GNTTABOP_error_msgs;

static const char *xen_gnttab_err_to_string(int16_t status)
{
	status = -status;
	if (status < 0 || status >= ARRAY_SIZE(gnttabop_error_msgs))
		return "bad status code";
	return gnttabop_error_msgs[status];
}

static inline struct xen_gem_object *
to_xen_gem_obj(struct drm_gem_object *gem_obj)
{
	return container_of(gem_obj, struct xen_gem_object, base);
}

#define xen_page_to_vaddr(page) ((phys_addr_t)pfn_to_kaddr(page_to_pfn(page)))

static int xen_do_map(struct xen_gem_object *xen_obj)
{
	struct gnttab_map_grant_ref *map_ops = NULL;
	int ret, i, size;

	if (xen_obj->pages) {
		DRM_ERROR("Mapping already mapped pages?\n");
		return -EINVAL;
	}
	DRM_DEBUG("++++++++++++ Allocating buffers\n");
	size = xen_obj->num_pages * sizeof(struct page *);
	xen_obj->pages = kzalloc(size, GFP_KERNEL);
	if (!xen_obj->pages) {
		ret = -ENOMEM;
		goto fail;
	}
	xen_obj->map_handles = kzalloc(size, GFP_KERNEL);
	if (!xen_obj->map_handles) {
		ret = -ENOMEM;
		goto fail;
	}
	map_ops = kzalloc(size, GFP_KERNEL);
	if (!map_ops) {
		ret = -ENOMEM;
		goto fail;
	}
	DRM_DEBUG("++++++++++++ Allocating %d ballooned pages\n",
		xen_obj->num_pages);
	ret = alloc_xenballooned_pages(xen_obj->num_pages, xen_obj->pages);
	if (ret < 0) {
		DRM_ERROR("++++++++++++ Cannot allocate %d ballooned pages, ret %d\n",
			xen_obj->num_pages, ret);
		goto fail;
	}
	DRM_DEBUG("++++++++++++ Setting GNTMAP_host_map|GNTMAP_device_map\n");
	for (i = 0; i < xen_obj->num_pages; i++) {
		phys_addr_t addr;

		/* Map the grant entry for access by I/O devices. */
		/* Map the grant entry for access by host CPUs. */
		addr = xen_page_to_vaddr(xen_obj->pages[i]);
		gnttab_set_map_op(&map_ops[i], addr,
			GNTMAP_host_map | GNTMAP_device_map,
			xen_obj->grefs[i], xen_obj->otherend_id);
	}
	DRM_DEBUG("++++++++++++ Mapping refs\n");
	ret = gnttab_map_refs(map_ops, NULL, xen_obj->pages,
		xen_obj->num_pages);
	BUG_ON(ret);
	for (i = 0; i < xen_obj->num_pages; i++) {
		xen_obj->map_handles[i] = map_ops[i].handle;
		if (unlikely(map_ops[i].status != GNTST_okay)) {
			DRM_ERROR("Failed to set map op for page %d, ref %d: %s (%d)\n",
				i, xen_obj->grefs[i],
				xen_gnttab_err_to_string(map_ops[i].status),
				map_ops[i].status);
		}
	}
	kfree(map_ops);
	return 0;
fail:
	if (xen_obj->pages)
		kfree(xen_obj->pages);
	xen_obj->pages = NULL;
	if (xen_obj->map_handles)
		kfree(xen_obj->map_handles);
	xen_obj->map_handles = NULL;
	if (map_ops)
		kfree(map_ops);
	return ret;

}

static int xen_do_unmap(struct xen_gem_object *xen_obj)
{
	struct gnttab_unmap_grant_ref *unmap_ops;
	int i, size;

	if (!xen_obj->pages || !xen_obj->map_handles)
		return 0;

	size = xen_obj->num_pages * sizeof(struct page *);
	unmap_ops = kzalloc(size, GFP_KERNEL);
	if (!unmap_ops)
		return -ENOMEM;
	DRM_DEBUG("++++++++++++ Setting GNTMAP_host_map|GNTMAP_device_map\n");
	for (i = 0; i < xen_obj->num_pages; i++) {
		phys_addr_t addr;

		/* Map the grant entry for access by I/O devices.
		 * Map the grant entry for access by host CPUs.
		 * If <host_addr> or <dev_bus_addr> is zero, that
		 * field is ignored. If non-zero, they must refer to
		 * a device/host mapping that is tracked by <handle>
		 */
		addr = xen_page_to_vaddr(xen_obj->pages[i]);
		gnttab_set_unmap_op(&unmap_ops[i], addr,
			GNTMAP_host_map | GNTMAP_device_map,
			xen_obj->map_handles[i]);
	}
	DRM_DEBUG("++++++++++++ Unmapping refs\n");
	BUG_ON(gnttab_unmap_refs(unmap_ops, NULL, xen_obj->pages,
		xen_obj->num_pages));

	DRM_DEBUG("++++++++++++ Freeing %d ballooned pages\n",
		xen_obj->num_pages);
	free_xenballooned_pages(xen_obj->num_pages, xen_obj->pages);
	kfree(xen_obj->pages);
	xen_obj->pages = NULL;
	kfree(xen_obj->map_handles);
	xen_obj->map_handles = NULL;
	kfree(unmap_ops);
	return 0;
}


static void xen_gem_close_object(struct drm_gem_object *gem_obj,
	struct drm_file *file_priv)
{
	struct xen_gem_object *xen_obj = to_xen_gem_obj(gem_obj);

	DRM_DEBUG("++++++++++++ Closing GEM object\n");
	xen_do_unmap(xen_obj);
}

static void xen_gem_free_object(struct drm_gem_object *gem_obj)
{
	struct xen_gem_object *xen_obj = to_xen_gem_obj(gem_obj);

	DRM_DEBUG("++++++++++++ Freeing GEM object\n");
	drm_gem_object_release(gem_obj);
	if (xen_obj->grefs)
		kfree(xen_obj->grefs);
	kfree(xen_obj);
}

static int xen_gem_create_with_handle(
	struct xen_gem_object *xen_obj, struct drm_file *file_priv,
	struct drm_device *dev)
{
	struct drm_gem_object *gem_obj;
	int ret;

	ret = drm_gem_object_init(dev, &xen_obj->base, xen_obj->size);
	if (ret < 0) {
		DRM_DEBUG("++++++++++++ Failed to initialize GEM, ret %d\n",
			ret);
		return ret;
	}
	gem_obj = &xen_obj->base;
	ret = drm_gem_create_mmap_offset(gem_obj);
	if (ret < 0) {
		drm_gem_object_release(gem_obj);
		return ret;
	}
	ret = drm_gem_handle_create(file_priv, gem_obj, &xen_obj->dumb_handle);
	DRM_ERROR("++++++++++++ Handle is %d, ret %d\n", xen_obj->dumb_handle, ret);
	/* drop reference from allocate - handle holds it now. */
	drm_gem_object_unreference_unlocked(gem_obj);
	return ret;
}

static int xendrm_create_dumb_obj(struct xen_gem_object *xen_obj,
	struct drm_device *dev, struct drm_file *file_priv)
{
	struct drm_gem_object *gem_obj;
	int ret;

	ret = xen_gem_create_with_handle(xen_obj, file_priv, dev);
	if (ret < 0)
		goto fail;
	gem_obj = drm_gem_object_lookup(file_priv, xen_obj->dumb_handle);
	if (!gem_obj) {
		DRM_ERROR("++++++++++++ Lookup for handle %d failed",
			xen_obj->dumb_handle);
		ret = -EINVAL;
		goto fail_destroy;
	}
	drm_gem_object_unreference_unlocked(gem_obj);
	return 0;

fail_destroy:
	drm_gem_dumb_destroy(file_priv, dev, xen_obj->dumb_handle);
fail:
	DRM_ERROR("++++++++++++ Failed to create dumb buffer, ret %d\n", ret);
	xen_obj->dumb_handle = 0;
	return ret;
}

static int xendrm_do_dumb_create(struct drm_device *dev,
	struct xendrmmap_ioctl_create_dumb *req,
	struct drm_file *file_priv)
{
	struct xen_gem_object *xen_obj;
	int sz, ret;

	xen_obj = kzalloc(sizeof(*xen_obj), GFP_KERNEL);
	if (!xen_obj)
		return -ENOMEM;
	DRM_DEBUG("++++++++++++ Creating DUMB\n");
	xen_obj->num_pages = req->num_grefs;
	xen_obj->otherend_id = req->otherend_id;
	xen_obj->size = req->dumb.size;

	sz = xen_obj->num_pages * sizeof(grant_ref_t);
	xen_obj->grefs = kzalloc(sz, GFP_KERNEL);
	if (!xen_obj->grefs) {
		ret = -ENOMEM;
		goto fail;
	}
	if (copy_from_user(xen_obj->grefs, req->grefs, sz)) {
		ret = -EINVAL;
		goto fail;
	}
	ret = xen_do_map(xen_obj);
	if (ret < 0)
		goto fail;
	ret = xendrm_create_dumb_obj(xen_obj, dev, file_priv);
	if (ret < 0)
		goto fail;
	/* return handle */
	req->dumb.handle = xen_obj->dumb_handle;
	return 0;

fail:
	if (xen_obj->grefs)
		kfree(xen_obj->grefs);
	xen_obj->grefs = NULL;
	return ret;
}

static int xen_create_dumb_ioctl(struct drm_device *dev,
	void *data, struct drm_file *file_priv)
{
	struct xendrmmap_ioctl_create_dumb *req =
		(struct xendrmmap_ioctl_create_dumb *)data;
	struct drm_mode_create_dumb *args = &req->dumb;
	uint32_t cpp, stride, size;

	if (!req->num_grefs || !req->grefs || !req->otherend_id)
		return -EINVAL;
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
	args->size = round_up(args->pitch * args->height, PAGE_SIZE);
	args->handle = 0;
	if (req->num_grefs < DIV_ROUND_UP(args->size, PAGE_SIZE)) {
		DRM_ERROR("++++++++++++ Provided %d pages, need %d\n",
			req->num_grefs,
			(int)DIV_ROUND_UP(args->size, PAGE_SIZE));
		return -EINVAL;
	}
	return xendrm_do_dumb_create(dev, req, file_priv);
}

static struct sg_table *xen_gem_prime_get_sg_table(
	struct drm_gem_object *gem_obj)
{
	struct xen_gem_object *xen_obj = to_xen_gem_obj(gem_obj);
	struct sg_table *sgt;

	if (unlikely(!xen_obj->pages))
		return NULL;
	/* N.B. there will be a single entry in the table if buffer
	 * is contiguous. otherwise CMA drivers will not accept
	 * the buffer
	 */
	sgt = drm_prime_pages_to_sg(xen_obj->pages, xen_obj->num_pages);
	if (unlikely(!sgt))
		DRM_DEBUG("++++++++++++ Failed to export sgt\n");
	else
		DRM_DEBUG("++++++++++++ Exporting %scontiguous buffer\n",
			sgt->nents == 1 ? "" : "non-");
	return sgt;
}

static const struct drm_ioctl_desc xen_ioctls[] = {
	DRM_IOCTL_DEF_DRV(XENDRM_CREATE_DUMB, xen_create_dumb_ioctl,
		DRM_AUTH | DRM_CONTROL_ALLOW | DRM_UNLOCKED),
};

static const struct file_operations xen_fops = {
	.owner          = THIS_MODULE,
	.open           = drm_open,
	.release        = drm_release,
	.unlocked_ioctl = drm_ioctl,
};

static struct drm_driver xen_driver = {
	.driver_features           = DRIVER_GEM | DRIVER_PRIME,
	.prime_handle_to_fd        = drm_gem_prime_handle_to_fd,
	.gem_prime_export          = drm_gem_prime_export,
	.gem_prime_get_sg_table    = xen_gem_prime_get_sg_table,
	.gem_close_object          = xen_gem_close_object,
	.gem_free_object_unlocked  = xen_gem_free_object,
	.fops                      = &xen_fops,
	.ioctls                    = xen_ioctls,
	.num_ioctls                = ARRAY_SIZE(xen_ioctls),
	.name                      = XENDRMMAP_DRIVER_NAME,
	.desc                      = "Xen PV DRM mapper",
	.date                      = "20161207",
	.major                     = 1,
	.minor                     = 0,
};

static int xen_remove(struct platform_device *pdev)
{
	struct xen_info *info = platform_get_drvdata(pdev);
	struct drm_device *drm_dev = info->drm_dev;

	drm_dev_unregister(drm_dev);
	drm_dev_unref(drm_dev);
	return 0;
}

static int xen_probe(struct platform_device *pdev)
{
	struct xen_info *info;
	struct drm_device *drm_dev;
	int ret;

	LOG0("Creating %s", xen_driver.desc);
	info = devm_kzalloc(&pdev->dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	drm_dev = drm_dev_alloc(&xen_driver, &pdev->dev);
	if (!drm_dev)
		return -ENOMEM;

	info->drm_dev = drm_dev;
	drm_dev->dev_private = info;
	platform_set_drvdata(pdev, info);

	ret = drm_dev_register(drm_dev, 0);
	if (ret)
		goto fail;

	DRM_INFO("Initialized %s %d.%d.%d %s on minor %d\n",
		xen_driver.name, xen_driver.major,
		xen_driver.minor, xen_driver.patchlevel,
		xen_driver.date, drm_dev->primary->index);
	return 0;

fail:
	xen_remove(pdev);
	return ret;
}

static struct platform_driver xen_ddrv_info = {
	.probe		= xen_probe,
	.remove		= xen_remove,
	.driver		= {
		.name	= XENDRMMAP_DRIVER_NAME,
	},
};

static struct platform_device *xen_pdev;

static int __init xen_init(void)
{
	int ret;

	xen_pdev = platform_device_alloc(XENDRMMAP_DRIVER_NAME, -1);
	if (!xen_pdev) {
		LOG0("Failed to allocate " XENDRMMAP_DRIVER_NAME \
			" device");
		return -ENODEV;
	}
	ret = platform_device_add(xen_pdev);
	if (ret != 0) {
		LOG0("Failed to register " XENDRMMAP_DRIVER_NAME \
			" device: %d\n", ret);
		platform_device_put(xen_pdev);
		return -ENODEV;
	}
	ret = platform_driver_register(&xen_ddrv_info);
	if (ret != 0) {
		LOG0("Failed to register " XENDRMMAP_DRIVER_NAME \
			" driver: %d\n", ret);
		platform_device_unregister(xen_pdev);
	}
	return 0;
}

static void __exit xen_cleanup(void)
{
	platform_driver_unregister(&xen_ddrv_info);
	if (xen_pdev)
		platform_device_unregister(xen_pdev);
}

module_init(xen_init);
module_exit(xen_cleanup);

MODULE_DESCRIPTION("Xen DRM buffer mapper");
MODULE_LICENSE("GPL");
