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
#include <drm/drm_gem.h>

#include <linux/dma-buf.h>
#include <linux/platform_device.h>

#ifdef CONFIG_XEN_HAVE_PVMMU
#include <xen/balloon.h>
#else
#include <xen/interface/memory.h>
#include <asm/xen/hypercall.h>
#include <xen/page.h>
#endif
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
#ifndef CONFIG_XEN_HAVE_PVMMU
	dma_addr_t paddr;
	void *vaddr;
#endif
	/* and their map grant handles */
	grant_handle_t *map_handles;
	uint64_t *dev_bus_addr;
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

#ifdef CONFIG_XEN_HAVE_PVMMU
/* FIXME: ARM platform has no concept of PVMMU,
 * so, most probably, drivers for ARM will require CMA
 */
static int xen_alloc_ballooned_pages(struct xen_gem_object *xen_obj)
{
	return alloc_xenballooned_pages(xen_obj->num_pages, xen_obj->pages);
}

static void xen_free_ballooned_pages(struct xen_gem_object *xen_obj)
{
	free_xenballooned_pages(xen_obj->num_pages, xen_obj->pages);
}
#else
static int xen_alloc_ballooned_pages(struct xen_gem_object *xen_obj)
{
	struct page **pages;
	xen_pfn_t *frame_list;
	int num_pages, i, tries_left;
	int ret;
	dma_addr_t paddr, cpu_addr;
	void *vaddr = NULL;
	size_t size;
	struct xen_memory_reservation reservation = {
		.address_bits = 0,
		.extent_order = 0,
		.domid        = DOMID_SELF
	};

	num_pages = xen_obj->num_pages;
	pages = xen_obj->pages;
	DRM_ERROR("Ballooning out %d pages", num_pages);

	frame_list = kcalloc(num_pages, sizeof(*frame_list), GFP_KERNEL);
	if (!frame_list)
		return -ENOMEM;
	size = num_pages * PAGE_SIZE;
	vaddr = dma_alloc_wc(xen_obj->base.dev->dev, size, &paddr,
		GFP_KERNEL | __GFP_NOWARN);
	if (!vaddr) {
		DRM_ERROR("Failed to allocate DMA buffer with size %zu\n", size);
		ret = -ENOMEM;
		goto fail;
	}
	cpu_addr = paddr;
	for (i = 0; i < num_pages; i++) {
		pages[i] = virt_to_page(cpu_addr);
		/* XENMEM_populate_physmap requires a PFN based on Xen
		 * granularity.
		 */
		frame_list[i] = page_to_xen_pfn(pages[i]);
		cpu_addr += PAGE_SIZE;
	}
	tries_left = 3;
again:
	set_xen_guest_handle(reservation.extent_start, frame_list);
	reservation.nr_extents = num_pages;
	/* rc will hold number of pages processed */
	ret = HYPERVISOR_memory_op(XENMEM_populate_physmap, &reservation);
	if (ret <= 0) {
		DRM_ERROR("Failed to balloon out %d pages, retrying", num_pages);
		if (--tries_left)
			goto again;
		WARN_ON(ret != num_pages);
		ret = -EFAULT;
		goto fail;
	}
	DRM_ERROR("Ballooned out %d pages", ret);
	xen_obj->vaddr = vaddr;
	xen_obj->paddr = paddr;
	kfree(frame_list);
	return 0;

fail:
	if (vaddr)
		dma_free_wc(xen_obj->base.dev->dev, size, vaddr, paddr);
	kfree(frame_list);
	return ret;
}

static void xen_free_ballooned_pages(struct xen_gem_object *xen_obj)
{
	struct page **pages;
	xen_pfn_t *frame_list;
	int num_pages, i;
	int ret;
	size_t size;
	struct xen_memory_reservation reservation = {
		.address_bits = 0,
		.extent_order = 0,
		.domid        = DOMID_SELF
	};

	num_pages = xen_obj->num_pages;
	pages = xen_obj->pages;
	if (!pages)
		return;
	if (!xen_obj->vaddr)
		return;
	frame_list = kcalloc(num_pages, sizeof(*frame_list), GFP_KERNEL);
	if (!frame_list) {
		DRM_ERROR("Failed to balloon in %d pages", num_pages);
		return;
	}
	DRM_ERROR("Ballooning in %d pages", num_pages);
	size = num_pages * PAGE_SIZE;
	for (i = 0; i < num_pages; i++) {
		/* XENMEM_populate_physmap requires a PFN based on Xen
		 * granularity.
		 */
		frame_list[i] = page_to_xen_pfn(pages[i]);
	}
	set_xen_guest_handle(reservation.extent_start, frame_list);
	reservation.nr_extents = num_pages;
	/* rc will hold number of pages processed */
	ret = HYPERVISOR_memory_op(XENMEM_decrease_reservation, &reservation);
	if (ret <= 0) {
		DRM_ERROR("Failed to balloon in %d pages", num_pages);
		WARN_ON(ret != num_pages);
	}
	DRM_ERROR("Ballooned in %d pages", ret);
	if (xen_obj->vaddr)
		dma_free_wc(xen_obj->base.dev->dev, size,
			xen_obj->vaddr, xen_obj->paddr);
	xen_obj->vaddr = NULL;
	xen_obj->paddr = 0;
	kfree(frame_list);
}
#endif

#define xen_page_to_vaddr(page) ((phys_addr_t)pfn_to_kaddr(page_to_xen_pfn(page)))

static int xen_do_map(struct xen_gem_object *xen_obj)
{
	struct gnttab_map_grant_ref *map_ops = NULL;
	int ret, i, size;

	if (xen_obj->pages) {
		DRM_ERROR("Mapping already mapped pages?\n");
		return -EINVAL;
	}
	DRM_DEBUG("++++++++++++ Allocating buffers\n");
	size = xen_obj->num_pages * sizeof(*(xen_obj->pages));
	xen_obj->pages = kzalloc(size, GFP_KERNEL);
	if (!xen_obj->pages) {
		ret = -ENOMEM;
		goto fail;
	}
	size = xen_obj->num_pages * sizeof(*(xen_obj->map_handles));
	xen_obj->map_handles = kzalloc(size, GFP_KERNEL);
	if (!xen_obj->map_handles) {
		ret = -ENOMEM;
		goto fail;
	}
	size = xen_obj->num_pages * sizeof(*(xen_obj->dev_bus_addr));
	xen_obj->dev_bus_addr = kzalloc(size, GFP_KERNEL);
	if (!xen_obj->dev_bus_addr) {
		ret = -ENOMEM;
		goto fail;
	}
	size = xen_obj->num_pages * sizeof(*map_ops);
	map_ops = kzalloc(size, GFP_KERNEL);
	if (!map_ops) {
		ret = -ENOMEM;
		goto fail;
	}
	DRM_DEBUG("++++++++++++ Allocating %d ballooned pages\n",
		xen_obj->num_pages);
	ret = xen_alloc_ballooned_pages(xen_obj);
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
		xen_obj->dev_bus_addr[i] = map_ops[i].dev_bus_addr;
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
	kfree(xen_obj->pages);
	xen_obj->pages = NULL;
	kfree(xen_obj->map_handles);
	xen_obj->map_handles = NULL;
	kfree(xen_obj->dev_bus_addr);
	xen_obj->dev_bus_addr = NULL;
	kfree(map_ops);
	return ret;

}

static int xen_do_unmap(struct xen_gem_object *xen_obj)
{
	struct gnttab_unmap_grant_ref *unmap_ops;
	int i, size;

	if (!xen_obj->pages || !xen_obj->map_handles)
		return 0;

	size = xen_obj->num_pages * sizeof(*unmap_ops);
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
		unmap_ops[i].dev_bus_addr = xen_obj->dev_bus_addr[i];
	}
	DRM_DEBUG("++++++++++++ Unmapping refs\n");
	BUG_ON(gnttab_unmap_refs(unmap_ops, NULL, xen_obj->pages,
		xen_obj->num_pages));

	DRM_DEBUG("++++++++++++ Freeing %d ballooned pages\n",
		xen_obj->num_pages);
	xen_free_ballooned_pages(xen_obj);
	kfree(xen_obj->pages);
	xen_obj->pages = NULL;
	kfree(xen_obj->map_handles);
	xen_obj->map_handles = NULL;
	kfree(xen_obj->dev_bus_addr);
	xen_obj->dev_bus_addr = NULL;
	kfree(unmap_ops);
	return 0;
}


static void xen_gem_close_object(struct drm_gem_object *gem_obj,
	struct drm_file *file_priv)
{
	struct xen_gem_object *xen_obj = to_xen_gem_obj(gem_obj);

	/* from drm_prime.c:
	 * On the export the dma_buf holds a reference to the exporting GEM
	 * object. It takes this reference in handle_to_fd_ioctl, when it
	 * first calls .prime_export and stores the exporting GEM object in
	 * the dma_buf priv. This reference is released when the dma_buf
	 * object goes away in the driver .release function.
	 * FIXME: this is too late, as we have to unmap now, so front
	 * can release granted references
	 * FIXME: if handle_count is 1 then dma_buf is not in use anymore
	 * waiting for the driver's .release. Otherwise it is a bug in
	 * the backend, e.g. the handle was not closed in the driver which
	 * imported our dma_buf
	 */
	mutex_lock(&gem_obj->dev->object_name_lock);
	DRM_DEBUG("++++++++++++ Closing GEM object handle %d, ref %d handle_count %d\n",
		xen_obj->dumb_handle, atomic_read(&gem_obj->refcount.refcount),
		gem_obj->handle_count);
	WARN_ON(gem_obj->handle_count != 1);
	if (gem_obj->handle_count == 1) {
		xen_do_unmap(xen_obj);
		kfree(xen_obj->grefs);
		xen_obj->grefs = NULL;
	}
	mutex_unlock(&gem_obj->dev->object_name_lock);
}

static void xen_gem_free_object(struct drm_gem_object *gem_obj)
{
	struct xen_gem_object *xen_obj = to_xen_gem_obj(gem_obj);

	/* FIXME: this gets called on driver .release because of
	 * .handle_to_fd_ioctl + .prime_export
	 */
	DRM_DEBUG("++++++++++++ Freeing GEM object\n");
	drm_gem_object_release(gem_obj);
	kfree(xen_obj);
}

static int xen_gem_create_with_handle(
	struct xen_gem_object *xen_obj, struct drm_file *file_priv,
	struct drm_device *dev)
{
	struct drm_gem_object *gem_obj;
	int ret;

	drm_gem_private_object_init(dev, &xen_obj->base, xen_obj->size);
	gem_obj = &xen_obj->base;
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
	xen_obj->size = round_up(req->dumb.size, PAGE_SIZE);

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
	DRM_DEBUG("++++++++++++ Create GEM object, ref %d\n",
		atomic_read(&xen_obj->base.refcount.refcount));
	return 0;

fail:
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
	args->size = args->pitch * args->height;
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

struct platform_device_info xen_ddrv_platform_info = {
	.name = XENDRMMAP_DRIVER_NAME,
	.id = 0,
	.num_res = 0,
	.dma_mask = DMA_BIT_MASK(32),
};

static struct platform_device *xen_pdev;

static int __init xen_init(void)
{
	int ret;

	xen_pdev = platform_device_register_full(&xen_ddrv_platform_info);
	if (!xen_pdev) {
		LOG0("Failed to register " XENDRMMAP_DRIVER_NAME \
			" device: %d\n", ret);
		return -ENODEV;
	}
	ret = platform_driver_register(&xen_ddrv_info);
	if (ret != 0) {
		LOG0("Failed to register " XENDRMMAP_DRIVER_NAME \
			" driver: %d\n", ret);
		platform_device_unregister(xen_pdev);
		return ret;
	}
	return 0;
}

static void __exit xen_cleanup(void)
{
	if (xen_pdev)
		platform_device_unregister(xen_pdev);
	platform_driver_unregister(&xen_ddrv_info);
}

module_init(xen_init);
module_exit(xen_cleanup);

MODULE_DESCRIPTION("Xen DRM buffer mapper");
MODULE_LICENSE("GPL");
