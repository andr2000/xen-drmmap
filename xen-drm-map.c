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

#include <drm/drm_gem_cma_helper.h>

#include <linux/dma-buf.h>
#include <linux/platform_device.h>

#include <asm/xen/hypercall.h>
#include <xen/balloon.h>
#include <xen/grant_table.h>
#include <xen/interface/memory.h>
#include <xen/page.h>

#include "xen-drm-map.h"
#include "xen-drm-logs.h"


struct xen_gem_object {
	struct list_head list;
	struct drm_gem_cma_object *cma_obj;
	uint32_t dumb_handle;
	/* original dumb's addresses */
	dma_addr_t dumb_paddr;
	void *dumb_vaddr;
	struct sg_table *sgt;
	/* and their map grant handles */
	grant_handle_t *map_handles;
	/* Xen */
	uint32_t num_pages;
	grant_ref_t *grefs;
	int otherend_id;
};

static struct xen_info {
	struct drm_device *drm_dev;
	/* dumb buffers */
	struct mutex mutex;
	struct xen_gem_object *dumb_list;
} xen_info;

static struct xen_gem_object *xen_find_obj(struct xen_info *drv_info,
	struct drm_gem_cma_object *cma_obj)
{
	struct list_head *pos, *q;

	if (!drv_info->dumb_list)
		return NULL;
	list_for_each_safe(pos, q, &drv_info->dumb_list->list) {
		struct xen_gem_object *gem;

		gem = list_entry(pos, struct xen_gem_object, list);
		if (gem->cma_obj == cma_obj)
			return gem;
	}
	return NULL;
}

static const char *gnttabop_error_msgs[] = GNTTABOP_error_msgs;

static const char *xen_gnttab_err_to_string(int16_t status)
{
	status = -status;
	if (status < 0 || status >= ARRAY_SIZE(gnttabop_error_msgs))
		return "bad status code";
	return gnttabop_error_msgs[status];
}

#define xen_page_to_vaddr(page) ((phys_addr_t)pfn_to_kaddr(page_to_xen_pfn(page)))

static int xen_balloon_in(int nr_pages, struct page **pages)
{
	return 0;
}

static int xen_balloon_out_pages(int num_pages, struct page **pages)
{
	xen_pfn_t *frame_list;
	int i, tries_left;
	int ret;
	struct xen_memory_reservation reservation = {
		.address_bits = 0,
		.extent_order = 0,
		.domid        = DOMID_SELF
	};

	DRM_ERROR("Ballooning out %d pages", num_pages);

	frame_list = kcalloc(num_pages, sizeof(*frame_list), GFP_KERNEL);
	if (!frame_list)
		return -ENOMEM;
	for (i = 0; i < num_pages; i++) {
		/* XENMEM_populate_physmap requires a PFN based on Xen
		 * granularity.
		 */
		frame_list[i] = page_to_xen_pfn(pages[i]);
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
		goto out;
	} else {
		DRM_ERROR("Ballooned out %d pages", ret);
		ret = 0;
	}

#ifdef CONFIG_XEN_HAVE_PVMMU
	for (i = 0; i < num_pages; i++) {
		struct page *page = pages[i];
		/*
		 * We don't support PV MMU when Linux and Xen is using
		 * different page granularity.
		 */
		BUILD_BUG_ON(XEN_PAGE_SIZE != PAGE_SIZE);

		if (!xen_feature(XENFEAT_auto_translated_physmap)) {
			unsigned long pfn = page_to_pfn(page);

			xen_set_phys_to_machine(pfn, frame_list[i]);

			/* Link back into the page tables if not highmem. */
			if (!PageHighMem(page)) {
				int ret;
				ret = HYPERVISOR_update_va_mapping(
						(unsigned long)__va(pfn << PAGE_SHIFT),
						mfn_pte(frame_list[i], PAGE_KERNEL),
						0);
				BUG_ON(ret);
			}
		}
	}
#endif
	/* Do not relinquish pages back to the allocator
	 * as after un-mapping we do need this memory
	 */
out:
	kfree(frame_list);
	return ret;
}

static int xen_balloon_out(int num_pages, struct page **pages)
{
	int ret, i;

	ret = xen_balloon_out_pages(num_pages, pages);
	if (ret < 0)
		return -EFAULT;
	for (i = 0; i < num_pages; i++) {
#ifdef CONFIG_XEN_HAVE_PVMMU
		/*
		 * We don't support PV MMU when Linux and Xen is using
		 * different page granularity.
		 */
		BUILD_BUG_ON(XEN_PAGE_SIZE != PAGE_SIZE);

		ret = xen_alloc_p2m_entry(page_to_pfn(pages[i]));
		if (ret < 0)
			goto out_undo;
#endif
	}
	return 0;

out_undo:
	xen_balloon_in(num_pages, pages);
	return ret;
}

static int xen_do_map(struct xen_gem_object *xen_obj)
{
	struct gnttab_map_grant_ref *map_ops = NULL;
	struct page **pages = NULL;
	struct sg_page_iter sg_iter;
	int ret, i, size;

	DRM_DEBUG("++++++++++++ Allocating buffers\n");
	size = xen_obj->num_pages * sizeof(*(xen_obj->map_handles));
	xen_obj->map_handles = kzalloc(size, GFP_KERNEL);
	if (!xen_obj->map_handles) {
		ret = -ENOMEM;
		goto fail;
	}
	size = xen_obj->num_pages * sizeof(*map_ops);
	map_ops = kzalloc(size, GFP_KERNEL);
	if (!map_ops) {
		ret = -ENOMEM;
		goto fail;
	}
	size = xen_obj->num_pages * sizeof(*pages);
	pages = kzalloc(size, GFP_KERNEL);
	if (!pages) {
		ret = -ENOMEM;
		goto fail;
	}
	DRM_DEBUG("++++++++++++ Setting GNTMAP_host_map|GNTMAP_device_map\n");
	i = 0;
	for_each_sg_page(xen_obj->sgt->sgl, &sg_iter, xen_obj->sgt->nents, 0) {
		phys_addr_t addr;

		pages[i] = sg_page_iter_page(&sg_iter);
		/* Map the grant entry for access by I/O devices. */
		/* Map the grant entry for access by host CPUs. */
		addr = xen_page_to_vaddr(pages[i]);
		gnttab_set_map_op(&map_ops[i], addr,
			GNTMAP_host_map | GNTMAP_device_map,
			xen_obj->grefs[i], xen_obj->otherend_id);
		i++;
	}
	DRM_DEBUG("++++++++++++ Ballooning out %d pages\n",
		xen_obj->num_pages);
	ret = xen_balloon_out(xen_obj->num_pages, pages);
	if (ret < 0) {
		DRM_ERROR("Failed to balloon out %d pages\n",
			xen_obj->num_pages);
		ret = -EFAULT;
		goto fail;
	}
	DRM_DEBUG("++++++++++++ Mapping refs for %d pages\n",
		xen_obj->num_pages);
	ret = gnttab_map_refs(map_ops, NULL, pages,
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

	/* this is what it was all about: substitute addresses of
	 * the CMA object, so it points to the mapped buffer
	 */
	xen_obj->cma_obj->paddr = map_ops[0].dev_bus_addr;
	xen_obj->cma_obj->vaddr = (void *)map_ops[0].host_addr;

	kfree(map_ops);
	kfree(pages);
	return 0;
fail:
	if (xen_obj->map_handles)
		kfree(xen_obj->map_handles);
	xen_obj->map_handles = NULL;
	if (map_ops)
		kfree(map_ops);
	if (pages)
		kfree(pages);
	return ret;

}
static int xen_do_unmap(struct xen_gem_object *xen_obj)
{
	struct gnttab_unmap_grant_ref *unmap_ops = NULL;
	struct page **pages = NULL;
	struct sg_page_iter sg_iter;
	int ret, i, size;

	if (!xen_obj->map_handles)
		return 0;

	size = xen_obj->num_pages * sizeof(*unmap_ops);
	unmap_ops = kzalloc(size, GFP_KERNEL);
	if (!unmap_ops) {
		ret = -ENOMEM;
		goto fail;
	}
	size = xen_obj->num_pages * sizeof(*pages);
	pages = kzalloc(size, GFP_KERNEL);
	if (!pages) {
		ret = -ENOMEM;
		goto fail;
	}
	DRM_DEBUG("++++++++++++ Setting GNTMAP_host_map|GNTMAP_device_map\n");
	i = 0;
	for_each_sg_page(xen_obj->sgt->sgl, &sg_iter, xen_obj->sgt->nents, 0) {
		phys_addr_t addr;

		pages[i] = sg_page_iter_page(&sg_iter);
		/* Map the grant entry for access by I/O devices.
		 * Map the grant entry for access by host CPUs.
		 * If <host_addr> or <dev_bus_addr> is zero, that
		 * field is ignored. If non-zero, they must refer to
		 * a device/host mapping that is tracked by <handle>
		 */
		addr = xen_page_to_vaddr(pages[i]);
		gnttab_set_unmap_op(&unmap_ops[i], addr,
			GNTMAP_host_map | GNTMAP_device_map,
			xen_obj->map_handles[i]);
	}
	DRM_DEBUG("++++++++++++ Unmapping refs for %d pages\n", i);
	BUG_ON(gnttab_unmap_refs(unmap_ops, NULL, pages, xen_obj->num_pages));

	/* return the pointers back */
	xen_obj->cma_obj->paddr = xen_obj->dumb_paddr;
	xen_obj->cma_obj->vaddr = xen_obj->dumb_vaddr;

	DRM_DEBUG("++++++++++++ Ballooning in %d\n", i);
	ret = xen_balloon_in(i, pages);
	if (ret < 0) {
		DRM_ERROR("Failed to balloon in %d pages\n", i);
		ret = -EFAULT;
		goto fail;
	}

	kfree(xen_obj->map_handles);
	xen_obj->map_handles = NULL;
	kfree(unmap_ops);
	kfree(pages);
	return 0;

fail:
	if (xen_obj->map_handles)
		kfree(xen_obj->map_handles);
	xen_obj->map_handles = NULL;
	if (unmap_ops)
		kfree(unmap_ops);
	if (pages)
		kfree(pages);
	return ret;
}

static void xen_gem_close_object(struct drm_gem_object *gem_obj,
	struct drm_file *file_priv)
{
	struct drm_gem_cma_object *cma_obj = to_drm_gem_cma_obj(gem_obj);
	struct xen_gem_object *xen_obj;

	DRM_DEBUG("++++++++++++ Closing GEM object\n");
	mutex_lock(&xen_info.mutex);
	xen_obj = xen_find_obj(&xen_info, cma_obj);
	if (!xen_obj) {
		DRM_ERROR("++++++++++++ Cannot find Xen object, handle\n");
		mutex_unlock(&xen_info.mutex);
		return;
	}
	xen_do_unmap(xen_obj);
	mutex_unlock(&xen_info.mutex);
}

static void xen_gem_free_object(struct drm_gem_object *gem_obj)
{
	struct drm_gem_cma_object *cma_obj = to_drm_gem_cma_obj(gem_obj);
	struct xen_info *drv_info = &xen_info;
	struct xen_gem_object *xen_obj;


	cma_obj = to_drm_gem_cma_obj(gem_obj);
	DRM_DEBUG("++++++++++++ Freeing GEM object\n");
	mutex_lock(&drv_info->mutex);
	xen_obj = xen_find_obj(drv_info, cma_obj);
	if (!xen_obj) {
		DRM_ERROR("++++++++++++ Cannot find Xen object\n");
		mutex_unlock(&drv_info->mutex);
		return;
	}
	drm_gem_cma_free_object(gem_obj);
	list_del(&xen_obj->list);
	if (drv_info->dumb_list == xen_obj)
		drv_info->dumb_list = NULL;
	if (xen_obj->grefs)
		kfree(xen_obj->grefs);
	if (xen_obj->sgt)
		sg_free_table(xen_obj->sgt);
	kfree(xen_obj);
	mutex_unlock(&xen_info.mutex);
}

static int xendrm_map_dumb(struct drm_device *dev, struct xen_info *drv_info,
	struct xendrmmap_ioctl_map_dumb *req,
	struct drm_gem_cma_object *cma_obj)
{
	struct xen_gem_object *xen_obj;
	int sz, ret;

	xen_obj = kzalloc(sizeof(*xen_obj), GFP_KERNEL);
	if (!xen_obj)
		return -ENOMEM;
	xen_obj->cma_obj = cma_obj;
	xen_obj->num_pages = req->num_grefs;
	xen_obj->otherend_id = req->otherend_id;
	xen_obj->dumb_handle = req->handle;
	sz = xen_obj->num_pages * sizeof(grant_ref_t);
	xen_obj->grefs = kzalloc(sz, GFP_KERNEL);
	if (!xen_obj->grefs) {
		ret = -ENOMEM;
		goto fail;
	}
	xen_obj->sgt = drm_gem_cma_prime_get_sg_table(&cma_obj->base);
	if (!xen_obj->sgt) {
		DRM_ERROR("Cannot allocate sg table\n");
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

	if (!drv_info->dumb_list) {
		drv_info->dumb_list = xen_obj;
		INIT_LIST_HEAD(&xen_obj->list);
	}
	list_add(&(xen_obj->list), &(drv_info->dumb_list->list));
	return 0;

fail:
	if (xen_obj->grefs)
		kfree(xen_obj->grefs);
	if (xen_obj->sgt)
		sg_free_table(xen_obj->sgt);
	if (xen_obj)
		kfree(xen_obj);
	return ret;
}

static int xen_map_dumb_ioctl(struct drm_device *dev,
	void *data, struct drm_file *file_priv)
{
	struct xendrmmap_ioctl_map_dumb *req =
		(struct xendrmmap_ioctl_map_dumb *)data;
	struct drm_gem_object *gem_obj;
	struct drm_gem_cma_object *cma_obj;
	struct xen_gem_object *xen_obj;
	int ret;

	if (!req->num_grefs || !req->grefs || !req->otherend_id)
		return -EINVAL;
	gem_obj = drm_gem_object_lookup(file_priv, req->handle);
	if (!gem_obj) {
		DRM_ERROR("++++++++++++ Failed to lookup GEM object, handle %d\n",
			req->handle);
		return -EINVAL;
	}
	drm_gem_object_unreference_unlocked(gem_obj);
	cma_obj = to_drm_gem_cma_obj(gem_obj);
	if (!cma_obj) {
		DRM_ERROR("++++++++++++ Failed to lookup GEM CMA object, handle %d\n",
			req->handle);
		return -EINVAL;
	}
	mutex_lock(&xen_info.mutex);
	xen_obj = xen_find_obj(&xen_info, cma_obj);
	if (xen_obj) {
		DRM_ERROR("++++++++++++ Already mapped, handle %d\n",
			req->handle);
		mutex_unlock(&xen_info.mutex);
		return -EINVAL;
	}
	ret = xendrm_map_dumb(dev, &xen_info, req, cma_obj);
	mutex_unlock(&xen_info.mutex);
	return ret;
}

static const struct drm_ioctl_desc xen_ioctls[] = {
	DRM_IOCTL_DEF_DRV(XENDRM_MAP_DUMB, xen_map_dumb_ioctl,
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
	.gem_prime_get_sg_table    = drm_gem_cma_prime_get_sg_table,
	.gem_close_object          = xen_gem_close_object,
	.gem_free_object_unlocked  = xen_gem_free_object,
	.dumb_create               = drm_gem_cma_dumb_create,
	.dumb_map_offset           = drm_gem_cma_dumb_map_offset,
	.dumb_destroy              = drm_gem_dumb_destroy,
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
	info = &xen_info;
	if (!info)
		return -ENOMEM;

	drm_dev = drm_dev_alloc(&xen_driver, &pdev->dev);
	if (!drm_dev)
		return -ENOMEM;

	mutex_init(&info->mutex);
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

	ret = platform_driver_register(&xen_ddrv_info);
	if (ret != 0) {
		LOG0("Failed to register " XENDRMMAP_DRIVER_NAME \
			" driver: %d\n", ret);
		platform_device_unregister(xen_pdev);
		return ret;
	}
	xen_pdev = platform_device_register_full(&xen_ddrv_platform_info);
	if (!xen_pdev) {
		LOG0("Failed to register " XENDRMMAP_DRIVER_NAME \
			" device: %d\n", ret);
		platform_device_unregister(xen_pdev);
		return -ENODEV;
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
