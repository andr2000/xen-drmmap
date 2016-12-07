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

#ifndef __XEN_DRM__BALLOON_H
#define __XEN_DRM__BALLOON_H

#include <linux/kernel.h>
#include <xen/interface/memory.h>

struct xen_drm_balloon {
	struct mutex lock;
	xen_pfn_t frame_list[PAGE_SIZE / sizeof(xen_pfn_t)];
};

int xendrm_balloon_init(struct xen_drm_balloon *balloon);
int xendrm_balloon_increase_reservation(struct xen_drm_balloon *balloon,
	int nr_pages, struct page **pages);
int xendrm_balloon_decrease_reservation(struct xen_drm_balloon *balloon,
	int nr_pages, struct page **pages);

#endif /* __XEN_DRM__BALLOON_H */
