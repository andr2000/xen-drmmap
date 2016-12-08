/*
 *  Xen para-virtual DRM device
 *
 *  Based on
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

#ifndef __XEN_DRM_MAP_H
#define __XEN_DRM_MAP_H

#define XENDRMMAP_DRIVER_NAME	"xen-drmmap"

/*
 * Xen DRM map specific ioctls.
 *
 * The device specific ioctl range is [DRM_COMMAND_BASE, DRM_COMMAND_END) i.e.
 * [0x40, 0xa0) (a0 is excluded). The numbers below are defined as offset
 * against DRM_COMMAND_BASE and should be between [0x0, 0x60).
 */
#define DRM_XENDRM_CREATE_DUMB	0x00

/* XXX: keep this structure properly aligned/padded */
struct xendrmmap_ioctl_create_dumb {
	/* Xen */
	uint32_t num_grefs;
	/* FIXME: user-space uses uint32_t instead of grant_ref_t
	 * for mapping
	 */
	uint32_t *grefs;
	uint64_t otherend_id;
	struct drm_mode_create_dumb dumb;
};

#define DRM_IOCTL_XENDRM_CREATE_DUMB	DRM_IOW(DRM_COMMAND_BASE + \
	DRM_XENDRM_CREATE_DUMB, struct xendrmmap_ioctl_create_dumb)

#endif /* __XEN_DRM_MAP_H*/
