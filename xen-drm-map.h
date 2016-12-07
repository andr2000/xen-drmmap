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

#include <drm/drmP.h>

/*
 * Xen DRM map specific ioctls.
 *
 * The device specific ioctl range is [DRM_COMMAND_BASE, DRM_COMMAND_END) i.e.
 * [0x40, 0xa0) (a0 is excluded). The numbers below are defined as offset
  * against DRM_COMMAND_BASE and should be between [0x0, 0x60).
  */
#define DRM_XENDRM_MAP		0x00
#define DRM_XENXEN_UNMAP	0x01

struct xendrmmap_ioctl_map {
	int dummy;
};

struct xendrmmap_ioctl_unmap {
	int dummy;
};

#define DRM_IOCTL_XENDRM_MAP	DRM_IOW(DRM_COMMAND_BASE + DRM_XENDRM_MAP, \
	struct xendrmmap_ioctl_map)
#define DRM_IOCTL_XENDRM_UNMAP	DRM_IOW(DRM_COMMAND_BASE + DRM_XENXEN_UNMAP, \
	struct xendrmmap_ioctl_unmap)

#endif /* __XEN_DRM_MAP_H*/

