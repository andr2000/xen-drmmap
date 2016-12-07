#ifndef __XEN_DRM_LOGS_H
#define __XEN_DRM_LOGS_H


#define LOG(log_level, fmt, ...) \
	do { \
		printk("xen-drmfront" #log_level " (%s:%d): " fmt "\n", \
			__FUNCTION__, __LINE__ , ## __VA_ARGS__); \
	} while (0)

#define LOG0(fmt, ...) LOG(0, fmt, ## __VA_ARGS__)

#endif /* __XEN_DRM_LOGS_H */
