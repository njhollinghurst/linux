/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * IOMMU driver for BCM2712
 *
 * Copyright (c) 2023 Raspberry Pi Ltd.
 */

#ifndef _BCM2712_IOMMU_H
#define _BCM2712_IOMMU_H

#include <linux/iommu.h>
#include <linux/scatterlist.h>

struct bcm2712_iommu_cache {
	struct device *dev;
	spinlock_t hw_lock; /* to protect HW registers */
	void __iomem *reg_base;
};

void bcm2712_iommu_cache_flush(struct bcm2712_iommu_cache *cache);

/* Number of chunks for lazy allocation of IOMMU L2 tables */
#define BCM2712_IOMMU_SEGMENTS 16

struct bcm2712_iommu {
	struct device *dev;
	struct iommu_device iommu;
	struct iommu_group *group;
	struct bcm2712_iommu_domain *domain;
	char const *name;
	struct bcm2712_iommu_cache *cache;
	struct sg_table *l1_default_sgt;
	u32 *l1_table;
	bool l1_dirty; /* true when table is coherent with CPU */
	struct sg_table *l2_sgts[BCM2712_IOMMU_SEGMENTS];
	u32 *l2_tables[BCM2712_IOMMU_SEGMENTS];
	u32 l2_dirty[BCM2712_IOMMU_SEGMENTS];
	spinlock_t hw_lock;   /* to protect HW registers */
	void __iomem *reg_base;
	u64 dma_iova_offset; /* Hack for IOMMU attached to PCIe RC */
	u32 bigpage_mask;
	u32 superpage_mask;
	unsigned int nmapped_pages;
};

struct bcm2712_iommu_domain {
	struct iommu_domain base;
	struct bcm2712_iommu *mmu;
};

#endif
