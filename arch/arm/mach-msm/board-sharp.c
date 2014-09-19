/* Copyright (c) 2011-2012, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/i2c/sx150x.h>
#include <linux/i2c/isl9519.h>
#include <linux/gpio.h>
#include <linux/msm_ssbi.h>
#include <linux/regulator/msm-gpio-regulator.h>
#include <linux/mfd/pm8xxx/pm8921.h>
#include <linux/mfd/pm8xxx/pm8xxx-adc.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <linux/slimbus/slimbus.h>
#include <linux/bootmem.h>
#include <linux/msm_kgsl.h>
#ifdef CONFIG_ANDROID_PMEM
#include <linux/android_pmem.h>
#endif
#ifndef CONFIG_SHTPS_TMA4XX_DEV
#include <linux/cyttsp-qc.h>
#endif  /* CONFIG_SHTPS_TMA4XX_DEV */
#include <linux/dma-mapping.h>
#include <linux/platform_data/qcom_crypto_device.h>
#include <linux/platform_data/qcom_wcnss_device.h>
#include <linux/leds.h>
#include <linux/leds-pm8xxx.h>
#include <linux/i2c/atmel_mxt_ts.h>
#include <linux/msm_tsens.h>
#include <linux/ks8851.h>
#include <linux/i2c/isa1200.h>
#include <linux/memory.h>
#include <linux/memblock.h>
#include <linux/msm_thermal.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/setup.h>
#include <asm/hardware/gic.h>
#include <asm/mach/mmc.h>

#include <mach/board.h>
#include <mach/msm_tspp.h>
#include <mach/msm_iomap.h>
#include <mach/msm_spi.h>
#include <mach/msm_serial_hs.h>
#ifdef CONFIG_USB_MSM_OTG_72K
#include <mach/msm_hsusb.h>
#else
#include <linux/usb/msm_hsusb.h>
#endif
#include <linux/usb/android.h>
#include <mach/usbdiag.h>
#include <mach/socinfo.h>
#include <mach/rpm.h>
#include <mach/gpiomux.h>
#include <mach/msm_bus_board.h>
#include <mach/msm_memtypes.h>
#include <mach/dma.h>
#include <mach/msm_dsps.h>
#include <mach/msm_xo.h>
#include <mach/restart.h>

#ifdef CONFIG_WCD9310_CODEC
#include <linux/mfd/wcd9xxx/core.h>
#include <linux/mfd/wcd9xxx/pdata.h>
#endif

#include <linux/smsc3503.h>
#include <linux/ion.h>
/* ADD_S shmds GSBI12 from DSPS to Krait */
#include <linux/bug.h>
/* ADD_E shmds GSBI12 from DSPS to Krait */
#include <mach/ion.h>
#include <mach/mdm2.h>
#include <mach/mdm-peripheral.h>
#include <mach/msm_rtb.h>
#include <mach/msm_cache_dump.h>
#include <mach/scm.h>
#include <mach/iommu_domains.h>

#include <linux/fmem.h>

#ifdef CONFIG_SHFELICA
#include <sharp/felica_cen.h>
#endif /* CONFIG_SHFELICA */

#include "timer.h"
#include "devices.h"
#include "devices-msm8x60.h"
#include "spm.h"
#include "board-8960.h"
#include "pm.h"
#include <mach/cpuidle.h>
#include "rpm_resources.h"
#include <mach/mpm.h>
#include "smd_private.h"
#include "pm-boot.h"
#include "msm_watchdog.h"
#ifdef CONFIG_SH_AUDIO_DRIVER /*02-141*/
#include <sharp/shspamp.h>
#endif /* CONFIG_SH_AUDIO_DRIVER *//*02-141*/

#if defined(CONFIG_TOUCHSCREEN_SHTPS)
    #include <sharp/shtps_dev.h>
#endif /* CONFIG_TOUCHSCREEN_SHTPS */
#ifdef CONFIG_MPU_SENSORS_MPU3050
#include <sharp/mpu.h>
#endif	/* CONFIG_MPU_SENSORS_MPU3050 */
#ifdef CONFIG_SENSORS_AMI603
#include <sharp/ami_hw.h>
#endif  /* CONFIG_SENSORS_AMI603 */
#ifdef CONFIG_SHTPS_TMA4XX_DEV
#include <sharp/sh_smem.h>
#endif  /* CONFIG_SHTPS_TMA4XX_DEV */

static struct platform_device msm_fm_platform_init = {
	.name = "iris_fm",
	.id   = -1,
};

#define KS8851_RST_GPIO		89
#define KS8851_IRQ_GPIO		90
#define HAP_SHIFT_LVL_OE_GPIO	47

#define MHL_GPIO_INT            4
#define MHL_GPIO_RESET          15

#if defined(CONFIG_GPIO_SX150X) || defined(CONFIG_GPIO_SX150X_MODULE)

struct sx150x_platform_data msm8960_sx150x_data[] = {
	[SX150X_CAM] = {
		.gpio_base         = GPIO_CAM_EXPANDER_BASE,
		.oscio_is_gpo      = false,
		.io_pullup_ena     = 0x0,
		.io_pulldn_ena     = 0xc0,
		.io_open_drain_ena = 0x0,
		.irq_summary       = -1,
	},
	[SX150X_LIQUID] = {
		.gpio_base         = GPIO_LIQUID_EXPANDER_BASE,
		.oscio_is_gpo      = false,
		.io_pullup_ena     = 0x0c08,
		.io_pulldn_ena     = 0x4060,
		.io_open_drain_ena = 0x000c,
		.io_polarity       = 0,
		.irq_summary       = -1,
	},
};

#endif

#if defined(CONFIG_MACH_LYNX_DL12) || defined(CONFIG_MACH_DECKARD_AS38) || defined(CONFIG_MACH_LYNX_DL10) || defined(CONFIG_MACH_LYNX_DL15) || defined(CONFIG_MACH_LYNX_DL18)
#define MSM_PMEM_ADSP_SIZE         0x6700000
#else
#define MSM_PMEM_ADSP_SIZE         0x7800000
#endif
#define MSM_PMEM_AUDIO_SIZE        0x4CF000
#if defined(CONFIG_MACH_LYNX_DL12) || defined(CONFIG_MACH_DECKARD_AS38) || defined(CONFIG_MACH_LYNX_DL10) || defined(CONFIG_MACH_LYNX_DL15) || defined(CONFIG_MACH_LYNX_DL18)
#define MSM_PMEM_SIZE 0x5640000
#else
#define MSM_PMEM_SIZE 0x2800000 /* 40 Mbytes */
#endif
#define MSM_LIQUID_PMEM_SIZE 0x4000000 /* 64 Mbytes */
#define MSM_HDMI_PRIM_PMEM_SIZE 0x4000000 /* 64 Mbytes */

#ifdef CONFIG_MSM_MULTIMEDIA_USE_ION
#define HOLE_SIZE      0x100000 /* 1 MB */
#define MSM_PMEM_KERNEL_EBI1_SIZE  0x65000
#ifdef CONFIG_MSM_IOMMU
#define MSM_ION_MM_SIZE            0x3800000 /* Need to be multiple of 64K */
#define MSM_ION_SF_SIZE            0x0
#define MSM_ION_QSECOM_SIZE        0x780000 /* (7.5MB) */
#define MSM_ION_HEAP_NUM	7
#else
#define MSM_ION_MM_SIZE            MSM_PMEM_ADSP_SIZE
#define MSM_ION_SF_SIZE            MSM_PMEM_SIZE
#define MSM_ION_QSECOM_SIZE        0x600000 /* (6MB) */
#define MSM_ION_HEAP_NUM	8
#endif
#define MSM_ION_MM_FW_SIZE  0x200000 /* 2 MB */
#define MSM_ION_MFC_SIZE	SZ_8K
#define MSM_ION_AUDIO_SIZE	MSM_PMEM_AUDIO_SIZE

#define MSM_LIQUID_ION_MM_SIZE (MSM_ION_MM_SIZE + 0x600000)
#define MSM_LIQUID_ION_SF_SIZE MSM_LIQUID_PMEM_SIZE
#define MSM_HDMI_PRIM_ION_SF_SIZE MSM_HDMI_PRIM_PMEM_SIZE

#define MSM_MM_FW_SIZE          (0x200000) /* 2mb */
#define MSM8960_FIXED_AREA_START (0xb0000000 - MSM_ION_MM_FW_SIZE)
#define MAX_FIXED_AREA_SIZE	0x10000000
#define MSM8960_FW_START	MSM8960_FIXED_AREA_START

static unsigned msm_ion_sf_size = MSM_ION_SF_SIZE;
#else
#define MSM_PMEM_KERNEL_EBI1_SIZE  0x110C000
#define MSM_ION_HEAP_NUM	1
#endif

#ifdef CONFIG_KERNEL_PMEM_EBI_REGION
static unsigned pmem_kernel_ebi1_size = MSM_PMEM_KERNEL_EBI1_SIZE;
static int __init pmem_kernel_ebi1_size_setup(char *p)
{
	pmem_kernel_ebi1_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_kernel_ebi1_size", pmem_kernel_ebi1_size_setup);
#endif

#ifdef CONFIG_ANDROID_PMEM
static unsigned pmem_size = MSM_PMEM_SIZE;
static unsigned pmem_param_set;
static int __init pmem_size_setup(char *p)
{
	pmem_size = memparse(p, NULL);
	pmem_param_set = 1;
	return 0;
}
early_param("pmem_size", pmem_size_setup);

static unsigned pmem_adsp_size = MSM_PMEM_ADSP_SIZE;

static int __init pmem_adsp_size_setup(char *p)
{
	pmem_adsp_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_adsp_size", pmem_adsp_size_setup);

static unsigned pmem_audio_size = MSM_PMEM_AUDIO_SIZE;

static int __init pmem_audio_size_setup(char *p)
{
	pmem_audio_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_audio_size", pmem_audio_size_setup);
#endif

#ifdef CONFIG_ANDROID_PMEM
#ifndef CONFIG_MSM_MULTIMEDIA_USE_ION
static struct android_pmem_platform_data android_pmem_pdata = {
	.name = "pmem",
	.allocator_type = PMEM_ALLOCATORTYPE_ALLORNOTHING,
	.cached = 1,
	.memory_type = MEMTYPE_EBI1,
};

static struct platform_device msm8960_android_pmem_device = {
	.name = "android_pmem",
	.id = 0,
	.dev = {.platform_data = &android_pmem_pdata},
};

static struct android_pmem_platform_data android_pmem_adsp_pdata = {
	.name = "pmem_adsp",
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 0,
	.memory_type = MEMTYPE_EBI1,
};
static struct platform_device msm8960_android_pmem_adsp_device = {
	.name = "android_pmem",
	.id = 2,
	.dev = { .platform_data = &android_pmem_adsp_pdata },
};

static struct android_pmem_platform_data android_pmem_audio_pdata = {
	.name = "pmem_audio",
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 0,
	.memory_type = MEMTYPE_EBI1,
};

static struct platform_device msm8960_android_pmem_audio_device = {
	.name = "android_pmem",
	.id = 4,
	.dev = { .platform_data = &android_pmem_audio_pdata },
};
#endif /*CONFIG_MSM_MULTIMEDIA_USE_ION*/
#endif /*CONFIG_ANDROID_PMEM*/

struct fmem_platform_data msm8960_fmem_pdata = {
};

#define DSP_RAM_BASE_8960 0x8da00000
#define DSP_RAM_SIZE_8960 0x1800000
static int dspcrashd_pdata_8960 = 0xDEADDEAD;

static struct resource resources_dspcrashd_8960[] = {
	{
		.name   = "msm_dspcrashd",
		.start  = DSP_RAM_BASE_8960,
		.end    = DSP_RAM_BASE_8960 + DSP_RAM_SIZE_8960,
		.flags  = IORESOURCE_DMA,
	},
};

static struct platform_device msm_device_dspcrashd_8960 = {
	.name           = "msm_dspcrashd",
	.num_resources  = ARRAY_SIZE(resources_dspcrashd_8960),
	.resource       = resources_dspcrashd_8960,
	.dev = { .platform_data = &dspcrashd_pdata_8960 },
};

static struct memtype_reserve msm8960_reserve_table[] __initdata = {
	[MEMTYPE_SMI] = {
	},
	[MEMTYPE_EBI0] = {
		.flags	=	MEMTYPE_FLAGS_1M_ALIGN,
	},
	[MEMTYPE_EBI1] = {
		.flags	=	MEMTYPE_FLAGS_1M_ALIGN,
	},
};

static void __init reserve_rtb_memory(void)
{
#if defined(CONFIG_MSM_RTB)
	msm8960_reserve_table[MEMTYPE_EBI1].size += msm8960_rtb_pdata.size;
#endif
}

static void __init size_pmem_devices(void)
{
#ifdef CONFIG_ANDROID_PMEM
#ifndef CONFIG_MSM_MULTIMEDIA_USE_ION
	android_pmem_adsp_pdata.size = pmem_adsp_size;

	if (!pmem_param_set) {
		if (machine_is_msm8960_liquid())
			pmem_size = MSM_LIQUID_PMEM_SIZE;
		if (msm8960_hdmi_as_primary_selected())
			pmem_size = MSM_HDMI_PRIM_PMEM_SIZE;
	}

	android_pmem_pdata.size = pmem_size;
	android_pmem_audio_pdata.size = MSM_PMEM_AUDIO_SIZE;
#endif /*CONFIG_MSM_MULTIMEDIA_USE_ION*/
#endif /*CONFIG_ANDROID_PMEM*/
}

#ifdef CONFIG_ANDROID_PMEM
#ifndef CONFIG_MSM_MULTIMEDIA_USE_ION
static void __init reserve_memory_for(struct android_pmem_platform_data *p)
{
	msm8960_reserve_table[p->memory_type].size += p->size;
}
#endif /*CONFIG_MSM_MULTIMEDIA_USE_ION*/
#endif /*CONFIG_ANDROID_PMEM*/

static void __init reserve_pmem_memory(void)
{
#ifdef CONFIG_ANDROID_PMEM
#ifndef CONFIG_MSM_MULTIMEDIA_USE_ION
	reserve_memory_for(&android_pmem_adsp_pdata);
	reserve_memory_for(&android_pmem_pdata);
	reserve_memory_for(&android_pmem_audio_pdata);
#endif
	msm8960_reserve_table[MEMTYPE_EBI1].size += pmem_kernel_ebi1_size;
#endif
}

static int msm8960_paddr_to_memtype(unsigned int paddr)
{
	return MEMTYPE_EBI1;
}

#define FMEM_ENABLED 0

#ifdef CONFIG_ION_MSM
#ifdef CONFIG_MSM_MULTIMEDIA_USE_ION
static struct ion_cp_heap_pdata cp_mm_msm8960_ion_pdata = {
	.permission_type = IPT_TYPE_MM_CARVEOUT,
	.align = SZ_64K,
	.reusable = FMEM_ENABLED,
	.mem_is_fmem = FMEM_ENABLED,
	.fixed_position = FIXED_MIDDLE,
	.iommu_map_all = 1,
	.iommu_2x_map_domain = VIDEO_DOMAIN,
};

static struct ion_cp_heap_pdata cp_mfc_msm8960_ion_pdata = {
	.permission_type = IPT_TYPE_MFC_SHAREDMEM,
	.align = PAGE_SIZE,
	.reusable = 0,
	.mem_is_fmem = FMEM_ENABLED,
	.fixed_position = FIXED_HIGH,
};

static struct ion_co_heap_pdata co_msm8960_ion_pdata = {
	.adjacent_mem_id = INVALID_HEAP_ID,
	.align = PAGE_SIZE,
	.mem_is_fmem = 0,
};

static struct ion_co_heap_pdata fw_co_msm8960_ion_pdata = {
	.adjacent_mem_id = ION_CP_MM_HEAP_ID,
	.align = SZ_128K,
	.mem_is_fmem = FMEM_ENABLED,
	.fixed_position = FIXED_LOW,
};
#endif

/**
 * These heaps are listed in the order they will be allocated. Due to
 * video hardware restrictions and content protection the FW heap has to
 * be allocated adjacent (below) the MM heap and the MFC heap has to be
 * allocated after the MM heap to ensure MFC heap is not more than 256MB
 * away from the base address of the FW heap.
 * However, the order of FW heap and MM heap doesn't matter since these
 * two heaps are taken care of by separate code to ensure they are adjacent
 * to each other.
 * Don't swap the order unless you know what you are doing!
 */
static struct ion_platform_data msm8960_ion_pdata = {
	.nr = MSM_ION_HEAP_NUM,
	.heaps = {
		{
			.id	= ION_SYSTEM_HEAP_ID,
			.type	= ION_HEAP_TYPE_SYSTEM,
			.name	= ION_VMALLOC_HEAP_NAME,
		},
#ifdef CONFIG_MSM_MULTIMEDIA_USE_ION
		{
			.id	= ION_CP_MM_HEAP_ID,
			.type	= ION_HEAP_TYPE_CP,
			.name	= ION_MM_HEAP_NAME,
			.size	= MSM_ION_MM_SIZE,
			.memory_type = ION_EBI_TYPE,
			.extra_data = (void *) &cp_mm_msm8960_ion_pdata,
		},
		{
			.id	= ION_MM_FIRMWARE_HEAP_ID,
			.type	= ION_HEAP_TYPE_CARVEOUT,
			.name	= ION_MM_FIRMWARE_HEAP_NAME,
			.size	= MSM_ION_MM_FW_SIZE,
			.memory_type = ION_EBI_TYPE,
			.extra_data = (void *) &fw_co_msm8960_ion_pdata,
		},
		{
			.id	= ION_CP_MFC_HEAP_ID,
			.type	= ION_HEAP_TYPE_CP,
			.name	= ION_MFC_HEAP_NAME,
			.size	= MSM_ION_MFC_SIZE,
			.memory_type = ION_EBI_TYPE,
			.extra_data = (void *) &cp_mfc_msm8960_ion_pdata,
		},
#ifndef CONFIG_MSM_IOMMU
		{
			.id	= ION_SF_HEAP_ID,
			.type	= ION_HEAP_TYPE_CARVEOUT,
			.name	= ION_SF_HEAP_NAME,
			.size	= MSM_ION_SF_SIZE,
			.memory_type = ION_EBI_TYPE,
			.extra_data = (void *) &co_msm8960_ion_pdata,
		},
#endif
		{
			.id	= ION_IOMMU_HEAP_ID,
			.type	= ION_HEAP_TYPE_IOMMU,
			.name	= ION_IOMMU_HEAP_NAME,
		},
		{
			.id	= ION_QSECOM_HEAP_ID,
			.type	= ION_HEAP_TYPE_CARVEOUT,
			.name	= ION_QSECOM_HEAP_NAME,
			.size	= MSM_ION_QSECOM_SIZE,
			.memory_type = ION_EBI_TYPE,
			.extra_data = (void *) &co_msm8960_ion_pdata,
		},
		{
			.id	= ION_AUDIO_HEAP_ID,
			.type	= ION_HEAP_TYPE_CARVEOUT,
			.name	= ION_AUDIO_HEAP_NAME,
			.size	= MSM_ION_AUDIO_SIZE,
			.memory_type = ION_EBI_TYPE,
			.extra_data = (void *) &co_msm8960_ion_pdata,
		},
#endif
	}
};

static struct platform_device msm8960_ion_dev = {
	.name = "ion-msm",
	.id = 1,
	.dev = { .platform_data = &msm8960_ion_pdata },
};
#endif

struct platform_device msm8960_fmem_device = {
	.name = "fmem",
	.id = 1,
	.dev = { .platform_data = &msm8960_fmem_pdata },
};

static void __init adjust_mem_for_liquid(void)
{
	unsigned int i;

	if (!pmem_param_set) {
		if (machine_is_msm8960_liquid())
			msm_ion_sf_size = MSM_LIQUID_ION_SF_SIZE;

		if (msm8960_hdmi_as_primary_selected())
			msm_ion_sf_size = MSM_HDMI_PRIM_ION_SF_SIZE;

		if (machine_is_msm8960_liquid() ||
			msm8960_hdmi_as_primary_selected()) {
			for (i = 0; i < msm8960_ion_pdata.nr; i++) {
				if (msm8960_ion_pdata.heaps[i].id ==
							ION_SF_HEAP_ID) {
					msm8960_ion_pdata.heaps[i].size =
						msm_ion_sf_size;
					pr_debug("msm_ion_sf_size 0x%x\n",
						msm_ion_sf_size);
					break;
				}
			}
		}
	}
}

static void __init reserve_mem_for_ion(enum ion_memory_types mem_type,
				      unsigned long size)
{
	msm8960_reserve_table[mem_type].size += size;
}

static void __init msm8960_reserve_fixed_area(unsigned long fixed_area_size)
{
#if defined(CONFIG_ION_MSM) && defined(CONFIG_MSM_MULTIMEDIA_USE_ION)
	int ret;

	if (fixed_area_size > MAX_FIXED_AREA_SIZE)
		panic("fixed area size is larger than %dM\n",
			MAX_FIXED_AREA_SIZE >> 20);

	reserve_info->fixed_area_size = fixed_area_size;
	reserve_info->fixed_area_start = MSM8960_FW_START;

	ret = memblock_remove(reserve_info->fixed_area_start,
		reserve_info->fixed_area_size);
	BUG_ON(ret);
#endif
}

/**
 * Reserve memory for ION and calculate amount of reusable memory for fmem.
 * We only reserve memory for heaps that are not reusable. However, we only
 * support one reusable heap at the moment so we ignore the reusable flag for
 * other than the first heap with reusable flag set. Also handle special case
 * for video heaps (MM,FW, and MFC). Video requires heaps MM and MFC to be
 * at a higher address than FW in addition to not more than 256MB away from the
 * base address of the firmware. This means that if MM is reusable the other
 * two heaps must be allocated in the same region as FW. This is handled by the
 * mem_is_fmem flag in the platform data. In addition the MM heap must be
 * adjacent to the FW heap for content protection purposes.
 */
static void __init reserve_ion_memory(void)
{
#if defined(CONFIG_ION_MSM) && defined(CONFIG_MSM_MULTIMEDIA_USE_ION)
	unsigned int i;
	unsigned int reusable_count = 0;
	unsigned int fixed_size = 0;
	unsigned int fixed_low_size, fixed_middle_size, fixed_high_size;
	unsigned long fixed_low_start, fixed_middle_start, fixed_high_start;

	adjust_mem_for_liquid();
	msm8960_fmem_pdata.size = 0;
	msm8960_fmem_pdata.reserved_size_low = 0;
	msm8960_fmem_pdata.reserved_size_high = 0;
	msm8960_fmem_pdata.align = PAGE_SIZE;
	fixed_low_size = 0;
	fixed_middle_size = 0;
	fixed_high_size = 0;

	/* We only support 1 reusable heap. Check if more than one heap
	 * is specified as reusable and set as non-reusable if found.
	 */
	for (i = 0; i < msm8960_ion_pdata.nr; ++i) {
		const struct ion_platform_heap *heap =
						&(msm8960_ion_pdata.heaps[i]);

		if (heap->type == ION_HEAP_TYPE_CP && heap->extra_data) {
			struct ion_cp_heap_pdata *data = heap->extra_data;

			reusable_count += (data->reusable) ? 1 : 0;

			if (data->reusable && reusable_count > 1) {
				pr_err("%s: Too many heaps specified as "
					"reusable. Heap %s was not configured "
					"as reusable.\n", __func__, heap->name);
				data->reusable = 0;
			}
		}
	}

	for (i = 0; i < msm8960_ion_pdata.nr; ++i) {
		struct ion_platform_heap *heap =
						&(msm8960_ion_pdata.heaps[i]);
		int align = SZ_4K;
		int iommu_map_all = 0;
		int adjacent_mem_id = INVALID_HEAP_ID;

		if (heap->extra_data) {
			int fixed_position = NOT_FIXED;
			int mem_is_fmem = 0;

			switch (heap->type) {
			case ION_HEAP_TYPE_CP:
				mem_is_fmem = ((struct ion_cp_heap_pdata *)
					heap->extra_data)->mem_is_fmem;
				fixed_position = ((struct ion_cp_heap_pdata *)
					heap->extra_data)->fixed_position;
				align = ((struct ion_cp_heap_pdata *)
						heap->extra_data)->align;
				iommu_map_all =
					((struct ion_cp_heap_pdata *)
					heap->extra_data)->iommu_map_all;
				break;
			case ION_HEAP_TYPE_CARVEOUT:
				mem_is_fmem = ((struct ion_co_heap_pdata *)
					heap->extra_data)->mem_is_fmem;
				fixed_position = ((struct ion_co_heap_pdata *)
					heap->extra_data)->fixed_position;
				adjacent_mem_id = ((struct ion_co_heap_pdata *)
					heap->extra_data)->adjacent_mem_id;
				break;
			default:
				break;
			}

			if (iommu_map_all) {
				if (heap->size & (SZ_64K-1)) {
					heap->size = ALIGN(heap->size, SZ_64K);
					pr_info("Heap %s not aligned to 64K. Adjusting size to %x\n",
						heap->name, heap->size);
				}
			}

			if (mem_is_fmem && adjacent_mem_id != INVALID_HEAP_ID)
				msm8960_fmem_pdata.align = align;

			if (fixed_position != NOT_FIXED)
				fixed_size += heap->size;
			else
				reserve_mem_for_ion(MEMTYPE_EBI1, heap->size);

			if (fixed_position == FIXED_LOW)
				fixed_low_size += heap->size;
			else if (fixed_position == FIXED_MIDDLE)
				fixed_middle_size += heap->size;
			else if (fixed_position == FIXED_HIGH)
				fixed_high_size += heap->size;

			if (mem_is_fmem)
				msm8960_fmem_pdata.size += heap->size;
		}
	}

	if (!fixed_size)
		return;

	if (msm8960_fmem_pdata.size) {
		msm8960_fmem_pdata.reserved_size_low = fixed_low_size +
							HOLE_SIZE;
		msm8960_fmem_pdata.reserved_size_high = fixed_high_size;
		msm8960_fmem_pdata.size += HOLE_SIZE;
	}

	/* Since the fixed area may be carved out of lowmem,
	 * make sure the length is a multiple of 1M.
	 */
	fixed_size = (fixed_size + HOLE_SIZE + SECTION_SIZE - 1)
		& SECTION_MASK;
	msm8960_reserve_fixed_area(fixed_size);

	fixed_low_start = MSM8960_FIXED_AREA_START;
	fixed_middle_start = fixed_low_start + fixed_low_size + HOLE_SIZE;
	fixed_high_start = fixed_middle_start + fixed_middle_size;

	for (i = 0; i < msm8960_ion_pdata.nr; ++i) {
		struct ion_platform_heap *heap = &(msm8960_ion_pdata.heaps[i]);

		if (heap->extra_data) {
			int fixed_position = NOT_FIXED;
			struct ion_cp_heap_pdata *pdata = NULL;

			switch (heap->type) {
			case ION_HEAP_TYPE_CP:
				pdata =
				(struct ion_cp_heap_pdata *)heap->extra_data;
				fixed_position = pdata->fixed_position;
				break;
			case ION_HEAP_TYPE_CARVEOUT:
				fixed_position = ((struct ion_co_heap_pdata *)
					heap->extra_data)->fixed_position;
				break;
			default:
				break;
			}

			switch (fixed_position) {
			case FIXED_LOW:
				heap->base = fixed_low_start;
				break;
			case FIXED_MIDDLE:
				heap->base = fixed_middle_start;
				pdata->secure_base = fixed_middle_start
							- HOLE_SIZE;
				pdata->secure_size = HOLE_SIZE + heap->size;
				break;
			case FIXED_HIGH:
				heap->base = fixed_high_start;
				break;
			default:
				break;
			}
		}
	}
#endif
}

static void __init reserve_mdp_memory(void)
{
	msm8960_mdp_writeback(msm8960_reserve_table);
}

static void __init reserve_cache_dump_memory(void)
{
#ifdef CONFIG_MSM_CACHE_DUMP
	unsigned int total;

	total = msm8960_cache_dump_pdata.l1_size +
		msm8960_cache_dump_pdata.l2_size;
	msm8960_reserve_table[MEMTYPE_EBI1].size += total;
#endif
}

static void __init msm8960_calculate_reserve_sizes(void)
{
	size_pmem_devices();
	reserve_pmem_memory();
	reserve_ion_memory();
	reserve_mdp_memory();
	reserve_rtb_memory();
	reserve_cache_dump_memory();
}

static struct reserve_info msm8960_reserve_info __initdata = {
	.memtype_reserve_table = msm8960_reserve_table,
	.calculate_reserve_sizes = msm8960_calculate_reserve_sizes,
	.reserve_fixed_area = msm8960_reserve_fixed_area,
	.paddr_to_memtype = msm8960_paddr_to_memtype,
};

static int msm8960_memory_bank_size(void)
{
	return 1<<29;
}

static void __init locate_unstable_memory(void)
{
	struct membank *mb = &meminfo.bank[meminfo.nr_banks - 1];
	unsigned long bank_size;
	unsigned long low, high;

	bank_size = msm8960_memory_bank_size();
	msm8960_reserve_info.bank_size = bank_size;

	low = meminfo.bank[0].start;
	high = mb->start + mb->size;

	/* Check if 32 bit overflow occured */
	if (high < mb->start)
		high = ~0UL;

	if (high < MAX_FIXED_AREA_SIZE + MSM8960_FIXED_AREA_START)
		panic("fixed area extends beyond end of memory\n");

	low &= ~(bank_size - 1);

	if (high - low <= bank_size)
		goto no_dmm;

#ifdef CONFIG_ENABLE_DMM
	msm8960_reserve_info.low_unstable_address = mb->start -
					MIN_MEMORY_BLOCK_SIZE + mb->size;
	msm8960_reserve_info.max_unstable_size = MIN_MEMORY_BLOCK_SIZE;
	pr_info("low unstable address %lx max size %lx bank size %lx\n",
		msm8960_reserve_info.low_unstable_address,
		msm8960_reserve_info.max_unstable_size,
		msm8960_reserve_info.bank_size);
	return;
#endif
no_dmm:
	msm8960_reserve_info.low_unstable_address = high;
	msm8960_reserve_info.max_unstable_size = 0;
}

static void __init place_movable_zone(void)
{
#ifdef CONFIG_ENABLE_DMM
	movable_reserved_start = msm8960_reserve_info.low_unstable_address;
	movable_reserved_size = msm8960_reserve_info.max_unstable_size;
	pr_info("movable zone start %lx size %lx\n",
		movable_reserved_start, movable_reserved_size);
#endif
}

static void __init msm8960_early_memory(void)
{
	reserve_info = &msm8960_reserve_info;
	locate_unstable_memory();
	place_movable_zone();
}

static char prim_panel_name[PANEL_NAME_MAX_LEN];
static char ext_panel_name[PANEL_NAME_MAX_LEN];
static int __init prim_display_setup(char *param)
{
	if (strnlen(param, PANEL_NAME_MAX_LEN))
		strlcpy(prim_panel_name, param, PANEL_NAME_MAX_LEN);
	return 0;
}
early_param("prim_display", prim_display_setup);

static int __init ext_display_setup(char *param)
{
	if (strnlen(param, PANEL_NAME_MAX_LEN))
		strlcpy(ext_panel_name, param, PANEL_NAME_MAX_LEN);
	return 0;
}
early_param("ext_display", ext_display_setup);

static void __init msm8960_reserve(void)
{
	msm8960_set_display_params(prim_panel_name, ext_panel_name);
	msm_reserve();
	if (msm8960_fmem_pdata.size) {
#if defined(CONFIG_ION_MSM) && defined(CONFIG_MSM_MULTIMEDIA_USE_ION)
		if (reserve_info->fixed_area_size) {
			msm8960_fmem_pdata.phys =
				reserve_info->fixed_area_start;
			pr_info("mm fw at %lx (fixed) size %x\n",
				reserve_info->fixed_area_start, MSM_MM_FW_SIZE);
			pr_info("fmem start %lx (fixed) size %lx\n",
				msm8960_fmem_pdata.phys,
				msm8960_fmem_pdata.size);
		}
#endif
	}
}

static int msm8960_change_memory_power(u64 start, u64 size,
	int change_type)
{
	return soc_change_memory_power(start, size, change_type);
}

static void __init msm8960_allocate_memory_regions(void)
{
	msm8960_allocate_fb_region();
}

#ifdef CONFIG_WCD9310_CODEC

#define TABLA_INTERRUPT_BASE (NR_MSM_IRQS + NR_GPIO_IRQS + NR_PM8921_IRQS)

/* Micbias setting is based on 8660 CDP/MTP/FLUID requirement
 * 4 micbiases are used to power various analog and digital
 * microphones operating at 1800 mV. Technically, all micbiases
 * can source from single cfilter since all microphones operate
 * at the same voltage level. The arrangement below is to make
 * sure all cfilters are exercised. LDO_H regulator ouput level
 * does not need to be as high as 2.85V. It is choosen for
 * microphone sensitivity purpose.
 */
static struct wcd9xxx_pdata tabla_platform_data = {
	.slimbus_slave_device = {
		.name = "tabla-slave",
		.e_addr = {0, 0, 0x10, 0, 0x17, 2},
	},
	.irq = MSM_GPIO_TO_INT(62),
	.irq_base = TABLA_INTERRUPT_BASE,
	.num_irqs = NR_WCD9XXX_IRQS,
	.reset_gpio = PM8921_GPIO_PM_TO_SYS(34),
	.micbias = {
		.ldoh_v = TABLA_LDOH_2P85_V,
		.cfilt1_mv = 1800,
		.cfilt2_mv = 2700,
		.cfilt3_mv = 1800,
		.bias1_cfilt_sel = TABLA_CFILT1_SEL,
		.bias2_cfilt_sel = TABLA_CFILT2_SEL,
		.bias3_cfilt_sel = TABLA_CFILT3_SEL,
		.bias4_cfilt_sel = TABLA_CFILT3_SEL,
	},
	.regulator = {
	{
		.name = "CDC_VDD_CP",
		.min_uV = 1800000,
		.max_uV = 1800000,
		.optimum_uA = WCD9XXX_CDC_VDDA_CP_CUR_MAX,
	},
	{
		.name = "CDC_VDDA_RX",
		.min_uV = 1800000,
		.max_uV = 1800000,
		.optimum_uA = WCD9XXX_CDC_VDDA_RX_CUR_MAX,
	},
	{
		.name = "CDC_VDDA_TX",
		.min_uV = 1800000,
		.max_uV = 1800000,
		.optimum_uA = WCD9XXX_CDC_VDDA_TX_CUR_MAX,
	},
	{
		.name = "VDDIO_CDC",
		.min_uV = 1800000,
		.max_uV = 1800000,
		.optimum_uA = WCD9XXX_VDDIO_CDC_CUR_MAX,
	},
	{
		.name = "VDDD_CDC_D",
		.min_uV = 1225000,
		.max_uV = 1250000,
		.optimum_uA = WCD9XXX_VDDD_CDC_D_CUR_MAX,
	},
	{
		.name = "CDC_VDDA_A_1P2V",
		.min_uV = 1225000,
		.max_uV = 1250000,
		.optimum_uA = WCD9XXX_VDDD_CDC_A_CUR_MAX,
	},
	},
};

static struct slim_device msm_slim_tabla = {
	.name = "tabla-slim",
	.e_addr = {0, 1, 0x10, 0, 0x17, 2},
	.dev = {
		.platform_data = &tabla_platform_data,
	},
};

static struct wcd9xxx_pdata tabla20_platform_data = {
	.slimbus_slave_device = {
		.name = "tabla-slave",
		.e_addr = {0, 0, 0x60, 0, 0x17, 2},
	},
	.irq = MSM_GPIO_TO_INT(62),
	.irq_base = TABLA_INTERRUPT_BASE,
	.num_irqs = NR_WCD9XXX_IRQS,
	.reset_gpio = PM8921_GPIO_PM_TO_SYS(34),
	.micbias = {
		.ldoh_v = TABLA_LDOH_2P85_V,
		.cfilt1_mv = 1800,
		.cfilt2_mv = 2700,
		.cfilt3_mv = 1800,
		.bias1_cfilt_sel = TABLA_CFILT1_SEL,
		.bias2_cfilt_sel = TABLA_CFILT2_SEL,
		.bias3_cfilt_sel = TABLA_CFILT3_SEL,
		.bias4_cfilt_sel = TABLA_CFILT3_SEL,
	},
	.regulator = {
	{
		.name = "CDC_VDD_CP",
		.min_uV = 1800000,
		.max_uV = 1800000,
		.optimum_uA = WCD9XXX_CDC_VDDA_CP_CUR_MAX,
	},
	{
		.name = "CDC_VDDA_RX",
		.min_uV = 1800000,
		.max_uV = 1800000,
		.optimum_uA = WCD9XXX_CDC_VDDA_RX_CUR_MAX,
	},
	{
		.name = "CDC_VDDA_TX",
		.min_uV = 1800000,
		.max_uV = 1800000,
		.optimum_uA = WCD9XXX_CDC_VDDA_TX_CUR_MAX,
	},
	{
		.name = "VDDIO_CDC",
		.min_uV = 1800000,
		.max_uV = 1800000,
		.optimum_uA = WCD9XXX_VDDIO_CDC_CUR_MAX,
	},
	{
		.name = "VDDD_CDC_D",
		.min_uV = 1225000,
		.max_uV = 1250000,
		.optimum_uA = WCD9XXX_VDDD_CDC_D_CUR_MAX,
	},
	{
		.name = "CDC_VDDA_A_1P2V",
		.min_uV = 1225000,
		.max_uV = 1250000,
		.optimum_uA = WCD9XXX_VDDD_CDC_A_CUR_MAX,
	},
	},
};

static struct slim_device msm_slim_tabla20 = {
	.name = "tabla2x-slim",
	.e_addr = {0, 1, 0x60, 0, 0x17, 2},
	.dev = {
		.platform_data = &tabla20_platform_data,
	},
};
#endif

static struct slim_boardinfo msm_slim_devices[] = {
#ifdef CONFIG_WCD9310_CODEC
	{
		.bus_num = 1,
		.slim_slave = &msm_slim_tabla,
	},
	{
		.bus_num = 1,
		.slim_slave = &msm_slim_tabla20,
	},
#endif
	/* add more slimbus slaves as needed */
};

#define MSM_WCNSS_PHYS	0x03000000
#define MSM_WCNSS_SIZE	0x280000

static struct resource resources_wcnss_wlan[] = {
	{
		.start	= RIVA_APPS_WLAN_RX_DATA_AVAIL_IRQ,
		.end	= RIVA_APPS_WLAN_RX_DATA_AVAIL_IRQ,
		.name	= "wcnss_wlanrx_irq",
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= RIVA_APPS_WLAN_DATA_XFER_DONE_IRQ,
		.end	= RIVA_APPS_WLAN_DATA_XFER_DONE_IRQ,
		.name	= "wcnss_wlantx_irq",
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= MSM_WCNSS_PHYS,
		.end	= MSM_WCNSS_PHYS + MSM_WCNSS_SIZE - 1,
		.name	= "wcnss_mmio",
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= 84,
		.end	= 88,
		.name	= "wcnss_gpios_5wire",
		.flags	= IORESOURCE_IO,
	},
};

static struct qcom_wcnss_opts qcom_wcnss_pdata = {
#if defined(CONFIG_MACH_DECKARD_AS38) || defined(CONFIG_MACH_DECKARD_AS70) || defined(CONFIG_MACH_DECKARD_GP4) || \
    defined(CONFIG_MACH_DECKARD_AF33) || defined(CONFIG_MACH_TOR)
    .has_48mhz_xo   = 1,
#else
    .has_48mhz_xo   = 0,
#endif  /* defined(CONFIG_MACH_DECKARD_AS38) || defined(CONFIG_MACH_DECKARD_AS70) || defined(CONFIG_MACH_DECKARD_GP4) || */
        /* defined(CONFIG_MACH_DECKARD_AF33) || defined(CONFIG_MACH_TOR) */
};

static struct platform_device msm_device_wcnss_wlan = {
	.name		= "wcnss_wlan",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(resources_wcnss_wlan),
	.resource	= resources_wcnss_wlan,
	.dev		= {.platform_data = &qcom_wcnss_pdata},
};

#ifdef CONFIG_QSEECOM
/* qseecom bus scaling */
static struct msm_bus_vectors qseecom_clks_init_vectors[] = {
	{
		.src = MSM_BUS_MASTER_SPS,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ib = 0,
		.ab = 0,
	},
	{
		.src = MSM_BUS_MASTER_SPS,
		.dst = MSM_BUS_SLAVE_SPS,
		.ib = 0,
		.ab = 0,
	},
	{
		.src = MSM_BUS_MASTER_SPDM,
		.dst = MSM_BUS_SLAVE_SPDM,
		.ib = 0,
		.ab = 0,
	},
};

static struct msm_bus_vectors qseecom_enable_dfab_vectors[] = {
	{
		.src = MSM_BUS_MASTER_SPS,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ib = (492 * 8) * 1000000UL,
		.ab = (492 * 8) *  100000UL,
	},
	{
		.src = MSM_BUS_MASTER_SPS,
		.dst = MSM_BUS_SLAVE_SPS,
		.ib = (492 * 8) * 1000000UL,
		.ab = (492 * 8) * 100000UL,
	},
	{
		.src = MSM_BUS_MASTER_SPDM,
		.dst = MSM_BUS_SLAVE_SPDM,
		.ib = 0,
		.ab = 0,
	},
};

static struct msm_bus_vectors qseecom_enable_sfpb_vectors[] = {
	{
		.src = MSM_BUS_MASTER_SPS,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ib = 0,
		.ab = 0,
	},
	{
		.src = MSM_BUS_MASTER_SPS,
		.dst = MSM_BUS_SLAVE_SPS,
		.ib = 0,
		.ab = 0,
	},
	{
		.src = MSM_BUS_MASTER_SPDM,
		.dst = MSM_BUS_SLAVE_SPDM,
		.ib = (64 * 8) * 1000000UL,
		.ab = (64 * 8) *  100000UL,
	},
};

static struct msm_bus_vectors qseecom_enable_dfab_sfpb_vectors[] = {
	{
		.src = MSM_BUS_MASTER_SPS,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ib = (492 * 8) * 1000000UL,
		.ab = (492 * 8) *  100000UL,
	},
	{
		.src = MSM_BUS_MASTER_SPS,
		.dst = MSM_BUS_SLAVE_SPS,
		.ib = (492 * 8) * 1000000UL,
		.ab = (492 * 8) * 100000UL,
	},
	{
		.src = MSM_BUS_MASTER_SPDM,
		.dst = MSM_BUS_SLAVE_SPDM,
		.ib = (64 * 8) * 1000000UL,
		.ab = (64 * 8) *  100000UL,
	},
};

static struct msm_bus_paths qseecom_hw_bus_scale_usecases[] = {
	{
		ARRAY_SIZE(qseecom_clks_init_vectors),
		qseecom_clks_init_vectors,
	},
	{
		ARRAY_SIZE(qseecom_enable_dfab_vectors),
		qseecom_enable_dfab_vectors,
	},
	{
		ARRAY_SIZE(qseecom_enable_sfpb_vectors),
		qseecom_enable_sfpb_vectors,
	},
	{
		ARRAY_SIZE(qseecom_enable_dfab_sfpb_vectors),
		qseecom_enable_dfab_sfpb_vectors,
	},
};

static struct msm_bus_scale_pdata qseecom_bus_pdata = {
	qseecom_hw_bus_scale_usecases,
	ARRAY_SIZE(qseecom_hw_bus_scale_usecases),
	.name = "qsee",
};

static struct platform_device qseecom_device = {
	.name		= "qseecom",
	.id		= 0,
	.dev		= {
		.platform_data = &qseecom_bus_pdata,
	},
};
#endif

#if defined(CONFIG_CRYPTO_DEV_QCRYPTO) || \
		defined(CONFIG_CRYPTO_DEV_QCRYPTO_MODULE) || \
		defined(CONFIG_CRYPTO_DEV_QCEDEV) || \
		defined(CONFIG_CRYPTO_DEV_QCEDEV_MODULE)

#define QCE_SIZE		0x10000
#define QCE_0_BASE		0x18500000

#define QCE_HW_KEY_SUPPORT	0
#define QCE_SHA_HMAC_SUPPORT	1
#define QCE_SHARE_CE_RESOURCE	1
#define QCE_CE_SHARED		0

/* Begin Bus scaling definitions */
static struct msm_bus_vectors crypto_hw_init_vectors[] = {
	{
		.src = MSM_BUS_MASTER_ADM_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 0,
		.ib = 0,
	},
	{
		.src = MSM_BUS_MASTER_ADM_PORT1,
		.dst = MSM_BUS_SLAVE_GSBI1_UART,
		.ab = 0,
		.ib = 0,
	},
};

static struct msm_bus_vectors crypto_hw_active_vectors[] = {
	{
		.src = MSM_BUS_MASTER_ADM_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 70000000UL,
		.ib = 70000000UL,
	},
	{
		.src = MSM_BUS_MASTER_ADM_PORT1,
		.dst = MSM_BUS_SLAVE_GSBI1_UART,
		.ab = 2480000000UL,
		.ib = 2480000000UL,
	},
};

static struct msm_bus_paths crypto_hw_bus_scale_usecases[] = {
	{
		ARRAY_SIZE(crypto_hw_init_vectors),
		crypto_hw_init_vectors,
	},
	{
		ARRAY_SIZE(crypto_hw_active_vectors),
		crypto_hw_active_vectors,
	},
};

static struct msm_bus_scale_pdata crypto_hw_bus_scale_pdata = {
		crypto_hw_bus_scale_usecases,
		ARRAY_SIZE(crypto_hw_bus_scale_usecases),
		.name = "cryptohw",
};
/* End Bus Scaling Definitions*/

static struct resource qcrypto_resources[] = {
	[0] = {
		.start = QCE_0_BASE,
		.end = QCE_0_BASE + QCE_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.name = "crypto_channels",
		.start = DMOV_CE_IN_CHAN,
		.end = DMOV_CE_OUT_CHAN,
		.flags = IORESOURCE_DMA,
	},
	[2] = {
		.name = "crypto_crci_in",
		.start = DMOV_CE_IN_CRCI,
		.end = DMOV_CE_IN_CRCI,
		.flags = IORESOURCE_DMA,
	},
	[3] = {
		.name = "crypto_crci_out",
		.start = DMOV_CE_OUT_CRCI,
		.end = DMOV_CE_OUT_CRCI,
		.flags = IORESOURCE_DMA,
	},
};

static struct resource qcedev_resources[] = {
	[0] = {
		.start = QCE_0_BASE,
		.end = QCE_0_BASE + QCE_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.name = "crypto_channels",
		.start = DMOV_CE_IN_CHAN,
		.end = DMOV_CE_OUT_CHAN,
		.flags = IORESOURCE_DMA,
	},
	[2] = {
		.name = "crypto_crci_in",
		.start = DMOV_CE_IN_CRCI,
		.end = DMOV_CE_IN_CRCI,
		.flags = IORESOURCE_DMA,
	},
	[3] = {
		.name = "crypto_crci_out",
		.start = DMOV_CE_OUT_CRCI,
		.end = DMOV_CE_OUT_CRCI,
		.flags = IORESOURCE_DMA,
	},
};

#endif

#if defined(CONFIG_CRYPTO_DEV_QCRYPTO) || \
		defined(CONFIG_CRYPTO_DEV_QCRYPTO_MODULE)

static struct msm_ce_hw_support qcrypto_ce_hw_suppport = {
	.ce_shared = QCE_CE_SHARED,
	.shared_ce_resource = QCE_SHARE_CE_RESOURCE,
	.hw_key_support = QCE_HW_KEY_SUPPORT,
	.sha_hmac = QCE_SHA_HMAC_SUPPORT,
	.bus_scale_table = &crypto_hw_bus_scale_pdata,
};

static struct platform_device qcrypto_device = {
	.name		= "qcrypto",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(qcrypto_resources),
	.resource	= qcrypto_resources,
	.dev		= {
		.coherent_dma_mask = DMA_BIT_MASK(32),
		.platform_data = &qcrypto_ce_hw_suppport,
	},
};
#endif

#if defined(CONFIG_CRYPTO_DEV_QCEDEV) || \
		defined(CONFIG_CRYPTO_DEV_QCEDEV_MODULE)

static struct msm_ce_hw_support qcedev_ce_hw_suppport = {
	.ce_shared = QCE_CE_SHARED,
	.shared_ce_resource = QCE_SHARE_CE_RESOURCE,
	.hw_key_support = QCE_HW_KEY_SUPPORT,
	.sha_hmac = QCE_SHA_HMAC_SUPPORT,
	.bus_scale_table = &crypto_hw_bus_scale_pdata,
};

static struct platform_device qcedev_device = {
	.name		= "qce",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(qcedev_resources),
	.resource	= qcedev_resources,
	.dev		= {
		.coherent_dma_mask = DMA_BIT_MASK(32),
		.platform_data = &qcedev_ce_hw_suppport,
	},
};
#endif

static struct mdm_platform_data sglte_platform_data = {
	.mdm_version = "4.0",
	.ramdump_delay_ms = 1000,
	.soft_reset_inverted = 1,
	.peripheral_platform_device = NULL,
	.ramdump_timeout_ms = 600000,
	.no_powerdown_after_ramdumps = 1,
};

#define MSM_TSIF0_PHYS			(0x18200000)
#define MSM_TSIF1_PHYS			(0x18201000)
#define MSM_TSIF_SIZE			(0x200)
#define MSM_TSPP_PHYS			(0x18202000)
#define MSM_TSPP_SIZE			(0x1000)
#define MSM_TSPP_BAM_PHYS		(0x18204000)
#define MSM_TSPP_BAM_SIZE		(0x2000)

#define TSIF_0_CLK       GPIO_CFG(75, 1, GPIO_CFG_INPUT, \
	GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)
#define TSIF_0_EN        GPIO_CFG(76, 1, GPIO_CFG_INPUT, \
	GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)
#define TSIF_0_DATA      GPIO_CFG(77, 1, GPIO_CFG_INPUT, \
	GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)
#define TSIF_0_SYNC      GPIO_CFG(82, 1, GPIO_CFG_INPUT, \
	GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)
#define TSIF_1_CLK       GPIO_CFG(79, 1, GPIO_CFG_INPUT, \
	GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)
#define TSIF_1_EN        GPIO_CFG(80, 1, GPIO_CFG_INPUT, \
	GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)
#define TSIF_1_DATA      GPIO_CFG(81, 1, GPIO_CFG_INPUT, \
	GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)
#define TSIF_1_SYNC      GPIO_CFG(78, 1, GPIO_CFG_INPUT, \
	GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)

static const struct msm_gpio tsif_gpios[] = {
	{ .gpio_cfg = TSIF_0_CLK,  .label =  "tsif0_clk", },
	{ .gpio_cfg = TSIF_0_EN,   .label =  "tsif0_en", },
	{ .gpio_cfg = TSIF_0_DATA, .label =  "tsif0_data", },
	{ .gpio_cfg = TSIF_0_SYNC, .label =  "tsif0_sync", },
	{ .gpio_cfg = TSIF_1_CLK,  .label =  "tsif1_clk", },
	{ .gpio_cfg = TSIF_1_EN,   .label =  "tsif1_en", },
	{ .gpio_cfg = TSIF_1_DATA, .label =  "tsif1_data", },
	{ .gpio_cfg = TSIF_1_SYNC, .label =  "tsif1_sync", },
};

static struct resource tspp_resources[] = {
	[0] = {
		.flags = IORESOURCE_IRQ,
		.start = TSIF_TSPP_IRQ,
		.end   = TSIF1_IRQ,
	},
	[1] = {
		.flags = IORESOURCE_MEM,
		.start = MSM_TSIF0_PHYS,
		.end   = MSM_TSIF0_PHYS + MSM_TSIF_SIZE - 1,
	},
	[2] = {
		.flags = IORESOURCE_MEM,
		.start = MSM_TSIF1_PHYS,
		.end   = MSM_TSIF1_PHYS + MSM_TSIF_SIZE - 1,
	},
	[3] = {
		.flags = IORESOURCE_MEM,
		.start = MSM_TSPP_PHYS,
		.end   = MSM_TSPP_PHYS + MSM_TSPP_SIZE - 1,
	},
	[4] = {
		.flags = IORESOURCE_MEM,
		.start = MSM_TSPP_BAM_PHYS,
		.end   = MSM_TSPP_BAM_PHYS + MSM_TSPP_BAM_SIZE - 1,
	},
};

static struct msm_tspp_platform_data tspp_platform_data = {
	.num_gpios = ARRAY_SIZE(tsif_gpios),
	.gpios = tsif_gpios,
	.tsif_pclk = "tsif_pclk",
	.tsif_ref_clk = "tsif_ref_clk",
};

static struct platform_device msm_device_tspp = {
	.name          = "msm_tspp",
	.id            = 0,
	.num_resources = ARRAY_SIZE(tspp_resources),
	.resource      = tspp_resources,
	.dev = {
		.platform_data = &tspp_platform_data
	},
};

#define MSM_SHARED_RAM_PHYS 0x80000000

static void __init msm8960_map_io(void)
{
	msm_shared_ram_phys = MSM_SHARED_RAM_PHYS;
	msm_map_msm8960_io();

	if (socinfo_init() < 0)
		pr_err("socinfo_init() failed!\n");
}

static void __init msm8960_init_irq(void)
{
	struct msm_mpm_device_data *data = NULL;

#ifdef CONFIG_MSM_MPM
	data = &msm8960_mpm_dev_data;
#endif

	msm_mpm_irq_extn_init(data);
	gic_init(0, GIC_PPI_START, MSM_QGIC_DIST_BASE,
						(void *)MSM_QGIC_CPU_BASE);
}

#ifdef CONFIG_SHIRDA
#define SHIRDA_GSBI10_UART_DM_IRDA  (0x1A240038)
#define UART_DM_IRDA_REG_IRDA_EN    (0x01)  /* Enable IRDA Transceiver */
#define UART_DM_IRDA_REG_INVERT_IRDA_RX (0x02)  /* Invert the polarity */

static void __init msm8960_init_irda(void)
{
    void *gsbi10_irda = ioremap_nocache(SHIRDA_GSBI10_UART_DM_IRDA, 4);
    struct clk *core_clk;

    /* Get the core clk */
    core_clk = clk_get_sys("msm_serial_hs.0", "core_clk");
    if(IS_ERR(core_clk))
        goto error_coreclk;

    /* Enable core clk */
    clk_set_rate(core_clk,7372800); /* Set to any rate that's define in clk_table */
    clk_prepare_enable(core_clk);

    /* Write the register */
    writel_relaxed((UART_DM_IRDA_REG_IRDA_EN|UART_DM_IRDA_REG_INVERT_IRDA_RX),
               gsbi10_irda);
    mb(); /* make sure data is completely written */

    clk_disable_unprepare(core_clk);
    clk_put(core_clk);

error_coreclk:
    iounmap(gsbi10_irda);

}
#endif  /* CONFIG_SHIRDA */

#ifdef	CONFIG_SERIAL_MSM_HSL_CONSOLE
static struct gpiomux_setting gpio_cons_rx_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_IN,
};

static struct gpiomux_setting gpio_cons_tx_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_OUT_LOW,
};

static struct msm_gpiomux_config msm8960_console_gpio_configs[] __initdata = {
	{
		.gpio = 10,
		.settings = {
			[GPIOMUX_ACTIVE]    = &gpio_cons_tx_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_cons_tx_cfg,
		},
	},
	{
		.gpio = 11,
		.settings = {
			[GPIOMUX_ACTIVE]    = &gpio_cons_rx_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_cons_rx_cfg,
		},
	},
};

static void __init msm8960_init_debug_console(void)
{
	msm_gpiomux_install(msm8960_console_gpio_configs, ARRAY_SIZE(msm8960_console_gpio_configs));
}
#endif	/* CONFIG_SERIAL_MSM_HSL_CONSOLE */

static void __init msm8960_init_buses(void)
{
#ifdef CONFIG_MSM_BUS_SCALING
	msm_bus_rpm_set_mt_mask();
	msm_bus_8960_apps_fabric_pdata.rpm_enabled = 1;
	msm_bus_8960_sys_fabric_pdata.rpm_enabled = 1;
	msm_bus_8960_mm_fabric_pdata.rpm_enabled = 1;
	msm_bus_apps_fabric.dev.platform_data =
		&msm_bus_8960_apps_fabric_pdata;
	msm_bus_sys_fabric.dev.platform_data = &msm_bus_8960_sys_fabric_pdata;
	msm_bus_mm_fabric.dev.platform_data = &msm_bus_8960_mm_fabric_pdata;
	msm_bus_sys_fpb.dev.platform_data = &msm_bus_8960_sys_fpb_pdata;
	msm_bus_cpss_fpb.dev.platform_data = &msm_bus_8960_cpss_fpb_pdata;
#endif
#ifdef CONFIG_SHIRDA
    msm8960_init_irda();        /* Enable GSBI10 IRDA */
#endif  /* CONFIG_SHIRDA */
#ifdef	CONFIG_SERIAL_MSM_HSL_CONSOLE
	msm8960_init_debug_console();
#endif	/* CONFIG_SERIAL_MSM_HSL_CONSOLE */
}

#ifdef CONFIG_SHLCDC_BOARD
static struct msm_spi_platform_data msm8960_qup_spi_gsbi5_pdata = {
    .max_clock_speed = 1100000,
};
#endif  /* CONFIG_SHLCDC_BOARD */

#ifdef	CONFIG_SHSYS_CUST
static struct msm_spi_platform_data msm8960_qup_spi_gsbi9_pdata = {
	.max_clock_speed = 1100000,
	.infinite_mode	 = 1
};
#else	/* CONFIG_SHSYS_CUST */
static struct msm_spi_platform_data msm8960_qup_spi_gsbi1_pdata = {
	.max_clock_speed = 15060000,
	.infinite_mode	 = 0xFFC0,
};
#endif	/* CONFIG_SHSYS_CUST */

#ifdef CONFIG_USB_MSM_OTG_72K
static struct msm_otg_platform_data msm_otg_pdata;
#else
static int wr_phy_init_seq[] = {
	0x44, 0x80, /* set VBUS valid threshold
			and disconnect valid threshold */
	0x68, 0x81, /* update DC voltage level */
	0x14, 0x82, /* set preemphasis and rise/fall time */
	0x13, 0x83, /* set source impedance adjusment */
	-1};

static int liquid_v1_phy_init_seq[] = {
	0x44, 0x80,/* set VBUS valid threshold
			and disconnect valid threshold */
	0x6C, 0x81,/* update DC voltage level */
	0x18, 0x82,/* set preemphasis and rise/fall time */
	0x23, 0x83,/* set source impedance sdjusment */
	-1};

#ifdef CONFIG_MSM_BUS_SCALING
/* Bandwidth requests (zero) if no vote placed */
static struct msm_bus_vectors usb_init_vectors[] = {
	{
		.src = MSM_BUS_MASTER_SPS,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 0,
		.ib = 0,
	},
};

/* Bus bandwidth requests in Bytes/sec */
static struct msm_bus_vectors usb_max_vectors[] = {
	{
		.src = MSM_BUS_MASTER_SPS,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 60000000,		/* At least 480Mbps on bus. */
		.ib = 960000000,	/* MAX bursts rate */
	},
};

static struct msm_bus_paths usb_bus_scale_usecases[] = {
	{
		ARRAY_SIZE(usb_init_vectors),
		usb_init_vectors,
	},
	{
		ARRAY_SIZE(usb_max_vectors),
		usb_max_vectors,
	},
};

static struct msm_bus_scale_pdata usb_bus_scale_pdata = {
	usb_bus_scale_usecases,
	ARRAY_SIZE(usb_bus_scale_usecases),
	.name = "usb",
};
#endif

#define MSM_MPM_PIN_USB1_OTGSESSVLD	40

static struct msm_otg_platform_data msm_otg_pdata = {
	.mode			= USB_OTG,
	.otg_control		= OTG_PMIC_CONTROL,
	.phy_type		= SNPS_28NM_INTEGRATED_PHY,
#ifndef CONFIG_USB_MSM_OTG_SH_CUST_SWIC
	.pmic_id_irq		= PM8921_USB_ID_IN_IRQ(PM8921_IRQ_BASE),
#endif /* CONFIG_USB_MSM_OTG_SH_CUST_SWIC */
#ifdef CONFIG_USB_MSM_OTG_SH_CUST
    .power_budget       = 100,
#else  /* CONFIG_USB_MSM_OTG_SH_CUST */
    .power_budget       = 750,
#endif  /* CONFIG_USB_MSM_OTG_SH_CUST */
#ifdef CONFIG_MSM_BUS_SCALING
	.bus_scale_table	= &usb_bus_scale_pdata,
	.mpm_otgsessvld_int	= MSM_MPM_PIN_USB1_OTGSESSVLD,
#endif
};
#endif

#ifdef CONFIG_USB_EHCI_MSM_HSIC
#define HSIC_HUB_RESET_GPIO	91
static struct msm_hsic_host_platform_data msm_hsic_pdata = {
	.strobe		= 150,
	.data		= 151,
};

static struct smsc_hub_platform_data hsic_hub_pdata = {
	.hub_reset		= HSIC_HUB_RESET_GPIO,
};
#else
static struct msm_hsic_host_platform_data msm_hsic_pdata;
static struct smsc_hub_platform_data hsic_hub_pdata;
#endif

static struct platform_device smsc_hub_device = {
	.name	= "msm_smsc_hub",
	.id	= -1,
	.dev	= {
		.platform_data = &hsic_hub_pdata,
	},
};

#define PID_MAGIC_ID		0x71432909
#define SERIAL_NUM_MAGIC_ID	0x61945374
#define SERIAL_NUMBER_LENGTH	127
#define DLOAD_USB_BASE_ADD	0x2A03F0C8

struct magic_num_struct {
	uint32_t pid;
	uint32_t serial_num;
};

struct dload_struct {
	uint32_t	reserved1;
	uint32_t	reserved2;
	uint32_t	reserved3;
	uint16_t	reserved4;
	uint16_t	pid;
	char		serial_number[SERIAL_NUMBER_LENGTH];
	uint16_t	reserved5;
	struct magic_num_struct magic_struct;
};

static int usb_diag_update_pid_and_serial_num(uint32_t pid, const char *snum)
{
	struct dload_struct __iomem *dload = 0;

	dload = ioremap(DLOAD_USB_BASE_ADD, sizeof(*dload));
	if (!dload) {
		pr_err("%s: cannot remap I/O memory region: %08x\n",
					__func__, DLOAD_USB_BASE_ADD);
		return -ENXIO;
	}

	pr_debug("%s: dload:%p pid:%x serial_num:%s\n",
				__func__, dload, pid, snum);
	/* update pid */
	dload->magic_struct.pid = PID_MAGIC_ID;
	dload->pid = pid;

	/* update serial number */
	dload->magic_struct.serial_num = 0;
	if (!snum) {
		memset(dload->serial_number, 0, SERIAL_NUMBER_LENGTH);
		goto out;
	}

	dload->magic_struct.serial_num = SERIAL_NUM_MAGIC_ID;
	strlcpy(dload->serial_number, snum, SERIAL_NUMBER_LENGTH);
out:
	iounmap(dload);
	return 0;
}

static struct android_usb_platform_data android_usb_pdata = {
	.update_pid_and_serial_num = usb_diag_update_pid_and_serial_num,
};

static struct platform_device android_usb_device = {
	.name	= "android_usb",
	.id	= -1,
	.dev	= {
		.platform_data = &android_usb_pdata,
	},
};

static uint8_t spm_wfi_cmd_sequence[] __initdata = {
			0x03, 0x0f,
};

static uint8_t spm_power_collapse_without_rpm[] __initdata = {
			0x00, 0x24, 0x54, 0x10,
			0x09, 0x03, 0x01,
			0x10, 0x54, 0x30, 0x0C,
			0x24, 0x30, 0x0f,
};

static uint8_t spm_power_collapse_with_rpm[] __initdata = {
			0x00, 0x24, 0x54, 0x10,
			0x09, 0x07, 0x01, 0x0B,
			0x10, 0x54, 0x30, 0x0C,
			0x24, 0x30, 0x0f,
};

static struct msm_spm_seq_entry msm_spm_seq_list[] __initdata = {
	[0] = {
		.mode = MSM_SPM_MODE_CLOCK_GATING,
		.notify_rpm = false,
		.cmd = spm_wfi_cmd_sequence,
	},
	[1] = {
		.mode = MSM_SPM_MODE_POWER_COLLAPSE,
		.notify_rpm = false,
		.cmd = spm_power_collapse_without_rpm,
	},
	[2] = {
		.mode = MSM_SPM_MODE_POWER_COLLAPSE,
		.notify_rpm = true,
		.cmd = spm_power_collapse_with_rpm,
	},
};

static struct msm_spm_platform_data msm_spm_data[] __initdata = {
	[0] = {
		.reg_base_addr = MSM_SAW0_BASE,
		.reg_init_values[MSM_SPM_REG_SAW2_CFG] = 0x1F,
#if defined(CONFIG_MSM_AVS_HW)
		.reg_init_values[MSM_SPM_REG_SAW2_AVS_CTL] = 0x00,
		.reg_init_values[MSM_SPM_REG_SAW2_AVS_HYSTERESIS] = 0x00,
#endif
		.reg_init_values[MSM_SPM_REG_SAW2_SPM_CTL] = 0x01,
		.reg_init_values[MSM_SPM_REG_SAW2_PMIC_DLY] = 0x02020204,
		.reg_init_values[MSM_SPM_REG_SAW2_PMIC_DATA_0] = 0x0060009C,
		.reg_init_values[MSM_SPM_REG_SAW2_PMIC_DATA_1] = 0x0000001C,
		.vctl_timeout_us = 50,
		.num_modes = ARRAY_SIZE(msm_spm_seq_list),
		.modes = msm_spm_seq_list,
	},
	[1] = {
		.reg_base_addr = MSM_SAW1_BASE,
		.reg_init_values[MSM_SPM_REG_SAW2_CFG] = 0x1F,
#if defined(CONFIG_MSM_AVS_HW)
		.reg_init_values[MSM_SPM_REG_SAW2_AVS_CTL] = 0x00,
		.reg_init_values[MSM_SPM_REG_SAW2_AVS_HYSTERESIS] = 0x00,
#endif
		.reg_init_values[MSM_SPM_REG_SAW2_SPM_CTL] = 0x01,
		.reg_init_values[MSM_SPM_REG_SAW2_PMIC_DLY] = 0x02020204,
		.reg_init_values[MSM_SPM_REG_SAW2_PMIC_DATA_0] = 0x0060009C,
		.reg_init_values[MSM_SPM_REG_SAW2_PMIC_DATA_1] = 0x0000001C,
		.vctl_timeout_us = 50,
		.num_modes = ARRAY_SIZE(msm_spm_seq_list),
		.modes = msm_spm_seq_list,
	},
};

static uint8_t l2_spm_wfi_cmd_sequence[] __initdata = {
			0x00, 0x20, 0x03, 0x20,
			0x00, 0x0f,
};

static uint8_t l2_spm_gdhs_cmd_sequence[] __initdata = {
			0x00, 0x20, 0x34, 0x64,
			0x48, 0x07, 0x48, 0x20,
			0x50, 0x64, 0x04, 0x34,
			0x50, 0x0f,
};
static uint8_t l2_spm_power_off_cmd_sequence[] __initdata = {
			0x00, 0x10, 0x34, 0x64,
			0x48, 0x07, 0x48, 0x10,
			0x50, 0x64, 0x04, 0x34,
			0x50, 0x0F,
};

static struct msm_spm_seq_entry msm_spm_l2_seq_list[] __initdata = {
	[0] = {
		.mode = MSM_SPM_L2_MODE_RETENTION,
		.notify_rpm = false,
		.cmd = l2_spm_wfi_cmd_sequence,
	},
	[1] = {
		.mode = MSM_SPM_L2_MODE_GDHS,
		.notify_rpm = true,
		.cmd = l2_spm_gdhs_cmd_sequence,
	},
	[2] = {
		.mode = MSM_SPM_L2_MODE_POWER_COLLAPSE,
		.notify_rpm = true,
		.cmd = l2_spm_power_off_cmd_sequence,
	},
};

static struct msm_spm_platform_data msm_spm_l2_data[] __initdata = {
	[0] = {
		.reg_base_addr = MSM_SAW_L2_BASE,
		.reg_init_values[MSM_SPM_REG_SAW2_SPM_CTL] = 0x00,
		.reg_init_values[MSM_SPM_REG_SAW2_PMIC_DLY] = 0x02020204,
		.reg_init_values[MSM_SPM_REG_SAW2_PMIC_DATA_0] = 0x00A000AE,
		.reg_init_values[MSM_SPM_REG_SAW2_PMIC_DATA_1] = 0x00A00020,
		.modes = msm_spm_l2_seq_list,
		.num_modes = ARRAY_SIZE(msm_spm_l2_seq_list),
	},
};

#define PM_HAP_EN_GPIO		PM8921_GPIO_PM_TO_SYS(33)
#define PM_HAP_LEN_GPIO		PM8921_GPIO_PM_TO_SYS(20)

static struct msm_xo_voter *xo_handle_d1;

static int isa1200_power(int on)
{
	int rc = 0;

	gpio_set_value(HAP_SHIFT_LVL_OE_GPIO, !!on);

	rc = on ? msm_xo_mode_vote(xo_handle_d1, MSM_XO_MODE_ON) :
			msm_xo_mode_vote(xo_handle_d1, MSM_XO_MODE_OFF);
	if (rc < 0) {
		pr_err("%s: failed to %svote for TCXO D1 buffer%d\n",
				__func__, on ? "" : "de-", rc);
		goto err_xo_vote;
	}

	return 0;

err_xo_vote:
	gpio_set_value(HAP_SHIFT_LVL_OE_GPIO, !on);
	return rc;
}

static int isa1200_dev_setup(bool enable)
{
	int rc = 0;

	struct pm_gpio hap_gpio_config = {
		.direction      = PM_GPIO_DIR_OUT,
		.pull           = PM_GPIO_PULL_NO,
		.out_strength   = PM_GPIO_STRENGTH_HIGH,
		.function       = PM_GPIO_FUNC_NORMAL,
		.inv_int_pol    = 0,
		.vin_sel        = 2,
		.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
		.output_value   = 0,
	};

	if (enable == true) {
		rc = pm8xxx_gpio_config(PM_HAP_EN_GPIO, &hap_gpio_config);
		if (rc) {
			pr_err("%s: pm8921 gpio %d config failed(%d)\n",
					__func__, PM_HAP_EN_GPIO, rc);
			return rc;
		}

		rc = pm8xxx_gpio_config(PM_HAP_LEN_GPIO, &hap_gpio_config);
		if (rc) {
			pr_err("%s: pm8921 gpio %d config failed(%d)\n",
					__func__, PM_HAP_LEN_GPIO, rc);
			return rc;
		}

		rc = gpio_request(HAP_SHIFT_LVL_OE_GPIO, "hap_shft_lvl_oe");
		if (rc) {
			pr_err("%s: unable to request gpio %d (%d)\n",
					__func__, HAP_SHIFT_LVL_OE_GPIO, rc);
			return rc;
		}

		rc = gpio_direction_output(HAP_SHIFT_LVL_OE_GPIO, 0);
		if (rc) {
			pr_err("%s: Unable to set direction\n", __func__);
			goto free_gpio;
		}

		xo_handle_d1 = msm_xo_get(MSM_XO_TCXO_D1, "isa1200");
		if (IS_ERR(xo_handle_d1)) {
			rc = PTR_ERR(xo_handle_d1);
			pr_err("%s: failed to get the handle for D1(%d)\n",
							__func__, rc);
			goto gpio_set_dir;
		}
	} else {
		gpio_free(HAP_SHIFT_LVL_OE_GPIO);

		msm_xo_put(xo_handle_d1);
	}

	return 0;

gpio_set_dir:
	gpio_set_value(HAP_SHIFT_LVL_OE_GPIO, 0);
free_gpio:
	gpio_free(HAP_SHIFT_LVL_OE_GPIO);
	return rc;
}

static struct isa1200_regulator isa1200_reg_data[] = {
	{
		.name = "vcc_i2c",
		.min_uV = ISA_I2C_VTG_MIN_UV,
		.max_uV = ISA_I2C_VTG_MAX_UV,
		.load_uA = ISA_I2C_CURR_UA,
	},
};

static struct isa1200_platform_data isa1200_1_pdata = {
	.name = "vibrator",
	.dev_setup = isa1200_dev_setup,
	.power_on = isa1200_power,
	.hap_en_gpio = PM_HAP_EN_GPIO,
	.hap_len_gpio = PM_HAP_LEN_GPIO,
	.max_timeout = 15000,
	.mode_ctrl = PWM_GEN_MODE,
	.pwm_fd = {
		.pwm_div = 256,
	},
	.is_erm = false,
	.smart_en = true,
	.ext_clk_en = true,
	.chip_en = 1,
	.regulator_info = isa1200_reg_data,
	.num_regulators = ARRAY_SIZE(isa1200_reg_data),
};

static struct i2c_board_info msm_isa1200_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("isa1200_1", 0x90>>1),
	},
};

#ifndef CONFIG_SHTPS_TMA4XX_DEV
#define CYTTSP_TS_GPIO_IRQ		11
#define CYTTSP_TS_SLEEP_GPIO		50
#define CYTTSP_TS_RESOUT_N_GPIO		52

/*virtual key support */
static ssize_t tma340_vkeys_show(struct kobject *kobj,
			struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, 200,
	__stringify(EV_KEY) ":" __stringify(KEY_BACK) ":73:1120:97:97"
	":" __stringify(EV_KEY) ":" __stringify(KEY_MENU) ":230:1120:97:97"
	":" __stringify(EV_KEY) ":" __stringify(KEY_HOME) ":389:1120:97:97"
	":" __stringify(EV_KEY) ":" __stringify(KEY_SEARCH) ":544:1120:97:97"
	"\n");
}

static struct kobj_attribute tma340_vkeys_attr = {
	.attr = {
		.mode = S_IRUGO,
	},
	.show = &tma340_vkeys_show,
};

static struct attribute *tma340_properties_attrs[] = {
	&tma340_vkeys_attr.attr,
	NULL
};

static struct attribute_group tma340_properties_attr_group = {
	.attrs = tma340_properties_attrs,
};


static int cyttsp_platform_init(struct i2c_client *client)
{
	int rc = 0;
	static struct kobject *tma340_properties_kobj;

	tma340_vkeys_attr.attr.name = "virtualkeys.cyttsp-i2c";
	tma340_properties_kobj = kobject_create_and_add("board_properties",
								NULL);
	if (tma340_properties_kobj)
		rc = sysfs_create_group(tma340_properties_kobj,
					&tma340_properties_attr_group);
	if (!tma340_properties_kobj || rc)
		pr_err("%s: failed to create board_properties\n",
				__func__);

	return 0;
}

static struct cyttsp_regulator regulator_data[] = {
	{
		.name = "vdd",
		.min_uV = CY_TMA300_VTG_MIN_UV,
		.max_uV = CY_TMA300_VTG_MAX_UV,
		.hpm_load_uA = CY_TMA300_CURR_24HZ_UA,
		.lpm_load_uA = CY_TMA300_SLEEP_CURR_UA,
	},
	/* TODO: Remove after runtime PM is enabled in I2C driver */
	{
		.name = "vcc_i2c",
		.min_uV = CY_I2C_VTG_MIN_UV,
		.max_uV = CY_I2C_VTG_MAX_UV,
		.hpm_load_uA = CY_I2C_CURR_UA,
		.lpm_load_uA = CY_I2C_SLEEP_CURR_UA,
	},
};

static struct cyttsp_platform_data cyttsp_pdata = {
	.panel_maxx = 634,
	.panel_maxy = 1166,
	.disp_maxx = 616,
	.disp_maxy = 1023,
	.disp_minx = 0,
	.disp_miny = 16,
	.flags = 0x01,
	.gen = CY_GEN3,	/* or */
	.use_st = CY_USE_ST,
	.use_mt = CY_USE_MT,
	.use_hndshk = CY_SEND_HNDSHK,
	.use_trk_id = CY_USE_TRACKING_ID,
	.use_sleep = CY_USE_DEEP_SLEEP_SEL | CY_USE_LOW_POWER_SEL,
	.use_gestures = CY_USE_GESTURES,
	.fw_fname = "cyttsp_8960_cdp.hex",
	/* activate up to 4 groups
	 * and set active distance
	 */
	.gest_set = CY_GEST_GRP1 | CY_GEST_GRP2 |
				CY_GEST_GRP3 | CY_GEST_GRP4 |
				CY_ACT_DIST,
	/* change act_intrvl to customize the Active power state
	 * scanning/processing refresh interval for Operating mode
	 */
	.act_intrvl = CY_ACT_INTRVL_DFLT,
	/* change tch_tmout to customize the touch timeout for the
	 * Active power state for Operating mode
	 */
	.tch_tmout = CY_TCH_TMOUT_DFLT,
	/* change lp_intrvl to customize the Low Power power state
	 * scanning/processing refresh interval for Operating mode
	 */
	.lp_intrvl = CY_LP_INTRVL_DFLT,
	.sleep_gpio = CYTTSP_TS_SLEEP_GPIO,
	.resout_gpio = CYTTSP_TS_RESOUT_N_GPIO,
	.irq_gpio = CYTTSP_TS_GPIO_IRQ,
	.regulator_info = regulator_data,
	.num_regulators = ARRAY_SIZE(regulator_data),
	.init = cyttsp_platform_init,
	.correct_fw_ver = 9,
};
#endif  /* CONFIG_SHTPS_TMA4XX_DEV */

#ifdef CONFIG_SHTPS_TMA4XX_DEV
#include <sharp/touch_platform.h>
#include <linux/input.h>

/* use the following define if the device is a TMA400 family part
 */
#define CY_USE_TMA400

/* use the following define if the device is a TMA884/616 family part
#define CY_USE_TMA884
 */

#define TOUCH_GPIO_RST_CYTTSP 16
#define TOUCH_GPIO_IRQ_CYTTSP 106

#define TP_PWR_INPUT_5V 0
#define TP_PWR_INPUT_3V 1

#define TP_PWR_EN 33
static int cyttsp4_hw_power(int on)
{
	sharp_smem_common_type *sharp_smem;
	unsigned long rev;
	int pwr_input = TP_PWR_INPUT_5V;
	int ret = 0;
	static int vcpin_is_on;
	static struct regulator *reg_8921_l17;
	struct pm_gpio param = {
		.direction  = PM_GPIO_DIR_OUT,
		.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
		.output_value   = 1,
		.pull       = PM_GPIO_PULL_NO,
		.vin_sel    = PM_GPIO_VIN_S4,
		.out_strength   = PM_GPIO_STRENGTH_MED,
		.function   = PM_GPIO_FUNC_NORMAL,
	};

	sharp_smem = sh_smem_get_common_address();
	rev = sharp_smem->sh_hw_revision;

#if defined(CONFIG_SHTPS_TMA4XX_TMA443)
	if (rev == 0x00000000 /* SH_HW_REV_ES0 */ ||
		rev == 0x00000005 /* SH_HW_REV_ES1 */ ||
		rev == 0x00000001 /* SH_HW_REV_ES15 */ ||
		rev == 0x00000006 /* SH_HW_REV_PP1 */)
		pwr_input = TP_PWR_INPUT_5V;
	else
		pwr_input = TP_PWR_INPUT_3V;
#elif defined(CONFIG_SHTPS_TMA4XX_TMA463_001)
	if (rev == 0x00000000 /* SH_HW_REV_ES0 */ ||
		rev == 0x00000005 /* SH_HW_REV_ES1 */ ||
		rev == 0x00000001 /* SH_HW_REV_ES15 */)
		pwr_input = TP_PWR_INPUT_5V;
	else
		pwr_input = TP_PWR_INPUT_3V;		
#elif defined(CONFIG_SHTPS_TMA4XX_TMA463_002)
	if (rev == 0x00000000 /* SH_HW_REV_ES0 */ ||
		rev == 0x00000005 /* SH_HW_REV_ES1 */ ||
		rev == 0x00000001 /* SH_HW_REV_ES15 */ ||
		rev == 0x00000006 /* SH_HW_REV_PP1 */)
		pwr_input = TP_PWR_INPUT_5V;
	else
		pwr_input = TP_PWR_INPUT_3V;
#elif defined(CONFIG_SHTPS_TMA4XX_TMA463_003)
	if (rev == 0x00000006)	/* SH_HW_REV_PP1 */
		pwr_input = TP_PWR_INPUT_3V;
	else
		pwr_input = TP_PWR_INPUT_5V;
#else
	if (rev == 0x00000000 /* SH_HW_REV_ES0 */ ||
		rev == 0x00000005 /* SH_HW_REV_ES1 */ ||
		rev == 0x00000001 /* SH_HW_REV_ES15 */ ||
		rev == 0x00000006 /* SH_HW_REV_PP1 */)
		pwr_input = TP_PWR_INPUT_5V;
	else
		pwr_input = TP_PWR_INPUT_3V;
#endif

	if (pwr_input == TP_PWR_INPUT_3V) {
		if (!reg_8921_l17) {
			pr_info("%s: regulator_get(reg_8921_l17)\n", __func__);
			reg_8921_l17 =
				regulator_get(&msm8960_device_qup_spi_gsbi9.dev,
					"cyttsp4_vdda");
			if (IS_ERR(reg_8921_l17)) {
				pr_err("could not get reg_8921_l17, "
					"ret = %ld\n", PTR_ERR(reg_8921_l17));
				return -ENODEV;
			}
			pr_info("%s: regulator_set_voltage(reg_8921_l17)\n",
				__func__);
			ret = regulator_set_voltage(reg_8921_l17,
						3000000, 3000000);
			if (ret) {
				pr_err("set_voltage failed for 8921_l17, "
					"ret=%d\n", ret);
				return -EINVAL;
			}
		}

		if (on == 99)
			return vcpin_is_on;
		if (vcpin_is_on == on)
			return ret;

		if (on) {
			pr_info("%s: regulator_enable(reg_8921_l17)\n", __func__);
			ret = regulator_enable(reg_8921_l17);
			if (ret) {
				pr_err("'%s' regulator enable failed, ret=%d\n",
					"cyttsp4_vdda", ret);
				return ret;
			}
			msleep(103);
			vcpin_is_on = on;
			return ret;
		} else {
			pr_info("%s: regulator_enable(reg_8921_l17)\n", __func__);
			ret = regulator_disable(reg_8921_l17);
			if (ret) {
				pr_err("'%s' regulator disable failed, ret=%d\n",
					"cyttsp4_vdda", ret);
				return ret;
			}
			vcpin_is_on = on;
			return ret;
		}
	} else if (pwr_input == TP_PWR_INPUT_5V) {
		if (on == 99)
			return vcpin_is_on;
		if (vcpin_is_on == on)
			return ret;

		if (on) {
			ret = gpio_request(PM8921_GPIO_PM_TO_SYS(TP_PWR_EN),
					"TP_PWR_EN");
			if (ret < 0) {
				pr_err("failed to request TP_PWR_EN gpio\n");
				vcpin_is_on = 0;
				return ret;
			}

			ret = pm8xxx_gpio_config(
				PM8921_GPIO_PM_TO_SYS(TP_PWR_EN), &param);
			if (ret < 0) {
				pr_err("failed to configure TP_PWR_EN gpio\n");
				gpio_free(PM8921_GPIO_PM_TO_SYS(TP_PWR_EN));
				vcpin_is_on = 0;
				return ret;
			}
			msleep(103);
			gpio_free(PM8921_GPIO_PM_TO_SYS(TP_PWR_EN));
			vcpin_is_on = on;
			return ret;
		} else {
			ret = gpio_request(PM8921_GPIO_PM_TO_SYS(TP_PWR_EN),
					"TP_PWR_EN");
			if (ret < 0) {
				pr_err("failed to request TP_PWR_EN gpio\n");
				vcpin_is_on = 0;
				return ret;
			}
			param.output_value = 0;
			ret = pm8xxx_gpio_config(
				PM8921_GPIO_PM_TO_SYS(TP_PWR_EN), &param);
			if (ret < 0) {
				pr_err("failed to configure TP_PWR_EN gpio\n");
				gpio_free(PM8921_GPIO_PM_TO_SYS(TP_PWR_EN));
				vcpin_is_on = 0;
				return ret;
			}
			gpio_free(PM8921_GPIO_PM_TO_SYS(TP_PWR_EN));
			vcpin_is_on = on;
			return ret;
		}
	}

	return ret;
}

static int cyttsp4_hw_reset(void)
{
	int ret = 0;

	gpio_set_value(TOUCH_GPIO_RST_CYTTSP, 1);
	pr_info("%s: gpio_set_value(step%d)=%d\n", __func__, 1, 1);
	msleep(20);
	gpio_set_value(TOUCH_GPIO_RST_CYTTSP, 0);
	pr_info("%s: gpio_set_value(step%d)=%d\n", __func__, 2, 0);
	msleep(40);
	gpio_set_value(TOUCH_GPIO_RST_CYTTSP, 1);
	msleep(91);
	pr_info("%s: gpio_set_value(step%d)=%d\n", __func__, 3, 1);

	return ret;
}

#define CY_WAKE_DFLT 99
static int cyttsp4_hw_recov(int on)
{
	int retval = 0;

	pr_info("%s: on=%d\n", __func__, on);
	switch (on) {
	case 0:
		cyttsp4_hw_reset();
		retval = 0;
		break;
	case CY_WAKE_DFLT:
		retval = gpio_request(TOUCH_GPIO_IRQ_CYTTSP, NULL);
		if (retval < 0) {
			pr_err("%s: Fail request IRQ pin r=%d\n",
				__func__, retval);
			break;
		}
		retval = gpio_direction_output(TOUCH_GPIO_IRQ_CYTTSP, 0);
		if (retval < 0) {
			pr_err("%s: Fail switch IRQ pin to output"
				" r=%d\n", __func__, retval);
		} else {
			udelay(2000);
			retval = gpio_direction_input(TOUCH_GPIO_IRQ_CYTTSP);
			if (retval < 0) {
				pr_err("%s: Fail switch IRQ pin to input"
					" r=%d\n", __func__, retval);
			}
		}
		gpio_free(TOUCH_GPIO_IRQ_CYTTSP);
		break;
	default:
		retval = -ENOSYS;
		break;
	}

	return retval;
}

static int cyttsp4_irq_stat(void)
{
	return gpio_get_value(TOUCH_GPIO_IRQ_CYTTSP);
}

#if defined(CONFIG_SHTPS_TMA4XX_TMA443)
#define CY_MAXX 719
#define CY_MAXY 1279
#elif defined(CONFIG_SHTPS_TMA4XX_TMA463_001)
#define CY_MAXX 539
#define CY_MAXY 959
#elif defined(CONFIG_SHTPS_TMA4XX_TMA463_002)
#define CY_MAXX 719
#define CY_MAXY 1279
#elif defined(CONFIG_SHTPS_TMA4XX_TMA463_003)
#define CY_MAXX 719
#define CY_MAXY 1279
#else
#define CY_MAXX 719
#define CY_MAXY 1279
#endif

#define CY_ABS_MIN_X 0
#define CY_ABS_MIN_Y 0
#define CY_ABS_MIN_P 0
#define CY_ABS_MIN_W 0
#ifdef CY_USE_TMA400
#define CY_ABS_MIN_T 0
#endif /* --CY_USE_TMA400 */
#ifdef CY_USE_TMA884
#define CY_ABS_MIN_T 1
#endif /* --CY_USE_TMA884 */

#define CY_ABS_MAX_X CY_MAXX
#define CY_ABS_MAX_Y CY_MAXY
#define CY_ABS_MAX_P 255
#define CY_ABS_MAX_W 255
#ifdef CY_USE_TMA400
#define CY_ABS_MAX_T 9
#endif /* --CY_USE_TMA400 */
#ifdef CY_USE_TMA884
#define CY_ABS_MAX_T 10
#endif /* --CY_USE_TMA884 */
#define CY_IGNORE_VALUE 0xFFFF

#if defined(CONFIG_SHTPS_TMA4XX_TMA443)
#include "../../../drivers/sharp/shtps/tma4xx/cyttsp4_params.h"
#elif defined(CONFIG_SHTPS_TMA4XX_TMA463_001)
#include "../../../drivers/sharp/shtps/tma4xx/tma463-001/cyttsp4_params.h"
#elif defined(CONFIG_SHTPS_TMA4XX_TMA463_002)
#include "../../../drivers/sharp/shtps/tma4xx/tma463-002/cyttsp4_params.h"
#elif defined(CONFIG_SHTPS_TMA4XX_TMA463_003)
#include "../../../drivers/sharp/shtps/tma4xx/tma463-003/cyttsp4_params.h"
#else
#include "../../../drivers/sharp/shtps/tma4xx/tma463-002/cyttsp4_params.h"
#endif

static struct touch_settings cyttsp4_sett_param_regs = {
	.data = (uint8_t *)&cyttsp4_param_regs[0],
	.size = sizeof(cyttsp4_param_regs),
	.tag = 0,
};

static struct touch_settings cyttsp4_sett_param_size = {
	.data = (uint8_t *)&cyttsp4_param_size[0],
	.size = sizeof(cyttsp4_param_size),
	.tag = 0,
};

/* Design Data Table */
static u8 cyttsp4_ddata[] = {
	0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15,
	16, 17, 18, 19, 20, 21, 22, 23, 24 /* test padding ,
	25, 26, 27, 28, 29, 30, 31 */
};

static struct touch_settings cyttsp4_sett_ddata = {
	.data = (uint8_t *)&cyttsp4_ddata[0],
	.size = sizeof(cyttsp4_ddata),
	.tag = 0,
};

/* Manufacturing Data Table */
static u8 cyttsp4_mdata[] = {
	65, 64, /* test truncation */63, 62, 61, 60, 59, 58, 57, 56, 55,
	54, 53, 52, 51, 50, 49, 48,
	47, 46, 45, 44, 43, 42, 41, 40, 39, 38, 37, 36, 35, 34, 33, 32,
	31, 30, 29, 28, 27, 26, 25, 24, 23, 22, 21, 20, 19, 18, 17, 16,
	15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0
};

static struct touch_settings cyttsp4_sett_mdata = {
	.data = (uint8_t *)&cyttsp4_mdata[0],
	.size = sizeof(cyttsp4_mdata),
	.tag = 0,
};

/* use this define to include auto boot image
 */
#define CY_USE_INCLUDE_FBL
#ifdef CY_USE_INCLUDE_FBL
#if defined(CONFIG_SHTPS_TMA4XX_TMA443)
#include "../../../drivers/sharp/shtps/tma4xx/cyttsp4_img.h"
#elif defined(CONFIG_SHTPS_TMA4XX_TMA463_001)
#include "../../../drivers/sharp/shtps/tma4xx/tma463-001/cyttsp4_img.h"
#elif defined(CONFIG_SHTPS_TMA4XX_TMA463_002)
#include "../../../drivers/sharp/shtps/tma4xx/tma463-002/cyttsp4_img.h"
#elif defined(CONFIG_SHTPS_TMA4XX_TMA463_003)
#include "../../../drivers/sharp/shtps/tma4xx/tma463-003/cyttsp4_img.h"
#else
#include "../../../drivers/sharp/shtps/tma4xx/tma463-002/cyttsp4_img.h"
#endif
static struct touch_firmware cyttsp4_firmware = {
	.img = cyttsp4_img,
	.size = sizeof(cyttsp4_img),
	.ver = cyttsp4_ver,
	.vsize = sizeof(cyttsp4_ver),
};
#else
static struct touch_firmware cyttsp4_firmware = {
	.img = NULL,
	.size = 0,
	.ver = NULL,
	.vsize = 0,
};
#endif

static const uint16_t cyttsp4_abs[] = {
	ABS_MT_POSITION_X, CY_ABS_MIN_X, CY_ABS_MAX_X, 0, 0,
	ABS_MT_POSITION_Y, CY_ABS_MIN_Y, CY_ABS_MAX_Y, 0, 0,
	ABS_MT_PRESSURE, CY_ABS_MIN_P, CY_ABS_MAX_P, 0, 0,
#ifdef CY_USE_TMA400
	CY_IGNORE_VALUE, CY_ABS_MIN_W, CY_ABS_MAX_W, 0, 0,
#endif /* ++CY_USE_TMA400 */
#ifndef CY_USE_TMA400
	ABS_MT_TOUCH_MAJOR, CY_ABS_MIN_W, CY_ABS_MAX_W, 0, 0,
#endif /* --CY_USE_TMA400 */
	ABS_MT_TRACKING_ID, CY_ABS_MIN_T, CY_ABS_MAX_T, 0, 0,
};

struct touch_framework cyttsp4_framework = {
	.abs = (uint16_t *)&cyttsp4_abs[0],
	.size = sizeof(cyttsp4_abs)/sizeof(uint16_t),
	.enable_vkeys = 0,
};

struct touch_platform_data cyttsp4_spi_touch_platform_data = {
	.sett = {
		NULL,   /* Reserved */
		NULL,   /* Command Registers */
		NULL,   /* Touch Report */
		NULL,   /* Cypress Data Record */
		NULL,   /* Test Record */
		NULL,   /* Panel Configuration Record */
		&cyttsp4_sett_param_regs,
		&cyttsp4_sett_param_size,
		NULL,   /* Reserved */
		NULL,   /* Reserved */
		NULL,   /* Operational Configuration Record */
		&cyttsp4_sett_ddata,    /* Design Data Record */
		&cyttsp4_sett_mdata,    /* Manufacturing Data Record */
	},
	.fw = &cyttsp4_firmware,
	.frmwrk = &cyttsp4_framework,
	.addr = {0xFF, 0xFF},   /* not used for SPI */
#if defined(CONFIG_SHTPS_TMA4XX_TMA443)
	.flags = 0x01,      /* TMA400 */
#else
	.flags = 0x00,
#endif
	.hw_reset = cyttsp4_hw_reset,
	.hw_recov = cyttsp4_hw_recov,
	.hw_power = cyttsp4_hw_power,
	.irq_stat = cyttsp4_irq_stat,
};
#endif  /* CONFIG_SHTPS_TMA4XX_DEV */

#ifdef CONFIG_SHLCDC_BOARD
static struct i2c_board_info msm_lcd_board_info[] __initdata = {
    {
        I2C_BOARD_INFO("bdic_i2c", (0xA8 >> 1)),
    },
#if defined(CONFIG_SHDISP_PANEL_NICOLE)
    {
        I2C_BOARD_INFO("nicole_i2c", (0x2C >> 1)),
    },
#elif defined(CONFIG_SHDISP_PANEL_TAKT)
    {
        I2C_BOARD_INFO("takt_i2c", (0x2C >> 1)),
    },
#endif
};
#endif /* CONFIG_SHLCDC_BOARD */

#ifdef CONFIG_SHLCDC_LED_BD2802GU
static struct i2c_board_info msm_ledc_board_info[] __initdata = {
    {
        I2C_BOARD_INFO("ledc_i2c", 0x1A),
    },
};
#endif /* CONFIG_SHLCDC_LED_BD2802GU */

#ifndef CONFIG_SHTPS_TMA4XX_DEV
static struct i2c_board_info cyttsp_info[] __initdata = {
	{
		I2C_BOARD_INFO(CY_I2C_NAME, 0x24),
		.platform_data = &cyttsp_pdata,
#ifndef CY_USE_TIMER
		.irq = MSM_GPIO_TO_INT(CYTTSP_TS_GPIO_IRQ),
#endif /* CY_USE_TIMER */
	},
};
#endif  /* CONFIG_SHTPS_TMA4XX_DEV */

/* configuration data for mxt1386 */
static const u8 mxt1386_config_data[] = {
	/* T6 Object */
	0, 0, 0, 0, 0, 0,
	/* T38 Object */
	11, 2, 0, 11, 11, 11, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0,
	/* T7 Object */
	100, 16, 50,
	/* T8 Object */
	8, 0, 0, 0, 0, 0, 8, 14, 50, 215,
	/* T9 Object */
	131, 0, 0, 26, 42, 0, 32, 63, 3, 5,
	0, 2, 1, 113, 10, 10, 8, 10, 255, 2,
	85, 5, 0, 0, 20, 20, 75, 25, 202, 29,
	10, 10, 45, 46,
	/* T15 Object */
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0,
	/* T18 Object */
	0, 0,
	/* T22 Object */
	5, 0, 0, 0, 0, 0, 0, 0, 30, 0,
	0, 0, 5, 8, 10, 13, 0,
	/* T24 Object */
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0,
	/* T25 Object */
	3, 0, 188, 52, 52, 33, 0, 0, 0, 0,
	0, 0, 0, 0,
	/* T27 Object */
	0, 0, 0, 0, 0, 0, 0,
	/* T28 Object */
	0, 0, 0, 8, 12, 60,
	/* T40 Object */
	0, 0, 0, 0, 0,
	/* T41 Object */
	0, 0, 0, 0, 0, 0,
	/* T43 Object */
	0, 0, 0, 0, 0, 0,
};

/* configuration data for mxt1386e using V1.0 firmware */
static const u8 mxt1386e_config_data_v1_0[] = {
	/* T6 Object */
	0, 0, 0, 0, 0, 0,
	/* T38 Object */
	12, 1, 0, 17, 1, 12, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0,
	/* T7 Object */
	100, 16, 50,
	/* T8 Object */
	25, 0, 20, 20, 0, 0, 20, 50, 0, 0,
	/* T9 Object */
	131, 0, 0, 26, 42, 0, 32, 80, 2, 5,
	0, 5, 5, 0, 10, 30, 10, 10, 255, 2,
	85, 5, 10, 10, 10, 10, 135, 55, 70, 40,
	10, 5, 0, 0, 0,
	/* T18 Object */
	0, 0,
	/* T24 Object */
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0,
	/* T25 Object */
	3, 0, 60, 115, 156, 99,
	/* T27 Object */
	0, 0, 0, 0, 0, 0, 0,
	/* T40 Object */
	0, 0, 0, 0, 0,
	/* T42 Object */
	2, 0, 255, 0, 255, 0, 0, 0, 0, 0,
	/* T43 Object */
	0, 0, 0, 0, 0, 0, 0,
	/* T46 Object */
	64, 0, 20, 20, 0, 0, 0, 0, 0,
	/* T47 Object */
	0, 0, 0, 0, 0, 0, 3, 64, 66, 0,
	/* T48 Object */
	31, 64, 64, 0, 0, 0, 0, 0, 0, 0,
	48, 40, 0, 10, 10, 0, 0, 100, 10, 80,
	0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
	52, 0, 12, 0, 17, 0, 1, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0,
	/* T56 Object */
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	2, 99, 33,
};

/* configuration data for mxt1386e using V2.1 firmware */
static const u8 mxt1386e_config_data_v2_1[] = {
	/* T6 Object */
	0, 0, 0, 0, 0, 0,
	/* T38 Object */
	12, 3, 0, 24, 5, 12, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0,
	/* T7 Object */
	100, 16, 50,
	/* T8 Object */
	25, 0, 20, 20, 0, 0, 20, 50, 0, 0,
	/* T9 Object */
	139, 0, 0, 26, 42, 0, 32, 80, 2, 5,
	0, 5, 5, 0, 10, 30, 10, 10, 255, 2,
	85, 5, 10, 10, 10, 10, 135, 55, 70, 40,
	10, 5, 0, 0, 0,
	/* T18 Object */
	0, 0,
	/* T24 Object */
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0,
	/* T25 Object */
	1, 0, 60, 115, 156, 99,
	/* T27 Object */
	0, 0, 0, 0, 0, 0, 0,
	/* T40 Object */
	0, 0, 0, 0, 0,
	/* T42 Object */
	0, 0, 255, 0, 255, 0, 0, 0, 0, 0,
	/* T43 Object */
	0, 0, 0, 0, 0, 0, 0, 64, 0, 8,
	16,
	/* T46 Object */
	64, 0, 20, 20, 0, 0, 0, 0, 0,
	/* T47 Object */
	0, 0, 0, 0, 0, 0, 3, 64, 66, 0,
	/* T48 Object */
	1, 64, 64, 0, 0, 0, 0, 0, 0, 0,
	48, 40, 0, 10, 10, 0, 0, 100, 10, 80,
	0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
	52, 0, 12, 0, 17, 0, 1, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0,
	/* T56 Object */
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	2, 99, 33, 0, 149, 24, 193, 255, 255, 255,
	255,
};

/* configuration data for mxt1386e on 3D SKU using V2.1 firmware */
static const u8 mxt1386e_config_data_3d[] = {
	/* T6 Object */
	0, 0, 0, 0, 0, 0,
	/* T38 Object */
	13, 1, 0, 23, 2, 12, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0,
	/* T7 Object */
	100, 10, 50,
	/* T8 Object */
	25, 0, 20, 20, 0, 0, 0, 0, 0, 0,
	/* T9 Object */
	131, 0, 0, 26, 42, 0, 32, 80, 2, 5,
	0, 5, 5, 0, 10, 30, 10, 10, 175, 4,
	127, 7, 26, 21, 17, 19, 143, 35, 207, 40,
	20, 5, 54, 49, 0,
	/* T18 Object */
	0, 0,
	/* T24 Object */
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0,
	/* T25 Object */
	0, 0, 72, 113, 168, 97,
	/* T27 Object */
	0, 0, 0, 0, 0, 0, 0,
	/* T40 Object */
	0, 0, 0, 0, 0,
	/* T42 Object */
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	/* T43 Object */
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0,
	/* T46 Object */
	68, 0, 16, 16, 0, 0, 0, 0, 0,
	/* T47 Object */
	0, 0, 0, 0, 0, 0, 3, 64, 66, 0,
	/* T48 Object */
	31, 64, 64, 0, 0, 0, 0, 0, 0, 0,
	32, 50, 0, 10, 10, 0, 0, 100, 10, 90,
	0, 0, 0, 0, 0, 0, 0, 10, 1, 30,
	52, 10, 5, 0, 33, 0, 1, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0,
	/* T56 Object */
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0,
};

#define MXT_TS_GPIO_IRQ			11
#define MXT_TS_LDO_EN_GPIO		50
#define MXT_TS_RESET_GPIO		52

static void mxt_init_hw_liquid(void)
{
	int rc;

	rc = gpio_request(MXT_TS_LDO_EN_GPIO, "mxt_ldo_en_gpio");
	if (rc) {
		pr_err("%s: unable to request mxt_ldo_en_gpio [%d]\n",
			__func__, MXT_TS_LDO_EN_GPIO);
		return;
	}

	rc = gpio_direction_output(MXT_TS_LDO_EN_GPIO, 1);
	if (rc) {
		pr_err("%s: unable to set_direction for mxt_ldo_en_gpio [%d]\n",
			__func__, MXT_TS_LDO_EN_GPIO);
		goto err_ldo_gpio_req;
	}

	return;

err_ldo_gpio_req:
	gpio_free(MXT_TS_LDO_EN_GPIO);
}

static struct mxt_config_info mxt_config_array_2d[] = {
	{
		.config		= mxt1386_config_data,
		.config_length	= ARRAY_SIZE(mxt1386_config_data),
		.family_id	= 0xA0,
		.variant_id	= 0x0,
		.version	= 0x10,
		.build		= 0xAA,
		.bootldr_id	= MXT_BOOTLOADER_ID_1386,
	},
	{
		.config		= mxt1386e_config_data_v1_0,
		.config_length	= ARRAY_SIZE(mxt1386e_config_data_v1_0),
		.family_id	= 0xA0,
		.variant_id	= 0x2,
		.version	= 0x10,
		.build		= 0xAA,
		.bootldr_id	= MXT_BOOTLOADER_ID_1386E,
		.fw_name	= "atmel_8960_liquid_v2_2_AA.hex",
	},
	{
		.config		= mxt1386e_config_data_v2_1,
		.config_length	= ARRAY_SIZE(mxt1386e_config_data_v2_1),
		.family_id	= 0xA0,
		.variant_id	= 0x7,
		.version	= 0x21,
		.build		= 0xAA,
		.bootldr_id	= MXT_BOOTLOADER_ID_1386E,
		.fw_name	= "atmel_8960_liquid_v2_2_AA.hex",
	},
	{
		/* The config data for V2.2.AA is the same as for V2.1.AA */
		.config		= mxt1386e_config_data_v2_1,
		.config_length	= ARRAY_SIZE(mxt1386e_config_data_v2_1),
		.family_id	= 0xA0,
		.variant_id	= 0x7,
		.version	= 0x22,
		.build		= 0xAA,
		.bootldr_id	= MXT_BOOTLOADER_ID_1386E,
	},
};

static struct mxt_platform_data mxt_platform_data_2d = {
	.config_array		= mxt_config_array_2d,
	.config_array_size	= ARRAY_SIZE(mxt_config_array_2d),
	.panel_minx		= 0,
	.panel_maxx		= 1365,
	.panel_miny		= 0,
	.panel_maxy		= 767,
	.disp_minx		= 0,
	.disp_maxx		= 1365,
	.disp_miny		= 0,
	.disp_maxy		= 767,
	.irqflags		= IRQF_TRIGGER_FALLING,
	.i2c_pull_up		= true,
	.reset_gpio		= MXT_TS_RESET_GPIO,
	.irq_gpio		= MXT_TS_GPIO_IRQ,
};

static struct mxt_config_info mxt_config_array_3d[] = {
	{
		.config		= mxt1386e_config_data_3d,
		.config_length	= ARRAY_SIZE(mxt1386e_config_data_3d),
		.family_id	= 0xA0,
		.variant_id	= 0x7,
		.version	= 0x21,
		.build		= 0xAA,
	},
};

static struct mxt_platform_data mxt_platform_data_3d = {
	.config_array		= mxt_config_array_3d,
	.config_array_size	= ARRAY_SIZE(mxt_config_array_3d),
	.panel_minx		= 0,
	.panel_maxx		= 1919,
	.panel_miny		= 0,
	.panel_maxy		= 1199,
	.disp_minx		= 0,
	.disp_maxx		= 1919,
	.disp_miny		= 0,
	.disp_maxy		= 1199,
	.irqflags		= IRQF_TRIGGER_FALLING,
	.i2c_pull_up		= true,
	.reset_gpio		= MXT_TS_RESET_GPIO,
	.irq_gpio		= MXT_TS_GPIO_IRQ,
};

static struct i2c_board_info mxt_device_info[] __initdata = {
	{
		I2C_BOARD_INFO("atmel_mxt_ts", 0x5b),
		.irq = MSM_GPIO_TO_INT(MXT_TS_GPIO_IRQ),
	},
};

static struct msm_mhl_platform_data mhl_platform_data = {
	.irq = MSM_GPIO_TO_INT(4),
	.gpio_mhl_int = MHL_GPIO_INT,
	.gpio_mhl_reset = MHL_GPIO_RESET,
	.gpio_mhl_power = 0,
	.gpio_hdmi_mhl_mux = 0,
};

#ifndef	CONFIG_SHSYS_CUST
static struct i2c_board_info sii_device_info[] __initdata = {
	{
#ifdef CONFIG_FB_MSM_HDMI_MHL_8334
		/*
		 * keeps SI 8334 as the default
		 * MHL TX
		 */
		I2C_BOARD_INFO("sii8334", 0x39),
		.platform_data = &mhl_platform_data,
#endif
#ifdef CONFIG_FB_MSM_HDMI_MHL_9244
		I2C_BOARD_INFO("Sil-9244", 0x39),
		.irq = MSM_GPIO_TO_INT(15),
#endif /* CONFIG_MSM_HDMI_MHL */
		.flags = I2C_CLIENT_WAKE,
	},
};
#endif	/* CONFIG_SHSYS_CUST */

#ifdef	CONFIG_SHSYS_CUST
static struct msm_i2c_platform_data msm8960_i2c_qup_gsbi2_pdata = {
	.clk_freq = 384000,
	.src_clk_rate = 24000000,
};

#endif	/* CONFIG_SHSYS_CUST */

static struct msm_i2c_platform_data msm8960_i2c_qup_gsbi4_pdata = {
#ifdef	CONFIG_SHSYS_CUST
	.clk_freq = 384000,
#else	/* CONFIG_SHSYS_CUST */
	.clk_freq = 100000,
#endif	/* CONFIG_SHSYS_CUST */
	.src_clk_rate = 24000000,
};

#ifndef	CONFIG_SHSYS_CUST
static struct msm_i2c_platform_data msm8960_i2c_qup_gsbi3_pdata = {
	.clk_freq = 100000,
	.src_clk_rate = 24000000,
};
#endif	/* CONFIG_SHSYS_CUST */

static struct msm_i2c_platform_data msm8960_i2c_qup_gsbi10_pdata = {
#ifdef	CONFIG_SHSYS_CUST
	.clk_freq = 384000,
#else	/* CONFIG_SHSYS_CUST */
	.clk_freq = 100000,
#endif	/* CONFIG_SHSYS_CUST */
	.src_clk_rate = 24000000,
#ifdef CONFIG_SHIRDA
    .use_gsbi_shared_mode = 1,
#endif  /* CONFIG_SHIRDA */
};

static struct msm_i2c_platform_data msm8960_i2c_qup_gsbi12_pdata = {
#ifdef	CONFIG_SHSYS_CUST
	.clk_freq = 384000,
#else	/* CONFIG_SHSYS_CUST */
	.clk_freq = 100000,
#endif	/* CONFIG_SHSYS_CUST */
	.src_clk_rate = 24000000,
/* ADD_S shmds GSBI12 from DSPS to Krait */
	.use_gsbi_shared_mode = 1,
/* ADD_E shmds GSBI12 from DSPS to Krait */
};

static struct msm_pm_sleep_status_data msm_pm_slp_sts_data = {
	.base_addr = MSM_ACC0_BASE + 0x08,
	.cpu_offset = MSM_ACC1_BASE - MSM_ACC0_BASE,
	.mask = 1UL << 13,
};

static struct ks8851_pdata spi_eth_pdata = {
	.irq_gpio = KS8851_IRQ_GPIO,
	.rst_gpio = KS8851_RST_GPIO,
};

#if defined( CONFIG_SHTPS_SY3X00_DEV )
static int msm_shtps_gpio_setup(struct device *dev)
{
    gpio_request(SHTPS_SY3X00_GPIO_IRQ, "shtps_irq");
    gpio_direction_input(SHTPS_SY3X00_GPIO_IRQ);

    gpio_request(SHTPS_SY3X00_GPIO_RST, "shtps_rst");
    gpio_direction_output(SHTPS_SY3X00_GPIO_RST, 0);

    return 0;
}

static void msm_shtps_gpio_teardown(struct device *dev)
{
    gpio_free(SHTPS_SY3X00_GPIO_IRQ);
    gpio_free(SHTPS_SY3X00_GPIO_RST);
}

static struct shtps_platform_data shtps_pdata = {
    .setup    = msm_shtps_gpio_setup,
    .teardown = msm_shtps_gpio_teardown,
    .gpio_rst = SHTPS_SY3X00_GPIO_RST,
};
#endif /* #if defined( CONFIG_SHTPS_SY3X00_DEV ) */

static struct spi_board_info spi_board_info[] __initdata = {
	{
		.modalias               = "ks8851",
		.irq                    = MSM_GPIO_TO_INT(KS8851_IRQ_GPIO),
		.max_speed_hz           = 19200000,
		.bus_num                = 0,
		.chip_select            = 0,
		.mode                   = SPI_MODE_0,
		.platform_data		= &spi_eth_pdata
	},
	{
		.modalias               = "dsi_novatek_3d_panel_spi",
		.max_speed_hz           = 10800000,
		.bus_num                = 0,
		.chip_select            = 1,
		.mode                   = SPI_MODE_0,
	},
#ifdef CONFIG_SHLCDC_BOARD
    {
        .modalias               = "mipi_spi",
        .max_speed_hz           = 1100000,
        .bus_num                = 5,
        .chip_select            = 0,
        .mode                   = SPI_MODE_3,
    },
#endif /* CONFIG_SHLCDC_BOARD */
#ifdef CONFIG_SHTPS_TMA4XX_DEV
    {
        .modalias      = "SH_touchpanel",
        .irq           = MSM_GPIO_TO_INT(TOUCH_GPIO_IRQ_CYTTSP),
        .max_speed_hz  = 1100000,
        .bus_num       = 9,
        .chip_select   = 0,
        .mode          = SPI_MODE_0,
        .platform_data = &cyttsp4_spi_touch_platform_data,
    },
#elif defined( CONFIG_SHTPS_SY3X00_DEV )
    {
        .modalias      = SH_TOUCH_DEVNAME,
        .irq           = MSM_GPIO_TO_INT(SHTPS_SY3X00_GPIO_IRQ),
        .max_speed_hz  = SHTPS_SY3X00_SPI_CLOCK,
        .bus_num       = 9,
        .chip_select   = 0,
        .mode          = SPI_MODE_3,
        .platform_data = &shtps_pdata,
    },
#endif  /* CONFIG_SHTPS_TMA4XX_TMA400 */
};

static struct platform_device msm_device_saw_core0 = {
	.name          = "saw-regulator",
	.id            = 0,
	.dev	= {
		.platform_data = &msm_saw_regulator_pdata_s5,
	},
};

static struct platform_device msm_device_saw_core1 = {
	.name          = "saw-regulator",
	.id            = 1,
	.dev	= {
		.platform_data = &msm_saw_regulator_pdata_s6,
	},
};

static struct tsens_platform_data msm_tsens_pdata  = {
		.slope			= {910, 910, 910, 910, 910},
		.tsens_factor		= 1000,
		.hw_type		= MSM_8960,
		.tsens_num_sensor	= 5,
};

static struct platform_device msm_tsens_device = {
	.name   = "tsens8960-tm",
	.id = -1,
};

static struct msm_thermal_data msm_thermal_pdata = {
	.sensor_id = 0,
	.poll_ms = 1000,
	.limit_temp = 60,
	.temp_hysteresis = 10,
	.limit_freq = 918000,
};

#ifdef CONFIG_MSM_FAKE_BATTERY
static struct platform_device fish_battery_device = {
	.name = "fish_battery",
};
#endif

static struct platform_device msm8960_device_ext_5v_vreg __devinitdata = {
	.name	= GPIO_REGULATOR_DEV_NAME,
	.id	= PM8921_MPP_PM_TO_SYS(7),
	.dev	= {
		.platform_data = &msm_gpio_regulator_pdata[GPIO_VREG_ID_EXT_5V],
	},
};

static struct platform_device msm8960_device_ext_l2_vreg __devinitdata = {
	.name	= GPIO_REGULATOR_DEV_NAME,
	.id	= 91,
	.dev	= {
		.platform_data = &msm_gpio_regulator_pdata[GPIO_VREG_ID_EXT_L2],
	},
};

static struct platform_device msm8960_device_ext_3p3v_vreg __devinitdata = {
	.name	= GPIO_REGULATOR_DEV_NAME,
	.id	= PM8921_GPIO_PM_TO_SYS(17),
	.dev	= {
		.platform_data =
			&msm_gpio_regulator_pdata[GPIO_VREG_ID_EXT_3P3V],
	},
};

static struct platform_device msm8960_device_ext_otg_sw_vreg __devinitdata = {
	.name	= GPIO_REGULATOR_DEV_NAME,
	.id	= PM8921_GPIO_PM_TO_SYS(42),
	.dev	= {
		.platform_data =
			&msm_gpio_regulator_pdata[GPIO_VREG_ID_EXT_OTG_SW],
	},
};

static struct platform_device msm8960_device_rpm_regulator __devinitdata = {
	.name	= "rpm-regulator",
	.id	= -1,
	.dev	= {
		.platform_data = &msm_rpm_regulator_pdata,
	},
};

#ifdef CONFIG_BATTERY_SH
static struct platform_device msm_device_shbatt = {
    .name = "shbatt",
};
static struct platform_device msm_device_shchg = {
    .name = "shchg",
};
#endif /* CONFIG_BATTERY_SH */

#ifdef CONFIG_SERIAL_MSM_HS
static int configure_uart_gpios(int on)
{
	int ret = 0, i;
	int uart_gpios[] = {93, 94, 95, 96};

	for (i = 0; i < ARRAY_SIZE(uart_gpios); i++) {
		if (on) {
			ret = gpio_request(uart_gpios[i], NULL);
			if (ret) {
				pr_err("%s: unable to request uart gpio[%d]\n",
						__func__, uart_gpios[i]);
				break;
			}
		} else {
			gpio_free(uart_gpios[i]);
		}
	}

	if (ret && on && i)
		for (; i >= 0; i--)
			gpio_free(uart_gpios[i]);
	return ret;
}

static struct msm_serial_hs_platform_data msm_uart_dm9_pdata = {
	.gpio_config	= configure_uart_gpios,
};
#else
static struct msm_serial_hs_platform_data msm_uart_dm9_pdata;
#endif

static struct platform_device *common_devices[] __initdata = {
	&msm8960_device_acpuclk,
	&msm8960_device_dmov,
	&msm_device_smd,
#ifdef	CONFIG_SHFELICA
    &msm8960_device_uart_gsbi2,
#endif	/* CONFIG_SHFELICA */
#ifdef	CONFIG_SHIRDA
	&msm_device_uart_dm10,
#endif	/* CONFIG_SHIRDA */
#if	defined(CONFIG_SERIAL_MSM_HSL_CONSOLE)
    &msm8960_device_uart_gsbi11,
#endif	/* CONFIG_SERIAL_MSM_HSL_CONSOLE */
#ifndef	CONFIG_SHSYS_CUST
	&msm_device_uart_dm6,
	&msm_device_uart_dm9,
#endif	/* CONFIG_SHSYS_CUST */
	&msm_device_saw_core0,
	&msm_device_saw_core1,
	&msm8960_device_ext_5v_vreg,
	&msm8960_device_ssbi_pmic,
	&msm8960_device_ext_otg_sw_vreg,
#ifdef CONFIG_SHLCDC_BOARD
    &msm8960_device_qup_spi_gsbi5,
#endif /* CONFIG_SHLCDC_BOARD */
#ifdef	CONFIG_SHSYS_CUST
	&msm8960_device_qup_spi_gsbi9,
	&msm8960_device_qup_i2c_gsbi2,
#else
	&msm8960_device_qup_spi_gsbi1,
	&msm8960_device_qup_i2c_gsbi3,
#endif
	&msm8960_device_qup_i2c_gsbi4,
	&msm8960_device_qup_i2c_gsbi10,
/* MOD_S shmds GSBI12 from DSPS to Krait */
/* #ifndef CONFIG_MSM_DSPS */
	&msm8960_device_qup_i2c_gsbi12,
/* #endif */
/* MOD_E shmds GSBI12 from DSPS to Krait */
	&msm_slim_ctrl,
	&msm_device_wcnss_wlan,
#if defined(CONFIG_QSEECOM)
	&qseecom_device,
#endif
#if defined(CONFIG_CRYPTO_DEV_QCRYPTO) || \
		defined(CONFIG_CRYPTO_DEV_QCRYPTO_MODULE)
	&qcrypto_device,
#endif

#if defined(CONFIG_CRYPTO_DEV_QCEDEV) || \
		defined(CONFIG_CRYPTO_DEV_QCEDEV_MODULE)
	&qcedev_device,
#endif
#ifdef CONFIG_MSM_ROTATOR
	&msm_rotator_device,
#endif
	&msm_device_sps,
#ifdef CONFIG_MSM_FAKE_BATTERY
	&fish_battery_device,
#endif
	&msm8960_fmem_device,
#ifdef CONFIG_ANDROID_PMEM
#ifndef CONFIG_MSM_MULTIMEDIA_USE_ION
	&msm8960_android_pmem_device,
	&msm8960_android_pmem_adsp_device,
	&msm8960_android_pmem_audio_device,
#endif
#endif
	&msm_device_vidc,
	&msm_device_bam_dmux,
	&msm_fm_platform_init,
#if defined(CONFIG_TSIF) || defined(CONFIG_TSIF_MODULE)
#ifdef CONFIG_MSM_USE_TSIF1
	&msm_device_tsif[1],
#else
	&msm_device_tsif[0],
#endif
#endif
	&msm_device_tspp,
#ifdef CONFIG_HW_RANDOM_MSM
	&msm_device_rng,
#endif
#ifdef CONFIG_ION_MSM
	&msm8960_ion_dev,
#endif
	&msm8960_rpm_device,
	&msm8960_rpm_log_device,
	&msm8960_rpm_stat_device,
	&msm_device_tz_log,
#ifdef CONFIG_MSM_QDSS
	&msm_qdss_device,
	&msm_etb_device,
	&msm_tpiu_device,
	&msm_funnel_device,
	&msm_etm_device,
#endif
	&msm_device_dspcrashd_8960,
	&msm8960_device_watchdog,
	&msm8960_rtb_device,
	&msm8960_cpu_idle_device,
	&msm8960_msm_gov_device,
	&msm8960_device_cache_erp,
	&msm8960_device_ebi1_ch0_erp,
	&msm8960_device_ebi1_ch1_erp,
	&msm8960_cache_dump_device,
	&msm8960_iommu_domain_device,
	&msm_tsens_device,
#ifdef CONFIG_BATTERY_SH
    &msm_device_shbatt,
    &msm_device_shchg,
#endif /* CONFIG_BATTERY_SH */
};

static struct platform_device *sim_devices[] __initdata = {
#ifndef	CONFIG_SHSYS_CUST
	&msm8960_device_uart_gsbi5,
#endif
	&msm8960_device_otg,
	&msm8960_device_gadget_peripheral,
	&msm_device_hsusb_host,
	&msm_device_hsic_host,
	&android_usb_device,
	&msm_device_vidc,
	&msm_bus_apps_fabric,
	&msm_bus_sys_fabric,
	&msm_bus_mm_fabric,
	&msm_bus_sys_fpb,
	&msm_bus_cpss_fpb,
	&msm_pcm,
	&msm_multi_ch_pcm,
	&msm_pcm_routing,
	&msm_cpudai0,
	&msm_cpudai1,
	&msm8960_cpudai_slimbus_2_rx,
	&msm8960_cpudai_slimbus_2_tx,
	&msm_cpudai_hdmi_rx,
	&msm_cpudai_bt_rx,
	&msm_cpudai_bt_tx,
	&msm_cpudai_fm_rx,
	&msm_cpudai_fm_tx,
	&msm_cpudai_auxpcm_rx,
	&msm_cpudai_auxpcm_tx,
	&msm_cpu_fe,
	&msm_stub_codec,
	&msm_voice,
	&msm_voip,
	&msm_lpa_pcm,
	&msm_compr_dsp,
	&msm_cpudai_incall_music_rx,
	&msm_cpudai_incall_record_rx,
	&msm_cpudai_incall_record_tx,

#if defined(CONFIG_CRYPTO_DEV_QCRYPTO) || \
		defined(CONFIG_CRYPTO_DEV_QCRYPTO_MODULE)
	&qcrypto_device,
#endif

#if defined(CONFIG_CRYPTO_DEV_QCEDEV) || \
		defined(CONFIG_CRYPTO_DEV_QCEDEV_MODULE)
	&qcedev_device,
#endif
#ifdef CONFIG_USB_SWIC
    &msm8960_device_shswic,
#endif /* CONFIG_USB_SWIC */
};

static struct platform_device *rumi3_devices[] __initdata = {
#ifndef	CONFIG_SHSYS_CUST
	&msm8960_device_uart_gsbi5,
#endif
	&msm_kgsl_3d0,
	&msm_kgsl_2d0,
	&msm_kgsl_2d1,
#ifdef CONFIG_MSM_GEMINI
	&msm8960_gemini_device,
#endif
#ifdef CONFIG_MSM_MERCURY
	&msm8960_mercury_device,
#endif
};

static struct platform_device *cdp_devices[] __initdata = {
	&msm_8960_q6_lpass,
	&msm_8960_q6_mss_fw,
	&msm_8960_q6_mss_sw,
	&msm_8960_riva,
	&msm_pil_tzapps,
	&msm_pil_dsps,
	&msm_pil_vidc,
	&msm8960_device_otg,
	&msm8960_device_gadget_peripheral,
	&msm_device_hsusb_host,
	&android_usb_device,
	&msm_pcm,
	&msm_multi_ch_pcm,
	&msm_lowlatency_pcm,
	&msm_pcm_routing,
	&msm_cpudai0,
	&msm_cpudai1,
	&msm8960_cpudai_slimbus_2_rx,
	&msm8960_cpudai_slimbus_2_tx,
	&msm_cpudai_hdmi_rx,
	&msm_cpudai_bt_rx,
	&msm_cpudai_bt_tx,
	&msm_cpudai_fm_rx,
	&msm_cpudai_fm_tx,
	&msm_cpudai_auxpcm_rx,
	&msm_cpudai_auxpcm_tx,
	&msm_cpu_fe,
	&msm_stub_codec,
	&msm_kgsl_3d0,
#ifdef CONFIG_MSM_KGSL_2D
	&msm_kgsl_2d0,
	&msm_kgsl_2d1,
#endif
#ifdef CONFIG_MSM_GEMINI
	&msm8960_gemini_device,
#endif
#ifdef CONFIG_MSM_MERCURY
	&msm8960_mercury_device,
#endif
	&msm_voice,
	&msm_voip,
	&msm_lpa_pcm,
	&msm_cpudai_afe_01_rx,
	&msm_cpudai_afe_01_tx,
	&msm_cpudai_afe_02_rx,
	&msm_cpudai_afe_02_tx,
	&msm_pcm_afe,
	&msm_compr_dsp,
	&msm_cpudai_incall_music_rx,
	&msm_cpudai_incall_record_rx,
	&msm_cpudai_incall_record_tx,
	&msm_pcm_hostless,
	&msm_bus_apps_fabric,
	&msm_bus_sys_fabric,
	&msm_bus_mm_fabric,
	&msm_bus_sys_fpb,
	&msm_bus_cpss_fpb,
#ifdef CONFIG_USB_SWIC
    &msm8960_device_shswic,
#endif /* CONFIG_USB_SWIC */
};

#ifdef CONFIG_SHIRDA
#define GSBIn_HCLK_CTL_REG(n)   (0x009029C0 + (32 * (n-1)))
#define MSM_GSBI10_PHYS     (0x1A200000)
#define CLK_BRANCH_ENA      (1<<4)
#define UART_I2C_PROTOCOL   (0x6<<4)
#endif  /* CONFIG_SHIRDA */

static void __init msm8960_i2c_init(void)
{
/* ADD_S shmds GSBI12 from DSPS to Krait */
    void *gsbi_mem = ioremap_nocache(0x12480000, 4);
    struct clk *iface_clk;
#ifdef CONFIG_SHIRDA
    void *gsbi_pclk = ioremap_nocache(GSBIn_HCLK_CTL_REG(10), 4);
    void *gsbi_ctrl = ioremap_nocache(MSM_GSBI10_PHYS, 4);
    uint32_t ClkStatus;

    if (!(ClkStatus = (readl_relaxed(gsbi_pclk) & CLK_BRANCH_ENA))) {
        writel_relaxed(CLK_BRANCH_ENA, gsbi_ctrl);
        mb();
    }
    writel_relaxed(UART_I2C_PROTOCOL, gsbi_ctrl);
    mb();

    if (!ClkStatus) {
        writel_relaxed(0x00, gsbi_pclk);
    }

    iounmap(gsbi_ctrl);
    iounmap(gsbi_pclk);
#endif  /* CONFIG_SHIRDA */

    iface_clk = clk_get_sys("qup_i2c.12", "iface_clk");

    BUG_ON(IS_ERR(iface_clk));

    clk_prepare_enable(iface_clk);

    /* Setting protocol code to 0x60 for dual UART/I2C in GSBI12 */
    writel_relaxed(0x6 << 4, gsbi_mem);
    /* Ensure protocol code is written before proceeding further */
    mb();
    iounmap(gsbi_mem);
    clk_disable_unprepare(iface_clk);
/* ADD_E shmds GSBI12 from DSPS to Krait */
	
	if (socinfo_get_platform_subtype() == PLATFORM_SUBTYPE_SGLTE)
		msm8960_i2c_qup_gsbi4_pdata.keep_ahb_clk_on = 1;

#ifdef	CONFIG_SHSYS_CUST
	msm8960_device_qup_i2c_gsbi2.dev.platform_data = &msm8960_i2c_qup_gsbi2_pdata;
#endif	/* CONFIG_SHSYS_CUST */
	msm8960_device_qup_i2c_gsbi4.dev.platform_data =
					&msm8960_i2c_qup_gsbi4_pdata;
#ifndef	CONFIG_SHSYS_CUST
	msm8960_device_qup_i2c_gsbi3.dev.platform_data =
					&msm8960_i2c_qup_gsbi3_pdata;
#endif	/* CONFIG_SHSYS_CUST */

	msm8960_device_qup_i2c_gsbi10.dev.platform_data =
					&msm8960_i2c_qup_gsbi10_pdata;

	msm8960_device_qup_i2c_gsbi12.dev.platform_data =
					&msm8960_i2c_qup_gsbi12_pdata;
}

static void __init msm8960_gfx_init(void)
{
	uint32_t soc_platform_version = socinfo_get_version();
	if (SOCINFO_VERSION_MAJOR(soc_platform_version) == 1) {
		struct kgsl_device_platform_data *kgsl_3d0_pdata =
				msm_kgsl_3d0.dev.platform_data;
		kgsl_3d0_pdata->pwrlevel[0].gpu_freq = 320000000;
		kgsl_3d0_pdata->pwrlevel[1].gpu_freq = 266667000;
	}
}

static struct msm_rpmrs_level msm_rpmrs_levels[] = {
	{
		MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT,
		MSM_RPMRS_LIMITS(ON, ACTIVE, MAX, ACTIVE),
		true,
		1, 784, 180000, 100,
	},

	{
		MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE,
		MSM_RPMRS_LIMITS(ON, ACTIVE, MAX, ACTIVE),
		true,
		1300, 228, 1200000, 2000,
	},

	{
		MSM_PM_SLEEP_MODE_POWER_COLLAPSE,
		MSM_RPMRS_LIMITS(ON, GDHS, MAX, ACTIVE),
		false,
		2000, 138, 1208400, 3200,
	},

	{
		MSM_PM_SLEEP_MODE_POWER_COLLAPSE,
		MSM_RPMRS_LIMITS(ON, HSFS_OPEN, ACTIVE, RET_HIGH),
		false,
		6000, 119, 1850300, 9000,
	},

	{
		MSM_PM_SLEEP_MODE_POWER_COLLAPSE,
		MSM_RPMRS_LIMITS(OFF, GDHS, MAX, ACTIVE),
		false,
		9200, 68, 2839200, 16400,
	},

	{
		MSM_PM_SLEEP_MODE_POWER_COLLAPSE,
		MSM_RPMRS_LIMITS(OFF, HSFS_OPEN, MAX, ACTIVE),
		false,
		10300, 63, 3128000, 18200,
	},

	{
		MSM_PM_SLEEP_MODE_POWER_COLLAPSE,
		MSM_RPMRS_LIMITS(OFF, HSFS_OPEN, ACTIVE, RET_HIGH),
		false,
		18000, 10, 4602600, 27000,
	},

	{
		MSM_PM_SLEEP_MODE_POWER_COLLAPSE,
		MSM_RPMRS_LIMITS(OFF, HSFS_OPEN, RET_HIGH, RET_LOW),
		false,
		20000, 2, 5752000, 32000,
	},
};

static struct msm_rpmrs_platform_data msm_rpmrs_data __initdata = {
	.levels = &msm_rpmrs_levels[0],
	.num_levels = ARRAY_SIZE(msm_rpmrs_levels),
	.vdd_mem_levels  = {
		[MSM_RPMRS_VDD_MEM_RET_LOW]	= 750000,
		[MSM_RPMRS_VDD_MEM_RET_HIGH]	= 750000,
		[MSM_RPMRS_VDD_MEM_ACTIVE]	= 1050000,
		[MSM_RPMRS_VDD_MEM_MAX]		= 1150000,
	},
	.vdd_dig_levels = {
		[MSM_RPMRS_VDD_DIG_RET_LOW]	= 500000,
		[MSM_RPMRS_VDD_DIG_RET_HIGH]	= 750000,
		[MSM_RPMRS_VDD_DIG_ACTIVE]	= 950000,
		[MSM_RPMRS_VDD_DIG_MAX]		= 1150000,
	},
	.vdd_mask = 0x7FFFFF,
	.rpmrs_target_id = {
		[MSM_RPMRS_ID_PXO_CLK]		= MSM_RPM_ID_PXO_CLK,
		[MSM_RPMRS_ID_L2_CACHE_CTL]	= MSM_RPM_ID_LAST,
		[MSM_RPMRS_ID_VDD_DIG_0]	= MSM_RPM_ID_PM8921_S3_0,
		[MSM_RPMRS_ID_VDD_DIG_1]	= MSM_RPM_ID_PM8921_S3_1,
		[MSM_RPMRS_ID_VDD_MEM_0]	= MSM_RPM_ID_PM8921_L24_0,
		[MSM_RPMRS_ID_VDD_MEM_1]	= MSM_RPM_ID_PM8921_L24_1,
		[MSM_RPMRS_ID_RPM_CTL]		= MSM_RPM_ID_RPM_CTL,
	},
};

static struct msm_pm_boot_platform_data msm_pm_boot_pdata __initdata = {
	.mode = MSM_PM_BOOT_CONFIG_TZ,
};

#ifdef CONFIG_I2C
#define I2C_SURF 1
#define I2C_FFA  (1 << 1)
#define I2C_RUMI (1 << 2)
#define I2C_SIM  (1 << 3)
#define I2C_FLUID (1 << 4)
#define I2C_LIQUID (1 << 5)

struct i2c_registry {
	u8                     machs;
	int                    bus;
	struct i2c_board_info *info;
	int                    len;
};

#ifdef CONFIG_SHFELICA
static struct i2c_board_info msm_felica_boardinfo[] __initdata = {
    {
    I2C_BOARD_INFO(SH_MFD_CEN, 0xAE >> 1),
    },
};
#endif /* CONFIG_SHFELICA */

#ifdef CONFIG_SII8334_MHL_TX
static struct i2c_board_info mhl_i2c_boardinfo[] __initdata = {
    {
        I2C_BOARD_INFO("SiI8334", 0x39),
        .flags = I2C_CLIENT_WAKE,
        .irq = MSM_GPIO_TO_INT(43),
    }
};
#endif /* CONFIG_SII8334_MHL_TX */

#ifdef CONFIG_USB_SWIC
static struct i2c_board_info shswic_i2c_info[] __initdata = {
    {
        I2C_BOARD_INFO("shswic_i2c", 0xD4 >> 1),
    },
};
#endif /* CONFIG_USB_SWIC */
/* Sensors DSPS platform data */
#ifdef CONFIG_MSM_DSPS
#define DSPS_PIL_GENERIC_NAME		"dsps"
#endif /* CONFIG_MSM_DSPS */

static void __init msm8960_init_dsps(void)
{
#ifdef CONFIG_MSM_DSPS
	struct msm_dsps_platform_data *pdata =
		msm_dsps_device.dev.platform_data;
	pdata->pil_name = DSPS_PIL_GENERIC_NAME;
	pdata->gpios = NULL;
	pdata->gpios_num = 0;

	platform_device_register(&msm_dsps_device);
#endif /* CONFIG_MSM_DSPS */
}

static int hsic_peripheral_status = 1;
static DEFINE_MUTEX(hsic_status_lock);

void peripheral_connect()
{
	mutex_lock(&hsic_status_lock);
	if (hsic_peripheral_status)
		goto out;
	platform_device_add(&msm_device_hsic_host);
	hsic_peripheral_status = 1;
out:
	mutex_unlock(&hsic_status_lock);
}
EXPORT_SYMBOL(peripheral_connect);

void peripheral_disconnect()
{
	mutex_lock(&hsic_status_lock);
	if (!hsic_peripheral_status)
		goto out;
	platform_device_del(&msm_device_hsic_host);
	hsic_peripheral_status = 0;
out:
	mutex_unlock(&hsic_status_lock);
}
EXPORT_SYMBOL(peripheral_disconnect);

static void __init msm8960_init_smsc_hub(void)
{
	uint32_t version = socinfo_get_version();

	if (SOCINFO_VERSION_MAJOR(version) == 1)
		return;

	if (machine_is_msm8960_liquid())
		platform_device_register(&smsc_hub_device);
}

static void __init msm8960_init_hsic(void)
{
#ifdef CONFIG_USB_EHCI_MSM_HSIC
	uint32_t version = socinfo_get_version();

	if (SOCINFO_VERSION_MAJOR(version) == 1)
		return;

	if (machine_is_msm8960_liquid())
		platform_device_register(&msm_device_hsic_host);
#endif
}

#ifdef CONFIG_ISL9519_CHARGER
static struct isl_platform_data isl_data __initdata = {
	.valid_n_gpio		= 0,	/* Not required when notify-by-pmic */
	.chg_detection_config	= NULL,	/* Not required when notify-by-pmic */
	.max_system_voltage	= 4200,
	.min_system_voltage	= 3200,
	.chgcurrent		= 1900,
	.term_current		= 0,
	.input_current		= 2048,
};

static struct i2c_board_info isl_charger_i2c_info[] __initdata = {
	{
		I2C_BOARD_INFO("isl9519q", 0x9),
		.irq		= 0,	/* Not required when notify-by-pmic */
		.platform_data	= &isl_data,
	},
};
#endif /* CONFIG_ISL9519_CHARGER */

#ifdef CONFIG_SH_AUDIO_DRIVER
/* SP-AMP control     */
#ifdef CONFIG_SHSPAMP_TPA2028D1
static struct shspamp_platform_data shspamp_data = {
    .gpio_shspamp_spk_en = PM8921_GPIO_PM_TO_SYS(35),
};
static struct i2c_board_info snd_shspamp_i2c_info[] __initdata = {
    {
        I2C_BOARD_INFO("shspamp_i2c", 0xB0 >> 1),
        .platform_data = &shspamp_data,
    },
};
#elif defined(CONFIG_SHSPAMP_AK7811)
static struct shspamp_platform_data shspamp_data = {
    .gpio_shspamp_spk_en = PM8921_GPIO_PM_TO_SYS(35),
};
static struct i2c_board_info snd_shspamp_i2c_info[] __initdata = {
    {
        I2C_BOARD_INFO("shspamp_i2c", 0xEC >> 1),
        .platform_data = &shspamp_data,
    },
};
#endif /* CONFIG_SHSPAMP_TPA2028D1 */
#endif /* CONFIG_SH_AUDIO_DRIVER */

#ifdef CONFIG_SHMDS
#ifdef CONFIG_MPU_SENSORS_MPU3050
static struct mpu_platform_data mpu3050_data = {
	.int_config  = 0x10,
	.orientation = { -1,  0,  0,
					  0, -1,  0,
					  0,  0,  1 },
};

/* accel */
static struct ext_slave_platform_data inv_mpu_lis331dlh_data = {
	.bus         = EXT_SLAVE_BUS_SECONDARY,
	.address     = 0x19,
	.orientation = {  1,  0,  0,
					  0,  1,  0,
					  0,  0,  1 },
};
/* compass */
static struct ext_slave_platform_data inv_mpu_hscdtd004a_data = {
	.bus         = EXT_SLAVE_BUS_PRIMARY,
	.address     = 0x0C,
	.orientation = {  1,  0,  0,
					  0,  1,  0,
					  0,  0,  1 },
};
#endif	/* CONFIG_MPU_SENSORS_MPU3050 */

static struct i2c_board_info shmds_i2c_info[] __initdata = {
#ifdef CONFIG_MPU_SENSORS_MPU3050
	{
		I2C_BOARD_INFO("mpu3050", 0x68),
		.irq 		   = MSM_GPIO_TO_INT(GPIO_GYRO_INT),
		.platform_data = &mpu3050_data,
	},
	{
		I2C_BOARD_INFO("lis331", 0x19),
		.irq		   = MSM_GPIO_TO_INT(GPIO_ACCEL_INT),
		.platform_data = &inv_mpu_lis331dlh_data,
	},
	{
		I2C_BOARD_INFO("hscdtd004a", 0x0C),
		//.irq		   = MSM_GPIO_TO_INT(GPIO_COMPASS_INT),
		.platform_data = &inv_mpu_hscdtd004a_data,
	},
#endif	/* CONFIG_MPU_SENSORS_MPU3050 */
#ifdef CONFIG_SENSORS_AMI603
	{
		I2C_BOARD_INFO("ami_sensor", 0x1E >> 1),
		.flags = I2C_CLIENT_WAKE,
		.irq   = MSM_GPIO_TO_INT(AMI_GPIO_INT)
	},
#endif /* CONFIG_SENSORS_AMI603 */
};
#endif	/* CONFIG_SHMDS */

#ifdef CONFIG_SH_AUDIO_DRIVER /*02-141*/
#ifdef CONFIG_SHRECEIVER_LM48560
static struct shreceiver_platform_data shreceiver_data = {
    .gpio_shreceiver_receiver_en = PM8921_GPIO_PM_TO_SYS(21),
};
static struct i2c_board_info snd_shreceiver_i2c_info[] __initdata = {
    {
        I2C_BOARD_INFO("shreceiver_i2c", 0xDE >> 1),
        .platform_data = &shreceiver_data,
    },
};
#endif /* CONFIG_SHRECEIVER_LM48560 */
#endif /* CONFIG_SH_AUDIO_DRIVER *//*02-141*/

static struct i2c_board_info liquid_io_expander_i2c_info[] __initdata = {
	{
		I2C_BOARD_INFO("sx1508q", 0x20),
		.platform_data = &msm8960_sx150x_data[SX150X_LIQUID]
	},
};

static struct i2c_registry msm8960_i2c_devices[] __initdata = {
#ifdef CONFIG_SHFELICA
    {
        I2C_SURF | I2C_FFA | I2C_FLUID,
        MSM_8960_GSBI2_QUP_I2C_BUS_ID,
        msm_felica_boardinfo,
        ARRAY_SIZE(msm_felica_boardinfo),
    },
#endif /* CONFIG_SHFELICA */
#ifdef CONFIG_ISL9519_CHARGER
	{
		I2C_LIQUID,
		MSM_8960_GSBI10_QUP_I2C_BUS_ID,
		isl_charger_i2c_info,
		ARRAY_SIZE(isl_charger_i2c_info),
	},
#endif /* CONFIG_ISL9519_CHARGER */
#ifndef CONFIG_SHTPS_TMA4XX_DEV
	{
		I2C_SURF | I2C_FFA | I2C_FLUID,
		MSM_8960_GSBI3_QUP_I2C_BUS_ID,
		cyttsp_info,
		ARRAY_SIZE(cyttsp_info),
	},
#endif  /* CONFIG_SHTPS_TMA4XX_DEV */
	{
		I2C_LIQUID,
		MSM_8960_GSBI3_QUP_I2C_BUS_ID,
		mxt_device_info,
		ARRAY_SIZE(mxt_device_info),
	},
#ifndef	CONFIG_SHSYS_CUST
	{
		I2C_SURF | I2C_FFA | I2C_LIQUID,
		MSM_8960_GSBI10_QUP_I2C_BUS_ID,
		sii_device_info,
		ARRAY_SIZE(sii_device_info),
	},
#endif	/* CONFIG_SHSYS_CUST */
#ifdef CONFIG_SII8334_MHL_TX
    {
        I2C_SURF | I2C_FFA | I2C_FLUID,
        MSM_8960_GSBI2_QUP_I2C_BUS_ID,
        mhl_i2c_boardinfo,
        ARRAY_SIZE(mhl_i2c_boardinfo),
    },
#endif /* CONFIG_SII8334_MHL_TX */
	{
		I2C_LIQUID,
		MSM_8960_GSBI10_QUP_I2C_BUS_ID,
		msm_isa1200_board_info,
		ARRAY_SIZE(msm_isa1200_board_info),
	},
	{
		I2C_LIQUID,
		MSM_8960_GSBI10_QUP_I2C_BUS_ID,
		liquid_io_expander_i2c_info,
		ARRAY_SIZE(liquid_io_expander_i2c_info),
	},
#ifdef CONFIG_USB_SWIC
    {
        I2C_SURF | I2C_FFA | I2C_FLUID,
        MSM_8960_GSBI2_QUP_I2C_BUS_ID,
        shswic_i2c_info,
        ARRAY_SIZE(shswic_i2c_info),
    },
#endif /* CONFIG_USB_SWIC */
#ifdef CONFIG_SHLCDC_BOARD
    {
        I2C_SURF | I2C_FFA | I2C_FLUID | I2C_RUMI,
        MSM_8960_GSBI10_QUP_I2C_BUS_ID,
        msm_lcd_board_info,
        ARRAY_SIZE(msm_lcd_board_info),
    },
#endif /* CONFIG_SHLCDC_BOARD */
#ifdef CONFIG_SHLCDC_LED_BD2802GU
    {
        I2C_SURF | I2C_FFA | I2C_FLUID | I2C_RUMI,
        MSM_8960_GSBI10_QUP_I2C_BUS_ID,
        msm_ledc_board_info,
        ARRAY_SIZE(msm_ledc_board_info),
    },
#endif /* CONFIG_SHLCDC_LED_BD2802GU */
#ifdef CONFIG_SH_AUDIO_DRIVER
/* SP-AMP control     */
#if defined(CONFIG_SHSPAMP_TPA2028D1) || defined(CONFIG_SHSPAMP_AK7811)
    {
        I2C_SURF | I2C_FFA | I2C_FLUID | I2C_RUMI,
        MSM_8960_GSBI2_QUP_I2C_BUS_ID,
        snd_shspamp_i2c_info,
        ARRAY_SIZE(snd_shspamp_i2c_info),
    },
#endif /* defined(CONFIG_SHSPAMP_TPA2028D1) || defined(CONFIG_SHSPAMP_AK7811) */
#endif /* CONFIG_SH_AUDIO_DRIVER */
#ifdef CONFIG_SHMDS
	{
		I2C_SURF | I2C_FFA | I2C_FLUID,
		MSM_8960_GSBI12_QUP_I2C_BUS_ID,
		shmds_i2c_info,
		ARRAY_SIZE(shmds_i2c_info),
	},
#endif /* CONFIG_SHMDS */
#ifdef CONFIG_SH_AUDIO_DRIVER /*02-141*/
/* Bone Conduction Receiver control */
#ifdef CONFIG_SHRECEIVER_LM48560
    {
        I2C_SURF | I2C_FFA | I2C_FLUID | I2C_RUMI,
        MSM_8960_GSBI2_QUP_I2C_BUS_ID,
        snd_shreceiver_i2c_info,
        ARRAY_SIZE(snd_shreceiver_i2c_info),
    },
#endif /* CONFIG_SHRECEIVER_LM48560 */
#endif /* CONFIG_SH_AUDIO_DRIVER *//*02-141*/
};
#endif /* CONFIG_I2C */

static void __init register_i2c_devices(void)
{
#ifdef CONFIG_I2C
	u8 mach_mask = 0;
	int i;
#ifdef CONFIG_MSM_CAMERA
	struct i2c_registry msm8960_camera_i2c_devices = {
		I2C_SURF | I2C_FFA | I2C_FLUID | I2C_LIQUID | I2C_RUMI,
		MSM_8960_GSBI4_QUP_I2C_BUS_ID,
		msm8960_camera_board_info.board_info,
		msm8960_camera_board_info.num_i2c_board_info,
	};
#endif

	/* Build the matching 'supported_machs' bitmask */
	if (machine_is_msm8960_cdp())
		mach_mask = I2C_SURF;
	else if (machine_is_msm8960_rumi3())
		mach_mask = I2C_RUMI;
	else if (machine_is_msm8960_sim())
		mach_mask = I2C_SIM;
	else if (machine_is_msm8960_fluid())
		mach_mask = I2C_FLUID;
	else if (machine_is_msm8960_liquid())
		mach_mask = I2C_LIQUID;
	else if (machine_is_msm8960_mtp())
		mach_mask = I2C_FFA;
       else if (machine_is_lynx_dl12())
              mach_mask = I2C_SURF;
       else if (machine_is_lynx_dl10())
              mach_mask = I2C_SURF;
       else if (machine_is_lynx_dl18())
              mach_mask = I2C_SURF;
       else if (machine_is_lynx_dl15())
              mach_mask = I2C_SURF;
	else if (machine_is_deckard_as38())
		mach_mask = I2C_SURF;
	else
		pr_err("unmatched machine ID in register_i2c_devices\n");

	if (machine_is_msm8960_liquid()) {
		if (SOCINFO_VERSION_MAJOR(socinfo_get_platform_version()) == 3)
			mxt_device_info[0].platform_data =
						&mxt_platform_data_3d;
		else
			mxt_device_info[0].platform_data =
						&mxt_platform_data_2d;
	}

	/* Run the array and install devices as appropriate */
	for (i = 0; i < ARRAY_SIZE(msm8960_i2c_devices); ++i) {
		if (msm8960_i2c_devices[i].machs & mach_mask)
			i2c_register_board_info(msm8960_i2c_devices[i].bus,
						msm8960_i2c_devices[i].info,
						msm8960_i2c_devices[i].len);
	}

	if (!mhl_platform_data.gpio_mhl_power)
		pr_debug("mhl device configured for ext debug board\n");

#ifdef CONFIG_MSM_CAMERA
	if (msm8960_camera_i2c_devices.machs & mach_mask)
		i2c_register_board_info(msm8960_camera_i2c_devices.bus,
			msm8960_camera_i2c_devices.info,
			msm8960_camera_i2c_devices.len);
#endif
#endif
}

static void __init msm8960_sim_init(void)
{
	struct msm_watchdog_pdata *wdog_pdata = (struct msm_watchdog_pdata *)
		&msm8960_device_watchdog.dev.platform_data;

	wdog_pdata->bark_time = 15000;
	msm_tsens_early_init(&msm_tsens_pdata);
	msm_thermal_init(&msm_thermal_pdata);
	BUG_ON(msm_rpm_init(&msm8960_rpm_data));
	BUG_ON(msm_rpmrs_levels_init(&msm_rpmrs_data));
	regulator_suppress_info_printing();
#ifdef CONFIG_BATTERY_SH
    msm8960_pm8921_regulator_init();
#endif /* CONFIG_BATTERY_SH */
	platform_device_register(&msm8960_device_rpm_regulator);
	msm_clock_init(&msm8960_clock_init_data);
	msm8960_init_pmic();

	msm8960_device_otg.dev.platform_data = &msm_otg_pdata;
	msm8960_init_gpiomux();
	msm8960_i2c_init();
	msm_spm_init(msm_spm_data, ARRAY_SIZE(msm_spm_data));
	msm_spm_l2_init(msm_spm_l2_data);
	msm8960_init_buses();
	platform_add_devices(common_devices, ARRAY_SIZE(common_devices));
	msm8960_pm8921_gpio_mpp_init();
	platform_add_devices(sim_devices, ARRAY_SIZE(sim_devices));

#ifdef CONFIG_SHLCDC_BOARD
    msm8960_device_qup_spi_gsbi5.dev.platform_data = &msm8960_qup_spi_gsbi5_pdata;
#endif

#ifdef	CONFIG_SHSYS_CUST
    msm8960_device_qup_spi_gsbi9.dev.platform_data = &msm8960_qup_spi_gsbi9_pdata;
#else
	msm8960_device_qup_spi_gsbi1.dev.platform_data =
				&msm8960_qup_spi_gsbi1_pdata;
#endif
	spi_register_board_info(spi_board_info, ARRAY_SIZE(spi_board_info));

	msm8960_init_mmc();
	msm8960_init_fb();
	slim_register_board_info(msm_slim_devices,
		ARRAY_SIZE(msm_slim_devices));
	BUG_ON(msm_pm_boot_init(&msm_pm_boot_pdata));
	msm_pm_init_sleep_status_data(&msm_pm_slp_sts_data);
}

static void __init msm8960_rumi3_init(void)
{
	msm_tsens_early_init(&msm_tsens_pdata);
	msm_thermal_init(&msm_thermal_pdata);
	BUG_ON(msm_rpm_init(&msm8960_rpm_data));
	BUG_ON(msm_rpmrs_levels_init(&msm_rpmrs_data));
	regulator_suppress_info_printing();
#ifdef CONFIG_BATTERY_SH
    msm8960_pm8921_regulator_init();
#endif /* CONFIG_BATTERY_SH */
	platform_device_register(&msm8960_device_rpm_regulator);
	msm8960_init_gpiomux();
	msm8960_init_pmic();
#ifdef CONFIG_SHLCDC_BOARD
    msm8960_device_qup_spi_gsbi5.dev.platform_data = &msm8960_qup_spi_gsbi5_pdata;
#endif

#ifdef	CONFIG_SHSYS_CUST
    msm8960_device_qup_spi_gsbi9.dev.platform_data = &msm8960_qup_spi_gsbi9_pdata;
#else
	msm8960_device_qup_spi_gsbi1.dev.platform_data =
				&msm8960_qup_spi_gsbi1_pdata;
#endif
	spi_register_board_info(spi_board_info, ARRAY_SIZE(spi_board_info));
	msm8960_i2c_init();
	msm_spm_init(msm_spm_data, ARRAY_SIZE(msm_spm_data));
	msm_spm_l2_init(msm_spm_l2_data);
	platform_add_devices(common_devices, ARRAY_SIZE(common_devices));
	msm8960_pm8921_gpio_mpp_init();
	platform_add_devices(rumi3_devices, ARRAY_SIZE(rumi3_devices));
	msm8960_init_mmc();
	register_i2c_devices();


	msm8960_init_fb();
	slim_register_board_info(msm_slim_devices,
		ARRAY_SIZE(msm_slim_devices));
	BUG_ON(msm_pm_boot_init(&msm_pm_boot_pdata));
	msm_pm_init_sleep_status_data(&msm_pm_slp_sts_data);
}

static void __init msm8960_cdp_init(void)
{
	if (meminfo_init(SYS_MEMORY, SZ_256M) < 0)
		pr_err("meminfo_init() failed!\n");

	msm_tsens_early_init(&msm_tsens_pdata);
	msm_thermal_init(&msm_thermal_pdata);
	BUG_ON(msm_rpm_init(&msm8960_rpm_data));
	BUG_ON(msm_rpmrs_levels_init(&msm_rpmrs_data));

	regulator_suppress_info_printing();
	if (msm_xo_init())
		pr_err("Failed to initialize XO votes\n");
#ifdef CONFIG_BATTERY_SH
    msm8960_pm8921_regulator_init();
#endif /* CONFIG_BATTERY_SH */
	platform_device_register(&msm8960_device_rpm_regulator);
	msm_clock_init(&msm8960_clock_init_data);
	if (machine_is_msm8960_liquid())
		msm_otg_pdata.mhl_enable = true;
	msm8960_device_otg.dev.platform_data = &msm_otg_pdata;
	if (machine_is_msm8960_mtp() || machine_is_msm8960_fluid() ||
           machine_is_msm8960_cdp() ||
           machine_is_lynx_dl12() || machine_is_deckard_as38() || machine_is_lynx_dl10() || machine_is_lynx_dl15() || machine_is_lynx_dl18()
        ) {
		msm_otg_pdata.phy_init_seq = wr_phy_init_seq;
	} else if (machine_is_msm8960_liquid()) {
			msm_otg_pdata.phy_init_seq =
				liquid_v1_phy_init_seq;
	}
	android_usb_pdata.swfi_latency =
		msm_rpmrs_levels[0].latency_us;
	msm_device_hsic_host.dev.platform_data = &msm_hsic_pdata;
	if (SOCINFO_VERSION_MAJOR(socinfo_get_version()) >= 2 &&
					machine_is_msm8960_liquid())
		msm_device_hsic_host.dev.parent = &smsc_hub_device.dev;
	msm8960_init_gpiomux();
#ifdef CONFIG_SHLCDC_BOARD
    msm8960_device_qup_spi_gsbi5.dev.platform_data = &msm8960_qup_spi_gsbi5_pdata;
#endif
#ifdef CONFIG_SHSYS_CUST
	msm8960_device_qup_spi_gsbi9.dev.platform_data = &msm8960_qup_spi_gsbi9_pdata;
#else
	msm8960_device_qup_spi_gsbi1.dev.platform_data =
				&msm8960_qup_spi_gsbi1_pdata;
#endif
	spi_register_board_info(spi_board_info, ARRAY_SIZE(spi_board_info));

	msm8960_init_pmic();
	if ((SOCINFO_VERSION_MAJOR(socinfo_get_version()) >= 2 &&
		(machine_is_msm8960_mtp())) || machine_is_msm8960_liquid())
		msm_isa1200_board_info[0].platform_data = &isa1200_1_pdata;
	msm8960_i2c_init();
	msm8960_gfx_init();
	msm_spm_init(msm_spm_data, ARRAY_SIZE(msm_spm_data));
	msm_spm_l2_init(msm_spm_l2_data);
	msm8960_init_buses();
	platform_add_devices(msm8960_footswitch, msm8960_num_footswitch);
	if (machine_is_msm8960_liquid())
		platform_device_register(&msm8960_device_ext_3p3v_vreg);
	if (machine_is_msm8960_cdp() ||
           machine_is_lynx_dl12() || machine_is_deckard_as38() || machine_is_lynx_dl10() || machine_is_lynx_dl15() || machine_is_lynx_dl18()
        )
		platform_device_register(&msm8960_device_ext_l2_vreg);

	if (socinfo_get_platform_subtype() == PLATFORM_SUBTYPE_SGLTE)
		platform_device_register(&msm8960_device_uart_gsbi8);
	else
#ifdef	CONFIG_SHSYS_CUST
		;
#else
		platform_device_register(&msm8960_device_uart_gsbi5);
#endif

	/* For 8960 Fusion 2.2 Primary IPC */
	if (socinfo_get_platform_subtype() == PLATFORM_SUBTYPE_SGLTE) {
		msm_uart_dm9_pdata.wakeup_irq = gpio_to_irq(94); /* GSBI9(2) */
		msm_device_uart_dm9.dev.platform_data = &msm_uart_dm9_pdata;
	}

	platform_add_devices(common_devices, ARRAY_SIZE(common_devices));
	msm8960_pm8921_gpio_mpp_init();
	platform_add_devices(cdp_devices, ARRAY_SIZE(cdp_devices));
	msm8960_init_smsc_hub();
	msm8960_init_hsic();
#ifdef CONFIG_MSM_CAMERA
	msm8960_init_cam();
#endif
	msm8960_init_mmc();
	if (machine_is_msm8960_liquid())
		mxt_init_hw_liquid();
	register_i2c_devices();
	msm8960_init_fb();
	slim_register_board_info(msm_slim_devices,
		ARRAY_SIZE(msm_slim_devices));
	msm8960_init_dsps();
	change_memory_power = &msm8960_change_memory_power;
	BUG_ON(msm_pm_boot_init(&msm_pm_boot_pdata));
	msm_pm_init_sleep_status_data(&msm_pm_slp_sts_data);
	if (socinfo_get_platform_subtype() == PLATFORM_SUBTYPE_SGLTE) {
		mdm_sglte_device.dev.platform_data = &sglte_platform_data;
		platform_device_register(&mdm_sglte_device);
	}
}

MACHINE_START(MSM8960_SIM, "QCT MSM8960 SIMULATOR")
	.map_io = msm8960_map_io,
	.reserve = msm8960_reserve,
	.init_irq = msm8960_init_irq,
	.handle_irq = gic_handle_irq,
	.timer = &msm_timer,
	.init_machine = msm8960_sim_init,
	.init_early = msm8960_allocate_memory_regions,
	.init_very_early = msm8960_early_memory,
	.restart = msm_restart,
MACHINE_END

MACHINE_START(MSM8960_RUMI3, "QCT MSM8960 RUMI3")
	.map_io = msm8960_map_io,
	.reserve = msm8960_reserve,
	.init_irq = msm8960_init_irq,
	.handle_irq = gic_handle_irq,
	.timer = &msm_timer,
	.init_machine = msm8960_rumi3_init,
	.init_early = msm8960_allocate_memory_regions,
	.init_very_early = msm8960_early_memory,
	.restart = msm_restart,
MACHINE_END

MACHINE_START(MSM8960_CDP, "QCT MSM8960 CDP")
	.map_io = msm8960_map_io,
	.reserve = msm8960_reserve,
	.init_irq = msm8960_init_irq,
	.handle_irq = gic_handle_irq,
	.timer = &msm_timer,
	.init_machine = msm8960_cdp_init,
	.init_early = msm8960_allocate_memory_regions,
	.init_very_early = msm8960_early_memory,
	.restart = msm_restart,
MACHINE_END

MACHINE_START(MSM8960_MTP, "QCT MSM8960 MTP")
	.map_io = msm8960_map_io,
	.reserve = msm8960_reserve,
	.init_irq = msm8960_init_irq,
	.handle_irq = gic_handle_irq,
	.timer = &msm_timer,
	.init_machine = msm8960_cdp_init,
	.init_early = msm8960_allocate_memory_regions,
	.init_very_early = msm8960_early_memory,
	.restart = msm_restart,
MACHINE_END

MACHINE_START(MSM8960_FLUID, "QCT MSM8960 FLUID")
	.map_io = msm8960_map_io,
	.reserve = msm8960_reserve,
	.init_irq = msm8960_init_irq,
	.handle_irq = gic_handle_irq,
	.timer = &msm_timer,
	.init_machine = msm8960_cdp_init,
	.init_early = msm8960_allocate_memory_regions,
	.init_very_early = msm8960_early_memory,
	.restart = msm_restart,
MACHINE_END

MACHINE_START(MSM8960_LIQUID, "QCT MSM8960 LIQUID")
	.map_io = msm8960_map_io,
	.reserve = msm8960_reserve,
	.init_irq = msm8960_init_irq,
	.handle_irq = gic_handle_irq,
	.timer = &msm_timer,
	.init_machine = msm8960_cdp_init,
	.init_early = msm8960_allocate_memory_regions,
	.init_very_early = msm8960_early_memory,
	.restart = msm_restart,
MACHINE_END

MACHINE_START(LYNX_DL12, "SHARP LYNX DL12")
    .map_io = msm8960_map_io,
    .reserve = msm8960_reserve,
    .init_irq = msm8960_init_irq,
    .handle_irq = gic_handle_irq,
    .timer = &msm_timer,
    .init_machine = msm8960_cdp_init,
    .init_early = msm8960_allocate_memory_regions,
    .init_very_early = msm8960_early_memory,
    .restart = msm_restart,
MACHINE_END

MACHINE_START(LYNX_DL10, "SHARP LYNX DL10")
    .map_io = msm8960_map_io,
    .reserve = msm8960_reserve,
    .init_irq = msm8960_init_irq,
    .handle_irq = gic_handle_irq,
    .timer = &msm_timer,
    .init_machine = msm8960_cdp_init,
    .init_early = msm8960_allocate_memory_regions,
    .init_very_early = msm8960_early_memory,
    .restart = msm_restart,
MACHINE_END

MACHINE_START(LYNX_DL15, "SHARP LYNX DL15")
    .map_io = msm8960_map_io,
    .reserve = msm8960_reserve,
    .init_irq = msm8960_init_irq,
    .handle_irq = gic_handle_irq,
    .timer = &msm_timer,
    .init_machine = msm8960_cdp_init,
    .init_early = msm8960_allocate_memory_regions,
    .init_very_early = msm8960_early_memory,
    .restart = msm_restart,
MACHINE_END

MACHINE_START(LYNX_DL18, "SHARP LYNX DL18")
    .map_io = msm8960_map_io,
    .reserve = msm8960_reserve,
    .init_irq = msm8960_init_irq,
    .handle_irq = gic_handle_irq,
    .timer = &msm_timer,
    .init_machine = msm8960_cdp_init,
    .init_early = msm8960_allocate_memory_regions,
    .init_very_early = msm8960_early_memory,
    .restart = msm_restart,
MACHINE_END

MACHINE_START(DECKARD_AS38, "SHARP DECKARD AS38")
	.map_io = msm8960_map_io,
	.reserve = msm8960_reserve,
	.init_irq = msm8960_init_irq,
	.handle_irq = gic_handle_irq,
	.timer = &msm_timer,
	.init_machine = msm8960_cdp_init,
	.init_early = msm8960_allocate_memory_regions,
	.init_very_early = msm8960_early_memory,
	.restart = msm_restart,
MACHINE_END
