/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Configuration for MT8188 based boards
 *
 * Copyright (C) 2022 MediaTek Inc.
 * Author: Macpaul Lin <macpaul.lin@mediatek.com>
 */

#ifndef __MT8188_H
#define __MT8188_H

#include <linux/sizes.h>

#define CONFIG_SYS_NS16550_SERIAL
#define CONFIG_SYS_NS16550_REG_SIZE	-4
#define CONFIG_SYS_NS16550_MEM32
#define CONFIG_SYS_NS16550_COM1		0x11002000
#define CONFIG_SYS_NS16550_CLK		26000000


#define GENIO_700_EVK_FIT_IMAGE_GUID \
	EFI_GUID(0x40ecb7c9, 0x416e, 0x4524, 0xb4, 0x95, \
		 0xa9, 0x67, 0xe0, 0x69, 0x9c, 0xb1)

#define GENIO_700_EVK_FIP_IMAGE_GUID \
	EFI_GUID(0x551cc92d, 0x9a22, 0x4dc8, 0x88, 0x72, \
		 0xdb, 0x1b, 0xb8, 0xdc, 0xaf, 0xa0)

/* Environment settings */
#include <config_distro_bootcmd.h>

#ifdef CONFIG_CMD_MMC
#define BOOT_TARGET_MMC(func) func(MMC, mmc, 0)
#else
#define BOOT_TARGET_MMC(func)
#endif

#ifdef CONFIG_CMD_USB
#define BOOT_TARGET_USB(func) func(USB, usb, 0)
#else
#define BOOT_TARGET_USB(func)
#endif

#ifdef CONFIG_CMD_SCSI
#define BOOT_TARGET_SCSI(func) func(SCSI, scsi, 2)
#else
#define BOOT_TARGET_SCSI(func)
#endif

#define BOOT_TARGET_DEVICES(func) \
	BOOT_TARGET_MMC(func) \
	BOOT_TARGET_USB(func) \
	BOOT_TARGET_SCSI(func)

#if !defined(CONFIG_EXTRA_ENV_SETTINGS)
#define CONFIG_EXTRA_ENV_SETTINGS \
	"scriptaddr=0x40000000\0" \
	"fdt_addr_r=0x44000000\0" \
	"fdtoverlay_addr_r=0x44c00000\0" \
	"kernel_addr_r=0x45000000\0" \
	"ramdisk_addr_r=0x46000000\0" \
	"fdtfile=" CONFIG_DEFAULT_DEVICE_TREE ".dtb\0" \
	BOOTENV
#endif

#endif