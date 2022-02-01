/*
 * Driver for MMC and SSD cards for Cavium ThunderX SOCs.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2016 Cavium Inc.
 */
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/mmc/mmc.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/property.h>
#include <linux/acpi.h>
#include <linux/pci.h>
#include <linux/delay.h>
#include <linux/bitfield.h>
#include "cavium.h"

static void thunder_mmc_acquire_bus(struct cvm_mmc_host *host)
{
#if IS_ENABLED(CONFIG_MMC_PSTORE)
	if (!host->pstore)
		down(&host->mmc_serializer);
#else
	down(&host->mmc_serializer);
#endif
}

static void thunder_mmc_release_bus(struct cvm_mmc_host *host)
{
#if IS_ENABLED(CONFIG_MMC_PSTORE)
	if (!host->pstore)
		up(&host->mmc_serializer);
#else
	up(&host->mmc_serializer);
#endif
}

static void thunder_mmc_int_enable(struct cvm_mmc_host *host, u64 val)
{
	writeq(val, host->base + MIO_EMM_INT(host));
	writeq(val, host->base + MIO_EMM_INT_EN_SET(host));
	writeq(MIO_EMM_DMA_INT_DMA,
		host->dma_base + MIO_EMM_DMA_INT(host));
}

static int thunder_mmc_register_interrupts(struct cvm_mmc_host *host,
					   struct pci_dev *pdev)
{
	int nvec, ret, i;

	nvec = pci_alloc_irq_vectors(pdev, 1, 9, PCI_IRQ_MSIX);
	if (nvec < 0)
		return nvec;

	/* register interrupts */
	for (i = 0; i < nvec; i++) {
		ret = devm_request_irq(&pdev->dev, pci_irq_vector(pdev, i),
				       cvm_mmc_interrupt, IRQF_NO_THREAD,
				       cvm_mmc_irq_names[i], host);
		if (ret)
			return ret;
	}
	return 0;
}

/* calibration evaluates the per tap delay */
static void thunder_calibrate_mmc(struct cvm_mmc_host *host)
{
	u32 retries = 10;
	u32 delay = 4;
	unsigned int ps;
	const char *how = "default";

	if (is_mmc_8xxx(host))
		return;

	/* set _DEBUG[CLK_ON]=1 as workaround for clock issue */
	if (host->cond_clock_glitch)
		writeq(1, host->base + MIO_EMM_DEBUG(host));

	if (host->calibrate_glitch) {
		/*
		 * Operation of up to 100 MHz may be achieved by skipping the
		 * steps that establish the tap delays and instead assuming
		 * that MIO_EMM_TAP[DELAY] returns 0x4 indicating 78 pS/tap.
		 */
	} else {
		u64 tap;
		u64 emm_cfg = readq(host->base + MIO_EMM_CFG(host));
		u64 tcfg;
		u64 emm_io_ctl;
		u64 emm_switch;
		u64 emm_wdog;
		u64 emm_sts_mask;
		u64 emm_debug;
		u64 emm_timing;
		u64 emm_rca;

		/*
		 * MIO_EMM_CFG[BUS_ENA] must be zero for calibration,
		 * but that resets whole host, so save state.
		 */
		emm_io_ctl = readq(host->base + MIO_EMM_IO_CTL(host));
		emm_switch = readq(host->base + MIO_EMM_SWITCH(host));
		emm_wdog = readq(host->base + MIO_EMM_WDOG(host));
		emm_sts_mask =
			readq(host->base + MIO_EMM_STS_MASK(host));
		emm_debug = readq(host->base + MIO_EMM_DEBUG(host));
		emm_timing = readq(host->base + MIO_EMM_TIMING(host));
		emm_rca = readq(host->base + MIO_EMM_RCA(host));

		/* reset controller */
		tcfg = emm_cfg;
		tcfg &= ~MIO_EMM_CFG_BUS_ENA;
		writeq(tcfg, host->base + MIO_EMM_CFG(host));
		udelay(1);

		/* restart with phantom slot 3 */
		tcfg |= FIELD_PREP(MIO_EMM_CFG_BUS_ENA, 1ull << 3);
		writeq(tcfg, host->base + MIO_EMM_CFG(host));
		mdelay(1);

		/* Start calibration */
		writeq(0, host->base + MIO_EMM_CALB(host));
		udelay(5);
		writeq(START_CALIBRATION, host->base + MIO_EMM_CALB(host));
		udelay(5);

		do {
			/* wait for approximately 300 coprocessor clock */
			udelay(5);
			tap = readq(host->base + MIO_EMM_TAP(host));
		} while (!tap && retries--);

		/* leave calibration mode */
		writeq(0, host->base + MIO_EMM_CALB(host));
		udelay(5);

		if (retries <= 0 || !tap) {
			how = "fallback";
		} else {
			/* calculate the per-tap delay */
			delay = tap & MIO_EMM_TAP_DELAY;
			how = "calibrated";
		}

		/* Reset eMMC subsystem */
		writeq(0, host->base + MIO_EMM_CFG(host));
		udelay(1);
		/* restore old state */
		writeq(emm_cfg, host->base + MIO_EMM_CFG(host));
		mdelay(1);
		writeq(emm_rca, host->base + MIO_EMM_RCA(host));
		writeq(emm_timing, host->base + MIO_EMM_TIMING(host));
		writeq(emm_debug, host->base + MIO_EMM_DEBUG(host));
		writeq(emm_sts_mask,
			host->base + MIO_EMM_STS_MASK(host));
		writeq(emm_wdog, host->base + MIO_EMM_WDOG(host));
		writeq(emm_switch, host->base + MIO_EMM_SWITCH(host));
		writeq(emm_io_ctl, host->base + MIO_EMM_IO_CTL(host));
		mdelay(1);
	}

	/*
	 * Scale measured/guessed calibration value to pS:
	 * The delay value should be multiplied by 10 ns(or 10000 ps)
	 * and then divided by no of taps to determine the estimated
	 * delay in pico second. The nominal value is 125 ps per tap.
	 */
	ps = (delay * PS_10000) / TOTAL_NO_OF_TAPS;
	if (host->per_tap_delay != ps) {
		dev_info(host->dev, "%s delay:%d per-tap delay:%dpS\n",
			how, delay, ps);
		host->per_tap_delay = ps;
		host->delay_logged = 0;
	}
}

static int thunder_mmc_slot_probe(struct platform_device *plat_dev)
{
	struct device *dev;
	struct device_node *np;
	struct acpi_device *adev;
	struct cvm_mmc_host *host;
	const struct cvm_mmc_host_info *info;
	int ret;


	dev = &plat_dev->dev;
	info = device_get_match_data(dev);

	if (IS_ERR_OR_NULL(info->host)) {
		if (!info->host)
			return -EPROBE_DEFER;
		else
			return PTR_ERR(info->host);
	}

	host = info->host;
	np = plat_dev->dev.of_node;
	adev = ACPI_COMPANION(&plat_dev->dev);

	if (!adev && !np)
		return -EINVAL;

	ret = cvm_mmc_of_slot_probe(&plat_dev->dev, host);
	if (ret) {
		if (ret != -EPROBE_DEFER)
			dev_info(dev, "Probe has failed with error %d\n", ret);
		return ret;
	}

	return 0;
}

static int thunder_mmc_slot_remove(struct platform_device *plat_dev)
{
	struct cvm_mmc_slot *slot;

	slot = platform_get_drvdata(plat_dev);
	if (slot)
		cvm_mmc_of_slot_remove(slot);

	return 0;
}

static struct cvm_mmc_host_info mmc_host_info = {
	.host = NULL,  /* Set when host is ready */
};

static const struct acpi_device_id thunder_mmc_slot_ids[] = {
	{ .id = "MRVL0003", .driver_data = (kernel_ulong_t)&mmc_host_info },
	{ },
};

static const struct of_device_id thunder_mmc_slot_of_match[] = {
	{ .compatible = "mmc-slot", .data = &mmc_host_info },
	{ },
};

static struct platform_driver thunder_mmc_slot_driver = {
	.driver = {
		.name = "thunder-mmc-slot",
		.of_match_table = thunder_mmc_slot_of_match,
		.acpi_match_table = thunder_mmc_slot_ids,
		},
	.probe = thunder_mmc_slot_probe,
	.remove = thunder_mmc_slot_remove,
};

MODULE_DEVICE_TABLE(acpi, thunder_mmc_slot_ids);
MODULE_DEVICE_TABLE(of, thunder_mmc_slots_of_match);

static int thunder_mmc_of_slot_create(struct device *dev)
{
	struct device_node *np, *child;

	np = dev->of_node;
	/* For Device tree case, we have to populate devices manually */
	for_each_child_of_node(np, child) {
		if (of_device_is_compatible(child, "mmc-slot"))
			of_platform_device_create(child, NULL, dev);
	}

	return 0;
}

static int thunder_mmc_probe(struct pci_dev *pdev,
			     const struct pci_device_id *id)
{
	struct device *dev = &pdev->dev;
	struct cvm_mmc_host *host;
	int ret;
	u8 chip_id;
	u8 rev;
	bool has_acpi;

	host = devm_kzalloc(dev, sizeof(*host), GFP_KERNEL);
	if (!host) {
		ret = -ENOMEM;
		goto error;
	}

	has_acpi = has_acpi_companion(dev);
	pci_set_drvdata(pdev, host);

	ret = pcim_enable_device(pdev);
	if (ret)
		goto error;

	ret = pci_request_regions(pdev, KBUILD_MODNAME);
	if (ret)
		goto error;

	host->base = pcim_iomap(pdev, 0, pci_resource_len(pdev, 0));
	if (!host->base) {
		ret = -EINVAL;
		goto error;
	}

	/* On ThunderX these are identical */
	host->dma_base = host->base;
	host->pdev = pdev;

	host->reg_off = 0x2000;
	host->reg_off_dma = 0x160;

	if (!has_acpi) {
		host->clk = devm_clk_get(dev, NULL);
		if (IS_ERR(host->clk)) {
			ret = PTR_ERR(host->clk);
			goto no_clk;
		}

		ret = clk_prepare_enable(host->clk);
		if (ret)
			goto no_clk;
		host->sys_freq = clk_get_rate(host->clk);
	} else
		host->sys_freq = 1000 * 1000000ul;

	spin_lock_init(&host->irq_handler_lock);
	sema_init(&host->mmc_serializer, 1);

	host->dev = dev;
	host->acquire_bus = thunder_mmc_acquire_bus;
	host->release_bus = thunder_mmc_release_bus;
	host->int_enable = thunder_mmc_int_enable;

	host->use_sg = true;
	host->big_dma_addr = true;
	host->need_irq_handler_lock = true;
	host->last_slot = -1;

	rev = pdev->revision;
	chip_id = (pdev->subsystem_device >> 8) & 0xff;
	switch (chip_id) {
	case PCI_SUBSYS_DEVID_96XX:
		if (rev == REV_ID_0) {
			host->calibrate_glitch = true;
			host->cond_clock_glitch = true;
			host->max_freq = MHZ_100;
		} else if (rev == REV_ID_1) {
			host->cond_clock_glitch = true;
			host->max_freq = MHZ_167;
		} else if (rev == REV_ID_2) {
			host->tap_requires_noclk = true;
			host->max_freq = MHZ_112_5;
		} else if (rev > REV_ID_2) {
			host->tap_requires_noclk = true;
			host->max_freq = MHZ_200;
		}
		break;
	case PCI_SUBSYS_DEVID_95XXMM:
	case PCI_SUBSYS_DEVID_98XX:
		host->tap_requires_noclk = true;
		host->max_freq = MHZ_200;
		break;
	case PCI_SUBSYS_DEVID_95XX:
		if (rev == REV_ID_0)
			host->cond_clock_glitch = true;
		host->max_freq = MHZ_167;
		break;
	case PCI_SUBSYS_DEVID_LOKI:
		host->max_freq = MHZ_167;
		break;
	default:
		break;
	}
	/*
	 * Clear out any pending interrupts that may be left over from
	 * bootloader. Writing 1 to the bits clears them.
	 * Clear DMA FIFO after IRQ disable, then stub any dangling events
	 */
	writeq(~0, host->base + MIO_EMM_INT(host));
	writeq(~0, host->dma_base + MIO_EMM_DMA_INT_ENA_W1C(host));
	writeq(~0, host->base + MIO_EMM_INT_EN_CLR(host));
	writeq(MIO_EMM_DMA_FIFO_CFG_CLR,
		host->dma_base + MIO_EMM_DMA_FIFO_CFG(host));
	writeq(~0, host->dma_base + MIO_EMM_DMA_INT(host));

	ret = thunder_mmc_register_interrupts(host, pdev);
	if (ret)
		goto no_irq;

	/* Run the calibration to calculate per tap delay that would be
	 * used to evaluate values. These values would be programmed in
	 * MIO_EMM_TIMING.
	 */
	thunder_calibrate_mmc(host);

	mmc_host_info.host = host;
	/* For platforms with device tree, the driver has to create slot nodes */
	if (!has_acpi)
		thunder_mmc_of_slot_create(dev);

	return 0;

	/* Handle all kind of errors */
no_irq:
	if (!has_acpi)
		clk_disable_unprepare(host->clk);
no_clk:
	dev_err(dev, "Can't get rclk frequency\n");
error:
	mmc_host_info.host = ERR_PTR(ret);
	return ret;
}

static void thunder_mmc_remove(struct pci_dev *pdev)
{
	struct cvm_mmc_host *host = pci_get_drvdata(pdev);
	u64 dma_cfg;

	dma_cfg = readq(host->dma_base + MIO_EMM_DMA_CFG(host));
	dma_cfg |= MIO_EMM_DMA_CFG_CLR;
	writeq(dma_cfg, host->dma_base + MIO_EMM_DMA_CFG(host));
	do {
		dma_cfg = readq(host->dma_base + MIO_EMM_DMA_CFG(host));
	} while (dma_cfg & MIO_EMM_DMA_CFG_EN);

	if (!has_acpi_companion(&pdev->dev))
		clk_disable_unprepare(host->clk);
}

static const struct pci_device_id thunder_mmc_id_table[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_CAVIUM, 0xa010) },
	{ 0, }  /* end of table */
};

MODULE_DEVICE_TABLE(pci, thunder_mmc_id_table);

static struct pci_driver thunder_mmc_driver = {
	.name = KBUILD_MODNAME,
	.id_table = thunder_mmc_id_table,
	.probe = thunder_mmc_probe,
	.remove = thunder_mmc_remove,
};

static int __init thunder_mmc_init(void)
{
	int ret;

	/* Must be initialize before the platform driver (poor man's bus) */
	ret = pci_register_driver(&thunder_mmc_driver);
	if (ret)
		return ret;

	ret = platform_driver_register(&thunder_mmc_slot_driver);
	if (!ret)
		return 0;

	pci_unregister_driver(&thunder_mmc_driver);
	return ret;
}
module_init(thunder_mmc_init);

static void __exit thunder_mmc_exit(void)
{
	platform_driver_unregister(&thunder_mmc_slot_driver);
	pci_unregister_driver(&thunder_mmc_driver);
}
module_exit(thunder_mmc_exit);

MODULE_AUTHOR("Cavium Inc.");
MODULE_DESCRIPTION("Cavium ThunderX eMMC Driver");
MODULE_LICENSE("GPL");
MODULE_DEVICE_TABLE(pci, thunder_mmc_id_table);
