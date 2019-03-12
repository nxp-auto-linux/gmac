/*
 * dwmac-s32cc.c - S32 GMAC glue layer
 *
 * Author: Jan Petrous <jan.petrous@nxp.com>
 *
 * Copyright 2019 NXP
 *
 * SPDX-License-Identifier:     GPL-2.0
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>. 
 */

#include <linux/device.h>
#include <linux/ethtool.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/stmmac.h>

#include "stmmac_platform.h"

struct s32_dwmac {
	struct device	*dev;
	void __iomem	*reg;
};

void s32_dwmac_fix_mac_speed(void *priv, unsigned int speed)
{
	/* no need for now (as on vdk) */
}

static int s32cc_dwmac_probe(struct platform_device *pdev)
{
	struct plat_stmmacenet_data *plat_dat;
	struct stmmac_resources stmmac_res;
	struct s32_dwmac *dwmac = NULL;
	struct resource *res;
	int ret;

	ret = stmmac_get_platform_resources(pdev, &stmmac_res);
	if (ret)
		return ret;

	plat_dat = stmmac_probe_config_dt(pdev, &stmmac_res.mac);
	if (IS_ERR(plat_dat))
		return PTR_ERR(plat_dat);

	if (dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32)) != 0) {
		dev_err(&pdev->dev, "System does not support DMA, aborting\n");
		return -EINVAL;
	}

	/* core feature set */
	plat_dat->has_gmac4 = true;
	plat_dat->pmt = 1;
	plat_dat->tso_en = of_property_read_bool(pdev->dev.of_node, "snps,tso");

#if 0
	dwmac = devm_kzalloc(&pdev->dev, sizeof(*dwmac), GFP_KERNEL);
	if (!dwmac) {
		ret = -ENOMEM;
		goto err_remove_config_dt;
	}

	dwmac->reg = stmmac_res->addr;
	if (!(dwmac->reg)) {
		ret = EINVAL;
		goto err_remove_config_dt;
	}

	dwmac->dev = &pdev->dev;
#endif

	//TODO: plat_dat->fix_mac_speed = s32_dwmac_fix_mac_speed;
	plat_dat->bsp_priv = dwmac;

	ret = stmmac_dvr_probe(&pdev->dev, plat_dat, &stmmac_res);
	if (ret)
		goto err_remove_config_dt;

	return 0;

err_remove_config_dt:
	stmmac_remove_config_dt(pdev, plat_dat);

	return ret;
}

static const struct of_device_id s32_dwmac_match[] = {
	{ .compatible = "fsl,s32cc-dwmac" },
	{ }
};
MODULE_DEVICE_TABLE(of, s32_dwmac_match);

static struct platform_driver s32_dwmac_driver = {
	.probe  = s32cc_dwmac_probe,
	.remove = stmmac_pltfr_remove,
	.driver = {
		.name           = "s32cc-dwmac",
		.pm		= &stmmac_pltfr_pm_ops,
		.of_match_table = s32_dwmac_match,
	},
};
module_platform_driver(s32_dwmac_driver);

MODULE_AUTHOR("Jan Petrous <jan.petrous@nxp.com>");
MODULE_DESCRIPTION("NXP S32 common chassis DWMAC glue layer");
MODULE_LICENSE("GPL v2");

