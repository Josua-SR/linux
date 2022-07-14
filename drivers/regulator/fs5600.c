// SPDX-License-Identifier: GPL-2.0+
/*
 * SF5600 DC/DC Controller
 * Copyright 2022 Josua Mayer <josua@solid-run.com>
 */

#include <linux/clk.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/regmap.h>

#define SF5600_REG_SW1CTRL 0x00
#define SF5600_REG_NOT_SW1CTRL 0x01
#define SF5600_REG_ID1 0x27
#define SF5600_REG_ID2 0x28

static const struct regmap_config fs5600_i2c_regmap = {
	.reg_bits = 8,
	.val_bits = 16,
	.val_format_endian = REGMAP_ENDIAN_BIG,

	.max_register = SF5600_REG_ID2,
};

struct sf5600_priv {
	struct regmap *regmap;
};

static int sf5600_probe(struct i2c_client *client, const struct i2c_device_id *i2c_id)
{
	struct device *dev = &client->dev;
	struct sf5600_priv *priv;
	uint16_t reg_id, reg_sw1ctrl, reg_not_sw1ctrl;
	uint16_t newval[2];
	int ret;

	/* allocate memory for private data */
	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if(!priv)
		return -ENOMEM;

	/* register private memory with i2c client instance */
	i2c_set_clientdata(client, priv);

	/* allocate regmap */
	priv->regmap = devm_regmap_init_i2c(client, &fs5600_i2c_regmap);
	if (IS_ERR(priv->regmap)) {
		ret = PTR_ERR(priv->regmap);
		dev_err(dev, "failed to allocate register map: %d\n", ret);
		// TODO: free priv
		return ret;
	}

	/* read ID1 */
	//ret = regmap_read(priv->regmap, SF5600_REG_ID1, (void *)&reg_id);
	ret = regmap_raw_read(priv->regmap, SF5600_REG_ID1, (void *)&reg_id, sizeof(reg_id));
	if (ret) {
		dev_err(dev, "failed to read id register: %i\n", ret);
		return ret;
	}
	dev_info(dev, "read id register: 0x%x\n", reg_id);

	/* read SW1CTRL */
	ret = regmap_raw_read(priv->regmap, SF5600_REG_SW1CTRL, (void *)&reg_sw1ctrl, sizeof(reg_sw1ctrl));
	if (ret) {
		dev_err(dev, "failed to read sw1ctrl register: %i\n", ret);
		return ret;
	}
	dev_info(dev, "read sw1ctrl register: 0x%x\n", reg_sw1ctrl);

	/* read NOT_SW1CTRL */
	ret = regmap_raw_read(priv->regmap, SF5600_REG_NOT_SW1CTRL, (void *)&reg_not_sw1ctrl, sizeof(reg_not_sw1ctrl));
	if (ret) {
		dev_err(dev, "failed to read not_sw1ctrl register: %i\n", ret);
		return ret;
	}
	dev_info(dev, "read not_sw1ctrl register: 0x%x\n", reg_not_sw1ctrl);

	/* enable SW1_EN */
	newval[0] = 0x4;
	newval[1] = ~0x4;
	ret = regmap_bulk_write(priv->regmap, SF5600_REG_SW1CTRL, newval, 2);
	if (ret) {
		dev_err(dev, "failed to write sw1ctrl registers: %i\n", ret);
		return ret;
	}

	// get audio pll0
	struct clk *clk_pll0 = of_clk_get_by_name(dev->of_node, "pll0");
	if (IS_ERR(clk_pll0)) {
		ret = PTR_ERR(clk_pll0);
		pr_err("Failed to get clock pll0 (%d)\n", ret);
		return ret;
	}

	// try setting rates
	ret = clk_set_rate(clk_pll0, 256000);
	if(ret) {
		pr_err("Failed to set pll0 to 256kHz (%d)\n", ret);
	}
	ret = clk_set_rate(clk_pll0, 128000);
	if(ret) {
		pr_err("Failed to set pll0 to 128kHz (%d)\n", ret);
	}
	ret = clk_set_rate(clk_pll0, 64000);
	if(ret) {
		pr_err("Failed to set pll0 to 64kHz (%d)\n", ret);
	}
	ret = clk_set_rate(clk_pll0, 32000);
	if(ret) {
		pr_err("Failed to set pll0 to 32kHz (%d)\n", ret);
	}

	// enable
	ret = clk_prepare_enable(clk_pll0);
	if(ret) {
		pr_err("Failed to enable pll0 (%d)\n", ret);
	}

	// get mclkout
	struct clk *clk_mclk = of_clk_get_by_name(dev->of_node, "mclk");
	if (IS_ERR(clk_mclk)) {
		ret = PTR_ERR(clk_mclk);
		pr_err("Failed to get clock mclk (%d)\n", ret);
		return ret;
	}
	ret = clk_set_rate(clk_mclk, 32000);
	if(ret) {
		pr_err("Failed to set mclk to 32kHz (%d)\n", ret);
	}
		ret = clk_prepare_enable(clk_mclk);
	if(ret) {
		pr_err("Failed to enable mclk (%d)\n", ret);
	}

	return 0;
}

static const struct i2c_device_id sf5600_i2c_ids[] = {
	{"sf5600", 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, sf5600_i2c_ids);

static const struct of_device_id sf5600_dt_ids[] = {
	{ .compatible = "nxp,fs5600", },
	{ }
};
MODULE_DEVICE_TABLE(of, sf5600_dt_ids);

static struct i2c_driver sf5600_driver = {
	.driver = {
		.name = "sf5600-regulator",
		.of_match_table = of_match_ptr(sf5600_dt_ids),
	},
	.probe = sf5600_probe,
	.id_table = sf5600_i2c_ids,
};

module_i2c_driver(sf5600_driver);

MODULE_AUTHOR("Josua Mayer <josua@solid-run.com>");
MODULE_DESCRIPTION("SF5600 regulator driver");
MODULE_LICENSE("GPL");
