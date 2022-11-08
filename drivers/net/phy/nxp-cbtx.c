// SPDX-License-Identifier: GPL-2.0
/* Driver for 100BASE-TX PHY embedded into NXP SJA1110 switch
 *
 * Copyright 2022 NXP
 */

#include <linux/kernel.h>
#include <linux/mii.h>
#include <linux/module.h>
#include <linux/phy.h>

#define PHY_ID_CBTX_SJA1110			0x001bb020

/* Registers */
#define  CBTX_MODE_CTRL_STAT			0x11
#define  CBTX_PDOWN_CTRL			0x18
#define  CBTX_RX_ERR_COUNTER			0x1a

/* Fields */
#define CBTX_MODE_CTRL_STAT_AUTO_MDIX_EN	BIT(7)
#define CBTX_MODE_CTRL_STAT_MDIX_MODE		BIT(6)

#define CBTX_PDOWN_CTL_TRUE_PDOWN		BIT(0)

static int cbtx_soft_reset(struct phy_device *phydev)
{
	int ret;

	/* Can't soft reset unless we remove PHY from true power down mode */
	ret = phy_clear_bits(phydev, CBTX_PDOWN_CTRL,
			     CBTX_PDOWN_CTL_TRUE_PDOWN);
	if (ret)
		return ret;

	return genphy_soft_reset(phydev);
}

static int cbtx_config_init(struct phy_device *phydev)
{
	/* Wait for cbtx_config_aneg() to kick in and apply this */
	phydev->mdix_ctrl = ETH_TP_MDI_AUTO;

	return 0;
}

static int cbtx_suspend(struct phy_device *phydev)
{
	int ret;

	ret = genphy_suspend(phydev);
	if (ret)
		return ret;

	return phy_set_bits(phydev, CBTX_PDOWN_CTRL,
			    CBTX_PDOWN_CTL_TRUE_PDOWN);
}

static int cbtx_resume(struct phy_device *phydev)
{
	int ret;

	ret = phy_clear_bits(phydev, CBTX_PDOWN_CTRL,
			     CBTX_PDOWN_CTL_TRUE_PDOWN);
	if (ret)
		return ret;

	return genphy_resume(phydev);
}

static int cbtx_mdix_status(struct phy_device *phydev)
{
	int ret;

	ret = phy_read(phydev, CBTX_MODE_CTRL_STAT);
	if (ret < 0)
		return ret;

	if (ret & CBTX_MODE_CTRL_STAT_MDIX_MODE)
		phydev->mdix = ETH_TP_MDI_X;
	else
		phydev->mdix = ETH_TP_MDI;

	return 0;
}

static int cbtx_read_status(struct phy_device *phydev)
{
	int ret;

	ret = cbtx_mdix_status(phydev);
	if (ret)
		return ret;

	return genphy_read_status(phydev);
}

static int cbtx_mdix_config(struct phy_device *phydev)
{
	int ret;

	switch (phydev->mdix_ctrl) {
	case ETH_TP_MDI_AUTO:
		return phy_set_bits(phydev, CBTX_MODE_CTRL_STAT,
				    CBTX_MODE_CTRL_STAT_AUTO_MDIX_EN);
	case ETH_TP_MDI:
		ret = phy_clear_bits(phydev, CBTX_MODE_CTRL_STAT,
				     CBTX_MODE_CTRL_STAT_AUTO_MDIX_EN);
		if (ret)
			return ret;

		return phy_clear_bits(phydev, CBTX_MODE_CTRL_STAT,
				      CBTX_MODE_CTRL_STAT_MDIX_MODE);
	case ETH_TP_MDI_X:
		ret = phy_clear_bits(phydev, CBTX_MODE_CTRL_STAT,
				     CBTX_MODE_CTRL_STAT_AUTO_MDIX_EN);
		if (ret)
			return ret;

		return phy_set_bits(phydev, CBTX_MODE_CTRL_STAT,
				    CBTX_MODE_CTRL_STAT_MDIX_MODE);
	}

	return 0;
}

static int cbtx_config_aneg(struct phy_device *phydev)
{
	int ret;

	ret = cbtx_mdix_config(phydev);
	if (ret)
		return ret;

	return genphy_config_aneg(phydev);
}

static int cbtx_get_sset_count(struct phy_device *phydev)
{
	return 1;
}

static void cbtx_get_strings(struct phy_device *phydev, u8 *data)
{
	strncpy(data, "100btx_rx_err", ETH_GSTRING_LEN);
}

static void cbtx_get_stats(struct phy_device *phydev,
			   struct ethtool_stats *stats, u64 *data)
{
	int ret;

	ret = phy_read(phydev, CBTX_RX_ERR_COUNTER);
	data[0] = (ret < 0) ? U64_MAX : ret;
}

static struct phy_driver cbtx_driver[] = {
	{
		PHY_ID_MATCH_MODEL(PHY_ID_CBTX_SJA1110),
		.name			= "NXP CBTX (SJA1110)",
		/* PHY_BASIC_FEATURES */
		.soft_reset		= cbtx_soft_reset,
		.config_init		= cbtx_config_init,
		.suspend		= cbtx_suspend,
		.resume			= cbtx_resume,
		.read_status		= cbtx_read_status,
		.config_aneg		= cbtx_config_aneg,
		.get_sset_count		= cbtx_get_sset_count,
		.get_strings		= cbtx_get_strings,
		.get_stats		= cbtx_get_stats,
	},
};

module_phy_driver(cbtx_driver);

static struct mdio_device_id __maybe_unused cbtx_tbl[] = {
	{ PHY_ID_MATCH_MODEL(PHY_ID_CBTX_SJA1110) },
	{ /* sentinel */ },
};

MODULE_DEVICE_TABLE(mdio, cbtx_tbl);

MODULE_AUTHOR("Vladimir Oltean <vladimir.oltean@nxp.com>");
MODULE_DESCRIPTION("NXP CBTX PHY driver");
MODULE_LICENSE("GPL v2");
