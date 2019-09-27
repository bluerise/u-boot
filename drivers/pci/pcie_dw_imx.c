// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2018 Texas Instruments, Inc
 */

#include <common.h>
#include <dm.h>
#include <log.h>
#include <pci.h>
#include <generic-phy.h>
#include <power-domain.h>
#include <regmap.h>
#include <reset.h>
#include <syscon.h>
#include <asm/io.h>
#include <asm/arch/clock.h>
#include <asm-generic/gpio.h>
#include <linux/err.h>

DECLARE_GLOBAL_DATA_PTR;

/* IOMUXC offsets */
#define IOMUXC_GPR12			0x30
#define  IMX8MQ_GPR_PCIE2_DEVICE_TYPE_MASK	(0xf << 8)
#define  IMX8MQ_GPR_PCIE2_DEVICE_TYPE_RC	(0x4 << 8)
#define  IMX8MQ_GPR_PCIE1_DEVICE_TYPE_MASK	(0xf << 12)
#define  IMX8MQ_GPR_PCIE1_DEVICE_TYPE_RC	(0x4 << 12)
#define IOMUXC_GPR14			0x38
#define IOMUXC_GPR16			0x40
#define  IMX8MQ_GPR_PCIE_REF_USE_PAD		(1 << 9)
#define  IMX8MQ_GPR_PCIE_CLK_REQ_OVERRIDE_EN	(1 << 10)
#define  IMX8MQ_GPR_PCIE_CLK_REQ_OVERRIDE	(1 << 11)

/* Anatop */
#define ANATOP_PLLOUT_CTL		0x74
#define  ANATOP_PLLOUT_CTL_CKE			(1 << 4)
#define  ANATOP_PLLOUT_CTL_SEL_SYSPLL1		0xb
#define  ANATOP_PLLOUT_CTL_SEL_MASK		0xf
#define ANATOP_PLLOUT_DIV		0x7c
#define  ANATOP_PLLOUT_DIV_SYSPLL1		0x7

/* PCI DBICS registers */
#define PCIE_LINK_CAPABILITY		0x7c
#define PCIE_LINK_CTL_2			0xa0
#define TARGET_LINK_SPEED_MASK		0xf
#define LINK_SPEED_GEN_1		0x1
#define LINK_SPEED_GEN_2		0x2
#define LINK_SPEED_GEN_3		0x3

#define PCIE_MISC_CONTROL_1_OFF		0x8bc
#define PCIE_DBI_RO_WR_EN		BIT(0)

#define PLR_OFFSET			0x700
#define PCIE_PORT_DEBUG0		(PLR_OFFSET + 0x28)
#define PCIE_PORT_DEBUG1		(PLR_OFFSET + 0x2c)
#define PCIE_PHY_DEBUG_R1_LINK_UP		(1 << 4)
#define PCIE_PHY_DEBUG_R1_LINK_IN_TRAINING	(1 << 29)

#define PCIE_LINK_WIDTH_SPEED_CONTROL	0x80c
#define PORT_LOGIC_SPEED_CHANGE		(0x1 << 17)

#define PCIE_LINK_UP_TIMEOUT_MS		100

/*
 * iATU Unroll-specific register definitions
 * From 4.80 core version the address translation will be made by unroll.
 */
#define PCIE_ATU_OFFSET			0x300000

#define PCIE_ATU_UNR_REGION_CTRL1	0x00
#define PCIE_ATU_UNR_REGION_CTRL2	0x04
#define PCIE_ATU_UNR_LOWER_BASE		0x08
#define PCIE_ATU_UNR_UPPER_BASE		0x0c
#define PCIE_ATU_UNR_LIMIT		0x10
#define PCIE_ATU_UNR_LOWER_TARGET	0x14
#define PCIE_ATU_UNR_UPPER_TARGET	0x18

#define PCIE_ATU_REGION_INDEX1		(0x1 << 0)
#define PCIE_ATU_REGION_INDEX0		(0x0 << 0)
#define PCIE_ATU_TYPE_MEM		(0x0 << 0)
#define PCIE_ATU_TYPE_IO		(0x2 << 0)
#define PCIE_ATU_TYPE_CFG0		(0x4 << 0)
#define PCIE_ATU_TYPE_CFG1		(0x5 << 0)
#define PCIE_ATU_ENABLE			(0x1 << 31)
#define PCIE_ATU_BAR_MODE_ENABLE	(0x1 << 30)
#define PCIE_ATU_BUS(x)			(((x) & 0xff) << 24)
#define PCIE_ATU_DEV(x)			(((x) & 0x1f) << 19)
#define PCIE_ATU_FUNC(x)		(((x) & 0x7) << 16)

/* Register address builder */
#define PCIE_GET_ATU_OUTB_UNR_REG_OFFSET(region)	((region) << 9)

/* Offsets from App base */
#define PCIE_CMD_STATUS			0x04
#define LTSSM_EN_VAL			BIT(0)

/* Parameters for the waiting for iATU enabled routine */
#define LINK_WAIT_MAX_IATU_RETRIES	5
#define LINK_WAIT_IATU			10000

/**
 * struct pcie_dw_imx - i.MX DW PCIe controller state
 *
 * @dbi_base: The base address of dbi register space
 * @cfg_base: The base address of configuration space
 * @cfg_size: The size of the configuration space which is needed
 *            as it gets written into the PCIE_ATU_LIMIT register
 * @first_busno: This driver supports multiple PCIe controllers.
 *               first_busno stores the bus number of the PCIe root-port
 *               number which may vary depending on the PCIe setup
 *               (PEX switches etc).
 */
struct pcie_dw_imx {
	void *dbi_base;
	void *cfg_base;
	fdt_size_t cfg_size;
	int first_busno;
	struct udevice *dev;
	uint32_t ctrl_id;
	bool internal_refclk;
	uint32_t link_gen;

	/* IO and MEM PCI regions */
	struct pci_region io;
	struct pci_region mem;

	/* Resets */
	struct reset_ctl pciephy_ctl;
	struct reset_ctl apps_ctl;
	struct reset_ctl turnoff_ctl;
	int has_turnoff_ctl;

	/* GPIO */
	struct gpio_desc clkreq_gpio;
	struct gpio_desc disable_gpio;
	struct gpio_desc reset_gpio;

	/* regmap */
	struct regmap *anatop;
	struct regmap *gpr;

	/* power */
	struct power_domain power;
};

static void dw_pcie_writel_ob_unroll(struct pcie_dw_imx *pci, u32 index, u32 reg,
				     u32 val)
{
	u32 offset = PCIE_GET_ATU_OUTB_UNR_REG_OFFSET(index);

	writel(val, pci->dbi_base + PCIE_ATU_OFFSET + offset + reg);
}

static u32 dw_pcie_readl_ob_unroll(struct pcie_dw_imx *pci, u32 index, u32 reg)
{
	u32 offset = PCIE_GET_ATU_OUTB_UNR_REG_OFFSET(index);

	return readl(pci->dbi_base + PCIE_ATU_OFFSET + offset + reg);
}

/**
 * pcie_dw_prog_outbound_atu_unroll() - Configure ATU for outbound accesses
 *
 * @pcie: Pointer to the PCI controller state
 * @index: ATU region index
 * @type: ATU accsess type
 * @cpu_addr: the physical address for the translation entry
 * @pci_addr: the pcie bus address for the translation entry
 * @size: the size of the translation entry
 */
static void pcie_dw_prog_outbound_atu_unroll(struct pcie_dw_imx *pci, int index,
					     int type, u64 cpu_addr,
					     u64 pci_addr, u32 size)
{
	u32 retries, val;

	debug("ATU programmed with: index: %d, type: %d, cpu addr: %8llx, pci addr: %8llx, size: %8x\n",
	      index, type, cpu_addr, pci_addr, size);

	dw_pcie_writel_ob_unroll(pci, index, PCIE_ATU_UNR_LOWER_BASE,
				 lower_32_bits(cpu_addr));
	dw_pcie_writel_ob_unroll(pci, index, PCIE_ATU_UNR_UPPER_BASE,
				 upper_32_bits(cpu_addr));
	dw_pcie_writel_ob_unroll(pci, index, PCIE_ATU_UNR_LIMIT,
				 lower_32_bits(cpu_addr + size - 1));
	dw_pcie_writel_ob_unroll(pci, index, PCIE_ATU_UNR_LOWER_TARGET,
				 lower_32_bits(pci_addr));
	dw_pcie_writel_ob_unroll(pci, index, PCIE_ATU_UNR_UPPER_TARGET,
				 upper_32_bits(pci_addr));
	dw_pcie_writel_ob_unroll(pci, index, PCIE_ATU_UNR_REGION_CTRL1,
				 type);
	dw_pcie_writel_ob_unroll(pci, index, PCIE_ATU_UNR_REGION_CTRL2,
				 PCIE_ATU_ENABLE);

	/*
	 * Make sure ATU enable takes effect before any subsequent config
	 * and I/O accesses.
	 */
	for (retries = 0; retries < LINK_WAIT_MAX_IATU_RETRIES; retries++) {
		val = dw_pcie_readl_ob_unroll(pci, index,
					      PCIE_ATU_UNR_REGION_CTRL2);
		if (val & PCIE_ATU_ENABLE)
			return;

		udelay(LINK_WAIT_IATU);
	}
	dev_err(pci->dev, "outbound iATU is not being enabled\n");
}

/**
 * set_cfg_address() - Configure the PCIe controller config space access
 *
 * @pcie: Pointer to the PCI controller state
 * @d: PCI device to access
 * @where: Offset in the configuration space
 *
 * Configures the PCIe controller to access the configuration space of
 * a specific PCIe device and returns the address to use for this
 * access.
 *
 * Return: Address that can be used to access the configation space
 *         of the requested device / offset
 */
static uintptr_t set_cfg_address(struct pcie_dw_imx *pcie,
				 pci_dev_t d, uint where)
{
	int bus = PCI_BUS(d) - pcie->first_busno;
	uintptr_t va_address;
	u32 atu_type;

	/* Use dbi_base for own configuration read and write */
	if (!bus) {
		va_address = (uintptr_t)pcie->dbi_base;
		goto out;
	}

	if (bus == 1)
		/* For local bus, change TLP Type field to 4. */
		atu_type = PCIE_ATU_TYPE_CFG0;
	else
		/* Otherwise, change TLP Type field to 5. */
		atu_type = PCIE_ATU_TYPE_CFG1;

	/*
	 * Not accessing root port configuration space?
	 * Region #0 is used for Outbound CFG space access.
	 * Direction = Outbound
	 * Region Index = 0
	 */
	d = PCI_MASK_BUS(d);
	d = PCI_ADD_BUS(bus, d);
	pcie_dw_prog_outbound_atu_unroll(pcie, PCIE_ATU_REGION_INDEX1,
					 atu_type, (u64)pcie->cfg_base,
					 d << 8, pcie->cfg_size);

	va_address = (uintptr_t)pcie->cfg_base;

out:
	va_address += where & ~0x3;

	return va_address;
}

/**
 * pcie_dw_addr_valid() - Check for valid bus address
 *
 * @d: The PCI device to access
 * @first_busno: Bus number of the PCIe controller root complex
 *
 * Return 1 (true) if the PCI device can be accessed by this controller.
 *
 * Return: 1 on valid, 0 on invalid
 */
static int pcie_dw_addr_valid(pci_dev_t d, int first_busno)
{
	if ((PCI_BUS(d) == first_busno) && (PCI_DEV(d) > 0))
		return 0;
	if ((PCI_BUS(d) == first_busno + 1) && (PCI_DEV(d) > 0))
		return 0;

	return 1;
}

/**
 * pcie_dw_imx_read_config() - Read from configuration space
 *
 * @bus: Pointer to the PCI bus
 * @bdf: Identifies the PCIe device to access
 * @offset: The offset into the device's configuration space
 * @valuep: A pointer at which to store the read value
 * @size: Indicates the size of access to perform
 *
 * Read a value of size @size from offset @offset within the configuration
 * space of the device identified by the bus, device & function numbers in @bdf
 * on the PCI bus @bus.
 *
 * Return: 0 on success
 */
static int pcie_dw_imx_read_config(struct udevice *bus, pci_dev_t bdf,
				  uint offset, ulong *valuep,
				  enum pci_size_t size)
{
	struct pcie_dw_imx *pcie = dev_get_priv(bus);
	uintptr_t va_address;
	ulong value;

	debug("PCIE CFG read: bdf=%2x:%2x:%2x ",
	      PCI_BUS(bdf), PCI_DEV(bdf), PCI_FUNC(bdf));

	if (!pcie_dw_addr_valid(bdf, pcie->first_busno)) {
		debug("- out of range\n");
		*valuep = pci_get_ff(size);
		return 0;
	}

	va_address = set_cfg_address(pcie, bdf, offset);

	value = readl(va_address);

	debug("(addr,val)=(0x%04x, 0x%08lx)\n", offset, value);
	*valuep = pci_conv_32_to_size(value, offset, size);

	pcie_dw_prog_outbound_atu_unroll(pcie, PCIE_ATU_REGION_INDEX1,
					 PCIE_ATU_TYPE_IO, pcie->io.phys_start,
					 pcie->io.bus_start, pcie->io.size);

	return 0;
}

/**
 * pcie_dw_imx_write_config() - Write to configuration space
 *
 * @bus: Pointer to the PCI bus
 * @bdf: Identifies the PCIe device to access
 * @offset: The offset into the device's configuration space
 * @value: The value to write
 * @size: Indicates the size of access to perform
 *
 * Write the value @value of size @size from offset @offset within the
 * configuration space of the device identified by the bus, device & function
 * numbers in @bdf on the PCI bus @bus.
 *
 * Return: 0 on success
 */
static int pcie_dw_imx_write_config(struct udevice *bus, pci_dev_t bdf,
				   uint offset, ulong value,
				   enum pci_size_t size)
{
	struct pcie_dw_imx *pcie = dev_get_priv(bus);
	uintptr_t va_address;
	ulong old;

	debug("PCIE CFG write: (b,d,f)=(%2d,%2d,%2d) ",
	      PCI_BUS(bdf), PCI_DEV(bdf), PCI_FUNC(bdf));
	debug("(addr,val)=(0x%04x, 0x%08lx)\n", offset, value);

	if (!pcie_dw_addr_valid(bdf, pcie->first_busno)) {
		debug("- out of range\n");
		return 0;
	}

	va_address = set_cfg_address(pcie, bdf, offset);

	old = readl(va_address);
	value = pci_conv_size_to_32(old, value, offset, size);
	writel(value, va_address);

	pcie_dw_prog_outbound_atu_unroll(pcie, PCIE_ATU_REGION_INDEX1,
					 PCIE_ATU_TYPE_IO, pcie->io.phys_start,
					 pcie->io.bus_start, pcie->io.size);

	return 0;
}

static inline void dw_pcie_dbi_write_enable(struct pcie_dw_imx *pci, bool en)
{
	u32 val;

	val = readl(pci->dbi_base + PCIE_MISC_CONTROL_1_OFF);
	if (en)
		val |= PCIE_DBI_RO_WR_EN;
	else
		val &= ~PCIE_DBI_RO_WR_EN;
	writel(val, pci->dbi_base + PCIE_MISC_CONTROL_1_OFF);
}

/**
 * pcie_dw_configure() - Configure link capabilities and speed
 *
 * @regs_base: A pointer to the PCIe controller registers
 * @cap_speed: The capabilities and speed to configure
 *
 * Configure the link capabilities and speed in the PCIe root complex.
 */
static void pcie_dw_configure(struct pcie_dw_imx *pci, u32 cap_speed)
{
	u32 val;

	dw_pcie_dbi_write_enable(pci, true);

	val = readl(pci->dbi_base + PCIE_LINK_CAPABILITY);
	val &= ~TARGET_LINK_SPEED_MASK;
	val |= cap_speed;
	writel(val, pci->dbi_base + PCIE_LINK_CAPABILITY);

	dw_pcie_dbi_write_enable(pci, false);
}

/**
 * is_link_up() - Return the link state
 *
 * @regs_base: A pointer to the PCIe DBICS registers
 *
 * Return: 1 (true) for active line and 0 (false) for no link
 */
static int is_link_up(struct pcie_dw_imx *pci)
{
	u32 val;

	val = readl(pci->dbi_base + PCIE_PORT_DEBUG1);
	if ((val & PCIE_PHY_DEBUG_R1_LINK_UP) != 0 &&
	    (val & PCIE_PHY_DEBUG_R1_LINK_IN_TRAINING) == 0)
		return 1;

	return 0;
}

/**
 * wait_link_up() - Wait for the link to come up
 *
 * @regs_base: A pointer to the PCIe controller registers
 *
 * Return: 1 (true) for active line and 0 (false) for no link (timeout)
 */
static int wait_link_up(struct pcie_dw_imx *pci)
{
	unsigned long timeout;

	timeout = get_timer(0) + PCIE_LINK_UP_TIMEOUT_MS;
	while (!is_link_up(pci)) {
		if (get_timer(0) > timeout)
			return 0;
	};

	return 1;
}

static void pcie_dw_imx_init_phy(struct pcie_dw_imx *pci)
{
	struct udevice *dev = pci->dev;
	u32 val;

	if (device_is_compatible(dev, "fsl,imx8mq-pcie")) {
		power_domain_on(&pci->power);

		if (pci->ctrl_id == 0)
			val = IOMUXC_GPR14;
		else
			val = IOMUXC_GPR16;

		if (!pci->internal_refclk) {
			regmap_update_bits(pci->gpr, val,
					   IMX8MQ_GPR_PCIE_REF_USE_PAD,
					   IMX8MQ_GPR_PCIE_REF_USE_PAD);
		} else {
			regmap_update_bits(pci->gpr, val,
					   IMX8MQ_GPR_PCIE_REF_USE_PAD,
					   0);

			regmap_write(pci->anatop, ANATOP_PLLOUT_CTL,
				     ANATOP_PLLOUT_CTL_CKE |
				     ANATOP_PLLOUT_CTL_SEL_SYSPLL1);
			regmap_write(pci->anatop, ANATOP_PLLOUT_DIV,
				     ANATOP_PLLOUT_DIV_SYSPLL1);
		}

		if (pci->ctrl_id == 0)
			regmap_update_bits(pci->gpr,
					   IOMUXC_GPR12,
					   IMX8MQ_GPR_PCIE1_DEVICE_TYPE_MASK,
					   IMX8MQ_GPR_PCIE1_DEVICE_TYPE_RC);
		else
			regmap_update_bits(pci->gpr,
					   IOMUXC_GPR12,
					   IMX8MQ_GPR_PCIE2_DEVICE_TYPE_MASK,
					   IMX8MQ_GPR_PCIE2_DEVICE_TYPE_RC);
	}
}

static void pcie_dw_imx_enable_ref_clk(struct pcie_dw_imx *pci)
{
	struct udevice *dev = pci->dev;
	u32 val;

	if (device_is_compatible(dev, "fsl,imx8mq-pcie")) {
		if (pci->ctrl_id == 0)
			val = IOMUXC_GPR14;
		else
			val = IOMUXC_GPR16;

		regmap_update_bits(pci->gpr, val,
				   IMX8MQ_GPR_PCIE_CLK_REQ_OVERRIDE,
				   0);
		regmap_update_bits(pci->gpr, val,
				   IMX8MQ_GPR_PCIE_CLK_REQ_OVERRIDE_EN,
				   IMX8MQ_GPR_PCIE_CLK_REQ_OVERRIDE_EN);
	}
}

static void pcie_dw_imx_assert_core_reset(struct pcie_dw_imx *pci)
{
	struct udevice *dev = pci->dev;

	if (device_is_compatible(dev, "fsl,imx8mq-pcie")) {
		reset_assert(&pci->pciephy_ctl);
		reset_assert(&pci->apps_ctl);
	}
}

static void pcie_dw_imx_deassert_core_reset(struct pcie_dw_imx *pci)
{
	struct udevice *dev = pci->dev;

#ifdef CONFIG_ARCH_IMX8M
	init_clk_pcie(pci->ctrl_id);
#endif

	pcie_dw_imx_enable_ref_clk(pci);

	udelay(200);

	if (dm_gpio_is_valid(&pci->reset_gpio)) {
		dm_gpio_set_value(&pci->reset_gpio, 1); /* assert */
		mdelay(200);
		dm_gpio_set_value(&pci->reset_gpio, 0); /* de-assert */
		mdelay(200);
	}

	if (device_is_compatible(dev, "fsl,imx8mq-pcie"))
		reset_deassert(&pci->pciephy_ctl);
}

static void pcie_dw_imx_ltssm_enable(struct pcie_dw_imx *pci)
{
	struct udevice *dev = pci->dev;

	if (device_is_compatible(dev, "fsl,imx8mq-pcie"))
		reset_deassert(&pci->apps_ctl);
}

static int pcie_dw_imx_establish_link(struct pcie_dw_imx *pci)
{
	u32 val;

	pcie_dw_configure(pci, LINK_SPEED_GEN_1);
	pcie_dw_imx_ltssm_enable(pci);

	if (!wait_link_up(pci))
		return 0;

	if (pci->link_gen == 2) {
		pcie_dw_configure(pci, LINK_SPEED_GEN_2);
		val = readl(pci->dbi_base + PCIE_LINK_WIDTH_SPEED_CONTROL);
		val |= PORT_LOGIC_SPEED_CHANGE;
		writel(val, pci->dbi_base + PCIE_LINK_WIDTH_SPEED_CONTROL);
		if (!wait_link_up(pci))
			return 0;
	}

	return 1;
}

/**
 * pcie_dw_setup_host() - Setup the PCIe controller for RC opertaion
 *
 * @pcie: Pointer to the PCI controller state
 *
 * Configure the host BARs of the PCIe controller root port so that
 * PCI(e) devices may access the system memory.
 */
static void pcie_dw_setup_host(struct pcie_dw_imx *pci)
{
	u32 val;

	/* setup RC BARs */
	writel(PCI_BASE_ADDRESS_MEM_TYPE_64,
	       pci->dbi_base + PCI_BASE_ADDRESS_0);
	writel(0x0, pci->dbi_base + PCI_BASE_ADDRESS_1);

	/* setup interrupt pins */
	dw_pcie_dbi_write_enable(pci, true);
	val = readl(pci->dbi_base + PCI_INTERRUPT_LINE);
	val &= 0xffff00ff;
	val |= 0x00000100;
	writel(val, pci->dbi_base + PCI_INTERRUPT_LINE);
	dw_pcie_dbi_write_enable(pci, false);

	/* setup bus numbers */
	val = readl(pci->dbi_base + PCI_PRIMARY_BUS);
	val &= 0xff000000;
	val |= 0x00ff0100;
	writel(val, pci->dbi_base + PCI_PRIMARY_BUS);

	/* setup command register */
	val = readl(pci->dbi_base + PCI_COMMAND);
	val &= 0xffff0000;
	val |= PCI_COMMAND_IO | PCI_COMMAND_MEMORY |
		PCI_COMMAND_MASTER | PCI_COMMAND_SERR;
	writel(val, pci->dbi_base + PCI_COMMAND);

	/* Enable write permission for the DBI read-only register */
	dw_pcie_dbi_write_enable(pci, true);
	/* program correct class for RC */
	writew(PCI_CLASS_BRIDGE_PCI, pci->dbi_base + PCI_CLASS_DEVICE);
	/* Better disable write permission right after the update */
	dw_pcie_dbi_write_enable(pci, false);

	val = readl(pci->dbi_base + PCIE_LINK_WIDTH_SPEED_CONTROL);
	val |= PORT_LOGIC_SPEED_CHANGE;
	writel(val, pci->dbi_base + PCIE_LINK_WIDTH_SPEED_CONTROL);
}

static int pcie_dw_imx_host_init(struct pcie_dw_imx *pci)
{
	pcie_dw_imx_assert_core_reset(pci);
	pcie_dw_imx_init_phy(pci);
	pcie_dw_imx_deassert_core_reset(pci);
	pcie_dw_setup_host(pci);
	return pcie_dw_imx_establish_link(pci);
}


/**
 * pcie_dw_imx_probe() - Probe the PCIe bus for active link
 *
 * @dev: A pointer to the device being operated on
 *
 * Probe for an active link on the PCIe bus and configure the controller
 * to enable this port.
 *
 * Return: 0 on success, else -ENODEV
 */
static int pcie_dw_imx_probe(struct udevice *dev)
{
	struct pcie_dw_imx *pci = dev_get_priv(dev);
	struct udevice *ctlr = pci_get_controller(dev);
	struct pci_controller *hose = dev_get_uclass_priv(ctlr);
	struct udevice *syscon;
	int ret;

	/* Get the controller base address */
	pci->dbi_base = (void *)dev_read_addr_index(dev, 0);
	if ((fdt_addr_t)pci->dbi_base == FDT_ADDR_T_NONE)
		return -EINVAL;

	/* Get the config space base address and size */
	pci->cfg_base = (void *)dev_read_addr_size_index(dev, 1,
							 &pci->cfg_size);
	if ((fdt_addr_t)pci->cfg_base == FDT_ADDR_T_NONE)
		return -EINVAL;

	if (dev_read_u32(dev, "ctrl-id", &pci->ctrl_id) < 0)
		pci->ctrl_id = 0;
	pci->internal_refclk = dev_read_bool(dev, "internal-refclk");
	if (dev_read_u32(dev, "fsl,max-link-speed", &pci->link_gen) < 0)
		pci->link_gen = 1;

	ret = power_domain_get(dev, &pci->power);
	if (ret) {
		dev_err(dev, "failed to get power domain\n");
		return ret;
	}

	ret = reset_get_by_name(dev, "pciephy", &pci->pciephy_ctl);
	if (ret) {
		dev_err(dev, "failed to get reset for pciephy\n");
		return ret;
	}

	ret = reset_get_by_name(dev, "apps", &pci->apps_ctl);
	if (ret) {
		dev_err(dev, "failed to get reset for apps\n");
		return ret;
	}

	ret = reset_get_by_name(dev, "turnoff", &pci->turnoff_ctl);
	if (!ret)
		pci->has_turnoff_ctl = 1;

	ret = uclass_get_device_by_phandle(UCLASS_SYSCON, dev,
					   "anatop", &syscon);
	if (ret) {
		dev_err(dev, "unable to find anatop\n");
		return ret;
	}

	pci->anatop = syscon_get_regmap(syscon);
	if (IS_ERR(pci->anatop)) {
		dev_err(dev, "failed to get regmap for anatop\n");
		return PTR_ERR(pci->anatop);
	}

	ret = uclass_get_device_by_phandle(UCLASS_SYSCON, dev,
					   "gpr", &syscon);
	if (ret) {
		dev_err(dev, "unable to find gpr\n");
		return ret;
	}

	pci->gpr = syscon_get_regmap(syscon);
	if (IS_ERR(pci->gpr)) {
		dev_err(dev, "failed to get regmap for gpr\n");
		return PTR_ERR(pci->gpr);
	}

	gpio_request_by_name(dev, "clkreq-gpio", 0, &pci->clkreq_gpio,
			     GPIOD_IS_OUT);
	if (dm_gpio_is_valid(&pci->clkreq_gpio))
		dm_gpio_set_value(&pci->clkreq_gpio, 1);
	gpio_request_by_name(dev, "disable-gpio", 0, &pci->disable_gpio,
			     GPIOD_IS_OUT);
	if (dm_gpio_is_valid(&pci->disable_gpio))
		dm_gpio_set_value(&pci->disable_gpio, 0);
	gpio_request_by_name(dev, "reset-gpio", 0, &pci->reset_gpio,
			     GPIOD_IS_OUT);
	if (dm_gpio_is_valid(&pci->reset_gpio))
		dm_gpio_set_value(&pci->reset_gpio, 1);

	pci->first_busno = dev->seq;
	pci->dev = dev;

	if (!pcie_dw_imx_host_init(pci)) {
		printf("PCIE-%d: Link down\n", dev->seq);
		return -ENODEV;
	}

	printf("PCIE-%d: Link up (Bus%d)\n", dev->seq, hose->first_busno);

	/* Store the IO and MEM windows settings for future use by the ATU */
	pci->io.phys_start = hose->regions[0].phys_start; /* IO base */
	pci->io.bus_start  = hose->regions[0].bus_start;  /* IO_bus_addr */
	pci->io.size	    = hose->regions[0].size;	   /* IO size */

	pci->mem.phys_start = hose->regions[1].phys_start; /* MEM base */
	pci->mem.bus_start  = hose->regions[1].bus_start;  /* MEM_bus_addr */
	pci->mem.size	     = hose->regions[1].size;	    /* MEM size */

	pcie_dw_prog_outbound_atu_unroll(pci, PCIE_ATU_REGION_INDEX0,
					 PCIE_ATU_TYPE_MEM,
					 pci->mem.phys_start,
					 pci->mem.bus_start, pci->mem.size);

	return 0;
}

static const struct dm_pci_ops pcie_dw_imx_ops = {
	.read_config	= pcie_dw_imx_read_config,
	.write_config	= pcie_dw_imx_write_config,
};

static const struct udevice_id pcie_dw_imx_ids[] = {
	{ .compatible = "fsl,imx8mq-pcie" },
	{ }
};

U_BOOT_DRIVER(pcie_dw_imx) = {
	.name			= "pcie_dw_imx",
	.id			= UCLASS_PCI,
	.of_match		= pcie_dw_imx_ids,
	.ops			= &pcie_dw_imx_ops,
	.probe			= pcie_dw_imx_probe,
	.priv_auto_alloc_size	= sizeof(struct pcie_dw_imx),
};
