/*
 * Copyright (c) 2014-2015, Overkiz SAS
 *                          Gaël PORTAY <g.portay@overkiz.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#include "common.h"
#include "hardware.h"
#include "pmc.h"
#include "usart.h"
#include "debug.h"
#include "ddramc.h"
#include "spi.h"
#include "gpio.h"
#include "timer.h"
#include "watchdog.h"

#include "arch/at91_pmc.h"
#include "arch/at91_rstc.h"
#include "arch/sama5_smc.h"
#include "arch/at91_pio.h"
#include "arch/at91_ddrsdrc.h"
#include "at91kizbox2.h"
#include "act8865.h"
#include "twi.h"

static void at91_dbgu_hw_init(void)
{
	/* Configure DBGU pin */
	const struct pio_desc dbgu_pins[] = {
		{"RXD", AT91C_PIN_PB(30), 0, PIO_DEFAULT, PIO_PERIPH_A},
		{"TXD", AT91C_PIN_PB(31), 0, PIO_DEFAULT, PIO_PERIPH_A},
		{(char *)0, 0, 0, PIO_DEFAULT, PIO_PERIPH_A},
	};

	/*  Configure the dbgu pins */
	pmc_enable_periph_clock(AT91C_ID_PIOB);
	pio_configure(dbgu_pins);

	/* Enable clock */
	pmc_enable_periph_clock(AT91C_ID_DBGU);
}

static void initialize_dbgu(void)
{
	at91_dbgu_hw_init();
	usart_init(BAUDRATE(MASTER_CLOCK, 115200));
}

static void ddramc_reg_config(struct ddramc_register *ddramc_config)
{
	ddramc_config->mdr = (AT91C_DDRC2_DBW_32_BITS
				| AT91C_DDRC2_MD_DDR2_SDRAM);

	ddramc_config->cr = (AT91C_DDRC2_NC_DDR10_SDR9
				| AT91C_DDRC2_NR_13
				| AT91C_DDRC2_CAS_3
				| AT91C_DDRC2_DLL_RESET_DISABLED
				| AT91C_DDRC2_DIS_DLL_DISABLED
				| AT91C_DDRC2_ENRDM_ENABLE
				| AT91C_DDRC2_NB_BANKS_8
				| AT91C_DDRC2_NDQS_DISABLED
				| AT91C_DDRC2_DECOD_INTERLEAVED
				| AT91C_DDRC2_UNAL_SUPPORTED);

#if defined(CONFIG_BUS_SPEED_133MHZ)
	/*
	 * The DDR2-SDRAM device requires a refresh every 15.625 us or 7.81 us.
	 * With a 133 MHz frequency, the refresh timer count register must to be
	 * set with (15.625 x 133 MHz) ~ 2084 i.e. 0x824
	 * or (7.81 x 133 MHz) ~ 1039 i.e. 0x40F.
	 */
	ddramc_config->rtr = 0x40F;     /* Refresh timer: 7.812us */

	/* One clock cycle @ 133 MHz = 7.5 ns */
	ddramc_config->t0pr = (AT91C_DDRC2_TRAS_(6)	/* 6 * 7.5 = 45 ns */
			| AT91C_DDRC2_TRCD_(2)		/* 2 * 7.5 = 22.5 ns */
			| AT91C_DDRC2_TWR_(2)		/* 2 * 7.5 = 15   ns */
			| AT91C_DDRC2_TRC_(8)		/* 8 * 7.5 = 75   ns */
			| AT91C_DDRC2_TRP_(2)		/* 2 * 7.5 = 15   ns */
			| AT91C_DDRC2_TRRD_(2)		/* 2 * 7.5 = 15   ns */
			| AT91C_DDRC2_TWTR_(2)		/* 2 clock cycles min */
			| AT91C_DDRC2_TMRD_(2));	/* 2 clock cycles */

	ddramc_config->t1pr = (AT91C_DDRC2_TXP_(2)	/* 2 clock cycles */
			| AT91C_DDRC2_TXSRD_(200)	/* 200 clock cycles */
			| AT91C_DDRC2_TXSNR_(19)	/* 19 * 7.5 = 142.5 ns */
			| AT91C_DDRC2_TRFC_(17));	/* 17 * 7.5 = 127.5 ns */

	ddramc_config->t2pr = (AT91C_DDRC2_TFAW_(6)	/* 6 * 7.5 = 45 ns */
			| AT91C_DDRC2_TRTP_(2)		/* 2 clock cycles min */
			| AT91C_DDRC2_TRPA_(2)		/* 2 * 7.5 = 15 ns */
			| AT91C_DDRC2_TXARDS_(8)	/* = TXARD */
			| AT91C_DDRC2_TXARD_(8));	/* MR12 = 1 */

#elif defined(CONFIG_BUS_SPEED_166MHZ)
	/*
	 * The DDR2-SDRAM device requires a refresh of all rows every 64ms.
	 * ((64ms) / 8192) * 166MHz = 1296 i.e. 0x510
	 */
	ddramc_config->rtr = 0x510;

	/* One clock cycle @ 166 MHz = 6.0 ns */
	ddramc_config->t0pr = (AT91C_DDRC2_TRAS_(8)	/* 8 * 6 = 48 ns */
			| AT91C_DDRC2_TRCD_(3)		/* 3 * 6 = 18 ns */
			| AT91C_DDRC2_TWR_(3)		/* 3 * 6 = 18 ns */
			| AT91C_DDRC2_TRC_(10)		/* 10 * 6 = 60 ns */
			| AT91C_DDRC2_TRP_(3)		/* 3 * 6 = 18 ns */
			| AT91C_DDRC2_TRRD_(2)		/* 2 * 6 = 12 ns */
			| AT91C_DDRC2_TWTR_(2)		/* 2 clock cycles */
			| AT91C_DDRC2_TMRD_(2));	/* 2 clock cycles */

	ddramc_config->t1pr = (AT91C_DDRC2_TXP_(2)	/* 2 * 6 = 12ns */
			| AT91C_DDRC2_TXSRD_(200)	/* 200 clock cycles */
			| AT91C_DDRC2_TXSNR_(23)	/* 23 * 6 = 138 ns */
			| AT91C_DDRC2_TRFC_(22));	/* 22 * 6 = 132 ns */

	ddramc_config->t2pr = (AT91C_DDRC2_TFAW_(8)	/* 45 ns */
			| AT91C_DDRC2_TRTP_(2)		/* 2 * 6 = 15ns */
			| AT91C_DDRC2_TRPA_(3)		/* 15 ns */
			| AT91C_DDRC2_TXARDS_(8)	/* = TXARD */
			| AT91C_DDRC2_TXARD_(8));	/* 8 clock cycles */

#else
#error "No bus clock provided!"
#endif
}

static void ddramc_init(void)
{
	struct ddramc_register ddramc_reg;
	unsigned int reg;

	ddramc_reg_config(&ddramc_reg);

	/* enable ddr2 clock */
	pmc_enable_periph_clock(AT91C_ID_MPDDRC);
	pmc_enable_system_clock(AT91C_PMC_DDR);

	/* Init the special register for sama5d3x */
	/* MPDDRC DLL Slave Offset Register: DDR2 configuration */
	reg = AT91C_MPDDRC_S0OFF_1
		| AT91C_MPDDRC_S2OFF_1
		| AT91C_MPDDRC_S3OFF_1;
	writel(reg, (AT91C_BASE_MPDDRC + MPDDRC_DLL_SOR));

	/* MPDDRC DLL Master Offset Register */
	/* write master + clk90 offset */
	reg = AT91C_MPDDRC_MOFF_7
		| AT91C_MPDDRC_CLK90OFF_31
		| AT91C_MPDDRC_SELOFF_ENABLED | AT91C_MPDDRC_KEY;
	writel(reg, (AT91C_BASE_MPDDRC + MPDDRC_DLL_MOR));

	/* MPDDRC I/O Calibration Register */
	/* DDR2 RZQ = 50 Ohm */
	/* TZQIO = 4 */
	reg = AT91C_MPDDRC_RDIV_DDR2_RZQ_50
		| AT91C_MPDDRC_TZQIO_4;
	writel(reg, (AT91C_BASE_MPDDRC + MPDDRC_IO_CALIBR));

	/* DDRAM2 Controller initialize */
	ddram_initialize(AT91C_BASE_MPDDRC, AT91C_BASE_DDRCS, &ddramc_reg);
}

#ifdef CONFIG_NANDFLASH_RECOVERY
static void recovery_buttons_hw_init(void)
{
	/* Configure recovery button PINs */
	const struct pio_desc recovery_button_pins[] = {
		{"RECOVERY_BUTTON", CONFIG_SYS_RECOVERY_BUTTON_PIN, 0, PIO_PULLUP, PIO_INPUT},
		{(char *)0, 0, 0, PIO_DEFAULT, PIO_PERIPH_A},
	};

	pmc_enable_periph_clock(AT91C_ID_PIOE);
	pio_configure(recovery_button_pins);
}
#endif /* #ifdef CONFIG_NANDFLASH_RECOVERY */

/*
 * Special setting for PM.
 * Since for the chips with no EMAC or GMAC, No actions is done to make
 * its phy to enter the power save mode when linux system enter suspend
 * to memory or standby.
 * And it causes the VDDCORE current is higher than our expection.
 * So set GMAC clock related pins GTXCK(PB8), GRXCK(PB11), GMDCK(PB16),
 * G125CK(PB18) and EMAC clock related pins EREFCK(PC7), EMDC(PC8)
 * to Pullup and Pulldown disabled, and output low.
 */

#define EMAC_PINS	((0x01 << 7) | (0x01 << 8))

static void at91_special_pio_output_low(void)
{
	unsigned int base;
	unsigned int value;

	base = AT91C_BASE_PIOC;
	value = EMAC_PINS;

	writel((1 << AT91C_ID_PIOC), (PMC_PCER + AT91C_BASE_PMC));

	writel(value, base + PIO_REG_PPUDR);	/* PIO_PPUDR */
	writel(value, base + PIO_REG_PPDDR);	/* PIO_PPDDR */
	writel(value, base + PIO_REG_PER);	/* PIO_PER */
	writel(value, base + PIO_REG_OER);	/* PIO_OER */
	writel(value, base + PIO_REG_CODR);	/* PIO_CODR */
}

#if defined(CONFIG_TWI1)
unsigned int at91_twi1_hw_init(void)
{
	unsigned int base_addr = AT91C_BASE_TWI1;

	const struct pio_desc twi_pins[] = {
		{"TWD", AT91C_PIN_PC(26), 0, PIO_DEFAULT, PIO_PERIPH_B},
		{"TWCK", AT91C_PIN_PC(27), 0, PIO_DEFAULT, PIO_PERIPH_B},
		{(char *)0, 0, 0, PIO_DEFAULT, PIO_PERIPH_A},
	};

	pio_configure(twi_pins);
	pmc_enable_periph_clock(AT91C_ID_PIOC);

	pmc_enable_periph_clock(AT91C_ID_TWI1);

	return base_addr;
}
#endif

int at91_board_act8865_set_reg_voltage(void)
{
	unsigned char reg, value;
	int ret;

	/* Check ACT8865 I2C interface */
	if (act8865_check_i2c_disabled())
		return 0;

	/* Enable REG6 output 3.3V */
	reg = REG6_0;
	value = ACT8865_3V3;
	ret = act8865_set_reg_voltage(reg, value);
	if (ret) {
		dbg_info("ACT8865: Failed to make REG6 output 3300mV\n");
		return -1;
	}

	dbg_info("ACT8865: The REG6 output 3300mV\n");

	return 0;
}

#ifdef CONFIG_HW_INIT
void hw_init(void)
{
	/* Disable watchdog */
	at91_disable_wdt();

	/*
	 * At this stage the main oscillator
	 * is supposed to be enabled PCK = MCK = MOSC
	 */

	/* Configure PLLA = MOSC * (PLL_MULA + 1) / PLL_DIVA */
	pmc_cfg_plla(PLLA_SETTINGS);

	/* Initialize PLLA charge pump */
	pmc_init_pll(AT91C_PMC_IPLLA_3);

	/* Switch PCK/MCK on Main clock output */
	pmc_cfg_mck(BOARD_PRESCALER_MAIN_CLOCK);

	/* Switch PCK/MCK on PLLA output */
	pmc_cfg_mck(BOARD_PRESCALER_PLLA);

	/* Set GMAC & EMAC pins to output low */
	at91_special_pio_output_low();

	/* Init timer */
	timer_init();

	/* initialize the dbgu */
	initialize_dbgu();

	/* Initialize MPDDR Controller */
	ddramc_init();

#ifdef CONFIG_USER_HW_INIT
	/* Configure LEDs pins */
	const struct pio_desc leds_pins[] = {
		{"LED_RED",   AT91C_PIN_PB(0), 1, PIO_DEFAULT, PIO_OUTPUT},
		{"LED_GREEN", AT91C_PIN_PB(4), 1, PIO_DEFAULT, PIO_OUTPUT},
		{"LED_BLUE",  AT91C_PIN_PB(8), 0, PIO_DEFAULT, PIO_OUTPUT},
		{(char *)0, 0, 0, PIO_DEFAULT, PIO_PERIPH_A},
	};

	pmc_enable_periph_clock(AT91C_ID_PIOB);
	pio_configure(leds_pins);
#endif

#ifdef CONFIG_NANDFLASH_RECOVERY
	/* Init the recovery buttons pins */
	recovery_buttons_hw_init();
#endif

#ifndef CONFIG_DISABLE_ACT8865_I2C
	if (!twi_init_done)
		twi_init();

	/* Set ACT8865 output voltage */
	at91_board_act8865_set_reg_voltage();
#endif
}
#endif /* #ifdef CONFIG_HW_INIT */

void nandflash_hw_init(void)
{
	/* Configure nand pins */
	const struct pio_desc nand_pins[] = {
		{"NANDALE", AT91C_PIN_PE(21), 0, PIO_PULLUP, PIO_PERIPH_A},
		{"NANDCLE", AT91C_PIN_PE(22), 0, PIO_PULLUP, PIO_PERIPH_A},
		{(char *)0, 0, 0, PIO_DEFAULT, PIO_PERIPH_A},
	};

	/* Configure the nand controller pins*/
	pmc_enable_periph_clock(AT91C_ID_PIOE);
	pio_configure(nand_pins);

	/* Enable the clock */
	pmc_enable_periph_clock(AT91C_ID_SMC);

	/* Configure SMC CS3 for NAND/SmartMedia */
	writel(AT91C_SMC_SETUP_NWE(1)
		| AT91C_SMC_SETUP_NCS_WR(1)
		| AT91C_SMC_SETUP_NRD(2)
		| AT91C_SMC_SETUP_NCS_RD(1),
		(ATMEL_BASE_SMC + SMC_SETUP3));

	writel(AT91C_SMC_PULSE_NWE(5)
		| AT91C_SMC_PULSE_NCS_WR(7)
		| AT91C_SMC_PULSE_NRD(5)
		| AT91C_SMC_PULSE_NCS_RD(7),
		(ATMEL_BASE_SMC + SMC_PULSE3));

	writel(AT91C_SMC_CYCLE_NWE(8)
		| AT91C_SMC_CYCLE_NRD(9),
		(ATMEL_BASE_SMC + SMC_CYCLE3));

	writel(AT91C_SMC_TIMINGS_TCLR(3)
		| AT91C_SMC_TIMINGS_TADL(10)
		| AT91C_SMC_TIMINGS_TAR(3)
		| AT91C_SMC_TIMINGS_TRR(4)
		| AT91C_SMC_TIMINGS_TWB(5)
		| AT91C_SMC_TIMINGS_RBNSEL(3)
		| AT91C_SMC_TIMINGS_NFSEL,
		(ATMEL_BASE_SMC + SMC_TIMINGS3));

	writel(AT91C_SMC_MODE_READMODE_NRD_CTRL
		| AT91C_SMC_MODE_WRITEMODE_NWE_CTRL
		| AT91C_SMC_MODE_EXNWMODE_DISABLED
		| AT91C_SMC_MODE_DBW_8
		| AT91C_SMC_MODE_TDF_CYCLES(1),
		(ATMEL_BASE_SMC + SMC_MODE3));
}
