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
#include "arch/at91_ccfg.h"
#include "arch/at91_matrix.h"
#include "arch/at91_rstc.h"
#include "arch/at91_pmc.h"
#include "arch/at91_smc.h"
#include "arch/at91_pio.h"
#include "arch/at91_sdramc.h"
#include "spi.h"
#include "gpio.h"
#include "pmc.h"
#include "usart.h"
#include "debug.h"
#include "sdramc.h"
#include "timer.h"
#include "watchdog.h"
#include "string.h"
#include "at91kizbox.h"

static void at91_dbgu_hw_init(void)
{
	/* Configure DBGU pin */
	const struct pio_desc dbgu_pins[] = {
		{"RXD", AT91C_PIN_PB(14), 0, PIO_DEFAULT, PIO_PERIPH_A},
		{"TXD", AT91C_PIN_PB(15), 0, PIO_DEFAULT, PIO_PERIPH_A},
		{(char *)0, 0, 0, PIO_DEFAULT, PIO_PERIPH_A},
	};

	/* Configure the dbgu pins */
	pmc_enable_periph_clock(AT91C_ID_PIOB);
	pio_configure(dbgu_pins);
}

static void initialize_dbgu(void)
{
	at91_dbgu_hw_init();

	usart_init(BAUDRATE(MASTER_CLOCK, 115200));
}

static void sdramc_init(void)
{
	struct sdramc_register sdramc_config;

	sdramc_config.cr = AT91C_SDRAMC_NC_9 | AT91C_SDRAMC_NR_13 | AT91C_SDRAMC_CAS_3
				| AT91C_SDRAMC_NB_4_BANKS | AT91C_SDRAMC_DBW_16_BITS
				| AT91C_SDRAMC_TWR_2 | AT91C_SDRAMC_TRC_9
				| AT91C_SDRAMC_TRP_3 | AT91C_SDRAMC_TRCD_3
				| AT91C_SDRAMC_TRAS_6 | AT91C_SDRAMC_TXSR_10;

	sdramc_config.tr = (MASTER_CLOCK * 7) / 1000000;
	sdramc_config.mdr = AT91C_SDRAMC_MD_SDRAM;

	/* Initialize the matrix (memory voltage = 3.3) */
	writel((readl(AT91C_BASE_CCFG + CCFG_EBICSA))
		| AT91C_EBI_CS1A_SDRAMC
		| (0x03 << 16),
		AT91C_BASE_CCFG + CCFG_EBICSA);

	sdramc_initialize(&sdramc_config, AT91C_BASE_CS1);
}

#ifdef CONFIG_NANDFLASH_RECOVERY
static void recovery_buttons_hw_init(void)
{
	/* Configure recovery button PINs */
	const struct pio_desc recovery_button_pins[] = {
		{"RECOVERY_BUTTON", CONFIG_SYS_RECOVERY_BUTTON_PIN, 0, PIO_PULLUP, PIO_INPUT},
		{(char *)0, 0, 0, PIO_DEFAULT, PIO_PERIPH_A},
	};

	pmc_enable_periph_clock(AT91C_ID_PIOB);
	pio_configure(recovery_button_pins);
}
#endif /* #ifdef CONFIG_NANDFLASH_RECOVERY */

#ifdef CONFIG_HW_INIT
void hw_init(void)
{
	/* Disable watchdog */
	at91_disable_wdt();

	/*
	 * At this stage the main oscillator is supposed to be enabled
	 * PCK = MCK = MOSC
	 */
	pmc_init_pll(0);

	/* Configure PLLA = MOSC * (PLL_MULA + 1) / PLL_DIVA */
	pmc_cfg_plla(PLLA_SETTINGS);

	/* PCK = PLLA/2 = 3 * MCK */
	pmc_cfg_mck(MCKR_SETTINGS);

	/* Switch MCK on PLLA output */
	pmc_cfg_mck(MCKR_CSS_SETTINGS);

	/* Configure the EBI Slave Slot Cycle to 64 */
	writel((readl((AT91C_BASE_MATRIX + MATRIX_SCFG3)) & ~0xFF) | 0x40,
		(AT91C_BASE_MATRIX + MATRIX_SCFG3));

	/* Initialize dbgu */
	initialize_dbgu();

	/* Initlialize sdram controller */
	sdramc_init();

#ifdef CONFIG_USER_HW_INIT
	/* Configure LEDs pins */
	const struct pio_desc leds_pins[] = {
		{"LED_RED",   AT91C_PIN_PB(2), 1, PIO_DEFAULT, PIO_OUTPUT},
		{"LED_GREEN", AT91C_PIN_PB(3), 1, PIO_DEFAULT, PIO_OUTPUT},
		{(char *)0, 0, 0, PIO_DEFAULT, PIO_PERIPH_A},
	};

	pmc_enable_periph_clock(AT91C_ID_PIOB);
	pio_configure(leds_pins);
#endif

#ifdef CONFIG_NANDFLASH_RECOVERY
	/* Init the recovery buttons pins */
	recovery_buttons_hw_init();
#endif
}
#endif /* #ifdef CONFIG_HW_INIT */

void nandflash_hw_init(void)
{
	unsigned int reg;

	/* Configure PIOs */
	const struct pio_desc nand_pins[] = {
		{"RDY_BSY",	AT91C_PIN_PC(13),	0, PIO_PULLUP, PIO_INPUT},
		{"NANDCS", 	AT91C_PIN_PC(14),	0, PIO_PULLUP, PIO_OUTPUT},
		{(char *)0, 	0, 0, PIO_DEFAULT, PIO_PERIPH_A},
	};

	/* Setup Smart Media, first enable the address range of CS3 in HMATRIX user interface */
	reg = readl(AT91C_BASE_CCFG + CCFG_EBICSA);
	reg |= AT91C_EBI_CS3A_SM;

	writel(reg, AT91C_BASE_CCFG + CCFG_EBICSA);

	/* Configure SMC CS3 */
	writel((AT91C_SMC_NWESETUP_(2)
		| AT91C_SMC_NCS_WRSETUP_(0)
		| AT91C_SMC_NRDSETUP_(2)
		| AT91C_SMC_NCS_RDSETUP_(0)),
		AT91C_BASE_SMC + SMC_SETUP3);

	writel((AT91C_SMC_NWEPULSE_(4)
		| AT91C_SMC_NCS_WRPULSE_(4)
		| AT91C_SMC_NRDPULSE_(4)
		| AT91C_SMC_NCS_RDPULSE_(4)),
		AT91C_BASE_SMC + SMC_PULSE3);

	writel((AT91C_SMC_NWECYCLE_(7)
		|  AT91C_SMC_NRDCYCLE_(7)),
		AT91C_BASE_SMC + SMC_CYCLE3);

	writel((AT91C_SMC_READMODE
		| AT91C_SMC_WRITEMODE
		| AT91C_SMC_NWAITM_NWAIT_DISABLE
		| AT91C_SMC_DBW_WIDTH_BITS_8
		| AT91_SMC_TDF_(3)),
		AT91C_BASE_SMC + SMC_CTRL3);

	/* Configure the PIO controller */
	pmc_enable_periph_clock(AT91C_ID_PIOC);
	pio_configure(nand_pins);
}
