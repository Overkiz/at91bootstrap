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
#ifndef __AT91KIZBOXMINI_H__
#define __AT91KIZBOXMINI_H__

/*
 * PMC Setting
 *
 * The main oscillator is enabled as soon as possible in the lowlevel_clock_init
 * and MCK is switched on the main oscillator.
 * PLL initialization is done later in the hw_init() function
 */
#define MASTER_CLOCK		132096000

#define BAUD_RATE		115200
#define BOARD_MAINOSC		12000000

/* PCK = 396MHz, MCK = 132MHz */
#define PLLA_MULA		199
#define PLLA_DIVA		3
#define BOARD_MCK		((unsigned long)(((BOARD_MAINOSC / \
					PLLA_DIVA) * (PLLA_MULA + 1)) / 2 / 3))
#define BOARD_OSCOUNT		(AT91C_CKGR_OSCOUNT & (64 << 8))
#define BOARD_CKGR_PLLA		(AT91C_CKGR_SRCA | AT91C_CKGR_OUTA_0)
#define BOARD_PLLACOUNT		(0x3F << 8)
#define BOARD_MULA		(AT91C_CKGR_MULA & (PLLA_MULA << 16))
#define BOARD_DIVA		(AT91C_CKGR_DIVA & PLLA_DIVA)

#define BOARD_PRESCALER_MAIN_CLOCK	(AT91C_PMC_PLLADIV2_2 \
					| AT91C_PMC_MDIV_3 \
					| AT91C_PMC_CSS_MAIN_CLK)

#define BOARD_PRESCALER_PLLA		(AT91C_PMC_PLLADIV2_2 \
					| AT91C_PMC_MDIV_3 \
					| AT91C_PMC_CSS_PLLA_CLK)

#define PLLA_SETTINGS	(BOARD_CKGR_PLLA \
			| BOARD_PLLACOUNT \
			| BOARD_MULA \
			| BOARD_DIVA)

/*
 * NandFlash Settings
 */
#define CONFIG_SYS_NAND_BASE            AT91C_BASE_CS3
#define CONFIG_SYS_NAND_MASK_ALE        (1 << 21)
#define CONFIG_SYS_NAND_MASK_CLE        (1 << 22)

#define CONFIG_SYS_NAND_OE_PIN		AT91C_PIN_PD(0)
#define CONFIG_SYS_NAND_WE_PIN		AT91C_PIN_PD(1)
#define CONFIG_SYS_NAND_ALE_PIN		AT91C_PIN_PD(2)
#define CONFIG_SYS_NAND_CLE_PIN		AT91C_PIN_PD(3)
#define CONFIG_SYS_NAND_ENABLE_PIN      AT91C_PIN_PD(4)

#define CONFIG_LOOKUP_TABLE_ALPHA_OFFSET	0xC000
#define CONFIG_LOOKUP_TABLE_INDEX_OFFSET	0x8000

#define CONFIG_LOOKUP_TABLE_ALPHA_OFFSET_1024	0x18000
#define CONFIG_LOOKUP_TABLE_INDEX_OFFSET_1024	0x10000

/*
 * Recovery function
 */
#define CONFIG_SYS_RECOVERY_BUTTON_PIN	AT91C_PIN_PC(16)
#define RECOVERY_BUTTON_NAME	"PB_RST"


#endif /* #ifndef __AT91KIZBOXMINI_H__ */