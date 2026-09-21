/*
 * Copyright (c) 2026 Linumiz
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ti_mspm0_system_timer

#include <zephyr/irq.h>
#include <zephyr/device.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

#include <zephyr/drivers/clock_control/mspm0_clock_control.h>
#include <zephyr/drivers/timer/system_timer.h>

#include <soc.h>

/* TODO:
 * TIMER_CORE_CYCLES_PER_SEC will change based on the prescale and clk div so need to adjust it.
 */
#define MSPM0_TMR_BASE     DT_REG_ADDR(DT_INST_PARENT(0))
#define MSPM0_TMR_IRQ_NUM  DT_IRQN(DT_INST_PARENT(0))
#define MSPM0_TMR_IRQ_PRIO DT_IRQ(DT_INST_PARENT(0), priority)
#define MSPM0_TMR_CLOCK    DT_CLOCKS_CELL_BY_IDX(DT_INST_PARENT(0), 0, clk)
#define MSPM0_TMR_PRESCALE DT_PROP(DT_INST_PARENT(0), ti_clk_prescaler)
#define MSPM0_TMR_CLK_DIV  (DT_PROP(DT_INST_PARENT(0), ti_clk_div) - 1U)

#define MSPM0_TMR_CYC_PER_SEC                                                                      \
	(CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC / ((MSPM0_TMR_CLK_DIV + 1) * (MSPM0_TMR_PRESCALE + 1)))

#define MSPM0_TMR_CYCLES_MAX UINT16_MAX
/*
 * MSPM0 timer register offset
 */

/* Power/Reset/Clock registers */
#define MSPM0_TMR_REG_PWREN  0x800
#define MSPM0_TMR_REG_RSTCTL 0x804
#define MSPM0_TMR_REG_CLKSEL 0x1008
#define MSPM0_TMR_REG_CPS    0x110c
#define MSPM0_TMR_REG_CLKDIV 0x1000

/* Counter control registers */
#define MSPM0_TMR_REG_CTR    0x1800
#define MSPM0_TMR_REG_CTRCTL 0x1804
#define MSPM0_TMR_REG_LOAD   0x1808

/* Compare and compare control registers */
#define MSPM0_TMR_REG_CC_0 0x1810

/* CPU interrupt registers */
#define MSPM0_TMR_REG_CPU_INT_IIDX  0x1020
#define MSPM0_TMR_REG_CPU_INT_IMASK 0x1028
#define MSPM0_TMR_REG_CPU_INT_ICLR  0x1048

/* Reset/Power reg masks */
#define MSPM0_TMR_RSTCTL_UNLOCK_KEY      ((uint32_t)0xb1000000U)
#define MSPM0_TMR_RSTCTL_CLEAR_RESETSTKY BIT(1)
#define MSPM0_TMR_RSTCTL_ASSERT_RESET    BIT(0)

#define MSPM0_TMR_PWREN_UNLOCK_KEY ((uint32_t)0x26000000U)
#define MSPM0_TMR_PWREN_ENABLE     BIT(0)

/* Clock reg mask */
#define MSPM0_TMR_CLKSEL_MASK MSPM0_CLOCK_PERIPH_REG_MASK(MSPM0_TMR_CLOCK)

/* Counter control */
#define MSPM0_TMR_CTRCTL_COUNT_UP_MASK GENMASK(5, 4)
#define MSPM0_TMR_CTRCTL_REPEAT_MASK   GENMASK(3, 1)
#define MSPM0_TMR_CTRCTL_ENABLE_MASK   BIT(0)
#define MSPM0_TMR_CTRCTL_DISABLE_MASK  MSPM0_TMR_CTRCTL_ENABLE_MASK

#define MSPM0_TMR_CTRCTL_COUNT_UP FIELD_PREP(MSPM0_TMR_CTRCTL_COUNT_UP_MASK, 0x2)
#define MSPM0_TMR_CTRCTL_REPEAT   FIELD_PREP(MSPM0_TMR_CTRCTL_REPEAT_MASK, 0x1)
#define MSPM0_TMR_CTRCTL_ENABLE   BIT(0)
#define MSPM0_TMR_CTRCTL_DISABLE  0x0

/* Interrupts (Events) */
#define MSPM0_TMR_CPU_INT_IIDX_COMPARE_UP 0x9
#define MSPM0_TMR_CPU_INT_COMPARE_UP_BIT  BIT(8)
#define MSPM0_TMR_CPU_INT_COMPARE_UP_MASK MSPM0_TMR_CPU_INT_COMPARE_UP_BIT

#define MSPM0_TMR_INTERRUPT_COMPARE_ENABLE  MSPM0_TMR_CPU_INT_COMPARE_UP_BIT
#define MSPM0_TMR_INTERRUPT_COMPARE_DISABLE 0

/* Helper macro around the sys_read and sys_write */
#define MSPM0_TMR_REG_ADDR(reg)       (MSPM0_TMR_BASE + MSPM0_TMR_REG_##reg)
#define MSPM0_TMR_WRITE(reg_off, val) sys_write32(val, MSPM0_TMR_REG_ADDR(reg_off))
#define MSPM0_TMR_READ(reg_off)       sys_read32(MSPM0_TMR_REG_ADDR(reg_off))
#define MSPM0_TMR_UPDATE(reg_off, mask, data)                                                      \
	mspm0_timer_reg_update(MSPM0_TMR_REG_ADDR(reg_off), mask, data)

/* Helper function to read modify write the register*/
static inline void mspm0_timer_reg_update(uint32_t reg, uint32_t mask, uint32_t data)
{
	uint32_t tmp;

	tmp = sys_read32(reg);
	tmp &= ~mask;
	tmp |= (data & mask);
	sys_write32(tmp, reg);
}

static inline void mspm0_compare_irq_enable(void)
{
	MSPM0_TMR_UPDATE(CPU_INT_IMASK, MSPM0_TMR_CPU_INT_COMPARE_UP_MASK,
			 MSPM0_TMR_INTERRUPT_COMPARE_ENABLE);
}

static inline void mspm0_compare_irq_disable(void)
{
	MSPM0_TMR_UPDATE(CPU_INT_IMASK, MSPM0_TMR_CPU_INT_COMPARE_UP_MASK,
			 MSPM0_TMR_INTERRUPT_COMPARE_DISABLE);
}

/*
 * The timer is a 16-bit free-running up-counter using compare channel 0.
 * A compare up interrupt occurs when CTR equals CC_0, so the COMPARE_EXACT
 * backend is used.
 */
#define TIMER_CORE_BACKEND_COMPARE_EXACT
#define TIMER_CORE_COUNTER_WIDTH 16

/*
 * Since we are doing the prescale and clock divide timer core expects
 * to define the prescaled cycles per second
 * #define TIMER_CORE_CYCLES_PER_SEC MSPM0_TMR_CYC_PER_SEC
 * #define TIMER_CORE_CYC_PER_TICK_IS_CONSTANT	idk should i ?!
 */
static uint32_t timer_driver_cycle_get(void)
{
	return MSPM0_TMR_READ(CTR);
}

static void timer_driver_set_compare(uint32_t cycles)
{
	MSPM0_TMR_WRITE(CC_0, cycles);
}

#include "system_timer_generic.h"

static void mspm0_timer_isr(void *arg)
{
	ARG_UNUSED(arg);
	uint32_t pending_irq;

	/* Pending interrupt is cleared on the read to IIDX */
	pending_irq = MSPM0_TMR_READ(CPU_INT_IIDX);
	if (pending_irq != MSPM0_TMR_CPU_INT_IIDX_COMPARE_UP) {
		return;
	}

	k_spinlock_key_t key = sys_clock_lock();

	timer_core_announce_from(key);
}

/* Diable the interrupt and the counter */
void sys_clock_disable(void)
{
	mspm0_compare_irq_disable();
	MSPM0_TMR_UPDATE(CTRCTL, MSPM0_TMR_CTRCTL_DISABLE_MASK, MSPM0_TMR_CTRCTL_DISABLE);
}

void sys_clock_idle_enter(uint32_t ticks)
{
	if (ticks != SYS_CLOCK_IDLE_FOREVER) {
		sys_clock_set_timeout(ticks, false);
		return;
	}
	mspm0_compare_irq_disable();
}

void sys_clock_idle_exit(void)
{
	mspm0_compare_irq_enable();
}

static int mspm0_timer_init(void)
{
	/* Reset and power on the timer */
	MSPM0_TMR_WRITE(RSTCTL, MSPM0_TMR_RSTCTL_UNLOCK_KEY | MSPM0_TMR_RSTCTL_CLEAR_RESETSTKY |
					MSPM0_TMR_RSTCTL_ASSERT_RESET);

	MSPM0_TMR_WRITE(PWREN, MSPM0_TMR_PWREN_UNLOCK_KEY | MSPM0_TMR_PWREN_ENABLE);

	msp_delay_peripheral_startup();

	/* Clock configuration */
	MSPM0_TMR_UPDATE(CLKSEL, MSPM0_TMR_CLKSEL_MASK, MSPM0_TMR_CLOCK);
	MSPM0_TMR_WRITE(CLKDIV, MSPM0_TMR_CLK_DIV);
	MSPM0_TMR_WRITE(CPS, MSPM0_TMR_PRESCALE);

	/* Initialize the counter to its maximum value for free-running operation. */
	MSPM0_TMR_WRITE(LOAD, MSPM0_TMR_CYCLES_MAX);

	/* Start the timer */
	MSPM0_TMR_WRITE(CTRCTL, MSPM0_TMR_CTRCTL_COUNT_UP | MSPM0_TMR_CTRCTL_REPEAT |
					MSPM0_TMR_CTRCTL_ENABLE);

	IRQ_CONNECT(MSPM0_TMR_IRQ_NUM, MSPM0_TMR_IRQ_PRIO, mspm0_timer_isr, 0, 0);

	timer_core_init();

	/* Clear the interrupt */
	MSPM0_TMR_WRITE(CPU_INT_ICLR, MSPM0_TMR_CPU_INT_COMPARE_UP_BIT);

	/* Enable the interrupt */
	mspm0_compare_irq_enable();
	irq_enable(MSPM0_TMR_IRQ_NUM);

	return 0;
}

SYS_INIT(mspm0_timer_init, PRE_KERNEL_2, CONFIG_SYSTEM_CLOCK_INIT_PRIORITY);
