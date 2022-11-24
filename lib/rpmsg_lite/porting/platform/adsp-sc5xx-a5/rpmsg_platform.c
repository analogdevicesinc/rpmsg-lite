/*
 * Copyright 2017-2020 NXP
 * Copyright 2021-2022 Analog Devices Inc.
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include <cycle_count.h>
#include <limits.h>
#include <stdio.h>
#include <string.h>
#include <runtime/int/interrupt.h>
#include <sys/platform.h>
#include <sys/adi_core.h>

#include "adi_initialize.h"
#include "rpmsg_env.h"
#include "rpmsg_platform.h"

#if !defined (__ADI_HAS_TRU__) || !defined(__ADSPCORTEXA5__)
#error unsupported platform
#endif

#include <services/tru/adi_tru.h>
#include <services/int/adi_gic.h>

static int32_t isr_counter     = 0;
static int32_t disable_counter = 0;
static uint32_t coreID;
static uint32_t iid = UINT_MAX;
static void *platform_lock;
#if defined(RL_USE_STATIC_API) && (RL_USE_STATIC_API == 1)
static LOCK_STATIC_CONTEXT platform_lock_static_ctxt;
#endif


static uint32_t cclk = 1000000000; /* Default to 1GHz */
#define _CYCLES_PER_MSEC (cclk / 1000)
#define _CYCLES_PER_USEC (cclk / 1000000)
#if !defined (CLKIN)
/* Default to 25 MHz, which is used for all the EZKITs*/
#define CLKIN 25000000
#endif
/* Inline functions for interrupt control and memory barrier
 */


static inline
void platform_global_isr_enable(uint32_t istate)
{
    __builtin_enable_interrupts();
}

static inline
uint32_t platform_global_isr_disable(void)
{
    __builtin_disable_interrupts();
    return 0u;
}

int32_t platform_init_interrupt(uint32_t vector_id, void *isr_data)
{
    /* Register ISR to environment layer */
    env_register_isr(vector_id, isr_data);

    env_lock_mutex(platform_lock);

    RL_ASSERT(0 <= isr_counter);
    if (isr_counter == 0)
    {
        adi_int_EnableInt(iid, true);
    }
    isr_counter++;

    env_unlock_mutex(platform_lock);

    return 0;
}

int32_t platform_deinit_interrupt(uint32_t vector_id)
{
    env_lock_mutex(platform_lock);

    RL_ASSERT(0 < isr_counter);
    isr_counter--;
    if (isr_counter == 0)
    {
        adi_int_EnableInt(iid, false);
    }

    /* Unregister ISR from environment layer */
    env_unregister_isr(vector_id);

    env_unlock_mutex(platform_lock);

    return 0;
}

void platform_notify(uint32_t vector_id)
{
    int32_t trigger;

    env_lock_mutex(platform_lock);

    /* TRGM_SOFT3 SLV3  ARM */
    /* TRGM_SOFT4 SLV7  SHARC1 */
    /* TRGM_SOFT5 SLV11 SHARC2 */

    switch(coreID){
    case ADI_CORE_ARM:
        trigger = TRGM_SOFT4 + RL_GET_LINK_ID(vector_id); // TRGM_SOFT4 or TRGM_SOFT5
        break;
    case ADI_CORE_SHARC0:
        trigger = TRGM_SOFT3 + 2*RL_GET_LINK_ID(vector_id); // TRGM_SOFT3 or TRGM_SOFT5
        break;
    case ADI_CORE_SHARC1:
        trigger = TRGM_SOFT3 + RL_GET_LINK_ID(vector_id); // TRGM_SOFT3 or TRGM_SOFT4
        break;
    default:
        // should never happen
        break;
    }

    adi_tru_RaiseTriggerMaster(trigger);

    env_unlock_mutex(platform_lock);
}

/**
 * platform_time_delay
 *
 * @param num_msec Delay time in ms.
 *
 * This is not an accurate delay, it ensures at least num_msec passed when return.
 */


void platform_time_delay(uint32_t num_msec)
{
    cycle_t start, elapsed, delay_cycles;
    start = CCNTR_READ;
    delay_cycles = (num_msec * _CYCLES_PER_MSEC);

    do {
        elapsed = CCNTR_READ - start;
    }while(elapsed < delay_cycles);
}

uint32_t platform_us_clock_tick(void){
	unsigned long long tick = CCNTR_READ / _CYCLES_PER_USEC;
	return (uint32_t)tick;
}

/**
 * platform_in_isr
 *
 * Return whether CPU is processing IRQ
 *
 * @return True for IRQ, false otherwise.
 *
 */
int32_t platform_in_isr(void)
{
    uint32_t local_cpsr;
    uint32_t mode;
    asm ("MRS %0, CPSR" :"=r"(local_cpsr): :);

    mode = (local_cpsr & ADI_RTL_ARM_MODE_MASK);

    if((mode == ADI_RTL_ARM_MODE_USR) || (mode == ADI_RTL_ARM_MODE_SYS) || (mode == ADI_RTL_ARM_MODE_SVC) )
        return false;
    else
        return true;
}

/**
 * platform_interrupt_enable
 *
 * Enable peripheral-related interrupt
 *
 * @param vector_id Virtual vector ID that needs to be converted to IRQ number
 *
 * @return vector_id Return value is never checked.
 *
 */
int32_t platform_interrupt_enable(uint32_t vector_id)
{
    uint32_t imask;
    RL_ASSERT(0 < disable_counter);

    imask = platform_global_isr_disable();
    disable_counter--;

    /* The iid interrupt is shared by both ARM and SHARC VirtIOs, enable if both wants to enable */
    if (disable_counter == 0)
    {
        adi_int_EnableInt(iid, true);
    }
    platform_global_isr_enable(imask);
    return ((int32_t)vector_id);
}

/**
 * platform_interrupt_disable
 *
 * Disable peripheral-related interrupt.
 *
 * @param vector_id Virtual vector ID that needs to be converted to IRQ number
 *
 * @return vector_id Return value is never checked.
 *
 */
int32_t platform_interrupt_disable(uint32_t vector_id)
{
    uint32_t imask;
    RL_ASSERT(0 <= disable_counter);

    imask = platform_global_isr_disable();
    /* The iid interrupt is shared by both ARM and SHARC VirtIOs, disable if one wants to disable */
    if (disable_counter == 0)
    {
        adi_int_EnableInt(iid, false);
    }
    disable_counter++;
    platform_global_isr_enable(imask);
    return ((int32_t)vector_id);
}

/**
 * platform_map_mem_region
 *
 * Dummy implementation
 *
 */
void platform_map_mem_region(uint32_t vrt_addr, uint32_t phy_addr, uint32_t size, uint32_t flags)
{
}

/**
 * platform_cache_all_flush_invalidate
 *
 * Dummy implementation
 *
 */
void platform_cache_all_flush_invalidate(void)
{
}

/**
 * platform_cache_disable
 *
 * Dummy implementation
 *
 */
void platform_cache_disable(void)
{
}

/**
 * platform_vatopa
 *
 * Dummy implementation
 *
 */
uint32_t platform_vatopa(void *addr)
{
    return ((uint32_t)(char *)addr);
}

/**
 * platform_patova
 *
 * Dummy implementation
 *
 */
void *platform_patova(uint32_t addr)
{
    return ((void *)(char *)addr);
}

static void iccInterruptHandler(uint32_t iid, void *handlerArg){
    env_isr(0);
    env_isr(1);
    env_isr(2);
    env_isr(3);
}

/**
 * platform_init
 *
 * platform/environment init
 */
int32_t platform_init(void)
{
    /*
    * Retrieve the CCLK for the A5.  See CDU Clock Configuration options in the 
    * Hardware Reference Manual
    */
    uint32_t cdu_sel = *pREG_CDU0_CFG2 & BITM_CDU_CFG_SEL;
    uint32_t cgu_mul;
    uint32_t cgu_div;

    switch (cdu_sel) {
    case ENUM_CDU_CFG_IN0:
        cgu_mul = (*pREG_CGU0_CTL & BITM_CGU_CTL_MSEL) >> BITP_CGU_CTL_MSEL;
        cgu_div = (*pREG_CGU0_DIV & BITM_CGU_DIV_CSEL) >> BITP_CGU_DIV_CSEL;
    	cclk = CLKIN * cgu_mul / cgu_div;
        break;
    case ENUM_CDU_CFG_IN1:
        /*SC58x and SC57x only*/
        cgu_mul = (*pREG_CGU0_CTL & BITM_CGU_CTL_MSEL) >> BITP_CGU_CTL_MSEL;
        cgu_div = (*pREG_CGU0_DIV & BITM_CGU_DIV_SYSSEL) >> BITP_CGU_DIV_SYSSEL;
    	cclk = CLKIN * cgu_mul / cgu_div;
        break;
    default:
        return -1;
    }

    coreID = adi_core_id();

    /*
     * Platform-specific interrupt initialization.
     * We'll be raising SEC/GIC interrupts via the Trigger Routing Unit (ADSP-SC58x/ADSP-215xx)
     */
    iid = (uint32_t)INTR_TRU0_INT3;

    /* Set up the Trigger Routing Unit to route the soft triggers 3,
     * 4, and 5 to the ARM, SHARC1, and SHARC2 cores respectively.
     * We do this on each node, even though only the first occurrence
     * is necessary, to ensure that the triggers are ready to raise
     * interrupts before any messages are sent.
     */
    adi_tru_Init(false); /* initialize the TRU, without reset */

    //ICC interrupts
    adi_tru_ConfigureSlave(TRGS_TRU0_IRQ3,  TRGM_SOFT3); /* SLV3  ARM */
    adi_tru_ConfigureSlave(TRGS_TRU0_IRQ7,  TRGM_SOFT4); /* SLV7  SHARC1 */
#if __NUM_SHARC_CORES__ > 1
    adi_tru_ConfigureSlave(TRGS_TRU0_IRQ11, TRGM_SOFT5); /* SLV11 SHARC2 */
#endif
    /* Set the interrupt to edge-sensitive for use with the TRU (default is level-sensitive) */
    adi_gic_ConfigInt(iid, ADI_GIC_INT_EDGE_SENSITIVE, ADI_GIC_INT_HANDLING_MODEL_1_N);

    /* Install the ICC interrupt handler.
     * One handler serves all connections into and out of this node (core).
     */
    if (ADI_INT_SUCCESS != adi_int_InstallHandler(iid, &iccInterruptHandler, NULL, false))
    {
        return -1;
    }

    /* Create lock used in multi-instanced RPMsg */
#if defined(RL_USE_STATIC_API) && (RL_USE_STATIC_API == 1)
    if (0 != env_create_mutex(&platform_lock, 1, &platform_lock_static_ctxt))
#else
    if (0 != env_create_mutex(&platform_lock, 1))
#endif
    {
        return -1;
    }

    // if (adi_pwr_GetCoreClkFreq(coreID, &cclk) == ADI_PWR_DEVICE_NOTINITIALIZED) {
    // 	if (adi_pwr_Init(0, CLKIN) != ADI_PWR_SUCCESS) {
    // 		return -1;
    // 	}
    // 	if (adi_pwr_GetCoreClkFreq(coreID, &cclk) == ADI_PWR_DEVICE_NOTINITIALIZED) {
    // 		return -1;
    // 	}
    // }

    /* Turn on core clock used for delay and tick counting */
    CCNTR_INIT;
    CCNTR_START;

    return 0;
}

/**
 * platform_deinit
 *
 * platform/environment deinit process
 */
int32_t platform_deinit(void)
{
    /* Delete lock used in multi-instanced RPMsg */
    env_delete_mutex(platform_lock);
    platform_lock = ((void *)0);
    return 0;
}
