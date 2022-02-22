/*
 * Copyright 2017-2020 NXP
 * Copyright 2021 Analog Devices Inc.
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include <stdio.h>
#include <string.h>
#include <limits.h>
#include "rpmsg_platform.h"
#include "rpmsg_env.h"

#include "adi_initialize.h"
#include <sys/platform.h>
#include <sys/adi_core.h>
#include <sys/cache.h>

#include <cycle_count.h>

#if !defined (__ADI_HAS_TRU__)
#error unsupported platform
#endif

/* TRU is available to both ARM and SHARC cores */
#include <services/tru/adi_tru.h>

#if defined(__ADSPARM__)
#define HAS_GIC /* GIC is only available to the ARM core */
#include <services/int/adi_gic.h>
#elif defined(__ADSPSHARC__)
#define HAS_SEC /* SEC is only available to the SHARC cores */
#include <services/int/adi_sec.h>
#else
#error unsupported platform, ARM/SHARC are not defined
#endif /* defined(__ADSPARM__) */

#define _CYCLES_PER_MSEC (__PROCESSOR_SPEED__ / 1000)
#define _CYCLES_PER_USEC (__PROCESSOR_SPEED__ / 1000000)

static int32_t isr_counter     = 0;
static int32_t disable_counter = 0;
static uint32_t coreID;
static uint32_t iid = UINT_MAX;
static void *platform_lock;
#if defined(RL_USE_STATIC_API) && (RL_USE_STATIC_API == 1)
static LOCK_STATIC_CONTEXT platform_lock_static_ctxt;
#endif

/* Inline functions for interrupt control and memory barrier
 */

#if defined (__GNUC__)
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
#elif defined(__ADSPSHARC__)
#pragma inline
void platform_global_isr_enable(uint32_t istate)
{
    /* Globally (conditionally) re-enable interrupts */
    if (istate != 0u)
    {
        asm volatile("bit SET MODE1 0x1000;"); /* set GIE */
    }
}

#pragma inline
uint32_t platform_global_isr_disable(void)
{
    uint32_t mask = (uint32_t)__builtin_sysreg_bit_tst(sysreg_MODE1, 0x1000u /*IRPTEN*/);

    /* in VISA the length of the instructions can be different so we can no
       longer do a jump (PC,3). But we cannot have a straight label to avoid
       multiple definitions so we automatically generate them */
    asm volatile(" #define atomic_disable JUMP (PC,.SH_INT_DISABLED?) (DB); \
             bit clr MODE1 0x1000; /* clear GIE*/ \
             nop; \
             .SH_INT_DISABLED?: "
    "\natomic_disable");

    return mask;
}
#else
#error unsupported platform
#endif

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
    start = __builtin_emuclk();
    delay_cycles = (num_msec * _CYCLES_PER_MSEC);

    do {
        elapsed = __builtin_emuclk() - start;
    }while(elapsed < delay_cycles);
}

uint32_t platform_us_clock_tick(void){
	unsigned long long tick = __builtin_emuclk() / _CYCLES_PER_USEC;
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
    return (sysreg_read(sysreg_IMASKP)) != 0u;
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

    coreID = adi_core_id();

    /*
     * Platform-specific interrupt initialization.
     * We'll be raising SEC/GIC interrupts via the Trigger Routing Unit (ADSP-SC58x/ADSP-215xx)
     */
    switch (coreID)
    {
    case ADI_CORE_ARM:
        iid = (uint32_t)INTR_TRU0_INT3;
        break;

    case ADI_CORE_SHARC0:
        iid = (uint32_t)INTR_TRU0_INT7;
        break;

    case ADI_CORE_SHARC1:
        iid = (uint32_t)INTR_TRU0_INT11;
        break;
    default:
        return -1; // should never happen
    }

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
    adi_tru_ConfigureSlave(TRGS_TRU0_IRQ11, TRGM_SOFT5); /* SLV11 SHARC2 */

    /* Set the interrupt to edge-sensitive for use with the TRU (default is level-sensitive) */
    #if defined(HAS_GIC)
        adi_gic_ConfigInt(iid, ADI_GIC_INT_EDGE_SENSITIVE, ADI_GIC_INT_HANDLING_MODEL_1_N);
    #elif defined(HAS_SEC)
        adi_sec_EnableEdgeSense(iid, true);
    #endif


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
