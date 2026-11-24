/*
 * Copyright 2017-2020 NXP
 * Copyright 2021-2026 Analog Devices Inc.
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include <stdio.h>
#include <string.h>
#include <limits.h>

#include "adi_initialize.h"
#include <sys/platform.h>
#include <sys/adi_core.h>
#include <sys/cache.h>
#include <cycle_count.h>
#include <services/int/adi_int.h>
#include <services/pwr/adi_pwr.h>
#include <services/tru/adi_tru.h>
#include <xtensa/hal.h>
#include <xtensa/corebits.h>
#include <xtensa/tie/xt_core.h>

#include "rpmsg_platform.h"
#include "rpmsg_env.h"

#ifndef RL_PLATFORM_CLKIN_HZ
#define RL_PLATFORM_CLKIN_HZ (25000000U)
#endif

#if defined(__ADSPSC846_FAMILY__)
#define RL_ADI_RX_TRU_GEN ((uint32_t)TRGM_SYS_SOFT4_GEN)
#define RL_ADI_RX_TRU_RCV ((uint32_t)TRGS_TRU0_RCV7)
#define RL_ADI_RX_IID     ((uint32_t)INTR_TRU0_RCV7)
#define RL_ADI_TX_TRU_GEN ((uint32_t)TRGM_SYS_SOFT3_GEN)
#define RL_ADI_TX_TRU_RCV ((uint32_t)TRGM_TRU0_RCV3)
#else
#error "No TRU routes defined for this processor"
#endif

static int32_t isr_counter     = 0;
static int32_t disable_counter = 0;
static uint32_t coreID;
static uint32_t iid = UINT_MAX;
static void *platform_lock;
#if defined(RL_USE_STATIC_API) && (RL_USE_STATIC_API == 1)
static LOCK_STATIC_CONTEXT platform_lock_static_ctxt;
#endif

static inline uint32_t platform_global_isr_disable(void)
{
	return xthal_disable_interrupts();
}

static inline void platfrom_global_isr_enable(uint32_t state)
{
	xthal_restore_interrupts(state);
}

int32_t platform_init_interrupt(uint32_t vector_id, void *isr_data)
{
    /* Register ISR to environment layer */
    env_register_isr(vector_id, isr_data);

    env_lock_mutex(platform_lock);

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
    while (num_msec-- > 0U)
	adi_delay_time(1000000U);
}

//uint32_t platform_us_clock_tick(void){
	//unsigned long long tick = __builtin_emuclk() / _CYCLES_PER_USEC;
//	return (uint32_t)tick;
//}

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
    uint32_t stack = (uint32_t)(XT_RSR_PS() & PS_STACK_MASK);
    return ((stack == PS_STACK_INTERRUPT) || (stack == PS_STACK_CROSS)) ? 1 : 0;
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
    return 0;
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
    return 0;
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

#if defined(RL_USE_DCACHE) && (RL_USE_DCACHE == 1)
#if defined(__ADSPSHARC__)
/*
 * SHARC+ L1 data cache maintenance by address range. flush_data_buffer() works on whole
 * 64-byte cache lines and takes the address of the last byte of the range. SHARC+ has no
 * invalidate-only range operation, so invalidate is implemented as write-back + invalidate;
 * a region passed to platform_cache_invalidate() must therefore not hold locally dirty lines.
 */
static void platform_cache_range(void *data, uint32_t len, int invalidate)
{
    if (len != 0U)
    {
        flush_data_buffer(data, (void *)((char *)data + (len - 1U)), invalidate);
    }
}

/**
 * platform_cache_flush
 *
 * Write back a range of shared memory from the L1 data cache.
 *
 */
void platform_cache_flush(void *data, uint32_t len)
{
    platform_cache_range(data, len, ADI_FLUSH_DATA_NOINV);
}

/**
 * platform_cache_invalidate
 *
 * Invalidate a range of shared memory in the L1 data cache.
 *
 */
void platform_cache_invalidate(void *data, uint32_t len)
{
    platform_cache_range(data, len, ADI_FLUSH_DATA_INV);
}
#else
#error "RL_USE_DCACHE == 1 is not supported on the ARM core of this port; keep the shared memory uncached"
#endif /* defined(__ADSPSHARC__) */
#endif /* defined(RL_USE_DCACHE) && (RL_USE_DCACHE == 1) */

/**
 * platform_vatopa
 *
 * Dummy implementation
 *
 */
uintptr_t platform_vatopa(void *addr)
{
    return ((uintptr_t)(char *)addr);
}

/**
 * platform_patova
 *
 * Dummy implementation
 *
 */
void *platform_patova(uintptr_t addr)
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

	ADI_PWR_RESULT pwr = adi_pwr_Init(0U, RL_PLATFORM_CLKIN_HZ);
	if ((pwr != ADI_PWR_SUCCESS) && (pwr != ADI_PWR_DEVICE_IN_USE)) {
		return -1;
	}

    //iid = (uint32_t)INTR_TRU0_INT7;


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
