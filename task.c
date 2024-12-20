/*
 * Copyright (C) 2023 Jan Hamal Dvořák <mordae@anilinux.org>
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include "task.h"

#include <pico/lock_core.h>

#include <hardware/irq.h>
#include <hardware/dma.h>
#include <hardware/structs/bus_ctrl.h>

#include <string.h>
#include <stdlib.h>
#include <limits.h>
#include <stdio.h>

#define HUNG_TIMEOUT 1000000

/* Meaning of status codes exchanged by tasks via task_swap_context. */
enum task_status {
	TASK_RUN = 0,	 /* Task is being resumed. */
	TASK_YIELD = 1,	 /* Task has yielded and can be resumed. */
	TASK_RETURN = 2, /* Task has exited and must be restarted. */
};

/*
 * Swap running task by storing and replacing register values.
 * First argument is used to pass status messages between the tasks.
 */
enum task_status task_swap_context(enum task_status, unsigned load[TASK_NUM_REGS],
				   unsigned save[TASK_NUM_REGS]);

/* Saved register offsets. */
enum reg_offset { SP = 0, R4, R5, R6, R7, R8, R9, R10, R11, R12, LR };

/* Offset to start selecting tasks from. */
static unsigned task_offset[NUM_CORES] = { 0 };

/* Saved scheduler registers for respective cores: */
static unsigned task_return[NUM_CORES][TASK_NUM_REGS];

/* Spinlock notifications for respective cores: */
static uint64_t lock_notify[NUM_CORES][NUM_SPIN_LOCKS] = { 0 };

/* DMA IRQ notifications: */
static uint64_t dma_notify[NUM_DMA_CHANNELS] = { 0 };

/* Microseconds since last per-core stats reset. */
static uint64_t last_reset[NUM_CORES] = { 0 };

/* Memory performance counters. */
static uint64_t perf_bus_ctr[20] = { 0 };

/* Currently running tasks for respective cores: */
task_t task_running[NUM_CORES] = { NULL };

/*
 * Start task in the R4 register and convert its return into a swap back
 * to the scheduler. Otherwise a task returning would be undefined and
 * probably catastrophic.
 */
static void task_sentinel(void)
{
	void (*fn)(void);
	asm volatile("mov %0, r4" : "=r"(fn));
	fn();

	unsigned core = get_core_num();
	task_t task = task_running[core];
	task_swap_context(TASK_RETURN, task_return[core], task->regs);
}

static task_t __time_critical_func(task_select)(void)
{
	unsigned core = get_core_num();
	task_t best_task = NULL;
	int min_pri = INT_MIN;

	for (size_t i = 0; i < MAX_TASKS; i++) {
		size_t tid = (task_offset[core] + i) % MAX_TASKS;
		task_t task = task_avail[core][tid];

		if (NULL == task)
			continue;

		if (TASK_WAITING_FOR_LOCK == task->state) {
			for (unsigned c = 0; c < NUM_CORES; c++) {
				if (lock_notify[c][task->awaitable] >= task->resumed_at) {
					task->state = TASK_READY;
					task->awaitable = -1;
					break;
				}
			}
		}

		if (TASK_WAITING_FOR_DMA == task->state) {
			if (!dma_channel_is_busy(task->awaitable) ||
			    (dma_notify[task->awaitable] >= task->resumed_at)) {
				task->state = TASK_READY;
				task->awaitable = -1;
				break;
			}
		}

		if (TASK_READY != task->state)
			continue;

		if (task->priority > min_pri) {
			min_pri = task->priority;
			best_task = task;
		}
	}

	task_offset[core] = (task_offset[core] + 1) % MAX_TASKS;

	return best_task;
}

static int64_t task_hung_alarm(__unused alarm_id_t alarm, __unused void *arg)
{
	for (unsigned i = 0; i < NUM_CORES; i++) {
		task_t task = task_running[i];

		if (NULL == task)
			continue;

		uint64_t running = time_us_64() - task->resumed_at;

		if (running > HUNG_TIMEOUT) {
			printf("task: hung task detected: %s (%us)\n", task->name,
			       (unsigned)(running / 1000000));
		}
	}

	return -HUNG_TIMEOUT;
}

static int64_t task_perf_sample(__unused alarm_id_t alarm, __unused void *arg)
{
	/* Determine currently active counters. */
	int ctr = bus_ctrl_hw->counter[0].sel;
	int next = ctr + 4;

	if (next >= 20)
		next = 0;

	for (int i = 0; i < 4; i++) {
		/* Add the values to 64b accumulators. */
		perf_bus_ctr[ctr + i] += bus_ctrl_hw->counter[i].value;

		/* Change to next counter in sequence. */
		bus_ctrl_hw->counter[i].sel = next + i;

		/* Clear the counter. */
		bus_ctrl_hw->counter[i].value = 0;
	}

	/* Repeat every 200ms. So one sample from each every second. */
	return -200 * 1000;
}

static void __isr __time_critical_func(dma_irq_0)(void)
{
	irq_clear(DMA_IRQ_0);

	uint64_t now = time_us_64();

	for (unsigned i = 0; i < NUM_DMA_CHANNELS; i++)
		if (dma_hw->ints0 & (1u << i))
			dma_notify[i] = now;

	dma_hw->ints0 = DMA_INTS0_BITS;
	__sev();
}

void task_init(void)
{
	/*
	 * Make sure that the default alarm pool is ready. We use it for
	 * task_sleep_{ms,us} functions and for the hung task detector.
	 */
	alarm_pool_init_default();

	/* Enable IRQ0 for all channels. */
	for (unsigned i = 0; i < NUM_DMA_CHANNELS; i++)
		dma_channel_set_irq0_enabled(i, true);

	/* We are handling DMA IRQ 0. */
	irq_set_exclusive_handler(DMA_IRQ_0, dma_irq_0);
	irq_set_enabled(DMA_IRQ_0, true);

	/* Initialize the defined tasks. */
	for (unsigned i = 0; i < NUM_CORES; i++) {
		for (unsigned t = 0; /**/; t++) {
			task_t task = task_avail[i][t];

			if (NULL == task)
				break;

			/* Install stack canary values. */
			task->canary_top = 0xdeadbeef;
			task->canary_bottom = 0xdeadbeef;

			/* Mark the whole stack as unused. */
			for (int j = 0; j < TASK_STACK_SIZE / 4; j++)
				task->stack[j] = 0xdeadbeef;

			/* Prepare to run the task procedure. */
			task->regs[SP] = (unsigned)&task->canary_top;
			task->regs[LR] = (unsigned)task_sentinel;
			task->regs[R4] = (unsigned)task->proc;
		}

		/* Make sure we start without any active tasks. */
		task_running[i] = NULL;
	}

	/* Start hung task detector. */
	(void)add_alarm_in_us(HUNG_TIMEOUT, task_hung_alarm, NULL, true);

	/* Clear perf counters. */
	bus_ctrl_hw->counter[0].sel = 0;
	bus_ctrl_hw->counter[1].sel = 1;
	bus_ctrl_hw->counter[2].sel = 2;
	bus_ctrl_hw->counter[3].sel = 3;

	bus_ctrl_hw->counter[0].value = 0;
	bus_ctrl_hw->counter[1].value = 0;
	bus_ctrl_hw->counter[2].value = 0;
	bus_ctrl_hw->counter[3].value = 0;

	/* Start memory performance counter servicing. */
	(void)add_alarm_in_us(10000, task_perf_sample, NULL, true);
}

bool __time_critical_func(task_run)(void)
{
	unsigned core = get_core_num();
	task_t task = task_select();

	if (NULL == task)
		return false;

	task_running[core] = task;
	task->resume_count++;
	task->resumed_at = time_us_64();

	enum task_status status;
	status = task_swap_context(TASK_RUN, task->regs, task_return[core]);

	task->runtime_us += time_us_64() - task->resumed_at;
	task_running[core] = NULL;

	if (0xdeadbeef != task->canary_top)
		panic("task [%s]: stack underflow", task->name);

	if (0xdeadbeef != task->canary_bottom)
		panic("task [%s]: stack overflow", task->name);

	if (TASK_YIELD == status)
		return true;

	if (TASK_RETURN == status) {
		/* Restart the task: */
		task->regs[SP] = (unsigned)&task->canary_top;
		task->regs[LR] = (unsigned)task_sentinel;
		task->regs[R4] = (unsigned)task->proc;
		return true;
	}

	panic("task: invalid status (%i)", status);
}

void __attribute__((__noreturn__)) __time_critical_func(task_run_loop)(void)
{
	while (true) {
		/* Work until we run out of ready tasks. */
		while (task_run())
			;

		/*
		 * Sleep until an interrupt or event happens.
		 *
		 * If an event arrived since the last WFE, it does not sleep.
		 * That way the potential race condition is mitigated.
		 */
		__wfe();
	}
}

void task_yield(void)
{
	unsigned core = get_core_num();
	task_t task = task_running[core];

	if (NULL == task) {
		__sev();
		return;
	}

	task_swap_context(TASK_YIELD, task_return[core], task->regs);
}

static int64_t task_ready_alarm(__unused alarm_id_t alarm, __unused void *arg)
{
	task_t task = arg;

	if (TASK_WAITING_FOR_ALARM == task->state) {
		/* Mark the task as ready. */
		task->state = TASK_READY;

		/* Unblock the scheduler. */
		__sev();
	}

	return 0;
}

void task_sleep_us(uint64_t us)
{
	unsigned core = get_core_num();
	task_t task = task_running[core];

	if (NULL == task) {
		sleep_us(us);
		return;
	}

	task->state = TASK_WAITING_FOR_ALARM;
	(void)add_alarm_in_us(us, task_ready_alarm, task, true);
	task_yield();
}

void task_sleep_ms(uint64_t ms)
{
	unsigned core = get_core_num();
	task_t task = task_running[core];

	if (NULL == task) {
		sleep_ms(ms);
		return;
	}

	task->state = TASK_WAITING_FOR_ALARM;
	(void)add_alarm_in_us(1000 * ms, task_ready_alarm, task, true);
	task_yield();
}

void task_yield_until(uint64_t us)
{
	unsigned core = get_core_num();
	task_t task = task_running[core];

	if (NULL == task) {
		uint64_t now = time_us_64();

		if (now < us)
			sleep_us(us - now);

		return;
	}

	task->state = TASK_WAITING_FOR_ALARM;
	(void)add_alarm_at(from_us_since_boot(us), task_ready_alarm, task, true);
	task_yield();
}

void task_wait_for_dma(uint8_t dma_ch_id)
{
	unsigned core = get_core_num();
	task_t task = task_running[core];

	if (NULL == task)
		panic("task_wait_for_dma called outside a task");

	task->awaitable = dma_ch_id;
	task->state = TASK_WAITING_FOR_DMA;
	task_yield();
}

static uint32_t stack_free_space(task_t task)
{
	uint32_t level = 0;

	for (size_t i = 0; i < TASK_STACK_SIZE / 4; i++) {
		if (0xdeadbeef == task->stack[i]) {
			level += 4;
		} else {
			break;
		}
	}

	return level;
}

void task_stats_report_reset(unsigned core)
{
	uint64_t total_us = time_us_64() - last_reset[core];

	if (!total_us)
		total_us = 1;

	printf("task: core %u:\n", core);

	for (int i = 0; i < MAX_TASKS; i++) {
		task_t task = task_avail[core][i];

		if (NULL == task)
			break;

		unsigned stack = stack_free_space(task);

		char flags[] = "?";

		int awaitable = task->awaitable;
		enum task_state state = task->state;

		if (TASK_READY == state)
			flags[0] = 'r';
		else if (TASK_WAITING_FOR_ALARM == state)
			flags[0] = 'A';
		else if (TASK_WAITING_FOR_LOCK == state)
			flags[0] = 'L';
		else if (TASK_WAITING_FOR_DMA == state)
			flags[0] = 'D';

		printf("task: %2i (%-2i ", i, task->priority);

		if (TASK_WAITING_FOR_LOCK == state) {
			printf("%s=%-2i", flags, awaitable);
		} else if (TASK_WAITING_FOR_DMA == state) {
			printf("%s=%-2i", flags, awaitable);
		} else {
			printf("%s   ", flags);
		}

		printf(") [%-11s]", task->name);
		printf(" [%4u] %8.2fx ", stack, 1000000.0 * task->resume_count / total_us);
		printf("=> %4llu ms/s\n", 1000llu * task->runtime_us / total_us);
	}

	task_stats_reset(core);
}

void task_stats_reset(unsigned core)
{
	for (unsigned t = 0; t < MAX_TASKS; t++) {
		task_t task = task_avail[core][t];

		if (NULL == task)
			break;

		task->resume_count = 0;
		task->runtime_us = 0;
	}

	last_reset[core] = time_us_64();
}

void task_stats_memory_reset(void)
{
	static const char *labels[] = { "apb", "per", "sry", "srx", "sr3",
					"sr2", "sr1", "sr0", "xip", "rom" };

	printf("mem:");

	for (int i = 0; i < 20; i += 2) {
		float contested = perf_bus_ctr[i];
		float total = perf_bus_ctr[i + 1];

		perf_bus_ctr[i] = 0;
		perf_bus_ctr[i + 1] = 0;

		float ratio = 100.0f * contested / total;

		printf(" %s=%05.02f", labels[i >> 1], ratio);

		if (8 == i)
			printf("\nmem:");
	}

	printf("\n");
}

/****************************************************************************
 * Compatibility layer to use tasks with blocking SDK functions.            *
 ****************************************************************************/

void task_lock_spin_unlock_with_notify(struct lock_core *lc, uint32_t save)
{
	/* Unlock the lock. */
	spin_unlock(lc->spin_lock, save);

	/* Notify the schedulers. */
	uint32_t lock_id = ((uint32_t)lc->spin_lock >> 2) & 0x1f;
	lock_notify[get_core_num()][lock_id] = time_us_64();

	/* Unblock the scheduler on the other core. */
	__sev();
}

void task_lock_spin_unlock_with_wait(struct lock_core *lc, uint32_t save)
{
	unsigned core = get_core_num();
	task_t task = task_running[core];

	if (task) {
		spin_unlock(lc->spin_lock, save);
		task->state = TASK_WAITING_FOR_LOCK;
		task->awaitable = ((uint32_t)lc->spin_lock >> 2) & 0x1f;
		task_yield();
	} else {
		spin_unlock(lc->spin_lock, save);
		__wfe();
	}
}

int task_lock_spin_unlock_with_timeout(struct lock_core *lc, uint32_t save, absolute_time_t time)
{
	unsigned core = get_core_num();
	task_t task = task_running[core];

	if (task) {
		spin_unlock(lc->spin_lock, save);
		task->state = TASK_WAITING_FOR_LOCK;
		task->awaitable = ((uint32_t)lc->spin_lock >> 2) & 0x1f;
		task_yield();
		return time_us_64() >= to_us_since_boot(time);
	} else {
		spin_unlock(lc->spin_lock, save);
		return best_effort_wfe_or_timeout(time);
	}
}

void task_sync_yield_until_before(absolute_time_t time)
{
	task_yield_until(to_us_since_boot(time));
}
