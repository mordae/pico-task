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

#include <string.h>
#include <stdlib.h>
#include <limits.h>
#include <stdio.h>

#define HUNG_TIMEOUT 1000000


enum {
	R4 = 0,
	R5,
	R6,
	R7,
	R8,
	R9,
	R10,
	FP,
	SP,
	PC,
};


enum task_status {
	TASK_SAVE = 0,
	TASK_YIELD,
	TASK_RETURN,
};


/* Saved scheduler context for respective cores: */
static jmp_buf task_return[NUM_CORES];

/* Spinlock notifications for respective cores: */
static uint64_t lock_notify[NUM_CORES][32] = {0};

/* Microseconds since last per-core stats reset. */
static uint64_t last_reset[NUM_CORES] = {0};

/* Currently running tasks for respective cores: */
task_t task_running[NUM_CORES] = {NULL};

/* Spinlock to protect private data from concurrent access. */
static spin_lock_t *priv_lock = NULL;


/*
 * Starts task in the R4 register and converts its return
 * into longjmp back into the scheduler.
 */
static void task_sentinel(void)
{
	void (*fn)(void);
	asm volatile ("mov %0, r4" : "=r"(fn));
	fn();
	longjmp(task_return[get_core_num()], TASK_RETURN);
}


static int task_select(uint64_t since)
{
	static size_t offset[NUM_CORES] = {0};

	unsigned core = get_core_num();
	int best_task_id = -1;
	int min_pri = INT_MIN;

	for (size_t i = 0; i < MAX_TASKS; i++) {
		size_t tid = (offset[core] + i) % MAX_TASKS;
		task_t task = task_avail[core][tid];

		if (NULL == task)
			continue;

		if (task->lock) {
			uint32_t lock_id = ((uint32_t)task->lock >> 2) & 0x1f;

			for (unsigned c = 0; c < NUM_CORES; c++) {
				if (lock_notify[c][lock_id] >= since) {
					task->lock = NULL;
					task->state = TASK_READY;
					break;
				}
			}
		}

		if (TASK_READY != task->state)
			continue;

		if (task->priority > min_pri) {
			min_pri = task->priority;
			best_task_id = tid;
		}
	}

	if (best_task_id >= 0)
		offset[core] = (offset[core] + 1) % MAX_TASKS;

	return best_task_id;
}


static int64_t task_hung_alarm(alarm_id_t, void *)
{
	for (unsigned i = 0; i < NUM_CORES; i++) {
		task_t task = task_running[i];

		if (!task)
			continue;

		uint64_t running = time_us_64() - task->resumed_at;

		if (running > HUNG_TIMEOUT) {
			printf("task: hung task detected: %s (%lus)\n",
			       task->name, (uint32_t)(running / 1000000));
		}
	}

	return -HUNG_TIMEOUT;
}


void task_init(void)
{
	/*
	 * Make sure that the default alarm pool is ready. We use it for
	 * task_sleep_{ms,us} functions and for the hung task detector.
	 */
	alarm_pool_init_default();

	/* Allocate a spin lock to protect our data structures. */
	priv_lock = spin_lock_init(spin_lock_claim_unused(true));

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
			task->regs[PC] = (unsigned)task_sentinel;
			task->regs[R4] = (unsigned)task->proc;
		}

		/* Make sure we start without any active tasks. */
		task_running[i] = NULL;
	}

	/* Start hung task detector. */
	(void)add_alarm_in_us(HUNG_TIMEOUT, task_hung_alarm, NULL, true);
}


bool task_run(uint64_t since)
{
	volatile unsigned core = get_core_num();

	__dmb();

	volatile int task_no = task_select(since);

	if (task_no < 0)
		return false;

	task_t task = task_avail[core][task_no];

	enum task_status status;
	status = setjmp(task_return[core]);

	if (TASK_SAVE == status) {
		task->resume_count++;
		task->resumed_at = time_us_64();
		task_running[core] = task;
		longjmp(task->regs, (int)task);
	}

	since = task->resumed_at;
	task->runtime_us += time_us_64() - since;
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
		task->regs[PC] = (unsigned)task_sentinel;
		task->regs[R4] = (unsigned)task->proc;
		return true;
	}

	panic("invalid setjmp status (%i)", status);
}


void __attribute__((__noreturn__)) task_run_loop(void)
{
	uint32_t now = 0;
	uint32_t prev = 0;

	while (true) {
		/* Work until we run out of ready tasks. */
		while (true) {
			now = time_us_64();
			task_run(prev);
			prev = now;
		}

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

	if (NULL == task_running[core]) {
		__sev();
		return;
	}

	if (0 == setjmp(task_running[core]->regs)) {
		longjmp(task_return[core], TASK_YIELD);
	}
}


static int64_t task_ready_alarm(alarm_id_t, void *arg)
{
	task_t task = arg;

	if (TASK_WAITING_FOR_ALARM == task->state)
		task->state = TASK_READY;

	return 0;
}


void task_sleep_us(uint64_t us)
{
	unsigned core = get_core_num();

	if (NULL == task_running[core]) {
		sleep_us(us);
		return;
	}

	task_t task = task_running[core];
	task_running[core]->state = TASK_WAITING_FOR_ALARM;
	(void)add_alarm_in_us(us, task_ready_alarm, task, true);
	task_yield();
}


void task_sleep_ms(uint64_t ms)
{
	unsigned core = get_core_num();

	if (NULL == task_running[core]) {
		sleep_ms(ms);
		return;
	}

	task_t task = task_running[core];
	task_running[core]->state = TASK_WAITING_FOR_ALARM;
	(void)add_alarm_in_us(1000 * ms, task_ready_alarm, task, true);
	task_yield();
}


void task_yield_until(uint64_t us)
{
	unsigned core = get_core_num();

	if (NULL == task_running[core]) {
		uint64_t now = time_us_64();

		if (now < us)
			sleep_us(us - now);

		return;
	}

	task_t task = task_running[core];
	task_running[core]->state = TASK_WAITING_FOR_ALARM;
	(void)add_alarm_at(us, task_ready_alarm, task, true);
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

		unsigned percent = 100llu * task->runtime_us / total_us;

		char flags[] = "?";

		if (TASK_READY == task->state)
			flags[0] = 'R';
		else if (TASK_WAITING_FOR_ALARM == task->state)
			flags[0] = 'A';
		else if (TASK_WAITING_FOR_LOCK == task->state)
			flags[0] = 'L';

		unsigned stack = stack_free_space(task);

		printf("task: %2i (%-2i %s", i, task->priority, flags);

		if (TASK_WAITING_FOR_LOCK == task->state) {
			uint32_t lock_id = ((uint32_t)task->lock >> 2) & 0x1f;
			printf("=%-2lu", lock_id);
		} else {
			printf("   ");
		}

		printf(") [%-11s]", task->name);
		printf(" [%4u] %5lux ", stack, task->resume_count);
		printf("= %8lu us = %3u%%\n", task->runtime_us, percent);
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


/****************************************************************************
 * Compatibility layer to use tasks with blocking SDK functions.            *
 ****************************************************************************/

void task_lock_spin_unlock_with_notify(struct lock_core *lc, uint32_t save)
{
	/* Unlock the lock. */
	spin_unlock(lc->spin_lock, save);

	uint32_t lock_id = ((uint32_t)lc->spin_lock >> 2) & 0x1f;
	lock_notify[get_core_num()][lock_id] = time_us_64();

	/* Unblock the schedulers. */
	__sev();
}

void task_lock_spin_unlock_with_wait(struct lock_core *lc, uint32_t save)
{
	unsigned core = get_core_num();
	task_t task = task_running[core];

	if (task) {
		task->lock = lc->spin_lock;
		task->state = TASK_WAITING_FOR_LOCK;
		spin_unlock(lc->spin_lock, save);
		task_yield();
	} else {
		spin_unlock(lc->spin_lock, save);
		__wfe();
	}
}

int task_lock_spin_unlock_with_timeout(struct lock_core *lc, uint32_t save, uint64_t time)
{
	unsigned core = get_core_num();
	task_t task = task_running[core];

	if (task) {
		task->lock = lc->spin_lock;
		task->state = TASK_WAITING_FOR_LOCK;
		spin_unlock(lc->spin_lock, save);
		task_yield();
		return time_us_64() >= time;
	} else {
		spin_unlock(lc->spin_lock, save);
		return best_effort_wfe_or_timeout(time);
	}
}

void task_sync_yield_until_before(uint64_t time)
{
	if (task_running[get_core_num()]) {
		task_yield_until(time);
	}
}
