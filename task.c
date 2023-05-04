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

#include <pico/stdlib.h>
#include <pico/lock_core.h>
#include <hardware/sync.h>

#include <string.h>
#include <stdlib.h>
#include <limits.h>
#include <setjmp.h>
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


enum wait_reason {
	READY = 0,
	NOT_READY,
	WAITING_FOR_ALARM,
	WAITING_FOR_LOCK,
};


struct task {
	/* Saved registers, including stack pointer. */
	jmp_buf regs;

	/* Bottom of the stack. */
	uint32_t *stack;

	/* Size of the stack. */
	uint32_t stack_size;

	/* '\0'-terminated task name. */
	char name[12];

	/* Timestamp when was the task resumed the last time. */
	uint64_t resumed_at;

	/* Reason the task is waiting. */
	enum wait_reason waiting : 3;

	/* Task priority. High priority tasks run first. */
	int8_t pri : 8;

	/* To pad the struct to 128 bytes. */
	uint32_t _align_pad_21 : 21;

	/* Lock for which the task is waiting. */
	spin_lock_t *lock;

	/* Stack top canary. */
	uint32_t canary;
};


enum task_status {
	TASK_SAVE = 0,
	TASK_YIELD,
	TASK_RETURN,
};


uint64_t lock_notify[NUM_CORES][32] = {0};


task_t task_running[NUM_CORES] = {0};
task_t task_avail[NUM_CORES][MAX_TASKS] = {0};

task_stats_t task_stats[NUM_CORES][MAX_TASKS] = {0};


/* Saved scheduler context for respective cores. */
static jmp_buf task_return[NUM_CORES];


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

		if (!task)
			continue;

		if (task->lock) {
			uint32_t lock_id = ((uint32_t)task->lock >> 2) & 0x1f;

			for (unsigned c = 0; c < NUM_CORES; c++) {
				if (lock_notify[c][lock_id] >= since) {
					task->lock = NULL;
					task->waiting = READY;
					break;
				}
			}
		}

		if (task->waiting)
			continue;

		if (task->pri > min_pri) {
			min_pri = task->pri;
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
	alarm_pool_init_default();

	priv_lock = spin_lock_init(spin_lock_claim_unused(true));

	for (unsigned i = 0; i < NUM_CORES; i++) {
		task_running[i] = NULL;
		memset(task_avail[i], 0, sizeof(task_avail[i]));
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
		task_stats[core][task_no].resumed++;
		task_avail[core][task_no]->resumed_at = time_us_64();
		task_running[core] = task;
		longjmp(task->regs, (int)task);
	}

	if (TASK_YIELD == status) {
		uint64_t since = task->resumed_at;
		task_stats[core][task_no].total_us += time_us_64() - since;
		task_running[core] = NULL;

		if (0xdeadbeef != task->canary)
			panic("task [%s]: stack underflow", task->name);

		if (0xdeadbeef != task->stack[0])
			panic("task [%s]: stack overflow", task->name);

		return true;
	}

	if (TASK_RETURN == status) {
		task_running[core] = NULL;

		uint32_t save = spin_lock_blocking(priv_lock);
		task_avail[core][task_no] = NULL;
		spin_unlock(priv_lock, save);

		if (0xdeadbeef != task->canary)
			panic("task [%s]: stack underflow", task->name);

		if (0xdeadbeef != task->stack[0])
			panic("task [%s]: stack overflow", task->name);

		free(task->stack);

		return true;
	}

	panic("invalid setjmp status (%i)", status);
}


__noreturn void task_run_loop(void)
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


task_t task_create(void (*fn)(void), size_t size)
{
	return task_create_on_core(get_core_num(), fn, size);
}


task_t task_create_on_core(unsigned core, void (*fn)(void), size_t size)
{
	static_assert(sizeof(struct task) % 8 == 0);

	if (size < 256)
		panic("task_create_on_core: requires at least 256 bytes");

	if (core >= NUM_CORES)
		panic("invalid core number");

	void *stack = malloc(size);

	if (NULL == stack)
		panic("task_create_on_core: failed to allocate stack (size=%u)", size);

	struct task *task = stack + size - sizeof(*task);
	memset(task, 0, sizeof(*task));

	task->stack = stack;
	task->stack_size = size - sizeof(*task);
	task->regs[SP] = (unsigned)task;
	task->regs[PC] = (unsigned)task_sentinel;
	task->regs[R4] = (unsigned)fn;
	task->waiting = NOT_READY;

	/* Mark stack so that we can determine its usage. */
	for (size_t i = 0; i < task->stack_size >> 2; i++)
		task->stack[i] = 0xdeadbeef;

	task->canary = 0xdeadbeef;

	uint32_t save = spin_lock_blocking(priv_lock);

	for (int i = 0; i < MAX_TASKS; i++) {
		if (!task_avail[core][i]) {
			task_avail[core][i] = task;
			spin_unlock(priv_lock, save);
			return task;
		}
	}

	panic("Too many tasks on core %u (MAX_TASKS=%u)", core, MAX_TASKS);
}


void task_yield(void)
{
	unsigned core = get_core_num();

	if (NULL == task_running[core])
		panic("task_yield called from outside of a task");

	if (0 == setjmp(task_running[core]->regs)) {
		longjmp(task_return[core], TASK_YIELD);
	}
}


void task_yield_until_ready(void)
{
	unsigned core = get_core_num();

	if (NULL == task_running[core])
		panic("task_yield_until_ready called from outside of a task");

	task_running[core]->waiting = NOT_READY;
	task_yield();
}


void task_set_ready(task_t task)
{
	uint32_t save = spin_lock_blocking(priv_lock);

	if (task->waiting == NOT_READY)
		task->waiting = READY;

	spin_unlock(priv_lock, save);
}


void task_set_priority(task_t task, int8_t pri)
{
	task->pri = pri;
}


static int64_t task_ready_alarm(alarm_id_t, void *arg)
{
	task_t task = arg;

	if (WAITING_FOR_ALARM == task->waiting)
		task->waiting = READY;

	return 0;
}


void task_sleep_us(uint64_t us)
{
	unsigned core = get_core_num();

	if (NULL == task_running[core])
		panic("task_sleep_us called from outside of a task");

	task_t task = task_running[core];
	task_running[core]->waiting = WAITING_FOR_ALARM;
	(void)add_alarm_in_us(us, task_ready_alarm, task, true);
	task_yield();
}


void task_sleep_ms(uint64_t ms)
{
	unsigned core = get_core_num();

	if (NULL == task_running[core])
		panic("task_sleep_ms called from outside of a task");

	task_t task = task_running[core];
	task_running[core]->waiting = WAITING_FOR_ALARM;
	(void)add_alarm_in_us(1000 * ms, task_ready_alarm, task, true);
	task_yield();
}


void task_yield_until(uint64_t us)
{
	unsigned core = get_core_num();

	if (NULL == task_running[core])
		panic("task_yield_until called from outside of a task");

	task_t task = task_running[core];
	task_running[core]->waiting = WAITING_FOR_ALARM;
	(void)add_alarm_at(us, task_ready_alarm, task, true);
	task_yield();
}


int8_t task_get_priority(task_t task)
{
	return task->pri;
}


void task_set_name(task_t task, const char *name)
{
	strncpy(task->name, name, sizeof(task->name));
	task->name[sizeof(task->name) - 1] = '\0';
}


void task_get_name(task_t task, char name[9])
{
	strcpy(name, task->name);
}


static uint32_t stack_free_space(task_t task)
{
	uint32_t level = 0;

	for (size_t i = 0; i < task->stack_size >> 2; i++) {
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
	uint32_t total_us = 0;

	for (int i = 0; i < MAX_TASKS; i++)
		total_us += task_stats[core][i].total_us;

	if (!total_us)
		total_us = 1;

	printf("task: core %u:\n", core);

	for (int i = 0; i < MAX_TASKS; i++) {
		if (NULL == task_avail[core][i])
			continue;

		task_t task = task_avail[core][i];
		task_stats_t *stats = &task_stats[core][i];

		unsigned percent = 100 * stats->total_us / total_us;

		char flags[] = "?";

		if (READY == task->waiting)
			flags[0] = 'R';
		else if (NOT_READY == task->waiting)
			flags[0] = 'X';
		else if (WAITING_FOR_ALARM == task->waiting)
			flags[0] = 'A';
		else if (WAITING_FOR_LOCK == task->waiting)
			flags[0] = 'L';

		unsigned stack = stack_free_space(task);

		if (WAITING_FOR_LOCK == task->waiting) {
			uint32_t lock_id = ((uint32_t)task->lock >> 2) & 0x1f;
			printf("task: %2i (%-2i %s=%-2lu) [%-11s] [%4u] %5lux = %8lu us = %3u%%\n",
				i, task->pri, flags, lock_id, task->name, stack,
				stats->resumed, stats->total_us, percent);
		} else {
			printf("task: %2i (%-2i %s   ) [%-11s] [%4u] %5lux = %8lu us = %3u%%\n",
				i, task->pri, flags, task->name, stack,
				stats->resumed, stats->total_us, percent);
		}
	}

	task_stats_reset(core);
}


void task_stats_reset(unsigned core)
{
	memset(task_stats[core], 0, sizeof(task_stats[core]));
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
		task->waiting = WAITING_FOR_LOCK;
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
		task->waiting = WAITING_FOR_LOCK;
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
