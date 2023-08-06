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

#pragma once
#include <pico/types.h>
#include <hardware/platform_defs.h>
#include <hardware/sync.h>

#include <setjmp.h>

#if !defined(TASK_STACK_SIZE)
# define TASK_STACK_SIZE 1024
#endif

#if !defined(MAX_TASKS)
# define MAX_TASKS 8
#endif


enum task_state {
	TASK_READY = 0,
	TASK_WAITING_FOR_ALARM,
	TASK_WAITING_FOR_LOCK,
};


struct task {
	/* Saved registers, including stack pointer. */
	jmp_buf regs;

	/* '\0'-terminated task name. */
	char name[12];

	/* Main procedure of this task. */
	void (*proc)(void);

	/* Timestamp when was the task resumed the last time. */
	uint64_t resumed_at;

	/* Task priority. High priority tasks run first. */
	uint8_t priority;

	/* Reason the task is waiting. */
	enum task_state state;

	/* How many times has the task been resumed. */
	uint32_t resume_count;

	/* How many microseconds of runtime has the task collected. */
	uint32_t runtime_us;

	/* Lock for which the task is waiting. */
	spin_lock_t *lock;

	/* Stack bottom canary. */
	uint32_t canary_bottom;

	/* Actual task stack. */
	uint32_t stack[TASK_STACK_SIZE / 4] __attribute__((__aligned__((8))));

	/* Stack top canary. */
	uint32_t canary_top;
};


/* Used to define a task in the task list. */
#define MAKE_TASK(PRI, NAME, PROC) \
	(&(struct task) { \
		.name = (NAME), \
		.proc = (PROC), \
		.priority = (PRI), \
	})


/* Alias for the task pointer. */
typedef struct task *task_t;


/* Current tasks running on respective cores. */
extern task_t task_running[NUM_CORES];


/*
 * Tasks assigned to respective cores.
 * Need to be supplied by the user.
 */
extern task_t task_avail[NUM_CORES][MAX_TASKS];


/* Initialize the task scheduler. */
void task_init(void);


/*
 * Run a single task scheduled on the current core until it yields or returns.
 * Returns `false` when no task is ready.
 *
 * You must call `__wfe()` to put the core to sleep when there is nothing more
 * to do, but make sure to only call it after `task_run` has returned `false`.
 * You also need to pass in the timestamp from just before the last time the
 * `task_run` was invoked.
 *
 * Or just use `task_run_loop`.
 */
bool task_run(uint64_t since);


/*
 * Run tasks on this core indefinitely.
 */
void __attribute__((__noreturn__)) task_run_loop(void);


/*
 * Pause current task and return to the scheduler.
 *
 * Task will remain ready to be resumed. This gives the scheduler an
 * opportunity to run higher priority tasks if needed.
 */
void task_yield(void);


/* Yield task until given amount of microseconds elapses. */
void task_sleep_us(uint64_t us);


/* Yield task until given amount of milliseconds elapses. */
void task_sleep_ms(uint64_t ms);


/* Yield task until given time in microseconds. */
void task_yield_until(uint64_t us);


/* Print per-task statistics for given core and then reset them. */
void task_stats_report_reset(unsigned core);


/* Reset task statistics for given core. */
void task_stats_reset(unsigned core);
