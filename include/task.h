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

/* Maximum number of tasks on a single core. */
#if !defined(MAX_TASKS)
# define MAX_TASKS 8
#endif

#if !defined(__noreturn)
# define __noreturn  __attribute__((noreturn))
#endif


/* Private task data. */
typedef struct task *task_t;


/* Per-task statistics. */
struct task_stats {
	/* How many times has the task been resumed. */
	uint32_t resumed;

	/* How many microseconds of runtime has the task collected. */
	uint32_t total_us;
};

typedef struct task_stats task_stats_t;


/* Current tasks running on respective cores. */
extern task_t task_running[NUM_CORES];


/* Tasks assigned to respective cores. */
extern task_t task_avail[NUM_CORES][MAX_TASKS];


/* Collected task statistics. */
extern task_stats_t task_stats[NUM_CORES][MAX_TASKS];


/* Initialize task scheduler. */
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
__noreturn void task_run_loop(void);


/*
 * Create new task with given stack size.
 * Task private data are stored at the top of the stack.
 *
 * Minimum stack size (including internal task data) is 256 bytes.
 *
 * Recommended minimum value is 1024 bytes for when you do not plan to call
 * complex SDK functions. Allocating 4096 bytes should be enough for anything.
 */
task_t task_create(void (*fn)(void), size_t size);


/* Same as `task_create`, but allows specifying the core. */
task_t task_create_on_core(unsigned core, void (*fn)(void), size_t size);


/*
 * Pause current task and return to the scheduler.
 *
 * Task will remain ready to be resumed. This gives the scheduler an
 * opportunity to run higher priority tasks if needed.
 */
void task_yield(void);


/*
 * Pause current task and return to the scheduler.
 *
 * Task won't be resumed until marked ready. You need to set an alarm or some
 * other interrupt to mark it once it should be resumed. Or mark it ready from
 * another task.
 */
void task_yield_until_ready(void);


/* Yield task until given amount of microseconds elapses. */
void task_sleep_us(uint64_t us);


/* Yield task until given amount of milliseconds elapses. */
void task_sleep_ms(uint64_t ms);


/* Yield task until given time in microseconds. */
void task_yield_until(uint64_t us);


/* Mark given task as ready to continue. */
void task_set_ready(task_t task);


/* Manage task priority. Higher priority tasks run first. */
void task_set_priority(task_t task, int8_t pri);
int8_t task_get_priority(task_t task);


/* Manage task name of up to 11 bytes. */
void task_set_name(task_t task, const char *name);
void task_get_name(task_t task, char name[9]);


/* Print per-task statistics for given core and then reset them. */
void task_stats_report_reset(unsigned core);

/* Reset task statistics for given core. */
void task_stats_reset(unsigned core);
