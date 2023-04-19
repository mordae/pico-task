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
#include <pico.h>

#if !defined(__ASSEMBLER__)

#define lock_internal_spin_unlock_with_best_effort_wait_or_timeout(lock, save, until) \
	task_lock_spin_unlock_with_timeout((lock), (save), (until))

#define lock_internal_spin_unlock_with_notify(lock, save) \
	task_lock_spin_unlock_with_notify((lock), (save))

#define lock_internal_spin_unlock_with_wait(lock, save) \
	task_lock_spin_unlock_with_wait((lock), (save))

#define sync_internal_yield_until_before(until) \
	task_sync_yield_until_before((until))

struct lock_core;

void task_lock_spin_unlock_with_notify(struct lock_core *lc, uint32_t save);
void task_lock_spin_unlock_with_wait(struct lock_core *lc, uint32_t save);
int task_lock_spin_unlock_with_timeout(struct lock_core *lc, uint32_t save, uint64_t time);
void task_sync_yield_until_before(uint64_t time);

#endif
