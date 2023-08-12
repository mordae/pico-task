#include "pico/asm_helper.S"

pico_default_asm_setup

/*
 * r0: return value to be passed to the target task
 * r1: address to restore registers from
 * r2: address to save registers to
 * r3: scratch register
 */
regular_func_with_section task_swap_context
	/* Save the lower registers + SP. */
	mov r3, sp
	stmia r2!, {r3-r7}

	/* Save the higher registers. */
	mov r3,  r8
	mov r4,  r9
	mov r5, r10
	mov r6, r11
	mov r7, r12
	stmia r2!, {r3-r7}

	/* Save LR. */
	mov r3, lr
	stmia r2!, {r3}

	/* Restore the higher registers. */
	adds r1, #20
	ldmia r1!, {r3-r7}
	mov  r8, r3
	mov  r9, r4
	mov r10, r5
	mov r11, r6
	mov r12, r7

	/* Restore LR. */
	ldmia r1!, {r3}
	mov lr, r3

	/* Restore the lower registers + SP. */
	subs r1, #44
	ldmia r1!, {r3-r7}
	mov sp, r3

	/* Return. */
	bx lr

	/* r1 is intact and now acts as the return value. */
