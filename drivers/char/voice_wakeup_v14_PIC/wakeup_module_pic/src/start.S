#define zero	$0	/* wired zero */
#define AT	$1	/* assembler temp  - uppercase because of ".set at" */
#define v0	$2	/* return value */
#define v1	$3
#define a0	$4	/* argument registers */
#define a1	$5
#define a2	$6
#define a3	$7
#define t0	$8	/* caller saved */
#define t1	$9
#define t2	$10
#define t3	$11
#define t4	$12
#define t5	$13
#define t6	$14
#define t7	$15
#define s0	$16	/* callee saved */
#define s1	$17
#define s2	$18
#define s3	$19
#define s4	$20
#define s5	$21
#define s6	$22
#define s7	$23
#define t8	$24	/* caller saved */
#define t9	$25
#define jp	$25	/* PIC jump register */
#define k0	$26	/* kernel scratch */
#define k1	$27
#define gp	$28	/* global pointer */
#define sp	$29	/* stack pointer */
#define fp	$30	/* frame pointer */
#define s8	$30	/* same like fp! */
#define ra	$31	/* return address */

/*
_start:
	void (*fw_load)(void* load_addr, void* interface, void* shared_param, void* stack);
*/
	.set noreorder
	.globl _start
	.ent   _start
	.text
_start:
	//--------  a0 <- load address
	//-------   -T 0
	addu     sp,sp,-12
	sw 	gp,0(sp)
	sw	ra,4(sp)
	sw  	t9,8(sp)
	/* Initialize $gp */
	bal	1f
	nop				# 
	.word	_gp			# ra + 0 (new ra after bal)
	.word	__rel_dyn_end		# ra + 4
	.word	__rel_dyn_start		# ra + 8
	.word	__image_copy_end	# ra + c
	.word	_GLOBAL_OFFSET_TABLE_	# ra + 16
	.word	num_got_entries		# ra + 20

1:
	lw	gp, 0(ra)
	addu     gp,gp,a0

	//----------------------------------------------------------------------
	// relocate_got:
	lw	t3, 20(ra)		# t3 <-- num_got_entries
	lw	t4, 16(ra)		# t4 <-- _GLOBAL_OFFSET_TABLE_
	addu	t4, a0,t4		# t4 now holds relocated _G_O_T_
	addi	t4, t4, 8		# skipping first two entries
	li	t2, 2
1:
	lw	t1, 0(t4)
	addu	t1, a0
	sw	t1, 0(t4)
2:
	addi	t2, 1
	blt	t2, t3, 1b
	addi	t4, 4
	//----------------------------------------------------------------------
	// relocate rel
	lw	t2, 4(ra)   // <-  __rel_dyn_end
	lw	t1, 8(ra)   // <- __rel_dyn_start

	addu    t2,t2,a0
	addu    t1,t1,a0

	b	2f			# skip first reserved entry
	addi	t1, 8

1:
	lw	t3, -4(t1)		# t3 <-- relocation info

	sub	t3, 3
	bnez	t3, 2f			# skip non R_MIPS_REL32 entries
	 nop

	lw	t3, -8(t1)		# t3 <-- location to fix up in FLASH
	addu	t3, a0			# t3 <-- location to fix up in RAM

	lw	t4, 0(t3)		# t4 <-- original pointer
	addu	t4, a0			# t4 <-- adjusted pointer
	sw	t4, 0(t3)

2:
	blt	t1, t2, 1b
	addi	t1, 8			# each rel.dyn entry is 8 bytes


	//----------------------------------------------------------------------
	// zero .bss
	la	t1, __bss_start		# t1 <-- __bss_start
	la	t2, __bss_end		# t2 <-- __bss_end
1:
	sw	zero, 0(t1)
	blt	t1, t2, 1b
	addi	t1, 4

	//----------------------------------------------------------------------
	// init parameters
	la	t2,fw_interface_entry
	sw      t2,0(a1)

	la  	t0, shared_param
	sw	a2,0(t0)


	la  	t0, program_stack
	sw	a3, 0(t0)

	//la      t0, extern_func
	//sw	a3,0(t0)

/*
	move    t3,a1
	la	t2,fw_interface_entry
	sw      t2,0(t3)

	addu    t3,t3,4
	la	t2,0xffffffff
	addi    t1,t3,0x40			// set -1 to p_slp_param  4~64 bytes
1:
	sw	t2,0(t3)
	addu    t3,t3,4
	bne	t1,t3,1b
	nop
*/
	lw	gp,0(sp)
	lw	ra,4(sp)
	lw      t9,8(sp)
	addu     sp,sp,12
	jr     ra
	nop
	.end _start

	// reserve firmware stack space
	.space  256
	//.space  1024

	.globl	fw_interface_entry
fw_interface_entry:
	.ent	fw_interface_entry
	.type   fw_interface_entry,@function
	.set    noreorder
	addu     sp,sp,-12
	sw 	gp,0(sp)
	sw	ra,4(sp)
	sw  	t9,8(sp)
	bal   1f
	nop
1:

	addu   t9,ra,4			// new ra here.
	.cpload t9

	//la  	t0, program_stack
	//sw 	sp, (t0)	// save the caller(kernel) sp to 1st word(or tail) of new stack.
	//addiu    sp, t0,-4

	// save sp
	//sw 	sp, -28(ra)	// save the caller(kernel) sp to 1st word(or tail) of new stack.
	//addiu    sp,ra,-32

	la	t9, command_dispatch
	jalr	t9
	nop

	// restore the old caller(kernel) sp
	//lw 	sp, 4(sp)

	lw	gp,0(sp)
	lw	ra,4(sp)
	lw      t9,8(sp)
	addu     sp,sp,12
	jr     ra
	nop
	.end fw_interface_entry

sleep_pm_exit:
	.globl	sleep_pm_exit
	.ent	sleep_pm_exit
	.set    noreorder
	bal   1f
	nop
1:
	addu   t9,ra,4
	.cpload t9

	//set sp
	//addiu    sp,ra,-4

	//la	t9,core_sleep_restore
	jr      t9
	nop
	.end sleep_pm_exit

call_function:
	.globl	call_function
	.ent	call_function
	.set    noreorder
	addu    sp,sp,-8
	sw	gp,0(sp)
	sw	ra,4(sp)
	jalr    a0
	nop
	lw	gp,0(sp)
	lw 	ra,4(sp)
	addu    sp,sp,8
	jr	ra
	nop
	.end call_function
