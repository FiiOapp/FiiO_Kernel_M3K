/*
 * start.S
 *
 * Entry point of the Voice Wakeup Module.
 * The firmware code are executed in the L2Cache.
 *
 * Copyright (C) 2006 Ingenic Semiconductor Inc.
 *
 */
#include <tcsm_layout.h>

	.global interface
	.extern open
	.extern voice_handler
	.extern close
	.extern cache_prefetch

	.extern set_handler
	.extern get_dma_address
	.extern ioctl
	.extern get_resource_addr
	.extern process_data
	.extern is_cpu_wakeup_by_dmic
	.extern set_sleep_buffer
	.extern get_sleep_process
	.extern set_dma_channel
	.extern voice_wakeup_enable
	.extern is_voice_wakeup_enabled
	.extern cpu_should_sleep
	.extern set_record_buffer
	.extern get_record_buffer
	.extern get_record_buffer_len
	.extern set_desc_addr

	.extern module_init
	.extern module_exit

	.globl	handler

	.section .text
	.type	interface, @function;
	.ent interface
interface:
	.word open
	.word handler
	.word close
	.word cache_prefetch
	.word set_handler
	.word get_dma_address
	.word ioctl
	.word get_resource_addr
	.word process_data
	.word is_cpu_wakeup_by_dmic
	.word set_sleep_buffer
	.word get_sleep_process
	.word set_dma_channel
	.word voice_wakeup_enable
	.word is_voice_wakeup_enabled
	.word cpu_should_sleep
	.word set_record_buffer
	.word get_record_buffer
	.word get_record_buffer_len
	.word set_desc_addr

	.word module_init
	.word module_exit
	.end  interface

handler:
	.ent	handler
	.set    noreorder
	addu    $29,$29,-16
	sw	$31,0($29)
	sw	$16, 4($29)
	sw	$25, 8($29)
	la  $16, CPU_SAVE_SP
	sw $29, 0($16)

	la    $29,CPU_RESUME_SP
	addu    $29,$29,-16
	la 	$25, voice_handler
	jalr    $25
	nop

	la  $16, CPU_SAVE_SP
	lw  $29, 0($16)
	lw	$31, 0($29)
	lw	$16, 4($29)
	lw	$25, 8($29)
	addu    $29,$29,16
	jr	$31
	nop
	.end handler
