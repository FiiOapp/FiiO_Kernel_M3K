/*
 * start.S
 *
 * Entry point of the Voice Wakeup Module.
 * The firmware code are executed in the L2Cache.
 *
 * Copyright (C) 2016 Ingenic Semiconductor Inc.
 *
 */


	.globl	interface

	.extern get_symbol
	.extern wakeup_handler

interface:
	.word get_symbol
	.word wakeup_handler
