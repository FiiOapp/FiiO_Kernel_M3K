OUTPUT_ARCH(mips)
ENTRY(_start)
SECTIONS
{
 . = 0x00000000;
 . = ALIGN(4);
 .text : {
  *(.text*)
 }
 . = ALIGN(4);
 .rodata : {
  *(SORT_BY_ALIGNMENT(SORT_BY_NAME(.rodata*)))
 }
 . = ALIGN(4);
 .data : {
  *(.data*)
 }
 . = .;
 _gp = ALIGN(16) + 0x7ff0;
 .got : {
  *(.got)
 }
 num_got_entries = SIZEOF(.got) >> 2;
 . = ALIGN(4);
 .sdata : {
  *(.sdata*)
 }
 . = ALIGN(4);
 .u_boot_list : {
  KEEP(*(SORT(.u_boot_list*)));
 }
 . = ALIGN(4);
 __image_copy_end = .;
 .rel.dyn : {
  __rel_dyn_start = .;
  *(.rel.dyn)
  __rel_dyn_end = .;
 }
 .deadcode : {
  LONG(0xffffffff);
 }
 .dynsym : {
  *(.dynsym)
 }
 .bss __rel_dyn_start (OVERLAY) : {
  __bss_start = .;
  *(.sbss*)
  *(.bss*)
  *(COMMON)
  . = ALIGN(4);
  __bss_end = .;
 }
 /DISCARD/ : {
  *(.dynbss)
  *(.dynstr)
  *(.dynamic)
  *(.interp)
  *(.hash)
  *(.gnu.*)
  *(.plt)
  *(.got.plt)
  *(.rel.plt)
 }
}
