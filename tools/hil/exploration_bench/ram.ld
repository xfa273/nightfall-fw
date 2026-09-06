/* Non-motor benchmark only. No section may be loaded into Flash. */
ENTRY(bench_entry)
MEMORY {
  RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 304K
  MAILBOX (rw) : ORIGIN = 0x2004C000, LENGTH = 16K
}
SECTIONS {
  .vectors : { KEEP(*(.vectors)) } > RAM
  .text : { *(.text*) *(.rodata*) } > RAM
  .ARM.exidx : { *(.ARM.exidx*) } > RAM
  .data : { *(.data*) } > RAM
  .bss (NOLOAD) : {
    . = ALIGN(8); __bss_start__ = .;
    *(.bss*) *(COMMON)
    . = ALIGN(8); __bss_end__ = .;
  } > RAM
  .stack (NOLOAD) : { . = ALIGN(8); . += 8192; } > RAM
  .mailbox (NOLOAD) : { KEEP(*(.mailbox)) } > MAILBOX
  __stack_top__ = ORIGIN(MAILBOX);
  /DISCARD/ : { *(.comment) *(.note*) }
}
