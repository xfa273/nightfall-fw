ENTRY(prepare_entry)
MEMORY { RAM (rwx) : ORIGIN = 0x2004E000, LENGTH = 4K }
SECTIONS {
  .text : { KEEP(*(.entry)) *(.text*) *(.rodata*) } > RAM
  /DISCARD/ : { *(.ARM.exidx*) *(.comment) *(.note*) }
}
