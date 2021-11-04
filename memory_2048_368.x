/* For STM32F765,767,768,769,777,778,779 devices */
MEMORY
{
  /* NOTE K = KiBi = 1024 bytes */
  FLASH : ORIGIN = 0x08000000, LENGTH = 2M
  RAM : ORIGIN = 0x20020000, LENGTH = 368K + 16K
  ITCM : ORIGIN = 0x00000000, LENGTH = 16K /* Instruction Tighly Coupled Memory */
  DTCM : ORIGIN = 0x20000000, LENGTH = 128K /* Data Tighly Coupled Memory */
}

SECTIONS
{
    .itcm : ALIGN(4)
    {
        *(.itcm .itcm.*);
        . = ALIGN(4);
    } > ITCM

    .dtcm : ALIGN(4)
    {
        *(.dtcm .dtcm.*);
        . = ALIGN(4);
    } > DTCM
}

/* You can then use something like this to place a variable into a specific section of memory:
 *  #[link_section = ".dtcm.BUFFER"]
 *  static mut BUF: [u8; 1024] = [3u8; 1024];
 *  Verifiable with: cargo size --release --example hello_world -- -A
 */

/* This is where the call stack will be allocated. */
/* The stack is of the full descending type. */
/* NOTE Do NOT modify `_stack_start` unless you know what you are doing */
_stack_start = ORIGIN(RAM) + LENGTH(RAM);
