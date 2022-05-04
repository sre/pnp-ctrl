/* memory.x - Linker script for the stm32f401rct6 */
MEMORY
{
  /* Flash memory begins at 0x80000000 and has a size of 256kB*/
  FLASH : ORIGIN = 0x08004000, LENGTH = 256K - 0x4000
  /* RAM begins at 0x20000000 and has a size of 64kB*/
  RAM : ORIGIN = 0x20000010, LENGTH = 64K - 0x10
}

/* This is where the call stack will be allocated. */
/* The stack is of the full descending type. */
/* NOTE Do NOT modify `_stack_start` unless you know what you are doing */
_stack_start = ORIGIN(RAM) + LENGTH(RAM);
