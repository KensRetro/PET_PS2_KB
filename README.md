# PET_PS2_KB
An adaptor to connect a PS2 keyboard to a to PET computer

How It Works:
 
An interrupt-on-change routine (The PET sets a ROW to read) sets outputs (Columns) for the PET to read back as keypresses.  
A main loop is created to catch bytes from the PS2 keyboard,
Once a PS2 byte is received, it is buffered until bytes stop coming in (A PS2 keypress generates between 2 and 6 bytes)
Once timer2 runs out (which is reset every byte receive, and whose period is slightly longer that the byte to byte interval)
It is assumed the PS2 keyboard has completed sending a Key status change.  The buffer is examined and the appropriate bits are set 
in the memory map setup to represent the PET's keyboard matrix.

Note: routines are optimized for speed, not space, this is to ensure the response is fast enough to suit the PET's 
normal keyboard scan speed

Currently supports GRAPHICS keyboard ONLY - tables would need updating for business style KB
