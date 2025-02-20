# PET_PS2_KB
An adaptor to connect a PS2 keyboard to a to PET computer.  Plugs directly into the motherboard at the keyboard connector.  
Needs a 5V supply from a convenient place on the motherboard.  There is a ground pin on the KB connector so optionally connect
a ground to the power connector.  The modular jack on the board is optional - Used for programming and debugging the PIC 
program.

Uses a PIC16F18877 to do pretty much everything.

Operation:
MOST keys are translated to the PETS keyboard matrix, see * notes below.
Currently, only the GRAPHIC keyboard is supported, but it would only take a bit of editing to change the tables for a BUSINESS keyboard.

Special keys are as follows:

-- PS2 Key -> PET Key --
* ESC      -> RUN/STOP
* Tilde    -> Left Arrow
* TAB      -> RVS
* SCRLOK   -> (Reboots keyboard interface)***
* END      -> Clear Screen (shift-home)
* Shift 6  -> Up Arrow
* ALT      -> Shift to Graphic on double function keys (1-9 etc.)**
* L/R CTRL -> Hardware equivalnt of SHIFT keys*

Non PET keys are generally No Function

*The PS2 shift keys are  generally treated as a "Soft-Shift" and sent to the PET as needed when a second key is pressed.
If you need to press the SHIFT key alone on the PET use the CTRL key instead.
Limitations of the PS2 keyboard seem to block both SHIFT or CTRL being detected as pressed at the same time, so translating
this to the PET is not possible.  Other multi-key combinations may also not work, but generally it's pretty good.

**Example: to get the graphic on the ! key you need to press SHIFT-ALT-1  (SHIFT-1 for ! and ALT for graphic)

***Once in a while (usually while pressing many keys at once) a key press or release is not detected, pressing the SCROLL-LOCK 
key reboots the PIC and resets the keymap to no keys pressed, usually there's no effect to the PET


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

DISCLAIMER: All information is provided 'AS-IS" for informational purposes only.
I have tried to be as accurate as possible but that doesn't mean there are no mistakes. 
Any errors or omissions etc. are your own responsibility.  
There is no Warranty or guarantee of any kind. See licence for details.
