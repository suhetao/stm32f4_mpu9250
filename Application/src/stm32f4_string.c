#include "stm32f4_string.h"

//make sure 4-bytes aligned
__asm void FastMemCpy(uint8_t* dest, uint8_t* src, uint16_t size)
{
	push {r4-r11};
	lsr r3, r2, #5; 32 bytes
	cmp r3, #0;
	beq notenough_32;
	and r2, r2, #31; left bytes
loop_32
	ldmia r1!, {r4-r11};
	stmia r0!, {r4-r11};
	subs r3, r3, #1;
	bne loop_32;	
notenough_32
	lsr r3, r2, #4; 16 bytes
	cmp r3, #0;
	beq notenough_16;
	and r2, r2, #15; left bytes
loop_16
	ldmia r1!, {r4-r7};
	stmia r0!, {r4-r7};
	subs r3, r3, #1;
	bne loop_16;	
notenough_16
	lsr r3, r2, #3; 8 bytes
	cmp r3, #0;
	beq notenough_8;
	and r2, r2, #7; left bytes
loop_8
	ldmia r1!, {r4-r5};
	stmia r0!, {r4-r5};
	subs r3, r3, #1;
	bne loop_8;
notenough_8
	lsr r3, r2, #2; 4 bytes
	cmp r3, #0;
	beq notenough_4;
	and r2, r2, #3; left bytes
loop_4
	ldmia r1!, {r4};
	stmia r0!, {r4};
	subs r3, r3, #1;
	bne loop_4;
notenough_4
	cmp r2, #0;
	beq exit

loop_1
	subs r2, r2, #1;
	ldrb r3, [r1, r2]
	strb r3, [r0, r2]
	cmp r2, #0
	bne loop_1;
exit
	pop {r4-r11}
	bx lr
}
