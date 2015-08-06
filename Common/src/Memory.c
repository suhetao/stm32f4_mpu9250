/*
The MIT License (MIT)

Copyright (c) 2015-? suhetao

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "Memory.h"

//make sure 4-bytes aligned
__asm void FastMemCpy(u8* dest, u8* src, u16 size)
{
	movs r3, r2, asr #5        ;any chunks of 8 words?
	beq copywords        ;jump if no 8-word chunks
	and r2, r2, #0x1f        ;subtract chunks from size
	stmfd   sp!, {r4-r11}    ;save working registers

octcopy
	ldmia r1!, {r4-r11};        ;load 8 words from src
	stmia r0!, {r4-r11};        ;write 8 words to dest
	subs r3, r3, #1;        ;more 8-word chunks to move?
	bne octcopy;        ;loop if more chunks
	ldmfd   sp!, {r4-r11}    ;restore working registers

copywords
	movs r3, r2, asr #2        ;any more whole words to move?
	beq     copybytes        ;jump if no more whole words
	
	stmdb sp!, {lr}    	;push return address
wordcopy
	ldr lr, [r1], #4  	;read next word from src
	str lr, [r0], #4  	;write next word to dest
	subs r3, r3, #1    	;decrement word count
	bne wordcopy   	;loop while more words to move
	ldmia sp!, {lr}    	;pop return address
	
copybytes
	ands r2, r2, #3    	;any last bytes to transfer?
	beq done    	;return if already done
	
bytecopy
	ldrb r3, [r1], #1  	;read byte from src
	strb r3, [r0], #1  	;write byte to dest
	subs r2, r2, #1    	;--size (decrement loop count)
	bne bytecopy  	;loop if more bytes to move
done
	bx lr;    	;return to caller
}

__asm void *MemCpy(u8* dest, u8* src, u16 size)
{
	teq r2, #0       	;is arg n == 0 ?
	bne start       	;if n == 0, return
	bx lr;
start
	stmdb sp!, {lr}    	;push return address
	mov r12, r0       	;copy pointer p1
	cmp r2, #0x8     	;is string long or short?
	ble ByteSerial  	;jump if long string

	sub r3, r0, r1    	;compare pointers p1, p2
	tst r3, #3       	;strings aligned same?
	bne ByteSerial  	;jump if strings not aligned

	;Both strings are similarly aligned WRT word boundaries.
	;At least a portion of the data can be copied an entire
	;word at a time, which is faster than copying bytes.

WordSerial
	ands r3, r0, #3    	;check byte alignment
	beq WordAligned 	;jump if p1, p2 word-aligned

	rsb r3, r3, #4    	;m = no. of odd initial bytes
	sub r2, r2, r3    	;n = n - m

 ;If the two strings do not begin on word boundaries, begin
 ;by copying the odd bytes that precede the first full word.

PreLoop
	ldrb	lr, [r1], #1  	;read byte from src
	subs	r3, r3, #1    	;--m (decrement loop count)
	strb	lr, [r12], #1  	;write byte to dest
	bne	PreLoop     	;loop if more bytes to move

WordAligned
	movs r3, r2, asr #5 	;any chunks of 8 words?
	beq OctsDone    	;jump if no 8-word chunks

	and r2, r2, #0x1f 	;subtract chunks from size
	stmdb sp!, {r4-r10} 	;save registers on stack
	
	;The strings are long enough that we can transfer at least
	;some portion of the data in 8-word chunks.

OctLoop
	ldmia r1!, {r4-r10, lr} ;load 8 words from src
	subs r3, r3, #1    	;more 8-word chunks to move?
	stmia r12!, {r4-r10, lr} ;write 8 words to dest
	bne OctLoop     	;loop if more chunks

	ldmia sp!,{r4-r10} 	;restore registers from stack

OctsDone
	movs r3,r2,asr #2 	;any more whole words to move?
	beq WordsDone   	;jump if no more whole words

	;Copy as much of the remaining data as possible one word at a time.

WordLoop2
	ldr lr, [r1], #4  	;read next word from src
	subs r3, r3, #1    	;decrement word count
	str lr, [r12], #4  	;write next word to dest
	bne WordLoop2   	;loop while more words to move

WordsDone
	ands r2,r2,#3    	;any last bytes to transfer?
	ldmeqia sp!,{pc}    	;return if already done

	;The two strings do not end on word boundaries.
	;Copy the remaining data one byte at a time.

ByteSerial
	ldrb lr,[r1],#1  	;read byte from src
	subs r2,r2,#1    	;--size (decrement loop count)
	strb lr,[r12],#1  	;write byte to dest
	bne ByteSerial  	;loop if more bytes to move

	ldmia sp!,{pc}    	;return to caller
}

__asm void *MemMove(u8* dest, u8* src, u16 size)
{
	cmp r0,r1          ;is d > s ?
	bls MemCpy     ;jump to MemCpy if d <= s

	;Need to copy backwards, starting at tail ends of source and
	;destination arrays.  Copy a word or a byte at a time?

	orr r3, r1, r0       ;tmp = s | d
	orr r3, r3, r2       ;tmp = s | d | c
	ands r3, r3, #3       ;is tmp even multiple of 4?

	add r1, r1, r2       ;s + c (end of source buffer)
	add r2, r2, r0       ;d + c (end of dest nth buffer)

	beq MOVE1          ;jump if tmp is multiple of 4
	b MOVE2

	;because count c is an even multiple of 4 and the source
	;and destination arrays begin on even word boundaries, move
	;an entire word at a time from source to destination.

MOVE3
	ldrb r3, [r1, #-1]!   ;load next byte from source
	strb r3, [r2, #-1]!   ;store next byte to dest Nth
MOVE2
	teq r0, r2          ;more bytes to move?
	bne MOVE3          ;jump if more bytes

	bx lr          ;all done

	;Because the source and destination arrays are not aligned to even
	;word boundaries in memory, transfer only a byte at a time.

MOVE4
	ldr r3, [r1, #-4]!   ;load next word from source
	str r3, [r2, #-4]!   ;store next word to dest Nth
MOVE1
	teq r0, r2          ;more words to move?
	bne MOVE4          ;jump if more words

	bx lr          ;all done
}

__asm s32 MemCmp(u8 *dest, u8 *src, u16 n)
{
	stmdb    sp!,{r4-r6,lr}    ;save registers on stack
	mov      r3,r0             ;copy 1st arg, p1
	mov      r0,#0             ;set return value = 0
	teq      r2,#0             ;is n == 0 ?
	ldmeqia  sp!,{r4-r6,pc}    ;return if n == 0

	cmp      r2,#12            ;is n > 12
	ble      ByteLoop          ;if n <= 12, jump

	sub      r0,r3,r1          ;p1 - p2
	ands     r0,r0,#3          ;(p1 - p2) & 3
	beq      WordCompare       ;jump if byte offsets match
	
	;The strings begin at different byte offsets WRT word boundaries.
	;Loop below processes only a single pair of bytes per iteration.
 
ByteLoop
	ldrb     r0,[r3],#1        ;b1 = next byte from string 1
	ldrb     r4,[r1],#1        ;b2 = next byte from string 2
	subs     r0,r0,r4		       ;b1 - b2
	bne      ByteBreak         ;if b1 != b2, break out of loop
	subs     r2,r2,#1          ;--n (decrement loop counter)
	bne      ByteLoop          ;loop again if n > 0

ByteBreak
	ldmia    sp!,{r4-r6,pc}    ;return to caller (ret val = b1-b2)

	;The two strings have same starting byte alignment WRT word boundary.
	;Set up inner loop that compares a pair of words per iteration.

WordCompare
	add      r5,r3,r2          ;e1 = p1 + n (point to trailing byte)
	and      lr,r3,#3          ;align = p1 & 3 (initial byte offset)
	bic      r3,r3,#3          ;p1 &= ~3 (point to word boundary)
	bic      r1,r1,#3          ;p2 &= ~3
	add      r2,r5,#3          ;e1 + 3
	sub      r2,r2,r3          ;e1 + 3 - p1
	mov      r2,r2,lsr #2      ;nWords = (e1 + 3 - p1) >> 2

	mvn      r6,#0             ;initialize mask to all 1s
	mov      r0,lr,lsl #3      ;convert byte offset to bit offset
	mov      r6,r6,lsl r0      ;poke holes in mask for invalid bytes
	ldr      r0,[r3],#4        ;w1 = *p1++
	and      r0,r0,r6          ;isolate starting bytes in 1st string
	ldr      r4,[r1],#4        ;w2 = *p2++
	and      r4,r4,r6          ;isolate starting bytes in 2nd string

	;Inner loop:  Compare the two strings one word at a time to look for
	;a mismatch.  If the two strings match, return 0.

WordLoop
	subs     r0,r0,r4          ;w1 - w2
	bne      WordBreak         ;if w1 != w2, break out of loop
	ldr      r0,[r3],#4        ;w1 = *p1++
	subs     r2,r2,#1          ;--nWords
	ldr      r4,[r1],#4        ;w2 = *p2++
	bne      WordLoop          ;loop again if more words in string

	mov      r0,#0             ;set return argument = 0
	ldmia    sp!,{r4-r6,pc}    ;all done -- return to caller

	;The strings may still match if the apparent mismatch happened in
	;the final pair of words from the two strings (in trailing bytes).

WordBreak
	teq      r2,#1             ;nWords == 1? (mismatch at EOS?)
	bne      FindMismatch      ;jump if nWords != 1

	ands     r5,r5,#3          ;is trailing byte word-aligned?
	beq      FindMismatch      ;jump if word-aligned (real mismatch)

	mvn      r6,#0             ;initialize mask to all 1s
	mov      r5,r5,lsl #3      ;convert byte offset to bit offset
	mov      r6,r6,lsl r5
	mvn      r6, r6            ;poke holes in mask for invalid bytes
	ands     r0,r0,r6          ;mask off trailing bytes, string 1
	ldmeqia  sp!,{r4-r6,pc}    ;if w1 == w2, return val = 0

	;We detected a mismatch in the current pair of words from the strings.
	;But in which byte position within the words did the mismatch occur?

FindMismatch
	add      r3,r0,r4          ;restore value w1

NextByte
	and      r0,r3,#0xff       ;b1 = w1 & 0xff (isolate byte)
	and      r2,r4,#0xff       ;b2 = w2 & 0xff
	subs     r0,r0,r2          ;return val = b1 - b2 ?

	ldmneia  sp!,{r4-r6,pc}    ;if val != 0, return to caller

	mov      r3,r3,lsr #8      ;w1 >>= 8 (position next byte)
	mov      r4,r4,lsr #8      ;w2 >>= 8
	b        NextByte          ;if b1 != b2, loop again
}

