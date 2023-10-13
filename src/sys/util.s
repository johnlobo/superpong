;;-----------------------------LICENSE NOTICE------------------------------------
;;  This file is part of CPCtelera: An Amstrad CPC Game Engine 
;;  Copyright (C) 2018 ronaldo / Fremos / Cheesetea / ByteRealms (@FranGallegoBR)
;;
;;  This program is free software: you can redistribute it and/or modify
;;  it under the terms of the GNU Lesser General Public License as published by
;;  the Free Software Foundation, either version 3 of the License, or
;;  (at your option) any later version.
;;
;;  This program is distributed in the hope that it will be useful,
;;  but WITHOUT ANY WARRANTY; without even the implied warranty of
;;  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
;;  GNU Lesser General Public License for more details.
;;
;;  You should have received a copy of the GNU Lesser General Public License
;;  along with this program.  If not, see <http://www.gnu.org/licenses/>.
;;-------------------------------------------------------------------------------

.module sys_util

.include "../common.h.s"
;;
;; Start of _DATA area 
;;  SDCC requires at least _DATA and _CODE areas to be declared, but you may use
;;  any one of them for any purpose. Usually, compiler puts _DATA area contents
;;  right after _CODE area contents.
;;
.area _DATA


string_buffer:: .asciz "          "


;;
;; Start of _CODE area
;; 
.area _CODE

;;-----------------------------------------------------------------;; 
;;  sys_util_h_times_e
;;
;; Inputs:
;;   H and E
;; Outputs:
;;   HL is the product
;;   D is 0
;;   A,E,B,C are preserved
;; 36 bytes
;; min: 190cc
;; max: 242cc
;; avg: 216cc
;; Credits:
;;  Z80Heaven (http://z80-heaven.wikidot.com/advanced-math#toc9)

sys_util_h_times_e::
  ld d,#0
  ld l,d
  sla h 
  jr nc,.+3 
  ld l,e
  add hl,hl 
  jr nc,.+3 
  add hl,de
  add hl,hl 
  jr nc,.+3 
  add hl,de
  add hl,hl 
  jr nc,.+3 
  add hl,de
  add hl,hl 
  jr nc,.+3 
  add hl,de
  add hl,hl 
  jr nc,.+3 
  add hl,de
  add hl,hl 
  jr nc,.+3 
  add hl,de
  add hl,hl 
  ret nc 
  add hl,de
  ret

;;-----------------------------------------------------------------;; 
;;  sys_util_hl_div_c
;;
;;Inputs:
;;     HL is the numerator
;;     C is the denominator
;;Outputs:
;;     A is the remainder
;;     B is 0
;;     C is not changed
;;     DE is not changed
;;     HL is the quotient
;;
sys_util_hl_div_c::
       ld b,#16
       xor a
         add hl,hl
         rla
         cp c
         jr c,.+4
           inc l
           sub c
         djnz .-7
       ret

;;-----------------------------------------------------------------
;;
;; sys_util_absHL
;;
;;  
;;  Input:  hl: number
;;  Output: hl: absolut value of number
;;  Destroyed: af
;;
;;  Cemetech code (https://learn.cemetech.net/index.php?title=Z80:Math_Routines#absHL)
;;
sys_util_absHL::
  bit #7,h
  ret z
  xor a
  sub l
  ld l,a
  sbc a,a
  sub h
  ld h,a
  ret

;;-----------------------------------------------------------------
;;
;; sys_util_BCD_GetEnd
;;
;;  
;;  Input:  b: number of bytes of the bcd number
;;          de: source for the first bcd bnumber
;;          hl: source for the second bcd number
;;  Output: 
;;  Destroyed: af, bc,de, hl
;;
;;  Chibi Akumas BCD code (https://www.chibiakumas.com/z80/advanced.php#LessonA1)
;;
sys_util_BCD_GetEnd::
;Some of our commands need to start from the most significant byte
;This will shift HL and DE along b bytes
	push bc
	ld c,b	;We want to add BC, but we need to add one less than the number of bytes
	dec c
	ld b,#0
	add hl,bc
	ex de, hl	;We've done HL, but we also want to do DE
	add hl,bc
	ex de, hl
	pop bc
	ret

;;-----------------------------------------------------------------
;;
;; BCD_Add
;;
;;   Add two BCD numbers
;;  Input:  hl: Number to add to de
;;          de: Number to store the sum 
;;  Output: 
;;  Destroyed: af, bc,de, hl
;;
;;  Chibi Akumas BCD code (https://www.chibiakumas.com/z80/advanced.php#LessonA1)
;;
sys_util_BCD_Add::
    or a
BCD_Add_Again:
    ld a, (de)
    adc (hl)
    daa
    ld (de), a
    inc de
    inc hl
    djnz BCD_Add_Again
    ret
  
;;-----------------------------------------------------------------
;;
;; sys_util_BCD_Compare
;;
;;  Compare two BCD numbers
;;  Input:  hl: BCD Number 1
;;          de: BCD Number 2
;;  Output: 
;;  Destroyed: af, bc,de, hl
;;
;;  Chibi Akumas BCD code (https://www.chibiakumas.com/z80/advanced.php#LessonA1)
;;
sys_util_BCD_Compare::
  ld b, #SCORE_NUM_BYTES
  call sys_util_BCD_GetEnd
BCD_cp_direct:
  ld a, (de)
  cp (hl)
  ret c
  ret nz
  dec de
  dec hl
  djnz BCD_cp_direct
  or a                    ;; Clear carry
  ret

;;-----------------------------------------------------------------
;;
;; sys_util_get_random_number
;;
;;  Returns a random number between 0 and <end>
;;  Input:  a: <end>
;;  Output: a: random number
;;  Destroyed: af, bc,de, hl

sys_util_get_random_number::
  ld (#random_max_number), a
  call cpct_getRandom_mxor_u8_asm
  ld a, l                             ;; Calculates a pseudo modulus of max number
  ld h,#0                             ;; Load hl with the random number
random_max_number = .+1
  ld c, #0                            ;; Load c with the max number
  ld b, #0
_random_mod_loop:
  or a                                ;; ??
  sbc hl,bc                           ;; hl = hl - bc
  jp p, _random_mod_loop              ;; Jump back if hl > 0
  add hl,bc                           ;; Adds MAX_MODEL_CARD to hl back to get back to positive values
  ld a,l                              ;; loads the normalized random number in a
ret

;;-----------------------------------------------------------------
;;
;; sys_util_delay
;;
;;  Waits a determined number of frames 
;;  Input:  b: number of frames
;;  Output: 
;;  Destroyed: af, bc
;;
sys_util_delay::
  push bc
  call cpct_waitVSYNCStart_asm
  pop bc
  djnz sys_util_delay
  ret


;;-----------------------------------------------------------------
;;
;; sys_util_negHL
;;
;;  Negates hl
;;  input: hl
;;  ouput: hl negated
;;  destroys: a
;;
;; WikiTI code (https://wikiti.brandonw.net/index.php?title=Z80_Routines:Math:Signed_Math)
sys_util_negHL::
	xor a
	sub l
	ld l,a
	sbc a,a
	sub h
	ld h,a
	ret

;;-----------------------------------------------------------------
;;
;; sys_util_hl_divided_d
;;
;;  Divides hl by d, and leaves the result in hl
;;  input:  hl: dividend
;;          d: divisor
;;  ouput:  hl: result
;;  destroys: af, de, bc, hl 
;;
;; code by Jonathan Cauldwell (https://chuntey.wordpress.com/category/z80-assembly/)
sys_util_hl_divided_d::
  ld b,#8              ; bits to check.
  ld a,d              ; number by which to divide.
idiv3:  
  rla                 ; check leftmost bit.
  jr c,idiv2          ; no more shifts required.
  inc b               ; extra shift needed.
  cp h
  jr nc,idiv2
  jp idiv3            ; repeat.

idiv2:  
  xor a
  ld e,a
  ld c,a              ; result.
idiv1:  
  sbc hl,de           ; do subtraction.
  jr nc,idiv0         ; no carry, keep the result.
  add hl,de           ; restore original value of hl.
idiv0: 
  ccf                 ; reverse carry bit.
  rl c                ; rotate in to ac.
  rla
  rr d                ; divide de by 2.
  rr e
  djnz idiv1          ; repeat.
  ld h,a              ; copy result to hl.
  ld l,c
  ret

;;-----------------------------------------------------------------
;;
;; sys_util_sqr_hl
;;
;;  Calculates de square root of hl in a
;;  fast 16 bit isqrt by John Metcalf
;;  92 bytes, 344-379 cycles (average 362)
;;  v2 - saved 3 cycles with a tweak suggested by Russ McNulty
;;  input: hl
;;  ouput: a
;;  destroys: de, hl 
;;
;; code by John Metcalf (https://github.com/impomatic/z80snippets/blob/master/fastisqr.asm)
sys_util_sqr_hl::

  ld a,h        ; 4
  ld de,#0x0B0C0  ; 10
  add a,e       ; 4
  jr c,sq7      ; 12 / 7
  ld a,h        ; 4
  ld d,#0x0F0     ; 7
sq7:

; ----------

  add a,d       ; 4
  jr nc,sq6     ; 12 / 7
  res 5,d       ; 8
  .db #254        ; 7
sq6:
  sub d         ; 4
  sra d         ; 8

; ----------

  set 2,d       ; 8
  add a,d       ; 4
  jr nc,sq5     ; 12 / 7
  res 3,d       ; 8
  .db #254        ; 7
sq5:
  sub d         ; 4
  sra d         ; 8

; ----------

  inc d         ; 4
  add a,d       ; 4
  jr nc,sq4     ; 12 / 7
  res 1,d       ; 8
  .db #254        ; 7
sq4:
  sub d         ; 4
  sra d         ; 8
  ld h,a        ; 4

; ----------

  add hl,de     ; 11
  jr nc,sq3     ; 12 / 7
  ld e,#0x040     ; 7
  .db #210        ; 10
sq3:
  sbc hl,de     ; 15
  sra d         ; 8
  ld a,e        ; 4
  rra           ; 4

; ----------

  or #0x010       ; 7
  ld e,a        ; 4
  add hl,de     ; 11
  jr nc,sq2     ; 12 / 7
  and #0x0DF      ; 7
  .db #218        ; 10
sq2:
  sbc hl,de     ; 15
  sra d         ; 8
  rra           ; 4

; ----------

  or #0x04        ; 7
  ld e,a        ; 4
  add hl,de     ; 11
  jr nc,sq1     ; 12 / 7
  and #0x0F7      ; 7
  .db #218        ; 10
sq1:
  sbc hl,de     ; 15
  sra d         ; 8
  rra           ; 4

; ----------

  inc a         ; 4
  ld e,a        ; 4
  add hl,de     ; 11
  jr nc,sq0     ; 12 / 7
  and #0x0FD      ; 7
sq0:
  sra d         ; 8
  rra           ; 4
  cpl           ; 4

ret


i16 sine(i16 angle) {
     if (angle < 90) {
          return sine_table[angle];
     } else if (angle < 180) {
          return sine_table[180 - angle];
     } else if (angle < 270) {
          return -sine_table[angle - 180];
     } else {
          return -sine_table[360 - angle];
     }
}

i16 cosine(i16 angle) {
     if (angle <= 90)
          return (sine(90 - angle));
     else
          return (-sine(angle - 90));
}


sine_table::
    .db #0 
    .db #4,   #9,   #13,  #18,  #22,  #27,  #31,  #36,  #40,  #44
    .db #49,  #53,  #58,  #62,  #66,  #71,  #75,  #79,  #83,  #88
    .db #92,  #96,  #100, #104, #108, #112, #116, #120, #124, #128
    .db #132, #136, #139, #143, #147, #150, #154, #158, #161, #165
    .db #168, #171, #175, #178, #181, #184, #187, #190, #193, #196
    .db #199, #202, #204, #207, #210, #212, #215, #217, #219, #222
    .db #224, #226, #228, #230, #232, #234, #236, #237, #239, #241
    .db #242, #243, #245, #246, #247, #248, #249, #250, #251, #252
    .db #253, #254, #254, #255, #255, #255, #255, #255, #255, #255
