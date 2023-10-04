ASxxxx Assembler V02.00 + NoICE + SDCC mods  (Zilog Z80 / Hitachi HD64180), page 1.
Hexadecimal [16-Bits]



                              1 ;;-----------------------------LICENSE NOTICE------------------------------------
                              2 ;;  This file is part of CPCtelera: An Amstrad CPC Game Engine 
                              3 ;;  Copyright (C) 2018 ronaldo / Fremos / Cheesetea / ByteRealms (@FranGallegoBR)
                              4 ;;
                              5 ;;  This program is free software: you can redistribute it and/or modify
                              6 ;;  it under the terms of the GNU Lesser General Public License as published by
                              7 ;;  the Free Software Foundation, either version 3 of the License, or
                              8 ;;  (at your option) any later version.
                              9 ;;
                             10 ;;  This program is distributed in the hope that it will be useful,
                             11 ;;  but WITHOUT ANY WARRANTY; without even the implied warranty of
                             12 ;;  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
                             13 ;;  GNU Lesser General Public License for more details.
                             14 ;;
                             15 ;;  You should have received a copy of the GNU Lesser General Public License
                             16 ;;  along with this program.  If not, see <http://www.gnu.org/licenses/>.
                             17 ;;-------------------------------------------------------------------------------
                             18 
                             19 
                             20 ;;===============================================================================
                             21 ;; PUBLIC VARIABLES
                             22 ;;===============================================================================
                             23 .globl string_buffer
                             24 
                             25 ;;===============================================================================
                             26 ;; PUBLIC METHODS
                             27 ;;===============================================================================
                             28 .globl sys_util_h_times_e
                             29 .globl sys_util_hl_div_c
                             30 .globl sys_util_absHL
                             31 .globl sys_util_BCD_GetEnd
                             32 .globl sys_util_BCD_Add
                             33 .globl sys_util_BCD_Compare
                             34 .globl sys_util_get_random_number
                             35 .globl sys_util_delay
