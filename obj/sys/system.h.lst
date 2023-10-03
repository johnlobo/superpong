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
                             19 .module system_system
                             20 
                             21 
                             22 ;;===============================================================================
                             23 ;; PUBLIC VARIABLES
                             24 ;;===============================================================================
                             25 .globl nInterrupt
                             26 
                             27 ;;===============================================================================
                             28 ;; PUBLIC MACROS
                             29 ;;===============================================================================
                             30 .mdelete m_inc_nInterrupt
                             31 .macro m_inc_nInterrupt
                             32     ld a, (nInterrupt)
                             33     inc a
                             34     ld (nInterrupt), a 
                             35 .endm
                             36 
                             37 .mdelete m_reset_nInterrupt
                             38 .macro m_reset_nInterrupt
                             39     xor a
                             40     ld (nInterrupt), a 
                             41 .endm
                             42 
                             43 
                             44 ;;===============================================================================
                             45 ;; PUBLIC METHODS
                             46 ;;===============================================================================
                             47 .globl sys_system_disable_firmware
