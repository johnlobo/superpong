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
                             19 ;;------------------------------------------------------------------------------
                             20 ;; Global variables
                             21 ;;------------------------------------------------------------------------------
                             22 ;;.globl sys_input_key_actions
                             23 
                             24 ;;------------------------------------------------------------------------------
                             25 ;; GLOBAL CONSTANTS
                             26 ;;------------------------------------------------------------------------------
                             27 
                             28 
                             29 ;;------------------------------------------------------------------------------
                             30 ;; Global routines
                             31 ;;------------------------------------------------------------------------------
                             32 .globl sys_input_init
                             33 .globl sys_input_clean_buffer
                             34 .globl sys_input_wait4anykey
                             35 .globl sys_input_waitKeyPressed
                             36 .globl sys_input_getKeyPressed
                             37 
                             38 .globl sys_input_main_menu_update
                             39 .globl sys_input_player_update
                             40 .globl sys_input_player_left
                             41 .globl sys_input_player_right
                             42 
