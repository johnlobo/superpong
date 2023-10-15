ASxxxx Assembler V02.00 + NoICE + SDCC mods  (Zilog Z80 / Hitachi HD64180 / ZX-Next / eZ80), page 1.
Hexadecimal [24-Bits]



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
                                     19 .module messages_system
                                     20 
                                     21 ;;------------------------------------------------------------------------------
                                     22 ;; Global constants
                                     23 ;;------------------------------------------------------------------------------
                                     24 
                           00001E    25 MINIMUM_WINDOW_WIDTH = 30
                                     26 
                                     27 ;;------------------------------------------------------------------------------
                                     28 ;; Global variables
                                     29 ;;------------------------------------------------------------------------------
                                     30 
                                     31 ;;------------------------------------------------------------------------------
                                     32 ;; Global routines
                                     33 ;;------------------------------------------------------------------------------
                                     34 
                                     35 .globl sys_messages_load_window_data
                                     36 .globl sys_messages_draw_window
                                     37 .globl sys_messages_show
                                     38 .globl sys_messages_draw_box
                                     39 .globl sys_messages_restore_message_background
