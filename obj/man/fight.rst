ASxxxx Assembler V02.00 + NoICE + SDCC mods  (Zilog Z80 / Hitachi HD64180), page 1.
Hexadecimal [16-Bits]



                              1 ;;-----------------------------LICENSE NOTICE------------------------------------
                              2 ;;  This program is free software: you can redistribute it and/or modify
                              3 ;;  it under the terms of the GNU Lesser General Public License as published by
                              4 ;;  the Free Software Foundation, either version 3 of the License, or
                              5 ;;  (at your option) any later version.
                              6 ;;
                              7 ;;  This program is distributed in the hope that it will be useful,
                              8 ;;  but WITHOUT ANY WARRANTY; without even the implied warranty of
                              9 ;;  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
                             10 ;;  GNU Lesser General Public License for more details.
                             11 ;;
                             12 ;;  You should have received a copy of the GNU Lesser General Public License
                             13 ;;  along with this program.  If not, see <http://www.gnu.org/licenses/>.
                             14 ;;-------------------------------------------------------------------------------
                             15 
                             16 
                             17 
                             18 .module fight_manager
                             19 
                             20 ;;
                             21 ;; Start of _DATA area 
                             22 ;;  SDCC requires at least _DATA and _CODE areas to be declared, but you may use
                             23 ;;  any one of them for any purpose. Usually, compiler puts _DATA area contents
                             24 ;;  right after _CODE area contents.
                             25 ;;
                             26 .area _DATA
                             27 
   2441                      28 first_lvl::
   2441 54 48 45 20 44 4F    29 .asciz "THE DOOR                     "  ;; Name
        4F 52 20 20 20 20
        20 20 20 20 20 20
        20 20 20 20 20 20
        20 20 20 20 20 00
   245F 05                   30 .db 5                                   ;; Rounds
   2460 00                   31 .db 0                                   ;; curent round
   2461 01 02 03             32 .db 1, 2, 3
   2464 01 02 03             33 .db 1, 2, 3
   2467 01 02 03             34 .db 1, 2, 3
   246A 01 02 03             35 .db 1, 2, 3
   246D 01 02 03             36 .db 1, 2, 3
                             37 
                             38 ;;
                             39 ;; Start of _CODE area
                             40 ;; 
                             41 .area _CODE
                             42 
                             43 ;;-----------------------------------------------------------------
                             44 ;;
                             45 ;; man_fight_init
                             46 ;;
                             47 ;;  
                             48 ;;  Input: 
                             49 ;;  Output: 
                             50 ;;  Modified: AF, BC, DE, HL
                             51 ;;
ASxxxx Assembler V02.00 + NoICE + SDCC mods  (Zilog Z80 / Hitachi HD64180), page 2.
Hexadecimal [16-Bits]



   05EA                      52 man_fight_init::
   05EA C9            [10]   53     ret
                             54 
                             55 
                             56 ;;-----------------------------------------------------------------
                             57 ;;
                             58 ;; man_fight_update
                             59 ;;
                             60 ;;  
                             61 ;;  Input: 
                             62 ;;  Output: 
                             63 ;;  Modified: AF, BC, DE, HL
                             64 ;;
   05EB                      65 man_fight_update::
   05EB C9            [10]   66     ret
