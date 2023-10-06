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
                             19 .module game_manager
                             20 
ASxxxx Assembler V02.00 + NoICE + SDCC mods  (Zilog Z80 / Hitachi HD64180), page 2.
Hexadecimal [16-Bits]



                             21 .include "macros.h.s"
                              1 ;;-----------------------------LICENSE NOTICE------------------------------------
                              2 ;;
                              3 ;;  This program is free software: you can redistribute it and/or modify
                              4 ;;  it under the terms of the GNU Lesser General Public License as published by
                              5 ;;  the Free Software Foundation, either version 3 of the License, or
                              6 ;;  (at your option) any later version.
                              7 ;;
                              8 ;;  This program is distributed in the hope that it will be useful,
                              9 ;;  but WITHOUT ANY WARRANTY; without even the implied warranty of
                             10 ;;  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
                             11 ;;  GNU Lesser General Public License for more details.
                             12 ;;
                             13 ;;  You should have received a copy of the GNU Lesser General Public License
                             14 ;;  along with this program.  If not, see <http://www.gnu.org/licenses/>.
                             15 ;;-------------------------------------------------------------------------------
                             16 
                             17 
                             18 ;;===============================================================================
                             19 ;; DEFINED MACROS
                             20 ;;===============================================================================
                             21 .mdelete BeginStruct
                             22 .macro BeginStruct struct
                             23     struct'_offset = 0
                             24 .endm
                             25 
                             26 .mdelete Field
                             27 .macro Field struct, field, size
                             28     struct'_'field = struct'_offset
                             29     struct'_offset = struct'_offset + size
                             30 .endm
                             31 
                             32 .mdelete EndStruct
                             33 .macro EndStruct struct
                             34     sizeof_'struct = struct'_offset
                             35 .endm
                             36 
                             37 ;;===============================================================================
                             38 ;; Macro
                             39 ;;
                             40 ;; Macro modified from cpctelera cpctm_screenPtr_asm
                             41 ;;===============================================================================
                             42 
                             43 .mdelete m_center_screen_ptr 
                             44 .macro m_center_screen_ptr REG16, VMEM, Y, WIDTH
                             45    ld REG16, #VMEM + 80 * (Y / 8) + 2048 * (Y & 7) + ((80 - WIDTH)/2)   ;; [3] REG16 = screenPtr
                             46 .endm
                             47 
                             48 ;;===============================================================================
                             49 ;; MACRO
                             50 ;;===============================================================================
                             51 .mdelete ld_de_backbuffer
                             52 .macro ld_de_backbuffer
                             53    ld   a, (sys_render_back_buffer)          ;; DE = Pointer to start of the screen
                             54    ld   d, a
ASxxxx Assembler V02.00 + NoICE + SDCC mods  (Zilog Z80 / Hitachi HD64180), page 3.
Hexadecimal [16-Bits]



                             55    ld   e, #00
                             56 .endm
                             57 
                             58 .mdelete ld_de_frontbuffer
                             59 .macro ld_de_frontbuffer
                             60    ld   a, (sys_render_front_buffer)         ;; DE = Pointer to start of the screen
                             61    ld   d, a
                             62    ld   e, #00
                             63 .endm
                             64 
                             65 .mdelete m_screenPtr_backbuffer
                             66 .macro m_screenPtr_backbuffer X, Y
                             67    push hl
                             68    ld de, #(80 * (Y / 8) + 2048 * (Y & 7) + X)
                             69    ld a, (sys_render_back_buffer)
                             70    ld h, a
                             71    ld l, #0         
                             72    add hl, de
                             73    ex de, hl
                             74    pop hl
                             75 .endm
                             76 
                             77 .mdelete m_screenPtr_frontbuffer
                             78 .macro m_screenPtr_frontbuffer X, Y
                             79    push hl
                             80    ld de, #(80 * (Y / 8) + 2048 * (Y & 7) + X)
                             81    ld a, (sys_render_front_buffer)
                             82    ld h, a
                             83    ld l, #0         
                             84    add hl, de
                             85    ex de, hl
                             86    pop hl
                             87 .endm
                             88 
                             89 
                             90 .mdelete m_draw_blank_small_number
                             91 .macro m_draw_blank_small_number
                             92    push de
                             93    push hl
                             94    ld c, #6
                             95    ld b, #5
                             96    ld a, #0                         ;; Patern of solid box
                             97    call cpct_drawSolidBox_asm
                             98    pop hl
                             99    pop de
                            100 .endm
                            101 
                            102 ;;===============================================================================
                            103 ;; ENTITY DEFINITION MACRO
                            104 ;;===============================================================================
                            105 .mdelete DefineEntity
                            106 .macro DefineEntity _cpms, _ptr, _type, _color, _x, _y, _w, _h, _vxh, _vxl _vyh, _vyl, _sprite, _address, _p_address, _collsion_callback
                            107     .dw _ptr
                            108     .db _cpms
                            109     .db _type
ASxxxx Assembler V02.00 + NoICE + SDCC mods  (Zilog Z80 / Hitachi HD64180), page 4.
Hexadecimal [16-Bits]



                            110     .db _color
                            111     .dw _x
                            112     .dw _y
                            113     .db _w
                            114     .db _h
                            115     .db _x+_w
                            116     .db _y+_h
                            117     .db #0
                            118     .db #0
                            119     .db _vxh
                            120     .db _vxl
                            121     .db _vyh
                            122     .db _vyl
                            123     .dw _sprite
                            124     .dw _address
                            125     .dw _p_address
                            126     .db #0
                            127     .dw _collsion_callback
                            128     .db #1           ;; moved 1 default
                            129 .endm
                            130 
                            131 ;;==============================================================================================================================
                            132 ;;==============================================================================================================================
                            133 ;;  MACRO FOR ENUM DEFINITIONS
                            134 ;;==============================================================================================================================
                            135 ;;==============================================================================================================================
                            136 .mdelete DefEnum
                            137 .macro DefEnum _name
                            138     _name'_offset = 0
                            139 .endm
                            140 
                            141 ;;  Define enumeration element for an enumeration name.
                            142 .mdelete Enum
                            143 .macro Enum _enumname, _element
                            144     _enumname'_'_element = _enumname'_offset
                            145     _enumname'_offset = _enumname'_offset + 1
                            146 .endm
                            147 
                            148 ;;==============================================================================================================================
                            149 ;;==============================================================================================================================
                            150 ;;  DEFINE LINKED LIST STRUCTURE
                            151 ;;==============================================================================================================================
                            152 ;;==============================================================================================================================
                            153 
                            154 ;;  Defines the structure for a basic memory manager.
                            155 .mdelete DefineBasicStructureArray_Size
                            156 .macro DefineBasicStructureArray_Size _Tname, _N, _ComponentSize
                            157     _Tname'_array::
                            158         .ds _N * _ComponentSize
                            159 .endm
                            160 
                            161 ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
                            162 ;;  Defines the structure of the entity array.
                            163 .mdelete DefineComponentArrayStructure_Size
                            164 .macro DefineComponentArrayStructure_Size _Tname, _N, _ComponentSize
ASxxxx Assembler V02.00 + NoICE + SDCC mods  (Zilog Z80 / Hitachi HD64180), page 5.
Hexadecimal [16-Bits]



                            165     _Tname'_num::         .db 0
                            166     _Tname'_list::        .dw nullptr
                            167     _Tname'_free_list::   .dw _Tname'_array
                            168     _Tname'_array::
                            169         .ds _N * _ComponentSize
                            170 .endm
                            171 
                            172 
                            173 ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
                            174 ;;  Defines the structure for the component handler.
                            175 .mdelete DefineComponentPointerTable
                            176 .macro DefineComponentPointerTable _Tname, _N_Cmps, _N
                            177     _c = 0
                            178     ;;  Array containing pointers to component pointer arrays.
                            179     _Tname'_access_table::
                            180     .rept _N_Cmps
                            181         DefineComponentPointerAccessTable _Tname, \_c, _N, _N_Cmps
                            182         _c = _c + 1
                            183     .endm
                            184     ;;  Zero-fill the component array with two additional words for the
                            185     ;;  next free position and a null pointer fot he end of the array.
                            186     _Tname'_components::
                            187    .rept _N_Cmps
                            188         DefineComponentArray _N
                            189         .dw 0x0000
                            190         .dw 0x0000
                            191     .endm
                            192 .endm
                            193 
                            194 ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
                            195 ;;  Defines the pointers of the componente array pointer access table.
                            196 .mdelete DefineComponentPointerAccessTable
                            197 .macro DefineComponentPointerAccessTable _Tname, _suf, _N, _N_Cmps
                            198     _Tname'_components'_suf'_ptr_pend::    .dw . + 2*_N_Cmps+ + _suf*2*_N + 2*_suf
                            199 .endm
                            200 
                            201 ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
                            202 ;;  Zero-pad an array of size n.
                            203 .mdelete DefineComponentArray
                            204 .macro DefineComponentArray _N
                            205     .rept _N
                            206         .dw 0x0000
                            207     .endm
                            208 .endm
ASxxxx Assembler V02.00 + NoICE + SDCC mods  (Zilog Z80 / Hitachi HD64180), page 6.
Hexadecimal [16-Bits]



                             22 
                             23 ;;===============================================================================
                             24 ;; PUBLIC VARIABLES
                             25 ;;===============================================================================
                             26 
                             27 ;;===============================================================================
                             28 ;; PUBLIC CONSTANTS
                             29 ;;===============================================================================
                     0000    30 GAME_LOOP == 0
                     0001    31 MAIN_MENU == 1
                     0002    32 MAP_LOAD == 2
                     0003    33 PAUSE_MENU == 3
                     0004    34 GAME_OVER == 4
                     0005    35 VICTORY == 5
                             36 
                             37 ;;===============================================================================
                             38 ;; PUBLIC METHODS
                             39 ;;===============================================================================
                             40 .globl man_game_init
                             41 .globl man_game_update
                             42 .globl man_game_get_state
                             43 .globl man_game_set_state
