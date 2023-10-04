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
                            106 .macro DefineEntity _cpms, _ptr, _type, _color, _x, _y, _w, _h, _vxh, _vxl _vyh, _vyl, _sprite, _address, _p_address, _on_platform, _orientation, _anim_ptr, _anim_status
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
                            126     .db _on_platform
                            127     .db _orientation ;; 0: right, 1:left
                            128     .db #0           ;; dashing
                            129     .dw _anim_ptr
                            130     .db _anim_status
                            131     .db #1           ;; moved 1 default
                            132 .endm
                            133 
                            134 ;;==============================================================================================================================
                            135 ;;==============================================================================================================================
                            136 ;;  MACRO FOR ENUM DEFINITIONS
                            137 ;;==============================================================================================================================
                            138 ;;==============================================================================================================================
                            139 .mdelete DefEnum
                            140 .macro DefEnum _name
                            141     _name'_offset = 0
                            142 .endm
                            143 
                            144 ;;  Define enumeration element for an enumeration name.
                            145 .mdelete Enum
                            146 .macro Enum _enumname, _element
                            147     _enumname'_'_element = _enumname'_offset
                            148     _enumname'_offset = _enumname'_offset + 1
                            149 .endm
                            150 
                            151 ;;==============================================================================================================================
                            152 ;;==============================================================================================================================
                            153 ;;  DEFINE LINKED LIST STRUCTURE
                            154 ;;==============================================================================================================================
                            155 ;;==============================================================================================================================
                            156 
                            157 ;;  Defines the structure for a basic memory manager.
                            158 .mdelete DefineBasicStructureArray_Size
                            159 .macro DefineBasicStructureArray_Size _Tname, _N, _ComponentSize
                            160     _Tname'_array::
                            161         .ds _N * _ComponentSize
                            162 .endm
                            163 
                            164 ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
ASxxxx Assembler V02.00 + NoICE + SDCC mods  (Zilog Z80 / Hitachi HD64180), page 5.
Hexadecimal [16-Bits]



                            165 ;;  Defines the structure of the entity array.
                            166 .mdelete DefineComponentArrayStructure_Size
                            167 .macro DefineComponentArrayStructure_Size _Tname, _N, _ComponentSize
                            168     _Tname'_num:         .db 0
                            169     _Tname'_list:        .dw nullptr
                            170     _Tname'_free_list:   .dw _Tname'_array
                            171     _Tname'_array::
                            172         .ds _N * _ComponentSize
                            173 .endm
                            174 
                            175 
                            176 ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
                            177 ;;  Defines the structure for the component handler.
                            178 .mdelete DefineComponentPointerTable
                            179 .macro DefineComponentPointerTable _Tname, _N_Cmps, _N
                            180     _c = 0
                            181     ;;  Array containing pointers to component pointer arrays.
                            182     _Tname'_access_table::
                            183     .rept _N_Cmps
                            184         DefineComponentPointerAccessTable _Tname, \_c, _N, _N_Cmps
                            185         _c = _c + 1
                            186     .endm
                            187     ;;  Zero-fill the component array with two additional words for the
                            188     ;;  next free position and a null pointer fot he end of the array.
                            189     _Tname'_components::
                            190    .rept _N_Cmps
                            191         DefineComponentArray _N
                            192         .dw 0x0000
                            193         .dw 0x0000
                            194     .endm
                            195 .endm
                            196 
                            197 ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
                            198 ;;  Defines the pointers of the componente array pointer access table.
                            199 .mdelete DefineComponentPointerAccessTable
                            200 .macro DefineComponentPointerAccessTable _Tname, _suf, _N, _N_Cmps
                            201     _Tname'_components'_suf'_ptr_pend::    .dw . + 2*_N_Cmps+ + _suf*2*_N + 2*_suf
                            202 .endm
                            203 
                            204 ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
                            205 ;;  Zero-pad an array of size n.
                            206 .mdelete DefineComponentArray
                            207 .macro DefineComponentArray _N
                            208     .rept _N
                            209         .dw 0x0000
                            210     .endm
                            211 .endm
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
