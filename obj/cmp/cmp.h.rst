ASxxxx Assembler V02.00 + NoICE + SDCC mods  (Zilog Z80 / Hitachi HD64180), page 1.
Hexadecimal [16-Bits]



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
ASxxxx Assembler V02.00 + NoICE + SDCC mods  (Zilog Z80 / Hitachi HD64180), page 2.
Hexadecimal [16-Bits]



                             17 .include "macros.h.s" 
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
                            106 .macro DefineEntity _cpms, _ptr, _type, _color, _x, _y, _w, _h, _vxh, _vxl _vyh, _vyl, _sprite, _address, _p_address, _collsion_callback, _ai_callback
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
                            128     .db #0
                            129     .dw _ai_callback
                            130     .db #1           ;; moved 1 default
                            131 .endm
                            132 
                            133 ;;==============================================================================================================================
                            134 ;;==============================================================================================================================
                            135 ;;  MACRO FOR ENUM DEFINITIONS
                            136 ;;==============================================================================================================================
                            137 ;;==============================================================================================================================
                            138 .mdelete DefEnum
                            139 .macro DefEnum _name
                            140     _name'_offset = 0
                            141 .endm
                            142 
                            143 ;;  Define enumeration element for an enumeration name.
                            144 .mdelete Enum
                            145 .macro Enum _enumname, _element
                            146     _enumname'_'_element = _enumname'_offset
                            147     _enumname'_offset = _enumname'_offset + 1
                            148 .endm
                            149 
                            150 ;;==============================================================================================================================
                            151 ;;==============================================================================================================================
                            152 ;;  DEFINE LINKED LIST STRUCTURE
                            153 ;;==============================================================================================================================
                            154 ;;==============================================================================================================================
                            155 
                            156 ;;  Defines the structure for a basic memory manager.
                            157 .mdelete DefineBasicStructureArray_Size
                            158 .macro DefineBasicStructureArray_Size _Tname, _N, _ComponentSize
                            159     _Tname'_array::
                            160         .ds _N * _ComponentSize
                            161 .endm
                            162 
                            163 ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
                            164 ;;  Defines the structure of the entity array.
ASxxxx Assembler V02.00 + NoICE + SDCC mods  (Zilog Z80 / Hitachi HD64180), page 5.
Hexadecimal [16-Bits]



                            165 .mdelete DefineComponentArrayStructure_Size
                            166 .macro DefineComponentArrayStructure_Size _Tname, _N, _ComponentSize
                            167     _Tname'_num::         .db 0
                            168     _Tname'_list::        .dw nullptr
                            169     _Tname'_free_list::   .dw _Tname'_array
                            170     _Tname'_array::
                            171         .ds _N * _ComponentSize
                            172 .endm
                            173 
                            174 
                            175 ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
                            176 ;;  Defines the structure for the component handler.
                            177 .mdelete DefineComponentPointerTable
                            178 .macro DefineComponentPointerTable _Tname, _N_Cmps, _N
                            179     _c = 0
                            180     ;;  Array containing pointers to component pointer arrays.
                            181     _Tname'_access_table::
                            182     .rept _N_Cmps
                            183         DefineComponentPointerAccessTable _Tname, \_c, _N, _N_Cmps
                            184         _c = _c + 1
                            185     .endm
                            186     ;;  Zero-fill the component array with two additional words for the
                            187     ;;  next free position and a null pointer fot he end of the array.
                            188     _Tname'_components::
                            189    .rept _N_Cmps
                            190         DefineComponentArray _N
                            191         .dw 0x0000
                            192         .dw 0x0000
                            193     .endm
                            194 .endm
                            195 
                            196 ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
                            197 ;;  Defines the pointers of the componente array pointer access table.
                            198 .mdelete DefineComponentPointerAccessTable
                            199 .macro DefineComponentPointerAccessTable _Tname, _suf, _N, _N_Cmps
                            200     _Tname'_components'_suf'_ptr_pend::    .dw . + 2*_N_Cmps+ + _suf*2*_N + 2*_suf
                            201 .endm
                            202 
                            203 ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
                            204 ;;  Zero-pad an array of size n.
                            205 .mdelete DefineComponentArray
                            206 .macro DefineComponentArray _N
                            207     .rept _N
                            208         .dw 0x0000
                            209     .endm
                            210 .endm
                            211 
                            212 ;; WinAPE special BRK instruction
                            213 ;; - more info at http://www.winape.net/help/debug.html
                            214 .mdelete BREAKPOINT
                            215 .macro BREAKPOINT
                            216   .db #0xed, #0xff
                            217 .endm
ASxxxx Assembler V02.00 + NoICE + SDCC mods  (Zilog Z80 / Hitachi HD64180), page 6.
Hexadecimal [16-Bits]



                             18 
                             19 ;;===============================================================================
                             20 ;; CARD DEFINITION MACRO
                             21 ;;===============================================================================
                             22 .macro DefineCard _cpms, _status, _sprite, _name, _rarity, _type, _energy, _description, _damage, _block, _vulnerable, _weak, _strengh, _exhaust, _add_card, _execute_routine
                             23     .db _cpms
                             24     .db _status
                             25     .dw _sprite
                             26     .db _type
                             27     .db _energy
                             28     .asciz "_description"
                             29     .db _damage
                             30     .db _block
                             31     .db _vulnerable
                             32     .db _weak
                             33     .db _strengh
                             34     .db _exhaust
                             35     .dw _execute_routine
                             36 .endm
                             37 
                             38 ;;===============================================================================
                             39 ;; CARD SCTRUCTURE CREATION
                             40 ;;===============================================================================
   0000                      41 BeginStruct c
                     0000     1     c_offset = 0
   0000                      42 Field c, cpms , 1
                     0000     1     c_cpms = c_offset
                     0001     2     c_offset = c_offset + 1
   0000                      43 Field c, status , 1
                     0001     1     c_status = c_offset
                     0002     2     c_offset = c_offset + 1
   0000                      44 Field c, sprite , 2
                     0002     1     c_sprite = c_offset
                     0004     2     c_offset = c_offset + 2
   0000                      45 Field c, type , 1
                     0004     1     c_type = c_offset
                     0005     2     c_offset = c_offset + 1
   0000                      46 Field c, energy , 1
                     0005     1     c_energy = c_offset
                     0006     2     c_offset = c_offset + 1
   0000                      47 Field c, description , 31
                     0006     1     c_description = c_offset
                     0025     2     c_offset = c_offset + 31
   0000                      48 Field c, damage , 1
                     0025     1     c_damage = c_offset
                     0026     2     c_offset = c_offset + 1
   0000                      49 Field c, block , 1
                     0026     1     c_block = c_offset
                     0027     2     c_offset = c_offset + 1
   0000                      50 Field c, vulnerable , 1
                     0027     1     c_vulnerable = c_offset
                     0028     2     c_offset = c_offset + 1
   0000                      51 Field c, weak , 1
                     0028     1     c_weak = c_offset
                     0029     2     c_offset = c_offset + 1
ASxxxx Assembler V02.00 + NoICE + SDCC mods  (Zilog Z80 / Hitachi HD64180), page 7.
Hexadecimal [16-Bits]



   0000                      52 Field c, strengh , 1
                     0029     1     c_strengh = c_offset
                     002A     2     c_offset = c_offset + 1
   0000                      53 Field c, exhaust , 1
                     002A     1     c_exhaust = c_offset
                     002B     2     c_offset = c_offset + 1
   0000                      54 Field c, execute_routine, 2
                     002B     1     c_execute_routine = c_offset
                     002D     2     c_offset = c_offset + 2
   0000                      55 EndStruct c
                     002D     1     sizeof_c = c_offset
                             56 
                             57 ;;===============================================================================
                             58 ;; POINTER TO CARD STRUCTURE CREATION
                             59 ;;===============================================================================
   0000                      60 BeginStruct e
                     0000     1     e_offset = 0
   0000                      61 Field e, cpms , 1
                     0000     1     e_cpms = e_offset
                     0001     2     e_offset = e_offset + 1
   0000                      62 Field e, status , 1
                     0001     1     e_status = e_offset
                     0002     2     e_offset = e_offset + 1
   0000                      63 Field e, p , 2
                     0002     1     e_p = e_offset
                     0004     2     e_offset = e_offset + 2
   0000                      64 EndStruct e
                     0004     1     sizeof_e = e_offset
