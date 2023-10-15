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
                             19 .module sys_util
                             20 
ASxxxx Assembler V02.00 + NoICE + SDCC mods  (Zilog Z80 / Hitachi HD64180), page 2.
Hexadecimal [16-Bits]



                             21 .include "../common.h.s"
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
ASxxxx Assembler V02.00 + NoICE + SDCC mods  (Zilog Z80 / Hitachi HD64180), page 3.
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
ASxxxx Assembler V02.00 + NoICE + SDCC mods  (Zilog Z80 / Hitachi HD64180), page 4.
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
ASxxxx Assembler V02.00 + NoICE + SDCC mods  (Zilog Z80 / Hitachi HD64180), page 5.
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
ASxxxx Assembler V02.00 + NoICE + SDCC mods  (Zilog Z80 / Hitachi HD64180), page 6.
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
ASxxxx Assembler V02.00 + NoICE + SDCC mods  (Zilog Z80 / Hitachi HD64180), page 7.
Hexadecimal [16-Bits]



                             18 
                             19 ;;===============================================================================
                             20 ;; SPRITES
                             21 ;;===============================================================================
                             22 .globl _g_palette
                             23 .globl _s_font_0
                             24 .globl _s_small_numbers_00
                             25 .globl _s_small_numbers_01
                             26 .globl _s_small_numbers_02
                             27 .globl _s_small_numbers_03
                             28 .globl _s_small_numbers_04
                             29 .globl _s_small_numbers_05
                             30 .globl _s_small_numbers_06
                             31 .globl _s_small_numbers_07
                             32 .globl _s_small_numbers_08
                             33 .globl _s_small_numbers_09
                             34 
                             35 
                             36 ;;===============================================================================
                             37 ;; CPCTELERA FUNCTIONS
                             38 ;;===============================================================================
                             39 .globl cpct_disableFirmware_asm
                             40 .globl cpct_getScreenPtr_asm
                             41 .globl cpct_drawSprite_asm
                             42 .globl cpct_setVideoMode_asm
                             43 .globl cpct_setPalette_asm
                             44 .globl cpct_setPALColour_asm
                             45 .globl cpct_memset_asm
                             46 .globl cpct_getScreenToSprite_asm
                             47 .globl cpct_scanKeyboard_asm
                             48 .globl cpct_scanKeyboard_if_asm
                             49 .globl cpct_isKeyPressed_asm
                             50 .globl cpct_waitHalts_asm
                             51 .globl cpct_drawSolidBox_asm
                             52 .globl cpct_getRandom_xsp40_u8_asm
                             53 .globl cpct_setSeed_xsp40_u8_asm
                             54 .globl cpct_isAnyKeyPressed_asm
                             55 .globl cpct_setInterruptHandler_asm
                             56 .globl cpct_waitVSYNC_asm
                             57 .globl cpct_drawSpriteBlended_asm
                             58 .globl _cpct_keyboardStatusBuffer
                             59 .globl cpct_memset_f64_asm
                             60 .globl cpct_getRandom_mxor_u8_asm
                             61 .globl cpct_waitVSYNCStart_asm
                             62 .globl cpct_setSeed_mxor_asm
                             63 .globl cpct_setVideoMemoryPage_asm
                             64 .globl cpct_etm_setDrawTilemap4x8_ag_asm
                             65 .globl cpct_etm_drawTilemap4x8_ag_asm
                             66 .globl cpct_etm_drawTileBox2x4_asm
                             67 .globl cpct_px2byteM0_asm
                             68 
                             69 
                             70 ;;===============================================================================
                             71 ;; DEFINED CONSTANTS
                             72 ;;===============================================================================
ASxxxx Assembler V02.00 + NoICE + SDCC mods  (Zilog Z80 / Hitachi HD64180), page 8.
Hexadecimal [16-Bits]



                             73 
                     0016    74 COF                     = #0x0016           ;; Coefficient of Friction
                     0024    75 GRAVITY                 = #0x0024           ;; Gravity
                             76 
                     0200    77 DASH_IMPULSE            = 0x0200
                     000A    78 DASH_TIMER              = 10
                             79 
                     0013    80 STEP_HORIZONTAL_SPEED       = 0x00013
                     0100    81 MAX_HORIZONTAL_SPEED_POS    = 0x0100
                     FF00    82 MAX_HORIZONTAL_SPEED_NEG    = 0xff00
                             83 
                     0026    84 STEP_VERTICAL_SPEED       = 0x0026
                     0200    85 MAX_VERTICAL_SPEED_POS    = 0x0200
                     FE00    86 MAX_VERTICAL_SPEED_NEG    = 0xfe00
                             87 
                     0016    88 STEP_HORIZONTAL_BALL_SPEED      = 0x0016
                     0100    89 MAX_HORIZONTAL_BALL_SPEED       = 0x0100
                             90 
                             91 
                     000A    92 MAX_ENTITIES = 10
                             93 
                             94 
                     0000    95 nullptr = 0x0000
                             96 
                             97 ;;==============================================================================================================================
                             98 ;;==============================================================================================================================
                             99 ;;  ENTITY TYPE MASKS AND BITS
                            100 ;;==============================================================================================================================
                            101 ;;==============================================================================================================================
                     0000   102 e_type_default              = 0x00
                     0001   103 e_type_player               = 0x01
                     0002   104 e_type_ball                 = 0x02
                     0004   105 e_type_wall                 = 0x04
                     0008   106 e_type_mob                  = 0x08
                     0010   107 e_type_shield               = 0x10
                     0020   108 e_type_dead                 = 0x20
                     00FF   109 e_type_invalid              = 0xff
                            110 
                            111 ;;===============================================================================
                            112 ;;COMPONENT TYPES
                            113 ;;===============================================================================
                     0000   114 e_cmp          = 0
                     0001   115 e_cmp_alive    = 0x01   ;;entidad renderizable
                     0002   116 e_cmp_render   = 0x02   ;;entidad renderizable
                     0004   117 e_cmp_physics  = 0x04   ;;entidad que se puede mover
                     0008   118 e_cmp_input    = 0x08   ;;entidad controlable por input  
                     0010   119 e_cmp_ai       = 0x10   ;;entidad controlable con ia
                     0020   120 e_cmp_animated = 0x20   ;;entidad animada
                     0040   121 e_cmp_collider = 0x40   ;;entidad que puede colisionar
                     0080   122 e_cmp_collisionable = 0x80   ;;entidad que puede ser colisionada
                     0047   123 e_cmp_paddle = e_cmp_alive | e_cmp_render | e_cmp_physics | e_cmp_collider  ;;componente por defecto
                     0057   124 e_cmp_oponent_paddle = e_cmp_alive | e_cmp_render | e_cmp_physics | e_cmp_collider | e_cmp_ai ;;componente por defecto
                     0087   125 e_cpm_ball = e_cmp_alive | e_cmp_render | e_cmp_physics | e_cmp_collisionable
                     0043   126 e_cmp_border_wall = e_cmp_alive | e_cmp_collider | e_cmp_render
                            127 
ASxxxx Assembler V02.00 + NoICE + SDCC mods  (Zilog Z80 / Hitachi HD64180), page 9.
Hexadecimal [16-Bits]



                            128 ;;===============================================================================
                            129 ;;COLISION TYPES
                            130 ;;===============================================================================
                     0000   131 e_col_null = 0
                     0001   132 e_col_left  = 0x01
                     0002   133 e_col_right = 0x02
                     0004   134 e_col_up    = 0x04
                     0008   135 e_col_down  = 0x08
                            136 
                            137 ;;===============================================================================
                            138 ;; Entity Component IDs
                            139 ;;===============================================================================
   0000                     140 DefEnum e_cmpID
                     0000     1     e_cmpID_offset = 0
   0000                     141 Enum e_cmpID Render
                     0000     1     e_cmpID_Render = e_cmpID_offset
                     0001     2     e_cmpID_offset = e_cmpID_offset + 1
   0000                     142 Enum e_cmpID Physics
                     0001     1     e_cmpID_Physics = e_cmpID_offset
                     0002     2     e_cmpID_offset = e_cmpID_offset + 1
   0000                     143 Enum e_cmpID AI
                     0002     1     e_cmpID_AI = e_cmpID_offset
                     0003     2     e_cmpID_offset = e_cmpID_offset + 1
   0000                     144 Enum e_cmpID Animation
                     0003     1     e_cmpID_Animation = e_cmpID_offset
                     0004     2     e_cmpID_offset = e_cmpID_offset + 1
   0000                     145 Enum e_cmpID Collision
                     0004     1     e_cmpID_Collision = e_cmpID_offset
                     0005     2     e_cmpID_offset = e_cmpID_offset + 1
   0000                     146 Enum e_cmpID Num_Components
                     0005     1     e_cmpID_Num_Components = e_cmpID_offset
                     0006     2     e_cmpID_offset = e_cmpID_offset + 1
                            147 
                            148 
                            149 
                            150 ;; Keyboard constants
                     000A   151 BUFFER_SIZE = 10
                     00FF   152 ZERO_KEYS_ACTIVATED = #0xFF
                            153 
                            154 ;; Score constants
                     0004   155 SCORE_NUM_BYTES = 4
                            156 
                            157 ;; SMALL NUMBERS CONSTANTS
                     0002   158 S_SMALL_NUMBERS_WIDTH = 2
                     0005   159 S_SMALL_NUMBERS_HEIGHT = 5
                            160 ;; Font constants
                     0002   161 FONT_WIDTH = 2
                     0009   162 FONT_HEIGHT = 9
                            163 
                            164 
                            165 ;;===============================================================================
                            166 ;; ENTITIY SCTRUCTURE CREATION
                            167 ;;===============================================================================
   0000                     168 BeginStruct e
                     0000     1     e_offset = 0
ASxxxx Assembler V02.00 + NoICE + SDCC mods  (Zilog Z80 / Hitachi HD64180), page 10.
Hexadecimal [16-Bits]



   0000                     169 Field e, ptr                , 2
                     0000     1     e_ptr = e_offset
                     0002     2     e_offset = e_offset + 2
   0000                     170 Field e, cmps               , 1
                     0002     1     e_cmps = e_offset
                     0003     2     e_offset = e_offset + 1
   0000                     171 Field e, type               , 1
                     0003     1     e_type = e_offset
                     0004     2     e_offset = e_offset + 1
   0000                     172 Field e, color              , 1
                     0004     1     e_color = e_offset
                     0005     2     e_offset = e_offset + 1
   0000                     173 Field e, x                  , 2
                     0005     1     e_x = e_offset
                     0007     2     e_offset = e_offset + 2
   0000                     174 Field e, y                  , 2
                     0007     1     e_y = e_offset
                     0009     2     e_offset = e_offset + 2
   0000                     175 Field e, w                  , 1
                     0009     1     e_w = e_offset
                     000A     2     e_offset = e_offset + 1
   0000                     176 Field e, h                  , 1
                     000A     1     e_h = e_offset
                     000B     2     e_offset = e_offset + 1
   0000                     177 Field e, end_x              , 1
                     000B     1     e_end_x = e_offset
                     000C     2     e_offset = e_offset + 1
   0000                     178 Field e, end_y              , 1
                     000C     1     e_end_y = e_offset
                     000D     2     e_offset = e_offset + 1
   0000                     179 Field e, last_x             , 1
                     000D     1     e_last_x = e_offset
                     000E     2     e_offset = e_offset + 1
   0000                     180 Field e, last_y             , 1
                     000E     1     e_last_y = e_offset
                     000F     2     e_offset = e_offset + 1
   0000                     181 Field e, vx                 , 2
                     000F     1     e_vx = e_offset
                     0011     2     e_offset = e_offset + 2
   0000                     182 Field e, vy                 , 2
                     0011     1     e_vy = e_offset
                     0013     2     e_offset = e_offset + 2
   0000                     183 Field e, sprite             , 2
                     0013     1     e_sprite = e_offset
                     0015     2     e_offset = e_offset + 2
   0000                     184 Field e, address            , 2
                     0015     1     e_address = e_offset
                     0017     2     e_offset = e_offset + 2
   0000                     185 Field e, p_address          , 2
                     0017     1     e_p_address = e_offset
                     0019     2     e_offset = e_offset + 2
   0000                     186 Field e, collision_status   , 1
                     0019     1     e_collision_status = e_offset
                     001A     2     e_offset = e_offset + 1
   0000                     187 Field e, collision_callback , 2
ASxxxx Assembler V02.00 + NoICE + SDCC mods  (Zilog Z80 / Hitachi HD64180), page 11.
Hexadecimal [16-Bits]



                     001A     1     e_collision_callback = e_offset
                     001C     2     e_offset = e_offset + 2
   0000                     188 Field e, ai_status          , 1
                     001C     1     e_ai_status = e_offset
                     001D     2     e_offset = e_offset + 1
   0000                     189 Field e, ai_callback        , 2
                     001D     1     e_ai_callback = e_offset
                     001F     2     e_offset = e_offset + 2
   0000                     190 Field e, moved              , 1
                     001F     1     e_moved = e_offset
                     0020     2     e_offset = e_offset + 1
   0000                     191 EndStruct e
                     0020     1     sizeof_e = e_offset
                            192 
                            193 ;;===============================================================================
                            194 ;; GLOBAL VARIABLES
                            195 ;;===============================================================================
ASxxxx Assembler V02.00 + NoICE + SDCC mods  (Zilog Z80 / Hitachi HD64180), page 12.
Hexadecimal [16-Bits]



                             22 ;;
                             23 ;; Start of _DATA area 
                             24 ;;  SDCC requires at least _DATA and _CODE areas to be declared, but you may use
                             25 ;;  any one of them for any purpose. Usually, compiler puts _DATA area contents
                             26 ;;  right after _CODE area contents.
                             27 ;;
                             28 .area _DATA
                             29 
                             30 
   2871 20 20 20 20 20 20    31 string_buffer:: .asciz "          "
        20 20 20 20 00
                             32 
                             33 
                             34 ;;
                             35 ;; Start of _CODE area
                             36 ;; 
                             37 .area _CODE
                             38 
                             39 ;;-----------------------------------------------------------------;; 
                             40 ;;  sys_util_h_times_e
                             41 ;;
                             42 ;; Inputs:
                             43 ;;   H and E
                             44 ;; Outputs:
                             45 ;;   HL is the product
                             46 ;;   D is 0
                             47 ;;   A,E,B,C are preserved
                             48 ;; 36 bytes
                             49 ;; min: 190cc
                             50 ;; max: 242cc
                             51 ;; avg: 216cc
                             52 ;; Credits:
                             53 ;;  Z80Heaven (http://z80-heaven.wikidot.com/advanced-math#toc9)
                             54 
   0D4F                      55 sys_util_h_times_e::
   0D4F 16 00         [ 7]   56   ld d,#0
   0D51 6A            [ 4]   57   ld l,d
   0D52 CB 24         [ 8]   58   sla h 
   0D54 30 01         [12]   59   jr nc,.+3 
   0D56 6B            [ 4]   60   ld l,e
   0D57 29            [11]   61   add hl,hl 
   0D58 30 01         [12]   62   jr nc,.+3 
   0D5A 19            [11]   63   add hl,de
   0D5B 29            [11]   64   add hl,hl 
   0D5C 30 01         [12]   65   jr nc,.+3 
   0D5E 19            [11]   66   add hl,de
   0D5F 29            [11]   67   add hl,hl 
   0D60 30 01         [12]   68   jr nc,.+3 
   0D62 19            [11]   69   add hl,de
   0D63 29            [11]   70   add hl,hl 
   0D64 30 01         [12]   71   jr nc,.+3 
   0D66 19            [11]   72   add hl,de
   0D67 29            [11]   73   add hl,hl 
   0D68 30 01         [12]   74   jr nc,.+3 
   0D6A 19            [11]   75   add hl,de
ASxxxx Assembler V02.00 + NoICE + SDCC mods  (Zilog Z80 / Hitachi HD64180), page 13.
Hexadecimal [16-Bits]



   0D6B 29            [11]   76   add hl,hl 
   0D6C 30 01         [12]   77   jr nc,.+3 
   0D6E 19            [11]   78   add hl,de
   0D6F 29            [11]   79   add hl,hl 
   0D70 D0            [11]   80   ret nc 
   0D71 19            [11]   81   add hl,de
   0D72 C9            [10]   82   ret
                             83 
                             84 ;;-----------------------------------------------------------------;; 
                             85 ;;  sys_util_hl_div_c
                             86 ;;
                             87 ;;Inputs:
                             88 ;;     HL is the numerator
                             89 ;;     C is the denominator
                             90 ;;Outputs:
                             91 ;;     A is the remainder
                             92 ;;     B is 0
                             93 ;;     C is not changed
                             94 ;;     DE is not changed
                             95 ;;     HL is the quotient
                             96 ;;
   0D73                      97 sys_util_hl_div_c::
   0D73 06 10         [ 7]   98        ld b,#16
   0D75 AF            [ 4]   99        xor a
   0D76 29            [11]  100          add hl,hl
   0D77 17            [ 4]  101          rla
   0D78 B9            [ 4]  102          cp c
   0D79 38 02         [12]  103          jr c,.+4
   0D7B 2C            [ 4]  104            inc l
   0D7C 91            [ 4]  105            sub c
   0D7D 10 F7         [13]  106          djnz .-7
   0D7F C9            [10]  107        ret
                            108 
                            109 ;;-----------------------------------------------------------------
                            110 ;;
                            111 ;; sys_util_absHL
                            112 ;;
                            113 ;;  
                            114 ;;  Input:  hl: number
                            115 ;;  Output: hl: absolut value of number
                            116 ;;  Destroyed: af
                            117 ;;
                            118 ;;  Cemetech code (https://learn.cemetech.net/index.php?title=Z80:Math_Routines#absHL)
                            119 ;;
   0D80                     120 sys_util_absHL::
   0D80 CB 7C         [ 8]  121   bit #7,h
   0D82 C8            [11]  122   ret z
   0D83 AF            [ 4]  123   xor a
   0D84 95            [ 4]  124   sub l
   0D85 6F            [ 4]  125   ld l,a
   0D86 9F            [ 4]  126   sbc a,a
   0D87 94            [ 4]  127   sub h
   0D88 67            [ 4]  128   ld h,a
   0D89 C9            [10]  129   ret
                            130 
ASxxxx Assembler V02.00 + NoICE + SDCC mods  (Zilog Z80 / Hitachi HD64180), page 14.
Hexadecimal [16-Bits]



                            131 ;;-----------------------------------------------------------------
                            132 ;;
                            133 ;; sys_util_BCD_GetEnd
                            134 ;;
                            135 ;;  
                            136 ;;  Input:  b: number of bytes of the bcd number
                            137 ;;          de: source for the first bcd bnumber
                            138 ;;          hl: source for the second bcd number
                            139 ;;  Output: 
                            140 ;;  Destroyed: af, bc,de, hl
                            141 ;;
                            142 ;;  Chibi Akumas BCD code (https://www.chibiakumas.com/z80/advanced.php#LessonA1)
                            143 ;;
   0D8A                     144 sys_util_BCD_GetEnd::
                            145 ;Some of our commands need to start from the most significant byte
                            146 ;This will shift HL and DE along b bytes
   0D8A C5            [11]  147 	push bc
   0D8B 48            [ 4]  148 	ld c,b	;We want to add BC, but we need to add one less than the number of bytes
   0D8C 0D            [ 4]  149 	dec c
   0D8D 06 00         [ 7]  150 	ld b,#0
   0D8F 09            [11]  151 	add hl,bc
   0D90 EB            [ 4]  152 	ex de, hl	;We've done HL, but we also want to do DE
   0D91 09            [11]  153 	add hl,bc
   0D92 EB            [ 4]  154 	ex de, hl
   0D93 C1            [10]  155 	pop bc
   0D94 C9            [10]  156 	ret
                            157 
                            158 ;;-----------------------------------------------------------------
                            159 ;;
                            160 ;; BCD_Add
                            161 ;;
                            162 ;;   Add two BCD numbers
                            163 ;;  Input:  hl: Number to add to de
                            164 ;;          de: Number to store the sum 
                            165 ;;  Output: 
                            166 ;;  Destroyed: af, bc,de, hl
                            167 ;;
                            168 ;;  Chibi Akumas BCD code (https://www.chibiakumas.com/z80/advanced.php#LessonA1)
                            169 ;;
   0D95                     170 sys_util_BCD_Add::
   0D95 B7            [ 4]  171     or a
   0D96                     172 BCD_Add_Again:
   0D96 1A            [ 7]  173     ld a, (de)
   0D97 8E            [ 7]  174     adc (hl)
   0D98 27            [ 4]  175     daa
   0D99 12            [ 7]  176     ld (de), a
   0D9A 13            [ 6]  177     inc de
   0D9B 23            [ 6]  178     inc hl
   0D9C 10 F8         [13]  179     djnz BCD_Add_Again
   0D9E C9            [10]  180     ret
                            181   
                            182 ;;-----------------------------------------------------------------
                            183 ;;
                            184 ;; sys_util_BCD_Compare
                            185 ;;
ASxxxx Assembler V02.00 + NoICE + SDCC mods  (Zilog Z80 / Hitachi HD64180), page 15.
Hexadecimal [16-Bits]



                            186 ;;  Compare two BCD numbers
                            187 ;;  Input:  hl: BCD Number 1
                            188 ;;          de: BCD Number 2
                            189 ;;  Output: 
                            190 ;;  Destroyed: af, bc,de, hl
                            191 ;;
                            192 ;;  Chibi Akumas BCD code (https://www.chibiakumas.com/z80/advanced.php#LessonA1)
                            193 ;;
   0D9F                     194 sys_util_BCD_Compare::
   0D9F 06 04         [ 7]  195   ld b, #SCORE_NUM_BYTES
   0DA1 CD 8A 0D      [17]  196   call sys_util_BCD_GetEnd
   0DA4                     197 BCD_cp_direct:
   0DA4 1A            [ 7]  198   ld a, (de)
   0DA5 BE            [ 7]  199   cp (hl)
   0DA6 D8            [11]  200   ret c
   0DA7 C0            [11]  201   ret nz
   0DA8 1B            [ 6]  202   dec de
   0DA9 2B            [ 6]  203   dec hl
   0DAA 10 F8         [13]  204   djnz BCD_cp_direct
   0DAC B7            [ 4]  205   or a                    ;; Clear carry
   0DAD C9            [10]  206   ret
                            207 
                            208 ;;-----------------------------------------------------------------
                            209 ;;
                            210 ;; sys_util_get_random_number
                            211 ;;
                            212 ;;  Returns a random number between 0 and <end>
                            213 ;;  Input:  a: <end>
                            214 ;;  Output: a: random number
                            215 ;;  Destroyed: af, bc,de, hl
                            216 
   0DAE                     217 sys_util_get_random_number::
   0DAE 32 B8 0D      [13]  218   ld (#random_max_number), a
   0DB1 CD 39 25      [17]  219   call cpct_getRandom_mxor_u8_asm
   0DB4 7D            [ 4]  220   ld a, l                             ;; Calculates a pseudo modulus of max number
   0DB5 26 00         [ 7]  221   ld h,#0                             ;; Load hl with the random number
                     0069   222 random_max_number = .+1
   0DB7 0E 00         [ 7]  223   ld c, #0                            ;; Load c with the max number
   0DB9 06 00         [ 7]  224   ld b, #0
   0DBB                     225 _random_mod_loop:
   0DBB B7            [ 4]  226   or a                                ;; ??
   0DBC ED 42         [15]  227   sbc hl,bc                           ;; hl = hl - bc
   0DBE F2 BB 0D      [10]  228   jp p, _random_mod_loop              ;; Jump back if hl > 0
   0DC1 09            [11]  229   add hl,bc                           ;; Adds MAX_MODEL_CARD to hl back to get back to positive values
   0DC2 7D            [ 4]  230   ld a,l                              ;; loads the normalized random number in a
   0DC3 C9            [10]  231 ret
                            232 
                            233 ;;-----------------------------------------------------------------
                            234 ;;
                            235 ;; sys_util_delay
                            236 ;;
                            237 ;;  Waits a determined number of frames 
                            238 ;;  Input:  b: number of frames
                            239 ;;  Output: 
                            240 ;;  Destroyed: af, bc
ASxxxx Assembler V02.00 + NoICE + SDCC mods  (Zilog Z80 / Hitachi HD64180), page 16.
Hexadecimal [16-Bits]



                            241 ;;
   0DC4                     242 sys_util_delay::
   0DC4 C5            [11]  243   push bc
   0DC5 CD EA 25      [17]  244   call cpct_waitVSYNCStart_asm
   0DC8 C1            [10]  245   pop bc
   0DC9 10 F9         [13]  246   djnz sys_util_delay
   0DCB C9            [10]  247   ret
                            248 
                            249 
                            250 ;;-----------------------------------------------------------------
                            251 ;;
                            252 ;; sys_util_negHL
                            253 ;;
                            254 ;;  Negates hl
                            255 ;;  input: hl
                            256 ;;  ouput: hl negated
                            257 ;;  destroys: a
                            258 ;;
                            259 ;; WikiTI code (https://wikiti.brandonw.net/index.php?title=Z80_Routines:Math:Signed_Math)
   0DCC                     260 sys_util_negHL::
   0DCC AF            [ 4]  261 	xor a
   0DCD 95            [ 4]  262 	sub l
   0DCE 6F            [ 4]  263 	ld l,a
   0DCF 9F            [ 4]  264 	sbc a,a
   0DD0 94            [ 4]  265 	sub h
   0DD1 67            [ 4]  266 	ld h,a
   0DD2 C9            [10]  267 	ret
                            268 
                            269 ;;-----------------------------------------------------------------
                            270 ;;
                            271 ;; sys_util_hl_divided_d
                            272 ;;
                            273 ;;  Divides hl by d, and leaves the result in hl
                            274 ;;  input:  hl: dividend
                            275 ;;          d: divisor
                            276 ;;  ouput:  hl: result
                            277 ;;  destroys: af, de, bc, hl 
                            278 ;;
                            279 ;; code by Jonathan Cauldwell (https://chuntey.wordpress.com/category/z80-assembly/)
   0DD3                     280 sys_util_hl_divided_d::
   0DD3 06 08         [ 7]  281   ld b,#8              ; bits to check.
   0DD5 7A            [ 4]  282   ld a,d              ; number by which to divide.
   0DD6                     283 idiv3:  
   0DD6 17            [ 4]  284   rla                 ; check leftmost bit.
   0DD7 38 07         [12]  285   jr c,idiv2          ; no more shifts required.
   0DD9 04            [ 4]  286   inc b               ; extra shift needed.
   0DDA BC            [ 4]  287   cp h
   0DDB 30 03         [12]  288   jr nc,idiv2
   0DDD C3 D6 0D      [10]  289   jp idiv3            ; repeat.
                            290 
   0DE0                     291 idiv2:  
   0DE0 AF            [ 4]  292   xor a
   0DE1 5F            [ 4]  293   ld e,a
   0DE2 4F            [ 4]  294   ld c,a              ; result.
   0DE3                     295 idiv1:  
ASxxxx Assembler V02.00 + NoICE + SDCC mods  (Zilog Z80 / Hitachi HD64180), page 17.
Hexadecimal [16-Bits]



   0DE3 ED 52         [15]  296   sbc hl,de           ; do subtraction.
   0DE5 30 01         [12]  297   jr nc,idiv0         ; no carry, keep the result.
   0DE7 19            [11]  298   add hl,de           ; restore original value of hl.
   0DE8                     299 idiv0: 
   0DE8 3F            [ 4]  300   ccf                 ; reverse carry bit.
   0DE9 CB 11         [ 8]  301   rl c                ; rotate in to ac.
   0DEB 17            [ 4]  302   rla
   0DEC CB 1A         [ 8]  303   rr d                ; divide de by 2.
   0DEE CB 1B         [ 8]  304   rr e
   0DF0 10 F1         [13]  305   djnz idiv1          ; repeat.
   0DF2 67            [ 4]  306   ld h,a              ; copy result to hl.
   0DF3 69            [ 4]  307   ld l,c
   0DF4 C9            [10]  308   ret
                            309 
                            310 ;;-----------------------------------------------------------------
                            311 ;;
                            312 ;; sys_util_sqr_hl
                            313 ;;
                            314 ;;  Calculates de square root of hl in a
                            315 ;;  fast 16 bit isqrt by John Metcalf
                            316 ;;  92 bytes, 344-379 cycles (average 362)
                            317 ;;  v2 - saved 3 cycles with a tweak suggested by Russ McNulty
                            318 ;;  input: hl
                            319 ;;  ouput: a
                            320 ;;  destroys: de, hl 
                            321 ;;
                            322 ;; code by John Metcalf (https://github.com/impomatic/z80snippets/blob/master/fastisqr.asm)
   0DF5                     323 sys_util_sqr_hl::
                            324 
   0DF5 7C            [ 4]  325   ld a,h        ; 4
   0DF6 11 C0 B0      [10]  326   ld de,#0x0B0C0  ; 10
   0DF9 83            [ 4]  327   add a,e       ; 4
   0DFA 38 03         [12]  328   jr c,sq7      ; 12 / 7
   0DFC 7C            [ 4]  329   ld a,h        ; 4
   0DFD 16 F0         [ 7]  330   ld d,#0x0F0     ; 7
   0DFF                     331 sq7:
                            332 
                            333 ; ----------
                            334 
   0DFF 82            [ 4]  335   add a,d       ; 4
   0E00 30 03         [12]  336   jr nc,sq6     ; 12 / 7
   0E02 CB AA         [ 8]  337   res 5,d       ; 8
   0E04 FE                  338   .db #254        ; 7
   0E05                     339 sq6:
   0E05 92            [ 4]  340   sub d         ; 4
   0E06 CB 2A         [ 8]  341   sra d         ; 8
                            342 
                            343 ; ----------
                            344 
   0E08 CB D2         [ 8]  345   set 2,d       ; 8
   0E0A 82            [ 4]  346   add a,d       ; 4
   0E0B 30 03         [12]  347   jr nc,sq5     ; 12 / 7
   0E0D CB 9A         [ 8]  348   res 3,d       ; 8
   0E0F FE                  349   .db #254        ; 7
   0E10                     350 sq5:
ASxxxx Assembler V02.00 + NoICE + SDCC mods  (Zilog Z80 / Hitachi HD64180), page 18.
Hexadecimal [16-Bits]



   0E10 92            [ 4]  351   sub d         ; 4
   0E11 CB 2A         [ 8]  352   sra d         ; 8
                            353 
                            354 ; ----------
                            355 
   0E13 14            [ 4]  356   inc d         ; 4
   0E14 82            [ 4]  357   add a,d       ; 4
   0E15 30 03         [12]  358   jr nc,sq4     ; 12 / 7
   0E17 CB 8A         [ 8]  359   res 1,d       ; 8
   0E19 FE                  360   .db #254        ; 7
   0E1A                     361 sq4:
   0E1A 92            [ 4]  362   sub d         ; 4
   0E1B CB 2A         [ 8]  363   sra d         ; 8
   0E1D 67            [ 4]  364   ld h,a        ; 4
                            365 
                            366 ; ----------
                            367 
   0E1E 19            [11]  368   add hl,de     ; 11
   0E1F 30 03         [12]  369   jr nc,sq3     ; 12 / 7
   0E21 1E 40         [ 7]  370   ld e,#0x040     ; 7
   0E23 D2                  371   .db #210        ; 10
   0E24                     372 sq3:
   0E24 ED 52         [15]  373   sbc hl,de     ; 15
   0E26 CB 2A         [ 8]  374   sra d         ; 8
   0E28 7B            [ 4]  375   ld a,e        ; 4
   0E29 1F            [ 4]  376   rra           ; 4
                            377 
                            378 ; ----------
                            379 
   0E2A F6 10         [ 7]  380   or #0x010       ; 7
   0E2C 5F            [ 4]  381   ld e,a        ; 4
   0E2D 19            [11]  382   add hl,de     ; 11
   0E2E 30 03         [12]  383   jr nc,sq2     ; 12 / 7
   0E30 E6 DF         [ 7]  384   and #0x0DF      ; 7
   0E32 DA                  385   .db #218        ; 10
   0E33                     386 sq2:
   0E33 ED 52         [15]  387   sbc hl,de     ; 15
   0E35 CB 2A         [ 8]  388   sra d         ; 8
   0E37 1F            [ 4]  389   rra           ; 4
                            390 
                            391 ; ----------
                            392 
   0E38 F6 04         [ 7]  393   or #0x04        ; 7
   0E3A 5F            [ 4]  394   ld e,a        ; 4
   0E3B 19            [11]  395   add hl,de     ; 11
   0E3C 30 03         [12]  396   jr nc,sq1     ; 12 / 7
   0E3E E6 F7         [ 7]  397   and #0x0F7      ; 7
   0E40 DA                  398   .db #218        ; 10
   0E41                     399 sq1:
   0E41 ED 52         [15]  400   sbc hl,de     ; 15
   0E43 CB 2A         [ 8]  401   sra d         ; 8
   0E45 1F            [ 4]  402   rra           ; 4
                            403 
                            404 ; ----------
                            405 
ASxxxx Assembler V02.00 + NoICE + SDCC mods  (Zilog Z80 / Hitachi HD64180), page 19.
Hexadecimal [16-Bits]



   0E46 3C            [ 4]  406   inc a         ; 4
   0E47 5F            [ 4]  407   ld e,a        ; 4
   0E48 19            [11]  408   add hl,de     ; 11
   0E49 30 02         [12]  409   jr nc,sq0     ; 12 / 7
   0E4B E6 FD         [ 7]  410   and #0x0FD      ; 7
   0E4D                     411 sq0:
   0E4D CB 2A         [ 8]  412   sra d         ; 8
   0E4F 1F            [ 4]  413   rra           ; 4
   0E50 2F            [ 4]  414   cpl           ; 4
                            415 
   0E51 C9            [10]  416 ret
                            417 
                            418 
                            419 
                            420 ;;-----------------------------------------------------------------
                            421 ;;
                            422 ;; sys_util_return_from_sine_table
                            423 ;;
                            424 ;;  Returns the number of sine table corresponding to the angle
                            425 ;;  Input:  hl: angle
                            426 ;;  Output: hl : sine table result
                            427 ;;  Destroyed: af, bc
                            428 ;;
   0E52                     429 sys_util_return_from_sine_table::
   0E52 01 5A 00      [10]  430   ld bc, #90
   0E55 B7            [ 4]  431   or a 
   0E56 ED 42         [15]  432   sbc hl, bc
   0E58 38 04         [12]  433   jr c, _sus_regular_return
   0E5A 21 00 01      [10]  434   ld hl, #0x0100
   0E5D C9            [10]  435   ret
   0E5E                     436 _sus_regular_return:
   0E5E 2A EF 0E      [16]  437   ld hl, (angle)
   0E61 EB            [ 4]  438   ex de, hl
   0E62 21 F1 0E      [10]  439   ld hl, #sine_table
   0E65 19            [11]  440   add hl, de
   0E66 7E            [ 7]  441   ld a, (hl)
   0E67 26 00         [ 7]  442   ld h, #0
   0E69 6F            [ 4]  443   ld l, a
   0E6A C9            [10]  444   ret 
                            445 
                            446 ;;-----------------------------------------------------------------
                            447 ;;
                            448 ;; sys_util_sine::
                            449 ;;
                            450 ;;  Waits a determined number of frames 
                            451 ;;  Input:  a: angle
                            452 ;;  Output: a : cosine(angle)
                            453 ;;  Destroyed: af, bc
                            454 ;;
                            455 ;;     if (angle < 90) {
                            456 ;;          return sine_table[angle];
                            457 ;;     } else if (angle < 180) {
                            458 ;;          return sine_table[180 - angle];
                            459 ;;     } else if (angle < 270) {
                            460 ;;          return -sine_table[angle - 180];
ASxxxx Assembler V02.00 + NoICE + SDCC mods  (Zilog Z80 / Hitachi HD64180), page 20.
Hexadecimal [16-Bits]



                            461 ;;     } else {
                            462 ;;          return -sine_table[360 - angle];
                            463 ;;     }
                            464 ;;
   0E6B                     465 sys_util_sine::
   0E6B 22 EF 0E      [16]  466   ld (angle), hl
   0E6E 01 5B 00      [10]  467   ld bc, #91
   0E71 B7            [ 4]  468   or a
   0E72 ED 42         [15]  469   sbc hl, bc
   0E74 38 29         [12]  470   jr c, _sus_return_minus90
   0E76 2A EF 0E      [16]  471   ld hl, (angle)
   0E79 01 B4 00      [10]  472   ld bc, #180
   0E7C B7            [ 4]  473   or a
   0E7D ED 42         [15]  474   sbc hl, bc
   0E7F 38 24         [12]  475   jr c, _sus_return_minus180
   0E81 2A EF 0E      [16]  476   ld hl, (angle)
   0E84 01 0E 01      [10]  477   ld bc, #270
   0E87 B7            [ 4]  478   or a
   0E88 ED 42         [15]  479   sbc hl, bc
   0E8A 38 29         [12]  480   jr c, _sus_return_minus270
   0E8C                     481 _sus_return_minus360:
                            482   ;; calculate 360 - angle
   0E8C 2A EF 0E      [16]  483   ld hl, (angle)
   0E8F 11 68 01      [10]  484   ld de, #360
   0E92 EB            [ 4]  485   ex de, hl
   0E93 B7            [ 4]  486   or a                                  ;; reset carry
   0E94 ED 52         [15]  487   sbc hl, de
   0E96 22 EF 0E      [16]  488   ld (angle), hl
   0E99 CD 52 0E      [17]  489   call sys_util_return_from_sine_table
   0E9C C3 CC 0D      [10]  490   jp sys_util_negHL
   0E9F                     491 _sus_return_minus90:
   0E9F 2A EF 0E      [16]  492   ld hl, (angle)
   0EA2 C3 52 0E      [10]  493   jp sys_util_return_from_sine_table
   0EA5                     494 _sus_return_minus180:
                            495   ;; calculate 180 - angle
   0EA5 2A EF 0E      [16]  496   ld hl, (angle)
   0EA8 11 B4 00      [10]  497   ld de, #180
   0EAB EB            [ 4]  498   ex de, hl
   0EAC B7            [ 4]  499   or a                                  ;; reset carry
   0EAD ED 52         [15]  500   sbc hl, de
   0EAF 22 EF 0E      [16]  501   ld (angle), hl
   0EB2 C3 52 0E      [10]  502   jp sys_util_return_from_sine_table
   0EB5                     503 _sus_return_minus270:
                            504   ;; calculate angle - 180
   0EB5 2A EF 0E      [16]  505   ld hl, (angle)
   0EB8 11 B4 00      [10]  506   ld de, #180
   0EBB B7            [ 4]  507   or a                                  ;; reset carry
   0EBC ED 52         [15]  508   sbc hl, de
   0EBE 22 EF 0E      [16]  509   ld (angle), hl
   0EC1 CD 52 0E      [17]  510   call sys_util_return_from_sine_table
   0EC4 C3 CC 0D      [10]  511   jp sys_util_negHL
                            512 
                            513 
                            514 ;;-----------------------------------------------------------------
                            515 ;;
ASxxxx Assembler V02.00 + NoICE + SDCC mods  (Zilog Z80 / Hitachi HD64180), page 21.
Hexadecimal [16-Bits]



                            516 ;; sys_util_cosine
                            517 ;;
                            518 ;;  Waits a determined number of frames 
                            519 ;;  Input:  hl: angle
                            520 ;;  Output: hl : cosine(angle)
                            521 ;;  Destroyed: af, bc
                            522 ;;
                            523 ;;     if (angle <= 90)
                            524 ;;          return (sine(90 - angle));
                            525 ;;     else
                            526 ;;          return (-sine(angle - 90));
                            527 ;;
   0EC7                     528 sys_util_cosine::
   0EC7 22 EF 0E      [16]  529   ld (angle), hl
   0ECA 11 5A 00      [10]  530   ld de, #90
   0ECD B7            [ 4]  531   or a
   0ECE ED 52         [15]  532   sbc hl, de
   0ED0 30 0E         [12]  533   jr nc, suc_more_than_90
                            534     ;;calculate 90-angle in hl
   0ED2 11 5A 00      [10]  535     ld de, #90
   0ED5 2A EF 0E      [16]  536     ld hl, (angle)
   0ED8 EB            [ 4]  537     ex de, hl
   0ED9 B7            [ 4]  538     or a
   0EDA ED 52         [15]  539     sbc hl, de
   0EDC CD 6B 0E      [17]  540     call sys_util_sine
   0EDF C9            [10]  541     ret
   0EE0                     542   suc_more_than_90:
                            543     ;; calculate angle-90 in hl
   0EE0 2A EF 0E      [16]  544     ld hl, (angle)
   0EE3 11 5A 00      [10]  545     ld de, #90
   0EE6 B7            [ 4]  546     or a
   0EE7 ED 52         [15]  547     sbc hl, de
   0EE9 CD 6B 0E      [17]  548     call sys_util_sine
   0EEC C3 CC 0D      [10]  549     jp sys_util_negHL
                            550 
   0EEF 00 00               551   angle:: .dw #0000
                            552 
                            553 
                            554 
                            555 ;;-------------------------------------------------------------------
                            556 ;;
                            557 ;;  Sine Table
                            558 ;;
                            559 ;; The sine table can be stored in bytes, becuase in the first quarter
                            560 ;; all the sinus are positive an lower than 255.
                            561 ;; in order to be able to return negative numbers, is necesary to
                            562 ;; transform the byte into word when returning the information and 
                            563 ;; have in mind that form 87 to 90 degres should return the word 0100
                            564 ;;
   0EF1                     565 sine_table::
   0EF1 00                  566   .db #0x00	
   0EF2 04 08 0D 11 16 1A   567   .db #0x04, #0x08, #0x0D, #0x11, #0x16, #0x1A, #0x1F, #0x23, #0x28, #0x2C	
        1F 23 28 2C
   0EFC 30 35 39 3D 42 46   568   .db #0x30, #0x35, #0x39, #0x3D, #0x42, #0x46, #0x4A, #0x4F, #0x53, #0x57	
        4A 4F 53 57
ASxxxx Assembler V02.00 + NoICE + SDCC mods  (Zilog Z80 / Hitachi HD64180), page 22.
Hexadecimal [16-Bits]



   0F06 5B 5F 64 68 6C 70   569   .db #0x5B, #0x5F, #0x64, #0x68, #0x6C, #0x70, #0x74, #0x78, #0x7C, #0x7F	
        74 78 7C 7F
   0F10 83 87 8B 8F 92 96   570   .db #0x83, #0x87, #0x8B, #0x8F, #0x92, #0x96, #0x9A, #0x9D, #0xA1, #0xA4	
        9A 9D A1 A4
   0F1A A7 AB AE B1 B5 B8   571   .db #0xA7, #0xAB, #0xAE, #0xB1, #0xB5, #0xB8, #0xBB, #0xBE, #0xC1, #0xC4	
        BB BE C1 C4
   0F24 C6 C9 CC CF D1 D4   572   .db #0xC6, #0xC9, #0xCC, #0xCF, #0xD1, #0xD4, #0xD6, #0xD9, #0xDB, #0xDD	
        D6 D9 DB DD
   0F2E DF E2 E4 E6 E8 E9   573   .db #0xDF, #0xE2, #0xE4, #0xE6, #0xE8, #0xE9, #0xEB, #0xED, #0xEE, #0xF0	
        EB ED EE F0
   0F38 F2 F3 F4 F6 F7 F8   574   .db #0xF2, #0xF3, #0xF4, #0xF6, #0xF7, #0xF8, #0xF9, #0xFA, #0xFB, #0xFC	
        F9 FA FB FC
   0F42 FC FD FE FE FF FF   575   .db #0xFC, #0xFD, #0xFE, #0xFE, #0xFF, #0xFF, #0xFF, #0xFF, #0xFF
        FF FF FF
