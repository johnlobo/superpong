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
   26C5 20 20 20 20 20 20    31 string_buffer:: .asciz "          "
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
   0D1B                      55 sys_util_h_times_e::
   0D1B 16 00         [ 7]   56   ld d,#0
   0D1D 6A            [ 4]   57   ld l,d
   0D1E CB 24         [ 8]   58   sla h 
   0D20 30 01         [12]   59   jr nc,.+3 
   0D22 6B            [ 4]   60   ld l,e
   0D23 29            [11]   61   add hl,hl 
   0D24 30 01         [12]   62   jr nc,.+3 
   0D26 19            [11]   63   add hl,de
   0D27 29            [11]   64   add hl,hl 
   0D28 30 01         [12]   65   jr nc,.+3 
   0D2A 19            [11]   66   add hl,de
   0D2B 29            [11]   67   add hl,hl 
   0D2C 30 01         [12]   68   jr nc,.+3 
   0D2E 19            [11]   69   add hl,de
   0D2F 29            [11]   70   add hl,hl 
   0D30 30 01         [12]   71   jr nc,.+3 
   0D32 19            [11]   72   add hl,de
   0D33 29            [11]   73   add hl,hl 
   0D34 30 01         [12]   74   jr nc,.+3 
   0D36 19            [11]   75   add hl,de
ASxxxx Assembler V02.00 + NoICE + SDCC mods  (Zilog Z80 / Hitachi HD64180), page 13.
Hexadecimal [16-Bits]



   0D37 29            [11]   76   add hl,hl 
   0D38 30 01         [12]   77   jr nc,.+3 
   0D3A 19            [11]   78   add hl,de
   0D3B 29            [11]   79   add hl,hl 
   0D3C D0            [11]   80   ret nc 
   0D3D 19            [11]   81   add hl,de
   0D3E C9            [10]   82   ret
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
   0D3F                      97 sys_util_hl_div_c::
   0D3F 06 10         [ 7]   98        ld b,#16
   0D41 AF            [ 4]   99        xor a
   0D42 29            [11]  100          add hl,hl
   0D43 17            [ 4]  101          rla
   0D44 B9            [ 4]  102          cp c
   0D45 38 02         [12]  103          jr c,.+4
   0D47 2C            [ 4]  104            inc l
   0D48 91            [ 4]  105            sub c
   0D49 10 F7         [13]  106          djnz .-7
   0D4B C9            [10]  107        ret
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
   0D4C                     120 sys_util_absHL::
   0D4C CB 7C         [ 8]  121   bit #7,h
   0D4E C8            [11]  122   ret z
   0D4F AF            [ 4]  123   xor a
   0D50 95            [ 4]  124   sub l
   0D51 6F            [ 4]  125   ld l,a
   0D52 9F            [ 4]  126   sbc a,a
   0D53 94            [ 4]  127   sub h
   0D54 67            [ 4]  128   ld h,a
   0D55 C9            [10]  129   ret
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
   0D56                     144 sys_util_BCD_GetEnd::
                            145 ;Some of our commands need to start from the most significant byte
                            146 ;This will shift HL and DE along b bytes
   0D56 C5            [11]  147 	push bc
   0D57 48            [ 4]  148 	ld c,b	;We want to add BC, but we need to add one less than the number of bytes
   0D58 0D            [ 4]  149 	dec c
   0D59 06 00         [ 7]  150 	ld b,#0
   0D5B 09            [11]  151 	add hl,bc
   0D5C EB            [ 4]  152 	ex de, hl	;We've done HL, but we also want to do DE
   0D5D 09            [11]  153 	add hl,bc
   0D5E EB            [ 4]  154 	ex de, hl
   0D5F C1            [10]  155 	pop bc
   0D60 C9            [10]  156 	ret
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
   0D61                     170 sys_util_BCD_Add::
   0D61 B7            [ 4]  171     or a
   0D62                     172 BCD_Add_Again:
   0D62 1A            [ 7]  173     ld a, (de)
   0D63 8E            [ 7]  174     adc (hl)
   0D64 27            [ 4]  175     daa
   0D65 12            [ 7]  176     ld (de), a
   0D66 13            [ 6]  177     inc de
   0D67 23            [ 6]  178     inc hl
   0D68 10 F8         [13]  179     djnz BCD_Add_Again
   0D6A C9            [10]  180     ret
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
   0D6B                     194 sys_util_BCD_Compare::
   0D6B 06 04         [ 7]  195   ld b, #SCORE_NUM_BYTES
   0D6D CD 56 0D      [17]  196   call sys_util_BCD_GetEnd
   0D70                     197 BCD_cp_direct:
   0D70 1A            [ 7]  198   ld a, (de)
   0D71 BE            [ 7]  199   cp (hl)
   0D72 D8            [11]  200   ret c
   0D73 C0            [11]  201   ret nz
   0D74 1B            [ 6]  202   dec de
   0D75 2B            [ 6]  203   dec hl
   0D76 10 F8         [13]  204   djnz BCD_cp_direct
   0D78 B7            [ 4]  205   or a                    ;; Clear carry
   0D79 C9            [10]  206   ret
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
   0D7A                     217 sys_util_get_random_number::
   0D7A 32 84 0D      [13]  218   ld (#random_max_number), a
   0D7D CD 8D 23      [17]  219   call cpct_getRandom_mxor_u8_asm
   0D80 7D            [ 4]  220   ld a, l                             ;; Calculates a pseudo modulus of max number
   0D81 26 00         [ 7]  221   ld h,#0                             ;; Load hl with the random number
                     0069   222 random_max_number = .+1
   0D83 0E 00         [ 7]  223   ld c, #0                            ;; Load c with the max number
   0D85 06 00         [ 7]  224   ld b, #0
   0D87                     225 _random_mod_loop:
   0D87 B7            [ 4]  226   or a                                ;; ??
   0D88 ED 42         [15]  227   sbc hl,bc                           ;; hl = hl - bc
   0D8A F2 87 0D      [10]  228   jp p, _random_mod_loop              ;; Jump back if hl > 0
   0D8D 09            [11]  229   add hl,bc                           ;; Adds MAX_MODEL_CARD to hl back to get back to positive values
   0D8E 7D            [ 4]  230   ld a,l                              ;; loads the normalized random number in a
   0D8F C9            [10]  231 ret
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
   0D90                     242 sys_util_delay::
   0D90 C5            [11]  243   push bc
   0D91 CD 3E 24      [17]  244   call cpct_waitVSYNCStart_asm
   0D94 C1            [10]  245   pop bc
   0D95 10 F9         [13]  246   djnz sys_util_delay
   0D97 C9            [10]  247   ret
                            248 
                            249 
                            250 ;;-----------------------------------------------------------------
                            251 ;;  negate hl
                            252 ;;  input: hl
                            253 ;;  ouput: hl negated
                            254 ;;  destroys a
                            255 ;; WikiTI code (https://wikiti.brandonw.net/index.php?title=Z80_Routines:Math:Signed_Math)
   0D98                     256 sys_util_negHL::
   0D98 AF            [ 4]  257 	xor a
   0D99 95            [ 4]  258 	sub l
   0D9A 6F            [ 4]  259 	ld l,a
   0D9B 9F            [ 4]  260 	sbc a,a
   0D9C 94            [ 4]  261 	sub h
   0D9D 67            [ 4]  262 	ld h,a
   0D9E C9            [10]  263 	ret
