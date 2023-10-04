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
                            106 .macro DefineEntity _cpms, _ptr, _type, _color, _x, _y, _w, _h, _vx, _vy, _sprite, _address, _p_address, _on_platform, _orientation, _anim_ptr, _anim_status
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
                            119     .dw _vx
                            120     .dw _vy
                            121     .dw _sprite
                            122     .dw _address
                            123     .dw _p_address
                            124     .db _on_platform
                            125     .db _orientation ;; 0: right, 1:left
                            126     .db #0           ;; dashing
                            127     .dw _anim_ptr
                            128     .db _anim_status
                            129     .db #1           ;; moved 1 default
                            130 .endm
                            131 
                            132 ;;==============================================================================================================================
                            133 ;;==============================================================================================================================
                            134 ;;  MACRO FOR ENUM DEFINITIONS
                            135 ;;==============================================================================================================================
                            136 ;;==============================================================================================================================
                            137 .mdelete DefEnum
                            138 .macro DefEnum _name
                            139     _name'_offset = 0
                            140 .endm
                            141 
                            142 ;;  Define enumeration element for an enumeration name.
                            143 .mdelete Enum
                            144 .macro Enum _enumname, _element
                            145     _enumname'_'_element = _enumname'_offset
                            146     _enumname'_offset = _enumname'_offset + 1
                            147 .endm
                            148 
                            149 ;;==============================================================================================================================
                            150 ;;==============================================================================================================================
                            151 ;;  DEFINE LINKED LIST STRUCTURE
                            152 ;;==============================================================================================================================
                            153 ;;==============================================================================================================================
                            154 
                            155 ;;  Defines the structure for a basic memory manager.
                            156 .mdelete DefineBasicStructureArray_Size
                            157 .macro DefineBasicStructureArray_Size _Tname, _N, _ComponentSize
                            158     _Tname'_array::
                            159         .ds _N * _ComponentSize
                            160 .endm
                            161 
                            162 ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
                            163 ;;  Defines the structure of the entity array.
                            164 .mdelete DefineComponentArrayStructure_Size
ASxxxx Assembler V02.00 + NoICE + SDCC mods  (Zilog Z80 / Hitachi HD64180), page 6.
Hexadecimal [16-Bits]



                            165 .macro DefineComponentArrayStructure_Size _Tname, _N, _ComponentSize
                            166     _Tname'_num:         .db 0
                            167     _Tname'_list:        .dw nullptr
                            168     _Tname'_free_list:   .dw _Tname'_array
                            169     _Tname'_array::
                            170         .ds _N * _ComponentSize
                            171 .endm
                            172 
                            173 
                            174 ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
                            175 ;;  Defines the structure for the component handler.
                            176 .mdelete DefineComponentPointerTable
                            177 .macro DefineComponentPointerTable _Tname, _N_Cmps, _N
                            178     _c = 0
                            179     ;;  Array containing pointers to component pointer arrays.
                            180     _Tname'_access_table::
                            181     .rept _N_Cmps
                            182         DefineComponentPointerAccessTable _Tname, \_c, _N, _N_Cmps
                            183         _c = _c + 1
                            184     .endm
                            185     ;;  Zero-fill the component array with two additional words for the
                            186     ;;  next free position and a null pointer fot he end of the array.
                            187     _Tname'_components::
                            188    .rept _N_Cmps
                            189         DefineComponentArray _N
                            190         .dw 0x0000
                            191         .dw 0x0000
                            192     .endm
                            193 .endm
                            194 
                            195 ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
                            196 ;;  Defines the pointers of the componente array pointer access table.
                            197 .mdelete DefineComponentPointerAccessTable
                            198 .macro DefineComponentPointerAccessTable _Tname, _suf, _N, _N_Cmps
                            199     _Tname'_components'_suf'_ptr_pend::    .dw . + 2*_N_Cmps+ + _suf*2*_N + 2*_suf
                            200 .endm
                            201 
                            202 ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
                            203 ;;  Zero-pad an array of size n.
                            204 .mdelete DefineComponentArray
                            205 .macro DefineComponentArray _N
                            206     .rept _N
                            207         .dw 0x0000
                            208     .endm
                            209 .endm
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
                     0018    80 STEP_HORIZONTAL_SPEED       = 0x0018
                     0100    81 MAX_HORIZONTAL_SPEED_POS    = 0x0100
                     FF00    82 MAX_HORIZONTAL_SPEED_NEG    = 0xff00
                             83 
                     0030    84 STEP_VERTICAL_SPEED       = 0x0030
                     0200    85 MAX_VERTICAL_SPEED_POS    = 0x0200
                     FE00    86 MAX_VERTICAL_SPEED_NEG    = 0xfe00
                             87 
                     000A    88 MAX_ENTITIES = 10
                             89 
                             90 
                     0000    91 nullptr = 0x0000
                             92 
                             93 ;;==============================================================================================================================
                             94 ;;==============================================================================================================================
                             95 ;;  ENTITY TYPE MASKS AND BITS
                             96 ;;==============================================================================================================================
                             97 ;;==============================================================================================================================
                     0000    98 e_type_default          = 0x00
                     0001    99 e_type_player           = 0x01
                     0002   100 e_type_ball             = 0x02
                     0004   101 e_type_life_potion      = 0x04
                     0008   102 e_type_mob              = 0x08
                     0010   103 e_type_shield           = 0x10
                     0020   104 e_type_dead             = 0x20
                     00FF   105 e_type_invalid          = 0xff
                            106 
                            107 ;;===============================================================================
                            108 ;;tipos de componentes
                            109 ;;===============================================================================
                     0000   110 e_cmp          = 0
                     0001   111 e_cmp_alive    = 0x01   ;;entidad renderizable
                     0002   112 e_cmp_render   = 0x02   ;;entidad renderizable
                     0004   113 e_cmp_physics  = 0x04   ;;entidad que se puede mover
                     0008   114 e_cmp_input    = 0x08   ;;entidad controlable por input  
                     0010   115 e_cmp_ai       = 0x10   ;;entidad controlable con ia
                     0020   116 e_cmp_animated = 0x20   ;;entidad animada
                     0040   117 e_cmp_collider = 0x40   ;;entidad que puede colisionar
                     0080   118 e_cmp_collisionable = 0x80   ;;entidad que puede ser colisionada
                     0047   119 e_cmp_paddle = e_cmp_alive | e_cmp_render | e_cmp_physics | e_cmp_collider  ;;componente por defecto
                     0087   120 e_cpm_ball = e_cmp_alive | e_cmp_render | e_cmp_physics | e_cmp_collisionable
                            121 
                            122 ;;===============================================================================
                            123 ;; Entity Component IDs
                            124 ;;===============================================================================
   0000                     125 DefEnum e_cmpID
                     0000     1     e_cmpID_offset = 0
   0000                     126 Enum e_cmpID Render
ASxxxx Assembler V02.00 + NoICE + SDCC mods  (Zilog Z80 / Hitachi HD64180), page 9.
Hexadecimal [16-Bits]



                     0000     1     e_cmpID_Render = e_cmpID_offset
                     0001     2     e_cmpID_offset = e_cmpID_offset + 1
   0000                     127 Enum e_cmpID Physics
                     0001     1     e_cmpID_Physics = e_cmpID_offset
                     0002     2     e_cmpID_offset = e_cmpID_offset + 1
   0000                     128 Enum e_cmpID AI
                     0002     1     e_cmpID_AI = e_cmpID_offset
                     0003     2     e_cmpID_offset = e_cmpID_offset + 1
   0000                     129 Enum e_cmpID Animation
                     0003     1     e_cmpID_Animation = e_cmpID_offset
                     0004     2     e_cmpID_offset = e_cmpID_offset + 1
   0000                     130 Enum e_cmpID Collision
                     0004     1     e_cmpID_Collision = e_cmpID_offset
                     0005     2     e_cmpID_offset = e_cmpID_offset + 1
   0000                     131 Enum e_cmpID Num_Components
                     0005     1     e_cmpID_Num_Components = e_cmpID_offset
                     0006     2     e_cmpID_offset = e_cmpID_offset + 1
                            132 
                            133 
                            134 
                            135 ;; Keyboard constants
                     000A   136 BUFFER_SIZE = 10
                     00FF   137 ZERO_KEYS_ACTIVATED = #0xFF
                            138 
                            139 ;; Score constants
                     0004   140 SCORE_NUM_BYTES = 4
                            141 
                            142 ;; SMALL NUMBERS CONSTANTS
                     0002   143 S_SMALL_NUMBERS_WIDTH = 2
                     0005   144 S_SMALL_NUMBERS_HEIGHT = 5
                            145 ;; Font constants
                     0002   146 FONT_WIDTH = 2
                     0009   147 FONT_HEIGHT = 9
                            148 
                            149 
                            150 ;;===============================================================================
                            151 ;; ENTITIY SCTRUCTURE CREATION
                            152 ;;===============================================================================
   0000                     153 BeginStruct e
                     0000     1     e_offset = 0
   0000                     154 Field e, ptr                , 2
                     0000     1     e_ptr = e_offset
                     0002     2     e_offset = e_offset + 2
   0000                     155 Field e, cmps               , 1
                     0002     1     e_cmps = e_offset
                     0003     2     e_offset = e_offset + 1
   0000                     156 Field e, type               , 1
                     0003     1     e_type = e_offset
                     0004     2     e_offset = e_offset + 1
   0000                     157 Field e, color              , 1
                     0004     1     e_color = e_offset
                     0005     2     e_offset = e_offset + 1
   0000                     158 Field e, x                  , 2
                     0005     1     e_x = e_offset
                     0007     2     e_offset = e_offset + 2
ASxxxx Assembler V02.00 + NoICE + SDCC mods  (Zilog Z80 / Hitachi HD64180), page 10.
Hexadecimal [16-Bits]



   0000                     159 Field e, y                  , 2
                     0007     1     e_y = e_offset
                     0009     2     e_offset = e_offset + 2
   0000                     160 Field e, w                  , 1
                     0009     1     e_w = e_offset
                     000A     2     e_offset = e_offset + 1
   0000                     161 Field e, h                  , 1
                     000A     1     e_h = e_offset
                     000B     2     e_offset = e_offset + 1
   0000                     162 Field e, end_x              , 1
                     000B     1     e_end_x = e_offset
                     000C     2     e_offset = e_offset + 1
   0000                     163 Field e, end_y              , 1
                     000C     1     e_end_y = e_offset
                     000D     2     e_offset = e_offset + 1
   0000                     164 Field e, last_x             , 1
                     000D     1     e_last_x = e_offset
                     000E     2     e_offset = e_offset + 1
   0000                     165 Field e, last_y             , 1
                     000E     1     e_last_y = e_offset
                     000F     2     e_offset = e_offset + 1
   0000                     166 Field e, vx                 , 2
                     000F     1     e_vx = e_offset
                     0011     2     e_offset = e_offset + 2
   0000                     167 Field e, vy                 , 2
                     0011     1     e_vy = e_offset
                     0013     2     e_offset = e_offset + 2
   0000                     168 Field e, sprite             , 2
                     0013     1     e_sprite = e_offset
                     0015     2     e_offset = e_offset + 2
   0000                     169 Field e, address            , 2
                     0015     1     e_address = e_offset
                     0017     2     e_offset = e_offset + 2
   0000                     170 Field e, p_address          , 2
                     0017     1     e_p_address = e_offset
                     0019     2     e_offset = e_offset + 2
   0000                     171 Field e, on_platform        , 1
                     0019     1     e_on_platform = e_offset
                     001A     2     e_offset = e_offset + 1
   0000                     172 Field e, orientation        , 1
                     001A     1     e_orientation = e_offset
                     001B     2     e_offset = e_offset + 1
   0000                     173 Field e, dashing            , 1
                     001B     1     e_dashing = e_offset
                     001C     2     e_offset = e_offset + 1
   0000                     174 Field e, animation_ptr      , 2
                     001C     1     e_animation_ptr = e_offset
                     001E     2     e_offset = e_offset + 2
   0000                     175 Field e, animation_status   , 1
                     001E     1     e_animation_status = e_offset
                     001F     2     e_offset = e_offset + 1
   0000                     176 Field e, moved              , 1
                     001F     1     e_moved = e_offset
                     0020     2     e_offset = e_offset + 1
   0000                     177 EndStruct e
ASxxxx Assembler V02.00 + NoICE + SDCC mods  (Zilog Z80 / Hitachi HD64180), page 11.
Hexadecimal [16-Bits]



                     0020     1     sizeof_e = e_offset
                            178 
                            179 ;;===============================================================================
                            180 ;; GLOBAL VARIABLES
                            181 ;;===============================================================================
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
   24BE 20 20 20 20 20 20    31 string_buffer:: .asciz "          "
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
   0D71                      55 sys_util_h_times_e::
   0D71 16 00         [ 7]   56   ld d,#0
   0D73 6A            [ 4]   57   ld l,d
   0D74 CB 24         [ 8]   58   sla h 
   0D76 30 01         [12]   59   jr nc,.+3 
   0D78 6B            [ 4]   60   ld l,e
   0D79 29            [11]   61   add hl,hl 
   0D7A 30 01         [12]   62   jr nc,.+3 
   0D7C 19            [11]   63   add hl,de
   0D7D 29            [11]   64   add hl,hl 
   0D7E 30 01         [12]   65   jr nc,.+3 
   0D80 19            [11]   66   add hl,de
   0D81 29            [11]   67   add hl,hl 
   0D82 30 01         [12]   68   jr nc,.+3 
   0D84 19            [11]   69   add hl,de
   0D85 29            [11]   70   add hl,hl 
   0D86 30 01         [12]   71   jr nc,.+3 
   0D88 19            [11]   72   add hl,de
   0D89 29            [11]   73   add hl,hl 
   0D8A 30 01         [12]   74   jr nc,.+3 
   0D8C 19            [11]   75   add hl,de
ASxxxx Assembler V02.00 + NoICE + SDCC mods  (Zilog Z80 / Hitachi HD64180), page 13.
Hexadecimal [16-Bits]



   0D8D 29            [11]   76   add hl,hl 
   0D8E 30 01         [12]   77   jr nc,.+3 
   0D90 19            [11]   78   add hl,de
   0D91 29            [11]   79   add hl,hl 
   0D92 D0            [11]   80   ret nc 
   0D93 19            [11]   81   add hl,de
   0D94 C9            [10]   82   ret
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
   0D95                      97 sys_util_hl_div_c::
   0D95 06 10         [ 7]   98        ld b,#16
   0D97 AF            [ 4]   99        xor a
   0D98 29            [11]  100          add hl,hl
   0D99 17            [ 4]  101          rla
   0D9A B9            [ 4]  102          cp c
   0D9B 38 02         [12]  103          jr c,.+4
   0D9D 2C            [ 4]  104            inc l
   0D9E 91            [ 4]  105            sub c
   0D9F 10 F7         [13]  106          djnz .-7
   0DA1 C9            [10]  107        ret
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
   0DA2                     120 sys_util_absHL::
   0DA2 CB 7C         [ 8]  121   bit #7,h
   0DA4 C8            [11]  122   ret z
   0DA5 AF            [ 4]  123   xor a
   0DA6 95            [ 4]  124   sub l
   0DA7 6F            [ 4]  125   ld l,a
   0DA8 9F            [ 4]  126   sbc a,a
   0DA9 94            [ 4]  127   sub h
   0DAA 67            [ 4]  128   ld h,a
   0DAB C9            [10]  129   ret
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
   0DAC                     144 sys_util_BCD_GetEnd::
                            145 ;Some of our commands need to start from the most significant byte
                            146 ;This will shift HL and DE along b bytes
   0DAC C5            [11]  147 	push bc
   0DAD 48            [ 4]  148 	ld c,b	;We want to add BC, but we need to add one less than the number of bytes
   0DAE 0D            [ 4]  149 	dec c
   0DAF 06 00         [ 7]  150 	ld b,#0
   0DB1 09            [11]  151 	add hl,bc
   0DB2 EB            [ 4]  152 	ex de, hl	;We've done HL, but we also want to do DE
   0DB3 09            [11]  153 	add hl,bc
   0DB4 EB            [ 4]  154 	ex de, hl
   0DB5 C1            [10]  155 	pop bc
   0DB6 C9            [10]  156 	ret
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
   0DB7                     170 sys_util_BCD_Add::
   0DB7 B7            [ 4]  171     or a
   0DB8                     172 BCD_Add_Again:
   0DB8 1A            [ 7]  173     ld a, (de)
   0DB9 8E            [ 7]  174     adc (hl)
   0DBA 27            [ 4]  175     daa
   0DBB 12            [ 7]  176     ld (de), a
   0DBC 13            [ 6]  177     inc de
   0DBD 23            [ 6]  178     inc hl
   0DBE 10 F8         [13]  179     djnz BCD_Add_Again
   0DC0 C9            [10]  180     ret
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
   0DC1                     194 sys_util_BCD_Compare::
   0DC1 06 04         [ 7]  195   ld b, #SCORE_NUM_BYTES
   0DC3 CD AC 0D      [17]  196   call sys_util_BCD_GetEnd
   0DC6                     197 BCD_cp_direct:
   0DC6 1A            [ 7]  198   ld a, (de)
   0DC7 BE            [ 7]  199   cp (hl)
   0DC8 D8            [11]  200   ret c
   0DC9 C0            [11]  201   ret nz
   0DCA 1B            [ 6]  202   dec de
   0DCB 2B            [ 6]  203   dec hl
   0DCC 10 F8         [13]  204   djnz BCD_cp_direct
   0DCE B7            [ 4]  205   or a                    ;; Clear carry
   0DCF C9            [10]  206   ret
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
   0DD0                     217 sys_util_get_random_number::
   0DD0 32 DA 0D      [13]  218   ld (#random_max_number), a
   0DD3 CD 0B 23      [17]  219   call cpct_getRandom_mxor_u8_asm
   0DD6 7D            [ 4]  220   ld a, l                             ;; Calculates a pseudo modulus of max number
   0DD7 26 00         [ 7]  221   ld h,#0                             ;; Load hl with the random number
                     0069   222 random_max_number = .+1
   0DD9 0E 00         [ 7]  223   ld c, #0                            ;; Load c with the max number
   0DDB 06 00         [ 7]  224   ld b, #0
   0DDD                     225 _random_mod_loop:
   0DDD B7            [ 4]  226   or a                                ;; ??
   0DDE ED 42         [15]  227   sbc hl,bc                           ;; hl = hl - bc
   0DE0 F2 DD 0D      [10]  228   jp p, _random_mod_loop              ;; Jump back if hl > 0
   0DE3 09            [11]  229   add hl,bc                           ;; Adds MAX_MODEL_CARD to hl back to get back to positive values
   0DE4 7D            [ 4]  230   ld a,l                              ;; loads the normalized random number in a
   0DE5 C9            [10]  231 ret
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
   0DE6                     242 sys_util_delay::
   0DE6 C5            [11]  243   push bc
   0DE7 CD BC 23      [17]  244   call cpct_waitVSYNCStart_asm
   0DEA C1            [10]  245   pop bc
   0DEB 10 F9         [13]  246   djnz sys_util_delay
   0DED C9            [10]  247   ret
