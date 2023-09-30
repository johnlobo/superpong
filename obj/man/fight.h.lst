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
ASxxxx Assembler V02.00 + NoICE + SDCC mods  (Zilog Z80 / Hitachi HD64180), page 2.
Hexadecimal [16-Bits]



                             16 .include "common.h.s"
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
                            106 .macro DefineEntity _cpms, _ptr, _type, _x, _y, _w, _h, _vx, _vy, _sprite, _address, _p_address, _on_platform, _orientation, _anim_ptr, _anim_status
                            107     .dw _ptr
                            108     .db _cpms
                            109     .db _type
ASxxxx Assembler V02.00 + NoICE + SDCC mods  (Zilog Z80 / Hitachi HD64180), page 5.
Hexadecimal [16-Bits]



                            110     .dw _x
                            111     .dw _y
                            112     .db _w
                            113     .db _h
                            114     .db _x+_w
                            115     .db _y+_h
                            116     .db #0
                            117     .db #0
                            118     .dw _vx
                            119     .dw _vy
                            120     .dw _sprite
                            121     .dw _address
                            122     .dw _p_address
                            123     .db #1           ;; moved 1 default
                            124     .db _on_platform
                            125     .db _orientation ;; 0: right, 1:left
                            126     .db #0           ;; dashing
                            127     .dw _anim_ptr
                            128     .db _anim_status
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
ASxxxx Assembler V02.00 + NoICE + SDCC mods  (Zilog Z80 / Hitachi HD64180), page 6.
Hexadecimal [16-Bits]



                            165     _Tname'_num:         .db 0
                            166     _Tname'_list:        .dw 0x0000
                            167     _Tname'_free_list:   .dw _Tname'_array
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
                             67 
                             68 
                             69 ;;===============================================================================
                             70 ;; DEFINED CONSTANTS
                             71 ;;===============================================================================
                             72 
ASxxxx Assembler V02.00 + NoICE + SDCC mods  (Zilog Z80 / Hitachi HD64180), page 8.
Hexadecimal [16-Bits]



                     0016    73 COF                     = #0x0016           ;; Coefficient of Friction
                     0024    74 GRAVITY                 = #0x0024           ;; Gravity
                     0200    75 DASH_IMPULSE            = 0x0200
                     000A    76 DASH_TIMER              = 10
                     0018    77 STEP_HORIZONTAL_SPEED   = 0x0018
                     0100    78 MAX_HORIZONTAL_SPEED_POS    = 0x0100
                     FF00    79 MAX_HORIZONTAL_SPEED_NEG    = 0xff00
                             80 
                     000A    81 MAX_ENTITIES = 10
                             82 
                             83 
                     0000    84 nullptr = 0x0000
                             85 
                             86 ;;==============================================================================================================================
                             87 ;;==============================================================================================================================
                             88 ;;  ENTITY TYPE MASKS AND BITS
                             89 ;;==============================================================================================================================
                             90 ;;==============================================================================================================================
                     0000    91 e_type_default          = 0x00
                     0001    92 e_type_player           = 0x01
                     0002    93 e_type_poison           = 0x02
                     0004    94 e_type_life_potion      = 0x04
                     0008    95 e_type_mob              = 0x08
                     0010    96 e_type_shield           = 0x10
                     0020    97 e_type_dead             = 0x20
                     00FF    98 e_type_invalid          = 0xff
                             99 
                            100 ;;===============================================================================
                            101 ;;tipos de componentes
                            102 ;;===============================================================================
                     0000   103 e_cmp          = 0
                     0001   104 e_cmp_alive    = 0x01   ;;entidad renderizable
                     0002   105 e_cmp_render   = 0x02   ;;entidad renderizable
                     0004   106 e_cmp_physics  = 0x04   ;;entidad que se puede mover
                     0008   107 e_cmp_input    = 0x08   ;;entidad controlable por input  
                     0010   108 e_cmp_ai       = 0x10   ;;entidad controlable con ia
                     0020   109 e_cmp_animated = 0x20   ;;entidad animada
                     0040   110 e_cmp_collider = 0x40   ;;entidad que puede colisionar
                     0080   111 e_cmp_collisionable = 0x80   ;;entidad que puede ser colisionada
                     0047   112 e_cmp_default = e_cmp_alive | e_cmp_render | e_cmp_physics | e_cmp_collider  ;;componente por defecto
                            113 
                            114 ;;===============================================================================
                            115 ;; Entity Component IDs
                            116 ;;===============================================================================
   0000                     117 DefEnum e_cmpID
                     0000     1     e_cmpID_offset = 0
   0000                     118 Enum e_cmpID Render
                     0000     1     e_cmpID_Render = e_cmpID_offset
                     0001     2     e_cmpID_offset = e_cmpID_offset + 1
   0000                     119 Enum e_cmpID Physics
                     0001     1     e_cmpID_Physics = e_cmpID_offset
                     0002     2     e_cmpID_offset = e_cmpID_offset + 1
   0000                     120 Enum e_cmpID AI
                     0002     1     e_cmpID_AI = e_cmpID_offset
                     0003     2     e_cmpID_offset = e_cmpID_offset + 1
ASxxxx Assembler V02.00 + NoICE + SDCC mods  (Zilog Z80 / Hitachi HD64180), page 9.
Hexadecimal [16-Bits]



   0000                     121 Enum e_cmpID Animation
                     0003     1     e_cmpID_Animation = e_cmpID_offset
                     0004     2     e_cmpID_offset = e_cmpID_offset + 1
   0000                     122 Enum e_cmpID Collisionable
                     0004     1     e_cmpID_Collisionable = e_cmpID_offset
                     0005     2     e_cmpID_offset = e_cmpID_offset + 1
   0000                     123 Enum e_cmpID Num_Components
                     0005     1     e_cmpID_Num_Components = e_cmpID_offset
                     0006     2     e_cmpID_offset = e_cmpID_offset + 1
                            124 
                            125 
                            126 
                            127 ;; Keyboard constants
                     000A   128 BUFFER_SIZE = 10
                     00FF   129 ZERO_KEYS_ACTIVATED = #0xFF
                            130 
                            131 ;; Score constants
                     0004   132 SCORE_NUM_BYTES = 4
                            133 
                            134 ;; SMALL NUMBERS CONSTANTS
                     0002   135 S_SMALL_NUMBERS_WIDTH = 2
                     0005   136 S_SMALL_NUMBERS_HEIGHT = 5
                            137 ;; Font constants
                     0002   138 FONT_WIDTH = 2
                     0009   139 FONT_HEIGHT = 9
                            140 
                            141 
                            142 ;;===============================================================================
                            143 ;; ENTITIY SCTRUCTURE CREATION
                            144 ;;===============================================================================
   0000                     145 BeginStruct e
                     0000     1     e_offset = 0
   0000                     146 Field e, ptr                , 2
                     0000     1     e_ptr = e_offset
                     0002     2     e_offset = e_offset + 2
   0000                     147 Field e, cmps               , 1
                     0002     1     e_cmps = e_offset
                     0003     2     e_offset = e_offset + 1
   0000                     148 Field e, type               , 1
                     0003     1     e_type = e_offset
                     0004     2     e_offset = e_offset + 1
   0000                     149 Field e, x                  , 2
                     0004     1     e_x = e_offset
                     0006     2     e_offset = e_offset + 2
   0000                     150 Field e, y                  , 2
                     0006     1     e_y = e_offset
                     0008     2     e_offset = e_offset + 2
   0000                     151 Field e, w                  , 1
                     0008     1     e_w = e_offset
                     0009     2     e_offset = e_offset + 1
   0000                     152 Field e, h                  , 1
                     0009     1     e_h = e_offset
                     000A     2     e_offset = e_offset + 1
   0000                     153 Field e, end_x              , 1
                     000A     1     e_end_x = e_offset
ASxxxx Assembler V02.00 + NoICE + SDCC mods  (Zilog Z80 / Hitachi HD64180), page 10.
Hexadecimal [16-Bits]



                     000B     2     e_offset = e_offset + 1
   0000                     154 Field e, end_y              , 1
                     000B     1     e_end_y = e_offset
                     000C     2     e_offset = e_offset + 1
   0000                     155 Field e, last_x             , 1
                     000C     1     e_last_x = e_offset
                     000D     2     e_offset = e_offset + 1
   0000                     156 Field e, last_y             , 1
                     000D     1     e_last_y = e_offset
                     000E     2     e_offset = e_offset + 1
   0000                     157 Field e, vx                 , 2
                     000E     1     e_vx = e_offset
                     0010     2     e_offset = e_offset + 2
   0000                     158 Field e, vy                 , 2
                     0010     1     e_vy = e_offset
                     0012     2     e_offset = e_offset + 2
   0000                     159 Field e, sprite             , 2
                     0012     1     e_sprite = e_offset
                     0014     2     e_offset = e_offset + 2
   0000                     160 Field e, address            , 2
                     0014     1     e_address = e_offset
                     0016     2     e_offset = e_offset + 2
   0000                     161 Field e, p_address          , 2
                     0016     1     e_p_address = e_offset
                     0018     2     e_offset = e_offset + 2
   0000                     162 Field e, moved              , 1
                     0018     1     e_moved = e_offset
                     0019     2     e_offset = e_offset + 1
   0000                     163 Field e, on_platform        , 1
                     0019     1     e_on_platform = e_offset
                     001A     2     e_offset = e_offset + 1
   0000                     164 Field e, orientation        , 1
                     001A     1     e_orientation = e_offset
                     001B     2     e_offset = e_offset + 1
   0000                     165 Field e, dashing            , 1
                     001B     1     e_dashing = e_offset
                     001C     2     e_offset = e_offset + 1
   0000                     166 Field e, animation_ptr      , 2
                     001C     1     e_animation_ptr = e_offset
                     001E     2     e_offset = e_offset + 2
   0000                     167 Field e, animation_status   , 1
                     001E     1     e_animation_status = e_offset
                     001F     2     e_offset = e_offset + 1
   0000                     168 EndStruct e
                     001F     1     sizeof_e = e_offset
                            169 
                            170 ;;===============================================================================
                            171 ;; GLOBAL VARIABLES
                            172 ;;===============================================================================
ASxxxx Assembler V02.00 + NoICE + SDCC mods  (Zilog Z80 / Hitachi HD64180), page 11.
Hexadecimal [16-Bits]



                             17 
                             18 .module fight_manager
                             19 
                             20 ;;------------------------------------------------------------------------------
                             21 ;; Global constants
                             22 ;;------------------------------------------------------------------------------
                             23 
                     0003    24 COMBAT_CARDS_PER_ROUND = 3
                     000C    25 COMBAT_MAX_ROUNDS = 12
                             26 
                             27 ;;------------------------------------------------------------------------------
                             28 ;; Global variables
                             29 ;;------------------------------------------------------------------------------
                             30 
                             31 ;;------------------------------------------------------------------------------
                             32 ;; Global routines
                             33 ;;------------------------------------------------------------------------------
                             34 
                             35 .globl man_fight_init
                             36 .globl man_fight_update
                             37 
                             38 
                             39 ;;===============================================================================
                             40 ;; MACROS
                             41 ;;===============================================================================
                             42 
                             43 
                             44 ;;===============================================================================
                             45 ;; DATA ARRAY STRUCTURE CREATION
                             46 ;;===============================================================================
   0000                      47 BeginStruct fight
                     0000     1     fight_offset = 0
   0000                      48 Field fight, name , 30
                     0000     1     fight_name = fight_offset
                     001E     2     fight_offset = fight_offset + 30
   0000                      49 Field fight, rounds , 1
                     001E     1     fight_rounds = fight_offset
                     001F     2     fight_offset = fight_offset + 1
   0000                      50 Field fight, current_round , 1
                     001F     1     fight_current_round = fight_offset
                     0020     2     fight_offset = fight_offset + 1
   0000                      51 Field fight, cards, (COMBAT_MAX_ROUNDS*COMBAT_CARDS_PER_ROUND) 
                     0020     1     fight_cards = fight_offset
                     0044     2     fight_offset = fight_offset + (COMBAT_MAX_ROUNDS*COMBAT_CARDS_PER_ROUND)
   0000                      52 EndStruct fight
                     0044     1     sizeof_fight = fight_offset
