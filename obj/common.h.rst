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
                            168     _Tname'_num::         .db 0
                            169     _Tname'_list::        .dw nullptr
                            170     _Tname'_free_list::   .dw _Tname'_array
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
ASxxxx Assembler V02.00 + NoICE + SDCC mods  (Zilog Z80 / Hitachi HD64180), page 7.
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
ASxxxx Assembler V02.00 + NoICE + SDCC mods  (Zilog Z80 / Hitachi HD64180), page 8.
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
ASxxxx Assembler V02.00 + NoICE + SDCC mods  (Zilog Z80 / Hitachi HD64180), page 9.
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
ASxxxx Assembler V02.00 + NoICE + SDCC mods  (Zilog Z80 / Hitachi HD64180), page 10.
Hexadecimal [16-Bits]



                     0020     1     sizeof_e = e_offset
                            178 
                            179 ;;===============================================================================
                            180 ;; GLOBAL VARIABLES
                            181 ;;===============================================================================
