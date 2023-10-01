;;-----------------------------LICENSE NOTICE------------------------------------
;;
;;  This program is free software: you can redistribute it and/or modify
;;  it under the terms of the GNU Lesser General Public License as published by
;;  the Free Software Foundation, either version 3 of the License, or
;;  (at your option) any later version.
;;
;;  This program is distributed in the hope that it will be useful,
;;  but WITHOUT ANY WARRANTY; without even the implied warranty of
;;  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
;;  GNU Lesser General Public License for more details.
;;
;;  You should have received a copy of the GNU Lesser General Public License
;;  along with this program.  If not, see <http://www.gnu.org/licenses/>.
;;-------------------------------------------------------------------------------

.module game_manager

.include "man/game.h.s"
.include "man/entities.h.s"
.include "sys/render.h.s"
.include "sys/physics.h.s"
.include "sys/input.h.s"
.include "common.h.s"
.include "cpctelera.h.s"



;;
;; Start of _DATA area 
;;  SDCC requires at least _DATA and _CODE areas to be declared, but you may use
;;  any one of them for any purpose. Usually, compiler puts _DATA area contents
;;  right after _CODE area contents.
;;
.area _DATA

player1Tpl::
DefineEntity e_cmp_default, #0000, e_type_player, 15, 10, 80, 1, 20, 0, 0, #0000, #0000, #0000, 1, 0, #0000, 0

game_state:: .db MAIN_MENU   ;; Game state ----- 0: Game loop, 1: Main menu, 2: Map loading, 3: Pause menu, 4: Game over, 5: Victory
;;
;; Start of _CODE area
;; 
.area _CODE

;;-----------------------------------------------------------------
;;
;; man_game_init
;;
;;  
;;  Input: 
;;  Output: 
;;  Modified: AF, BC, DE, HL
;;
man_game_init::

    call man_entity_init
    
    ;; Create an entity in 100, 100
    ld hl, #player1Tpl                  ;; Template of the entity to create
    call man_entity_create              ;; Create new entity

    ;; Create an entity in 140, 100
    ld hl, #player1Tpl                  ;; Template of the entity to create
    call man_entity_create              ;; Create new entity
    ld e_x(ix), #60                     ;; x = 60
    ld e_color(ix), #1                  ;; color = pink 

    call sys_physics_init               ;; initilize physics system
    
    ret

;;-----------------------------------------------------------------
;;
;; man_game_update
;;
;;   
;;  Input: 
;;  Output: 
;;  Modified: AF, BC, DE, HL
;;
man_game_update::
    call sys_input_player_update

    call man_entity_getPlayerPosition
    call man_entity_getOponentPosition

    ;; Copy player movements on Oponent
    ld a, e_vx(ix)
    ld e_vx(iy), a
    ld a, e_vx+1(ix)
    ld e_vx+1(iy), a

    ld a, e_vy(ix)
    ld e_vy(iy), a
    ld a, e_vy+1(ix)
    ld e_vy+1(iy), a
    ;; -----------------------------
_exit_clone_input:

    call sys_physics_update
    call sys_render_update
    call sys_render_debug_entity
    ;;delay 
    ;;ld b, #10
    ;;call cpct_waitHalts_asm
    ret

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;  Gets the current state of the game loop
;;  MODIFIES:
;;      - A: Returns state
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
man_game_get_state::
    ld a, (game_state)
    ret

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;  Sets the current state of the game loop
;;  MODIFIES:
;;      - A: Returns state
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
man_game_set_state::
    ld (game_state), a
    ret
