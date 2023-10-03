;;-----------------------------LICENSE NOTICE------------------------------------
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

.module physics_system

.include "sys/collision.h.s"
.include "man/components.h.s"
.include "common.h.s"
.include "cpctelera.h.s"


;;
;; Start of _DATA area 
;;  SDCC requires at least _DATA and _CODE areas to be declared, but you may use
;;  any one of them for any purpose. Usually, compiler puts _DATA area contents
;;  right after _CODE area contents.
;;
.area _DATA


;;
;; Start of _CODE area
;; 
.area _CODE


;;-----------------------------------------------------------------
;;
;; sys_physics_init
;;
;;  Initilizes render system
;;  Input: 
;;  Output: 
;;  Modified: AF, BC, DE, HL
;;
sys_collision_init::
    ;; set pointer array address 
    ld a, #e_cmpID_Collision
    call man_components_getArrayHL
    ld  (_ent_array_ptr), hl
    ret

;;-----------------------------------------------------------------
;;
;; sys_collision_update_one_entity
;;
;;  Initilizes render system
;;  Input: ix : pointer to the entity
;;  Output: 
;;  Modified: AF, BC, HL
;;
sys_collision_update_one_entity::
    ret

;;-----------------------------------------------------------------
;;
;; sys_collision_update
;;
;;  Initilizes collision system
;;  Input: 
;;  Output: 
;;  Modified: AF, BC, DE, HL
;;

sys_collision_update::

_ent_array_ptr = . + 1
    ld  hl, #0x0000

    _loop:
    ;;  Select the pointer to the entity with collision and prepare the next position for the next iteration.
    ld e, (hl)
    inc hl
    ld d, (hl)
    inc hl

    ;;  The entities are finished traversing when find a pointer to null.
    ld a, e
    or d
    ret z

    push hl

    ld__ixl_e
    ld__ixh_d

    call sys_collision_update_one_entity

	pop hl

    jr _loop

    ret
