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
.include "man/entities.h.s"
.include "sys/util.h.s"
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
;; sys_collision_check_collider_colisionable
;;
;;  Initilizes render system
;;  Input: ix : pointer to the colider entity
;;  Input: iy : pointer to the collisionable entity
;;  Output: carry activated if no collision
;;  Modified: AF, BC, HL
;;
;;  Code copied from lronaldo (https://www.youtube.com/watch?v=f-4F7SoaHFQ)
sys_collision_check_collider_colisionable::
    ;; x axis collision
    ;; case 1
    ld a, e_x(iy)                   ;; a = iy.x
    ld b, a                         ;; b = a = iy.x
    add e_w(iy)                     ;; a = iy.x + iy.w
    dec a                           ;; a = iy.x + iy.w - 1
    ld c, e_x(ix)                   ;; c = ix.x
    sub c                           ;; a = iy.x + iy.w - 1 - ix.x
    ret c                           ;; return if no collision
    ;; case 2
    ld a, c                         ;; a = ix.x
    add e_w(ix)                     ;; a = ix.x + ix.w
    dec a                           ;; a = ix.x + ix.w - 1
    sub b                           ;; a = iy.x + ix.w - 1 - iy.x
    ret c                           ;; return if no collision

    ;; y axis collision
    ;; case 3
    ld a, e_y(iy)                   ;; a = iy.y
    ld b, a                         ;; b = a = iy.y
    add e_h(iy)                     ;; a = iy.y + iy.y
    dec a                           ;; a = iy.y + iy.y - 1
    ld c, e_y(ix)                   ;; c = ix.y
    sub c                           ;; a = iy.y + iy.y - 1 - ix.y
    ret c                           ;; return if no collision
    ;; case 4
    ld a, c                         ;; a = ix.y
    add e_h(ix)                     ;; a = ix.y + ix.y
    dec a                           ;; a = ix.y + ix.y - 1
    sub b                           ;; a = iy.y + ix.y - 1 - iy.y
    ;;ret c                           ;; return if no collision - unncessary
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
sys_collision_collider_colisionable::
    call sys_collision_check_collider_colisionable  ;; check if there is a collision
    ret c                                           ;; return if there is no collisión between ix and iy

    ;; change x velocity because of the collision
    ld h, e_vx(iy)
    ld l, e_vx+1(iy)
    call sys_util_negHL
    ld e_vx(iy), h
    ld e_vx+1(iy), l
    ld e_moved(iy), #1

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
sys_collision_update_one_collider::
    ld hl, #sys_collision_collider_colisionable
    ld b, #e_cmp_collisionable
    call man_entity_forall_matching_iy
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

    call sys_collision_update_one_collider

	pop hl

    jr _loop

    ret
