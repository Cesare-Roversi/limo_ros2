(define (domain limoros2)
(:requirements :strips :typing :adl :fluents :durative-actions)

;; Types ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:types
robot
waypoint
object
);; end Types ;;;;;;;;;;;;;;;;;;;;;;;;;

;; Predicates ;;;;;;;;;;;;;;;;;;;;;;;;;
(:predicates

(object_at ?o - object ?wp - waypoint)
(object_picked_by_robot ?o - object ?r - robot)

(robot_at ?r - robot ?wp - waypoint)
(can_robot_reach ?wp - waypoint)

(is_gripper_free ?r - robot)
(can_gripper_reach ?r - robot ?wp - waypoint)
(gripper_at ?r - robot ?ap - waypoint)

);; end Predicates ;;;;;;;;;;;;;;;;;;;;

;; Functions ;;;;;;;;;;;;;;;;;;;;;;;;;
(:functions

);; end Functions ;;;;;;;;;;;;;;;;;;;;
;; Actions ;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; !lasciamo (at end (can_gripper_reach ?r ?wp_gripper_dest)) ?
(:durative-action move
    :parameters (?r - robot ?wp_origin ?wp_dest - waypoint)
    :duration (= ?duration 5)
    :condition (and
        (at start (can_robot_reach ?wp_dest))
        (at start (robot_at ?r ?wp_origin))
        )
    :effect (and
        (at start (not (robot_at ?r ?wp_origin)))
        (at end (robot_at ?r ?wp_dest))
        (at end (can_gripper_reach ?r ?wp_dest))
    )
)


;; !Il problema sta tutto in come settiamo can_gripper_reach
(:durative-action gripper_move
    :parameters (?r - robot ?wp_gripper_origin ?wp_gripper_dest - waypoint)
    :duration (= ?duration 5)
    :condition (and
        (at start (can_gripper_reach ?r ?wp_gripper_dest))
        )
    :effect (and
        (at start (not (gripper_at ?wp_gripper_origin)))
        (at end (gripper_at ?r ?wp_gripper_dest))
        )
)

;;! arm_pick e arm_unload hanno entrambi un waypoint come argomento
;; per semplificare: quando un oggetto viene preso dal robot elimino la sua vecchia posizione (quella che aveva prima di essere preso)
(:durative-action arm_pick
    :parameters (?r - robot ?o - object ?wp_obj - waypoint)
    :duration (= ?duration 5)
    :condition (and
        (at start (is_gripper_free ?r))
        (at start (gripper_at ?r ?wp_obj))
        (at start (object_at ?o ?wp_obj))
        )
    :effect (and
        (at start (not (is_gripper_free ?r)))
        (at end (object_picked_by_robot ?o ?r))
        (at end (not (object_at ?o ?wp_obj)))
        )
)

(:durative-action arm_unload
    :parameters (?r - robot ?o - object ?wp_gripper - waypoint)
    :duration (= ?duration 5)
    :condition (and
        (at start (object_picked_by_robot ?o ?r))
        (at start (gripper_at ?r ?wp_gripper))
        )
    :effect (and
        (at end (not (object_picked_by_robot ?o ?r)))
        (at end (is_gripper_free ?r))
        (at end (object_at ?o ?wp_gripper)) 
        )
)


);; end Domain ;;;;;;;;;;;;;;;;;;;;;;;;