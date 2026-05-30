(define (domain limoros2)
(:requirements :strips :typing :adl :fluents :durative-actions)

;; Types ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:types
robot
waypoint
arm_position
object
);; end Types ;;;;;;;;;;;;;;;;;;;;;;;;;

;; Predicates ;;;;;;;;;;;;;;;;;;;;;;;;;
(:predicates

(robot_at ?r - robot ?wp - waypoint)
(is_reachable ?wp - waypoint)
(free_arm_pos ?ap - arm_position)
(robot_arm_at ?r - robot ?ap - arm_position)
(object_picked_by_robot ?o - object ?r - robot)
(free_hand ?r - robot)

);; end Predicates ;;;;;;;;;;;;;;;;;;;;
;; Functions ;;;;;;;;;;;;;;;;;;;;;;;;;
(:functions

);; end Functions ;;;;;;;;;;;;;;;;;;;;
;; Actions ;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(:durative-action move
    :parameters (?r - robot ?wp1 ?wp2 - waypoint)
	:duration (= ?duration 5)
    :condition (and
        (at start (is_reachable ?wp2))
        (at start (robot_at ?r ?wp1))
        )
    :effect (and
        (at end(robot_at ?r ?wp2))
    )
)

(:durative-action move_with_object
    :parameters (?r - robot ?wp1 ?wp2 - waypoint ?o - object)
    :duration (= ?duration 5)
	:condition (and
        (at start (is_reachable ?wp2))
        (at start (robot_at ?r ?wp1))
		(at start (object_picked_by_robot ?o ?r))
        )
    :effect (and
        (at end(robot_at ?r ?wp2))
    )
)

(:durative-action arm_move
    :parameters (?r - robot ?ap - arm_position)
	:duration (= ?duration 5)
    :condition (and
        (at start (free_arm_pos ?ap))
		)
    :effect (and
        (at end(robot_arm_at ?r ?ap))
		)
)

(:durative-action arm_pick
    :parameters (?r - robot ?o - object)
	:duration (= ?duration 5)
    :condition (and
        (at start (free_hand ?r))
		)
    :effect (and
		(at end(object_picked_by_robot ?o ?r))
		)
)

(:durative-action arm_unload
    :parameters (?r - robot ?ap - arm_position ?o - object)
    :duration (= ?duration 5)
	:condition (and
        (at start(object_picked_by_robot ?o ?r))
		)
    :effect (and
		(at end(free_hand ?r))
		)
)


);; end Domain ;;;;;;;;;;;;;;;;;;;;;;;;
