(define (domain limoros2)
(:requirements :strips :typing :adl :fluents :durative-actions)

;; Types ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:types
robot
position
arm_position
object
);; end Types ;;;;;;;;;;;;;;;;;;;;;;;;;

;; Predicates ;;;;;;;;;;;;;;;;;;;;;;;;;
(:predicates

(robot_at ?r - robot ?p - position)
(free ?p - position)
(free_arm_pos ?p - arm_position)
(robot_arm_at ?r - robot ?p - arm_position)
(object_picked_by_robot ?o - object ?r - robot)
(free_hand ?r - robot)

);; end Predicates ;;;;;;;;;;;;;;;;;;;;
;; Functions ;;;;;;;;;;;;;;;;;;;;;;;;;
(:functions

);; end Functions ;;;;;;;;;;;;;;;;;;;;
;; Actions ;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(:durative-action move
    :parameters (?r - robot ?p1 ?p2 - position)
	:duration (= ?duration 5)
    :condition (and
        (at start (free ?p2))
        (at start (robot_at ?r ?p1))
        )
    :effect (and
        (at end(robot_at ?r ?p2))
    )
)

(:durative-action move_with_object
    :parameters (?r - robot ?p1 ?p2 - position ?o - object)
    :duration (= ?duration 5)
	:condition (and
        (at start (free ?p2))
        (at start (robot_at ?r ?p1))
		(at start (object_picked_by_robot ?o ?r))
        )
    :effect (and
        (at end(robot_at ?r ?p2))
    )
)

(:durative-action arm_move
    :parameters (?r - robot ?p - arm_position)
	:duration (= ?duration 5)
    :condition (and
        (at start (free_arm_pos ?p))
		)
    :effect (and
        (at end(robot_arm_at ?r ?p))
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
    :parameters (?r - robot ?p - arm_position ?o - object)
    :duration (= ?duration 5)
	:condition (and
        (at start(object_picked_by_robot ?o ?r))
		)
    :effect (and
		(at end(free_hand ?r))
		)
)


);; end Domain ;;;;;;;;;;;;;;;;;;;;;;;;
