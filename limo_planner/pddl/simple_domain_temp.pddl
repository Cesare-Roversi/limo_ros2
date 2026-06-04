(define (domain patrol)
(:requirements :strips :typing :adl :fluents :durative-actions)

;; Types ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:types
robot
waypoint
object
);; end Types ;;;;;;;;;;;;;;;;;;;;;;;;;

;; Predicates ;;;;;;;;;;;;;;;;;;;;;;;;;
(:predicates

;; Robot
(robot_at ?r - robot ?wp - waypoint)
(robot_in_range ?r - robot ?wp - waypoint)
(reachable ?wp - waypoint)

;; Object
(object_at ?o - object ?wp - waypoint)

);; end Predicates ;;;;;;;;;;;;;;;;;;;;
;; Functions ;;;;;;;;;;;;;;;;;;;;;;;;;
(:functions

);; end Functions ;;;;;;;;;;;;;;;;;;;;

;; Actions ;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:durative-action move
    :parameters (?r - robot ?wp1 ?wp2 - waypoint)
    :duration ( = ?duration 5)
    :condition (and
        (at start(reachable ?wp2))
        (at start(robot_at ?r ?wp1))
        )
    :effect (and
        (at start(not(robot_at ?r ?wp1)))
        (at end(robot_at ?r ?wp2))
    )
)


(:durative-action get_in_range
    :parameters (?r - robot ?wp1 ?wp2 - waypoint)
    :duration ( = ?duration 5)
    :condition (and
        (at start(reachable ?wp2))
        (at start(robot_at ?r ?wp1))
        )
    :effect (and
        (at start(not(robot_at ?r ?wp1)))
        (at end(robot_at ?r ?wp2))
        (at end(robot_in_range ?r ?wp2))
    )
)

);; end Domain ;;;;;;;;;;;;;;;;;;;;;;;;
