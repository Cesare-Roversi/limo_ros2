(define (domain patrol)
(:requirements :strips :typing :adl :fluents :durative-actions)

;; Types ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:types
robot
waypoint
object
charging_station
);; end Types ;;;;;;;;;;;;;;;;;;;;;;;;;

;; Predicates ;;;;;;;;;;;;;;;;;;;;;;;;;
(:predicates

;; Robot
(robot_at ?r - robot ?wp - waypoint)
(reachable ?wp - waypoint)
(nat_battery_low ?r - robot)

;; Object
(object_at ?o - object ?wp - waypoint)

;; Waypoint
(patrolled ?wp - waypoint)

;; Charging station
(charging_station_at ?cs - charging_station ?wp - waypoint)


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


(:durative-action patrol
  :parameters (?r - robot ?wp - waypoint)
  :duration ( = ?duration 5)
  :condition (at start (robot_at ?r ?wp))
  :effect (at end (patrolled ?wp))
)



(:durative-action charge
  :parameters (?r - robot ?cs - charging_station ?wp - waypoint)
  :duration ( = ?duration 10)
  :condition (and
        (at start (robot_at ?r ?wp))
        (at start (charging_station_at ?cs ?wp))
        )
  :effect (at end (nat_battery_low ?r))
)

);; end Domain ;;;;;;;;;;;;;;;;;;;;;;;;
