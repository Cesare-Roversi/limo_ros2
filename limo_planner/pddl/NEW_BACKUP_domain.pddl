(define (domain robot_domain)

  (:requirements :strips :typing :durative-actions)

  (:types
    robot
    object
    waypoint
    charging_station
  )

  (:predicates
    (robot_at     ?r  - robot   ?wp  - waypoint)
    (arm_free     ?r  - robot)
    (arm_retracted ?r - robot)
    (picked_up_obj ?r - robot   ?o   - object)
    (not_battery_low  ?r  - robot)

    (object_at    ?o  - object  ?wp  - waypoint)
    (connected   ?wp1 - waypoint ?wp2 - waypoint)
    (patrolled ?wp - waypoint)

    (charging_station_at ?cs - charging_station ?wp - waypoint)
  )

  (:durative-action retract_arm
    :parameters (?r - robot)
    :duration (= ?duration 1)
    :condition (and )
    :effect (at end (arm_retracted ?r))
  )

  (:durative-action check_distance
    :parameters (?wp1 - waypoint ?wp2 - waypoint)
    :duration (= ?duration 1)
    :condition (and )
    :effect (at end (connected ?wp1 ?wp2))
  )

  (:durative-action move
    :parameters (?r - robot ?wp1 - waypoint ?wp2 - waypoint)
    :duration (= ?duration 1)
    :condition (and
      (at start (robot_at ?r ?wp1))
      (at start (arm_retracted ?r))
      (at start (connected ?wp1 ?wp2))
    )
    :effect (and
      (at end (robot_at ?r ?wp2))
      (at start (not (robot_at ?r ?wp1)))
    )
  )

  (:durative-action pickup_obj
    :parameters (?r - robot ?o - object ?wp - waypoint)
    :duration (= ?duration 1)
    :condition (and
      (at start (robot_at ?r ?wp))
      (at start (object_at ?o ?wp))
      (at start (arm_free ?r))
      (at start (arm_retracted ?r))
      (at start (not_battery_low ?r))
    )
    :effect (and
      (at end (picked_up_obj ?r ?o))
      (at start (not (arm_free ?r)))
      (at start (not (arm_retracted ?r)))
      (at start (not (object_at ?o ?wp)))
    )
  )

  (:durative-action put_down_object
    :parameters (?r - robot ?o - object ?wp - waypoint)
    :duration (= ?duration 1)
    :condition (and
      (at start (robot_at ?r ?wp))
      (at start (picked_up_obj ?r ?o))
      (at start (arm_retracted ?r))
      (at start (not_battery_low ?r))
    )
    :effect (and
      (at start (not (picked_up_obj ?r ?o)))
      (at end (arm_free ?r))
      (at end (object_at ?o ?wp))
      (at start (not (arm_retracted ?r)))
    )
  )

  (:durative-action move_arm
    :parameters (?r - robot)
    :duration (= ?duration 1)
    :condition (at start (not_battery_low ?r))
    :effect (at start (not (arm_retracted ?r)))
  )

  (:durative-action charge
    :parameters (?r - robot ?cs - charging_station ?wp - waypoint)
    :duration (= ?duration 1)
    :condition (and
      (at start (robot_at ?r ?wp))
      (at start (charging_station_at ?cs ?wp))
    )
    :effect (at end (not_battery_low ?r))
  )

  (:durative-action patrol
    :parameters (?r - robot ?wp - waypoint)
    :duration (= ?duration 1)
    :condition (and
      (at start (robot_at ?r ?wp))
      (at start (not_battery_low ?r))
    )
    :effect (at end (patrolled ?wp))
  )

)
