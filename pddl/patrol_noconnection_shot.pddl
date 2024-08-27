(define (domain patrol)
(:requirements :strips :typing :adl :fluents :durative-actions)

;; Types ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:types
robot
waypoint
);; end Types ;;;;;;;;;;;;;;;;;;;;;;;;;

;; Predicates ;;;;;;;;;;;;;;;;;;;;;;;;;
(:predicates

(robot_at ?r - robot ?wp - waypoint)
(connected ?wp1 ?wp2 - waypoint)
(patrolled ?wp - waypoint)
(is_shot ?wp - waypoint)
(not_shot ?wp - waypoint)
(charging_point_at ?wp - waypoint)
(battery_low ?r - robot)
(battery_full ?r - robot)

);; end Predicates ;;;;;;;;;;;;;;;;;;;;
;; Functions ;;;;;;;;;;;;;;;;;;;;;;;;;
(:functions

);; end Functions ;;;;;;;;;;;;;;;;;;;;
;; Actions ;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(:durative-action askcharge
    :parameters (?r - robot ?wp1 ?wp2 - waypoint)
    :duration (= ?duration 5)
    :condition (and 
        (at start(robot_at ?r ?wp1))
        (at start(charging_point_at ?wp2))
        (at start(battery_low ?r))
    )
    :effect (and 
        (at start(not(robot_at ?r ?wp1)))
        (at end(robot_at ?r ?wp2))
    )
)

(:durative-action move
    :parameters (?r - robot ?wp1 ?wp2 - waypoint)
    :duration ( = ?duration 20)
    :condition (and
        (at start(robot_at ?r ?wp1))
        (over all(battery_full ?r))
        )
    :effect (and
        (at start(not(robot_at ?r ?wp1)))
        (at end(robot_at ?r ?wp2))
    )
)

(:durative-action charge
    :parameters (?r - robot ?wp - waypoint)
    :duration ( = ?duration 5)
    :condition (and
        (at start(robot_at ?r ?wp))
        (at start(charging_point_at ?wp))
        (at start(battery_low ?r))
    )
    :effect (and
         (at end(not(battery_low ?r)))
         (at end(battery_full ?r))
    )
)

(:durative-action patrol
    :parameters (?r - robot ?wp - waypoint)
    :duration ( = ?duration 15)
    :condition (and
        (at start(robot_at ?r ?wp))
       )
    :effect (and
        (at end(patrolled ?wp))
    )
)

(:durative-action shot
    :parameters (?r - robot ?wp - waypoint)
    :duration ( = ?duration 5)
    :condition (and
        (at start(robot_at ?r ?wp))
       )
    :effect (and
        (at end(is_shot ?wp))
    )
)

);; end Domain ;;;;;;;;;;;;;;;;;;;;;;;;
