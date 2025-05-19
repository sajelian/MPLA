(define (domain robplan)
(:requirements :typing :durative-actions :strips :fluents)
  (:types  
     turtlebot robot camera camera_eo camera_ir robo_arm charger - vehicle
     vehicle photo valve pump pipe sound gas_ind obj battery - subject
     city_location city - location
     waypoint battery_station - city_location
     route
     )

  (:predicates
    (at ?physical_obj1 - subject ?location1 - location)
    (available ?vehicle1 - vehicle)
    (available ?camera1 - camera)
    (available ?camera_eo1 - camera_eo)
    (available ?camera_ir1 - camera_ir)
    (available ?robo_arm1 - robo_arm)    
    (connects ?route1 - route ?location1 - location ?location2 - location)
    (in_city ?location1 - location ?city1 - city)
    (route_available ?route1 - route)
    (no_photo ?subject1 - subject)
    (photo ?subject1 - subject)
    (no_seals_check ?subject1 - subject)
    (seals_check ?subject1 - subject)
    (no_pump_check ?subject1 - subject)
    (pump_check ?subject1 - subject)
    (no_manipulate ?subject1 - valve)
    (manipulate ?subject1 - valve)
    (pump_check ?subject1 - subject)
    (battery-high ?battery - battery)
    (battery-medium ?battery - battery)
    (battery-low ?battery - battery)
    (battery-critical ?battery - battery)
   )

(:functions 
           (distance ?O - location ?L - location)
           (route-length ?O - route)
	    (speed ?V - vehicle)
            )

  (:durative-action move_robot_high_to_medium
       :parameters ( ?V - robot ?O - location ?L - location ?R - route ?B - battery)
       :duration (= ?duration (/ (route-length ?R) (speed ?V)))
       :condition (and 
			(at start (at ?V ?O))
            		(at start (connects ?R ?O ?L))
            		(at start (battery-high ?B))
       )
       :effect (and 
		  (at start (not (at ?V ?O)))
                  (at end (at ?V ?L))
                  (at start (not (battery-high ?B)))
                  (at end (battery-medium ?B))
        )
    )
    
   (:durative-action move_robot_medium_to_low
       :parameters ( ?V - robot ?O - location ?L - location ?R - route ?B - battery)
       :duration (= ?duration (/ (route-length ?R) (speed ?V)))
       :condition (and 
			(at start (at ?V ?O))
            		(at start (connects ?R ?O ?L))
            		(at start (battery-medium ?B))
       )
       :effect (and 
		  (at start (not (at ?V ?O)))
                  (at end (at ?V ?L))
                  (at start (not (battery-medium ?B)))
                  (at end (battery-low ?B))
        )
    )
    
   (:durative-action move_robot_low_to_critical
       :parameters ( ?V - robot ?O - location ?L - location ?R - route ?B - battery)
       :duration (= ?duration (/ (route-length ?R) (speed ?V)))
       :condition (and 
			(at start (at ?V ?O))
            		(at start (connects ?R ?O ?L))
            		(at start (battery-low ?B))
       )
       :effect (and 
		  (at start (not (at ?V ?O)))
                  (at end (at ?V ?L))
                  (at start (not (battery-low ?B)))
                  (at end (battery-critical ?B))
        )
    )
    
     (:durative-action charge_medium_battery
       :parameters (?V - robot ?L - location ?C - charger ?B - battery)
       :duration (= ?duration 5)
       :condition (and 
			(at start (at ?V ?L))
			(at start (at ?C ?L))
            		(at start (battery-medium ?B))
       )
       :effect (and
                  (at start (not (battery-medium ?B)))
                  (at end (battery-high ?B))
        )
    )

    (:durative-action charge_low_battery
       :parameters (?V - robot ?L - location ?C - charger ?B - battery)
       :duration (= ?duration 10)
       :condition (and 
			(at start (at ?V ?L))
			(at start (at ?C ?L))
            		(at start (battery-low ?B))
       )
       :effect (and
                  (at start (not (battery-low ?B)))
                  (at end (battery-high ?B))
        )
    )
    
 (:durative-action charge_critical_battery
       :parameters (?V - robot ?L - location ?C - charger ?B - battery)
       :duration (= ?duration 15)
       :condition (and 
			(at start (at ?V ?L))
			(at start (at ?C ?L))
            		(at start (battery-critical ?B))
       )
       :effect (and
                  (at start (not (battery-critical ?B)))
                  (at end (battery-high ?B))
        )
    )

 (:durative-action check_seals_valve_picture_EO
       :parameters ( ?V - robot ?L - location ?G - camera_eo ?B - valve)
       :duration (= ?duration 10)
       :condition (and 
            (over all (at ?V ?L))
            (at start (at ?B ?L))
            (at start (available ?G))
            (at start (no_seals_check ?B))
       )
       :effect (and 
	    (at start (not (no_seals_check ?B)))
            (at end (seals_check ?B))
        )
    )

 (:durative-action check_pump_picture_IR
       :parameters ( ?V - robot ?L - location ?G - camera_ir ?B - pump)
       :duration (= ?duration 10)
       :condition (and 
            (over all (at ?V ?L))
            (at start (at ?B ?L))
            (at start (available ?G))
            (at start (no_pump_check ?B))
       )
       :effect (and 
	    (at start (not (no_pump_check ?B)))
            (at end (pump_check ?B))
        )
    )
    

 (:durative-action photograph_subject
       :parameters ( ?V - robot ?L - location ?G - camera ?B - subject)
       :duration (= ?duration 10)
       :condition (and 
            (over all (at ?V ?L))
            (at start (at ?B ?L))
            (at start (available ?G))
            (at start (no_photo ?B))
       )
       :effect (and 
	    (at start (not (no_photo ?B)))
            (at end (photo ?B))
        )
    )
        

 (:durative-action manipulate_valve
       :parameters ( ?V - robot ?L - location ?A - robo_arm ?B - valve)
       :duration (= ?duration 10)
       :condition (and 
            (over all (at ?V ?L))
            (at start (at ?B ?L))
            (at start (seals_check ?B))
            (at start (available ?A))
            (at start (no_manipulate ?B))
       )
       :effect (and 
	    (at start (not (no_manipulate ?B)))
            (at end (manipulate ?B))
        )
    )
)
