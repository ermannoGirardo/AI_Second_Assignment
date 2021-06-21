(define (domain localization)

(:requirements :typing :durative-actions :numeric-fluents :negative-preconditions :action-costs :conditional-effects :equality :fluents )


(:types 	robot region 
)

(:predicates
		(robot_in ?v - robot ?r - region) (visited ?r - region )(flag)
		(not_flag)(path ?from ?to - region)
	      
)

(:functions 
		(act-cost) (triggered ?from ?to - region) (dummy)
)

(:durative-action goto_region
		:parameters (?v - robot ?from ?to - region)
		:duration (= ?duration 100)
		:condition (and (at start (robot_in ?v ?from))(at start(not_flag)))
	        :effect (and (at start (not (robot_in ?v ?from))) (at start (increase (triggered ?from ?to) 1))
		(at end (robot_in ?v ?to))  (at end (visited ?to))(at end(flag))
	        (at end (not(not_flag)))(at end (path ?from ?to)) 	
                )
)


(:durative-action localize
	:parameters (?from ?to - region)
	:duration (= ?duration 1)
	:condition (and 
		(at start (flag)
		)
		(at start (path ?from ?to))

	)
	:effect (and 
		(at end (assign (triggered ?from ?to) 0))
		(at end (increase (act-cost) (dummy)))
		(at end (not(flag)))
		(at end (not_flag))
		(at end (not (path ?from ?to)))
		
		)
	
)
)






