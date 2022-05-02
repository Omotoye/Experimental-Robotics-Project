(define (problem cluedo-problem)
    (:domain cluedo-domain)
    (:objects
        room1 room2 room3 room4 oracle_loc - location 
    )

    (:init
        (at-robby room1)
        (oracle-loc oracle_loc)
        (= (conn-length room1 room2) 15)
        (= (conn-length room1 room3) 15)
        (= (conn-length room1 room4) 20)
        (= (conn-length room1 oracle_loc) 10)

        (= (conn-length room2 room1) 15)
        (= (conn-length room2 room4) 15)
        (= (conn-length room2 room3) 20)
        (= (conn-length room2 oracle_loc) 10)

        (= (conn-length room3 room4) 15)
        (= (conn-length room3 room1) 15)
        (= (conn-length room3 room2) 20)
        (= (conn-length room3 oracle_loc) 10)

        (= (conn-length room4 room2) 15)
        (= (conn-length room4 room3) 15)
        (= (conn-length room4 room1) 20)
        (= (conn-length room4 oracle_loc) 10)

        (= (conn-length oracle_loc room2) 10)
        (= (conn-length oracle_loc room3) 10)
        (= (conn-length oracle_loc room4) 10)
        (= (conn-length oracle_loc room1) 10)

        (= (speed) 0.5)
        (= (search-dur) 5)
    )

    (:goal
        (and
            (game-over)
        )
    )
)