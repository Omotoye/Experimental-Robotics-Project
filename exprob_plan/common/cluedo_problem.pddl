(define (problem cluedo-problem)
    (:domain cluedo-domain)
    (:objects
        room1 room2 room3 room4 - room
        oracle-location - oracle-loc
    )

    (:init
        (at-robby oracle-location)

        (= (conn-length room1 room2) 15)
        (= (conn-length room1 room3) 15)
        (= (conn-length room1 room4) 20)
        (= (conn-length room1 oracle-location) 10)

        (= (conn-length room2 room1) 15)
        (= (conn-length room2 room4) 15)
        (= (conn-length room2 room3) 20)
        (= (conn-length room2 oracle-location) 10)

        (= (conn-length room3 room4) 15)
        (= (conn-length room3 room1) 15)
        (= (conn-length room3 room2) 20)
        (= (conn-length room3 oracle-location) 10)

        (= (conn-length room4 room2) 15)
        (= (conn-length room4 room3) 15)
        (= (conn-length room4 room1) 20)
        (= (conn-length room4 oracle-location) 10)

        (= (conn-length oracle-location room2) 10)
        (= (conn-length oracle-location room3) 10)
        (= (conn-length oracle-location room4) 10)
        (= (conn-length oracle-location room1) 10)

        (= (speed) 0.5)
        (= (search-dur) 5)
    )

    (:goal
        (and
            (game-over)
        )
    )
)