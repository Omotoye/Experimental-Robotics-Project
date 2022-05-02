(define (domain cluedo-domain)

    (:requirements :typing :fluents :durative-actions :duration-inequalities :negative-preconditions :disjunctive-preconditions :conditional-effects)
    (:types
        location
    )

    (:predicates
        (at-robby ?location - location) ; the location the robot is at 
        (visited ?room - location)
        (hint-found ?room - location)
        (hypothesis-found)
        (hint-added ?room - location)
        (oracle-loc ?oracle - location)
        (new-hypo)
        (hypo-announced)
        (hypo-correct)
        (game-over)
    )

    (:functions
        (conn-length ?from ?to - location)
        (speed)
        (search-dur)

    )

    (:durative-action go-to-room
        :parameters (?from ?to - location)
        :duration (= ?duration (/ (conn-length ?from ?to) (speed)))
        :condition (and
            (over all (not(oracle-loc ?to)))
            (at start (at-robby ?from))
            (at start (not (visited ?to)))
        )
        :effect (and
            (at start (not (at-robby ?from)))
            (at end (at-robby ?to))
            (at end (not (at-robby ?from)))
        )
    )

    (:durative-action go-to-oracle
        :parameters (?from ?to - location)
        :duration (= ?duration (/ (conn-length ?from ?to) (speed)))
        :condition (and
            (over all (oracle-loc ?to))
            (at start (at-robby ?from))
            (at start (hypothesis-found))
        )
        :effect (and
            (at start (not (at-robby ?from)))
            (at end (at-robby ?to))
            (at end (not (at-robby ?from)))
        )
    )

    (:durative-action search-hint
        :parameters (?room - location)
        :duration (= ?duration (search-dur))
        :condition (and
            (over all (not(oracle-loc ?room)))
            (at start (at-robby ?room))
            (at start (not (visited ?room)))
            (at start (not (hint-found ?room)))
        )
        :effect (and
            (at end (at-robby ?room))
            (at end (visited ?room))
            (at end (hint-found ?room))
            (at end (not (hint-added ?room)))
        )
    )

    (:action update-knowledge
        :parameters (?room - location)
        :precondition (and (not (hint-added ?room)) (hint-found ?room))
        :effect (and (hint-added ?room) (hint-found ?room) (new-hypo))
    )

    (:action check-hypothesis
        :parameters ()
        :precondition (and (new-hypo))
        :effect (and (not (new-hypo)) (hypothesis-found))
    )

    (:action announce-hypo
        :parameters (?loc - location)
        :precondition (and (at-robby ?loc) (oracle-loc ?loc) (hypothesis-found) (not (new-hypo)))
        :effect (and (hypo-announced) (at-robby ?loc) (not (hypothesis-found)) (not (new-hypo)))
    )

    (:action oracle-check
        :parameters (?loc - location)
        :precondition (and (at-robby ?loc) (oracle-loc ?loc) (hypo-announced))
        :effect (and (hypo-correct) (game-over))
    )
)