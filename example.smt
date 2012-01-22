(declare-fun x () Int)
(declare-fun y () Int)

(assert (or (Utvpi Plus x Minus y (~ 1)) (Utvpi Plus x Minus y 1)))
; (assert (Utvpi Plus x Minus y (~ 1))
(assert (Svpi Plus y 0))
(assert (Svpi Minus x (~ 1)))
