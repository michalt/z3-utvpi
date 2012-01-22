; Example from the paper by Lahiri and Musuvathi.

(declare-fun x () Int)
(declare-fun y () Int)
(declare-fun z () Int)
(declare-fun w () Int)

(assert (Utvpi Plus x Plus y (~ 5)))
(assert (Utvpi Plus w Minus x 4))
(assert (Utvpi Minus w Minus x 3))
(assert (Utvpi Plus z Minus y 2))
(assert (Utvpi Minus z Minus y 1))
