# Warning!

SET your IDF_PATH and toolchain ( gcc, binutils, xtensa ... ) PATH on Makefile.

CHANGE the clock prescale in mcpwm.c file to make possible get 30000hz of pwm frequency (15Khz in center aligned mode)
See more here: https://gist.github.com/fabiovila/77c544286e9b0260a5a09823b47ba39f

# Results

Don't used in prodution but worked well with a three-phase motor of 1HP 220V in a board (designed by me) with IPM modules of 10A 600V.
