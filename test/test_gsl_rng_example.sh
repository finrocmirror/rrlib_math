#! /bin/bash
#
# the following command concatenation
# 1. generates 100000 Gaussian random variates with mean zero and standard deviation 2 and prints them to stdout
# 2. generates a histogram with lower bound -10, upper bound 10 and 1000 bins with values read from stdin
# 3. plots the histogram
#
# run <test_gsl_rng> alone to look for other options

rrlib_math_test_gsl_rng gauss 100000 2 -p | gsl-histogram -10 10 1000 | awk '{print $1, $3 ; print $2, $3}'  | graph -T X

