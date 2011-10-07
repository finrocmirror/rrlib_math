//
// You received this file as part of RRLib
// Robotics Research Library
//
// Copyright (C) Finroc GbR (finroc.org)
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
//
//----------------------------------------------------------------------
/*!\file    test_gsl_rng.cpp
 *
 * \author  Jens Wettach
 *
 * \date    2011-03-07
 *
 */
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <cstdio>
#include <cmath>
#include <cstring>
#include <sys/time.h>

#include <gsl/gsl_randist.h>

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "rrlib/math/tGSLRandomNumberGenerator.h"

//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------
#include <cassert>

//----------------------------------------------------------------------
// Namespace usage
//----------------------------------------------------------------------
using namespace rrlib::math;

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

int main(int argc, char **argv)
{
  char* mode(0);
  int n(1000);
  double sigma(1.0);
  if (argc >= 4)
  {
    mode = argv[1];
    n = atoi(argv[2]);
    sigma = atof(argv[3]);
  }
  else
  {
    printf("usage: gsl_rng_test <mode> <number_of_samples> <sigma> -p\n");
    printf("where mode is element of {gauss, fast_gauss, exp, fzi}\n");
    printf("-p is optional and prints calculated values to stdout at end of program\n");
    exit(0);
  }
//   printf ("generator type: %s\n", gsl_rng_name (r));
//   printf ("seed = %lu\n", gsl_rng_default_seed);
//   printf ("first value = %lu\n", gsl_rng_get (r));

//  FILE *f = fopen("data.txt", "w");
  //fprintf(f, "%f\n", gsl_ran_gaussian_ziggurat (r, 5.0));

//    printf("%f\n", gsl_ran_gaussian_ziggurat (r, sigma));
//   printf("%f\n", gsl_ran_exponential(r,1.0));
//    printf("%f\n", -1.00 *(rand() < (RAND_MAX >> 1) ? -1 : 1) * sigma * log(double(rand() + 1) / RAND_MAX));
//    x = gsl_ran_gaussian_ziggurat (r, sigma);

  timeval start, end;
  double *value = new double[n];

  if (strcmp(mode, "gauss") == 0)
  {
    gettimeofday(&start, 0);
    for (int i = 0; i < n; ++i)
    {
      value[i] = gsl_ran_gaussian(tGSLRandomNumberGenerator::GetInstance().Generator(), sigma);
    }
    gettimeofday(&end, 0);
    fprintf(stderr, "took %f ms\n", ((end.tv_sec*1000000 + end.tv_usec) - (start.tv_sec*1000000 + start.tv_usec)) / 1000.);
  }
  else if (strcmp(mode, "fast_gauss") == 0)
  {
    gettimeofday(&start, 0);
    for (int i = 0; i < n; ++i)
    {
      value[i] = gsl_ran_gaussian_ziggurat(tGSLRandomNumberGenerator::GetInstance().Generator(), sigma);
    }
    gettimeofday(&end, 0);
    fprintf(stderr, "took %f ms\n", ((end.tv_sec*1000000 + end.tv_usec) - (start.tv_sec*1000000 + start.tv_usec)) / 1000.);
  }
  else if (strcmp(mode, "exp") == 0)
  {
    gettimeofday(&start, 0);
    for (int i = 0; i < n; ++i)
    {
      value[i] = gsl_ran_exponential(tGSLRandomNumberGenerator::GetInstance().Generator(), sigma);
    }
    gettimeofday(&end, 0);
    fprintf(stderr, "took %f ms\n", ((end.tv_sec*1000000 + end.tv_usec) - (start.tv_sec*1000000 + start.tv_usec)) / 1000.);
  }
  else if (strcmp(mode, "fzi") == 0)
  {
    gettimeofday(&start, 0);
    for (int i = 0; i < n; ++i)
    {
      value[i] = -1.00 * (rand() < (RAND_MAX >> 1) ? -1 : 1) * sigma * log(double(rand() + 1) / RAND_MAX);
    }
    gettimeofday(&end, 0);
    fprintf(stderr, "took %f ms\n", ((end.tv_sec*1000000 + end.tv_usec) - (start.tv_sec*1000000 + start.tv_usec)) / 1000.);
  }
  if (argc == 5 && strcmp(argv[4], "-p") == 0)
  {
    for (int i = 0; i < n; ++i)
    {
      printf("%f\n", value[i]);
    }
  }
  delete []value;
  return 0;
};
