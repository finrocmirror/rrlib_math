#include <stdio.h>
#include <gsl/gsl_randist.h>
#include <math.h>
#include <sys/time.h>
#include <string.h>

#include "rrlib/math/sGslRandomNumberGenerator.h"

using namespace rrlib::math;

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
  gsl_rng *r = sGslRandomNumberGenerator::GetGslRandomNumberGenerator();

  if (strcmp(mode, "gauss") == 0)
  {
    gettimeofday(&start, 0);
    for (int i = 0; i < n; ++i)
    {
      value[i] = gsl_ran_gaussian(r, sigma);
    }
    gettimeofday(&end, 0);
    fprintf(stderr, "took %f ms\n", ((end.tv_sec*1000000 + end.tv_usec) - (start.tv_sec*1000000 + start.tv_usec)) / 1000.);
  }
  else if (strcmp(mode, "fast_gauss") == 0)
  {
    gettimeofday(&start, 0);
    for (int i = 0; i < n; ++i)
    {
      value[i] = gsl_ran_gaussian_ziggurat(r, sigma);
    }
    gettimeofday(&end, 0);
    fprintf(stderr, "took %f ms\n", ((end.tv_sec*1000000 + end.tv_usec) - (start.tv_sec*1000000 + start.tv_usec)) / 1000.);
  }
  else if (strcmp(mode, "exp") == 0)
  {
    gettimeofday(&start, 0);
    for (int i = 0; i < n; ++i)
    {
      value[i] = gsl_ran_exponential(r, sigma);
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
