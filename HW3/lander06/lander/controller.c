/*****************************************************************************/
/*
  controller.c: control strategy.
*/
/*****************************************************************************/

#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#include "matrix3.h"
#include "main.h"
#include "main2.h"
#include "sdfast/alien.h"
#include "sdfast/lander.h"

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
/* call this once to do one-time initialize: allocate memeory etc. */

void init_controller( SIM *s )
{
}

/*****************************************************************************/
/* call this many times to restart a controller */

void reinit_controller( SIM *s )
{
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/

int controller( SIM *s )
{
  FILE *fp;
  fp = fopen("problem_3.dat", "r");

  float x_pos[10000];
  float y_pos[10000];
  float z_pos[10000];
  float quat_0[10000];
  float quat_1[10000];
  float quat_2[10000];
  float quat_3[10000];
  int n;

  for (n = 0; n < 10000; n++)
    {
      fscanf(fp, "%f %f %f %f %f %f %f", &x_pos[n], &y_pos[n], &z_pos[n],
      &quat_0[n], &quat_1[n], &quat_2[n], &quat_3[n]);
    }


  int i;
  static int count = 0;
  double k_x = 1.0;
  double b_x = 2.0;
  double k_r = 1.0;
  double b_r = 2.0;
  double q_minus[N_Q];
  double q_diff[N_Q];

  /*
  // Handy for generating a data file for Matlab
  if ( count % 1000 == 0 )
    {
      printf( "%d ", count );
      for( i = 0; i < s->n_markers; i++ )
	{
	  printf( "%20.15f %20.15f %20.15f ", s->markers_lander[i][0],
		  s->markers_lander[i][1], s->markers_lander[i][2] );
	}
      printf( "\n" );
    }
  */

  // desired lander position
  s->lander_x_d[XX] = x_pos[count];
  s->lander_x_d[YY] = y_pos[count];
  s->lander_x_d[ZZ] = z_pos[count];

  // desired lander orientation
  s->lander_q_d[Q0] = quat_0[count];
  s->lander_q_d[Q1] = quat_1[count];
  s->lander_q_d[Q2] = quat_2[count];
  s->lander_q_d[Q3] = quat_3[count];

  // lander translational control (PD servo)
  for ( i = 0; i < 3; i++ )
    {
      s->lander_thrust_world[i] =
	k_x*( s->lander_x_d[i] - s->lander_x[i] ) +
	b_x*( - s->lander_xd[i] );
    }
  multiply_transpose_m3_v3( s->lander_r, s->lander_thrust_world,
			    s->lander_thrust );

  // lander orientation control
  // Attempt to do PD control, but orientations not vectors, so complicated
  // "subtract quaternions"
  invert_q( s->lander_q, q_minus );
  compose_q( q_minus, s->lander_q_d, q_diff );
  q_to_rotvec( q_diff, s->rotvec );
  // printf( "%g %g %g %g\n", q_diff[0], q_diff[1], q_diff[2], q_diff[3] );
  // printf( "%g %g %g\n", s->rotvec[0], s->rotvec[1], s->rotvec[2] );

  // PD servo for orientation
  // w and rotvec are in body coordinates.
  for ( i = 0; i < N_XYZ; i++ )
    {
      s->lander_torque[i] = k_r*s->rotvec[i] - b_r*s->lander_w[i];
    }
  multiply_m3_v3( s->lander_r, s->lander_torque, s->lander_torque_world );

  // To get better control should compute desired acceleration and use
  // inverse dynamics to compute torque.

  fclose(fp);

  count++;

  return 0;
}

/*****************************************************************************/
