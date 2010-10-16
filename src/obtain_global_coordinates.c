/*
 * obtain_global_coordinates.c
 *
 *  Created on: 19/01/2010
 *      Author: fcastanedo
 *
 * obtain the common global coordinates applying the calibration data for each point
 * based on Reg Willson implementation of Tsai's calibration. http://www.cs.cmu.edu/~rgw/TsaiCode.html
 */

#include "apidis-main.h"

/**
 * obtain_global_coordinates
 *
 */
int
obtain_global_coordinates (track_t list_of_tracks [MAX_CAM - MIN_CAM][MAX_FRAMES])
{
  int i,j;
  char calibration_data_name[255];
  FILE *calibration_data_values;
  double Xf, Yf, Xw, Yw, Zw;

#ifdef DEBUG
  printf ("obtain_global_coordinates()\n");
#endif

  for (i = 0; i < (MAX_CAM - MIN_CAM); i++) {
    sprintf (calibration_data_name, "cal-cam%1d", i+1);
    printf ("Opening: %s \n", calibration_data_name);

    /* load the file with the calibration data */
    if ((calibration_data_values = fopen (calibration_data_name, "r")) == NULL) {
      printf ("Error opening file with calibration data: %s\n", calibration_data_name);
      exit(-1);
    }

    /* reading data calibration */
    load_cp_cc_data (calibration_data_values, &cp, &cc);
    fclose (calibration_data_values);

    printf ("camera: %d \n", i);
    print_cp_cc_data (stdout, &cp, &cc);

    for (j = 0; j < MAX_FRAMES; j++) {
      if (track_px_local (i,j) > 0) {
        Xf = (double) track_px_local (i,j);
        Yf = (double) track_py_local (i,j);
        Zw = 0.0;

        image_coord_to_world_coord (Xf, Yf, Zw, &Xw, &Yw);

        track_px_global (i,j) = (float) Xw;
        track_py_global (i,j) = (float) Yw;
        track_pz_global (i,j) = (float) Zw;

        if (track_px_local_kalman (i, j) != 0) { /* check!, it is possible that the kalman filter data is 0 */
          Xf = (double) track_px_local_kalman (i,j);
          Yf = (double) track_py_local_kalman (i,j);
          Zw = 0.0;
          /* transformation to global coordinates */
          image_coord_to_world_coord (Xf, Yf, Zw, &Xw, &Yw);
          /* save the data */
          track_px_global_kalman(i,j) = (float) Xw;
          track_py_global_kalman(i,j) = (float) Yw;
        }
        //printf ("obtain_global_coordinates() X: %f Y: %f\n", (float) Xw, (float) Yw);
#ifdef DEBUG
        printf ("Kalman filtered data: Cam: %d Frame: %d, Px_local: %f Py_local: %f Calibration. Px_global: %f Py_global: %f\n", i ,j ,
            track_px_local_kalman (i,j), track_py_local_kalman (i,j),
            track_px_global_kalman(i,j), track_py_global_kalman (i,j) );
        printf ("Data without filtered: Cam: %d Frame: %d, Px_local: %f Py_local: %f Calibration GT Px_global: %f Py_global: %f\n", i, j,
            track_px_local (i, j), track_px_local (i, j), track_px_global (i, j), track_py_global (i, j));
#endif
      }
    }
  }
return 0;
}
