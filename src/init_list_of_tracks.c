/*
 * init_list_of_tracks.c
 *
 *  Created on: 19/01/2010
 *      Author: fcastanedo
 */

#include "apidis-main.h"

/*
 *  Initialize the list of tracks
 */

int
init_list_of_tracks (track_t list_of_tracks[MAX_CAM][MAX_FRAMES])
{
  int i, j;
#ifdef DEBUG
  printf ("init_list_of_tracks()\n");
#endif
  for (i = 0; i < (MAX_CAM - MIN_CAM); i++)
    for (j = 0; j < MAX_FRAMES; j++){
        track_camid (i, j) = i;
        track_playerid (i, j) = 0;
        track_frame (i, j) = j;
        track_x (i, j) = 1.0;
        track_y (i, j) = 1.0;
        track_w (i, j) = 1.0;
        track_h (i, j) = 1.0;
        track_kalman_varx (i, j) = 0.0;
        track_kalman_vary (i, j) = 0.0;
        track_px_local (i, j) = 0.0;
        track_py_local (i, j) = 0.0;
        track_px_global (i, j) = 0.0;
        track_py_global (i, j) = 0.0;
        track_pz_global (i, j) = 0.0;
        track_fused_x (i, j) = 0.0;
        track_fused_y (i, j) = 0.0;

        /* to simulate sensors errors */
        track_fused_x_errors (i, j) = 0.0;
        track_fused_y_errors (i, j) = 0.0;
        track_px_global_errors (i, j) = 0.0;
        track_py_global_errors (i, j) = 0.0;
        track_error_x (i, j) = 0.0;
        track_error_y (i, j) = 0.0;
        track_cont_frames (i, j) = 0;
        track_error_medio_x (i, j) = 0.0;
        track_error_medio_y (i, j) = 0.0;
        track_desv_x (i, j) = 0.0;
        track_desv_y (i, j) = 0.0;
        px_global_mean_5 (i, j) = 0.0;
        py_global_mean_5 (i, j) = 0.0;
        px_global_var_5 (i, j) = 0.0;
        py_global_var_5 (i, j) = 0.0;
        cont_stat (i, j) = 0;
    }
return 0;
}
