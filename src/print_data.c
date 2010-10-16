/*
 * print_data.c
 *
 * This file provides the implementation of several methods for printing
 * and saving the obtained data.
 *
 */

#include "apidis-main.h"

/**
* obtain_and_save_stat_values:
*
*
* Compute the Root Mean Square Error (RMSA) and Mean Absolute Error (MAE)
* between the point projected on the ground plane of the annotated ground-truht
* and the obtained with the fusion algorithm.
*
*/
int
obtain_and_save_stat_values (const char* name, track_t list_of_tracks [MAX_CAM - MIN_CAM][MAX_FRAMES])
{
  char filename[255];
  FILE *fp;
  float x_mae[MAX_CAM] = {0.0, 0.0, 0.0, 0.0, 0.0};
  float y_mae[MAX_CAM] = {0.0, 0.0, 0.0, 0.0, 0.0};
  float x_rmse[MAX_CAM] = {0.0, 0.0, 0.0, 0.0, 0.0};
  float y_rmse[MAX_CAM] = {0.0, 0.0, 0.0, 0.0, 0.0};
  int cont[MAX_CAM] = {0, 0, 0, 0, 0};
  int cam = 0, frame = 0;

  sprintf (filename, "%s-stats.csv", name);
  fp = fopen (filename, "w");
  fprintf (fp, "C1-X-MAE;C1-Y-MAE;C2-X-MAE;C2-Y-MAE;C4-X-MAE;C4-Y-MAE;C6-X-MAE;C6-Y-MAE;C7-X-MAE;C7-Y-MAE;C1-X-RMSE;C1-Y-RMSE;C2-X-RMSE;C2-Y-RMSE;C4-X-RMSE;C4-Y-RMSE;C6-X-RMSE;C6-Y-RMSE;C7-X-RMSE;C7-Y-RMSE;\n");

  for (frame = 0; frame < MAX_FRAMES; frame++) {
    for (cam = 0; cam < (MAX_CAM - MIN_CAM); cam++) {
      if (track_fused_x (cam, frame) > 0 && track_fused_x (cam, frame) < X_MAX) { /* check ranges */
        cont[cam]++;
        x_mae[cam] += fabs (track_px_global (cam, frame) - track_fused_x (cam, frame));
        y_mae[cam] += fabs (track_py_global (cam, frame) - track_fused_y (cam, frame));
        x_rmse[cam] += pow ((track_px_global (cam, frame) - track_fused_x (cam, frame)) ,2);
        y_rmse[cam] += pow ((track_py_global (cam, frame) - track_fused_y (cam, frame)) ,2);
      }
    }
  }
  for (cam = 0; cam < (MAX_CAM - MIN_CAM); cam++) {
    x_mae[cam] = x_mae[cam] / (float) cont[cam];
    y_mae[cam] = y_mae[cam] / (float) cont[cam];
    x_rmse[cam] = sqrt((x_rmse[cam] / (float) cont[cam]));
    y_rmse[cam] = sqrt((y_rmse[cam] / (float) cont[cam]));
  }

  fprintf (fp, "%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f\n",
      x_mae[0], y_mae[0], x_mae[1], y_mae[1], x_mae[2], y_mae[2],
      x_mae[3], y_mae[3], x_mae[4], y_mae[4],
      x_rmse[0], y_rmse[0], x_rmse[1], y_rmse[1], x_rmse[2], y_rmse[2],
      x_rmse[3], y_rmse[3], x_rmse[4], y_rmse[4]);

  fclose (fp);
return 0;
}

/**
*
* save_error_values:
*
* stores in the file (%s-error-stats.csv) the MEA and the RMSE for each frame
*
*/
int
save_error_values (const char* name, track_t list_of_tracks [MAX_CAM - MIN_CAM][MAX_FRAMES])
{
  char filename[255];
  FILE *fp;
  float x_mae = 0.0;
  float y_mae = 0.0;
  float x_rmse = 0.0;
  float y_rmse = 0.0;
  int cont = 0;
  int frame = 0;

  sprintf (filename, "%s-error-stats.csv", name);
  fp = fopen (filename, "w");
  fprintf (fp, "mae-x;mae-y;rmse-x;rmse-y;\n");
  for (frame = 0; frame < MAX_FRAMES; frame++) {
    if (track_fused_x (0, frame) > 0 && track_fused_x (0, frame) < 30000) {
      cont++;
      x_mae += fabs  (track_fused_x_errors (0, frame) - track_fused_x (0, frame));
      y_mae += fabs  (track_fused_y_errors (0, frame) - track_fused_y (0, frame));
      x_rmse += pow ((track_fused_x_errors (0, frame) - track_fused_x (0, frame)), 2);
      y_rmse += pow ((track_fused_y_errors (0, frame) - track_fused_y (0, frame)), 2);
    }
  }

  x_mae = x_mae / (float) cont;
  y_mae = y_mae / (float) cont;
  x_rmse = sqrt ((x_rmse / (float) cont));
  y_rmse = sqrt ((y_rmse / (float) cont));

  fprintf (fp, "%.2f;%.2f;%.2f;%.2f;\n", x_mae, y_mae, x_rmse, y_rmse);
  fclose (fp);

return 0;
}


/**
 * print_fused_values:
 *
 */
int
print_fused_values (const char* name, track_t list_of_tracks [MAX_CAM - MIN_CAM][MAX_FRAMES])
{
        int i;
        char filename[255];
        FILE *fp;

        sprintf (filename, "%s.csv", name);
        fp = fopen (filename, "w");
        for (i = 0; i < MAX_FRAMES; i++) {
                fprintf (fp, "%d;%.2f;%.2f\n",
                        track_frame (0, i), track_fused_x (0, i), track_fused_y (0, i));
        }
        fclose (fp);
return 0;
}

/**
 * save_track_values:
 *
 * for each frame and each camera stores the processed data
 *
 */
int
save_track_values (const char* name, track_t list_of_tracks [MAX_CAM - MIN_CAM][MAX_FRAMES])
{
  int i,j;
  char filename[255];
  FILE *fp;

#ifdef DEBUG
  printf ("print_track_values\n");
#endif

#ifdef JINT
  for (i = 0; i < (MAX_CAM - MIN_CAM); i ++) {
    sprintf (filename, "%s%d.csv", name, i);
    fp = fopen (filename, "w");
    fprintf (fp, "frame;pos-global-x;pos-global-y;track_fused_x_errors;track_fused_y_errors;track_fused_x;track_fused_y\n");
      for (j = 0; j < MAX_FRAMES; j ++)
        fprintf (fp, "%d;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f\n",
            track_frame (i, j), track_px_global (i, j), track_py_global (i, j), track_fused_x_errors (i,j),
            track_fused_y_errors (i, j), track_fused_x(i,j), track_fused_y(i,j));
        fclose(fp);
  }
  return 0;
#endif

  for (i = 0; i < (MAX_CAM - MIN_CAM); i++) {
    sprintf (filename, "%s%d.csv", name, i);
    fp = fopen (filename, "w");
    fprintf (fp,
        "frame;GT-Local-X;GT-Local-Y;GT-Local-W;GT-Local-H;Kalman-VarX;Kalman-VarY;Local-PX;Local-PY;Global-PX;Global-PY;Fused-X;Fused-Y;E-Track-X;E-Track-Y;E-medio-X;E-medio-Y;Desv-X;Desv-Y;mean5x,var5x;mean5y;var5y\n");
    for (j = 0; j < MAX_FRAMES; j++) {
      fprintf (fp, "%d;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f\n",
          track_frame(i,j), track_x(i,j), track_y(i,j), track_w(i,j), track_h(i,j), track_kalman_varx(i,j), track_kalman_vary(i,j),
          track_px_local(i,j), track_py_local(i,j), track_px_global (i,j), track_py_global (i,j),
          track_fused_x(i,j), track_fused_y(i,j), track_error_x(i,j), track_error_y(i,j), track_error_medio_x(i,j),
          track_error_medio_y(i,j), track_desv_x(i,j), track_desv_y(i,j), px_global_mean_5(i,j),
          px_global_var_5(i,j), py_global_mean_5(i,j), py_global_var_5(i,j)  );

     }
     fclose (fp);
   }
return 0;
}
