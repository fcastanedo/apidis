/*
 * aux_methods.c
 *
 *  Created on: 26/05/2010
 *      Author: fcastanedo
 */

#include "apidis-main.h"

/**
 * cont_frames_with_info
 *
 * count the frames with information for a specific player
 */
int
cont_frames_with_info (track_t list_of_tracks [MAX_CAM][MAX_FRAMES])
{
  int i,j = 0;

  for (i = 0; i < (MAX_CAM - MIN_CAM); i++)
    for (j = 0; j < MAX_FRAMES; j++)
      if (track_px_local (i, j) > 0)
        cam_values[i]++;

return 0;
}


/**
 * usage()
 */
void
usage (const char* name) {
        printf ("Uso: %s location_data player_number team_name video_output.mjpg text_file.csv \n", name);
}
