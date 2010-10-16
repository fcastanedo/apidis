/*
 * process_and_show_images.c
 *
 */

#include "apidis-main.h"

/**
 *
 * obtain_track_errors
 *
 * get the absolute error between the fused and filtered data
 *
 */
int
obtain_track_errors (const int frame, track_t list_of_tracks [MAX_CAM - MIN_CAM][MAX_FRAMES])
{
  int i;

  for (i = 0; i < (MAX_CAM - MIN_CAM); i++) {
    if (track_py_local (i, frame) > 0) {
      track_error_x (i, frame) = abs (track_fused_x (i, frame) - track_px_global_kalman (i, frame));
      track_error_y (i, frame) = abs (track_fused_y (i, frame) - track_py_global_kalman (i, frame));
    }
  }
return 0;
}

/**
 *
 * obtain_mean_errors
 *
 * get the mean errors of each camera
 *
 */
int
obtain_mean_errors (track_t list_of_tracks [MAX_CAM - MIN_CAM][MAX_FRAMES])
{
  int i,j;
  float acum_x[5] = {0.0,0.0,0.0,0.0,0.0};
  float acum_y[5] = {0.0,0.0,0.0,0.0,0.0};

  for (j = 0; j < MAX_FRAMES; j++) {
    for (i = 0; i < (MAX_CAM - MIN_CAM); i++) {
      acum_x[i] += track_error_x (i, j);
      acum_y[i] += track_error_y (i, j);
    }
  }
  for (i = 0; i < (MAX_CAM - MIN_CAM); i ++) {
    track_error_medio_x (i,MAX_FRAMES-1) = (acum_x[i]) / (float) cam_values [i];
    track_error_medio_y (i,MAX_FRAMES-1) = (acum_y[i]) / (float) cam_values [i];
  }
return 0;
}

/**
 * process_and_show_images
 *
 * show the different images and save a video file with the results
 *
 */
int
process_and_show_images (const char* name_output_video,
                         track_t list_of_tracks [MAX_CAM - MIN_CAM][MAX_FRAMES])
{

  IplImage *ground_plane_image = NULL, *cam[MAX_CAM - MIN_CAM];
  char cam_name [5][255];
  IplImage *DispImage;
  int current_frame, i, j ,k;
  float x_fused = 0.0, y_fused = 0.0, x_fused_errors = 0.0, y_fused_errors = 0.0;
  CvVideoWriter *writer = NULL;
  CvSize s = cvSize (X_RESOLUTION, Y_RESOLUTION);
  int   contcam[MAX_CAM - MIN_CAM] = {0, 0, 0, 0, 0};
  float acumcam_x[MAX_CAM - MIN_CAM] = {0.0, 0.0, 0.0, 0.0, 0.0};
  float acumcam_y[MAX_CAM - MIN_CAM] = {0.0, 0.0, 0.0, 0.0, 0.0};

#ifdef DEBUG
    printf ("process_and_show_images() \n");
#endif

#ifdef SHOW_IMAGE
    printf ("Creating output video file: %s\n", name_output_video);
    writer = cvCreateVideoWriter (name_output_video, CV_FOURCC ('M', 'J', 'P', 'G'), 15, s, 1);
    //writer = cvCreateVideoWriter (name_output_video, -1, 15, s, 1);
    cvNamedWindow ("Apidis", CV_WINDOW_AUTOSIZE);
    DispImage = cvCreateImage (cvSize (X_RESOLUTION, Y_RESOLUTION), IPL_DEPTH_8U, 3);
#endif

    /* for each frame */
    for (current_frame = 0; current_frame < MAX_FRAMES; current_frame++) {
      printf (".");
      fflush (stdout);
#ifdef SHOW_IMAGE
      sprintf (cam_name[0], "%s-%04d.jpeg", LOCATION_CAM1_IMAGES, current_frame);
      sprintf (cam_name[1], "%s-%04d.jpeg", LOCATION_CAM2_IMAGES, current_frame);
      sprintf (cam_name[2], "%s-%04d.jpeg", LOCATION_CAM4_IMAGES, current_frame);
      /* Note that cam3 and cam4 are mapped to camera 6 and camera 7 images */
      sprintf (cam_name[3], "%s-%04d.jpeg", LOCATION_CAM6_IMAGES, current_frame);
      sprintf (cam_name[4], "%s-%04d.jpeg", LOCATION_CAM7_IMAGES, current_frame);

      for (i = 0; i < MAX_CAM; i++) {
        cam[i] = cvLoadImage (cam_name[i], CV_LOAD_IMAGE_COLOR);
          if (cam[i] == NULL) {
            printf ("Error opening image: %s \n", cam_name[i]);
            exit (-1);
          }
        }

      if (ground_plane_image == NULL)
        ground_plane_image = cvLoadImage (LOCATION_GROUND_PLANE_IMAGE, CV_LOAD_IMAGE_COLOR);

      cvSetImageROI (DispImage, cvRect (0, 0, 400, 300));
#endif
        for (i = 0; i < MAX_CAM; i++) {
          if (track_x (i, current_frame) != 0) { /* if camera 1 in current frame have information */
#ifdef SHOW_IMAGE
            cvRectangle (cam[i], cvPoint ((int) track_x (i, current_frame), (int) track_y (i, current_frame)),
            cvPoint ((int) track_x (i, current_frame) + track_w (i, current_frame),
                     (int) track_y (i, current_frame) + track_h (i, current_frame)), CV_RGB (255,255,0), 2, 8, 0);

            cvRectangle (cam[i], cvPoint ((int) track_x_kalman (i, current_frame), (int) track_y_kalman (i, current_frame)),
            cvPoint ((int) track_x_kalman (i, current_frame) + track_w_kalman (i, current_frame),
                     (int) track_y_kalman (i, current_frame) + track_h_kalman (i, current_frame)), CV_RGB (255,0,0), 2, 8, 0);
#endif
            if (contcam[i] < 5) /* wait for having 5 values */
              contcam[i]++;

            if (contcam[i] == 5) {/* save stats */
              /* get the current mean */
              acumcam_x[i] = 0.0;
              acumcam_y[i] = 0.0;
              for (j = 0; j< 5; j++) {
                acumcam_x[i] += track_px_global_kalman (i, current_frame - j);
                acumcam_y[i] += track_py_global_kalman (i, current_frame - j);
              }
              px_global_mean_5 (i, current_frame) = acumcam_x[i] / (float) contcam[i];
              py_global_mean_5 (i, current_frame) = acumcam_y[i] / (float) contcam[i];

              for (j = 0; j < 5; j++) {
                px_global_var_5 (i, current_frame) +=
                    pow(track_px_global (i, current_frame-j) - px_global_mean_5 (i, current_frame),2);
                py_global_var_5 (i, current_frame) +=
                    pow(track_py_global (i, current_frame-j) - py_global_mean_5 (i, current_frame),2);

              }
              px_global_var_5 (i, current_frame) = px_global_var_5 (i, current_frame) / 5.0;
              py_global_var_5 (i, current_frame) = py_global_var_5 (i, current_frame) / 5.0;
            }
        }
        /* if we lost the objective initialize */
        if (track_x (i, current_frame) == 0)
          contcam[i] = 0;

#ifdef SHOW_IMAGE
        cvResize (cam[i], DispImage, CV_INTER_AREA);
        cvResetImageROI (DispImage);
        if (i == 0)
          cvSetImageROI (DispImage, cvRect (400, 0, 400, 300));
        if (i == 1)
          cvSetImageROI (DispImage, cvRect (800, 0, 400, 300));
        if (i == 2)
          cvSetImageROI (DispImage, cvRect (0, 300, 400, 300));
        if (i == 3)
          cvSetImageROI (DispImage, cvRect (400, 300, 400, 300));
        if (i == 4)
          cvSetImageROI (DispImage, cvRect (800, 300, 400, 300));
#endif
        } /* for each camera */

        x_fused = fused_px_global (current_frame, list_of_tracks);
        y_fused = fused_py_global (current_frame, list_of_tracks);

        x_fused_errors = fused_px_global_errors (current_frame, list_of_tracks);
        y_fused_errors = fused_py_global_errors (current_frame, list_of_tracks);

        /* for each camera, save the fused data in the data structure list_of_tracks */
        for (k = 0; k < (MAX_CAM - MIN_CAM); k++) {
           track_fused_x (k, current_frame) = x_fused;
           track_fused_y (k, current_frame) = y_fused;
         }

        /* save the fused error (difference between filtered data and fused data) */
        for (k = 0; k < (MAX_CAM - MIN_CAM); k++) {
          if (track_py_local (k, current_frame) > 0) {
               track_error_x (k, current_frame) = abs (track_fused_x (k, current_frame) - track_px_global_kalman (k, current_frame));
               track_error_y (k, current_frame) = abs (track_fused_y (k, current_frame) - track_py_global_kalman (k, current_frame));
           }
        }

        /* get the absolute errors between local data and fused data */
        obtain_track_errors (current_frame, list_of_tracks);

#ifdef SHOW_IMAGE
        /* drawing the player positions on the ground plane */

        cvCircle (ground_plane_image, cvPoint (((int)(track_px_global (1, current_frame)/SCALE_X)),
                ((int) (track_py_global(1, current_frame)/SCALE_Y))) , 4, CV_RGB(255, 0, 0), CV_FILLED, CV_AA, 0);

        printf ("Frame:%d, establishing player @: %f,%f --%f,%f --%f,%f\n", current_frame, track_fused_x(0, current_frame),
                 track_fused_y(0, current_frame), x_fused, y_fused, track_px_global(1, current_frame), track_py_global (1, current_frame));

        cvResize (ground_plane_image, DispImage, CV_INTER_AREA);

        cvResetImageROI (DispImage);
        cvShowImage ("Apidis", DispImage);
        cvWriteFrame (writer, DispImage);

        /* free the memory */
        for (i = 0; i < MAX_CAM; i++)
          cvReleaseImage (&cam[i]);

        cvWaitKey(2);
#endif

    } /* for each frame */

    /* get the mean errors of each camera */
    obtain_mean_errors (list_of_tracks);

#ifdef SHOW_IMAGE
    cvReleaseImage (&ground_plane_image);
    cvReleaseVideoWriter (&writer);
    cvDestroyWindow ("Apidis");
#endif

return 0;
}
