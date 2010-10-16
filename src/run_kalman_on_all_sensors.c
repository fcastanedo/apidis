/*
 * run_kalman_on_all_sensors.c
 *
 *  Created on: 19/01/2010
 *      Author: fcastanedo
 *
 *  Run a standard kalman filter using data of the N cameras on the image plane
 */

#include "apidis-main.h"


int
run_kalman_on_all_sensors (track_t list_of_tracks [MAX_CAM - MIN_CAM][MAX_FRAMES])
{
    float Z1 [MAX_CAM][MEASUREMENT_VECTOR];
    CvRandState rng [MAX_CAM];
    int i, current_frame;
    CvMat x_k [MAX_CAM - MIN_CAM];
    CvMat* w_k[MAX_CAM - MIN_CAM]; /* process noise */
    CvMat* z_k[5]; /* measurements */

    srand (time(0));

    for (i = 0; i < MAX_CAM - MIN_CAM; i++)
      cvRandInit (&rng[i], 0, 1, -1, CV_RAND_UNI);

    /* Create the filter for the N cameras */
    /* with 8 state variables (x,y,w,h,dx,dy,dw,dh) and 4 measurement variables (x,y,w,h) */
    for (i = 0; i < (MAX_CAM - MIN_CAM); i++)
        kalman[i] = cvCreateKalman (STATE_VECTOR, MEASUREMENT_VECTOR, 0);

    /* the state (x_k) is x,y,w,h */
    /* TODO init with the data of the first frame? */
    for (i = 0; i < MAX_CAM - MIN_CAM; i++)
      x_k[i] = cvMat (4, 1, CV_32F, Z1[i]);

    for (i = 0; i < (MAX_CAM - MIN_CAM); i++) {
      cvZero (kalman[i]->state_post); /* save the result of cvKalmanCorrect() */
      cvZero (kalman[i]->state_pre);  /* save the result of cvKalmanPredict() */
    }

    /* process noise */
    for (i = 0; i < MAX_CAM - MIN_CAM; i++)
      w_k[i] = cvCreateMat (4, 1, CV_32FC1);

    /* measurements (x,y,w,h) */
    for (i = 0; i < MAX_CAM - MIN_CAM; i++)
      z_k[i] = cvCreateMat (4, 1, CV_32FC1);

    /* transition matrix F. Describe the relation between the model parameters at instant k and k+1 */
    const float F[] = {1, 0, 0, 0, 1, 0, 0, 0,
                       0, 1, 0, 0, 0, 1, 0, 0,
                       0, 0, 1, 0, 0, 0, 1, 0,
                       0, 0, 0, 1, 0, 0, 0, 1,
                       0, 0, 0, 0, 1, 0, 0, 0,
                       0, 0, 0, 0, 0, 1, 0, 0,
                       0, 0, 0, 0, 0, 0, 1, 0,
                       0, 0, 0, 0, 0, 0, 0, 1};

    for (i = 0; i < (MAX_CAM - MIN_CAM); i++)
       memcpy (kalman[i]->transition_matrix->data.fl, F, sizeof (F));

    /* measurement matrix */
    const float H[] = {1, 0, 0, 0, 0, 0, 0, 0,
                       0, 1, 0, 0, 0, 0, 0, 0,
                       0, 0, 1, 0, 0, 0, 0, 0,
                       0, 0, 0, 1, 0, 0, 0, 0};

    for (i = 0; i < (MAX_CAM - MIN_CAM); i++)
        memcpy (kalman[i]->measurement_matrix->data.fl, H, sizeof (H));

    /* initialize other kalman filter parameters */
    for (i = 0; i < (MAX_CAM - MIN_CAM); i++) {
      cvSetIdentity (kalman[i]->process_noise_cov, cvRealScalar(1e-5));
      cvSetIdentity (kalman[i]->measurement_noise_cov, cvRealScalar(1e-1));
      cvSetIdentity (kalman[i]->error_cov_post, cvRealScalar (1));
    }

    /* for each frame */
    for (current_frame = 0; current_frame < MAX_FRAMES; current_frame++) {
      if (current_frame < 2) { /* first call */
        for (i = 0; i < (MAX_CAM - MIN_CAM); i++) {
          if (track_x (i, current_frame) > 0) { /* check if this camera is giving information */
            kalman[i]->state_post->data.fl[0+4] =
                track_x (i, current_frame) - kalman[i]->state_post->data.fl[0];
            kalman[i]->state_post->data.fl[1+4] =
                track_y (i, current_frame) - kalman[i]->state_post->data.fl[1];
            kalman[i]->state_post->data.fl[2+4] =
                track_w (i, current_frame) - kalman[i]->state_post->data.fl[2];
            kalman[i]->state_post->data.fl[3+4] =
                track_h (i, current_frame) - kalman[i]->state_post->data.fl[3];
            kalman[i]->state_post->data.fl[0] = track_x (i, current_frame);
            kalman[i]->state_post->data.fl[1] = track_y (i, current_frame);
            kalman[i]->state_post->data.fl[2] = track_w (i, current_frame);
            kalman[i]->state_post->data.fl[3] = track_h (i, current_frame);
          }
        }
       }
       else {
         /* predict */
         for (i = 0; i < (MAX_CAM - MIN_CAM); i++) {
           if (track_x (i, current_frame) > 0) { /* check if this camera is giving information */
             cvKalmanPredict (kalman[i], 0);

             /* assign the new observations */
             Z1[i][0] = track_x (i, current_frame);
             Z1[i][1] = track_y (i, current_frame);
             Z1[i][2] = track_w (i, current_frame);
             Z1[i][3] = track_h (i, current_frame);

             track_kalman_varx (i, current_frame) =
                 CV_MAT_ELEM (*kalman[i]->error_cov_post, float, 0, 0);
             track_kalman_vary (i, current_frame) =
                 CV_MAT_ELEM (*kalman[i]->error_cov_post, float, 1, 1);

             /* correct the state */
             cvKalmanCorrect (kalman[i], &x_k[i]);

             /* added process noise */
             cvRandSetRange (&rng[i], 0, sqrt (kalman[i]->process_noise_cov->data.fl[0]), 0);
             cvRand (&rng[i], w_k[i]);

             cvMatMulAdd (kalman[i]->measurement_matrix, kalman[i]->state_post, NULL, &x_k[i]);

             /* set the global values of the output kalman filter */
             track_x_kalman (i, current_frame) = Z1[i][0];
             track_y_kalman (i, current_frame) = Z1[i][1];
             track_w_kalman (i, current_frame) = Z1[i][2];
             track_h_kalman (i, current_frame) = Z1[i][3];

             /* take the projection of the centroid on the ground plane */
             track_px_local_kalman (i, current_frame) = Z1[i][0] + (Z1[i][2]/2.0);
             track_py_local_kalman (i, current_frame) = Z1[i][1] + Z1[i][3];
#ifdef DEBUG
             printf ("Frame: %d, Cam: %d, track_px_local: %f, track_px_local_kalman: %f\n",
                 current_frame, i, track_px_local (i, current_frame), track_px_local_kalman (i, current_frame));
             printf ("Frame: %d, Cam: %d, track_py_local: %f, track_py_local_kalman: %f\n",
                 current_frame, i, track_py_local (i, current_frame), track_py_local_kalman (i, current_frame));
#endif
           }
         }
       }
    }
return 0;
}
