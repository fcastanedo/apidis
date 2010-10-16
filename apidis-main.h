/*
 * Header file
 *
 * Federico Castanedo <castanedofede@gmail.com>
 */

/* opencv headers */
#include <highgui.h>
#include <cv.h>

/* standard headers */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

/* libxml headers */
#include <libxml/xmlreader.h>
#include <libxml/parser.h>
#include <libxml/tree.h>

/* Tsai calibration headers */
#include "cal_main.h"

/* Max number of frames */
#define MAX_FRAMES 1500

/*
 * * These values indicates the number of cameras you want to employ.
 * i.e. if you want to use the 5 cameras leave it as 0 and 5, if you only want data from camera 3 (index 2): MIN_CAM = 2 and MAX_CAM = 3
 */
#define MIN_CAM 0
#define MAX_CAM 5

#define X_RESOLUTION 1200
#define Y_RESOLUTION 600
#define X_RESOLUTION_INPUT_IMAGE 800
#define Y_RESOLUTION_INPUT_IMAGE 600

/* size of the measurement and state vector matrixes of the kalman filter */
#define STATE_VECTOR 8
#define MEASUREMENT_VECTOR 4

/* for scaling the image sizes */
#define SCALE_X 18.613
#define SCALE_Y 19.986

/* comment or un-comment for obtain debug information in the console */
//#define DEBUG
//#define VERBOSE

/* flag used to show the images */
#define SHOW_IMAGE 0

/* Threshold for rule out data out of range */
#define X_MAX 30000
#define Y_MAX 15000

/* track difference value */
#define DISTANCE 1500

/* para hacer fusion o no */
//#define FUSION

/* images of each frame and sensor */
IplImage* cam [MAX_CAM - MIN_CAM][MAX_FRAMES];

/* each sensor kalman filter */
CvKalman* kalman [MAX_CAM - MIN_CAM];

/* 
 * used to detect frames without information,  when getting ground truth information
 * from the xml
 */
static int   cont_frames = -1;
static int   cont_finds  = -1;
static float cam_values[5] = {0.0, 0.0, 0.0, 0.0, 0.0};


/* main data structure which provides the track information */
typedef struct {
	int camid; 	/* camera id */
	int playerid; 	/* which player it refers to */
	int frame;	/* which frame */
	float x;	/* top-left coordinate on the image plane of the bounding box */
	float y;	/* y coordinate on the image plane */
	float w;	/* bounding box width in pixels (image plane) */
	float h;  	/* bounding box height size in pixels (image plane) */
	float x_kalman;	/* filtered top left coordinate on the image plane of the bounding box */
	float y_kalman;	/* filtered y coordinate */
	float w_kalman;	/* filtered bounding box width (in pixels)   */
	float h_kalman; /* filtered bounding box height (in pixels)  */
	float kalman_varx;
	float kalman_vary; 
	float px_local;        /* x local coordinate (image plane) projected on the ground plane */
	float py_local;        /* y local coordinate (image plane) projected on the ground plane */
	float px_local_kalman; /* filtered x local coordinate (image plane) on the ground plane */
	float py_local_kalman; /* filtered y local coordinate (image plane) on the ground plane */
	float px_global;       /* X global coordinate on the ground plane */
	float py_global;       /* Y global coordinate on the ground plane */
	float pz_global;       /* Z global coordinate on the ground plane */
	float px_global_errors;
	float py_global_errors;
	float px_global_kalman;
	float py_global_kalman;
	float px_global_mean5; 
	float px_global_var5;
	float py_global_mean5;
	float py_global_var5;
	float cont_stat;
	float fused_x_errors;
	float fused_y_errors;
	float fused_x;       /* fused x value */
	float fused_y;       /* fused y value */
	float error_x;       /* absolute error of x coordinate (ground truth) and fused value (0 if there is no data) */
	float error_y;       /* absolute error of y coordinate (ground truth) and fused value  */
	int cont_frames;     /* for counting the number of frames with information */
	float error_medio_x; /* absolute mean error of the global x coordinate */
	float error_medio_y; /* absolute mean error of the global y coordinate */
	float desv_x; /* standard deviation of the mean absolute error of x coordinate */
	float desv_y; /* standard deviation of the mean absolute error of y coordinate */
	
}track_t;

/* Macros definitions */
#define track_px_global_errors(i,j) list_of_tracks [i][j].px_global_errors
#define track_py_global_errors(i,j) list_of_tracks [i][j].py_global_errors
#define track_fused_x_errors(i,j)   list_of_tracks [i][j].fused_x_errors
#define track_fused_y_errors(i,j)   list_of_tracks [i][j].fused_y_errors

#define track_camid(i,j)    list_of_tracks[i][j].camid
#define track_playerid(i,j) list_of_tracks[i][j].playerid
#define track_frame(i,j)    list_of_tracks[i][j].frame

/* these macros provides access to the ground truth obtained data */
#define track_x(i,j) list_of_tracks[i][j].x  //X value of the ground truth
#define track_y(i,j) list_of_tracks[i][j].y  //Y value of the ground truth
#define track_w(i,j) list_of_tracks[i][j].w  //W value of the ground truth
#define track_h(i,j) list_of_tracks[i][j].h  //H value of the ground truth

/* these macros provides access to the kalman filter data */
#define track_x_kalman(i,j)    list_of_tracks[i][j].x_kalman  //X value after kalman
#define track_y_kalman(i,j)    list_of_tracks[i][j].y_kalman  //Y value after kalman
#define track_w_kalman(i,j)    list_of_tracks[i][j].w_kalman  //W value after kalman
#define track_h_kalman(i,j)    list_of_tracks[i][j].h_kalman  //H value after kalman
#define track_kalman_varx(i,j) list_of_tracks[i][j].kalman_varx //X variance from kalman filter
#define track_kalman_vary(i,j) list_of_tracks[i][j].kalman_vary //Y variance from kalman filter

#define track_px_local(i,j)    list_of_tracks[i][j].px_local //X coordinate on the ground plane (local-image plane)
#define track_py_local(i,j)    list_of_tracks[i][j].py_local //Y coordinate on the ground plane (local-image plane)

#define track_px_local_kalman(i,j)  list_of_tracks[i][j].px_local_kalman //X coordinate on the ground plane (local-image plane) filtered
#define track_py_local_kalman(i,j)  list_of_tracks[i][j].py_local_kalman //Y coordinate on the ground plane (local-image plane) filtered

#define track_px_global(i,j)  list_of_tracks[i][j].px_global //X global coordinate on the ground plane
#define track_py_global(i,j)  list_of_tracks[i][j].py_global //Y global coordinate on the ground plane
#define track_pz_global(i,j)  list_of_tracks[i][j].pz_global //Z global coordinate on the ground plane

#define track_px_global_kalman(i,j) list_of_tracks[i][j].px_global_kalman //X global coordinate after Kalman filtering
#define track_py_global_kalman(i,j) list_of_tracks[i][j].py_global_kalman //Y global coordinate after Kalman filtering

#define track_fused_x(i,j)  list_of_tracks[i][j].fused_x  //fused x coordinate (global)
#define track_fused_y(i,j)  list_of_tracks[i][j].fused_y  //fused y coordinate (global)

/* these macros provides access to statistics information */
#define track_error_x(i,j)       list_of_tracks[i][j].error_x
#define track_error_y(i,j)       list_of_tracks[i][j].error_y
#define track_cont_frames(i,j)   list_of_tracks[i][j].cont_frames
#define track_error_medio_x(i,j) list_of_tracks[i][j].error_medio_x
#define track_error_medio_y(i,j) list_of_tracks[i][j].error_medio_y
#define track_desv_x(i,j)	 list_of_tracks[i][j].desv_x 
#define track_desv_y(i,j) 	 list_of_tracks[i][j].desv_y
#define px_global_mean_5(i,j)    list_of_tracks[i][j].px_global_mean5
#define px_global_var_5(i,j)     list_of_tracks[i][j].px_global_var5
#define py_global_mean_5(i,j)    list_of_tracks[i][j].py_global_mean5
#define py_global_var_5(i,j)     list_of_tracks[i][j].py_global_var5
#define cont_stat(i,j) 		 list_of_tracks[i][j].cont_stat
	

/* Methods definitions*/
int
init_list_of_tracks (track_t list_of_tracks[MAX_CAM][MAX_FRAMES]);

int
get_ground_truth_values (const char* player_id,
                         const char* team_id,
                         track_t list_of_tracks[MAX_CAM][MAX_FRAMES]);

int
process_and_show_images (const char* name_output_video,
                         track_t list_of_tracks [MAX_CAM][MAX_FRAMES]);

int
obtain_global_coordinates (track_t list_of_tracks [MAX_CAM][MAX_FRAMES]);

int
run_kalman_on_all_sensors (track_t list_of_tracks [MAX_CAM][MAX_FRAMES]);

void
load_cp_cc_data (FILE* fp,
                 struct camera_parameters *cp,
                 struct calibration_constants *cc);

void
print_cp_cc_data (FILE* fp,
                  struct camera_parameters *cp,
                  struct calibration_constants *cc);

float
fused_px_global (const int frame,
                 track_t list_of_tracks [MAX_CAM][MAX_FRAMES]);

float
fused_py_global (const int frame,
                 track_t list_of_tracks [MAX_CAM][MAX_FRAMES]);

int
obtain_and_save_stat_values (const char* name,
                             track_t list_of_tracks [MAX_CAM][MAX_FRAMES]);

float
fused_px_global (const int frame,
                 track_t list_of_tracks [MAX_CAM][MAX_FRAMES]);

float
fused_px_global_errors (const int frame,
                        track_t list_of_tracks[MAX_CAM][MAX_FRAMES]);

float
fused_py_global (const int frame,
                 track_t list_of_tracks [MAX_CAM][MAX_FRAMES]);

float
fused_py_global_errors (const int frame,
                        track_t list_of_tracks [MAX_CAM][MAX_FRAMES]);
