/* 
 * This program shows the information of several cameras using the APIDIS dataset
 * and the ground-truth information.
 * The global result is plotted on the ground plane basketball court.
 *
 * Federico Castanedo. <castanedofede@gmail.com>
 *
 * 26-01-2009: Implementation of a test program which use the information of the APIDIS data set
 * to track basketball players.
 *
 */

#include "apidis-main.h"

int
main (int argc, char* argv[]){

    /*
     * Main variable of the program; which stores a track_t type structure for each camera (MAX_CAM) and for each
     * frame (MAX_FRAMES)
     */
    static track_t list_of_tracks [MAX_CAM][MAX_FRAMES];

    if (argc < 6) { /* hey!, we want 6 parameters */
      usage(argv[0]);
      return -1;
    }

    srand (time (NULL));

    printf ("\n\n Reading GroundTruth from: %s\n", argv[1]);
    printf ("Saving output video in: %s\n\n", argv[4]);
    printf ("Saving output file data in: %s\n\n", argv[5]);

    printf ("Initializing list of tracks....init_list_of_tracks();\n");
    init_list_of_tracks (list_of_tracks);

    printf ("Obtaining ground-truth data....get_ground_truth_values();\n");
    get_ground_truth_values (argv[2], argv[3], list_of_tracks);

    printf("Counting frames with information in each camera....cont_frames_with_info();\n");
    cont_frames_with_info (list_of_tracks);

    printf ("Running Kalman filter on the image plane of all visual sensors....run_kalman_on_all_sensors()\n");
    run_kalman_on_all_sensors (list_of_tracks);

    printf ("Apply the calibration values to obtain global coordinates....obtain_global_coordinates\n");
    obtain_global_coordinates (list_of_tracks);

    printf ("Show the images and save a video with the result....process_and_show_images()\n");
    process_and_show_images (argv[4], list_of_tracks);

    printf ("Saving data files...\n");
    save_track_values (argv[5], list_of_tracks);
    save_error_values (argv[5], list_of_tracks);

return 0;
}
