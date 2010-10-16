/*
 * get_ground_truth_values.c
 *
 *  Created on: 19/01/2010
 *      Author: fede
 */

#include "apidis-main.h"

/**
* get_element_names:
*
* take the data related to the player and save it in the list of tracks
*
*/
static void
get_element_names (xmlNode* a_node, const char* player_id, const int camid, const char* team_id,
                  track_t list_of_tracks[MAX_CAM][MAX_FRAMES])
{
    xmlNode *cur_node = NULL;
    xmlNode *next_node = NULL;

    if ((cont_frames-1) > cont_finds)  /* there is one frame without information */
        cont_finds++;
    for (cur_node = a_node; cur_node; cur_node = cur_node->next) {
        if (strcmp (cur_node->name,"frame") == 0) {
            cont_frames++;
        }
        if (xmlGetProp (cur_node, "type") != NULL) {
                if (strcmp ((const char *)xmlGetProp (cur_node, "type"),"player") == 0 &&
                    strcmp ((const char *)xmlGetProp (cur_node, "player-id"), player_id) == 0 &&
                    strcmp ((const char *)xmlGetProp (cur_node, "player-team"), team_id) == 0) {
                        cont_finds++;
                        next_node = cur_node->children->next;
                        next_node = next_node->children->next;
                     #ifdef VERBOSE
                        printf ("%d;%s", cont_frames, xmlNodeGetContent (next_node));
                     #endif
                        /* save  track information */
                        list_of_tracks [camid][cont_frames].camid = camid;
                        list_of_tracks [camid][cont_frames].frame = cont_frames;
                        list_of_tracks [camid][cont_frames].playerid = atoi (player_id);
                        list_of_tracks [camid][cont_frames].x = atof (xmlNodeGetContent (next_node));
                        next_node = next_node->next;
                        next_node = next_node->next;
                     #ifdef VERBOSE
                        printf (";%s", xmlNodeGetContent (next_node));
                     #endif
                        list_of_tracks [camid][cont_frames].y = atof (xmlNodeGetContent (next_node));
                        next_node = next_node->next;
                        next_node = next_node->next;
                     #ifdef VERBOSE
                        printf (";%s", xmlNodeGetContent (next_node));
                     #endif
                        list_of_tracks [camid][cont_frames].w = atof (xmlNodeGetContent (next_node));
                        next_node = next_node->next;
                        next_node = next_node->next;
                     #ifdef VERBOSE
                        printf (";%s\n", xmlNodeGetContent (next_node));
                     #endif
                        list_of_tracks [camid][cont_frames].h = atof (xmlNodeGetContent (next_node));

                        /* save also the projection of the center of the bounding box on the ground plane (LOCAL COORDINATES) */
                        list_of_tracks [camid][cont_frames].px_local =
                                (float) list_of_tracks [camid][cont_frames].x + (list_of_tracks [camid][cont_frames].w/2.0);
                        list_of_tracks [camid][cont_frames].py_local =
                                (float) list_of_tracks [camid][cont_frames].y + (list_of_tracks [camid][cont_frames].h);
                      #ifdef VERBOSE
                        printf ("Saving at camid:%d - %d - %d,%d \n",
                                camid, cont_frames, (int) list_of_tracks [camid][cont_frames].px_local,
                                (int) list_of_tracks [camid][cont_frames].py_local );
                     #endif
                }
        }
        get_element_names(cur_node->children, player_id, camid, team_id, list_of_tracks);
    }
}

/**
 * get_ground_truth_values
 *
 * take the player data for all the cameras
 *
 */
int
get_ground_truth_values (const char* player_id, const char* team_id,
                         track_t list_of_tracks[MAX_CAM][MAX_FRAMES])
{
#ifdef DEBUG
  printf ("get_ground_truth_values()\n");
#endif

  xmlDoc *doc = NULL;
  xmlNode *root_element = NULL;

  char name_xml[255];
  int i = 0;

  LIBXML_TEST_VERSION

  /* for all the cameras */
  for (i = 0; i < (MAX_CAM - MIN_CAM); i++) {
    sprintf (name_xml, "camera%1d-ps_20080409T164700Z.objects.xml", i+1);
  #ifdef DEBUG
    printf ("Taking ground-truth from %s\n", name_xml);
  #endif

    /* parsing the file and obtaining the DOM tree */
    doc = xmlReadFile (name_xml, NULL, 0);
    if (doc == NULL) {
      printf("We couldnt parse the file %s\n", name_xml);
      continue;
    }

    /* taking the root element */
    root_element = xmlDocGetRootElement (doc);
    cont_finds = -1;
    cont_frames = -1;

    /* taking the player data */
    get_element_names (root_element, player_id, i, team_id, list_of_tracks);

    /* free the document*/
    xmlFreeDoc (doc);
    xmlCleanupParser ();
  }
  return 0;
}
