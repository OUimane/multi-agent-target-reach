/*
 * File:          supervisor_efficacite.c
 * Date: 25/05
 * Description: This supervisor places all robots randomly. The robots must be in the world and
 *              have a DEF. If all robots have found the target, the supervisor pause 
 *              the simulation. It also calculate the efficiency of the program : sum of the distance at_the beginning of
 *              simulation of each robot and the target / sum of distance traveled by each robot.
 * Author: Nolwenn
 * Modifications:
 */

#include <webots/robot.h>
#include <webots/supervisor.h>
#include <webots/emitter.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define TIME_STEP 32
#define NB_EPUCK 8
#define ARENA_SIZE 100
#define RANGE_DETECTION 0.25
#define RANDOM false

WbNodeRef robots[NB_EPUCK];
WbFieldRef trans_field[NB_EPUCK];
WbFieldRef rttn_field[NB_EPUCK];
WbFieldRef controller_field[NB_EPUCK];
WbFieldRef window_field[NB_EPUCK];
WbFieldRef name_field[NB_EPUCK];
WbFieldRef trans_field_target;
WbFieldRef synch_field[NB_EPUCK];

/*
 * random number between min and max
 */
double random_between(int min, int max)
{
  int delta;
  double res;

  delta = max - min + 1;
  res = rand() % delta;
  res = (res + min) / 100;

  return res;
}

/*
 * Initialize robots[] and trans_field
 * function based on static void reset(void); in advanced_particle_swarm_optimization_supervisor.c
 */
void set_robots(void)
{
  // Name of the robot we are linking.
  char rob[7] = "epuck0";
  char def[30] = "DEF ";
  int i = 0;
   
  for (; i < NB_EPUCK; i++)
  {
      WbNodeRef root_node = wb_supervisor_node_get_root();
      WbFieldRef root_children_field = wb_supervisor_node_get_field(root_node, "children");
      strcpy(def,"DEF ");
      strcat(def,rob);
      strcat(def," E-puckCameraRecognition{ }");
      wb_supervisor_field_import_mf_node_from_string(root_children_field, 4, def);

    
    // Indicate where robot location has to be stored.
    robots[i] = wb_supervisor_node_get_from_def(rob);
    if (robots[i] == NULL)
    {
      fprintf(stderr, "No DEF node found in the current world file\n");
      exit(1);
    }

    trans_field[i] = wb_supervisor_node_get_field(robots[i], "translation");
    rttn_field[i] = wb_supervisor_node_get_field(robots[i], "rotation");
    window_field[i]= wb_supervisor_node_get_field(robots[i], "window");
    controller_field[i] = wb_supervisor_node_get_field(robots[i], "controller");
    name_field[i]= wb_supervisor_node_get_field(robots[i], "name");
    synch_field[i]=wb_supervisor_node_get_field(robots[i], "synchronization");


    // Update robot name.
    rob[5]++;
  }

  return;
}

/*
 * Places each robot in the arena, randomly or not
 * function from advanced_particle_swarm_optimization_supervisor.c
 */
void random_position(void)
{
  double p[3] = {0, 0, 0};
  double r[4] = {0, 0, 1, 0};
  bool synch=false;
  char rob[7] = "epuck0";
  
  if(! RANDOM){
  double x=0.12;
  double y=0.12;  
   
  int n = 0;
  for (n=0; n < NB_EPUCK; n++)
  {   
    p[0] = x; // x coordinate
    p[1] = y; // y coordinate

    x = x + 0.2;
    if(x > 0.8){
      x = 0.12;
      y = y - 0.2;
    }

    r[3] = 0; 

    wb_supervisor_field_set_sf_vec3f(trans_field[n], p);
    wb_supervisor_field_set_sf_rotation(rttn_field[n], r);
    wb_supervisor_field_set_sf_string(window_field[n], "e-puck");
    wb_supervisor_field_set_sf_string(controller_field[n], "AutonomyThree");
    wb_supervisor_field_set_sf_string(name_field[n], rob);
    wb_supervisor_field_set_sf_bool(synch_field[n],synch);
    rob[5]++;        
  }
  }
  else{
  
  int n = 0;
  for (; n < NB_EPUCK; n++)
  {
    p[0] = random_between(-ARENA_SIZE, ARENA_SIZE); // x coordinate
    p[1] = random_between(-ARENA_SIZE, ARENA_SIZE); // y coordinate

    r[3] = random_between(-314, 314);

    wb_supervisor_field_set_sf_vec3f(trans_field[n], p);
    wb_supervisor_field_set_sf_rotation(rttn_field[n], r);
    wb_supervisor_field_set_sf_string(window_field[n], "e-puck");
    wb_supervisor_field_set_sf_string(controller_field[n], "AutonomyThree");
    wb_supervisor_field_set_sf_string(name_field[n], rob);
    rob[5]++;        
  }
}
  printf("positioning the target...\n");
  p[0] = random_between(-ARENA_SIZE * 2, ARENA_SIZE * 2); // x coordinate
  p[1] = random_between(-ARENA_SIZE, ARENA_SIZE);         // y coordonate
 
  p[2] = 0.055;                                   // z coordinate

  wb_supervisor_field_set_sf_vec3f(trans_field_target, p);

  printf("Positioning done.\n");
  return;
}

/*
 * Return a distance between 2 points of the plan
 */
double distance(double x1, double y1, double x2, double y2)
{
  return sqrt(((x1 - x2) * (x1 - x2)) + ((y1 - y2) * (y1 - y2)));
}


/*
 * Return true if a robot is in the range
 */
bool in_the_range(const double *values_robots, const double *values_cible)
{
 bool x_range = values_robots[0] < values_cible[0] + RANGE_DETECTION && values_robots[0] > values_cible[0] - RANGE_DETECTION;
 bool y_range = values_robots[1] < values_cible[1] + RANGE_DETECTION && values_robots[1] > values_cible[1] - RANGE_DETECTION;
  
 return x_range && y_range;
}

/*
 * Return true if all robots have found the target
 */
bool simulation_finished(bool robots_stopped[NB_EPUCK])
{
  bool res = true;

  for (int i = 0; i < NB_EPUCK; i++)
    res = res && robots_stopped[i];

  return res;
}


int main()
{
  wb_robot_init();
  srand(time(NULL));

  // Target
  WbNodeRef target = wb_supervisor_node_get_from_def("target");
  trans_field_target = wb_supervisor_node_get_field(target, "translation");
  if (target == NULL)
  {
    fprintf(stderr, "No DEF target node found in the current world file\n");
    exit(1);
  }

  // setup
  set_robots();
  random_position();

  const double *values_robots[NB_EPUCK];
  const double *values_cible = wb_supervisor_field_get_sf_vec3f(trans_field_target);
  double distance_min_target[NB_EPUCK];
  double distance_traveled[NB_EPUCK];
  double efficacite, sum_distance_traveled = 0.0, sum_distance_min = 0.0, last_x = 0.0, last_y = 0.0;
  int i;

  bool finished = false;

  /* Initialize all arrays and check the distance between a robot and the target*/
  bool robots_stopped[NB_EPUCK];
  for (i = 0; i < NB_EPUCK; i++)
  {
    robots_stopped[i] = false;
    distance_traveled[i] = 0.0;

    values_robots[i] = wb_supervisor_field_get_sf_vec3f(trans_field[i]);

    distance_min_target[i] = distance(values_robots[i][0], values_robots[i][1], values_cible[0], values_cible[1]);
    printf("distance minimale target/epuck%d : %g m\n", i, distance_min_target[i]);
  }

  while (wb_robot_step(TIME_STEP) != -1)
  {
    /*
     * Calculate the distance traveled by each robot each step
     */
    for (i = 0; i < NB_EPUCK; i++)
    {
      if (!robots_stopped[i])
      {
        last_x = values_robots[i][0];
        last_y = values_robots[i][1];
        values_robots[i] = wb_supervisor_field_get_sf_vec3f(trans_field[i]);

        distance_traveled[i] += distance(last_x, last_y, values_robots[i][0], values_robots[i][1]);

        if (in_the_range(values_robots[i], values_cible))
        {
          printf("epuck%d is in the range\n", i);

          robots_stopped[i] = true;
          finished = simulation_finished(robots_stopped);
        }
      }
    }

    /* all robots have found the target
     * it checks the distance traveled and calculate the efficiency
     * it pauses the simulation at the end
     */
     
     
    if (finished)
    {
      printf("--- All robots have found the target ---\n");
      for (i = 0; i < NB_EPUCK; i++)
      {
        printf("distance parcourue epuck%d : %g m\n", i, distance_traveled[i]);
        printf("Efficacité d'epuck%d : %g\n\n", i, (distance_min_target[i] - RANGE_DETECTION) / distance_traveled[i]);

        sum_distance_min += (distance_min_target[i] - RANGE_DETECTION);
        sum_distance_traveled += distance_traveled[i];
      }

      efficacite = sum_distance_min / sum_distance_traveled * 100;
      printf("Efficacité : %g \n", efficacite);

      wb_supervisor_simulation_set_mode(WB_SUPERVISOR_SIMULATION_MODE_PAUSE);
    }
    
  }

  wb_robot_cleanup();
  return 0;
}