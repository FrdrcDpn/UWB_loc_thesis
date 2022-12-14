#ifndef ANIMATION_THREAD_H
#define ANIMATION_THREAD_H

#include <GL/freeglut.h>

#include "main.h"
#include "draw.h"
#include "user_interaction.h"
#include "terminalinfo.h"
#include "trigonometry.h"

bool animation_running = false;
/**
 * Main animation loop.
 * Takes care of drawing the agents in their corrective location.
 */
void main_loop_function()
{
  if (!animation_running) {
    terminalinfo::info_msg("Animation started.");
    animation_running = true;
  }

  // Add depth (used internally to block obstructed objects)
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glLoadIdentity();

  // Get current window size w.r.t. beginning
  xrat = (float)param->window_width() / (float)glutGet(GLUT_WINDOW_WIDTH);
  yrat = (float)param->window_height() / (float)glutGet(GLUT_WINDOW_HEIGHT);

  zoom_scale = -(float)10 / (-(float)10 + (float)zoom);
  glTranslatef(center_x, center_y, -10 + zoom);

  // Draw fixed one time objects
  static draw drawer; // Drawer object
  drawer.data(); // Put data in corner
  drawer.axes(); // Put x and y global axes
  drawer.axis_label(); // Axis label
  environment.animate(); // Animate the environment walls

  // Draw all robot's
  uint r = s.size();
  if (r > 0) {
    for (uint16_t ID = 0; ID < r; ID++) {
      // Input: ID, p_x global, p_y global, orientation global
      drawer.agent_trajectory(ID, s[ID]->state.at(0), s[ID]->state.at(1), s[ID]->orientation);
      drawer.agent_estimate(ID, s[ID]->state_estimate.at(0), s[ID]->state_estimate.at(1), s[ID]->orientation);
      drawer.covellipse(s[ID]->state.at(1), s[ID]->state.at(0), sqrtf(s[ID]->Cov[2]),sqrtf(s[ID]->Cov[0]));
     // drawer.agent_estimate(ID, s[ID]->ekf_estimate.at(0), s[ID]->ekf_estimate.at(1), s[ID]->orientation);
      // Input: ID, p_x global, p_y global, v_x global, v_y global
     // drawer.velocity_arrow(ID,  s[ID]->state_desired_traj.at(0), s[ID]->state_desired_traj.at(1), s[ID]->state_desired_traj.at(2), s[ID]->state_desired_traj.at(3));
      //drawer.velocity_arrow(ID,  s[ID]->state_estimate.at(0), s[ID]->state_estimate.at(1), s[ID]->state_estimate.at(2), s[ID]->state_estimate.at(3));
    }
  }

  if(param->enable_UWB() == 1){
   // Draw all beacons
  uint l = b.size();
  if (l > 0) {
    for (uint16_t ID_b = 0; ID_b < l; ID_b++) {
      // If our beacon is enabled, draw its position
      if(b[ID_b]->state_b[2]==1){
      drawer.uwb_beacon(b[ID_b]->state_b.at(0), b[ID_b]->state_b.at(1));
      }
    }
  }
  }
  // Swap buffers (color buffers, makes previous render visible)
  glutSwapBuffers();

  user_interaction(); // Activate interactive functions (mouse + keyboard), important: use this before draw functions!
  if (!program_running) {std::terminate();}
}

/**
 * Initialze OpenGL perspective matrix
 *
 * @param width Width of the animation window
 * @param height Height of the animation window
 */
void GL_Setup(int width, int height)
{
  glViewport(0, 100, width, height);
  glMatrixMode(GL_PROJECTION);
  gluPerspective(45, (float)width / height, .1, 100);
  glMatrixMode(GL_MODELVIEW);
}

/**
 * Thread function that initiates the animation
 */
void main_animation_thread()
{
  // Initialize all variables declared in drawingparams.h
  center_x = 0;
  center_y = 0;
  sx = 0;
  sy = 0;
  zoom = param->zoom(); // From parameters file
  zoom_scale = 0;
  pointer_x = 0;
  pointer_y = 0;
  paused = false;
  xrat = 1.0;
  yrat = 1.0;

  // Set up simulation window
  int argc = 1;
  char *argv[1] = {(char *)"  "};
  glutInit(&argc, argv);
  glutInitWindowPosition(0, 0); // Initialize at top left corner of screen
  glutInitWindowSize(param->window_width(), param->window_height()); // Set dimensions
  glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE);
  glutCreateWindow("Swarmulator"); // Set window title to Swarmulator
  glutIdleFunc(main_loop_function); // Set main loop
  GL_Setup(param->window_width(), param->window_height()); // Set up window parameters
  glutMainLoop(); // Initiate main drawing loop
}

#endif /*ANIMATION_THREAD_H*/
