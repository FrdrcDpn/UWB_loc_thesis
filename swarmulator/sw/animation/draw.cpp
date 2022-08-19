#include "draw.h"
#include "trigonometry.h"
#include <cmath>
#include "fitness_functions.h"

void draw::uwb_beacon_range(const uint16_t &ID)
{
    glRasterPos2f(-0.01, 0.035);
    glColor3f(1.0, 1.0, 1.0); // Background color
    std::stringstream ss;
    ss << (int)ID;
    glutBitmapString(GLUT_BITMAP_8_BY_13, (unsigned char *)ss.str().c_str());
}

void draw::uwb_beacon(const float &xb, const float &yb)
{
    glPushMatrix();
    glTranslatef(yb * xrat, xb * yrat, 0.0);
    glRotatef(90, 0.0, 0.0, 1.0);

    glBegin(GL_POLYGON);
    float scl = 0.125;
    glColor3ub(255, 0, 255); // fushia :P
    glVertex2f(-1 * scl,  1 * scl);
    glVertex2f(-1 * scl, -1 * scl);
    glVertex2f(1.0 * scl,  0 * scl);
    glEnd();

    glColor3ub(255, 255, 255); // White
    glPopMatrix();
    
    glPopMatrix();
}

void draw::data()
{
  if(param->enable_UWB() == 1){
  glColor3ub(255, 0, 255);
  glRasterPos2f((-3.9 / zoom_scale - center_x), (-3.3 / zoom_scale - center_y));
  std::stringstream kk;
  kk << "UWB Beacons";
  glutBitmapString(GLUT_BITMAP_8_BY_13, (unsigned char *)kk.str().c_str());
  }
  glColor3ub(0, 191, 255);
  glRasterPos2f((-3.9 / zoom_scale - center_x), (-3.5 / zoom_scale - center_y));
  std::stringstream ss;
  ss << "Realtime trajectory position";
  glutBitmapString(GLUT_BITMAP_8_BY_13, (unsigned char *)ss.str().c_str());
  
  glColor3ub(124, 252, 000);
  glRasterPos2f((-3.9 / zoom_scale - center_x), (-3.7 / zoom_scale - center_y));
  std::stringstream dd;
  dd << "Realtime EKF position estimate";
  glutBitmapString(GLUT_BITMAP_8_BY_13, (unsigned char *)dd.str().c_str());
  
  glColor3ub(255, 255, 255); // White
  glRasterPos2f((-3.9 / zoom_scale - center_x), (-3.9 / zoom_scale - center_y));
  std::stringstream pp;
  pp << "Time[s]:" << simtime_seconds << " \t" << "Fitness: " << evaluate_fitness();
  glutBitmapString(GLUT_BITMAP_8_BY_13, (unsigned char *)pp.str().c_str());
  

}

void draw::axis_label()
{
  glRasterPos2f(3.9 / zoom_scale - center_x, 0.1 / zoom_scale);
  glutBitmapString(GLUT_BITMAP_8_BY_13, (unsigned char *)std::string("E").c_str());
  glRasterPos2f(0.1 / zoom_scale, 3.9 / zoom_scale - center_y);
  glutBitmapString(GLUT_BITMAP_8_BY_13, (unsigned char *)std::string("N").c_str());
}

void draw::agent_number(const uint16_t &ID)
{
  glRasterPos2f(-0.01, 0.035);
  glColor3f(1.0, 1.0, 1.0); // Background color

  std::stringstream ss;
  ss << (int)ID;
  glutBitmapString(GLUT_BITMAP_8_BY_13, (unsigned char *)ss.str().c_str());
}

void draw::triangle(const float &scl)
{
  glPushMatrix();

  glBegin(GL_POLYGON);
  glColor3ub(200, 000, 000); // Red
  glVertex2f(-1 * scl,  1 * scl);
  glVertex2f(-1 * scl, -1 * scl);
  glVertex2f(2.0 * scl,  0 * scl);
  glEnd();

  glColor3ub(255, 255, 255); // White
  glPopMatrix();
}

void draw::circle(const float &d)
{
  float angle, x, y;
  glPushMatrix();
  glBegin(GL_POLYGON);
  glColor3ub(200, 000, 000); // Redish
  for (int i = 0; i <= 10; i++) { // Resolution
    angle = 2 * M_PI * i / 10;
    x = (d * yrat) * cos(angle);
    y = (d * xrat) * sin(angle);
    glVertex2d(x, y);
  }
  glEnd();

  glColor3ub(255, 255, 255); // White
  glPopMatrix();
}

void draw::circle_loop(const float &r)
{
  int num_segments = 30; // Resolution
  glPushMatrix();
  glLineWidth(1);
  glBegin(GL_LINE_LOOP);
  for (int i = 0; i < num_segments; i++) {
    float theta = 2.0f * M_PI * float(i) / float(num_segments);//get the current angle
    float x = r * yrat * cosf(theta);                 //calculate the x component
    float y = r * xrat * sinf(theta);                 //calculate the y component
    glVertex2d(x, y);
  }
  glEnd();
  glColor3ub(255, 255, 255); // White
  glPopMatrix();
}

void draw::line(const float &x, const float &y)
{
  glPushMatrix();
  glLineWidth(2.5);
  glColor3f(1.0, 1.0, 1.0);
  glBegin(GL_LINES);
  glVertex3f(0.0, 0.0, 0.0);
  glVertex3f(x * xrat, -y * yrat, 0);
  glEnd();
  glPopMatrix();
}

void draw::line(const float &x, const float &y, const float &width)
{
  glPushMatrix();
  glLineWidth(width);
  glColor3f(1.0, 1.0, 1.0);
  glBegin(GL_LINES);
  glVertex3f(0.0, 0.0, 0.0);
  glVertex3f(x * xrat, -y * yrat, 0);
  glEnd();
  glPopMatrix();
}

void draw::point()
{
  glPointSize(10.0);
  glBegin(GL_POINTS);
  glVertex3f(0, 0, 0);
  glEnd();
}

void draw::axes()
{
  float lineintensity = 1.0;
  glLineWidth(0.5);
  glBegin(GL_LINES);
  glLineStipple(1, 0xAAAA);  // [1]
  glEnable(GL_LINE_STIPPLE);
  glColor3ub(255 * lineintensity, 255 * lineintensity, 255 * lineintensity); // white
  glVertex3f(-1000,  0.0, 0.0);
  glVertex3f(1000.0,  0.0, 0.0);
  glEnd();

  glBegin(GL_LINES);
  glLineStipple(1, 0xAAAA);  // [1]
  glEnable(GL_LINE_STIPPLE);
  glColor3ub(255 * lineintensity, 255 * lineintensity, 255 * lineintensity); // white
  glVertex3f(0.0, -1000.0, 0.0);
  glVertex3f(0.0,  1000.0, 0.0);
  glEnd();
  glPopAttrib();
}

void draw::segment(const float &x0, const float &y0, const float &x1, const float &y1)
{
  glLineWidth(5);
  float lineintensity = 1.0;
  glBegin(GL_LINES);
  glColor3ub(128 * lineintensity, 128 * lineintensity, 128 * lineintensity); // white
  glVertex3f(x0 * xrat, y0 * yrat, 0.0);
  glVertex3f(x1 * xrat, y1 * yrat, 0.0);
  glEnd();
}

void draw::agent(const uint16_t &ID, const float &x, const float &y, const float &orientation)
{
  glPushMatrix();
  glTranslatef(y * xrat, x * yrat, 0.0); // ENU to NED
  glRotatef(90.0 - rad2deg(orientation), 0.0, 0, 1);
  s[ID]->animation(); // Uses the animation function defined by the agent in use
  s[ID]->controller->animation(ID); // Draws additional stuff from the controller, such as sensors
  agent_number(ID);
  glPopMatrix();
}

void draw::covellipse(float cx, float cy, float rx, float ry) 
{  


  glPushMatrix();
  glLineWidth(2.5);
  glColor3f(1.0, 1.0, 1.0);
    float num_segments = 300;
    float theta = 2 * 3.1415926 / float(num_segments); 
    float c = cosf(theta);//precalculate the sine and cosine
    float s = sinf(theta);
    float t;

    float x = 1;//we start at angle = 0 
    float y = 0; 

    glBegin(GL_LINE_LOOP); 

    for(int ii = 0; ii < num_segments; ii++) 
    { 
        //apply radius and offset
        glVertex2f(x * rx + cx, y * ry + cy);//output vertex 

        //apply the rotation matrix
        t = x;
        x = c * x - s * y;
        y = s * t + c * y;
    } 
    glEnd(); 
    glPopMatrix();
}

void draw::agent_estimate(const uint16_t &ID, const float &x, const float &y, const float &orientation)
{
  glPushMatrix();
  glTranslatef(y * xrat, x * yrat, 0.0); // ENU to NED
  glRotatef(90.0 - rad2deg(orientation), 0.0, 0, 1);
  glPushMatrix();

  glBegin(GL_POLYGON);
  float scl = 0.2;
  glColor3ub(124, 252, 000); // green ish
  glVertex2f(-1 * scl,  1 * scl);
  glVertex2f(-1 * scl, -1 * scl);
  glVertex2f(1.0 * scl,  0 * scl);
  glEnd();

  glColor3ub(255, 255, 255); // White
  glPopMatrix();
  
  agent_number(ID);
  glPopMatrix();
}


void draw::agent_trajectory(const uint16_t &ID, const float &x, const float &y, const float &orientation)
{
  glPushMatrix();
  glTranslatef(y * xrat, x * yrat, 0.0); // ENU to NED
  glRotatef(90.0 - rad2deg(orientation), 0.0, 0, 1);
  //s[ID]->animation(); // Uses the animation function defined by the agent in use
  //s[ID]->controller->animation(ID); // Draws additional stuff from the controller, such as sensors
  glPushMatrix();

  glBegin(GL_POLYGON);
  float scl = 0.2;
  glColor3ub(0, 191, 255); // deepsky blue :P
  glVertex2f(-1 * scl,  1 * scl);
  glVertex2f(-1 * scl, -1 * scl);
  glVertex2f(1.0 * scl,  0 * scl);
  glEnd();

  glColor3ub(255, 255, 255); // White
  glPopMatrix();
  
  agent_number(ID);
  glPopMatrix();
}

void draw::velocity_arrow(const uint16_t &ID, const float &x, const float &y, const float &v_x, const float &v_y)
{
  glPushMatrix();
  glTranslatef(y * xrat, x * yrat, 0.0); // ENU to NED
  glRotatef(90.0, 0.0, 0.0, 1.0);
  line(v_x, v_y);
  glPopMatrix();
}

void draw::food(const float &x, const float &y)
{
  glPushMatrix();
  glTranslatef(y * xrat, x * yrat, 0.0);
  glRotatef(90, 0.0, 0.0, 1.0);
  point();
  glPopMatrix();
}
