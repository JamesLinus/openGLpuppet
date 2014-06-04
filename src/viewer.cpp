#include "viewer.hpp"
#include "algebra.hpp"
#include <iostream>
#include <math.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <assert.h>

Viewer::Viewer(SceneNode *root)
  : puppet(root),
  circle(false),
  frontcull(false),
  backcull(false),
  zbuf(false),
  mode(TRACK)
{
  Glib::RefPtr<Gdk::GL::Config> glconfig;

  // Ask for an OpenGL Setup with
  //  - red, green and blue component colour
  //  - a depth buffer to avoid things overlapping wrongly
  //  - double-buffered rendering to avoid tearing/flickering
  glconfig = Gdk::GL::Config::create(Gdk::GL::MODE_RGB |
                                     Gdk::GL::MODE_DEPTH |
                                     Gdk::GL::MODE_DOUBLE);
  if (glconfig == 0) {
    // If we can't get this configuration, die
    std::cerr << "Unable to setup OpenGL Configuration!" << std::endl;
    abort();
  }

  // Accept the configuration
  set_gl_capability(glconfig);

  // Register the fact that we want to receive these events
  add_events(Gdk::BUTTON1_MOTION_MASK    |
             Gdk::BUTTON2_MOTION_MASK    |
             Gdk::BUTTON3_MOTION_MASK    |
             Gdk::BUTTON_PRESS_MASK      | 
             Gdk::BUTTON_RELEASE_MASK    |
             Gdk::VISIBILITY_NOTIFY_MASK);

  // steal the root nodes transformation so that trackball
  // rotations can be multiplied in correctly
  init = puppet->get_transform();
  puppet->set_transform(Matrix4x4());
  position = init;
}

Viewer::~Viewer()
{
  // Nothing to do here right now.
}

void Viewer::invalidate()
{
  // Force a rerender
  Gtk::Allocation allocation = get_allocation();
  get_window()->invalidate_rect( allocation, false);
}

void Viewer::on_realize()
{
  // Do some OpenGL setup.
  // First, let the base class do whatever it needs to
  Gtk::GL::DrawingArea::on_realize();
  
  Glib::RefPtr<Gdk::GL::Drawable> gldrawable = get_gl_drawable();
  
  if (!gldrawable)
    return;

  if (!gldrawable->gl_begin(get_gl_context()))
    return;

  glShadeModel(GL_SMOOTH);
  //glClearColor( 0.4, 0.4, 0.4, 0.0 );
  glClearColor(0,0,0,0);
  glEnable(GL_DEPTH_TEST);

  gldrawable->gl_end();
}

bool Viewer::on_expose_event(GdkEventExpose* event)
{
  Glib::RefPtr<Gdk::GL::Drawable> gldrawable = get_gl_drawable();

  if (!gldrawable) return false;

  if (!gldrawable->gl_begin(get_gl_context()))
    return false;

  // Set up for perspective drawing 
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glViewport(0, 0, get_width(), get_height());
  gluPerspective(40.0, (GLfloat)get_width()/(GLfloat)get_height(), 0.1, 1000.0);

  // change to model view for drawing
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  // Clear framebuffer
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // z buffer
  if(zbuf) {
    glDepthMask(GL_TRUE);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
  } else {
    glDisable(GL_DEPTH_TEST);
  }

  // culling
  if(frontcull && backcull) {
    glCullFace(GL_FRONT_AND_BACK);
    glEnable(GL_CULL_FACE);
  } else if(frontcull) {
    glCullFace(GL_FRONT);
    glEnable(GL_CULL_FACE);
  } else if(backcull) {
    glCullFace(GL_BACK);
    glEnable(GL_CULL_FACE);
  } else {
    glDisable(GL_CULL_FACE);
  }

  // Set up lighting
  float light[] = {1,1,1,0};
  glLightfv(GL_LIGHT0,GL_POSITION,light);
  glEnable(GL_NORMALIZE);
  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);

  // Global transformations (from TRACK mode)
  glMultMatrixd(position.transpose().begin());
  glMultMatrixd(orientation.transpose().begin());

  // Draw stuff
  puppet->walk_gl();
  if(circle) draw_trackball_circle();

  // Swap the contents of the front and back buffers so we see what we
  // just drew. This should only be done if double buffering is enabled.
  gldrawable->swap_buffers();

  gldrawable->gl_end();

  return true;
}

int Viewer::select(double x, double y, double w, double h)
{
  int viewport[4] = {0,0,0,0};
  double proj[16];
  glSelectBuffer(buffer_size,selection_buffer);
  glRenderMode(GL_SELECT);
  glInitNames();
  //glPushName(-1); // I wont be using this
  glGetIntegerv(GL_VIEWPORT,viewport);
  glGetDoublev(GL_PROJECTION_MATRIX,proj);
  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  glLoadIdentity();
  gluPickMatrix(x,y,w,h,viewport);
  glMultMatrixd(proj);
  glMatrixMode(GL_MODELVIEW);
  puppet->walk_gl(true);
  glMatrixMode(GL_PROJECTION);
  glPopMatrix();
  glMatrixMode(GL_MODELVIEW);
  return glRenderMode(GL_RENDER);
}

static void print_record(GLuint *record, SceneNode *puppet)
{
  std::cerr
    << "nodes: " << (GLint)record[0] << ", "
    << "minz: " << (GLint)record[1] << ", "
    << "maxz: " << (GLint)record[2] << std::endl;

  for(GLuint i=0; i < record[0]; i++) {
    std::cerr << i << ":" << (GLint)record[3+i] << ":" << puppet->find(record[3+i])->name() << ", ";
  }
  std::cerr << std::endl;
}

// determine which object was selected and then update state accordingly
void Viewer::process_hits(unsigned int hits)
{
  unsigned int *record = selection_buffer;
  unsigned int last;
  int min_z;
  SceneNode *node = NULL;
  SceneNode *joint = NULL;
//std::cerr << "hits: " << hits << std::endl;
  for(unsigned int i=0; i < hits; i++)
  {
//print_record(record, puppet);
    last = record[0] + 2;
    if((i == 0) || (int)record[1] < min_z)
    {
      min_z = record[1];
      if(record[0] > 1) // fewer than 2 nodes -> not selectable
      {
        node = puppet->find(record[last]);
        joint = puppet->find(record[last-1]);
        assert(node); assert(node->is_geometry()); assert(joint);
        if(!joint->is_joint()) node = joint = NULL; // no joint -> not selectable
      }
    }
    record += last + 1; // next record
  }
  // if node or joint is NULL, then we can assume no item selected
  // else assume that node and joint are correctly assigned
  if(node && joint)
  {
    node->toggle();
    if(node->is_selected()) {
      active_joints.insert((JointNode*)joint);
    } else {
      active_joints.erase((JointNode*)joint);
    }
  }
}

// This has been translated from the trackball demo
static Vector3D calcRotVec(Point2D np, Point2D op, double d)
{
  Vector3D nv, ov, axis;
  double z, c;

  c = 2.0 / d;
  np = Point2D(c*np[0],c*np[1]);
  z = 1.0 - np[0] * np[0] - np[1] * np[1];
  if(z < 0.0) {
    nv = (1.0 / sqrt(1.0 - z)) * Vector3D(np[0],np[1],0);
  } else {
    nv = Vector3D(np[0],np[1],sqrt(z));
  }

  op = Point2D(c*op[0],c*op[1]);
  z = 1.0 - op[0] * op[0] - op[1] * op[1];
  if(z < 0.0) {
    z = 1.0 / sqrt(1.0 - z);
    ov = z * Vector3D(op[0],op[1],0);
  } else {
    ov = Vector3D(op[0],op[1],sqrt(z));
  }

  // negating the entire vector makes things work out right
  axis = -1.0 * nv.cross(ov);

  return axis;
}

// This has been translated from the trackball demo
static Matrix4x4 axisRotMatrix(Vector3D axis)
{
  Matrix4x4 rot;
  Vector3D v = axis;
  double length = axis.length();
  if((length < 0.000001) && (length > -0.000001)) {
    return rot;
  }
  v.normalize();

  // I have no idea how or why the following works:
  double a = sin(length);
  double b = cos(length);
  double c = 1.0 - b;
  rot = Matrix4x4(
    Vector4D( (b + v[0]*v[0]*c)      , (v[0]*v[1]*c - a*v[2]) , (v[2]*v[0]*c + a*v[1]) ,0),
    Vector4D( (v[0]*v[1]*c + v[2]*a) , (b + v[1]*v[1]*c)      , (v[2]*v[1]*c - a*v[0]) ,0),
    Vector4D( (v[0]*v[2]*c - v[1]*a) , (v[1]*v[2]*c + a*v[0]) , (b + v[2]*v[2]*c)      ,0),
    Vector4D( 0                      , 0                      , 0                      ,1)
  );
  return rot;
}

void Viewer::undo()
{
  std::vector<JointState> redo_set;
  if(undo_stack.empty()) {
    std::cerr << "undo stack is empty" << std::endl;
    // inform user
  } else {
    for(std::vector<JointState>::iterator unbend = undo_stack.back().begin(); unbend != undo_stack.back().end(); unbend++) {
      redo_set.push_back(unbend->apply());
    }
    redo_stack.push_back(redo_set);
    undo_stack.pop_back();
  }
  invalidate();
}

void Viewer::redo()
{
  std::vector<JointState> undo_set;
  if(redo_stack.empty()) {
    std::cerr << "redo stack is empty" << std::endl;
    // inform user
  } else {
    for(std::vector<JointState>::iterator bend = redo_stack.back().begin(); bend != redo_stack.back().end(); bend++) {
      undo_set.push_back(bend->apply());
    }
    undo_stack.push_back(undo_set);
    redo_stack.pop_back();
  }
  invalidate();
}

bool Viewer::on_configure_event(GdkEventConfigure* event)
{
  Glib::RefPtr<Gdk::GL::Drawable> gldrawable = get_gl_drawable();

  if (!gldrawable) return false;
  
  if (!gldrawable->gl_begin(get_gl_context()))
    return false;

  // Set up perspective projection, using current size and aspect
  // ratio of display

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glViewport(0, 0, event->width, event->height);
  gluPerspective(40.0, (GLfloat)event->width/(GLfloat)event->height, 0.1, 1000.0);

  // Reset to modelview matrix mode
  
  glMatrixMode(GL_MODELVIEW);

  gldrawable->gl_end();

  return true;
}

bool Viewer::on_button_press_event(GdkEventButton* event)
{
  double tol = 5.0;
  double gl_y = get_height() - event->y; // convert y from window system to gl system
  int hits;
  old_point = Point2D(event->x,gl_y);
  std::vector<JointState> snapshot;
  bool snap = false;
  if(mode == JOINT) {
    switch(event->button) {
      case 1: // translation
        hits = select(event->x,gl_y,tol,tol);
        if(hits < 0) {
          std::cerr << "Warning: selection buffer too small for puppet" << std::endl;
        } else {
          process_hits(hits);
        }
        break;
      case 2: // zoom
        snap = !(event->state & GDK_BUTTON3_MASK);
        break;
      case 3: // rotation
        snap = !(event->state & GDK_BUTTON2_MASK);
        break;
    }
    if(snap) {
      // take a snapshot
      for(std::set<JointNode*>::iterator joint = active_joints.begin(); joint != active_joints.end(); joint++) {
        snapshot.push_back(JointState((*joint),(*joint)->get_x(),(*joint)->get_y()));
      }
    }
    if(!snapshot.empty()) {
      undo_stack.push_back(snapshot);
      redo_stack.clear();
    }
  } else if(mode == TRACK) {
    // trackball mode code
    switch(event->button) {
      case 1: // translation
      case 2: // zoom
      case 3: // rotation
        // set track mode to rotation?
        break;
    }
  }
  invalidate();
  //std::cerr << "Stub: Button " << event->button << " pressed" << std::endl;
  return true;
}

bool Viewer::on_button_release_event(GdkEventButton* event)
{
  //std::cerr << "Stub: Button " << event->button << " released" << std::endl;
  return true;
}

bool Viewer::on_motion_notify_event(GdkEventMotion* event)
{
  double gl_y = get_height() - event->y; // convert y from window system to gl system
  Point2D diff(event->x - old_point[0], gl_y - old_point[1]);
  double angle = (diff[0] * 180) / get_width();
  Point2D np, op;
  // copied from trackball code
  double diameter = std::min(get_width(),get_height()) * 0.5;
  if(mode == JOINT) {
    for(std::set<JointNode*>::iterator joint = active_joints.begin(); joint != active_joints.end(); joint++)
    {
      if(event->state & GDK_BUTTON2_MASK) {
        (*joint)->bend('y',angle);
      }
      if(event->state & GDK_BUTTON3_MASK) {
        (*joint)->bend('x',angle);
      }
    }
  } else {
    if(event->state & GDK_BUTTON1_MASK) {
      // translation (x,y)
      position = position * translation(Vector3D(diff[0]/80.0,diff[1]/80.0,0));
    }
    if(event->state & GDK_BUTTON2_MASK) {
      // translation (z)
      position = position * translation(Vector3D(0,0,-diff[1]/50.0));
    }
    if(event->state & GDK_BUTTON3_MASK) {
      // rotation
      np = Point2D(event->x - (get_width() / 2.0), gl_y - (get_height() / 2.0));
      op = Point2D(old_point[0] - (get_width() / 2.0), old_point[1] - (get_height() / 2.0));
      orientation = axisRotMatrix(calcRotVec(np,op,diameter)) * orientation;
    }
  }
  invalidate();
  old_point = Point2D(event->x,gl_y);
  //std::cerr << "Stub: Motion at " << event->x << ", " << event->y << std::endl;
  return true;
}

// I don't understand why resetting the transformation
// matrices HERE fixes things, but it does.
void Viewer::draw_trackball_circle()
{
  int current_width = get_width();
  int current_height = get_height();
  
  // Set up for orthogonal drawing to draw a circle on screen.
  // You'll want to make the rest of the function conditional on
  // whether or not we want to draw the circle this time around.
  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  glLoadIdentity();
  glViewport(0, 0, current_width, current_height);
  glOrtho(0.0, (float)current_width, 
           0.0, (float)current_height, -0.1, 0.1);

  // change to model view for drawing
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();

  // Reset modelview matrix
  glLoadIdentity();

  // draw a circle for use with the trackball 
  glDisable(GL_LIGHTING);
  glEnable(GL_LINE_SMOOTH);
  glColor3f(1.0, 1.0, 1.0);
  double radius = current_width < current_height ? 
    (float)current_width * 0.25 : (float)current_height * 0.25;
  glTranslated((float)current_width * 0.5, (float)current_height * 0.5, 0);
  glBegin(GL_LINE_LOOP);
  for(size_t i=0; i<40; ++i) {
    double cosine = radius * cos(i*2*M_PI/40);
    double sine = radius * sin(i*2*M_PI/40);
    glVertex2f(cosine, sine);
  }
  glEnd();
  glColor3f(0.0, 0.0, 0.0);
  glDisable(GL_LINE_SMOOTH);
  glPopMatrix();
  glMatrixMode(GL_PROJECTION);
  glPopMatrix();
  glMatrixMode(GL_MODELVIEW);
}
