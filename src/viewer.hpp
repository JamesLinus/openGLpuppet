#ifndef CS488_VIEWER_HPP
#define CS488_VIEWER_HPP

#include <gtkmm.h>
#include <gtkglmm.h>
#include "scene.hpp"
#include <set>
#include "algebra.hpp"

#define BUFFER_SIZE 400 // easier than dynamic resize, but less robust

// The "main" OpenGL widget
class Viewer : public Gtk::GL::DrawingArea {
public:
  Viewer(SceneNode *root);
  virtual ~Viewer();

  // A useful function that forces this widget to rerender. If you
  // want to render a new frame, do not call on_expose_event
  // directly. Instead call this, which will cause an on_expose_event
  // call when the time is right.
  void invalidate();

  void reset_joints() { puppet->reset(); invalidate(); };
  //void reset_position() { position = translation(Vector3D(0,0,-15)); invalidate(); }
  void reset_position() { position = init; invalidate(); }
  void reset_orientation() { orientation = Matrix4x4(); invalidate(); };
  void reset_all() {
    reset_joints();
    reset_position();
    reset_orientation();
  }
  void toggle_circle() { circle = !circle; invalidate(); }
  void depth_buffer() { zbuf = !zbuf; invalidate(); }
  void back_culling() { backcull = !backcull; invalidate(); }
  void front_culling() { frontcull = !frontcull; invalidate(); }

  enum Mode {
    JOINT,
    TRACK
  };
  void set_mode(Mode m) { mode = m; }

  struct JointState {
    JointState(JointNode* j, double a, double b) : joint(j), x(a), y(b) {}
    JointNode *joint;
    double x;
    double y;
    JointState apply() {
      JointState cur = JointState(joint,joint->get_x(),joint->get_y());
      double dx = x - cur.x;
      double dy = y - cur.y;
      joint->bend('x',dx);
      joint->bend('y',dy);
      return cur;
    }
  };
  void undo();
  void redo();

protected:

  // Events we implement
  // Note that we could use gtkmm's "signals and slots" mechanism
  // instead, but for many classes there's a convenient member
  // function one just needs to define that'll be called with the
  // event.

  // Called when GL is first initialized
  virtual void on_realize();
  // Called when our window needs to be redrawn
  virtual bool on_expose_event(GdkEventExpose* event);
  // Called when the window is resized
  virtual bool on_configure_event(GdkEventConfigure* event);
  // Called when a mouse button is pressed
  virtual bool on_button_press_event(GdkEventButton* event);
  // Called when a mouse button is released
  virtual bool on_button_release_event(GdkEventButton* event);
  // Called when the mouse moves
  virtual bool on_motion_notify_event(GdkEventMotion* event);

  // Draw a circle for the trackball, with OpenGL commands.
  // Assumes the context for the viewer is active.
  void draw_trackball_circle();

  // do picking, results will be found in sbuffer;
  int select(double,double,double,double);
  void process_hits(unsigned int);

private:
	SceneNode *puppet;
  static const size_t buffer_size = BUFFER_SIZE;
  unsigned int selection_buffer[buffer_size];
  std::set<JointNode*> active_joints;
  bool circle, frontcull, backcull, zbuf;
  Mode mode;
  Matrix4x4 init;
  Matrix4x4 position;
  Matrix4x4 orientation;
  Point2D old_point;
  std::vector<std::vector<JointState> > undo_stack;
  std::vector<std::vector<JointState> > redo_stack;
};

#endif
