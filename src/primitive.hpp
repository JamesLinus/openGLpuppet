#ifndef CS488_PRIMITIVE_HPP
#define CS488_PRIMITIVE_HPP

#include "algebra.hpp"
#include <GL/gl.h>
#include <GL/glu.h>

class Primitive {
public:
  virtual ~Primitive();
  virtual void walk_gl(bool picking = false) = 0;
};

class Sphere : public Primitive {
public:
  Sphere() : list_name(0) {}
  virtual ~Sphere();
  virtual void walk_gl(bool picking);
protected:
  GLuint list_name;
};

#endif
