#include "primitive.hpp"
#include <GL/gl.h>
#include <GL/glu.h>

Primitive::~Primitive()
{
}

Sphere::~Sphere()
{
}

void Sphere::walk_gl(bool picking)
{
  if(list_name) {
    glCallList(list_name);
  } else {
    list_name = glGenLists(1);
    if(list_name) glNewList(list_name,GL_COMPILE_AND_EXECUTE);
    GLUquadricObj *sphere = gluNewQuadric();
    glMatrixMode(GL_MODELVIEW);
    gluQuadricOrientation(sphere,GLU_OUTSIDE);
    gluQuadricDrawStyle(sphere,GLU_FILL);
    gluSphere(sphere,1.0,36,36);
    if(list_name) glEndList();
  }
}
