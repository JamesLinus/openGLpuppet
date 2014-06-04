#include "scene.hpp"
#include <iostream>
#include <GL/gl.h>
#include <GL/glu.h>
#include <math.h>
#include <assert.h>

// grabbin the codes from A2
Matrix4x4 rotation(char axis, double angle)
{
  double rad = angle * (2.0 * M_PI) / 360.0;
	Matrix4x4 r;
	switch(axis)
	{
		case 'z':
			r = Matrix4x4(
				Vector4D(cos(rad),(-1.0*sin(rad)),0.0,0.0),
				Vector4D(sin(rad),cos(rad),0.0,0.0),
				Vector4D(0.0,0.0,1.0,0.0),
				Vector4D(0.0,0.0,0.0,1.0)
			);
			break;
		case 'x':
			r = Matrix4x4(
				Vector4D(1.0,0.0,0.0,0.0),
				Vector4D(0.0,cos(rad),(-1.0*sin(rad)),0.0),
				Vector4D(0.0,sin(rad),cos(rad),0.0),
				Vector4D(0.0,0.0,0.0,1.0)
			);
			break;
		case 'y':
			r = Matrix4x4(
				Vector4D(cos(rad),0.0,sin(rad),0.0),
				Vector4D(0.0,1.0,0.0,0.0),
				Vector4D((-1.0*sin(rad)),0.0,cos(rad),0.0),
				Vector4D(0.0,0.0,0.0,1.0)
			);
			break;
	}
	return r;
}

// Return a matrix to represent a displacement of the given vector.
Matrix4x4 translation(const Vector3D& displacement)
{
	Matrix4x4 t(
		Vector4D(1.0,0.0,0.0,displacement[0]),
		Vector4D(0.0,1.0,0.0,displacement[1]),
		Vector4D(0.0,0.0,1.0,displacement[2]),
		Vector4D(0.0,0.0,0.0,1.0)
	);
	return t;
}

// Return a matrix to represent a nonuniform scale with the given factors.
Matrix4x4 scaling(const Vector3D& scale)
{
	Matrix4x4 s(
		Vector4D(scale[0],0.0,0.0,0.0),
		Vector4D(0.0,scale[1],0.0,0.0),
		Vector4D(0.0,0.0,scale[2],0.0),
		Vector4D(0.0,0.0,0.0,1.0)
	);
	return s;
}

static unsigned int NODE_ID = 0;
SceneNode::SceneNode(const std::string& name)
  : m_id(NODE_ID++), m_name(name), selected(false)
{
}

SceneNode::~SceneNode()
{
}

SceneNode* SceneNode::find(unsigned int id)
{
  SceneNode *node = NULL;
  if(m_id == id) {
    //std::cerr << "node " << id << " is " << m_name << std::endl;
    return this;
  }
  for(ChildList::iterator child = m_children.begin(); child != m_children.end(); child++)
  {
    if((node = (*child)->find(id))) return node;
  }
  // id not found
  return NULL;
}

void SceneNode::toggle()
{
  std::cerr << "Warning: toggle() called on non-GeometryNode " << m_name << std::endl;
}

void SceneNode::walk_gl(bool picking)
{
  if(picking) glPushName(m_id);
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glMultMatrixd(m_trans.transpose().begin());
  for(ChildList::iterator node = m_children.begin(); node != m_children.end(); node++)
  {
    (*node)->walk_gl(picking);
  }
  glPopMatrix();
  if(picking) glPopName();
}

void SceneNode::reset()
{
  selected = false;
  for(ChildList::iterator node = m_children.begin(); node != m_children.end(); node++)
  {
    (*node)->reset();
  }
}

void SceneNode::rotate(char axis, double angle)
{
  //std::cerr << "Stub: Rotate " << m_name << " around " << axis << " by " << angle << std::endl;
  set_transform(m_trans * rotation(axis,angle));
}

void SceneNode::scale(const Vector3D& amount)
{
  //std::cerr << "Stub: Scale " << m_name << " by " << amount << std::endl;
  set_transform(m_trans * scaling(amount));
}

void SceneNode::translate(const Vector3D& amount)
{
  //std::cerr << "Stub: Translate " << m_name << " by " << amount << std::endl;
  set_transform(m_trans * translation(amount));
}

bool SceneNode::is_joint() const
{
  return false;
}

bool SceneNode::is_geometry() const
{
  return false;
}

JointNode::JointNode(const std::string& name)
  : SceneNode(name), x_angle(0), y_angle(0)
{
}

JointNode::~JointNode()
{
}

bool JointNode::is_joint() const
{
  return true;
}

//UndoAction JointNode::bend(char axis, double angle)
void JointNode::bend(char axis, double angle)
{
  double val;
  switch(axis) {
    case 'x':
      val = x_angle + angle;
      val = std::min(m_joint_x.max,std::max(m_joint_x.min,val));
      val -= x_angle;
      x_angle += val;
      break;
    case 'y':
      val = y_angle + angle;
      val = std::min(m_joint_y.max,std::max(m_joint_y.min,val));
      val -= y_angle;
      y_angle += val;
      break;
    default:
      std::cerr << "Warning: call to JointNode::bend() with invalid axis argument: " << axis << std::endl;
  }
  //return UndoAction(this,axis,-val);
}

void JointNode::reset()
{
  x_angle = m_joint_x.init;
  y_angle = m_joint_y.init;
  SceneNode::reset();
}

void JointNode::walk_gl(bool picking)
{
  Matrix4x4 bending = rotation('x',x_angle) * rotation('y',y_angle);
  if(picking) glPushName(m_id);
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glMultMatrixd(m_trans.transpose().begin());
  glMultMatrixd(bending.transpose().begin());
  for(ChildList::iterator node = m_children.begin(); node != m_children.end(); node++)
  {
    (*node)->walk_gl(picking);
  }
  glPopMatrix();
  if(picking) glPopName();
}

void JointNode::set_joint_x(double min, double init, double max)
{
  m_joint_x.min = min;
  m_joint_x.init = init;
  m_joint_x.max = max;
  x_angle = init;
}

void JointNode::set_joint_y(double min, double init, double max)
{
  m_joint_y.min = min;
  m_joint_y.init = init;
  m_joint_y.max = max;
  y_angle = init;
}

GeometryNode::GeometryNode(const std::string& name, Primitive* primitive)
  : SceneNode(name), m_material(NULL), m_primitive(primitive)
{
}

GeometryNode::~GeometryNode()
{
}

void GeometryNode::toggle()
{
  selected = !selected;
}

bool GeometryNode::is_geometry() const
{
  return true;
}

void GeometryNode::walk_gl(bool picking)
{
  const Material *mat = selected ? &sel_material : m_material;
  if(picking) glPushName(m_id);
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glMultMatrixd(m_trans.transpose().begin());
  if (mat) {
    mat->apply_gl();
  } else {
    std::cerr << "Error: " << m_name << " doesn't have a material!" << std::endl;
  }
  m_primitive->walk_gl(picking);
  for(ChildList::iterator node = m_children.begin(); node != m_children.end(); node++)
  {
    (*node)->walk_gl(picking);
  }
  glPopMatrix();
  if(picking) glPopName();
}

const PhongMaterial GeometryNode::sel_material = PhongMaterial((Colour(1.0)),(Colour(0.1)),10.0); // white

#if 0
void UndoAction::print()
{
  std::cerr << joint->name() << " " << a << " " << v << " " << std::endl;
}
#endif
