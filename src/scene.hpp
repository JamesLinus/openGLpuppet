#ifndef SCENE_HPP
#define SCENE_HPP

#include <list>
#include "algebra.hpp"
#include "primitive.hpp"
#include "material.hpp"

Matrix4x4 rotation(char,double);
Matrix4x4 translation(const Vector3D&);
Matrix4x4 scaling(const Vector3D&);
//class UndoAction;

class SceneNode {
public:
  SceneNode(const std::string& name);
  virtual ~SceneNode();

  virtual void walk_gl(bool picking = false);

  const Matrix4x4& get_transform() const { return m_trans; }
  const Matrix4x4& get_inverse() const { return m_invtrans; }
  
  void set_transform(const Matrix4x4& m)
  {
    m_trans = m;
    m_invtrans = m.invert();
  }

  void set_transform(const Matrix4x4& m, const Matrix4x4& i)
  {
    m_trans = m;
    m_invtrans = i;
  }

  void add_child(SceneNode* child)
  {
    m_children.push_back(child);
  }

  void remove_child(SceneNode* child)
  {
    m_children.remove(child);
  }

  // Callbacks to be implemented.
  // These will be called from Lua.
  void rotate(char axis, double angle);
  void scale(const Vector3D& amount);
  void translate(const Vector3D& amount);

  // Returns true if and only if this node is a JointNode
  virtual bool is_joint() const;
  // Returns true if and only if this node is a GeometryNode
  virtual bool is_geometry() const;
  virtual bool is_selected() const { return selected; }

  virtual void toggle();

  virtual SceneNode* find(unsigned int id);

  virtual std::string name() { return m_name; }
  virtual unsigned int id() { return m_id; }

  virtual void reset();

  // debugging helper
  void print(std::string prefix = "")
  {
    std::cerr << prefix << m_name << ":" << m_id << std::endl;
    for(ChildList::iterator child = m_children.begin(); child != m_children.end(); child++)
    {
      (*child)->print(prefix + "  ");
    }
  }
  
protected:
  
  // Useful for picking
  unsigned int m_id;
  std::string m_name;

  bool selected;

  // Transformations
  Matrix4x4 m_trans;
  Matrix4x4 m_invtrans;

  // Hierarchy
  typedef std::list<SceneNode*> ChildList;
  ChildList m_children;
};

class JointNode : public SceneNode {
public:
  JointNode(const std::string& name);
  virtual ~JointNode();

  virtual void walk_gl(bool picking = false);

  virtual bool is_joint() const;

  void set_joint_x(double min, double init, double max);
  void set_joint_y(double min, double init, double max);

  struct JointRange {
    double min, init, max;
  };

  double get_x() { return x_angle; }
  double get_y() { return y_angle; }

  //UndoAction bend(char axis, double angle);
  void bend(char axis, double angle);
  virtual void reset();

protected:
  JointRange m_joint_x, m_joint_y;
  double x_angle, y_angle;
};

class GeometryNode : public SceneNode {
public:
  GeometryNode(const std::string& name,
               Primitive* primitive);
  virtual ~GeometryNode();

  virtual void walk_gl(bool picking = false);

  virtual bool is_geometry() const;

  const Material* get_material() const;
  Material* get_material();

  void set_material(Material* material)
  {
    m_material = material;
  }

  virtual void toggle();

protected:
  Material* m_material;
  Primitive* m_primitive;
private:
  static const PhongMaterial sel_material;
};

#if 0
// a simple data type for undo and redo
class UndoAction {
public:
  UndoAction(JointNode* j, char c, double d) : joint(j), a(c), v(d) {}
  UndoAction apply() {
    return joint->bend(a,v);
  }
  void print();
private:
  JointNode* joint;
  char a;
  double v;
};
#endif

#endif
