#include "material.hpp"
#include <GL/gl.h>
#include <GL/glu.h>

Material::~Material()
{
}

PhongMaterial::PhongMaterial(const Colour& kd, const Colour& ks, double shininess)
  : m_kd(kd), m_ks(ks), m_shininess(shininess)
{
}

PhongMaterial::~PhongMaterial()
{
}

void PhongMaterial::apply_gl() const
{
  float diff[] = {m_kd.R(),m_kd.G(),m_kd.B()};
  float spec[] = {m_ks.R(),m_ks.G(),m_ks.B()};
  float shin[] = {m_shininess};
  glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,diff);
  glMaterialfv(GL_FRONT_AND_BACK,GL_SPECULAR,spec);
  glMaterialfv(GL_FRONT_AND_BACK,GL_SHININESS,shin);
}
