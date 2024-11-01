#ifndef KINEMATIC_HPP
#define KINEMATIC_HPP

#include <math.h>

namespace kinematic
{
  class Motor
  {
  public:
    Motor()
    {
      v1 = .0;
      v2 = .0;
      v3 = .0;
    }
    Motor(float a1, float a2, float a3) : a1(a1 * M_PI / 180), a2(a2 * M_PI / 180), a3(a3 * M_PI / 180)
    {
    }
    float a1, a2, a3;
    float v1, v2, v3;
    float r = 0.07;
  };
}

#endif