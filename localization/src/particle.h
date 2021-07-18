//
// Created by jason on 7/5/21.
//

#ifndef SRC_PARTICLE_H
#define SRC_PARTICLE_H

#include <random>
#include <iostream>

namespace localization {

struct Particle
{
  // global positions
  double x = 0;
  double y = 0;
  double yaw = 0;

  // body rates
  double vx = 0;
  double vy = 0;

  // normalized weight of the particle
  double weight = 1.0;
};

class ParticleNoise {
public:
  ParticleNoise() {
    normal_dist_ = std::normal_distribution<double>(0, 1.0);
    uniform_dist_ = std::uniform_real_distribution<double>(0, 1);
    generator_ = std::default_random_engine(100);
  }

  double sampleUniform()
  {
    return uniform_dist_(generator_);
  }

  double sampleGaussian()
  {
    return normal_dist_(generator_);
  }
private:
  // creates N(0,1)
  std::normal_distribution<double> normal_dist_;

  // creates U(0,1)
  std::uniform_real_distribution<double> uniform_dist_;
  std::default_random_engine generator_;
};

}
#endif //SRC_PARTICLE_H
