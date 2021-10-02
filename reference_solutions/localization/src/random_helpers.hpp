// Copyright 2021 RoboJackets
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef RANDOM_HELPERS_HPP_
#define RANDOM_HELPERS_HPP_

#include <random>

namespace localization
{

class UniformRandomGenerator
{
public:
  UniformRandomGenerator();

  double Sample();

private:
  std::default_random_engine engine_;
  std::uniform_real_distribution<double> distribution_;
};

class GaussianRandomGenerator
{
public:
  GaussianRandomGenerator();

  double Sample();

private:
  std::default_random_engine engine_;
  std::normal_distribution<double> distribution_;
};

}  // namespace localization

#endif  // RANDOM_HELPERS_HPP_
