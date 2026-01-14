/*
File:   prob_util.hpp
Author: pdsherman-uva
Date:   Aug 2024
*/

#pragma once

#include <vector>

namespace amrl {
namespace util {

double multivariable_normal_cdf(
  const std::vector<double> &x_upper,
  const std::vector<double> &x_lower,
  const std::vector<double> &mu,
  const std::vector<double> &sigma
);

}
}