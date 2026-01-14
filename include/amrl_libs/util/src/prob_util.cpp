
#include <amrl_libs/util/prob_util.hpp>

#include <approxcdf.h>

namespace amrl {
namespace util {

double multivariable_normal_cdf(
  const std::vector<double> &x_upper,
  const std::vector<double> &x_lower,
  const std::vector<double> &mu,
  const std::vector<double> &sigma
)
{
  const int n = mu.size();

  std::vector<std::vector<double>> X;
  X.push_back({x_upper[0], x_upper[1]});
  X.push_back({x_upper[0], x_lower[1]});
  X.push_back({x_lower[0], x_upper[1]});
  X.push_back({x_lower[0], x_lower[1]});

  std::vector<double> prob(4);

  for(size_t i = 0; i < 4; ++i) {
    prob[i] = norm_cdf(
      X[i].data(),  // x
      sigma.data(), // Sigma
      n,            // leading dimension of Sigma
      mu.data(),    // mu
      n,            // n (num of variables)
      false,        // True if standard normal distribution
      false,        // If true, return the logarithm of the probability
      NULL
    );
  }

  return prob[0] - prob[1] - prob[2] + prob[3];
}

}
}