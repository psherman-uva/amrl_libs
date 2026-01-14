
#include <amrl_libs/filter/SavitzkyGolayFilter.hpp>
#include <amrl_common/util/util.hpp>

#include <iostream>
#include <algorithm>

namespace amrl {

SavitzkyGolayFilter::SavitzkyGolayFilter(uint32_t input_size) :
  _c(kWindownLen),
  _output(input_size),
  _mirror(input_size + 2*kM),
  kInputSize(input_size)
{
 _c[0] = -0.08571429;
 _c[1] =  0.34285714;
 _c[2] =  0.48571429;
 _c[3] =  0.34285714;
 _c[4] = -0.08571429;
}

std::vector<double> SavitzkyGolayFilter::filter(const std::vector<double> &input)
{
  if(input.size() < kPolyOrder) { return input; }
  
  // Mirror Values at start/end of input 
  std::copy_n(input.begin(), kInputSize, _mirror.begin() + kM);
  for(size_t i = 0 ; i < kM; ++i) {
    *(_mirror.begin() + i)  = *(input.begin()  + kM - i);
    *(_mirror.rbegin() + i) = *(input.rbegin() + kM - i);
  }
  
  // Digital low pass filter
  for(size_t i = 0; i < kInputSize; ++i) {
    _output[i] = 0.0;

    for (size_t j = 0; j < kWindownLen; ++j) {
      _output[i] += _c[j] * _mirror[i + j];
    }
  }

  return _output;
}

}
