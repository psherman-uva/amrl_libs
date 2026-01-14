
#pragma once

#include <vector>
#include <cstdint>

namespace amrl {

class SavitzkyGolayFilter
{
public:
  SavitzkyGolayFilter(uint32_t input_size);
  ~SavitzkyGolayFilter(void) = default;

  std::vector<double> filter(const std::vector<double> &input);

private:

  std::vector<double> _c; // Filter coefficients

  std::vector<double> _output;
  std::vector<double> _mirror;
  const uint32_t kInputSize;
  
  static constexpr size_t kPolyOrder  = 3;
  static constexpr size_t kWindownLen = 5;
  static constexpr size_t kM = (kWindownLen - 1) / 2;
};


} // namespace amrl