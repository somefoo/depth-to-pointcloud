#pragma once
#include <iostream>
#include <string>
#include <string_view>
#include <typeinfo>

// A simple bit of code to parse arguments.
template <typename RT>
static constexpr RT parse_args(const int argc, char *argv[],
                               const std::string_view name,
                               const RT default_value) {
  for (std::size_t i = 1; i < argc; ++i) {
    if (argv[i] == name && i < argc - 1) {
      try {
        if constexpr (std::is_same<int, RT>::value) {
          return std::stoi(argv[i + 1]);
        } else if constexpr (std::is_same<long, RT>::value) {
          return std::stol(argv[i + 1]);
        } else if constexpr (std::is_same<long long, RT>::value) {
          return std::stoll(argv[i + 1]);
        } else if constexpr (std::is_same<float, RT>::value) {
          return std::stof(argv[i + 1]);
        } else if constexpr (std::is_same<double, RT>::value) {
          return std::stod(argv[i + 1]);
        } else if constexpr (std::is_same<long double, RT>::value) {
          return std::stold(argv[i + 1]);
        } else if constexpr (std::is_same<std::string, RT>::value) {
          return std::string(argv[i + 1]);
        }
      } catch (...) {
        std::cerr << "Error, value for parameter " << name
                  << " is not valid.\n";
        std::cerr << "  Received value  : " << argv[i + 1] << '\n';
        std::cerr << "  Expected type   : " << typeid(RT).name() << '\n';
        std::cerr << "  Defaulting to   : " << default_value << '\n';
      }
    }
  }
  return default_value;
}
