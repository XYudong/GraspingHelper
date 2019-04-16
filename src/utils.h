//
// Created by yudong on 4/13/19.
//

#ifndef FIRSTTEST_UTILS_H
#define FIRSTTEST_UTILS_H

//#include "pclPlay.h"

//#include <iostream>
#include <string>
#include <vector>
#include <cmath>


void get_all_paths(const std::string& root,
  const std::string& ext,
  std::vector<std::string>& ret);

template <typename T>
T dotProduct(std::vector<T> vec_a, std::vector<T> vec_b) {
  T product = 0;
  size_t n = vec_a.size();
  // Loop for calculate cot product
  for (size_t i = 0; i < n; i++) {
    product += vec_a[i] * vec_b[i];
  }

//  std::cout << product << std::endl;
  return product;
}

float radianToDegree(float radian);


#endif //FIRSTTEST_UTILS_H
