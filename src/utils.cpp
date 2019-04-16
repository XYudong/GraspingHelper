//
// Created by yudong on 4/13/19.
//

#include "utils.h"
#include <boost/filesystem.hpp>


namespace fs = boost::filesystem;


void get_all_paths(const std::string& root, const std::string& ext, std::vector<std::string>& ret) {
  //// return the paths to of all files that have the specified extension
  //// in the specified directory and all subdirectories

  fs::path dir(root);
  if(!fs::exists(dir) || !fs::is_directory(dir)) return;

  fs::recursive_directory_iterator it(dir);
  fs::recursive_directory_iterator endit;

  while(it != endit) {
    if(fs::is_regular_file(*it) && it->path().extension() == ext) {
      ret.push_back((it->path()).string());
    }
    ++it;
  }
}

float radianToDegree(float radian) {
  float pi = 3.141593;
  float degree = radian / pi * 180;
  return degree > 90 ? 180 - degree : degree;
}

