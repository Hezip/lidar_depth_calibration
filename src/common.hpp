//
// Created by he on 2020/4/7.
//

#ifndef HZP_COMMON_HPP
#define HZP_COMMON_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <fstream>

bool loadMatrix(const std::string& path, Eigen::Matrix4d &estimate){
  std::ifstream ifs(path);
  if(!ifs) {
    return false;
  }
  while(!ifs.eof()){
    std::string token;
    ifs >> token;

    if(token == "estimate"){
      Eigen::Matrix4d mat;
      for(int i=0; i<4; i++) {
        for(int j=0; j<4; j++) {
          ifs >> mat(i, j);
        }
      }
      estimate = mat;
    }
  }
  return true;
}

void print4x4Matrix (const Eigen::Matrix4d & matrix) {
  printf ("Rotation matrix :\n");
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
  printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
  printf ("Translation vector :\n");
  printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}


//void printInformation() {
//
//}

#endif //HZP_COMMON_HPP
