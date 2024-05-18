#pragma once
#ifndef BLAZERT_NUMPY_WRAPPER_H
#define BLAZERT_NUMPY_WRAPPER_H

#include "cnpy.h"

template<typename T, typename P>
std::unique_ptr<T> read_1d_array(const std::string &fname) {

  auto arr = cnpy::npy_load(fname);
  auto data = std::make_unique<T>(arr.num_vals);

  for (size_t i = 0; i < arr.num_vals; i++) {
    (*data)[i] = arr.data<P>()[i];
  }

  return data;
}

template<typename T, typename P>
std::unique_ptr<T> read_2d_array(const std::string &fname) {

  auto arr = cnpy::npy_load(fname);
  auto data = std::make_unique<T>(arr.shape[0]);
  //data->resize(arr.shape[0];

  std::cout << arr.shape[0] << ", " << arr.shape[1] << std::endl;
  for (size_t i = 0; i < arr.shape[0]; i++) {
    for (size_t k = 0; k < arr.shape[1]; k++) {
      (*data)[i][k] = arr.data<P>()[k + i * arr.shape[1]];
    }
  }

  return data;
}

template<typename T, typename P>
std::unique_ptr<T> read_arbitrary_2d_array(const std::string &fname) {

  auto arr = cnpy::npy_load(fname);
  auto data = std::make_unique<T>(arr.shape[0] * arr.shape[1]);
  //data->resize(arr.shape[0];

  std::cout << arr.shape[0] << ", " << arr.shape[1] << std::endl;
  for (size_t i = 0; i < arr.shape[0]; i++) {
    for (size_t k = 0; k < arr.shape[1]; k++) {
      (*data)[k + i * arr.shape[1]] = arr.data<P>()[k + i * arr.shape[1]];
    }
  }

  return data;
}

template<typename T, typename P>
std::unique_ptr<T> read_3d_array(const std::string &fname) {

  auto arr = cnpy::npy_load(fname);
  auto data = std::make_unique<T>(arr.shape[0] * arr.shape[1] * arr.shape[2]);
  //data->resize(arr.shape[0];

  std::cout << arr.shape[0] << ", " << arr.shape[1] << ", " << arr.shape[2] << std::endl;
  for (size_t i = 0; i < arr.shape[0]; i++) {
    for (size_t j = 0; j < arr.shape[1]; j++) {
      for (size_t k = 0; k < arr.shape[2]; k++) {
        (*data)[k + j * arr.shape[2] + i * arr.shape[2] * arr.shape[1]] =
            arr.data<P>()[k + j * arr.shape[2] + i * arr.shape[2] * arr.shape[1]];
      }
    }
  }

  return data;
}
#endif//BLAZERT_NUMPY_WRAPPER_H
