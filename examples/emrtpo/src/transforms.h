#pragma once
#ifndef EMRTPO_TRANSFORMS_H
#define EMRTPO_TRANSFORMS_H

#include "constants.h"
#include <blazert/datatypes.h>

// Theta Phi
inline __attribute__((always_inline)) blazert::Mat3r<double> cart2sph_matrix(const double &theta, const double &phi) {
  // FEKO EFE: r, theta, phi
  // Matrix conforming wiki:
  //         x y z
  // r
  // theta
  // phi

  const double cos_theta = cos(theta);
  const double sin_theta = sin(theta);
  const double cos_phi = cos(phi);
  const double sin_phi = sin(phi);

  // Column Major
  blazert::Mat3r<double> m;

  m(0, 0) = sin_theta * cos_phi;
  m(0, 1) = sin_theta * sin_phi;
  m(0, 2) = cos_theta;

  m(1, 0) = cos_theta * cos_phi;
  m(1, 1) = cos_theta * sin_phi;
  m(1, 2) = -sin_theta;

  m(2, 0) = -sin_phi;
  m(2, 1) = cos_phi;
  m(2, 2) = 0.;

  return m;
}

// Theta Phi
inline __attribute__((always_inline)) blazert::Mat3r<double> sph2cart_matrix(const double &theta, const double &phi) {
  return cart2sph_matrix(theta, phi).transpose();
}

// Assume cart is normalized!
inline __attribute__((always_inline)) blazert::Vec3r<double> cart2sph(const blazert::Vec3r<double> &cart) {

  blazert::Vec3r<double> sph = {
      acos(cart[2]),          // theta
      atan2(cart[1], cart[0]),// phi
      1.                      // rho
  };

  // TODO: originally, this was sph[1] but sph[1] is phi and not theta
  if (sph[0] < 0.)
    sph[0] += 2. * pi;// theta 0...180 deg

  return sph;
}

// Assume sph is normlized (i.e. rho==1.)!
inline __attribute__((always_inline)) blazert::Vec3r<double> sph2cart(const blazert::Vec3r<double> &sph) {
  const double tmp = sin(sph[0]);
  // theta phi rho
  blazert::Vec3r<double> cart = {tmp * cos(sph[1]), tmp * sin(sph[1]), cos(sph[0])};

  return cart;
}

#endif //EMRTPO_TRANSFORMS_H
