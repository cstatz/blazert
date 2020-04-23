
template <typename T>
inline bool IntersectRayAABB(T *tminOut,  // [out]
                             T *tmaxOut,  // [out]
                             T min_t, T max_t, const T bmin[3], const T bmax[3],
                             real3<T> ray_org, real3<T> ray_inv_dir,
                             int ray_dir_sign[3]);
template <>
inline bool IntersectRayAABB<float>(float *tminOut,  // [out]
                                    float *tmaxOut,  // [out]
                                    float min_t, float max_t,
                                    const float bmin[3], const float bmax[3],
                                    real3<float> ray_org,
                                    real3<float> ray_inv_dir,
                                    int ray_dir_sign[3]) {
  float tmin, tmax;

  const float min_x = ray_dir_sign[0] ? bmax[0] : bmin[0];
  const float min_y = ray_dir_sign[1] ? bmax[1] : bmin[1];
  const float min_z = ray_dir_sign[2] ? bmax[2] : bmin[2];
  const float max_x = ray_dir_sign[0] ? bmin[0] : bmax[0];
  const float max_y = ray_dir_sign[1] ? bmin[1] : bmax[1];
  const float max_z = ray_dir_sign[2] ? bmin[2] : bmax[2];

  // X
  const float tmin_x = (min_x - ray_org[0]) * ray_inv_dir[0];
  // MaxMult robust BVH traversal(up to 4 ulp).
  // 1.0000000000000004 for double precision.
  const float tmax_x = (max_x - ray_org[0]) * ray_inv_dir[0] * 1.00000024f;

  // Y
  const float tmin_y = (min_y - ray_org[1]) * ray_inv_dir[1];
  const float tmax_y = (max_y - ray_org[1]) * ray_inv_dir[1] * 1.00000024f;

  // Z
  const float tmin_z = (min_z - ray_org[2]) * ray_inv_dir[2];
  const float tmax_z = (max_z - ray_org[2]) * ray_inv_dir[2] * 1.00000024f;

  tmin = safemax(tmin_z, safemax(tmin_y, safemax(tmin_x, min_t)));
  tmax = safemin(tmax_z, safemin(tmax_y, safemin(tmax_x, max_t)));

  if (tmin <= tmax) {
    (*tminOut) = tmin;
    (*tmaxOut) = tmax;

    return true;
  }
  return false;  // no hit
}

template <>
inline bool IntersectRayAABB<double>(double *tminOut,  // [out]
                                     double *tmaxOut,  // [out]
                                     double min_t, double max_t,
                                     const double bmin[3], const double bmax[3],
                                     real3<double> ray_org,
                                     real3<double> ray_inv_dir,
                                     int ray_dir_sign[3]) {
  double tmin, tmax;

  const double min_x = ray_dir_sign[0] ? bmax[0] : bmin[0];
  const double min_y = ray_dir_sign[1] ? bmax[1] : bmin[1];
  const double min_z = ray_dir_sign[2] ? bmax[2] : bmin[2];
  const double max_x = ray_dir_sign[0] ? bmin[0] : bmax[0];
  const double max_y = ray_dir_sign[1] ? bmin[1] : bmax[1];
  const double max_z = ray_dir_sign[2] ? bmin[2] : bmax[2];

  // X
  const double tmin_x = (min_x - ray_org[0]) * ray_inv_dir[0];
  // MaxMult robust BVH traversal(up to 4 ulp).
  const double tmax_x =
      (max_x - ray_org[0]) * ray_inv_dir[0] * 1.0000000000000004;

  // Y
  const double tmin_y = (min_y - ray_org[1]) * ray_inv_dir[1];
  const double tmax_y =
      (max_y - ray_org[1]) * ray_inv_dir[1] * 1.0000000000000004;

  // Z
  const double tmin_z = (min_z - ray_org[2]) * ray_inv_dir[2];
  const double tmax_z =
      (max_z - ray_org[2]) * ray_inv_dir[2] * 1.0000000000000004;

  tmin = safemax(tmin_z, safemax(tmin_y, safemax(tmin_x, min_t)));
  tmax = safemin(tmax_z, safemin(tmax_y, safemin(tmax_x, max_t)));

  if (tmin <= tmax) {
    (*tminOut) = tmin;
    (*tmaxOut) = tmax;

    return true;
  }
  return false;  // no hit
}

// NaN-safe min and max function.
template <class T>
const T &safemin(const T &a, const T &b) {
  return (a < b) ? a : b;
}
template <class T>
const T &safemax(const T &a, const T &b) {
  return (a > b) ? a : b;
}

