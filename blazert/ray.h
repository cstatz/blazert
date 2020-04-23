
// RayType
typedef enum {
  RAY_TYPE_NONE = 0x0,
  RAY_TYPE_PRIMARY = 0x1,
  RAY_TYPE_SECONDARY = 0x2,
  RAY_TYPE_DIFFUSE = 0x4,
  RAY_TYPE_REFLECTION = 0x8,
  RAY_TYPE_REFRACTION = 0x10
} RayType;

#ifdef __clang__
#pragma clang diagnostic push
#if __has_warning("-Wzero-as-null-pointer-constant")
#pragma clang diagnostic ignored "-Wzero-as-null-pointer-constant"
#endif
#endif


template <typename T = float>
class Ray {
 public:
  Ray()
      : min_t(static_cast<T>(0.0)),
        max_t(std::numeric_limits<T>::max()),
        type(RAY_TYPE_NONE) {
    org[0] = static_cast<T>(0.0);
    org[1] = static_cast<T>(0.0);
    org[2] = static_cast<T>(0.0);
    dir[0] = static_cast<T>(0.0);
    dir[1] = static_cast<T>(0.0);
    dir[2] = static_cast<T>(-1.0);
  }

  T org[3];           // must set
  T dir[3];           // must set
  T min_t;            // minimum ray hit distance.
  T max_t;            // maximum ray hit distance.
  unsigned int type;  // ray type

  // TODO(LTE): Align sizeof(Ray)
};

