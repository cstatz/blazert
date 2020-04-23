// SAH functions
//

template <typename T>
inline T SAH(size_t ns1, T leftArea, size_t ns2, T rightArea, T invS, T Taabb,
             T Ttri) {
  T sah;

  sah = static_cast<T>(2.0) * Taabb +
        (leftArea * invS) * static_cast<T>(ns1) * Ttri +
        (rightArea * invS) * static_cast<T>(ns2) * Ttri;

  return sah;
}

