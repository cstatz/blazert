#pragma once
#ifndef BLAZERT_BVH_SAH_H_
#define BLAZERT_BVH_SAH_H_

namespace blazert {

/**
 * Compute surface area heuristics (SAH)
 * @tparam T
 * @param ns1
 * @param leftArea
 * @param ns2
 * @param rightArea
 * @param invS
 * @param Taabb
 * @param Ttri
 * @return
 */
template<typename T>
inline T SAH(size_t ns1, T leftArea, size_t ns2, T rightArea, T invS, T Taabb, T Ttri) {

  // TODO: Is the static_cast really necessary in this case?
  const T sah = static_cast<T>(2.0) * Taabb + (leftArea * invS) * static_cast<T>(ns1) * Ttri + (rightArea * invS) * static_cast<T>(ns2) * Ttri;
  return sah;
}
}// namespace blazert

#endif// BLAZERT_BVH_SAH_H_