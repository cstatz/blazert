#pragma once
#ifndef BLAZERT_PROGRESS_BAR_H
#define BLAZERT_PROGRESS_BAR_H

#include <string>
#include <iostream>
#include <cmath>

void progress_bar(unsigned long tick, unsigned long total, unsigned long width = 100) {
  double ratio = 100.0 * double(tick) / double(total);
  auto count = static_cast<unsigned int>(std::floor(width * tick / total));
  std::string bar(width, ' ');
  std::fill(bar.begin(), bar.begin() + count, '+');
  std::cout << "[ " << ratio << "%% ] [ " << bar << " ]" << (tick == total ? '\n' : '\r');
  std::fflush(stdout);
}

#endif //BLAZERT_PROGRESS_BAR_H
