#pragma once
#ifndef EMRTPO_RADARGRAMM_READER_NOTEBOOK_H
#define EMRTPO_RADARGRAMM_READER_NOTEBOOK_H

#include "jupyter_notebook.h"

void generate_radargramm_reader_ipynb(const std::string &output_fname, const std::string &antenna_postition_fname, const std::string &radargram_fname) {

  JupyterNotebook nb{};

  nb.add_markdown_cell("# Results");

  nb.add_code_cell("import numpy as np\n"
                   "import matplotlib.pyplot as plt");

  nb.add_markdown_cell("## Antenna Positions");

  nb.add_code_cell("positions = np.load(\"" + antenna_postition_fname + "\")\n"
                                                                        "print(positions)");

  nb.add_markdown_cell("## Radargram");
  nb.add_code_cell("radargram = np.load(\"" + radargram_fname + "\")\n"
                                                                "print(radargram.shape)");

  nb.add_code_cell("signal = np.abs(np.fft.ifft(radargram*np.hanning(radargram.shape[1]),1024,axis=1)*np.linspace(1,1,1024)**2).T\n"
                   "plt.figure(figsize=(10,10))\n"
                   "abs_amp = np.max(np.abs(signal))\n"
                   "plt.imshow(20*np.log10(signal/abs_amp),origin='upper',aspect='auto',vmin=-125,vmax=0)\n"
                   "\n"
                   "plt.ylim(300, 0)\n"
                   "#plt.xlim(0, 5)\n"
                   "plt.colorbar()");

  std::ofstream output(output_fname.c_str());
  output << nb;
}

#endif//EMRTPO_RADARGRAMM_READER_NOTEBOOK_H
