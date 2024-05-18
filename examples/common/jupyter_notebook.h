#pragma once
#ifndef JUPYTER_NOTEBOOK_H
#define JUPYTER_NOTEBOOK_H

#include <fstream>
#include <string>
#include <vector>
#include <random>
#include <sstream>
#include <nlohmann/json.hpp>
#include <iostream>

namespace uuid {

static std::random_device random_device;
static std::mt19937 generator(random_device());
static std::uniform_int_distribution<> dis(0, 15);
static std::uniform_int_distribution<> dis2(8, 11);

std::string generate_uuid4() {

  std::stringstream ss;

  ss << std::hex;
  for (int i = 0; i < 8; i++) ss << dis(generator);
  ss << "-";
  for (int i = 0; i < 4; i++) ss << dis(generator);
  ss << "-4";
  for (int i = 0; i < 3; i++) ss << dis(generator);
  ss << "-" << dis2(generator);
  for (int i = 0; i < 3; i++) ss << dis(generator);
  ss << "-";
  for (int i = 0; i < 12; i++) ss << dis(generator);

  return ss.str();
}
}

using json = nlohmann::json;

json create_ipynb_cell() {
  json cell;
  cell["cell_type"] = "code";
  cell["execution_count"] = 0;
  cell["id"] = uuid::generate_uuid4();
  cell["metadata"] = json::object();
  cell["outputs"] = json::array();
  cell["source"] = json();
  return cell;
}

class JupyterNotebook {
public:

  std::vector<json> cells{};

  [[maybe_unused]] void add_raw_cell(const std::string &content) {
    json cell = create_ipynb_cell();
    cell["cell_type"] = "raw";
    cell["source"].push_back(content);
    cells.push_back(cell);
  }

  [[maybe_unused]] void add_code_cell(const std::string &content) {
    json cell = create_ipynb_cell();
    cell["cell_type"] = "code";
    cell["source"].push_back(content);
    cells.push_back(cell);
  }

  [[maybe_unused]] void add_markdown_cell(const std::string &content) {
    json cell = create_ipynb_cell();
    cell["cell_type"] = "markdown";
    cell["source"].push_back(content);
    cells.push_back(cell);
  }

  void save(const std::string &filename);

};


std::ostream &operator<<(std::ostream &stream, const JupyterNotebook &nb) {
  json j;

  j["cells"] = json();
  for (const auto &cell : nb.cells) j["cells"].push_back(cell);

  j["metadata"] = json::object();
  j["nbformat"] = 4;
  j["nbminor"] = 5;

  stream << j.dump(4, ' ', true);
  return stream;
}

void JupyterNotebook::save(const std::string &filename) {
  std::ofstream output(filename.c_str());
  output << *this;
}

#endif //JUPYTER_NOTEBOOK_H
