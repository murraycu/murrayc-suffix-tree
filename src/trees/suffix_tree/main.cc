#include <cstdlib>
#include <iostream>
#include <fstream>
#include <cassert>

#include "suffix_tree.h"

#include <boost/timer/timer.hpp>

static
void test_full_text_index() {
  std::ifstream in;
  in.open("src/trees/suffix_tree/test_pg1400.txt");
  assert(in.is_open());
  std::string str;
  in.seekg(0, std::ios::end);
  str.resize(in.tellg());
  in.seekg(0, std::ios::beg);
  in.read(&str[0], str.size());
  in.close();

  std::cout << "SuffixTree: Construction:" << std::endl;
  boost::timer::auto_cpu_timer timer;
  using Tree = SuffixTree;
  Tree suffix_tree(str.c_str(), str.size());
  timer.stop();
  timer.report();

  std::cout << "SuffixTree: Search:" << std::endl;
  timer.start();
  const auto positions = suffix_tree.find_candidate_values("xio");
  timer.stop();
  timer.report();
  for (const auto position : positions) {
    std::cout << static_cast<const void*>(position.first) << ": " << std::string(position.first, std::distance(position.first, position.second)) << std::endl;
  }
}

int main() {
  test_full_text_index();

  return EXIT_SUCCESS;
}

