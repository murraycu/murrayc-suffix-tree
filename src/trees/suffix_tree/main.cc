#include <cstdlib>
#include <iostream>
#include <list>
#include <fstream>
#include <cassert>

#include "suffix_tree.h"

#include <boost/timer/timer.hpp>

static
void test_full_text_index() {
  std::ifstream in;
  in.open("src/trees/suffix_tree/test_pg1400.txt");
  assert(in.is_open());

  std::cout << "SuffixTree: Construction:" << std::endl;
  boost::timer::auto_cpu_timer timer;
  using Tree = SuffixTree<std::size_t>;
  Tree suffix_tree;
  
  // The actual strings are stored outside of the SuffixTree,
  // and must exist for as long as the SuffixTree is used.
  // A vector would be more efficient,
  // but we would need to know the size in advance,
  // to avoid invalidating the references after emplace_back().
  std::list<std::string> strings;

  std::size_t pos = 0;
  while(in) {
    std::string str;
    in >> str;
    strings.emplace_back(str);

    // The reference will stay valid becaue this is a std::list,
    // not a std::vector.
    const auto& word = strings.back();
    suffix_tree.insert(word, pos);
    ++pos;
  }
  timer.stop();
  timer.report();

  std::cout << "SuffixTree: Search:" << std::endl;
  timer.start();
  const auto results = suffix_tree.find_candidate_values("xio");
  timer.stop();
  timer.report();
  for (const auto& result : results) {
    const auto& position = result.first;
    std::cout << static_cast<const void*>(position.first) << ": " << std::string(position.first, std::distance(position.first, position.second)) << std::endl;
  }
}

int main() {
  test_full_text_index();

  return EXIT_SUCCESS;
}

