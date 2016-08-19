#include <cstdlib>
#include <iostream>
#include <fstream>
#include <cassert>

#include <murrayc-suffix-tree/suffix_tree.h>

#include <boost/timer/timer.hpp>

static
void test_simple_single() {
  using Tree = SuffixTree<std::string, std::size_t>;
  Tree suffix_tree;

  suffix_tree.insert("banana", 0);
  suffix_tree.debug_print();

  {
    auto results = suffix_tree.find("band");
    std::cout << "results.size(): " << results.size() << std::endl;
    assert(results.size() == 0);
  }

  {
    auto results = suffix_tree.find("ana");
    std::cout << "results.size(): " << results.size() << std::endl;
    assert(results.size() == 1);
    assert(results == Tree::Candidates({0}));
    for (const auto& result : results) {
      std::cout << result << ": " << std::endl;
    }
  }
}

int main() {
  test_simple_single();

  return EXIT_SUCCESS;
}

