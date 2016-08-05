#include <cstdlib>
#include <iostream>
#include <cassert>
#include "radix_tree.h"


/*
void test_prefix_matches() {
  using Tree = RadixTree<std::string, int>;

  assert(Tree::prefix_matches("banana", 0, "banana", 0));
  assert(!Tree::prefix_matches("banan", 0, "banana", 0));
  assert(Tree::prefix_matches("banana", 0, "banan", 0));
  assert(Tree::prefix_matches("banana", 0, "ban", 0));
  assert(!Tree::prefix_matches("foo", 0, "banana", 0));
}

void test_common_prefix() {
  using Tree = RadixTree<std::string, int>;
  assert(Tree::common_prefix("banana", 0, "bandana", 0) == 3);
  assert(Tree::common_prefix("banana", 0, "foo", 0) == 0);
  assert(Tree::common_prefix("banana", 0, "banana", 0) == 6);
}
*/

int main() {
  //test_prefix_matches();
  //test_common_prefix();

  using Tree = RadixTree<std::string, int>;
  Tree radix_tree;
  radix_tree.insert("banana", 1);
  radix_tree.insert("banana", 9);
  radix_tree.insert("bandana", 2);
  radix_tree.insert("foo", 3);
  radix_tree.insert("foobar", 4);

  assert(radix_tree.exists("foo"));
  assert(radix_tree.exists("banana"));
  assert(radix_tree.get_value("banana") == 1);
  assert(radix_tree.exists("bandana"));
  assert(radix_tree.get_value("bandana") == 2);
  assert(radix_tree.get_value("foo") == 3);

  assert(!radix_tree.exists("foop"));
  assert(radix_tree.get_value("foop") == 0);
  assert(!radix_tree.exists("ban"));
  assert(radix_tree.get_value("ban") == 0);

  const auto candidates = radix_tree.find_candidates("ban");
  // TODO: Check wthout caring about the order:
  const auto expected_candidates = Tree::Candidates{{"bandana", {2}},
      {"banana", {1, 9}}};
   for (const auto& candidate : candidates) {
    std::cout << "candidate: " << candidate.first << ": ";
    for (const auto& value : candidate.second) {
      std::cout << value << ", ";
    }
    std::cout << '\n';
  }

  assert(candidates == expected_candidates);

  return EXIT_SUCCESS;
}

