#include <cstdlib>
#include <iostream>
#include <cassert>
#include <murrayc-suffix-tree/radix_tree.h>

/*
void test_has_prefix() {
  using Tree = RadixTree<std::string, int>;

  assert(Tree::has_prefix("banana", 0, "banana"));
  assert(!Tree::has_prefix("banan", 0, "banana"));
  assert(Tree::has_prefix("banana", 0, "banan"));
  assert(Tree::has_prefix("banana", 0, "ban"));
  assert(!Tree::has_prefix("foo", 0, "banana"));
}

void test_common_prefix() {
  using Tree = RadixTree<std::string, int>;
  assert(Tree::common_prefix("banana", 0, "bandana", 0) == 3);
  assert(Tree::common_prefix("banana", 0, "foo", 0) == 0);
  assert(Tree::common_prefix("banana", 0, "banana", 0) == 6);
}
*/

int main() {
  //test_has_prefix();
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

  const auto matches = radix_tree.find_matches("ban");
  // TODO: Check wthout caring about the order:
  const auto expected_matches = Tree::Matches{{"bandana", {2}},
      {"banana", {1, 9}}};
  /*
  for (const auto& match : matches) {
    std::cout << "match: " << match.first << ": ";
    for (const auto& value : match.second) {
      std::cout << value << ", ";
    }
    std::cout << '\n';
  }
  */

  assert(matches == expected_matches);

  return EXIT_SUCCESS;
}

