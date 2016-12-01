#include <cstdlib>
#include <iostream>
#include <vector>
#include <cassert>
#include <stack>
#include <utility>
#include <murrayc-suffix-tree/trie.h>

int main() {

  Trie<std::string, int> trie;
  trie.insert("banana", 1);
  trie.insert("bandana", 2);
  trie.insert("foo", 3);
  trie.insert("foobar", 4);

  assert(trie.exists("banana"));
  assert(trie.get_value("banana") == 1);
  assert(trie.exists("foo"));
  assert(trie.get_value("foo") == 3);

  assert(!trie.exists("foop"));
  assert(trie.get_value("foop") == 0);
  assert(!trie.exists("ban"));
  assert(trie.get_value("ban") == 0);

  const auto matches = trie.find_matches("ban");
  //for (const auto match : matches) {
  //  std::cout << "match: " << match << '\n';
  //}

  // TODO: Check wthout caring about the order:
  const auto expected_matches = std::vector<std::string>({"bandana", "banana"});

  assert(matches == expected_matches);

  return EXIT_SUCCESS;
}

