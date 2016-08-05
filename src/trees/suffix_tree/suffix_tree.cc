#include <cstdlib>
#include <iostream>
#include <fstream>
#include <vector>
#include <cassert>
#include <stack>
#include <utility>

#include "../radix_tree/radix_tree.h"

template<typename T_Key, typename T_Value>
class SuffixTree {
public:
  void insert(const T_Key& key, const T_Value& value) {
    //Insert every suffix of the key:
    T_Key suffix = key;
    while(!suffix.empty()) {
      //std::cout << "insert(): suffix=" << suffix << ", value=" << value <<std::endl;
      radix_tree_.insert(suffix, value);

      // Remove the first character:
      suffix = suffix.substr(1);
    }
  }

  bool exists(const T_Key& key_suffix) const {
    return radix_tree_.exists(key_suffix);
  }

  /**
   * Returns T_Value() if the key was not found.
   */
  std::set<T_Value> find_candidate_values(const T_Key& substr) const {
    return radix_tree_.find_candidate_values(substr);
  }

private:
  RadixTree<T_Key, T_Value> radix_tree_;
};

static
void test_full_text_index() {
  std::ifstream stream;
  stream.open("src/trees/suffix_tree/test_pg1400.txt");
  assert(stream.is_open());

  using Tree = SuffixTree<std::string, std::size_t>;
  Tree suffix_tree;

  std::size_t pos = 0;
  while(stream) {
    std::string str;
    stream >> str;
    suffix_tree.insert(str, pos);
    ++pos;
  }

  const auto positions = suffix_tree.find_candidate_values("Pip");
  for (const auto position : positions) {
    std::cout << position << ", ";
  }
  std::cout << std::endl;
}

int main() {
  using Tree = SuffixTree<std::string, int>;
  Tree suffix_tree;
  suffix_tree.insert("banana", 1);
  suffix_tree.insert("banana", 9);
  suffix_tree.insert("bandana", 2);
  suffix_tree.insert("foo", 3);
  suffix_tree.insert("foobar", 4);

  //These test the Suffix Tree's knowledge of substrings,
  //instead of just prefixes.
  assert(suffix_tree.exists("oo"));
  assert(suffix_tree.exists("ana"));

  //TODO: Compare without caring about the order:
  const std::set<int> expected_candidates = {1, 9, 2};
  const auto candidates = suffix_tree.find_candidate_values("an");
  assert(candidates == expected_candidates);
/*
  std::cout << "candidates size: " << candidates.size() << std::endl;
  for (const auto& value : candidates) {
    std::cout << "candidate: " << value << '\n';
  }
*/

  test_full_text_index();

  return EXIT_SUCCESS;
}

