#include <cstdlib>
#include <iostream>
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
      std::cout << "insert(): suffix=" << suffix << std::endl;
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
  std::vector<T_Value> get_values(const T_Key& key_suffix) const {
    return radix_tree_.get_values(key_suffix);
  }

  using Candidates = typename std::vector<T_Value>;

  Candidates find_candidates(const T_Key& substr) const {
    Candidates result;
    const auto keys_and_values = radix_tree_.find_candidates(substr);
    for (const auto& key_and_values : keys_and_values) {
      const auto& values = key_and_values.second;
      result.insert(std::end(result), std::begin(values), std::end(values));
    }

    return result;
  }

private:
  RadixTree<T_Key, T_Value> radix_tree_;
};

int main() {
  using Tree = SuffixTree<std::string, int>;
  Tree suffix_tree;
  suffix_tree.insert("banana", 1);
  suffix_tree.insert("bandana", 2);
  suffix_tree.insert("foo", 3);
  suffix_tree.insert("foobar", 4);

  //These test the Suffix Tree's knowledge of substrings,
  //instead of just prefixes.
  assert(suffix_tree.exists("oo"));

  //TODO: Compare without caring about the order:
  const Tree::Candidates expected_candidates = {3, 4};
  const auto candidates = suffix_tree.find_candidates("oo");
  assert(candidates == expected_candidates);

  /*
  std::cout << "candidates size: " << candidates.size() << std::endl;
  for (const auto& value : candidates) {
    std::cout << "candidate: " << value << '\n';
  }
  */

  return EXIT_SUCCESS;
}

