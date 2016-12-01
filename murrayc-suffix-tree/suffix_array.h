#ifndef MURRAYC_SUFFIX_TREE_SUFFIX_ARRAY_H
#define MURRAYC_SUFFIX_TREE_SUFFIX_ARRAY_H

#include "iter_range.h"
#include <algorithm>
#include <iostream>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <set>
#include <stack>

/**
 * @tparam T_Key For instance, std::string, or something other container.
 * @tparam T_Value The value to associate with each inserted key.
 */
template <typename T_Key, typename T_Value>
class SuffixArray {
public:
  SuffixArray() {
  }

  SuffixArray(const T_Key& key, const T_Value& value) {
    const auto start = std::cbegin(key);
    const auto end = start + key.size();
    const Range substr(start, end);
    if (str_empty(substr)) {
      return;
    }

    // TODO: Implement DC3 algorithm (linear),
    // instead of this naive O(n log(n)) algorithm:
    const auto n = substr.size();
    suffixes_.reserve(n);
    for (auto i = start; i != end; ++i) {
      Range suffix(i, end); 
      suffixes_.emplace_back(suffix, value);
    }

    std::sort(std::begin(suffixes_), std::end(suffixes_),
      [](const auto& a, const auto& b) {
        return a.first < b.first;
      });

    // Build LCP array:
    lcp_array_.reserve(n - 1);
    Range prev = suffixes_[0].first;
    for (auto i = std::cbegin(suffixes_) + 1; i < std::cend(suffixes_); ++i) {
      const auto& suffix = i->first; 
      const auto lcp = prev.common_prefix(0, suffix);
      lcp_array_.emplace_back(lcp);
      prev = suffix;
    }
  }

  /// Start and end (1 past last position) of a substring in text_;
  using KeyIterator = typename T_Key::const_iterator;

  using Range = IterRange<KeyIterator>;

  /**
   * The suffix's begin/end, and the associated value.
   */
  using suffix_array_type = std::vector<std::pair<Range, T_Value>>;

  using lcp_array_type = std::vector<std::size_t>;

  SuffixArray(const suffix_array_type& suffixes, const lcp_array_type& lcp_array)
  : suffixes_(suffixes), lcp_array_(lcp_array) {
  }

  using Matches = std::set<T_Value>;

  /** Finds the values for any key containing this substring.
   */
  Matches find(const T_Key& substr) const {
    Matches result;

    if (substr.empty()) {
      return result;
    }


    const auto start = std::cbegin(substr);
    const auto end = start + substr.size();
    const Range substr_key(start, end);

    const auto matches_with_positions = find_with_positions(substr_key);

    // Convert one container into another:
    for (const auto& kv : matches_with_positions) {
      result.emplace(kv.second);
    }

    return result;
  }

  /**
   * Like Matches, but provides the range of the prefix too,
   * so the caller can know where in the original insert()ed string,
   * the substring was found. That would refer to the originally-inserted
   * string, but we already require the caller to keep that alive.
   */
  using MatchesWithPositions = std::vector<std::pair<Range, T_Value>>;

  /** Finds the values for any key containing this substring.
   */
  MatchesWithPositions find_with_positions(const T_Key& substr) const {
    // TODO: This uses binary search (via std::lower_bound() and std::upper_bound()),
    // to find the matching suffixes, but that is O(log(n)). A SuffixTree could
    // do this in O(m), where m is the length of the pattern being searched.
    // We can apparently get that too by using the LCP array:
    // https://en.wikipedia.org/wiki/Suffix_array#Applications
    MatchesWithPositions result;

    if (substr.empty()) {
      return result;
    }


    const auto start = std::cbegin(substr);
    const auto end = start + substr.size();
    const Range substr_key(start, end);
    return find_with_positions(substr_key);
  }

  /** Finds the values for any key containing this substring.
   */
  MatchesWithPositions find_with_positions(const Range& substr) const {
    MatchesWithPositions result;

    if (str_empty(substr)) {
      return result;
    }

    const auto sz = substr.size();

    // Find the first matching suffix:
    const auto range_start = std::lower_bound(std::cbegin(suffixes_), std::cend(suffixes_), substr,
      [sz](const auto& a, const auto& b) {  
        const auto& a_start = a.first.substr(0, sz);
        return a_start < b; 
      });
    if (range_start == std::cend(suffixes_)) {
      return result;
    }

    // Find 1 past the last matching suffix:
    const auto range_end = std::upper_bound(range_start, std::cend(suffixes_), substr,
      [sz](const auto& a, const auto& b) {  
        const auto& b_start = b.first.substr(0, sz);
        return a < b_start; 
      });

    for (auto i = range_start; i < range_end; ++i) {
      result.emplace_back(*i);
    }

    return result;
  }

private:
  static
  inline std::size_t str_size(const Range& key) {
    return key.size();
  }

  static
  inline bool str_empty(const Range& key) {
    return key.empty();
  }

  static
  bool has_prefix(const Range& str, std::size_t str_start_pos, const Range& prefix, std::size_t prefix_start_pos = 0) {
    const auto prefix_start = prefix.start_ + prefix_start_pos;
    const auto prefix_end = str_end(prefix);
    const auto iters = std::mismatch(str.start_ + str_start_pos, str_end(str),
        prefix_start, prefix_end);
    return iters.second == prefix_end;
  }

  suffix_array_type suffixes_;
  lcp_array_type lcp_array_;
};

#endif // MURRAYC_SUFFIX_TREE_SUFFIX_ARRAY_H
