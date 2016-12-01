#include <cstdlib>
#include <iostream>
#include <fstream>
#include <cassert>

#include <murrayc-suffix-tree/suffix_array.h>

#include <boost/timer/timer.hpp>

static
bool starts_with(const std::string& a, const std::string& b) {
  return a.compare(0, b.size(), b) == 0;
}

static
void test_simple_single() {
  using SA = SuffixArray<std::string, std::size_t>;

  const std::string str = "banana";
  const std::size_t value = 0;

  const SA::suffix_array_type sa = {
    std::make_pair(SA::Range(std::cbegin(str) + 5, std::cend(str)), value),
    std::make_pair(SA::Range(std::cbegin(str) + 3, std::cend(str)), value),
    std::make_pair(SA::Range(std::cbegin(str) + 1, std::cend(str)), value),
    std::make_pair(SA::Range(std::cbegin(str) + 0, std::cend(str)), value),
    std::make_pair(SA::Range(std::cbegin(str) + 4, std::cend(str)), value),
    std::make_pair(SA::Range(std::cbegin(str) + 2, std::cend(str)), value)};
  const SA::lcp_array_type lcp_array = {1, 3, 0, 0, 2};

  SA suffix_array(sa, lcp_array);

  {
    auto results = suffix_array.find("bob");
    std::cout << "results.size(): " << results.size() << std::endl;
    assert(results.size() == 0);
  }

  {
    auto results = suffix_array.find("an");
    std::cout << "results.size(): " << results.size() << std::endl;
    assert(results.size() == 1);
    for (const auto& result : results) {
      std::cout << result << std::endl;
    }
  }
}

static
void test_simple_single_with_positions() {
  using SA = SuffixArray<std::string, std::size_t>;

  const std::string str = "xyzxyaxyz";
  SA suffix_array(str, 0);

  {
    auto results = suffix_array.find_with_positions("bob");
    std::cout << "results.size(): " << results.size() << std::endl;
    assert(results.size() == 0);
  }

  {
    auto results = suffix_array.find_with_positions("an");
    std::cout << "results.size(): " << results.size() << std::endl;
    assert(results.size() == 0);
  }

  {
    auto results = suffix_array.find_with_positions("zx");
    std::cout << "results.size(): " << results.size() << std::endl;
    assert(results.size() == 1);

    const SA::Range expected_range(std::cbegin(str) + 2, std::cend(str));
    const SA::CandidatesWithPositions expected = {{expected_range, 0}};
    assert(results == expected);
    for (const auto& result : results) {
      const auto& range = result.first;
      const auto& value = result.second;
      std::cout << std::distance(std::cbegin(str), range.start_) << ": "
        << std::string(range.start_, range.end_) << ": " << value << std::endl;
    }
  }

  {
    const std::string KEY = "xy";
    auto results = suffix_array.find_with_positions(KEY);
    std::cout << "results.size(): " << results.size() << std::endl;
    assert(results.size() == 3);
    //TODO: Don't check the order:
    const SA::CandidatesWithPositions expected = {
      {SA::Range(std::cbegin(str) + 3, std::cend(str)), 0},
      {SA::Range(std::cbegin(str) + 6, std::cend(str)), 0},
      {SA::Range(std::cbegin(str) + 0, std::cend(str)), 0}
    };
    for (const auto& result : results) {
      const auto& range = result.first;
      const auto& value = result.second;
      const auto result_str = range.to_string();

      std::cout << std::distance(std::cbegin(str), range.start_) << ": "
        << result_str << ": " << value << std::endl;
      assert(starts_with(result_str, KEY));
    }
    assert(results == expected);
  }
}

int main() {
  test_simple_single();
  test_simple_single_with_positions();

  return EXIT_SUCCESS;
}

