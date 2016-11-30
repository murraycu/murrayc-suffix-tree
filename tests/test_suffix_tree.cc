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

  suffix_tree.insert("xyzxyaxyz", 0);

  {
    auto results = suffix_tree.find("bob");
    std::cout << "results.size(): " << results.size() << std::endl;
    assert(results.size() == 0);
  }

  {
    auto results = suffix_tree.find("an");
    std::cout << "results.size(): " << results.size() << std::endl;
    assert(results.size() == 0);
  }

  {
    auto results = suffix_tree.find("zx");
    std::cout << "results.size(): " << results.size() << std::endl;
    assert(results.size() == 1);
    assert(results == Tree::Candidates({0}));
    for (const auto& result : results) {
      std::cout << result << ": " << std::endl;
    }
  }
}

static
void test_simple_multiple() {
  using Tree = SuffixTree<std::string, std::size_t>;
  Tree suffix_tree;

  suffix_tree.insert("banana", 0);
  suffix_tree.insert("bandana", 1);
  suffix_tree.insert("bar", 2);
  suffix_tree.insert("foobar", 3);

  auto results = suffix_tree.find("an");
  std::cout << "results.size(): " << results.size() << std::endl;
  assert(results.size() == 2);
  assert(results == Tree::Candidates({0, 1}));
  for (const auto& result : results) {
    std::cout << result << ": " << std::endl;
  }

  results = suffix_tree.find("bar");
  std::cout << "results.size(): " << results.size() << std::endl;
  assert(results.size() == 2);
  assert(results == Tree::Candidates({2, 3}));
}

static
void test_full_text_index_individual_strings() {
  std::ifstream in;
  in.open("tests/test_pg1400.txt");
  assert(in.is_open());

  // The actual strings are stored outside of the SuffixTree,
  // and must exist for as long as the SuffixTree is used.
  std::vector<std::string> strings;
  while (in) {
    std::string str;
    in >> str;
    strings.emplace_back(str);
  }
  in.close();

  std::cout << "SuffixTree: Construction:" << std::endl;
  boost::timer::auto_cpu_timer timer;
  using Tree = SuffixTree<std::string, std::size_t>;
  Tree suffix_tree;
  std::size_t pos = 0;
  for (const auto& str : strings) {
    suffix_tree.insert(str, pos);
    ++pos;
  }
  timer.stop();
  timer.report();

  std::cout << "SuffixTree: Search:" << std::endl;
  timer.start();
  const auto results = suffix_tree.find("xio");
  timer.stop();
  timer.report();

  assert(results.size() > 10); //TODO: Exact.
  for (const auto& result : results) {
    std::cout << result << ": " << strings[result] << std::endl;
  }
}

static
void test_full_text_index_one_string() {
  // Load the whole text file into one std::string.
  std::string str;
  std::ifstream in;
  in.open("tests/test_pg1400.txt");
  assert(in.is_open());
  in.seekg(0, std::ios::end);
  str.resize(in.tellg());
  in.seekg(0, std::ios::beg);
  in.read(&str[0], str.size());
  in.close();

  std::cout << "SuffixTree: Construction:" << std::endl;
  boost::timer::auto_cpu_timer timer;
  using Tree = SuffixTree<std::string, std::size_t>;
  Tree suffix_tree;

  // The actual strings are stored outside of the SuffixTree,
  // and must exist for as long as the SuffixTree is used.

  // Parse the text to find the words,
  // and add them to the SuffixTree.
  // TODO: Make SuffixTree: Support a T_Key type of const char*.
  const auto start = std::cbegin(str);
  const auto end = std::cend(str);
  auto pos = start;
  auto word_start = pos;
  std::size_t i = 0;
  while (pos < end) {
    if (std::isspace(*pos)) {
      const auto word_end = pos;
      suffix_tree.insert(word_start, word_end, i);
      word_start = word_end + 1;
      ++i;
    }

    ++pos;
  }

  timer.stop();
  timer.report();

  std::cout << "SuffixTree: Search:" << std::endl;
  timer.start();
  const auto results = suffix_tree.find("xio");
  timer.stop();
  timer.report();

  assert(results.size() > 10); //TODO: Exact.
  for (const auto& result : results) {
    std::cout << result << std::endl;
  }
}

static
void test_simple_single_with_positions() {
  using Tree = SuffixTree<std::string, std::size_t>;
  Tree suffix_tree;

  const std::string str = "xyzxyaxyz";
  suffix_tree.insert(str, 0);

  {
    auto results = suffix_tree.find_with_positions("bob");
    std::cout << "results.size(): " << results.size() << std::endl;
    assert(results.size() == 0);
  }

  {
    auto results = suffix_tree.find_with_positions("an");
    std::cout << "results.size(): " << results.size() << std::endl;
    assert(results.size() == 0);
  }

  {
    auto results = suffix_tree.find_with_positions("zx");
    std::cout << "results.size(): " << results.size() << std::endl;
    assert(results.size() == 1);

    const Tree::Range expected_range(std::cbegin(str) + 2, std::cend(str));
    const Tree::CandidatesWithPositions expected = {{expected_range, 0}};
    assert(results == expected);
    for (const auto& result : results) {
      const auto& range = result.first;
      const auto& value = result.second;
      std::cout << std::distance(std::cbegin(str), range.start_) << ": "
        << std::string(range.start_, range.end_) << ": " << value << std::endl;
    }
  }
}

static
void test_simple_multiple_with_positions() {
  using Tree = SuffixTree<std::string, std::size_t>;
  Tree suffix_tree;

  // We keep the strings alive,
  // and just pass a reference,
  // so we can use the iterators that will
  // be returned by find_with_positions()
  const std::string str1 = "banana";
  suffix_tree.insert(str1, 0);
  const std::string str2 = "bandana";
  suffix_tree.insert(str2, 1);
  const std::string str3 = "bar";
  suffix_tree.insert(str3, 2);
  const std::string str4 = "foobar";
  suffix_tree.insert(str4, 3);

  {
    const auto results = suffix_tree.find_with_positions("an");
    std::cout << "results.size(): " << results.size() << std::endl;
    assert(results.size() == 4);
    // TODO: Don't test the order:
    const Tree::CandidatesWithPositions expected = {
      {Tree::Range(std::cbegin(str2) + 1, std::cend(str2)), 1},
      {Tree::Range(std::cbegin(str1) + 3, std::cend(str1)), 0},
      {Tree::Range(std::cbegin(str2) + 4, std::cend(str2)), 1},
      {Tree::Range(std::cbegin(str1) + 1, std::cend(str1)), 0}
    };
    assert(results == expected);

    for (const auto& result : results) {
      const auto& range = result.first;
      const auto& value = result.second;
      //std::cout << std::distance(std::cbegin(str), range.start_) << ": "
      std::cout << std::string(range.start_, range.end_) << ": " << value << std::endl;
    }
  }

  {
    const auto results = suffix_tree.find_with_positions("bar");
    std::cout << "results.size(): " << results.size() << std::endl;
    assert(results.size() == 2);
    const Tree::CandidatesWithPositions expected = {
      {Tree::Range(std::cbegin(str3) + 0, std::cend(str3)), 2},
      {Tree::Range(std::cbegin(str4) + 3, std::cend(str4)), 3}
    };
    assert(results == expected);

    for (const auto& result : results) {
      const auto& range = result.first;
      const auto& value = result.second;
      //std::cout << std::distance(std::cbegin(str), range.start_) << ": "
      std::cout << std::string(range.start_, range.end_) << ": " << value << std::endl;
    }
  }
}

static void
test_get_suffix_array() {
  using Tree = SuffixTree<std::string, std::size_t>;
  Tree suffix_tree;

  // We keep the string alive,
  // and just pass a reference,
  // so we can use the iterators that will
  // be returned by get_suffix_array()
  const std::string str = "bananabanana";
  suffix_tree.insert(str, 0);

  const auto sa_and_lcp = suffix_tree.get_suffix_array_and_lcp_array();
  const auto& sa = sa_and_lcp.first;
  const auto& lcp = sa_and_lcp.second;
  std::cout << "Suffix array size: " << sa.size() << std::endl;
  assert(sa.size() == 12);
  assert(lcp.size() == 11);

  const std::vector<std::size_t> expected_lcp = {1, 1, 3, 3, 5, 0, 6, 0, 2, 2, 4};
  assert(lcp == expected_lcp);

  for (const auto p : sa) {
    const auto& range = p.first;
    const auto& value = p.second;
    std::cout << std::distance(std::cbegin(str), range.start_) << ": "
      << std::string(range.start_, range.end_) << ": " << value << std::endl;
  }

  // Check that these are in lexographic order:
  // The actual comparision in get_suffix_array() is more efficient:
  const bool sorted = std::is_sorted(std::cbegin(sa), std::cend(sa),
    [](const auto& a, const auto& b) {
      const auto& arange = a.first;
      const auto& brange = b.first;
      const auto astr = std::string(arange.start_, arange.end_);
      const auto bstr = std::string(brange.start_, brange.end_);
      return astr < bstr;
    });
  assert(sorted);
}

static void
test_create_from_suffix_array_and_lcp_array() {
  using Tree = SuffixTree<std::string, std::size_t>;
  Tree suffix_tree1;

  // We keep the string alive,
  // and just pass a reference,
  // so we can use the iterators that will
  // be returned by get_suffix_array()
  const std::string str = "xyzxyaxyz";
  suffix_tree1.insert(str, 0);

  const auto sa_and_lcp = suffix_tree1.get_suffix_array_and_lcp_array();
  const auto& sa = sa_and_lcp.first;
  const auto& lcp = sa_and_lcp.second;

  Tree suffix_tree2(sa, lcp);

  {
    auto results = suffix_tree2.find_with_positions("zx");
    std::cout << "results.size(): " << results.size() << std::endl;
    assert(results.size() == 1);

    const Tree::Range expected_range(std::cbegin(str) + 2, std::cend(str));
    const Tree::CandidatesWithPositions expected = {{expected_range, 0}};
    assert(results == expected);
    for (const auto& result : results) {
      const auto& range = result.first;
      const auto& value = result.second;
      std::cout << std::distance(std::cbegin(str), range.start_) << ": "
        << std::string(range.start_, range.end_) << ": " << value << std::endl;
    }
  }
}

int main() {
  test_simple_single();
  test_simple_multiple();

  test_full_text_index_individual_strings();
  test_full_text_index_one_string();

  test_simple_single_with_positions();
  test_simple_multiple_with_positions();

  test_get_suffix_array();
  test_create_from_suffix_array_and_lcp_array();

  return EXIT_SUCCESS;
}

