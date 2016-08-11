#include <cstdlib>
#include <iostream>
#include <fstream>
#include <cassert>

#include <murrayc-suffix-tree/suffix_tree.h>

#include <boost/timer/timer.hpp>

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

  assert(suffix_tree.exists("Pip"));
  assert(!suffix_tree.exists("Pi"));

  std::cout << "SuffixTree: Search:" << std::endl;
  timer.start();
  const auto results = suffix_tree.find_candidate_values("xio");
  timer.stop();
  timer.report();
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

  assert(suffix_tree.exists("Pip"));
  assert(!suffix_tree.exists("Pi"));

  std::cout << "SuffixTree: Search:" << std::endl;
  timer.start();
  const auto results = suffix_tree.find_candidate_values("xio");
  timer.stop();
  timer.report();
  for (const auto& result : results) {
    std::cout << result << std::endl;
  }
}

int main() {
  test_full_text_index_individual_strings();
  test_full_text_index_one_string();

  return EXIT_SUCCESS;
}

