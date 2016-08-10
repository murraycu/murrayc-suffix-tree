#ifndef MURRAYC_SUFFIX_TREE_H
#define MURRAYC_SUFFIX_TREE_H

#include <iostream>
#include <vector>
#include <set>
#include <stack>

template <typename T_Value>
class SuffixTree {
public:
  using T_Key = std::string;

  /// Start and end (1 past last position) of a substring in text_;
  using T_Key_Internal = std::pair<const char*, const char*>;

  using T_Value_Internal = std::pair<T_Key_Internal, T_Value>;

  SuffixTree() {
  }

  void insert(const T_Key& key, const T_Value& value) {
    const auto start = key.c_str();
    const auto end = start + key.size();
    const auto substr = std::make_pair(start, end);
    insert(substr, std::make_pair(substr, value));
  }

  bool exists(const T_Key& key) const {
    const auto node = find_node(key);
    return node != nullptr;
  };

  /** Finds the values for any key contains this substring.
   */
  std::set<T_Value_Internal> find_candidate_values(const T_Key& prefix) const {
    std::set<T_Value_Internal> result;

    //std::cout << "find_candidates(): prefix=" << prefix << std::endl;
    if (prefix.empty()) {
      return result;
    }

    const auto prefix_len = prefix.size();

    // Find all the nodes whose descendants (including themselves) should be in the result,
    // putting these in a second stack.
    using Item = std::pair<std::size_t /* prefix_pos */, const Node*>;
    std::stack<Item> stack;
    stack.emplace(0, &root_);

    while (!stack.empty()) {
      const auto item = stack.top();
      stack.pop();

      const auto prefix_pos = item.first;
      const auto node = item.second;

      //If we have already used all of the prefix,
      //then use all subsequent leaf nodes.
      if (prefix_pos >= prefix_len) {
        if (node->has_value()) {
          result.insert(std::begin(node->values_), std::end(node->values_));
        }
      }

      for (auto edge : node->children_) {
        const auto& edge_part = edge.part_;

        if (has_prefix(prefix, prefix_pos, edge_part, 0)) {
          // The part is a prefix of the remaining key, so follow it:
          stack.emplace(prefix_pos + str_size(edge_part), edge.dest_);
        } else if (has_prefix(edge_part, 0, prefix, prefix_pos)) {
          // The remaining key is a prefix of the part, so it is a candidate:
          stack.emplace(prefix_len, edge.dest_);
        }
      }
    }

    return result;
  }

private:

  class Node {
  public:
    class Edge {
    public:
      Edge(const T_Key_Internal& part, Node* dest)
        : part_(part),
          dest_(dest) {
      }

      Edge(const Edge& src) = default;
      Edge& operator=(const Edge& src) = default;
      Edge(Edge&& src) = default;
      Edge& operator=(Edge&& src) = default;

      T_Key_Internal part_;
      Node* dest_ = nullptr;
    };

    inline bool has_value() const {
      return !values_.empty();
    }

    //We could instead have a std::vector<Node*> children_,
    //of size alphabet (such as 26),
    //to allow O(1) lookup, at the cost of wasted space.
    std::vector<Edge> children_;

    // TODO: Wastes space on non-leaves.
    std::vector<T_Value_Internal> values_;
  };

  void insert(const T_Key_Internal& key, const T_Value_Internal& value) {
    //std::cout << "debug: insert(): key.first=" << static_cast<const void*>(key.first) << ", second=" << static_cast<const void*>(key.second) << std::endl;
    //Insert every suffix of the key:
    T_Key_Internal suffix = key;
    while(!str_empty(suffix)) {
      //std::cout << "insert(): suffix=" << suffix << ", value=" << value <<std::endl;
      insert_single(suffix, value);

      // Remove the first character:
      suffix = str_substr(suffix, 1);
      //std::cout << "suffix: first=" << static_cast<const void*>(suffix.first) << ", second=" << static_cast<const void*>(suffix.second) << std::endl;
    }
  }

  static
  bool has_prefix(const T_Key& str, std::size_t str_start_pos, const T_Key_Internal& prefix, std::size_t prefix_start_pos = 0) {
    const auto prefix_start = prefix.first + prefix_start_pos;
    const auto prefix_end = prefix.second;
    const auto iters = std::mismatch(std:: begin(str) + str_start_pos, std::end(str),
        prefix_start, prefix_end);
    return iters.second == prefix_end;
  }

  static
  bool has_prefix(const T_Key_Internal& str, std::size_t str_start_pos, const T_Key& prefix, std::size_t prefix_start_pos = 0) {
    const auto prefix_start = std::begin(prefix) + prefix_start_pos;
    const auto prefix_end = std::end(prefix);
    const auto iters = std::mismatch(str.first + str_start_pos, str.second,
        prefix_start, prefix_end);
    return iters.second == prefix_end;

    /*
    const std::size_t prefix_start_pos = 0;
    const auto str_len = str_size(str) - str_start_pos;
    const auto prefix_len = str_size(prefix.size()) - prefix_start_pos;

    // prefix cannot be a prefix of str if it is longer than str:
    if (prefix_len > str_len) {
      return false;
    }

    const auto len = std::min(str_len, prefix_len);
    const auto end = str_start_pos + len;

    auto prefix_iter = std::begin(prefix) + prefix_start_pos;
    std::size_t i = str_start_pos;
    for (; i < end; ++i) {
      if (str[i] != *prefix_iter) {
        // Some part of the prefix doesn't match:
        return false;
      }

      ++prefix_iter;
    }

    //All characters in prefix match at the start of str:
    return true;
    */
  }

  static
  std::size_t common_prefix(const T_Key_Internal& str, std::size_t str_start_pos, const T_Key_Internal& prefix, std::size_t prefix_start_pos) {
    //TODO: Use std::mismatch().
    const auto str_start = str.first + str_start_pos;
    const auto iters = std::mismatch(str_start, str.second,
        prefix.first + prefix_start_pos, prefix.second);
    return std::distance(str_start, iters.first);

    /*
    const auto str_len = str_size(str.size()) - str_start_pos;
    const auto prefix_len = str_size(prefix.size()) - prefix_start_pos;

    const auto len = std::min(str_len, prefix_len);
    const auto str_end = str_start_pos + len;

    std::size_t i_str = str_start_pos;
    std::size_t i_prefix = prefix_start_pos;
    while(i_str < str_end) {
      if(str[i_str] != prefix[i_prefix]) {
        break;
      }

      ++i_str;
      ++i_prefix;
    }

    return i_str - str_start_pos;
    */
  }

  /*
  static std::string debug_key(const T_Key_Internal& key) {
    if (key.first == nullptr) {
      return std::string();
    }

    if (key.second <= key.first) {
      return std::string();
    }

    const auto len = std::distance(key.first, key.second);
    return debug_key(key.first, len);
  }

  static std::string debug_key(const char* first, std::size_t len) {
    if (first == nullptr) {
      return std::string();
    }

    if (len == 0) {
      return std::string();
    }

    return std::string(first, len); 
  }
  */

  void insert_single(const T_Key_Internal& key, const T_Value_Internal& value) {
    //std::cout << "insert(): key=" << debug_key(key) << std::endl;
    if (str_empty(key)) {
      return;
    }

    auto node = &root_;
    std::size_t key_pos = 0;
    const auto key_size = str_size(key);
    //std::cout << "debug: insert_single(): key_size=" << key_size << std::endl;
    while (key_pos < key_size) {
      //std::cout << "debug: insert(): remaining key=" << key_pos << std::endl;
      //std::cout << "  debug: node=" << node << std::endl;
      //Choose the child node, if any:
      Node* next = nullptr;
      for (auto& edge : node->children_) {
        const auto& part = edge.part_;

        const auto prefix_len = common_prefix(part, 0, key, key_pos);
        const auto part_len = str_size(part);
        //std::cout << "key=" << debug_key(key) << ", key_pos=" << key_pos << ", part=" << debug_key(part) <<
        //  ", prefix_len=" << prefix_len << ", part_len=" << part_len << "\n";
        //If the edge's part is a prefix of the remaining key:
        if (prefix_len == 0) {
          // No match.
          continue;
        } else if (prefix_len < part_len) {
          // If the key is a prefix of the edge's part:

          // Split it,
          // adding a new intermediate node in it original node's place, with the original node as a child.
          const auto prefix = str_substr(part, 0, prefix_len);
          assert(str_size(prefix) == prefix_len);
          //std::cout << "  splitting part=" << debug_key(part) << ", at key prefix: " << debug_key(str_substr(key, 0, key_pos + 1)) <<
          //  ", with prefix=" << debug_key(prefix) << std::endl;
          const auto suffix_part = str_substr(part, prefix_len);
          //assert(part == (prefix + suffix_part));
          //std::cout << "    suffix_part=" << suffix_part << std::endl;

          const auto dest = edge.dest_;

          auto extra_node = new Node;
          extra_node->children_.emplace_back(suffix_part, dest);

          edge.part_ = prefix;
          edge.dest_ = extra_node;

          // Try the same node again.
          // This time it might be a perfect match.
          next = node;
          break;
        } else {
          next = edge.dest_;
          key_pos += part_len;
          break;
        }
      }

      // Stop when we cannot go further.
      if (!next) {
        break;
      }

      node = next;
    }

    if (key_pos > key_size) {
      std::cerr << "Unexpected key_pos.\n";
      return;
    }

    if (key_pos == key_size) {
      //The node already exists, so just add the extra value:
      node->values_.emplace_back(value);
      return;
    }

    // Add a node for the remaining characters:
    const auto suffix = str_substr(key, key_pos);
    //std::cout << "Adding suffix: " << suffix << ", with value: " << value << '\n';

    const auto next = new Node;
    node->children_.emplace_back(suffix, next);

    next->values_.emplace_back(value);
    //std::cout << "next: " << next << std::endl;
  }

  const Node* find_node(const T_Key& key) const {
    //std::cout << "find_node(): key=" << key << std::endl;
    if (key.empty()) {
      return nullptr;
    }

    auto node = &root_;
    std::size_t key_pos = 0;
    const auto key_size = key.size();
    while (key_pos < key_size) {
      //std::cout << "find_node(): remaining key=" << str_substr(key, key_pos) << std::endl;
      //std::cout << "  children_ size: " << node->children_.size() << std::endl;
      //Choose the child node, if any:
      Node* next = nullptr;
      for (const auto& edge : node->children_) {
        const auto& part = edge.part_;
        const auto part_size = str_size(part);
        //std::cout << "  key=" << key << ", key_pos=" << key_pos << ", part=" << part << "\n";
        if(!has_prefix(key, key_pos, part)) {
          continue;
        }

        next = edge.dest_;
        key_pos += part_size;
        //std::cout << "    next: " << next << std::endl;
        break;
      }

      if (!next) {
        return nullptr;
      }

      node = next;
    }

    if (key_pos < key_size) {
      //We didn't find all the parts of the prefix:
      return nullptr;
    }

    //std::cout << "node: " << node << std::endl;
    return node->has_value() ? node : nullptr;
  }

  static
  inline std::size_t str_size(const T_Key_Internal& key) {
    if (key.second <= key.first) {
      return 0;
    }

    return key.second - key.first;
  }

  static
  inline bool str_empty(const T_Key_Internal& key) {
    return key.first >= key.second;
  }

  static
  inline T_Key_Internal str_substr(const T_Key_Internal& key, std::size_t start) {
    const auto start_used = key.first + start;
    return std::make_pair(
      (start_used < key.second) ? start_used : key.second,
      key.second);
  }

  static
  inline T_Key_Internal str_substr(const T_Key_Internal& key, std::size_t start, std::size_t len) {
    const auto start_used = key.first + start;
    const auto end_used = key.first + len;
    return std::make_pair(
      (start_used < key.second) ? start_used : key.second,
      (end_used < key.second) ? end_used : key.second);
  }

  Node root_;
};

#endif // MURRAYC_SUFFIX_TREE_H
