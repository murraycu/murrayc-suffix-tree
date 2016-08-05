#ifndef MURRAYC_RADIX_TREE_H
#define MURRAYC_RADIX_TREE_H

#include <vector>
#include <stack>
#include <utility>

template <typename T_Key, typename T_Value>
class RadixTree {
public:
  bool exists(const T_Key& key) const {
    const auto node = find_node(key);
    return node != nullptr;
  };

  /** Get the first value associates with the key.
   *
   * Returns T_Value() if the key was not found.
   */
  T_Value get_value(const T_Key& key) const {
    const auto node = find_node(key);
    if (!node) {
      return T_Value();
    }

    if (node->values_.empty()) {
      return T_Value();
    }

    return node->values_[0];
  }

  /** Get the first value associates with the key.
   *
   * Returns T_Value() if the key was not found.
   */
  std::vector<T_Value> get_values(const T_Key& key) const {
    const auto node = find_node(key);
    if (!node) {
      return {};
    }

    return node->values_;
  }

  using Candidates = std::vector<std::pair<T_Key, std::vector<T_Value>>>;
  Candidates find_candidates(const T_Key& prefix) const {
    std::cout << "find_candidates(): prefix=" << prefix << std::endl;
    if (prefix.empty()) {
      return {};
    }

    const auto prefix_node = find_node(prefix, false /* not just leaves */);
    if (!prefix_node) {
      return {};
    }

    std::cout << "  prefix_node: part=" << prefix_node << ", values size:" << prefix_node->values_.size() << std::endl;

    Candidates result;

    //Stack of prefixes+nodes.
    std::stack<std::pair<T_Key, const Node*>> stack;
    stack.emplace(prefix, prefix_node);

    while (!stack.empty()) {
      const auto item = stack.top();
      stack.pop();

      const auto& node = item.second;
      if (node->has_value()) {
        result.emplace_back(item.first, item.second->values_);
      }

      for (auto edge : node->children_) {
        std::cout << "  edge: " << edge.part_ << std::endl;
        stack.emplace(item.first + edge.part_, edge.dest_);
      }
    }

    return result;
  }

  void insert(const T_Key& key, const T_Value& value) {
    // std::cout << "insert(): key=" << key << std::endl;
    if (key.empty()) {
      return;
    }

    auto node = &root;
    std::size_t key_pos = 0;
    const auto key_size = key.size();
    while (key_pos < key_size) {
      //std::cout << "insert(): remaining key=" << key.substr(key_pos) << std::endl;
      //Choose the child node, if any:
      Node* next = nullptr;
      for (auto& edge : node->children_) {
        const auto& part = edge.part_;

        const auto prefix_len = common_prefix(part, 0, key, key_pos);
        const auto part_len = part.size();
        //std::cout << "key=" << key << ", key_pos=" << key_pos << ", part=" << part << "\n";
        //If the edge's part is a prefix of the remaining key:
        if (prefix_len == 0) {
          // No match.
          continue;
        } else if (prefix_len < part_len) {
          // If the key is a prefix of the edge's part:

          // Split it,
          // adding a new intermediate node in it original node's place, with the original node as a child.
          const auto prefix = key.substr(0, prefix_len);
          //std::cout << "  splitting with prefix=" << prefix << std::endl;
          const auto suffix_part = part.substr(prefix_len);
          //std::cout << "    suffix_part=" << suffix_part << std::endl;

          const auto dest = edge.dest_;

          auto extra_node = new Node;
          extra_node->children_.emplace_back(suffix_part, dest);

          edge.part_ = prefix;
          edge.dest_ = extra_node;

          next = edge.dest_;
          key_pos += prefix_len;
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
    const auto suffix = key.substr(key_pos);
    //std::cout << "Adding suffix: " << suffix << ", with value: " << value << '\n';

    const auto next = new Node;
    node->children_.emplace_back(suffix, next);

    next->values_.emplace_back(value);
    //std::cout << "next: " << next << std::endl;
  }

  void remove(const T_Key& key) {
    auto node = find_node(key);
    if (!node) {
      return;
    }

    //TODO: Each node needs a pointer to its parent.
    //We then need to ask its parent to delete its edge to this node,
    //and then combine itself with its own parent if it now has only one edge.
  }

private:
  class Node {
  public:
    class Edge {
    public:
      Edge(const T_Key& part, Node* dest)
        : part_(part),
          dest_(dest) {
      }

      Edge(const Edge& src) = default;
      Edge& operator=(const Edge& src) = default;
      Edge(Edge&& src) = default;
      Edge& operator=(Edge&& src) = default;

      T_Key part_ = T_Key();
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
    std::vector<T_Value> values_;
  };

  /** Returns the number of characters at the end of the prefix that do not match the @a str from position
   * @a str_start_pos.
   *
   * @param match Whether @a str has the @a prefix.
   */
  static
  bool has_prefix(const std::string& str, std::size_t str_start_pos, const std::string& prefix, std::size_t prefix_start_pos = 0) {
    const auto prefix_start = std::begin(prefix) + prefix_start_pos;
    const auto prefix_end = std::end(prefix);
    const auto iters = std::mismatch(std:: begin(str) + str_start_pos, std::end(str),
        prefix_start, prefix_end);
    return iters.second == prefix_end;

    /*
    const std::size_t prefix_start_pos = 0;
    const auto str_len = str.size() - str_start_pos;
    const auto prefix_len = prefix.size() - prefix_start_pos;

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
  std::size_t common_prefix(const std::string& str, std::size_t str_start_pos, const std::string& prefix, std::size_t prefix_start_pos) {
    //TODO: Use std::mismatch().
    const auto str_start = std::begin(str) + str_start_pos;
    const auto iters = std::mismatch(str_start, std::end(str),
        std::begin(prefix) + prefix_start_pos, std::end(prefix));
    return std::distance(str_start, iters.first);

    /*
    const auto str_len = str.size() - str_start_pos;
    const auto prefix_len = prefix.size() - prefix_start_pos;

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

  const Node* find_node(const T_Key& key, bool leaf_only = true) const {
    //std::cout << "find_node(): key=" << key << std::endl;
    if (key.empty()) {
      return nullptr;
    }

    auto node = &root;
    std::size_t key_pos = 0;
    const auto key_size = key.size();
    while (key_pos < key_size) {
      //std::cout << "find_node(): remaining key=" << key.substr(key_pos) << std::endl;
      //std::cout << "  children_ size: " << node->children_.size() << std::endl;
      //Choose the child node, if any:
      Node* next = nullptr;
      for (const auto& edge : node->children_) {
        const auto& part = edge.part_;
        const auto part_size = part.size();
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
    return (!leaf_only || node->has_value()) ? node : nullptr;
  }

  Node root;
};

#endif // MURRAYC_RADIX_TREE_H
