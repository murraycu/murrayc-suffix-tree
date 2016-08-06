#ifndef MURRAYC_RADIX_TREE_H
#define MURRAYC_RADIX_TREE_H

#include <vector>
#include <stack>
#include <set>
#include <tuple>
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
    //std::cout << "find_candidates(): prefix=" << prefix << std::endl;
    if (prefix.empty()) {
      return {};
    }

    const auto prefix_len = prefix.size();

    std::stack<std::pair<T_Key, const Node*>> stack_result;

    //Stack of prefixes+nodes.
    using Item = std::tuple<std::size_t /* prefix_pos */, T_Key, const Node*>;
    std::stack<Item> stack_starts;
    stack_starts.emplace(0, T_Key(), &root);

    while (!stack_starts.empty()) {
      const auto item = stack_starts.top();
      stack_starts.pop();

      const auto prefix_pos = std::get<0>(item);
      const auto& key = std::get<1>(item);
      const auto node = std::get<2>(item);

      //If we have already used all of the prefix,
      //then use all leaf nodes,
      //because we are just looking for the (candidate) values below an identified intermediate candidate node.
      if (prefix_pos >= prefix_len) { 
        stack_result.emplace(key, node);
        continue;
      }

      for (auto edge : node->children_) {
        const auto& edge_part = edge.part_;
        //std::cout << "  edge: " << edge_part << std::endl;

        const auto child_key = key + edge_part;

        if (has_prefix(prefix, prefix_pos, edge_part, 0)) {
          // The part is a prefix of the remaining key, so follow it:
          stack_starts.emplace(prefix_pos + edge_part.size(), child_key, edge.dest_);
        } else if (has_prefix(edge_part, 0, prefix, prefix_pos)) { 
          // The remaining key is a prefix of the part, so use it as part of candidates:
          stack_starts.emplace(prefix_len, child_key, edge.dest_);
        }
      }
    }

    //Find all descendent leaves:
    Candidates result;
    while(!stack_result.empty()) {
      const auto item = stack_result.top();
      stack_result.pop();

      const auto& key = item.first;
      const auto node = item.second; 

      //Use it if it is a leaf node:
      if (node->has_value()) {
        result.emplace_back(key, node->values_);
      }

      for (auto edge : node->children_) {
        stack_result.emplace(key + edge.part_, edge.dest_);
      }
    }

    return result;
  }

  /**
   * Returns T_Value() if the key was not found.
   */
  std::set<T_Value> find_candidate_values(const T_Key& prefix) const {
    std::set<T_Value> result;

    //std::cout << "find_candidates(): prefix=" << prefix << std::endl;
    if (prefix.empty()) {
      return result;
    }

    const auto prefix_len = prefix.size();

    // Find all the nodes whose descendants (including themselves) should be in the result,
    // putting these in a second stack.
    using Item = std::pair<std::size_t /* prefix_pos */, const Node*>;
    std::stack<Item> stack;
    stack.emplace(0, &root);

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
          stack.emplace(prefix_pos + edge_part.size(), edge.dest_);
        } else if (has_prefix(edge_part, 0, prefix, prefix_pos)) { 
          // The remaining key is a prefix of the part, so it is a candidate:
          stack.emplace(prefix_len, edge.dest_);
        }
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
          const auto prefix = part.substr(0, prefix_len);
          //std::cout << "  splitting part=" << part << ", at key prefix: " << key.substr(0, key_pos + 1) << ", with prefix=" << prefix << ", values size: " << dest->values_.size() << std::endl;
          const auto suffix_part = part.substr(prefix_len);
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

public:
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

  const Node* find_node(const T_Key& key) const {
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
    return node->has_value() ? node : nullptr;
  }

  Node root;
};

#endif // MURRAYC_RADIX_TREE_H
