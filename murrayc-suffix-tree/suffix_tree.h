#ifndef MURRAYC_SUFFIX_TREE_SUFFIX_TREE_H
#define MURRAYC_SUFFIX_TREE_SUFFIX_TREE_H

#include <iostream>
#include <vector>
#include <set>
#include <stack>

/**
 * @tparam T_Key For instance, std::string, or something other container.
 * @tparam T_Value The value to associate with each inserted key.
 */
template <typename T_Key, typename T_Value>
class SuffixTree {
public:
  SuffixTree() {
  }

  void insert(const T_Key& key, const T_Value& value) {
    const auto start = std::cbegin(key);
    const auto end = start + key.size();
    const auto substr = std::make_pair(start, end);
    insert(substr, value);
  }

  void insert(const typename T_Key::const_iterator& start, const typename T_Key::const_iterator& end, const T_Value& value) {
    const auto substr = std::make_pair(start, end);
    insert(substr, value);
  }


  using Candidates = std::set<T_Value>;

  /** Finds the values for any key containing this substring.
   */
  Candidates find_candidate_values(const T_Key& substr) const {
    Candidates result;

    if (substr.empty()) {
      return result;
    }


    const auto start = std::cbegin(substr);
    const auto end = start + substr.size();
    const auto substr_key = std::make_pair(start, end);
    return find_candidate_values(substr_key);
  }

private:

  /// Start and end (1 past last position) of a substring in text_;
  using KeyIterator = typename T_Key::const_iterator;
  using T_Key_Internal = std::pair<KeyIterator, KeyIterator>;

  class Node {
  public:
    Node() = default;
    Node(const Node& src) = default;
    Node& operator=(const Node& src) = default;
    Node(Node&& src) = default;
    Node& operator=(Node&& src) = default;

    ~Node() {
      for(auto edge : children_) {
        delete edge.dest_;
      }
    }

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

      /** This inserts an intermediate node by splitting the edge's part at
       * position @pos.
       * @result The new intermediate node.
       */
      Node* split(std::size_t part_pos) {
        const auto prefix_part = str_substr(part_, 0, part_pos);
        const auto suffix_part = str_substr(part_, part_pos);
        const auto dest = dest_;

        auto extra_node = new Node;
        extra_node->append_node(suffix_part, dest);

        part_ = prefix_part;
        dest_ = extra_node;

        return extra_node;
      }

      T_Key_Internal part_;
      Node* dest_ = nullptr;
    };

    void append_node(const T_Key_Internal& part, const T_Value& value) {
      const auto extra_node = new Node();
      extra_node->values_.emplace_back(value);
      children_.emplace_back(part, extra_node);
    }

    void append_node(const T_Key_Internal& part, Node* node) {
      children_.emplace_back(part, node);
    }

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

  void insert(const T_Key_Internal& key, const T_Value& value) {
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

  /** Finds the values for any key containing this substring.
   */
  std::set<T_Value> find_candidate_values(const T_Key_Internal& substr) const {
    std::set<T_Value> result;

    if (str_empty(substr)) {
      return result;
    }
    const auto substr_len = str_size(substr);

    using Item = std::pair<std::size_t /* substr_pos */, const Node*>;
    std::stack<Item> stack;
    stack.emplace(0, &root_);

    while (!stack.empty()) {
      const auto item = stack.top();
      stack.pop();

      const auto substr_pos = item.first;
      const auto node = item.second;

      //If we have already used all of the substring,
      //then use all subsequent leaf nodes.
      if (substr_pos >= substr_len) {
        if (node->has_value()) {
          result.insert(std::cbegin(node->values_), std::cend(node->values_));

          //And continue to examine children, because they can have values too.
        }
      }

      for (auto edge : node->children_) {
        const auto& edge_part = edge.part_;

        if (has_prefix(substr, substr_pos, edge_part, 0)) {
          // The whole part is a prefix of the remaining substring, so follow it:
          stack.emplace(substr_pos + str_size(edge_part), edge.dest_);
        } else if (has_prefix(edge_part, 0, substr, substr_pos)) {
          // The whole remaining substr is a prefix of the part, so it is a candidate:
          // We will then use the value because substr_pos==substr_len.
          stack.emplace(substr_len, edge.dest_);
        }
      }
    }

    return result;
  }

  static
  bool has_prefix(const T_Key_Internal& str, std::size_t str_start_pos, const T_Key_Internal& prefix, std::size_t prefix_start_pos = 0) {
    const auto prefix_start = prefix.first + prefix_start_pos;
    const auto prefix_end = prefix.second;
    const auto iters = std::mismatch(str.first + str_start_pos, str.second,
        prefix_start, prefix_end);
    return iters.second == prefix_end;
  }

  static
  std::size_t common_prefix(const T_Key_Internal& str, std::size_t str_start_pos, const T_Key_Internal& prefix, std::size_t prefix_start_pos) {
    const auto str_start = str.first + str_start_pos;
    const auto iters = std::mismatch(str_start, str.second,
        prefix.first + prefix_start_pos, prefix.second);
    return std::distance(str_start, iters.first);
  }


  void insert_single(const T_Key_Internal& key, const T_Value& value) {
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

          // TODO:
          // If this edge leads to the same value (It can lead to many values),
          // then do nothing, because find_candidate_values() would already use this edge to find this value.
          // I think this is an "implicit" value. murrayc.

          // Split it,
          // adding a new intermediate node in it original node's place, with the original node as a child.
          edge.split(prefix_len);

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

    node->append_node(suffix, value);
  }

  const typename Node::Edge* find_edge(const T_Key& key_str) const {
    //std::cout << "find_node(): key=" << key << std::endl;
    if (key_str.empty()) {
      return nullptr;
    }

    const auto start = std::cbegin(key_str);
    const auto end = start + key_str.size();
    const auto key = std::make_pair(start, end);
    return find_edge(key);
  }

  const typename Node::Edge* find_edge(const T_Key_Internal& key) const {
    //std::cout << "find_node(): key=" << key << std::endl;
    if (str_empty(key)) {
      return nullptr;
    }

    const typename Node::Edge* edge = nullptr;
    std::size_t key_pos = 0;
    const auto key_size = str_size(key);
    while (key_pos < key_size) {
      //std::cout << "find_node(): remaining key=" << str_substr(key, key_pos) << std::endl;
      //std::cout << "  children_ size: " << node->children_.size() << std::endl;
      //Choose the child node, if any:
      const typename Node::Edge* edge_next = nullptr;

      const Node* node = edge ? edge->dest_ : &root_;
      for (const auto& child_edge : node->children_) {
        const auto& part = child_edge.part_;
        const auto part_size = str_size(part);
        //std::cout << "  key=" << key << ", key_pos=" << key_pos << ", part=" << part << "\n";
        if(!has_prefix(key, key_pos, part)) {
          continue;
        }

        edge_next = &child_edge;
        key_pos += part_size;
        //std::cout << "    next: " << next << std::endl;
        break;
      }

      if (!edge_next) {
        return nullptr;
      }

      edge = edge_next;
    }

    if (key_pos < key_size) {
      //We didn't find all the parts of the prefix:
      return nullptr;
    }

    //std::cout << "node: " << node << std::endl;
    const auto node = edge->dest_;
    return node->has_value() ? edge : nullptr;
  }

  const Node* find_node(const T_Key& key) const {
    const auto edge = find_edge(key);
    if (!edge) {
      return nullptr;
    }

    return edge->dest_;
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

  Node root_;
};

#endif // MURRAYC_SUFFIX_TREE_SUFFIX_TREE_H
