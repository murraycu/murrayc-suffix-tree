#ifndef MURRAYC_SUFFIX_TREE_SUFFIX_TREE_H
#define MURRAYC_SUFFIX_TREE_SUFFIX_TREE_H

#include <iostream>
#include <vector>
#include <set>
#include <stack>
#include <algorithm>

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
    key_with_terminator_ = key + "$";
    const auto start = std::cbegin(key_with_terminator_);
    const auto end = start + key_with_terminator_.size();
    const KeyInternal substr(start, end);
    if (str_empty(substr)) {
      return;
    }

    insert_ukkonen(substr, value);
  }

  /*
  void insert(const typename T_Key::const_iterator& start, const typename T_Key::const_iterator& end, const T_Value& value) {
    const KeyInternal substr(start, end);
    if (str_empty(substr)) {
      return;
    }

    insert_ukkonen(substr, value);
  }
  */

  using Candidates = std::set<T_Value>;

  /** Finds the values for any key containing this substring.
   */
  Candidates find(const T_Key& substr) const {
    Candidates result;

    if (substr.empty()) {
      return result;
    }


    const auto start = std::cbegin(substr);
    const auto end = start + substr.size();
    const KeyInternal substr_key(start, end);
    return find(substr_key);
  }

  void debug_print() const {
    std::cout << "Tree:" << std::endl;
    debug_print(&root_, 0);
    std::cout << std::endl << std::endl;
  }

private:
  /// Start and end (1 past last position) of a substring in text_;
  using KeyIterator = typename T_Key::const_iterator;

  class KeyInternal {
  public:
    KeyInternal() = default;

    KeyInternal(const KeyIterator& start, const KeyIterator& end)
    : start_(start), end_(end) {
      }

    KeyInternal(const KeyInternal& src) = default;
    KeyInternal& operator=(const KeyInternal& src) = default;
    KeyInternal(KeyInternal&& src) = default;
    KeyInternal& operator=(KeyInternal&& src) = default;

    KeyIterator start_;
    KeyIterator end_;
  };

  inline static KeyIterator str_end(const KeyInternal& key) {
    return key.end_;
  }

  class Node {
  public:
    Node() = default;
    Node(const Node& src) = default;
    Node& operator=(const Node& src) = default;
    Node(Node&& src) = default;
    Node& operator=(Node&& src) = default;

    ~Node() {
      for(auto& edge : children_) {
        delete edge.dest_;
      }
    }

    class Edge {
    public:
      Edge(const KeyInternal& part, Node* dest)
        : part_(part),
          dest_(dest) {
        assert(str_size(part));
      }

      Edge(const Edge& src) = default;
      Edge& operator=(const Edge& src) = default;
      Edge(Edge&& src) = default;
      Edge& operator=(Edge&& src) = default;

      inline bool dest_has_children() {
        if (!dest_) {
          return false;
        }

        return !dest_->children_.empty();
      }

      inline bool dest_has_value() {
        if (!dest_) {
          return false;
        }

        return dest_->has_value();
      }

      void append_node_to_dest(const KeyInternal& part, const T_Value& value) {
        dest_->append_node(part, value);
      }

      /** This inserts an intermediate node by splitting the edge's part at
       * position @pos.
       * @result The new intermediate node.
       */
      Node* split(std::size_t part_pos) {
        const auto prefix_part = str_substr(part_, 0, part_pos);
        assert(str_size(prefix_part) > 0);
        const auto suffix_part = str_substr(part_, part_pos);
        assert(str_size(suffix_part) > 0);
        const auto dest = dest_;

        auto extra_node = new Node;
        extra_node->append_node(suffix_part, dest);

        part_ = prefix_part;
        dest_ = extra_node;

        return extra_node;
      }

      void ensure_dest_has_value(const T_Value& value) {
        dest_->ensure_has_value(value);
      }

      KeyInternal part_;
      Node* dest_ = nullptr;
    };

    void append_node(const KeyInternal& part, const T_Value& value) {
      const auto extra_node = new Node();
      extra_node->values_.emplace_back(value);
      children_.emplace_back(part, extra_node);
    }

    void append_node(const KeyInternal& part) {
      const auto extra_node = new Node();
      children_.emplace_back(part, extra_node);
    }

    void append_node(const KeyInternal& part, Node* node) {
      children_.emplace_back(part, node);
    }

    void ensure_has_value(const T_Value& value) {
      const auto end = std::end(values_);
      if (std::find(std::begin(values_), end, value) == end) {
        values_.emplace_back(value);
      }
    }

    inline bool has_value() const {
      return !values_.empty();
    }

    //We could instead have a std::vector<Node*> children_,
    //of size alphabet (such as 26),
    //to allow O(1) lookup, at the cost of wasted space.
    std::vector<Edge> children_;

    // TODO: Wastes space on non-leaves.
    // TODO: Use a set, though that would not allow duplicates.
    std::vector<T_Value> values_;
  };

  void insert_ukkonen(const KeyInternal& key, const T_Value& value) {
    //std::cout << "insert_ukkonen(): " << debug_key(key) << std::endl;

    // Use Ukkonen's algorithm for suffix tree construction:
    const auto key_start = key.start_;
    const auto key_end = str_end(key);

    // The "phases"
    for (auto i = key_start; i != key_end; ++i) {
      //std::cout << "  i=" << *i << std::endl;

      // The "extensions".
      const auto j_end = i + 1;
      for (auto j = key_start; j != j_end; ++j) {
        //std::cout << "    j=" << *j << std::endl;

        const KeyInternal key_prefix(j, i + 1);
        //std::cout << "    key_prefix: " << debug_key(key_prefix) << std::endl;
        //std::cout << "      key_prefix len:" << str_size(key_prefix) << std::endl;

        const auto edge_and_part_pos = find_partial_edge(key_prefix);
        const auto edge = edge_and_part_pos.edge_;
        const auto part_len_used = edge_and_part_pos.edge_part_used_;
        const auto key_prefix_len_used = edge_and_part_pos.substr_used_; 

        //std::cout << "     key_prefix_len_used=" << key_prefix_len_used << std::endl;
 
        const bool whole_part_used = edge ? (part_len_used == str_size(edge->part_)) : false;
        const bool whole_prefix_used = key_prefix_len_used == str_size(key_prefix);

        if (!whole_prefix_used && !whole_part_used) {
          // Rule 2 extension: There is no match, or a partial match:
          if (!edge) {
            //std::cout << "      Rule 2: Adding edge to root: " << std::endl;
            root_.append_node(key_prefix, value);
          } else {
            //std::cout << "      Rule 2: Splitting edge " << debug_key(edge->part_) << " at " << part_len_used << " and adding." << std::endl;
            auto extra_node = edge->split(part_len_used);
            const auto suffix = str_substr(key_prefix, key_prefix_len_used);
            extra_node->append_node(suffix, value);
          }

          continue;
        };

        assert(edge);

        if (whole_part_used) {
          // Rule 1 extension: There is a path that is a partial match:
          //std::cout << "      Rule 1: Appending to edge's substring." << std::endl;
          // Extend the edge's path by adding the extra character:
          // Note: If the suffix tree was storing only one string,
          // we could avoid the need for this by just using a special "end" value on
          // all these (to be) leaves, that wouldn't need to be updated.
          edge->part_ = key_prefix;
          //std::cout << "        new part: " << debug_key(edge->part_) << std::endl;

          continue;
        }

        // Rule 3 extension:
        //std::cout << "      Rule 3: Do nothing." << std::endl;
      }
    }
  }

  /** Finds the values for any key containing this substring.
   */
  std::set<T_Value> find(const KeyInternal& substr) const {
    std::set<T_Value> result;

    if (str_empty(substr)) {
      return result;
    }

    auto unconst = const_cast<SuffixTree<T_Key, T_Value>*>(this);
    const auto start = unconst->find_partial_edge(substr);
    const auto start_edge = start.edge_;
    if (!start_edge) {
      return result;
    }
    
    const auto start_substr_used = start.substr_used_;
    if (start_substr_used != str_size(substr)) {
      return result;
    }

    const auto substr_len = str_size(substr);

    using Item = std::pair<std::size_t /* substr_pos */, const Node*>;
    std::stack<Item> stack;
    stack.emplace(start_substr_used, start_edge->dest_);

    while (!stack.empty()) {
      const auto item = stack.top();
      stack.pop();

      const auto substr_pos = item.first;
      const auto node = item.second;

      if (node->has_value()) {
        result.insert(std::cbegin(node->values_), std::cend(node->values_));

        //And continue to examine children, because they can have values too.
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
  bool has_prefix(const KeyInternal& str, std::size_t str_start_pos, const KeyInternal& prefix, std::size_t prefix_start_pos = 0) {
    const auto prefix_start = prefix.start_ + prefix_start_pos;
    const auto prefix_end = str_end(prefix);
    const auto iters = std::mismatch(str.start_ + str_start_pos, str_end(str),
        prefix_start, prefix_end);
    return iters.second == prefix_end;
  }

  static
  std::size_t common_prefix(const KeyInternal& str, std::size_t str_start_pos, const KeyInternal& prefix, std::size_t prefix_start_pos) {
    const auto str_start = str.start_ + str_start_pos;
    const auto iters = std::mismatch(str_start, str_end(str),
        prefix.start_ + prefix_start_pos, str_end(prefix));
    return std::distance(str_start, iters.first);
  }

  void insert_single(const KeyInternal& key, const T_Value& value) {
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
          // then do nothing, because find() would already use this edge to find this value.
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

  /**
   * The Edge and the end of matching prefix of the edge's part.
   */
  class EdgeMatch {
  public:
    EdgeMatch() {
    }

    EdgeMatch(typename Node::Edge* edge, std::size_t edge_part_used, std::size_t substr_used)
    : edge_(edge), edge_part_used_(edge_part_used), substr_used_(substr_used) {
    }

    EdgeMatch(const EdgeMatch& src) = default;
    EdgeMatch& operator=(const EdgeMatch& src) = default;
    EdgeMatch(EdgeMatch&& src) = default;
    EdgeMatch& operator=(EdgeMatch&& src) = default;

    typename Node::Edge* edge_ = nullptr;
    std::size_t edge_part_used_ = 0;
    std::size_t substr_used_ = 0;
  };

  /** Returns the edge and how much of the edge's part represents the @a substr.
   */
  EdgeMatch find_partial_edge(const KeyInternal& substr) {
    return find_partial_edge(&root_, substr);
  }

  /** Returns the edge and how much of the edge's part represents the @a substr.
   */
  EdgeMatch find_partial_edge(Node* start_node, const KeyInternal& substr) {
    //std::cout << "find_partial_edge(): substr=" << debug_key(substr) << std::endl;
    EdgeMatch result;

    if (str_empty(substr)) {
      return result;
    }

    const auto substr_len = str_size(substr);
    //std::cout << "substr_len=" << substr_len << std::endl;

    Node* node = start_node;
    std::size_t substr_pos = 0;
    typename Node::Edge* parent_edge = nullptr;
    std::size_t parent_edge_len_used = 0;
    while (node) {
      //std::cout << "  node:" << std::endl;
      bool edge_found = false;
      for (auto& edge : node->children_) {
        const auto& edge_part = edge.part_;
        //std::cout << "    edge: part=" << debug_key(edge_part) << std::endl;

        const auto len = common_prefix(substr, substr_pos, edge_part, 0);
        //std::cout << "      common_prefix_len=" << len << std::endl;
        if (len == 0) {
          continue;
        }

        const auto substr_remaining_len = substr_len - substr_pos;
        //std::cout << "      substr_remaining_len=" << substr_remaining_len << std::endl;
        if (len == str_size(edge_part)) {
          // The remaining substr has edge_part as a prefix.
          if (len == substr_remaining_len) {
            // And that uses up all of our substr:
            return EdgeMatch(&edge, len, substr_len);
          } else {
            // Some of our substr is still unused.
            //std::cout << "        following partial edge." << std::endl;
            // Follow the edge to try to use the rest of the substr:
            node = edge.dest_;
            substr_pos += str_size(edge_part);
            edge_found = true;

            // Remember how we got to the followed edge,
            // so we can return that as a partial path if necessary. 
            parent_edge = &edge;
            parent_edge_len_used = len;
            break;
          }
        } else if (len == substr_remaining_len) {
          // The edge has the remaining substr as its prefix.
          return EdgeMatch(&edge, len, substr_len);
        } else {
          // The edge has some of the remaining substr as its prefix.
          return EdgeMatch(&edge, len, substr_pos + len);
        }
      }

      if (!edge_found) {
        break;
      }
    }

    //std::cout << "  returning parent_edge=" << static_cast<void*>(parent_edge) <<
      //"parent_edge_len_used=" << parent_edge_len_used <<
      //"substr_pos=" << substr_pos << std::endl;
    return EdgeMatch(parent_edge, parent_edge_len_used, substr_pos);
  }

  static
  inline std::size_t str_size(const KeyInternal& key) {
    const auto end = str_end(key);
    if (end <= key.start_) {
      return 0;
    }

    return end - key.start_;
  }

  static
  inline bool str_empty(const KeyInternal& key) {
    return key.start_ >= str_end(key);
  }

  static
  inline KeyInternal str_substr(const KeyInternal& key, std::size_t start) {
    const auto start_used = key.start_ + start;
    const auto key_end = str_end(key);
    return KeyInternal(
      (start_used < key_end) ? start_used : key_end,
      key_end);
  }

  static
  inline KeyInternal str_substr(const KeyInternal& key, std::size_t start, std::size_t len) {
    const auto start_used = key.start_ + start;
    const auto end_used = key.start_ + len;
    const auto key_end = str_end(key);
    return KeyInternal(
      (start_used < key_end) ? start_used : key_end,
      (end_used < key_end) ? end_used : key_end);
  }

  static std::string debug_key(const KeyInternal& key) {
    const auto key_end = str_end(key);
    if (key_end <= key.start_) {
      return std::string();
    }

    return std::string(key.start_, key_end);
  }

  static void debug_print_indent(std::size_t indent) {
    for (std::size_t i = 0; i < indent; ++i) {
      std::cout << ' ';
    }
  }

  static void debug_print(const Node* node, std::size_t indent) {
    if (!node) {
      return;
    }


    for (const auto& edge : node->children_) {
      debug_print_indent(indent);
      std::cout << debug_key(edge.part_);
      if (edge.dest_has_value()) {
        std::cout << "(";
        bool first = true;
        for (const auto value : edge.dest_->values_) {
          if (!first) {
            std::cout << ", ";
          }
          std::cout << value;
          first = false;
        }
        std::cout << ")";
      }
      std::cout << std::endl;

      debug_print(edge.dest_, indent + str_size(edge.part_));
    }
  }

  Node root_;
  T_Key key_with_terminator_;
};

#endif // MURRAYC_SUFFIX_TREE_SUFFIX_TREE_H
