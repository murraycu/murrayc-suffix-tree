#ifndef MURRAYC_SUFFIX_TREE_SUFFIX_TREE_H
#define MURRAYC_SUFFIX_TREE_SUFFIX_TREE_H

#include <iostream>
#include <vector>
#include <set>
#include <stack>
#include <algorithm>
#include <memory>

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
    std::cout << &root_ << ": " << std::endl;
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

    KeyInternal(const KeyIterator& start, const std::shared_ptr<const KeyIterator>& end)
    : start_(start),
      global_end_(end) {
    }

    KeyInternal(const KeyInternal& src) = default;
    KeyInternal& operator=(const KeyInternal& src) = default;
    KeyInternal(KeyInternal&& src) = default;
    KeyInternal& operator=(KeyInternal&& src) = default;

    KeyIterator start_;
    KeyIterator end_;
    std::shared_ptr<const KeyIterator> global_end_;
  };

  inline static KeyIterator str_end(const KeyInternal& key) {
    if (key.global_end_) {
      return *(key.global_end_);
    }

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
        //assert(str_size(part));
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

      inline bool dest_has_value() const {
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

    // For Ukkonen's Suffix Tree construction algorithm.
    Node* suffix_link_ = nullptr;
  };

  class ActivePoint {
  public:
    Node* node = nullptr;
    bool edge_valid = false;
    KeyIterator edge; // Instead of starting with -1, we set active.edge_valid to false.
    std::size_t length = 0;
  };

  void insert_ukkonen(const KeyInternal& key, const T_Value& value) {
    //std::cout << "insert_ukkonen(): " << debug_key(key) << std::endl;

    // Use Ukkonen's algorithm for suffix tree construction:
    const auto key_start = key.start_;
    const auto key_end = str_end(key);
    const auto key_last = key_end - 1;

    // These determine where the next phase will start.
    // We start at the active.node, on the edge with first character key[active.edge],
    // and active.length characters along that edge.
    ActivePoint active;
    active.node = &root_;
    active.edge_valid = false;
    active.length = 0;

    std::size_t remaining = 0;
    auto end_ptr = std::make_shared<KeyIterator>(key_start);
    KeyIterator& end = *end_ptr; //end is 1 past the end, so this is equivalent to -1 in the traditional Ukkonnen implementation.

    // The "phases"
    for (auto i = key_start; i != key_end; ++i) {
      std::cout << "  character: " << *i << std::endl;
      const bool is_last_char = (i == key_last);

      ++remaining;
      ++end; //This extends all existing paths by one character.

      Node* prev_created_internal_node = nullptr;

      // The "extensions".
      while(remaining) {

        std::cout << "    remaining: " << remaining << std::endl;
        std::cout << "    end: " << std::distance(key_start, end) << std::endl;
        if (active.edge_valid) {
          std::cout << "    active.edge: " << std::distance(key_start, active.edge) << std::endl;
          std::cout << "    active.length: " << active.length << std::endl;
        }

        // An active.length of 0 means we ignore the active.edge.
        const auto edge_match = (active.edge_valid && active.length) ?
          find_partial_edge(active, i) :
          find_partial_edge(active.node, i);
        const auto edge = edge_match.edge_;
        const auto part_len_used = edge_match.edge_part_used_;
        assert(edge_match.substr_used_ <= 1);
        const bool prefix_used = edge_match.substr_used_ > 0;

        const bool whole_part_used = edge ? (part_len_used == str_size(edge->part_)) : false;

        if ((!prefix_used || is_last_char) && !whole_part_used) {
          KeyInternal prefix(i, end_ptr);

          // Rule 2 extension: There is no match:
          if (part_len_used == 0) {
            // There is no match from root:
            if (!edge) {
              std::cout << "      Rule 2: Adding edge to root: " << debug_key(prefix) << std::endl;
              root_.append_node(prefix, value);
            } else {
              // There is no match from a non-root node, so add to the parent instead
              std::cout << "      Rule 2: Adding edge to parent node: " << debug_key(prefix) << std::endl;
              edge_match.parent_node_->append_node(prefix, value);
            }
          } else {
            // There is a partial match, in the middle of an edge:
            std::cout << "      Rule 2: Splitting edge " << debug_key(edge->part_) << " at " << part_len_used << " and adding." << std::endl;
            auto extra_node = edge->split(part_len_used);
            auto suffix = str_substr(prefix, 1);
            suffix.global_end_ = end_ptr;
            extra_node->append_node(suffix, value);

            // Every internal node should have a suffix link:
            extra_node->suffix_link_ = &root_;

            // A previously-created internal node should now have its suffix link
            // updated to this new internal node.
            if (prev_created_internal_node) {
              prev_created_internal_node->suffix_link_ = extra_node;
            }
            prev_created_internal_node = extra_node;

            // Follow previous suffix link if the active node is not root:
            if (active.node != &root_) {
              active.node = active.node->suffix_link_;
              // Not changing active.edge or active.length.
              // Note: If there are multiple constructions, then active.length
              // might now be past the end of the actual edge's part.
            } else {
              // After creating an internal node,
              // decrement active.length and increment active.edge,
              // so we look for the same character in an edge that is the same as
              // the previously active edge but without the leading character.
              --active.length;
              ++active.edge;
            }
          }

          // There is no change to active.node, active.edge, or active.length
          // after a Rule 2 extension.
          --remaining;
          continue;
        }

        assert(edge);

        if (whole_part_used && !edge->dest_has_children()) {
          // Rule 1 extension: There is a path that is a partial match:
          std::cout << "      Rule 1: Appending to edge's substring, automatically by incrementing end." << std::endl;

          continue;
        }

        // Rule 3 extension:
        std::cout << "      Rule 3: Do nothing." << std::endl;

        active.node = edge_match.parent_node_;
        active.edge = edge->part_.start_; //Start of range of the existing edge.
        active.edge_valid = true;
        active.length = part_len_used;

        // After a rule 3 extension, immediately start the next "phase".
        break;
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

  /**
   * The Edge and the end of matching prefix of the edge's part.
   */
  class EdgeMatch {
  public:
    EdgeMatch()
    : edge_(nullptr),
      edge_part_used_(0),
      substr_used_(0),
      parent_node_(nullptr) {
    }

    EdgeMatch(typename Node::Edge* edge, std::size_t edge_part_used, std::size_t substr_used, Node* parent_node)
    : edge_(edge), edge_part_used_(edge_part_used), substr_used_(substr_used), parent_node_(parent_node) {
    }

    EdgeMatch(const EdgeMatch& src) = default;
    EdgeMatch& operator=(const EdgeMatch& src) = default;
    EdgeMatch(EdgeMatch&& src) = default;
    EdgeMatch& operator=(EdgeMatch&& src) = default;

    typename Node::Edge* edge_;
    std::size_t edge_part_used_;
    std::size_t substr_used_;
    Node* parent_node_;
  };

  /** Returns the edge and how much of the edge's part represents the @a substr.
   */
  static
  EdgeMatch find_partial_edge(Node* start_node, const KeyIterator& next_char) {
    auto edge = find_edge(start_node, next_char);
    if (!edge) {
      return EdgeMatch();
    }

    return EdgeMatch(edge, 1, 1, start_node);
  }

  static
  typename Node::Edge* find_edge(Node* node, const KeyIterator& next_char) {
    typename Node::Edge* result = nullptr;

    const auto& ch = *next_char;
    const auto end = std::end(node->children_);
    auto iter = std::find_if(std::begin(node->children_), end,
      [&ch]( auto& edge) {
        return *(edge.part_.start_) == ch;
      });
    if (iter != end) {
      result = &(*iter);
    }

    return result;
  }

  /** Returns the edge and how much of the edge's part represents the @a substr.
   */
  static
  EdgeMatch find_partial_edge(const ActivePoint& active, const KeyIterator& next_char) {
    auto start_node = active.node;
    assert(start_node);

    auto start_edge = find_edge(start_node, active.edge);
    assert(start_edge);

    return find_partial_edge_from_edge(start_node, start_edge, active.length, next_char);
  }

  static
  EdgeMatch find_partial_edge_from_edge(Node* start_edge_parent_node, typename Node::Edge* start_edge, std::size_t start_edge_pos, const KeyIterator& next_char) {
    const KeyInternal substr(next_char, next_char + 1);

    const auto substr_len = str_size(substr);
    auto edge = start_edge;
    auto edge_part_pos = start_edge_pos;
    auto substr_pos = 0;
    Node* parent_node = start_edge_parent_node;
    typename Node::Edge* parent_edge = start_edge;
    while(true) {

      const auto& edge_part = edge->part_;
      //std::cout << "    edge: part=" << debug_key(edge_part) << ", edge_part_pos=" << edge_part_pos <<
      //  ", substr: " << debug_key(substr, substr_pos) << std::endl;

      const auto len = common_prefix(substr, substr_pos, edge_part, edge_part_pos);
      //std::cout << "      common_prefix_len=" << len << std::endl;

      const auto substr_remaining_len = substr_len - substr_pos;
      //std::cout << "      substr_remaining_len=" << substr_remaining_len << std::endl;
      const auto edge_part_used = len + edge_part_pos;
      //std::cout << "      edge_part_used=" << edge_part_used << std::endl;
      if (len == str_size(edge_part, edge_part_pos)) {
        // The remaining substr has edge_part as a prefix.
        if (len == substr_remaining_len) {
          // And that uses up all of our substr:
          return EdgeMatch(edge, edge_part_used, substr_len, parent_node);
        } else {
          //std::cout << "  Examining child nodes" << std::endl;
          // Some of our substr is still unused.
          //std::cout << "        following partial edge." << std::endl;
          // Follow the edge to try to use the rest of the substr:
          substr_pos += str_size(edge_part, edge_part_pos);

          //Examine all the next edges, to choose one.
          typename Node::Edge* next_edge = nullptr;
          auto node = edge->dest_;
          const auto end = std::end(node->children_);
          const auto iter = std::find_if(std::begin(node->children_),
            end,
            [substr, substr_pos](const auto& e) {
              const auto& next_edge_part = e.part_;

              // Only one edge should match:
              return has_prefix(next_edge_part, 0, substr, substr_pos);
            });
          if (iter == end) {
            return EdgeMatch(edge, len, substr_pos, parent_node);
          }

          //std::cout << "    child node found." << std::endl;
          next_edge = &(*iter);
          edge_part_pos = 0;

          // Follow this edge:
          parent_node = node;
          parent_edge = edge;
          edge = next_edge;
          continue;
        }
      } else if (len == 0) {
        return EdgeMatch(parent_edge, edge_part_pos, 0, parent_node);
      }
      else if (len == substr_remaining_len) {
        // The edge has the remaining substr as its prefix.
        return EdgeMatch(edge, edge_part_used, substr_len, parent_node);
      } else {
        // The edge has some of the remaining substr as its prefix.
        return EdgeMatch(edge, edge_part_used, substr_pos + len, parent_node);
      }
    }
  }

  static
  inline std::size_t str_size(const KeyInternal& key, std::size_t key_pos = 0) {
    const auto start = key.start_ + key_pos;
    const auto end = str_end(key);
    if (end <= start) {
      return 0;
    }

    return end - start;
  }

  static
  inline bool str_empty(const KeyInternal& key) {
    return key.start_ >= str_end(key);
  }

  static
  inline KeyInternal str_substr(const KeyInternal& key, std::size_t start) {
    const auto start_used = key.start_ + start;
    const auto key_end = str_end(key);
    auto result= KeyInternal(
      (start_used < key_end) ? start_used : key_end,
      key_end);

    // If the input used the global end, then so should the substring,
    // because it has the same end.
    if (key.global_end_) {
      result.global_end_= key.global_end_;
    }

    return result;
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

  static std::string debug_key(const KeyInternal& key, std::size_t pos) {
    const auto key_start = key.start_ + pos;
    const auto key_end = str_end(key);
    if (key_end <= key_start) {
      return std::string();
    }

    return std::string(key_start, key_end);
  }

  static std::string debug_key(const KeyInternal& key) {
    return debug_key(key, 0);
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
      std::cout << edge.dest_ << ": ";
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
};

#endif // MURRAYC_SUFFIX_TREE_SUFFIX_TREE_H
