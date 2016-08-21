#ifndef MURRAYC_SUFFIX_TREE_SUFFIX_TREE_H
#define MURRAYC_SUFFIX_TREE_SUFFIX_TREE_H

#include <iostream>
#include <unordered_set>
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
    key_terminated_ = key + "$";
    const auto start = std::cbegin(key_terminated_);
    const auto end = std::next(start, key_terminated_.size());
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

    const KeyInternal substr_key(std::cbegin(substr), std::cend(substr));
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
      extra_node->values_.emplace(value);
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

    inline void add_value(const T_Value& value) {
      values_.emplace(value);
    }

    // We could instead have a std::vector<Node*> children_,
    // of size alphabet (such as 26),
    // to allow O(1) lookup, at the cost of wasted space.o
    // A hash table might be the simplest way, giving amortized O(1) lookup.
    // But see the mention of suffix trays here:
    // http://www.murrayc.com/permalink/2016/08/19/suffix-tree-ukkonen-c/
    std::vector<Edge> children_;

    // TODO: Wastes space on non-leaves.
    // TODO: Use a set, though that would not allow duplicates.
    std::unordered_set<T_Value> values_;

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

      ++remaining;
      ++end; //This extends all existing paths by one character.

      Node* prev_created_internal_node = nullptr;

      // The "extensions".
      while(remaining) {

        std::cout << "    remaining: " << remaining << std::endl;
        std::cout << "    end: " << std::distance(key_start, end) << std::endl;
        std::cout << "    active.node: " << active.node << std::endl;
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

        if (!edge_match.char_found_) {
          KeyInternal prefix(i, end_ptr);

          // Rule 2 extension: There is no match:
          if (part_len_used == 0) {
            // There is no match:
            std::cout << "      Rule 2: Adding edge to active node: " << debug_key(prefix) << std::endl;
            active.node->append_node(prefix, value);
          } else {
            // There is a partial match, in the middle of an edge:
            std::cout << "      Rule 2: Splitting edge " << debug_key(edge->part_) << " at " << part_len_used << " and adding." << std::endl;
            auto extra_node = edge->split(part_len_used);
            extra_node->append_node(prefix, value);

            // Every internal node should have a suffix link:
            std::cout << "      Setting suffix link from " << extra_node << " to root " << &root_ << std::endl;
            extra_node->suffix_link_ = &root_;

            // A previously-created internal node should now have its suffix link
            // updated to this new internal node.
            if (prev_created_internal_node) {
              std::cout << "      Updating suffix link from " << prev_created_internal_node << " to " << extra_node << std::endl;
              prev_created_internal_node->suffix_link_ = extra_node;
            }
            prev_created_internal_node = extra_node;

            // Follow previous suffix link if the active node is not root:
            if (active.node != &root_) {
              std::cout << "      Following suffix link of active node " << active.node << " to " << active.node->suffix_link_ << std::endl;
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
    const auto prefix_start = std::next(prefix.start_, prefix_start_pos);
    const auto prefix_end = str_end(prefix);
    const auto iters = std::mismatch(std::next(str.start_, str_start_pos), str_end(str),
        prefix_start, prefix_end);
    return iters.second == prefix_end;
  }

  /**
   * The Edge and the end of matching prefix of the edge's part.
   */
  class EdgeMatch {
  public:
    EdgeMatch()
    : edge_(nullptr),
      edge_part_used_(0),
      char_found_(false),
      parent_node_(nullptr) {
    }

    EdgeMatch(typename Node::Edge* edge, std::size_t edge_part_used, bool char_found, Node* parent_node)
    : edge_(edge), edge_part_used_(edge_part_used), char_found_(char_found), parent_node_(parent_node) {
    }

    EdgeMatch(const EdgeMatch& src) = default;
    EdgeMatch& operator=(const EdgeMatch& src) = default;
    EdgeMatch(EdgeMatch&& src) = default;
    EdgeMatch& operator=(EdgeMatch&& src) = default;

    typename Node::Edge* edge_;
    std::size_t edge_part_used_;
    bool char_found_;
    Node* parent_node_;
  };

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

  /** Returns the edge and how far along the edge's part the character was found..
   */
  static
  EdgeMatch find_partial_edge(Node* start_node, const KeyIterator& next_char) {
    auto edge = find_edge(start_node, next_char);
    if (!edge) {
      return EdgeMatch();
    }

    return EdgeMatch(edge, 1, 1, start_node);
  }

  /** Returns the edge and how far along the edge's part the character was found.
   */
  static
  EdgeMatch find_partial_edge(const ActivePoint& active, const KeyIterator& next_char) {
    assert(active.node);
    auto edge = find_edge(active.node, active.edge);
    assert(edge);

    auto edge_part_pos = active.length;
    Node* parent_node = active.node;
    while(true) {
      const auto& edge_part = edge->part_;

      //This cannot step more than one character away from an intermediate node.
      assert(edge_part_pos < (str_size(edge_part) + 1));

      const auto part_next = std::next(edge_part.start_, edge_part_pos);
      if (part_next >= str_end(edge_part)) {
        // If the active length tells us to go further than the length of the part,
        // step over the destination.
        //
        // Find the edge from the destination that has the next character:
        parent_node = edge->dest_;
        edge = find_edge(parent_node, next_char);
        if (!edge) {
          return EdgeMatch(edge, edge_part_pos, false, parent_node);
        }

        //Try again at the start of the followed edge:
        edge_part_pos = 0;
        continue;
      }

      if (*part_next == *next_char) {
        return EdgeMatch(edge, edge_part_pos + 1, true, parent_node);
      }

      return EdgeMatch(edge, edge_part_pos, false, parent_node);
    }
  }

  static
  inline std::size_t str_size(const KeyInternal& key, std::size_t key_pos = 0) {
    const auto start = std::next(key.start_, key_pos);
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
    const auto start_used = std::next(key.start_, start);
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
    const auto start_used = std::next(key.start_, start);
    const auto end_used = std::next(key.start_, len);
    const auto key_end = str_end(key);
    return KeyInternal(
      (start_used < key_end) ? start_used : key_end,
      (end_used < key_end) ? end_used : key_end);
  }

  static std::string debug_key(const KeyInternal& key, std::size_t pos) {
    const auto key_start = std::next(key.start_, pos);
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
  T_Key key_terminated_; //To keep the memory allocated.
};

#endif // MURRAYC_SUFFIX_TREE_SUFFIX_TREE_H
