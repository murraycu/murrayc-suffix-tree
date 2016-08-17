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
    assert(debug_exists(key_with_terminator_));
  }

  /*
  void insert(const typename T_Key::const_iterator& start, const typename T_Key::const_iterator& end, const T_Value& value) {
    const KeyInternal substr(start, end);
    if (str_empty(substr)) {
      return;
    }

    insert_ukkonen(substr, value);
    assert(debug_exists(substr));
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
    KeyInternal() 
    : uses_global_end_(false) {
    }

    KeyInternal(const KeyIterator& start, const KeyIterator& end)
    : start_(start), end_(end),
      uses_global_end_(false) {
      const char* p = &(*(start_));
      assert(p != 0);
    }

    KeyInternal(const KeyInternal& src) = default;
    KeyInternal& operator=(const KeyInternal& src) = default;
    KeyInternal(KeyInternal&& src) = default;
    KeyInternal& operator=(KeyInternal&& src) = default;

    KeyIterator start_;
    KeyIterator end_;
    bool uses_global_end_;
  };

  inline KeyIterator str_end(const KeyInternal& key) const {
    if (key.uses_global_end_) {
      return end_;
    }

    return key.end_;
  }

  bool debug_exists(const T_Key& key) const {
    // TODO: Add const overloads of find_node(), find_edge(), etc,
    // without duplicating code.
    //auto unconst = const_cast<std::remove_const_t<decltype(this)>>(this);
    auto unconst = const_cast<SuffixTree<T_Key, T_Value>*>(this);
    const auto node = unconst->find_node(key);
    return node != nullptr;
  };

  bool debug_exists(const KeyInternal& key) const {
    // TODO: Add const overloads of find_node(), find_edge(), etc,
    // without duplicating code.
    //auto unconst = const_cast<std::remove_const_t<decltype(this)>>(this);
    auto unconst = const_cast<SuffixTree<T_Key, T_Value>*>(this);
    const auto node = unconst->find_node(key);
    return node != nullptr;
  };

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
      Node* split(SuffixTree* tree, std::size_t part_pos) {
        const auto prefix_part = tree->str_substr(part_, 0, part_pos);
        assert(tree->str_size(prefix_part) > 0);
        const auto suffix_part = tree->str_substr(part_, part_pos);
        assert(tree->str_size(suffix_part) > 0);
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


    // These determine where the next phase will start.
    // We start at the active.node, on the edge with first character key[active.edge],
    // and active.length characters along that edge.
    ActivePoint active;
    active.node = &root_;
    active.edge_valid = false;
    active.length = 0;

    std::size_t remaining = 0;
    end_ = key_start; //end is 1 past the end, so this is equivalent to -1 in the traditional Ukkonnen implementation.

    // The "phases"
    for (auto i = key_start; i != key_end; ++i) {
      //TODO: Check for just one character:
      const KeyInternal key_prefix(i, i + 1);
      std::cout << "  key_prefix: " << debug_key(key_prefix) << std::endl;

      ++remaining;
      ++end_; //This extends all existing paths by one character.

      Node* prev_created_internal_node = nullptr;

      // The "extensions".
      while(remaining) {

        std::cout << "    remaining: " << remaining << std::endl;
        std::cout << "    end: " << std::distance(key_start, end_) << std::endl;
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
        const auto key_prefix_len_used = edge_match.substr_used_; 

        const bool whole_part_used = edge ? (part_len_used == str_size(edge->part_)) : false;
        const bool whole_prefix_used = key_prefix_len_used == str_size(key_prefix);

        if (!whole_prefix_used && !whole_part_used) {
          KeyInternal prefix = key_prefix;
          prefix.uses_global_end_ = true;

          // Rule 2 extension: There is no match, or a partial match:
          if (part_len_used == 0) {
            // There is no match:
            if (!edge) {
              std::cout << "      Rule 2: Adding edge to root: " << debug_key(prefix) << std::endl;
              root_.append_node(prefix, value);
            } else {
              // Add to the parent instead
              std::cout << "      Rule 2: Adding edge to parent node: " << debug_key(prefix) << std::endl;
              edge_match.parent_node_->append_node(prefix, value);
            }
          } else if (key_prefix_len_used == 0) {
            // There is a partial match:
            std::cout << "      Rule 2: Splitting edge " << debug_key(edge->part_) << " at " << part_len_used << " and adding." << std::endl;
            auto extra_node = edge->split(this, part_len_used);
            auto suffix = str_substr(prefix, key_prefix_len_used);
            suffix.uses_global_end_ = true;
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

  bool has_prefix(const KeyInternal& str, std::size_t str_start_pos, const KeyInternal& prefix, std::size_t prefix_start_pos = 0) const {
    const auto prefix_start = prefix.start_ + prefix_start_pos;
    const auto prefix_end = str_end(prefix);
    const auto iters = std::mismatch(str.start_ + str_start_pos, str_end(str),
        prefix_start, prefix_end);
    return iters.second == prefix_end;
  }

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
  EdgeMatch find_partial_edge(const KeyInternal& substr) {
    return find_partial_edge(&root_, substr);
  }

  /** Returns the edge and how much of the edge's part represents the @a substr.
   */
  EdgeMatch find_partial_edge(Node* start_node, const KeyIterator& next_char) {

    const KeyInternal substr(next_char, next_char + 1);
    return find_partial_edge(start_node, substr);
  }

  /** Returns the edge and how much of the edge's part represents the @a substr.
   */
  EdgeMatch find_partial_edge(Node* start_node, const KeyInternal& substr) {
    // Try all edges.
    for (auto& edge : start_node->children_) {
      const auto result = find_partial_edge_from_edge(start_node, &edge, 0, substr);
      if (result.edge_part_used_ > 0) {
        return result;
      }
    }

    return EdgeMatch();
  }

  /** Returns the edge and how much of the edge's part represents the @a substr.
   */
  EdgeMatch find_partial_edge(const ActivePoint& active, const KeyIterator& next_char) {

    const KeyInternal substr(next_char, next_char + 1);
    return find_partial_edge(active, substr);
  }

  /** Returns the edge and how much of the edge's part represents the @a substr.
   */
  EdgeMatch find_partial_edge(const ActivePoint& active, const KeyInternal& substr) {
    auto start_node = active.node;
    assert(start_node);

    if (str_empty(substr)) {
      return EdgeMatch();
    }

    // If an edge was specified, start there first,
    // instead of examining all edges from the start node.
    typename Node::Edge* start_edge = nullptr;

    const auto end = std::end(start_node->children_);
    auto iter = std::find_if(std::begin(start_node->children_), end,
      [active](const auto& edge) {
        return *(edge.part_.start_) == *active.edge;
      });
    if (iter != end) {
      //std::cout << "active_edge found: " << debug_key(iter->part_) << std::endl;
      start_edge = &(*iter);
    }
    assert(start_edge);

    return find_partial_edge_from_edge(start_node, start_edge, active.length, substr);
  }

  EdgeMatch find_partial_edge_from_edge(Node* start_edge_parent_node, typename Node::Edge* start_edge, std::size_t start_edge_pos, const KeyInternal& substr) {
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
          for (auto& e : node->children_) {
            const auto& next_edge_part = e.part_;

            // Only one edge should match:
            if (has_prefix(next_edge_part, 0, substr, substr_pos)) {
              //std::cout << "    child node found." << std::endl;
              next_edge = &e;
              edge_part_pos = 0;
              break;
            }
          }

          if (next_edge) {
            // Follow this edge:
            parent_node = node;
            parent_edge = edge;
            edge = next_edge;
            continue;
          } else {
            return EdgeMatch(edge, len, substr_pos, parent_node);
          }
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

  typename Node::Edge* find_edge(const T_Key& key_str) {
    //std::cout << "find_node(): key=" << key << std::endl;
    if (key_str.empty()) {
      return nullptr;
    }

    const auto start = std::cbegin(key_str);
    const auto end = start + key_str.size();
    const KeyInternal key(start, end);
    return find_edge(key);
  }

  typename Node::Edge* find_edge(const KeyInternal& key) {
    //std::cout << "find_node(): key=" << key << std::endl;
    if (str_empty(key)) {
      return nullptr;
    }

    typename Node::Edge* edge = nullptr;
    std::size_t key_pos = 0;
    const auto key_size = str_size(key);
    while (key_pos < key_size) {
      //std::cout << "find_node(): remaining key=" << str_substr(key, key_pos) << std::endl;
      //std::cout << "  children_ size: " << node->children_.size() << std::endl;
      //Choose the child node, if any:
      typename Node::Edge* edge_next = nullptr;

      Node* node = edge ? edge->dest_ : &root_;
      for (auto& child_edge : node->children_) {
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
    auto node = edge->dest_;
    return node->has_value() ? edge : nullptr;
  }

  Node* find_node(const T_Key& key) {
    const auto edge = find_edge(key);
    if (!edge) {
      return nullptr;
    }

    return edge->dest_;
  }

  Node* find_node(const KeyInternal& key) {
    const auto edge = find_edge(key);
    if (!edge) {
      return nullptr;
    }

    return edge->dest_;
  }

  inline std::size_t str_size(const KeyInternal& key, std::size_t key_pos = 0) const {
    const auto start = key.start_ + key_pos;
    const auto end = str_end(key);
    if (end <= start) {
      return 0;
    }

    return end - start;
  }

  inline bool str_empty(const KeyInternal& key) const {
    return key.start_ >= str_end(key);
  }

  inline KeyInternal str_substr(const KeyInternal& key, std::size_t start) const {
    const auto start_used = key.start_ + start;
    const auto key_end = str_end(key);
    auto result= KeyInternal(
      (start_used < key_end) ? start_used : key_end,
      key_end);

    // If the input used the global end, then so should the substring,
    // because it has the same end.
    if (key.uses_global_end_) {
      result.uses_global_end_ = true;
    }
 
    return result;
  }

  inline KeyInternal str_substr(const KeyInternal& key, std::size_t start, std::size_t len) const {
    const auto start_used = key.start_ + start;
    const auto end_used = key.start_ + len;
    const auto key_end = str_end(key);
    return KeyInternal(
      (start_used < key_end) ? start_used : key_end,
      (end_used < key_end) ? end_used : key_end);
  }
 
  std::string debug_key(const KeyInternal& key, std::size_t pos) const {
    const auto key_start = key.start_ + pos;
    const auto key_end = str_end(key);
    if (key_end <= key_start) {
      return std::string();
    }

    return std::string(key_start, key_end);
  }
  std::string debug_key(const KeyInternal& key) const {
    return debug_key(key, 0);
  }

  static void debug_print_indent(std::size_t indent) {
    for (std::size_t i = 0; i < indent; ++i) {
      std::cout << ' ';
    }
  }

  void debug_print(const Node* node, std::size_t indent) const {
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
  KeyIterator end_;
};

#endif // MURRAYC_SUFFIX_TREE_SUFFIX_TREE_H
