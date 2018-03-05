#ifndef MURRAYC_SUFFIX_TREE_SUFFIX_TREE_H
#define MURRAYC_SUFFIX_TREE_SUFFIX_TREE_H

#include "iter_range.h"
#include <algorithm>
#include <iostream>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <set>
#include <stack>
#include <cassert>

/**
 * @tparam T_Key For instance, std::string, or something other container.
 * @tparam T_Value The value to associate with each inserted key.
 */
template <typename T_Key, typename T_Value>
class SuffixTree {
public:
  SuffixTree() {
  }

  /// Start and end (1 past last position) of a substring in text_;
  using KeyIterator = typename T_Key::const_iterator;

  using Range = IterRange<KeyIterator>;

  SuffixTree(const T_Key& key, const T_Value& value) {
    const auto start = std::cbegin(key);
    const auto end = start + key.size();
    const Range substr(start, end);
    if (str_empty(substr)) {
      return;
    }

    insert_ukkonen(substr, value);
  }

  /**
   * The suffix's begin/end, and the associated value.
   */
  using suffix_array_type = std::vector<std::pair<Range, T_Value>>;

  using lcp_array_type = std::vector<std::size_t>;

  /**
   * Construct a suffix tree based on a suffix array and LCP array.
   */
  SuffixTree(const suffix_array_type& suffixes, const lcp_array_type& lcp_array) {
    if (suffixes.empty()) {
      return;
    }

    insert_sa_and_lcp_array(suffixes, lcp_array);
  }

  void insert(const T_Key& key, const T_Value& value) {
    const auto start = std::cbegin(key);
    const auto end = start + key.size();
    const Range substr(start, end);
    if (str_empty(substr)) {
      return;
    }

    insert(substr, value);
  }

  void insert(const typename T_Key::const_iterator& start, const typename T_Key::const_iterator& end, const T_Value& value) {
    const Range substr(start, end);
    if (str_empty(substr)) {
      return;
    }

    insert(substr, value);
  }

  using Matches = std::set<T_Value>;

  /** Finds the values for any key containing this substring.
   */
  Matches find(const T_Key& substr) const {
    Matches result;

    if (substr.empty()) {
      return result;
    }


    const auto start = std::cbegin(substr);
    const auto end = start + substr.size();
    const Range substr_key(start, end);

    const auto matches_with_positions = find_with_positions(substr_key);

    // Convert one container into another:
    for (const auto& kv : matches_with_positions) {
      result.emplace(kv.second);
    }

    return result;
  }

  /**
   * Like Matches, but provides the range of the prefix too,
   * so the caller can know where in the original insert()ed string,
   * the substring was found. That would refer to the originally-inserted
   * string, but we already require the caller to keep that alive.
   */
  using MatchesWithPositions = std::vector<std::pair<Range, T_Value>>;

  /** Finds the values for any key containing this substring.
   */
  MatchesWithPositions find_with_positions(const T_Key& substr) const {
    MatchesWithPositions result;

    if (substr.empty()) {
      return result;
    }


    const auto start = std::cbegin(substr);
    const auto end = start + substr.size();
    const Range substr_key(start, end);
    return find_with_positions(substr_key);
  }

  void debug_print() const {
    std::cout << "Tree:" << std::endl;
    debug_print(&root_, 0);
    std::cout << std::endl << std::endl;
  }

  /** Get the suffix array and the LCP array.
   * The LCP array is the length of the common prefix of an item compared to the previous
   * item in the suffix array. Therefore there is no LCP value for the first suffix in the
   * suffix array.
   */
  std::pair<suffix_array_type, lcp_array_type>
  get_suffix_array_and_lcp_array() const {
    // Build a suffix array by doing a lexographically-ordered DFS:
    suffix_array_type suffixes;
    lcp_array_type lcp;

    // The node and its ancestor:
    std::stack<std::pair<const Node*, path_type>> s;
    s.emplace(&root_, path_type());
    path_type previous_path;
    while (!s.empty()) {
      const auto node_and_ancestor = s.top();
      s.pop();
      const auto& node = node_and_ancestor.first;
      const auto path = node_and_ancestor.second;
      const std::size_t depth = path.empty() ? 0 : path.back().second;

      if (node->has_value()) {
        for (auto& kv : node->keys_and_values_) {
          suffixes.emplace_back(kv.first, kv.second);
          if (!previous_path.empty()) {
            // TODO: Just calling std::mismatch() on the strings would be much simpler,
            // and would be just as linear (O(h), where h is the height of the tree).
            // Is it worth keeping these paths in the stack, just to
            // avoid a few extra character compares?
            const auto common_depth = depth_of_lca(previous_path, path);
            //std::cout << "lcp: " << common_depth << std::endl;
            lcp.emplace_back(common_depth);
          }
          previous_path = path;
        }
      }

      // Reverse sort this child edges,
      // so we can put the lexographically-first on the stack last,
      // so we deal with it first:
      auto children = node->children_;
      std::sort(std::begin(children), std::end(children),
        [](auto a, auto b) {
          // The first characters of the edges will always be different:
          const auto& achar = *(a.part_.start_);
          const auto& bchar = *(b.part_.start_);
          return bchar < achar;
        });

      for (auto edge : children) {
        const auto& edge_part = edge.part_;
        const auto& d = edge.dest_;

        auto new_path = path;
        const auto substr_len = str_size(edge_part);
        new_path.emplace_back(d, depth + substr_len);
        s.emplace(d, new_path);
      }
    }

    return std::make_pair(suffixes, lcp);
  }

  inline static KeyIterator str_end(const Range& key) {
    return key.end();
  }

private:

  class Node {
  public:
    Node() = default;

    ~Node() {
      for(auto& edge : children_) {
        delete edge.dest_;
      }
    }

    class Edge {
    public:
      Edge(const Range& part, Node* dest)
        : part_(part),
          dest_(dest) {
        assert(str_size(part));
      }

      void append_node_to_dest(const Range& part, const Range& key, const T_Value& value) {
        dest_->append_node(part, key, value);
      }

      inline bool dest_has_value() const {
        if (!dest_) {
          return false;
        }

        return dest_->has_value();
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

      Range part_;
      Node* dest_ = nullptr;
    };

    Node* append_node(const Range& part, const Range& key, const T_Value& value) {
      const auto extra_node = new Node();
      extra_node->keys_and_values_.emplace_back(key, value);
      children_.emplace_back(part, extra_node);
      return extra_node;
    }

    void append_node(const Range& part, Node* node) {
      children_.emplace_back(part, node);
    }

    inline bool has_value() const {
      return !keys_and_values_.empty();
    }

    inline void add_value(const Range& key, const T_Value& value) {
      keys_and_values_.emplace_back(std::make_pair(key, value));
    }

    Edge* find_edge_starting_with(const KeyIterator& iter) {
      const auto& ch = *iter;
      for (auto& edge : children_) {
        const auto& start_ch = *(edge.part_.start_);
        if (ch == start_ch) {
          return &edge;
        }
      }

      return nullptr;
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
    // first: The start and end of the actual suffix that this leaf node represents:
    // second: The value associated with the string for which this leaf node represents a suffix:
    std::vector<std::pair<Range, T_Value>> keys_and_values_;
  };

  void insert(const Range& key, const T_Value& value) {
    //std::cout << "debug: insert(): key.start_=" << static_cast<const void*>(key.start_) << ", second=" << static_cast<const void*>(key.end_) << std::endl;
    //Insert every suffix of the key:
    Range suffix = key;
    while(!str_empty(suffix)) {
      //std::cout << "insert(): suffix=" << suffix << ", value=" << value <<std::endl;
      insert_single(suffix, value);

      // Remove the first character:
      suffix = str_substr(suffix, 1);
      //std::cout << "suffix: first=" << static_cast<const void*>(suffix.start_) << ", second=" << static_cast<const void*>(suffix.end_) << std::endl;
    }
  }

  /** Finds the values for any key containing this substring.
   */
  MatchesWithPositions find_with_positions(const Range& substr) const {
    MatchesWithPositions result;

    if (str_empty(substr)) {
      return result;
    }

    const auto start = find_partial_edge(substr);
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
        result.insert(std::end(result), std::cbegin(node->keys_and_values_),
          std::cend(node->keys_and_values_));

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

  class ActivePoint {
  public:
    Node* node = nullptr;
    bool edge_valid = false;
    KeyIterator edge; // Instead of starting with -1, we set active.edge_valid to false.
    std::size_t length = 0;
  };

  void insert_ukkonen(const Range& key, const T_Value& value) {
    // This code won't work unless the tree is empty:
    assert(root_.children_.empty());

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

    std::unordered_map<Node*, Node*> suffix_links;

    // The "phases"
    for (auto i = key_start; i != key_end; ++i) {
      //std::cout << "  character: " << *i << std::endl;
      const bool is_last_char = (i == key_last);

      ++remaining;
      ++end; //This extends all existing paths by one character.

      Node* prev_created_internal_node = nullptr;

      // The "extensions".
      while(remaining) {

        //std::cout << "    remaining: " << remaining << std::endl;
        //std::cout << "    end: " << std::distance(key_start, end) << std::endl;
        //std::cout << "    active.node: " << active.node << std::endl;
        if (active.edge_valid) {
          //std::cout << "    active.edge: " << std::distance(key_start, active.edge) << std::endl;
          //std::cout << "    active.length: " << active.length << std::endl;
        }

        // An active.length of 0 means we ignore the active.edge.
        const auto edge_match = (active.edge_valid && active.length) ?
          find_partial_edge(active, i) :
          find_partial_edge(active.node, i);
        auto edge = edge_match.edge_;
        const auto part_len_used = edge_match.edge_part_used_;
        const bool prefix_used = edge_match.substr_used_ > 0;

        if (!prefix_used || is_last_char) {
          Range prefix(i, end_ptr);
          Range actual_prefix(i - remaining + 1, end_ptr);

          // Rule 2 extension: There is no match:
          if (part_len_used == 0) {
            // There is no match:
            //std::cout << "      Rule 2: Adding edge to active node: " << debug_key(prefix) << std::endl;
            active.node->append_node(prefix, actual_prefix, value);
          } else {
            // There is a partial match, in the middle of an edge:
            //std::cout << "      Rule 2: Splitting edge " << debug_key(edge->part_)
            //  << " at " << part_len_used - 1
            //  << " and adding: " << debug_key(prefix)
            //  << " for prefix: " << debug_key(actual_prefix) << "(" << std::distance(key_start, actual_prefix.start_) << ")" << std::endl;
            auto extra_node = edge->split(part_len_used);
            if (is_last_char) {
              // Just let the intermediate node have a value,
              // instead of having extra leaf nodes just for values.
              extra_node->add_value(actual_prefix, value);
            } else {
              extra_node->append_node(prefix, actual_prefix, value);
            }

            // Every internal node should have a suffix link:
            //std::cout << "      Setting suffix link from " << extra_node << " to root " << &root_ << std::endl;
            suffix_links[extra_node] = &root_;

            // A previously-created internal node should now have its suffix link
            // updated to this new internal node.
            if (prev_created_internal_node) {
              //std::cout << "      Updating suffix link from " << prev_created_internal_node << " to " << extra_node << std::endl;
              suffix_links[prev_created_internal_node] = extra_node;
            }
            prev_created_internal_node = extra_node;

            // Follow previous suffix link if the active node is not root:
            if (active.node != &root_) {
              const auto iter = suffix_links.find(active.node);
              assert(iter != suffix_links.end());
              //std::cout << "      Following suffix link of active node " << active.node << " to " << iter->second << std::endl;
              active.node = iter->second;

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
        //std::cout << "      Rule 3: Do nothing." << std::endl;

        active.node = edge_match.parent_node_;
        active.edge = edge->part_.start_; //Start of range of the existing edge.
        active.edge_valid = true;
        active.length = part_len_used;

        // After a rule 3 extension, immediately start the next "phase".
        break;
      }
    }

    // Update all the ranges to use real end values
    // instead of one shared end value.
    set_global_ends();
  }

  /** Update all ranges (in edge parts and in nodes),
   * to stop using the shared_ptr<>.
   * This makes subsequent changes and comparisons simpler.
   */
  void set_global_ends() {
    std::stack<Node*> s;
    s.emplace(&root_);
    while (!s.empty()) {
      auto node = s.top();
      s.pop();

      for (auto& p : node->keys_and_values_) {
        p.first.set_end_from_global();
      }

      for (auto& e : node->children_) {
        e.part_.set_end_from_global();
        s.emplace(e.dest_);
      }
    }
  }

  void insert_sa_and_lcp_array(const suffix_array_type& suffixes, const lcp_array_type& lcp_array) {
    assert(suffixes.size() == lcp_array.size() + 1);

    // This code won't work unless the tree is empty:
    assert(root_.children_.empty());

    // Add the first suffix, then subsequent suffixes,
    // keeping the path to the currently-added leaf node,
    // so we can travel back up it when creating the next leaf node,
    // to split at the specified lcp.

    // The parent nodes and their depths:
    std::stack<std::pair<Node*, std::size_t>> path;

    // Add the first suffix:
    auto i = std::cbegin(suffixes);
    {
      const auto& suffix = i->first;
      const auto& value = i->second;
      root_.append_node(suffix, suffix /* key */, value);
      path.emplace(std::make_pair(&root_, 0));
    }

    auto l = std::cbegin(lcp_array);
    for (++i; i != std::cend(suffixes); ++i, ++l) {
      const auto& suffix = i->first;

      const auto& value = i->second;
      const auto lcp = *l;
      //std::cout << debug_key(suffix) << ": lcp=" << lcp << std::endl;

      // Find the parent node at, or higher than, the lcp:
      Node* parent = nullptr;
      std::size_t depth = 0;
      while (!path.empty()) {
        const auto p = path.top();
        parent = p.first;
        depth = p.second;
        if (depth <= lcp) {
          // Use it, without popping it:
          break;
        }

        path.pop();
      }

      // Split if necessary:
      const auto suffix_part = str_substr(suffix, lcp);
      Node* node = nullptr;
      if (depth == lcp) {
        // Just add the end of the suffix to the parent node:
        node = parent->append_node(suffix_part, suffix /* key */, value);
      } else {
        // Split the parent node's edge at the appropriate place,
        // and add the new node from the split:
        const auto iter = suffix.start_ + depth;
        auto edge = parent->find_edge_starting_with(iter);
        /*
        if (!edge) {
          std::cerr << "Cannot find edge beginning with " << *iter
            << " from node with depth: " << depth << std::endl;
        }
        */
        assert(edge);

        // Split it:
        const auto split_node = edge->split(lcp - depth);
        path.emplace(std::make_pair(split_node, lcp));
        node = split_node->append_node(suffix_part, suffix /* key */, value);
      }

      path.emplace(std::make_pair(node, str_size(suffix)));
    }
  }

  static
  bool has_prefix(const Range& str, std::size_t str_start_pos, const Range& prefix, std::size_t prefix_start_pos = 0) {
    return str.has_prefix(str_start_pos, prefix, prefix_start_pos);
  }

  static
  std::size_t common_prefix(const Range& str, std::size_t str_start_pos, const Range& prefix, std::size_t prefix_start_pos) {
    return str.common_prefix(str_start_pos, prefix, prefix_start_pos);
  }

  void insert_single(const Range& key, const T_Value& value) {
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
      node->keys_and_values_.emplace_back(key, value);
      return;
    }

    // Add a node for the remaining characters:
    const auto suffix = str_substr(key, key_pos);
    //std::cout << "Adding suffix: " << suffix << ", with value: " << value << '\n';

    node->append_node(suffix, key, value);
  }

  /**
   * The Edge and the end of matching prefix of the edge's part.
   */
  class EdgeMatch {
  public:
    EdgeMatch() {
    }

    EdgeMatch(typename Node::Edge* edge, std::size_t edge_part_used, std::size_t substr_used, Node* parent_node)
    : edge_(edge), edge_part_used_(edge_part_used), substr_used_(substr_used), parent_node_(parent_node) {
    }

    EdgeMatch(const typename Node::Edge* edge, std::size_t edge_part_used, std::size_t substr_used, const Node* parent_node)
    : edge_(const_cast<typename Node::Edge*>(edge)), edge_part_used_(edge_part_used), substr_used_(substr_used), parent_node_(const_cast<Node*>(parent_node)) {
    }

    typename Node::Edge* edge_ = nullptr;
    std::size_t edge_part_used_ = 0;
    std::size_t substr_used_ = 0;
    Node* parent_node_ = nullptr;
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

  /** Returns the edge and how much of the edge's part represents the @a substr.
   */
  EdgeMatch find_partial_edge(const Range& substr) const {
    return find_partial_edge(&root_, substr);
  }

  /** Returns the edge and how much of the edge's part represents the @a substr.
   */
  EdgeMatch find_partial_edge(const Node* start_node, const Range& substr) const {
    //std::cout << "find_partial_edge(): substr=" << debug_key(substr) << std::endl;
    EdgeMatch result;

    if (str_empty(substr)) {
      return result;
    }

    const auto substr_len = str_size(substr);

    const Node* node = start_node;
    std::size_t substr_pos = 0;
    const typename Node::Edge* parent_edge = nullptr;
    std::size_t parent_edge_len_used = 0;
    while (node) {
      bool edge_found = false;
      for (auto& edge : node->children_) {
        const auto& edge_part = edge.part_;

        const auto len = common_prefix(substr, substr_pos, edge_part, 0);
        if (len == 0) {
          continue;
        }

        const auto substr_remaining_len = substr_len - substr_pos;
        //std::cout << "      substr_remaining_len=" << substr_remaining_len << std::endl;
        if (len == str_size(edge_part)) {
          // The remaining substr has edge_part as a prefix.
          if (len == substr_remaining_len) {
            // And that uses up all of our substr:
            return EdgeMatch(&edge, len, substr_len, node);
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
          return EdgeMatch(&edge, len, substr_len, node);
        } else {
          // The edge has some of the remaining substr as its prefix.
          return EdgeMatch(&edge, len, substr_pos + len, node);
        }
      }

      if (!edge_found) {
        break;
      }
    }

    //std::cout << "  returning parent_edge=" << static_cast<void*>(parent_edge) <<
      //"parent_edge_len_used=" << parent_edge_len_used <<
      //"substr_pos=" << substr_pos << std::endl;
    return EdgeMatch(parent_edge, parent_edge_len_used, substr_pos, node);
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
          return EdgeMatch(edge, edge_part_pos, 0, parent_node);
        }

        //Try again at the start of the followed edge:
        edge_part_pos = 0;
        continue;
      }

      if (*part_next == *next_char) {
        return EdgeMatch(edge, edge_part_pos + 1, 1, parent_node);
      }

      return EdgeMatch(edge, edge_part_pos, 0, parent_node);
    }
  }

  static
  inline std::size_t str_size(const Range& key) {
    return key.size();
  }

  static
  inline bool str_empty(const Range& key) {
    return key.empty();
  }

  static
  inline Range str_substr(const Range& key, std::size_t start) {
    return key.substr(start);
  }

  static
  inline Range str_substr(const Range& key, std::size_t start, std::size_t len) {
    return key.substr(start, len);
  }

  static std::string debug_key(const Range& key) {
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
        for (const auto value : edge.dest_->keys_and_values_) {
          if (!first) {
            std::cout << ", ";
          }
          std::cout << value.second;
          first = false;
        }
        std::cout << ")";
      }
      std::cout << std::endl;

      debug_print(edge.dest_, indent + str_size(edge.part_));
    }
  }

  using node_and_depth_type = std::pair<const Node*, std::size_t>;
  using path_type = std::vector<node_and_depth_type>;

  static std::size_t
  depth_of_lca(const path_type& a, const path_type& b) {
    // Find the first mismatching nodes in the paths:
    const auto p = std::mismatch(std::cbegin(a), std::cend(a),
      std::cbegin(b), std::cend(b));
    if (p.first == std::cend(a)) {
      assert("No common ancestor");
    }

    if (p.first == std::cbegin(a)) {
      return 0;
    }

    // Get the node just before the mismatch:
    auto iter = p.first;
    iter--;
    return iter->second;
  }

  Node root_;
};

#endif // MURRAYC_SUFFIX_TREE_SUFFIX_TREE_H
