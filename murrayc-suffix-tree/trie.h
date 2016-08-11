#ifndef MURRAYC_SUFFIX_TREE_TRIE_H
#define MURRAYC_SUFFIX_TREE_TRIE_H

#include <cstdlib>
#include <iostream>
#include <vector>
#include <cassert>
#include <stack>
#include <utility>

template <typename T_Key, typename T_Value>
class Trie {
public:
  bool exists(const T_Key& key) const {
    const auto node = find_node(key);
    return node != nullptr;
  };

  /**
   * Returns T_Value() if the key was not found.
   */
  T_Value get_value(const T_Key& key) const {
    const auto node = find_node(key);
    return node ? node->value : T_Value();
  }

  std::vector<T_Key> find_candidates(const T_Key& prefix) {
    if (prefix.empty()) {
      return {};
    }

    const auto prefix_node = find_node(prefix, false /* not just leaves */);
    if (!prefix_node) {
      return {};
    }

    std::vector<T_Key> result;

    //Stack of prefixes+nodes.
    std::stack<std::pair<T_Key, const Node*>> stack;
    stack.emplace(prefix, prefix_node);

    while (!stack.empty()) {
      const auto item = stack.top();
      stack.pop();

      const auto& node = item.second;
      if (node->children.empty()) {
        result.emplace_back(item.first);
      }

      for (auto edge : node->children) {
        stack.emplace(item.first + edge.part, edge.dest);
      }
    }

    return result;
  }

  void insert(const T_Key& key, const T_Value& value) {
    if (key.empty()) {
      return;
    }

    auto node = &root;
    auto appending = false;
    for (const auto ch : key) {
      Node* next = nullptr;

      if (!appending) {
        //Choose the child node, if any:
        for (const auto& edge : node->children) {
          if (edge.part == ch) {
            next = edge.dest;
            break;
          }
        }
      }

      // Add a new node, if necessary:
      if (!next) {
        appending = true;
        next = new Node;
        typename Node::Edge edge;
        edge.part = ch;
        edge.dest = next;
        node->children.emplace_back(edge);
      }

      node = next;
    }

    node->is_leaf = true;
    node->value = value;
  }

private:
  class Node {
  public:
    class Edge {
    public:
      char part = 0;
      Node* dest = nullptr;
    };

    //We could instead have a std::vector<Node*> children,
    //of size alphabet (such as 26),
    //to allow O(1) lookup, at the cost of wasted space.
    std::vector<Edge> children;

    // TODO: Wastes space on non-leaves.
    bool is_leaf = false;
    T_Value value = 0;
  };

  const Node* find_node(const T_Key& key, bool leaf_only = true) const {
    if (key.empty()) {
      return nullptr;
    }

    auto node = &root;
    for (const auto ch : key) {

      //Choose the child node, if any:
      Node* next = nullptr;
      for (const auto& edge : node->children) {
        if (edge.part == ch) {
          next = edge.dest;
          break;
        }
      }

      if (!next) {
        return nullptr;
      }

      node = next;
    }

    return (!leaf_only || node->is_leaf) ? node : nullptr;
  }

  Node root;
};

#endif // MURRAYC_SUFFIX_TREE_TRIE_H
