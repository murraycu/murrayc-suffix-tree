#include <cstdlib>
#include <iostream>
#include <vector>
#include <cassert>
#include <stack>
#include <utility>

class Trie {
public:
  bool exists(const std::string& key) {
    const auto node = find_node(key);
    return node != nullptr;
  };

  std::vector<std::string> find_candidates(const std::string& prefix) {
    if (prefix.empty()) {
      return {};
    }

    auto prefix_node = find_node(prefix);
    if (!prefix_node) {
      return {};
    }

    std::vector<std::string> result;

    //Stack of prefixes+nodes.
    std::stack<std::pair<std::string, Node*>> stack;
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

  void insert(const std::string& key) {
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
        Node::Edge edge;
        edge.part = ch;
        edge.dest = next;
        node->children.emplace_back(edge);
      }

      node = next;
    }
  }

private:
  class Node {
  public:
    class Edge {
    public:
      char part = 0;
      Node* dest = nullptr;
    };

    std::vector<Edge> children;
  };

  Node* find_node(const std::string& key) {
    //std::cout << "find_node: key=" << key << '\n';
    if (key.empty()) {
      return nullptr;
    }

    auto node = &root;
    for (const auto ch : key) {
      //std::cout << ch << '\n';

      //Choose the child node, if any:
      Node* next = nullptr;
      for (const auto& edge : node->children) {
        if (edge.part == ch) {
          next = edge.dest;
          break;
        }
      }

      if (!next) {
        //std::cout << "  not found.\n";
        return nullptr;
      }

      node = next;
    }

    return node;
  }

  Node root;
};

int main() {

  Trie trie;
  trie.insert("banana");
  trie.insert("bandana");
  trie.insert("foo");
  trie.insert("foobar");

  assert(trie.exists("banana"));
  assert(trie.exists("foo"));
  assert(!trie.exists("foop"));

  const auto candidates = trie.find_candidates("ban");
  //for (const auto candidate : candidates) {
  //  std::cout << "candidate: " << candidate << '\n';
  //}

  // TODO: Check wthout caring about the order:
  const auto expected_candidates = std::vector<std::string>({"bandana", "banana"});

  assert(candidates == expected_candidates);

  return EXIT_SUCCESS;
}

