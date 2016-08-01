#include <cstdlib>
#include <iostream>
#include <vector>
#include <cassert>

class Trie {
public:
  bool exists(const std::string& key) {
    const auto node = find_node(key);
    return node != nullptr;
  };

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
        for (const auto& child : node->children) {
          if (child->part == ch) {
            next = child;
            break;
          }
        }
      }

      // Add a new node, if necessary:
      if (!next) {
        appending = true;
        next = new Node;
        next->part = ch;
        node->children.emplace_back(next);
      }

      node = next;
    }
  }

private:
  class Node {
  public:
    char part = ' ';
    std::vector<Node*> children;
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
      for (const auto& child : node->children) {
        if (child->part == ch) {
          next = child;
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

  return EXIT_SUCCESS;
}

