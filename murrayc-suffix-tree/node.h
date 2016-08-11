#ifndef MURRAYC_SUFFIX_TREE_NODE_H
#define MURRAYC_SUFFIX_TREE_NODE_H

#include <stack>

class Node {
public:
  Node() = default;
  Node(const Node& src) = default;
  Node& operator=(const Node& src) = default;
  Node(Node&& src) = default;
  Node& operator=(Node&& src) = default;

  explicit Node(const std::string& data, Node* left = nullptr, Node* right = nullptr)
  : left_(left),
    right_(right),
    data_(data) {
    if (left_) {
      left_->parent_ = this;
    }

    if (right_) {
      right_->parent_ = this;
    }
  }

  Node* parent_ = nullptr;
  Node* left_ = nullptr;
  Node* right_ = nullptr;

  std::string data_;
};

Node* construct_example_tree() {
  auto ll = new Node("m");
  auto lr = new Node("r");
  auto l = new Node("u", ll, lr);
  auto rl = new Node("a");
  auto r = new Node("y", rl);
  return new Node("r", l, r);
}

/*
     m
    / \
   a   u
      / \
     r   y
*/

Node* construct_example_binary_search_tree() {
  auto l = new Node("a");
  auto rl = new Node("r");
  auto rr = new Node("y");
  auto r = new Node("u", rl, rr);
  return new Node("m", l, r);
}

void
release_tree(Node* root) {
  if (!root) {
    return;
  }

  std::stack<Node*> stack;
  stack.emplace(root);
  while (!stack.empty()) {
    auto node = stack.top();
    stack.pop();

    if (node->left_) {
      stack.emplace(node->left_);
    }

    if (node->right_) {
      stack.emplace(node->right_);
    }

    delete node;
  }
}

bool
is_right_child(Node* node) {
  if (!node) {
    return false;
  }

  return node->parent_ &&
    (node->parent_->right_ == node);
}

#endif //MURRAYC_SUFFIX_TREE_NODE_H
