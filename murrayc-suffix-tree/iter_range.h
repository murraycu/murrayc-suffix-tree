#ifndef MURRAYC_SUFFIX_TREE_ITER_RANGE_H 
#define MURRAYC_SUFFIX_TREE_ITER_RANGE_H

#include <memory>

template <typename T_Iterator>
class IterRange {
public:
  IterRange() = default;

  IterRange(const T_Iterator& start, const T_Iterator& end)
  : start_(start), end_(end) {
  }

  IterRange(const T_Iterator& start, const std::shared_ptr<const T_Iterator>& end)
  : start_(start), global_end_(end) {
  }

  IterRange(const IterRange& src) = default;
  IterRange& operator=(const IterRange& src) = default;
  IterRange(IterRange&& src) = default;
  IterRange& operator=(IterRange&& src) = default;

  /*
  static char
  debug_global_end(const std::shared_ptr<const T_Iterator>& end) {
    if (!end) {
      return '!';
    }

    T_Iterator iter = *end;
    return (iter == T_Iterator() ? '?' : *iter);
  }
  */

  bool operator==(const IterRange& src) const {
    return start_ == src.start_ &&
      end_ == src.end_ &&
      global_end_ == src.global_end_;
  }

  inline bool empty() const {
    return start_ >= end();
  }

  inline T_Iterator end() const {
    if (global_end_) {
      return *(global_end_);
    }

    return end_;
  }

  inline std::size_t size() const {
    const auto e = end();
    if (e <= start_) {
      return 0;
    }

    return e - start_;
  }

  inline IterRange substr(std::size_t start) const {
    const auto start_used = start_ + start;
    const auto key_end = end();
    return IterRange(
      (start_used < key_end) ? start_used : key_end,
      key_end);
  }

  inline IterRange substr(std::size_t start, std::size_t len) const {
    const auto start_used = start_ + start;
    const auto end_used = start_ + len;
    const auto key_end = end();
    return IterRange(
      (start_used < key_end) ? start_used : key_end,
      (end_used < key_end) ? end_used : key_end);
  }

  bool has_prefix(std::size_t str_start_pos, const IterRange& prefix, std::size_t prefix_start_pos = 0) const {
    const auto prefix_start = prefix.start_ + prefix_start_pos;
    const auto prefix_end = prefix.end();
    const auto iters = std::mismatch(start_ + str_start_pos, end(),
        prefix_start, prefix_end);
    return iters.second == prefix_end;
  }

  std::size_t common_prefix(std::size_t str_start_pos, const IterRange& prefix, std::size_t prefix_start_pos = 0) const {
    const auto str_start = start_ + str_start_pos;
    const auto iters = std::mismatch(str_start, end(),
        prefix.start_ + prefix_start_pos, prefix.end());
    return std::distance(str_start, iters.first);
  }
  /** Make sure that the range has its own fixed end value,
   * instead of sharing a variable end value.
   */
  void set_end_from_global() {
    if (!global_end_) {
      end_ = T_Iterator();
      return;
    }

    end_ = *global_end_;

    global_end_.reset();
  }

  std::string to_string() const {
    if (global_end_) {
      return std::string(start_, *global_end_);
    } else {
      return std::string(start_, end_);
    }
  }

  T_Iterator start_;
  T_Iterator end_;
  std::shared_ptr<const T_Iterator> global_end_;
};

#endif // MURRAYC_SUFFIX_TREE_ITER_RANGE_H
