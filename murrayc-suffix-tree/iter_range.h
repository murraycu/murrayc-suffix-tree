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

  bool operator==(const IterRange& src) const {
    return start_ == src.start_ &&
      end_ == src.end_ &&
      global_end_ == src.global_end;
  }

  T_Iterator start_;
  T_Iterator end_;
  std::shared_ptr<const T_Iterator> global_end_;
};

#endif // MURRAYC_SUFFIX_TREE_ITER_RANGE_H
