#ifndef MURRAYC_SUFFIX_TREE_ITER_RANGE_H 
#define MURRAYC_SUFFIX_TREE_ITER_RANGE_H

template <typename T_Iterator>
class IterRange {
public:
  IterRange() = default;

  IterRange(const T_Iterator& start, const T_Iterator& end)
  : start_(start), end_(end) {
    }

  IterRange(const IterRange& src) = default;
  IterRange& operator=(const IterRange& src) = default;
  IterRange(IterRange&& src) = default;
  IterRange& operator=(IterRange&& src) = default;

  bool operator==(const IterRange& src) const {
    return start_ == src.start_ &&
      end_ == src.end_;
  }

  T_Iterator start_;
  T_Iterator end_;
};

#endif // MURRAYC_SUFFIX_TREE_ITER_RANGE_H
