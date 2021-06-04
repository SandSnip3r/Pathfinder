#ifndef PATHFINDER_BINARY_BIN_HEAP_H_
#define PATHFINDER_BINARY_BIN_HEAP_H_

#include <algorithm>
#include <functional>
#include <stdexcept>
#include <unordered_map>
#include <vector>

namespace pathfinder {

/**
 * BinaryMinHeap
 *
 * A template class to represent a min-heap.
 * @tparam KeyType     The hashable type of the item stored in the
 *                       BinaryMinHeap
 * @tparam ValueType   The type of the item stored in the BinaryMinHeap
 * @tparam GreaterComp (optional) A comparison function for ValueType to
 *                       enforce the ordering of the heap
 */
template <typename KeyType, typename ValueType, typename GreaterComp = std::greater<ValueType>>
class BinaryMinHeap {
 public:
  /**
   * push_back()  Adds an item onto the heap
   * 
   * @param key   The key of the item being added
   * @param val   The item being added
   */
  void push_back(const KeyType &key, const ValueType& val) {
    heapData_.push_back(val);
    heapLocationLookup_[key] = heapData_.size() - 1;
  }

  /**
   * top()     Returns the minimum element of the heap
   * 
   * @return   The minimum element of the heap
   */
  ValueType& top() { return heapData_[0]; }
  const ValueType& top() const { return heapData_[0]; }

  /**
   * pop()     Deletes the minimum element of the heap and maintains the heapyness
   */
  void pop() {
    if (empty()) {
      return;
    }
    // Swap first element with last
    std::swap(heapData_.front(), heapData_.back());
    std::swap(heapLocationLookup_.at(static_cast<KeyType>(heapData_.front())),
              heapLocationLookup_.at(static_cast<KeyType>(heapData_.back())));
    // Remove location lookup for this element (now at the back)
    auto lookupLocationEraseResult = heapLocationLookup_.erase(static_cast<KeyType>(heapData_.back()));
    if (lookupLocationEraseResult != 1) {
      throw std::runtime_error("BinaryMinHeap: Tried to erase location lookup while popping, but failed");
    }
    // Delete last element
    heapData_.pop_back();
    // Bubble first down until it is in position
    size_t self_pos = 0;
    // While we have at least one child
    while (self_pos * 2 + 1 < size()) {
      size_t left_child_pos = self_pos * 2 + 1;
      auto& left_child = heapData_[left_child_pos];
      size_t smaller_child_pos = left_child_pos;
      if (self_pos * 2 + 2 < size() && GreaterComp()(left_child, heapData_[self_pos * 2 + 2])) {
        // Have right_child and right_child is smaller than left_child
        smaller_child_pos = self_pos * 2 + 2;
      }
      auto& smaller_child = heapData_[smaller_child_pos];
      auto& self = heapData_[self_pos];
      if (GreaterComp()(self, smaller_child)) {
        // Self is greater than child, swap
        std::swap(self, smaller_child);
        std::swap(heapLocationLookup_.at(static_cast<KeyType>(self)),
                  heapLocationLookup_.at(static_cast<KeyType>(smaller_child)));
        // Set self as child
        self_pos = smaller_child_pos;
      } else {
        // Children are both greater
        // Done
        break;
      }
    }
  }

  /**
   * size()    Returns the number of elements in the min heap
   * 
   * @return   The number of elements in the min heap
   */
  size_t size() const { return heapData_.size(); }

  /**
   * contains()   Returns if min heap contains the key
   * 
   * @param key   The key of the item to look for
   * @return      `true` if the key exists in the min heap, `false` otherwise
   */
  bool contains(const KeyType &key) const {
    return heapLocationLookup_.find(key) != heapLocationLookup_.end();
  }

  /**
   * empty()   Returns if the min heap is empty
   * 
   * @return   `true` if the min heap is empty, `false` otherwise
   */
  bool empty() const { return heapData_.empty(); }

  /**
   * bubbleUp()   Moves an item up to its correct location in the min heap
   * 
   * @param key   The key of the item to re-adjust
   * Attention: This is usually called by the user when the user decreases
   *   the value of the data in this container via an external reference
   */
  void bubbleUp(const KeyType &key) {
    // NOTE: key is not guaranteed to exist, that is the user's responsibility
    size_t selfPos = heapLocationLookup_.at(key);
    // While a parent exists and we are less than the parent
    while (selfPos != 0 && GreaterComp()(heapData_[(selfPos - 1) / 2], heapData_[selfPos])) {
      size_t parent_pos = (selfPos - 1) / 2;
      auto& parent = heapData_[parent_pos];
      auto& self = heapData_[selfPos];
      // Swap parent an ourselves
      std::swap(parent, self);
      // Update location lookup
      std::swap(heapLocationLookup_.at(static_cast<KeyType>(parent)),
                heapLocationLookup_.at(static_cast<KeyType>(self)));
      selfPos = parent_pos;
    }
  }

 private:
  std::vector<ValueType> heapData_;
  std::unordered_map<KeyType, size_t> heapLocationLookup_;
};

}  // namespace pathfinder

#endif  // PATHFINDER_BINARY_BIN_HEAP_H_
