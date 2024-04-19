// Yoinked from https://stackoverflow.com/a/50978188/1148866

#ifndef PATHFINDER_HASH_COMBINE_H_
#define PATHFINDER_HASH_COMBINE_H_

#include <cstdint>
#include <limits>
#include <type_traits>

namespace pathfinder {
namespace internal {

// a hash function with another name as to not confuse with std::hash
uint32_t distribute(const uint32_t& n);
// a hash function with another name as to not confuse with std::hash
uint64_t distribute(const uint64_t& n);

template<typename T>
T xorshift(const T& n, int i){
  return n ^ (n>>i);
}

// if c++20 rotl is not available:
template <typename T,typename S>
typename std::enable_if<std::is_unsigned<T>::value,T>::type
constexpr rotl(const T n, const S i){
  const T m = std::numeric_limits<T>::digits - 1;
  const T c = i & m;
  return (n << c) | (n >> ((T{0} - c) & m)); // this is usually recognized by the compiler to mean rotation, also c++20 now gives us rotl directly
}

} // namespace internal

// call this function with the old seed and the new key to be hashed and combined into the new seed value, respectively the final hash
template <class T>
inline size_t hash_combine(std::size_t& seed, const T& v) {
  return internal::rotl(seed, std::numeric_limits<size_t>::digits/3) ^ internal::distribute(std::hash<T>{}(v));
}

} // namespace pathfinder

#endif // PATHFINDER_HASH_COMBINE_H_