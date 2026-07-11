/*
 * @file alloc_probe.hpp
 * @date 2026-07-11
 * @brief RAII allocation counter for the ingestion benchmark suite
 *        (docs/ingestion_contract.md, requirement I2).
 *
 * The family S1 methodology (telemetry perf tier, xmMessaging M9): replace
 * the global allocation functions and count allocations made ON THE PROBING
 * THREAD between AllocProbe construction and query. Include from exactly
 * ONE translation unit per binary — the replacement operator new/delete
 * definitions below are deliberately non-inline (one definition per
 * program).
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_TESTS_BENCHMARKS_INGESTION_ALLOC_PROBE_HPP
#define QUICKVIZ_TESTS_BENCHMARKS_INGESTION_ALLOC_PROBE_HPP

#include <cstdint>
#include <cstdlib>
#include <new>

namespace quickviz {
namespace bench {

inline thread_local bool g_alloc_counting = false;
inline thread_local std::uint64_t g_alloc_count = 0;

class AllocProbe {
 public:
  AllocProbe() {
    g_alloc_count = 0;
    g_alloc_counting = true;
  }
  ~AllocProbe() { g_alloc_counting = false; }
  AllocProbe(const AllocProbe&) = delete;
  AllocProbe& operator=(const AllocProbe&) = delete;

  std::uint64_t allocations() const { return g_alloc_count; }
};

inline void* CountingAlloc(std::size_t size) {
  if (g_alloc_counting) {
    ++g_alloc_count;
  }
  if (void* p = std::malloc(size != 0 ? size : 1)) {
    return p;
  }
  throw std::bad_alloc();
}

inline void* CountingAllocNoThrow(std::size_t size) noexcept {
  if (g_alloc_counting) {
    ++g_alloc_count;
  }
  return std::malloc(size != 0 ? size : 1);
}

}  // namespace bench
}  // namespace quickviz

// Replaceable global allocation functions. Every form is replaced (throwing
// and nothrow) so allocation/deallocation pairs stay malloc/free-consistent
// under sanitizer interceptors.
void* operator new(std::size_t size) {
  return quickviz::bench::CountingAlloc(size);
}
void* operator new[](std::size_t size) {
  return quickviz::bench::CountingAlloc(size);
}
void* operator new(std::size_t size, const std::nothrow_t&) noexcept {
  return quickviz::bench::CountingAllocNoThrow(size);
}
void* operator new[](std::size_t size, const std::nothrow_t&) noexcept {
  return quickviz::bench::CountingAllocNoThrow(size);
}
void operator delete(void* ptr) noexcept { std::free(ptr); }
void operator delete[](void* ptr) noexcept { std::free(ptr); }
void operator delete(void* ptr, std::size_t) noexcept { std::free(ptr); }
void operator delete[](void* ptr, std::size_t) noexcept { std::free(ptr); }
void operator delete(void* ptr, const std::nothrow_t&) noexcept {
  std::free(ptr);
}
void operator delete[](void* ptr, const std::nothrow_t&) noexcept {
  std::free(ptr);
}

#endif  // QUICKVIZ_TESTS_BENCHMARKS_INGESTION_ALLOC_PROBE_HPP
