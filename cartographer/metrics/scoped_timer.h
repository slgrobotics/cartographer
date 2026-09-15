/*
 * Copyright 2018 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CARTOGRAPHER_METRICS_SCOPED_TIMER_H_
#define CARTOGRAPHER_METRICS_SCOPED_TIMER_H_

#include <chrono>

#include "cartographer/metrics/histogram.h"

namespace cartographer {
namespace metrics {

// Records wall seconds, including early returns. Disabled metrics avoid clock
// reads. The histogram's bucket counts also count completed calls.
class ScopedTimer {
 public:
  explicit ScopedTimer(Histogram* histogram) : histogram_(histogram) {
    if (histogram_ != Histogram::Null()) start_ = Clock::now();
  }
  ~ScopedTimer() {
    if (histogram_ != Histogram::Null())
      histogram_->Observe(
          std::chrono::duration<double>(Clock::now() - start_).count());
  }
  ScopedTimer(const ScopedTimer&) = delete;
  ScopedTimer& operator=(const ScopedTimer&) = delete;

 private:
  using Clock = std::chrono::steady_clock;
  Histogram* histogram_;
  Clock::time_point start_;
};

}  // namespace metrics
}  // namespace cartographer
#endif
