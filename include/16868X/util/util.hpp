#pragma once
#include "api.h"
#include <functional>
#include <queue>
#include <vector>

namespace lib16868X {
template<typename T> std::vector<T> queueToVector(const std::queue<T> q);
template<typename T> std::queue<T> vectorToQueue(const std::vector<T> v);

void runAsBlocking(std::function<void()> fn, std::function<bool()> endCond, int timeout = -1, int pollRate = 10, int paddingDelay = 5);
void blocking(std::function<bool()> endCond, int timeout = -1, int pollRate = 10, int paddingDelay = 5);
pros::Task runAsync(std::function<void()> fn);
} // namespace lib16868X