#pragma once

#include "HeapAllocation.hpp"
constexpr size_t O1HEAP_SIZE = 65536; 
using LocalHeap = HeapAllocation<O1HEAP_SIZE>;