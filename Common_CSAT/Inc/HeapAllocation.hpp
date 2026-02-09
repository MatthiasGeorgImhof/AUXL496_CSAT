#ifndef __HEAP_ALLOCATION_HPP__
#define __HEAP_ALLOCATION_HPP__
#include <cstddef>
#include <cstdint>
#include <memory>
#include <utility>

#include "o1heap.h"
#include "cyphal.hpp"
#include "canard.h"
#include "serard.h"
#include "udpard.h"

#include "Logger.hpp"
#include "IRQLock.hpp"

#ifdef __arm____
#include "stm32l4xx_hal.h"
#else
#include "mock_hal.h"
#endif

#undef DEBUG_ALLOCATIONS

typedef struct
{
	size_t capacity;
	size_t allocated;
	size_t peak_allocated;
	size_t peak_request_size;
	uint64_t oom_count;
} HeapDiagnostics;

template <size_t HeapSize = 65536>
class HeapAllocation
{
private:
	static uint8_t o1heap_buffer[HeapSize] __attribute__((aligned(O1HEAP_ALIGNMENT)));
	static O1HeapInstance *o1heap;

	static void disableCANInterrupts()
	{
		CanTxIrqLock::lock();
		CanRx0IrqLock::lock();
		CanRx1IrqLock::lock();
	}

	static void enableCANInterrupts()
	{
		CanTxIrqLock::unlock();
		CanRx0IrqLock::unlock();
		CanRx1IrqLock::unlock();
	}

	static void *safeAllocate(const size_t size)
	{
		disableCANInterrupts();
		void *ptr = o1heapAllocate(o1heap, size);
		enableCANInterrupts();
#ifdef DEBUG_ALLOCATIONS
		log(LOG_LEVEL_INFO, "allocate: %8p %4ld\r\n", ptr, size);
#endif // DEBUG_ALLOCATIONS
		return ptr;
	}

	static void *unsafeAllocate(const size_t size)
	{
		return o1heapAllocate(o1heap, size);
	}

	static void safeDeallocate(void *const pointer)
	{
		if (pointer == nullptr)
		{
#ifdef DEBUG_ALLOCATIONS
			log(LOG_LEVEL_INFO, "skip deallocate: %8p\r\n", pointer);
#endif
			return;
		}

		disableCANInterrupts();
		o1heapFree(o1heap, pointer);
		enableCANInterrupts();
#ifdef DEBUG_ALLOCATIONS
		log(LOG_LEVEL_INFO, "deallocate: %8p\r\n", pointer);
#endif
	}

	static void unsafeDeallocate(void *const pointer)
	{
		if (pointer == nullptr)
			return;
		o1heapFree(o1heap, pointer);
	}

public:
	static void initialize()
	{
		o1heap = o1heapInit(o1heap_buffer, HeapSize);
	}

	static void *heapAllocate(void *const /*handle*/, const size_t amount)
	{
		return safeAllocate(amount);
	}

	static void heapFree(void *const /*handle*/, void *const pointer)
	{
		safeDeallocate(pointer);
	}

	static void *canardMemoryAllocate(CanardInstance *const /*canard*/, const size_t size)
	{
		return safeAllocate(size);
	}

	static void canardMemoryDeallocate(CanardInstance *const /*canard*/, void *const pointer)
	{
		safeDeallocate(pointer);
	}

	static void *serardMemoryAllocate(void *const /*user_reference*/, const size_t size)
	{
		return safeAllocate(size);
	}

	static void serardMemoryDeallocate(void *const /*user_reference*/, const size_t /*size*/, void *const pointer)
	{
		safeDeallocate(pointer);
	}

	static void *udpardMemoryAllocate(void *const /*user_reference*/, const size_t size)
	{
		return safeAllocate(size);
	}

	static void udpardMemoryDeallocate(void *const /*user_reference*/, const size_t /*size*/, void *const pointer)
	{
		safeDeallocate(pointer);
	};

	static void *loopardMemoryAllocate(const size_t size)
	{
		return safeAllocate(size);
	}

	static void loopardMemoryDeallocate(void *const pointer)
	{
		safeDeallocate(pointer);
	}

	static O1HeapInstance *getO1Heap()
	{
		return o1heap;
	}

	static HeapDiagnostics getDiagnostics()
	{
		O1HeapInstance *inst = getO1Heap();
		// In production you might assert; in tests we just return zeros if uninitialized.
		if (inst == nullptr)
		{
			return HeapDiagnostics{0, 0, 0, 0, 0};
		}

		const O1HeapDiagnostics o1diag = o1heapGetDiagnostics(inst);
		return HeapDiagnostics{
			o1diag.capacity,
			o1diag.allocated,
			o1diag.peak_allocated,
			o1diag.peak_request_size,
			o1diag.oom_count};
	}
};

template <size_t HeapSize>
uint8_t HeapAllocation<HeapSize>::o1heap_buffer[HeapSize];

template <size_t HeapSize>
O1HeapInstance *HeapAllocation<HeapSize>::o1heap = nullptr;

template <typename T, typename Heap>
class SafeAllocator
{
public:
	using value_type = T;
	using pointer = T *;
	using const_pointer = const T *;
	using size_type = std::size_t;
	using difference_type = std::ptrdiff_t;

	template <typename U>
	struct rebind
	{
		using other = SafeAllocator<U, Heap>;
	};

	SafeAllocator() noexcept = default;

	template <typename U>
	SafeAllocator(const SafeAllocator<U, Heap> &) noexcept {}

	pointer allocate(size_type n)
	{
		return static_cast<pointer>(Heap::heapAllocate(nullptr, n * sizeof(T)));
	}

	// *** make this const ***
	void deallocate(pointer p, size_type) const noexcept
	{
		Heap::heapFree(nullptr, p);
	}

	// *** make this const ***
	void destroy(pointer p) const
	{
		if (!p)
			return;

		if constexpr (std::is_same_v<T, CyphalTransfer>)
		{
#ifdef DEBUG_ALLOCATIONS
			log(LOG_LEVEL_INFO, "destroy CyphalTransfer %p payload %p size %u\r\n",
				static_cast<void *>(p),
				p->payload,
				unsigned(p->payload_size));
#endif // DEBUG_ALLOCATIONS
			if (p->payload)
			{
				Heap::heapFree(nullptr, p->payload);
			}
		}
		else if constexpr (std::is_same_v<T, CanardRxTransfer>)
		{
			if (p->payload)
			{
				Heap::heapFree(nullptr, p->payload);
			}
		}

		p->~T();
	}

	template <typename U>
	bool operator==(const SafeAllocator<U, Heap> &) const noexcept { return true; }

	template <typename U>
	bool operator!=(const SafeAllocator<U, Heap> &) const noexcept { return false; }

	struct Deletor
	{
		SafeAllocator alloc;

		void operator()(T *p) const
		{
			if (!p)
				return;
			alloc.destroy(p); // now OK: destroy is const
			alloc.deallocate(p, 1);
		}
	};

	Deletor getDeletor() const { return Deletor{*this}; }
};

// For SafeAllocator<T, Heap>
template <typename T, typename Heap, typename... Args>
std::unique_ptr<T, typename SafeAllocator<T, Heap>::Deletor>
alloc_unique_custom(SafeAllocator<T, Heap> alloc, Args &&...args)
{
	using Alloc = SafeAllocator<T, Heap>;
	T *ptr = alloc.allocate(1);
	if (ptr != nullptr)
	{
		// You can also use std::allocator_traits<Alloc>::construct if you prefer
		new (ptr) T(std::forward<Args>(args)...);
	}
	return std::unique_ptr<T, typename Alloc::Deletor>(ptr, alloc.getDeletor());
}

template <typename T, typename Heap, typename... Args>
std::shared_ptr<T>
alloc_shared_custom(SafeAllocator<T, Heap> alloc, Args &&...args)
{
	// This uses SafeAllocator<T, Heap> for both control block and T.
	// When the shared_ptr refcount hits zero, it will:
	//   1) call allocator_traits<SafeAllocator>::destroy(alloc, ptr)
	//   2) call allocator_traits<SafeAllocator>::deallocate(alloc, ptr, 1)
	//
	// Our destroy() handles payload cleanup for CyphalTransfer/CanardRxTransfer.
	return std::allocate_shared<T>(alloc, std::forward<Args>(args)...);
}

#endif // __HEAP_ALLOCATION_HPP__
