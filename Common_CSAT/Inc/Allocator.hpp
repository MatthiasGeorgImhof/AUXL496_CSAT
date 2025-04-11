#ifndef INC_ALLOCATOR_HPP_
#define INC_ALLOCATOR_HPP_

#include <memory>
#include "o1heap.h"
#include "canard.h"
#include "cyphal.hpp"

#include <cstdio>
#include <cstdint>
#include <cstring>

// General O1HeapAllocator Definition
template <typename T>
class O1HeapAllocator
{
public:
    using value_type = T;

    explicit O1HeapAllocator(O1HeapInstance *h) : heap(h)
    {
        // fprintf(stderr, "O1HeapAllocator<> constructed with heap %p\n", heap);
    }

    template <typename U>
    O1HeapAllocator(const O1HeapAllocator<U> &other) noexcept : heap(other.getHeap()) {}

    T *allocate(std::size_t n)
    {
        return static_cast<T *>(o1heapAllocate(heap, n * sizeof(T)));
    }

    void deallocate(T *p, std::size_t /*n*/) const noexcept
    {
        o1heapFree(heap, p);
    }

    void destroy(T *p) const
    {
        p->~T();
    }

    template <typename U>
    bool operator==(const O1HeapAllocator<U> &) const { return true; }

    template <typename U>
    bool operator!=(const O1HeapAllocator<U> &) const { return false; }

    O1HeapInstance *getHeap() const noexcept { return heap; }

    struct Deletor
    {
        Deletor(O1HeapAllocator<T> allocator) : allocator_(allocator) {};

        void operator()(T *p) const
        {
            if (p != nullptr)
            {
                allocator_.destroy(p);
                allocator_.deallocate(p, 0);
            }
        }

    private:
        O1HeapAllocator<T> allocator_;
    };

    Deletor getDeletor()
    {
        return Deletor(*this);
    }

private:
    O1HeapInstance *heap;
};

// Specialize destroy for CanardRxTransfer
template <>
inline void O1HeapAllocator<CanardRxTransfer>::destroy(CanardRxTransfer *p) const
{
    o1heapFree(heap, p->payload);
    p->~CanardRxTransfer();
}

// Specialize destroy for CyphalTransfer
template <>
inline void O1HeapAllocator<CyphalTransfer>::destroy(CyphalTransfer *p) const
{
    o1heapFree(heap, p->payload);
    p->~CyphalTransfer();
}

// Use std::allocate_shared with O1HeapAllocator
template <typename T, typename Alloc, typename... Args>
std::shared_ptr<T> allocate_shared_custom(Alloc alloc, Args &&...args)
{
    return std::allocate_shared<T>(alloc, std::forward<Args>(args)...);
}

// Use std::unique_ptr with O1HeapAllocator
template <typename T, typename Alloc, typename... Args>
std::unique_ptr<T, typename Alloc::Deletor> allocate_unique_custom(Alloc alloc, Args &&...args)
{
    T *ptr = alloc.allocate(1);
    if (ptr != nullptr)
    {
        new (ptr) T(std::forward<Args>(args)...); // Placement new
    }
    return std::unique_ptr<T, typename Alloc::Deletor>(ptr, alloc.getDeletor());
}

// GeneralAllocator Definition
template <typename T>
class GeneralAllocator
{
public:
    using value_type = T;

    T *allocate(std::size_t n)
    {
        return static_cast<T *>(malloc(n * sizeof(T)));
    }

    void deallocate(T *p, std::size_t n) const noexcept
    {
        free(p);
    }

    void destroy(T *p) const
    {
        p->~T();
    }

    template <typename U>
    bool operator==(const GeneralAllocator<U> &) const { return true; }

    template <typename U>
    bool operator!=(const GeneralAllocator<U> &) const { return false; }

    struct Deletor
    {
        Deletor(GeneralAllocator<T> allocator) : allocator_(allocator) {};

        void operator()(T *p) const
        {
            if (p != nullptr)
            {
                allocator_.destroy(p);
                allocator_.deallocate(p, 0);
            }
        }

    private:
        GeneralAllocator<T> allocator_;
    };

    Deletor getDeletor()
    {
        return Deletor(*this);
    }
};

#endif /* INC_ALLOCATOR_HPP_ */