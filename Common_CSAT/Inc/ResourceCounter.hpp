#pragma once

#include <cstddef>

template <typename Resource>
class ResourceCounter {
private:
    size_t count_{0};
public:
    Resource resource_;

public:
    ResourceCounter(Resource resource = {}, size_t initial_claims = 0)
        : count_(initial_claims), resource_(resource) {}

    ~ResourceCounter() {}

    void increment() {
        count_++;
    }

    bool decrement() {
        count_--;
        return hasClaims();
    }

    bool hasClaims() const {
        return count_ > 0;
    }

    size_t getCount() const {
        return count_;
    }

    void reset() {
        count_ = 0;
    }
};