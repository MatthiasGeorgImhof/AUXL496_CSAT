#ifndef INC_ARRAYLIST_HPP_
#define INC_ARRAYLIST_HPP_

#include <array>
#include <stdexcept>
#include <algorithm> // For std::sort and std::swap
#include <cstdint>
#include <utility>  // For std::move

template <typename T, size_t capacity_>
class ArrayList {
private:
    std::array<T, capacity_> data_;
    size_t count_;
    T dummy;

public:
    ArrayList() : count_(0), dummy() {}

    // Copy constructor (use default)
    ArrayList(const ArrayList& other) = default;

	// Move constructor
	ArrayList(ArrayList&& other) noexcept : data_(std::move(other.data_)), count_(other.count_), dummy(std::move(other.dummy)) {
		other.count_ = 0; // Important: Set the moved-from object to an empty state
	}


    // Copy assignment operator (use default)
    ArrayList& operator=(const ArrayList& other) = default;

	// Move assignment operator
	ArrayList& operator=(ArrayList&& other) noexcept {
		if (this != &other) {
			data_ = std::move(other.data_);
			dummy = std::move(other.dummy);
			count_ = other.count_;

			other.count_ = 0; // Important: Set the moved-from object to an empty state
		}
		return *this;
	}

    void push(const T& value) {
        if (count_ < capacity_) {
            data_[count_++] = value;
        }
    }

    // ... (rest of your class implementation, unchanged) ...
    template <typename Compare>
	size_t find(const T& value, Compare comp) {
		for (size_t i = 0; i < count_; ++i) {
			if (comp(data_[i], value))
			{
				return i;
			}
		}
		return capacity_;
	}

	// Add push or replace functionality
	template <typename Compare>
	void pushOrReplace(const T& value, Compare comp)
	{
		for (size_t i = 0; i < count_; ++i)
		{
			if(comp(data_[i],value))
			{
				data_[i] = value;
				return;
			}
		}
		push(value);
	}


	T& operator[](size_t index) {
			if (index < count_) {
				return data_[index];
			} else {
				return dummy;
			}
	}

	const T& operator[](size_t index) const {
		if (index < count_) {
			return data_[index];
		} else {
			return dummy;
		}
	}

	size_t size() const {
		return count_;
	}

	bool empty() const {
		return count_ == 0;
	}

	bool full() const {
		return count_ == capacity_;
	}

	size_t capacity() const {
		return capacity_;
	}

	void remove(size_t index) {
		if (index >= count_) {
			return; // Do nothing if index is out of bounds
		}

		// Shift elements after the removed index
		for (size_t i = index; i < count_ - 1; ++i) {
			data_[i] = data_[i + 1];
		}

		--count_;
	}

	// Remove all items satisfying a predicate
	template <typename Predicate>
    void removeIf(Predicate pred) {
        size_t writeIndex = 0;
        for (size_t readIndex = 0; readIndex < count_; ++readIndex) {
            if (!pred(data_[readIndex])) {
                if (writeIndex != readIndex) { // Avoid self-assignment
                    data_[writeIndex] = data_[readIndex];
                }
                writeIndex++;
            }
        }

        // Clear the remaining elements after writeIndex
        for (size_t i = writeIndex; i < count_; ++i) {
            data_[i] = T{}; // Destruct existing object
        }
        count_ = writeIndex;
    }

	template <typename Predicate>
    bool containsIf(Predicate pred) const {
        for (size_t i = 0; i < count_; ++i) {
            if (pred(data_[i])) {
                return true;
            }
        }
        return false;
    }

	// Iterator implementation
	class iterator {
	private:
		size_t index;
		ArrayList<T, capacity_>* list;

	public:
		using iterator_category = std::random_access_iterator_tag;
		using value_type = T;
		using difference_type = std::ptrdiff_t;
		using pointer = T*;
		using reference = T&;

		iterator(size_t index, ArrayList<T, capacity_>* list) : index(index), list(list) {}

		iterator(const iterator& other) = default;

		iterator& operator=(const iterator& other) = default;

		reference operator*() const {
			return list->data_[index];
		}

		pointer operator->() const { return &list->data_[index]; }


		iterator& operator++() {
			++index;
			return *this;
		}

		iterator operator++(int) {
			iterator temp = *this;
			++(*this);
			return temp;
		}
		iterator& operator--() {
			--index;
			return *this;
		}

		iterator operator--(int) {
			iterator temp = *this;
			--(*this);
			return temp;
		}

		iterator& operator+=(difference_type n) {
			index += static_cast<size_t>(n);
			return *this;
		}

		iterator operator+(difference_type n) const {
			iterator temp = *this;
			temp += n;
			return temp;
		}

		iterator& operator-=(difference_type n) {
			index -= static_cast<size_t>(n);
			return *this;
		}


		iterator operator-(difference_type n) const {
			iterator temp = *this;
			temp -= n;
			return temp;
		}

		difference_type operator-(const iterator& other) const {
			return static_cast<difference_type>(index) - static_cast<difference_type>(other.index);
		}

		reference operator[](difference_type n) const {
			return list->data_[index + static_cast<size_t>(n)];
		}

		bool operator==(const iterator& other) const { return index == other.index && list == other.list; }
		bool operator!=(const iterator& other) const { return index != other.index || list != other.list;}
		bool operator<(const iterator& other) const {return index < other.index;}
		bool operator>(const iterator& other) const {return index > other.index;}
		bool operator<=(const iterator& other) const {return index <= other.index;}
		bool operator>=(const iterator& other) const {return index >= other.index;}
	};

	// Const iterator implementation
	class const_iterator {
	private:
		size_t index;
		const ArrayList<T, capacity_>* list;

	public:
		using iterator_category = std::random_access_iterator_tag;
		using value_type = T;
		using difference_type = std::ptrdiff_t;
		using pointer = const T*;
		using reference = const T&;

		const_iterator(size_t index, const ArrayList<T, capacity_>* list) : index(index), list(list) {}
		const_iterator(const const_iterator& other) = default;

		const_iterator& operator=(const const_iterator& other) = default;

		reference operator*() const {
			return list->data_[index];
		}
		pointer operator->() const { return &list->data_[index]; }

		const_iterator& operator++() {
			++index;
			return *this;
		}

		const_iterator operator++(int) {
			const_iterator temp = *this;
			++(*this);
			return temp;
		}
		const_iterator& operator--() {
			--index;
			return *this;
		}

		const_iterator operator--(int) {
			const_iterator temp = *this;
			--(*this);
			return temp;
		}

		const_iterator& operator+=(difference_type n) {
			index += static_cast<size_t>(n);
			return *this;
		}
		const_iterator operator+(difference_type n) const {
			const_iterator temp = *this;
			temp += n;
			return temp;
		}
		const_iterator& operator-=(difference_type n) {
			index -= static_cast<size_t>(n);
			return *this;
		}


		const_iterator operator-(difference_type n) const {
			const_iterator temp = *this;
			temp -= n;
			return temp;
		}

		difference_type operator-(const const_iterator& other) const {
			return static_cast<difference_type>(index) - static_cast<difference_type>(other.index);
		}

		reference operator[](difference_type n) const {
			return list->data_[index + static_cast<size_t>(n)];
		}

		bool operator==(const const_iterator& other) const { return index == other.index && list == other.list; }
		bool operator!=(const const_iterator& other) const { return index != other.index || list != other.list;}
		bool operator<(const const_iterator& other) const {return index < other.index;}
		bool operator>(const const_iterator& other) const {return index > other.index;}
		bool operator<=(const const_iterator& other) const {return index <= other.index;}
		bool operator>=(const const_iterator& other) const {return index >= other.index;}
	};

	iterator begin() { return iterator(0, this); }
	iterator end() { return iterator(count_, this); }

	const_iterator cbegin() const { return const_iterator(0, this); }
	const_iterator cend() const { return const_iterator(count_, this); }

    const_iterator begin() const { return const_iterator(0, this); }
	const_iterator end() const { return const_iterator(count_, this); }
};
#endif /* INC_ARRAYLIST_HPP_ */