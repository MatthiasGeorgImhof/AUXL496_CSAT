#pragma once

#include <array>
#include <type_traits>
#include <cstdint>
#include <limits>
#include <algorithm>
#include <stdexcept>

/**
 * @brief A container that stores elements in a fixed-size array,
 *        keeping track of which slots are in use with a bitset.
 *
 * @tparam CONTENT The type of the elements to store.
 * @tparam N The size of the fixed-size array (must be 8, 16, 32, or 64).
 */
template <typename CONTENT, uint8_t N>
class BoxSet
{
private:
    using UType = typename std::conditional<N == 8, uint8_t,
                                            typename std::conditional<N == 16, uint16_t,
                                                                      typename std::conditional<N == 32, uint32_t,
                                                                                                typename std::conditional<N == 64, uint64_t, void>::type>::type>::type>::type;

    static_assert(!std::is_same_v<UType, void>, "N must be 8, 16, 32, or 64");

    std::array<CONTENT, N> content;
    UType active = {};

public:
    /**
     * @brief Default constructor.
     *
     * Initializes an empty BoxSet.
     */
    BoxSet() = default;

    /**
     * @brief Constructor that initializes the BoxSet with an array.
     *
     * @param init_content The array to initialize the BoxSet with. All slots are considered to be active.
     */
    BoxSet(const std::array<CONTENT, N> &init_content) : content(init_content)
    {
        active = std::numeric_limits<UType>::max();
    }

    /**
     * @brief Checks if the BoxSet is empty.
     *
     * @return True if the BoxSet is empty, false otherwise.
     */
    bool is_empty() const { return (active == 0); }

    /**
     * @brief Checks if the BoxSet is full.
     *
     * @return True if the BoxSet is full, false otherwise.
     */
    bool is_full() const { return (active == std::numeric_limits<UType>::max()); }

    /**
     * @brief Returns the capacity of the BoxSet.
     *
     * @return The maximum number of elements that the BoxSet can hold.
     */
    uint8_t capacity() const { return N; }

    /**
     * @brief Gets the number of elements currently in the BoxSet.
     *
     * @return The number of elements in use in the BoxSet.
     */
    uint8_t size() const
    {
        UType temp = active;
        uint8_t count = 0;
        while (temp != 0)
        {
            temp &= (temp - 1);
            count++;
        }
        return count;
    }

    /**
     * @brief Checks if a given index is in use.
     *
     * @param index The index to check.
     * @return True if the index is in use, false otherwise.
     */
    bool is_used(uint8_t index) const
    {
        return (active & (static_cast<UType>(1) << index));
    }

    /**
     * @brief Adds an item to the BoxSet.
     *
     * @param item The item to add to the container.
     *
     *  The item will be added to the first available slot.
     */
    CONTENT *add(const CONTENT &item)
    {
        for (uint8_t i = 0; i < N; ++i)
        {
            if (!is_used(i))
            {
                content[i] = item;
                activate(i);
                return &content[i];
            }
        }
        return nullptr;
    }

    /**
     * @brief Removes an item at a given index.
     *
     * @param index The index to remove.
     * @return The item that was removed.
     */
    CONTENT remove(uint8_t index)
    {
        CONTENT tmp = content[index];
        deactivate(index);
        return tmp;
    }

    /**
     * @brief Removes an item from the BoxSet using its pointer.
     *
     * @param itemPtr The pointer to the item to remove.
     *
     */
    void remove(CONTENT *itemPtr)
    {
        for (uint8_t i = 0; i < N; ++i)
        {
            if (is_used(i) && &content[i] == itemPtr)
            {
                deactivate(i);
                return; // Exit once item is removed
            }
        }
    }

    /**
     * @brief Clears the BoxSet, removing all elements.
     *
     *  Removes all elements from the BoxSet and sets its size to 0.
     */
    void clear()
    {
        active = 0;
    }
    
    /**
     * @brief Finds an item in the BoxSet using a custom comparator and returns a pointer to it or nullptr if not found.
     *
     * @tparam Comparator The type of the custom comparator.
     * @param item The item to search for.
     * @param comp The custom comparator to use.
     * @return A pointer to the item if found, or nullptr if not found.
     */
    template <typename Comparator>
    CONTENT *find(const CONTENT &item, Comparator comp) const
    {
        auto it = std::find_if(cbegin(), cend(), [&](const CONTENT &storedItem)
                            { return comp(item, storedItem); });
        if (it != cend())
        {
            return &(*it);
        }
        return nullptr;
    }

    /**
     * @brief Searches the BoxSet for an item using a custom comparator. If found, returns its pointer.
     *        If not found, finds the first inactive slot and sets it to active, and returns a pointer to it.
     *        If the container is full, it returns a nullptr.
     * @tparam Comparator The type of the custom comparator.
     * @param item The item to search for.
     * @param comp The custom comparator to use.
     * @return A pointer to the item if found, or a pointer to the new item or nullptr if container is full.
     */
    template <typename Comparator>
    CONTENT *find_or_create(const CONTENT &item, Comparator comp)
    {
        auto it = find(item, comp);
        if (it) return it;

        // Item not found, lets find a free slot
        for (uint8_t i = 0; i < N; ++i)
        {
            if (!is_used(i))
            {
                content[i] = item;
                activate(i);
                return &content[i]; // Return the newly created item
            }
        }

        return nullptr; // Container is full
    }

    /**
     * @brief Checks if the BoxSet contains a particular item with a custom comparator.
     *
     * @tparam Comparator The type of the custom comparator.
     * @param item The item to search for.
     * @param comp The custom comparator to use.
     * @return True if the BoxSet contains the item, false otherwise.
     */
    template <typename Comparator>
    bool contains(const CONTENT &item, Comparator comp) const
    {
        return (find(item, comp) != nullptr);
    }

    /**
     * @brief Checks if the BoxSet contains a particular item using equality comparison.
     *
     * @param item The item to search for.
     * @return True if the BoxSet contains the item, false otherwise.
     */
    bool contains(const CONTENT &item) const
    {
        return contains(item, std::equal_to<CONTENT>());
    }

    /**
     * @brief Iterator class for BoxSet.
     *
     * Provides forward iteration over active items in the container.
     */
    class iterator
    {
    public:
        using iterator_category = std::forward_iterator_tag;
        using value_type = CONTENT;
        using difference_type = std::ptrdiff_t;
        using pointer = CONTENT *;
        using reference = CONTENT &;

        iterator(BoxSet &boxSet, size_t index) : m_boxSet(&boxSet), m_index(index)
        {
            _findNextActive();
        }

        iterator &operator++()
        {
            m_index++;
            _findNextActive();
            return *this;
        }

        iterator operator++(int)
        {
            iterator tmp = *this;
            ++(*this);
            return tmp;
        }

        bool operator==(const iterator &other) const { return m_boxSet == other.m_boxSet && m_index == other.m_index; }
        bool operator!=(const iterator &other) const { return !(*this == other); }

        reference operator*() const { return m_boxSet->content[m_index]; }
        pointer operator->() const { return &(m_boxSet->content[m_index]); }

    private:
        BoxSet *m_boxSet;
        size_t m_index;

        void _findNextActive()
        {
            while (m_index < m_boxSet->content.size() && !m_boxSet->is_used(static_cast<uint8_t>(m_index)))
            {
                m_index++;
            }
        }
    };

    /**
     * @brief Returns an iterator to the beginning of the BoxSet.
     *
     * @return An iterator pointing to the first active element in the BoxSet.
     */
    iterator begin() { return iterator(*this, 0); }

    /**
     * @brief Returns a const iterator to the beginning of the BoxSet.
     *
     * @return A const iterator pointing to the first active element in the BoxSet.
     */
    const iterator cbegin() const { return iterator(const_cast<BoxSet &>(*this), 0); }

    /**
     * @brief Returns an iterator to the end of the BoxSet.
     *
     * @return An iterator pointing one past the last active element in the BoxSet.
     */
    iterator end() { return iterator(*this, N); }

    /**
     * @brief Returns a const iterator to the end of the BoxSet.
     *
     * @return A const iterator pointing one past the last active element in the BoxSet.
     */
    const iterator cend() const { return iterator(const_cast<BoxSet &>(*this), N); }

private:
    /**
     * @brief Sets the bit at the given index in the active bitset.
     * @param index The index of the bit to set.
     */
    void activate(uint8_t index)
    {
        active |= static_cast<UType>(static_cast<UType>(1) << index);
    }
    /**
     * @brief Resets the bit at the given index in the active bitset.
     * @param index The index of the bit to reset.
     */
    void deactivate(uint8_t index)
    {
        active &= static_cast<UType>(~(static_cast<UType>(1) << index));
    }
};