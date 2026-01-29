#ifndef IMAGE_BUFFER_CONCEPT_HPP
#define IMAGE_BUFFER_CONCEPT_HPP

#include "imagebuffer/metadata.hpp"

enum class ImageBufferError : uint16_t
{
    NO_ERROR = 0,
    WRITE_ERROR = 1,
    READ_ERROR = 2,
    OUT_OF_BOUNDS = 3,
    CHECKSUM_ERROR = 4,
    EMPTY_BUFFER = 5,
    FULL_BUFFER = 6,
    DATA_ERROR = 7,
};

template<typename BufferType>
concept ImageBufferConcept =
    requires(BufferType b,
             const BufferType cb,
             ImageMetadata meta,
             uint8_t* data,
             size_t size)
{
    // State queries
    { cb.is_empty() }      -> std::convertible_to<bool>;
    { cb.count() }         -> std::convertible_to<size_t>;

    // { cb.size() } -> std::convertible_to<size_t>; 
    // { cb.available() } -> std::convertible_to<size_t>; 
    // { cb.capacity() } -> std::convertible_to<size_t>; 
    
    // Proactive capacity check
    { cb.has_room_for(size) } -> std::convertible_to<bool>;

    // Producer API
    { b.add_image(meta) }        -> std::same_as<ImageBufferError>;
    { b.add_data_chunk(data, size) } -> std::same_as<ImageBufferError>;
    { b.push_image() }           -> std::same_as<ImageBufferError>;

    // Consumer API
    { b.get_image(meta) }        -> std::same_as<ImageBufferError>;
    { b.get_data_chunk(data, size) } -> std::same_as<ImageBufferError>;
    { b.pop_image() }            -> std::same_as<ImageBufferError>;
};

#endif // IMAGE_BUFFER_CONCEPT_HPP