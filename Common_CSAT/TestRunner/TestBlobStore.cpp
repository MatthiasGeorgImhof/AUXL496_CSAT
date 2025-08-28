#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "BlobStore.hpp"
#include <cstdint>
#include <array>
#include <span>

struct BlobStruct
{
    uint8_t sensor_data[64];
    uint8_t config_data[16];

    enum class FieldIndex : size_t
    {
        sensor_data,
        config_data
    };
};

static constexpr std::array<BlobMemberInfo, 2> blob_map = {{{"sensor_data", offsetof(BlobStruct, sensor_data), sizeof(BlobStruct::sensor_data)},
                                                            {"config_data", offsetof(BlobStruct, config_data), sizeof(BlobStruct::config_data)}}};

uint8_t ram_memory[1024] = {};
SPIBlobStoreAccess spi_access(1024, ram_memory);

TEST_CASE("BlobStore - Direct offset-based access")
{
    BlobStore<SPIBlobStoreAccess, BlobStruct> store(spi_access);

    uint8_t sensor_data[64] = {11, 22, 33};
    REQUIRE(store.write_blob(sensor_data, sizeof(sensor_data),
                             blob_map[static_cast<size_t>(BlobStruct::FieldIndex::sensor_data)].offset,
                             blob_map[static_cast<size_t>(BlobStruct::FieldIndex::sensor_data)].size));

    uint8_t readback[64] = {};
    REQUIRE(store.read_blob(readback, sizeof(readback),
                            blob_map[static_cast<size_t>(BlobStruct::FieldIndex::sensor_data)].offset,
                            blob_map[static_cast<size_t>(BlobStruct::FieldIndex::sensor_data)].size));

    for (size_t i = 0; i < 3; ++i)
        CHECK(readback[i] == sensor_data[i]);

    SUBCASE("Reject oversized write")
    {
        uint8_t too_large[128] = {};
        REQUIRE_FALSE(store.write_blob(too_large, sizeof(too_large),
                                       blob_map[static_cast<size_t>(BlobStruct::FieldIndex::sensor_data)].offset,
                                       blob_map[static_cast<size_t>(BlobStruct::FieldIndex::sensor_data)].size));
    }

    SUBCASE("Reject undersized read buffer")
    {
        uint8_t tiny[10] = {};
        REQUIRE_FALSE(store.read_blob(tiny, sizeof(tiny),
                                      blob_map[static_cast<size_t>(BlobStruct::FieldIndex::sensor_data)].offset,
                                      blob_map[static_cast<size_t>(BlobStruct::FieldIndex::sensor_data)].size));
    }
}

TEST_CASE("NamedBlobStore - Write/read by name")
{
    NamedBlobStore<SPIBlobStoreAccess, BlobStruct, BlobMemberInfo,
                   blob_map.size()>
        named_store(spi_access, blob_map);

    uint8_t config_data[16] = {99, 100, 101};
    REQUIRE(named_store.write_blob_by_name("config_data", config_data, sizeof(config_data)));

    uint8_t buffer[32] = {};
    auto span = named_store.read_blob_by_name("config_data", buffer, sizeof(buffer));
    CHECK(span.size() == 16);
    CHECK(span[0] == 99);

    SUBCASE("Unknown blob name returns empty span")
    {
        uint8_t dummy[10];
        auto unknown = named_store.read_blob_by_name("bogus", dummy, sizeof(dummy));
        CHECK(unknown.size() == 0);
    }

    SUBCASE("Reject oversized write by name")
    {
        uint8_t bloated[128] = {};
        REQUIRE_FALSE(named_store.write_blob_by_name("sensor_data", bloated, sizeof(bloated)));
    }

    SUBCASE("Reject undersized buffer by name")
    {
        uint8_t small[8] = {};
        auto result = named_store.read_blob_by_name("sensor_data", small, sizeof(small));
        CHECK(result.size() == 0);
    }
}

TEST_CASE("NamedBlobStore - Direct access to offsets")
{
    NamedBlobStore<SPIBlobStoreAccess, BlobStruct, BlobMemberInfo,
                   blob_map.size()>
        named_store(spi_access, blob_map);

    uint8_t input[16] = {42, 43, 44};
    REQUIRE(named_store.direct_write_blob(input, sizeof(input),
                                          blob_map[static_cast<size_t>(BlobStruct::FieldIndex::config_data)].offset,
                                          blob_map[static_cast<size_t>(BlobStruct::FieldIndex::config_data)].size));

    uint8_t output[32] = {};
    REQUIRE(named_store.direct_read_blob(output, sizeof(output),
                                         blob_map[static_cast<size_t>(BlobStruct::FieldIndex::config_data)].offset,
                                         blob_map[static_cast<size_t>(BlobStruct::FieldIndex::config_data)].size));

    for (size_t i = 0; i < 3; ++i)
        CHECK(output[i] == input[i]);

    SUBCASE("Reject overflow write at direct offset")
    {
        uint8_t too_big[128] = {};
        REQUIRE_FALSE(named_store.direct_write_blob(too_big, sizeof(too_big),
                                                    blob_map[static_cast<size_t>(BlobStruct::FieldIndex::config_data)].offset,
                                                    blob_map[static_cast<size_t>(BlobStruct::FieldIndex::config_data)].size));
    }

    SUBCASE("Reject undersized read buffer at direct offset")
    {
        uint8_t too_small[8] = {};
        REQUIRE_FALSE(named_store.direct_read_blob(too_small, sizeof(too_small),
                                                   blob_map[static_cast<size_t>(BlobStruct::FieldIndex::config_data)].offset,
                                                   blob_map[static_cast<size_t>(BlobStruct::FieldIndex::config_data)].size));
    }
}
