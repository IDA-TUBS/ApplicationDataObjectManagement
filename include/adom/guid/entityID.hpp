#ifndef SCPS_EntityID_h
#define SCPS_EntityID_h

#include<boost/endian/conversion.hpp>

#include <string>
#include <cstdint>
#include <cstring>
#include <sstream>
#include <iomanip>
#include <algorithm>

namespace scps
{

/********************** Entity IDs ***************************/
#define ENTITYID_READER 0x0010
#define ENTITYID_WRITER 0x0001

/**
 * @brief Structure EntityID_t, entity id part of GUID_t.
 * 
 * 
 */
struct EntityID_t
{
    static constexpr unsigned int size = 4;
    
    unsigned char value[size];

    EntityID_t()
    {
        memset(value, 0, size);
    };

    /**
     * @brief Main constructor.
     * @param id Entity id
     */
    EntityID_t(
            uint32_t id)
    {
        id = boost::endian::native_to_big(id);
        memcpy(value, &id, size);
    }

    /**
     * @brief Copy constructor
     */
    EntityID_t(
            const EntityID_t& id)
    {
        memcpy(value, id.value, size);
    }

    /**
     * @brief Move constructor
     */
    EntityID_t(
            EntityID_t&& id)
    {
        memmove(value, id.value, size);
    }

    static EntityID_t unknown()
    {
        return EntityID_t();
    }

    EntityID_t& operator =(
            const EntityID_t& id)
    {
        memcpy(value, id.value, size);
        return *this;
    }

    EntityID_t& operator =(
            EntityID_t&& id)
    {
        memmove(value, id.value, size);
        return *this;
    }

    /**
     * Assignment operator.
     * @param id Entity id to copy
     */
    EntityID_t& operator =(
            uint32_t id)
    {
        memcpy(value, &id, size);
        return *this;
    }

    /*!
     * @brief conversion to uint32_t
     * @return uint32_t representation
     */
    uint32_t to_uint32() const
    {
        uint32_t res = *reinterpret_cast<const uint32_t*>(value);
        res = boost::endian::big_to_native(res);
        return res;
    }
};


/**
 * Guid prefix comparison operator
 * @param id1 EntityId to compare
 * @param id2 ID prefix to compare
 * @return True if equal
 */
inline bool operator ==(
        EntityID_t& id1,
        const uint32_t id2)
{
    const bool result = 0 == memcmp(id1.value, &id2, sizeof(id2));
    return result;
}

/**
 * Guid prefix comparison operator
 * @param id1 First EntityId to compare
 * @param id2 Second EntityId to compare
 * @return True if equal
 */
inline bool operator ==(
        const EntityID_t& id1,
        const EntityID_t& id2)
{
    for (uint8_t i = 0; i < 4; ++i)
    {
        if (id1.value[i] != id2.value[i])
        {
            return false;
        }
    }
    return true;
}

/**
 * Guid prefix comparison operator
 * @param id1 First EntityId to compare
 * @param id2 Second EntityId to compare
 * @return True if not equal
 */
inline bool operator !=(
        const EntityID_t& id1,
        const EntityID_t& id2)
{
    for (uint8_t i = 0; i < 4; ++i)
    {
        if (id1.value[i] != id2.value[i])
        {
            return true;
        }
    }
    return false;
}

inline std::ostream& operator <<(
        std::ostream& output,
        const EntityID_t& enI)
{
    output << std::hex;
    output << (int)enI.value[0] << "." << (int)enI.value[1] << "." << (int)enI.value[2] << "." << (int)enI.value[3];
    return output << std::dec;
}

inline std::istream& operator >>(
        std::istream& input,
        EntityID_t& enP)
{
    std::istream::sentry s(input);

    if (s)
    {
        char point;
        unsigned short hex;
        std::ios_base::iostate excp_mask = input.exceptions();

        try
        {
            input.exceptions(excp_mask | std::ios_base::failbit | std::ios_base::badbit);
            input >> std::hex >> hex;

            if (hex > 255)
            {
                input.setstate(std::ios_base::failbit);
            }

            enP.value[0] = static_cast<unsigned char>(hex);

            for (int i = 1; i < 4; ++i)
            {
                input >> point >> hex;
                if ( point != '.' || hex > 255 )
                {
                    input.setstate(std::ios_base::failbit);
                }
                enP.value[i] = static_cast<unsigned char>(hex);
            }

            input >> std::dec;
        }
        catch (std::ios_base::failure& )
        {
        }

        input.exceptions(excp_mask);
    }

    return input;
}

/*********************************** Available Entity IDs *******************************************/
const EntityID_t c_entityID_reader = ENTITYID_READER;
const EntityID_t c_entityID_writer = ENTITYID_WRITER;


}; // end namespace 

#endif