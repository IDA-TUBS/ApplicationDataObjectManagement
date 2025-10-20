#ifndef SCPS_Guid_h
#define SCPS_Guid_h

#include <adom/guid/guidPrefix.hpp>
#include <adom/guid/entityID.hpp>

#include <string>
#include <cstdint>
#include <cstring>
#include <sstream>
#include <iomanip>
#include <algorithm>

namespace scps
{

/**
 * @brief GUID for applications / entities
 *  
 */
struct GUID_t
{
    GuidPrefix_t prefix;
    EntityID_t entityID;

    /**
     * @brief Default constructor. Contructs an unknown GUID.
     */
    GUID_t() noexcept
    {
    }


    /**
     * @brief Construct a new guid t object
     * 
     * @param guid_prefix Guid prefix
     * @param id Entity id
     */
    GUID_t(
        const GuidPrefix_t& guid_prefix,
        uint32_t id) noexcept
        : prefix(guid_prefix)
        , entityID(id)
    {
    }

    /**
     * @brief Construct a new guid t object
     * 
     * @param guid_prefix Guid prefix
     * @param entity_id Entity id
     */
    GUID_t(
            const GuidPrefix_t& guid_prefix,
            const EntityID_t& entity_id) noexcept
        : prefix(guid_prefix)
        , entityID(entity_id)
    {
    }

    /**
     * @brief Checks whether this guid is for an entity on the same host as another guid.
     *
     * @param other_guid GUID_t to compare to.
     *
     * @return true when this guid is on the same host, false otherwise.
     */
    bool is_on_same_host_as(
            const GUID_t& other_guid) const
    {
        return memcmp(prefix.value, other_guid.prefix.value, VENDOR_ID_LEN+HOST_ID_LEN) == 0;
    }

    /**
     * Checks whether this guid is for an entity on the same host and process as another guid.
     *
     * @param other_guid GUID_t to compare to.
     *
     * @return true when this guid is on the same host and process, false otherwise.
     */
    bool is_on_same_process_as(
            const GUID_t& other_guid) const
    {
        return memcmp(prefix.value, other_guid.prefix.value, VENDOR_ID_LEN+HOST_ID_LEN+PROCESS_ID_LEN) == 0;
    }

    static GUID_t unknown() noexcept
    {
        return GUID_t();
    }

};


/**
 * GUID comparison operator
 * @param g1 First GUID to compare
 * @param g2 Second GUID to compare
 * @return True if equal
 */
inline bool operator ==(
        const GUID_t& g1,
        const GUID_t& g2)
{
    if (g1.prefix == g2.prefix && g1.entityID == g2.entityID)
    {
        return true;
    }
    else
    {
        return false;
    }
}

/**
 * GUID comparison operator
 * @param g1 First GUID to compare
 * @param g2 Second GUID to compare
 * @return True if not equal
 */
inline bool operator !=(
        const GUID_t& g1,
        const GUID_t& g2)
{
    if (g1.prefix != g2.prefix || g1.entityID != g2.entityID)
    {
        return true;
    }
    else
    {
        return false;
    }
}

inline bool operator <(
        const GUID_t& g1,
        const GUID_t& g2)
{
    for (uint8_t i = 0; i < 12; ++i)
    {
        if (g1.prefix.value[i] < g2.prefix.value[i])
        {
            return true;
        }
        else if (g1.prefix.value[i] > g2.prefix.value[i])
        {
            return false;
        }
    }
    for (uint8_t i = 0; i < 4; ++i)
    {
        if (g1.entityID.value[i] < g2.entityID.value[i])
        {
            return true;
        }
        else if (g1.entityID.value[i] > g2.entityID.value[i])
        {
            return false;
        }
    }
    return false;
}

const GUID_t Guid_Unknown;

/**
 * @brief Stream operator, prints a GUID.
 * @param output Output stream.
 * @param guid GUID_t to print.
 * @return Stream operator.
 */
inline std::ostream& operator <<(
        std::ostream& output,
        const GUID_t& guid)
{
    if (guid != Guid_Unknown)
    {
        output << guid.prefix << "|" << guid.entityID;
    }
    else
    {
        output << "|GUID UNKNOWN|";
    }
    return output;
}

/**
 * Stream operator, retrieves a GUID.
 * @param input Input stream.
 * @param guid GUID_t to print.
 * @return Stream operator.
 */
inline std::istream& operator >>(
        std::istream& input,
        GUID_t& guid)
{
    std::istream::sentry s(input);

    if (s)
    {
        std::ios_base::iostate excp_mask = input.exceptions();

        try
        {
            input.exceptions(excp_mask | std::ios_base::failbit | std::ios_base::badbit);

            char sep;
            input >> guid.prefix >> sep >> guid.entityID;

            if (sep != '|')
            {
                input.setstate(std::ios_base::failbit);
            }
        }
        catch (std::ios_base::failure&)
        {
            // maybe is unknown or just invalid
            guid = Guid_Unknown;
        }

        input.exceptions(excp_mask);
    }

    return input;
}

}; // end namespace 

#endif 