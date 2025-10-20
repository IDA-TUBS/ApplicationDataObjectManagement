#ifndef SCPS_GuidPrefix_h
#define SCPS_GuidPrefix_h

#include <string>
#include <cstdint>
#include <cstring>
#include <sstream>
#include <iomanip>
#include <algorithm>

#include <boost/endian/conversion.hpp>

namespace scps
{

const uint16_t vendorID_IDA = 0x01DA;

const uint8_t VENDOR_ID_LEN = 2;
const uint8_t HOST_ID_LEN = 4;
const uint8_t PARTICIPANT_ID_LEN = 2;
const uint8_t PROCESS_ID_LEN = 4;

const uint8_t VENDOR_ID_OFFSET = 0;
const uint8_t HOST_ID_OFFSET = 2;
const uint8_t PARTICIPANT_ID_OFFSET = HOST_ID_OFFSET + HOST_ID_LEN;
const uint8_t PROCESS_ID_OFFSET = PARTICIPANT_ID_OFFSET + PARTICIPANT_ID_LEN;

/**
 * @brief GUID prefix for applications / entities
 * 
 * Encoding:
 * Bytes 0-1: VendorID
 * Bytes 2-9: HostID
 * Bytes 10-11: ProcessID
 * 
 */
struct GuidPrefix_t
{
    static constexpr unsigned int size = 12;
    
    unsigned char value[size];

    GuidPrefix_t()
    {
        memset(value, 0, size);
    };

    GuidPrefix_t(unsigned char val[size])
    {
        memcpy(value, val, size);
    }

    GuidPrefix_t(const unsigned char val[size])
    {
        memcpy(value, val, size);
    }

    static GuidPrefix_t unknown()
    {
        return GuidPrefix_t();
    }

    uint32_t get_host()
    {
        uint32_t host_ID;
        memcpy(&host_ID, value+HOST_ID_OFFSET, HOST_ID_LEN);
        host_ID = boost::endian::big_to_native(host_ID);
        return host_ID;
    }

    /**
     * Guid prefix comparison operator
     * @param prefix guid prefix to compare
     * @return True if the guid prefixes are equal
     */
    bool operator ==(
            const GuidPrefix_t& prefix) const
    {
        return (memcmp(value, prefix.value, size) == 0);
    }

    /**
     * Guid prefix comparison operator
     * @param prefix Second guid prefix to compare
     * @return True if the guid prefixes are not equal
     */
    bool operator !=(
            const GuidPrefix_t& prefix) const
    {
        return (memcmp(value, prefix.value, size) != 0);
    }

    /**
     * Guid prefix minor operator
     * @param prefix Second guid prefix to compare
     * @return True if prefix is higher
     */
    bool operator <(
            const GuidPrefix_t& prefix) const
    {
        return std::memcmp(value, prefix.value, size) < 0;
    }
};

/**
 * @brief define default "Unknown" guid prefix
 * 
 */
const GuidPrefix_t GuidPrefix_Unknown;

inline std::ostream& operator <<(
        std::ostream& output,
        const GuidPrefix_t& guiP)
{
    output << std::hex;
    char old_fill = output.fill('0');
    for (uint8_t i = 0; i < 11; ++i)
    {
        output << std::setw(2) << (int)guiP.value[i] << ".";
    }
    output << std::setw(2) << (int)guiP.value[11];
    output.fill(old_fill);
    return output << std::dec;
}

inline std::istream& operator >>(
        std::istream& input,
        GuidPrefix_t& guiP)
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

            guiP.value[0] = static_cast<unsigned char>(hex);

            for (int i = 1; i < 12; ++i)
            {
                input >> point >> hex;
                if ( point != '.' || hex > 255 )
                {
                    input.setstate(std::ios_base::failbit);
                }
                guiP.value[i] = static_cast<unsigned char>(hex);
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
   
}; // end namespace 
#endif