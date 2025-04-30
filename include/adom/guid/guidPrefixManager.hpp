#ifndef SCPS_guidPrefixManager_h
#define SCPS_guidPrefixManager_h

#include <adom/guid/guidPrefix.hpp>
#include <adom/log.hpp>

#include <boost/endian/conversion.hpp>

#include <unistd.h>

namespace scps{

class guidPrefixManager
{
    public:

    void create(
        uint32_t host_id, 
        uint16_t participant_id,
        GuidPrefix_t &guid_prefix) const
    {
        // Copy vendor ID
        std::copy(
            prefix_.value, 
            prefix_.value + VENDOR_ID_LEN, 
            guid_prefix.value
        );

        // Insert HOST ID (big endian)
        host_id = boost::endian::native_to_big(host_id);
        memcpy(
            guid_prefix.value+HOST_ID_OFFSET, 
            &host_id, 
            HOST_ID_LEN
        );

        // Insert participant ID (big endian)
        participant_id = boost::endian::native_to_big(participant_id);
        memcpy(
            guid_prefix.value+PARTICIPANT_ID_OFFSET, 
            &participant_id, 
            PARTICIPANT_ID_LEN
        );

        // Copy process ID
        std::copy(
            prefix_.value + PROCESS_ID_OFFSET, 
            prefix_.value + PROCESS_ID_OFFSET + PROCESS_ID_LEN, 
            guid_prefix.value + PROCESS_ID_OFFSET
        );
    }

    /**
     * Get a reference to the singleton instance.
     *
     * @return reference to the singleton instance.
     */
    static const guidPrefixManager& instance()
    {
        static guidPrefixManager singleton;
        return singleton;
    }
    
    private:

    guidPrefixManager()
    {
        // This is to comply with RTPS section 9.3.1.5 - Mapping of the GUID_t (big endian)
        uint16_t vendor = boost::endian::native_to_big(scps::vendorID_IDA);
        memcpy(prefix_.value, &vendor, VENDOR_ID_LEN);

        // Host ID supplied by create method parameter

        // Read process ID (big endian)
        pid_t processID = getpid();
        uint32_t pid = boost::endian::native_to_big(processID);
        memcpy(prefix_.value+PROCESS_ID_OFFSET, &pid, PROCESS_ID_LEN);
    }

    GuidPrefix_t prefix_;    

};

} // end namespace

#endif