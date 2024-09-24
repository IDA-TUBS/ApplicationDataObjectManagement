#include <adom/log.hpp>
#include <boost/date_time.hpp>

void adom::init_console_log()
{
    boost::log::add_common_attributes();
    
    #ifdef CONSOLE_ON
    boost::log::add_console_log
    (
        std::cout,
        boost::log::keywords::format = "[%TimeStamp%][%Severity%]: %Message%",
        boost::log::keywords::auto_flush = true,
        boost::log::keywords::filter = boost::log::trivial::severity >= boost::log::trivial::debug
    );
    #endif
}

void adom::init_file_log(std::string log_prefix, std::string log_suffix)
{
    char* home_dir = getenv("HOME");

    std::string log_file = std::string(home_dir) + "/" + log_directory + log_prefix + log_suffix + ".log";
    
    boost::log::add_common_attributes();    

    #ifdef FILE_ON
    boost::log::add_file_log
    (
        boost::log::keywords::file_name = log_file,
        boost::log::keywords::target_file_name = log_file,
        boost::log::keywords::auto_flush = true,
        boost::log::keywords::format = (
            boost::log::expressions::stream
                << boost::log::expressions::format_date_time<
                    boost::posix_time::ptime>("TimeStamp", "%Y-%m-%d %H:%M:%S.%f")  // Standard time format
                << ","
                << boost::log::expressions::smessage
        ),
        boost::log::keywords::filter = boost::log::trivial::severity == boost::log::trivial::trace
    );
    #endif
}