#ifndef COMMONS_HPP
#define COMMONS_HPP

#include <chrono>
#include <string>

namespace commons
{
    using clock = std::chrono::system_clock;
    static std::chrono::time_point<std::chrono::system_clock> now()
    {
        return clock::now();
    }

    static double elapsed(std::chrono::time_point<std::chrono::system_clock> stop, std::chrono::time_point<std::chrono::system_clock> start)
    {
        return std::chrono::duration<double, std::milli>(stop - start).count();
    }

    struct Timer
    {
        std::string function_name;
        std::chrono::time_point<std::chrono::system_clock> start;

        Timer(std::string function_name = "Function")
            : function_name(function_name)
        {
            start = now();
        }

        ~Timer()
        {
            auto duration = std::chrono::duration<double, std::milli>(now() - start);
            ROS_DEBUG_STREAM(function_name << " took: " << duration.count() << " ms");
        }
    };

    struct Topics
    {
        inline static std::string pains = "/pains";
        inline static std::string action = "/action";
        inline static std::string rgb = "/camera/rgb/image_raw";
        inline static std::string laser_scan = "/scan";
    };

    // struct Frames
    // {
    //     inline static std::string robot = "robot";
    // };

} // namespace commons

#endif
