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

    struct Topics
    {
        inline static std::string pains = "/pains";
        inline static std::string action = "/action";
    };

    // struct Frames
    // {
    //     inline static std::string robot = "robot";
    // };

} // namespace commons

#endif
