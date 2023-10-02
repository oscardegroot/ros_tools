#ifndef __ROSTOOLS_PROFILING_H__
#define __ROSTOOLS_PROFILING_H__

#include <fstream>
#include <thread>
#include <mutex>
#include <chrono>
#include <string>

namespace RosTools
{

/********** Some Fancy timing classes for profiling (from TheCherno) ***********/
#define PROFILER 1
#if PROFILER
#define PROFILE_SCOPE(name) RosTools::InstrumentationTimer timer##__LINE__(name)
#define PROFILE_FUNCTION() PROFILE_SCOPE(__FUNCTION__)
#define PROFILE_AND_LOG(debug_enable, name) \
    if (debug_enable)                       \
    {                                       \
        ROS_INFO_STREAM(name);              \
    }                                       \
    RosTools::InstrumentationTimer timer##__LINE__(name)
#else
#define PROFILE_SCOPE(name)
#define PROFILE_FUNCTION()
#endif

    struct ProfileResult
    {
        std::string Name;
        long long Start, End;
        uint32_t ThreadID;
    };

    struct InstrumentationSession
    {
        std::string Name;
    };

    class Instrumentor
    {
    private:
        InstrumentationSession *m_CurrentSession;
        std::ofstream m_OutputStream;
        int m_ProfileCount;
        std::mutex m_lock;

    public:
        Instrumentor() : m_CurrentSession(nullptr), m_ProfileCount(0) {}

        void BeginSession(const std::string &name, const std::string &filepath = "profiler.json");
        void EndSession();

        void WriteProfile(const ProfileResult &result);

        void WriteHeader();
        void WriteFooter();

        static Instrumentor &Get()
        {
            static Instrumentor *instance = new Instrumentor();
            return *instance;
        }
    };

    class InstrumentationTimer
    {
    public:
        InstrumentationTimer(const char *name);
        ~InstrumentationTimer();

        void Stop();

    private:
        const char *m_Name;
        std::chrono::system_clock::time_point m_StartTimepoint;
        bool m_Stopped;
    };
};
#endif // __ROSTOOLS_PROFILING_H__