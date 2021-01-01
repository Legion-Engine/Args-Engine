#pragma once

#define SPDLOG_HEADER_ONLY
#include <sstream>

#if !defined(DOXY_EXCLUDE)
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/pattern_formatter.h>
#endif

#include <core/platform/platform.hpp>
#include <thread>
#include <core/math/math.hpp>
#include <core/common/exception.hpp>

/** @file logging.hpp */
#if !defined(DOXY_EXCLUDE)
namespace fmt
{
    template <>
    struct formatter<std::thread::id>
    {

        constexpr auto parse(format_parse_context& ctx)
        {
            auto it = ctx.begin(), end = ctx.end();

            if (it != end && *it != '}')
                throw format_error("invalid format");
            return it++;
        }

        template <typename FormatContext>
        auto format(const std::thread::id& p, FormatContext& ctx) {
            std::ostringstream oss;
            oss << p;
            return format_to(ctx.out(), "{}", oss.str());
        }

    };

    template <>
    struct formatter<legion::core::exception>
    {

        constexpr auto parse(format_parse_context& ctx)
        {
            auto it = ctx.begin(), end = ctx.end();

            if (it != end && *it != '}')
                throw format_error("invalid format");
            return it++;
        }

        template <typename FormatContext>
        auto format(const legion::core::exception& error, FormatContext& ctx)
        {
            return format_to(ctx.out(), "[{}({}) T {}(...)] {}", error.file(), error.line(), error.func(), error.what());
        }
    };

    template <>
    struct formatter<legion::core::fs_error>
    {

        constexpr auto parse(format_parse_context& ctx)
        {
            auto it = ctx.begin(), end = ctx.end();

            if (it != end && *it != '}')
                throw format_error("invalid format");
            return it++;
        }

        template <typename FormatContext>
        auto format(const legion::core::fs_error& error, FormatContext& ctx)
        {
            return format_to(ctx.out(), "[{}({}) T {}(...)] {}", error.file(), error.line(), error.func(), error.what());
        }
    };

    template <>
    struct fmt::formatter<legion::core::math::vec2> {
        // Presentation format: 'f' - fixed, 'e' - exponential.
        char presentation = 'f';

        // Parses format specifications of the form ['f' | 'e'].
        constexpr auto parse(format_parse_context& ctx) {
            // auto parse(format_parse_context &ctx) -> decltype(ctx.begin()) // c++11
              // [ctx.begin(), ctx.end()) is a character range that contains a part of
              // the format string starting from the format specifications to be parsed,
              // e.g. in
              //
              //   fmt::format("{:f} - point of interest", point{1, 2});
              //
              // the range will contain "f} - point of interest". The formatter should
              // parse specifiers until '}' or the end of the range. In this example
              // the formatter should parse the 'f' specifier and return an iterator
              // pointing to '}'.

              // Parse the presentation format and store it in the formatter:
            auto it = ctx.begin(), end = ctx.end();
            if (it != end && (*it == 'f' || *it == 'e')) presentation = *it++;

            // Check if reached the end of the range:
            if (it != end && *it != '}')
                throw format_error("invalid format");

            // Return an iterator past the end of the parsed range:
            return it;
        }

        // Formats the point p using the parsed format specification (presentation)
        // stored in this formatter.
        template <typename FormatContext>
        auto format(const legion::core::math::vec2& p, FormatContext& ctx) {
            // auto format(const point &p, FormatContext &ctx) -> decltype(ctx.out()) // c++11
              // ctx.out() is an output iterator to write to.
            return format_to(
                ctx.out(),
                presentation == 'f' ? "({:.1f}, {:.1f})" : "({:.1e}, {:.1e})",
                p.x, p.y);
        }
    };

    template <>
    struct fmt::formatter<legion::core::math::ivec2> {

        constexpr auto parse(format_parse_context& ctx) {
            auto it = ctx.begin(), end = ctx.end();

            if (it != end && *it != '}')
                throw format_error("invalid format");
            return it++;
        }

        template <typename FormatContext>
        auto format(const legion::core::math::ivec2& p, FormatContext& ctx) {
            return format_to(
                ctx.out(),
                "({}, {})",
                p.x, p.y);
        }
    };

    template <>
    struct fmt::formatter<legion::core::math::vec3> {
        char presentation = 'f';

        constexpr auto parse(format_parse_context& ctx) {
            auto it = ctx.begin(), end = ctx.end();
            if (it != end && (*it == 'f' || *it == 'e')) presentation = *it++;

            if (it != end && *it != '}')
                throw format_error("invalid format");

            return it;
        }

        template <typename FormatContext>
        auto format(const legion::core::math::vec3& p, FormatContext& ctx) {
            return format_to(
                ctx.out(),
                presentation == 'f' ? "({:.1f}, {:.1f}, {:.1f})" : "({:.1e}, {:.1e}, {:.1e})",
                p.x, p.y, p.z);
        }
    };

    template <>
    struct fmt::formatter<legion::core::math::vec4> {
        char presentation = 'f';

        constexpr auto parse(format_parse_context& ctx) {
            auto it = ctx.begin(), end = ctx.end();
            if (it != end && (*it == 'f' || *it == 'e')) presentation = *it++;

            if (it != end && *it != '}')
                throw format_error("invalid format");

            return it;
        }

        template <typename FormatContext>
        auto format(const legion::core::math::vec4& p, FormatContext& ctx) {
            return format_to(
                ctx.out(),
                presentation == 'f' ? "({:.1f}, {:.1f}, {:.1f}, {:.1f})" : "({:.1e}, {:.1e}, {:.1e}, {:.1e})",
                p.x, p.y, p.z, p.w);
        }
    };

    template <>
    struct fmt::formatter<legion::core::math::quat> {
        char presentation = 'f';

        constexpr auto parse(format_parse_context& ctx) {
            auto it = ctx.begin(), end = ctx.end();
            if (it != end && (*it == 'f' || *it == 'e')) presentation = *it++;

            if (it != end && *it != '}')
                throw format_error("invalid format");

            return it;
        }

        template <typename FormatContext>
        auto format(const legion::core::math::quat& p, FormatContext& ctx) {
            return format_to(
                ctx.out(),
                presentation == 'f' ? "(({:.1f}, {:.1f}, {:.1f}),r: {:.1f})" : "(({:.1e}, {:.1e}, {:.1e}),r: {:.1e})",
                p.x, p.y, p.z, p.w);
        }
    };

}
#endif


namespace legion::core::log
{
    /** @brief Holds the non const static data of logging. */
    struct impl {
        static cstring log_file;
        static std::shared_ptr<spdlog::logger> logger;
        static std::shared_ptr<spdlog::logger> file_logger;
        static std::shared_ptr<spdlog::logger> console_logger;
        static std::unordered_map<std::thread::id, std::string> thread_names;
    };


    /** @brief the time point at which the engine started */
    const static inline std::chrono::time_point<std::chrono::high_resolution_clock> genesis = std::chrono::high_resolution_clock::now();

    /** @class genesis_formatter_flag
     *  @brief Custom formatter flag that prints the time since the engine started in seconds.milliseconds
     */
    class genesis_formatter_flag : public spdlog::custom_flag_formatter
    {
    public:
        void format(const spdlog::details::log_msg& msg, const std::tm& tm_time, spdlog::memory_buf_t& dest) override
        {
            //get seconds since engine start
            const auto now = std::chrono::high_resolution_clock::now();
            const auto time_since_genesis = now - genesis;
            const auto seconds = std::chrono::duration_cast<std::chrono::duration<float, std::ratio<1, 1>>>(time_since_genesis).count();

            //convert to "--s.ms---"
            const auto str = std::to_string(seconds);

            //append to data
            dest.append(str.data(), str.data() + str.size());

        }

        //generates a new formatter flag
        L_NODISCARD std::unique_ptr<custom_flag_formatter> clone() const override
        {
            return spdlog::details::make_unique<genesis_formatter_flag>();
        }
    };

    /** @class thread_name_formatter_flag
     *  @brief Prints the name of the thread (if available) and otherwise the the TID.
     */
    class thread_name_formatter_flag : public spdlog::custom_flag_formatter
    {
        void format(const spdlog::details::log_msg& msg, const std::tm& tm_time, spdlog::memory_buf_t& dest) override
        {
            std::string thread_ident;

            if (const auto it = impl::thread_names.find(std::this_thread::get_id()); it != impl::thread_names.end())
            {
                thread_ident = it->second;
            }
            else
            {
                std::ostringstream oss;
                oss << std::this_thread::get_id();
                thread_ident = oss.str();

                //NOTE(algo-ryth-mix): this conversion is not portable 
                //thread_ident = std::to_string(legion::core::force_value_cast<uint>(std::this_thread::get_id()));
            }

            dest.append(thread_ident.data(), thread_ident.data() + thread_ident.size());
        }
        std::unique_ptr<custom_flag_formatter> clone() const override
        {
            return spdlog::details::make_unique<thread_name_formatter_flag>();

        }
    };


#define logger impl::logger


    /** @brief sets up logging (do not call, invoked by engine) */
    inline void setup()
    {
#if defined(LEGION_KEEP_CONSOLE) || defined(LEGION_DEBUG)
        logger = impl::console_logger;
#else
        impl::file_logger = spdlog::rotating_logger_mt(impl::log_file, impl::log_file, 1'048'576, 5);
        logger = impl::file_logger;
#endif
        auto f = std::make_unique<spdlog::pattern_formatter>();

        f->add_flag<thread_name_formatter_flag>('f');
        f->add_flag<genesis_formatter_flag>('*');
        f->set_pattern("T+ %* [%^%=7l%$] [%=13!f] : %v");

        logger->set_formatter(std::move(f));
    }

    /** @brief selects the severity you want to filter for or print with */
    enum class severity
    {
        trace,   // lowest severity
        debug,
        info,
        warn,
        error,
        fatal // highest severity
    };

    inline spdlog::level::level_enum args2spdlog(severity s)
    {
        switch (s)
        {
        case severity::trace:return spdlog::level::trace;
        case severity::debug:return spdlog::level::debug;
        case severity::info: return spdlog::level::info;
        case severity::warn: return spdlog::level::warn;
        case severity::error:return spdlog::level::err;
        case severity::fatal:return spdlog::level::critical;
        }
    }

    /** @brief prints a log line, using the specified `severity`
     *  @param s The severity you wan't to report this log with
     *  @param format The format string you want to print
     *  @param a The arguments to the format string
     *  @note This uses fmt lib style syntax check
     *         https://fmt.dev/latest/syntax.html
     */
    template <class... Args, class FormatString>
    void println(severity s, const FormatString& format, Args&&... a)
    {
        logger->log(args2spdlog(s),format,std::forward<Args>(a)...);
    }


    /** @brief prints a log line, using the specified `severity`
     *  @param level selects the severity level you are interested in
     */
    inline void filter(severity level)
    {
        logger->set_level(args2spdlog(level));
    }

     /** @brief same as println but with severity = trace */
    template<class... Args, class FormatString>
    void trace(const FormatString& format, Args&&... a)
    {
        println(severity::trace, format, std::forward<Args>(a)...);
    }

    /** @brief same as println but with severity = debug */
    template<class... Args, class FormatString>
    void debug(const FormatString& format, Args&&...a)
    {
        println(severity::debug, format, std::forward<Args>(a)...);
    }

    /** @brief same as println but with severity = info */
    template<class... Args, class FormatString>
    void info(const FormatString& format, Args&&...a)
    {
        println(severity::info, format, std::forward<Args>(a)...);
    }

    /** @brief same as println but with severity = warn */
    template<class... Args, class FormatString>
    void warn(const FormatString& format, Args&&...a)
    {
        println(severity::warn, format, std::forward<Args>(a)...);
    }

    /** @brief same as println but with severity = error */
    template<class... Args, class FormatString>
    void error(const FormatString& format, Args&&...a)
    {
        println(severity::error, format, std::forward<Args>(a)...);
    }

    /** @brief same as println but with severity = fatal */
    template<class... Args, class FormatString>
    void fatal(const FormatString& format, Args&&...a)
    {
        println(severity::fatal, format, std::forward<Args>(a)...);
    }

}
#undef logger
