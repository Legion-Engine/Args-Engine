#pragma once
#include <algorithm>
#include <cctype>
#include <locale>
#include <vector>
#include <sstream>
#include <iterator>
#include <core/types/primitives.hpp>
#include <cstring>

#include <Optick/optick.h>

/**
 * @file string_extra.hpp
 */
namespace legion::core::common {

    inline bool ends_with(const std::string& src, cstring value)
    {
        auto valueLength = strlen(value);
        if (src.size() < valueLength)
            return false;

        std::string_view srcEnd(src.data() + (src.size() - valueLength), valueLength);

        for (int i = 0; i < valueLength; i++)
        {
            if (srcEnd[i] != value[i])
                return false;
        }

        return true;
    }

    /**@brief Replace each occurrence of [item] in [source] with [value].
     * @return Amount of items replaced.
     */
    template<typename StringType>
    inline size_t replace_items(StringType& source, const std::string& item, const std::string& value)
    {
        OPTICK_EVENT();
        size_t count = 0;
        auto it = source.begin();

        while ((it = std::search(it, source.end(), item.begin(), item.end())) != source.end()) // While there's items to be found, keep replacing them with value.
        {
            count++;
            source.erase(it, it + item.size());
            source.insert(it, value.begin(), value.end());
        }

        return count;
    }

    struct isChars
    {
        isChars(const char* chars) :_chars(chars) {}

        bool operator()(char c) const
        {
            OPTICK_EVENT();
            for (auto testChar = _chars; *testChar != 0; ++testChar)
                if (*testChar == c) return true;
            return false;
        }
        const char* _chars;
    };


    template <typename t>
    std::string string_from_data(t data);
    template <typename T>
    T data_from_string_(std::string str);



    template <char token, char... tokens>
    bool str_tokens_helper(std::ctype<char>::mask* rc)
    {
        OPTICK_EVENT();
        rc[token] = std::ctype<char>::space;
        if constexpr (sizeof...(tokens) != 0) str_tokens_helper<tokens...>(rc);
        return true;
    }

    //dilems helper classlo
    template <char token, char... tokens>
    struct str_tokens : std::ctype<char>
    {
        str_tokens() : ctype<char>(get_table())
        {
        }

        static mask const* get_table()
        {
            //create const and normal rc
            static const mask* const_rc = std::use_facet<std::ctype<char>>(std::locale()).table();
            static mask rc[table_size];
            static auto memory = memcpy(rc, const_rc, table_size * sizeof(mask));
            static char memory1 = rc[' '] = std::ctype<char>::digit;
            static char memory2 = rc['\t'] = std::ctype<char>::digit;
            static bool memory3 = str_tokens_helper<token, tokens...>(rc);
            //set spaces

            return rc;
        }
    };

    //split string at any given char via variadic template and insert into vector
    template <char token_1, char... token>
    std::vector<std::string> split_string_at(const std::string& string)
    {
        OPTICK_EVENT();
        //copy string into stringstream
        std::stringstream ss(string);

        //set tokens
        ss.imbue(std::locale(std::locale(), new str_tokens<token_1, token...>()));

        //create iterators
        const std::istream_iterator<std::string> begin(ss);
        const std::istream_iterator<std::string> end;

        //copy using iterators
        const std::vector<std::string> vstrings(begin, end);

        return vstrings;
    }

    template <const char* const delim, typename Range, typename Value = typename Range::value_type>
    std::string join_strings_with(Range const& elements) {
        OPTICK_EVENT();
        std::ostringstream os;
        auto b = begin(elements), e = end(elements);

        if (b != e) {
            std::copy(b, prev(e), std::ostream_iterator<Value>(os, delim));
            b = prev(e);
        }
        if (b != e) {
            os << *b;
        }

        return os.str();
    }

    template <typename Range, typename Value = typename Range::value_type>
    std::string join_strings_with(Range const& elements, const char* const delim) {
        OPTICK_EVENT();
        std::ostringstream os;
        auto b = begin(elements), e = end(elements);

        if (b != e) {
            std::copy(b, prev(e), std::ostream_iterator<Value>(os, delim));
            b = prev(e);
        }
        if (b != e) {
            os << *b;
        }

        return os.str();
    }

    template <const char delim, typename Range, typename Value = typename Range::value_type>
    std::string join_strings_with(Range const& elements) {
        OPTICK_EVENT();
        std::ostringstream os;
        auto b = begin(elements), e = end(elements);

        char promoter[2] = { delim };

        if (b != e) {
            std::copy(b, prev(e), std::ostream_iterator<Value>(os, delim));
            b = prev(e);
        }
        if (b != e) {
            os << *b;
        }

        return os.str();
    }
    template <typename Range, typename Value = typename Range::value_type>
    std::string join_strings_with(Range const& elements, char delim) {
        OPTICK_EVENT();
        std::ostringstream os;
        auto b = begin(elements), e = end(elements);

        char promoter[2] = { delim };

        if (b != e) {
            std::copy(b, prev(e), std::ostream_iterator<Value>(os, promoter));
            b = prev(e);
        }
        if (b != e) {
            os << *b;
        }

        return os.str();
    }

    //remove given word from string at offset and return true if success and false on fail
    inline bool find_and_remove_at(std::string& src, const std::string& search, size_t offset = 0)
    {
        OPTICK_EVENT();
        //create temporary
        size_t loc;

        //find given word
        if ((loc = src.find(search, offset)) == std::string::npos) return false;

        //erase word
        src.erase(loc, search.size());

        return true;
    }

    //remove given word from string at offset and return the position
    inline size_t locate_and_delete_at(std::string& src, const std::string& search, size_t offset = 0)
    {
        OPTICK_EVENT();
        //create temporary
        size_t loc;

        //find given word
        if ((loc = src.find(search, offset)) == std::string::npos) return std::string::npos;

        //erase word
        src.erase(loc, search.size());

        return loc;
    }

    //find the nearest of given tokens in a string at an offset
    template <char token_1, char... tokens>
    inline size_t nearest_of_any_at(std::string string, size_t offset = 0)
    {
        OPTICK_EVENT();
        //create tokens using variadic unfolding
        std::vector<char> tokensVector = { token_1 , tokens... };

        //create locations
        std::vector<size_t> locationsVector(tokensVector.size());

        //find locations
        for (char& token : tokensVector) locationsVector.push_back(string.find(token, offset));

        //find smallest element and return
        return *min_element(begin(locationsVector), end(locationsVector));
    }


    template <>
    inline std::string data_from_string_<std::string>(std::string str)
    {
        return str;
    }

    template <>
    inline std::vector<std::string> data_from_string_<std::vector<std::string>>(std::string str)
    {
        OPTICK_EVENT();
        return split_string_at<',', ' '>(str);
    }

    template <typename T>
    inline T data_from_string_(std::string str)
    {
        OPTICK_EVENT();
        std::stringstream temp(str);
        T value;
        temp >> value;
        return value;
    }


    template <>
    inline std::string string_from_data<std::string>(std::string data)
    {
        return data;
    }

    template <>
    inline std::string string_from_data<std::vector<std::string>>(std::vector<std::string> data)
    {
        OPTICK_EVENT();
        std::string ret;
        for (const std::string& str : data)
            ret += str;
        return ret;
    }


    template <typename t>
    inline std::string string_from_data(t data)
    {
        OPTICK_EVENT();
        std::stringstream temp;
        temp << data;
        return temp.str();
    }
    template <char D = '.'>
    constexpr size_t count_delimiter(const char* str)
    {
        return str[0] == '\0' ? 0 : str[0] == D ? count_delimiter<D>(&str[1]) + 1 : count_delimiter<D>(&str[1]);
    }

    constexpr size_t cstrptr_length(const char* str)
    {
        return *str ? 1 + cstrptr_length(str + 1) : 0;
    }


    template <class T, size_t N> constexpr size_t arr_length(T[N])
    {
        return N;
    }

    // trim from start (in place)
    static inline void ltrim(std::string& s) {
        s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](int ch) {
            return !std::isspace(ch);
            }));
    }

    template <class Trimmer>
    static inline void ltrim(std::string& s, Trimmer&& t)
    {
        s.erase(std::find_if(s.begin(), s.end(), [&t](int ch) {
            return !t(ch);
            }), s.end());
    }

    // trim from end (in place)
    static inline void rtrim(std::string& s) {
        s.erase(std::find_if(s.rbegin(), s.rend(), [](int ch) {
            return !std::isspace(ch);
            }).base(), s.end());
    }
    template <class Trimmer>
    static inline void rtrim(std::string& s, Trimmer&& t)
    {
        s.erase(std::find_if(s.rbegin(), s.rend(), [&t](int ch) {
            return !t(ch);
            }).base(), s.end());
    }


    // trim from both ends (in place)
    static inline void trim(std::string& s) {
        ltrim(s);
        rtrim(s);
    }

    template <class Trimmer>
    static inline void trim(std::string& s, Trimmer&& t)
    {
        ltrim(s, t);
        rtrim(s, t);
    }

    // trim from start (copying)
    static inline std::string ltrim_copy(std::string s) {
        ltrim(s);
        return s;
    }

    // trim from end (copying)
    static inline std::string rtrim_copy(std::string s) {
        rtrim(s);
        return s;
    }

    // trim from both ends (copying)
    static inline std::string trim_copy(std::string s) {
        trim(s);
        return s;
    }
    template<class Trimmer>
    // trim from start (copying)
    static inline std::string ltrim_copy(std::string s, Trimmer&& t) {
        ltrim(s, t);
        return s;
    }
    template<class Trimmer>
    // trim from end (copying)
    static inline std::string rtrim_copy(std::string s, Trimmer&& t) {
        rtrim(s, t);
        return s;
    }
    template<class Trimmer>
    // trim from both ends (copying)
    static inline std::string trim_copy(std::string s, Trimmer&& t) {
        trim(s, t);
        return s;
    }

}
