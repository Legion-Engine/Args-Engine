#pragma once
#include <deque>
#include <vector>
#include <list>
#include <string>
#include <array>
#include <queue>
#include <type_traits>
#include <core/platform/platform.hpp>

namespace legion::core
{
#define HAS_FUNC(x)                                                                                                     \
    template<typename, typename T>                                                                                      \
    struct CONCAT(has_, x) {                                                                                            \
        static_assert(std::integral_constant<T, false>::value, "Second template param needs to be of function type.");  \
    };                                                                                                                  \
                                                                                                                        \
    template<typename C, typename Ret, typename... Args>                                                                \
    struct CONCAT(has_, x)<C, Ret(Args...)> {                                                                           \
    private:                                                                                                            \
        template<typename T>                                                                                            \
        static constexpr auto check(T*)                                                                                 \
            -> typename std::is_same<decltype(std::declval<T>(). x (std::declval<Args>()...)), Ret>::type;              \
                                                                                                                        \
        template <typename>                                                                                             \
        static constexpr auto check(...)                                                                                \
            ->std::false_type;                                                                                          \
                                                                                                                        \
        typedef decltype(check<C>(nullptr)) type;                                                                       \
    public:                                                                                                             \
        static constexpr bool value = type::value;                                                                      \
    };                                                                                                                  \
                                                                                                                        \
    template<typename C, typename F>                                                                                    \
    constexpr bool CONCAT_DEFINE(has_, CONCAT(x, _v)) = CONCAT(has_, x)<C, F>::value;


    HAS_FUNC(setup);
    HAS_FUNC(update);

    template<typename derived_type, typename base_type>
    using inherits_from = typename std::enable_if<std::is_base_of<base_type, derived_type>::value, int>::type;

    template<typename derived_type, typename base_type>
    using doesnt_inherit_from = typename std::enable_if<!std::is_base_of<base_type, derived_type>::value, int>::type;

    template<typename T>
    using remove_cvr = std::remove_cv<std::remove_reference_t<T>>;

    template<typename T>
    using remove_cvr_t = typename remove_cvr<T>::type;

    template<typename, typename T>
    struct has_resize
    {
        static_assert(
            std::integral_constant<T, false>::value,
            "Second template param needs to be of function type.");
    };

    template <typename C, typename Ret, typename... Args>
    struct has_resize<C, Ret(Args...)>
    {
    private:
        template<typename T>
        static constexpr auto check(T*)
            -> typename std::is_same<decltype(std::declval<T>().resize(std::declval<Args>()...)), Ret>::type;

        template <typename>
        static constexpr auto check(...)
            ->std::false_type;

        typedef decltype(check<C>(nullptr)) type;
    public:
        static constexpr bool value = type::value;
    };

    template <typename C, typename F>
    constexpr bool has_resize_v = has_resize<C, F>::value;


    template <class T>
    struct is_vector
        : public std::false_type
    {
    };

    template <class T>
    struct is_vector<std::vector<T>>
        : public std::true_type
    {
    };

    template<class T, typename... Args>
    decltype(void(T{ std::declval<Args>()... }), std::true_type())
        brace_construct_test(int);

    template<class T, typename... Args>
    std::false_type
        brace_construct_test(...);

    template<class T, typename... Args>
    struct is_brace_constructible : decltype(brace_construct_test<T, Args...>(0))
    {
    };

    template<class T, typename... Args>
    inline constexpr bool is_brace_constructible_v = is_brace_constructible<T, Args...>::value;


    template <class T>
    struct is_container
        : public std::false_type
    {};

    template <class T>
    struct is_container<std::vector<T>>
        : public std::true_type
    {};

    template <class T>
    struct is_container<std::basic_string<T>>
        :public std::true_type
    {};

    template <class T>
    struct is_container<std::deque<T>>
        :public std::true_type
    {};

    template <class T>
    struct is_container<std::list<T>>
        :public std::true_type
    {};

    template <class T,size_t N>
    struct is_container<std::array<T,N>>
        :public std::true_type
    {};

    template <class T>
    struct is_container<std::queue<T>>
        :public std::true_type
    {};
}
