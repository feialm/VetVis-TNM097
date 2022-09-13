#pragma once

#include <modules/tnm067lab1/tnm067lab1moduledefine.h>
#include <inviwo/core/util/glm.h>


namespace inviwo {

template <typename T>
struct float_type {
    using type = double;
};

template <>
struct float_type<float> {
    using type = float;
};
template <>
struct float_type<vec3> {
    using type = float;
};
template <>
struct float_type<vec2> {
    using type = float;
};
template <>
struct float_type<vec4> {
    using type = float;
};

namespace TNM067 {
namespace Interpolation {

#define ENABLE_LINEAR_UNITTEST 1
template <typename T, typename F = double>
T linear(const T& a, const T& b, F x) {
    if (x <= 0) return a;
    if (x >= 1) return b;

    // linjär här men skriv bilinjär i cpp filen
    T XX = a * (1 - x) + b * x;
    //a,b - koefficienter

    return  XX;
}

// clang-format off
    /*
     2------3
     |      |
    y|  •   |
     |      |
     0------1
        x
    */
    // clang format on
#define ENABLE_BILINEAR_UNITTEST 1
template<typename T, typename F = double> 
T bilinear(const std::array<T, 4> &v, F x, F y) {

    T l1 = linear(v[0], v[1], x);
    T l2 = linear(v[2], v[3], x);
    T l3 = linear(l1, l2, y);

    return l3;
}


    // clang-format off
    /* 
    a--•----b------c
    0  x    1      2
    */
// clang-format on
#define ENABLE_QUADRATIC_UNITTEST 1
template <typename T, typename F = double>
T quadratic(const T& a, const T& b, const T& c, F x) {

    //le.2 slide 45
    // x = t
    // a = f1
    // b = f2
    // c = f3

    T f = (1-x) * (1-2*x) * a + 4 * x * (1-x) * b + x * (2*x-1) * c;
    return f;
}

// clang-format off
    /* 
    6-------7-------8
    |       |       |
    |       |       |
    |       |       |
    3-------4-------5
    |       |       |
   y|  •    |       |
    |       |       |
    0-------1-------2
    0  x    1       2
    */
// clang-format on
#define ENABLE_BIQUADRATIC_UNITTEST 1
template <typename T, typename F = double>
T biQuadratic(const std::array<T, 9>& v, F x, F y) {
    
    T q1 = quadratic(v[0], v[1], v[2], x);
    T q2 = quadratic(v[3], v[4], v[5], x);
    T q3 = quadratic(v[6], v[7], v[8], x);

    T q4 = quadratic(q1, q2, q3, y);

    return q4;
}

// clang-format off
    /*
     2---------3
     |'-.      |
     |   -,    |
   y |  •  -,  |
     |       -,|
     0---------1
        x
    */
// clang-format on
#define ENABLE_BARYCENTRIC_UNITTEST 0
template <typename T, typename F = double>
T barycentric(const std::array<T, 4>& v, F x, F y) {

    // We want to find alpha,beta,gamma coordinates and A,B,C vertices
    // Lec.2, slide 50: P(alpha,beta,gamma) = alpha * A + beta * B + gamma * C
    // We know: alpha + beta + gamma = 1

    F alpha, beta, gamma;
    T p_Alpha, p_Beta, p_Gamma; //A,B,C

    p_Beta = v[1]; // B
    p_Gamma = v[2]; // C

    // We know: alpha + beta + gamma = 1
    // determinate alpha, beta and gamma
    // x and y is (x,y) coordinates for point P in triangle, with that we can figure out the variables
    // overall: there are ratios to consider!
    if (x + y < 1.0f) { // point P is situated 
        alpha = 1.0 - y - x;
        beta = y;
        gamma = x;

        p_Alpha = v[0]; // A
    }
    else {
        //alpha + beta + gamma > 1
        alpha = y + x - 1.0;
        beta = 1.0 - x;
        gamma = 1.0 - y;

        p_Alpha = v[3]; // A

    }

    //P(alpha, beta, gamma) = alpha * A + beta * B + gamma * C
    T p_abg = alpha * p_Alpha + beta * p_Beta + gamma * p_Gamma;



    return p_abg;
}

}  // namespace Interpolation
}  // namespace TNM067
}  // namespace inviwo
