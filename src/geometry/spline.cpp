
#include "../geometry/spline.h"

template <typename T>
T Spline<T>::at(float time) const
{

    // A1T1b: Evaluate a Catumull-Rom spline

    // Given a time, find the nearest positions & tangent values
    // defined by the control point map.

    // Transform them for use with cubic_unit_spline

    // Be wary of edge cases! What if time is before the first knot,
    // before the second knot, etc...
    if (!any())
        return T();

    if (knots.size() == 1)
        return knots.begin()->second;

    if (knots.begin()->first >= time)
        return knots.begin()->second;

    if (std::prev(knots.end())->first <= time)
        return std::prev(knots.end())->second;

    float t_2 = knots.upper_bound(time)->first;
    T p_2 = knots.upper_bound(time)->second;

    float t_1 = std::prev(knots.upper_bound(time))->first;
    T p_1 = std::prev(knots.upper_bound(time))->second;

    float t_0;
    T p_0;
    if (std::prev(knots.upper_bound(time)) != knots.begin())
    {
        t_0 = std::prev(std::prev(knots.upper_bound(time)))->first;
        p_0 = std::prev(std::prev(knots.upper_bound(time)))->second;
    }
    else
    {
        t_0 = t_1 - (t_2 - t_1);
        p_0 = p_1 - (p_2 - p_1);
    }

    float t_3;
    T p_3;
    if (std::next(knots.upper_bound(time)) != knots.end())
    {
        t_3 = std::next(knots.upper_bound(time))->first;
        p_3 = std::next(knots.upper_bound(time))->second;
    }
    else
    {
        t_3 = t_2 + t_2 - t_1;
        p_3 = p_2 + p_2 - p_1;
    }

    float normalized_t = (time - t_1) / (t_2 - t_1);
    T tangent0 = (t_2 - t_1) * (p_2 - p_0) / (t_2 - t_0);
    T tangent1 = (t_2 - t_1) * (p_3 - p_1) / (t_3 - t_1);

    return cubic_unit_spline(normalized_t, p_1, p_2, tangent0, tangent1);
}

template <typename T>
T Spline<T>::cubic_unit_spline(float time, const T &position0, const T &position1,
                               const T &tangent0, const T &tangent1)
{

    // A4T1a: Hermite Curve over the unit interval

    // Given time in [0,1] compute the cubic spline coefficients and use them to compute
    // the interpolated value at time 'time' based on the positions & tangents

    // Note that Spline is parameterized on type T, which allows us to create splines over
    // any type that supports the * and + operators.
    float time_cub = time * time * time;
    float time_sqr = time * time;

    float h_00 = 2.0f * time_cub - 3.0f * time_sqr + 1.0f;
    float h_10 = time_cub - 2.0f * time_sqr + time;
    float h_01 = -2.0f * time_cub + 3.0f * time_sqr;
    float h_11 = time_cub - time_sqr;

    return h_00 * position0 + h_10 * tangent0 + h_01 * position1 + h_11 * tangent1;
}

template class Spline<float>;
template class Spline<double>;
template class Spline<Vec4>;
template class Spline<Vec3>;
template class Spline<Vec2>;
template class Spline<Mat4>;
template class Spline<Spectrum>;
