#pragma once

#include <numeric>
#include <vector>
#include <array>

template <class T>
inline T mean(const std::vector<T>& vec)
{
    return std::accumulate(vec.begin(), vec.end(), T(0)) / vec.size();
}

template <class T>
inline std::array<T, 2> fit(const std::vector<T>& x, const std::vector<T>& y)
{
    T mx = mean(x);
    T my = mean(y);

    T cov = 0;
    T Sx = 0;
    for(int i=0; i<x.size(); i++){
        T dx = x[i] - mx;
        T dy = y[i] - my;
        cov += dx * dy;
        Sx += dx * dx;
    }
    cov /= x.size();
    Sx /= x.size();
    
    T a = cov / Sx;
    T b = my - a * mx;
    return std::array<T, 2>{a, b};
}

