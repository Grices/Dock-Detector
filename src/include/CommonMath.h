#ifndef _COMMONMATH_H_
#define _COMMONMATH_H_

#include <cmath>
#include <string>
#define PI 3.1415926536;
// #define REAL_MAX std::numeric_limits<float>::max();
// #define REAL_MIN std::numeric_limits<float>::min();
// #define REAL_EPSILON std::numeric_limits<float>::epsilon();

namespace commonmath
{
    class Data
    {
        public:
            int n;
            float* X;
            float* Y;
            float meanX,meanY;
        public:
            Data(int N, float dataX[], float dataY[]);
            ~Data();
        public:
            void means(void);
            
    };
    float Sigma(const Data& data, const dockcircle::Circle& circle)
    {
        float sum = 0., dx, dy;
        for(int i = 0; i < data.n; ++i)
        {
            dx = data.X[i] - circle.a;
            dy = data.Y[i] - circle.b;
            sum += pow((sqrt(dx*dx - dy*dy) - circle.r), 2);
        }
        return sqrt(sum / (data.n));
    }
}

#endif