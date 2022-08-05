#ifndef _COMMONMATH_H_
#define _COMMONMATH_H_

#include <string>

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
    float Sigma(const Data& data, const dockcircle::Circle& circle);
}

#endif