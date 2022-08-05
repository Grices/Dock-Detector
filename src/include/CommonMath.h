#ifndef _COMMONMATH_H_
#define _COMMONMATH_H_

#include <string>
#include "../../include/GetCircle.h"

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
            Data(int N, float dataX[], float dataY[])
            {
                n = N;
                X = new float[n];
                Y = new float[n];
                for(int i = 0; i < n; ++i)
                {
                    X[i] = dataX[i];
                    Y[i] = dataY[i];
                }
            }
            ~Data()
            {
                delete []X;
                delete []Y;           
            }
        public:
            void means(void)
            {
                meanX = 0; meanY = 0;
                for(int i = 0; i < n; ++i)
                {
                    meanX += X[i];
                    meanY += Y[i];
                }
                meanX /= n;
                meanY /= n;
            }     
    };
    float Sigma(const Data& data, const dockcircle::Circle& circle)
        {
            float sum = 0., dx, dy;
            for(int i = 0; i < data.n; ++i)
            {
                dx = data.X[i] - circle.a;
                dy = data.Y[i] - circle.b;
                sum += pow((sqrt(dx*dx + dy*dy) - circle.r), 2);
            }
            return sqrt(sum / (data.n));
        }
}

#endif