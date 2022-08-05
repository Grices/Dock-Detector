#include <cmath>
#include <limits>
#include "include/CommonMath.h"
#include "../include/GetCircle.h"

namespace commonmath
{
    Data::Data(int N, float dataX[], float dataY[])
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

    void Data::means(void)
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

    Data::~Data()
    {
        delete []X;
        delete []Y;           
    }

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

} // namespace commonmath