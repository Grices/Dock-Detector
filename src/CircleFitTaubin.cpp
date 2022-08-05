#include <iostream>
#include <algorithm>
#include <cmath>
#include <memory>
#include "../include/GetCircle.h"
#include "include/Realse.h"
#include "include/CommonMath.h"

#include "include/DataReader.h"

dockcircle::Circle CircleFitting::CiecleFit(commonmath::Data& data)
{
    int i, iter, IterMAX = 10;
    float Xi, Yi, Zi;
    float Mz, Mxx, Mxy, Mxz, Myy, Myz, Mzz, Cov_xy, Var_z;

    dockcircle::Circle circle;
    data.means();

    Mxx = Myy = Mxy = Mxz = Myz = Mzz = 0;
    for(i = 0; i < data.n; ++i)
    {
        Xi = data.X[i] - data.meanX;
        Yi = data.Y[i] - data.meanY;
        Zi = Xi*Xi + Yi*Yi;
        Mxx += Xi*Xi; Mxy += Xi*Yi; Mxz += Xi*Zi;
        Myy += Yi*Yi; Myz += Yi*Zi; Mzz += Zi*Zi;
    }
    Mxx /= data.n; Mxy /= data.n; Mxz /= data.n;
    Myy /= data.n; Myz /= data.n; Mzz /= data.n;

    float A0, A1, A2, A3;
    Mz = Mxx + Myy;
    Cov_xy = Mxx*Myy - pow(Mxy,2);
    Var_z = Mzz - Mz*Mz;
    A3 = (4)*Mz; A2 = (-3)*Mz*Mz -Mzz;
    A1 = Var_z*Mz + (4)*Cov_xy*Mz - Mxz*Mxz - Myz*Myz;
    A0 = Mxz*(Mxz*Myy - Myz*Mxz) + Myz*(Myz*Mxx - Mxz*Mxy) - Var_z*Cov_xy;
    float A22 = A2 + A2; float A33 = A3 + A3 + A3;

    float x = 0., y = A0;
    float Dy, xnew, ynew;
    for(iter = 0; iter < IterMAX; iter++)
    {
        Dy = A1 + x*(A22 + A33*x);
        xnew = x - y/Dy;
        if((xnew == x) || (!std::isfinite(xnew))) {break;}
        ynew = A0 + xnew*(A1 + xnew*(A2 + xnew*A3));
        if(abs(ynew) >= abs(y)) {break;}
        x = xnew; y = ynew;
    }
    float DET = x*x - x*Mz + Cov_xy;
    float Xcenter = (Mxz*(Myy - x) - Myz*Mxy) / DET / 2;
    float Ycenter = (Myz*(Mxx - x) - Mxz*Mxy) / DET / 2;

    circle.a = Xcenter + data.meanX;
    circle.b = Ycenter + data.meanY;
    circle.r = sqrt(Xcenter*Xcenter + Ycenter*Ycenter + Mz);
    circle.s = commonmath::Sigma(data, circle);
    circle.i = 0;
    circle.j = iter;

    return circle;
}

dockcircle::Circle* CircleFitting::Fitting(const std::vector<dockcircle::lds_point>& pointcloud_)
{
    std::vector<dockcircle::lds_point> res = pointcloud_;
    std::vector<dockcircle::Circle> candidate_class;

    static const int A = 110;
    static const int D = 100;
    static const int E = 15;

    for(int C = A; C > 40; C-=20)
    {
        for(int k = 0; k < int(res.size() - C); ++k)
        {
            if(res[k].theta > 2000) {continue;}
            float pos_linepoint = (res[k].x - res[k + C/2].x)*(res[k + C].y - res[k + C/2].y) - (res[k + C].x - res[k + C/2].x)*(res[k].y - res[k + C/2].y);
            if(pos_linepoint >= 0) {continue;}
            float ac_dis = sqrt(pow(res[k + C].y - res[k].y, 2) + pow(res[k + C].x - res[k].x, 2));
            float ab_dis = sqrt(pow(res[k + C/2].y - res[k].y, 2) + pow(res[k + C/2].x - res[k].x, 2));
            float bc_dis = sqrt(pow(res[k + C].y - res[k + C/2].y, 2) + pow(res[k + C].x - res[k + C/2].x, 2));
            float differ_dis = (ab_dis + bc_dis) / ac_dis;
            if(differ_dis<1.2 || differ_dis>2.3) {continue;}

            bool break_flag = false;
            for(int m = k; m < k+C; ++m)
            {
                float neighbor_dis = pow(res[m+1].y - res[m].y, 2) + pow(res[m+1].x - res[m].x, 2);
                if(neighbor_dis > D*D) {break_flag = true; k = m+1; break;}
            }
            if(break_flag == true) {continue;}

            float* xarr = new float[C];
            float* yarr = new float[C];
            float* px = xarr;
            float* py = yarr;
            for(int t = k; t < k+C; ++t, ++px, ++py)
            {
                *px = res[t].x;
                *py = res[t].y;
            }
            commonmath::Data data(C, xarr, yarr);
            dockcircle::Circle cir = CircleFitting::CiecleFit(data);
            delete[]xarr;
            delete[]yarr;
            k += C;
            candidate_class.push_back(cir);
        }
        float mins = 100.0;
        dockcircle::Circle* temp = nullptr;
        dockcircle::Circle fitcircle;
        if(!candidate_class.empty())
        {
            for(std::vector<dockcircle::Circle>::iterator ii = candidate_class.begin(); ii != candidate_class.end(); ++ii)
            {
                std::cout << (*ii).s << std::endl;
                if((*ii).s < E)
                {
                    fitcircle = *ii;
                    mins = (*ii).s;
                    temp = new dockcircle::Circle(fitcircle);
                    return temp;
                }
                else {break;}
            }
        }
    }
    return nullptr;
}

const dockcircle::Circle* const CircleFitting::GetCircle(void) const
{
    return this->finalcir;
}
CircleFitting::CircleFitting(const std::vector<dockcircle::lds_point>& pointcloud_)
{
    this->finalcir = Fitting(pointcloud_);
}
const dockcircle::Circle* const dockcircle::InterUser::Fit(void)
{
    std::shared_ptr<CircleFitting> fit_ptr = std::make_shared<CircleFitting>(this->m_pointcloud);
    return fit_ptr->GetCircle();
}

int main()
{
    std::string filepath = "";
    if(filepath.empty()) {return 0;}
    else
    {
        ReadtoPoints test_points(filepath);
        std::vector<dockcircle::lds_point> points_stl = test_points.m_read_points;
        CircleFitting target_cir(points_stl);
        
        if(target_cir.GetCircle() != nullptr)
        {
            std::cout << "Sigma:" << target_cir.GetCircle()->s << " Position:" << target_cir.GetCircle()->a << "," << target_cir.GetCircle()->b << std::endl;
        }
        else
        {
            std::cout << "There is no target" << std::endl;
        }
        return 0;
    }
}