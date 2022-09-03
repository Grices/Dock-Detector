#include <iostream>
#include <algorithm>
#include <cmath>
#include <memory>
#include "../include/GetCircle.h"
#include "include/Realse.h"
#include "include/CommonMath.h"
#include "include/DataReader.h"

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

    float angle_tms = (data.X[0] - (Xcenter + data.meanX)) * (data.X[data.n-1] - (Xcenter + data.meanX))
                    + (data.Y[0] - (Ycenter + data.meanY)) * (data.Y[data.n-1] - (Ycenter + data.meanY));
    float angle_tmm = sqrt(pow(data.X[0] - (Xcenter + data.meanX), 2) + pow(data.Y[0] - (Ycenter + data.meanY), 2)) 
                    * sqrt(pow(data.X[data.n-1] - (Xcenter + data.meanX), 2) + pow(data.Y[data.n-1] - (Ycenter + data.meanY), 2));

    circle.a = Xcenter + data.meanX;
    circle.b = Ycenter + data.meanY;
    circle.o = acos(angle_tms / angle_tmm);
    circle.r = sqrt(Xcenter*Xcenter + Ycenter*Ycenter + Mz);
    circle.s = commonmath::Sigma(data, circle);
    circle.itera_times = iter;
    circle.size_interval = data.n;

    return circle;
}

dockcircle::Circle* CircleFitting::Fitting(const std::vector<dockcircle::lds_point>& pointcloud_)
{
    commonmath::Timer runtime("Tmclc");
    std::vector<dockcircle::lds_point> res = pointcloud_;
    std::vector<dockcircle::Circle> candidate_class;
    std::vector<dockcircle::lds_point> points_tmp;

    constexpr int simple_init = 65; //采样区间最大值
    constexpr int point_dis_MAX = 100; //相邻点最大间距
    constexpr int fitcir_rad_MAX = 180; //拟合圆最大半径
    constexpr int fitcir_rad_MIN = 140; //拟合圆最小半径
    constexpr float ld_MAX = 2.30F; //弧长比弦长最大值
    constexpr float ld_MIN = 1.65F; //弧长比弦长最小值
    constexpr float rr_MAX = 18.0F; //圆半径的最大差值
    constexpr float angle_MAX = 2.792F; //拟合圆心角最大值
    constexpr float angle_MIN = 1.919F; //拟合圆心角最小值
    constexpr float dis_rh_MAX = 1.35F; //半径比高度最大值
    constexpr float dis_rh_MIN = 0.85F; //半径比高度最小值
    constexpr float tmp_angle_MAX = 0.26F; //中点向量夹角最大值
    constexpr float tmp_angle_MIN = 0.13F; //中点向量夹角最小值
    constexpr float region_r_MAX = 130.0F; //弧半径的最大值
    constexpr float region_r_MIN = 90.0F; //弧半径的最小值
    constexpr float differ_dis_MAX = 0.65F; //内接三角形边长最大比
    constexpr float differ_dis_MIN = 0.55F; //内接三角形边长最小比
    constexpr float sigma_threshold = 15.0F; //拟合误差最大值
    for(int length = simple_init; length >= 60; length -= 5)
    {
        points_tmp.reserve(length);
        for(int loc = 0; loc < int(res.size()); loc++)
        {
            if(res[loc].theta > 2000.0F || res[loc].theta < 500.0F) {continue;}
            points_tmp.clear();
            if(loc + length > res.size()) {break;}
            for(int tmp = loc; tmp < loc + length; tmp++)
            {
                points_tmp.emplace_back(res[tmp]);
            }
            float pos_line_point1 = (points_tmp.front().x - points_tmp.at(length / 2).x)*(points_tmp.back().y - points_tmp.at(length / 2).y) - (points_tmp.back().x - points_tmp.at(length / 2).x)*(points_tmp.front().y - points_tmp.at(length / 2).y);
            if (pos_line_point1 >= 0) { continue; }
           
            float ac_dis = sqrt(pow(points_tmp.back().y - points_tmp.front().y, 2) + pow(points_tmp.back().x - points_tmp.front().x, 2));
            float ab_dis = sqrt(pow(points_tmp.at(length / 2).y - points_tmp.front().y, 2) + pow(points_tmp.at(length / 2).x - points_tmp.front().x, 2));
            float differ_dis = ab_dis / ac_dis;
            if (differ_dis < differ_dis_MIN || differ_dis > differ_dis_MAX){ continue; }

            float actms = (points_tmp.at(length / 5).x - points_tmp.front().x)*(points_tmp.at(length / 3).x - points_tmp.front().x) + (points_tmp.at(length / 5).y - points_tmp.front().y)*(points_tmp.at(length / 3).y - points_tmp.front().y);
            float actmm = sqrt(pow(points_tmp.at(length / 5).x - points_tmp.front().x, 2) + pow(points_tmp.at(length / 5).y - points_tmp.front().y, 2)) * sqrt(pow(points_tmp.at(length / 3).x - points_tmp.front().x, 2) + pow(points_tmp.at(length / 3).y - points_tmp.front().y, 2));
            float tmp_angle = acos(actms / actmm);
            // if (tmp_angle < tmp_angle_MIN || tmp_angle > tmp_angle_MAX) { continue; }

            bool break_flag = false;
            float sum_length = 0;
            for (int m = 1; m < int(points_tmp.size()); m++)
            {
                float neighbor_dis = sqrt(pow(points_tmp.at(m).y - points_tmp.at(m - 1).y, 2) + pow(points_tmp.at(m).x - points_tmp.at(m - 1).x, 2));
                if (neighbor_dis > point_dis_MAX)
                {
                    break_flag = true;
                    loc += m - 1;
                    break;
                }
                if(m < (int(points_tmp.size()) / 2))
                {
                    sum_length += neighbor_dis;
                }
            }
            if (break_flag == true){ continue; }
            float l_d = sum_length / ab_dis;
            if (l_d < ld_MIN || l_d > ld_MAX){ continue; }

            float middle_x = (points_tmp.front().x + points_tmp.back().x) / 2;

            float middle_y = (points_tmp.front().y + points_tmp.back().y) / 2;
            float region_r1 = sqrt(pow(points_tmp[length / 4].x - middle_x, 2) + pow(points_tmp[length / 4].y - middle_y, 2));
            float region_r2 = sqrt(pow(points_tmp[3 * length / 4].x - middle_x, 2) + pow(points_tmp[3 * length / 4].y - middle_y, 2));
            if (region_r1 < region_r_MIN || region_r2 < region_r_MIN || region_r1 > region_r_MAX || region_r2 > region_r_MAX) {continue;}
            float dif_rr = abs(region_r1 - region_r2);
            if (dif_rr > rr_MAX){ continue; }
            float region_h = sqrt(pow(points_tmp[length / 2].x - middle_x, 2) + pow(points_tmp[length / 2].y - middle_y, 2));
            float dif_rh1 = region_r1 / region_h;
            float dif_rh2 = region_r2 / region_h;
            if (dif_rh1 < dis_rh_MIN || dif_rh1 > dis_rh_MAX){ continue; }
            if (dif_rh2 < dis_rh_MIN || dif_rh2 > dis_rh_MAX){ continue; }

            float* xarr = new float[length];
            float* yarr = new float[length];
            float* px = xarr;
            float* py = yarr;
            for (std::vector<dockcircle::lds_point>::iterator wr = points_tmp.begin(); wr != points_tmp.end(); wr++, px++, py++)
            {
                *px = wr->x;
                *py = wr->y;
            }
            commonmath::Data data(length, xarr, yarr);
            dockcircle::Circle cir = CircleFitting::CiecleFit(data);
            if (cir.s > sigma_threshold || cir.r < fitcir_rad_MIN || cir.r > fitcir_rad_MAX) { continue; }
            if (cir.o < angle_MIN || cir.o > angle_MAX) { continue; }

            candidate_class.push_back(cir);
            loc += length;
            delete[]xarr;
            delete[]yarr;
        }
    }
    float cir_rmin = sigma_threshold;
    dockcircle::Circle* temp = nullptr;
    dockcircle::Circle fitcircle;
    if (!candidate_class.empty())
    {
        for (std::vector<dockcircle::Circle>::iterator ii = candidate_class.begin(); ii != candidate_class.end(); ii++)
        {
            if (ii->s < cir_rmin)
            {
                fitcircle = *ii;
                cir_rmin = ii->s;
            }
        }
        temp = new dockcircle::Circle(fitcircle);
        return temp;
    }
    return nullptr;
}

int main()
{
    std::string filepath = "D:/C_Project/A_star/B60/lds_points4.bin";
    int count_main = 0;
    if (filepath.empty()) { return 0; }
    else
    {
        ReadtoPoints test_points(filepath);
        for (std::vector<std::vector<dockcircle::lds_point>>::iterator i = test_points.mf_points.begin(); i != test_points.mf_points.end(); i++)
        {
            std::cout << "--------------------" << std::endl;
            std::cout << "FPS: " << count_main << std::endl;
            CircleFitting target_cir(*i);
            count_main++;
            if (target_cir.GetCircle() != nullptr)
            {
                std::cout << "Sigma:" << target_cir.GetCircle()->s << " Angle:" << target_cir.GetCircle()->o * (180/3.14159265)<< " Radius:" << target_cir.GetCircle()->r << std::endl;
                std::cout << "Position:" << target_cir.GetCircle()->a << "," << target_cir.GetCircle()->b << std::endl;
                std::cout << "Size:" << target_cir.GetCircle()->size_interval << std::endl;
            }
            else
            {
                std::cout << "There is no target" << std::endl;
            }
        }
        system("pause");
        return 0;
    }
}