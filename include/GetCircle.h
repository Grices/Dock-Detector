#ifndef _GETCIRCLE_H_
#define _GETCIRCLE_H_

#include <string>
#include <vector>


namespace dockcircle
{
    struct lds_point
    {
        float x, y, rho, theta;
        lds_point(){}
        lds_point(float x_, float y_, float r_, float t_) : x(x_), y(y_), rho(r_), theta(t_){}
    };

    class Circle
    {
        public:
            float a,b,r,s,o;
            int itera_times, size_interval;
        public:
            Circle() : a(0), b(0), r(0), s(0), o(0), itera_times(0), size_interval(0){}
            Circle(float _a,float _b,float _r) : a(_a), b(_b), r(_r){}
    };

    class InterUser
    {
        private:
            std::vector<lds_point> m_pointcloud;
        public:
            InterUser(const std::vector<lds_point>& pointcloud_):m_pointcloud(pointcloud_){}
            const Circle* const Fit(void);        
    };

}

#endif