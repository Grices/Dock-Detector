#ifndef _GETCIRCLE_H_
#define _GETCIRCLE_H_

#include <string>

namespace dockcircle
{
    class Circle
    {
        public:
            float a,b,r,s,g;
            float Gx,Gy;
            int i,j;
        public:
            Circle() : a(0), b(0), r(1), s(0), i(0), j(0){}
            Circle(float _a,float _b,float _r) : a(_a), b(_b), r(_r){}
    };


    class InterUser
    {
        private:
            std::string m_filepath;
        public:
            InterUser(const std::string& filepath) : m_filepath(filepath) {}
            const Circle* const Fit(void);        
    };

}

#endif