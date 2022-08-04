#ifndef _RELEASE_H_
#define _RELEASE_H_

#include "../../include/GetCircle.h"
#include "CommonMath.h"

class CircleFitting
{
    private:
        std::string filepath;
        dockcircle::Circle* finalcir;

        dockcircle::Circle CiecleFit(commonmath::Data& data);
        dockcircle::Circle* Fitting(const std::string& filepath);
    
    public:
        CircleFitting(const std::string& _filepath);
        const dockcircle::Circle* const GetCircle(void) const;
};

#endif