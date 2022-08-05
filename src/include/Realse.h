#ifndef _RELEASE_H_
#define _RELEASE_H_

#include "../../include/GetCircle.h"
#include "CommonMath.h"

class CircleFitting
{
    private:
        dockcircle::Circle* finalcir;
        dockcircle::Circle CiecleFit(commonmath::Data& data);
        dockcircle::Circle* Fitting(const std::vector<dockcircle::lds_point>& pointcloud_);
    
    public:
        CircleFitting(const std::vector<dockcircle::lds_point>& pointcloud_);
        const dockcircle::Circle* const GetCircle(void) const;
};

#endif