#ifndef _WRITEPOINTS_H_
#define _WRITEPOINTS_H_

#include <vector>
#include <fstream>

class WriteToPoints
{
public:
    struct lds_points
    {
        float x, y, rho, theta;
        lds_points(){};
        lds_points(float x_, float y_, float r_, float t_) : x(x_), y(y_), rho(r_), theta(t_) {}
    };
private:
    std::vector<lds_points> m_write_points;
public:
    void operator()(void){
        middleware::RobotInfo *rz_info = middleware::RobotInfo::getRobotInfo();
        robot::Robot &rz_robot = rz_info->getRobotMsg();
        int data_size = rz_robot.ldsPic.points.size();
        this->m_write_points.reserve(data_size);

        for(std::vector<lds_points>::iterator lds = rz_robot.ldsPic.points.begin(); lds != rz_robot.ldsPic.points.end(); lds++)
        {
            lds_points tmp(lds->x, lds->y, lds->rho, lds->theta);
            this->m_write_points.emplace_back();
        }
        const std::string& filename = "/tmp/log/lds.bin";
        FILE* fp = fopen(filename.c_str(), "a");
        if(fp != nullptr)
        {
            // fwrite(const void* buffer, size, count, FILE* stream)
            fwrite(&data_size, sizeof(data_size), 1, fp);
            fclose(fp);
        }
        else
        {
            ECO_CST_NMLTS(return_log_file, 1, "file open fail\n");
        }
    }
}

#endif