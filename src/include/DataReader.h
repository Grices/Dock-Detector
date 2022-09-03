#ifndef _DATAREADER_H_
#define _DATAREADER_H_

#include <iostream>
#include <vector>
#include <fstream>
#include <string>

class ReadtoPoints
{
    public:
        std::vector<dockcircle::lds_point> m_read_points;
        std::vector<std::vector<dockcircle::lds_point>> mf_points;
    public:
        const std::vector<dockcircle::lds_point>& GetPoints(void) const
        {
            return this->m_read_points;
        }

        ReadtoPoints(const std::string& filepath)
        {
            std::ifstream iFile(filepath, std::ios::binary);
            if(!iFile.is_open())
            {
                std::cout << "File not exit." << std::endl;
                return;
            }
            else
            {
                int times = 0;
                int data_size = 0;
                while (iFile.peek() != EOF)
                {
                    iFile.read((char*)(&data_size), sizeof(data_size));
                    this->m_read_points.clear();
                    this->m_read_points.reserve(data_size);
                    for(int i = 0; i < data_size; ++i)
                    {
                        dockcircle::lds_point tmp;
                        iFile.read(reinterpret_cast<char*>(&tmp), sizeof(dockcircle::lds_point));
                        this->m_read_points.emplace_back(tmp);
                    }
                    this->mf_points.emplace_back(m_read_points);
                }
            }
            iFile.close();
        }        
};

#endif