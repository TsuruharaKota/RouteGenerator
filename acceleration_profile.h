#ifndef ACCELE
#include<cmath>
#include<utility>
using route_pair = std::pair<float, float>;
//----------<x, y, initial speed, final speed>----------//
using route_tuple = std::tuple<float, float, float, float>;
class accelProfile{
    public:
        accelProfile(accelParam &_param):param(_param){}
        void operator()(std::vector<route_pair> &_dist){
            for(auto &param : _dist){
                float ACCEL_INI = std::tuple::get<2>(param);
                float ACCEL_FIN = std::tuple::get<3>(param);
            }
        }
    private:
        void calDistance(){}
        vector<route_pair> dist;
        std::vector<route_tuple> 
        float distance;
        float accel_section;
        float decel_section;
        constexpr float ACCEL_MAX;
        constexpr float ACCEL_MIN;
}
#endif ACCELE