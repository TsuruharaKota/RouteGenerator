#ifndef ACCELE
#include<cmath>
#include<utility>
using route_pair = std::pair<float, float>;
//----------<x, y, initial speed, final speed>----------//
using route_tuple = std::tuple<float, float, float, float>;
template<typename T>
class accelProfile{
    public:
        accelProfile(accelParam &_param):param(_param){}
        void operator()(std::vector<route_tuple> &_dist){
            for(auto &param : _dist){
                float VEL_INI = std::tuple::get<2>(param);
                float VEL_FIN = std::tuple::get<3>(param);
                float total_distance = calDistance();
                //等速区間に達しない場合の例外処理
                float accel_section_time = (VEL_MAX - VEL_INI) / ACCEL; 
                float accel_section_time_pos = 0.5 * (VEL_FIN + TARGET_VEL) * accel_section_time; 
                float decel_section_time = (VEL_MAX - VEL_FIN) / ACCEL;
                float decel_section_pos = 0.5 * (VEL_INI + TARGET_VEL) * decel_section_time;
                float constant_vel_section_pos = total_distance - (accel_section_time_pos + decel_section_pos);
                float constant_vel_section_time = constant_vel_section_pos / TARGET_VEL;
            }
        }
    private:
        float calDistance(){
            float temp_distance{};
            if(isSpline()){
                for(int i = 0; i < 100; ++i){
                    float x = [i];
                    float y = [i];
                    temp_distance += std::sqrt((x * x) + (y * y));
                }
            }else{
                x_fin == x_ini ? temp_distance = y_fin - y_ini : x_fin - x_ini;
            }
            return temp_distance;
        }
        bool isSpline(){
            if(std::tuple::get<2>())
        }
        vector<route_pair> dist;
        std::vector<route_tuple> route;
        constexpr float TARGET_VEL;
        constexpr float ACCEL; 
        constexpr float VEL_MAX;
        constexpr float VEL_MIN;
};
#endif ACCELE