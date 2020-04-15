#include<iostream>
#include<vector>
#include<chrono>
#include<utility>
#include<tuple>
#include<fstream>
#include<string>
#include<Eigen/Dense>
#include<vector>
#include<cmath>
#include<iomanip>
#include<queue>
using route_pair = std::pair<float, float>;
using route_tuple = std::tuple<float, float, unsigned int>;
namespace route{
  template<typename T>
  inline T map(T x, T in_min, T in_max, T out_min, T out_max){
    if(x > in_max or x < in_min){std::cerr << "this value is over range" << std::endl;}
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }
}
enum class Coat{
  red1,
  red2,
  blue1,
  blue2
};
template<typename T>
class SplineParam{
  public:
    SplineParam(Eigen::Matrix<T, 4, 2> &_param):param(_param);
    route_pair splineMain(T fanctor){
      return (route_pair(param(0, 0) * pow(fanctor, 3) + param(1, 0) * pow(fanctor, 2) + param(2, 0) * fanctor + param(3, 0), 
                         param(0, 1) * pow(fanctor, 3) + param(1, 1) * pow(fanctor, 2) + param(2, 1) * fanctor + param(3, 1))); 
    }
    route_pair converterPos(float distance){
      static float fanctor{};
      route_pair temp_pos{};
      while(((temp_distance - 0.1) < temp_distance) and (temp_distance < (temp_distance + 0.1))){
        fanctor += 0.01
        temp_pos = this -> splineMain(fanctor);
        temp_distance += std::sqrt((temp_pos.first * temp_pos.first) + (temp_pos.second * temp_pos.second));
      }
      return temp_pos;
    }
  private:
    Eigen::Matrix<T, 4, 2> param;
    float total_distance;
};
template<typename T, int N>
class RouteGenerator{
  public:
    RouteGenerator(std::vector<route_pair>&& passing_point):route(passing_point){}
    void operator()(){
      try{
        if((this -> generateRoute()) == false)throw 1;
        if((this -> fileSet()) == false)throw 2;
      }
      catch (int error_num){
        switch(error_num){
          case 1:
            std::cerr << "error in generateRoute function" << std::endl;
          case 2:
            std::cerr << "error in fileSet function" << std::endl;
          default:
            std::cerr << "error occurred" << std::endl;
        }
      }
    }
  private:
    bool generateRoute(){
      for(int i = 0; i < 9; ++i){
        if((route[i].first != route[i + 1].first) && (route[i].second == route[i + 1].second)){
          //----------X Linear interpolation----------//
          float devide_x = (route[i + 1].first - route[i].first) / 5;
          for(int j = 0; j < 5; ++j){route_goal.push_back(route_pair(route[i].first + (devide_x * j), route[i].second));}
        }else if((route[i].first == route[i + 1].first) && (route[i].second != route[i + 1].second)){
          //----------Y Linear interpolation----------//
          float devide_y = (route[i + 1].second - route[i].second) / 5;
          for(int j = 0; j < 5; ++j){route_goal.push_back(route_pair(route[i].first, route[i].second + (devide_y * j)));}
        }else{
          //Curve interpolation
          std::array<float, 4> u;
          //auto uCal = [](std::array<float, 4> &_u, int k, int l) -> float{
          //  return  _u[l - 1] + std::sqrt(std::pow(route[k + l].first - route[k + l - 1].first, 2) + 
          //          std::pow(route[k + l].second - route[k + l - 1].second, 2));
          //};
          for(int j = 0; j < 4; ++j){
            if(j == 0){
              u[j] = 0;
            }else{
              //u[j] = uCal(u, i, j);
              u[j] = u[j - 1] + std::sqrt(std::pow(route[i + j].first - route[i + j - 1].first, 2) + 
                     std::pow(route[i + j].second - route[i + j - 1].second, 2));
            }
          }
          Eigen::Matrix4f A;
          A << std::pow(u[0], 3), std::pow(u[0], 2), u[0], 1,
               std::pow(u[1], 3), std::pow(u[1], 2), u[1], 1,
               std::pow(u[2], 3), std::pow(u[2], 2), u[2], 1,
               std::pow(u[3], 3), std::pow(u[3], 2), u[3], 1;
          Eigen::Matrix<float, 4, 2> b;
          b << route[i    ].first, route[i    ].second,
               route[i + 1].first, route[i + 1].second,
               route[i + 2].first, route[i + 2].second,
               route[i + 3].first, route[i + 3].second;
          Eigen::ColPivHouseholderQR<Eigen::Matrix4f> dec(A);
          Eigen::Matrix<float, 4, 2> x = A.colPivHouseholderQr().solve(b);
          splineObj.push(SplineParam<float>(x));
          splineConverterObj.push(SplineParam<float>(x));
          for(float j = u[0]; j <= u[3]; j+= (u[3] - u[0]) / 10){
              route_goal.push_back(route_pair(x(0, 0) * pow(j, 3) + x(1, 0) * pow(j, 2) + x(2, 0) * j + x(3, 0), 
     　　　　　　　　　                          x(0, 1) * pow(j, 3) + x(1, 1) * pow(j, 2) + x(2, 1) * j + x(3, 1))); 
          }
          spline_factor.push(u[3]);
          i += 2;
        }
      }
      int devide_time = N / 500;
      for(int i = 0; i < devide_time; ++i){
      }
      return true;
    }
    bool fileSet(){
      std::ofstream outputFile("Test.txt");
      for(auto &point : route_goal){outputFile << std::fixed << std::setprecision(5) << point.first << " " << point.second << "\n";}
      outputFile.close();
      return true;
    }
    route_pair positionGetter(float u_temp, float counter){
      if(counter == 1){splineObj.pop();}
      SplineParam tempSplineObj = splineObj.front();
      return tempSplineObj.splineMain(u_temp);
    }
    float splineFactorGetter(){
      float factor = spline_factor.front();
      spline_factor.pop();
      return factor;
    }
    route_pair splineDisPosConverter(float distance, float total_distance){
      if(distance == total_distance){splineConverterObj.pop();}
      SplineParam ConverterObj = splineConverterObj.front();
      return ConverterObj.converterPos(u_temp);
    }
    route_pair linearDisPosConverter(float distance){
      if(counter == 1){splineObj.pop();}
      SplineParam ConverterObj = splineObj.front();
      return ConverterObj.converterPos(u_temp);
    }
    float Timer();
    float time;
    std::vector<route_pair> route;
    std::vector<route_pair> route_goal;
    std::queue<float> spline_factor;
    std::queue<SplineParam<float>> splineObj;
    std::queue<SplineParam<float>> splineConverterObj;
};
template<typename T>
class AccelProfile{
    public:
        accelProfile(route_tuple &_param, RouteGenerator *_generatorObj):param(_param), generatorobj(_generatorObj){
          VEL_INI = std::tuple::get<2>(param);
          VEL_FIN = std::tuple::get<3>(param);
          total_distance = calDistance();
          //等速区間に達しない場合の例外処理
          accel_section_time = (VEL_MAX - VEL_INI) / ACCEL; 
          accel_section_time_pos = 0.5 * (VEL_FIN + TARGET_VEL) * accel_section_time; 
          decel_section_time = (VEL_MAX - VEL_FIN) / ACCEL;
          decel_section_pos = 0.5 * (VEL_INI + TARGET_VEL) * decel_section_time;
          constant_vel_section_pos = total_distance - (accel_section_time_pos + decel_section_pos);
          constant_vel_section_time = constant_vel_section_pos / TARGET_VEL;
        }
        route_pair operator()(float time){
          float distance_cal{};
          if(time < accel_section_time){
            //加速区間のときの移動距離
            distance_cal = 0.5 * (VEL_INI + (((VEL_MAX - VEL_INI) / accel_section_time) * time));
          }else if(time > accel_section_time + constant_vel_section_time){
            //減速区間のときの移動距離
            distance_cal = 0.5 * (VEL_FIN + (((VEL_FIN - VEL_MAX) / (accel_section_time + decel_section_time + constant_vel_section_time)) * time)); 
          }else{
            //等速区間のときの移動距離
            distance_cal = accel_section_time_pos + (time * VEL_MAX);
          }
          if(isSpline()){
            //スプライン補間の場合
            return generatorObj -> splineDisPosConverter(distance_cal);
          }else{
            //直線補間の場合
            return generatorObj -> linearDisPosConverter(distance_cal);
          }
        }
        float timerLimitGetter(float timer){
          return accel_section_time + decel_section_time + constant_vel_section_time;
        }
    private:
        float calDistance(){
          float temp_distance{};
          if(isSpline()){
            //スプライン曲線であった場合
            //uをRouteGeneratorから持ってくる
            float spline_factor = generatorObj -> splineFactorGetter();
            float map_counter{};
            for(int i = 0; i < 100; ++i){
                map_counter += 0.01
                float position_getter_val = route::map(map_counter, 0, 1, 0, spline_factor)
                route_pair temp_pos = generatorObj -> positionGetter(position_getter_val, i);
                temp_distance += std::sqrt((temp_pos.first * temp_pos.first) + (temp_pos.second * temp_pos.second));
            }
          }else{
            //スプライン曲線ではない場合
            x_fin == x_ini ? temp_distance = y_fin - y_ini : temp_distance = x_fin - x_ini;
          }
          return temp_distance;
        }
        bool isSpline(){
          return (std::tuple::get<2>([i]) != std::tuple::get<2>([i + 1]) and std::tuple::get<3>([i]) != std::tuple::get<3>([i + 1]))
        }
  
        RouteGenerator<T, N> *generatorObj;
        vector<route_pair> dist;
        std::vector<route_tuple> route;
        constexpr float TARGET_VEL;
        constexpr float ACCEL; 
        constexpr float VEL_MAX;
        constexpr float VEL_MIN;
        float VEL_INI;
        float VEL_FIN;
        float total_distance;
        float accel_section_time; 
        float accel_section_time_pos; 
        float decel_section_time;
        float decel_section_pos;
        float constant_vel_section_pos;
        float constant_vel_section_time;
};
template<typename T, long long N>
class TargetPosition{
  public:
    TargetPosition(std::vector<route_tuple>& _point) : point(_point){
      targetRoute = new RouteGenerator(_point);
    }
    route_pair operator()(float timer){
      //入力された時間の位置を出力する
      if(targetQueue.empty() == false){
        //キューを更新する条件かどうか
        if(timer > timer_limit){
          targetQueue.pop();
          timer_limit = timer;
          AccelProfile targetLimitObj = targetQueue.front();
          timer_limit += targetLimitObj.timerLimitGetter(timer);
        }
        AccelProfile tagetObj = targetQueue.front();
        route_pair get_point = targetObj(timer);
        return get_point;
      }else{
        return route_pair(0.0f, 0.0f);
      }
    }
  private:
    void setQueue(){
      //目標位置をキューごとで管理する
      for(int i = 0; i < 10; ++i){
        if(std::tuple::get<2>(point[i]) == 'ERR' && std::tuple::get<3>(point[i]) == 'ERR'){
          AccelProfile *target_point = new AccelProfile(point[i], targetRoute);
          targetQueue.push();
        }
      }
    }
    AccelProfile<T> *targetProfile;  
    RouteGenerator<T, N> *targetRoute;
    std::vector<route_tuple> point;
    std::queue<AccelProfile*> targetQueue;
    float timer_limit;
};
template<Coat color>
std::vector<route_pair> routeInit(){
  std::vector<route_pair> point;
  switch(color){
    case Coat::red1:
      point.push_back(route_pair(1.0f, 0.1f));
      point.push_back(route_pair(2.0f, 0.3f));
      point.push_back(route_pair(2.5f, 0.5f));
      point.push_back(route_pair(4.0f, 1.0f));
      point.push_back(route_pair(4.0f, 2.0f));
      point.push_back(route_pair(4.0f, 3.0f));
      point.push_back(route_pair(4.0f, 4.0f));
      point.push_back(route_pair(4.0f, 5.0f));
      point.push_back(route_pair(4.0f, 6.0f));
      point.push_back(route_pair(4.0f, 7.0f));
      break;
    case Coat::red2:
      point.push_back(route_pair(0.0f, 1.0f));
      point.push_back(route_pair(0.0f, 2.0f));
      point.push_back(route_pair(0.0f, 3.0f));
      point.push_back(route_pair(0.0f, 4.0f));
      point.push_back(route_pair(1.0f, 5.0f));
      point.push_back(route_pair(2.0f, 6.0f));
      point.push_back(route_pair(2.5f, 7.0f));
      point.push_back(route_pair(2.5f, 8.0f));
      point.push_back(route_pair(2.5f, 9.0f));
      point.push_back(route_pair(2.5f, 10.0f));
      break;
    case Coat::blue1:
      point.push_back(route_pair(18.0f, 6.0f));
      point.push_back(route_pair(45.0f, 6.0f));
      point.push_back(route_pair(50.0f, 7.0f));
      point.push_back(route_pair(53.0f, 10.5f));
      point.push_back(route_pair(55.0f, 15.0f));
      point.push_back(route_pair(55.0f, 30.0f));
      point.push_back(route_pair(55.0f, 45.0f));
      point.push_back(route_pair(55.0f, 60.0f));
      point.push_back(route_pair(55.0f, 75.0f));
      point.push_back(route_pair(55.0f, 100.0f));
      break;
    case Coat::blue2:
      point.push_back(route_pair(30, 10));
      point.push_back(route_pair(50, 10));
      point.push_back(route_pair(200, 10));
      point.push_back(route_pair(225, 15));
      point.push_back(route_pair(250, 90));
      point.push_back(route_pair(400, 100));
      point.push_back(route_pair(400, 200));
      point.push_back(route_pair(400, 300));
      point.push_back(route_pair(400, 400));
      point.push_back(route_pair(400, 500));
      break;
  }
  return point;
}
int main(){
  route_pair target;
  TargetPosition<float, 90000> targetPoint(routeInit<Coat::blue1>());
  while(1){
    target = targetPoint(timer);
  }
}
