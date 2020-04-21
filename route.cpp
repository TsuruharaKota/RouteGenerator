#include<iostream>
#include<vector>
#include<chrono>
#include<utility>
#include<tuple>
#include<fstream>
#include<string>
#include<eigen3/Dense>
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
    SplineParam(Eigen::Matrix<T, 4, 2> &_param):param(_param){};
    route_pair splineMain(T fanctor){
      return (route_pair(param(0, 0) * pow(fanctor, 3) + param(1, 0) * pow(fanctor, 2) + param(2, 0) * fanctor + param(3, 0), 
                         param(0, 1) * pow(fanctor, 3) + param(1, 1) * pow(fanctor, 2) + param(2, 1) * fanctor + param(3, 1))); 
    }
  private:
    Eigen::Matrix<T, 4, 2> param;
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
          for(float j = u[0]; j <= u[3]; j+= (u[3] - u[0]) / 10){
              route_goal.push_back(route_pair(x(0, 0) * pow(j, 3) + x(1, 0) * pow(j, 2) + x(2, 0) * j + x(3, 0), 
     　　　　　　　　　                          x(0, 1) * pow(j, 3) + x(1, 1) * pow(j, 2) + x(2, 1) * j + x(3, 1))); 
          }
          i += 2;
          spline_factor.push(u[3]);
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
    float splineFactorGetter(){
      float factor = spline_factor.front();
      spline_factor.pop();
      return factor;
    }
    route_pair positionGetter(float u_temp, float counter){
      if(counter == 1){splineObj.pop();}
      SplineParam tempSplineObj = splineObj.front();
      return tempSplineObj.splineMain(u_temp);
    }
    float time;
    std::vector<route_pair> route;
    std::vector<route_pair> route_goal;
    std::queue<float> spline_factor;
    std::queue<SplineParam<float>> splineObj;
};
template<typename T>
class AccelProfile{
    public:
        AccelProfile(route_tuple &_param, float _total_distance):param(_param), total_distance(_total_distance){
          VEL_INI = std::tuple::get<2>(param);
          VEL_FIN = std::tuple::get<3>(param);
          //等速区間に達しない場合の例外処理
          accel_section_time = (VEL_MAX - VEL_INI) / ACCEL; 
          accel_section_time_pos = 0.5 * (VEL_FIN + TARGET_VEL) * accel_section_time; 
          decel_section_time = (VEL_MAX - VEL_FIN) / ACCEL;
          decel_section_pos = 0.5 * (VEL_INI + TARGET_VEL) * decel_section_time;
          constant_vel_section_pos = total_distance - (accel_section_time_pos + decel_section_pos);
          constant_vel_section_time = constant_vel_section_pos / TARGET_VEL;
        }
        float operator()(float time){
          //目標現在速度を出力する
          float cmd_vel{};
          if(time < accel_section_time){
            //加速区間のときの速度
            cmd_vel = 0;
          }else if(time > accel_section_time + constant_vel_section_time){
            //減速区間のときの速度
            cmd_vel = 0; 
          }else{
            //等速区間のときの速度
            cmd_vel = 0;
          }
          return cmd_vel;
        }
        float timerLimitGetter(float timer){
          return accel_section_time + decel_section_time + constant_vel_section_time;
        }
    private:
        route_tuple param; 
        vector<route_pair> dist;
        std::vector<route_tuple> route;
        float TARGET_VEL;
        float ACCEL; 
        float VEL_MAX;
        float VEL_MIN;
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
class AngleControl{
  public:
    AngleControl(){}
  private:
};
template<typename T, long long N>
class TargetPosition{
  public:
    TargetPosition(std::vector<route_tuple>& _input_param) : input_param(_input_param){
      //ルートを作成
      vector<route_pair> param_pair;
      for(int i = 0; i < input_param.size(); ++i){
        param_pair.add(route_pair(std::tuple::get<0>(input_param[i]), std::tuple::get<1>(input_param[i])));
      }
      targetRoute = new RouteGenerator(param_pair);
    }
    void setQueue(){
      //目標位置をキューごとで管理する
      for(int i = 0; i < 10; ++i){
        if(std::tuple::get<2>(input_param[i]) == 'ERR' && std::tuple::get<3>(input_param[i]) == 'ERR'){
          AccelProfile *target_point = new AccelProfile(point[i]);
          //ポインタを外す必要がある
          targetQueue.push(target_point);
          pointQueue.push(route_pair(std::tuple::get<0>(input_param[i]), std::tuple::get<1>(input_param[i])));
        }
      }
    }
    route_tuple operator()(float timer){
      //入力された時間の位置を出力する
      if(targetQueue.empty() == false){
        //キューを更新する条件かどうか
        if(timer > timer_limit){
          targetQueue.pop();
          AccelProfile targetLimitObj = targetQueue.front();
          timer_limit = timer;
          timer_limit += targetLimitObj.timerLimitGetter(timer);
        }
        AccelProfile tagetObj = targetQueue.front();
        float vel = targetObj(timer);
        float angle = 0;
        float angle_vel = 0;
        return route_tuple(vel, angle, angle_vel); 
      }else{
        return route_pair(0.0f, 0.0f, 0.0f);
      }
    }
  private:
    float calDistance(){
      float total_distance{};
      if(pointQueue.size() >= 2){
        if(isSpline()){
          //スプライン曲線であった場合
          //残りの座標キューの数が2個以上であるか
          //pointQueue[0]とpointQueue[1]の間の距離を求める
          //それぞれの点の間の100個の座標を求めてそれの長さを三平方の定理を使って求めて、全体の長さとして近似する
          float map_counter{};
          float spline_factor = targetRoute -> splineFactorGetter();
          for(int i = 0; i < 100; ++i){
            map_counter += 0.01;
            float position_getter_val = route::map(map_counter, 0, 1, 0, spline_factor);
            route_pair temp_pos = targetRoute -> positionGetter(position_getter_val, i);
            total_distance += std::sqrt((temp_pos.first * temp_pos.first) + (temp_pos.second * temp_pos.second));
          }
        }else{
          //スプライン曲線ではない場合
          pointQueue[0].x == pointQueue[1].x ? total_distance = pointQueue[1].y - pointQueue[0].y : 
                                               total_distance = pointQueue[1].x - pointQueue[0].x;
        }
        pointQueue.pop();
      }
      return total_distance;
    }
    bool isSpline(){
      return (pointQueue[0].x != pointQueue[1].x and pointQueue[0].y != pointQueue[1].y);
    }
    RouteGenerator<T, N> *targetRoute;
    AccelProfile<T> *targetProfile;
    AngleControl *targetAngle;
    std::vector<route_tuple> input_param;
    std::vector<route_pair> point;
    std::queue<AccelProfile*> targetQueue;
    std::queue<route_pair> pointQueue;
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
    int timer;
    target = targetPoint(timer);
  }
}
