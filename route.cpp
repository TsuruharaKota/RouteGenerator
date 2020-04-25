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
using target_tuple = std::tuple<float, float, float>;
using route_tuple = std::tuple<float, float, float, float>;
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
    SplineParam(Eigen::Matrix<T, 4, 2> _param):param(_param){
    };
    route_pair splineMain(T fanctor){
      route_pair ans = std::make_pair(param(0, 0) * std::pow(fanctor, 3) + param(1, 0) * std::pow(fanctor, 2) + param(2, 0) * fanctor + param(3, 0), 
                                      param(0, 1) * std::pow(fanctor, 3) + param(1, 1) * std::pow(fanctor, 2) + param(2, 1) * fanctor + param(3, 1));
      return ans; 
    }
  private:
    const Eigen::Matrix<T, 4, 2> param;
};
template<typename T>
class RouteGenerator{
  public:
    //----------route_pair(X, Y)----------//
    RouteGenerator(std::vector<route_pair>& passing_point):route(passing_point){}
    void Main(){
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
    float splineFactorGetter(bool &&check_angle){
      //check_angle == trueの場合は角度制御用のキュー
      float factor;
      if(check_angle == false){
        factor = spline_factor.front();
        spline_factor.pop();
      }else{
        factor = angle_factor.front();
        angle_factor.pop();
      }
      return factor;
    }
    route_pair positionGetter(float u_temp, float counter){
      if(counter == 100){splineObj.pop();}
      SplineParam<float> &tempSplineObj = splineObj.front();
      return tempSplineObj.splineMain(u_temp);
    }
    route_pair positionGetterAngle(float u_temp, float counter){
      if(counter == 1){splineAngleObj.pop();}
      SplineParam<float> &tempSplineObj = splineAngleObj.front();
      return tempSplineObj.splineMain(u_temp);
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
          //std::cout << "x = " << x << std::endl;
          SplineParam<float> splineX(x);
          splineObj.push(splineX);
          std::cout << "-----------------------------" << std::endl;
          std::cout << "size now = " << splineObj.size() << std::endl;
          splineAngleObj.push(SplineParam<float>(x));
          for(float j = u[0]; j <= u[3]; j+= (u[3] - u[0]) / 10){
              route_goal.push_back(route_pair(x(0, 0) * std::pow(j, 3) + x(1, 0) * std::pow(j, 2) + x(2, 0) * j + x(3, 0), 
                                              x(0, 1) * std::pow(j, 3) + x(1, 1) * std::pow(j, 2) + x(2, 1) * j + x(3, 1))); 
          }
          i += 2;
          spline_factor.push(u[3]);
          angle_factor.push(u[3]);
        }
      }
      return true;
    }
    bool fileSet(){
      std::ofstream outputFile("Test.txt");
      for(auto &point : route_goal){outputFile << std::fixed << std::setprecision(5) << point.first << " " << point.second << "\n";}
      outputFile.close();
      return true;
    }
    float time;
    std::vector<route_pair> route;
    std::vector<route_pair> route_goal;
    std::queue<float> spline_factor;
    std::queue<float> angle_factor;
    std::queue<SplineParam<float>> splineObj;
    std::queue<SplineParam<float>> splineAngleObj;
};
template<typename T>
struct AccelParam{
  T ACCEL = 1.0f; 
  T VEL_MIN = 0.0f;
  T VEL_MAX = 2.5f;
  T VEL_INI;
  T VEL_FIN;
  T total_distance;
  T accel_section_time; 
  T accel_section_time_pos; 
  T decel_section_time;
  T decel_section_pos;
  T constant_vel_section_pos;
  T constant_vel_section_time;
  T time_total;
  void set(route_pair &&_param, T _total_distance){
    //----------route_pair(VEL_INI, VEL_FIN)----------//
    VEL_INI = _param.first;
    VEL_FIN = _param.second;
    total_distance = _total_distance;
    accel_section_time = static_cast<T>((VEL_MAX - VEL_INI) / ACCEL);
    accel_section_time_pos = static_cast<T>(0.5 * (VEL_INI + VEL_MAX) * accel_section_time); 
    decel_section_time = static_cast<T>((VEL_MAX - VEL_FIN) / ACCEL);
    decel_section_pos = static_cast<T>(0.5 * (VEL_FIN + VEL_MAX) * decel_section_time);
    constant_vel_section_pos = static_cast<T>(total_distance - (accel_section_time_pos + decel_section_pos));
    constant_vel_section_time = static_cast<T>(constant_vel_section_pos / VEL_MAX);
    if(constant_vel_section_time == 0){
      //等速区間に達しない場合の例外処理
    }
    time_total = accel_section_time + constant_vel_section_time + decel_section_time;
  }
};
template<typename T>
class AccelProfile{
    public:
        AccelProfile(AccelParam<T> _paramObj):paramObj(_paramObj){}
        float operator()(float time, float TIME_INI){
          float cmd_vel{};
          if(time - TIME_INI < paramObj.accel_section_time){
            //加速区間のときの速度
            cmd_vel = paramObj.VEL_INI + paramObj.ACCEL * (time - TIME_INI);
          }else if(time - TIME_INI > paramObj.accel_section_time + paramObj.constant_vel_section_time){
            //減速区間のときの速度
            float limit = this -> timerLimitGetter();
            cmd_vel = paramObj.VEL_FIN + paramObj.ACCEL * (limit - (time - TIME_INI)); 
          }else{
            //等速区間のときの速度
            cmd_vel = paramObj.VEL_MAX;
          }
          return cmd_vel;
        }
        float timerLimitGetter(){
          return paramObj.accel_section_time + paramObj.decel_section_time + paramObj.constant_vel_section_time;
        }
    private:
        route_pair param; 
        const AccelParam<T> paramObj;
        std::vector<route_pair> dist;
        std::vector<route_tuple> route;
};
//template<typename T>
class AngleControl{
  public:
    AngleControl(){std::cout << "fuck" << std::endl;}
    AngleControl(route_pair _point_ini, route_pair &&_point_fin, RouteGenerator<float> *_targetRoute):point_ini(_point_ini), point_fin(_point_fin), targetRoute(_targetRoute){
      std::cout << "nice" << std::endl;
      if(point_fin.first != point_ini.first and point_fin.second != point_ini.second){
        spline_factor = targetRoute -> splineFactorGetter(true);
      }
    }
    float operator()(float vel_now, float timer_now){
      //移動済みの距離を求める
      static float goal_angle_prev{};
      static route_pair point_prev = point_ini;
      distance_elapsed += vel_now * (timer_now - timer_prev);
      if(vel_now < 0 or timer_now - timer_prev < 0){
        std::cout << vel_now << " " << timer_now - timer_prev << " " << timer_now << " " << timer_prev << std::endl;
      }
      timer_prev = timer_now;
      //移動距離から座標に変換する
      float integral_distance{};
      float dx, dy;
      route_pair coor_now, coor_prev = point_ini;
      float u_counter{};
      if(point_fin.first != point_ini.first and point_fin.second != point_ini.second){
        //スプライン曲線の場合
        float u;
        while(distance_elapsed > integral_distance){
          //最終点での座標を知りたい
          u_counter += 0.01;
          u = floor(u_counter * 100) / 100;
          float position_getter_val = route::map<float>(u, 0.0f, 1.0f, 0.0f, spline_factor);
          coor_now = targetRoute -> positionGetterAngle(position_getter_val, u_counter);
          dx = coor_now.first - coor_prev.first;
          dy = coor_now.second - coor_prev.second;
          integral_distance += std::sqrt((dx * dx) + (dy * dy));
          coor_prev = coor_now;
        }
        float diff_y = coor_now.second - point_prev.second;
        float diff_x = coor_now.first - point_prev.first;
        if(diff_x == 0 and diff_y == 0){return goal_angle_prev;}
        float goal_angle = atan2(diff_y, diff_x);
        if(goal_angle < 0)goal_angle += 2 * M_PI; //M_PI ~ -M_PI の範囲を0 ~ 2M_PIに変更する
        point_prev = coor_now;
        goal_angle_prev = goal_angle;
        return goal_angle;  
      }else{
        //スプライン曲線では無い場合
        if(point_fin.first == point_ini.first){
          if(point_fin.second > point_ini.second){
            goal_angle_prev = 0.5 * M_PI;
            return 0.5 * M_PI;
          }else{
            goal_angle_prev = 1.5 * M_PI;
            return 1.5 * M_PI;
          }
        }else{
          if(point_fin.first > point_ini.first){
            goal_angle_prev = 0;
            return 0;
          }else{
            goal_angle_prev = M_PI;
            return M_PI;
          }
        }
      }
    }
  private:
    RouteGenerator<float> *targetRoute;
    float distance_elapsed{}; 
    float spline_factor{};
    route_pair point_ini;
    route_pair point_fin;
    float timer_prev{};
};
template<typename T>
class AngularControl{
  public:
    AngularControl(){}
    AngularControl(T _angle_prev, T _angle_now, T _timer_total){
      angular_vel = (_angle_now - _angle_prev) / _timer_total;
      std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
    }
    T operator()(){
      return angular_vel;
    }
  private:
    T angular_vel;
};
template<typename T, long long N>
class TargetPosition{
  public:
    TargetPosition(std::vector<route_tuple>&& _input_param):input_param(_input_param){
      //ルートを作成
      std::vector<route_pair> param_pos;
      for(int i = 0; i < input_param.size(); ++i){
        param_pos.push_back(route_pair(std::get<0>(input_param[i]), std::get<1>(input_param[i])));
        if(std::get<2>(input_param[i]) != -1.0f){
          pointQueue.push(route_pair(std::get<0>(input_param[i]), std::get<1>(input_param[i])));
        }
      }
      targetRoute = new RouteGenerator<float>(param_pos);
      targetRoute -> Main();
      this -> setQueue();
      std::cout << "finish" << std::endl;
    }
    target_tuple operator()(float timer){
      //入力された時間の位置を出力する
      if(targetQueue.empty() == false){
        //キューを更新する条件かどうか
        //std::cout << "queue size = " << targetQueue.size() << "limit = " << timer_limit << "time = " << timer << std::endl;
        if(timer > timer_limit){
          //----------accel object----------//
          targetQueue.pop();
          AccelProfile<float> targetLimitObj = targetQueue.front();
          timer_limit = timer;
          timer_limit += targetLimitObj.timerLimitGetter();
          init_time = timer;
          std::cout << "-----------------" << std::endl;
          //----------angle object----------//
          angleQueue.pop();
          targetAngle = angleQueue.front();
          //----------angular object----------//
          angularQueue.pop();
          targetAngular = angularQueue.front();
        }
        AccelProfile<float> targetObj = targetQueue.front();
        float vel = targetObj(timer, init_time);
        float angle = targetAngle(vel, timer - init_time);
        float angular_vel = targetAngular();
        return target_tuple(vel, angle, angular_vel); 
      }else{
        return target_tuple(0.0f, 0.0f, 0.0f);
      }
    }
  private:
    void setQueue(){
      //目標位置をキューごとで管理する
      float prev_vel{};
      float angle_prev{};
      route_pair point_prev{};
      for(int i = 0; i < 10; ++i){
        //-1.0fの場合はそこを挟む点がスプライン補間される
        if(std::get<2>(input_param[i]) != -1.0f){
          float distance = this -> calDistance();
          AccelParam<float> param;
          //----------set accel object queue----------//
          param.set(route_pair(prev_vel, std::get<2>(input_param[i])), distance);
          AccelProfile<float> target_point(param);
          targetQueue.push(target_point);
          //----------finish accel object queue----------//
          //----------set angle object queue----------//
          AngleControl angle(point_prev, route_pair(std::get<0>(input_param[i]), std::get<1>(input_param[i])), targetRoute);
          angleQueue.push(angle);     
          prev_vel = std::get<2>(input_param[i]);
          point_prev = std::make_pair(std::get<0>(input_param[i]), std::get<1>(input_param[i]));
          //----------finish angle object queue----------//
          //----------set angular object queue----------//
          AngularControl<float> target_angular(angle_prev, std::get<3>(input_param[i]), param.time_total);
          angularQueue.push(target_angular);
          angle_prev = std::get<3>(input_param[i]);
        }
      }
    }
    float calDistance(){
      float total_distance{};
      static route_pair prev_point{};
      route_pair now_point = pointQueue.front();
      //この条件式いらないかも
      //if(pointQueue.size() >= 2){
        if(now_point.first != prev_point.first and now_point.second != prev_point.second){
          //スプライン曲線であった場合
          //残りの座標キューの数が2個以上であるか
          //pointQueue[0]とpointQueue[1]の<<間の距離を求める
          //それぞれの点の間の100個の座標を求めてそれの長さを三平方の定理を使って求めて、全体の長さとして近似する
          float map_counter{};
          route_pair temp_pos_prev = std::make_pair(prev_point.first, prev_point.second);
          float spline_factor = targetRoute -> splineFactorGetter(false);
          float dx, dy;
          for(int i = 0; i < 100; ++i){
            map_counter += 0.01;
            float position_getter_val = route::map<float>(map_counter, 0.0f, 1.0f, 0.0f, spline_factor);
            route_pair temp_pos_now = targetRoute -> positionGetter(position_getter_val, i);
            //std::cout << "temp_pos_now = " << temp_pos_now.first << " " << temp_pos_now.second << std::endl;
            dx = temp_pos_now.first - temp_pos_prev.first;
            dy = temp_pos_now.second - temp_pos_prev.second;
            //std::cout << "dx = " << dx << " " << "dy = " << dy << std::endl;
            total_distance += std::sqrt((dx * dx) + (dy * dy));
            temp_pos_prev = temp_pos_now;
          }
        }else{
          //スプライン曲線ではない場合
          prev_point.first == now_point.first ? total_distance = std::abs(now_point.second - prev_point.second) : 
                                                total_distance = std::abs(now_point.first - prev_point.first);
        }
        prev_point = now_point;
        pointQueue.pop();
      //}
      std::cout << "total_distance = " << total_distance << std::endl;
      return total_distance;
    }
    RouteGenerator<T> *targetRoute;
    AngleControl targetAngle;
    AngularControl<float> targetAngular;
    std::vector<route_tuple> input_param;
    std::queue<AccelProfile<float>> targetQueue;
    std::queue<AngleControl> angleQueue;
    std::queue<AngularControl<float>> angularQueue;
    std::queue<route_pair> pointQueue;
    float timer_limit{};
    float init_time{};
};
template<Coat color>
std::vector<route_tuple> routeInit(){
  std::vector<route_tuple> point;
  switch(color){
    case Coat::red1:
      point.push_back(route_tuple( 0.0f,  0.0f,  1.0f,   0.0f));
      point.push_back(route_tuple( 0.0f, 10.0f,  1.0f,  30.0f));
      point.push_back(route_tuple( 0.0f, 20.0f,  1.0f,  40.0f));
      point.push_back(route_tuple( 0.0f, 30.0f,  1.0f,  50.0f));
      point.push_back(route_tuple( 1.5f, 35.0f, -1.0f,  60.0f));
      point.push_back(route_tuple( 5.0f, 39.0f, -1.0f,  70.0f));
      point.push_back(route_tuple(10.0f, 40.0f,  1.0f,  80.0f));
      point.push_back(route_tuple(20.0f, 40.0f,  1.0f,  90.0f));
      point.push_back(route_tuple(30.0f, 40.0f,  1.0f, 100.0f));
      point.push_back(route_tuple(40.0f, 40.0f,  1.0f, 110.0f));
      break;
    case Coat::red2:
      point.push_back(route_tuple(1.0f, 0.1f, 0.0f, 0.0f));
      point.push_back(route_tuple(2.0f, 0.3f, 0.0f, 0.0f));
      point.push_back(route_tuple(2.5f, 0.5f, 0.0f, 0.0f));
      point.push_back(route_tuple(4.0f, 1.0f, 0.0f, 0.0f));
      point.push_back(route_tuple(4.0f, 2.0f, 0.0f, 0.0f));
      point.push_back(route_tuple(4.0f, 3.0f, 0.0f, 0.0f));
      point.push_back(route_tuple(4.0f, 4.0f, 0.0f, 0.0f));
      point.push_back(route_tuple(4.0f, 5.0f, 0.0f, 0.0f));
      point.push_back(route_tuple(4.0f, 6.0f, 0.0f, 0.0f));
      point.push_back(route_tuple(4.0f, 7.0f, 0.0f, 0.0f));
      break;
    case Coat::blue1:
      point.push_back(route_tuple(1.0f, 0.1f, 0.0f, 0.0f));
      point.push_back(route_tuple(2.0f, 0.3f, 0.0f, 0.0f));
      point.push_back(route_tuple(2.5f, 0.5f, 0.0f, 0.0f));
      point.push_back(route_tuple(4.0f, 1.0f, 0.0f, 0.0f));
      point.push_back(route_tuple(4.0f, 2.0f, 0.0f, 0.0f));
      point.push_back(route_tuple(4.0f, 3.0f, 0.0f, 0.0f));
      point.push_back(route_tuple(4.0f, 4.0f, 0.0f, 0.0f));
      point.push_back(route_tuple(4.0f, 5.0f, 0.0f, 0.0f));
      point.push_back(route_tuple(4.0f, 6.0f, 0.0f, 0.0f));
      point.push_back(route_tuple(4.0f, 7.0f, 0.0f, 0.0f));
      break;
    case Coat::blue2:
      point.push_back(route_tuple(1.0f, 0.1f, 0.0f, 0.0f));
      point.push_back(route_tuple(2.0f, 0.3f, 0.0f, 0.0f));
      point.push_back(route_tuple(2.5f, 0.5f, 0.0f, 0.0f));
      point.push_back(route_tuple(4.0f, 1.0f, 0.0f, 0.0f));
      point.push_back(route_tuple(4.0f, 2.0f, 0.0f, 0.0f));
      point.push_back(route_tuple(4.0f, 3.0f, 0.0f, 0.0f));
      point.push_back(route_tuple(4.0f, 4.0f, 0.0f, 0.0f));
      point.push_back(route_tuple(4.0f, 5.0f, 0.0f, 0.0f));
      point.push_back(route_tuple(4.0f, 6.0f, 0.0f, 0.0f));
      point.push_back(route_tuple(4.0f, 7.0f, 0.0f, 0.0f));
      break;
  }
  return point;
}
int main(){
  target_tuple target;
  TargetPosition<float, 90000> targetPoint(routeInit<Coat::red1>());
  auto start = std::chrono::system_clock::now();
  float timer{};
  std::ofstream outputFile("Angle.txt");
  while(1){
    auto end = std::chrono::system_clock::now(); 
    timer = static_cast<float>(std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()) / 1000;
    target = targetPoint(timer);
    std::cout << std::get<0>(target) << " " << (std::get<1>(target) / M_PI) * 180 << " " << std::get<2>(target) << std::endl;
    outputFile << std::fixed << std::setprecision(5) << (std::get<1>(target) / M_PI) * 180 << "\n";
  }
  outputFile.close();
}
