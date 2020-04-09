#include<iostream>
#include<vector>
#include<chrono>
#include<utility>
#include<tuple>
#include<fstream>
#include<string>
#include<Eigen/Dense>
using route_pair = std::pair<float, float>;
using route_tuple = std::tuple<float, float, unsigned int>;
template<typename T, int N>
class RouteGenerator{
  public:
    RouteGenerator(*T passing_point):route(passing_point){
    }
    routeMain(){
      try{
        if((this -> generateRoute(N)) == false)throw 1;
        if((this -> fileSet()) == false)throw 2;
      }
      catch (int error_num){
        switch(error_num){
          case 1:
            cout << "error in generateRoute function" << endl;
          case 2:
            cout << "error in fileSet function" << endl;
          default:
            cout << "error occurred" << endl;
        }
        return 1;
      }
    }
  private:
    bool generateRoute(){
      constexpr int devide_time = N / 500;
      for(int i = 10){
        if((route[i].first != route[i + 1].first) && (route[i].second == route[i + 1].second)){
          //X Linear interpolation
          constexpr float devide_x = (route[i + 1].first - route[i].first) / 5;
          for(int j = 0; j < 5; ++j){
            route_goal.push_back(route[i].first + (devide_x * j), route[i].second);
          }
        }else if((route[i].first == route[i + 1].first) && (route[i].y != route[i + 1].second)){
          //Y Linear interpolation
          constexpr float devide_y = (route[i + 1].second - route[i].second) / 5;
          for(int j = 0; j < 5; ++j){
            route_goal.push_back(route[i].first, route[i].second + (devide_y * j));
          }
        }else{
          //Curve interpolation
          std::pair<float, float> point[4];
          for(int j = 0; j < 4; ++j){
            point[j].first = route[i + j].x;
            point[j].second = route[i + j].y
          }
          std::array<float, 4> u;
          for(int j = 0; j < 4; ++j){
            if(j == 0){
              if(i == 0) u[j] = 0;
              else u[j] = std::sqrt(std::pow(route[i].first - point[i - 1].first, 2) + 
                          std::pow(route[j].second - route[j - 1].second));
            }else{
              u[j] = u[j - 1] + std::sqrt(std::pow(point[j].first - point[j - 1].first, 2) + 
                     std::pow(point[j].second - point[j - 1].second));
            }
          }
          i += 4;
          Eigen::Matrix4f A;
          A << std::pow(u[0], 3), std::pow(u[0], 2), u[0], 1,
               std::pow(u[1], 3), std::pow(u[1], 2), u[1], 1,
               std::pow(u[2], 3), std::pow(u[2], 2), u[2], 1,
               std::pow(u[3], 3), std::pow(u[3], 2), u[3], 1;
          Eigen::Matrix<float, 4, 2> b;
          b << point[0].first, point[0].second,
               point[1].first, point[1].second,
               point[2].first, point[2].second,
               point[3].first, point[4].second;
          Eigen::ColPivHouseholderQR<Eigen::Matrix4f> dec(A);
          Eigen::Matrix<float, 4, 2> x = dec.solve(b);
          for(int j = u[0] j <= j[3] j+=j[3] - j[0] / 10){
            route_goal.push_back(x(0, 0) * pow(j, 3) + x(0,1) * pow(j, 2) + x(0, 2) * j + x(0, 3), 
                                 x(1, 0) * pow(j, 3) + x(1, 1) * pow(j, 2) + x(1, 2) * j + x(1, 3));  
          }
        }
      }
      for(int i = 0; i < devide_time; ++i){
      }
      return true;
    }
    bool fileSet(){
      std::ofstream writeFile;
      writeFile.open("RouteGenerator/Test.txt");
      writeFile <<;
    }
    float Timer();
    float time;
    vector<route_pair> route;
    vector<route_tuple> route_goal;
};
enum class Coat{
  red1,
  red2,
  blue1,
  blue2
}
template<Color color>
constexpr vector<route_pair> routeInit(){
  constexpr vector<route_pair> point;
  switch(color){
    case red1:
      point.push_back(route_pair(0.0f, 1.0f));
      point.push_back(route_pair(0.0f, 2.0f));
      point.push_back(route_pair(0.0f, 3.0f));
      point.push_back(route_pair(0.0f, 4.0f));
      point.push_back(route_pair(0.0f, 5.0f));
      point.push_back(route_pair(0.0f, 6.0f));
      point.push_back(route_pair(0.0f, 7.0f));
      point.push_back(route_pair(0.0f, 8.0f));
      point.push_back(route_pair(0.0f, 9.0f));
      point.push_back(route_pair(0.0f, 10.0f));
    case red2:
      point.push_back(route_pair(0.0f, 1.0f));
      point.push_back(route_pair(0.0f, 2.0f));
      point.push_back(route_pair(0.0f, 3.0f));
      point.push_back(route_pair(0.0f, 4.0f));
      point.push_back(route_pair(0.0f, 5.0f));
      point.push_back(route_pair(0.0f, 6.0f));
      point.push_back(route_pair(0.0f, 7.0f));
      point.push_back(route_pair(0.0f, 8.0f));
      point.push_back(route_pair(0.0f, 9.0f));
      point.push_back(route_pair(0.0f, 10.0f));
      break;
    case blue1:
      point.push_back(route_pair(0.0f, 1.0f));
      point.push_back(route_pair(0.0f, 2.0f));
      point.push_back(route_pair(0.0f, 3.0f));
      point.push_back(route_pair(0.0f, 4.0f));
      point.push_back(route_pair(0.0f, 5.0f));
      point.push_back(route_pair(0.0f, 6.0f));
      point.push_back(route_pair(0.0f, 7.0f));
      point.push_back(route_pair(0.0f, 8.0f));
      point.push_back(route_pair(0.0f, 9.0f));
      point.push_back(route_pair(0.0f, 10.0f));
      break;
    case blue2:
      point.push_back(route_pair(0.0f, 1.0f));
      point.push_back(route_pair(0.0f, 2.0f));
      point.push_back(route_pair(0.0f, 3.0f));
      point.push_back(route_pair(0.0f, 4.0f));
      point.push_back(route_pair(0.0f, 5.0f));
      point.push_back(route_pair(0.0f, 6.0f));
      point.push_back(route_pair(0.0f, 7.0f));
      point.push_back(route_pair(0.0f, 8.0f));
      point.push_back(route_pair(0.0f, 9.0f));
      point.push_back(route_pair(0.0f, 10.0f));
      break;
  }
  return point;
}
int main(){
  constexpr vector<route_pair> route = routeInit<color::red1>();
  RouteGenerator<float, 90000> route(std::move(routeInit()));
  route.routeMain();
}
