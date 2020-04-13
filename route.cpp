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
using route_pair = std::pair<float, float>;
using route_tuple = std::tuple<float, float, unsigned int>;
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
          for(float j = u[0]; j <= u[3]; j+= (u[3] - u[0]) / 10){
              route_goal.push_back(route_pair(x(0, 0) * pow(j, 3) + x(1, 0) * pow(j, 2) + x(2, 0) * j + x(3, 0), 
                               x(0, 1) * pow(j, 3) + x(1, 1) * pow(j, 2) + x(2, 1) * j + x(3, 1))); 
          }
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
      for(auto &point : route_goal){
        outputFile << std::fixed << std::setprecision(5) << point.first << " " << point.second << "\n";
      }
      outputFile.close();
      return true;
    }
    float Timer();
    float time;
    std::vector<route_pair> route;
    std::vector<route_pair> route_goal;
};
enum class Coat{
  red1,
  red2,
  blue1,
  blue2
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
  RouteGenerator<float, 90000> routeObject(routeInit<Coat::blue1>());
  routeObject();
}
