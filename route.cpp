#include<iostream>
#include<vector>
#include<chrono>
#include<utility>
#include<tuple>
#include<fstream>
#include<string>
#include<Eigen/Dense>
#include<vector>
using route_pair = std::pair<float, float>;
using route_tuple = std::tuple<float, float, unsigned int>;
template<typename T, int N>
class RouteGenerator{
  public:
    RouteGenerator(std::vector<route_pair>& passing_point):route(passing_point){}
    void routeMain(){
      try{
        if((this -> generateRoute()) == false)throw 1;
        //if((this -> fileSet()) == false)throw 2;
      }
      catch (int error_num){
        switch(error_num){
          case 1:
            std::cout << "error in generateRoute function" << std::endl;
          case 2:
            std::cout << "error in fileSet function" << std::endl;
          default:
            std::cout << "error occurred" << std::endl;
        }
      }
    }
  private:
    bool generateRoute(){
      for(int i = 0; i < 9; ++i){
        if((route[i].first != route[i + 1].first) && (route[i].second == route[i + 1].second)){
          //X Linear interpolation
          float devide_x = (route[i + 1].first - route[i].first) / 5;
          for(int j = 0; j < 5; ++j){
            route_goal.push_back(route_pair(route[i].first + (devide_x * j), route[i].second));
          }
        }else if((route[i].first == route[i + 1].first) && (route[i].second != route[i + 1].second)){
          //Y Linear interpolation
          float devide_y = (route[i + 1].second - route[i].second) / 5;
          for(int j = 0; j < 5; ++j){
            route_goal.push_back(route_pair(route[i].first, route[i].second + (devide_y * j)));
          }
        }else{
          //Curve interpolation
          std::array<float, 4> u;
          for(int j = 0; j < 4; ++j){
            if(j == 0){
              if(i == 0) u[j] = 0;
              else u[j] = std::sqrt(std::pow(route[i + j].first - route[i + j - 1].first, 2) + 
                          std::pow(route[i + j].second - route[i + j - 1].second, 2));
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
          Eigen::Matrix<float, 2, 4> b;
          b << route[i    ].first, route[i    ].second,
               route[i + 1].first, route[i + 1].second,
               route[i + 2].first, route[i + 2].second,
               route[i + 3].first, route[i + 3].second;
          Eigen::ColPivHouseholderQR<Eigen::Matrix4f> dec(A);
          Eigen::Matrix<float, 2, 4> x = dec.solve(b);
          for(int j = u[0]; j <= u[3]; j+= (u[3] - u[0]) / 10){
            route_goal.push_back(route_pair(x(0, 0) * pow(j, 3) + x(0, 1) * pow(j, 2) + x(0, 2) * j + x(0, 3), 
                                 x(1, 0) * pow(j, 3) + x(1, 1) * pow(j, 2) + x(1, 2) * j + x(1, 3)));  
          }
          i += 4;
        }
      }
      int devide_time = N / 500;
      for(int i = 0; i < devide_time; ++i){
      }
      return true;
    }
    bool fileSet(){
      //std::ofstream writeFile;
      //writeFile.open("RouteGenerator/Test.txt");
      //writeFile <<;
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
  std::vector<route_pair> point(10);
  switch(color){
    case Coat::red1:
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
    case Coat::red2:
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
    case Coat::blue1:
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
    case Coat::blue2:
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
  std::vector<route_pair> route = routeInit<Coat::red1>();
  RouteGenerator<float, 90000> routeObject(route);
  routeObject.routeMain();
}
