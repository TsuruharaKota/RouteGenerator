#include<cmath>
#include<vector>
#include<tuple>
#include<array>
#include<utility>
#include</usr/local/include/eigen3/Dense>
#include<iostream>
#include<queue>
#include<fstream>
#include<string>
#include<iomanip>

using namespace Eigen;
using catmull_tuple = std::tuple<float, float, float>;
using route_tuple = std::tuple<float, float, float, float>;
using std::cout;
using std::endl;
template<typename T>
class PosData3{
    public:
        T x;
        T y;
        T z;
        explicit PosData3(T _x, T _y, T _z):x(_x), y(_y), z(_z){}
        PosData3 operator+(PosData3 object){
            x += object.x;
            y += object.y;
            return *this;
        }
        PosData3 operator-(PosData3 object){
            x -= object.x;
            y -= object.y;
            return *this;
        }
        PosData3 operator*(PosData3 object){
            x *= object.x;
            y *= object.y;
            return *this;
        }
        void pow(){
            x *= x;
            y *= y;
        }
};
class CatmullRomSpline{
    public:
        explicit CatmullRomSpline(std::vector<route_tuple> &&input_param, int _frequency):frequency(_frequency){
            //全ての通過点を取得
            for(auto &point : input_param){
                //速度情報を省いてPosData3型にする
                PosData3<float> point3d(std::get<0>(point), std::get<1>(point), std::get<3>(point));
                transit_point.push_back(point3d);
                queue_angle.push(std::get<2>(point));
            }
        }
        std::vector<Vector4f> operator()(){
            //始点通過、終点通過、一般経路を分類
            int angle_count = 0;
            float x_prev = 0;
            float y_prev = 0;
            float angle_prev = 0;
            float angle_sum = 0;
            for(int counter = 0; counter < transit_point.size() - 1; ++counter){
                cal_L = 0;
                auto NomalCal = [&](int n, float p0, float p1, float p2, float p3){
                    a[n] = -p0 + 3.0f * p1 - 3.0f * p2 + p3;
                    b[n] = 2.0f * p0 - 5.0f * p1 + 4.0f * p2 - p3;
                    c[n] = -p0 + p2;
                    d[n] = 2.0f * p1;
                };
                auto ExceptionCal = [&](int n, float p0, float p1, float p2){
                    if(counter == 0){
                        //First Curve
                        b[n] = p0 - 2.0f * p1 + p2;
                        c[n] = -3.0f * p0 + 4.0f * p1 - p2;
                        d[n] = 2.0f * p0;
                    }else{
                        //Last Curve
                        b[n] = p0 - 2.0f * p1 + p2;
                        c[n] = -p0 + p2;
                        d[n] = 2.0f * p1;
                    }
                };
                if(counter == 0){
                    ExceptionCal(0, transit_point.at(counter).x, transit_point.at(counter + 1).x, transit_point.at(counter + 2).x);
                    ExceptionCal(1, transit_point.at(counter).y, transit_point.at(counter + 1).y, transit_point.at(counter + 2).y);
                    ExceptionCal(2, transit_point.at(counter).z, transit_point.at(counter + 1).z, transit_point.at(counter + 2).z);
                }else if(counter == transit_point.size() - 2){
                    ExceptionCal(0, transit_point.at(counter - 1).x, transit_point.at(counter).x, transit_point.at(counter + 1).x);
                    ExceptionCal(1, transit_point.at(counter - 1).y, transit_point.at(counter).y, transit_point.at(counter + 1).y);
                    ExceptionCal(2, transit_point.at(counter - 1).z, transit_point.at(counter).z, transit_point.at(counter + 1).z);
                }else{
                    NomalCal(0, transit_point.at(counter - 1).x, transit_point.at(counter).x, transit_point.at(counter + 1).x, transit_point.at(counter + 2).x);
                    NomalCal(1, transit_point.at(counter - 1).y, transit_point.at(counter).y, transit_point.at(counter + 1).y, transit_point.at(counter + 2).y);
                    NomalCal(2, transit_point.at(counter - 1).z, transit_point.at(counter).z, transit_point.at(counter + 1).z, transit_point.at(counter + 2).z);
                }
                for(int i = 0; i < frequency; ++i){
                    float t = static_cast<float>(i) / frequency;
                    float temp[4];
                    for(int k = 0; k < 3; ++k){
                        if(counter == 0){
                            temp[k] = 0.5f * ((b[k] * t * t) + (c[k] * t) + d[k]);
                        }else if(counter == transit_point.size() - 2){
                            temp[k] = 0.5f * ((b[k] * t * t) + (c[k] * t) + d[k]);
                        }else{
                            temp[k] = 0.5f * ((a[k] * t * t * t) + (b[k] * t * t) + (c[k] * t) + d[k]);
                        }
                    }
                    //区間距離を計算
                    cal_L += std::sqrt(temp[0] * temp[0] + temp[1] * temp[1]);
                    //x, yをそれぞれ入れる
                    //ここでvector4fにしたい
                    //x座標が距離でy座標が速度
                    angle_sum += atan2(temp[1] - y_prev, temp[0] - x_prev);
                    if(angle_count >= 9){
                        temp[3] = angle_sum / 10;
                        angle_prev = temp[3];  
                        angle_count = 0;
                        angle_sum = 0; 
                    }else{
                        temp[3] = angle_prev;
                        ++angle_count;
                    }
                    Vector4f Pos(temp[0], temp[1], temp[2], temp[3]);
                    goal_point.push_back(Pos);
                    x_prev = temp[0];
                    y_prev = temp[1];
                }
                queue_L_total.push(cal_L);
            }
            //この時点で座標は生成できている
            std::ofstream outputFile("Route.txt");
            for(auto &point : goal_point){outputFile << std::fixed << std::setprecision(5) << point(0) << " " << point(1) << " " << point(2) << " " << point(3) * (180 / M_PI) << "\n";}
            outputFile.close();
            return goal_point;
        }
        Vector2f getSubPoint(){
            //区間距離、角度の取得関数
            Vector2f info(queue_L_total.front(), queue_angle.front());
            queue_L_total.pop();
            queue_angle.pop();
            return info;
        }
    private:
        std::vector<float> transit_theta;
        std::vector<Vector4f> goal_point;
        std::vector<PosData3<float>> transit_point;
        std::queue<float> queue_L_total;
        std::queue<float> queue_angle;
        int frequency;
        float a[3], b[3], c[3], d[3];
        float cal_L;
};

enum class Coat{
  red1,
  red2,
  blue1,
  blue2
};
template<Coat color>
std::vector<route_tuple> routeInit(){
  std::vector<route_tuple> point;
  switch(color){
    case Coat::red1:
      point.push_back(route_tuple( 0.0f,  0.0f,  1.0f,   0.0f));
      point.push_back(route_tuple( 0.0f, 10.0f,  1.0f,  10.0f));
      point.push_back(route_tuple( 0.0f, 20.0f,  1.0f,  20.0f));
      point.push_back(route_tuple( 0.0f, 30.0f,  1.0f,  10.0f));
      point.push_back(route_tuple( 1.5f, 35.0f, -1.0f,  5.0f));
      point.push_back(route_tuple( 5.0f, 39.0f, -1.0f,  10.0f));
      point.push_back(route_tuple(10.0f, 40.0f,  1.0f,  20.0f));
      point.push_back(route_tuple(20.0f, 40.0f,  1.0f,  20.0f));
      point.push_back(route_tuple(30.0f, 40.0f,  1.0f,  10.0f));
      point.push_back(route_tuple(40.0f, 40.0f,  1.0f,  0.0f));
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

int main(int argc, char **argv){
    //経路の生成
    CatmullRomSpline splineObject(routeInit<Coat::red1>(), 1000);
    std::vector<Vector4f> route_info = splineObject();
    
    for(auto pos:route_info){
        //std::cout << pos(0) << " " << pos(1) << " " << pos(2) << " " << pos(3) << std::endl;
    }
}