#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

bool pre_cte_flag = false;
double pre_cte = 0.0;
double sum_cte = 0.0;

int main()
{
  uWS::Hub h;

  PID pid;
  PID pid_speed;
  // TODO: Initialize the pid variable.
  double Kp = 0.2;
  double Kd = 3.0;
  double Ki = 0.0;
  // double Kp_speed = 0.0;
  // double Kd_speed = 0.0;
  // double Ki_speed = 0.0;

  pid.Init(Kp, Ki, Kd);
  // pid_speed.Init(Kp_speed, Ki_speed, Kd_speed);

  h.onMessage([&pid, &pid_speed](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;
          double control_throttle;
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          if(!pre_cte_flag){
            pre_cte = cte;
            pre_cte_flag = true;
          }

          double diff_cte = cte - pre_cte;
          pre_cte = cte;
          sum_cte += cte;

          std::cout<<"diff_cte is: "<<diff_cte<<std::endl;
          std::cout<<"sum_cte is: "<<sum_cte<<std::endl;

          if(pid.TotalError() > 0.05){
            pid.UpdateError(cte);
          }
          std::cout << "Kp, Ki, Kd: " << pid.Kp << ", " << pid.Ki << ", " << pid.Kd << std::endl;

          steer_value = - pid.Kp * cte - pid.Kd * diff_cte - pid.Ki * sum_cte;

          std::cout << "Steering Value before norm is: " << steer_value << std::endl;

          if(steer_value > 1) steer_value = 1;
          if(steer_value < -1) steer_value = -1;
          
          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          // if(pid_speed.TotalError() > 0.2){
          //   pid_speed.UpdateError(cte);
          // }
          // std::cout << "Kp_speed, Ki_speed, Kd_speed: " << pid_speed.Kp << ", " << pid_speed.Ki << ", " << pid_speed.Kd << std::endl;

          // control_throttle = - pid_speed.Kp * cte - pid_speed.Kd * diff_cte - pid_speed.Ki * sum_cte;

          // std::cout << "throttle before norm is: " << control_throttle << std::endl;

          // if(control_throttle > 0.3) control_throttle = 0.3;
          // if(control_throttle < -0.1) control_throttle = 0.05;
          // if(sum_cte > 80){
          //   control_throttle = 0.1;
          // }
          // else{
            control_throttle = 0.3;
          // }

          std::cout << "throttle: " << control_throttle << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = control_throttle;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
