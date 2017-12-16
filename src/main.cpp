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

//int main()
int main(int argc, char *argv[])
{
  double p_gain(0.0),i_gain(0.0), d_gain(0.0);
  bool tune(false);
  if (argc > 4)
    tune = (std::stoi(argv[4]) == 0)? false: true;
  if (argc > 3) {
    p_gain = std::stod(argv[1]);
    i_gain = std::stod(argv[2]);
    d_gain = std::stod(argv[3]);
  }

  uWS::Hub h;

  PID pid;
  // TODO: Initialize the pid variable.
  //pid.Init(0.0, 0.0, 0.0, true);
  //pid.Init(0.5, 0.0, 3);
  //pid.Init(0.454726, 0.00492453, 3.14923); //GOOD Parameters!!
  //pid.Init(1.68966, 0.0047348, 4.87035); //GOOD Parameters!!!


  // The command line argumensts are used either as final
  // gains or initial values for tuning
  pid.Init(p_gain, i_gain, d_gain);
  if (tune) pid.Tune();

  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          
          pid.UpdateError(cte,speed);
          steer_value = pid.TotalError();
          if (steer_value < -1) steer_value = -1;
          if (steer_value > 1) steer_value = 1;
          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          // Once tuning is complete the messages are sent as is without manipulation
          if (!pid.IsTuning()){
            json msgJson;
            msgJson["steering_angle"] = steer_value;
            // When the angle is pretty much staright accelerate
            // Otherwise don't accelerate much.
            msgJson["throttle"] = (fabs(angle) < 7.5)?0.3:0.1;
            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
            std::cout << msg << std::endl;
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }
          // The PID Controller is still tuning and the vehicle may go
          // off track. Need to reset the simulator after few cycles
          // of hyper parameter adjustments.
          // The simulater resets every 10 adjustments. Hence, the reset
          // message is not sent until 10 adjustments.
          else if ( pid.NeedsReset() && (pid.TwiddlerAdjustments() < 10)){
            json msgJson;
            msgJson["steering_angle"] = steer_value;
            // Drive very slow to keep the vehicle at a constant
            // speed and adjust the gains such that the vehicle
            // will be on track. 
            msgJson["throttle"] = (speed > 2)? 0.0 : 0.1;
            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
            std::cout << msg << std::endl;
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }
          // After 10 adjustments, the simulator resets
          else if (pid.NeedsReset()) {
            std::string  msg = "42[\"reset\",{}]";
            std::cout << msg << std::endl;
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            pid.ResetTwiddlerAdjustments();
          }
          // The PID controller is still tuning but controlled enoguh
          // to keep the vehicle on track.
          else {
            json msgJson;
            msgJson["steering_angle"] = steer_value;
            // Drive at a reasonable speed but try to keep
            // keep constant speed. Tune the parametrs as
            // much as possibe.
            msgJson["throttle"] = (speed > 5)? 0.0 : 0.1;
            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
            std::cout << msg << std::endl;
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }
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
