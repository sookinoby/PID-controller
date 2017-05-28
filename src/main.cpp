#define _USE_MATH_DEFINES

#include "uWS/uWS.h"
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>


using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }



// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::stringstream hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return std::stringstream();
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    std::stringstream tmp = std::stringstream();
    tmp.str(s.substr(b1, b2 - b1 + 1));
    return tmp;
  }
  return std::stringstream();
}

double twiddle(double tol = 0.2) {
	PID pid;

	double p[] = { 0, 0, 0 };
	double dp[] = { 1, 1, 1 };
	pid.UpdateError(0.0);
	double steer_value = -pid.Kp * pid.p_error - pid.Kd * pid.d_error - pid.Ki * pid.i_error;
	if (steer_value > 1.0)
		steer_value = 0.99;
	if (steer_value < -1.0)
		steer_value = -0.99;
	double bestError = 100000;
	double sum_dp = dp[0] + dp[1] + dp[2];
	int it = 0;
	while (sum_dp > tol) {
		for (auto i = 0; i < 3; ++i)
		{
			p[i] += dp[i];
			pid.UpdateError(0.0);
			double steer_value = -pid.Kp * pid.p_error - pid.Kd * pid.d_error - pid.Ki * pid.i_error;

			if (pid.TotalError() < bestError)
			{
				bestError = pid.TotalError();
				dp[i] *= 1.1;
			}
			else {
				p[i] -= 2 * dp[i];
				pid.UpdateError(0.0);
				double steer_value = -pid.Kp * pid.p_error - pid.Kd * pid.d_error - pid.Ki * pid.i_error;
				if (pid.TotalError() < bestError)
				{
					bestError = pid.TotalError();
					dp[i] *= 1.1;
				}
				else {
					p[i] += dp[i];
					dp[i] *= 0.9;
					it += 1;
				}
			}
		}
	}
	return 0.0;
}


int main()
{

  uWS::Hub h;

  PID pid;
  pid.Init(0.1, 0.0002, 0.6);
  // TODO: Initialize the pid variable.


  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data));
      if (s.str() != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */


		  ///here is the twiddle alogirthm
		  
		  pid.UpdateError(cte);
		  double steer_value = -pid.Kp * pid.p_error - pid.Kd * pid.d_error - pid.Ki * pid.i_error;
		  if(steer_value > 1.0)
			steer_value = 0.99;
		  if(steer_value < -1.0)
			steer_value = -0.99;
		

          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value <<"angle :" << angle << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.4;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          (ws).send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      }
      else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        (ws).send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    (ws).close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen("0.0.0.0", port))
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