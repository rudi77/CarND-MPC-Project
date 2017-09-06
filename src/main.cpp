#include <assert.h>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <string>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"
#include "InputParser.h"
#include "VehicleState.h"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

/* Converts coordinates from one coordinate system into another one
* @param x x position in the target coordinate system.
* @param y y position in the target coordinate system.
* @param psi orientation in the target coordinate system.
* @param x0 x position in source coordinate system.
* @param y0 y position in source coordinate system.
*
* @return The new coordinates in the destination coordinate system.
*/
vector<double> transform_coordinates(double x, double y, double psi, double x0, double y0)
{
  auto cos_psi = cos(psi);
  auto sin_psi = sin(psi);

  // Rotation + Translation
  auto x_new = cos_psi * (x0-x) + sin_psi * (y0-y);
  auto y_new = -sin_psi * (x0-x) + cos_psi * (y0-y);

  return { x_new, y_new };
}



void printUsage()
{
  cout << "Usage: mpc [-w weights | -v ref_v| -h]" << endl;
  cout << "CmdLine args description:" << endl;
  cout << "-w weights  The weights used in the cost function. A space separted string" << endl;
  cout << "-v ref_v    The reference velocity" << endl;
  cout << "-h         Help description" << endl;
}

int main(int argc, char **argv) 
{
  // Used to read possible command line arguments
  InputParser input(argc, argv);

  if (input.cmdOptionExists("-h"))
  {
    printUsage();
    return 0;
  }

  // Default weights used in the cost function
  vector<double> weights = { 1, 10, 200, 200, 5, 2000, 100 };

  // The reference velocity
  auto ref_v = 80.0;

  if (input.cmdOptionExists("-w"))
  {
    auto weight_string = input.getCmdOption("-w");
    istringstream iss(weight_string);
    vector<string> result{ istream_iterator<string>(iss),{} };

    assert(result.size() == 7);
    weights.clear();

    std::transform
    (
      result.begin(), 
      result.end(), 
      back_inserter(weights),
      [](const string& str) { return stod(str); }
    );
  }

  if (input.cmdOptionExists("-v"))
  {
    const auto ref_v_string = input.getCmdOption("-v");
    ref_v = stod(ref_v_string);
  }

  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc(weights, ref_v);

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    //cout << sdata << endl;

    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') 
    {
      auto s = hasData(sdata);

      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];

          double px   = j[1]["x"];
          double py   = j[1]["y"];
          double psi  = j[1]["psi"];
          double v    = j[1]["speed"];
          double d    = j[1]["steering_angle"];
          double a    = j[1]["throttle"];


          // 1.) Transform waypoints into vehicles coordinate system
          assert(ptsx.size() == ptsy.size());
          auto wpoints = Eigen::MatrixXd(2, ptsx.size());

          for (size_t i=0; i<ptsx.size(); i++)
          {
            auto p_new = transform_coordinates(px, py, psi, ptsx[i], ptsy[i]);
            wpoints(0, i) = p_new[0];
            wpoints(1, i) = p_new[1];
          }
          
          // 2.) Compute polynom of third order for reference trajectory
          auto coeffs = polyfit(wpoints.row(0), wpoints.row(1), 3);

          // 3.) Compute cte and epsi
          const auto cte  = polyeval(coeffs, 0);
          const auto epsi = -atan(coeffs[1]);

          // 4.) Create state vector
          VehicleState vState = { 0, 0, v, psi, cte, epsi, d, a };

          // 5.) Predict state in 100ms
          const auto future_state = vState.State();

          // 6.) Call MPC solver and retrieve optimal trajectory
          const auto mpcOutput = mpc.Solve(future_state, coeffs);

          // 7.) Calculate steering angle and throttle using MPC.
          // Tips and Tricks provided by Udacity:Note if δ is positive we rotate counter - clockwise, or turn left.
          // In the simulator however, a positive value implies a right turn and a 
          // negative value implies a left turn
          const auto delay_index = 1;
          const auto steer_value = -1 * (mpcOutput.delta[delay_index] / 0.436332);
          const auto throttle_value = mpcOutput.a[delay_index];

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"]       = throttle_value;

          //Display the MPC predicted trajectory 
          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpcOutput.ptsx;
          msgJson["mpc_y"] = mpcOutput.ptsy;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          for (size_t i=0; i<ptsx.size(); i++)
          {
            next_x_vals.push_back(wpoints(0, i));
            next_y_vals.push_back(wpoints(1, i));
          }

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";

          //std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
