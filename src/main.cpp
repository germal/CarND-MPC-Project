/*                                                                         80->|
 * main.cpp
 *
 * Modifications: James William Dunn
 *          Date: June 15, 2017
 */

#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"
#include "timer.h"




// NOTE: to turn on yellow and green lines, toggle the
// following boolean to "true"

bool visualize_paths = false;




// By default, a 100ms latency between actuation command and actuation
// action is enabled. To operate without actuation latency, toggle the 
// following boolean to "false"

bool add_latency = true;



// Solver duration is predicted based on the exponential averaging
// of solver times from prior frames. The alpha value determines the
// weight of the current value. For example 0.65 means 65% of
// the current frame's solver duration will be factored into the
// running average. Other trials: 0.5, 0.1909, 0.2565

double alpha = 0.65;


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
// github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
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
double maxSpeed = 0;
Timer1 timerSolver;
Timer1 timerLatency;
Timer1 timerFrame;
double solver_timeavg = 0.03;  // Start with 30ms average
long frameCount = 0;

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          timerLatency.start();
          timerSolver.start();

          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double px_ = px, py_ = py;
          double psi = j[1]["psi"]; // radians
          double v = j[1]["speed"]; // mph
          if (v > maxSpeed) maxSpeed = v;
          double steering_angle = j[1]["steering_angle"];  // radians
          double throttle = j[1]["throttle"];
          double dx, dy;
          // Vector of last waypoint and next five waypoints
          Eigen::VectorXd carx(6);
          Eigen::VectorXd cary(6);

          // Roughly account for latency by projection into future
          double latency = solver_timeavg; // by solver time
          //double latency = 0.035;
          if (add_latency) latency += 0.103; // plus apx 103ms 
          px = px + latency * v * 0.44704 * cos(psi);
          py = py + latency * v * 0.44704 * sin(psi);
          psi = psi - latency * v * 0.44704 * steering_angle/2.67;
          
          double cosPsi = cos(-psi);
          double sinPsi = sin(-psi);
          // Transform global waypoints to vehicle coordinates
          for (int i = 0; i < ptsx.size(); ++i) {
            dx = ptsx[i] - px; // translate
            dy = ptsy[i] - py;
            carx[i] = dx * cosPsi - dy * sinPsi; // rotate
            cary[i] = dy * cosPsi + dx * sinPsi;
          }
          Eigen::VectorXd coeffs = polyfit(carx, cary, 3);
          
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;
          double cte = coeffs[0]; // polyeval(coeffs, 0.0);
          
          // Build the current state variable to pass to the solver
          Eigen::VectorXd state(6);
          state << 0,                     // x
                   0,                     // y
                   0,                     // psi
                   v,                     // velocity
                   cte,                   // CTE
                   -atan(coeffs[1]);      // ePsi
       
          // Calculate steering angle and throttle using MPC.
          vector<double> solution = mpc.Solve(state, coeffs, 
            mpc_x_vals, mpc_y_vals);
          

          double steer_value = solution[0];
          double throttle_value = solution[1];


          // Prepare control packet to send to simulator
          json msgJson;
          // delta is in the range: [-.4, .4] (constrained in solver).
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          if (visualize_paths) {
            // The predicted path points in the simulator are connected by a
            // Green line: vectors are dynamically resized to N-1 in MPC.cpp
            msgJson["mpc_x"] = mpc_x_vals;
            msgJson["mpc_y"] = mpc_y_vals;

            // The waypoints in the simulator are connected by a Yellow line
            vector<double> next_x(10);
            vector<double> next_y(10);
            for (int i = 0; i < 10; ++i) {
              next_x[i] = i*4;
              next_y[i] = polyeval(coeffs, i*4);
            }
            msgJson["next_x"] = next_x;
            msgJson["next_y"] = next_y;
          } else { // do not visualize...
            msgJson["mpc_x"] = "";
            msgJson["mpc_y"] = "";
            msgJson["next_x"] = "";
            msgJson["next_y"] = "";
          }

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          Timer1::ms duration = timerSolver.duration();
          long solverTime = duration.count();
          if (solverTime < 100) // filter outliers
            //solver_timeavg = (solver_timeavg + solverTime/1000.0) / 2.0; 
            // Exponential moving average
            solver_timeavg = alpha*solverTime/1000.0 + (1-alpha)*solver_timeavg;
          
          //long solverDiff = 35 - solverTime;
          //if (solverDiff > 0)
          //  this_thread::sleep_for(chrono::milliseconds(solverDiff));

          // AT THIS POINT, the actuator command is ready to be delivered
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does NOT actuate the commands instantly.
          //
          //
          // SET THIS TO 100 MILLISECONDS BEFORE SUBMITTING.
          if (add_latency)
            this_thread::sleep_for(chrono::milliseconds(100));
                    
          Timer1::ms durationLat = timerLatency.duration();
          Timer1::ms frameDuration = timerFrame.duration(); // frame-to-frame
          long outfd = frameDuration.count();
          
          // Output metrics
          std::cout << ++frameCount 
          << "," << ptsx[0]             // waypoint behind     (int)round()
          << "," << ptsy[0]
          << "," << px_                 // position
          << "," << py_ 
          << "," << durationLat.count() // total latency
          << "," << solverTime 
          << "," << (long)round(solver_timeavg*1000) 
          << "," << outfd               // total frame-to-frame
          << "," << maxSpeed 
          << "," << cte 
          << std::endl;
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
  // program doesn't compile :-(
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
