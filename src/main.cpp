#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <fstream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

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
double polyeval_main(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

double average(double arr[], int LIMIT) {
    double average = 0.0;
    for (int i = 0; i < LIMIT; ++i) {
        average += arr[i];
    }
    average /= LIMIT;
    return average;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit_main(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
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

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  // int prev_psis_i = 0;
  const int LIMIT = 12;
  int prev_psis_i = 0;
  const double MIN_PSI_THRESHOLD = 0.1;
  const double MED_PSI_THRESHOLD = 0.6;
  double prev_psis[LIMIT] = {0.0};

  const double Lf = 2.67;
  const double MIN_COEFF_SUM_THRESHOLD = 1e-2;

  int prev_i = 0;
  double prev_coeff_sums[LIMIT] = {1.0};

  const int latency_ms = 100;
  const double dt = 0.1;

  h.onMessage([&mpc, &Lf, &latency_ms, &dt/*, &MIN_COEFF_SUM_THRESHOLD, &prev_coeff_sums, &prev_psis, &prev_psis_i, &LIMIT, &MIN_PSI_THRESHOLD, &MED_PSI_THRESHOLD*/](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
          double delta = j[1]["steering_angle"];
          double acc = j[1]["throttle"];

          // Could maybe try choosing the order based on size of psi.
          // If psi small, on a straight. Can get away with evaluating
          // using a straight line with minimal loss of accuracy.
          // If psi large, in a corner. Evaluate using higher order.
          Eigen::VectorXd xvals(ptsx.size());
          Eigen::VectorXd yvals(ptsy.size());

          // Convert incoming way-points to vehicle's coordinate system.
          // First subtract px and py off of them, and then apply the transform.
          for (int i = 0; i < ptsx.size(); ++i) {
            xvals[i] = (ptsx[i] - px) * std::cos(-psi) - (ptsy[i] - py) * std::sin(-psi);
            yvals[i] = (ptsx[i] - px) * std::sin(-psi) + (ptsy[i] - py) * std::cos(-psi);
          }

          // prev_psis[++prev_psis_i % LIMIT] = psi;

          // Fit a third-degree polynomial to the way-points - this is the reference trajectory. 
          // double average_psi = fabs(average(prev_psis, LIMIT));
          Eigen::VectorXd coeffs = polyfit_main(xvals, yvals, 3);
          // if(average_psi <= MIN_PSI_THRESHOLD)
          //   coeffs = polyfit_main(xvals, yvals, 1);
          // else if(average_psi <= MED_PSI_THRESHOLD) {
          //   coeffs = polyfit_main(xvals, yvals, 2);
          // }
          // else
          //   coeffs = polyfit_main(xvals, yvals, 3);

          // for (int i = 0; i < coeffs.size(); ++i) {
          //     std::cout << coeffs[i] << ", ";
          // }
          // std::cout << "\nPREV_PSIS:\n";

          // for (int i = 0; i < LIMIT; ++i) {
          //     std::cout << prev_psis[i] << ", ";
          // }
          // std::cout << "\nAverage: " << average << "\n";

          // Solve CTE at point x=0, since from the vehicle's perspective, we ARE 0,0
          double cte = polyeval_main(coeffs, 0);

          double epsi = -atan(coeffs[1]);

          // Check if quadratic and cubic terms are sufficiently small, increase latency by 10%.
          // prev_coeff_sums[prev_i % LIMIT] = fabs(coeffs[2]) + fabs(coeffs[3]);
          // double average_coeff_sum = average(prev_coeff_sums, LIMIT);
          // std::cout << "\n#######################\n" << average_coeff_sum << ", " << MIN_COEFF_SUM_THRESHOLD << "\n#######################\n";
          // if(average_coeff_sum <= MIN_COEFF_SUM_THRESHOLD) {
          //   std::cout << "\n#######################\n" << "Cool down" << "\n#######################\n";
          //   dt = 0.15;
          //   latency_ms = 150;
          // }
          // else {
          //   std::cout << "\n#######################\n" << "Warm up" << "\n#######################\n";
          //   dt = 0.1;
          //   latency_ms = 100;
          // }
          // prev_i++;

          Eigen::VectorXd state(6);

          // Accounts for the latency - use the model to predict where we will be in 100ms...
          // and act according on THAT information rather than what is the case right now.
          double delayed_state_x = v * dt;
          double delayed_state_y = 0.0;
          double delayed_state_psi = 0.0 - delta * (v / Lf) * dt;
          double delayed_state_v = v + acc * dt;
          double delayed_state_cte = cte + v * std::sin(epsi) * dt;
          double delayed_state_epsi = epsi - delta * (v / Lf) * dt;

          state << delayed_state_x, delayed_state_y, delayed_state_psi, delayed_state_v, delayed_state_cte, delayed_state_epsi;

          std::vector<double> res = mpc.Solve(state, coeffs, dt);

          double steer_value = res[0];
          double throttle_value = res[1];

          json msgJson;
          // Divide by deg2rad(25) before sending the steering value back to ensure correct range.
          // Including Lf accounts for actual turn radius.
          msgJson["steering_angle"] = steer_value / (deg2rad(25) * Lf);
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals = {state[0]};
          vector<double> mpc_y_vals = {state[1]};

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a green line

          for(int i = 2; i < res.size(); i += 4) {
            mpc_x_vals.push_back(res[i]);
            mpc_y_vals.push_back(res[i + 1]);
          }

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          double step = 3;
          int num_points = 30;
          
          for (int i = 1; i < num_points; ++i) {
            next_x_vals.push_back(step * i);
            next_y_vals.push_back(polyeval_main(coeffs, step * i));
          }

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does not actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(latency_ms));
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
