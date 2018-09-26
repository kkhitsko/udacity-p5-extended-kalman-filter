#include <uWS/uWS.h>
#include <iostream>
#include <fstream>
#include "json.hpp"
#include <math.h>
#include "FusionEKF.h"
#include "tools.h"

using namespace std;

// for convenience
using json = nlohmann::json;

struct RMSEResult {
      VectorXd RMSE;
      double p_x;
      double p_y;
};

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.find_first_of("]");
    if (found_null != std::string::npos) {
        return "";
    }
    else if (b1 != std::string::npos && b2 != std::string::npos) {
        return s.substr(b1, b2 - b1 + 1);
    }
    return "";
}

struct RMSEResult processData( istringstream &iss, FusionEKF &fusionEKF, vector<VectorXd> &estimations, vector<VectorXd> &ground_truth ) {

    MeasurementPackage meas_package;

    long long timestamp = 0;

    string sensor_type;
    iss >> sensor_type;

    if (sensor_type.compare("L") == 0) {
        meas_package.sensor_type_ = MeasurementPackage::LASER;
        meas_package.raw_measurements_ = VectorXd(2);

        double px, py = 0.0;
        iss >> px >> py >> timestamp;

        meas_package.raw_measurements_ << px, py;
        meas_package.timestamp_ = timestamp;
        // uncomment if you want to skip all Laser measurements
        //return;
    } else if (sensor_type.compare("R") == 0) {

        meas_package.sensor_type_ = MeasurementPackage::RADAR;
        meas_package.raw_measurements_ = VectorXd(3);

        double ro, theta, ro_dot;
        iss >> ro >> theta >> ro_dot >> timestamp;

        meas_package.raw_measurements_ << ro,theta, ro_dot;
        meas_package.timestamp_ = timestamp;
        // uncomment if you want to skip all Radar measurements
        //return;
    }

    double x_gt, y_gt, vx_gt, vy_gt = 0.0;

    iss >> x_gt >> y_gt >> vx_gt >> vy_gt;

    VectorXd gt_values(4);
    gt_values << x_gt, y_gt, vx_gt, vy_gt;


    ground_truth.emplace_back(gt_values);

    //Call ProcessMeasurment(meas_package) for Kalman filter
    fusionEKF.ProcessMeasurement(meas_package);

    //Push the current estimated x,y positon from the Kalman filter's state vector

    double p_x = fusionEKF.ekf_.x_(0);
    double p_y = fusionEKF.ekf_.x_(1);
    double v1  = fusionEKF.ekf_.x_(2);
    double v2 = fusionEKF.ekf_.x_(3);

    VectorXd estimate(4);
    estimate << p_x, p_y, v1, v2;

    estimations.emplace_back(estimate);

    Tools tools;
    VectorXd RMSE = tools.CalculateRMSE(estimations, ground_truth);

    cout << RMSE << endl;

    struct RMSEResult result;
    result.RMSE = RMSE;
    result.p_x = p_x;
    result.p_y = p_y;

    return result;
}

int main(int argc, char* argv[])
{
    uWS::Hub h;

    // Create a Kalman Filter instance
    FusionEKF fusionEKF;

    // used to compute the RMSE later
    Tools tools;
    vector<VectorXd> estimations;
    vector<VectorXd> ground_truth;

    if ( argc == 2 ) {
        string in_file_name_ = argv[1];
        ifstream in_file_(in_file_name_.c_str(), ifstream::in);

        string line;

        vector<MeasurementPackage> measurements_list;

        // prep the measurement packages (each line represents a measurement at a
        // timestamp)
        while (getline(in_file_, line)) {
            istringstream iss(line);
            (void)processData(iss, fusionEKF, estimations, ground_truth );
        }
    }
    else {

        h.onMessage([&fusionEKF,&tools,&estimations,&ground_truth](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
            // "42" at the start of the message means there's a websocket message event.
            // The 4 signifies a websocket message
            // The 2 signifies a websocket event

            if (length && length > 2 && data[0] == '4' && data[1] == '2')
            {
                auto s = hasData(std::string(data));
                if (s != "") {

                    auto j = json::parse(s);
                    std::string event = j[0].get<std::string>();

                    if (event == "telemetry") {
                        // j[1] is the data JSON object

                        string sensor_measurment = j[1]["sensor_measurement"];
                        istringstream iss(sensor_measurment);

                        auto result = processData(iss, fusionEKF, estimations, ground_truth );
                        auto RMSE = result.RMSE;

                        json msgJson;
                        msgJson["estimate_x"] = result.p_x;
                        msgJson["estimate_y"] = result.p_y;
                        msgJson["rmse_x"] =  RMSE(0);
                        msgJson["rmse_y"] =  RMSE(1);
                        msgJson["rmse_vx"] = RMSE(2);
                        msgJson["rmse_vy"] = RMSE(3);
                        auto msg = "42[\"estimate_marker\"," + msgJson.dump() + "]";
                        // std::cout << msg << std::endl;
                        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

                    }
                } else {
                    std::string msg = "42[\"manual\",{}]";
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                }
            }
        });

        // We don't need this since we're not using HTTP but if it's removed the program
        // doesn't compile :-(
        h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
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

        h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
            ws.close();
            std::cout << "Disconnected" << std::endl;
        });

        int port = 4567;
        if (h.listen(port))
        {
            std::cout << "Listening to port " << port << std::endl;
        } else {
            std::cerr << "Failed to listen to port" << std::endl;
            return -1;
        }
        h.run();
    }
}
