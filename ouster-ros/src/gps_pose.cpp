#include <ros/ros.h>
#include <serial/serial.h>
#include <sensor_msgs/NavSatFix.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

// Function to parse GPGGA sentence
bool parseGPGGA(const std::string &data, std::string &timestamp, double &latitude, double &longitude, double &altitude) {
    std::stringstream ss(data);
    std::string item;
    std::vector<std::string> parsedData;
    
    while (std::getline(ss, item, ',')) {
        parsedData.push_back(item);
    }
    
    if (parsedData.size() < 15 || parsedData[0] != "$GPGGA") {
        return false;
    }
    
    // Parse timestamp
    timestamp = parsedData[1];

    // Parse latitude
    double lat = std::stod(parsedData[2].substr(0, 2)) + std::stod(parsedData[2].substr(2)) / 60.0;
    if (parsedData[3] == "S") {
        lat = -lat;
    }
    
    // Parse longitude
    double lon = std::stod(parsedData[4].substr(0, 3)) + std::stod(parsedData[4].substr(3)) / 60.0;
    if (parsedData[5] == "W") {
        lon = -lon;
    }
    
    // Parse altitude
    double alt = std::stod(parsedData[9]);
    
    latitude = lat;
    longitude = lon;
    altitude = alt;
    
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "gps_serial_node");
    ros::NodeHandle nh("~");

    std::string port;
    int baudrate;
    std::string output_file;
    
    nh.param<std::string>("port", port, "/dev/ttyUSB0");
    nh.param<int>("baudrate", baudrate, 115200);
    nh.param<std::string>("output_file", output_file, "/media/xec/yjy/data/0/gps.txt");

    serial::Serial ser;
    try {
        ser.setPort(port);
        ser.setBaudrate(baudrate);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    } catch (serial::IOException &e) {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if (ser.isOpen()) {
        ROS_INFO_STREAM("Serial Port initialized");
    } else {
        return -1;
    }

    ros::Publisher gps_pub = nh.advertise<sensor_msgs::NavSatFix>("/gps/fix", 10);

    std::ofstream outfile;
    outfile.open(output_file, std::ios_base::app);

    ros::Rate loop_rate(10);
    while (ros::ok()) {
        if (ser.available()) {
            std::string line = ser.readline();
            std::string timestamp;
            double latitude, longitude, altitude;

            if (parseGPGGA(line, timestamp, latitude, longitude, altitude)) {
                std::stringstream output_line;
                output_line << timestamp << "," << latitude << "," << longitude << "," << altitude << "\n";
                outfile << output_line.str();
                ROS_INFO_STREAM("Saved data: " << output_line.str());

                // Create and publish NavSatFix message
                sensor_msgs::NavSatFix gps_msg;
                gps_msg.header.stamp = ros::Time::now();
                gps_msg.header.frame_id = "gps";
                gps_msg.latitude = latitude;
                gps_msg.longitude = longitude;
                gps_msg.altitude = altitude;
                gps_msg.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
                gps_msg.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;

                gps_pub.publish(gps_msg);
                ROS_INFO_STREAM("Published GPS fix: [" << latitude << ", " << longitude << ", " << altitude << "]");
            }
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    outfile.close();
    return 0;
}
