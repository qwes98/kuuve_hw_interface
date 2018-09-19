#ifndef KUUVEF1_MOTOR_H
#define KUUVEF1_MOTOR_H

#include <string>

namespace kuuvef1_hw_interface
{
    class Kuuvef1Motor
    {
        public:
            Kuuvef1Motor(std::string steer_name = "steer", std::string drive_name = "drive")
                : steer_name_(steer_name), drive_name_(drive_name)
            {}

            std::string getSteerName() { return steer_name_; }
            std::string getDriveName() { return drive_name_; }

            void init(std::string port, int baudrate);

            void readSteerAndDrive(double& steer, double& drive);

            // TODO: just make message and send it to arduino (this is based on that steer_drive_controller use control mechanism(e.g. pid control))
            void actuate(double steer, double drive);

        private:
            void tokenize(std::string const& str, const char delim, std::vector<std::string>& out);

            serial::Serial ser_;

            std::string steer_name_;
            std::string drive_name_;

            int cur_steer_pos_;
            int cur_drive_vel_;
    };
}

#endif