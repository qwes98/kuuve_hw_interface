#include <kuuvef1_hw_interface/kuuvef1_motor.h>
#include <serial/serial.h>
#include <sstream>
#include <string>
#include <vector>

using namespace std;

namespace kuuvef1_hw_interface
{
    void Kuuvef1Motor::tokenize(string const& str, const char delim, vector<string>& out)
    {
        stringstream ss(str);

        string s;
        while(getline(ss, s, delim)) {
            out.push_back(s);
        }
    }

    void Kuuvef1Motor::readSteerAndDrive(double& steer, double& drive)
    {
        string raw_str = ser_.readline(ser_.available());
        stringstream trans(raw_str);

        const char delim = ',';
        vector<string> vec;
        tokenize(raw_str, delim, vec);

        steer = stod(vec[0]);
        drive = stod(vec[1]);
    }


    void Kuuvef1Motor::actuate(double steer, double drive)
    {
        string packet = to_string(int(steer)) + "," + to_string(int(drive));
        ser_.write(packet);
    }

    void Kuuvef1Motor::init(string port, int baudrate)
    {
        ser_.setPort(port);
        ser_.setBaudrate(baudrate);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser_.setTimeout(to);
        ser_.open();

    }
}
