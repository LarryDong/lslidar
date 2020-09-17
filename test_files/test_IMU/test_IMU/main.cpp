#include <string>
#include <boost/asio.hpp>
#include <utility>
#include <iostream>
#include <iomanip>
using namespace std;
using namespace boost::asio;
struct imu_data
{
    uint8_t valid;
    int sequence_num;
    int16_t temperature;
    float gyro[3];
    float acc[3];
};

class imu_1750
{
public:
    imu_1750() = default;
    explicit imu_1750(std::string dev):
    portName_(std::move(dev))
    {
        try
        {
            sp_ = new serial_port(ioSev_, portName_);
            sp_->set_option(serial_port::baud_rate(921600));
            sp_->set_option(serial_port::flow_control(serial_port::flow_control::none));
            sp_->set_option(serial_port::parity(serial_port::parity::none));
            sp_->set_option(serial_port::stop_bits(serial_port::stop_bits::one));
            sp_->set_option(serial_port::character_size(8));
        }
        catch (...)
        {
            std::cerr << "Exception Error: " << err_.message() << std::endl;
        }
    }
    bool get_data(imu_data &data)
    {
        unsigned char msg[36];
        try
        {
            while(true)
            {
                read(*sp_, boost::asio::buffer(msg, 1), err_);
                if(msg[0] == 0xFE)
                    read(*sp_, boost::asio::buffer(&msg[1], 3), err_);
                if(msg[0] == 0xFE && msg[1] == 0x81 &&
                   msg[2] == 0xFF && msg[3] == 0x55)
                {
                    read(*sp_, boost::asio::buffer(&msg[4], 32), err_);
                    break;
                }
            }

        }
        catch (...)
        {
            std::cerr << "Exception Error: " << err_.message() << std::endl;
            return false;
        }
        {
            printf("head : %d \n",msg[0]);
            data.valid = msg[28];
            data.sequence_num = msg[29];
            data.temperature = msg[30]<<8|msg[31];
            for(int i=0;i<3;i++)
            {
                for(int j=0;j<4;j++)
                {
                    bag1_.raw[3-j] = msg[4+j+i*4];
                    bag2_.raw[3-j] = msg[16+j+i*4];
                }
                data.gyro[i] = bag1_.value;
                data.acc[i] = bag2_.value;
            }
        }
        return true;
    }
private:
    boost::asio::serial_port *sp_{};
    boost::asio::io_service ioSev_;
    std::string portName_;
    boost::system::error_code err_;
    union bag
    {
        float value;
        uint8_t raw[4];
    }bag1_{},bag2_{};
};

float roll, pitch, yaw;

int main()
{
    imu_1750 imu("/dev/ttyUSB0");
    imu_data data{};

    while (true)
    {
        imu.get_data(data);
        if(data.valid==0x77)
        {
            // std::cout<<"sequence:"<<data.sequence_num<<
            // " roll:"<<data.gyro[0]<<" pitch:"<<data.gyro[1]<<
            // " yaw:"<<data.gyro[2]<<std::endl<<" acc x:"<<data.acc[0]<<
            // "acc y:"<<data.acc[1]<<" acc z:"<<data.acc[2]<<std::endl;
            roll += data.gyro[0];
            pitch +=data.gyro[1];
            yaw += data.gyro[2];
            cout << fixed << setprecision(5) << "X: " << roll << ", Y: " << pitch << ", Z: " << yaw << endl;
        }
    }
    return 0;
}
