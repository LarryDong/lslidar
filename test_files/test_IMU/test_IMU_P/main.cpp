#include <string>
#include <boost/asio.hpp>
#include <utility>
#include <iostream>

using namespace boost::asio;
struct imu_data
{
    uint8_t valid;
    int sequence_num;
    int16_t temperature;
    float gyro[3];
    float angle[3];//yaw pitch roll 
    float acc[3];
    float mag[3];
};

class imu_p
{
public:
    imu_p() = default;
    explicit imu_p(std::string dev):
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
        // 初始化时发送开始传输数据指令才会开始发送数据
        unsigned char msg[9] = {0xAA,0x55,0x00,0x00,0x07,0x00,0x33,0x3A,0x00};
         try
         {
            read(*sp_, boost::asio::buffer(msg, 9), err_);
         }
         catch (...)
        {
            std::cerr << "Exception Error: " << err_.message() << std::endl;
        }
    }
    bool get_data(imu_data &data)
    {
       unsigned char msg[42];
        try
        {
            while(true)
            {
                read(*sp_, boost::asio::buffer(msg, 1), err_);
                if(msg[0] == 0xAA)
                    read(*sp_, boost::asio::buffer(&msg[1], 4), err_);
                if(msg[1] == 0x55 && msg[2] == 0x01 &&
                   msg[3] == 0x33 && msg[4] == 0x28)
                {
                    read(*sp_, boost::asio::buffer(&msg[5], 37), err_);
                    break;
                }
            }

        }
        catch (...)
        {
            std::cerr << "Exception Error: " << err_.message() << std::endl;
            return false;
        }
        for(int i=0;i<3;i++)
        {
            data.angle[i] = (float)(msg[7+2*i] <<8|msg[7+2*i+1])*0.01f;
            if(i>0)
            {// pitch 和 roll 采用补码的方式存储
                data.angle[i] = discode(msg[7+2*i],msg[7+2*i+1])*0.01f;
            }
            data.gyro[i] = (float)discode(msg[13+2*i],msg[13+2*i+1])/50;
            data.acc[i] = (float)discode(msg[19+2*i],msg[19+2*i+1])/2000;
            data.mag[i] = discode(msg[25+2*i],msg[25+2*i+1]);
        }
        return true;
    }
private:
    boost::asio::serial_port *sp_{};
    boost::asio::io_service ioSev_;
    std::string portName_;
    boost::system::error_code err_;
    int discode(uint8_t first,uint8_t second)
    {
        uint16_t tmp = ((uint16_t)first<<8|second) - 0x01;
        if(tmp>>15)
        {
            tmp = ~tmp;
            return -tmp;
        }
        else 
        {
            return tmp;
        }
    }
};

int main()
{
    imu_p imu("/dev/ttyUSB0");
    imu_data data{};

    while (true)
    {
        imu.get_data(data);
        {
            std::cout<<"sequence:"<<data.sequence_num<<
            " yaw:"<<data.angle[0]<<" pitch:"<<data.angle[1]<<
            " roll:"<<data.angle[2]<<std::endl<<" acc x:"<<data.acc[0]<<
            "acc y:"<<data.acc[1]<<" acc z:"<<data.acc[2]<<std::endl;
        }
    }
    return 0;
}
