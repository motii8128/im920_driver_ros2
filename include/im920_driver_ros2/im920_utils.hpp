#ifndef SERIAL_UTILS_HPP_
#define SERIAL_UTILS_HPP_

#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include <time.h>
#include <sys/time.h>
#include <sys/ioctl.h>

#include <string>

namespace im920_driver_ros2
{
    class IM920Serial
    {
        public:
        IM920Serial(std::string port_name="/dev/ttyACM0", int baud_rate=115200):port_(port_name),baud_rate_(baud_rate)
        {

        }
        static IM920Serial *init_im920_serial(std::string port_name, int baud_rate)
        {
            return (IM920Serial *)(new IM920Serial(port_name, baud_rate));
        }

        int open_port()
        {
            fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
            struct termios conf_tio;
            tcgetattr(fd_, &conf_tio);
            speed_t BAUDRATE = get_baud_rate(baud_rate_);

            cfsetispeed(&conf_tio, BAUDRATE);
            cfsetospeed(&conf_tio, BAUDRATE);

            conf_tio.c_lflag &= ~(ECHO | ICANON);
            conf_tio.c_cc[VMIN]=0;
            conf_tio.c_cc[VTIME]=10;
            conf_tio.c_cflag &= ~PARENB;
            conf_tio.c_cflag &= ~CSTOPB;
            conf_tio.c_cflag |= CS8;
            tcsetattr(fd_,TCSANOW,&conf_tio);

            return fd_;
        }

        void close_port()
        {
            close(fd_);
        }

        int write_serial(std::string packet)
        {
            if(fd_ < 0)
            {
                return -1;
            }
            else
            {
                return write(fd_, packet.c_str(), packet.size());
            }
        }

        int read_serial(std::string *packet)
        {
            if(fd_ < 0)
            {
                return -1;
            }
            else
            {
                char buf[256];
                ssize_t bytes = read(fd_, buf, sizeof(buf));
                if(bytes < 0)
                {
                    return -1;
                }
                else
                {
                    buf[bytes] = '\0';
                    *packet = buf;
                    return 1;
                }
            }
        }

        std::string get_port_name()
        {
            return port_;
        }

        int get_baud_rate()
        {
            return baud_rate_;
        }

        private:
        speed_t get_baud_rate(int baud_rate)
        {
            switch(baud_rate)
            {
                case 9600:
                return B9600;
                case 19200:
                return B19200;
                case 38400:
                return B38400;
                case 57600:
                return B57600;
                case 115200:
                return B115200;
                case 230400:
                return B230400;
                case 460800:
                return B460800;
                case 500000:
                return B500000;
                case 576000:
                return B576000;
                case 921600:
                return B921600;
                case 1000000:
                return B1000000;
                case 1152000:
                return B1152000;
                case 1500000:
                return B1500000;
                case 2000000:
                return B2000000;
                case 2500000:
                return B2500000;
                case 3000000:
                return B3000000;
                case 3500000:
                return B3500000;
                case 4000000:
                return B4000000;
                default:
                return -1;
            }
        }
        std::string port_;
        int baud_rate_;
        int fd_;
    };
}

#endif