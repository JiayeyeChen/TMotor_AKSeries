#include <iostream>
#include <boost/asio.hpp>
#include <main.hpp>
#include <serial_test.hpp>
class test_class_parent
{
    public:
        test_class_parent(int parentval);
        int getval();
    private:
        int parent_value;

};

class test_class_son:public test_class_parent
{
    public:
        test_class_son(int parentval);
    private:
        int son_value;
};

class test_class_third
{
    public:
        test_class_third(test_class_parent* parent);
        int getval();
    private:
        int thirdval;
};

void writeString(boost::asio::serial_port& serial, std::string s)
{
    boost::asio::write(serial,boost::asio::buffer(s.c_str(),s.size()));
}

int main(int argc, char** argv)
{
    boost::asio::io_service myIO;
    
    boost::asio::serial_port mySerialPort(myIO, "/dev/ttyACM0");
    if (mySerialPort.is_open())
    {
        std::cout<<"Originally opened, now closed already!"<<std::endl;
        mySerialPort.close();
    }
    else
    {
        std::cout<<"Originally not opened yet!"<<std::endl;
    }
    mySerialPort.open("/dev/ttyACM0");
    mySerialPort.set_option(boost::asio::serial_port_base::baud_rate(115200));
    mySerialPort.set_option(boost::asio::serial_port::flow_control());
	mySerialPort.set_option(boost::asio::serial_port::parity());
	mySerialPort.set_option(boost::asio::serial_port::stop_bits());
	mySerialPort.set_option(boost::asio::serial_port::character_size(8));
    
    
    if (mySerialPort.is_open())
    {
        std::cout<<"Serial port opened successfully!"<<std::endl;
    }
    for (int i = 1; i < argc; i++)
    {
        writeString(mySerialPort, (std::string)argv[i]);
    }
    


    std::cout<<"Goodluck Jiaye"<<std::endl;
    test_class_parent myParent(8);
    test_class_son mySon(7);
    std::cout<<"My parent value is:  " << myParent.getval()<<std::endl;
    std::cout<<"My son value is:  " << mySon.getval()<<std::endl;
    test_class_third myThird(&mySon);
    std::cout<<"My third value is:  " << myThird.getval()<<std::endl;
}


test_class_parent::test_class_parent(int parentval):parent_value(parentval)
{}
int test_class_parent::getval()
{
    return parent_value;
}
test_class_son::test_class_son(int parentval):test_class_parent(parentval)
{}
test_class_third::test_class_third(test_class_parent* parent)
{
    thirdval = parent->getval();
}

int test_class_third::getval()
{
    return thirdval;
}



// using namespace std;
// using namespace boost;

// int main(int argc, char* argv[])
// {
//     try {

//         SimpleSerial serial("/dev/ttyACM0",9600);
//         // serial.serial.open("/dev/ttyACM0");
//         if (serial.serial.is_open())
//         {
//             cout<<"serial opened successfully!"<<endl;
//         }
//         serial.writeString("a");

//         cout<<serial.readLine()<<endl;

//     } catch(boost::system::system_error& e)
//     {
//         cout<<"Error: "<<e.what()<<endl;
//         return 1;
//     }
// }
