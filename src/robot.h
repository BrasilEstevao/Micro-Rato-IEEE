#include <Arduino.h>
#include <math.h>

typedef enum{
    followLine,  //0
    turnRight,   //1
    turnLeft,    //2
    turnBack     //3

} estados;

class robot_t{
    public:

    byte state;
    uint32_t tis, tes;


    robot_t();    
    void setRobotVW( float Vnom, float Wnom);
    void setState(byte new_state);
};