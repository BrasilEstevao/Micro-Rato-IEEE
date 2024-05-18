// Left Motor
#define PWM_A 26
#define IN_1A 17
#define IN_2A 25

// Right Motor
#define PWM_B 14
#define IN_1B 16
#define IN_2B 27

#define STBY 18

//Line sensor

#define IR 15
#define S1 2 //2
#define S2 4 //4
#define S3 35
#define S4 39
#define S5 34

// Encoders
#define ENCODER_A_PIN 12

typedef enum States {
    start,
    follow,
    turn_right,
    turn_left,
    turn_back,
    small_forward_1,
    small_forward_2,
    decision,
    rev,
    end
} States;
