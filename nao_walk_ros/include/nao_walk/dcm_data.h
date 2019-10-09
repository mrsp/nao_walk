#ifndef DCM_DATA_H 
#define DCM_DATA_H

#define LEFT 0
#define RIGHT 1

#define WALK_ENGINE_ID 1001
#define STEP_MAX_CMD 32

#define STATUS_DONE 0
#define STATUS_PENDING 1
#define STATUS_ABORTED 2
#define STATUS_INV -1
#define PARAM_INV -1
#define STEPS_INV -1 //invalid steps-dcm not running
#define STEPS_INIT 0
#define INV -1
#define RECONFIG_SIZE 15
enum COMMANDS
{
    NONE=0,
    STEPS,
    SPEEK,
    RECONFIG,
    VELOCITY,
    COMMANDS_SIZE    
};

typedef struct
{
    float vx;
    float vy;
    float vw;
} velocity_t;


enum BEHAVIOURS
{    
    CROUCH=COMMANDS_SIZE,
    WAKE_UP,
    MAKARENA,
    BALL,
    DANCE_EVOL,
    GANGNAM_ST,
    VANGELIS,
    TIRED,
    HELLO,
    TAICHI,
    EYE_OF_TIGER,
    BEHAVIOURS_SIZE
};


// #define NONE 0
// #define STEPS 1

#define STAND 1
#define WALK 0

#define CMD_FINISHED -1
#define CMD_ERR -2

typedef struct
{
    float trans[3];
    float quat[4];    
} odom_t;


typedef struct
{ 
    bool isValid;
    odom_t odom;
    
    float copl[4];
    float copr[4];
    float acc[3];
    float gyro[3];
    float joint_states[26];
    
    int last_footstep;
    
    int last_cmd;
    unsigned int last_cmd_id;
    int last_cmd_param;
    int last_cmd_status;
} dcm_data_t;

typedef struct
{ 
    dcm_data_t dcm_data;    
    int stamp;
} dcm_data_stamp_t;

typedef struct
{
    float x;
    float y;
    float theta;
    char leg;
    char cmd;
} step_t;

typedef struct
{
//     int engine_id;
    int command;
    unsigned int id;
    int data_size;
    void *data;
}command_t;

#endif
