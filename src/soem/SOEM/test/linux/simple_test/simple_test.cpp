// Inclusion of necessary packages
#include <sys/mman.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>
#include <pthread.h>
#include <math.h>
#include <stdlib.h>
#include <sched.h>
#include <time.h>
#include <signal.h>
#include <inttypes.h>
#include "ethercat.h"
#include <ethercattype.h>
#include <nicdrv.h>
#include <ethercatbase.h>
#include <ethercatmain.h> 
#include <ethercatdc.h>
#include <ethercatcoe.h>
#include <ethercatfoe.h>
#include <ethercatconfig.h>
#include <ethercatprint.h>
// ROS
#include "ros/ros.h"
#include <vector>
#include <iostream>
//~ #include "std_msgs/MultiArrayLayout.h"
//~ #include "std_msgs/MultiArrayDimension.h"
//~ #include "std_msgs/Int32MultiArray.h"
#include "std_msgs/String.h"
//~ #include "soem/MotorDataMsg.msg"
#include <tada_ros/MotorDataMsg.h>
#include <tada_ros/MotorListenMsg.h>

// Definition and Initialization of variables
#define EC_TIMEOUTMON 500 // 500 seems ideal
#define NSEC_PER_SEC 1000000000
#define stack64k (64 * 1024)
float time_increment; 
int timestep = 1000; //250, 500, 1000
clock_t start_t, end_t, start2, end2;
double total_t = 0;
int total_itr = 10000;
int dummy, dummy2 = 0;
int num;
int itr_fail = 0;
FILE *fptr;
pthread_mutex_t lock;
int move_to_final = 0;
int counts_per_rev = 567;
long long cur_DCtime=0, max_DCtime=0;
unsigned long long  cur_dc32=0, pre_dc32=0;
long long  diff_dc32;
//~ static int64 integral = 0;
int64 integral = 0;
int64 delta;
int motor_command_received[6];
//~ void motor_commandCallback(const std_msgs::Int32MultiArray::ConstPtr& motor_command);
void motor_commandCallback(const tada_ros::MotorDataMsg::ConstPtr& motor_command);
//~ void motor_listenCallback(const tada_ros::MotorListenMsg::ConstPtr& motor_listen);
int mode;
int duration;
float movement;
int move;
float movement2;
int move2;
int torque;
int torque2;
struct sched_param schedp;
char IOmap[4096]; 
pthread_t thread1, thread2, thread3, thread4;
int expectedWKC;
boolean needlf; 
volatile int wkc;
boolean inOP;
uint8 *digout = 0;
uint8 currentgroup = 0;
int32 pre_move = (int32)0;
int itr = 0;
int i, j = 0;
int print_state = 1;
int finished = 0;
int32 curr_pos = 0; 
int32 tar_pos = 0;
int final_pos = 0;
int initial_pos = 0;
int32 curr_pos2 = 0; 
int32 tar_pos2 = 0;
int final_pos2 = 0;
int initial_pos2 = 0;
int brute_force = 0;
int test_var = 0;
int test_var2 = 0;
int delay = 5000; 
int iret1, iret2;
struct timeval tv, t1, t2;
int dorun = 0;
int deltat, tmax = 0;
int64 toff, gl_delta;
int DCdiff;
int os, wkc_count;

struct TorqueOut {
    int32 position;
    //~ int32 dig_input;
    int32 velocity;
    uint16 torque;
    uint16 status;
};

struct TorqueIn {
    int32 position;
    //~ int32 dig_input;
    //~ int32 velocity;
    int16 torque; 
    uint16 status; 
    int8 profile;
};

/**
 * helper macros
 */
#define READ(slaveId, idx, sub, buf, comment)    \
    {   \
        buf=0;  \
        int __s = sizeof(buf);    \
        int __ret = ec_SDOread(slaveId, idx, sub, FALSE, &__s, &buf, EC_TIMEOUTRXM);   \
        printf("Slave: %d - Read at 0x%04x:%d => wkc: %d; data: 0x%.*x (%d)\t[%s]\n", slaveId, idx, sub, __ret, __s, (unsigned int)buf, (unsigned int)buf, comment);    \
     }

#define WRITE(slaveId, idx, sub, buf, value, comment) \
    {   \
        int __s = sizeof(buf);  \
        buf = value;    \
        int __ret = ec_SDOwrite(slaveId, idx, sub, FALSE, __s, &buf, EC_TIMEOUTRXM);  \
        printf("Slave: %d - Write at 0x%04x:%d => wkc: %d; data: 0x%.*x\t{%s}\n", slaveId, idx, sub, __ret, __s, (unsigned int)buf, comment);    \
    }

#define CHECKERROR(slaveId)   \
{   \
    ec_readstate();\
    printf("EC> \"%s\" %x - %x [%s] \n", (char*)ec_elist2string(), ec_slave[slaveId].state, ec_slave[slaveId].ALstatuscode, (char*)ec_ALstatuscode2string(ec_slave[slaveId].ALstatuscode));    \
}

// Function to initialize the CANopen on EtherCAT system and to send the movement commands to the motor driver
OSAL_THREAD_FUNC simpletest(int input[6])
{
    // Difinition of local variables
    mode = (int)input[0];
    duration = input[1];
    movement = (float)(input[2]);
    move = (int)(movement); ///360*585);
    movement2 = (float)(input[3]);
    move2 = (int)(movement2); ///360*585);
    torque = input[4];
    torque2 = input[5];
    test_var = move;
    test_var2 = move2;
    float total_time = (float)duration/1000;
    int oloop, iloop, wkc_count, chk;
    needlf = FALSE;
    inOP = FALSE;
    char ifname[] = "eth0";
    uint32 buf32;
    uint16 buf16;
    uint8 buf8;
    struct TorqueIn *val;
    struct TorqueOut *target;
    struct TorqueIn *val2;
    struct TorqueOut *target2;

    printf("Starting simple test\n");

    /* initialise SOEM, bind socket to ifname */
    if (ec_init(ifname))
    {
        /* find and auto-config slaves */
        /** network discovery */
        if ( ec_config_init(FALSE) > 0 )
        {
            for (int i=1; i<=ec_slavecount; i++) {
                //~ printf("Slave %d has CA? %s\n", i, ec_slave[i].CoEdetails & ECT_COEDET_SDOCA ? "true":"false" );
                /** CompleteAccess disabled for Elmo driver */
                //~ ec_slave[i].CoEdetails ^= ECT_COEDET_SDOCA;
            }        
            ec_statecheck(0, EC_STATE_PRE_OP,  EC_TIMEOUTSTATE);

            /** set PDO mapping */
            /** opMode: 8  => Position profile */
            for (int i=1; i<=ec_slavecount; i++) {
                uint16 op_mode = 8;
                int op_size = sizeof(op_mode); 
                ec_SDOwrite(i, 0x6060, 0x00, FALSE, op_size, &op_mode, EC_TIMEOUTRXM);   
            }

            int32 ob2;int os;
            for (int i=1; i<=ec_slavecount; i++) {                
               os=sizeof(ob2); ob2 = 0x16040001; //0x16060001;
               ec_SDOwrite(i, 0x1c12, 0, TRUE, os, &ob2, EC_TIMEOUTRXM);
               os=sizeof(ob2); ob2 = 0x1a020001; //0x1a040001;
                ec_SDOwrite(i, 0x1c13, 0, TRUE, os, &ob2, EC_TIMEOUTRXM);
            }
            
            /** if CA disable => automapping works */
            ec_config_map(&IOmap);         

            /* wait for all slaves to reach SAFE_OP state */
            ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE * 4);

            /* configure DC options for every DC capable slave found in the list */
            ec_configdc();           
            expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;

            /** going operational */
            ec_slave[0].state = EC_STATE_OPERATIONAL;
            /* send one valid process data to make outputs in slaves happy*/
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);
            
            /* request OP state for all slaves */
            ec_writestate(0);
            chk = 50;
                   
            /* wait for all slaves to reach OP state */
            do
            {
                ec_send_processdata();
                ec_receive_processdata(EC_TIMEOUTRET);
                ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);                 
            }
            
            while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

            // if master is in operational mode then activate the state machine
            if (ec_slave[0].state == EC_STATE_OPERATIONAL ) {
                wkc_count = 0;
                inOP = TRUE;

                /** Drive state machine transistions 0 -> 6 -> 7 -> 15 */
                for (int i=1; i<=ec_slavecount; i++) {
                    if(buf16 == 0x218)
                    {
                        WRITE(i, 0x6040, 0, buf16, 128, "*control word*"); usleep(delay);
                    }

                    WRITE(i, 0x6040, 0, buf16, 0, "*control word*"); usleep(delay);
                    WRITE(i, 0x6040, 0, buf16, 6, "*control word*"); usleep(delay);
                    WRITE(i, 0x6040, 0, buf16, 7, "*control word*"); usleep(delay);
                    WRITE(i, 0x6040, 0, buf16, 15, "*control word*"); usleep(delay);
                    CHECKERROR(i);
                }
                
        /* cyclic loop for two slaves*/
        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);  
        target = (struct TorqueOut *)(ec_slave[1].outputs);
        val = (struct TorqueIn *)(ec_slave[1].inputs);
        //~ ec_receive_processdata(EC_TIMEOUTRET);

        target2 = (struct TorqueOut *)(ec_slave[2].outputs);
        val2 = (struct TorqueIn *)(ec_slave[2].inputs);
    
        initial_pos = (val -> position);
        final_pos = test_var + initial_pos;
        initial_pos2 = (val2 -> position);
        final_pos2 = test_var2 + initial_pos2;
        
        // Start the sychronization process
        printf("Start\n"); 
        dorun = 1; 
        sleep(1);        
        start2 = clock();
        
        // continuous loop to send commands to the motor
        //~ while(1) {
        while(inOP){
            //~ if(wkc >= expectedWKC){
            itr = itr + 1;
            //~ end_t = clock();
            //~ total_t = (double)(end_t - start_t) / CLOCKS_PER_SEC;
            curr_pos = (val -> position);
            curr_pos2 = (val2 -> position);
            /** PDO I/O refresh */
		    //~ wkc = ec_receive_processdata(EC_TIMEOUTRET);
            target->velocity = (int32) (500);
            target2->velocity = (int32) (500);
            target->torque = (int16) torque;
            target2->torque = (int16) torque2;

            if(wkc >= expectedWKC){
                        /** if in fault or in the way to normal status, we update the state machine */
                        // slave 1
                        switch(target->status){
                        case 0:
                            target->status = 6; 
                            break;
                        case 6:
                            target->status = 7; 
                            break;
                        case 7:
                            target->status = 15; 
                            break;
                        case 128:
                            target->status = 0; 
                            break;
                        default:
                            if(val->status >> 3 & 0x01) {
                                READ(1, 0x1001, 0, buf8, "Error");
                                target->status = 128; 
                            }
                        }
                        /** if in fault or in the way to normal status, we update the state machine */
                        // slave 2
                        switch(target2->status){
                        case 0:
                            target2->status = 6; 
                            break;
                        case 6:
                            target2->status = 7; 
                            break;
                        case 7:
                            target2->status = 15;
                            break;
                        case 128:
                            target2->status = 0;
                            break;
                        default:
                            if(val2->status >> 3 & 0x01) {
                                READ(1, 0x1001, 0, buf8, "Error");
                                target2->status = 128; 
                            }
                        }
            
            if ((val->status & 0x0fff) == 0x0237) {
                
                if (mode == 1){
                    // motor not moving in this setting                    
                        target->position = (int32) (final_pos);
                        target2->position = (int32) (final_pos2);
                    }
                    //~ else {
                        //~ target->position = (int32) (initial_pos);
                        //~ target2->position = (int32) (initial_pos2);
                    //~ }
                else {
                    //~ if (itr <= total_itr) {
                        target->position = (int32) (final_pos);
                        target2->position = (int32) (final_pos2);
                    //~ }
                    }
                }
            else 
            {   
                printf("."); //("status error\n"); 
                itr_fail += 1;
            }
            needlf = TRUE;
                
                //~ if (abs(curr_pos - final_pos) == 0 && abs(curr_pos2 == final_pos2) == 0 && print_state == 1) {
                //if (move_to_final == 0){
                    //~ end2 = clock();
                    //~ double motor_move_time = (double) (end2-start2)/CLOCKS_PER_SEC;
                    //~ printf("\nMovement took %f sec\n",motor_move_time); 
                    //~ print_state = 0;
                //~ } 
            }
            else {                
                //~ end2 = clock();
                //~ double wkc_err_time = (double) (end2-start2)/CLOCKS_PER_SEC;
                //~ printf("\nwkc error after %f sec\n",wkc_err_time);
                printf("?"); //("wkc error\n");
                itr_fail += 1;
                //~ start2 = clock();
            }
            
            //~ printf("Itr %4d", itr); 
            //~ printf("  Initial_pos: %d,", initial_pos);
            //~ printf("  Actual_pos: %d,", val->position);  
            //~ printf("  Target_pos: %d,", final_pos);
            //~ printf(" move: %d", test_var);
            //~ printf("\r"); 
            
            osal_usleep(timestep);
            }
            
            dorun = 0;
            inOP = FALSE;

            }
        else
            {
                printf("Not all slaves reached operational state.\n");
                ec_readstate();
                for(i = 1; i<=ec_slavecount ; i++)
                {
                    if(ec_slave[i].state != EC_STATE_OPERATIONAL)
                    {
                        printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                            i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
                    }
                }
                printf("Request safe operational state for all slaves\n");
                ec_slave[0].state = EC_STATE_SAFE_OP;
                /* request SAFE_OP state for all slaves */
                ec_writestate(0);
            }

        printf("End simple test, close socket\n");
        ec_close();
    }
    else
    {
        printf("No socket connection on %s\nExcecute as root\n",ifname);
    }
    
    // Print when program ends
    printf("Itr %4d", itr); 
    printf("  Initial_pos: %d,", initial_pos);
    printf("  Actual_pos: %d,", val->position);  
    printf("  Target_pos: %d,", final_pos);
    printf(" move: %d", test_var);
    printf("\n"); 
    }
}

/* RT EtherCAT synchronization thread */
OSAL_THREAD_FUNC_RT ecatthread(void *ptr)
{
   struct timespec   ts, tleft;
   int ht;
   int64 cycletime;

   clock_gettime(CLOCK_MONOTONIC, &ts);
   ht = (ts.tv_nsec / 1000000) + 1; /* round to nearest ms */
   ts.tv_nsec = ht * 1000000;
   if (ts.tv_nsec >= NSEC_PER_SEC) {
      ts.tv_sec++;
      ts.tv_nsec -= NSEC_PER_SEC;
   }
   cycletime = *(int*)ptr * 1000; /* cycletime in ns */
   toff = 0;
   dorun = 0;
   //~ pthread_mutex_lock(&lock);
   ec_send_processdata();
   while(1)
   {
      /* calculate next cycle start */
        int64 sec, nsec;
        int64 addtime = cycletime + toff;

        nsec = addtime % NSEC_PER_SEC;
        sec = (addtime - nsec) / NSEC_PER_SEC;
        ts.tv_sec += sec;
        ts.tv_nsec += nsec;
        if ( ts.tv_nsec >= NSEC_PER_SEC )
        {
            nsec = ts.tv_nsec % NSEC_PER_SEC;
            ts.tv_sec += (ts.tv_nsec - nsec) / NSEC_PER_SEC;
            ts.tv_nsec = nsec;
        }
      /* wait to cycle start */
      clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, &tleft);
      if (dorun>0)
      {
        pthread_mutex_lock(&lock); 
         wkc = ec_receive_processdata(EC_TIMEOUTRET);
         dorun++;
         /* if we have some digital output, cycle */
         if( digout ) *digout = (uint8) ((dorun / 16) & 0xff);

         if (ec_slave[0].hasdc) 
         {
         
            /* calulate toff to get linux time and DC synced */
                cur_dc32= (uint32_t) (ec_DCtime & 0xffffffff); 	//use 32-bit only
                if (cur_dc32>pre_dc32)							//normal case
                    diff_dc32=cur_dc32-pre_dc32;
                else												//32-bit data overflow
                    diff_dc32=(0xffffffff-pre_dc32)+cur_dc32;
                pre_dc32=cur_dc32;
                cur_DCtime+=diff_dc32;
                // //~ if (cur_DCtime>max_DCtime) max_DCtime=cur_DCtime;
                
                /* set linux sync point 50 us later than DC sync, just as example I checked the raw ec_DCtime*/
                delta = (cur_DCtime - 50000) % cycletime; // 100000 didn't seem to help
                //~ printf("%ld, ",cur_DCtime);
                if(delta> (cycletime / 2)) { delta= delta - cycletime; }
                if(delta>0){ integral++; }
                if(delta<0){ integral--; }
                //~ integral = delta*cycletime + integral;
                
                // change P to 1, 10, 100 with I as 1000 and ctime as 50
                // change I to 100, 1000, 10000 with P as 10 and ctime as 50
                toff = -(delta / 10) - (integral / 1000); // 10, 1000 Adjusted these values till toff was not too large or did not drift
                //~ toff = -(delta / 10) - (integral / 100); //
                //~ printf("%ld \n",toff);
                gl_delta = delta;
         }
         
         //~ pthread_mutex_unlock(&lock);
         
         //~ pthread_mutex_lock(&lock); 
         ec_send_processdata();
         pthread_mutex_unlock(&lock); 
         
      }
   }
}

// Error checking and correcting thread
OSAL_THREAD_FUNC ecatcheck( void *ptr )
{
    int slave;
    while(1)
    {
        if( inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate))
        {
            if (needlf)
            {
               needlf = FALSE;
                printf("Error_needlf\n");                
            }
            /* one ore more slaves are not responding */
            ec_group[currentgroup].docheckstate = FALSE;
            ec_readstate();
            for (slave = 1; slave <= ec_slavecount; slave++)
            {
               if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
               {
                  ec_group[currentgroup].docheckstate = TRUE;
                  if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
                  {
                     printf("ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
                     ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                     ec_writestate(slave);
                  }
                  else if(ec_slave[slave].state == EC_STATE_SAFE_OP)
                  {
                    printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
                    ec_slave[slave].state = EC_STATE_OPERATIONAL;
                    ec_writestate(slave);
                  }
                  else if(ec_slave[slave].state > 0)
                  {
                     if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
                     {
                        ec_slave[slave].islost = FALSE;
                        printf("MESSAGE : slave %d reconfigured\n",slave);
                     }
                  }
                  else if(!ec_slave[slave].islost)
                  {
                     /* re-check state */
                     ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                     if (!ec_slave[slave].state)
                     {
                        ec_slave[slave].islost = TRUE;
                        printf("ERROR : slave %d lost\n",slave);
                     }
                  }
               }
               if (ec_slave[slave].islost)
               {
                  if(!ec_slave[slave].state)
                  {
                     if (ec_recover_slave(slave, EC_TIMEOUTMON))
                     {
                        ec_slave[slave].islost = FALSE;
                        printf("MESSAGE : slave %d recovered\n",slave);
                     } 
                  }
                  else
                  {
                     ec_slave[slave].islost = FALSE;
                     printf("MESSAGE : slave %d found\n",slave);
                  }
               }
            }
            if(!ec_group[currentgroup].docheckstate){
               printf("OK : all slaves resumed OPERATIONAL.\n");
               itr_fail = 0;
           }
        }
        osal_usleep(1000); //20000 usleep(250);
    }
}  

// Extra thread to handle ROS publishing and subscribing
void extra_function(clock_t start_time) {
    sleep(1);
    ros::NodeHandle m;
    ros::Subscriber sub_motor = m.subscribe("motor_command", 10, motor_commandCallback);
    ros::Publisher pub_motor = m.advertise<tada_ros::MotorListenMsg>("motor_listen", 10);
    ros::spinOnce();
    ros::Rate rate(100);
    dummy = 0; dummy2 = 0; 
    
    while(ros::ok()){
        if(itr!=0) {
            
            //Get current time
            ros::Time::init();
            ros::Time now = ros::Time::now();
            int lowtime = now.nsec/1000000;
            int hightime = now.sec%100000;
            float lower_final_time = (float) lowtime;
            float high_final_time = (float) hightime;
            float final_time = floorf(lower_final_time)/1000 + high_final_time;
            
            
            ros::Publisher pub_motor = m.advertise<tada_ros::MotorListenMsg>("motor_listen", 10);
            tada_ros::MotorListenMsg motor_listen;
            motor_listen.curr_pos1 = curr_pos;
            motor_listen.curr_pos2 = curr_pos2;
            
            // if the CoE errors are persistent, the connected brain node will restart the motor node
            if (itr_fail > 2000) motor_listen.motor_fail = 1;
            else motor_listen.motor_fail = 0;
            
            //~ float curr_dc_time = (float)cur_DCtime;
            //~ printf("%lld \n",cur_DCtime);
            motor_listen.toff = toff; //curr_dc_time; // toff
            motor_listen.t = final_time;
            
            pub_motor.publish(motor_listen);
                   
            ros::Subscriber sub_motor = m.subscribe("motor_command", 10, motor_commandCallback);
            mode = (int)motor_command_received[0];
            duration = (int)motor_command_received[1];
            dummy = (int)motor_command_received[2];
            dummy2 = (int)motor_command_received[3];
            torque = (int)motor_command_received[4];
            torque2 = (int)motor_command_received[5];
            
            final_pos = dummy;
            final_pos2 = dummy2; 
            start2 = clock();
            print_state = 0;
            ros::spinOnce();
            
            rate.sleep(); 
        }
    }
}

// Functions to process the custom messages and multiarray
void motor_commandCallback(const tada_ros::MotorDataMsg::ConstPtr& motor_command){
    motor_command_received[0] = motor_command->mode;
    motor_command_received[1] = motor_command->duration;
    motor_command_received[2] = motor_command->motor1_move;
    motor_command_received[3] = motor_command->motor2_move;
    motor_command_received[4] = motor_command->motor1_torque;
    motor_command_received[5] = motor_command->motor2_torque;
    // currently not collecting data for PF, EV, and timestamp
//~ void motor_commandCallback(const std_msgs::Int32MultiArray::ConstPtr& motor_command){
    // Code for MultiArray processing
    //~ int i = 0;
    //~ for(std::vector<int>::const_iterator it = motor_command->data.begin(); it != motor_command->data.end(); ++it) {
        //~ motor_command_received[i] = *it;
        //~ printf("%d ", *it);
        //~ i++;
    //~ }
    //~ printf("\n");
    return;
}

// main function of this program that specifies the the thread functions, priorities, real time use, and appropiate CPU utilization
int main(int argc, char **argv)
{

    
    printf("SOEM (Simple Open EtherCAT Master)\nSimple test\n");
    ros::init(argc, argv,"motor");
    
    //~ if (argc == 7)
    //~ {
        start_t = clock();
        dorun = 0;
        int ctime = 1000; //250, 500, 1000
        struct sched_param param;
        struct sched_param param1;
        int policy = SCHED_FIFO;
        //~ mlockall(MCL_CURRENT | MCL_FUTURE); // seems to cause CoE to fail when other programs are loading

        // specify loaction to save data to
        //~ int num; FILE *fptr;
        //~ fptr = fopen("/home/pi/TADA_new/timing_data.csv","w+");
      
        /* create RT thread */
        osal_thread_create_rt(&thread1, stack64k * 2, (void*) &ecatthread, (void*) &ctime);

        /* create thread to handle slave error handling in OP */
        osal_thread_create(&thread2, stack64k * 4, (void*) &ecatcheck, NULL); 
        
        int input[6] = {0, 0, 0, 0, 0, 0};
        input[0] = (int)strtol(argv[1], NULL, 10);
        input[1] = (int)strtol(argv[2], NULL, 10);
        input[2] = (int)strtol(argv[3], NULL, 10);
        input[3] = (int)strtol(argv[4], NULL, 10);
        input[4] = (int)strtol(argv[5], NULL, 10);
        input[5] = (int)strtol(argv[6], NULL, 10);
        
        osal_thread_create(&thread4, NULL, (void*) &extra_function, &start_t); 
        
        osal_thread_create(&thread3, NULL, (void *)&simpletest, (int *)input);

        /* Core-Iso */
        //Isolate core 3 from the CPU scheduler. Append " isolcpus=3" to the end of /boot/cmdline.txt (note the space).
        cpu_set_t CPU3;
        CPU_ZERO(&CPU3);
        CPU_SET(3, &CPU3);
        pthread_setaffinity_np(thread1, sizeof(CPU3), &CPU3);
        
        cpu_set_t CPU2;
        CPU_ZERO(&CPU2);
        CPU_SET(2, &CPU2);
        pthread_setaffinity_np(thread3, sizeof(CPU2), &CPU2);
        
        /* Scheduler */
        // need these schedulers as the programs fails as other programs take up resources
        int prio = sched_get_priority_max(SCHED_FIFO);
        memset(&param, 0, sizeof(param));
        param.sched_priority = prio; 
        pthread_setschedparam(thread3, policy, &param); 
        
        memset(&param1, 0, sizeof(param1));
        param1.sched_priority = prio; //99
        pthread_setschedparam(thread1, policy, &param1); 
        
        pthread_join(thread3, NULL);
        
    //~ }
    //~ else
    //~ {
        //~ printf("Usage: red_test 1 800 292 292 200 1000\n\n");
        //~ printf("Meaning\n1: Run Mode (0-nothing yet, 1-move, 2-move then return)\n");
        //~ printf("800: Duration (800 ms)\n");
        //~ printf("180: Motor1 movement (deg; 1 rev is 360)\n");
        //~ printf("360: Motor2 movement)\n");
        //~ // printf("292: Motor1 movement (cnts per rev; 1 rev is 585)\n");
        //~ // printf("292: Motor2 movement)\n");
        //~ printf("200: Motor1 torque (Around 200 should be min and 1500 max)\n");
        //~ printf("1500: Motor2 torque)\n\n");
    //~ }

    printf("End program\n");
    end_t = clock();
    total_t = (double)(end_t - start_t) / CLOCKS_PER_SEC;
    printf("Total time (s) taken by CPU: %f\n", total_t);
    //~ fclose(fptr);
    //~ ec_close(); 
    return (0);
}
