#define _GNU_SOURCE 
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
//# include <ros/ros.h>

#define EC_TIMEOUTMON 500 // 500 1000 seems ideal
#define NSEC_PER_SEC 1000000000
#define stack64k (64 * 1024)
float time_increment; 
int timestep = 500; //500 //larger numbers like 5000 and above seems too slow a frequency for movement to occur
clock_t start_t, end_t, start2, end2;
double total_t = 0;
int total_itr = 10000;
int dummy, dummy2 = 0;
int num;
FILE *fptr;
pthread_mutex_t lock;
int move_to_final = 0;

long long cur_DCtime=0, max_DCtime=0;
unsigned long long  cur_dc32=0, pre_dc32=0;
long long  diff_dc32;
static int64 integral = 0;
int64 delta;

// Ctrl-c
//~ int sig = 0;
//if (sig == SIGINT) break;

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

// kinematics
int brute_force = 0;

int test_var = 0;
int test_var2 = 0;
// delay time
int delay = 50; // 5000 too long, 1000 seem good, 500 seem too short


int iret1, iret2;
struct timeval tv, t1, t2;
int dorun = 0;
int deltat, tmax = 0;
int64 toff, gl_delta;
int DCdiff;
int os, wkc_count;

char IOmap[4096];
//~ pthread_t thread1;
int expectedWKC;
boolean needlf;
volatile int wkc;
boolean inOP;
//~ uint8 currentgroup = 0;

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
    uint16 status; //there is some confusion in this code for status and control;
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

OSAL_THREAD_FUNC simpletest(int input[6])
//~ void simpletest(int input[6])
//~ void simpletest(int mode, int duration, int move, int move2, int torque, int torque2)
{
    int mode = input[0];
    int duration = input[1];
    float movement = (float)(input[2]);
    int move = (int)(movement/360*585);
    float movement2 = (float)(input[3]);
    int move2 = (int)(movement2/360*585);
    int torque = input[4];
    int torque2 = input[5];
    
    //~ int mode = 1;
    //~ int duration = 700;
    //~ int move = 242;
    //~ int move2 = 242;
    //~ int torque = 1000;
    //~ int torque2 = 1000;
    
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
        //~ printf("ec_init on %s succeeded.\n",ifname);
        /* find and auto-config slaves */

        /** network discovery */
        if ( ec_config_init(FALSE) > 0 )
        {
            //~ printf("%d slaves found and configured.\n", ec_slavecount);

            for (int i=1; i<=ec_slavecount; i++) {
                //~ printf("Slave %d has CA? %s\n", i, ec_slave[i].CoEdetails & ECT_COEDET_SDOCA ? "true":"false" );

                /** CompleteAccess disabled for Elmo driver */
                //~ ec_slave[i].CoEdetails ^= ECT_COEDET_SDOCA;
            }
            
            ec_statecheck(0, EC_STATE_PRE_OP,  EC_TIMEOUTSTATE);
            //~ dorun = 1;

            /** set PDO mapping */
            /** opMode: 8  => Position profile */
            for (int i=1; i<=ec_slavecount; i++) {
                uint16 op_mode = 8;
                int op_size = sizeof(op_mode); 
                ec_SDOwrite(i, 0x6060, 0x00, FALSE, op_size, &op_mode, EC_TIMEOUTRXM);   
                //~ READ(i, 0x6061, 0, buf8, "OpMode display");

                //~ READ(i, 0x1c12, 0, buf32, "rxPDO:0");
                //~ READ(i, 0x1c13, 0, buf32, "txPDO:0");

                //~ READ(i, 0x1c12, 1, buf32, "rxPDO:1");
                //~ READ(i, 0x1c13, 1, buf32, "txPDO:1");
            }

            int32 ob2;int os;
            for (int i=1; i<=ec_slavecount; i++) {                
               os=sizeof(ob2); ob2 = 0x16040001; //0x16060001;
               ec_SDOwrite(i, 0x1c12, 0, TRUE, os, &ob2, EC_TIMEOUTRXM);
               os=sizeof(ob2); ob2 = 0x1a020001; //0x1a040001;
                ec_SDOwrite(i, 0x1c13, 0, TRUE, os, &ob2, EC_TIMEOUTRXM);
                
                //~ READ(i, 0x1c12, 0, buf32, "rxPDO:0");
                //~ READ(i, 0x1c13, 0, buf32, "txPDO:0");

                //~ READ(i, 0x1c12, 1, buf32, "rxPDO:1");
                //~ READ(i, 0x1c13, 1, buf32, "txPDO:1");
            }
            
            /** if CA disable => automapping works */
            ec_config_map(&IOmap);

            // show slave info
            //~ for (int i=1; i<=ec_slavecount; i++) {
                //~ printf("\nSlave:%d\n Name:%s\n Output size: %dbits\n Input size: %dbits\n State: %d\n Delay: %d[ns]\n Has DC: %d\n",
                //~ i, ec_slave[i].name, ec_slave[i].Obits, ec_slave[i].Ibits,
                //~ ec_slave[i].state, ec_slave[i].pdelay, ec_slave[i].hasdc);
            //~ }

            /** disable heartbeat alarm */
            for (int i=1; i<=ec_slavecount; i++) {
                //~ READ(i, 0x10F1, 2, buf32, "Heartbeat?");
                //~ WRITE(i, 0x10F1, 2, buf32, 1, "Heartbeat");

                //~ WRITE(i, 0x60c2, 1, buf8, 2, "Time period");
                //~ WRITE(i, 0x2f75, 0, buf16, 2, "Interpolation timeout");
                //~ ec_dcsync0(i, TRUE, timestep, 0);
            }           

            //~ printf("Slaves mapped, state to SAFE_OP.\n");

            /* wait for all slaves to reach SAFE_OP state */
            ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE * 4);

            /** old SOEM code, inactive */
            //~ oloop = ec_slave[0].Obytes;
            //~ if ((oloop == 0) && (ec_slave[0].Obits > 0)) oloop = 1;
            //~ if (oloop > 20) oloop = 8;
            //~ iloop = ec_slave[0].Ibytes;
            //~ if ((iloop == 0) && (ec_slave[0].Ibits > 0)) iloop = 1;
            //~ if (iloop > 20) iloop = 8;

            /* configure DC options for every DC capable slave found in the list */
            ec_configdc();
            //~ dorun = 1;
            //~ sleep(1);
            

            //~ printf("segments : %d : %d %d %d %d\n",ec_group[0].nsegments ,ec_group[0].IOsegment[0],ec_group[0].IOsegment[1],ec_group[0].IOsegment[2],ec_group[0].IOsegment[3]);

            //~ printf("Request operational state for all slaves\n");
            expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
            //~ printf("Calculated workcounter %d\n", expectedWKC);

            //~ dorun = 1;
            /** going operational */
            ec_slave[0].state = EC_STATE_OPERATIONAL;
            /* send one valid process data to make outputs in slaves happy*/
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);

            //~ for (int i=1; i<=ec_slavecount; i++) {
                //~ READ(i, 0x6083, 0, buf32, "Profile acceleration");
                //~ READ(i, 0x6084, 0, buf32, "Profile deceleration");
                //~ READ(i, 0x6085, 0, buf32, "Quick stop deceleration");
                //~ ec_dcsync0(i, TRUE, timestep, 0);
            //~ }
            
            /* request OP state for all slaves */
            ec_writestate(0);
            chk = 100;
                   
            /* wait for all slaves to reach OP state */
            do
            {
                ec_send_processdata();
                ec_receive_processdata(EC_TIMEOUTRET);
                ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);                 
            }
            
            while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

            if (ec_slave[0].state == EC_STATE_OPERATIONAL ) {
                //~ printf("Operational state reached for all slaves.\n");
                wkc_count = 0;
                inOP = TRUE;

                /**
                 * Drive state machine transistions
                 *   0 -> 6 -> 7 -> 15
                 */
                for (int i=1; i<=ec_slavecount; i++) {
                    //~ READ(i, 0x6041, 0, buf16, "*status word*");
                    if(buf16 == 0x218)
                    {
                        WRITE(i, 0x6040, 0, buf16, 128, "*control word*"); usleep(delay);
                        //~ READ(i, 0x6041, 0, buf16, "*status word*");
                    }


                    WRITE(i, 0x6040, 0, buf16, 0, "*control word*"); usleep(delay);
                    //~ READ(i, 0x6041, 0, buf16, "*status word*");

                    WRITE(i, 0x6040, 0, buf16, 6, "*control word*"); usleep(delay);
                    //~ READ(i, 0x6041, 0, buf16, "*status word*");

                    WRITE(i, 0x6040, 0, buf16, 7, "*control word*"); usleep(delay);
                    //~ READ(i, 0x6041, 0, buf16, "*status word*");

                    WRITE(i, 0x6040, 0, buf16, 15, "*control word*"); usleep(delay);
                    //~ READ(i, 0x6041, 0, buf16, "*status word*");

                    CHECKERROR(i);
                    //~ READ(i, 0x1a0b, 0, buf8, "OpMode Display");

                    //~ READ(i, 0x1001, 0, buf8, "Error");
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
        printf("Start\n"); 
        
        //~ for(itr = 1; itr < 100; itr++) {
            //~ osal_usleep(timestep);
        //~ }
        dorun = 1; 
        sleep(1);
        //~ for (int i=1; i<=ec_slavecount; i++) {
            //~ ec_dcsync01(1, FALSE, 0, 0, 0);
        //~ }
        
        start2 = clock();
       while(1) {
        //~ for(itr = 1; itr < total_itr; itr++) {
            itr = itr + 1;
            //~ end_t = clock();
            //~ total_t = (double)(end_t - start_t) / CLOCKS_PER_SEC;
            curr_pos = (val -> position);
            curr_pos2 = (val2 -> position);
            /** PDO I/O refresh */
            //~ ec_send_processdata();
		    //~ wkc = ec_receive_processdata(EC_TIMEOUTRET);
            //~ ec_send_processdata();
            target->velocity = (int32) (500);
            target2->velocity = (int32) (500);
            target->torque = (int16) torque;
            target2->torque = (int16) torque2;

                    //~ if(wkc >= expectedWKC) {

                        /** if in fault or in the way to normal status, we update the state machine */
                        // slave 1
                        switch(target->status){
                        case 0:
                            target->status = 6; //usleep(delay);
                            break;
                        case 6:
                            target->status = 7; //usleep(delay);
                            break;
                        case 7:
                            target->status = 15; //usleep(delay);
                            break;
                        case 128:
                            target->status = 0; //usleep(delay);
                            break;
                        default:
                            if(val->status >> 3 & 0x01) {
                                READ(1, 0x1001, 0, buf8, "Error");
                                target->status = 128; //usleep(delay);
                            }
                        }
                        /** if in fault or in the way to normal status, we update the state machine */
                        // slave 2
                        switch(target2->status){
                        case 0:
                            target2->status = 6; //usleep(delay);
                            break;
                        case 6:
                            target2->status = 7; //usleep(delay);
                            break;
                        case 7:
                            target2->status = 15; //usleep(delay);
                            break;
                        case 128:
                            target2->status = 0; //usleep(delay);
                            break;
                        default:
                            if(val2->status >> 3 & 0x01) {
                                READ(1, 0x1001, 0, buf8, "Error");
                                target2->status = 128; //usleep(delay);
                            }
                        }
                    //~ }
            
            if (mode == 1){// && (val->status & 0x0fff) == 0x0237) {
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
            
            //~ printf("%d,", itr); 
            //~ printf("  %d,", val->position);
            //printf("  %d,", val->velocity);  
            //~ printf("  %d,", val->torque);
            //~ printf("\n");
            

                        needlf = TRUE;
                    //~ }                            
                    //~ osal_usleep(timestep); 
            
                        //~ break;
                    //~ }
                    
                     //~ dorun = 0;
                //~ }
            
            //~ if (abs(curr_pos - final_pos) == 0 && abs(curr_pos2 - final_pos2) == 0) move_to_final = 0;
            //~ else move_to_final = 1;
                
                if (abs(curr_pos - final_pos) == 0 && abs(curr_pos2 == final_pos2) == 0 && print_state == 1) {
                //~ if (move_to_final == 0){
                    end2 = clock();
                    double motor_move_time = (double) (end2-start2)/CLOCKS_PER_SEC;
                    printf("\nMovement took %f sec\n",motor_move_time); 
                    print_state = 0;
                    //~ move_to_final = 1; 
                    //~ initial_pos = curr_pos;
                    //~ initial_pos2 = curr_pos2;
                    //~ // break;
                } 
                
                osal_usleep(timestep);
                fprintf(fptr, "%lld\n ", toff);
                
                //~ if (sig == SIGINT) {
                    //~ pthread_kill(&thread4,sig);
                    //~ pthread_kill(&thread2,sig);
                    //~ pthread_kill(&thread1,sig);
                    //~ //pthread_kill(&thread3,sig);
                    //~ //exit(1);
                    //~ break;
                //~ }
                //~ printf("%lld, ", toff);
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

        //~ else
        //~ {
            //~ printf("No slaves found!\n");
        //~ }
        // Seems to have lots of problems when I close the socket
        printf("End simple test, close socket\n");
        //~ /* stop SOEM, did not close socket */
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

// /* add ns to timespec */
// void add_timespec(struct timespec *ts, int64 addtime)
// {
//    int64 sec, nsec;

//    nsec = addtime % NSEC_PER_SEC;
//    sec = (addtime - nsec) / NSEC_PER_SEC;
//    ts->tv_sec += sec;
//    ts->tv_nsec += nsec;
//    if ( ts->tv_nsec >= NSEC_PER_SEC )
//    {
//       nsec = ts->tv_nsec % NSEC_PER_SEC;
//       ts->tv_sec += (ts->tv_nsec - nsec) / NSEC_PER_SEC;
//       ts->tv_nsec = nsec;
//    }
// }

/* PI calculation to get linux time synced to DC time */
// void ec_sync(int64 reftime, int64 cycletime , int64 *offsettime)
// {
//    static int64 integral = 0;
//    int64 delta;
//    cur_dc32= (uint32_t) (ec_DCtime & 0xffffffff); 	//use 32-bit only
//    if (cur_dc32>pre_dc32)							//normal case
//        diff_dc32=cur_dc32-pre_dc32;
//    else												//32-bit data overflow
//        diff_dc32=(0xffffffff-pre_dc32)+cur_dc32;
//    pre_dc32=cur_dc32;
//    cur_DCtime+=diff_dc32;
//    //~ if (cur_DCtime>max_DCtime) max_DCtime=cur_DCtime;
   
//    /* set linux sync point 540us later than DC sync, just as example */
//    delta = (cur_DCtime - 1664) % cycletime;
//    //~ printf("%lu ",cur_DCtime);
//    if(delta> (cycletime / 2)) { delta= delta - cycletime; }
//    if(delta>0){ integral++; }
//    if(delta<0){ integral--; }
//    *offsettime = -(delta / 2500) - (integral / 50); // 100 and 20
//    gl_delta = delta;
// }

/* RT EtherCAT thread */
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
    //   add_timespec(&ts, cycletime + toff);
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
        //// add_timespec
      /* wait to cycle start */
      clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, &tleft);
      if (dorun>0)
      {
        pthread_mutex_lock(&lock); 
        //~ osal_usleep(10);
         wkc = ec_receive_processdata(EC_TIMEOUTRET);
        //~ pthread_mutex_lock(&lock); 
         dorun++;
         /* if we have some digital output, cycle */
         if( digout ) *digout = (uint8) ((dorun / 16) & 0xff);

         if (ec_slave[0].hasdc)
         {
            /* calulate toff to get linux time and DC synced */
            // ec_sync(ec_DCtime, cycletime, &toff);
               //~ static int64 integral = 0;
                //~ int64 delta;
                cur_dc32= (uint32_t) (ec_DCtime & 0xffffffff); 	//use 32-bit only
                if (cur_dc32>pre_dc32)							//normal case
                    diff_dc32=cur_dc32-pre_dc32;
                else												//32-bit data overflow
                    diff_dc32=(0xffffffff-pre_dc32)+cur_dc32;
                pre_dc32=cur_dc32;
                cur_DCtime+=diff_dc32;
                //~ if (cur_DCtime>max_DCtime) max_DCtime=cur_DCtime;
                
                /* set linux sync point 1664us later than DC sync, just as example I checked the raw ec_DCtime*/
                delta = (cur_DCtime - 1664) % cycletime;
                //~ printf("%lu ",cur_DCtime);
                if(delta> (cycletime / 2)) { delta= delta - cycletime; }
                if(delta>0){ integral++; }
                if(delta<0){ integral--; }
                toff = -(delta / 10) - (integral / 1000); // 2500 and 200; Adjusted this value until toff was between 30 and -30
                gl_delta = delta;
                //// ec_sync
            //~ printf(" %d, \n",toff);
         }
         pthread_mutex_unlock(&lock);
         ec_send_processdata();
         //~ pthread_mutex_unlock(&lock);
      }
   }
   //~ pthread_mutex_unlock(&lock);
}


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
            if(!ec_group[currentgroup].docheckstate)
               printf("OK : all slaves resumed OPERATIONAL.\n");
        }
        osal_usleep(20000); //usleep(250);
        //if (sig == SIGINT) exit(1);
    }
}  

void extra_function(clock_t start_time) {
    sleep(3);
    
    while(1){
        //~ int dummy1 = 0;
    if(itr!=0) {// && itr<total_itr) {

        //~ sleep(1);
        printf("Itr %4d", itr); 
        printf("  Initial_pos: %d,", initial_pos);
        printf("  Actual_pos: %d,", curr_pos);   
        printf("  Target_pos: %d,", final_pos);
        printf(" move: %d", dummy);
        printf("\n");   
        
        sleep(1);
        dummy = 0; dummy2 = 0; 
        printf("\nEnter increment number to move:");	
		scanf(" %d, %d", &dummy, &dummy2); 
        //~ sleep(1); 
        //~ dummy = 180; dummy1 = 180;  
        float dummy_deg = (float)(dummy)/360*585;
        float dummy_deg2 = (float)(dummy2)/360*585;
        dummy = (int)(dummy_deg);
        dummy2 = (int)(dummy_deg2);
        final_pos = dummy + curr_pos;
        final_pos2 = dummy2 + curr_pos2;
        start2 = clock();
        print_state = 1;
        //~ sleep(10); 
    }
    //~ if(dummy1==1 && itr!=0) {
        //~ sleep(5);
        //~ final_pos = curr_pos + 180;
        //~ final_pos2 = curr_pos2 + 180;
        //~ start2 = clock();
        //~ print_state = 1;
        //~ sleep(3);
    //~ }
    }
    //if (sig == SIGINT) exit(1);
}


int main(int argc, char *argv[])
{

    
    printf("SOEM (Simple Open EtherCAT Master)\nSimple test\n");
    //ros_init();
    //~ if (argc == 7)
    //~ {
        start_t = clock();
        dorun = 0;
        int ctime = 50; //100 works better than 500
        struct sched_param param;
        struct sched_param param1;
        int policy = SCHED_FIFO;
        //~ mlockall(MCL_CURRENT | MCL_FUTURE); // seems to cause CoE to fail when other programs are loading

        // use appropriate location if you are using MacOS or Linux
        //~ int num;
        //~ FILE *fptr;
        fptr = fopen("/home/pi/TADA_new/timing_data.csv","w+");
        
        //~ memset(&schedp, 0, sizeof(schedp));
        /* do not set priority above 49, otherwise sockets are starved */
        //~ schedp.sched_priority = 30;
        //~ sched_setscheduler(0, SCHED_FIFO, &schedp);
      
        /* create RT thread */
        //~ dorun = 1;
        osal_thread_create_rt(&thread1, stack64k * 2, &ecatthread, (void*) &ctime);

        /* create thread to handle slave error handling in OP */
        osal_thread_create(&thread2, stack64k * 4, &ecatcheck, NULL); 
        
        int input[6] = {0, 0, 0, 0, 0, 0};

        input[0] = (int)strtol(argv[1], NULL, 10);
        input[1] = (int)strtol(argv[2], NULL, 10);
        input[2] = (int)strtol(argv[3], NULL, 10);
        input[3] = (int)strtol(argv[4], NULL, 10);
        input[4] = (int)strtol(argv[5], NULL, 10);
        input[5] = (int)strtol(argv[6], NULL, 10);
        
        
        osal_thread_create(&thread4, NULL, &extra_function, &start_t); 
        //~ extra_function(start_t);
        
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
        int prio = sched_get_priority_max(SCHED_FIFO);
        // need these schedulers as the programs fails as other programs take up resources
        memset(&param, 0, sizeof(param));
        param.sched_priority = prio; 
        pthread_setschedparam(thread3, policy, &param); 
        
        memset(&param1, 0, sizeof(param1));
        param1.sched_priority = prio; //99
        pthread_setschedparam(thread1, policy, &param1); 
        
        //~ mlockall(MCL_CURRENT | MCL_FUTURE);     
        
        //~ pthread_join(thread1, NULL);
        pthread_join(thread3, NULL);
        //~ osal_thread_create(&thread3, stack64k * 4, &simpletest, NULL);
        //~ extra_function(start_t);
        //~ simpletest((int *)input);
        //~ simpletest(mode, duration, move, move2, torque, torque2);
        
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
    fclose(fptr);
    //~ ec_close(); 
    return (0);
}
