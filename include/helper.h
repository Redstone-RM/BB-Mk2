#pragma once
#include <Arduino.h>
#include <stdio.h> // LMX 
#include <task.h> // LMX
#include <log.h> // LMX
#include <sysclock.h> // LMX

#define VERSION "Bum Biter MK-2.2.0"

/*######################## <libtask LMX dpa> #########################################*/
#define BAUDRATE 57600  // LMX 
#define PRINTF Serial.println // LMX 
#define SPRINTF sprintf // LMX 
#define MACHINE MACH_AVR // this is currently set in task.h so this does nothing but the functionalty needs to be migrated.
#define WAIT(d)  { wake_after(d); }
#define BAUDRATE 57600
#define PRINTF Serial.println
#define SPRINTF sprintf


/* ------------ <dpa> -------------------------- */
/* Count idle cycles per second dpa original function */ 
unsigned long idle_cnt;
void cpu_idle(ASIZE ignored){
    unsigned long t;
    unsigned long cnt;
    
    t = sysclock + 1000;
    while (1) {
        idle_cnt = proc_counter;
        proc_counter = 0;
        // WAIT(1000);
        PERIOD(&t,1000);
    }
}

void stats_task(ASIZE delay) {
    TSIZE t;
    t = sysclock + delay;

    while (1) {
      // WAIT(delay);
      PERIOD(&t,delay);
      PRINTF("");
      SPRINTF(sbuf,"# Sysclock\t%ld\tSampleclock\t%ld\tIdleHz\t%ld",
              sysclock,sampleclock,idle_cnt);
      PRINTF(sbuf);
      PRINTF("");
      print_llist(1);
    }
}



/* 
Choose one of the below:  
#define WAIT(d)  { d *= 10; cnt = 0; while (cnt++ < d) defer(); }
#define WAIT(d)  { msleep(d); }
#define WAIT(d)  { wake_after(d); }
*/
#define WAIT(d)  { wake_after(d); } // LMX 



/* ----------- Functions --------------------------- */

bool randomBool() {
   return rand() > (RAND_MAX / 2);
}

float clip(float value, float min, float max ){ // dpa inspired cliping function
  if (value > max ) return max;
  if (value < min ) return min;
  return value;
}

/* ------------ <dpa> -------------------------- */
/* Create signon and terminate task */
void signon(ASIZE version)
{
  PRINTF(VERSION);
  wake_after(2000);
  PRINTF("# SIGNON Messages signing off!\n");
  DELAY(1000);
  terminate();
}

