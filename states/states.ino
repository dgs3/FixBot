#include <Servo.h>

//Define pin for luminosity
#define TSL_FREQ_PIN 2

unsigned long pulse_cnt = 0;


// need to measure what to divide freq by
// 1x sensitivity = 10,
// 10x sens       = 100,
// 100x sens      = 1000

int calc_sensitivity = 10;

// set our frequency multiplier to a default of 1
// which maps to output frequency scaling of 100x

int freq_mult = 100;


// track time
unsigned long cur_tm = millis();
unsigned long pre_tm = cur_tm;

// Time passed
unsigned int tm_diff = 0;

int light_type = 0;

// our wavelengths (nm) we're willing to calculate illuminance for (lambda)
int wavelengths[18] = { 
  380, 400, 420, 440, 460, 480, 500, 520, 540, 560, 580, 600, 620, 640, 660, 680, 700, 720 };
// the CIE V(l) for photopic vision - CIE Vm(l) 1978 - mapping to the same (l) above
float v_lambda[18]  = { 
  0.0002, 0.0028, 0.0175, 0.0379, 0.06, 0.13902, 0.323, 0.71, 0.954, 0.995, 0.87, 0.631, 0.381, 0.175, 0.061, 0.017, 0.004102, 0.001047 };
// CIE SPD graphs for D65 and Illuminant A light sources, again mapping to same lambda as included in wavelengths
float spd_graphs[2][18] = {
  { 
    49.975500, 82.754900, 93.431800, 104.865000, 117.812000, 115.923000, 109.354000, 104.790000, 104.405000, 100.000000, 95.788000, 90.006200, 87.698700, 83.699200, 80.214600, 78.284200, 71.609100, 61.604000   }
  ,
  { 
    9.795100, 14.708000, 20.995000, 28.702700, 37.812100, 48.242300, 59.861100, 72.495900, 85.947000, 100.000000, 114.436000, 129.043000, 143.618000, 157.979000, 171.963000, 185.429000, 198.261000, 210.365000   }
};


//Define joints, states, clamp values for servos
int jointPos[] = {
  0, 0, 0, 0};
Servo joint_0, joint_1, joint_2, joint_3;
Servo joints[] = {
  joint_0, joint_1, joint_2, joint_3};
int clamp[5][2] = {
  {
    0, 255  }
  ,
  {
    0, 255  }
  ,
  {
    0, 255  }
  ,
  {
    100, 240  }
  ,
};
int numServos = 4;

int stop_time = 1000;

//States the bot can enter
enum States
{
  fidget,
  sleep,
  stoic,
  inquisitive,
  sleepy,
  excited,
};

States stateList[] = {
  fidget, sleep, stoic, inquisitive, sleepy, excited};

States state;

long maxLux = 20000;



void setup(void)
{
  Serial.begin(115200);
  // Attach interrupt to pin2, send output pin of sensor to arduino 2
  // call handler on each rising pulse
  pinMode(TSL_FREQ_PIN, INPUT);

  for(int i = 0; i < numServos; ++i)
  {
    joints[i].attach(13-i);
  }
  randomSeed(analogRead(0));
  extend();
}

void loop(void)
{
  long lux = sample_and_calc_lux(1000);
  Serial.println(lux);
  if( lux < maxLux )
  {
    slowRetract(); 
  }
  else
  {
    stateTransition();
    doActivity();
  }
}

long sample_and_calc_lux(int miliseconds)
{
  attachInterrupt(0, add_pulse, RISING);
  // Check light sensor every READ_TM ms
  // Calc time passed
  while( tm_diff < miliseconds)
  {
    pre_tm = cur_tm;
    cur_tm = millis();

    if (cur_tm > pre_tm )
    {
      tm_diff += cur_tm - pre_tm; 
    }
    else if( cur_tm < pre_tm )
    {
      // handle overflow
      tm_diff += (cur_tm + ( 34359737 - pre_tm )); 
    }
  }

  // reset counter
  tm_diff = 0;
  // Get reading
  unsigned long frequency = get_tsl_freq(); 
  float uw_cm2 = calc_uwatt_cm2( frequency );
  float lux = calc_lux_gauss( uw_cm2 );
  detachInterrupt(0);
  return lux;
}

float calc_lux_gauss( float uw_cm2 )
{
  int nm_cnt = sizeof(wavelengths) / sizeof(int);
  // watts/m2
  float w_m2 = ( uw_cm2 * ( (float) 1 / (float) 0.0136) / (float) 1000000 ) * (float) 100; 
  float result = 0;
  // integrate XIV(1)
  for( int i = 0; i < nm_cnt; ++i)
  {
    if (i > 0)
    {
      result += (spd_graphs[light_type][i] / (float) 1000000) * (wavelengths[i] - wavelengths[i-1]) * w_m2 * v_lambda[i];
    }
    else
    {
      result += (spd_graphs[light_type][i] / (float) 1000000) * wavelengths[i] * w_m2 * v_lambda[i];

    }
  }
  return result * (float) 683; 
}

void add_pulse(void)
{
  pulse_cnt++;
  return; 
}

unsigned long get_tsl_freq() {

  // we have to scale out the frequency --
  // Scaling on the TSL230R requires us to multiply by a factor
  // to get actual frequency

  unsigned long freq = pulse_cnt * freq_mult;

  // reset the pulse counter

  pulse_cnt = 0;

  return freq;
}


float calc_uwatt_cm2(unsigned long freq) {

  // get uW observed - assume 640nm wavelength
  // calc_sensitivity is our divide-by to map to a given signal strength
  // for a given sensitivity (each level of greater sensitivity reduces the signal
  // (uW) by a factor of 10)

  float uw_cm2 = (float) freq / (float) calc_sensitivity;

  // extrapolate into entire cm2 area

  uw_cm2       *= ( (float) 1 / (float) 0.0136 );

  return(uw_cm2);

}


void stateTransition(void)
{
  int randState = random(0, sizeof(stateList));
  state = stateList[randState];
}

void doActivity(void)
{
  switch(state)
  {
  case fidget:
    Serial.println("Transition: Fidget");
    doFidget();
    break;
  case sleep:
    Serial.println("Transition: Sleep");
    doSleep();
    break;
  case stoic:
    Serial.println("Transition: Stoic");
    doStoic();
    break;
  case inquisitive:
    Serial.println("Transition: Inquisitive");
    doInquisitive();
    break;
  case sleepy:
    Serial.println("Transition: Sleepy");
    doSleepy();
    break;
  case excited:
    Serial.println("Transition: Excited");
    doExcited();
    break;
  }
}

void doStoic(void)
{
  Serial.println("Do Stoic");
  delay(10000); 
}

void doSleep(void)
{
  extend();
  Serial.println("Do Sleep");
  slowRetract();
  delay(500000); 
}

void doSleepy(void)
{
  Serial.println("Do Sleepy");
  extend();
  int joint_2_max = 255;
  int joint_2_min = 110;
  for(int i = 0; i < 4; ++i)
  {
    for(int j = joint_2_max; j > joint_2_min; --j)
    {
      jointWriteOut(2, j);
      delay(75);
    }
    delay(5000);
    jointWriteOut(2, joint_2_max);
  } 
}

void doExcited(void)
{
  Serial.println("Do Excited");
  extend();
  int joint_2_min = 110;
  int joint_2_max = 255;
  int theDelay = 2;
  for(int i = 0; i < 15; ++i)
  {
    for(int j = joint_2_max; j > joint_2_min; --j)
    {
      jointWriteOut(2, j);
      delay(theDelay);
    }
    for(int j = joint_2_min; j < joint_2_max; ++j)
    {
      jointWriteOut(2, j);
      delay(theDelay);

    }
  } 
}


void doFidget(void)
{
  Serial.println("Do Fidget");
  extend();
  for(int i = 0; i < 50; ++i)
  {
    randomMovement();
    delay(1500);
  } 
}

void doInquisitive(void)
{
  Serial.println("Do Inquisitive");
  extend();
  cockHead();

  //Sweep Tongue
  int tongueMax = 125;
  int tongueMin = 75;
  for(int j = 0; j < 2; ++j)
  {

    for(int i = tongueMin; i < tongueMax; ++i)
    {
      jointWriteOut(0, i);
      delay(100);
    }
    for(int i = tongueMax; i > tongueMin; --i)
    {
      jointWriteOut(0, i);
      delay(100); 
    }
  }

  //Extend Fully
  for(int i = jointPos[2]; i > 110; --i)
  {
    jointWriteOut(2, i);
    delay(30);
  }
  cockHead();
}

void cockHead(void)
{
  //Turn Head
  centerJoint1();
  jointWriteOut(0, 100);
  delay(1000);
  int headTurn = 100;
  for(int i = 0; i < headTurn; ++i)
  {
    jointWriteOut(1, i);
    delay(100); 
  }
}


void randomMovement(void)
{
  int joint = random(0, numServos);
  jointWriteOut(joint, jointPos[joint]+random(-8, 9)); 
}

void randomWiggle(void)
{
  if(random(2) == 0)
  {
    quickWiggle();
  } 
  else
  {
    slowWiggle(); 
  }
}

void quickWiggle(void)
{
  for(int i = 0; i < random(15); ++i)
  {
    fingerWiggle(250);
  }
}

void slowWiggle(void)
{
  for(int i = 0; i < random(7); ++i)
  {
    fingerWiggle(1500);
  } 
}

void fingerWiggle(int time)
{
  jointWriteOut(0, 50);
  delay(time);
  jointWriteOut(0, 150);
  delay(time);
}

void extend(void)
{
  jointWriteOut(0, 100);
  jointWriteOut(1, 100);
  jointWriteOut(2, 150);
  jointWriteOut(3, 100);
}

void retract(void)
{
  jointWriteOut(0, 100);
  jointWriteOut(1, 100);
  jointWriteOut(2, 0);
  jointWriteOut(3, 240);
}

void slowRetract(void)
{
  int theDelay = 150;
  for(int i = 0; i < 2; ++i)
  {
    jointWriteOut(i, 100);
  }
  for(int i = 255; i > 0; --i)
  {
    jointWriteOut(2, jointPos[2] - 1);
    jointWriteOut(3, jointPos[3] + 1);
    delay(theDelay);
  }
}

void straightUp(void)
{
  jointWriteOut(2, 100);
  jointWriteOut(1, 200);
}

void centerJoint1(void)
{
  jointWriteOut(1, 100);
}

void jointWriteOut(int joint_num, int value)
{
  value = clampValue(joint_num, value);
  jointPos[joint_num] = value;
  joints[joint_num].write(value);
}

int clampValue(int joint_num, int value)
{
  if(value < clamp[joint_num][0])
  {
    return clamp[joint_num][0];
  }
  else if(value > clamp[joint_num][1])
  {
    return clamp[joint_num][1]; 
  }
  else{
    return value;
  }
}

