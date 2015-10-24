/*****************************************************************************
 * 
 * At the lowest point the top of the aluminum frame should be at 20.25 inches from the floor. 
 * 
 * Total travel of hydraulics is 12.00 inches
 * 
 *****************************************************************************/
 
const int BYPASS_SAFETY_PIN  = 7;
const int MOMENTARY_UP_PIN   = 6;
const int MOMENTARY_DOWN_PIN = 5;
const int SWITCH_GND_PIN     = 4;

const int MOTOR_PULSE_SIG_PIN     = 12;
const int MOTOR_DIRECTION_GND_PIN = 11;
const int MOTOR_DIRECTION_SIG_PIN = 10;
const int MOTOR_PULSE_GND_PIN     = 9;

const int APEX_UP_PIN   = A5;
const int APEX_DOWN_PIN = A4;
const int APEX_GND_PIN  = A2;

//-----------------------------------------------------------------------------
unsigned long usElapsedSince( unsigned long us )
{ 
  unsigned long now = micros();
  
  if( now < 0x7FFFFFFF && us > 0x7FFFFFFF )
  {
    // We've wrapped! Correct
    unsigned long elapsed = now + (0xFFFFFFFF - us);
    return elapsed;
  }
  
  return now - us;
}

//-----------------------------------------------------------------------------
unsigned long msElapsedSince( unsigned long ms )
{
  unsigned long now = millis();
  
  if( now < 0x7FFFFFFF && ms > 0x7FFFFFFF )
  {
    // We've wrapped! Correct
    unsigned long elapsed = now + (0xFFFFFFFF - ms);
    return elapsed;
  }
  
  return now - ms;
}

//-----------------------------------------------------------------------------
unsigned long usPerStep( unsigned long stepsPerSecond )
{
  if( stepsPerSecond == 0 )
    return 0xFFFFFFFF;
  
  // Maximum is 200k steps per second or 1 step every 5 us
  if( stepsPerSecond > 200000 )
    stepsPerSecond = 200000;
  
  return 1000000 / stepsPerSecond;
}

struct SwitchInput
{
  unsigned long debounceStartMs;
  unsigned long debounceTimeMs;
  int pin;
  int state;
  const char *name;
  bool justTriggered;
  bool justReleased;
  bool isActive;
  bool activeLow;
  
  void Initialize( unsigned int _pin, const char *_name, int inputMode, bool _activeLow, unsigned long _debounceTimeMs )
  {
    pin = _pin;
    name = _name;
    pinMode( _pin, inputMode );
    state = _activeLow ? HIGH : LOW;
    debounceStartMs = 0;
    justReleased = false;
    justTriggered = false;
    isActive = false;
    activeLow = _activeLow;
    debounceTimeMs = _debounceTimeMs;
  }
  
  void DebounceInput()
  {
    justTriggered = false;
    justReleased = false;

    // Debounce input switch
    int tmpState = digitalRead( pin );  
    if( tmpState != state )
    {
      if( debounceStartMs == 0 )
      {
        debounceStartMs = millis();
      }
      else if( msElapsedSince( debounceStartMs ) > debounceTimeMs )
      {
        state = tmpState;
        debounceStartMs = 0;
        
        isActive = activeLow ^ state;
        justTriggered = isActive;
        justReleased = !isActive;

        Serial.print( name );
        if( isActive )
        {
          Serial.println( justReleased ? ": Released" : ": Triggered" );
        }
        else
        {
          Serial.println( justTriggered ? ": Triggered" : ": Released" );
        }
      }
    }
  }
};

SwitchInput momentaryUp;
SwitchInput momentaryDown;
SwitchInput bypassSafety;
SwitchInput apexUp;
SwitchInput apexDown;

unsigned long startTimeMs = 0;
unsigned long endTimeMs = 0;
unsigned long dbgOutputTimeMs = 0;
unsigned long lastRampStepTimeUs = 0;
unsigned long lastStepTimeUs = 0;

long pos = 0;
long motorDir = 0;
long stepCount = 0;
long stepsPerSecond = 0;
long rampStep = 0;

const unsigned long rampIntervalUs = 8000;

const long stepsPerRotation = 200;
const long maxStepsPerSecond = 700;

const long maxPos = 11875;
const long minPos = 0;

void setup()
{
  Serial.begin(9600);
  Serial.println( "Initializing..." );
  
  momentaryUp.Initialize(   MOMENTARY_UP_PIN,   "Momentary Up",   INPUT_PULLUP, true, 50 );
  momentaryDown.Initialize( MOMENTARY_DOWN_PIN, "Momentary Down", INPUT_PULLUP, true, 50 );
  bypassSafety.Initialize(  BYPASS_SAFETY_PIN,  "Bypass Pin",     INPUT_PULLUP, true, 50 );
  apexUp.Initialize(        APEX_UP_PIN,        "Apex Up",        INPUT,        false, 1000 );
  apexDown.Initialize(      APEX_DOWN_PIN,      "Apex DOWN",      INPUT,        false, 1000 );
  
  
  pinMode( MOTOR_DIRECTION_SIG_PIN, OUTPUT );
  pinMode( MOTOR_DIRECTION_GND_PIN, OUTPUT );
  pinMode( MOTOR_PULSE_SIG_PIN,     OUTPUT );
  pinMode( MOTOR_PULSE_GND_PIN,     OUTPUT );
  pinMode( SWITCH_GND_PIN,          OUTPUT );

  pinMode( APEX_UP_PIN,   INPUT );
  pinMode( APEX_DOWN_PIN, INPUT );
  pinMode( APEX_GND_PIN, OUTPUT );

  // Ground pins
  digitalWrite( MOTOR_DIRECTION_GND_PIN, LOW );
  digitalWrite( MOTOR_PULSE_GND_PIN, LOW );
  digitalWrite( SWITCH_GND_PIN, LOW );
  digitalWrite( APEX_GND_PIN, LOW );

  pos = 0;
  motorDir = 0;
  rampStep = 0;
  lastRampStepTimeUs = 0;
  lastStepTimeUs = 0;
  stepsPerSecond = 0;
  endTimeMs = 0;
  startTimeMs = 0;

  Serial.println( "Ready!" );
}

// Warning: This does raw low level register manipulation and bypasses ardiuino crap for speed. 
// It ignores the pulse pin defined at the top!
void Pulse( int sps )
{
  // Pulse
  if( sps > 0 )
  {
    if( usElapsedSince( lastStepTimeUs ) > usPerStep( sps )/2 )
    {
      static int state = 0;
      lastStepTimeUs = micros();
      state = !state;
      if( state ) 
      {
        PORTB |= _BV(PB4);
        stepCount++;
        pos += motorDir;
      }
      else
      {
        PORTB &= ~_BV(PB4);
      }
    }
  }
  else
  {
    PORTB &= ~_BV(PB4);
  }
}

void PrintDebug()
{
  if( Serial.available() )
  {
    while( Serial.available() ) Serial.read();
    Serial.print( "Bypass Safety: " );
    Serial.print( bypassSafety.isActive ? "true" : "false" );
    Serial.print( "\tSps " );
    Serial.print( stepsPerSecond );
    Serial.print( "\tPos: " );
    Serial.print( pos );
    Serial.print( "\tStepCount: " );
    Serial.print( stepCount );
    Serial.print( "\tRotations: " );
    Serial.print( ((float)stepCount) / ((float)stepsPerRotation) );
    Serial.print( "\tMotorDir: " );
    Serial.print( motorDir );
    Serial.print( "\tElapsed: " );
    if( endTimeMs > startTimeMs )
    {
      Serial.print( (endTimeMs - startTimeMs)/1000.0f );
    }
    else
    {
      Serial.print( 0 );
    }
    
    Serial.println( "" );
  }
}

unsigned long dbgTime = 0;

void loop()
{
  // Deboune inputs
  momentaryUp.DebounceInput();
  momentaryDown.DebounceInput();
  bypassSafety.DebounceInput();

  apexUp.DebounceInput();
  apexDown.DebounceInput();

  if( (momentaryUp.isActive && momentaryDown.isActive) ||
      (apexUp.isActive && apexDown.isActive) )
  {
    // Nope
    Serial.println( "Two at once!" );
    return;
  }

  if( bypassSafety.isActive )
  {
    if( momentaryUp.isActive || apexUp.isActive )
    {
      digitalWrite( MOTOR_DIRECTION_SIG_PIN, 0 );
      motorDir = 1;
      Pulse( 100 );
    }
  
    if( momentaryDown.isActive || apexDown.isActive )
    {
      digitalWrite( MOTOR_DIRECTION_SIG_PIN, 1 );
      motorDir = -1;
      Pulse( 100 );
    }

    PrintDebug();
    return;
  }
  
  if( (momentaryUp.isActive || apexUp.isActive) && motorDir == 0  )
  {
    if( pos != maxPos )
    {
      startTimeMs = millis();
      endTimeMs = 0;
      stepCount = 0;     
    }
    
    motorDir = 1;
    digitalWrite( MOTOR_DIRECTION_SIG_PIN, 0 );
  }

  if( (momentaryDown.isActive || apexDown.isActive) && motorDir == 0 )
  {
    if( pos != minPos )
    {
      startTimeMs = millis();
      endTimeMs = 0;
      stepCount = 0;
    }
    
    motorDir = -1;
    digitalWrite( MOTOR_DIRECTION_SIG_PIN, 1 );
  }

  if( ((momentaryUp.isActive || apexUp.isActive) && motorDir != -1) ||
      ((momentaryDown.isActive || apexDown.isActive) && motorDir != 1) )
  {
    rampStep = 1;
  }
  else
  {
    rampStep = -1;
  }

  if( usElapsedSince( lastRampStepTimeUs ) > rampIntervalUs )
  {
    lastRampStepTimeUs = micros();
    stepsPerSecond += rampStep;
  }

  long currMaxStepsPerSecond = maxStepsPerSecond;

  // Buffer
  if( motorDir == 1 && pos < maxPos && (maxPos - pos)/3 < maxStepsPerSecond )
  {
    currMaxStepsPerSecond = max( 50, (maxPos - pos)/3 );
  }

  if( motorDir == -1 && pos > minPos && (pos - minPos)/3 < maxStepsPerSecond )
  {
    currMaxStepsPerSecond = max( 50, (pos - minPos)/3 );
  }

  // Final clamp on sps
  if( stepsPerSecond > currMaxStepsPerSecond )
  {
    stepsPerSecond = currMaxStepsPerSecond;
  }
  
  if( stepsPerSecond < 0 )
  {
    stepsPerSecond = 0;
  }

  if( (pos >= maxPos && motorDir == 1) || (pos <= minPos && motorDir == -1) )
  {
    stepsPerSecond = 0;
  }

  if( stepsPerSecond == 0 )
  {
    motorDir = 0;

    if( endTimeMs == 0 )
    {
      endTimeMs = millis();
    }
  }

  Pulse( stepsPerSecond );

  PrintDebug();
}
