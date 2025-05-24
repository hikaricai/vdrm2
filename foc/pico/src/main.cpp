#include <Arduino.h>
#include <SimpleFOC.h>
#define PIN_SYNC     (28u)

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
// magnetic sensor instance - MagneticSensorI2C
//MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
// MagneticSensorAnalog sensor = MagneticSensorAnalog(A1, 14, 1020);

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(6, 8, 10, 12);
// Stepper motor & driver instance
//StepperMotor motor = StepperMotor(50);
//StepperDriver4PWM driver = StepperDriver4PWM(9, 5, 10, 6,  8);

// velocity set point variable
float target_velocity = -6.18 * 1.0;
// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_velocity, cmd); }

void setup() {

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIN_SYNC, OUTPUT);
  // use monitoring with serial 
  Serial.begin(115200);
  // enable more verbose output for debugging
  // comment out if not needed
  SimpleFOCDebug::enable(&Serial);

  // initialise magnetic sensor hardware
  sensor.init();
  // link the motor to the sensor
  motor.linkSensor(&sensor);

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  driver.init();
  // link the motor and the driver
  motor.linkDriver(&driver);

  // set motion control loop to be used
  motor.controller = MotionControlType::velocity;
  motor.sensor_direction = Direction::CW;
  motor.zero_electric_angle = 3.98;

  // contoller configuration
  // default parameters in defaults.h

  // velocity PI controller parameters
  motor.PID_velocity.P = 0.1f;
  motor.PID_velocity.I = 1;
  motor.PID_velocity.D = 0;
  // motor.PID_velocity.D = 0;

  motor.velocity_limit = 50;
  // motor.PID_velocity.output_ramp = 1200; // 调整这个值可以影响电机的加速和减速性能。较高的值会使电机加速和减速更快，但可能导致振动或电流峰值。
  // motor.LPF_velocity.Tf = 0.01f;         // 这可以滤除电机的噪声和高频振动，从而使速度控制更加稳定。
  // default voltage_power_supply
  motor.voltage_limit = 6;
  // jerk control using voltage voltage ramp
  // default value is 300 volts per sec  ~ 0.3V per millisecond
  motor.PID_velocity.output_ramp = 1200;

  // velocity low pass filtering
  // default 5ms - try different values to see what is the best.
  // the lower the less filtered
  motor.LPF_velocity.Tf = 0.01f;

  // comment out if not needed
  motor.useMonitoring(Serial);

  // initialize motor
  motor.init();
  // align sensor and start FOC
  motor.initFOC();

  // add target command T
  command.add('T', doTarget, "target velocity");

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target velocity using serial terminal:"));
  _delay(1000);
}

void triggle_pin(pin_size_t pin, PinStatus *pin_status) {
  *pin_status = ((*pin_status == PinStatus::LOW) ? PinStatus::HIGH: PinStatus::LOW);
  digitalWrite(pin, *pin_status);
}

PinStatus pin_status = PinStatus::LOW;
PinStatus sync_status = PinStatus::LOW;
uint32_t last_ts = 0;
uint32_t last_region = 0;

void loop() {
  uint32_t ts = time_us_32() / 1000000;
  if (last_ts != ts) {
    triggle_pin(LED_BUILTIN, &pin_status);
  }
  last_ts = ts;
  // main FOC algorithm function
  // the faster you run this function the better
  // Arduino UNO loop  ~1kHz
  // Bluepill loop ~10kHz
  motor.loopFOC();
  uint32_t region = (uint32_t)(sensor.getMechanicalAngle() * 8.0f / _2PI);
  if (last_region != region) {
    triggle_pin(PIN_SYNC, &sync_status);
  }
  last_region = region;

  // Motion control function
  // velocity, position or voltage (defined in motor.controller)
  // this function can be run at much lower frequency than loopFOC() function
  // You can also use motor.move() and set the motor.target in the code
  motor.move(target_velocity);

  // function intended to be used with serial plotter to monitor motor variables
  // significantly slowing the execution down!!!!
  // motor.monitor();

  // user communication
  command.run();
}
