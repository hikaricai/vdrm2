#include <Arduino.h>
#include <Wire.h>
#include <SimpleFOC.h>
#include <iostream>
#include <sstream>
#include "BluetoothSerial.h" //蓝牙库

// M创动工坊测试代码，支持蓝牙串口输入，支持PC串口输入，硬件与客服见淘宝店：mcdgf.taobao.com
// 蓝牙控制：请使用安卓手机安装蓝牙控制串口APK，PC串口：发送Txx，xx表示数值。
//  SDA 21
//  SCL 22
//  magnetic sensor instance - I2C

#define _RAD2DEG 57.2957795131  // 180/PI

struct PIDParams
{
    float P;
    float I;
    float D;
    float LPF_Tf;
};

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
BluetoothSerial SerialBT; // 实例化蓝牙

BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(32, 33, 25, 12); // 定义ESP32与电机驱动器的连接引脚，12是使能引脚

float target_torque = 0.03; // 扭矩值
char model = 'p';           // 模式控制，默认位置模式
float change_value = 0.5;   // 变化值

PIDParams pos_pid = {20, 25, 0, 0};      // 位置环参数
PIDParams vel_pid = {0.1f, 1, 0, 0.01f}; // 速度环参数

// 添加必要的全局变量
float target_position = 0;
float target_velocity = 10;        // 默认速度10rad/s
float current_status = 0;        // 当前状态参数
const float max_velocity = 110;    // 最大速度限制
const float default_velocity = 10; // 默认速度
const float max_torque = 6;       // 最大扭矩限制，实际是电压限制
const float default_torque = 1;    // 默认扭矩
String serialBuffer = "";          // 串口缓冲区
void processCommand(String cmd);   // 处理串口命令

Commander command = Commander(Serial); // 使用USB串口发送命令
void doTarget(char *cmd) { command.scalar(&target_position, cmd); }

void setup()
{
    // 初始化磁传感器
    Wire.setClock(400000);
    // Wire1.begin(19,23,(uint32_t)400000);//本程序使用esp32默认I2C端口，如果需要更改I2C端口，取消注释这一行，并注释掉上一行
    // sensor.init(&Wire);
    sensor.init();

    // 连接磁传感器
    motor.linkSensor(&sensor);
    // 设置电源电压为12V
    driver.voltage_power_supply = 12;
    driver.init();
    // 将电机与驱动器连接
    motor.linkDriver(&driver);
    // 选择电机控制器的电压模式
    motor.torque_controller = TorqueControlType::voltage;
    // 选择电机的FOC模式
    motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
    // 初始化电机控制器为位置模式
    motor.controller = MotionControlType::angle; // 位置环控制

    // 初始化PID参数
    motor.P_angle.P = pos_pid.P;
    motor.P_angle.I = pos_pid.I;
    motor.PID_velocity.P = vel_pid.P;
    motor.PID_velocity.I = vel_pid.I;
    motor.LPF_velocity.Tf = vel_pid.LPF_Tf;

    // 限制最大電壓和最大速度
    motor.voltage_limit = 6;
    motor.velocity_limit = 50;

    motor.PID_velocity.output_ramp = 1200; // 调整这个值可以影响电机的加速和减速性能。较高的值会使电机加速和减速更快，但可能导致振动或电流峰值。
    motor.LPF_velocity.Tf = 0.01f;         // 这可以滤除电机的噪声和高频振动，从而使速度控制更加稳定。

    Serial.begin(115200);
    // 串口调试
    motor.useMonitoring(Serial);
    // 电机初始化
    motor.init();
    // 电机FOC初始化
    motor.initFOC();
    // 使用命令行控制电机Txx，xx表示数值
    command.add('T', doTarget, "target angle"); // 通过串口T命令发送位置，比如T6.28,表示电机转6.28弧度,即1圈
    SerialBT.begin("ESP32-Motor-Control-A");      // 蓝牙设备的名称，这个就是在手机蓝牙端看到的名称
    Serial.println(F("电机准备就绪。"));
    _delay(1000);
}

// 蓝牙串口调试函数
// 全局变量
unsigned long lastCharTime = 0; // 最后接收字符时间戳
const int commandTimeout = 50;  // 命令超时时间（毫秒）
void serial_debug()
{
    while (SerialBT.available() > 0)
    {
        char c = SerialBT.read();

        if (isAlpha(c) && serialBuffer.length() > 0)
        {
            processCommand(serialBuffer);
            serialBuffer = "";
        }
        serialBuffer += c;
        lastCharTime = millis();
    }

    // 处理超时命令（无新字符且缓冲区有数据）
    if (serialBuffer.length() > 0 && (millis() - lastCharTime) > commandTimeout)
    {
        processCommand(serialBuffer);
        serialBuffer = "";
    }
}

void processCommand(String cmd)
{
    cmd.trim();
    if (cmd.length() == 0)
        return;

    char modeChar = cmd[0];
    String param = cmd.substring(1);

    switch (modeChar)
    {
    /*--- 模式控制 ---*/
    case 'p': // 位置模式
        model = 'p';
        if (param.length() > 0)
        {
            // 相对位置模式：p100表示从当前位置转动100弧度
            target_position = motor.shaftAngle() + param.toFloat();
        }
        else
        {
            // 立即停止在当前位置
            target_position = motor.shaftAngle();
        }
        break;

    case 'v': // 速度模式
        model = 'v';
        if (param.length() > 0)
        {
            target_velocity = constrain(param.toFloat(), -max_velocity, max_velocity);
        }
        else
        {
            target_velocity = default_velocity;
        }
        break;

    case 't': // 扭矩模式
        model = 't';
        if (param.length() > 0)
        {
            target_torque = constrain(param.toFloat(), -max_torque, max_torque);
        }
        else
        {
            target_torque = default_torque;
        }
        break;

    /*--- 运动控制 ---*/
    case 's': // 位置归零
        target_position = 0;
        break;

    case 'f': // 正转（带可选速度参数）
        model = 'v';
        if (param.length() > 0)
        {
            target_velocity = constrain(param.toFloat(), 0, max_velocity);
        }
        else
        {
            target_velocity = default_velocity;
        }
        break;

    case 'b': // 反转（带可选速度参数）
        model = 'v';
        if (param.length() > 0)
        {
            target_velocity = constrain(-param.toFloat(), -max_velocity, 0);
        }
        else
        {
            target_velocity = -default_velocity;
        }
        break;

    case 'e': // 显示当前位置和速度
        model = 'e';
        break;

    default:
        SerialBT.println("Unknown command");
        return;
    }

    // 发送状态反馈
    SerialBT.print("控制模式: ");
    SerialBT.print(model);
    if (model == 'p')
    {
        SerialBT.print(" | 目标位置（弧度）: ");
        SerialBT.println(target_position, 2);
    }
    else if (model == 'v')
    {
        SerialBT.print(" | 目标速度（弧度/秒）: ");
        SerialBT.println(target_velocity, 2);
    }
    else if (model == 't')
    {
        SerialBT.print(" | 目标扭矩(电压模式): ");
        SerialBT.println(target_torque, 2);
    }
    else if (model == 'e') // 显示当前位置和速度
    {
        current_status = motor.shaftAngle();
        SerialBT.print(" | 当前位置(弧度): ");
        SerialBT.println(current_status, 2);
        current_status = motor.shaftVelocity();
        SerialBT.print(" | 当前速度值（弧度/秒）: ");
        SerialBT.println(current_status, 2);
    }
}

void loop()
{
    motor.loopFOC(); // 给上劲
    serial_debug();  //
    // 模式切换控制
    switch (model)
    {
    case 'p':
        motor.controller = MotionControlType::angle;
        motor.move(target_position);
        break;
    case 'v':
        motor.controller = MotionControlType::velocity;
        motor.move(target_velocity);
        break;
    case 't':
        motor.controller = MotionControlType::torque;
        motor.move(target_torque); // 需要根据实际情况调整扭矩控制
        break;
    }
    command.run(); // 监控串口输入的命令
}
