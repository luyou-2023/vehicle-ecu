# Vehicle-ECU
Designed to run a v6 engine (99 Ford Mustang). Can be adapted to any engine size or arrangement.

# 核心流程
1. 传感器的输入引脚定义
   定义和配置：

在嵌入式系统中，首先需要定义传感器连接到微控制器的引脚。这些引脚将用于接收传感器提供的输入信号。
输入引脚的定义：通过 const int 或 const byte 定义输入引脚编号，以确保代码中的引脚编号清晰且易于维护。
示例：

cpp
复制代码
const int throttle_input = A1; // 油门传感器连接到模拟引脚 A1
const byte crank_input = 3; // 曲轴传感器连接到数字引脚 3
工作原理：

油门传感器：通常是模拟传感器，通过读取模拟引脚上的电压值来获得油门的位置。
曲轴传感器：通常是数字传感器，通过读取数字引脚上的脉冲信号来获得曲轴的位置或转速。
2. 执行器的输出引脚定义
   定义和配置：

执行器输出引脚定义用于控制发动机的各个部分，如喷油器或点火系统。
输出引脚的定义：通过 const byte 定义输出引脚编号，用于精确控制执行器的动作。
示例：

cpp
复制代码
const byte inj1 = 5; // 喷油器1连接到数字引脚 5
const byte inj2 = 6; // 喷油器2连接到数字引脚 6
工作原理：

喷油器：通过控制数字引脚的高低电平来调节喷油器的开启或关闭状态，从而控制燃油的喷射。
点火系统：类似地，通过控制引脚的高低电平来控制点火系统的工作状态。
3. 获取输入、计算控制变量、输出控制信号
   步骤：

获取输入：

从传感器获取输入信号。这些信号提供了系统当前状态的信息（如油门位置、曲轴位置等）。
代码示例：
cpp
复制代码
throttle_pos = analogRead(throttle_input); // 读取油门位置
计算控制变量：

根据输入信号计算控制变量。这通常包括将传感器数据转换为实际的控制参数（如喷油时间、点火提前角等）。
代码示例：
cpp
复制代码
throttle_percent = ((throttle_pos + 1) / 1024.0) * 100; // 计算油门百分比
输出控制信号：

根据计算出的控制变量，通过输出引脚控制执行器的动作。这涉及到设置引脚状态以执行具体的操作（如启动喷油器或点火）。
代码示例：
cpp
复制代码
digitalWrite(inj1, HIGH); // 打开喷油器1
总结
在嵌入式系统中，清晰的设计和配置传感器输入引脚、执行器输出引脚、以及通过获取输入、计算控制变量和输出控制信号的流程是至关重要的。这种架构确保了系统能够实时、准确地控制发动机或其他机械系统，从而实现所需的性能和功能。
# end核心流程


## Files
eec-v_full.ino - Arduino sketch. Currently using the Atmega328.

eec-iv_icm.ino - Arduino Sketch. An attempt to replace the ignition control module in a 94 Ford Probe which uses eec-iv.

tooth_wheel_generator.ino - Arduino Sketch. Generates a crankshaft signal of varying RPM. Only used for testing and developing.

main.py - Monitor and Tuning software written in Python 3.6.

ecu_sch-v1.0a.sch - Schematic for the ECU. Created with KiCad


Upload the sketch to any atmega328. Read the top of the sketch for what pins do what.

## Features
Arduino <-> PC communication is via USB

Can decode toothed wheel crankshaft signals. Signal must already be conditioned (0-5v square wave).

Fuel and spark advance maps, as well as on-the-fly adjustment.

## Note
This was written with the intention of replacing the stock ecu in a  99 Mustang V6. Its kind of a dream project. It should be capable of running any v6 engine using a wasted spark ignition system, and can be adapted for other types or cylinder counts. Inline or V-Engine should not matter.
