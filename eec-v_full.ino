// 99 Mustang V6 with EEC-V
// v1.1a
/***************************************************************************
代码的整体功能概述
这段代码用于控制一个发动机管理系统（ECU, Engine Control Unit）的核心逻辑。主要包括以下几个功能：

点火控制：根据发动机的转速（RPM）和传感器输入，计算点火提前角度，控制点火线圈的通断，以确保火花塞在合适的时间点火。

喷油控制：根据节气门开度和发动机转速，计算喷油时间，控制喷油器的打开和关闭时间，以确保适量的燃油喷入发动机气缸。

信号同步和转速计算：通过曲轴传感器检测发动机旋转状态，计算发动机的转速（RPM），并检查信号同步状态，以确保点火和喷油操作与发动机的实际运行同步。

数据采集和处理：实时监控发动机的输入（如节气门位置）和计算所需的燃油和点火参数，以优化发动机性能。
***************************************************************************/


//这些宏定义允许开发者绕过Arduino库的抽象层，直接操作底层硬件，提供更高效的控制方法。在性能要求高或资源受限的嵌入式应用程序中，这种方法特别有用。不过，只有在开发者非常熟悉硬件的工作原理时才应使用这些宏。对于大多数Arduino项目，使用标准的pinMode()、digitalWrite()和digitalRead()函数已经足够。
#define portOfPin(P)(((P)>=0&&(P)<8)?&PORTD:(((P)>7&&(P)<14)?&PORTB:&PORTC)) // 根据引脚编号P，返回对应的PORT寄存器地址
#define ddrOfPin(P)(((P)>=0&&(P)<8)?&DDRD:(((P)>7&&(P)<14)?&DDRB:&DDRC)) // 根据引脚编号P，返回对应的DDR寄存器地址
#define pinOfPin(P)(((P)>=0&&(P)<8)?&PIND:(((P)>7&&(P)<14)?&PINB:&PINC)) // 根据引脚编号P，返回对应的PIN寄存器地址
#define pinIndex(P)((uint8_t)(P>13?P-14:P&7)) // 根据引脚编号P，计算出引脚的索引
#define pinMask(P)((uint8_t)(1<<pinIndex(P))) // 根据引脚编号P，计算出引脚的掩码

#define pinAsInput(P) *(ddrOfPin(P))&=~pinMask(P) // 将引脚P设为输入模式
#define pinAsInputPullUp(P) *(ddrOfPin(P))&=~pinMask(P);digitalHigh(P) // 将引脚P设为输入模式，并启用上拉电阻
#define pinAsOutput(P) *(ddrOfPin(P))|=pinMask(P) // 将引脚P设为输出模式
#define digitalLow(P) *(portOfPin(P))&=~pinMask(P) // 将引脚P的电平设为低电平
#define digitalHigh(P) *(portOfPin(P))|=pinMask(P) // 将引脚P的电平设为高电平
#define isHigh(P)((*(pinOfPin(P))& pinMask(P))>0) // 判断引脚P的电平是否为高电平
#define isLow(P)((*(pinOfPin(P))& pinMask(P))==0) // 判断引脚P的电平是否为低电平
#define digitalState(P)((uint8_t)isHigh(P)) // 返回引脚P的数字状态（高电平或低电平）

/***
const int: 这表示 throttle_input 是一个常量整数，值在程序运行时不会改变。使用 const 关键字确保引脚号不会被意外修改。
throttle_input: 这是变量的名称，表示油门输入信号的引脚编号。
A1: 在许多微控制器（如 Arduino）中，A1 是一个代表模拟输入引脚的预定义常量。A1 表示的是第一个模拟输入引脚，这通常是与微控制器的某个特定引脚连接的。
模拟输入信号
输入信号范围：油门输入信号的范围是 0 到 5V。模拟引脚可以读取电压值并将其转换为一个对应的整数值，通常是从 0 到 1023 的范围（对于 10 位分辨率的 ADC）。
读数方法：你可以使用 analogRead(throttle_input) 函数来读取连接到 A1 引脚的模拟信号。这个函数会返回一个 0 到 1023 之间的整数值，表示输入电压相对于 5V 的比例。
**/

/********
曲轴位置传感器的工作原理
曲轴位置传感器用于监测曲轴的旋转位置，通常具有以下特点：

传感器类型：

磁性传感器（如霍尔效应传感器）：这些传感器通常会输出一个数字信号，根据曲轴上的齿轮或磁铁的变化产生脉冲。
光电传感器：使用光学传感器来检测曲轴上的标记或槽的变化，输出脉冲信号。
信号处理：

信号调理：传感器的信号经过调理，以确保其电压范围适合微控制器的输入范围（例如 0 到 5V）。调理通常包括放大和滤波，以确保信号稳定和准确。
数字信号：曲轴位置传感器通常输出一个数字信号（高/低脉冲），该信号代表曲轴的当前位置。微控制器通过读取这个信号来确定曲轴的角度和转速。
微控制器读取：

中断或轮询：微控制器可以通过中断方式（当曲轴位置传感器产生脉冲时触发中断）或轮询方式（周期性检查引脚状态）来读取曲轴信号。
数据处理：读取到的脉冲信号可以用于计算曲轴的转速、位置等重要数据，用于发动机控制和优化。
******/


// 数字输入引脚 在Arduino编程中，3代表的是Arduino开发板上的数字引脚编号（即引脚D3）
/****
引脚3能代表曲轴传感器？
物理连接: 在一个实际的Arduino项目中，曲轴传感器的信号输出线被物理连接到Arduino开发板的数字引脚3。因此，引脚3被用来接收曲轴传感器的信号。在代码中，用数字3来表示所选用的引脚编号。

输入信号类型: 曲轴传感器通常产生脉冲信号，这些信号表示发动机曲轴的位置和速度。在Arduino项目中，将曲轴传感器信号接入数字输入引脚（如3号引脚），可以用Arduino的数字读取功能（如digitalRead()）来检测脉冲信号。

信号调理: 代码注释中的说明提到：“来自曲轴传感器的输入信号，应该已调理到 0-5V 范围。”这意味着传感器的输出信号已经通过某种方式（例如使用电压分压器、电平转换器或其他电路）调理成了Arduino可安全读取的电压范围（0-5V）。这对于Arduino来说是必要的，因为大多数Arduino板仅支持0到5V或0到3.3V的输入电压范围。
***/
const byte crank_input = 3; // 来自曲轴传感器的输入信号, 应该已调理到 0-5V 范围。

/*****
throttle_input 变量定义的 A1 引脚与实际的传感器相连。具体来说，它用于连接一个模拟油门位置传感器，这种传感器通常用来测量油门踏板的位置或开度。

传感器的工作原理
油门位置传感器通常是一个电位计（或类似的传感器），其工作原理如下：

电压信号：传感器输出一个与油门踏板位置成比例的电压信号。这个电压信号通常在 0 到 5V 之间变化，具体取决于油门踏板的位置。

模拟输入：微控制器的模拟输入引脚（如 A1）读取这个电压信号，并将其转换为一个数字值。这个值通常是 10 位分辨率的，即从 0 到 1023 的范围，表示输入电压在 0 到 5V 之间的比例。

转换和处理：读取到的数字值可以被用来计算油门的实际百分比或其他相关数据
******/
// 模拟输入引脚 在 Arduino 编程环境中，A1 表示模拟输入引脚 1。在许多 Arduino 板上（如 Arduino Uno），模拟引脚被标记为 A0, A1, A2，等等。这些引脚用于读取模拟信号，例如来自传感器的电压变化。
const int throttle_input = A1; // 油门输入信号，范围 0-5V

// 点火输出设置
unsigned int ign_dwell_time = 3000; // 点火线圈充电时间，单位微秒
const byte coila = 9; // 点火线圈A的引脚编号
bool coila_enabled = true; // 点火线圈A是否启用
const byte coilb = 10; // 点火线圈B的引脚编号
bool coilb_enabled = true; // 点火线圈B是否启用
const byte coilc = 11; // 点火线圈C的引脚编号
bool coilc_enabled = true; // 点火线圈C是否启用
byte current_coil = 0; // 当前使用的点火线圈

//引脚 (inj1, inj2, inj3, inj4, inj5, inj6) 是用于控制喷油器的执行器引脚。它们通常连接到喷油器驱动电路，控制喷油器的开启和关闭，从而实现燃油喷射。
// 喷油器输出设置
const byte inj1 = 5; // 喷油器1的引脚编号
const byte inj2 = 6; // 喷油器2的引脚编号
const byte inj3 = 7; // 喷油器3的引脚编号
const byte inj4 = 7; // 喷油器4的引脚编号
const byte inj5 = 5; // 喷油器5的引脚编号
const byte inj6 = 6; // 喷油器6的引脚编号
byte inja = 0; // 当前喷油器A的编号
byte injb = 0; // 当前喷油器B的编号

// 曲轴相关变量
volatile unsigned long last_crank_time = 0; // 上次曲轴信号时间，单位微秒
volatile unsigned long last_crank_duration = 0; // 上次曲轴信号持续时间，单位微秒
const int tooth_count = 36; // 曲轴齿轮的齿数
volatile int current_tooth = 0; // 当前的齿编号
unsigned long tooth_times[tooth_count - 1] = {}; // 存储每个齿的时间间隔
volatile bool crank_toggle = false; // 曲轴信号的切换状态

// 其他参数
unsigned int engine_rpm = 0; // 发动机的转速
unsigned int engine_rpm_max = 5500; // 发动机的最大转速
bool over_rev = false; // 是否超速
bool signal_sync = false; // 是否同步信号

unsigned long ign_advance_in_us = 0; // 点火提前量，单位微秒
int ign_advance_trim_in_deg = 0; // 点火提前量的修正，单位度
unsigned long ign_start = 0; // 点火开始时间
unsigned long ign_end = 0; // 点火结束时间

unsigned long inj_duration = 100; // 喷油持续时间，单位微秒
unsigned long inj_trim = 0; // 喷油时间修正
unsigned long inj_start = 0; // 喷油开始时间
unsigned long inj_end = 0; // 喷油结束时间

int throttle_pos = 1; // 油门位置
byte throttle_percent = 0; // 油门百分比

byte s_buffer[128]; // 缓冲区，用于存储数据
int bytes_waiting = 0; // 等待处理的字节数

/************
这两个表格分别定义了燃油喷射量 (fuel_table) 和点火提前角度 (ignition_table) 的数据，根据不同的发动机转速和油门开度来进行调节。

1. 燃油喷射表 (fuel_table)
用途：定义在不同发动机转速和油门开度下需要的燃油喷射持续时间（单位可能是微秒或毫秒）。
x轴：表示发动机转速，每个增量为 500 RPM，从 0 到 5000 RPM（假设从第一个到最后一个，分别是 0, 500, 1000, ..., 5000）。
y轴：表示油门开度的百分比，每个增量为 10%，从 0% 到 100%。
表格中每个数值表示在对应转速和油门开度下的燃油喷射持续时间。例如：

fuel_table[0][0] 表示在 0% 油门 和 0 RPM 下的喷射时间是 1200。
fuel_table[10][10] 表示在 100% 油门 和 5000 RPM 下的喷射时间是 2400。
2. 点火提前表 (ignition_table)
用途：定义在不同发动机转速和油门开度下的点火提前角度（单位通常为度数）。
x轴：表示发动机转速，每个增量为 500 RPM，从 0 到 5000 RPM。
y轴：表示油门开度的百分比，每个增量为 10%，从 0% 到 100%。
表格中每个数值表示在对应转速和油门开度下的点火提前角度。例如：

ignition_table[0][0] 表示在 0% 油门 和 0 RPM 下的点火提前角为 10 度。
ignition_table[10][10] 表示在 100% 油门 和 5000 RPM 下的点火提前角为 17 度。
如何使用这些表格
在程序中，表格中的数据被用于计算发动机的运行参数。例如：

根据当前油门开度（throttle_percent）和转速（engine_rpm），确定表格的索引位置（throttle_index 和 rpm_index）。
然后从 fuel_table 中获取燃油喷射时间，从 ignition_table 中获取点火提前角度。
使用这些参数来调整发动机的燃油喷射和点火时间，以优化燃烧效率和动力输出。

调校发动机的主要工作之一就是修改这两个表格（燃油喷射表和点火提前表）。

调校的目的
燃油经济性：通过调整燃油喷射量，在不同的转速和负载条件下达到最佳的燃烧效率，从而提高燃油经济性。
动力输出：通过调整点火提前角度和喷油时间，优化发动机在不同转速和油门开度下的动力输出，使发动机在各种驾驶条件下都有良好的表现。
排放控制：通过精确控制燃油喷射和点火时机，减少有害气体的排放，符合排放标准。
驾驶体验：改善油门响应、提高驾驶舒适性，优化换挡时机等。
如何调校
燃油喷射表 (fuel_table) 调校：

增加或减少燃油喷射时间可以调节混合气的浓度。
在高转速或高负载时，通常需要更多燃油，以保持动力输出和冷却效果。
在低转速或低负载时，可以减少燃油以提高燃油经济性。
点火提前表 (ignition_table) 调校：

点火提前角度直接影响发动机的燃烧效率和动力输出。
通常在高转速下需要提前点火，以便燃烧在活塞到达上止点时达到最大压力，从而获得最大的动力输出。
调整点火提前角度还可以避免爆震（发动机过早燃烧导致的冲击），保护发动机。
调校过程
数据收集：使用发动机台架或车载数据记录设备，收集不同条件下的发动机性能数据（如空气流量、发动机负荷、排气成分、温度等）。

分析与测试：分析这些数据，确定在哪些条件下需要优化燃油喷射或点火提前角度。然后逐步调整表格中的值，并进行测试，查看结果是否符合预期。

验证与优化：通过反复测试和微调，找到最优的参数组合。需要考虑发动机的可靠性和长寿命，确保在各种环境条件下的稳定性。

使用工具
调校通常会使用专业的 ECU（发动机控制单元）调校软件和硬件设备，如 OBD（车载诊断系统）接口、台架测试设备等。

通过修改这两个表格，可以显著改变发动机的性能表现。专业的调校需要结合丰富的经验和实际的测试数据。
*****************/


/**********
燃油喷射表 (fuel_table)
燃油喷射表的结构是一个 11x11 的矩阵，其中的每个元素表示在特定转速和油门开度下的燃油喷射量。行表示油门百分比，列表示转速。数据单位一般是燃油喷射时间（微秒等），具体单位取决于你的系统设置。

表格解释：
行：表示油门开度（以 10% 增量递增），从 0% 到 100%。例如，第一行表示油门开度为 0%，第二行为 10%，依此类推。

列：表示发动机转速，以 500 转/分钟为增量。第一个列表示 500 转/分钟，第二列为 1000 转/分钟，以此类推。

例如：

fuel_table[0][0] 的值为 1200，表示在油门开度为 0% 时，发动机转速为 500 转/分钟时的燃油喷射量为 1200 微秒。
fuel_table[1][4] 的值为 1500，表示在油门开度为 10% 时，发动机转速为 2000 转/分钟时的燃油喷射量为 1500 微秒。
fuel_table[10][10] 的值为 2400，表示在油门开度为 100% 时，发动机转速为 5000 转/分钟时的燃油喷射量为 2400 微秒。
点火提前表 (ignition_table)
点火提前表也是一个 11x11 的矩阵，其中的每个元素表示在特定转速和油门开度下的点火提前角度（通常以度为单位）。

表格解释：
行：表示油门开度（以 10% 增量递增），从 0% 到 100%。

列：表示发动机转速，以 500 转/分钟为增量。

例如：

ignition_table[0][0] 的值为 10，表示在油门开度为 0% 时，发动机转速为 500 转/分钟时的点火提前角度为 10 度。
ignition_table[4][5] 的值为 13，表示在油门开度为 40% 时，发动机转速为 2500 转/分钟时的点火提前角度为 13 度。
ignition_table[10][10] 的值为 17，表示在油门开度为 100% 时，发动机转速为 5000 转/分钟时的点火提前角度为 17 度。
总结
燃油喷射表：控制不同油门开度和转速条件下的喷油量。数值越高，喷油时间越长，混合气越浓。
点火提前表：控制不同油门开度和转速条件下的点火提前角度。角度越大，点火时间越早，以优化发动机的燃烧效率。
通过对这两个表的调整，可以实现对发动机性能的精细调控，从而达到更好的燃油经济性、动力输出和排放控制。
***********/
// 燃油喷射表
int fuel_table[11][11] = {
  // x轴 = 转速（500的增量）
  // y轴 = 油门百分比（10的增量）
  {1200, 1200, 1300, 1400, 1500, 1600, 1700, 1800, 1900, 2000, 2100},
  {1200, 1200, 1300, 1400, 1500, 1600, 1700, 1800, 1900, 2000, 2100},
  {1200, 1200, 1300, 1400, 1500, 1600, 1700, 1800, 1900, 2000, 2100},
  {1300, 1300, 1400, 1500, 1600, 1700, 1800, 1900, 2000, 2100, 2200},
  {1300, 1300, 1400, 1500, 1600, 1700, 1800, 1900, 2000, 2100, 2200},
  {1300, 1300, 1400, 1500, 1600, 1700, 1800, 1900, 2000, 2100, 2200},
  {1400, 1400, 1500, 1600, 1700, 1800, 1900, 2000, 2100, 2200, 2300},
  {1400, 1400, 1500, 1600, 1700, 1800, 1900, 2000, 2100, 2200, 2300},
  {1400, 1400, 1500, 1600, 1700, 1800, 1900, 2000, 2100, 2200, 2300},
  {1450, 1450, 1550, 1650, 1750, 1850, 1950, 2050, 2150, 2250, 2350},
  {1500, 1500, 1600, 1700, 1800, 1900, 2000, 2100, 2200, 2300, 2400}
};


/*********************
如何读取表格
确定油门开度和转速：

油门开度（以百分比表示）决定了表格的行数。例如，油门开度为 30% 对应表格的第 3 行（索引 2）。
转速（以 500 转/分钟为单位）决定了表格的列数。例如，转速为 2500 转/分钟 对应表格的第 5 列（索引 4）。
查找对应的点火提前角度：

根据油门开度和转速找到对应的表格行和列，从而得到点火提前角度。例如，油门开度为 30%（第 3 行），转速为 3000 转/分钟（第 6 列），那么点火提前角度是 ignition_table[2][5] 的值。
表格示例解释
ignition_table[0][0] = 10：油门开度为 0% 时，转速为 500 转/分钟 的点火提前角度是 10 度。
ignition_table[4][4] = 13：油门开度为 40% 时，转速为 2000 转/分钟 的点火提前角度是 13 度。
ignition_table[10][10] = 17：油门开度为 100% 时，转速为 5000 转/分钟 的点火提前角度是 17 度。
*************************/
// 点火提前表
int ignition_table[11][11] = {
  // x轴 = 转速（500的增量）
  // y轴 = 油门百分比（10的增量）
  {10, 10, 12, 12, 13, 13, 14, 14, 15, 15, 16},
  {10, 10, 12, 12, 13, 13, 14, 14, 15, 15, 16},
  {10, 10, 12, 12, 13, 13, 14, 14, 15, 15, 16},
  {10, 10, 12, 12, 13, 13, 14, 14, 15, 15, 16},
  {12, 10, 12, 12, 13, 13, 14, 14, 15, 15, 16},
  {12, 10, 12, 12, 13, 13, 14, 14, 15, 15, 16},
  {12, 12, 13, 13, 14, 14, 15, 15, 16, 16, 17},
  {12, 12, 13, 13, 14, 14, 15, 15, 16, 16, 17},
  {12, 12, 13, 13, 14, 14, 15, 15, 16, 16, 17},
  {12, 12, 13, 13, 14, 14, 15, 15, 16, 16, 17},
  {12, 12, 13, 13, 14, 14, 15, 15, 16, 16, 17}
};

// 曲轴中断处理函数
void crank_interrupt() {
  // 如果是第一次检测到曲轴信号
  if (last_crank_time == 0 && last_crank_duration == 0) {
    last_crank_time = micros();  // 记录当前时间
    current_tooth = 0;           // 初始化当前齿轮位置
  }
  else {
    unsigned long duration = micros() - last_crank_time; // 计算当前齿轮周期时间
    last_crank_time = micros(); // 更新最后一次齿轮信号时间
    current_tooth += 1;         // 增加齿轮计数
    // 如果已检测到完整的齿轮周期，重置齿轮计数
    if (current_tooth == tooth_count) {
      current_tooth = 1;
    }
    tooth_times[current_tooth - 1] = duration; // 保存当前齿轮周期时间
    // 如果还未完成完整齿轮周期
    if (current_tooth != tooth_count - 1) {
      last_crank_duration = duration; // 保存最后一次齿轮周期时间
    }
    else { check_sync(); } // 检查同步状态
  }
}

// 初始化设置函数
void setup() {
  Serial.begin(115200); // 初始化串口通讯
  pinMode(crank_input, INPUT); // 设置曲轴信号输入引脚
  digitalLow(13); // 关闭13号引脚输出
  digitalLow(coila); // 关闭点火线圈A
  digitalLow(coilb); // 关闭点火线圈B
  digitalLow(coilc); // 关闭点火线圈C
  digitalLow(inj1); // 关闭喷油器1
  digitalLow(inj2); // 关闭喷油器2
  digitalLow(inj3); // 关闭喷油器3
  attachInterrupt(digitalPinToInterrupt(crank_input), crank_interrupt, FALLING); // 设置曲轴信号中断
}

// 根据角度计算点火提前量
int calculate_advance_from_deg(int deg) {
  return (last_crank_duration / 10) * deg; // 计算提前角
}

// 检查同步状态函数
void check_sync() {
  if (tooth_times[0] > 0) { // 如果有有效的齿轮周期时间
    unsigned int tooth_index = 0; // 用于存储最长齿轮周期的索引
    unsigned long tooth_value = 0; // 用于存储最长齿轮周期的时间值

    // 查找最长的齿轮周期时间
    for (int i = 0; i < tooth_count - 1; i++) { // 遍历所有齿轮周期时间
      if (tooth_times[i] > tooth_value) { // 如果当前齿轮周期时间大于之前记录的最长时间
        tooth_value = tooth_times[i]; // 更新最长齿轮周期时间
        tooth_index = i; // 更新最长齿轮周期时间对应的索引
      }
      tooth_times[i] = 0; // 重置齿轮周期时间数组
    }

    // 判断同步状态
    if (tooth_index == current_tooth - 1) { // 如果最长齿轮周期的索引是当前齿轮位置的前一个齿
      if (!signal_sync) { // 如果当前未同步状态
        byte to_send[3] = {0x00, 0x01, 0x00}; // 发送同步信号给上位机
        add_to_serial(to_send, 3);
      }
      signal_sync = true; // 标记为同步状态
      crank_toggle = !crank_toggle; // 切换曲轴状态
    }
    else { // 如果未找到正确的最长齿轮周期
      if (signal_sync) { // 如果当前是同步状态
        byte to_send[3] = {0x00, 0x00, 0x00}; // 发送失步信号给上位机
        add_to_serial(to_send, 3);
      }
      signal_sync = false; // 标记为未同步状态
      current_tooth = current_tooth - 1 - tooth_index; // 更新当前齿轮位置，重新计算同步
    }
  }
}

/***
在动力控制单元（ECU）的设计中，获取点火和喷油时间的标准方法通常包括以下步骤：

1. 传感器数据获取
首先，ECU需要实时获取来自不同传感器的数据，如油门传感器、发动机转速传感器、空气流量传感器等。这些数据用于计算点火和喷油时间。

2. 查找表格数据
使用查找表（Lookup Tables）是一种常见的做法。查找表是根据不同的传感器数据（如油门开度和发动机转速）预先计算并存储的点火和喷油时间值。查找表的数据通常以二维或三维的网格形式存在。例如：

喷油时间查找表：fuel_table，存储了不同油门开度和发动机转速条件下的喷油时间。
点火时间查找表：ignition_table，存储了不同油门开度和发动机转速条件下的点火时间。
3. 插值计算
由于实际的传感器数据可能不会完全匹配查找表中的数据点，ECU通常需要进行插值计算以获取更准确的点火和喷油时间。插值方法可以是线性插值、双线性插值或更复杂的多项式插值等。

4. 计算点火和喷油时间
根据查找表和插值结果，ECU会计算出相应的点火和喷油时间。这些计算结果会用于调整点火时机和喷油量，以优化发动机性能和燃油效率。

标准方法的特点
查找表方法：是一种预先计算和存储的方案，能够快速查找和响应传感器数据。这种方法适合于实时控制应用，因为它能够减少计算量和提高响应速度。

插值方法：通过插值计算来处理传感器数据与查找表数据点之间的差距，能够提供更精确的控制。

示例流程
数据采集：实时读取油门开度和发动机转速。
数据查找：在查找表中找到最接近的离散点。
插值计算：如果传感器数据不完全匹配表格点，则使用插值算法计算精确的点火和喷油时间。
控制输出：根据计算结果调整点火时机和喷油量。
实际应用中的细节
表格精度：查找表的精度和范围需要根据发动机特性和应用需求来确定。
插值算法：根据需要选择合适的插值算法，确保计算结果的准确性和实时性。
参数调整：需要在实际测试中对表格数据和插值算法进行调整和优化，以满足发动机的性能要求。
**/
// 获取输入信号函数
void get_inputs() {
  throttle_pos = analogRead(throttle_input); // 读取油门输入信号（模拟信号值）

  // 计算油门百分比，将模拟输入信号转换为百分比表示
  throttle_percent = ((throttle_pos + 1) / 1024.0) * 100;

  // 根据油门位置和发动机转速计算燃油需求
  byte throttle_index = throttle_percent / 10; // 将油门百分比映射到索引（每 10% 作为一个级别）
  if (throttle_index > 10) { throttle_index = 10; } // 限制油门索引在 0 到 10 之间

  byte rpm_index = engine_rpm / 500; // 将发动机转速映射到索引（每 500 RPM 作为一个级别）
  if (rpm_index > 10) { rpm_index = 10; } // 限制转速索引在 0 到 10 之间

  // 从燃油表中查找燃油喷射持续时间并加上校正值
  inj_duration = fuel_table[throttle_index][rpm_index] + inj_trim;

  // 计算点火提前时间（以微秒为单位），从点火表中查找值并加上校正
  ign_advance_in_us = calculate_advance_from_deg(ignition_table[throttle_index][rpm_index] + ign_advance_trim_in_deg);

  // 计算当前发动机转速（RPM）
  engine_rpm = ((1000000 / (last_crank_duration * 36)) * 60);
}

// 发动机计算函数
void engine_calc() {
  // 检查当前发动机转速是否超出最大限制
  if (engine_rpm >= engine_rpm_max) {
    // 如果发动机没有被标记为超转状态
    if (!over_rev) {
      // 创建一个字节数组，表示超转速信号给上位机
      byte to_send[3] = {0x00, 0x01, 0x01};
      add_to_serial(to_send, 3); // 将信号添加到串行发送队列
      over_rev = true; // 设置发动机超转标记为 true
    }
  } else {
    // 如果当前发动机被标记为超转状态但已恢复正常
    if (over_rev) {
      // 创建一个字节数组，表示正常转速信号给上位机
      byte to_send[3] = {0x00, 0x00, 0x01};
      add_to_serial(to_send, 3); // 将信号添加到串行发送队列
      over_rev = false; // 设置发动机超转标记为 false
    }
  }

  // 如果信号已同步且没有超转状态
  if (signal_sync && !over_rev) {
    // 检查当前齿位置是否为曲轴最后两个齿之一
    if (current_tooth == tooth_count - 2) {
      // 计算上止点时间（tdc），此时曲轴应处于上止点
      unsigned long tdc = last_crank_time + (last_crank_duration * 7);
      current_coil = coila; // 设置当前点火线圈为 coila
      inja = inj1; // 设置当前喷油器为 inj1
      injb = inj5; // 设置备用喷油器为 inj5
      set_times(tdc); // 设置点火和喷油时间
    }
    // 如果当前齿位置为 11
    else if (current_tooth == 11) {
      unsigned long tdc = last_crank_time + (last_crank_duration * 7); // 计算上止点时间
      current_coil = coilc; // 设置当前点火线圈为 coilc
      inja = inj3; // 设置当前喷油器为 inj3
      injb = inj4; // 设置备用喷油器为 inj4
      set_times(tdc); // 设置点火和喷油时间
    }
    // 如果当前齿位置为 23
    else if (current_tooth == 23) {
      unsigned long tdc = last_crank_time + (last_crank_duration * 7); // 计算上止点时间
      current_coil = coilb; // 设置当前点火线圈为 coilb
      inja = inj2; // 设置当前喷油器为 inj2
      injb = inj6; // 设置备用喷油器为 inj6
      set_times(tdc); // 设置点火和喷油时间
    }
  }
}

// 设置点火和喷油时间
void set_times(unsigned long tdc) {
  // 计算点火开始时间
  // 点火开始时间 = 上止点时间（tdc） - 点火提前时间（ign_advance_in_us） - 点火持续时间（ign_dwell_time）
  ign_start = tdc - ign_advance_in_us - ign_dwell_time;

  // 计算点火结束时间
  // 点火结束时间 = 上止点时间（tdc） - 点火提前时间（ign_advance_in_us）
  ign_end = tdc - ign_advance_in_us;

  // 计算喷油开始时间
  // 喷油开始时间 = 上止点时间（tdc） - 喷油持续时间（inj_duration）
  inj_start = tdc - inj_duration;

  // 计算喷油结束时间
  // 喷油结束时间 = 上止点时间（tdc）
  inj_end = tdc;

  // 如果当前时间大于喷油开始时间
  // 根据当前时间（micros()）计算实际的喷油结束时间
  if (micros() > inj_start) {
    inj_end += micros() - inj_start;
  }
}

// 主循环函数
void loop() {
  // 如果最后一次曲轴转动的时间为0（表示发动机停止转动），将发动机转速设为0
  if (last_crank_duration == 0) {
    engine_rpm = 0;
  }

  // 检查是否有超过3个字节的数据从串口接收，如果是则调用get_serial()函数处理接收到的数据
  if (Serial.available() > 3) {
    get_serial();
  }
  // 如果没有足够的串口数据，但有等待发送的数据，调用send_serial()函数发送数据
  else if (bytes_waiting > 0) {
    send_serial();
  }

  // 如果信号同步成功，调用do_coils_and_inj()函数控制点火线圈和喷油器
  if (signal_sync) {
    do_coils_and_inj(micros());
  }
  // 如果信号不同步，调用press_the_panic_button()函数关闭所有线圈和喷油器
  else {
    press_the_panic_button();
  }

  // 如果当前齿轮的位置是倒数第二个齿（即最后一个完整的转动周期前的齿）
  if (current_tooth == tooth_count - 2) {
    // 获取输入传感器数据
    get_inputs();
    // 进行发动机相关的计算
    engine_calc();
  }
  // 如果当前齿轮的位置是第11个或第23个齿（这些是特定的时间点，通常用于参考点）
  else if (current_tooth == 11 || current_tooth == 23) {
    // 进行发动机相关的计算
    engine_calc();
  }
}


// 紧急情况处理函数，用于关闭所有线圈和喷油器
void press_the_panic_button() {
  // 将线圈A设为低电平，表示关闭
  digitalLow(coila);

  // 将线圈B设为低电平，表示关闭
  digitalLow(coilb);

  // 将线圈C设为低电平，表示关闭
  digitalLow(coilc);

  // 将喷油器1设为低电平，表示关闭
  digitalLow(inj1);

  // 将喷油器2设为低电平，表示关闭
  digitalLow(inj2);

  // 将喷油器3设为低电平，表示关闭
  digitalLow(inj3);

  // 将喷油器4设为低电平，表示关闭
  digitalLow(inj4);

  // 将喷油器5设为低电平，表示关闭
  digitalLow(inj5);

  // 将喷油器6设为低电平，表示关闭
  digitalLow(inj6);
}


// 控制点火线圈和喷油器的开关状态
bool coil_on = false;
bool inj_on = false;


// 函数：do_coils_and_inj
// 描述：根据当前时间控制点火和喷油的开闭状态
void do_coils_and_inj(unsigned long time_now) {
  // 点火控制
  if (time_now >= ign_start && time_now < ign_end) {
    if (!coil_on) { // 点火线圈未开启
      digitalHigh(current_coil); // 打开当前点火线圈
      coil_on = true; // 设置点火线圈状态为开启
    }
  }
  else {
    if (coil_on) { // 点火线圈已开启
      digitalLow(current_coil); // 关闭当前点火线圈
      coil_on = false; // 设置点火线圈状态为关闭
    }
  }

  // 喷油控制
  if (crank_toggle && time_now >= inj_start && time_now < inj_end) {
    if (!inj_on) { // 喷油器未开启
      digitalHigh(inja); // 打开喷油器A
      digitalHigh(injb); // 打开喷油器B
      inj_on = true; // 设置喷油器状态为开启
    }
  }
  else {
    if (inj_on) { // 喷油器已开启
      digitalLow(inja); // 关闭喷油器A
      digitalLow(injb); // 关闭喷油器B
      inj_on = false; // 设置喷油器状态为关闭
    }
  }
}

// 函数：get_serial
// 描述：接收串口命令，并根据命令类型执行相应操作
void get_serial() {
  // 从监控软件接收命令
  //
  // 0x00 - 请求数据
  //        后续字节表示请求的数据类型
  //        0x00 - 状态标志
  //        0x01 - 燃油喷射持续时间
  //        0x02 - 燃油修正
  //        0x03 - 点火提前角
  //        0x04 - 点火提前角修正
  //        0x05 - 节气门位置
  //        0x06 - 发动机转速
  //        0x07 - 最大发动机转速
  //        0x08 - 燃油表
  //        0x09 - 点火表
  // 0x01 - 调整燃油修正
  // 0x02 - 调整点火提前角
  // 0x03 - 调整最大转速限制
  byte input = Serial.read();
  if (input != 0xff) { return; } // 确保数据同步
  input = Serial.read();
  if (input == 0x00) { // 请求数据
    byte input1 = Serial.read();
    byte input2 = Serial.read();
    if (input2 == 0x00) { // 状态标志
      if (input1 == 0x00) { // 信号同步
        byte to_send[3] = {input2, byte(signal_sync), 0x00};
        add_to_serial(to_send, 3);
      }
      else if (input1 == 0x01) { // 超速
        byte to_send[3] = {input2, byte(over_rev), 0x01};
        add_to_serial(to_send, 3);
      }
    }
    else if (input2 == 0x01) { // 燃油喷射持续时间
      byte to_send[3] = {input2, byte(inj_duration), byte(inj_duration >> 8)};
      add_to_serial(to_send, 3);
    }
    else if (input2 == 0x02) { // 燃油修正
      byte to_send[3] = {input2, byte(inj_trim), byte(inj_trim >> 8)};
      add_to_serial(to_send, 3);
    }
    else if (input2 == 0x03) { // 点火提前角
      byte to_send[3] = {input2, byte(ign_advance_in_us), byte(ign_advance_in_us >> 8)};
      add_to_serial(to_send, 3);
    }
    else if (input2 == 0x04) { // 点火提前角修正
      byte to_send[3] = {input2, byte(ign_advance_trim_in_deg), byte(ign_advance_trim_in_deg >> 8)};
      add_to_serial(to_send, 3);
    }
    else if (input2 == 0x05) { // 节气门位置
      byte to_send[3] = {input2, byte(throttle_percent), byte(throttle_percent >> 8)};
      add_to_serial(to_send, 3);
    }
    else if (input2 == 0x06) { // 发动机转速
      byte to_send[3] = {input2, byte(engine_rpm), byte(engine_rpm >> 8)};
      add_to_serial(to_send, 3);
    }
    else if (input2 == 0x07) { // 最大发动机转速
      byte to_send[3] = {input2, byte(engine_rpm_max), byte(engine_rpm_max >> 8)};
      add_to_serial(to_send, 3);
    }
    else if (input2 == 0x08) { // 燃油表
      int total = 11*11;
      byte to_send[total] = {input2, byte(total), byte(total >> 8)};
      for(byte x = 0; x < 11; x++) {
        for (byte y = 0; y < 11; y++) {
          to_send[3 +(x*11)+y] = byte(fuel_table[x][y] / 100);
        }
      }
      add_to_serial(to_send, 3+total);
    }
    else if (input2 == 0x09) { // 点火表
      int total = 11*11;
      byte to_send[total] = {input2, byte(total), byte(total >> 8)};
      for(byte x = 0; x < 11; x++) {
        for (byte y = 0; y < 11; y++) {
          to_send[3 +(x*11)+y] = byte(ignition_table[x][y]);
        }
      }
      add_to_serial(to_send, 3+total);
    }
  }
  else if (input == 0x01) {
    // 调整燃油修正
    byte input2 = Serial.read();
    input2 = Serial.read();
    if (input2 == 0x00) {
      inj_trim -= 100;
    }
    else if (input2 == 0x01) {
      inj_trim += 100;
    }
    byte to_send[3] = {0x02, byte(inj_trim), byte(inj_trim >> 8)};
    add_to_serial(to_send, 3);
  }
  else if (input == 0x02) {
    // 调整点火提前角修正
    byte input2 = Serial.read();
    input2 = Serial.read();
    if (input2 == 0x00) {
      ign_advance_trim_in_deg -= 1;
    }
    else if (input2 == 0x01) {
      ign_advance_trim_in_deg += 1;
    }
    byte to_send[3] = {0x04, byte(ign_advance_trim_in_deg), byte(ign_advance_trim_in_deg >> 8)};
    add_to_serial(to_send, 3);
  }
  else if (input == 0x03) {
    // 调整最大转速限制
    byte input2 = Serial.read();
    input2 = Serial.read();
    if (input2 == 0x00) {
      engine_rpm_max -= 500;
    }
    else if (input2 == 0x01) {
      engine_rpm_max += 500;
    }
    byte to_send[3] = {0x07, byte(engine_rpm_max), byte(engine_rpm_max >> 8)};
    add_to_serial(to_send, 3);
  }
}

// 函数：add_to_serial
// 描述：将字节数组添加到串口发送缓冲区
void add_to_serial(byte bytes_to_add[], int amount) {
  s_buffer[bytes_waiting] = 0xff; // 起始字节标记
  bytes_waiting += 1;
  for (int b = 0; b < amount; b++) {
    s_buffer[bytes_waiting + b] = bytes_to_add[b];
  }
  bytes_waiting += amount;
}

// 函数：send_serial
// 描述：从缓冲区获取下一个字节并通过串口发送
void send_serial() {
  byte next_byte = s_buffer[0];
  Serial.write(next_byte); // 发送下一个字节
  for (int b = 0; b < sizeof(s_buffer) - 1; b++) {
    s_buffer[b] = s_buffer[b + 1]; // 左移缓冲区
  }
  bytes_waiting--; // 减少等待字节数
}
