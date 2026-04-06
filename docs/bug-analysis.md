# EcatV2_Master Bug 分析报告

> **分析日期：** 2026-04-06  
> **版本：** commit `634228c` (branch `copilot/debug-issues-in-ros2-package`)  
> **分析范围：** `src/soem_wrapper/` 下全部 `.cpp` / `.hpp` 文件

---

## 目录

1. [Bug #1 — SOEM API 多线程并发访问（严重）](#bug-1)
2. [Bug #2 — `init_task_list()` 未知任务类型时空指针解引用（严重）](#bug-2)
3. [Bug #3 — `setup_ecat()` 在 OPERATIONAL 状态未达成时仍返回 `true`（高危）](#bug-3)
4. [Bug #4 — 首次延迟测量值错误（高危）](#bug-4)
5. [Bug #5 — `DJI_MOTOR::on_command()` PDO 偏移错位（高危）](#bug-5)
6. [Bug #6 — `SlaveDevice::init()` 缓冲区 `resize()` 后重复 `clear()`（中危）](#bug-6)
7. [Bug #7 — `on_shutdown()` 可能永远阻塞（中危）](#bug-7)
8. [Bug #8 — `DM_MOTOR::on_command_mit_control()` 使用 `static` 局部缓冲区（中危）](#bug-8)
9. [Bug #9 — 实时循环中频繁创建 `rclcpp::Clock` 对象（中危）](#bug-9)
10. [Bug #10 — `system()` 调用忽略返回值且存在命令注入风险（中危）](#bug-10)
11. [Bug #11 — 同类任务的静态共享消息对象（低危/隐患）](#bug-11)
12. [Bug #12 — `TaskWrapper::shared_offset_` 跨实例共享（低危）](#bug-12)
13. [Bug #13 — 多处双分号（代码质量）](#bug-13)

---

## Bug #1 — SOEM API 多线程并发访问（严重）  {#bug-1}

### 涉及文件
- `src/soem_wrapper/src/ecat_node.cpp`：`datacycle_callback()`、`state_check_callback()`

### 问题描述

`EthercatNode` 启动了两个独立线程：
- **`data_thread_`**（运行 `datacycle_callback`）：每个控制周期调用 SOEM 的帧收发接口。
- **`checker_thread_`**（运行 `state_check_callback`）：周期检测从站状态，在异常时调用 SOEM 恢复接口。

两个线程**没有任何互斥锁保护**地同时访问 SOEM 全局上下文（`ecx_context`）：

| `datacycle_callback` 调用的 SOEM API | `state_check_callback` 调用的 SOEM API |
|---|---|
| `ec_receive_processdata()` | `ec_readstate()` |
| `ec_send_processdata()` | `ec_reconfig_slave()` |
| — | `ec_writestate()` |
| — | `ec_statecheck()` |
| — | `ec_recover_slave()` |

**SOEM 的官方文档明确说明该库不是线程安全的。** 所有 SOEM 函数共享同一个全局 `ecx_context` 结构体，在没有外部同步的情况下并发调用会导致：

- EtherCAT 数据帧损坏（部分帧被覆写）
- SOEM 内部状态机出现非法状态转换
- 潜在的内存越界写入
- 电机收到错误指令（赛场上最危险的后果）

### 复现逻辑

只需系统正常运行并触发一次 EtherCAT 连接中断（如从站掉电或网线抖动），即可触发该问题。此时 `state_check_callback` 开始调用 `ec_readstate`/`ec_reconfig_slave`，与同时在运行的 `datacycle_callback` 产生竞争。

### 建议修复

**方案 A（推荐）**：引入一把全局 SOEM API 互斥锁（`std::mutex soem_api_mtx_`），所有 SOEM API 调用均加锁：

```cpp
// ecat_node.hpp 中添加
std::mutex soem_api_mtx_;

// datacycle_callback 中
{
    std::lock_guard lock(soem_api_mtx_);
    wkc_ = ec_receive_processdata(100);
}
// ... 数据处理 ...
{
    std::lock_guard lock(soem_api_mtx_);
    ec_send_processdata();
}

// state_check_callback 中所有 SOEM 调用同样加锁
{
    std::lock_guard lock(soem_api_mtx_);
    ec_readstate();
}
```

**方案 B**：使用单线程方案，将状态检测逻辑集成到 `datacycle_callback` 中（每 N 个周期执行一次状态检测），彻底消除并发。

---

## Bug #2 — `init_task_list()` 未知任务类型时空指针解引用（严重）  {#bug-2}

### 涉及文件
- `src/soem_wrapper/src/soem_backend.cpp`：`SlaveDevice::init_task_list()`

### 问题代码

```cpp
// soem_backend.cpp ~ line 150
std::unique_ptr<task::TaskWrapper> task_wrapper{};  // 初始化为 nullptr

switch (task_type) {
    case task::DJIRC_APP_ID: {
        task_wrapper = std::make_unique<task::dbus_rc::DBUS_RC>();
        break;
    }
    // ... 其他已知类型 ...
    default: {
        RCLCPP_ERROR(*get_cfg_logger(), "Unknown task type = %d", task_type);
        // ⚠️ task_wrapper 仍然是 nullptr！
    }
}

// 下一行：无条件解引用 task_wrapper
task_wrapper->init_sdo(arg_buf_.data(), &arg_buf_idx, index_,
                       fmt::format("sn{}_app_{}_", sn_, app_idx));
// ☠️ 如果 task_type 未知，这里会立即发生 null pointer dereference，程序崩溃
```

### 问题描述

当 YAML 配置文件中填入了尚未被代码支持的 `task_type` 值（例如开发中的新任务、配置错误），`switch` 的 `default` 分支只打印一条 ERROR 日志，但 `task_wrapper` 指针保持为 `nullptr`。随后对其调用 `init_sdo()` 会立即引发段错误（Segmentation Fault），程序崩溃退出。

### 复现逻辑

在 YAML 配置文件中，将某个 task 的 `sdowrite_task_type` 设为任何未在 `switch` 中列出的值（如 `99`），启动节点，即刻崩溃。

### 建议修复

在 `default` 分支中直接返回 `false`，阻止继续执行：

```cpp
default: {
    RCLCPP_ERROR(*get_cfg_logger(), "Unknown task type = %d for slave %d, task %d",
                 task_type, index_, app_idx);
    return false;  // 阻止继续执行
}
```

并在 `switch` 之后添加空指针检查：

```cpp
if (!task_wrapper) {
    RCLCPP_ERROR(*get_cfg_logger(), "task_wrapper is null for slave %d, aborting", index_);
    return false;
}
task_wrapper->init_sdo(...);
```

---

## Bug #3 — `setup_ecat()` 在 OPERATIONAL 状态未达成时仍返回 `true`（高危）  {#bug-3}

### 涉及文件
- `src/soem_wrapper/src/ecat_node.cpp`：`EthercatNode::setup_ecat()`

### 问题代码

```cpp
// ecat_node.cpp 末段
if (ec_slave[0].state == EC_STATE_OPERATIONAL) {
    RCLCPP_INFO(*logging::get_cfg_logger(), "Operational state reached for all slaves.");
    data_thread_ = std::thread(&EthercatNode::datacycle_callback, this);
    checker_thread_ = std::thread(&EthercatNode::state_check_callback, this);
}
// ⚠️ 无论是否到达 OPERATIONAL，都 return true
return true;
```

调用方：

```cpp
// soem_backend.cpp main run()
if (get_node()->setup_ecat()) {
    RCLCPP_INFO(*get_sys_logger(), "Initialization succeeded");
    rclcpp::on_shutdown([&] { get_node()->on_shutdown(); });
    rclcpp::spin(get_node());  // 即便没有启动数据线程，也会进入 spin
}
```

### 问题描述

如果经过 50 次尝试后从站仍未达到 `EC_STATE_OPERATIONAL`（如 EtherCAT 拓扑错误、固件问题、连接不稳定），`data_thread_` 和 `checker_thread_` 都**不会被启动**，但 `setup_ecat()` 依然返回 `true`。结果：

- 日志输出 "Initialization succeeded"，具有误导性
- `rclcpp::spin()` 正常执行，节点看起来在运行
- 实际上没有任何 EtherCAT 数据被收发
- 后续 `on_shutdown()` 中会等待 `exiting_reset_called_`，但数据线程从未启动，导致永远阻塞（与 Bug #7 形成复合问题）

### 建议修复

```cpp
if (ec_slave[0].state != EC_STATE_OPERATIONAL) {
    RCLCPP_ERROR(*logging::get_cfg_logger(),
                 "Failed to reach OPERATIONAL state. Aborting.");
    return false;
}
RCLCPP_INFO(*logging::get_cfg_logger(), "Operational state reached for all slaves.");
data_thread_ = std::thread(&EthercatNode::datacycle_callback, this);
checker_thread_ = std::thread(&EthercatNode::state_check_callback, this);
return true;
```

---

## Bug #4 — 首次延迟测量值错误（高危）  {#bug-4}

### 涉及文件
- `src/soem_wrapper/src/soem_backend.cpp`：`SlaveDevice::process_pdo()`

### 问题代码

```cpp
// wrapper.hpp 成员变量
rclcpp::Time last_latency_check_packet_send_time_{};  // 默认值 = epoch (0 纳秒)

// soem_backend.cpp process_pdo()
void SlaveDevice::process_pdo(const rclcpp::Time &current_time) {
    if (slave_status_ == master_status_) {
        // ⚠️ 第一次匹配时：current_time - 0 = 自 epoch 以来的纳秒数（巨大的值）
        std_msgs_float32_shared_msg.data =
            static_cast<float>((current_time - last_latency_check_packet_send_time_).seconds() * 1000.f);
        // ...
        latency_publisher_->publish(std_msgs_float32_shared_msg);
    }
}
```

### 问题描述

`last_latency_check_packet_send_time_` 通过默认构造初始化为 `rclcpp::Time(0, 0, RCL_ROS_TIME)`（即 Unix 纪元时间 0）。当系统第一次检测到 `slave_status_ == master_status_` 时，延迟计算为 `current_time - 0`，结果约为自 1970 年 1 月 1 日至今的毫秒数（数十亿毫秒）。

该错误值会发布到延迟 topic，可能触发上层控制算法的错误判断（如将延迟阈值超标误认为通信故障）。

### 复现逻辑

任意启动系统，查看第一条 `/latency` topic 消息，其值会异常巨大，之后恢复正常。

### 建议修复

在 `SlaveDevice::process_pdo()` 中引入一个标志位跳过首次测量，或在首次匹配时仅初始化时间戳而不发布：

```cpp
void SlaveDevice::process_pdo(const rclcpp::Time &current_time) {
    if (slave_status_ == master_status_) {
        if (last_latency_check_packet_send_time_.nanoseconds() != 0) {
            // 只在 send_time 已被合法初始化后才计算并发布
            std_msgs_float32_shared_msg.data =
                static_cast<float>((current_time - last_latency_check_packet_send_time_).seconds() * 1000.f);
            current_data_stamp_ = last_latency_check_packet_send_time_;
            latency_publisher_->publish(std_msgs_float32_shared_msg);
        } else {
            // 首次：仅记录时间戳
            current_data_stamp_ = current_time;
        }
        last_latency_check_packet_send_time_ = current_time;

        if (master_status_ >= 250) {
            master_status_ = MASTER_READY + 2;
        } else {
            master_status_++;
        }
        for (const auto &task: task_list_) {
            if (task->has_publishers()) {
                task->read();
            }
        }
    }
}
```

---

## Bug #5 — `DJI_MOTOR::on_command()` PDO 偏移错位（高危）  {#bug-5}

### 涉及文件
- `src/soem_wrapper/src/tasks/dji_motor.cpp`

### 问题代码

```cpp
// dji_motor.cpp - init_value()：无条件写入 4 个电机槽位
void DJI_MOTOR::init_value() {
    int offset = pdowrite_offset_;
    write_uint8(0, ..., &offset);   // motor1 enable
    write_int16(0, ..., &offset);   // motor1 cmd
    write_uint8(0, ..., &offset);   // motor2 enable
    write_int16(0, ..., &offset);   // motor2 cmd
    write_uint8(0, ..., &offset);   // motor3 enable
    write_int16(0, ..., &offset);   // motor3 cmd
    write_uint8(0, ..., &offset);   // motor4 enable
    write_int16(0, ..., &offset);   // motor4 cmd
}

// dji_motor.cpp - on_command()：仅写入已启用的电机（跳过未启用的槽位！）
void DJI_MOTOR::on_command(const custom_msgs::msg::WriteDJIMotor::SharedPtr msg) const {
    int offset = pdowrite_offset_;
    if (is_motor_enabled[0]) {
        write_uint8(msg->motor1_enable, ..., &offset);
        write_int16(msg->motor1_cmd, ..., &offset);
    }
    if (is_motor_enabled[1]) {
        // ⚠️ 如果 motor0 未启用，offset 没有前进 3 字节，
        //    motor2 的数据被写入 motor1 的位置
        write_uint8(msg->motor2_enable, ..., &offset);
        write_int16(msg->motor2_cmd, ..., &offset);
    }
    // ...以此类推
}
```

### 问题描述

从站固件根据 SDO 配置阶段收到的参数分配 PDO 内存空间，并期望在 PDO 区的固定偏移处读取每个电机的使能位和指令值。`init_value()` 总是写满 4 个电机的数据（共 12 字节），与固件期望一致。但 `on_command()` 在遇到未启用的电机时**不前进偏移量**，导致后续已启用电机的数据被写入错误的位置。

**示例**：若配置了电机 1、3（motor0 和 motor2 已启用，motor1 和 motor3 未启用）：

| 写入操作 | 固件期望地址 | 实际写入地址 |
|---|---|---|
| motor1_enable + motor1_cmd | offset+0 ~ offset+2 | offset+0 ~ offset+2 ✓ |
| motor3_enable + motor3_cmd | offset+6 ~ offset+8 | offset+3 ~ offset+5 ✗ |

固件会把 motor3 的指令当作 motor2 的指令处理，motor3 的槽位保持上一次的值（初始化时是 0）。

### 建议修复

`on_command()` 中对所有电机槽位都前进偏移，无论是否启用：

```cpp
void DJI_MOTOR::on_command(const custom_msgs::msg::WriteDJIMotor::SharedPtr msg) const {
    std::lock_guard lock(slave_device_->mtx_);
    int offset = pdowrite_offset_;

    // 方法一：始终写入，但未启用的电机写 0
    write_uint8(is_motor_enabled[0] ? msg->motor1_enable : 0, ..., &offset);
    write_int16(is_motor_enabled[0] ? msg->motor1_cmd : 0,    ..., &offset);
    write_uint8(is_motor_enabled[1] ? msg->motor2_enable : 0, ..., &offset);
    write_int16(is_motor_enabled[1] ? msg->motor2_cmd : 0,    ..., &offset);
    // ...
}
```

---

## Bug #6 — `SlaveDevice::init()` 缓冲区 `resize()` 后重复 `clear()`（中危）  {#bug-6}

### 涉及文件
- `src/soem_wrapper/src/soem_backend.cpp`：`SlaveDevice::init()`

### 问题代码

```cpp
// soem_backend.cpp ~ line 102
if (oldSn == 0) {
    master_to_slave_buf_.clear();
    master_to_slave_buf_.resize(get_node()->get_device_master_to_slave_buf_len(...)); // size=N
    master_to_slave_buf_len_ = get_node()->get_device_master_to_slave_buf_len(...);
    master_to_slave_buf_.clear();  // ⚠️ size 重新变为 0！

    slave_to_master_buf_.clear();
    slave_to_master_buf_.resize(get_node()->get_device_slave_to_master_buf_len(...)); // size=M
    slave_to_master_buf_len_ = get_node()->get_device_slave_to_master_buf_len(...);
    slave_to_master_buf_.clear();  // ⚠️ size 重新变为 0！
}
```

### 问题描述

`std::vector::clear()` 将 `size()` 置为 0，但**保留已分配的内存（`capacity()` 不变）**。在 `resize(N)` 之后调用 `clear()` 导致：

- `master_to_slave_buf_.size() == 0`，但 `master_to_slave_buf_len_ == N`
- 所有通过 `.data()` + 手动 `len` 做的 `memcpy` 仍然正常工作（访问的是 capacity 内的内存）
- 但任何依赖 `.size()` 的代码（如 `recover_master_to_slave_buf()` 中对 backup 的检查、未来的迭代等）可能出现意外行为
- 主要风险：后续如果对这两个 vector 调用 `push_back()`，会从下标 0 开始写（size=0），覆盖现有数据

显然程序员的本意是先 `clear()` 再 `resize(N)` 后保留大小为 N 的零初始化 buffer，但错误地在 `resize` 之后又调用了 `clear()`。

### 建议修复

删除 `resize()` 之后多余的 `clear()` 调用：

```cpp
if (oldSn == 0) {
    master_to_slave_buf_.assign(
        get_node()->get_device_master_to_slave_buf_len(ec_slave[index_].eep_id), 0);
    master_to_slave_buf_len_ = master_to_slave_buf_.size();

    slave_to_master_buf_.assign(
        get_node()->get_device_slave_to_master_buf_len(ec_slave[index_].eep_id), 0);
    slave_to_master_buf_len_ = slave_to_master_buf_.size();
}
```

使用 `assign(N, 0)` 一步完成大小设置和清零，语义更清晰。

---

## Bug #7 — `on_shutdown()` 可能永远阻塞（中危）  {#bug-7}

### 涉及文件
- `src/soem_wrapper/src/ecat_node.cpp`：`EthercatNode::on_shutdown()`

### 问题代码

```cpp
void EthercatNode::on_shutdown() {
    RCLCPP_INFO(*logging::get_sys_logger(), "Shutting down");

    exiting_ = true;
    // ⚠️ 无限循环等待数据线程设置 exiting_reset_called_
    while (!exiting_reset_called_) {
        rclcpp::sleep_for(std::chrono::milliseconds(10));
    }
    // ...
    running_ = false;
    if (data_thread_.joinable()) data_thread_.join();
    // ...
}
```

### 问题描述

`on_shutdown()` 通过忙等待等待 `data_thread_` 在 `datacycle_callback` 中设置 `exiting_reset_called_ = true`。以下情况均会导致永远阻塞：

1. **Bug #3 触发后**：从站未到达 OPERATIONAL，`data_thread_` 从未启动，`exiting_reset_called_` 永远为 false。
2. **数据线程中途崩溃或异常退出**（如 Bug #1 引发的内存损坏）：线程已退出但没有设置 `exiting_reset_called_`。
3. **系统收到 SIGTERM/SIGKILL** 时，ROS2 shutdown 钩子调用 `on_shutdown()`，数据线程尚未响应 `exiting_` 标志。

结果：ROS2 节点无法正常退出，需要强制 kill 进程。赛场上这可能导致电机无法安全停止。

### 建议修复

增加超时机制：

```cpp
void EthercatNode::on_shutdown() {
    exiting_ = true;
    const int max_wait_ms = 2000;  // 最多等待 2 秒
    int waited_ms = 0;
    while (!exiting_reset_called_ && waited_ms < max_wait_ms) {
        rclcpp::sleep_for(std::chrono::milliseconds(10));
        waited_ms += 10;
    }
    if (!exiting_reset_called_) {
        RCLCPP_ERROR(*logging::get_sys_logger(),
                     "Timeout waiting for reset command, forcing shutdown");
    }
    running_ = false;
    // ...
}
```

---

## Bug #8 — `DM_MOTOR::on_command_mit_control()` 使用 `static` 局部缓冲区（中危）  {#bug-8}

### 涉及文件
- `src/soem_wrapper/src/tasks/dm_motor.cpp`

### 问题代码

```cpp
void DM_MOTOR::on_command_mit_control(
    custom_msgs::msg::WriteDmMotorMITControl::SharedPtr msg) const
{
    std::lock_guard lock(slave_device_->mtx_);
    int offset = pdowrite_offset_;
    write_uint8(msg->enable, ..., &offset);

    static uint8_t WriteDmMotorMITControl_data[8] = {};  // ⚠️ static 局部数组！

    // 计算并写入 WriteDmMotorMITControl_data ...
    WriteDmMotorMITControl_data[0] = writeDmMotorMITControl_pos >> 8;
    // ...

    memcpy(slave_device_->get_master_to_slave_buf().data() + offset,
           WriteDmMotorMITControl_data, 8);
}
```

### 问题描述

`WriteDmMotorMITControl_data` 是一个 `static` 局部数组，在所有 `DM_MOTOR` 实例之间共享同一块内存。若有多个从站都配置了 DM 电机任务，且使用 ROS2 **多线程执行器**（`MultiThreadedExecutor`），多个订阅者回调可能并发触发，导致：

- 线程 A 写入 `WriteDmMotorMITControl_data` 的过程中被线程 B 打断
- 线程 B 将 B 自己的数据写入同一块 buffer
- 线程 A 从 buffer 中拷贝到 slave A 的 PDO 时，读到的是 B 的数据

`slave_device_->mtx_` 只保护了各自从站的 PDO buffer，**无法保护这个跨实例共享的 `static` 局部数组**。

### 建议修复

将 `static` 改为栈上分配的本地变量：

```cpp
uint8_t WriteDmMotorMITControl_data[8] = {};  // 移除 static
```

---

## Bug #9 — 实时循环中频繁创建 `rclcpp::Clock` 对象（中危）  {#bug-9}

### 涉及文件
- `src/soem_wrapper/src/ecat_node.cpp`：`datacycle_callback()`
- 各任务文件的 `publish_empty_message()` 方法

### 问题代码

```cpp
// datacycle_callback() 中，每个控制周期均执行
current_time = rclcpp::Clock().now();  // ⚠️ 每次创建新的 Clock 对象

// publish_empty_message() 中
custom_msgs_readdjimotor_shared_msg.header.stamp = rclcpp::Clock().now();  // ⚠️ 同上
```

### 问题描述

`rclcpp::Clock` 构造函数涉及系统调用（查询时钟类型、注册等），在实时控制循环（目标周期 < 1ms）中每次迭代都创建一个临时 `Clock` 对象会引入不必要的延迟和内存分配抖动，影响实时性。此问题也导致代码绕过了 Node 内置的时钟（`get_node()->get_clock()`），在仿真模式下时钟源不一致。

### 建议修复

在 `datacycle_callback` 开始处用 Node 的时钟一次性获取时间，或创建一个成员变量 `rclcpp::Clock` 复用：

```cpp
// 在 EthercatNode 中添加
std::shared_ptr<rclcpp::Clock> rt_clock_;  // 在构造函数中初始化

// datacycle_callback 中
current_time = rt_clock_->now();

// publish_empty_message 应接受 stamp 参数，而不是自行创建 Clock
```

---

## Bug #10 — `system()` 调用忽略返回值且存在命令注入风险（中危）  {#bug-10}

### 涉及文件
- `src/soem_wrapper/src/utils/sys_utils.cpp`：`move_threads()`

### 问题代码

```cpp
void move_threads(const int cpu_id, const std::string &cpu_list, const std::string &nic_name) {
    // ...
    std::string taskset_cmd = "sudo taskset -cp " + cpu_list + " " + std::to_string(pid) +
                              " > /dev/null 2>&1";
    system(taskset_cmd.c_str());  // ⚠️ 返回值被丢弃；cpu_list 来自用户参数
}
```

### 问题描述

1. **命令注入**：`cpu_list` 来自 ROS2 参数 `non_rt_cpus`。若该参数被设置为包含 shell 特殊字符的值（如 `; rm -rf /`），`system()` 会执行任意命令。赛场上如果有人能控制参数源（如通过 ROS2 参数服务器），这是一个安全漏洞。

2. **静默失败**：`system()` 的返回值被丢弃。如果 `taskset` 不可用（未安装），或 `sudo` 需要密码，命令静默失败，实时性保证无法达成，但程序不会告警。

### 建议修复

- 对 `cpu_list` 做严格格式验证（仅允许数字、逗号、连字符）。
- 检查 `system()` 返回值并记录警告。
- 更安全的方案：使用 `execvp` + `fork`，避免 shell 注入。

```cpp
// 参数验证示例
if (!std::regex_match(cpu_list, std::regex("^[0-9,\\-]+$"))) {
    RCLCPP_ERROR(*get_sys_logger(), "Invalid cpu_list format: %s", cpu_list.c_str());
    return;
}
int ret = system(taskset_cmd.c_str());
if (ret != 0) {
    RCLCPP_WARN(*get_sys_logger(), "taskset command failed with code %d: %s",
                ret, taskset_cmd.c_str());
}
```

---

## Bug #11 — 同类任务的静态共享消息对象（低危/隐患）  {#bug-11}

### 涉及文件
- `src/soem_wrapper/include/soem_wrapper/task_defs.hpp` 及各任务 `.cpp`

### 问题描述

所有任务类（`DBUS_RC`、`DJI_MOTOR`、`LK_MOTOR`、`DM_MOTOR`、`HIPNUC_IMU_CAN`、`SBUS_RC`、`SUPER_CAP`、`VT13_RC`）的 ROS2 消息对象都声明为 `static` 类成员：

```cpp
// 所有 DJI_MOTOR 实例共用同一块消息内存
static custom_msgs::msg::ReadDJIMotor custom_msgs_readdjimotor_shared_msg;
```

以及 `TaskWrapper` 基类中的偏移量：

```cpp
inline static int shared_offset_ = 0;  // 所有 TaskWrapper 子类实例共用！
```

**当前的串行处理机制**保证了同一时刻只有一个任务的 `read()` 在运行，所以现在不会出错。但这是一个**隐患**：

- 若未来将同一从站上的多个任务并行处理，所有同类任务会竞争同一块消息内存。
- `shared_offset_` 在不同类型任务之间也是共享的（`inline static` 在基类中定义），如果两个不同类型任务并发调用 `read()`，偏移量会相互干扰。
- 另外，`SlaveDevice` 也有 `inline static std_msgs::msg::Float32 std_msgs_float32_shared_msg{}`，同样是所有从站共用。

### 建议修复

将消息对象改为**实例成员变量**而非静态成员：

```cpp
// 修改前
static custom_msgs::msg::ReadDJIMotor custom_msgs_readdjimotor_shared_msg;

// 修改后
custom_msgs::msg::ReadDJIMotor msg_{};  // 每个实例独立
```

将 `shared_offset_` 改为局部变量：

```cpp
// 各 read() 方法中
int local_offset = pdoread_offset_;
// 使用 local_offset 替代 shared_offset_
```

---

## Bug #12 — `TaskWrapper::shared_offset_` 跨实例共享（低危）  {#bug-12}

见 [Bug #11](#bug-11) 中关于 `shared_offset_` 的分析，问题性质相同，此处不重复。

---

## Bug #13 — 多处双分号（代码质量）  {#bug-13}

### 涉及文件

```
src/soem_wrapper/src/tasks/dm_motor.cpp
src/soem_wrapper/src/tasks/sbus_rc.cpp
src/soem_wrapper/src/tasks/hipnuc_imu_can.cpp
src/soem_wrapper/src/ecat_node.cpp (datacycle_callback 中)
```

### 问题代码示例

```cpp
// dm_motor.cpp
custom_msgs_readdmmotor_shared_msg.header.stamp = slave_device_->get_current_data_stamp();;
//                                                                                          ^^

// hipnuc_imu_can.cpp
sensor_msgs_imu_shared_msg.header.stamp = slave_device_->get_current_data_stamp();;
```

虽然多余的分号在 C++ 中被视为空语句不影响功能，但应修复以保持代码整洁，避免未来 linter 报警掩盖真正问题。

---

## 附：静态 SDO 变量的线程安全隐患

### 涉及文件
- `src/soem_wrapper/include/soem_wrapper/wrapper.hpp`

```cpp
// SlaveDevice 中的 inline static 成员（所有从站实例共享！）
inline static int sdo_sn_size_ptr_ = 4;
inline static int sdo_rev_size_ptr_ = 3;
inline static char sw_rev_str_[4]{};
inline static uint16_t sdo_size_write_ptr_ = 0;
```

这些变量在 `SlaveDevice::init()` 中被修改，而 `init()` 由 SOEM 的 `config_ec_slave` 回调触发。正常初始化时 SOEM 顺序调用，不存在并发。但当 `state_check_callback` 调用 `ec_reconfig_slave()` 时（可能对多个从站并发重配置），这些静态变量会被竞争访问。建议将它们改为本地变量：

```cpp
// 在 init() 方法内部改为局部变量
int sdo_sn_size = 4;
int sdo_rev_size = 3;
char sw_rev_str[4]{};
uint16_t sdo_size_write = 0;
```

---

## 问题优先级汇总

| # | 文件 | 严重程度 | 赛场影响 |
|---|---|---|---|
| 1 | ecat_node.cpp | 🔴 严重 | 电机收到错误指令，EtherCAT 帧损坏 |
| 2 | soem_backend.cpp | 🔴 严重 | 配置文件错误时程序立即崩溃 |
| 3 | ecat_node.cpp | 🟠 高危 | 从站启动失败时节点假装成功，实际不工作 |
| 4 | soem_backend.cpp | 🟠 高危 | 首次延迟测量值异常，可能触发上层误判 |
| 5 | dji_motor.cpp | 🟠 高危 | 部分电机被禁用时其余电机接收到错误指令 |
| 6 | soem_backend.cpp | 🟡 中危 | buffer size 为 0，潜在边界问题 |
| 7 | ecat_node.cpp | 🟡 中危 | 关机时可能永远阻塞，电机无法安全停止 |
| 8 | dm_motor.cpp | 🟡 中危 | 多线程执行器下 DM 电机指令错乱 |
| 9 | ecat_node.cpp | 🟡 中危 | 实时循环抖动增大 |
| 10 | sys_utils.cpp | 🟡 中危 | 命令注入安全漏洞，系统调用失败被忽略 |
| 11-12 | task_defs.hpp | 🟢 低危 | 并行化后会出现数据竞争 |
| 13 | 多文件 | ⚪ 代码质量 | 无功能影响 |

---

*此报告由 GitHub Copilot 自动生成，基于静态代码分析。建议在修复后结合实机测试验证。*
