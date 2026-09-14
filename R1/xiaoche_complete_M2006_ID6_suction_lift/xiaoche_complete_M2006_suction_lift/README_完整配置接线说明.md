# STM32F405 三全向轮小车完整工程（BTMCU bool版）

本工程已经把下列已经分别测试成功的功能合并到同一个 STM32F405 工程中：

1. 3 个 C620 + 3 个 M3508 三全向轮底盘，三个电机分别使用独立 PID。
2. 3 个 C610 + 3 个 M2006：夹爪采用固定角度闭环，夹爪升降和吸盘升降均采用目标位置闭环与重力补偿保持。
3. L298N 控制吸气泵。
4. PB0 控制大功率 MOS 模块和电磁阀气缸。
5. PA6/TIM3_CH1 控制 SG90 围栏舵机。
6. HC-05 通过 USART2 接收 BTMCU 专业模式的 8 字节数据包。
7. C610 ID6 + M2006 控制吸盘升降，按住移动、松手保持目标位置。

主控为 `STM32F405RGT6`，外部晶振 8 MHz，系统时钟 168 MHz，HC-05 串口为 115200、8N1。

> 本版本已经取消原来的字母命令和 6 字节底盘协议。手机端必须发送本说明中的 8 字节 BTMCU 数据包。

前 9 个 bool 的位顺序与提供的 `R1.zip` 一致；R1 中可用的手机按键布局可以继续使用。区别是本工程不再让夹爪以固定电流一直转，而是利用 C610 返回的 M2006 编码器位置做多圈角度闭环。

> **上电前必须把夹爪放在完全张开基准位，并把两套升降都放在最低基准位。** 本工程没有原点开关，首次收到 ID4/ID5/ID6 编码器反馈时会把当时的位置当作相对零点。

## 一、工程文件

- CubeMX 配置：`xiaoche_complete.ioc`
- Keil 工程：`MDK-ARM/xiaoche_complete.uvprojx`
- 主要控制代码：`Core/Src/main.c`
- 引脚定义：`Core/Inc/main.h`
- 手机端当前配置：`BTMCU/调试工程_H_20260731-203123_M2006吸盘升降.pro`
- 该配置已经包含bool0～bool13；bool12/13继续作为吸盘升降上/下按键。

## 二、最终引脚分配

| 功能 | STM32 引脚 | 连接对象 |
| --- | --- | --- |
| HC-05 TX | PA2 / USART2_TX | 接 HC-05 RXD |
| HC-05 RX | PA3 / USART2_RX | 接 HC-05 TXD |
| SG90 PWM | PA6 / TIM3_CH1 | 接舵机橙色或黄色信号线 |
| CAN1 RX | PA11 / CAN1_RX | 板载 CAN 收发器内部连接 |
| CAN1 TX | PA12 / CAN1_TX | 板载 CAN 收发器内部连接 |
| 电磁阀 MOS 控制 | PB0 / `VALVE_SIG` | 接 MOS 模块 `HIGH/PWM` 输入 |
| 吸气泵方向 1 | PB1 / `PUMP_IN1` | 接 L298N IN1 |
| 吸气泵方向 2 | PB2 / `PUMP_IN2` | 接 L298N IN2 |
| ST-Link SWDIO | PA13 | 接 ST-Link SWDIO |
| ST-Link SWCLK | PA14 | 接 ST-Link SWCLK |

原520使用的PA5、PB10、PB11已经释放，不再连接吸盘升降。气泵和电磁阀仍默认关闭。

## 三、6个电调的CAN ID

所有电调共用同一对 CANH/CANL，但 ID 不能重复。

| 电调ID | 电调与电机 | 用途 | 反馈ID | STM32控制报文位置 |
| ---: | --- | --- | --- | --- |
| 1 | C620 + M3508 | 底盘1号轮 | `0x201` | `0x200` 的 DATA[0..1] |
| 2 | C620 + M3508 | 底盘2号轮 | `0x202` | `0x200` 的 DATA[2..3] |
| 3 | C620 + M3508 | 底盘3号轮 | `0x203` | `0x200` 的 DATA[4..5] |
| 4 | C610 + M2006 | 夹爪张开/夹紧 | `0x204` | `0x200` 的 DATA[6..7] |
| 5 | C610 + M2006 | 夹爪上升/下降 | `0x205` | `0x1FF` 的 DATA[0..1] |
| 6 | C610 + M2006 | 吸盘上升/下降 | `0x206` | `0x1FF` 的 DATA[2..3] |

C610/C620 重新上电后，通常可以通过绿色状态灯每轮闪烁次数判断 ID。6 个电调应分别显示 1、2、3、4、5、6。ID 设置方法以你所用电调版本的说明书为准。

注意：

- ID4 必须是夹爪 C610，因为它占用 `0x200` 的第4路电流命令。
- ID5 必须是升降 C610，因为 ID5～8 使用 `0x1FF` 控制报文。
- 新增吸盘升降 C610 必须设为ID6，对应 `0x1FF` 的第2路电流命令。
- 同一条 CAN 总线上只要出现重复 ID，对应电调反馈和控制就会冲突。
- CANH 只能接 CANH，CANL 只能接 CANL。

## 四、详细接线

### 1. 整车供电原则

建议按电压等级分开供电：

| 负载 | 建议供电 |
| --- | --- |
| STM32开发板 | 按开发板要求使用 5V 或 USB，禁止直接接24V |
| HC-05带底板模块 | 稳定5V |
| SG90 | 独立稳定5V，建议预留至少1A电流 |
| C620/C610 | 按电调要求使用动力电源，常用为24V |
| 气泵 | 严格按照气泵铭牌额定电压 |
| 电磁阀 | 严格按照电磁阀线圈额定电压 |

STM32、HC-05、舵机电源、L298N逻辑侧和MOS控制侧必须共地。动力电流不能经过STM32小排针或面包板。

### 2. HC-05

```text
STM32 PA2 / USART2_TX  -> HC-05 RXD
STM32 PA3 / USART2_RX  <- HC-05 TXD
STM32 GND              -> HC-05 GND
稳定5V                  -> 带稳压底板的 HC-05 VCC
```

- PA2和PA3必须交叉连接。
- 常见带稳压底板的HC-05可以给VCC输入5V；裸核心板应按模块说明使用3.3V。
- HC-05的串口波特率必须是115200；手机蓝牙无线连接本身不需要再选择波特率。
- 不要把24V接入HC-05。

### 3. CAN总线和6个电调

如果你的开发板与照片/原成功工程一样已经引出 CANH/CANL，说明板上已经有 CAN 收发器，不再外接第二个收发器：

```text
开发板 CANH -> CAN集线器 CANH -> 6个电调的 CANH
开发板 CANL -> CAN集线器 CANL -> 6个电调的 CANL
开发板/收发器 GND ------------> CAN系统参考地
```

每个电调还要完成：

- C620连接对应M3508的三相动力线和传感器线。
- C610连接对应M2006的三相动力线和传感器线。
- 电调动力正负极接动力电源分配板，极性不能反接。
- 总线物理两端各保留一个120Ω终端电阻，中间节点不要重复并联终端电阻。
- 断电测量CANH与CANL之间的电阻，两个120Ω并联时通常约为60Ω。

如果开发板只有PA11/PA12而没有CANH/CANL，就必须增加3.3V逻辑CAN收发器；不能把PA11/PA12直接接到电调CANH/CANL。

### 4. SG90围栏舵机

```text
SG90 橙/黄色信号线 -> PA6
SG90 红线           -> 外部稳定5V
SG90 棕/黑线        -> 外部5V GND
外部5V GND          -> STM32 GND
```

- 不要从STM32的3.3V脚给SG90供电。
- 第一次测试先拆掉机械连杆，确认上下方向和脉宽，再安装围栏。
- 舵机抖动或STM32复位，通常是5V电源电流不足或没有共地。

### 5. L298N和吸气泵

```text
PB1                 -> L298N IN1
PB2                 -> L298N IN2
L298N OUT1          -> 气泵一根线
L298N OUT2          -> 气泵另一根线
气泵额定电源正极     -> L298N VMS/12V/电机电源+
气泵额定电源负极     -> L298N GND
STM32 GND           -> L298N GND
```

- ENA保留跳帽；没有跳帽时，把ENA接到稳定5V高电平。
- 如果负载电源高于12V，建议拔掉常见L298N模块的 `5V-EN` 跳帽，并从 `5V` 端提供独立逻辑5V。必须以你模块丝印和说明书为准。
- 程序中PB1=1、PB2=0时气泵启动；方向不对可以断电后交换OUT1/OUT2两根泵线。
- 本工程不使用L298N PWM调速，只控制气泵开和关。
- 气泵额定电压如果是12V，绝对不能直接使用24V。

### 6. 新增C610 + M2006吸盘升降

新增电机不再连接L298N，也不再使用PA5、PB10、PB11。它与原夹爪升降一样接入CAN系统：

```text
M2006三相动力线       -> 新增C610的M/A、M/B、M/C电机端
M2006传感器线         -> 新增C610的传感器接口
新增C610 CANH         -> CAN集线器CANH
新增C610 CANL         -> CAN集线器CANL
新增C610动力正负极     -> 电调动力电源（极性不能反接）
新增C610 ID           -> 设置为6
```

必须注意：

- 新增C610必须设置为ID6；反馈报文为 `0x206`，控制电流位于 `0x1FF` 的DATA[2..3]。
- bool12按住时提高目标位置，bool13按住时降低目标位置；松手后目标被冻结，位置环和重力补偿继续保持。
- 上电时吸盘升降必须位于最低基准位，程序收到第一帧ID6反馈时把该位置作为相对零点。
- 上下方向相反时只把 `SUCTION_LIFT_UP_SIGN` 从1改成-1，不要交换无刷电机三相线尝试改向。
- 程序只有相对软限位，没有原点开关或实体限位输入。必须保留上下机械限位、安全绳和防坠结构。
- CAN反馈丢失超过100ms、C610断电或24V断电时，软件保持力立即消失；M2006减速箱不能当作可靠断电抱闸。

### 7. MOS模块、电磁阀和气缸

保持你已经测试成功的MOS负载接法，只把控制信号合入PB0：

```text
PB0                 -> MOS模块 HIGH/PWM 高电平触发输入
STM32 GND           -> MOS模块信号GND
电磁阀额定电源正极   -> MOS模块 VIN+
电磁阀额定电源负极   -> MOS模块 VIN-
电磁阀正线           -> MOS模块 OUT+
电磁阀负线           -> MOS模块 OUT-
```

不同批次模块端子顺序可能不同，必须根据实物丝印 `VIN+、VIN-、OUT+、OUT-、HIGH/PWM、GND` 接线，不能只根据照片中端子位置猜测。

程序采用PB0高电平开启电磁阀、低电平关闭。不要把电磁阀直接接在PB0上。如果模块说明没有明确包含感性负载续流保护，电磁阀线圈还需要正确的续流保护器件。

### 8. ST-Link

```text
ST-Link SWDIO -> PA13
ST-Link SWCLK -> PA14
ST-Link GND   -> STM32 GND
ST-Link VTref -> 开发板3.3V目标电压检测端（按下载器说明）
```

第一次烧录时可以先断开24V动力电源，只保留STM32和ST-Link供电。

### 9. 升降防坠、限位和实体急停

当前工程没有占用额外GPIO接限位开关，也没有驱动制动器。为了防止断电下坠，建议机械部分至少采用一种不依赖程序的措施：

- 断电抱闸、常闭制动器：线圈得电才释放，断电自动抱住升降轴；线圈必须通过符合额定电压和电流的驱动器，不能直接接STM32。
- 蜗杆自锁减速、棘爪或棘轮：必须确认在最大负载和振动下确实不能反驱。
- 配重、恒力弹簧或气弹簧：把失电后的净下落力降到可控范围，但不能单独当作可靠制动。
- 上下硬限位和独立实体急停：限位应串入安全回路或接入后续专用输入代码，实体急停应按整机风险设计。

不要仅凭“36:1减速”判断M2006可以自锁。减速比会增大反驱阻力，但能否自锁取决于整套传动结构、效率、负载和磨损。两套M2006升降都有编码器位置闭环和相对软限位，但仍没有原点开关或实体限位输入，必须安装可靠的上下限位和防坠措施；本版代码不能替代它们。

## 五、BTMCU手机端配置

### 1. 必须使用周期发送

导入你提供的 `.pro` 工程后：

- 选择专业控制模式。
- 发送周期设为50ms。
- 使用“周期/定时发送”方式，保证即使摇杆不动也持续发送数据包。
- 如果界面有“仅操作控件时发送”，不要只启用这种模式，否则会触发300ms失联保护。

### 2. 8字节数据包

数据区共5字节：2字节bool位图和3个有符号byte摇杆值。bool从12个增加到14个后仍装在原来的2字节位图中，所以数据包长度不变。

| 字节 | 内容 |
| ---: | --- |
| 0 | 包头 `A5` |
| 1 | bool0～bool7，bool0位于最低位 |
| 2 | bool8～bool15，bool8位于最低位 |
| 3 | BYTE0：底盘X方向 |
| 4 | BYTE1：底盘Y方向 |
| 5 | BYTE2：底盘旋转 |
| 6 | 字节1～5相加后取低8位 |
| 7 | 包尾 `5A` |

全部数据为0时：

```text
A5 00 00 00 00 00 00 5A
```

只有bool0为1、其他数据为0时：

```text
A5 01 00 00 00 00 01 5A
```

手机软件会自动生成包头、校验和包尾，不需要在BYTE变量中手动填写。

只有bool12为1、其他数据为0时，正确帧为：

```text
A5 00 10 00 00 00 10 5A
```

bool12位于第2个bool字节的bit4，所以该字节为 `0x10`；bool13则是 `0x20`。

### 3. 现有9个bool的功能

| bool | 手机变量名 | 功能 | 操作方式 |
| ---: | --- | --- | --- |
| 0 | M1_FORWARD | 夹爪夹紧 | 按一下即转到 `CLAW_CLOSE_OUTPUT_DEG`，到位后保持；松手不改变目标 |
| 1 | M1_BACK | 夹爪张开 | 按一下回到上电零点并保持；松手不改变目标 |
| 2 | M2_FORWARD | 夹爪上升 | 按住提高目标高度，松手后在该位置保持 |
| 3 | M2_BACK | 夹爪下降 | 按住降低目标高度，松手后在该位置保持 |
| 4 | ON | 气泵开启 | 按一下后保持开启 |
| 5 | OFF | 气泵关闭 | 按一下关闭 |
| 6 | qigang | 电磁阀/气缸伸出 | 按住伸出，松手收回 |
| 7 | servo_zheng | 围栏向下 | 按住转动，松手停在当前位置 |
| 8 | servo_fan | 围栏向上 | 按住转动，松手停在当前位置 |

每个按钮都设置为“按下值 1、松开值 0”，不要使用字母发送。夹爪的两个按钮是两个位置选择命令；同时按下时忽略本次新命令并保持原目标。升降和舵机的相反方向同时为 1 时不再改变目标。

### 4. bool9～bool13的新增功能

当前手机工程已经有bool0～bool11。在它后面继续增加bool12和bool13；14个bool仍然只占2字节，所以总包长仍为8字节，三个BYTE的位置也不改变。

| bool | 建议名称 | 功能 |
| ---: | --- | --- |
| 9 | AUTO_CAPTURE | 一键完成围栏下降、开泵、延时、气缸伸出 |
| 10 | AUTO_RELEASE | 气缸收回、保持吸力800ms、关泵、围栏抬起 |
| 11 | EMERGENCY_STOP | 紧急停止所有电机、泵和电磁阀 |
| 12 | SUCTION_LIFT_UP | 按住时ID6 M2006提高吸盘升降目标，松手保持位置 |
| 13 | SUCTION_LIFT_DOWN | 按住时ID6 M2006降低吸盘升降目标，松手保持位置 |

新增步骤：

1. 导入 `BTMCU/邹凯_H_20260807-154905(1).pro`，确认原来12个bool和3个byte都在。
2. 在“逻辑值/bool”区域点击加号两次，新增索引12和13，名称分别设为 `SUCTION_LIFT_UP`、`SUCTION_LIFT_DOWN`。
3. 给两个变量各链接一个“点动/按住”按钮：按下值1、松开值0，禁止设置成按一下保持1。
4. AUTO_CAPTURE和AUTO_RELEASE仍设置按下1、松开0，程序只在0变1时启动一次流程。
5. EMERGENCY_STOP建议使用可保持的开关；保持为1时程序持续禁止动作。
6. 检查数据包结构仍显示：14*bool、3*byte、0*short、0*int、0*float，总计8字节。

吸盘升降两个方向键互锁：只按bool12时提高目标，只按bool13时降低目标；两个都为0或两个同时为1时停止改变目标，并继续闭环保持。蓝牙失联超过300ms时同样停止改变目标；bool11急停会把M2006输出置0，解除后以当时位置重新建立相对零点。

AUTO_CAPTURE过程：

```text
围栏转到下方
-> 吸气泵开启
-> 等待 PUMP_LEAD_TIME_MS（默认300ms）
-> 电磁阀通电，气缸伸出
-> 吸气泵保持开启
```

AUTO_RELEASE过程：

```text
电磁阀断电，气缸收回
-> 气泵继续保持 CYLINDER_RETRACT_TIME_MS（默认800ms）
-> 气泵关闭
-> 围栏抬起
```

自动流程只管理围栏、气泵和气缸。物体被吸住后，夹爪、夹爪升降和吸盘升降仍分别由bool0～3、bool12/13手动控制。

### 5. BYTE0～BYTE2

- BYTE0、BYTE1、BYTE2必须使用有符号byte，建议范围 `-100～100`。
- 摇杆松手时两个平移BYTE必须回到0；旋转控件松手时旋转BYTE必须回到0。
- 程序对 `-5～5` 设置了死区，防止摇杆轻微漂移。
- 松手归零后，程序不再只是给0电流让轮子滑行，而是对三个轮子同时执行零转速闭环刹车；转速降到阈值以内后电流才清零。
- 最高轮速由代码中的 `CHASSIS_MAX_RPM` 设置；初次测试建议先改为800～1000，确认方向后再增加。

## 六、实际操作顺序

### 新增M2006吸盘升降

- 按住 `SUCTION_LIFT_UP`（bool12）：ID6 M2006持续提高吸盘升降目标；到位立刻松手。
- 按住 `SUCTION_LIFT_DOWN`（bool13）：机构持续下降；到位立刻松手。
- 松开按钮后目标高度被冻结，ID6继续输出受限的位置环和重力补偿电流来保持。
- bool12/13只控制新增的M2006吸盘升降；bool2/3仍控制原来的M2006夹爪整体升降，两套机构互不替代。
- 编码器闭环只能知道相对转角，仍不知道机构是否碰到端点；操作员必须看着机构并使用实体限位和防坠。

### 手动吸取

1. 上电前确认夹爪完全张开、两套升降均位于最低基准位；上电后等待 ID4、ID5、ID6 的 CAN 反馈建立零点。
2. 使用底盘摇杆靠近物体。
3. 按住 `servo_zheng`，让围栏下降到合适位置后松手。
4. 点击 `ON`，气泵开始吸气并保持运行。
5. 按住 `qigang`，电磁阀通电、气缸带着吸盘伸出。
6. 吸盘接触并吸住物体后松开 `qigang`，气缸收回，气泵继续保持吸力。
7. 如机构需要，按住 `SUCTION_LIFT_UP` 或 `SUCTION_LIFT_DOWN` 调整吸盘组件高度，到位立即松手。
8. 短按一次 `M1_FORWARD` 后松开。夹爪自动转到设定闭合角度并继续用受限电流保持，不需要一直按住。
9. 按住 `M2_FORWARD` 抬升；到所需高度时松开。松手只是停止改变目标，位置闭环和重力补偿仍继续工作。

### 手动释放

1. 按住 `M2_BACK` 降低物体，到最低位置后松手。
2. 短按一次 `M1_BACK`，夹爪自动回到上电张开零点。
3. 点击 `OFF` 关闭气泵。
4. 如需复位吸盘高度，按住 `SUCTION_LIFT_DOWN`，到机械下限前松手。
5. 确认气缸已收回后，按住 `servo_fan` 抬起围栏。

### 自动吸取

增加bool9后，按一下 `AUTO_CAPTURE`。围栏、气泵和气缸会按状态机自动动作；随后手动控制夹爪和升降。

## 七、CubeMX完整配置

### 1. MCU和时钟

- MCU：STM32F405RGT6，LQFP64。
- SYS Debug：Serial Wire。
- RCC HSE：Crystal/Ceramic Resonator。
- HSE：8 MHz。
- PLLM=8，PLLN=336，PLLP=2，PLLQ=7。
- SYSCLK=168 MHz。
- AHB=168 MHz。
- APB1=42 MHz，APB1定时器时钟84 MHz。
- APB2=84 MHz，APB2定时器时钟168 MHz。

### 2. CAN1

- PA11：CAN1_RX。
- PA12：CAN1_TX。
- Mode：Normal。
- Prescaler：3。
- SJW：1 TQ。
- BS1：11 TQ。
- BS2：2 TQ。
- 波特率：1 Mbit/s。
- Automatic Bus-Off：Enable。
- Auto Retransmission：Enable。
- CAN1 RX0中断：Enable，抢占优先级0。

### 3. USART2

- PA2：USART2_TX。
- PA3：USART2_RX。
- Asynchronous，115200，8 Data Bits，No Parity，1 Stop Bit。
- TX/RX Enable，无硬件流控。
- USART2全局中断：Enable，抢占优先级1。

### 4. TIM3舵机PWM

- PA6：TIM3_CH1。
- PWM Generation CH1，PWM mode 1，高电平有效。
- Prescaler=83。
- Counter Period=19999。
- Pulse=1500。

TIM3时钟84 MHz，分频后计数频率1 MHz，每个计数为1us；周期20000us，即50Hz。

### 5. GPIO

- PB0：Output Push-Pull，Pull-down，初始Low，标签 `VALVE_SIG`。
- PB1：Output Push-Pull，Pull-down，初始Low，标签 `PUMP_IN1`。
- PB2：Output Push-Pull，Pull-down，初始Low，标签 `PUMP_IN2`。

吸盘升降改用C610 CAN控制后，不再需要PA5、PB10、PB11。

## 八、参数调节方法

所有主要参数位于 `Core/Src/main.c` 顶部的 `USER CODE BEGIN PD` 区域。

### 1. 调节底盘最高速度

当前8字节协议不包含INT调速量，最高速度直接在代码中修改：

```c
#define CHASSIS_MAX_RPM 2000.0f
```

第一次架空测试建议先改为800～1000rpm；确认轮子方向、松手刹停和整车运动方向均正常后，再逐步增加。

### 2. 底盘方向

如果整个摇杆方向相反，修改：

```c
#define CHASSIS_VX_SIGN  1.0f
#define CHASSIS_VY_SIGN  1.0f
#define CHASSIS_VW_SIGN  1.0f
```

把对应的 `1.0f` 改为 `-1.0f`：

- 前后相反：改 `CHASSIS_VX_SIGN` 或 `CHASSIS_VY_SIGN`，取决于手机摇杆轴链接。
- 平移方向相反：改另一个平移轴符号。
- 左右旋转相反：改 `CHASSIS_VW_SIGN`。

如果只有某一个轮方向相反，修改：

```c
#define MOTOR1_DIRECTION 1.0f
#define MOTOR2_DIRECTION 1.0f
#define MOTOR3_DIRECTION 1.0f
```

只把对应轮改成 `-1.0f`。一次只改一个参数并重新测试。

### 3. 三个底盘电机PID

三个电机可以单独修改：

```c
#define MOTOR1_KP 3.0f
#define MOTOR1_KI 0.2f
#define MOTOR1_KD 0.0f

#define MOTOR2_KP 3.0f
#define MOTOR2_KI 0.2f
#define MOTOR2_KD 0.0f

#define MOTOR3_KP 3.0f
#define MOTOR3_KI 0.09285f
#define MOTOR3_KD 0.0f
```

建议调节顺序：

1. 先把KI和KD临时设为0，只调KP。
2. 从低速500～800rpm开始，逐渐增加KP，直到响应足够快但不持续振荡。
3. 再小幅增加KI，消除负载下的长期速度误差。
4. 出现低频来回摆动、声音尖锐或电流很大时，减小KP或KI。
5. KD通常保持0；只有明确需要抑制快速超调时再小幅增加。

其他限制：

```c
#define C620_CURRENT_LIMIT  15000.0f
#define PID_INTEGRAL_LIMIT  2000.0f
#define CHASSIS_STOP_RPM_THRESHOLD  20
#define CHASSIS_BRAKE_CURRENT_LIMIT 5000.0f
```

`CHASSIS_BRAKE_CURRENT_LIMIT` 是摇杆松手后的主动刹车电流上限；停止太猛可降到3000，仍会滑行可逐步增加，但不要超过 `C620_CURRENT_LIMIT`。`CHASSIS_STOP_RPM_THRESHOLD` 是停止判定阈值，默认20rpm可避免静止后反复抖动。初次调车不要先提高电流上限。电机反馈超过100ms未更新时，本工程会把对应底盘电机电流置0。

### 4. M2006角度换算、方向和上电零点

程序按 M2006 P36 的 36:1 减速比计算：电机转子一圈为8192个编码器计数，输出轴1°约为819.2个计数。

```c
#define M2006_ENCODER_COUNTS_PER_REV 8192.0f
#define M2006_GEAR_RATIO             36.0f
#define CLAW_CLOSE_SIGN              1
#define LIFT_UP_SIGN                 1
#define SUCTION_LIFT_UP_SIGN         1
```

- 这里的角度是 **M2006减速箱输出轴角度**，不是夹爪两指的直接张角。
- 夹爪闭合方向相反：只把 `CLAW_CLOSE_SIGN` 改为 `-1`。
- 升降上升方向相反：只把 `LIFT_UP_SIGN` 改为 `-1`。
- 吸盘升降方向相反：只把 `SUCTION_LIFT_UP_SIGN` 改为 `-1`。
- 上电张开位置及两套升降的最低位置都是相对零点；没有限位开关时，程序无法判断绝对位置。
- 更改方向前先把夹爪闭合角设为5°、电流上限降到400，空载验证，禁止直接用45°撞机械限位。

### 5. 夹爪闭合角度和保持力

```c
#define CLAW_CLOSE_OUTPUT_DEG   45.0f
#define CLAW_POSITION_KP        0.040f
#define CLAW_SPEED_KD           0.35f
#define CLAW_CURRENT_LIMIT      1000.0f
#define CLAW_HOLD_BIAS_CURRENT  150.0f
```

参数作用：

| 参数 | 增大后的效果 | 调节建议 |
| --- | --- | --- |
| `CLAW_CLOSE_OUTPUT_DEG` | 夹爪继续向闭合方向转更多 | 从5°开始，每次增加2°～5°，以不撞限位且能夹住为准 |
| `CLAW_POSITION_KP` | 回到目标更快、保持更硬 | 每次增加0.005；若来回抖动、啸叫则减小 |
| `CLAW_SPEED_KD` | 阻尼更强、超调更小 | 抖动时每次增加0.05；太大时动作会迟钝 |
| `CLAW_CURRENT_LIMIT` | 最大动作力和堵转力增大 | 从400～600开始，确认机构后再逐步增加，不要一步调很大 |
| `CLAW_HOLD_BIAS_CURRENT` | 闭合后持续增加预紧力 | 从0开始，每次增加50；能可靠夹住就停止增加 |

bool0只在按键从0变1时选择闭合角度；bool1选择上电张开零点。松开按键不会把电流直接清零，位置误差和预紧项仍会输出受限保持电流。因此物体轻微滑动时，控制器会向闭合方向补力。

固定角度方案适合尺寸基本一致的物体。如果物体较厚，夹爪可能在到达目标前接触物体并长期输出限幅电流；这属于堵转保持，会发热。应把闭合角和保持电流调到“刚好可靠夹住”，并定时检查 M2006/C610 温升。

### 6. 升降速度、行程和抗重力保持

```c
#define LIFT_MIN_OUTPUT_DEG            0.0f
#define LIFT_MAX_OUTPUT_DEG            800.0f
#define LIFT_TARGET_SPEED_DEG_PER_SEC  300.0f
#define LIFT_POSITION_KP               0.045f
#define LIFT_SPEED_KD                  0.35f
#define LIFT_CURRENT_LIMIT             5000.0f
#define LIFT_GRAVITY_HOLD_CURRENT      1000.0f
```

- `LIFT_MAX_OUTPUT_DEG` 是从上电最低点算起的最大输出轴角度。必须按实际机械行程修改，800°只是当前起始值。
- `LIFT_TARGET_SPEED_DEG_PER_SEC` 决定按住升降键时目标高度变化速度；它不是电机转子rpm。
- 若卷筒直接装在输出轴且没有额外传动，单层绕线的近似行程为：`行程(mm) ≈ 角度/360 × 3.1416 × 卷筒直径(mm)`。
- `LIFT_POSITION_KP` 增大后位置保持更硬；出现振荡时减小KP或增加 `LIFT_SPEED_KD`。
- `LIFT_GRAVITY_HOLD_CURRENT` 是向上的重力补偿。绑好安全绳、让负载离地很低后，从0开始每次增加50：松键后仍缓慢下落就增加，自己上升就减小。
- 力量仍不足时才小步增加 `LIFT_CURRENT_LIMIT`，每次100～200，并监测温度和供电电流。

松开bool2/bool3后，目标高度被冻结，但电机不一定是0电流：位置环和重力补偿会继续通电抵抗下落。CAN反馈超过100ms时，为避免失控，该路会暂时输出0电流。

> 这些参数只能解决“系统正常供电时”的保持。断开24V、拔掉CAN、C610故障或实体急停切断动力时，软件无法产生保持力；没有机械自锁/制动器的升降会因重力下落。

### 7. 新增M2006吸盘升降方向、速度和行程

```c
#define SUCTION_LIFT_MIN_OUTPUT_DEG            0.0f
#define SUCTION_LIFT_MAX_OUTPUT_DEG            800.0f
#define SUCTION_LIFT_TARGET_SPEED_DEG_PER_SEC  300.0f
#define SUCTION_LIFT_POSITION_KP               0.045f
#define SUCTION_LIFT_SPEED_KD                  0.35f
#define SUCTION_LIFT_CURRENT_LIMIT             5000.0f
#define SUCTION_LIFT_GRAVITY_HOLD_CURRENT      1000.0f
#define SUCTION_LIFT_UP_SIGN                    1
```

- 上下方向相反：只把 `SUCTION_LIFT_UP_SIGN` 改为 `-1` 后重新编译。
- `SUCTION_LIFT_MAX_OUTPUT_DEG` 是从上电最低点计算的相对软行程，必须按吸盘机构实测修改。
- `SUCTION_LIFT_TARGET_SPEED_DEG_PER_SEC` 控制按住bool12/13时目标变化速度。
- `SUCTION_LIFT_POSITION_KP`、`SUCTION_LIFT_SPEED_KD`、电流上限和重力补偿的调节方法与上一节夹爪升降相同，但两套参数可以独立修改。
- 第一次空载测试建议把目标速度、最大行程、电流上限和重力补偿都先调低，确认方向后逐步增加。
- 松手会保持位置，但断电或CAN反馈丢失仍会失去保持；实体上下限位和机械防坠不能取消。

### 8. 舵机角度和速度

```c
#define SERVO_MIN_US     600U
#define SERVO_CENTER_US  1500U
#define SERVO_MAX_US     2400U
#define SERVO_STEP_US    10U
#define FENCE_UP_US      SERVO_MIN_US
#define FENCE_DOWN_US    SERVO_MAX_US
```

- 围栏上下相反：交换 `FENCE_UP_US` 与 `FENCE_DOWN_US`。
- 围栏行程太大：不要改PWM频率，直接把上下脉宽向1500靠近，例如900和2100。
- 舵机移动太慢：适当增大 `SERVO_STEP_US`。
- 舵机移动太快：减小 `SERVO_STEP_US`。
- 每次只改50～100us并拆掉机械连杆测试，避免撞死舵机。

粗略角度换算：

```text
角度比例 ≈ (脉宽 - SERVO_MIN_US) / (SERVO_MAX_US - SERVO_MIN_US)
```

不同SG90实际端点差异很大，应以机械实测为准。

### 9. 自动流程时间

```c
#define PUMP_LEAD_TIME_MS         300U
#define CYLINDER_RETRACT_TIME_MS  800U
```

- 吸盘还没建立吸力就发射：增大 `PUMP_LEAD_TIME_MS`。
- 气缸还没完全收回就关泵：增大 `CYLINDER_RETRACT_TIME_MS`。
- 先以100ms为步长逐渐调整。

### 10. 摇杆死区和蓝牙失联时间

```c
#define BT_AXIS_DEADZONE    5
#define BT_LOSS_TIMEOUT_MS  300U
```

- 摇杆松手后仍有轻微运动：把死区从5增加到8或10。
- 摇杆已经归零但轮子因惯性继续滚：程序会自动进行主动刹车；需要调节时优先修改 `CHASSIS_BRAKE_CURRENT_LIMIT`。
- 发送周期必须保持50ms；300ms超时相当于连续丢失约6帧。
- 不建议为了掩盖通信问题把失联时间改得很长。

## 九、安全保护和已知限制

本工程已经加入：

- 300ms没有收到合法8字节包：底盘进入零转速主动刹车、两套M2006升降不再改变目标、舵机手动动作停止、气泵和电磁阀关闭、自动流程取消；只要24V和CAN仍正常，三个M2006机构继续保持最后目标位置。
- bool11显式急停：底盘和三个M2006机构电流命令置0、泵阀关闭，并在解除后把当时位置重新作为M2006机构相对零点，避免突然返回旧目标。
- 夹爪相反按键同时按下时忽略新位置；升降和舵机相反方向同时按下时停止改变目标。
- 任一电机CAN反馈超过100ms未更新时，该路电流命令置0。
- 两套升降目标分别受到 `LIFT_*` 和 `SUCTION_LIFT_*` 范围限制，但都只是相对软限位。
- 摇杆死区和速度硬限制。

仍然必须注意：

- 失联或急停会关闭吸气泵，正在吸住的物体仍可能掉落，禁止在人、玻璃或设备上方测试。
- bool11急停、24V断电、C610断电、CAN反馈丢失都会让相应M2006失去电气保持；升降必须有机械防坠。
- 两套M2006升降虽有编码器位置闭环，但断电、CAN丢失或C610故障后仍会失去保持，必须配置实体限位和防坠/常闭抱闸。
- 三个M2006机构目前都没有原点开关和实体限位输入；每次上电的初始姿态必须正确，参数必须在机械极限以内。
- 夹爪接触物体却未达到目标角时可能长期堵转保持；保持电流过大或保持时间过长会造成电机、电调和线束发热。
- 软件急停不能代替切断动力电源的实体急停开关。
- 第一次测试必须架空底盘、拆下舵机连杆并让气缸远离人员。

## 十、第一次测试顺序

1. 机械上先把夹爪置于完全张开基准位、两套升降均置于最低基准位，并给升降负载系安全绳。
2. 先只给STM32、HC-05和ST-Link供电，动力部分全部断开。
3. 导入BTMCU工程，确认周期发送50ms和数据包总长8字节。
4. 让所有bool为0、三个BYTE回0；手机重新连接前也要先松开所有按钮。
5. 暂不接电磁阀，按住qigang，用万用表确认PB0为约3.3V，松手后为0V。
6. 暂不接气泵，点击ON/OFF，确认PB1/PB2电平和L298N输出逻辑。
7. 单独接入新增C610与M2006，把C610设为ID6；先不连接机械负载，确认能持续收到 `0x206` 反馈。
8. 临时把吸盘升降速度、最大行程和电流上限调低，短按bool12/13确认方向；方向正确后才连接升降机构，并安装上下限位和安全绳。
9. 舵机不安装连杆，测试servo_zheng和servo_fan并校准脉宽。
10. 一次只接一个电调，先确认ID和方向，再接全部6个。
11. 架空3个轮，先把代码中的 `CHASSIS_MAX_RPM` 改为800，测试平移、旋转方向和松手刹车。
12. 夹爪先使用5°闭合角、400电流上限、0保持偏置，空载点按bool0确认方向，再逐步调到需要的角度和保持力。
13. 两套M2006升降都先空载、低行程测试方向和软限位；再系安全绳、贴近地面分别调节重力补偿，不要直接悬挂正式物体。
14. 最后连接围栏、气泵、电磁阀和气缸，并测试完整流程。

## 十一、Keil编译和烧录

1. 打开 `MDK-ARM/xiaoche_complete.uvprojx`。
2. 点击 Rebuild，工程已配置生成HEX。
3. Debug选择ST-Link Debugger，Port选择SW，调试时钟先设1MHz。
4. Flash Download选择STM32F4xx 1MB Flash算法，勾选Reset and Run。
5. 点击Download烧录。

交付压缩包不包含旧版本编译输出。请在本机Keil中执行Rebuild，以生成与BTMCU bool版源码一致的新HEX。

## 十二、技术依据

- [DJI RoboMaster M2006 P36直流无刷减速电机使用说明](https://cdn-hz.robomaster.com/tem/RM%20M2006%20P36%E7%9B%B4%E6%B5%81%E6%97%A0%E5%88%B7%E5%87%8F%E9%80%9F%E7%94%B5%E6%9C%BA%E4%BD%BF%E7%94%A8%E8%AF%B4%E6%98%8E%20%E5%8F%91%E5%B8%83%E7%89%88.pdf)：P36减速比、位置传感器和避免长时间堵转等依据。
- [DJI RoboMaster C610无刷电机调速器使用说明](https://rm-static.djicdn.com/tem/RM%20C610%E6%97%A0%E5%88%B7%E7%94%B5%E6%9C%BA%E8%B0%83%E9%80%9F%E5%99%A8%E4%BD%BF%E7%94%A8%E8%AF%B4%E6%98%8E%20%E5%8F%91%E5%B8%83%E7%89%88.pdf)：CAN控制、转子位置/转速/电流反馈和供电要求等依据。

厂家资料没有把M2006 P36描述为断电抱闸或保证自锁机构。因此“断电后可能被负载反驱并下落”是基于其结构和整机负载的安全推断，最终必须通过机械防坠设计和实物测试确认。

L298N功能依据：[STMicroelectronics L298 Dual Full Bridge Driver](https://www.st.com/en/motor-drivers/l298.html)。具体模块上的稳压跳帽、接线端子、散热器和保护器件会因厂家而异，必须以实物丝印和模块说明书为准。
