# SW2303 寄存器列表

## 1. 版本历史

V1.0 初始版本，针对芯片版本号0；V1.1 针对芯片版本号1；V1.2 更新页眉图标；V1.3 更新默认值及REG0xAE[2:1]；V1.4 更换文档模板；

## 2. 寄存器

注意：未定义的寄存器或bit不能被改写

### 2.1. REG0x01：芯片版本

<table><tr><td>Bit</td><td>Description</td><td>R/W</td><td>Default</td></tr><tr><td>7-2</td><td>/</td><td>/</td><td>/</td></tr><tr><td>1-0</td><td>芯片版本号</td><td>R</td><td>0x1</td></tr></table>

### 2.2. REG0x03：设置电压高8位

<table><tr><td>Bit</td><td>Description</td><td>R/W</td><td>Default</td></tr><tr><td>7-0</td><td>dac_vol[11:4]
当前的设置电压：dac_vol[11:0]*10mV</td><td>R</td><td>0x0</td></tr></table>

### 2.3. REG0x04：设置电压低4位

<table><tr><td>Bit</td><td>Description</td><td>R/W</td><td>Default</td></tr><tr><td>7-4</td><td>dac_vol[3:0]
当前的设置电压：dac_vol[11:0]*10mV</td><td>R</td><td>0x0</td></tr><tr><td>3-0</td><td>/</td><td>/</td><td>/</td></tr></table>

### 2.4. REG0x05：设置限流

<table><tr><td>Bit</td><td>Description</td><td>R/W</td><td>Default</td></tr><tr><td>7</td><td>/</td><td>/</td><td>/</td></tr><tr><td>6-0</td><td>ctrl_icc[6:0]</td><td>R</td><td>0x0</td></tr><tr><td></td><td>当前设置的限流：1000mA+ctrl_icc[6:0]*50mA</td><td></td><td></td></tr></table>

### 2.5. REG0x06：快充指示

<table><tr><td>Bit</td><td>Description</td><td>R/W</td><td>Default</td></tr><tr><td>7</td><td>处于快充协议
0: 未处于快充协议
1: 处于快充协议</td><td>R</td><td>0x0</td></tr><tr><td>6</td><td>处于快充电压
0: 未处于快充电压
1: 处于快充电压</td><td>R</td><td>0x0</td></tr><tr><td>5-4</td><td>PD 协议版本
1: PD 2.0
2: PD 3.0
other: Reserved</td><td>R</td><td>0x0</td></tr><tr><td>3-0</td><td>快充协议指示
1: QC2.0
2: QC3.0
3: FCP
4: /
5: SCP
6: PD FIX
7: PD PPS
8: PE1.1
9: PE2.0
C: SFCP
D: AFC
other: reserved</td><td>R</td><td>0x0</td></tr></table>

### 2.6. REG0x07：系统状态0

<table><tr><td>Bit</td><td>Description</td><td>R/W</td><td>Default</td></tr><tr><td>7-5</td><td>/</td><td>/</td><td>/</td></tr><tr><td>4</td><td>异常拉光耦状态
0: 未出现异常拉光耦
1: 出现异常拉光耦</td><td>R</td><td>0x0</td></tr><tr><td>3</td><td>/</td><td>/</td><td>/</td></tr><tr><td>2</td><td>CC 环路状态
0: CC 环路打开
1: CC 环路关闭</td><td>R</td><td>0x0</td></tr><tr><td>1</td><td>线补打开状态</td><td>R</td><td>0x0</td></tr><tr><td></td><td>0:未打开线补
1:线补打开</td><td></td><td></td></tr><tr><td>0</td><td>通路管状态
0:通路管关闭
1:通路管打开</td><td>R</td><td>0x0</td></tr></table>

### 2.7. REG 0x0B: 系统状态1

<table><tr><td>Bit</td><td>Description</td><td>R/W</td><td>Default</td></tr><tr><td>7-6</td><td>/</td><td>/</td><td>/</td></tr><tr><td>5</td><td>Vin 超过 25V 指示
0: Vin 低于 25V
1: Vin 高于 25V</td><td>R</td><td>0x0</td></tr><tr><td>4</td><td>过流状态指示
0: 未过流
1: 电流超过 112.5%</td><td>R</td><td>0x0</td></tr><tr><td>3</td><td>Die 过温指示
0: die 未过温
1: die 过温</td><td>R</td><td>0x0</td></tr><tr><td>2</td><td>/</td><td>/</td><td>/</td></tr><tr><td>1</td><td>Vin 过压指示
0: vin 未过压
1: vin 过压
当 Vin 电压高于 DAC 请求电压的 20%，认为 Vin 过压</td><td>R</td><td>0x0</td></tr><tr><td>0</td><td>Vin 欠压指示
0: vin 未欠压
1: vin 欠压
Vin 低于 4V 则认为欠压</td><td>R</td><td>0x0</td></tr></table>

### 2.8. REG 0x0C: 系统状态2

<table><tr><td>Bit</td><td>Description</td><td>R/W</td><td>Default</td></tr><tr><td>7</td><td>CC1 过压指示
0:CC1 未过压
1:CC1 过压</td><td>R</td><td>0x0</td></tr><tr><td>6</td><td>CC2 过压指示
0:CC2 未过压
1:CC2 过压</td><td>R</td><td>0x0</td></tr><tr><td>5</td><td>DP 过压指示
0:DP 未过压
1:DP 过压</td><td>R</td><td>0x0</td></tr><tr><td>4</td><td>DM 过压指示
0: DM 未过压
1: DM 过压</td><td>R</td><td>0x0</td></tr><tr><td>3</td><td>/</td><td>/</td><td>/</td></tr><tr><td>2</td><td>低电指示
0: 未处于低电
1: 处于低电
Vin 低于 3V 则认为低电</td><td>R</td><td>0x0</td></tr><tr><td>1</td><td>/</td><td>/</td><td>/</td></tr><tr><td>0</td><td>过流保护指示
0: 未过流
1: 过流保护</td><td>R</td><td>0x0</td></tr></table>

### 2.9. REG 0x0D: 系统状态3

<table><tr><td>Bit</td><td>Description</td><td>R/W</td><td>Default</td></tr><tr><td>7</td><td>在线指示
0: 不在线
1: 在线
C 口接入或 A 口电流大于门限，此位置 1; C 口拔出或 A 口电流小于门限，此位清零。</td><td>R</td><td>0x0</td></tr><tr><td>6-0</td><td>Reserved</td><td>/</td><td>/</td></tr></table>

### 2.10. REG 0x12: I2C 写使能控制0

<table><tr><td>Bit</td><td>Description</td><td>R/W</td><td>Default</td></tr><tr><td>7-5</td><td>I2C 写操作使能
如果要操作寄存器 reg0x14, reg0xA0~BF,需要先执行如下操作:
1. 写 reg0x12 = 0x20;
2. 写 reg0x12 = 0x40;
3. 写 reg0x12 = 0x80;</td><td>R/W</td><td>0x0</td></tr><tr><td>4-0</td><td>/</td><td>/</td><td>/</td></tr></table>

### 2.11. REG 0x14: 连接控制

<table><tr><td>Bit</td><td>Description</td><td>R/W</td><td>Default</td></tr><tr><td>7-3</td><td>/</td><td>/</td><td>/</td></tr><tr><td>2</td><td>线补控制</td><td>R/W</td><td>0x0</td></tr><tr><td></td><td>0：打开线补
1：关闭线补</td><td></td><td></td></tr><tr><td>1</td><td>Type-C CC un-driving 使能
0：无影响
1：CC un-driving 1s，之后自动清零</td><td>R/WC</td><td>0x0</td></tr><tr><td>0</td><td>/</td><td>/</td><td>/</td></tr></table>

### 2.12. REG0x15:I2C写使能控制1

<table><tr><td>Bit</td><td>Description</td><td>R/W</td><td>Default</td></tr><tr><td>7-5</td><td>I2C写操作使能
如果要操作寄存器reg0x16需要先执行如下操作：
1.写reg0x12=0x20;
2.写reg0x12=0x40;
3.写reg0x12=0x80;</td><td>R/W</td><td>0x0</td></tr><tr><td>4-0</td><td>/</td><td>/</td><td>/</td></tr></table>

### 2.13. REG0x16：强制控制使能

<table><tr><td>Bit</td><td>Description</td><td>R/W</td><td>Default</td></tr><tr><td>7-4</td><td>/</td><td>/</td><td>/</td></tr><tr><td>3</td><td>强制开通路
0：无影响
1：强制开通路</td><td>R/W</td><td>0x0</td></tr><tr><td>2</td><td>强制关通路
0：无影响
1：强制关通路</td><td>R/W</td><td>0x0</td></tr><tr><td>1</td><td>强制控制 DAC
0：无影响
1：强制控制 dac vol</td><td>R/W</td><td>0x0</td></tr><tr><td>0</td><td>强制控制限流
0：无影响
1：强制控制 ctr_ic</td><td>R/W</td><td>0x0</td></tr></table>

### 2.14. REG0x30:ADC Vin数据

<table><tr><td>Bit</td><td>Description</td><td>R/W</td><td>Default</td></tr><tr><td>7-0</td><td>Vin 电压的高 8bit
7.5*16mv/bit; (若取 12bit 时分辨率为 7.5mv/bit,参见 reg0x3B)</td><td>R</td><td>0x0</td></tr></table>

### 2.15. REG 0x31:ADC Vbus 数据

<table><tr><td>Bit</td><td>Description</td><td>R/W</td><td>Default</td></tr><tr><td>7-0</td><td>Vbus 电压的高 8bit
7.5*16mv/bit; (若取 12bit 时分辨率为 7.5mv/bit,参见 reg0x3B)</td><td>R</td><td>0x0</td></tr></table>

### 2.16. REG 0x33:ADC Ich 数据

<table><tr><td>Bit</td><td>Description</td><td>R/W</td><td>Default</td></tr><tr><td>7-0</td><td>输出电流的高 8bit
50mA/bit; (若取 12bit 时分辨率为 3.125mA/bit,参见 reg0x3B)</td><td>R</td><td>0x0</td></tr></table>

### 2.17. REG 0x36:ADC Tdiet 数据

<table><tr><td>Bit</td><td>Description</td><td>R/W</td><td>Default</td></tr><tr><td>7-0</td><td>Die 温度的高 8bit,
2.38℃/bit; (若取 12bit 时分辨率为 0.1488℃/bit,参见 reg0x3B)</td><td>R</td><td>0x0</td></tr></table>

### 2.18. REG 0x3B:ADC 配置

<table><tr><td>Bit</td><td>Description</td><td>R/W</td><td>Default</td></tr><tr><td>7-5</td><td>/</td><td>/</td><td>/</td></tr><tr><td>2-0</td><td>ADC 数据选择
写此寄存器之后，将对应的 ADC 数据锁存到 Reg0x3C 和 Reg0x3D，防止读到的数据高低位不对应
对应关系如下：
1: adc_vin[11:0], 7.5mV/bit
2: adc_vbus[11:0], 7.5mV/bit
3: adc_ich[11:0], 3.125mA/bit
4: adc_diet[11:0], 0.1488℃/bit; Tdiet = (adc_diet[11:0]-
1848)/6.72℃
Other: reserved</td><td>R/W</td><td>0x0</td></tr></table>

### 2.19. REG 0x3C:ADC 数据高8位

<table><tr><td>Bit</td><td>Description</td><td>R/W</td><td>Default</td></tr><tr><td>7-0</td><td>ADC 高 8bit 数据锁存
adc_data[11:04]</td><td>R</td><td>0x0</td></tr></table>

### 2.20. REG0x3D:ADC数据低4位

<table><tr><td>Bit</td><td>Description</td><td>R/W</td><td>Default</td></tr><tr><td>7-4</td><td>/</td><td>/</td><td>/</td></tr><tr><td>3-0</td><td>ADC 低 4bit 数据锁存
adc_data[03:00]</td><td>R</td><td>0x0</td></tr></table>

### 2.21. REG0xA1：异常处理配置

<table><tr><td>Bit</td><td>Description</td><td>R/W</td><td>Default</td></tr><tr><td>7</td><td>Die 过温异常处理使能
0: 使能
1: 不使能</td><td>R/W</td><td>0x0</td></tr><tr><td>6-0</td><td>Reserved
注意不能修改默认值</td><td>R/W</td><td>0x2</td></tr></table>

### 2.22. REG0xA3：输出电压偏移配置

<table><tr><td>Bit</td><td>Description</td><td>R/W</td><td>Default</td></tr><tr><td>7-6</td><td>输出电压固定偏移设置(对 PPS, QC3.0 和低压直充无效)
0: 200mV
1: 0mV
2: 250mV
3: 100mV</td><td>R/W</td><td>0x3</td></tr><tr><td>5</td><td>Vin 超过 25V 的异常处理使能
0: 不使能
1: 使能</td><td>R/W</td><td>0x1</td></tr><tr><td>4:0</td><td>Reserved
注意不能修改默认值</td><td>R/W</td><td>0x0</td></tr></table>

### 2.23. REG0xA4：线补阻抗配置

<table><tr><td>Bit</td><td>Description</td><td>R/W</td><td>Default</td></tr><tr><td>7-6</td><td>线补阻抗设置
0: 50mΩ
1: 0mΩ
2: 100mΩ
3: 150mΩ
根据阻抗和电流计算得到线降后，通过反馈调高 Vin</td><td>R/W</td><td>0x0</td></tr><tr><td>5-0</td><td>Reserved
注意不能修改默认值</td><td>R/W</td><td>0x0</td></tr></table>

### 2.24. REG0xA5：过压检测配置

<table><tr><td>Bit</td><td>Description</td><td>R/W</td><td>Default</td></tr><tr><td>7</td><td>DP 过压保护使能
0: 使能
1: 不使能</td><td>R/W</td><td>0x0</td></tr><tr><td>6</td><td>DM 过压保护使能
0: 使能
1: 不使能</td><td>R/W</td><td>0x0</td></tr><tr><td>5</td><td>CC1/CC2 过压保护使能
0: 使能
1: 不使能</td><td>R/W</td><td>0x0</td></tr><tr><td>4-0</td><td>Reserved
注意不能修改默认值</td><td>R/W</td><td>0x0</td></tr></table>

### 2.25. REG0xA6:PD配置3

<table><tr><td>Bit</td><td>Description</td><td>R/W</td><td>Default</td></tr><tr><td>7</td><td>Reserved
注意不能修改默认值</td><td>R/W</td><td>0x1</td></tr><tr><td>6</td><td>PD 5A 电流是否需检测 emark
0: 需要检测到 emark 才能广播 5A 电流
1: 无 emark 也可以广播 5A</td><td>R/W</td><td>0x0</td></tr><tr><td>5-0</td><td>Reserved
注意不能修改默认值</td><td>R/W</td><td>0x30</td></tr></table>

### 2.26. REG0xA8：广播电流配置

<table><tr><td>Bit</td><td>Description</td><td>R/W</td><td>Default</td></tr><tr><td>7</td><td>PD PPS3 广播 5A 电流使能
0: 不使能
1: 使能，即强制广播 5A</td><td>R/W</td><td>0x0</td></tr><tr><td>6</td><td>Type-C 广播电流设置
0: 有 PD 功率决定
1: 固定广播 1.5A</td><td>R/W</td><td>0x0</td></tr><tr><td>5-0</td><td>Reserved
注意不能修改默认值</td><td>R/W</td><td>0x0</td></tr></table>

### 2.27. REG0xAB：异常保护配置

<table><tr><td>Bit</td><td>Description</td><td>R/W</td><td>Default</td></tr><tr><td>7-4</td><td>Reserved
注意不能修改默认值</td><td>R/W</td><td>0x8</td></tr><tr><td>3-2</td><td>Die 过温保护门限
0: 105℃
1: 115℃
2: 125℃
3: 135℃</td><td>R/W</td><td>0x2</td></tr><tr><td>1-0</td><td>Reserved
注意不能修改默认值</td><td>R/W</td><td>0x0</td></tr></table>

### 2.28. REG0xAC：在线配置

<table><tr><td>Bit</td><td>Description</td><td>R/W</td><td>Default</td></tr><tr><td>7-3</td><td>Reserved</td><td>R/W</td><td>0x6</td></tr><tr><td>2</td><td>电流小于掉线门限的 debounce 时间
0: 6~8s
1: 3~4s</td><td>R/W</td><td>0x0</td></tr><tr><td>1-0</td><td>判别 A 口为在线的电流门限
0: 200mA
1: 100mA
2: 300mA
3: 400mA</td><td>R/W</td><td>0x2</td></tr></table>

### 2.29. REG0xAD：快充配置0

<table><tr><td>Bit</td><td>Description</td><td>R/W</td><td>Default</td></tr><tr><td>7</td><td>QC2.0/QC3.0/PD FIX 线补使能
0: 使能
1: 不使能</td><td>R/W</td><td>0x0</td></tr><tr><td>6-4</td><td>Reserved
注意不能修改默认值</td><td>R/W</td><td>0x0</td></tr><tr><td>3</td><td>FCP/AFC/SFCP 协议的高压限流
0: 3.25A
1: 2.25A</td><td>R/W</td><td>0x0</td></tr><tr><td>2-1</td><td>Reserved
注意不能修改默认值</td><td>R/W</td><td>0x0</td></tr><tr><td>0</td><td>PD 是否禁止 SCP 协议
0: 不禁止，即处于 PD 协议时可以响应 SCP 请求</td><td>R/W</td><td>0x0</td></tr><tr><td></td><td>1: 禁止</td><td></td><td></td></tr></table>

### 2.30. REG0xAE：快充配置1

<table><tr><td>Bit</td><td>Description</td><td>R/W</td><td>Default</td></tr><tr><td>7-6</td><td>Reserved
注意不能修改默认值</td><td>R/W</td><td>0x3</td></tr><tr><td>5</td><td>调压支持的最低电压
0: 最低电压为 3.0V
1: 最低电压为 5.0V</td><td>R/W</td><td>0x0</td></tr><tr><td>4</td><td>协议支持 5V 以下电压
0: 支持小于 5V 的电压
1: 支持的最低电压为 5V</td><td>R/W</td><td>0x0</td></tr><tr><td>3</td><td>QC3.0 协议输出支持 20V 使能
0: 使能
1: 不使能</td><td>R/W</td><td>0x0</td></tr><tr><td>2</td><td>QC2.0 协议输出支持 20V 使能
0: 不使能
1: 使能</td><td>R/W</td><td>0x0</td></tr><tr><td>1</td><td>PE2.0 协议输出支持 20V 使能
0: 不使能
1: 使能</td><td>R/W</td><td>0x0</td></tr><tr><td>0</td><td>非 PD 协议输出支持 12V 使能
0: 使能
1: 不使能
注意: 禁止 12V 时, 必须同时禁止 QC2.0/QC3.0/PE2.0 的 20V 输出</td><td>R/W</td><td>0x0</td></tr></table>

### 2.31. REG0xAF：功率配置

<table><tr><td>Bit</td><td>Description</td><td>R/W</td><td>Default</td></tr><tr><td>7</td><td>最大功率配置方式选择
0: 外部电阻
1: 寄存器</td><td>R/W</td><td>0x0</td></tr><tr><td>6-0</td><td>最大功率设置
W/bit</td><td>R/W</td><td>0x0</td></tr></table>

### 2.32. REG0xB0：快充配置2

<table><tr><td>Bit</td><td>Description</td><td>R/W</td><td>Default</td></tr><tr><td>7-6</td><td>Reserved</td><td>R/W</td><td>0x3</td></tr><tr><td></td><td>注意不能修改默认值</td><td></td><td></td></tr><tr><td>5</td><td>高压SCP协议使能
0:使能高压SCP
1:关闭高压SCP</td><td>R/W</td><td>0x0</td></tr><tr><td>4</td><td>低压SCP协议使能
0:使能低压SCP
1:关闭低压SCP</td><td>R/W</td><td>0x1</td></tr><tr><td>3</td><td>QC3.0使能
0:使能
1:不使能</td><td>R/W</td><td>0x0</td></tr><tr><td>2</td><td>QC2.0使能
0:使能
1:不使能</td><td>R/W</td><td>0x0</td></tr><tr><td>1</td><td>快充使能总开关
0:使能
1:不使能</td><td>R/W</td><td>0x0</td></tr><tr><td>0</td><td>BC1.2使能
0:使能
1:不使能
注意:不使能BC1.2时,将不驱动DPDM,但DPDM保持短接</td><td>R/W</td><td>0x0</td></tr></table>

### 2.33. REG 0xB1: 快充配置3

<table><tr><td>Bit</td><td>Description</td><td>R/W</td><td>Default</td></tr><tr><td rowspan="2">7</td><td>Vin ADC 过压保护使能(高于 DAC 设定值 20%)</td><td rowspan="2">R/W</td><td rowspan="2">0x0</td></tr><tr><td>0: 使能
1: 不使能</td></tr><tr><td>6</td><td>Vin 模拟过压保护使能(高于 DAC 设定值 2V)
0: 使能
1: 不使能</td><td>R/W</td><td>0x0</td></tr><tr><td>5</td><td>三星 1.2V 模式使能
0: 使能
1: 不使能</td><td>R/W</td><td>0x0</td></tr><tr><td>4</td><td>SFCP 协议使能
0: 使能
1: 不使能</td><td>R/W</td><td>0x0</td></tr><tr><td>3</td><td>FCP 使能
0: 使能
1: 不使能</td><td>R/W</td><td>0x0</td></tr><tr><td>2</td><td>AFC 使能
0: 使能
1: 不使能</td><td>R/W</td><td>0x0</td></tr><tr><td>1</td><td>PE 使能
0: 使能
1: 不使能</td><td>R/W</td><td>0x0</td></tr><tr><td>0</td><td>Reserved
注意不能修改默认值</td><td>R/W</td><td>0x0</td></tr></table>

### 2.34. REG 0xB2: 快充配置 4

<table><tr><td>Bit</td><td>Description</td><td>R/W</td><td>Default</td></tr><tr><td>7-6</td><td>Reserved
注意不能修改默认值</td><td>R/W</td><td>0x0</td></tr><tr><td>5-4</td><td>SCP 协议广播的电流
0: 5.0A
1: 4.0A
2: 2.0A
3: Reserved</td><td>R/W</td><td>0x2</td></tr><tr><td>3</td><td>25w 时 9V PDO 的电流
0: 2.78A
1: 2.77A</td><td>R/W</td><td>0x1</td></tr><tr><td>2-0</td><td>Reserved
注意不能修改默认值</td><td>R/W</td><td>0x1</td></tr></table>

### 2.35. REG 0xB3: PD 配置 0

<table><tr><td>Bit</td><td>Description</td><td>R/W</td><td>Default</td></tr><tr><td>7</td><td>功率在 60~70W 时是否需 Emarker 线
0: 需要为 emarker 线
1: 不需要为 emarker 线</td><td>R/W</td><td>0x0</td></tr><tr><td>6</td><td>收到非法 PD 请求的处理方式
0: reject 非法请求
1: 发 hardreset 并禁止 PPS</td><td>R/W</td><td>0x1</td></tr><tr><td>5</td><td>Reserved
注意不要修改默认值</td><td>R/W</td><td>0x1</td></tr><tr><td>4</td><td>Emarker 检测使能
0: 使能
1: 不使能</td><td>R/W</td><td>0x0</td></tr><tr><td>3</td><td>PD dr_swap 命令支持
0: 不支持
1: 支持</td><td>R/W</td><td>0x0</td></tr><tr><td>2</td><td>PD vconn swap 命令支持
0: 不支持
1: 支持</td><td>R/W</td><td>0x1</td></tr><tr><td>1</td><td>Reserved
注意不要修改默认值</td><td>R/W</td><td>0x0</td></tr><tr><td>0</td><td>PD 使能
0: 使能
1: 不使能</td><td>R/W</td><td>0x0</td></tr></table>

### 2.36. REG 0xB4: PD 配置 1

<table><tr><td>Bit</td><td>Description</td><td>R/W</td><td>Default</td></tr><tr><td>7</td><td>PPS0/PPS1/PPS2/PPS3 寄存器配置使能
0: 自动配置
1: 寄存器配置</td><td>R/W</td><td>0x0</td></tr><tr><td>6</td><td>PPS3 最大电流配置
0: 5A
1: 3A</td><td>R/W</td><td>0x0</td></tr><tr><td>5-4</td><td>Reserved
注意不要修改默认值</td><td>R/W</td><td>0x0</td></tr><tr><td>3</td><td>PD discovery identity 命令支持
0: 不支持
1: 支持</td><td>R/W</td><td>0x0</td></tr><tr><td>2</td><td>PD discovery SVID 命令支持
0: 不支持
1: 支持</td><td>R/W</td><td>0x0</td></tr><tr><td>1-0</td><td>PD peak current 配置
注意此值仅修改 PDO 中对应的项</td><td>R/W</td><td>0x0</td></tr></table>

### 2.37. REG 0xB5: PD 配置 2

<table><tr><td>Bit</td><td>Description</td><td>R/W</td><td>Default</td></tr><tr><td>7</td><td>3.3~21V PPS 使能(PPS3)
0: 使能
1: 不使能</td><td>R/W</td><td>0x0</td></tr><tr><td>6</td><td>3.3~16V PPS 使能(PPS2)
0: 使能
1: 不使能</td><td>R/W</td><td>0x0</td></tr><tr><td>5</td><td>3.3~11V PPS 使能(PPS1)
0: 使能
1: 不使能</td><td>R/W</td><td>0x0</td></tr><tr><td>4</td><td>3.3~5.9V PPS 使能(PPS0)
0: 使能
1: 不使能</td><td>R/W</td><td>0x0</td></tr><tr><td>3</td><td>PD fixed 20V 使能
0: 使能
1: 不使能</td><td>R/W</td><td>0x0</td></tr><tr><td>2</td><td>PD fixed 15V 使能
0: 使能
1: 不使能</td><td>R/W</td><td>0x0</td></tr><tr><td>1</td><td>PD fixed 12V 使能
0: 使能
1: 不使能</td><td>R/W</td><td>0x0</td></tr><tr><td>0</td><td>PD fixed 9V 使能
0: 使能
1: 不使能</td><td>R/W</td><td>0x0</td></tr></table>

### 2.38. REG 0xB6:VID配置0

<table><tr><td>Bit</td><td>Description</td><td>R/W</td><td>Default</td></tr><tr><td>7-0</td><td>vendor ID 配置 VID[15:8]</td><td>R/W</td><td>0x0</td></tr></table>

### 2.39. REG 0xB7:VID配置11

<table><tr><td>Bit</td><td>Description</td><td>R/W</td><td>Default</td></tr><tr><td>7-0</td><td>vendor ID 配置 VID[7:0]</td><td>R/W</td><td>0x0</td></tr></table>

### 2.40. REG 0xB8:XID配置0

<table><tr><td>Bit</td><td>Description</td><td>R/W</td><td>Default</td></tr><tr><td>7-0</td><td>XID 配置 XID[31:24]</td><td>R/W</td><td>0x0</td></tr></table>

### 2.41. REG 0xB9:XID配置1

<table><tr><td>Bit</td><td>Description</td><td>R/W</td><td>Default</td></tr><tr><td>7-0</td><td>XID 配置 XID[23:16]</td><td>R/W</td><td>0x0</td></tr></table>

### 2.42. REG 0xBA:XID配置2

<table><tr><td>Bit</td><td>Description</td><td>R/W</td><td>Default</td></tr><tr><td>7-0</td><td>XID 配置 XID[15:8]</td><td>R/W</td><td>0x0</td></tr></table>

### 2.43. REG 0xB: XID 配置 3

<table><tr><td>Bit</td><td>Description</td><td>R/W</td><td>Default</td></tr><tr><td>7-0</td><td>XID 配置 XID[7:0]</td><td>R/W</td><td>0x0</td></tr></table>

### 2.44. REG 0xBC: PID 配置 0

<table><tr><td>Bit</td><td>Description</td><td>R/W</td><td>Default</td></tr><tr><td>7-0</td><td>PID 配置 PID[15:8]</td><td>R/W</td><td>0x0</td></tr></table>

### 2.45. REG 0xBD: PID 配置 1

<table><tr><td>Bit</td><td>Description</td><td>R/W</td><td>Default</td></tr><tr><td>7-0</td><td>PID 配置 PID[7:0]</td><td>R/W</td><td>0x0</td></tr></table>

### 2.46. REG 0xBE: SVID 配置 0

<table><tr><td>Bit</td><td>Description</td><td>R/W</td><td>Default</td></tr><tr><td>7-0</td><td>SVID 配置 SVID[15:8]</td><td>R/W</td><td>0x0</td></tr></table>

### 2.47. REG 0xBF: SVID 配置 1

<table><tr><td>Bit</td><td>Description</td><td>R/W</td><td>Default</td></tr><tr><td>7-0</td><td>SVID 配置 SVID[7:0]</td><td>R/W</td><td>0x0</td></tr></table>

## 免责声明

珠海智融科技股份有限公司（以下简称“智融科技”）可能随时对所提供的产品、服务及本文件作出修改或更新，且不另行通知。客户应在下订单前获取最新的相关信息，并确认这些信息是否完整且是最新的。

本文件所含信息仅为您提供便利，智融科技不对这些信息作任何明示或暗示、书面或口头、法定或其他形式的声明或保证，包括不但限于产品的用途、特性、使用情况、适销性等方面。智融科技对这些信息及不合理使用这些信息而引起的后果不承担任何责任。

智融科技对应用帮助或客户产品设计不承担任何义务。客户应对其使用智融科技的产品和应用自行负责。客户应提供充分的设计与操作安全验证，且保证在将智融产品集成到任何应用

程序中时不会侵犯第三方知识产权，如发生侵权行为智融科技对此概不承担任何责任。

在转售智融科技产品时，如果对该产品参数及其陈述相比存在差异或虚假成分，则会自动丧失智融科技相关产品的所有明示或暗示授权，且对此不正当的、欺诈性商业行为，智融科技保留采取一切合法方式维权。智融科技对任何此类虚假陈述均不承担任何责任或义务。

本文件仅在没有对内容进行任何篡改且带有相关授权、条件、限制和声明的情况下才允许进行复制，否则智融科技有权追究其法律责任。智融科技对此类篡改过的文件不承担任何责任或义务。复制如涉及第三方的信息应当服从额外的限制条件。