# SW2303 寄存器列表

# 1. 版本历史

V1.0 初始版本，针对芯片版本号 0；  
V1.1 针对芯片版本号1；  
V1.2 更新页眉图标；  
V1.3 更新默认值及 REG0xAE[2:1]；  
V1.4 更换文档模板；

# 2. 寄存器

注意 :未定义的寄存器或bit 不能被改写

# 2.1. REG 0x01: 芯片版本

<html><body><table><tr><td>Bit</td><td>Description</td><td></td><td>R/W</td><td>Default</td></tr><tr><td>7-2</td><td>/</td><td></td><td></td><td>/</td></tr><tr><td>1-0</td><td></td><td></td><td>R</td><td>0x1</td></tr></table></body></html>

# 2.2. REG 0x03: 设置电压高 8 位

<html><body><table><tr><td>Bit</td><td>Description</td><td></td><td>R/W</td><td>Default</td></tr><tr><td>7-0</td><td colspan="3">dac_vol[11:4] R J: dac_vol[11:0]*10mV</td><td>0x0</td></tr></table></body></html>

# 2.3. REG 0x04: 设置电压低 4 位

<html><body><table><tr><td>Bit</td><td>Description</td><td>R/W</td><td>Default</td></tr><tr><td>7-4</td><td>dac_vol[3:0] Ji: dac_vol[11:0]*10mV</td><td>R</td><td>0x0</td></tr><tr><td>3-0</td><td>/</td><td>/</td><td>/</td></tr></table></body></html>

# 2.4. REG 0x05: 设置限流

<html><body><table><tr><td>Bit</td><td>Description</td><td>R/W</td><td>Default</td></tr><tr><td>7</td><td>/</td><td></td><td></td></tr><tr><td>6-0</td><td>ctrl_icc[6:0]</td><td>R</td><td>0x0</td></tr></table></body></html>

<html><body><table><tr><td> i   FR it:</td><td>1000mA+ctrl_icc[6:0]*50mA</td><td></td><td></td></tr></table></body></html>

2.5. REG 0x06: 快充指示  

<html><body><table><tr><td>Bit</td><td>Description</td><td>R/W</td><td>Default</td></tr><tr><td>7</td><td>xtF R3tJiX 0: *TDiX 1: JTbiX</td><td>R</td><td>0x0</td></tr><tr><td>6</td><td>xT+E 0: *TE 1: TE</td><td>R</td><td>0x0</td></tr><tr><td>5-4</td><td>PD tiXH 1: PD 2.0 2: PD 3.0 other: Reserved t3tt iX##T</td><td>R R</td><td>0x0</td></tr><tr><td>3-0 1: QC2.0 2: QC3.0 3: FCP 4:/ 5: SCP 6: PD FIX 7: PD PPS 8: PE1.1 9: PE2.0 C: SFCP D: AFC other: reserved</td><td></td><td></td><td>0x0</td></tr></table></body></html>

# 2.6. REG 0x07: 系统状态 0

<html><body><table><tr><td>Bit</td><td>Description</td><td>R/W</td><td>Default</td></tr><tr><td>7-5</td><td>/</td><td>/</td><td></td></tr><tr><td>4</td><td>0: *#*</td><td>R</td><td>0x0</td></tr><tr><td>3</td><td>1: W## /</td><td>/</td><td>/</td></tr><tr><td rowspan="2">2</td><td>CC #</td><td>R</td><td>0x0</td></tr><tr><td>0: CC T#TJF 1: CC T#J</td><td></td><td></td></tr><tr><td>1</td><td>x*X#TFFR</td><td>R</td><td>0x0</td></tr></table></body></html>

<html><body><table><tr><td></td><td>0: *#TFT* 1: *TFF</td><td></td><td></td></tr><tr><td>0</td><td>iW 0: i 1: jTF</td><td>R</td><td>0x0</td></tr></table></body></html>

# 2.7. REG 0x0B: 系统状态 1

<html><body><table><tr><td>Bit</td><td>Description</td><td>R/W</td><td>Default</td></tr><tr><td>7-6</td><td></td><td>/</td><td></td></tr><tr><td>5</td><td>Vin #i 25V ##T 0: Vin 1EF 25V 1: Vin #F 25V</td><td>R</td><td>0x0</td></tr><tr><td>4</td><td>0: * 1:  112.5%</td><td>R</td><td>0x0</td></tr><tr><td>3</td><td>Die 0: die * 1: die i</td><td>R</td><td>0x0</td></tr><tr><td>2 / 1</td><td></td><td>/</td><td>/</td></tr><tr><td>Vin iE# 0: vin * 1: vin it</td><td> Vin E#F DAC i#REJ 20%, i Vin E</td><td>R</td><td>0x0</td></tr><tr><td>0</td><td>Vin KE#T 0: vin* 1: vin KE Vin 1 F 4V JlJiKE</td><td>R</td><td>0x0</td></tr></table></body></html>

# 2.8. REG 0x0C: 系统状态 2

<html><body><table><tr><td>Bit</td><td>Description</td><td>R/W</td><td>Default</td></tr><tr><td>7</td><td>CC1 iE#T 0: CC1 * 1: CC1 </td><td>R</td><td>0x0</td></tr><tr><td>6</td><td>CC2 iE# 0: CC2 * 1: CC2 </td><td>R</td><td>0x0</td></tr><tr><td>5</td><td>DP iE#T 0: DP * 1: DP t</td><td>R</td><td>0x0</td></tr></table></body></html>

<html><body><table><tr><td>4 1: DM </td><td>DM E#T 0: DM</td><td>R</td><td>0x0</td></tr><tr><td>3 /</td><td></td><td>/</td><td>/</td></tr><tr><td>2</td><td>0: *F1 1: 3T1E Vin 1F 3V JJiI1E</td><td>R</td><td>0x0</td></tr><tr><td>1</td><td>/</td><td></td><td></td></tr><tr><td>0</td><td>iR$HT 0: * 1: </td><td>R</td><td>0x0</td></tr></table></body></html>

# 2.9. REG 0x0D: 系统状态 3

<html><body><table><tr><td>Bit</td><td>Description</td><td>R/W</td><td>Default</td></tr><tr><td>7</td><td>#EHT 0: T 1: C gA XFiR 1 C #WA </td><td>R</td><td>0x0</td></tr><tr><td>6-0</td><td>JFR Reserved</td><td></td><td></td></tr></table></body></html>

# 2.10. REG 0x12: I2C 写使能控制 0

<html><body><table><tr><td>Bit</td><td>Description</td><td>R/W</td><td>Default</td></tr><tr><td>7-5</td><td>I2C S#*1F1E tF* reg0x14, reg0xA0~BF,$ F1F: 1.  reg0x12= 0x20; 2. Fj reg0x12=0x40;</td><td>R/W</td><td>0x0</td></tr><tr><td>4-0</td><td>3. j reg0x12= 0x80;</td><td></td><td></td></tr></table></body></html>

# 2.11. REG 0x14: 连接控制

<html><body><table><tr><td>Bit</td><td>Description</td><td>R/W</td><td>Default</td></tr><tr><td>7-3</td><td>/</td><td>/</td><td>/</td></tr><tr><td>2</td><td>*##J</td><td>R/W</td><td>0x0</td></tr></table></body></html>

<html><body><table><tr><td></td><td>0: tTFF+ 1: *</td><td></td><td></td></tr><tr><td>1</td><td>Type-C CC un-driving 1 0: F$m 1: CC un-driving 1s, Z#</td><td>R/WC</td><td>0x0</td></tr><tr><td>0</td><td></td><td></td><td></td></tr></table></body></html>

# 2.12. REG 0x15: I2C 写使能控制 1

<html><body><table><tr><td>Bit</td><td>Description</td><td>R/W</td><td>Default</td></tr><tr><td>7-5</td><td>I2C 5#*1F1 t1F reg0x16 #t F1F: 1. Fj reg0x12=0x20;</td><td> R/W</td><td>0x0</td></tr><tr><td>4-0</td><td>2. j reg0x12= 0x40; 3. reg0x12=0x80;</td><td></td><td></td></tr><tr><td></td><td></td><td></td><td></td></tr></table></body></html>

# 2.13. REG 0x16: 强制控制使能

<html><body><table><tr><td>Bit</td><td>Description</td><td>R/W</td><td>Default</td></tr><tr><td>7-4</td><td></td><td>/</td><td>/</td></tr><tr><td>3</td><td>3#$J FF i x 0: FmJ 1: 3$Ti</td><td>R/W</td><td>0x0</td></tr><tr><td>2</td><td>3$J Xi 0: F 1: 3$Xi</td><td>R/W</td><td>0x0</td></tr><tr><td>1</td><td>3$J##J DAC 0: F$mj 1: 5## dac_vol</td><td>R/W</td><td>0x0</td></tr><tr><td>0</td><td>3@#J$ FR yt 0:F#qJ 1: 3## ctr_icc</td><td>R/W</td><td>0x0</td></tr></table></body></html>

# 2.14. REG 0x30: ADC Vin 数据

<html><body><table><tr><td>Bit</td><td>Description</td><td>R/W</td><td>Default</td></tr><tr><td>7-0</td><td>Vin J 8bit 7.5*16mv/bit; ( 12bit }##J 7.5mv/bit,JL reg0x3B)</td><td>R</td><td>0x0</td></tr></table></body></html>

# 2.15. REG 0x31: ADC Vbus 数据

<html><body><table><tr><td>Bit</td><td>Description</td><td>R/W</td><td>Default</td></tr><tr><td>7-0</td><td>Vbus EJ 8bit 7.5*16mv/bit; (#X 12bit H>#*J 7.5mv/bit,,JL reg0x3B)</td><td>R</td><td>0x0</td></tr></table></body></html>

# 2.16. REG 0x33: ADC Ich 数据

![](images/382d5040a10771d9745539dc0a5589691e3b883cbeeab298e3c54bdfa3147155.jpg)

<html><body><table><tr><td>Bit</td><td>Description</td><td>R/W</td><td>Default</td></tr><tr><td rowspan="2">7-0</td><td>#  J 8bit</td><td>R</td><td>0x0</td></tr><tr><td>50mA/bit; (x 12bit J* 3.125mA/bit, ,JL reg0x3B)</td><td></td><td></td></tr></table></body></html>

# 2.17. REG 0x36: ADC Tdiet 数据

<html><body><table><tr><td>Bit</td><td>Description</td><td>R/W</td><td>Default</td></tr><tr><td>7-0</td><td>Die  J 8bit, 2.38'C/bit; (#x 12bit JJ#j 0.1488'C/bit, ,JL reg0x3B)</td><td>R</td><td>0x0</td></tr></table></body></html>

# 2.18. REG 0x3B: ADC 配置

<html><body><table><tr><td>Bit</td><td>Description</td><td>R/W</td><td>Default</td></tr><tr><td>7-5</td><td></td><td>/</td><td>/</td></tr><tr><td>2-0</td><td>ADC ##i# SH*#Z,KXJJ ADC ##$7J Reg0x3C F Reg0x3D,jILiEJJ#1TXJ Xf3tnF : 1: adc_vin[11:0], 7.5mV/bit 2: adc_vbus[11:0], 7.5mV/bit 3: adc_ich[11:0],3.125mA/ bit 4: adc_diet[11:0],0.1488'C/bit;Tdiet = (adc_diet[11:0]- 1848)/6.72'C</td><td>R/W</td><td>0x0</td></tr></table></body></html>

# 2.19. REG 0x3C: ADC 数据高 8 位

<html><body><table><tr><td>Bit</td><td>Description</td><td>R/W</td><td>Default</td></tr><tr><td rowspan="2">7-0</td><td>ADC  8bit #</td><td>R</td><td>0x0</td></tr><tr><td>adc_data[11:04]</td><td></td><td></td></tr></table></body></html>

2.20. REG 0x3D: ADC 数据低 4 位  

<html><body><table><tr><td>Bit</td><td>Description</td><td>R/W</td><td>Default</td></tr><tr><td>7-4</td><td>/</td><td></td><td>/</td></tr><tr><td>3-0</td><td>ADC 1 4bit ##7 adc_data[03:00]</td><td>R</td><td>0x0</td></tr></table></body></html>

# 2.21. REG 0xA1: 异常处理配置

<html><body><table><tr><td>Bit</td><td colspan="2">Description</td><td>R/W</td><td>Default</td></tr><tr><td>7</td><td>Die i# e 0: 1AE 1: T</td><td></td><td>R/W</td><td>0x0</td></tr><tr><td>6-0</td><td>Reserved</td><td></td><td>R/W</td><td>0x2</td></tr></table></body></html>

# 2.22. REG 0xA3: 输出电压偏移配置

<html><body><table><tr><td>Bit</td><td>Description</td><td>R/W</td><td>Default</td></tr><tr><td>7-6</td><td># i( PPS, QC3.0F) 0: 200mV 1: 0mV 2: 250mV 3: 100mV</td><td>R/W</td><td>0x3</td></tr><tr><td>5</td><td>Vin j 25V J#I1E 0: TAE 1: 1</td><td>R/W</td><td>0x1</td></tr><tr><td>4:0</td><td>Reserved 7T2</td><td>R/W</td><td>0x0</td></tr></table></body></html>

# 2.23. REG 0xA4: 线补阻抗配置

<html><body><table><tr><td>Bit</td><td>Description</td><td>R/W</td><td>Default</td></tr><tr><td rowspan="4">7-6</td><td>+H$i</td><td>R/W</td><td>0x0</td></tr><tr><td>0: 50mQ 1: 0mQ 2: 100mQ</td><td></td><td></td></tr><tr><td>3: 150mQ</td><td></td><td></td></tr><tr><td>tR#$i#3Jiti# Vin Reserved</td><td>R/W</td><td>0x0</td></tr></table></body></html>

2.24. REG 0xA5: 过压检测配置   

<html><body><table><tr><td>Bit Description</td><td></td><td>R/W</td><td>Default</td></tr><tr><td>7</td><td>DP iE$A 0: 1 1: T</td><td>R/W</td><td>0x0</td></tr><tr><td>6</td><td>DM iER$ 0: 1 1: T</td><td>R/W</td><td>0x0</td></tr><tr><td>5</td><td>CC1/CC2 i1R#1 0: 1AE 1: TE</td><td>R/W</td><td>0x0</td></tr><tr><td>4-0</td><td>Reserved</td><td>R/W</td><td>0x0</td></tr></table></body></html>

# 2.25. REG 0xA6: PD 配置 3

<html><body><table><tr><td>Bit</td><td>Description</td><td>R/W</td><td>Default</td></tr><tr><td>7</td><td>Reserved 7ET#2#i</td><td>R/W</td><td>0x1</td></tr><tr><td>6</td><td>PD 5A  emark 0:  emark 5A 1: T emark tJ D 5A</td><td>R/W</td><td>0x0</td></tr><tr><td>5-0</td><td>Reserved</td><td>R/W</td><td>0x30</td></tr></table></body></html>

# 2.26. REG 0xA8: 广播电流配置

<html><body><table><tr><td>Bit</td><td>Description</td><td>R/W</td><td>Default</td></tr><tr><td>7</td><td>PD PPS3  5A 1 0: TnE 1: 1 Q$J5A</td><td>R/W</td><td>0x0</td></tr><tr><td>6</td><td>Type-C* 0: #PDI* 1: 1.5A</td><td>R/W</td><td>0x0</td></tr><tr><td>5-0</td><td>Reserved T</td><td>R/W</td><td>0x0</td></tr></table></body></html>

2.27. REG 0xAB: 异常保护配置  

<html><body><table><tr><td>Bit</td><td>Description</td><td>R/W</td><td>Default</td></tr><tr><td>7-4</td><td>Reserved</td><td>R/W</td><td>0x8</td></tr><tr><td>3-2</td><td>Die i#R$`]R 0: 105C 1: 115C 2: 125'C</td><td>R/W</td><td>0x2</td></tr><tr><td>1-0</td><td>3: 135'C Reserved</td><td>R/W</td><td>0x0</td></tr></table></body></html>

# 2.28. REG 0xAC: 在线配置

<html><body><table><tr><td>Bit</td><td>Description</td><td>R/W</td><td>Default</td><td></td></tr><tr><td>7-3</td><td>Reserved</td><td>R/W</td><td>0x6 0x0</td><td></td></tr><tr><td>2</td><td>J>Fj#1J5RJ debounce f[J 0: 6~8s 1: 3~4s</td><td>R/W</td><td></td><td></td></tr><tr><td>1-0</td><td>#J5J A y7E2#1JFR 0: 200mA 1: 100mA 2: 300mA 3: 400mA</td><td></td><td>R/W</td><td>0x2</td></tr></table></body></html>

# 2.29. REG 0xAD: 快充配置 0

<html><body><table><tr><td>Bit</td><td>Description</td><td>R/W</td><td>Default</td></tr><tr><td>7</td><td>QC2.0/QC3.0/PD FIX F 0: 1E 1: TE</td><td>R/W</td><td>0x0</td></tr><tr><td>6-4</td><td>Reserved 7TA2x</td><td>R/W</td><td>0x0</td></tr><tr><td>3</td><td>FCP/AFC/SFCP bJiX JFR 0: 3.25A 1: 2.25A</td><td>R/W</td><td>0x0</td></tr><tr><td>2-1</td><td>Reserved</td><td>R/W</td><td>0x0</td></tr><tr><td>0</td><td>PD E#1L SCP tiX 0: T#E, QPzF PD tJiXHfaJ DqJ SCP i#R</td><td>R/W</td><td>0x0</td></tr></table></body></html>

<html><body><table><tr><td>1: #</td><td></td><td></td></tr></table></body></html>

# 2.30. REG 0xAE: 快充配置 1

<html><body><table><tr><td>Bit Description</td><td></td><td>R/W</td><td>Default</td></tr><tr><td>7-6</td><td>Reserved</td><td>R/W</td><td>0x3</td></tr><tr><td>5</td><td>iE#1E 0: #3.0V 1: #5.0V</td><td>R/W</td><td>0x0</td></tr><tr><td>4</td><td>tiXX#5V JFE 0: #JF5V 1:*5V</td><td>R/W</td><td>0x0</td></tr><tr><td>3</td><td>QC3.0 tJiX## 20V 1 0: 1 1: T*</td><td>R/W</td><td>0x0</td></tr><tr><td>2</td><td>QC2.0 tiX#20V 1E 0: TAE 1: 1</td><td>R/W</td><td>0x0</td></tr><tr><td>1</td><td>PE2.0 tiX##20V 1E 0: TAE 1: 1</td><td>R/W</td><td>0x0</td></tr><tr><td>0</td><td>#F PD tJiX## 12V lE 0: 1 1: T* 7#: #E 12V HJ, LJiFJJ#QC2.0/QC3.0/PE2.0 J 20V</td><td>R/W</td><td>0x0</td></tr></table></body></html>

# 2.31. REG 0xAF: 功率配置

<html><body><table><tr><td>Bit</td><td>Description</td><td>R/W</td><td>Default</td></tr><tr><td>7</td><td>#*i 0: y1#BH 1: *</td><td>R/W</td><td>0x0</td></tr><tr><td>6-0</td><td>#* W/bit</td><td>R/W</td><td>0x0</td></tr></table></body></html>

# 2.32. REG 0xB0: 快充配置 2

<html><body><table><tr><td>Bit</td><td>Description</td><td>R/W</td><td>Default</td></tr><tr><td>7-6</td><td>Reserved</td><td>R/W</td><td>0x3</td></tr></table></body></html>

<html><body><table><tr><td></td><td></td><td></td><td></td></tr><tr><td>5</td><td>E SCP tiX 0: 1 SCP 1: SCP</td><td>R/W</td><td>0x0</td></tr><tr><td>4</td><td>1 SCP bJiX1*AE 0: 1 SCP 1: 1 SCP</td><td>R/W</td><td>0x1</td></tr><tr><td>3 0: 1AE 1: T*</td><td>QC3.0 1 QC2.0 1</td><td>R/W</td><td>0x0</td></tr><tr><td>2 1</td><td>0: 1E 1: T</td><td>R/W R/W</td><td>0x0</td></tr><tr><td>0: 1 0</td><td>1: T BC1.2 </td><td></td><td>0x0</td></tr><tr><td>0: 1AE 1: T*</td><td>7: T1 BC1.2 j, TyJDPDM, 1 DPDM R</td><td>R/W</td><td>0x0</td></tr></table></body></html>

# 2.33. REG 0xB1: 快充配置 3

<html><body><table><tr><td>Bit</td><td>Description</td><td>R/W</td><td>Default</td></tr><tr><td>7</td><td>Vin ADC iE$1(F DAC iEi 20%) 0: 1AE 1: T</td><td>R/W</td><td>0x0</td></tr><tr><td>6</td><td>Vin #itE(F DAC iE1i 2V) 0: 1E 1:TE</td><td>R/W</td><td>0x0</td></tr><tr><td>5</td><td>EE1.2V A 0: 1E 1: T</td><td>R/W</td><td>0x0</td></tr><tr><td>4</td><td>SFCP tJiX1E 0: 1E 1: T</td><td>R/W</td><td>0x0</td></tr><tr><td>3</td><td>FCP O: 1 1: T</td><td>R/W</td><td>0x0</td></tr><tr><td>2</td><td>AFC 1E 0: 1 1: T*</td><td>R/W</td><td>0x0</td></tr></table></body></html>

<html><body><table><tr><td>1</td><td>PE 1AE 0: 1E 1: T</td><td>R/W</td><td>0x0</td></tr><tr><td>0</td><td>Reserved</td><td>R/W</td><td>0x0</td></tr></table></body></html>

# 2.34. REG 0xB2: 快充配置 4

<html><body><table><tr><td>Bit</td><td>Description</td><td>R/W</td><td>Default</td></tr><tr><td>7-6</td><td>Reserved 7T2xi</td><td>R/W</td><td>0x0</td></tr><tr><td>5-4</td><td>SCP tiXF*t#HJE 0: 5.0A 1: 4.0A 2: 2.0A</td><td>R/W</td><td>0x2</td></tr><tr><td>3 0: 2.78A 1: 2.77A</td><td>3: Reserved 25w Hf 9V PDO J</td><td>R/W</td><td>0x1</td></tr><tr><td>2-0</td><td>Reserved</td><td>R/W</td><td>0x1</td></tr></table></body></html>

# 2.35. REG 0xB3: PD 配置 0

<html><body><table><tr><td>Bit</td><td>Description</td><td>R/W</td><td>Default</td></tr><tr><td>7</td><td>Ij7 60~70W # Emarker 0:# emarker 1: T emarker </td><td>R/W</td><td>0x0</td></tr><tr><td>6</td><td>4x3J#F7 PD i#KHJXIJj rQ 0: reject #i# 1:  hardreset ## PPS</td><td>R/W</td><td>0x1</td></tr><tr><td>5</td><td>Reserved 7T</td><td>R/W</td><td>0x1</td></tr><tr><td>4</td><td>Emarker  0: 1E 1: T</td><td>R/W</td><td>0x0</td></tr><tr><td>3</td><td>PD dr_swap ## 0: T# 1: #</td><td>R/W</td><td>0x0</td></tr></table></body></html>

<html><body><table><tr><td>2</td><td>PD vconn_swap ## 0: T# 1: </td><td>R/W</td><td>0x1</td></tr><tr><td>1 Reserved</td><td>7T</td><td>R/W</td><td>0x0</td></tr><tr><td>0</td><td>PD 1 0: 1E 1: Te</td><td>R/W</td><td>0x0</td></tr></table></body></html>

# 2.36. REG 0xB4: PD 配置 1

<html><body><table><tr><td>Bit</td><td>Description</td><td> R/W</td><td>Default</td></tr><tr><td>7</td><td>PPSO/PPS1/PPS2/PPS3 0: 1:</td><td>R/W</td><td>0x0</td></tr><tr><td>6</td><td>PPS3 0: 5A 1: 3A</td><td>R/W</td><td>0x0</td></tr><tr><td>5-4</td><td>Reserved 7T2</td><td>R/W</td><td>0x0</td></tr><tr><td>3</td><td>PD discovery identity 0: T# 1: #</td><td>R/W</td><td>0x0</td></tr><tr><td>2</td><td>PD discovery SVID ## 0: Tx# 1: #</td><td>R/W</td><td>0x0</td></tr><tr><td>1-0</td><td>PD peak current 7X12x PDO#XJHJI</td><td>R/W</td><td>0x0</td></tr></table></body></html>

# 2.37. REG 0xB5: PD 配置 2

<html><body><table><tr><td>Bit</td><td>Description</td><td>R/W</td><td>Default</td></tr><tr><td>7</td><td>3.3~21V PPS E(PPS3) 0: 1 1: T</td><td>R/W</td><td>0x0</td></tr><tr><td>6</td><td>3.3~16V PPS 1#E(PPS2) 0: 1 1: T</td><td>R/W</td><td>0x0</td></tr><tr><td>5</td><td>3.3~11V PPS 1E(PPS1) 0: 1 1: T</td><td>R/W</td><td>0x0</td></tr></table></body></html>

<html><body><table><tr><td>4 0: 1E</td><td>3.3~5.9V PPS 1#E(PPSO) 1: T</td><td>R/W</td><td>0x0</td></tr><tr><td>3 0: 1 1: T</td><td>PD fixed 20V 1</td><td>R/W</td><td>0x0</td></tr><tr><td>2 1</td><td>PD fixed 15V I 0: 1 1: T PD fixed 12V I</td><td>R/W R/W</td><td>0x0</td></tr><tr><td>0: 1AE 1: T 0</td><td></td><td>R/W</td><td>0x0</td></tr><tr><td></td><td>PD fixed 9V fE 0: 1 1: T</td><td></td><td>0x0</td></tr></table></body></html>

# 2.38. REG 0xB6: VID 配置 0

<html><body><table><tr><td>Bit</td><td>Description</td><td></td><td>R/W</td><td>Default</td></tr><tr><td>7-0</td><td>vendor ID # VID[15:8]</td><td></td><td>R/W</td><td>0x0</td></tr></table></body></html>

# 2.39. REG 0xB7: VID 配置 11

<html><body><table><tr><td>Bit</td><td>Description</td><td></td><td>R/W</td><td>Default</td></tr><tr><td>7-0</td><td>vendor ID VID[7:0]</td><td></td><td>R/W</td><td>0x0</td></tr></table></body></html>

# 2.40. REG 0xB8: XID 配置 0

<html><body><table><tr><td>Bit</td><td>Description</td><td>R/W</td><td>Default</td></tr><tr><td>7-0</td><td>XID  XID[31:24]</td><td>R/W</td><td>0x0</td></tr></table></body></html>

# 2.41. REG 0xB9: XID 配置 1

<html><body><table><tr><td>Bit</td><td>Description</td><td>R/W</td><td>Default</td></tr><tr><td>7-0</td><td>XID  XID[23:16]</td><td>R/W</td><td>0x0</td></tr></table></body></html>

# 2.42. REG 0xBA: XID 配置 2

<html><body><table><tr><td>Bit</td><td>Description</td><td></td><td>R/W Default</td></tr></table></body></html>

<html><body><table><tr><td>7-0</td><td>XID # XID[15:8]</td><td>R/W</td><td>0x0</td><td></td></tr></table></body></html>

# 2.43. REG 0xBB: XID 配置 3

<html><body><table><tr><td>Bit</td><td>Description</td><td>R/W</td><td>Default</td></tr><tr><td>7-0</td><td>XID # XID[7:0]</td><td>R/W</td><td>0x0</td></tr></table></body></html>

# 2.44. REG 0xBC: PID 配置 0

<html><body><table><tr><td>Bit</td><td>Description</td><td></td><td>R/W Default</td></tr><tr><td>7-0</td><td>PID  PID[15:8]</td><td></td><td>R/W 0x0</td></tr></table></body></html>

# 2.45. REG 0xBD: PID 配置 1

<html><body><table><tr><td>Bit</td><td>Description</td><td></td><td>R/W</td><td>Default</td></tr><tr><td>7-0</td><td>PID  PID[7:0]</td><td></td><td>R/W</td><td>0x0</td></tr></table></body></html>

# 2.46. REG 0xBE: SVID 配置 0

<html><body><table><tr><td>Bit</td><td>Description</td><td>R/W</td><td>Default</td></tr><tr><td>7-0</td><td>SVID # SVID[15:8]</td><td>R/W</td><td>0x0</td></tr></table></body></html>

# 2.47. REG 0xBF: SVID 配置 1

<html><body><table><tr><td>Bit</td><td>Description</td><td></td><td>R/W</td><td>Default</td></tr><tr><td>7-0</td><td colspan="2">SVID # SVID[7:0]</td><td>R/W</td><td>0x0</td><td></td></tr></table></body></html>

# 免责声明

珠海智融科技股份有限公司（以下简称“智融科技”）可能随时对所提供的产品、服务及本文件作出修改或更新，且不另行通知。客户应在下订单前获取最新的相关信息，并确认这些信息是否完整且是最新的。

本文件所含信息仅为您提供便利，智融科技不对这些信息作任何明示或暗示、书面或口头、法定或其他形式的声明或保证，包括不但限于产品的用途、特性、使用情况、适销性等方面。智融科技对这些信息及不合理使用这些信息而引起的后果不承担任何责任。

智融科技对应用帮助或客户产品设计不承担任何义务。客户应对其使用智融科技的产品和应用自行负责。客户应提供充分的设计与操作安全验证，且保证在将智融产品集成到任何应用

程序中时不会侵犯第三方知识产权，如发生侵权行为智融科技对此概不承担任何责任。

在转售智融科技产品时，如果对该产品参数及其陈述相比存在差异或虚假成分，则会自动丧失智融科技相关产品的所有明示或暗示授权，且对此不正当的、欺诈性商业行为，智融科技保留采取一切合法方式维权。智融科技对任何此类虚假陈述均不承担任何责任或义务。

本文件仅在没有对内容进行任何篡改且带有相关授权、条件、限制和声明的情况下才允许进行复制，否则智融科技有权追究其法律责任。智融科技对此类篡改过的文件不承担任何责任或义务。复制如涉及第三方的信息应当服从额外的限制条件。