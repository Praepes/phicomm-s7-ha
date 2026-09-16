# Phicomm S7 Body Fat Scale — ESP8266 Replacement Firmware

斐讯 S7 体脂秤 ESP8266 替代固件，原生支持 Home Assistant MQTT 集成。

斐讯云服务已关闭，此固件让你的 S7 体脂秤重获新生。

![License](https://img.shields.io/badge/license-MIT-blue)
![Platform](https://img.shields.io/badge/platform-ESP8266-orange)
![HA](https://img.shields.io/badge/Home%20Assistant-MQTT%20Discovery-41BDF5)

## 功能

- **体重测量** — MCU UART 协议解析，实时体重读数
- **8 电极阻抗测量** — CS1258 BIA AFE 驱动，9 通道扫描（双脚 + 双手握柄）
- **体成分分析** — 体脂率、水分率、BMI、去脂体重、基础代谢、肌肉率、骨量
- **Home Assistant 集成** — MQTT Discovery 自动发现，零配置
- **HA 侧参数配置** — 年龄、身高、性别可直接在 HA 中修改
- **加热器控制** — GPIO13 脚垫加热，HA 开关实体
- **体重消抖** — 体重稳定 3 秒后才触发阻抗扫描，避免误触发
- **中文 Web UI** — 简洁的配置界面，调试信息折叠
- **OTA 升级** — 支持 ArduinoOTA 和网页升级

## 截图

### Web UI
浏览器访问秤的 IP 地址即可看到中文界面：
- 体重 / 阻抗 / 体脂率 / 水分率 / BMI / 去脂体重 / 基础代谢 / 肌肉率 / 骨量
- WiFi / MQTT / 用户资料 / 加热器 / OTA 设置
- 高级调试信息（折叠）

### Home Assistant
MQTT Discovery 自动创建以下实体：

| 类型 | 实体 | 说明 |
|------|------|------|
| sensor | 体重 | kg |
| sensor | BMI | — |
| sensor | 体脂率 | % |
| sensor | 水分率 | % |
| sensor | 阻抗 | Ω |
| sensor | 去脂体重 | kg |
| sensor | 基础代谢 | kcal |
| sensor | 肌肉率 | % |
| sensor | 骨量 | kg |
| sensor | WiFi RSSI | dBm |
| binary_sensor | MCU 在线 | — |
| binary_sensor | CS1258 | — |
| switch | 加热器 | 开/关 |
| button | 测量阻抗 | 手动触发 |
| number | 年龄 | 输入框 |
| number | 身高 | cm 输入框 |
| select | 性别 | 男/女 |

## 硬件

### 斐讯 S7 硬件架构

```
┌─────────────────────────────────────┐
│  Phicomm S7 Body Fat Scale          │
│                                     │
│  ┌──────────┐    UART     ┌──────┐  │
│  │  MCU     │◄──────────►│ESP8266│  │
│  │(称重ADC) │  115200     │CSM64F│  │
│  │          │  C5/C6 协议 │ 02   │  │
│  └──────────┘             └──┬───┘  │
│                              │GPIO  │
│  ┌──────────┐   bit-bang   │      │
│  │  CS1258  │◄────────────┘      │
│  │(BIA AFE) │  CS=16,CLK=14      │
│  │ 8电极    │  DIO=12            │
│  └──────────┘                     │
│                                     │
│  加热器 ──── GPIO13 (高电平有效)     │
│  4脚垫电极 + 4握柄电极 = 8电极      │
└─────────────────────────────────────┘
```

- **ESP8266** (CSM64F02) — WiFi + MQTT + Web UI
- **MCU** — 称重传感器 ADC，通过 UART 发送体重数据
- **CS1258** — Chipsea BIA 模拟前端，8 电极阻抗测量
- **加热器** — GPIO13，脚垫加热

## 安装

### 方法一：直接刷入编译好的固件

1. 从 [Releases](../../releases) 下载 `phicomm_s7_v1.0.bin`
2. 使用 `esptool.py` 刷入：
   ```bash
   esptool.py --port COMx --baud 115200 write_flash 0x0 phicomm_s7_v1.0.bin
   ```
3. 秤重启后会创建热点 `S7-xxxxxx`，连接后访问 `192.168.4.1` 配置 WiFi

### 方法二：PlatformIO 编译

```bash
git clone https://github.com/Praepes/phicomm-s7-ha.git
cd phicomm-s7-ha
pio run -e s7_esp8266_1m          # 编译
pio run -e s7_esp8266_1m -t upload # 串口刷入
```

#### OTA 升级（已刷入固件后）

编辑 `platformio.ini` 中的 `upload_port` 为秤的 IP 地址，然后：
```bash
pio run -e s7_esp8266_1m_ota -t upload
```

或通过 Web UI：浏览器访问 `http://<秤IP>/update`

## 配置

### 首次配置

1. 刷入固件后，秤会创建 WiFi 热点 `S7-xxxxxx`
2. 手机/电脑连接该热点
3. 浏览器访问 `192.168.4.1`
4. 设置你的 WiFi SSID 和密码
5. 设置 MQTT 服务器地址和凭据
6. 设置用户资料（性别、年龄、身高）
7. 如需加热器，将 GPIO 设为 `13`
8. 点击「保存设置」

### Home Assistant MQTT 配置

确保你的 HA 已安装并配置 [MQTT 集成](https://www.home-assistant.io/integrations/mqtt/)。

秤连接 MQTT 后会自动发布 Discovery 消息，所有实体会自动出现在 HA 中，归属于 "Phicomm S7" 设备。

**MQTT Topic 结构：**

```
s7/phicomm_s7/state          # 状态 JSON（retained）
s7/phicomm_s7/availability   # online/offline
s7/phicomm_s7/cmd/impedance  # 触发阻抗扫描
s7/phicomm_s7/cmd/heater     # ON/OFF
s7/phicomm_s7/cmd/age        # 设置年龄 (10-99)
s7/phicomm_s7/cmd/height     # 设置身高 (cm 或 m)
s7/phicomm_s7/cmd/sex        # 男/女 或 male/female
```

### 使用方式

1. 踩上秤 → 体重实时更新
2. 体重稳定 3 秒后 → 自动进行阻抗扫描
3. 扫描完成 → 体成分数据发布到 MQTT
4. 下秤后 → HA 保留最后一次测量结果

## 体成分算法

使用 [bodymiscale](https://github.com/dckiller51/bodymiscale) / Xiaomi Zepp Life 验证公式：

- **LBM (去脂体重)**: 基于身高、体重、阻抗、年龄的多元回归
- **体脂率**: Xiaomi / Zepp Life 公式，区分男女
- **水分率**: `(100 - 体脂率) × 0.7` + 修正
- **BMR (基础代谢)**: Katch-McArdle 公式: `370 + 21.6 × LBM`
- **骨量**: bodymiscale 公式
- **肌肉率**: `体重 - 脂肪质量 - 骨量`

## 自定义

### config.h（可选）

创建 `src/config.h` 覆盖默认值：

```cpp
#define S7_DEVICE_ID   "my_scale"
#define S7_DEVICE_NAME "My Scale"
```

### MCU UART 协议

秤的 MCU 通过 UART 发送称重数据，帧格式：

```
C5 <len> <cmd> <payload...> <xor>
```

- `cmd = 0x10`：体重数据帧
- 默认 `payload[0]:payload[1]` 为体重高低字节，÷ 100 = kg
- 可在 Web UI 的「高级设置」中调整字节偏移和除数

### CS1258 通道映射

| 通道 | 代码 | 路径 |
|------|------|------|
| 0xEB | lrLeg | 左脚→右脚 |
| 0xBE | rlLeg | 右脚→左脚 |
| 0x14 | lrHand | 左手→右手 |
| 0xDB | rLeg | 右脚 |
| 0x8E | lLeg | 左脚 |
| 0x3C | lhLl | 左手→左脚 |
| 0x28 | lhRl | 左手→右脚 |
| 0x7D | rhLl | 右手→左脚 |
| 0x69 | rhRl | 右手→右脚 |

## FAQ

**Q: 阻抗读数为 0？**
A: 确保双脚踩在金属电极上、双手握住握柄。CS1258 需要人体导通才能测量。

**Q: HA 中看不到实体？**
A: 确认 MQTT 集成已配置，秤的 MQTT 状态灯（Web UI）为绿色。可尝试在 HA 中重新加载 MQTT 集成。

**Q: 加热器不工作？**
A: 在 Web UI 中将加热器 GPIO 设为 `13`。默认公版固件加热器是禁用的 (`-1`)。

**Q: MCU 离线？**
A: MCU 在无操作 ~30 秒后进入休眠。踩上秤即可唤醒。

**Q: 体成分数据不准确？**
A: 可在 Web UI 的高级设置中调整阻抗系数 (`zScale`) 和偏移 (`zOffset`)。

## License

MIT — 详见 [LICENSE](LICENSE)

## 致谢

- [bodymiscale](https://github.com/dckiller51/bodymiscale) — 体成分算法参考
- [PubSubClient](https://github.com/knolleary/pubsubclient) — MQTT 库
- [Chipsea CS1258](https://www.chipsea.com/) — BIA AFE 数据手册
