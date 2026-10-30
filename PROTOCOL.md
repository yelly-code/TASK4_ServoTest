# 串口协议（TASK4_ServoTest）

串口：**115200 8N1**，直接发**文本**：一个方向字母 + 十进制数字。

```
d30     →  Yaw 右转 30 度
a5      →  Yaw 左转 5 度
```

数字 1~3 位，最大 180（超过按 180 处理）。
**数字后面按回车，或者停顿 0.3 秒自动执行**，两种都行。

## 指令表

| 指令 | 含义 |
|---|---|
| `a10` | Yaw 左转 10 度 |
| `d30` | Yaw 右转 30 度 |
| `w15` | Pitch 上转 15 度 |
| `s45` | Pitch 下转 45 度 |
| `y90` | Yaw 直接转到 90 度（绝对角度） |
| `p45` | Pitch 直接转到 45 度（绝对角度） |
| `m` | 两轴回中 90 度（不需要数字） |
| `t` | 舵机自检扫动一遍 |

`a/d/w/s` 是**相对移动**（在当前角度的基础上加/减），`y/p` 是**直接设定**。

角度统一限幅在 0~180 度，转到边界就停住，不会硬顶。

## 回显

每执行一条指令，板子会回：

```
RECV d30
当前: YAW=120  PIT_SERVO=090  ROLL=+12.3  PITCH=-45.6
```

`RECV d30` 是确认收到的指令，第二行是执行后的状态（含 OLED 上的 roll/pitch 角度）。

## 例子

```
d30        Yaw 从 90 转到 120
a90        Yaw 从 120 转到 30
w20        Pitch 从 90 到 110
y0         Yaw 直接回到 0 度（最左）
m          两轴都回 90 度
```

## Python 测试

```python
import serial, time

ser = serial.Serial('COM3', 115200, timeout=0.3)
time.sleep(0.5)

def send(text):
    ser.write(text.encode())
    time.sleep(0.1)
    print(ser.read(300).decode('utf-8', 'ignore'), end='')

send('d30')   # 右转 30
send('w15')   # 上转 15
send('y90')   # Yaw 回 90
send('m')     # 回中
```

需要 `pip install pyserial`，COM 口号看设备管理器。

## 说明

- 大小写都行（`D30` 等同 `d30`）；
- 连续输入会依次执行，例如 `d10d10d10` 等于右转三次；
- 万一输错（比如只发了 `d` 没有数字），不会动作，直接发新的指令即可。
