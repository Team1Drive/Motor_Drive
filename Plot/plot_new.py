import serial
import struct
import threading
from collections import deque
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# ======================= 用户配置区 =======================
COM_PORT = 'COM4'          # 你的虚拟串口号（如 COM5、/dev/ttyACM0）
BAUDRATE = 115200          # 波特率（与STM32设置一致，但虚拟串口实际无影响）
SYNC_BYTES = b'\xAA\x55'   # 帧头

# 定义字段列表：（名称，字节数，struct格式字符，掩码位）
FIELDS = [
    ('HALLBIN',    1, 'B', 1 << 0),
    ('HALLDEC',    1, 'B', 1 << 1),
    ('RPM',        4, 'f', 1 << 2),
    ('POS',        2, 'H', 1 << 3),
    ('DUTY_A',     4, 'f', 1 << 4),
    ('DUTY_B',     4, 'f', 1 << 5),
    ('DUTY_C',     4, 'f', 1 << 6),
    ('IA',         4, 'f', 1 << 7),
    ('IB',         4, 'f', 1 << 8),
    ('IC',         4, 'f', 1 << 9),
    ('VA',         4, 'f', 1 << 10),
    ('VB',         4, 'f', 1 << 11),
    ('VBATT',      4, 'f', 1 << 12),
    ('IBATT',      4, 'f', 1 << 13),
    ('IA_RAW',     2, 'H', 1 << 14),
    ('IB_RAW',     2, 'H', 1 << 15),
    ('IC_RAW',     2, 'H', 1 << 16),
    ('VA_RAW',     2, 'H', 1 << 17),
    ('VB_RAW',     2, 'H', 1 << 18),
    ('VBATT_RAW',  2, 'H', 1 << 19),
    ('IBATT_RAW',  2, 'H', 1 << 20),
]

# 选择要显示的字段（从上面列表中选）
PLOT_FIELDS = ['RPM', 'IA', 'IB', 'IC', 'VA', 'VB', 'VBATT', 'IBATT']   # 示例：转速和三相电流

# 每个字段的Y轴范围（可选，根据实际量程调整）
Y_LIMITS = {
    'RPM': (-4000, 4000),
    'IA':  (-3, 3),
    'IB':  (-3, 3),
    'IC':  (-3, 3),
    'VA':  (-25, 25),
    'VB':  (-25, 25),
    'VBATT':  (0, 25),
    'IBATT': (-3, 3),
}

WINDOW_SIZE = 200          # 波形窗口显示的点数
# =========================================================

# 为每个字段创建一个固定长度的队列（用于存储最近的数据）
data_queues = {name: deque(maxlen=WINDOW_SIZE) for name, _, _, _ in FIELDS}

# 打开串口
ser = serial.Serial(COM_PORT, BAUDRATE, timeout=1)

# -------------------- 数据包解析函数 --------------------
def parse_packet(buffer, start_idx):
    """
    从 buffer[start_idx:] 尝试解析一个完整的数据包
    返回 (数据字典, 结束索引) 或 (None, 起始索引+1) 如果失败
    """
    # 检查是否有足够空间读取帧头 + mask
    if len(buffer) < start_idx + 2 + 4:
        return None, start_idx

    # 查找帧头
    if buffer[start_idx:start_idx+2] != SYNC_BYTES:
        return None, start_idx + 1   # 跳过当前字节，继续寻找

    # 读取 mask (小端32位)
    mask_bytes = buffer[start_idx+2:start_idx+6]
    mask = struct.unpack('<I', mask_bytes)[0]

    # 开始解析数据区
    pos = start_idx + 6
    data = {}

    for name, size, fmt, bit in FIELDS:
        if mask & bit:                     # 该字段存在
            if len(buffer) < pos + size:   # 数据不足，等待后续字节
                return None, start_idx
            raw = buffer[pos:pos+size]
            # 根据格式解包
            if fmt == 'f':
                val = struct.unpack('<f', raw)[0]
            elif fmt == 'B':
                val = struct.unpack('<B', raw)[0]
            elif fmt == 'H':
                val = struct.unpack('<H', raw)[0]
            else:
                val = None
            data[name] = val
            pos += size
        # 字段不存在则跳过，不移动指针

    return data, pos   # 成功解析一个完整包

# -------------------- 后台串口读取线程 --------------------
def serial_reader():
    """持续读取串口数据，解析并存入队列"""
    buffer = bytearray()
    while True:
        chunk = ser.read(ser.in_waiting or 1)   # 非阻塞读取
        if not chunk:
            continue
        buffer.extend(chunk)

        idx = 0
        while idx < len(buffer):
            data, new_idx = parse_packet(buffer, idx)
            if data is not None:
                # 将解析出的数据存入各字段的队列
                for name, val in data.items():
                    if name in data_queues:
                        data_queues[name].append(val)
                idx = new_idx
            else:
                # 未找到完整包，退出循环，等待更多数据
                break

        # 丢弃已经处理过的字节（保留不完整部分）
        if idx > 0:
            buffer = buffer[idx:]

# 启动后台读取线程（守护线程，主程序退出时自动终止）
reader_thread = threading.Thread(target=serial_reader, daemon=True)
reader_thread.start()

# -------------------- 实时绘图 --------------------
num_plots = len(PLOT_FIELDS)
if num_plots == 0:
    print("错误：PLOT_FIELDS 为空，请在配置区添加要显示的字段")
    exit()

fig, axes = plt.subplots(num_plots, 1, sharex=True)
if num_plots == 1:
    axes = [axes]

lines = []
for i, field in enumerate(PLOT_FIELDS):
    ax = axes[i]
    line, = ax.plot([], [], label=field)
    lines.append(line)
    ax.set_ylabel(field)
    ax.grid(True)
    ax.legend(loc='upper right')
    if field in Y_LIMITS:
        ax.set_ylim(*Y_LIMITS[field])
axes[-1].set_xlabel('Sample')

def animate(frame):
    """动画回调函数，更新每个子图的数据"""
    for i, field in enumerate(PLOT_FIELDS):
        queue = data_queues.get(field)
        if queue and len(queue) > 0:
            ydata = list(queue)
            xdata = list(range(len(ydata)))
            lines[i].set_data(xdata, ydata)
            axes[i].set_xlim(0, max(len(ydata), WINDOW_SIZE))
    return lines

ani = FuncAnimation(fig, animate, interval=50, blit=False)  # 50ms刷新一次
plt.tight_layout()
plt.show()

# 程序结束时关闭串口（可选）
ser.close()