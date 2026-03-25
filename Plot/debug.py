import serial
import struct
import time

# ================== 配置 ==================
COM_PORT = 'COM4'          # 改成你的实际端口
BAUDRATE = 115200
PACKET_SIZE = 16           # 假设你的结构体大小为16字节
# ==========================================

print(f"1. 尝试打开串口 {COM_PORT} ...")
try:
    ser = serial.Serial(COM_PORT, BAUDRATE, timeout=1)
    print(f"2. 串口打开成功，状态: {ser.is_open}")
except Exception as e:
    print(f"打开串口失败: {e}")
    exit()

buffer = b''
packet_count = 0

print("3. 开始循环读取数据...")

while True:
    # 读取当前所有可用数据
    if ser.in_waiting > 0:
        chunk = ser.read(ser.in_waiting)
        print(f"4. 读取到 {len(chunk)} 字节原始数据: {chunk.hex()}")
        buffer += chunk
        print(f"   当前缓冲区大小: {len(buffer)} 字节")

        # 只要缓冲区足够，就处理完整包
        while len(buffer) >= PACKET_SIZE:
            packet = buffer[:PACKET_SIZE]
            buffer = buffer[PACKET_SIZE:]
            packet_count += 1
            print(f"5. 提取第 {packet_count} 个完整包，hex: {packet.hex()}")

            # 尝试解析，假设四个float
            try:
                iq, id_, speed, angle = struct.unpack('<ffff', packet)
                print(f"   解析成功: iq={iq:.3f}, id={id_:.3f}, speed={speed:.1f}, angle={angle:.3f}")
            except struct.error as e:
                print(f"   解析失败: {e}")
    else:
        # 没有数据时短暂休眠，避免CPU占满
        time.sleep(0.01)