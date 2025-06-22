'''
Author: IcyClouda 2330329778@qq.com
Date: 2025-06-22 00:57:49
LastEditors: IcyClouda 2330329778@qq.com
LastEditTime: 2025-06-22 03:31:59
FilePath: \Wheeled Bipedal Robot\Communication\Keyboard.py
Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
'''
import serial
import keyboard
import time
import tkinter as tk
from threading import Thread, Lock

import serial.tools.list_ports

# 配置串口参数
BAUD_RATE = 115200    # 波特率，需与单片机配置一致

# 自动检测USB串行设备并提供选择
def find_usb_serial_port():
    ports = serial.tools.list_ports.comports()
    if not ports:
        return None
    
    # 尝试多种可能的描述匹配
    possible_descriptions = [
        "USB 串行设备",  # 中文Windows
        "USB Serial Device",  # 英文Windows
        "USB Serial", 
        "USB UART",
        "CP210",  # 常见USB转串口芯片
        "CH340"   # 常见USB转串口芯片
    ]
    
    # 首先尝试匹配已知描述
    matched_ports = []
    for port in ports:
        for desc in possible_descriptions:
            if desc in port.description:
                matched_ports.append(port)
                break
    
    # 如果没有匹配的描述，使用所有可用串口
    if not matched_ports:
        matched_ports = ports
    
    # 如果只有一个可用串口，直接返回
    if len(matched_ports) == 1:
        print(f"使用唯一可用串口: {matched_ports[0].device} ({matched_ports[0].description})")
        return matched_ports[0].device
    
    # 多个串口可用时，让用户选择
    print("检测到多个串口，请选择要使用的串口:")
    for i, port in enumerate(matched_ports):
        print(f"{i+1}. {port.device} - {port.description}")
    
    while True:
        try:
            choice = int(input("请输入序号选择串口: "))
            if 1 <= choice <= len(matched_ports):
                selected_port = matched_ports[choice-1]
                print(f"已选择串口: {selected_port.device} ({selected_port.description})")
                return selected_port.device
            else:
                print(f"无效选择，请输入1-{len(matched_ports)}之间的数字")
        except ValueError:
            print("请输入有效的数字")

# 初始化串口（带重试机制）
def init_serial():
    port = find_usb_serial_port()
    if not port:
        raise ValueError("未找到USB串行设备")
    
    max_retries = 5
    retry_delay = 1  # 秒
    
    for attempt in range(max_retries):
        try:
            # 添加额外的配置参数解决访问问题
            ser = serial.Serial(
                port=port,
                baudrate=BAUD_RATE,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=2,  # 增加超时时间
                write_timeout=2,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False
            )
            print(f"串口 {port} 已连接，波特率: {BAUD_RATE}")
            
            # 添加延迟让端口稳定
            time.sleep(1)
            return ser
        except serial.SerialException as e:
            if attempt < max_retries - 1:
                print(f"无法打开串口 {port} (尝试 {attempt+1}/{max_retries}): {e}")
                print(f"{retry_delay}秒后重试...")
                time.sleep(retry_delay)
                retry_delay *= 2  # 指数退避
            else:
                raise ValueError(f"无法打开串口 {port} (尝试 {max_retries}次后失败): {e}")

# 发送数据到单片机
def send_to_device(ser, data):
    try:
        ser.write(bytes([data]))
        return True
    except serial.SerialException as e:
        print(f"串口发送失败: {e}")
        return False

class KeyboardGUI:
    def __init__(self, ser):
        self.ser = ser
        self.key_state = 0
        self.emergency_stop = False  # 急停状态
        self.received_data = []
        self.lock = Lock()
        
        self.root = tk.Tk()
        self.root.title("键盘监控 - 单片机通信")
        self.root.geometry("600x400")
        
        # 顶部状态栏 - 急停指示灯
        top_frame = tk.Frame(self.root)
        top_frame.pack(side=tk.TOP, fill=tk.X, padx=10, pady=5)
        
        tk.Label(top_frame, text="急停状态:", font=("Arial", 12)).pack(side=tk.LEFT)
        self.status_canvas = tk.Canvas(top_frame, width=30, height=30, bg='white')
        self.status_canvas.pack(side=tk.RIGHT, padx=5)
        self.status_light = self.status_canvas.create_oval(5, 5, 25, 25, fill="green")
        tk.Label(top_frame, text="正常", font=("Arial", 12)).pack(side=tk.RIGHT)
        
        # 添加状态说明
        status_note = tk.Label(top_frame, text="(ENTER: 急停 | ENTER+SPACE: 恢复)", font=("Arial", 10), fg="gray")
        status_note.pack(side=tk.RIGHT, padx=10)
        
        # 更新状态灯
        self.update_status_light()
        
        # 左侧面板 - 按键状态
        left_frame = tk.Frame(self.root)
        left_frame.pack(side=tk.LEFT, padx=10, pady=10, fill=tk.BOTH, expand=True)
        
        tk.Label(left_frame, text="按键状态", font=("Arial", 14, "bold")).pack(pady=5)
        
        self.key_frame = tk.Frame(left_frame)
        self.key_frame.pack(pady=10)
        
        self.key_labels = {}
        keys = [('W', 0), ('A', 1), ('S', 2), ('D', 3)]
        for key, bit in keys:
            frame = tk.Frame(self.key_frame)
            frame.pack(pady=5)
            tk.Label(frame, text=key, font=("Arial", 16, "bold"), width=3).pack(side=tk.LEFT)
            label = tk.Label(frame, text="未按下", fg="red", font=("Arial", 14))
            label.pack(side=tk.LEFT)
            self.key_labels[bit] = label
        
        # 右侧面板 - 接收数据
        right_frame = tk.Frame(self.root)
        right_frame.pack(side=tk.RIGHT, padx=10, pady=10, fill=tk.BOTH, expand=True)
        
        tk.Label(right_frame, text="单片机返回数据", font=("Arial", 14, "bold")).pack(pady=5)
        
        self.data_frame = tk.Frame(right_frame)
        self.data_frame.pack(fill=tk.BOTH, expand=True)
        
        self.data_text = tk.Text(self.data_frame, height=15, state=tk.DISABLED)
        scrollbar = tk.Scrollbar(self.data_frame, command=self.data_text.yview)
        self.data_text.config(yscrollcommand=scrollbar.set)
        
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.data_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        
        # 启动按键监控和串口读取线程
        self.running = True
        Thread(target=self.monitor_keys, daemon=True).start()
        Thread(target=self.read_serial, daemon=True).start()
        
        # 关闭窗口时清理资源
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
    
    def update_key_state(self, key_state):
        """更新按键状态显示"""
        for bit, label in self.key_labels.items():
            if key_state & (1 << bit):
                label.config(text="按下", fg="green")
            else:
                label.config(text="未按下", fg="red")
    
    def update_status_light(self):
        """更新急停状态指示灯"""
        if self.emergency_stop:
            self.status_canvas.itemconfig(self.status_light, fill="red")
            self.status_canvas.create_text(15, 15, text="停", fill="white", font=("Arial", 10, "bold"))
        else:
            self.status_canvas.itemconfig(self.status_light, fill="green")
            self.status_canvas.create_text(15, 15, text="行", fill="white", font=("Arial", 10, "bold"))
    
    def add_received_data(self, data, packet_size):
        """添加接收到的数据到显示区域，并显示数据包大小"""
        self.data_text.config(state=tk.NORMAL)
        
        if packet_size > 0:
            # 正常数据包
            self.data_text.insert(tk.END, f"[{packet_size}字节] 接收: {data}\n")
        else:
            # 错误信息
            self.data_text.insert(tk.END, f"错误: {data}\n", "error")
            self.data_text.tag_config("error", foreground="red")
        
        self.data_text.see(tk.END)  # 滚动到底部
        self.data_text.config(state=tk.DISABLED)
    
    def monitor_keys(self):
        """监控键盘按键状态"""
        last_send_time = time.time()
        
        while self.running:
            # 检测急停按键组合
            enter_pressed = keyboard.is_pressed('enter')
            space_pressed = keyboard.is_pressed('space')
            
            # 处理急停逻辑
            if enter_pressed and not space_pressed and not self.emergency_stop:
                # 按下ENTER进入急停状态
                self.emergency_stop = True
                self.update_status_light()
                print("进入急停状态!")
                # 发送0状态给单片机
                if self.ser and self.ser.is_open:
                    try:
                        self.ser.write(bytes([0]))
                    except serial.SerialException:
                        pass
            elif enter_pressed and space_pressed and self.emergency_stop:
                # 同时按下ENTER+SPACE退出急停状态
                self.emergency_stop = False
                self.update_status_light()
                print("退出急停状态，恢复正常操作")
                last_send_time = time.time()  # 重置发送计时器
            
            # 如果不是急停状态，正常检测WASD按键
            if not self.emergency_stop:
                w_pressed = 1 if keyboard.is_pressed('w') else 0
                a_pressed = 1 if keyboard.is_pressed('a') else 0
                s_pressed = 1 if keyboard.is_pressed('s') else 0
                d_pressed = 1 if keyboard.is_pressed('d') else 0
                
                new_state = (w_pressed << 0) | (a_pressed << 1) | (s_pressed << 2) | (d_pressed << 3)
                
                # 更新按键状态显示
                if new_state != self.key_state:
                    self.key_state = new_state
                    self.update_key_state(new_state)
                
                # 每5ms发送一次当前状态 (200Hz)
                current_time = time.time()
                if current_time - last_send_time >= 0.005:  # 5ms
                    if self.ser and self.ser.is_open:
                        try:
                            self.ser.write(bytes([self.key_state]))
                        except serial.SerialException:
                            pass
                    last_send_time = current_time
            else:
                # 急停状态下发送全0状态
                if self.key_state != 0:
                    self.key_state = 0
                    self.update_key_state(0)
                    if self.ser and self.ser.is_open:
                        try:
                            self.ser.write(bytes([0]))
                        except serial.SerialException:
                            pass
            
            time.sleep(0.001)  # 更短的睡眠时间以提高响应速度
    
    def read_serial(self):
        """从串口读取数据（固定4字节包大小）"""
        PACKET_SIZE = 4  # 固定数据包大小
        while self.running:
            if self.ser and self.ser.is_open:
                try:
                    if self.ser.in_waiting >= PACKET_SIZE:
                        data = self.ser.read(PACKET_SIZE)
                        self.add_received_data(data.hex(' '), PACKET_SIZE)  # 以十六进制格式显示
                    # 如果有数据但不足一个包，读取并显示错误
                    elif self.ser.in_waiting > 0:
                        partial_data = self.ser.read(self.ser.in_waiting)
                        self.add_received_data(f"数据包大小错误: 期望{PACKET_SIZE}字节, 收到{len(partial_data)}字节", 0)
                except serial.SerialException:
                    pass
            time.sleep(0.01)  # 10ms检测周期
    
    def on_closing(self):
        """关闭窗口时的清理操作"""
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.root.destroy()
    
    def run(self):
        """运行GUI主循环"""
        self.root.mainloop()

# 主程序
def main():
    # 初始化串口
    try:
        ser = init_serial()
    except ValueError as e:
        print(e)
        return
    
    print("开始监听WASD按键，启动GUI界面...")
    
    # 启动GUI
    app = KeyboardGUI(ser)
    app.run()
    
    print("程序已终止")


if __name__ == "__main__":
    main()
