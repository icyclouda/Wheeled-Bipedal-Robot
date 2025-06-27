'''
Author: IcyClouda 2330329778@qq.com
Date: 2025-06-22 00:57:49
LastEditors: IcyClouda 2330329778@qq.com
FilePath: Wheeled Bipedal Robot\\Communication\\Keyboard.py
Description: 轮足机器人控制面板
'''
import serial
import keyboard
import time
import tkinter as tk
from threading import Thread, Lock
import struct
from tkinter import ttk
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

class KeyboardGUI:
    def __init__(self, ser):
        self.ser = ser
        self.key_state = 0
        self.emergency_stop = False  # 急停状态
        self.lock = Lock()
        self.sending_enabled = True  # 发送状态标志
        
        # 用户数据初始化
        self.userdata1 = [1.2345, 0.4321, 9.8765, 5.4678]  # 用户指定的float数组
        self.flag1 = 0  # uint8标志位
        self.flag_lock = Lock()  # 添加flag锁
        
        # 创建主窗口
        self.root = tk.Tk()
        self.root.title("轮足机器人控制面板")
        self.root.geometry("800x550")
        self.root.configure(bg="#f0f0f0")
        
        # 主框架
        main_frame = tk.Frame(self.root, bg="#f0f0f0")
        main_frame.pack(fill=tk.BOTH, expand=True, padx=20, pady=20)
        
        # 顶部状态栏 - 急停指示灯
        status_frame = tk.LabelFrame(main_frame, text="系统状态", font=("Arial", 12, "bold"), 
                                   bg="#f0f0f0", padx=10, pady=10)
        status_frame.pack(fill=tk.X, pady=(0, 15))
        
        tk.Label(status_frame, text="急停状态:", font=("Arial", 12), bg="#f0f0f0").pack(side=tk.LEFT, padx=10)
        
        # 状态指示灯
        self.status_canvas = tk.Canvas(status_frame, width=40, height=40, bg='white', highlightthickness=0)
        self.status_canvas.pack(side=tk.LEFT, padx=5)
        self.status_light = self.status_canvas.create_oval(5, 5, 35, 35, fill="green")
        self.status_text = self.status_canvas.create_text(20, 20, text="行", fill="white", 
                                                       font=("Arial", 12, "bold"))
        
        tk.Label(status_frame, text="正常", font=("Arial", 12), fg="green", bg="#f0f0f0").pack(side=tk.LEFT, padx=5)
        
        # 状态说明
        status_note = tk.Label(status_frame, 
                             text="(ENTER: 急停 | ENTER+SPACE: 恢复)", 
                             font=("Arial", 10), 
                             fg="#666666", 
                             bg="#f0f0f0")
        status_note.pack(side=tk.RIGHT, padx=10)
        
        # 更新状态灯
        self.update_status_light()
        
        # 主内容区域
        content_frame = tk.Frame(main_frame, bg="#f0f0f0")
        content_frame.pack(fill=tk.BOTH, expand=True)
        
        # 左侧面板 - 按键状态
        left_frame = tk.LabelFrame(content_frame, text="控制面板", font=("Arial", 12, "bold"), 
                                 bg="#f0f0f0", padx=10, pady=10)
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 10))
        
        # 按键状态区域
        key_frame = tk.Frame(left_frame, bg="#f0f0f0")
        key_frame.pack(pady=10, padx=10, fill=tk.BOTH, expand=True)
        
        self.key_labels = {}
        keys = [('W', 0, "前进"), ('A', 1, "左转"), ('S', 2, "后退"), ('D', 3, "右转")]
        
        for key, bit, desc in keys:
            frame = tk.Frame(key_frame, bg="#f0f0f0")
            frame.pack(pady=8, fill=tk.X)
            
            # 按键标签
            key_label = tk.Label(frame, text=key, font=("Arial", 16, "bold"), 
                               width=3, bg="#e0e0e0", relief=tk.RAISED, padx=5)
            key_label.pack(side=tk.LEFT, padx=(0, 10))
            
            # 状态标签
            status_label = tk.Label(frame, text="未按下", fg="red", 
                                  font=("Arial", 12), bg="#f0f0f0", width=8, anchor="w")
            status_label.pack(side=tk.LEFT)
            
            # 功能描述
            desc_label = tk.Label(frame, text=desc, font=("Arial", 10), 
                                fg="#555555", bg="#f0f0f0")
            desc_label.pack(side=tk.RIGHT)
            
            self.key_labels[bit] = status_label
        
        # 分隔线
        separator = ttk.Separator(content_frame, orient=tk.VERTICAL)
        separator.pack(side=tk.LEFT, fill=tk.Y, padx=5)
        
        # 右侧面板 - 数据通信
        right_frame = tk.LabelFrame(content_frame, text="数据通信", font=("Arial", 12, "bold"), 
                                  bg="#f0f0f0", padx=10, pady=10)
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=(10, 0))
        
        # 解析数据显示区域
        data_frame = tk.Frame(right_frame, bg="#f0f0f0")
        data_frame.pack(fill=tk.X, pady=(0, 10))
        
        # 创建数据显示标签
        data_labels = [
            ("按键状态:", "key_state"),
            ("浮点数 1:", "float1"),
            ("浮点数 2:", "float2"),
            ("浮点数 3:", "float3"),
            ("浮点数 4:", "float4"),
            ("标志位:", "flag")
        ]
        
        self.data_values = {}
        
        for label_text, value_name in data_labels:
            frame = tk.Frame(data_frame, bg="#f0f0f0")
            frame.pack(fill=tk.X, pady=3)
            
            label = tk.Label(frame, text=label_text, font=("Arial", 11), 
                           width=10, anchor=tk.W, bg="#f0f0f0")
            label.pack(side=tk.LEFT, padx=(0, 5))
            
            value = tk.Label(frame, text="0.0000", font=("Arial", 11), 
                           width=12, anchor=tk.W, bg="white", relief=tk.SUNKEN, padx=5)
            value.pack(side=tk.LEFT)
            
            self.data_values[value_name] = value
        
        # 原始数据区域
        raw_frame = tk.LabelFrame(right_frame, text="原始数据", font=("Arial", 10), 
                                bg="#f0f0f0", padx=5, pady=5)
        raw_frame.pack(fill=tk.BOTH, expand=True)
        
        self.raw_text = tk.Text(raw_frame, height=8, state=tk.DISABLED, 
                              font=("Consolas", 9), wrap=tk.WORD)
        scrollbar = tk.Scrollbar(raw_frame, command=self.raw_text.yview)
        self.raw_text.config(yscrollcommand=scrollbar.set)
        
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.raw_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        
        # 启动线程
        self.running = True
        Thread(target=self.monitor_keys, daemon=True).start()
        Thread(target=self.read_serial, daemon=True).start()
        Thread(target=self.toggle_flag, daemon=True).start()
        
        # 关闭窗口时清理资源
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
    
    def update_status_light(self):
        """更新急停状态指示灯"""
        if self.emergency_stop:
            self.status_canvas.itemconfig(self.status_light, fill="red")
            self.status_canvas.itemconfig(self.status_text, text="停")
        else:
            self.status_canvas.itemconfig(self.status_light, fill="green")
            self.status_canvas.itemconfig(self.status_text, text="行")
    
    def update_key_state(self, key_state):
        """更新按键状态显示"""
        for bit, label in self.key_labels.items():
            if key_state & (1 << bit):
                label.config(text="按下", fg="green")
            else:
                label.config(text="未按下", fg="red")
    
    def update_data_display(self, key_state, f1, f2, f3, f4, flag):
        """更新数据显示"""
        self.data_values["key_state"].config(text=f"{key_state:08b}")
        self.data_values["float1"].config(text=f"{f1:.4f}")
        self.data_values["float2"].config(text=f"{f2:.4f}")
        self.data_values["float3"].config(text=f"{f3:.4f}")
        self.data_values["float4"].config(text=f"{f4:.4f}")
        self.data_values["flag"].config(text=f"{flag}")
    
    def add_raw_data(self, data):
        """添加原始数据到显示区域"""
        self.raw_text.config(state=tk.NORMAL)
        self.raw_text.insert(tk.END, data + "\n")
        self.raw_text.see(tk.END)
        self.raw_text.config(state=tk.DISABLED)
    
    def toggle_flag(self):
        """每1秒切换一次flag1状态"""
        while self.running:
            time.sleep(1.0)
            with self.flag_lock:
                self.flag1 = 1 - self.flag1
    
    def monitor_keys(self):
        """监控键盘按键状态（优化急停响应）"""
        last_send_time = time.time()
        
        while self.running:
            # 检测急停按键组合
            enter_pressed = keyboard.is_pressed('enter')
            space_pressed = keyboard.is_pressed('space')
            
            # 处理急停逻辑
            if enter_pressed and not space_pressed and not self.emergency_stop:
                # 按下ENTER进入急停状态
                self.emergency_stop = True
                self.sending_enabled = False  # 立即停止发送
                self.update_status_light()
                print("进入急停状态!")
            elif enter_pressed and space_pressed and self.emergency_stop:
                # 同时按下ENTER+SPACE退出急停状态
                self.emergency_stop = False
                self.sending_enabled = True  # 恢复发送
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
            
            # 每5ms发送一次当前状态 (200Hz)，只在发送启用时发送
            current_time = time.time()
            if current_time - last_send_time >= 0.005 and self.sending_enabled:
                if self.ser and self.ser.is_open:
                    try:
                        # 打包结构体数据: 1字节按键状态 + 4个float + 1字节标志位
                        data_packet = struct.pack('<B4fB', 
                                                self.key_state,
                                                *self.userdata1,
                                                self.flag1)
                        self.ser.write(data_packet)
                    except serial.SerialException:
                        pass
                last_send_time = current_time
            
            time.sleep(0.001)  # 1ms延迟提高响应速度
    
    def read_serial(self):
        """从串口读取数据并解析（优化实时性）"""
        PACKET_SIZE = 18  # 1(byte) + 4*4(float) + 1(byte) = 18 bytes
        buffer = bytearray()
        while self.running:
            if self.ser and self.ser.is_open:
                try:
                    # 读取所有可用数据
                    available = self.ser.in_waiting
                    if available > 0:
                        data = self.ser.read(available)
                        buffer.extend(data)
                        
                        # 处理缓冲区中所有完整的数据包
                        while len(buffer) >= PACKET_SIZE:
                            # 提取一个完整数据包
                            packet = bytes(buffer[:PACKET_SIZE])
                            del buffer[:PACKET_SIZE]
                            
                            # 解析数据包
                            try:
                                unpacked = struct.unpack('<B4fB', packet)
                                key_state, f1, f2, f3, f4, flag = unpacked
                                
                                # 更新GUI显示
                                self.update_data_display(key_state, f1, f2, f3, f4, flag)
                                
                                # 显示原始数据
                                self.add_raw_data(f"接收: {packet.hex(' ')}")
                            except struct.error as e:
                                self.add_raw_data(f"解析错误: {str(e)}")
                except serial.SerialException:
                    pass
            time.sleep(0.001)  # 更短的等待时间提高响应速度
    
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
