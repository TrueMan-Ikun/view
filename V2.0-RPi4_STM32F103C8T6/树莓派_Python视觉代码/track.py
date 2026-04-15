import cv2
import serial
import time
from ultralytics import YOLO

# ==========================================
# 1. 配置参数区
# ==========================================
SERIAL_PORT = '/dev/ttyUSB0'  # 树莓派上USB转TTL模块的串口号，可能是ttyUSB1或ttyACM0
BAUD_RATE   = 115200          # 必须和STM32设置的波特率保持一致
MODEL_PATH  = 'yolov8n.pt'    # YOLO模型路径，会自动下载yolov8n，也可以替换成你重新训练的权重
CAMERA_ID   = 0               # 默认USB摄像头，如果有多个摄像头可以填1或2

# STM32期望的坐标系中心 (也是最大范围边界)
STM32_FOV_WIDTH  = 320
STM32_FOV_HEIGHT = 240

# ==========================================
# 2. 初始化硬件与模型
# ==========================================
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
    print(f"[OK] 串口 {SERIAL_PORT} 打开成功。")
except Exception as e:
    print(f"[Error] 串口打开失败，请检查接线或权限: {e}")
    # 调试模式下即使没串口也继续运行
    ser = None

model = YOLO(MODEL_PATH) 
cap = cv2.VideoCapture(CAMERA_ID)

if not cap.isOpened():
    print("[Error] 无法打开摄像头！")
    exit()

print("[OK] 系统启动成功！随时准备发送追踪坐标...")

# ==========================================
# 3. 主控制循环
# ==========================================
while True:
    start_time = time.time()
    
    # a. 读取一帧画面
    success, frame = cap.read()
    if not success: 
        print("[Warning] 摄像头掉线或读取失败！")
        break

    # b. 使用YOLO推理，设置置信度阈值为0.5
    results = model(frame, conf=0.5, verbose=False)

    found_target = False
    
    # 寻找画面中的目标
    for r in results:
        # 如果检测到了不止一个物体，只取第一个可信度最高的
        if len(r.boxes) > 0:
            box = r.boxes[0]
            # YOLO 的中心点坐标和宽高
            xywh = box.xywh[0] 
            
            raw_x = float(xywh[0])
            raw_y = float(xywh[1])
            w = float(xywh[2])
            h = float(xywh[3])

            # c. 【核心魔法】将摄像头的物理分辨率缩小到STM32认识的320x240星球里！
            scale_x = STM32_FOV_WIDTH / frame.shape[1]
            scale_y = STM32_FOV_HEIGHT / frame.shape[0]
            
            target_x = int(raw_x * scale_x)
            target_y = int(raw_y * scale_y)
            
            # 制造一个千分比的面积参数给 STM32 前进后退用
            area_permille = int((w * h * 1000) / (frame.shape[0] * frame.shape[1])) 

            # d. 组装串口字符串，结尾必须加 \r\n 触发STM32接收中断
            cmd = f"X:{target_x},Y:{target_y},A:{area_permille}\r\n"
            
            if ser is not None:
                ser.write(cmd.encode('ascii'))
            
            print(f"[Track] 发现目标! 发送 => {cmd.strip()}")
            found_target = True
            break

    if not found_target:
        # 没有目标！由于 STM32 端已经非常智能了，
        # 我们这边只要不发帧，那边1秒后自动触发搜索逻辑。
        # 所以啥都不用干。
        print("[Seek] 视野中暂无目标...")

    # FPS 控制 (防止串口发送过快淹没STM32，建议稍微延时，保留出8ms以上的喘息时间)
    # YOLO 推理本身会花费 50-100ms，如果不卡顿的话加一点点 sleep 保护
    time.sleep(0.01)

    # =============== 可选：弹窗显示画面（这会在树莓派上消耗额外性能，实机运行时建议注释掉）===============
    annotated_frame = results[0].plot()
    cv2.imshow("V2.0 YOLO Tracking", annotated_frame)
    if cv2.waitKey(1) & 0xFF == ord("q"): 
        break

# 释放资源
cap.release()
if ser:
    ser.close()
cv2.destroyAllWindows()
