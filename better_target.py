import time
import math
from ultralytics import YOLO
import cv2
from pymavlink import mavutil

# ================= НАСТРОЙКИ =================
TARAN_DISTANCE = 3.5          # дистанция перехода в режим тарана (м)
TARAN_SPEED = 7.0             # максимальная скорость тарана (м/с)
SAFE_SPEED = 3.0              # скорость преследования (м/с)
K_P_HOR = 1.2                 # коэффициент горизонтального регулятора
MAX_HOR_SPEED = 4.0           # лимит горизонтальной скорости (м/с)
MAX_VERT_SPEED = 2.0          # лимит вертикальной скорости (м/с)
CONF_THRESHOLD = 0.5          # минимальный confidence для детекции
TRACK_LOSS_MAX = 5            # макс. кадров без трека перед переходом в hover

# Инициализация
model = YOLO("yolov8n.pt")    # можно заменить на свою обученную модель
cap = cv2.VideoCapture(0)     # MaixCam2 как USB-камера; при необходимости указать индекс/поток
master = mavutil.mavlink_connection("udpin:0.0.0.0:14550")
master.wait_heartbeat()

last_target_time = time.time() 
track_loss_count = 0

def capture_frame():
    ret, frame = cap.read()
    if not ret:
        return None
    return frame

def yolo_detect(frame):
    results = model(frame, verbose=False)
    return results

def find_target(results, target_class=0):
    """
    Возвращает bbox [x1,y1,x2,y2] и confidence лучшей цели.
    target_class: класс цели (например, 0 = дрон).
    """
    best_box = None
    best_conf = 0.0
    for r in results:
        boxes = r.boxes
        for box in boxes:
            cls = int(box.cls[0])
            conf = float(box.conf[0])
            if cls == target_class and conf > CONF_THRESHOLD and conf > best_conf:
                best_conf = conf
                best_box = box.xyxy[0].tolist()
    return best_box, best_conf

def estimate_distance_from_bbox(bbox, img_w, img_h):
    """
    Грубая оценка дистанции по размеру bbox (для fallback, если TOF недоступен).
    Лучше использовать TOF, но это страховка.
    """
    x1, y1, x2, y2 = bbox
    area = (x2 - x1) * (y2 - y1)
    norm_area = area / (img_w * img_h)
    # Калибровать под вашу камеру и фокусное расстояние
    dist = max(1.0, 12.0 - 22.0 * norm_area)
    return dist

def get_tof_distance():
    """
    Заглушка: здесь нужно реальное чтение TOF (I2C/UART).
    Для теста можно вернуть None, тогда будет использоваться fallback по bbox.
    """
    # Пример для VL53L0X (через adafruit_circuitpython_vl53l0x)
    # return tof.range / 1000.0  # в метрах
    return None  # пока заглушка

def compute_intercept_velocity(target_bbox, dist, img_w, img_h):
    """
    Вычисляет целевую скорость (vx, vy, vz) для режима преследования.
    """
    x1, y1, x2, y2 = target_bbox
    center_x = (x1 + x2) / 2
    center_error = (center_x - img_w / 2) / img_w  # нормализованное отклонение [-0.5, 0.5]

    vx = SAFE_SPEED
    vy = -K_P_HOR * center_error
    vy = max(-MAX_HOR_SPEED, min(MAX_HOR_SPEED, vy))  # анти-насыщение
    vz = 0.0

    return vx, vy, vz

def set_velocity_command(vx, vy, vz):
    """
    Отправка MAVLink-команды SET_POSITION_TARGET_LOCAL_NED (только скорости).
    """
    master.mav.set_position_target_local_ned_send(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111,  # mask: только vx,vy,vz
        0, 0, 0, vx, vy, vz,
        0, 0, 0, 0, 0
    )

def hover_or_patrol():
    """
    Режим ожидания/поиска: зависание или медленный патруль.
    """
    set_velocity_command(0.0, 0.0, 0.0)

def pursuit_and_ram():
    global last_target_time, track_loss_count

    while True:
        frame = capture_frame()
        if frame is None:
            time.sleep(0.1)
            continue

        img_h, img_w = frame.shape[:2]
        results = yolo_detect(frame)
        target_bbox, conf = find_target(results)

        # Логика потери цели
        if target_bbox is None:
            track_loss_count += 1
            if track_loss_count > TRACK_LOSS_MAX:
                last_target_time = time.time()
                hover_or_patrol()
                continue
            else:
                # Небольшая инерция: продолжаем лететь ещё несколько кадров
                vx, vy, vz = 0.0, 0.0, 0.0
                set_velocity_command(vx, vy, vz)
                time.sleep(0.1)
                continue

        track_loss_count = 0
        last_target_time = time.time()

        # Оценка дистанции: сначала TOF, потом fallback по bbox
        dist = get_tof_distance()
        if dist is None or math.isnan(dist):
            dist = estimate_distance_from_bbox(target_bbox, img_w, img_h)

        # Фаза преследования
        vx, vy, vz = compute_intercept_velocity(target_bbox, dist, img_w, img_h)

        # Фаза финального рывка (таран)
        if dist < TARAN_DISTANCE:
            vx = TARAN_SPEED
            # Удержание цели по центру с усиленным коэффициентом
            x1, y1, x2, y2 = target_bbox
            center_x = (x1 + x2) / 2
            center_error = (center_x - img_w / 2) / img_w
            vy = -2.0 * K_P_HOR * center_error  # более агрессивный регулятор
            vy = max(-MAX_HOR_SPEED, min(MAX_HOR_SPEED, vy))
            vz = 0.0
            # Здесь не добавляем никаких «безопасных» снижений скорости

        set_velocity_command(vx, vy, vz)
        time.sleep(0.1)  # 10 Гц — хороший компромисс для Pi

if __name__ == "__main__":
    pursuit_and_ram()
