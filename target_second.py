import time
from pymavlink import mavutil

master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
master.wait_heartbeat()

def set_velocity(vx, vy, vz):
    master.mav.set_position_target_local_ned_send(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111,  # только скорости
        0, 0, 0, vx, vy, vz,
        0, 0, 0, 0, 0
    )

def disable_failsafe():
    # Внимание: это опасно. Используйте только в защищённой зоне.
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,
        1, 0, 0, 0, 0, 0, 0  # пример: переход в кастомный режим
    )

# Пример цикла тарана
while True:
    # Здесь должна быть логика детекции и оценки дистанции
    distance = 15.0  # условное значение
    vx, vy = 5.0, 0.0  # летим на цель

    if distance < 10.0:
        disable_failsafe()  # отключаем автопосадку и т.п.
        vx = 7.0  # увеличиваем скорость

    set_velocity(vx, vy, 0)
    time.sleep(0.1)




# Псевдокод для Raspberry Pi
def pursuit_and_ram():
    while True:
        frame = capture_frame()
        results = yolo_detect(frame)
        target = find_target(results)

        if not target:
            hover_or_patrol()
            continue

        dist = tof_distance()  # точная дистанция от TOF
        vx, vy, vz = compute_intercept_velocity(target, dist)

        # Фаза финального рывка (таран)
        if dist < TARAN_DISTANCE:  # например, 3–5 м
            vx = TARAN_SPEED  # максимальная скорость вперёд
            vy = center_error(target) * K_p  # удержание цели по центру
            vz = 0.0
            # Здесь можно дополнительно отключить любые «безопасные» коррекции

        send_velocity_command(vx, vy, vz)
