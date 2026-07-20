#!/usr/bin/env python3
"""
safe_target_lock_px4_sitl.py

Учебный пример для PX4 SITL + Gazebo:
- находит цветной маркер в видеопотоке OpenCV;
- поворачивает дрон в PX4 Offboard так, чтобы маркер был ближе к центру кадра;
- если маркер потерян — отправляет нулевую скорость, то есть дрон зависает.

Важно:
- только для симулятора / лабораторного стенда;
- нет распознавания людей;
- нет сближения с объектом;
- нет оружейной логики.
"""

import argparse
import asyncio
from dataclasses import dataclass
from typing import Optional, Tuple, Union

import cv2
import numpy as np

try:
    from mavsdk import System
    from mavsdk.offboard import OffboardError, VelocityBodyYawspeed
except ImportError:
    System = None
    OffboardError = Exception
    VelocityBodyYawspeed = None


@dataclass
class Target:
    x: int
    y: int
    area: float
    bbox: Tuple[int, int, int, int]
    err_x: float
    area_ratio: float


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def parse_hsv_triplet(text: str) -> Tuple[int, int, int]:
    try:
        values = tuple(int(part.strip()) for part in text.split(","))
    except ValueError as exc:
        raise argparse.ArgumentTypeError(
            "HSV нужно указать как H,S,V, например 20,80,80"
        ) from exc

    if len(values) != 3:
        raise argparse.ArgumentTypeError("HSV нужно указать как три числа: H,S,V")

    h, s, v = values
    if not (0 <= h <= 179 and 0 <= s <= 255 and 0 <= v <= 255):
        raise argparse.ArgumentTypeError(
            "Диапазоны OpenCV HSV: H=0..179, S=0..255, V=0..255"
        )

    return values


def parse_camera_source(source: str) -> Union[int, str]:
    """
    Если source = '0', '1' и т.п. — открывается камера.
    Если source = путь к файлу или URL — открывается видео/поток.
    """
    try:
        return int(source)
    except ValueError:
        return source


def detect_colored_marker(
    frame_bgr: np.ndarray,
    hsv_lower: Tuple[int, int, int],
    hsv_upper: Tuple[int, int, int],
    min_area_px: float,
) -> Tuple[Optional[Target], np.ndarray]:
    """
    Ищет крупнейший объект заданного цвета.

    По умолчанию код настроен на жёлтый маркер:
    hsv_lower = 20,80,80
    hsv_upper = 35,255,255
    """
    height, width = frame_bgr.shape[:2]

    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(
        hsv,
        np.array(hsv_lower, dtype=np.uint8),
        np.array(hsv_upper, dtype=np.uint8),
    )

    kernel = np.ones((5, 5), np.uint8)

    # Убираем мелкий шум.
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)

    # Закрываем небольшие дырки внутри объекта.
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )

    if not contours:
        return None, mask

    contour = max(contours, key=cv2.contourArea)
    area = float(cv2.contourArea(contour))

    if area < min_area_px:
        return None, mask

    moments = cv2.moments(contour)

    if abs(moments["m00"]) < 1e-6:
        return None, mask

    x = int(moments["m10"] / moments["m00"])
    y = int(moments["m01"] / moments["m00"])

    bbox = cv2.boundingRect(contour)

    # Нормированная ошибка по горизонтали:
    # -1 = объект у левого края, 0 = центр, +1 = объект у правого края.
    err_x = (x - width / 2.0) / (width / 2.0)

    area_ratio = area / float(width * height)

    return Target(
        x=x,
        y=y,
        area=area,
        bbox=bbox,
        err_x=err_x,
        area_ratio=area_ratio,
    ), mask


def compute_yaw_rate_deg_s(
    target: Optional[Target],
    k_yaw: float,
    deadband: float,
    max_yaw_rate: float,
    search_yaw_rate: float,
) -> Tuple[float, bool]:
    """
    Простой P-регулятор по горизонтальной ошибке.

    target.err_x < 0  => маркер слева  => поворачиваем влево
    target.err_x > 0  => маркер справа => поворачиваем вправо
    target.err_x ~= 0 => маркер в центре
    """
    if target is None:
        # Если цель потеряна: по умолчанию не ищем, а зависаем.
        # Для медленного поиска можно задать --search-yaw-rate 5
        return search_yaw_rate, False

    if abs(target.err_x) < deadband:
        return 0.0, True

    yaw_rate = clamp(
        k_yaw * target.err_x,
        -max_yaw_rate,
        max_yaw_rate,
    )

    locked = abs(target.err_x) < 0.12

    return yaw_rate, locked


def draw_overlay(
    frame_bgr: np.ndarray,
    target: Optional[Target],
    yaw_rate: float,
    locked: bool,
) -> np.ndarray:
    height, width = frame_bgr.shape[:2]
    cx, cy = width // 2, height // 2

    # Прицел в центре кадра.
    cv2.line(frame_bgr, (cx - 25, cy), (cx + 25, cy), (255, 255, 255), 1)
    cv2.line(frame_bgr, (cx, cy - 25), (cx, cy + 25), (255, 255, 255), 1)

    status = "LOCKED" if locked else "SEARCH/HOVER"

    cv2.putText(
        frame_bgr,
        f"{status} | yaw_rate={yaw_rate:+.1f} deg/s",
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 255, 255),
        2,
    )

    if target is not None:
        x, y, w, h = target.bbox

        cv2.rectangle(
            frame_bgr,
            (x, y),
            (x + w, y + h),
            (0, 255, 255),
            2,
        )

        cv2.circle(
            frame_bgr,
            (target.x, target.y),
            5,
            (0, 255, 255),
            -1,
        )

        cv2.line(
            frame_bgr,
            (cx, cy),
            (target.x, target.y),
            (0, 255, 255),
            2,
        )

        cv2.putText(
            frame_bgr,
            f"err_x={target.err_x:+.2f} area={target.area:.0f}",
            (10, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 255),
            2,
        )

    return frame_bgr


async def connect_px4(connection: str) -> "System":
    if System is None:
        raise RuntimeError(
            "Пакет mavsdk не установлен. Установите: python3 -m pip install --user mavsdk"
        )

    drone = System()

    await drone.connect(system_address=connection)

    print(f"Ожидаю PX4 по адресу {connection} ...")

    async for state in drone.core.connection_state():
        if state.is_connected:
            print("PX4 подключён.")
            return drone

    raise RuntimeError("Не удалось подключиться к PX4")


async def start_offboard(
    drone: "System",
    takeoff: bool,
    takeoff_wait_s: float,
) -> None:
    if takeoff:
        print("Arm...")
        await drone.action.arm()

        print("Takeoff...")
        await drone.action.takeoff()

        await asyncio.sleep(takeoff_wait_s)

    # Важно: перед включением Offboard нужно отправить начальный setpoint.
    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(
            0.0,  # forward_m_s
            0.0,  # right_m_s
            0.0,  # down_m_s
            0.0,  # yawspeed_deg_s
        )
    )

    print("Start Offboard...")

    try:
        await drone.offboard.start()
    except OffboardError as exc:
        print(f"Offboard не запустился: {exc}")

        if takeoff:
            await drone.action.land()

        raise


async def stop_offboard_safely(
    drone: Optional["System"],
    land_on_exit: bool,
) -> None:
    if drone is None:
        return

    try:
        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(
                0.0,
                0.0,
                0.0,
                0.0,
            )
        )

        await asyncio.sleep(0.2)

        await drone.offboard.stop()

    except Exception as exc:
        print(f"Предупреждение: не удалось корректно остановить Offboard: {exc}")

    if land_on_exit:
        try:
            print("Landing...")
            await drone.action.land()
        except Exception as exc:
            print(f"Предупреждение: не удалось отправить land: {exc}")


async def main() -> None:
    parser = argparse.ArgumentParser(
        description="Учебный автозахват цветного маркера для PX4 SITL + Gazebo."
    )

    parser.add_argument(
        "--camera",
        default="0",
        help="Индекс камеры, путь к видео или RTSP/UDP URL. По умолчанию: 0",
    )

    parser.add_argument(
        "--connection",
        default="udp://:14540",
        help="MAVSDK endpoint. Для PX4 SITL обычно udp://:14540",
    )

    parser.add_argument(
        "--enable-control",
        action="store_true",
        help="Разрешить отправку команд в PX4. Без этого только визуальная проверка.",
    )

    parser.add_argument(
        "--takeoff",
        action="store_true",
        help="Перед включением Offboard выполнить arm + takeoff. Только для SITL.",
    )

    parser.add_argument(
        "--land-on-exit",
        action="store_true",
        help="При выходе отправить land.",
    )

    parser.add_argument(
        "--no-window",
        action="store_true",
        help="Не показывать OpenCV-окно.",
    )

    parser.add_argument(
        "--show-mask",
        action="store_true",
        help="Показывать бинарную маску.",
    )

    parser.add_argument(
        "--hsv-lower",
        type=parse_hsv_triplet,
        default=(20, 80, 80),
        help="Нижний HSV, например 20,80,80",
    )

    parser.add_argument(
        "--hsv-upper",
        type=parse_hsv_triplet,
        default=(35, 255, 255),
        help="Верхний HSV, например 35,255,255",
    )

    parser.add_argument(
        "--min-area",
        type=float,
        default=400.0,
        help="Минимальная площадь объекта в пикселях",
    )

    parser.add_argument(
        "--k-yaw",
        type=float,
        default=45.0,
        help="Коэффициент P-регулятора yaw, deg/s на нормированную ошибку",
    )

    parser.add_argument(
        "--deadband",
        type=float,
        default=0.05,
        help="Мёртвая зона по горизонтали, 0..1",
    )

    parser.add_argument(
        "--max-yaw-rate",
        type=float,
        default=25.0,
        help="Максимальная скорость рыскания, deg/s",
    )

    parser.add_argument(
        "--search-yaw-rate",
        type=float,
        default=0.0,
        help="Скорость медленного поиска, если цель потеряна. 0 = зависнуть",
    )

    parser.add_argument(
        "--dt",
        type=float,
        default=0.05,
        help="Пауза цикла управления, секунды",
    )

    parser.add_argument(
        "--takeoff-wait",
        type=float,
        default=5.0,
        help="Сколько ждать после takeoff перед Offboard",
    )

    args = parser.parse_args()

    cap = cv2.VideoCapture(parse_camera_source(args.camera))

    if not cap.isOpened():
        raise RuntimeError(f"Не удалось открыть видеопоток: {args.camera}")

    drone = None

    if args.enable_control:
        drone = await connect_px4(args.connection)

        await start_offboard(
            drone,
            takeoff=args.takeoff,
            takeoff_wait_s=args.takeoff_wait,
        )

    print("Работает. Нажмите 'q' в окне OpenCV или Ctrl+C в терминале для выхода.")

    try:
        while True:
            ok, frame = cap.read()

            if not ok:
                print("Кадр не получен. Завершение.")
                break

            target, mask = detect_colored_marker(
                frame_bgr=frame,
                hsv_lower=args.hsv_lower,
                hsv_upper=args.hsv_upper,
                min_area_px=args.min_area,
            )

            yaw_rate, locked = compute_yaw_rate_deg_s(
                target=target,
                k_yaw=args.k_yaw,
                deadband=args.deadband,
                max_yaw_rate=args.max_yaw_rate,
                search_yaw_rate=args.search_yaw_rate,
            )

            if args.enable_control and drone is not None:
                # Только разворот вокруг вертикальной оси.
                # Нет полёта вперёд и нет сближения с объектом.
                await drone.offboard.set_velocity_body(
                    VelocityBodyYawspeed(
                        0.0,       # forward_m_s
                        0.0,       # right_m_s
                        0.0,       # down_m_s
                        yaw_rate,  # yawspeed_deg_s
                    )
                )

            if not args.no_window:
                vis = draw_overlay(
                    frame_bgr=frame,
                    target=target,
                    yaw_rate=yaw_rate,
                    locked=locked,
                )

                cv2.imshow("PX4 SITL safe target lock", vis)

                if args.show_mask:
                    cv2.imshow("mask", mask)

                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

            await asyncio.sleep(args.dt)

    except KeyboardInterrupt:
        print("Остановлено пользователем.")

    finally:
        cap.release()
        cv2.destroyAllWindows()

        await stop_offboard_safely(
            drone=drone,
            land_on_exit=args.land_on_exit,
        )


if __name__ == "__main__":
    asyncio.run(main())