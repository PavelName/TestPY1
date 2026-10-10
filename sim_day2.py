import numpy as np
import time
import csv
import matplotlib.pyplot as plt
import pandas as pd

# --- Конфигурация ---
KP_X = 1.2                  # коэффициент P-регулятора по X
KP_Y = 1.0                  # коэффициент P-регулятора по Y
RAM_ENTER_DIST = 10.0       # дистанция для входа в RAM
RAM_EXIT_DIST = 13.0        # дистанция для выхода из RAM (гистерезис)
CLOSE_IN_ENTER_DIST = 15.0  # дистанция для входа в CLOSE_IN (было 16.0 — лучше чуть ниже, чтобы чаще заходить)
CLOSE_IN_EXIT_DIST = 16.0   # дистанция для выхода из CLOSE_IN (гистерезис)
FAILSAFE_TIMEOUT = 2.0      # сколько секунд без цели → FAILSAFE

# Эмуляция движения цели (синусоида + шум + редкие «пропуски»)
def simulate_target(t):
    x = 0.5 + 0.35 * np.sin(t * 1.5)
    y = 0.4 + 0.25 * np.cos(t * 2.0)
    w = 0.45 + 0.15 * np.random.rand()  # диапазон ~0.45–0.60, чтобы w > 0.4 выполнялось часто
    conf = 0.95 - 0.3 * np.random.rand()

    # Иногда «теряем» цель (эмуляция пропусков детекции)
    #if np.random.rand() < 0.05:
    if 15.0 <= t <= 18.0:
        return None
    return (x, y, w, conf)

# Эмуляция TOF (дистанция в метрах)
def simulate_tof(t):
    base_dist = 12.0
    noise = 1.5 * (np.random.rand() - 0.5)
    osc = 4.0 * np.sin(t * 0.8)
    return max(1.0, base_dist + osc + noise)

# Эмуляция тепловизора (условные единицы температуры)
def simulate_thermal():
    return 2800 + 300 * (np.random.rand() - 0.5)

class AutopilotSim:
    def __init__(self):
        self.state = "SEARCH"
        self.last_valid_time = time.time()
        self.log_rows = []

    def decide_action(self, bbox, dist, temp):
        t_now = time.time()

        # 1. FAILSAFE — абсолютный приоритет
        if bbox is None:
            if t_now - self.last_valid_time > FAILSAFE_TIMEOUT:
                self.state = "FAILSAFE"
                return {"mode": "FAILSAFE", "vx": 0.0, "vy": 0.0}
            # Пока таймаут не вышел — держим состояние, но обнуляем скорости
            return {"mode": self.state, "vx": 0.0, "vy": 0.0}

        self.last_valid_time = t_now
        x, y, w, conf = bbox

        # 2. Переключение состояний (гистерезис) — через elif, чтобы не перезаписывать
        if self.state == "RAM":
            if dist > RAM_EXIT_DIST:
                self.state = "CLOSE_IN"
        elif self.state == "CLOSE_IN":
            if dist > CLOSE_IN_EXIT_DIST:
                self.state = "TRACK"
        else:
            # Для состояний SEARCH/TRACK: вход в CLOSE_IN
            if dist < CLOSE_IN_ENTER_DIST:
                self.state = "CLOSE_IN"
            else:
                self.state = "TRACK"

        # 3. Вход в RAM — отдельная проверка с приоритетом над CLOSE_IN
        if self.state != "RAM" and dist < RAM_ENTER_DIST and w > 0.4 and temp > 2900:
            self.state = "RAM"

        # 4. Выдача скоростей по текущему состоянию
        if self.state == "RAM":
            return {"mode": "RAM", "vx": 5.0, "vy": 0.0}
        if self.state == "CLOSE_IN":
            return {"mode": "CLOSE_IN", "vx": 3.5, "vy": -0.2}

        # TRACK — P-регулятор
        err_x = 0.5 - x
        err_y = 0.5 - y
        vx = KP_X * err_x
        vy = KP_Y * err_y
        return {"mode": "TRACK", "vx": vx, "vy": vy}

    

    def run_cycle(self, t, bbox, dist, temp, action):
        if bbox is None:
            row = [
                t,
                action["mode"],
                None, None, None, None,
                dist,
                temp,
                action["vx"],
                action["vy"],
            ]
        else:
            x, y, w, conf = bbox
            row = [
                t,
                action["mode"],
                x, y, w, conf,
                dist,
                temp,
                action["vx"],
                action["vy"],
            ]
        self.log_rows.append(row)

    def save_log(self, filename="sim_log.csv"):
        header = ["time_s","state","bbox_x","bbox_y","bbox_w","bbox_conf","dist_m","temp","vx","vy"]
        with open(filename, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(header)
            writer.writerows(self.log_rows)
        print(f"Логи сохранены в {filename}")

    def plot_logs(self,filename="sim_log.csv"):
        df = pd.read_csv(filename)

        plt.figure(figsize=(12, 8))

        plt.subplot(3, 1, 1)
        plt.plot(df["time_s"], df["dist_m"], label="dist_m")
        plt.axhline(CLOSE_IN_ENTER_DIST, color="orange", linestyle="--", label="CLOSE_IN_THR")
        plt.axhline(RAM_ENTER_DIST, color="red", linestyle="--", label="RAM_THR_ENTER")
        plt.ylabel("Dist (m)")
        plt.legend()
        plt.grid(True)

        plt.subplot(3, 1, 2)
        plt.plot(df["time_s"], df["vx"], label="vx", color="blue")
        plt.plot(df["time_s"], df["vy"], label="vy", color="green")
        plt.ylabel("Velocity (m/s)")
        plt.legend()
        plt.grid(True)

        plt.subplot(3, 1, 3)
        state_map = {"SEARCH":0, "TRACK":1, "CLOSE_IN":2, "RAM":3, "FAILSAFE":4}
        df["state_code"] = df["state"].map(state_map)
        plt.step(df["time_s"], df["state_code"], where="post")
        plt.yticks(list(state_map.values()), list(state_map.keys()))
        plt.ylabel("State")
        plt.xlabel("Time (s)")
        plt.grid(True)

        plt.tight_layout()
        plt.show()

if __name__ == "__main__":
    sim = AutopilotSim()
    t0 = time.time()
    try:
        while time.time() - t0 < 60:
            t = time.time() - t0 
            bbox = simulate_target(t)
            dist = simulate_tof(t)
            temp = simulate_thermal()

            action = sim.decide_action(bbox, dist, temp)
            sim.run_cycle(t, bbox, dist, temp, action)

            # Вывод в терминал (каждые 0.2 с, чтобы не засорять)
            if int(t * 10) % 2 == 0:
                log_line = f"[{t:6.2f}] State: {action['mode']:10} | Bbox: {bbox} | Dist: {dist:5.2f} m | Temp: {temp:6.1f} | Cmd: vx={action['vx']:5.2f}, vy={action['vy']:5.2f}"
                print(log_line)

            time.sleep(0.02)  # ~50 Гц
    except KeyboardInterrupt:
        print("\nСимуляция прервана вручную.")
    finally:
        print("Сохраняем логи и строим графики...")
        sim.save_log()
        sim.plot_logs()
