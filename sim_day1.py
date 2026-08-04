import numpy as np
import time
import csv
import matplotlib.pyplot as plt

# --- Конфигурация ---
KP_X = 1.2                  # коэффициент P-регулятора по X
KP_Y = 1.0                  # коэффициент P-регулятора по Y
RAM_ENTER_DIST = 10.0       # дистанция для входа в RAM
RAM_EXIT_DIST = 13.0        # дистанция для выхода из RAM (гистерезис)
CLOSE_IN_ENTER_DIST = 15.0  # дистанция для CLOSE_IN
FAILSAFE_TIMEOUT = 2.0      # сколько секунд без цели → FAILSAFE

# Эмуляция движения цели (синусоида + шум + редкие «пропуски»)
def simulate_target(t):
    x = 0.5 + 0.35 * np.sin(t * 1.5)
    y = 0.4 + 0.25 * np.cos(t * 2.0)
    w = 0.12 + 0.04 * np.random.rand()
    conf = 0.95 - 0.3 * np.random.rand()

    # Иногда «теряем» цель (эмуляция пропусков детекции)
    if np.random.rand() < 0.05:
        return None
    return (x, y, w, conf)

# Эмуляция TOF (дистанция в метрах)
def simulate_tof(t):
    base_dist = 18.0
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

        # Если нет цели — проверяем failsafe
        if bbox is None:
            if t_now - self.last_valid_time > FAILSAFE_TIMEOUT:
                self.state = "FAILSAFE"
                return {"mode": "FAILSAFE", "vx": 0.0, "vy": 0.0}
            # Пока не превышен таймаут — держим hover
            return {"mode": self.state, "vx": 0.0, "vy": 0.0}

        self.last_valid_time = t_now
        x, y, w, conf = bbox

        # Логика сценариев с гистерезисом
        if self.state == "RAM":
            # Выход из RAM при увеличении дистанции
            if dist > RAM_EXIT_DIST:
                self.state = "CLOSE_IN"
        else:
            # Вход в RAM
            if dist < RAM_ENTER_DIST and w > 0.4 and temp > 2900:
                self.state = "RAM"

        if self.state != "RAM":
            if dist < CLOSE_IN_ENTER_DIST:
                self.state = "CLOSE_IN"
            else:
                self.state = "TRACK"

        # Режим RAM
        if self.state == "RAM":
            return {"mode": "RAM", "vx": 5.0, "vy": 0.0}

        # Режим CLOSE_IN
        if self.state == "CLOSE_IN":
            return {"mode": "CLOSE_IN", "vx": 3.5, "vy": -0.2}

        # Режим TRACK с P-регулятором
        err_x = 0.5 - x
        err_y = 0.5 - y
        vx = KP_X * err_x
        vy = KP_Y * err_y
        return {"mode": "TRACK", "vx": vx, "vy": vy}

    def run_cycle(self, t, bbox, dist, temp, action):
        row = [
            t,
            action["mode"],
            bbox[0] if bbox else None,
            bbox[1] if bbox else None,
            bbox[2] if bbox else None,
            bbox[3] if bbox else None,
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

def plot_logs(filename="sim_log.csv"):
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
    import pandas as pd  # нужен для графиков

    sim = AutopilotSim()
    t0 = time.time()
    try:
        while True:
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
        print("\nСимуляция остановлена. Сохраняем логи и строим графики...")
        sim.save_log()
        plot_logs()
