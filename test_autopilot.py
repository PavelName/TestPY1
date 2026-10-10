import numpy as np
import matplotlib.pyplot as plt
import time
import sys

class AutopilotTestBench:
    def __init__(self):
        # --- НАСТРОЙКИ ---
        self.alpha_dist = 0.3   # Вес новых данных для дистанции (0.1-0.5)
        self.beta_dist = 0.1    # Вес скорости изменения
        
        # Гистерезис (пороги)
        self.ram_enter = 10.0
        self.ram_exit = 14.0
        self.close_in_enter = 15.0
        self.close_in_exit = 16.0
        
        # Ограничения физики
        self.max_accel = 1.5    # м/с^2
        self.max_speed = 6.0     # м/с
        
        # Переменные состояния
        self.dist_est = 15.0
        self.dist_vel_est = 0.0
        self.current_state = "SEARCH"
        self.vx_current = 0.0
        self.vy_current = 0.0
        
        # Для расчета реального dt
        self.last_time = time.time()
        self.max_dt_safety = 0.1 # Если цикл зависнет > 0.1с, сбрасываем фильтр

    def get_dynamic_dt(self):
        """Критически важно для Orange Pi: считаем реальное время"""
        now = time.time()
        dt = now - self.last_time
        self.last_time = now
        
        # Защита от скачков времени (например, при выходе из сна)
        if dt > self.max_dt_safety or dt < 0.001:
            dt = 0.02 # Возвращаемся к номиналу, чтобы не сломать фильтр
        return dt

    def filter_distance(self, raw_dist, dt):
        """Alpha-Beta фильтр"""
        predicted = self.dist_est + self.dist_vel_est * dt
        residual = raw_dist - predicted
        
        self.dist_est = predicted + self.alpha_dist * residual
        self.dist_vel_est = self.dist_vel_est + self.beta_dist * (residual / dt)
        return self.dist_est

    def limit_acceleration(self, target_vx, target_vy, dt):
        """Rate Limiter: плавное изменение скорости"""
        max_delta = self.max_accel * dt
        
        # Ограничение по X
        delta_x = target_vx - self.vx_current
        if abs(delta_x) > max_delta:
            self.vx_current += max_delta if delta_x > 0 else -max_delta
        else:
            self.vx_current = target_vx
            
        # Ограничение по Y
        delta_y = target_vy - self.vy_current
        if abs(delta_y) > max_delta:
            self.vy_current += max_delta if delta_y > 0 else -max_delta
        else:
            self.vy_current = target_vy
            
        # Ограничение максимальной скорости (опционально)
        self.vx_current = np.clip(self.vx_current, -self.max_speed, self.max_speed)
        self.vy_current = np.clip(self.vy_current, -self.max_speed, self.max_speed)
        
        return self.vx_current, self.vy_current

    def decide_state(self, dist):
        """Логика с гистерезисом"""
        state = self.current_state

        if state == "FAILSAFE":
            # Выход из FAILSAFE: цель снова видна, начинаем поиск заново
            state = "SEARCH"
        elif state == "RAM":
            if dist > self.ram_exit:
                state = "CLOSE_IN"
        elif state == "CLOSE_IN":
            if dist < self.ram_enter:
                state = "RAM"
            elif dist > self.close_in_exit:
                state = "TRACK"
        elif state == "TRACK":
            if dist < self.close_in_enter:
                state = "CLOSE_IN"
        elif state == "SEARCH":
            if dist < self.close_in_enter:
                state = "TRACK"

        self.current_state = state
        return state

    def step(self, raw_dist, has_target=True):
        """Один шаг цикла управления"""
        dt = self.get_dynamic_dt()
        
        # 1. Фильтрация
        dist_filtered = self.filter_distance(raw_dist, dt)
        
        # 2. Логика состояний
        if not has_target:
            self.current_state = "FAILSAFE"
        else:
            self.decide_state(dist_filtered)
            
        # 3. Расчет целевых скоростей
        target_vx, target_vy = 0.0, 0.0
        
        if self.current_state == "FAILSAFE":
            target_vx, target_vy = 0.0, 0.0
        elif self.current_state == "RAM":
            target_vx = 5.0
            target_vy = 0.0
        elif self.current_state == "CLOSE_IN":
            target_vx = 3.5
            target_vy = 0.0
        elif self.current_state == "TRACK":
            target_vx = 2.0
            target_vy = -0.5 * np.random.normal(0, 0.2) # Имитация ошибки трекинга
        elif self.current_state == "SEARCH":
            target_vx = 1.0
            target_vy = 0.0
            
        # 4. Ограничение ускорения (самое важное для графиков!)
        final_vx, final_vy = self.limit_acceleration(target_vx, target_vy, dt)
        
        return {
            'time': time.time(),
            'dt': dt,
            'dist_raw': raw_dist,
            'dist_filtered': dist_filtered,
            'state': self.current_state,
            'vx': final_vx,
            'vy': final_vy
        }

def run_simulation():
    ap = AutopilotTestBench()
    
    duration = 60.0
    t_vals = []
    data_log = [] 
    
    print("Запуск симуляции...")
    
    start_time = time.time()
    
    while (time.time() - start_time) < duration:
        t = time.time() - start_time
        t_vals.append(t)
        
        ideal_dist = 12 + 4 * np.sin(t * 0.5)
        noise = np.random.normal(0, 0.8)
        raw_dist = ideal_dist + noise
        
        has_target = True
        if 15.0 <= t <= 16.0:
            has_target = False
            
        res = ap.step(raw_dist, has_target)
        data_log.append(res)

        # ... предыдущий код цикла while ...

        # ... код выше (цикл while) ...

    # 1. Проверка: если данных нет, выходим, чтобы не падать дальше
    if not data_log:
        print("Ошибка: список data_log пуст! Симуляция не записала данные.")
        return

    # 2. ПРАВИЛЬНОЕ создание словаря df
    # Берем ключи у ПЕРВОГО элемента списка (data_log), а не у самого списка
    first_entry_keys = data_log[0].keys()
    df = {k: [d[k] for d in data_log] for k in first_entry_keys}

    # Дальше идет код с plt.subplots... (не меняй его)
    fig, axs = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    # ... остальной код графиков ...


    fig, axs = plt.subplots(3, 1, figsize=(12, 10), sharex=True)

    # 1. Дистанция
        # --- Создание графиков (ОДИН РАЗ!) ---
   

    # 1. Дистанция
    axs[0].plot(df['time'], df['dist_raw'], color='gray', alpha=0.3, label='Raw Sensor Data')
    axs[0].plot(df['time'], df['dist_filtered'], color='blue', linewidth=2, label='Filtered (Alpha-Beta)')
    axs[0].axhline(ap.ram_enter, color='red', linestyle='--', label='RAM Enter Thresh')
    axs[0].axhline(ap.ram_exit, color='red', linestyle=':', label='RAM Exit Thresh')
    axs[0].set_ylabel('Distance (m)')
    axs[0].legend(loc='upper right')
    axs[0].grid(True, alpha=0.3)
    axs[0].set_title('Sensor Data: Noise vs Filtering')

    # 2. Скорости
    axs[1].plot(df['time'], df['vx'], color='blue', linewidth=2, label='Vx (Limited Accel)')
    axs[1].plot(df['time'], df['vy'], color='green', linewidth=2, label='Vy')
    axs[1].set_ylabel('Velocity (m/s)')
    axs[1].legend(loc='upper right')
    axs[1].grid(True, alpha=0.3)
    axs[1].set_title('Velocity Profiles: Rate Limiter in Action')

    # 3. Состояния
    state_map = {"SEARCH": 0, "TRACK": 1, "CLOSE_IN": 2, "RAM": 3, "FAILSAFE": 4}
    states_num = [state_map[s] for s in df['state']]

    axs[2].step(df['time'], states_num, where='post', color='cyan', linewidth=2)
    axs[2].set_yticks([0, 1, 2, 3, 4])
    axs[2].set_yticklabels(['SEARCH', 'TRACK', 'CLOSE_IN', 'RAM', 'FAILSAFE'])
    axs[2].set_xlabel('Time (s)')
    axs[2].set_ylabel('State')
    axs[2].grid(True, alpha=0.3, axis='y')
    axs[2].set_title('State Machine with Hysteresis')

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    run_simulation()
