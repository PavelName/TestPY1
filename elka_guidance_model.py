# ============================================================
#  УЧЕБНАЯ МОДЕЛЬ КОНТУРА НАВЕДЕНИЯ ДРОНА-ПЕРЕХВАТЧИКА
#  Корреляционный трекер + ИК-доводка + пропорциональное наведение.
#  Педагогическая реконструкция по открытым публикациям о БПЛА
#  перехватчиках типа "Ёлка". Не является бортовым кодом.
# ============================================================
import numpy as np
from scipy import ndimage

# ------------------------------------------------------------
# 0. КОНФИГУРАЦИЯ
# ------------------------------------------------------------
class Config:
    dt = 0.01              # шаг интегрирования, с
    target_speed  = 20.0   # м/с, цель (~72 км/ч)
    jink_period   = 3.0    # период "змейки", с
    jink_accel    = 8.0    # боковое ускорение манёвра, м/с^2
    pursuer_speed = 60.0   # м/с, перехватчик (~216 км/ч)
    N             = 4.0    # навигационная константа ПН (обычно 3-5)
    max_turn_rate = 2.4    # рад/с, потолок угловой скорости (~14g при 60 м/с)
    autopilot_tau = 0.12   # с, инерция контура управления
    frame_hz      = 30     # Гц, частота кадров камер
    eo_noise      = 1.0    # м, шум измерения ЭО-канала
    ir_noise      = 0.3    # м, шум измерения ИК-канала
    handover_range= 150.0  # м, смена канала ЭО -> ИК
    gate_sigma    = 5.0    # ворота фильтра Калмана, сигм
    lock_range    = 1000.0 # м, дальность захвата

# ------------------------------------------------------------
# 1. ФИЛЬТР КАЛМАНА: сглаживание, упреждение, ворота
#    Состояние: [x, y, vx, vy]. Модель: постоянная скорость + шум.
# ------------------------------------------------------------
class KalmanFilter:
    _gate2 = Config.gate_sigma**2

    def __init__(self, x0, dt, sigma_a=6.0, sigma_meas=1.0):
        self.x = np.array(x0, float)
        self.P = np.eye(4) * 100.0
        self.F = np.eye(4); self.F[0, 2] = dt; self.F[1, 3] = dt
        q = sigma_a**2
        self.Q = q * np.array([[dt**4/4, 0, dt**3/2, 0],
                               [0, dt**4/4, 0, dt**3/2],
                               [dt**3/2, 0, dt**2, 0],
                               [0, dt**3/2, 0, dt**2]])
        self.H = np.array([[1., 0, 0, 0],
                           [0, 1., 0, 0]])
        self.R = np.eye(2) * sigma_meas**2
        self.I = np.eye(4)

    def predict(self):
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        return self.x[:2].copy()

    def update(self, z):
        y = z - self.H @ self.x                      # инновация
        S = self.H @ self.P @ self.H.T + self.R
        # ВОРОТА: расстояние Махаланобиса выше порога -> выброс (помеха)
        if (y @ np.linalg.solve(S, y)) > KalmanFilter._gate2:
            return None
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x += K @ y
        self.P = (self.I - K @ self.H) @ self.P
        return self.x[:2].copy()

    def lead(self, tau):
        """Упреждение: где цель будет через tau секунд."""
        return self.x[:2] + self.x[2:4] * tau

# ------------------------------------------------------------
# 2. КОРРЕЛЯЦИОННЫЙ ТРЕКЕР (NCC)
#    Шаблон вырезается из первого кадра, дальше ищется в узком
#    окне вокруг прошлой позиции. Дешевле полного сканирования
#    на порядки. На настоящем борту дополняется сетью-детектором,
#    которая периодически пересоздаёт шаблон.
# ------------------------------------------------------------
class CorrelationTracker:
    def __init__(self, frame, center, half=12, win=20):
        self.tpl = self._grab(frame, center, half)
        self.pos = np.array(center, float)
        self.win, self.half = win, half

    @staticmethod
    def _grab(f, c, h):
        r0, c0 = int(c[1]) - h, int(c[0]) - h
        return f[r0:r0 + 2*h, c0:c0 + 2*h].astype(float)

    def step(self, frame):
        h, w = self.tpl.shape
        cx, cy = int(self.pos[0]), int(self.pos[1])
        best, best_score = None, -2.0
        for dy in range(-self.win, self.win + 1, 2):     # грубый проход
            for dx in range(-self.win, self.win + 1, 2):
                r0, c0 = cy + dy - h, cx + dx - h
                cand = frame[max(0, r0):max(0, r0) + 2*h,
                             max(0, c0):max(0, c0) + 2*w].astype(float)
                if cand.size != self.tpl.size:
                    continue
                t = self.tpl - self.tpl.mean()
                k = cand - cand.mean()
                nt, nk = np.linalg.norm(t), np.linalg.norm(k)
                if nt == 0 or nk == 0:
                    continue
                score = float(t @ k / (nt * nk))         # норм. кросс-корреляция
                if score > best_score:
                    best_score, best = score, (cx + dx, cy + dy)
        self.pos = np.array(best, float)
        return self.pos, best_score

# ------------------------------------------------------------
# 3. ИК-ЦЕНТРОИД (терминальная доводка)
#    Пороговая бинаризация теплового кадра -> связные области ->
#    центроид самой горячей компактной (моторы, батарея цели).
# ------------------------------------------------------------
def ir_centroid(heat_map, rel_threshold=0.55):
    thr = heat_map.max() * rel_threshold
    lbl, n = ndimage.label(heat_map > thr)
    if n == 0:
        return None, 0.0
    sizes = ndimage.sum(heat_map > thr, lbl, range(1, n + 1))
    ys, xs = np.where(lbl == int(np.argmax(sizes)) + 1)
    return np.array([xs.mean(), ys.mean()]), float(sizes.max())

# ------------------------------------------------------------
# 4. ЗАКОН НАВЕДЕНИЯ: пропорциональное сближение (ПН)
#    a_cmd = N * V_сближения * d(lambda)/dt
#    Если линия визирования не поворачивается - цель уйти не может.
# ------------------------------------------------------------
def los_rate(rp, vp, rt, vt):
    r = rt - rp
    vr = vt - vp
    rn = np.linalg.norm(r)
    if rn < 1e-6:
        return 0.0, rn
    return (r[0]*vr[1] - r[1]*vr[0]) / rn**2, rn

def pn_command(lam_dot, Vc, N=Config.N):
    return np.clip(N * Vc * lam_dot, -147.0, 147.0)   # потолок ~15g

# ------------------------------------------------------------
# 5. СИМУЛЯЦИЯ ПЕРЕХВАТА (2D-план)
#    Измерения приходят на частоте кадров камеры (30 Гц) с одним
#    кадром задержки; автопилот - звено первого порядка с tau.
# ------------------------------------------------------------
def simulate(jink_accel=0.0, jink_delay=None, t_end=60.0, seed=2):
    rng = np.random.default_rng(seed)
    cfg = Config
    tp = np.array([900.0, 400.0])              # цель
    tv = np.array([-cfg.target_speed, 0.0])
    pp = np.array([0.0, 0.0])                  # перехватчик
    heading = np.arctan2(tp[1], tp[0])
    V = cfg.pursuer_speed
    kf, z_prev = None, None
    T, R, CH, X, Y = [], [], [], [], []
    min_range, t_min, n_rej = 1e9, 0.0, 0
    turn_act = 0.0
    meas_period = 1.0 / cfg.frame_hz
    next_meas = 0.0

    for step in range(int(t_end / cfg.dt)):
        t = step * cfg.dt
        # --- цель: манёвр "змейкой" ---
        if (jink_delay is None or t >= jink_delay) and jink_accel > 0:
            a_lat = jink_accel * np.sin(2*np.pi*(t - (jink_delay or 0)) / cfg.jink_period)
            tv = tv + np.array([-tv[1], tv[0]]) / cfg.target_speed * a_lat * cfg.dt
            tv = tv / np.linalg.norm(tv) * cfg.target_speed
        tp = tp + tv * cfg.dt

        r_true = np.linalg.norm(tp - pp)
        if r_true > cfg.lock_range:            # до захвата - прямолинейный разгон
            pp += np.array([V*np.cos(heading), V*np.sin(heading)]) * cfg.dt
            X.append(pp[0]); Y.append(pp[1])
            continue

        # --- канал: ЭО до handover_range, дальше ИК-доводка ---
        if r_true <= cfg.handover_range:
            noise, ch = cfg.ir_noise, 1
        else:
            noise, ch = cfg.eo_noise, 0

        # --- измерение на частоте кадров, с задержкой на кадр ---
        if t >= next_meas - 1e-9:
            next_meas += meas_period
            z = tp + rng.normal(0, noise, 2)
            z_used = z_prev if z_prev is not None else z
            z_prev = z
            if kf is None:
                kf = KalmanFilter([z[0], z[1], -cfg.target_speed, 0],
                                  cfg.dt, sigma_meas=noise)
            kf.predict()
            upd = kf.update(z_used)
            if upd is None:
                n_rej += 1
                meas = kf.x[:2].copy()         # coasting по прогнозу
            else:
                meas = upd
        else:
            meas = kf.predict()                # межкадровая экстраполяция

        # --- ПН: командная угловая скорость ---
        lam_dot, r = los_rate(pp,
                              np.array([V*np.cos(heading), V*np.sin(heading)]),
                              meas, tv)
        turn_cmd = np.clip(cfg.N * V * lam_dot / V,
                           -cfg.max_turn_rate, cfg.max_turn_rate)
        turn_act += (turn_cmd - turn_act) * (cfg.dt / cfg.autopilot_tau)
        heading += turn_act * cfg.dt
        pp += np.array([V*np.cos(heading), V*np.sin(heading)]) * cfg.dt

        if r < min_range:
            min_range, t_min = r, t
        T.append(t); R.append(r); CH.append(ch); X.append(pp[0]); Y.append(pp[1])

    return dict(t=np.array(T), r=np.array(R), ch=np.array(CH),
                miss=min_range, t_min=t_min,
                px=np.array(X), py=np.array(Y), n_rej=n_rej)

# ------------------------------------------------------------
# 6. ДЕМО
# ------------------------------------------------------------
if __name__ == "__main__":
    base = simulate()
    tA = base["t_min"]
    print("прямолинейная цель:              промах %.2f м (сближение %.1f с)" % (base["miss"], tA))
    print("змейка 8 м/с^2 с начала:         промах %.2f м" % simulate(jink_accel=8.0)["miss"])
    print("рывок 14 м/с^2 за 4 с до финиша: промах %.2f м"
          % simulate(jink_accel=14.0, jink_delay=tA-4.0)["miss"])
    print("рывок 14 м/с^2 за 1 с до финиша: промах %.2f м"
          % simulate(jink_accel=14.0, jink_delay=tA-1.0)["miss"])
