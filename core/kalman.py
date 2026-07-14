# ------------------------------------------------------------------
# Constant-Velocity (CV) Kalman Filtresi
# ------------------------------------------------------------------
class CVKalmanFilter2D:
    """
    Sabit hız (Constant Velocity) modelli 2D Kalman filtresi.

    State vektoru:  x = [px, py, vx, vy]^T
    Ölçüm vektörü:  z = [px, py]^T   (pozisyon ölçümü)

    Process model:
        px_k = px_{k-1} + vx_{k-1} * dt
        py_k = py_{k-1} + vy_{k-1} * dt
        vx_k = vx_{k-1}
        vy_k = vy_{k-1}

    Not: numpy bağımlılığı yok, 4x4/4x2 matris işlemleri elle (saf Python
    listeleriyle) yapılıyor. Performans kritikse numpy'a geçirilebilir.
    """

    __slots__ = ("x", "P", "q_pos", "q_vel", "r_pos")

    def __init__(self, px: float, py: float, vx: float = 0.0, vy: float = 0.0,
                 p0_pos: float = 5.0, p0_vel: float = 10.0,
                 q_pos: float = 0.05, q_vel: float = 0.5,
                 r_pos: float = 1.0):
        # State: [px, py, vx, vy]
        self.x = [px, py, vx, vy]

        # Covariance (4x4), başlangıçta diagonal / belirsizlik yüksek
        self.P = [
            [p0_pos, 0.0,    0.0,    0.0],
            [0.0,    p0_pos, 0.0,    0.0],
            [0.0,    0.0,    p0_vel, 0.0],
            [0.0,    0.0,    0.0,    p0_vel],
        ]

        # Process noise (model belirsizliği) ve measurement noise (sensör gürültüsü)
        self.q_pos = q_pos   # pozisyon süreç gürültüsü
        self.q_vel = q_vel   # hız süreç gürültüsü (manevra/ivme belirsizliğini temsil eder)
        self.r_pos = r_pos   # ölçüm gürültüsü (sensör hassasiyeti)

    # ------------------------------------------------------------
    def predict(self, dt: float):
        px, py, vx, vy = self.x

        # F matrisi (state transition) uygulanmış hali:
        self.x = [px + vx * dt, py + vy * dt, vx, vy]

        P = self.P
        # F = [[1,0,dt,0],[0,1,0,dt],[0,0,1,0],[0,0,0,1]]
        # P' = F P F^T + Q   (elle açılmış hali)
        p00, p01, p02, p03 = P[0]
        p10, p11, p12, p13 = P[1]
        p20, p21, p22, p23 = P[2]
        p30, p31, p32, p33 = P[3]

        # F P
        fp00 = p00 + dt * p20
        fp01 = p01 + dt * p21
        fp02 = p02 + dt * p22
        fp03 = p03 + dt * p23

        fp10 = p10 + dt * p30
        fp11 = p11 + dt * p31
        fp12 = p12 + dt * p32
        fp13 = p13 + dt * p33

        fp20, fp21, fp22, fp23 = p20, p21, p22, p23
        fp30, fp31, fp32, fp33 = p30, p31, p32, p33

        # (F P) F^T
        n00 = fp00 + dt * fp02
        n01 = fp01 + dt * fp03
        n02 = fp02
        n03 = fp03

        n10 = fp10 + dt * fp12
        n11 = fp11 + dt * fp13
        n12 = fp12
        n13 = fp13

        n20 = fp20 + dt * fp22
        n21 = fp21 + dt * fp23
        n22 = fp22
        n23 = fp23

        n30 = fp30 + dt * fp32
        n31 = fp31 + dt * fp33
        n32 = fp32
        n33 = fp33

        q_pos = self.q_pos * dt
        q_vel = self.q_vel * dt

        self.P = [
            [n00 + q_pos, n01,          n02, n03],
            [n10,          n11 + q_pos, n12, n13],
            [n20,          n21,          n22 + q_vel, n23],
            [n30,          n31,          n32,          n33 + q_vel],
        ]

    # ------------------------------------------------------------
    def update(self, zx: float, zy: float):
        """Yeni pozisyon ölçümü geldiğinde state'i düzelt."""
        P = self.P
        px, py, vx, vy = self.x

        # Innovation: y = z - Hx  (H sadece pozisyonu seçer)
        yx = zx - px
        yy = zy - py

        r = self.r_pos

        # S = H P H^T + R  -> 2x2, H = [[1,0,0,0],[0,1,0,0]]
        s00 = P[0][0] + r
        s01 = P[0][1]
        s10 = P[1][0]
        s11 = P[1][1] + r

        det = s00 * s11 - s01 * s10
        if abs(det) < 1e-12:
            return  # sayısal olarak kararsız, güncelleme atla

        inv00 = s11 / det
        inv01 = -s01 / det
        inv10 = -s10 / det
        inv11 = s00 / det

        # K = P H^T S^{-1}  -> 4x2
        # P H^T ilk iki sütun (H sadece px,py seçtiği için P'nin ilk iki sütunu)
        k = [[0.0, 0.0] for _ in range(4)]
        for i in range(4):
            phT0 = P[i][0]
            phT1 = P[i][1]
            k[i][0] = phT0 * inv00 + phT1 * inv10
            k[i][1] = phT0 * inv01 + phT1 * inv11

        # x = x + K y
        new_x = [
            px + k[0][0] * yx + k[0][1] * yy,
            py + k[1][0] * yx + k[1][1] * yy,
            vx + k[2][0] * yx + k[2][1] * yy,
            vy + k[3][0] * yx + k[3][1] * yy,
        ]
        self.x = new_x

        # P = (I - K H) P
        new_P = [[0.0] * 4 for _ in range(4)]
        for i in range(4):
            for j in range(4):
                khp = k[i][0] * P[0][j] + k[i][1] * P[1][j]
                new_P[i][j] = P[i][j] - khp
        self.P = new_P

    # ------------------------------------------------------------
    def step(self, dt: float, zx: float = None, zy: float = None):
        """Bir predict + (varsa) update adımı."""
        self.predict(dt)
        if zx is not None and zy is not None:
            self.update(zx, zy)
        return self.x

    def predicted_position(self, lead_time: float):
        """Mevcut kestirilmiş state'ten lead_time kadar ileri ekstrapolasyon."""
        px, py, vx, vy = self.x
        return px + vx * lead_time, py + vy * lead_time

    @property
    def position(self):
        return self.x[0], self.x[1]

    @property
    def velocity(self):
        return self.x[2], self.x[3]