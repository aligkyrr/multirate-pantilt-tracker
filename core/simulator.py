import math

import config
from .pantilt_hardware import PanTiltDeviceSimulator


class PanTiltTracker:
    def __init__(self):
        self.azimuth = 0.0
        self.elevation = 0.0
        self.mode = "COARSE"
        self.locked = False

        self._fine_confirm_ticks = 0
        self._coarse_confirm_ticks = 0
        self._lock_confirm_ticks = 0
        self._unlock_confirm_ticks = 0

        self.accel_limit_enabled = config.ACCEL_LIMIT_ENABLED_DEFAULT
        self.max_accel_az = config.MAX_ACCEL_AZ_DEFAULT_DEG_S2
        self.max_accel_el = config.MAX_ACCEL_EL_DEFAULT_DEG_S2

        self._last_v_az = 0.0
        self._last_v_el = 0.0

        # --- Donanım gerçekçilik katmanı -------------------------------
        # Açık olduğunda, coarse/fine/PID mantığının ürettiği hız komutu
        # doğrudan açıya entegre edilmez; bunun yerine PanTiltDeviceSimulator
        # üzerinden geçirilir (min/max hız, ivme, açı limiti, haberleşme
        # komut kaçırma, hız dalgalanması, çözünürlük, accuracy/repeatability,
        # settling time). self.azimuth/self.elevation bu durumda cihazın
        # RAPORLADIĞI (gürültülü) konumu yansıtır.
        #
        # Kapalıyken davranış, bu katman eklenmeden önceki orijinal
        # (ideal) simülasyonla birebir aynıdır -- geriye dönük uyumluluk
        # korunur.
        self.hardware_realism_enabled = config.HARDWARE_REALISM_ENABLED_DEFAULT
        self.device = PanTiltDeviceSimulator.from_config(config)
        self._sim_time = 0.0

    def reset(self):
        self.azimuth = 0.0
        self.elevation = 0.0
        self.mode = "COARSE"
        self.locked = False
        self._fine_confirm_ticks = 0
        self._coarse_confirm_ticks = 0
        self._lock_confirm_ticks = 0
        self._unlock_confirm_ticks = 0
        self._last_v_az = 0.0
        self._last_v_el = 0.0

        self.device = PanTiltDeviceSimulator.from_config(config)
        self._sim_time = 0.0

    # ------------------------------------------------------------
    def set_hardware_realism_enabled(self, enabled: bool):
        """
        Çalışma zamanında donanım gerçekçilik katmanını aç/kapat.
        Geçiş anında ani bir sıçrama olmasın diye, cihazın iç konumu
        mevcut self.azimuth/self.elevation ile senkronize edilir.
        """
        if enabled and not self.hardware_realism_enabled:
            self.device.az.true_position_deg = self.azimuth
            self.device.el.true_position_deg = self.elevation
            self.device.az.true_velocity_deg_s = self._last_v_az / max(config.DT_PANTILT, 1e-9)
            self.device.el.true_velocity_deg_s = self._last_v_el / max(config.DT_PANTILT, 1e-9)
        self.hardware_realism_enabled = enabled

    def update_hardware_profile(self):
        """
        config.py üzerindeki AZ_*/EL_*/COMM_* değerleri UI'dan çalışma
        zamanında değiştirildiğinde çağrılmalı -- cihaz profilini
        güncel config değerleriyle yeniden oluşturur, mevcut konum/hızı
        korur (parametre değişikliği anlık bir sıçrama yaratmaz).
        """
        old_az_pos, old_az_vel = self.device.az.true_position_deg, self.device.az.true_velocity_deg_s
        old_el_pos, old_el_vel = self.device.el.true_position_deg, self.device.el.true_velocity_deg_s

        self.device = PanTiltDeviceSimulator.from_config(config, az_start_deg=old_az_pos, el_start_deg=old_el_pos)
        self.device.az.true_velocity_deg_s = old_az_vel
        self.device.el.true_velocity_deg_s = old_el_vel

    # ------------------------------------------------------------
    @staticmethod
    def _wrap_angle_deg(angle_deg: float) -> float:
        return (angle_deg + 180.0) % 360.0 - 180.0

    def _resolve_az_error(self, des_az: float) -> float:
        """
        Azimuth hatasını hesaplar.

        `_wrap_angle_deg` her zaman en kısa (<=180 derece) yolu seçer.
        Ancak AZ ekseni kablo dolanmasını önlemek için sert bir açı
        sınırına sahipse (AZ_LIMIT_ENABLED, ör. +-185 derece), en kısa
        yol bu sınırın DIŞINA çıkabilir -- bu durumda eksen limite
        "çarpar" ve hedefe bir daha asla ulaşamaz (hedef sürekli en
        kısa yol üzerinden aynı yöne itmeye devam eder, motor limitte
        kilitli kalır).

        Bunun yerine: hedef açının +-360 derecelik tüm eşdeğerleri
        arasından, (varsa) sert limit içinde kalan VE mevcut konuma en
        yakın olanı seçilir. Gerekirse bu, kısa yol yerine ters
        yönden -- daha uzun ama fiilen ulaşılabilir yoldan -- gitmek
        anlamına gelir.
        """
        candidates = [des_az - 360.0, des_az, des_az + 360.0]

        if config.AZ_LIMIT_ENABLED:
            reachable = [
                c for c in candidates
                if config.AZ_LIMIT_MIN_DEG <= c <= config.AZ_LIMIT_MAX_DEG
            ]
            if reachable:
                candidates = reachable
            # `reachable` boşsa (hiçbir eşdeğer limit içine düşmüyorsa --
            # normal şartlarda 370 derecelik bir sınır aralığında bu
            # olmaz) orijinal adaylarla devam edilir.

        best = min(candidates, key=lambda c: abs(c - self.azimuth))
        return best - self.azimuth

    @staticmethod
    def desired_angles(tx: float, ty: float):
        distance = math.hypot(tx, ty)
        distance = max(distance, 1e-6)
        azimuth = math.degrees(math.atan2(tx, ty))
        elevation = math.degrees(math.atan2(-config.CAMERA_HEIGHT, distance))
        return azimuth, elevation

    # ------------------------------------------------------------
    def step(self, tx: float, ty: float, pid_az: "PIDController", pid_el: "PIDController", dt: float):
        des_az, des_el = self.desired_angles(tx, ty)
        err_az = self._resolve_az_error(des_az)
        err_el = des_el - self.elevation

        mode_threshold = (
            config.COARSE_REENTRY_THRESHOLD_DEG if self.mode == "FINE"
            else config.TRACK_ANGLE_THRESHOLD_DEG
        )
        within_fine_band = abs(err_az) <= mode_threshold and abs(err_el) <= mode_threshold

        if self.mode == "COARSE":
            self._fine_confirm_ticks = self._fine_confirm_ticks + 1 if within_fine_band else 0
            if self._fine_confirm_ticks >= config.MODE_SWITCH_CONFIRM_TICKS:
                self.mode = "FINE"
                self._fine_confirm_ticks = 0
                self._coarse_confirm_ticks = 0
        else:
            self._coarse_confirm_ticks = 0 if within_fine_band else self._coarse_confirm_ticks + 1
            if self._coarse_confirm_ticks >= config.MODE_SWITCH_CONFIRM_TICKS:
                self.mode = "COARSE"
                self._coarse_confirm_ticks = 0
                self._fine_confirm_ticks = 0

        if self.mode == "FINE":
            v_az = pid_az.update(err_az, dt)
            v_el = pid_el.update(err_el, dt)
        else:
            v_az = max(-config.COARSE_MAX_SPEED_DEG_PER_TICK, min(config.COARSE_MAX_SPEED_DEG_PER_TICK, err_az))
            v_el = max(-config.COARSE_MAX_SPEED_DEG_PER_TICK, min(config.COARSE_MAX_SPEED_DEG_PER_TICK, err_el))
            pid_az.sync_error(err_az)
            pid_el.sync_error(err_el)

        if self.hardware_realism_enabled:
            # Donanım katmanı kendi ivme sınırını (AZ/EL_MAX_ACCEL_DEG_S2)
            # zaten uyguluyor -- tracker'ın kendi _apply_accel_limit'i
            # burada ATLANIR, aksi halde ivme iki kez sınırlanmış olurdu.
            v_az_dps = v_az / dt
            v_el_dps = v_el / dt

            self.device.send_command(v_az_dps, v_el_dps, now=self._sim_time)
            self.device.step(dt)
            self._sim_time += dt

            self.azimuth, self.elevation = self.device.read_position()
        else:
            if self.accel_limit_enabled:
                v_az = self._apply_accel_limit(v_az, self._last_v_az, self.max_accel_az, dt, error_deg=err_az)
                v_el = self._apply_accel_limit(v_el, self._last_v_el, self.max_accel_el, dt, error_deg=err_el)

            self._last_v_az = v_az
            self._last_v_el = v_el
            self.azimuth += v_az
            self.azimuth = self._wrap_angle_deg(self.azimuth)
            self.elevation += v_el
            self.elevation = max(config.ELEVATION_DEG_MIN, min(config.ELEVATION_DEG_MAX, self.elevation))

        lock_threshold = (
            config.LOCK_EXIT_THRESHOLD_DEG if self.locked
            else config.LOCK_ANGLE_THRESHOLD_DEG
        )
        within_lock_band = abs(err_az) <= lock_threshold and abs(err_el) <= lock_threshold

        if not self.locked:
            self._lock_confirm_ticks = self._lock_confirm_ticks + 1 if within_lock_band else 0
            if self._lock_confirm_ticks >= config.LOCK_CONFIRM_TICKS:
                self.locked = True
                self._lock_confirm_ticks = 0
                self._unlock_confirm_ticks = 0
        else:
            self._unlock_confirm_ticks = 0 if within_lock_band else self._unlock_confirm_ticks + 1
            if self._unlock_confirm_ticks >= config.LOCK_CONFIRM_TICKS:
                self.locked = False
                self._unlock_confirm_ticks = 0
                self._lock_confirm_ticks = 0

        return self.azimuth, self.elevation, v_az, v_el, des_az, des_el, err_az, err_el

    @staticmethod
    def _apply_accel_limit(v_new: float, v_last: float, max_accel_deg_s2: float, dt: float,
                            error_deg: float = None) -> float:

        if max_accel_deg_s2 is None or max_accel_deg_s2 <= 0 or dt <= 0:
            return v_new

        if error_deg is not None:
            v_brake_max_deg_s = math.sqrt(max(0.0, 2.0 * max_accel_deg_s2 * abs(error_deg)))
            v_brake_max_deg_tick = v_brake_max_deg_s * dt
            v_new = max(-v_brake_max_deg_tick, min(v_brake_max_deg_tick, v_new))

        max_dv = max_accel_deg_s2 * dt * dt
        dv = v_new - v_last
        if dv > max_dv:
            dv = max_dv
        elif dv < -max_dv:
            dv = -max_dv
        return v_last + dv

    def aim_point(self, ref_y: float):
        ax = ref_y * math.tan(math.radians(self.azimuth))
        return ax, ref_y

    def aim_direction_vector(self):
        rad = math.radians(self.azimuth)
        return math.sin(rad), math.cos(rad)