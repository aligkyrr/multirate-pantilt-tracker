"""
pantilt_hardware.py
====================

Gerçek bir pan-tilt/servo cihazının fiziksel ve elektromekanik
kısıtlarını/kusurlarını simüle eden bağımsız modül.

Amaç: `simulator.py` içindeki "ideal" açı-hız-ivme entegrasyonunun
üzerine, gerçek donanımda karşılaşılan şu efektleri eklemek:

  - Hız zarfı (min/max hız) ve açı sınırları (sınırsız veya sert limit)
  - İvme sınırı
  - Haberleşme komut güncelleme hızı sınırı (komut kaçırma / drop)
  - Hız dalgalanması (velocity ripple) -- cihazın kendi iç kontrolcüsünün
    kusuru
  - Açısal çözünürlük (encoder/step quantization)
  - Konum doğruluğu (accuracy) -- sabit/sistematik hata
  - Tekrarlanabilirlik (repeatability) -- rastgele, her ölçümde değişen hata
  - Yerleşme süresi (settling time)

Bu modül `config.py`'a bağımlıdır ama `simulator.py` / `target.py` gibi
diğer modüllere bağımlı DEĞİLDİR -- tek yönlü, temiz bir katman.

Kullanım (özet):

    from pantilt_hardware import PanTiltDeviceSimulator

    device = PanTiltDeviceSimulator.from_config(config)

    # Kontrol döngüsü CONTROL_LOOP_HZ'de (120 Hz) komut üretirken:
    accepted = device.send_command(az_vel_cmd_deg_s, el_vel_cmd_deg_s, now=t)
    if not accepted:
        log.warning("Komut kaçırıldı (comm rate limit)")

    # Fizik döngüsü PANTILT_UPDATE_HZ'de (60 Hz) her tick'te:
    device.step(dt)

    # UI/telemetri, "gerçek" iç durumu değil cihazın RAPORLADIĞI
    # (gürültülü/quantize) konumu okumalı:
    az_reported, el_reported = device.read_position()
    print(device.is_settled)   # True/False
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass, field


class _RippleNoise:
    """
    Sınırlı genlikli, smooth (aniden sıçramayan) rastgele gürültü.
    target.py'deki hedef ivme gürültüsüyle aynı OU-tipi mantık;
    burada hız dalgalanmasını (velocity ripple) modellemek için kullanılır.
    """

    __slots__ = ("value", "max_amplitude", "damping")

    def __init__(self, max_amplitude: float, damping: float):
        self.max_amplitude = max_amplitude
        self.damping = damping
        self.value = random.uniform(-max_amplitude, max_amplitude) * 0.3

    def step(self, dt: float) -> float:
        if self.max_amplitude <= 0.0:
            self.value = 0.0
            return 0.0
        target = random.uniform(-self.max_amplitude, self.max_amplitude)
        self.value += (target - self.value) * min(1.0, self.damping * dt)
        # Sayısal güvenlik: genliği asla aşmasın
        if self.value > self.max_amplitude:
            self.value = self.max_amplitude
        elif self.value < -self.max_amplitude:
            self.value = -self.max_amplitude
        return self.value


@dataclass
class AxisHardwareProfile:
    name: str

    min_speed_deg_s: float
    max_speed_deg_s: float

    limit_enabled: bool
    limit_min_deg: float
    limit_max_deg: float

    max_accel_deg_s2: float

    velocity_ripple_max_deg_s: float
    ripple_damping: float

    angular_resolution_deg: float

    position_accuracy_deg: float
    position_repeatability_deg: float

    settling_time_sec: float
    settling_band_deg: float

    @classmethod
    def from_config(cls, cfg, axis: str) -> "AxisHardwareProfile":
        """axis: 'AZ' veya 'EL'"""
        p = axis.upper()
        return cls(
            name=p,
            min_speed_deg_s=getattr(cfg, f"{p}_MIN_SPEED_DEG_S"),
            max_speed_deg_s=getattr(cfg, f"{p}_MAX_SPEED_DEG_S"),
            limit_enabled=getattr(cfg, f"{p}_LIMIT_ENABLED", True),
            limit_min_deg=getattr(cfg, f"{p}_LIMIT_MIN_DEG"),
            limit_max_deg=getattr(cfg, f"{p}_LIMIT_MAX_DEG"),
            max_accel_deg_s2=getattr(cfg, f"{p}_MAX_ACCEL_DEG_S2"),
            velocity_ripple_max_deg_s=getattr(cfg, f"{p}_VELOCITY_RIPPLE_MAX_DEG_S"),
            ripple_damping=getattr(cfg, "VELOCITY_RIPPLE_DAMPING", 6.0),
            angular_resolution_deg=getattr(cfg, f"{p}_ANGULAR_RESOLUTION_DEG"),
            position_accuracy_deg=getattr(cfg, f"{p}_POSITION_ACCURACY_DEG"),
            position_repeatability_deg=getattr(cfg, f"{p}_POSITION_REPEATABILITY_DEG"),
            settling_time_sec=getattr(cfg, f"{p}_SETTLING_TIME_SEC"),
            settling_band_deg=getattr(cfg, "SETTLING_BAND_DEG"),
        )


# ======================================================================
# Tek eksen simülatörü
# ======================================================================
class RealisticAxisSimulator:
    """
    Tek bir eksenin (azimuth VEYA elevation) gerçekçi fiziğini yürütür.

    İki ayrı konum kavramı vardır:
      - `true_position_deg`      : simülasyonun içsel, "gerçek" fiziksel konumu
      - `read_position_deg()`    : cihazın DIŞARIYA raporladığı konum
                                    (çözünürlük + accuracy + repeatability
                                    hatası eklenmiş hali). UI/telemetri/
                                    kontrol döngüsü bunu okumalı.
    """

    def __init__(self, profile: AxisHardwareProfile, start_pos_deg: float = 0.0):
        self.profile = profile

        self.true_position_deg = start_pos_deg
        self.true_velocity_deg_s = 0.0

        self._commanded_velocity_deg_s = 0.0

        self._ripple = _RippleNoise(
            max_amplitude=profile.velocity_ripple_max_deg_s,
            damping=profile.ripple_damping,
        )

        # Accuracy: oturum başına SABİT bias (kalibrasyon hatası).
        # Bir kere çekilir, simülasyon boyunca değişmez.
        self._accuracy_bias_deg = random.uniform(
            -profile.position_accuracy_deg, profile.position_accuracy_deg
        )

        # Settling: eksen ne kadar süredir "durgun" (settling band içinde
        # ve komutlanan hız ~0) olduğunu izler.
        self._settled_duration = 0.0
        self.is_settled = True

    # ------------------------------------------------------------
    def set_commanded_velocity(self, v_deg_s: float):
        """
        Kontrol döngüsünden gelen hedef hız komutu. Bu değer doğrudan
        uygulanmaz -- ivme sınırı, min-hız stiction'ı ve dalgalanma
        `step()` içinde uygulanır.
        """
        p = self.profile
        v = max(-p.max_speed_deg_s, min(p.max_speed_deg_s, v_deg_s))

        # Stiction / deadband: min_speed'in altındaki (ama sıfır olmayan)
        # komutlanmış hızlarda motor gerçekte hareket edemez ve durur --
        # gerçek step/servo motorlarda görülen bir davranıştır.
        if 0.0 < abs(v) < p.min_speed_deg_s:
            v = 0.0

        self._commanded_velocity_deg_s = v

    # ------------------------------------------------------------
    def step(self, dt: float):
        p = self.profile

        # 1) İvme sınırlı şekilde gerçek hız, komutlanan hıza yaklaşır.
        dv = self._commanded_velocity_deg_s - self.true_velocity_deg_s
        max_dv = p.max_accel_deg_s2 * dt
        if dv > max_dv:
            dv = max_dv
        elif dv < -max_dv:
            dv = -max_dv
        self.true_velocity_deg_s += dv

        # 2) Hız dalgalanması (ripple) eklenir -- cihazın kendi iç
        # kontrolcüsünün kusuru. Sadece eksen fiilen hareket halindeyken
        # (ya da hareket etmesi komutlanmışken) anlamlıdır; komple
        # dururken de küçük bir "creep" olarak devam eder (gerçekçi).
        ripple = self._ripple.step(dt)
        applied_velocity = self.true_velocity_deg_s + ripple
        applied_velocity = max(-p.max_speed_deg_s, min(p.max_speed_deg_s, applied_velocity))

        # 3) Pozisyonu entegre et.
        self.true_position_deg += applied_velocity * dt

        # 4) Açı sınırı -- sert stop (gerçek cihaz limit switch'e çarpar,
        # sekmez, orada durur).
        if p.limit_enabled:
            if self.true_position_deg < p.limit_min_deg:
                self.true_position_deg = p.limit_min_deg
                self.true_velocity_deg_s = 0.0
            elif self.true_position_deg > p.limit_max_deg:
                self.true_position_deg = p.limit_max_deg
                self.true_velocity_deg_s = 0.0

        # 5) Yerleşme (settling) takibi: komutlanan hız ~0 VE gerçek hız
        # çok küçükse "durgun" sayılır; bu durum settling_time kadar
        # kesintisiz sürerse eksen "settled" (yerleşmiş) kabul edilir.
        is_quiescent = (
            abs(self._commanded_velocity_deg_s) < 1e-9
            and abs(applied_velocity) < max(p.settling_band_deg / max(p.settling_time_sec, 1e-6), 1e-6)
        )
        if is_quiescent:
            self._settled_duration += dt
            self.is_settled = self._settled_duration >= p.settling_time_sec
        else:
            self._settled_duration = 0.0
            self.is_settled = False

    # ------------------------------------------------------------
    def read_position_deg(self) -> float:
        """
        Cihazın DIŞARIYA raporladığı konum: gerçek konum + sabit accuracy
        bias'ı + her okumada yeniden çekilen repeatability gürültüsü,
        son olarak açısal çözünürlüğe (encoder step) yuvarlanır.
        """
        p = self.profile
        repeat_noise = random.uniform(
            -p.position_repeatability_deg, p.position_repeatability_deg
        )
        reported = self.true_position_deg + self._accuracy_bias_deg + repeat_noise

        res = p.angular_resolution_deg
        if res > 0.0:
            reported = round(reported / res) * res
        return reported


# ======================================================================
# İki eksenli cihaz simülatörü (Azimuth + Elevation) + haberleşme katmanı
# ======================================================================
class PanTiltDeviceSimulator:
    """
    Azimuth ve elevation eksenlerini birlikte yöneten, tek bir haberleşme
    hattı (comm link) üzerinden komut alan cihaz simülatörü.

    Gerçek donanımda tipik olarak tek bir seri/CAN paketiyle hem az hem
    el hızı birlikte gönderilir, bu yüzden komut kaçırma (drop) mantığı
    eksen bazlı değil, cihaz (tüm paket) bazlıdır.
    """

    def __init__(self, az_profile: AxisHardwareProfile, el_profile: AxisHardwareProfile,
                 comm_max_rate_hz: float,
                 az_start_deg: float = 0.0, el_start_deg: float = 0.0):
        self.az = RealisticAxisSimulator(az_profile, start_pos_deg=az_start_deg)
        self.el = RealisticAxisSimulator(el_profile, start_pos_deg=el_start_deg)

        self._comm_min_interval = 1.0 / comm_max_rate_hz if comm_max_rate_hz > 0 else 0.0
        self._last_accepted_time = None

        self.commands_sent = 0
        self.commands_accepted = 0
        self.commands_dropped = 0

    @classmethod
    def from_config(cls, cfg, az_start_deg: float = 0.0, el_start_deg: float = 0.0) -> "PanTiltDeviceSimulator":
        az_profile = AxisHardwareProfile.from_config(cfg, "AZ")
        el_profile = AxisHardwareProfile.from_config(cfg, "EL")
        return cls(
            az_profile, el_profile,
            comm_max_rate_hz=cfg.COMM_MAX_COMMAND_RATE_HZ,
            az_start_deg=az_start_deg, el_start_deg=el_start_deg,
        )

    # ------------------------------------------------------------
    def send_command(self, az_vel_cmd_deg_s: float, el_vel_cmd_deg_s: float, now: float) -> bool:
        """
        Yeni bir hız komutu göndermeyi dener.

        Dönüş: True  -> komut kabul edildi ve eksenlere uygulandı
               False -> komut KAÇIRILDI (comm rate limit aşıldı);
                        eksenler önceki komutu uygulamaya devam eder.

        `now`: saniye cinsinden simülasyon zamanı (config.DT_CONTROL gibi
        sabit bir adımla artan bir sayaç kullanılabilir).
        """
        self.commands_sent += 1

        if (self._last_accepted_time is not None
                and (now - self._last_accepted_time) < self._comm_min_interval):
            self.commands_dropped += 1
            return False

        self._last_accepted_time = now
        self.commands_accepted += 1
        self.az.set_commanded_velocity(az_vel_cmd_deg_s)
        self.el.set_commanded_velocity(el_vel_cmd_deg_s)
        return True

    # ------------------------------------------------------------
    def step(self, dt: float):
        """Fizik güncellemesi -- PANTILT_UPDATE_HZ'de her tick çağrılmalı,
        komut gönderilsin ya da gönderilmesin (eksenler son komutlanan
        hızı uygulamaya devam eder)."""
        self.az.step(dt)
        self.el.step(dt)

    # ------------------------------------------------------------
    def read_position(self) -> tuple[float, float]:
        """(azimuth_deg, elevation_deg) -- cihazın raporladığı (gürültülü,
        quantize edilmiş) konum çifti."""
        return self.az.read_position_deg(), self.el.read_position_deg()

    @property
    def is_settled(self) -> bool:
        return self.az.is_settled and self.el.is_settled

    @property
    def drop_rate(self) -> float:
        """0-1 arası, gönderilen komutların ne kadarının kaçırıldığı."""
        if self.commands_sent == 0:
            return 0.0
        return self.commands_dropped / self.commands_sent


HARDWARE_MENU_SCHEMA = [
    # (Görünen isim, config anahtarı, min, max, step, birim)
    ("Azimuth Min Hız", "AZ_MIN_SPEED_DEG_S", 0.0, 5.0, 0.05, "°/s"),
    ("Azimuth Max Hız", "AZ_MAX_SPEED_DEG_S", 1.0, 200.0, 1.0, "°/s"),
    ("Azimuth Sınır Aktif", "AZ_LIMIT_ENABLED", None, None, None, "bool"),
    ("Azimuth Min Açı", "AZ_LIMIT_MIN_DEG", -360.0, 0.0, 1.0, "°"),
    ("Azimuth Max Açı", "AZ_LIMIT_MAX_DEG", 0.0, 360.0, 1.0, "°"),
    ("Azimuth Max İvme", "AZ_MAX_ACCEL_DEG_S2", 10.0, 2000.0, 10.0, "°/s²"),

    ("Elevation Min Hız", "EL_MIN_SPEED_DEG_S", 0.0, 5.0, 0.05, "°/s"),
    ("Elevation Max Hız", "EL_MAX_SPEED_DEG_S", 1.0, 150.0, 1.0, "°/s"),
    ("Elevation Min Açı", "EL_LIMIT_MIN_DEG", -90.0, 0.0, 1.0, "°"),
    ("Elevation Max Açı", "EL_LIMIT_MAX_DEG", 0.0, 90.0, 1.0, "°"),
    ("Elevation Max İvme", "EL_MAX_ACCEL_DEG_S2", 10.0, 2000.0, 10.0, "°/s²"),

    ("Haberleşme Komut Hızı", "COMM_MAX_COMMAND_RATE_HZ", 1.0, 500.0, 1.0, "Hz"),

    ("Azimuth Hız Dalgalanması (max)", "AZ_VELOCITY_RIPPLE_MAX_DEG_S", 0.0, 5.0, 0.05, "°/s"),
    ("Elevation Hız Dalgalanması (max)", "EL_VELOCITY_RIPPLE_MAX_DEG_S", 0.0, 5.0, 0.05, "°/s"),

    ("Azimuth Açı Çözünürlüğü", "AZ_ANGULAR_RESOLUTION_DEG", 0.001, 1.0, 0.001, "°"),
    ("Elevation Açı Çözünürlüğü", "EL_ANGULAR_RESOLUTION_DEG", 0.001, 1.0, 0.001, "°"),

    ("Azimuth Konum Doğruluğu", "AZ_POSITION_ACCURACY_DEG", 0.0, 2.0, 0.01, "°"),
    ("Elevation Konum Doğruluğu", "EL_POSITION_ACCURACY_DEG", 0.0, 2.0, 0.01, "°"),

    ("Azimuth Tekrarlanabilirlik", "AZ_POSITION_REPEATABILITY_DEG", 0.0, 1.0, 0.01, "°"),
    ("Elevation Tekrarlanabilirlik", "EL_POSITION_REPEATABILITY_DEG", 0.0, 1.0, 0.01, "°"),

    ("Azimuth Yerleşme Süresi", "AZ_SETTLING_TIME_SEC", 0.0, 3.0, 0.05, "s"),
    ("Elevation Yerleşme Süresi", "EL_SETTLING_TIME_SEC", 0.0, 3.0, 0.05, "s"),
    ("Yerleşme Bandı", "SETTLING_BAND_DEG", 0.01, 2.0, 0.01, "°"),
]
