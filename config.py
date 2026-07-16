import os
KP_X = 0.22
KD_X = 0.15
KP_Y = 0.22
KD_Y = 0.15

TRACK_ANGLE_THRESHOLD_DEG = 2.5
COARSE_MAX_SPEED_DEG_PER_TICK = 8.0
MAX_VEL = 2.0

ELEVATION_DEG_MIN = -75.0
ELEVATION_DEG_MAX = 75.0

CAMERA_HEIGHT = 1.5

TARGET_FORWARD_DIST = 8.0
TARGET_X_RANGE = 5.0
TARGET_PERIOD_SEC = 6.0

WINDOW = 100
TRAIL = 50

MAP_X_MIN = -8.0
MAP_X_MAX = 8.0
MAP_Y_MIN = -1.0
MAP_Y_MAX = 12.0
MAP_BOUNDS = (MAP_X_MIN, MAP_X_MAX, MAP_Y_MIN, MAP_Y_MAX)

ANGLE_PLOT_MIN = -90.0
ANGLE_PLOT_MAX = 90.0
VEL_PLOT_RANGE = COARSE_MAX_SPEED_DEG_PER_TICK * 1.3

TICK_MS = 4

#   - TARGET_UPDATE_HZ  : Hedeflerin (target.py) fizik/hareket güncellemesi
#   - CONTROL_LOOP_HZ   : PID kazançlarının UI'dan okunup kontrolcülere
#                          uygulanması (kontrol döngüsü)
#   - PANTILT_UPDATE_HZ : Pan/tilt açı-hız-ivme fiziğinin (simulator.py)
#                          entegrasyonu ve PID hesaplarının uygulanması

TARGET_UPDATE_HZ = 60.0
CONTROL_LOOP_HZ = 120.0
PANTILT_UPDATE_HZ = 60.0

# dt = 1.0 / Hz  (saniye cinsinden sabit zaman adımı)
DT_TARGET = 1.0 / TARGET_UPDATE_HZ
DT_CONTROL = 1.0 / CONTROL_LOOP_HZ
DT_PANTILT = 1.0 / PANTILT_UPDATE_HZ

# Bir alt-döngünün, gerçek geçen zamanla arasında bir "tick"te telafi
# edeceği azami dt (saniye). Pencere sürükleme, breakpoint, GC duraklaması
# vb. yüzünden gerçek geçen süre çok büyürse (spiral-of-death riski), bu
# üst sınırdan sonrası atlanır / biriktirici şimdiki zamana yakınsanır.
MAX_DT = 0.1

# Bir tick içinde bir alt-döngünün en fazla kaç kez "telafi" adımı
# atabileceği (aşırı uzun donmalardan sonra sonsuz döngüye girmesin diye).
MAX_CATCHUP_STEPS = 10

PID_KP = 0.34
PID_KI = 0.015
PID_KD = 0.060
PID_INTEGRAL_LIMIT = 12.0      # anti-windup
PID_OUTPUT_LIMIT = MAX_VEL     # derece/tick, fine mod hız tavanı

# Lock-on: açı hatası (az VE el) bu eşiğin altına düşerse "LOCKED".
LOCK_ANGLE_THRESHOLD_DEG = 1.2

# ---- Hysteresis (titreme/chatter önleme) ----
# Hata, eşik sınırında küçük dalgalanma yapınca mod/kilit her tick'te
# açılıp kapanmasın diye "çıkış" eşiği "giriş" eşiğinden daha yüksek
# tutulur (Schmitt-trigger mantığı).
COARSE_REENTRY_THRESHOLD_DEG = TRACK_ANGLE_THRESHOLD_DEG * 1.6   # FINE -> COARSE dönüş eşiği
LOCK_EXIT_THRESHOLD_DEG = LOCK_ANGLE_THRESHOLD_DEG * 1.8          # LOCKED -> unlock eşiği

# Eşik-histerezisi tek başına yeterli olmayabilir: hedef sürekli hareket
# ettiği için hata, eşik sınırının hemen etrafında da salınabilir ve yine
# her tick'te (16ms) durum değiştirebilir. Bunu kesin olarak engellemek
# için: bir durum değişikliği (mod veya kilit) yalnızca yeni durum bu
# kadar ARDIŞIK tick boyunca sürerse kalıcı hale gelir ("debounce").
MODE_SWITCH_CONFIRM_TICKS = 6   # ~96ms @ TICK_MS=16
LOCK_CONFIRM_TICKS = 6

# PID türev teriminin alçak geçiren filtre katsayısı (0-1). Hedefin
# smooth-random-walk hareketindeki küçük gürültüler D teriminde
# büyütülüp motor hızında (ve dolayısıyla 3D modelde) titremeye yol
# açmasın diye türev filtrelenir. Küçük değer = daha güçlü filtre.
PID_DERIVATIVE_FILTER_ALPHA = 0.25

# Hedef tahmini (lead / prediction): hedefin vx,vy'sine göre
# LEAD_TIME_SEC kadar ileri projekte edilmiş konumu hedeflenir.
LEAD_TIME_SEC = 0.35

# ==================================================================
# YENİ: MULTI-TARGET SİSTEMİ
# ==================================================================
INITIAL_TARGET_COUNT = 4
MAX_TARGETS = 12
MIN_TARGETS = 1

TARGET_TYPES = ("normal", "aggressive", "slow")
DEFAULT_TARGET_TYPE = "normal"

# (min_speed, max_speed) m/s -- tip başına hız zarfı
TARGET_SPEED_RANGE = {
    "normal": (1.0, 2.2),
    "aggressive": (2.4, 4.2),
    "slow": (0.35, 0.9),
}

# Smooth-random-walk (OU tipi) ivme gürültüsü büyüklüğü (m/s^2 jerk skala)
TARGET_NOISE_SCALE = {
    "normal": 3.0,
    "aggressive": 5.5,
    "slow": 1.4,
}

# İvmenin sönümlenme katsayısı (büyük değer -> daha çabuk yön değiştirme
# eğilimi bastırılır -> daha "yumuşak" hareket)
TARGET_ACCEL_DAMPING = 1.4

# Harita sınırına çarpınca sekme sönümü (1.0 = enerji kaybı yok)
TARGET_BOUNCE_DAMPING = 0.85

TARGET_RADIUS_M = 0.28   # görsel + tıklama yarıçapı (metre)
CLICK_TOLERANCE_PX = 18  # radar üzerinde mouse tıklama toleransı (piksel)

# Auto-mode hedef seçim stratejileri
AUTO_STRATEGY_NEAREST = "nearest"          # pan-tilt'e (0,0) en yakın
AUTO_STRATEGY_CENTER = "center"            # |azimuth|'u en küçük (ortalamaya en yakın)
DEFAULT_AUTO_STRATEGY = AUTO_STRATEGY_NEAREST

# AUTO modda hedef "flip-flop" önleme: iki hedef skoru birbirine çok
# yakınsa (ör. iki hedef eşit mesafede dolaşıyorsa), ölçüm gürültüsü
# yüzünden her tick T1<->T2 arasında geçiş yapılmasın diye:
#   1) yeni aday, mevcut aktif hedeften en az MARGIN kadar daha iyi olmalı,
#   2) art arda iki geçiş arasında en az COOLDOWN saniye geçmiş olmalı.
AUTO_SWITCH_COOLDOWN_SEC = 1.2
AUTO_SWITCH_MARGIN_M = 0.6      # 'nearest' stratejisi için (metre)
AUTO_SWITCH_MARGIN_DEG = 4.0    # 'center' stratejisi için (derece)

# Genel çalışma modu: hedef takibi mi, yoksa kullanıcı tanımlı rota mı?
TRACKING_MODE_TARGET = "target"
TRACKING_MODE_ROUTE = "route"
DEFAULT_TRACKING_MODE = TRACKING_MODE_TARGET

# Rota son waypoint'e ulaşınca ne olacağı (kullanıcı arayüzden seçer)
ROUTE_LOOP_MODE_LOOP = "loop"          # başa dönüp tekrar başlar
ROUTE_LOOP_MODE_STOP = "stop"          # son noktada durur
ROUTE_LOOP_MODE_PINGPONG = "pingpong"  # yönü tersine çevirip geri gider
DEFAULT_ROUTE_LOOP_MODE = ROUTE_LOOP_MODE_LOOP

# Rota üzerinde ilerleme hızı (m/s, harita koordinatlarında)
ROUTE_SPEED_MIN_MPS = 0.2
ROUTE_SPEED_MAX_MPS = 6.0
ROUTE_DEFAULT_SPEED_MPS = 2.0

# Radar üzerinde waypoint çizim ayarları
ROUTE_WAYPOINT_RADIUS_PX = 6
COLOR_ROUTE_LINE = "#c78bff"
COLOR_ROUTE_POINT = "#c78bff"
COLOR_ROUTE_CURRENT = "#ffffff"
COLOR_ROUTE_EDIT_HINT = "#ffd166"


ACCEL_LIMIT_ENABLED_DEFAULT = True

MAX_ACCEL_AZ_DEFAULT_DEG_S2 = 150.0
MAX_ACCEL_EL_DEFAULT_DEG_S2 = 150.0

ACCEL_LIMIT_MIN_DEG_S2 = 10.0
ACCEL_LIMIT_MAX_DEG_S2 = 2000.0

# ==================================================================
# DONANIM GERÇEKÇİLİK PROFİLİ (Hardware Realism Profile)
# ==================================================================
# Bu blok, gerçek bir pan-tilt cihazının fiziksel/elektromekanik
# kısıtlarını ve kusurlarını simüle etmek için kullanılır. Amaç,
# "ideal" bir simülasyon yerine gerçek donanımda karşılaşılacak
# sınırları (hız zarfı, ivme, açı limiti) ve kusurları (hız
# dalgalanması, çözünürlük, doğruluk, tekrarlanabilirlik, yerleşme
# süresi) modellemektir. Bkz. `pantilt_hardware.py`.

HARDWARE_REALISM_ENABLED_DEFAULT = False

# ---- Azimuth hız zarfı ----
# AZ_MIN_SPEED_DEG_S: bu değerin altındaki komutlanmış hızlarda motor
# "stiction" (statik sürtünme) nedeniyle düzgün hareket edemez ve durur.
AZ_MIN_SPEED_DEG_S = 0.3
AZ_MAX_SPEED_DEG_S = 60.0

# ---- Azimuth açı sınırı ----
# AZ_LIMIT_ENABLED=False -> sürekli/sınırsız dönüş (slip-ring vb.)
# AZ_LIMIT_ENABLED=True  -> kablo dolanmasını önlemek için yazılımsal
# sert sınır (ör. çoğu pan-tilt üniteside +-185 derece civarı, 360
# derecelik tam turdan biraz fazla kablo payı bırakır).
AZ_LIMIT_ENABLED = True
AZ_LIMIT_MIN_DEG = -185.0
AZ_LIMIT_MAX_DEG = 185.0

# ---- Elevation hız zarfı ----
EL_MIN_SPEED_DEG_S = 0.3
EL_MAX_SPEED_DEG_S = 45.0

# ---- Elevation açı sınırı ----
# Elevation'da genelde "sınırsız" seçenek yoktur (mekanik olarak
# şasiye çarpar), bu yüzden AZ'nin aksine burada bir enable flag'i yok.
EL_LIMIT_MIN_DEG = -75.0
EL_LIMIT_MAX_DEG = 75.0

# ---- İvme sınırları (mevcut ACCEL_LIMIT_* ile aynı fiziksel anlamda,
# donanım profilinin kendi başına tutarlı olması için burada da
# tekrar tanımlanıyor -- istenirse tek bir kaynağa indirgenebilir) ----
AZ_MAX_ACCEL_DEG_S2 = MAX_ACCEL_AZ_DEFAULT_DEG_S2
EL_MAX_ACCEL_DEG_S2 = MAX_ACCEL_EL_DEFAULT_DEG_S2

# ---- Haberleşme komut güncelleme hızı ----
# Cihazın kabul ettiği azami komut hızı (Hz). Bu hızdan daha sık komut
# gönderilirse (ör. UI/kontrol döngüsü 120 Hz'de çalışırken cihaz
# sadece 50 Hz kabul ediyorsa), aradaki fazla komutlar cihaz tarafından
# KAÇIRILIR (drop edilir) -- son kabul edilen komut yürütülmeye devam
# eder. Gerçek seri/CAN/RS485 haberleşmeli servo sürücülerde tipik bir
# kısıttır.
COMM_MAX_COMMAND_RATE_HZ = 50.0

# ---- Hız dalgalanması (velocity ripple) ----
# Cihaz komutlanan hıza "oturmaya" çalışırken kendi iç kontrolcüsü
# (genelde ucuz bir PID veya bang-bang sürücü) küçük, rastgele bir
# dalgalanma üretir. Bu, TÜM gerçek servo/step motor sürücülerinde
# görülen bir olgudur. Burada bir smooth-random-walk (OU tipi) gürültü
# olarak modellenir ve *_RIPPLE_MAX_DEG_S ile genliği sınırlanır.
AZ_VELOCITY_RIPPLE_MAX_DEG_S = 0.8
EL_VELOCITY_RIPPLE_MAX_DEG_S = 0.6
VELOCITY_RIPPLE_DAMPING = 6.0   # büyük değer -> dalgalanma daha çabuk sönümlenir/yön değiştirir

# ---- Açı çözünürlüğü (encoder/step çözünürlüğü) ----
# Cihazın açıyı raporlayabildiği en küçük adım. Gerçek konum bu
# çözünürlüğe yuvarlanarak "okunur" (quantization).
AZ_ANGULAR_RESOLUTION_DEG = 0.01
EL_ANGULAR_RESOLUTION_DEG = 0.01

# ---- Konum doğruluğu (accuracy) ----
# Cihazın kalibrasyon/mekanik toleransından kaynaklanan SABİT
# (sistematik) hata payı. Her eksende cihaz açılışında rastgele
# ama sabit bir bias olarak çekilir (gerçek cihazlarda kalibrasyon
# hatası oturumlar arası değişebilir ama bir oturum içinde sabittir).
AZ_POSITION_ACCURACY_DEG = 0.05
EL_POSITION_ACCURACY_DEG = 0.05

# ---- Tekrarlanabilirlik (repeatability) ----
# Aynı komutlanmış açıya defalarca gidildiğinde, her seferinde biraz
# farklı bir gerçek konumda durma payı (backlash, dişli boşluğu vb.).
# Accuracy'den farklı olarak bu RASTGELE ve her ölçümde yeniden çekilir.
AZ_POSITION_REPEATABILITY_DEG = 0.02
EL_POSITION_REPEATABILITY_DEG = 0.02

# ---- Yerleşme süresi (settling time) ----
# Eksen, komutlanan hız sıfıra indikten sonra ne kadar sürede
# SETTLING_BAND_DEG içine girip orada kalıyor (kısa süreli
# creep/salınım sonrası tam durma). Gerçek servo sistemlerinde
# görülen bir davranıştır ve lock-on mantığının ne kadar "temkinli"
# davranması gerektiğini etkiler.
AZ_SETTLING_TIME_SEC = 0.25
EL_SETTLING_TIME_SEC = 0.20
SETTLING_BAND_DEG = 0.1

# ==================================================================
# Log paneli
# ==================================================================
MAX_LOG_LINES = 10

COLOR_LOG_INFO = "#8a97a8"       # soft gri-mavi
COLOR_LOG_WARNING = "#c9922f"    # mat amber
COLOR_LOG_ERROR = "#a6453f"      # mat kırmızı (neon değil)

# ==================================================================
# Renkler
# ==================================================================
COLOR_BG = "#0d0f12"
COLOR_PANEL_BG = "#14171c"
COLOR_ACCENT = "#28c76f"
COLOR_TARGET = "#ff4d4f"
COLOR_CURRENT = "#3ea6ff"
COLOR_TRAIL = "#28c76f"
COLOR_GRID = "#2a2f36"
COLOR_TEXT = "#e6e6e6"
COLOR_LOG_TEXT = "#39ff6a"

COLOR_TARGET_NORMAL = "#3ea6ff"
COLOR_TARGET_AGGRESSIVE = "#ff4d4f"
COLOR_TARGET_SLOW = "#ffd166"
COLOR_TARGET_TYPES = {
    "normal": COLOR_TARGET_NORMAL,
    "aggressive": COLOR_TARGET_AGGRESSIVE,
    "slow": COLOR_TARGET_SLOW,
}

COLOR_ACTIVE_RING = "#28c76f"
COLOR_LOCK = "#ff2d2d"
COLOR_AIM_LINE = "#28c76f"
COLOR_ORIGIN = "#e6e6e6"

COLOR_TOGGLE_ON = COLOR_ACCENT
COLOR_TOGGLE_OFF = "#ff9f43"

BASE_DIR = os.path.dirname(os.path.abspath(__file__))

def _p(*paths):
    return os.path.join(BASE_DIR, *paths)


STL_MOTOR_BRACKET_PAN = _p("models", "Motor_Bracket_Pan.stl")
STL_MOTOR_BRACKET_TILT = _p("models", "Motor_Bracket_Tilt.stl")
STL_CAMERA_ASSEMBLY = _p("models", "Camera_Bracket.stl")

STL_FIXED_PATHS = [
    _p("models", "Servo_Base_Bracket.stl"),
    _p("models", "Servo_Motor.stl"),
]

STL_PAN_PATHS = [STL_MOTOR_BRACKET_PAN]
STL_TILT_PATHS = [STL_MOTOR_BRACKET_TILT, STL_CAMERA_ASSEMBLY]

CAMERA_ASSEMBLY_Y_CUTOFF = 33.0

GL_CAMERA_DISTANCE = 6.0
GL_TOTAL_HEIGHT = 2.6

GL_FIXED_COLOR = (0.50, 0.53, 0.57)
GL_PAN_COLOR = (0.62, 0.65, 0.69)
GL_TILT_COLOR = (0.82, 0.84, 0.87)

PAN_PIVOT_RAW = (0.0, 36.05, 0.0)
TILT_PIVOT_RAW = (0.0, 43.25, -4.15)

LASER_ORIGIN_RAW = (0.0, 80.0, 0.3)
GL_LASER_COLOR = (1.0, 0.15, 0.15)
GL_LASER_LENGTH = 3.0
GL_LASER_WIDTH = 3.0
GL_LASER_YAW_OFFSET_DEG = 0.0
GL_LASER_PITCH_OFFSET_DEG = 0.0