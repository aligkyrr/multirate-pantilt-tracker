# Çok Frekanslı Pan-Tilt Hedef Takip Sistemi

Python / PyQt5 / OpenGL ile geliştirilmiş, iki eksenli (azimuth/elevation) bir pan-tilt platformu için gerçek zamanlı, kapalı-çevrim takip ve kontrol simülasyonu.

Sistem; bağımsız frekanslarda çalışan hedef/kontrol/aktüatör döngülerini, PID kontrolü, bağımlılıksız bir constant-velocity Kalman filtresini, çoklu hedef seçimini ve donanım kısıtlarını modelleyen bir aktüatör simülatörünü içerir.

**Öne çıkan sonuçlar:**
- Gürültülü konum ölçümü altında (σ = 0.15 m), Kalman tabanlı lead-prediction, finite-difference ekstrapolasyona kıyasla RMSE'yi **%87.8–%97.2** oranında iyileştirdi.
- Sert ±185° azimuth limiti altında, hedef platformun arkasına geçtiğinde ortaya çıkan kalıcı lock-on kaybı, limit-aware açısal yol çözümüyle giderildi (2000 tick boyunca hiç kilitlenmeme → 346 tick'te ≈ 5.8 s'de kilitlenme, 60 Hz'de; azimuth ivme limiti 2000 deg/s² iken).
- Çoklu hedef otomatik seçiminde cooldown + margin mekanizması, sentetik bir testte 300 tick üzerinde hedef değişim sayısını 300'den 2'ye düşürdü (flip-flop önleme).

## Demo

[<img src="thumbnail.png" width="900" alt="Demo Video">](https://youtu.be/XFFDIRdZATw)

*Sistemin çalışır halini görmek için görsele tıklayın (YouTube).*

## İçindekiler

1. [Demo](#demo)
2. [Sonuçlar](#sonuçlar)
3. [Sistem Mimarisi](#sistem-mimarisi)
4. [Çok Frekanslı Döngü Tasarımı](#çok-frekanslı-döngü-tasarımı)
5. [Takip ve Kontrol](#takip-ve-kontrol)
6. [Aktüatör / Donanım Gerçekçilik Katmanı](#aktüatör--donanım-gerçekçilik-katmanı)
7. [Öne Çıkan Problem: Limit-Aware Tracking](#öne-çıkan-problem-limit-aware-tracking)
8. [Proje Yapısı](#proje-yapısı)
9. [Kurulum](#kurulum)
10. [Sınırlamalar](#sınırlamalar)
11. [Planlanan İyileştirmeler](#planlanan-i̇yileştirmeler)
12. [Geliştirici](#geliştirici)

---

## Sonuçlar

### Gürültülü ölçüm benchmark'ı

Lead-prediction hattı iki yöntem karşılaştırılarak doğrulandı: konum + hız × `LEAD_TIME_SEC` şeklindeki naif (finite-difference) ekstrapolasyon ve `CVKalmanFilter2D.predicted_position()`. Her iki çıktı da, hedefin `LEAD_TIME_SEC = 0.35s` sonraki gerçek konumuna göre RMSE olarak ölçüldü.

**Metodoloji:** Gaussian ölçüm gürültüsü σ = 0.15 m, `DT_TARGET` adımında (60 Hz) 3000 tick, üç hedef profili (`normal`, `aggressive`, `slow`) ayrı ayrı test edildi. Metrik: lead-time ufkunda konum RMSE'si.

| Hedef profili | Finite-difference RMSE | Kalman RMSE | İyileşme |
|---|---:|---:|---:|
| normal | 6.465 m | 0.356 m | **%94.5** |
| aggressive | 6.443 m | 0.787 m | **%87.8** |
| slow | 6.442 m | 0.181 m | **%97.2** |

Gürültülü bir konumdan tek tick üzerinden (`Δt = 1/60s`) hız türetmek, ölçüm gürültüsünü `1/Δt` kadar büyütür; 15 cm'lik bir konum hatası bu şekilde çok büyük bir hız hatasına dönüşür. Kalman filtresi, konum ve hızı birlikte kestirerek bu etkiyi büyük ölçüde bastırır.

> **Not:** Bu benchmark, planlanan YOLO tabanlı görüntü dedektörünün çıktısını kullanmıyor — dedektör henüz entegre edilmedi. σ = 0.15 m'lik sentetik Gaussian gürültü, olası bir görüntü-tabanlı ölçümün doğruluğunu temsil etmesi amaçlanan bir *varsayımdır*. Gerçek bir dedektör entegre edildiğinde, bunun gerçek gürültü karakteristiğine göre yeniden ölçülmesi gerekir.

### Gürültüsüz (ground-truth) girdi karşılaştırması

Aynı test, ölçüm gürültüsü olmadan tekrarlandı (simülasyonun mevcut varsayılanı — `Target.step()` filtreyi kendi kusursuz konumuyla besliyor):

| Hedef profili | Naif RMSE | Kalman RMSE |
|---|---:|---:|
| normal | 0.175 m | 0.201 m |
| aggressive | 0.445 m | 0.489 m |
| slow | 0.050 m | 0.063 m |

Burada naif ekstrapolasyon hafifçe daha iyi — beklenen bir sonuç. Girdi zaten gürültüsüz olduğunda filtrenin uyguladığı yumuşatmanın temizleyecek bir gürültüsü kalmıyor ve bu yumuşatma kendisi küçük bir gecikme maliyetine dönüşüyor. Bu karşılaştırma, filtrenin faydasının yalnızca gürültülü rejimde ortaya çıktığını doğruluyor — tam olarak planlanan görüntü-tabanlı (YOLO) girdinin çalışacağı rejim.

---

## Sistem Mimarisi

```
   Target (ground-truth / gürültülü ölçüm)
          │ measurement
          ▼
   CVKalmanFilter2D  ──► predicted_position(lead_time)
          │
          ▼
   TargetManager  ──► auto-select (nearest/center, flip-flop önleme)
          │
          ▼
   PanTiltTracker (state machine: COARSE / FINE / LOCKED)
          │ hysteresis + debounce
          ▼
   PID Controller  ──► açısal hız komutu
          │
          ▼
   Acceleration Limiter
          │
          ▼
   PanTiltDeviceSimulator (hardware realism layer)
          │
          └──► read_position_deg()  ──► UI / telemetri / kontrol döngüsü (feedback)
```

`core` katmanının `ui`'a hiçbir bağımlılığı yoktur — kontrol/simülasyon mantığı arayüzden bağımsız olarak birim test edilebilir. `core/kalman.py` da benzer şekilde `core/target.py`'dan izole tutulmuştur: filtre genel amaçlıdır ve `config.py` ya da hedef profilleri hakkında bilgisi yoktur; süreç gürültüsü kalibrasyonunun hedef tipine göre eşlenmesi `target.py`'nin sorumluluğundadır.

---

## Çok Frekanslı Döngü Tasarımı

| Döngü | Frekans | Sorumluluk |
|---|---|---|
| Hedef güncelleme | 60 Hz | Hedef fiziği (OU-tipi smooth random walk, sınır sekmesi) |
| Kontrol döngüsü | 120 Hz | UI'dan PID kazançlarını okur ve PID kontrolcü parametrelerini günceller |
| Pan-tilt güncelleme | 60 Hz | Açısal hatayı hesaplar, PID kontrolcüleri çalıştırır ve pan-tilt durumunu günceller |


Zamanlama, sabit `dt` adımlarıyla ilerleyen bir accumulator modeliyle yürütülür (`DT_TARGET`, `DT_CONTROL`, `DT_PANTILT`). Pencere sürükleme veya bir GC duraklaması nedeniyle gerçek geçen süre anormal şekilde sıçrarsa, `MAX_DT = 0.1s` tavanı ve `MAX_CATCHUP_STEPS = 10` sınırı, döngünün yetişmeye çalışırken kilitlenmesini (spiral-of-death) önler. `ui/main_window/loop.py` bu döngüyü Qt'nin olay döngüsüne (`QTimer`, `TICK_MS = 4`) bağlar — QTimer yalnızca bir UI/scheduler tetikleyicisi olarak kullanılır; asıl simülasyon zaman adımları, `DT_TARGET`, `DT_CONTROL` ve `DT_PANTILT` kullanan accumulator'lar tarafından bağımsız olarak yönetilir.

Not: bu bir masaüstü Python/PyQt uygulamasıdır ve donanım seviyesinde hard real-time garantisi vermez; yukarıdaki mekanizmalar, soft real-time bir simülasyon döngüsünün zamanlama disiplinini korumaya yöneliktir.

---

## Takip ve Kontrol

### PID Kontrolcü

```
PID çalışma frekansı : 60 Hz (PID, pan-tilt güncelleme adımının içinde çalışır)
Parametre güncelleme  : 120 Hz (PID kazançları UI'dan okunur ve her iki kontrolcüye uygulanır)
error                 : derece
integral              : derece·s
derivative            : saniye başına hata değişimi (alçak geçiren filtreden geçirilmiş, α = 0.25)
output                : açısal hız komutu
output limit          : ±2.0 deg/tick (tracker/aktüatör tarafından bir hız komutu olarak yorumlanır)


PID_KP = 0.34   PID_KI = 0.015   PID_KD = 0.060
PID_INTEGRAL_LIMIT = 12.0   (anti-windup)
```

Katsayılar, farklı hedef profillerinde (normal/aggressive/slow) gözlemlenen tracking error ve salınım davranışına göre simülasyon üzerinde deneysel olarak ayarlanmıştır.

- **Anti-windup:** sürekli büyük hataların kontrolcüyü doygunluğa sürüklememesi için integral terimi kelepçelenir.
- **Türev filtreleme:** hedefin random-walk hareketinden gelen küçük gürültüler D teriminde büyütülüp motor çıkışında titremeye (chatter) yol açabileceğinden, türev terimi alçak geçiren filtreden geçirilir.
- **Lead prediction:** kontrolcü, hedefin anlık konumuna değil, Kalman hız kestirimi kullanılarak `LEAD_TIME_SEC = 0.35s` ileri projekte edilmiş bir konuma nişan alır.

### Coarse / Fine / Lock Durum Makinesi

- **COARSE:** hata büyükken sabit bir tavan hızda (`8.0°/tick`) hızlı yaklaşma
- **FINE:** hata `2.5°`'nin altına düştüğünde PID devreye girer
- **LOCKED:** hata `1.2°`'nin altına düştüğünde kilit durumuna girilir

Tek bir eşiğe göre mod değiştirmek, hata bu eşik civarında salındığında her tick'te mod titremesine (chatter) yol açar. Bu, iki mekanizmayla önlenir:

1. **Histerezis (Schmitt-trigger mantığı):** çıkış eşiği giriş eşiğinden yüksek tutulur (`COARSE_REENTRY = 4.0°`, `LOCK_EXIT = 2.16°`)
2. **Debounce:** bir durum değişikliği (mod veya kilit) ancak yeni durum `MODE_SWITCH_CONFIRM_TICKS = 6` (60 Hz'de ~100 ms) ardışık tick boyunca sürerse kalıcı hale gelir

Histerezis ve ardışık-tick doğrulaması birlikte kullanılarak, eşik çevresindeki küçük hedef hareketlerinin gereksiz mod değişimlerine yol açması önlenir.

### Hedef Durum Kestirimi — CV Kalman Filtresi

`CVKalmanFilter2D` (`core/kalman.py`), `[px, py, vx, vy]` durum vektörüne sahip, saf Python ile yazılmış (NumPy bağımlılığı yok) bir constant-velocity Kalman filtresidir. 4×4 kovaryans yayılımı ve 2×2 innovation-kovaryans tersi elle, açık şekilde yazılmıştır. Bu bağımlılık, bu boyuttaki bir problem için (4×4/2×2) gereksiz görüldüğünden bilinçli olarak eklenmemiştir.

Her `Target` kendi filtresini taşır; süreç gürültüsü (`q_vel`) hedef profiline göre ölçeklenir: `aggressive` profili daha yüksek süreç gürültüsü kullanır, böylece filtre sabit-hız modeline daha az güvenir ve ani manevralara daha hızlı adapte olur; `slow` profili ise daha ağır bir smoothing için daha düşük süreç gürültüsü kullanır. Bu filtre, planlanan YOLO tabanlı görüntü girdisinin üzerine oturacağı kestirim katmanı olarak tasarlanmıştır (bkz. [Planlanan İyileştirmeler](#planlanan-i̇yileştirmeler)).

### Çoklu Hedef ve Rota Sistemi

- 1–12 arası ayarlanabilir hedef sayısı, üç hedef profili
- Otomatik hedef seçimi: `nearest` veya `center` stratejisi
- **Flip-flop önleme:** yeni bir aday, mevcut hedefi en az bir marjla (`0.6 m` / `4.0°`) geçmelidir ve son geçişten bu yana en az `AUTO_SWITCH_COOLDOWN_SEC = 1.2s` geçmiş olmalıdır. Sentetik bir testte bu, 300 tick üzerinde geçiş sayısını 300'den 2'ye düşürdü.
- Waypoint rotaları: `loop` / `stop` / `pingpong` rota-sonu davranışları, 0.2–6.0 m/s ayarlanabilir hız

---

## Aktüatör / Donanım Gerçekçilik Katmanı

`core/pantilt_hardware.py`, bir pan-tilt/servo cihazının fiziksel ve haberleşme kısıtlarını modelleyen bağımsız bir donanım-gerçekçilik katmanıdır. `simulator.py` ve `target.py`'dan bilinçli olarak bağımsızdır ve simülasyon mimarisinin geri kalanı değiştirilmeden açılıp kapatılabilir.

Katman, iki konum temsilini ayrı tutar:

- `true_position_deg`: simülatörün içsel fiziksel konumu.
- `read_position_deg()`: accuracy bias'ı, repeatability gürültüsü ve açısal çözünürlük quantization'ından sonra simüle edilen cihazın raporladığı konum.

Kontrol/telemetri tarafının içsel gerçek konum yerine raporlanan konumu tüketmesi amaçlanmıştır.

| Model | Amaç |
|---|---|
| Speed envelope | Komutlanan hız yapılandırılmış azami hıza kelepçelenir; minimum hız eşiğinin altındaki sıfır olmayan komutlar stiction/deadband olarak değerlendirilir ve komutlanan hız sıfır olur |
| Hard limit | Opsiyonel sert konumsal limitler; azimuth varsayılan olarak `±185°` |
| Acceleration limit | Fiziksel hızın ne kadar hızlı değişebileceğini sınırlar |
| Command rate | Cihaz komutları yalnızca `COMM_MAX_COMMAND_RATE_HZ = 50 Hz`'e kadar kabul eder; fazla komutlar düşürülür (drop edilir) ve önceki komut etkin kalmaya devam eder |
| Velocity ripple | İçsel sürücü/kontrol kusurlarını modellemek için fiziksel hıza eklenen düzgün, sınırlı rastgele dalgalanma |
| Angular resolution | Raporlanan konum, yapılandırılmış encoder/step çözünürlüğüne quantize edilir |
| Accuracy | Simüle edilen cihazın ömrü boyunca sabit kalan rastgele bir kalibrasyon bias'ı uygulanır |
| Repeatability | Her konum okumasında yeni bir rastgele konumlama hatası örneklenir |
| Settling time | Eksen, komutlanan hız sıfır olduğunda ve uygulanan hız, yapılandırılmış settling-band'den türetilen eşik içinde `SETTLING_TIME_SEC` boyunca kesintisiz kaldığında "yerleşmiş" (settled) sayılır |

Haberleşme-hızı modeli, bilinçli olarak fiziksel güncelleme hızından ayrıdır: komutlar 120 Hz'de üretilirken simüle edilen cihaz bunları daha düşük yapılandırılmış bir hızda kabul edebilir. Düşürülen (drop edilen) komutlar aktüatörü durdurmaz; son kabul edilen hız komutu, başka bir komut kabul edilene kadar uygulanmaya devam eder.

Parametreler, `HARDWARE_MENU_SCHEMA`'dan üretilen PyQt5 donanım panelinden (`hardware_panel.py`) çalışma zamanında ayarlanabilir.

---

## Öne Çıkan Problem: Limit-Aware Tracking

**Problem:** Sert bir azimuth açı limiti (`±185°`) etkinken, pan-tilt `+170°`'deyken hedef `-170°`'ye geçer (des_az her zaman `atan2` ile `±180°` aralığında hesaplandığından, bu, hedefin gerçekten platformun arkasına geçtiği her durumda ortaya çıkan gerçek bir senaryodur). Naif "en kısa yol" (`±180°`) hesabı, eksen sert limite çarpıp orada kilitlenene kadar hedefe doğru gitmeye çalışır — ve bir daha asla kurtulamaz.

**Çözüm:** `PanTiltTracker._resolve_az_error()`, hedef açının `±360°` eşdeğerleri arasından sert limit içinde kalan ve mevcut konuma en yakın olanı seçer — gerektiğinde bu, en kısa yol yerine daha uzun ama fiilen ulaşılabilir bir yol izlemek anlamına gelir.

**Sonuç** — test için `PanTiltDeviceSimulator` ve donanım gerçekçilik katmanı etkinken uçtan uca simüle edildi. Azimuth/elevation ivme limitinin 150'den 2000 deg/s²'ye çıkarılması dışında varsayılan donanım parametreleri kullanıldı. Platform +170°'de başlar, hedef -170° azimuth'ta (8 m uzakta) sabittir; `DT_PANTILT` adımında 2000 tick, farklı rastgele seed'lerle 10 çalıştırma:

| | Düzeltme öncesi (naif ±180°) | Düzeltme sonrası (`_resolve_az_error`) |
|---|---|---|
| Hedefe kilitlenme | Hayır (10 çalıştırmanın 0'ı, 2000 tick) | Evet (10 çalıştırmanın 10'u, tick 346'da ≈ 5.8 s, 60 Hz'de) |
| Son azimuth | 185.0° (sert limitte asılı kalır) | ≈ -169.7° |
| Son gerçek açısal hata | 5.0° (kalıcı) | 0.3–0.4° |

Karşılaştırma için: 60 deg/s azimuth hız limitinde 340°'lik bir dönüş yaklaşık 5.7 s sürer, dolayısıyla kilitlenme süresi kinematik alt sınıra yakındır.

> **Not:** Varsayılan ivme limitinde (150 deg/s²), yol çözümü yine doğru 340°'lik rotayı seçer, ancak COARSE mod hedefi aşar ve etrafında yaklaşık ±11° salınır; kararlı kilitlenme, çalıştırmaların yalnızca küçük bir azınlığında (30'da yaklaşık 4) elde edilmiştir. Donanım katmanı etkinken COARSE moddaki komut fren mesafesiyle sınırlandırılmaz. Bkz. [Sınırlamalar](#sınırlamalar).

---

## Proje Yapısı

```
pantilt_tracker/
├── core/
│   ├── kalman.py             # CVKalmanFilter2D (bağımlılıksız)
│   ├── target.py              # Hedef fiziği, TargetManager
│   ├── pantilt_hardware.py    # Donanım gerçekçilik katmanı
│   └── ...                    # Kontrol döngüsü, PID, rota mantığı, durum makinesi
├── ui/
│   ├── control_panel/          # Kontrol paneli (PID, rota, hedef, donanım)
│   ├── main_window/            # Ana pencere, simülasyon döngüsü (QTimer)
│   └── radar_widget/           # 2D radar/harita görselleştirme
├── visualization/               # OpenGL tabanlı 3D render hattı
├── models/                      # STL 3D model varlıkları
├── config.py                    # Tüm sistem/kontrol parametreleri
└── main.py
```

UI katmanı sorumluluğa göre ayrıştırılmıştır; çizim, koordinat dönüşümleri ve kullanıcı etkileşimi `radar_widget` içinde ayrı dosyalarda tutulur, `control_panel` içinde ise UI etkileşim mantığı (`interactions.py`), core'a dışa dönük API'den (`api.py`) ayrılmıştır.

---

## Kurulum

```bash
git clone https://github.com/aligkyrr/multirate-pantilt-tracker.git
cd multirate-pantilt-tracker
pip install -r requirements.txt
python main.py
```

---

## Sınırlamalar

- Fiziksel servo donanımı üzerinde doğrulanmamıştır; `pantilt_hardware.py`, belirli bir ticari aktüatörden alınan ölçümler yerine hız, ivme, haberleşme hızı, ripple, encoder çözünürlüğü, accuracy, repeatability ve settling davranışı için yapılandırılabilir parametrik modeller kullanır.
- Hedef ölçümü şu an ground-truth veya sentetik Gaussian gürültüyle modellenmektedir; bir kamera/görüntü pipeline'ı henüz entegre edilmemiştir. Kalman filtresi henüz gerçek bir görüntü-dedektörü çıktısına karşı doğrulanmamıştır — noisy-measurement benchmark'ı sentetik gürültü kullanır, gerçek YOLO çıktısı değil.
- Varsayılan ivme limitinde (150 deg/s²) ve donanım katmanı etkinken, çok büyük bir başlangıç hatasından sonra (örneğin limit-aware senaryosunda) COARSE mod hedefi aşar ve etrafında salınır; kararlı kilitlenme güvenilir şekilde elde edilemez. Donanım yolundaki hız komutuna bir fren-mesafesi limiti eklenmesi bilinen açık bir maddedir.
- Python/PyQt + QTimer mimarisi soft real-time'dır; hard real-time zamanlama garantisi vermez.
- PID katsayıları belirli hedef profilleri üzerinde deneysel olarak ayarlanmıştır; farklı dinamiklere sahip hedefler için yeniden ayar gerekebilir.

---

## Planlanan İyileştirmeler

- Kalman filtresinin ground-truth hedef konumu yerine, YOLO tabanlı bir görüntü dedektöründen gelen bounding-box/centroid ölçümleriyle beslenmesi
- Donanım entegrasyonu (servo motor sürücü / Raspberry Pi dağıtımı)
- Ağ tabanlı uzaktan kontrol arayüzü

---

## Geliştirici

**Ali İhsan Gökyer**
Elektrik-Elektronik Mühendisliği Öğrencisi