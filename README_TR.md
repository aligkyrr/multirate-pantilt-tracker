# Çok Frekanslı Pan-Tilt Hedef Takip Sistemi

Python / PyQt5 / OpenGL ile geliştirilmiş, iki eksenli (azimuth/elevation) bir pan-tilt platformunun gerçek zamanlı takip ve kapalı-çevrim kontrol simülasyonu.

Sistem; bağımsız frekanslarda çalışan hedef/kontrol/aktüatör döngüleri, PID kontrol, bağımlılıksız bir constant-velocity Kalman filtresi, çoklu hedef seçimi ve donanım kısıtlarını modelleyen bir aktüatör simülatörü içerir.

**Öne çıkan sonuçlar:**
- Gürültülü konum ölçümü altında (σ = 0.15 m), Kalman tabanlı lead-prediction, finite-difference ekstrapolasyona göre RMSE'yi **%87.8–%97.2** oranında iyileştirdi.
- ±185° sert azimuth limiti altında, hedef platformun arkasına geçtiğinde ortaya çıkan kalıcı lock-on kaybı, limit-aware açısal yol çözümü ile giderildi (2000 tick boyunca hiç kilitlenmeme → 351 tick ≈ 5.85 s'de kilitlenme, 60 Hz'de).
- Çoklu hedef otomatik seçiminde cooldown + margin mekanizması, sentetik testte hedef değişim sayısını 300 tick üzerinde 300'den 2'ye düşürdü (flip-flop önleme).

## Demo

[![Demo Video](https://img.youtube.com/vi/XFFDIRdZATw/0.jpg)](https://youtu.be/XFFDIRdZATw)

## İçindekiler

1. [Sonuçlar](#sonuçlar)
2. [Demo](#demo)
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

Hedef-önden-kestirim (lead prediction) hattı, iki yöntem karşılaştırılarak doğrulandı: konum + hız × `LEAD_TIME_SEC` şeklindeki naif (finite-difference) ekstrapolasyon ve `CVKalmanFilter2D.predicted_position()`. Her iki yöntemin çıktısı, hedefin `LEAD_TIME_SEC = 0.35s` sonraki gerçek konumuna göre RMSE olarak ölçüldü.

**Metodoloji:** Gaussian ölçüm gürültüsü σ = 0.15 m, `DT_TARGET` adımında (60 Hz) 3000 tick, üç hedef profili (`normal`, `aggressive`, `slow`) ayrı ayrı test edildi. Metrik: lead-time ufkunda konum RMSE'si.

| Hedef profili | Finite-difference RMSE | Kalman RMSE | İyileşme |
|---|---:|---:|---:|
| normal | 6.465 m | 0.356 m | **%94.5** |
| aggressive | 6.443 m | 0.787 m | **%87.8** |
| slow | 6.442 m | 0.181 m | **%97.2** |

Gürültülü konumdan tek tick üzerinden (`Δt = 1/60s`) hız türetmek, ölçüm gürültüsünü `1/Δt` kadar büyütür; 15 cm'lik konum hatası bu şekilde çok büyük bir hız hatasına dönüşür. Kalman filtresi, konum ve hızı birlikte kestirerek bu etkiyi büyük ölçüde bastırır.

> **Not:** Bu benchmark, planlanan YOLO tabanlı görüntü dedektörünün sonucu değildir — dedektör henüz entegre edilmemiştir. σ = 0.15 m'lik sentetik Gaussian gürültü, olası bir görüntü-tabanlı ölçümün doğruluğunu *temsil eden bir varsayımdır*. Gerçek dedektör entegre edildiğinde gerçek gürültü karakteristiği ile yeniden ölçülmelidir.

### Gürültüsüz (ground-truth) girdi karşılaştırması

Aynı test, ölçüm gürültüsü olmadan (mevcut simülasyonun varsayılan hali — `Target.step()` filtreyi kendi kusursuz konumuyla besliyor) tekrarlandı:

| Hedef profili | Naif RMSE | Kalman RMSE |
|---|---:|---:|
| normal | 0.175 m | 0.201 m |
| aggressive | 0.445 m | 0.489 m |
| slow | 0.050 m | 0.063 m |

Burada naif ekstrapolasyon hafifçe daha iyi — beklenen bir sonuç. Girdi zaten gürültüsüz olduğunda filtrenin uyguladığı yumuşatma, temizleyecek bir gürültü bulamıyor ve kendisi küçük bir gecikme maliyetine dönüşüyor. Bu karşılaştırma, filtrenin kazancının yalnızca gürültülü rejimde ortaya çıktığını doğruluyor — tam olarak planlanan görüntü-tabanlı (YOLO) girdinin çalışacağı rejim.

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

`core` katmanının `ui`'a hiçbir bağımlılığı yoktur — kontrol/simülasyon mantığı arayüzden bağımsız olarak birim test edilebilir. `core/kalman.py` da benzer şekilde `core/target.py`'dan izole tutulmuştur: filtre genel amaçlıdır ve `config.py` ya da hedef profilleri hakkında bilgisi yoktur; hedef tipine göre süreç gürültüsü kalibrasyonu `target.py`'nin sorumluluğundadır.

---

## Çok Frekanslı Döngü Tasarımı

| Döngü | Frekans | Sorumluluk |
|---|---|---|
| Hedef Güncelleme | 60 Hz | Hedef fiziği (OU-tipi smooth random walk, sınır sekmesi) |
| Kontrol Döngüsü (PID) | 120 Hz | Açısal hata hesaplama, PID çıktısı üretimi |
| Pan-Tilt Güncelleme | 60 Hz | Açı/hız/ivme entegrasyonu, aktüatör fiziği |

Kontrol döngüsü, hedef/aktüatör güncellemesinin iki katı frekansta çalıştırılır; bu, kontrol komutlarının daha düşük örnekleme gecikmesiyle üretilmesini amaçlayan bir tasarım tercihidir.

Zamanlama, sabit `dt` adımlarıyla ilerleyen bir accumulator modeliyle yürütülür (`DT_TARGET`, `DT_CONTROL`, `DT_PANTILT`). Pencere sürükleme veya GC duraklaması nedeniyle gerçek geçen süre anormal sıçrarsa, `MAX_DT = 0.1s` tavanı ve `MAX_CATCHUP_STEPS = 10` sınırı döngünün yetişmeye çalışırken kilitlenmesini (spiral-of-death) engeller. `ui/main_window/loop.py`, bu döngüyü Qt'nin olay döngüsüne (`QTimer`, `TICK_MS = 4`) bağlar — QTimer yalnızca UI/scheduler tetikleyicisi olarak kullanılır, gerçek simülasyon zaman adımları accumulator tarafından `DT_TARGET`, `DT_CONTROL` ve `DT_PANTILT` üzerinden bağımsız olarak yönetilir.

Not: bu bir masaüstü Python/PyQt uygulamasıdır ve donanım seviyesinde hard real-time garantisi vermez; yukarıdaki mekanizmalar, soft real-time bir simülasyon döngüsünün zamanlama disiplinini korumaya yöneliktir.

---

## Takip ve Kontrol

### PID Kontrolcü

```
Kontrol döngüsü : 120 Hz
error            : derece
integral         : derece·s
derivative       : derece/s (alçak geçiren filtreden geçirilmiş, α = 0.25)
output           : açısal hız komutu (angular velocity command)
output limit     : ±2.0 deg/tick = ±240 deg/s @ 120 Hz (PID_OUTPUT_LIMIT)

PID_KP = 0.34   PID_KI = 0.015   PID_KD = 0.060
PID_INTEGRAL_LIMIT = 12.0   (anti-windup)
```

Katsayılar, simülasyon üzerinde farklı hedef profilleri (normal/aggressive/slow) için gözlemlenen tracking error ve salınım davranışına göre deneysel olarak ayarlanmıştır.

- **Anti-windup:** integral terim sınırlanarak sürekli büyük hataların kontrolcüyü doygunluğa itmesi engellenir.
- **Türev filtreleme:** hedefin random-walk hareketinden gelen küçük gürültüler D teriminde büyütülüp titremeye yol açabileceğinden, türev terimi alçak geçiren filtreden geçirilir.
- **Lead prediction:** kontrolcü, hedefin anlık konumuna değil, Kalman hız kestiriminden `LEAD_TIME_SEC = 0.35s` ileri projekte edilmiş konuma nişan alır.

### Coarse / Fine / Lock Durum Makinesi

- **COARSE:** hata büyükken sabit tavan hızda (`8.0°/tick`) yaklaşma
- **FINE:** hata `2.5°`'nin altına düşünce PID devreye girer
- **LOCKED:** hata `1.2°`'nin altına düşünce kilit durumu

Tek eşiğe göre mod değiştirmek, hata eşik civarında salındığında her tick'te mod değişmesine (chatter) yol açar. Bu, iki mekanizma ile önlenmiştir:

1. **Histerezis:** çıkış eşiği giriş eşiğinden yüksek tutulur (`COARSE_REENTRY = 4.0°`, `LOCK_EXIT = 2.16°`)
2. **Debounce:** bir durum değişikliği ancak `MODE_SWITCH_CONFIRM_TICKS = 6` (~96 ms) ardışık tick boyunca sürerse kalıcı hale gelir

Histerezis ve ardışık-tick doğrulaması, eşik çevresindeki küçük hedef hareketlerinin gereksiz mod değişimlerine yol açmasını önlemek için birlikte uygulanmıştır.

### Hedef Durum Kestirimi — CV Kalman Filtresi

`CVKalmanFilter2D` (`core/kalman.py`), `[px, py, vx, vy]` durum vektörlü, saf Python ile yazılmış (NumPy bağımlılığı yok) bir constant-velocity Kalman filtresidir. 4×4 kovaryans yayılımı ve 2×2 innovation-kovaryans tersi elle açık şekilde yazılmıştır. Bağımlılık, 4×4/2×2 boyutlu bu problem için gereksiz görüldüğünden bilinçli olarak eklenmemiştir.

Her `Target` kendi filtresini taşır; süreç gürültüsü (`q_vel`), hedef profiline göre ölçeklenir: `aggressive` profil için daha yüksek süreç gürültüsü kullanılarak filtrenin sabit-hız modeline daha az güvenmesi ve ani manevralara daha hızlı adapte olması sağlanır; `slow` profil için daha düşük süreç gürültüsüyle daha fazla smoothing uygulanır. Bu filtre, planlanan YOLO tabanlı görüntü girdisinin üzerine oturacağı kestirim katmanı olarak tasarlanmıştır (bkz. [Planlanan İyileştirmeler](#planlanan-i̇yileştirmeler)).

### Çoklu Hedef ve Rota

- 1–12 arası ayarlanabilir hedef sayısı, üç hedef profili
- Otomatik hedef seçimi: `nearest` veya `center` stratejisi
- **Flip-flop önleme:** yeni aday mevcut hedefi en az bir marjla (`0.6 m` / `4.0°`) geçmeli ve son geçişten bu yana `AUTO_SWITCH_COOLDOWN_SEC = 1.2s` geçmiş olmalı. Sentetik testte bu, 300 tick üzerinde geçiş sayısını 300'den 2'ye düşürdü.
- Waypoint rotaları: `loop` / `stop` / `pingpong` uç davranışları, 0.2–6.0 m/s ayarlanabilir hız

---

## Aktüatör / Donanım Gerçekçilik Katmanı

`core/pantilt_hardware.py`, ideal açı/hız/ivme entegrasyonunun üzerine gerçek bir servo sisteminde karşılaşılabilecek fiziksel ve haberleşme kısıtlarını modelleyen bağımsız bir katmandır — `simulator.py` ve `target.py`'a bağımlı değildir, çalışma zamanında açılıp kapatılabilir (kapalıyken sistem bu katman öncesindeki davranışla birebir aynıdır).

| Model | Amaç |
|---|---|
| Speed envelope | `MIN_SPEED` altında stiction nedeniyle hareketsizlik, `MAX_SPEED` üst tavan |
| Hard limit | Azimuth ekseninde `±185°`'de sert stop (kablo dolanmasını önlemek için) |
| Acceleration limit | Ani hız değişimlerini sınırlar (donanım katmanı açıkken tracker'ın kendi ivme sınırlayıcısı devre dışı kalır) |
| Command rate | Haberleşme kapasitesi kısıtı: cihaz `COMM_MAX_COMMAND_RATE_HZ = 50 Hz`'den sık komut kabul etmez, aradakiler drop edilir |
| Velocity ripple | Cihazın kendi iç kontrolcüsünden kaynaklanan küçük rastgele dalgalanma (OU-tipi gürültü) |
| Angular resolution | Encoder/step çözünürlüğüne quantization |
| Accuracy / repeatability | Accuracy: oturum başına sabit kalibrasyon bias'ı. Repeatability: tekrarlı konumlama hatasını temsil eden, her okumada yeniden çekilen rastgele bileşen (backlash gibi mekanik etkilerin basitleştirilmiş bir temsili) |
| Settling time | Hız sıfıra indikten sonra `SETTLING_BAND_DEG` içinde `SETTLING_TIME_SEC` kadar kesintisiz kalma koşulu |

İki konum kavramı ayrı tutulur: `true_position_deg` içsel gerçek fiziksel konum, `read_position_deg()` ise cihazın dışarıya raporladığı (çözünürlük + accuracy + repeatability hatası eklenmiş) konumdur — UI/telemetri/kontrol döngüsü daima ikincisini okur.

Parametreler, `HARDWARE_MENU_SCHEMA`'dan otomatik üretilen bir PyQt5 panelinden (`hardware_panel.py`) gruplu şekilde çalışma zamanında ayarlanabilir.

---

## Öne Çıkan Problem: Limit-Aware Tracking

**Problem:** Azimuth ekseninde sert açı limiti (`±185°`) varken, pan-tilt `+170°`'de olduğu sırada hedef `-170°`'ye geçiyor (des_az `atan2` ile her zaman `±180°` aralığında hesaplandığından, bu hedefin gerçekten arkaya geçtiği her durumda ortaya çıkan gerçek bir senaryo). Naif "en kısa yol" (`±180°`) hesabı, eksen limite çarpıp orada kilitlenene kadar hedefe doğru gitmeye çalışır ve bir daha asla kurtulamaz.

**Çözüm:** `PanTiltTracker._resolve_az_error()`, hedef açının `±360°` eşdeğerleri arasından, sert limit içinde kalan ve mevcut konuma en yakın olanı seçer — gerektiğinde bu, kısa yol yerine daha uzun ama fiilen ulaşılabilir yoldan gitmek anlamına gelir.

**Sonuç** — `PanTiltDeviceSimulator` ile uçtan uca, `DT_PANTILT` adımında azami 2000 tick:

| | Düzeltme öncesi (naif ±180°) | Düzeltme sonrası (`_resolve_az_error`) |
|---|---|---|
| Hedefe kilitlenme | ❌ hiçbir zaman (2000 tick) | ✅ 351 tick (~5.85 s) |
| Son azimuth | 184.97° (sert limitte asılı) | -167.41° |
| Son gerçek açısal hata | 5.03° (kalıcı) | 2.59° (lock eşiği bandında) |
| Görülen en küçük hata | 5.01° | 0.20° |

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

UI katmanı sorumluluklara göre ayrıştırılmıştır; çizim, koordinat dönüşümü ve kullanıcı etkileşimi `radar_widget` içinde ayrı dosyalarda tutulur, `control_panel` içinde UI etkileşim mantığı (`interactions.py`) core'a dışa dönük API'den (`api.py`) ayrılmıştır.

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

- Fiziksel servo donanımı üzerinde doğrulanmamıştır; `pantilt_hardware.py` gerçek bir datasheet'e değil, parametrik varsayımlara dayanır.
- Hedef ölçümü şu an ground-truth veya sentetik Gaussian gürültü ile modellenmektedir; kamera/görüntü pipeline'ı henüz entegre edilmemiştir. Kalman filtresi henüz gerçek bir görüntü dedektörünün çıktısıyla doğrulanmamıştır — noisy-measurement benchmark'ı sentetik gürültü kullanır, gerçek YOLO çıktısı değildir.
- Python/PyQt + QTimer mimarisi soft real-time'dır; hard real-time zamanlama garantisi vermez.
- PID katsayıları belirli hedef profilleri üzerinde deneysel olarak ayarlanmıştır; farklı dinamiklere sahip hedefler için yeniden ayar gerekebilir.

---

## Planlanan İyileştirmeler

- Ground-truth hedef konumu yerine, YOLO tabanlı bir görüntü dedektöründen gelen bounding-box/centroid ölçümlerinin Kalman filtresine bağlanması
- Donanım entegrasyonu (servo motor sürücü / Raspberry Pi dağıtımı)
- Ağ tabanlı uzaktan kontrol arayüzü

---

## Geliştirici

**Ali İhsan Gökyer**
Elektrik-Elektronik Mühendislik Öğrencisi
