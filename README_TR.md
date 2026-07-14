# Multi-Rate Pan-Tilt Hedef Takip Sistemi

**Gerçek zamanlı, multi-rate kontrol mimarisine sahip bir pan-tilt izleme sisteminin simülasyonu.**
PyQt5 tabanlı arayüz, OpenGL ile 3D görselleştirme ve PID tabanlı kapalı çevrim kontrol.

---

## Özet

Bu proje, iki eksenli (azimuth / elevation) bir pan-tilt platformunun hareketli hedefleri gerçek zamanlı olarak izlemesini simüle eden bir kontrol ve simülasyon yazılımıdır. Amaç görsel bir demo üretmek değil; **gerçek zamanlı sistemlerde karşılaşılan tasarım problemlerini** — farklı frekanslarda çalışan alt sistemlerin senkronizasyonu, kapalı çevrim kontrol kararlılığı, mod/kilit geçişlerinde titreme (chatter) önleme, hedef kaybı ve yeniden yakalama — yazılım seviyesinde gerçekçi biçimde modellemektir.

Sistem, sabit zaman adımlı (fixed-timestep) üç bağımsız döngü üzerine kuruludur ve `config.py` üzerinden tamamen parametrize edilmiştir; hiçbir kontrol veya zamanlama sabiti kod içine gömülü (hardcoded) değildir.

---

## Kontrol Sistemi — Teknik Detaylar

### Multi-Rate Döngü Yapısı

| Döngü | Frekans | Sorumluluk |
|---|---|---|
| Hedef Güncelleme | 60 Hz | Hedef fiziği (OU-tipi smooth random walk, sınır sekmesi) |
| Kontrol Döngüsü (PID) | 120 Hz | Açısal hata hesabı, PID çıktısı üretimi |
| Pan-Tilt Güncelleme | 60 Hz | Açı/hız/ivme entegrasyonu, aktüatör fiziği |

Kontrol döngüsünün diğer döngülerin iki katı frekansta çalışması bilinçli bir tasarım tercihidir: aktüasyon ve hedef fiziği güncellenmeden önce kontrolcünün daha sık örnekleme yapması, PID çıktısını daha kararlı ve gecikmesiz hale getirir.

Zamanlama, sabit `dt` (`DT_TARGET`, `DT_CONTROL`, `DT_PANTILT`) ile ilerleyen bir biriktirici (accumulator) modeliyle yürütülür. Pencere sürükleme, breakpoint veya GC duraklaması gibi nedenlerle gerçek geçen süre anormal büyürse, `MAX_DT = 0.1s` üst sınırı ve `MAX_CATCHUP_STEPS = 10` ile **spiral-of-death** riski (döngünün yetişmeye çalışırken kilitlenmesi) engellenir. Bu, gerçek zamanlı gömülü sistemlerde standart bir korunma mekanizmasıdır ve simülasyonun rastgele bir demo değil, gerçek zamanlı sistem disipliniyle yazıldığını gösterir.

### PID Kontrolcü

```
PID_KP = 0.34   PID_KI = 0.015   PID_KD = 0.060
PID_INTEGRAL_LIMIT = 12.0     (anti-windup)
PID_OUTPUT_LIMIT   = 2.0 derece/tick   (fine-mod hız tavanı)
PID_DERIVATIVE_FILTER_ALPHA = 0.25
```

- **Anti-windup:** integral terim `PID_INTEGRAL_LIMIT` ile sınırlanarak, uzun süreli büyük hatalarda integral doygunluğunun kontrolcüyü kararsızlaştırması engellenir.
- **Türev filtreleme:** Hedefin rastgele-yürüyüş (random-walk) hareketindeki küçük gürültüler, D teriminde büyütülüp motor çıkışında titremeye yol açabilir. Bu nedenle türev terimi alçak geçiren filtre (`α = 0.25`) ile yumuşatılır.
- **Hedef kestirimi (lead):** Kontrolcü hedefin anlık konumunu değil, hedefin `vx, vy`'sine göre `LEAD_TIME_SEC = 0.35s` ileri projekte edilmiş konumunu hedefler — hareketli hedeflerde faz gecikmesini azaltan klasik bir izleme tekniği.

### Coarse/Fine Mod Geçişi ve Kilit (Lock-On) Mantığı

Sistem iki kontrol modu arasında geçiş yapar:

- **COARSE:** açı hatası büyükken sabit tavan hızla (`COARSE_MAX_SPEED_DEG_PER_TICK = 8.0`) hızlı yaklaşma
- **FINE:** açı hatası `TRACK_ANGLE_THRESHOLD_DEG = 2.5°` altına düşünce PID devrede, hassas yaklaşma
- **LOCKED:** açı hatası `LOCK_ANGLE_THRESHOLD_DEG = 1.2°` altına düşünce kilit durumu

Bu geçişlerin naif biçimde tek bir eşik üzerinden yapılması, hata değeri eşik civarında salındığında moddan moda her tick'te geçiş (chatter) yapılmasına yol açar. Bu proje bunu iki katmanlı bir mekanizmayla çözer:

1. **Histerezis (Schmitt-trigger mantığı):** çıkış eşiği giriş eşiğinden yüksek tutulur — `COARSE_REENTRY_THRESHOLD_DEG = 4.0°`, `LOCK_EXIT_THRESHOLD_DEG = 2.16°`
2. **Debounce (ardışık tick onayı):** histerezis tek başına yetmez, çünkü hedef sürekli hareket ettiği için hata eşik etrafında yine salınabilir. Bir durum değişikliği yalnızca yeni durum `MODE_SWITCH_CONFIRM_TICKS = 6` (~96 ms) / `LOCK_CONFIRM_TICKS = 6` ardışık tick boyunca sürerse kalıcı hale gelir.

Bu iki mekanizmanın birlikte kullanılması, gerçek radar/izleme sistemlerinde track kararlılığı sağlamak için kullanılan yaklaşımla örtüşür.

### İvme (Acceleration) Limitleme

PID/coarse çıktısı doğrudan açıya uygulanırsa, hız bir tick'ten diğerine anlık sıçrayabilir — gerçek bir servo motor bunu fiziksel olarak yapamaz. Bu nedenle hızın saniye başına değişimi ayrıca sınırlanır (`MAX_ACCEL_AZ/EL_DEFAULT_DEG_S2 = 150.0°/s²`), pan ve tilt eksenleri için bağımsız olarak ayarlanabilir ve limit çalışma zamanında (runtime) arayüzden değiştirilebilir. Bu adım, COARSE/FINE hız hesabından **sonra**, açılara entegre edilmeden **hemen önce** uygulanan bir post-processing katmanıdır.

---

## Çoklu Hedef ve Rota Sistemi

### Multi-Target

- `INITIAL_TARGET_COUNT = 4`, çalışma zamanında `MIN_TARGETS = 1` – `MAX_TARGETS = 12` arasında dinamik olarak ayarlanabilir
- Üç hedef profili (`normal`, `aggressive`, `slow`), her biri kendi hız aralığı ve OU-tipi ivme gürültü büyüklüğüne sahip — farklı tehdit/hedef davranışlarını simüle etmek için
- **Otomatik hedef seçimi:** `nearest` (platforma en yakın) veya `center` (en küçük azimuth) stratejileriyle çalışır
- **Flip-flop önleme:** iki hedefin skoru birbirine yakınsa otomatik mod her tick'te hedef değiştirmesin diye, yeni adayın en az bir **margin** kadar (`0.6 m` / `4.0°`) daha iyi olması ve son geçişten bu yana en az `AUTO_SWITCH_COOLDOWN_SEC = 1.2s` geçmiş olması şartı aranır. Sentetik bir testte bu koruma, korumasız yaklaşımın 300 tick'te 300 kez hedef değiştirdiği bir senaryoda geçiş sayısını 2'ye indirdi

### Waypoint / Rota Sistemi

- Kullanıcı tanımlı waypoint dizileri üzerinden hareket (`TRACKING_MODE_ROUTE`)
- Üç bitiş davranışı: `loop` (başa dön), `stop` (son noktada dur), `pingpong` (yönü ters çevir)
- Rota hızı çalışma zamanında `0.2–6.0 m/s` aralığında ayarlanabilir

---

## Mimari

Proje, kontrol mantığı ile arayüzün birbirinden tamamen ayrıştırıldığı, tek yönlü bağımlılığa sahip (UI → Core) katmanlı bir yapı kullanır. `core` katmanı UI'a bağımlı değildir; bu da kontrol/simülasyon mantığının arayüzden bağımsız olarak birim testlerle doğrulanabilmesini sağlar.

```
pantilt_tracker/
├── core/              # Kontrol döngüsü, PID, hedef/rota mantığı, durum makinesi
├── ui/                # PyQt5 arayüz katmanı — bkz. aşağıdaki modül kırılımı
├── visualization/      # OpenGL tabanlı 3D render pipeline
├── models/             # STL 3D model varlıkları (servo, kamera, braket)
├── config.py           # Tüm sistem/kontrol parametreleri (tek gerçek kaynak)
└── main.py             # Uygulama giriş noktası
```

### `ui/` Modül Kırılımı

Arayüz katmanı da kendi içinde sorumluluklarına göre alt paketlere bölünmüş durumda; hiçbir dosya birden fazla sorumluluk üstlenmiyor:

```
ui/
├── components/          # Genel/paylaşılan arayüz bileşenleri
│
├── control_panel/        # Kontrol paneli (sol panel)
│   ├── api.py             # Panelin dış dünyaya (core) sunduğu arayüz
│   ├── interactions.py    # Kullanıcı etkileşim mantığı (buton/slider callback'leri)
│   ├── panel.py           # Panel widget'ının kendisi / layout kurulumu
│   └── sections.py        # Panel içindeki alt bölümler (PID, rota, hedef vb.)
│
├── main_window/           # Ana pencere ve uygulama döngüsü
│   ├── handlers.py         # Olay/sinyal handler'ları
│   ├── layout.py           # Ana pencere yerleşimi
│   ├── loop.py             # Multi-rate döngünün UI thread'ine bağlanması (QTimer)
│   ├── selection.py        # Hedef/waypoint seçim mantığı
│   └── window.py           # QMainWindow tanımı
│
└── radar_widget/          # 2D radar/harita görselleştirme widget'ı
    ├── coordinates.py      # Dünya <-> ekran koordinat dönüşümleri
    ├── drawing_grid.py     # Izgara ve harita sınırları çizimi
    ├── drawing_hud.py      # HUD katmanı (açı, durum, telemetri overlay)
    ├── drawing_route.py    # Waypoint/rota çizimi
    ├── drawing_targets.py  # Hedef işaretleri, lock ring, aim-line çizimi
    ├── interaction.py      # Fare tıklama/sürükleme ile hedef-waypoint etkileşimi
    ├── style.py             # Renk/stil sabitlerinin widget'a uygulanması
    └── widget.py             # QWidget kök sınıfı, paint event orkestrasyon
```

Bu kırılımın öne çıkan noktaları:

- **`radar_widget` içinde çizim, koordinat dönüşümü ve etkileşim ayrı dosyalarda** — tek bir "god widget" yerine her sorumluluk kendi dosyasında, bu da örneğin yalnızca çizim stilini değiştirmenin (`style.py`) etkileşim mantığına (`interaction.py`) dokunmadan yapılabilmesini sağlar.
- **`control_panel` içinde arayüz mantığı (`interactions.py`) ile dış API (`api.py`) ayrı** — panelin `core`'a nasıl konuştuğu tek bir noktadan (`api.py`) yönetiliyor, bu da `core` tarafında bir arayüz değişikliği olduğunda etkiyi tek dosyaya izole ediyor.
- **`main_window/loop.py`**, multi-rate simülasyon döngüsünü Qt'nin event loop'una (`QTimer`, `TICK_MS = 4`) bağlayan katman — simülasyon zamanlaması ile UI thread'i arasındaki köprü burada izole edilmiş durumda.

---

## Görselleştirme

- **2D Radar (`radar_widget`):** hedefler, aktif hedef vurgusu, kilit halkası, nişan hattı (aim-line), rota/waypoint çizimi ve gerçek zamanlı HUD overlay
- **3D Sahne (OpenGL):** STL model tabanlı pan-tilt mekanizması (servo braketi, kamera assembly), pan/tilt pivotlarına göre gerçek zamanlı rotasyon, lazer/nişan çizgisi simülasyonu
- **Telemetri grafikleri:** açısal hata ve hız, `WINDOW = 100` örneklik kayan pencerede, `TRAIL = 50` örneklik iz ile

---

## Kurulum

```bash
git clone https://github.com/aligkyrr/multirate-pantilt-tracker.git
cd multirate-pantilt-tracker
pip install -r requirements.txt
python main.py
```

---

## Mühendislik Kapsamı

Proje aşağıdaki konularda somut, parametreleştirilmiş uygulama içerir:

- Multi-rate gerçek zamanlı sistem tasarımı (bağımsız Hz'lerde çalışan alt döngüler, sabit-dt entegrasyon, spiral-of-death koruması)
- Kapalı çevrim PID kontrol tasarımı (anti-windup, türev filtreleme, hedef kestirimi/lead)
- Durum makinesi tasarımında histerezis + debounce ile kararlılık (chatter önleme)
- Fiziksel kısıtları (ivme/hız limitleri) modelleyen aktüatör simülasyonu
- Çok hedefli ortamda otomatik hedef seçimi ve flip-flop önleme
- Katmanlı, tek-sorumluluk prensibine dayalı modüler yazılım mimarisi (özellikle `ui/` altında ayrıştırılmış alt paketler)
- Gerçek zamanlı 2D/3D veri görselleştirme

---

## Planlanan Geliştirmeler

- YOLO tabanlı gerçek zamanlı hedef tespiti (görüntü tabanlı giriş)
- Kalman filtresi ile hedef durumu kestirimi (state estimation)
- Donanım entegrasyonu (servo motor sürücü / Raspberry Pi üzerinde çalıştırma)
- Ağ üzerinden uzaktan kontrol arayüzü

---

## Geliştirici

**Ali İhsan Gökyer**
Elektrik-Elektronik Mühendislik Öğrencisi

---