# Çok Frekanslı Pan-Tilt Hedef Takip Sistemi

**İki eksenli bir pan-tilt takip platformunun gerçek zamanlı, çok frekanslı kontrol simülasyonu.**
PyQt5 tabanlı arayüz, OpenGL 3D görselleştirme, kapalı-çevrim PID kontrol ve Kalman filtresi tabanlı hedef durum kestirimi.

---

## Genel Bakış

Bu proje, hareketli hedefleri gerçek zamanlı takip eden iki eksenli (azimuth / elevation) bir pan-tilt platformunu modelleyen bir kontrol ve simülasyon sistemidir. Amaç görsel bir demo değil — gerçek zamanlı takip sistemlerinin karşılaştığı asıl problemlerin yazılım seviyesinde doğru bir modelini kurmak: farklı frekanslarda çalışan alt sistemlerin senkronizasyonu, kapalı-çevrim kontrol kararlılığı, titremesiz mod/kilit geçişleri, ölçüm gürültüsü altında hedef durum kestirimi ve hedef kaybı / yeniden-kilitlenme davranışı.

Sistem, üç bağımsız sabit-zaman-adımlı döngü üzerine kurulu ve tamamen `config.py` üzerinden parametrize edilmiştir; kod tabanında hiçbir kontrol veya zamanlama sabiti hardcode edilmemiştir.

---

## Kontrol Sistemi — Teknik Detaylar

### Çok Frekanslı Döngü Mimarisi

| Döngü | Frekans | Sorumluluk |
|---|---|---|
| Hedef Güncelleme | 60 Hz | Hedef fiziği (OU-tipi smooth random walk, sınır sekmesi) |
| Kontrol Döngüsü (PID) | 120 Hz | Açısal hata hesaplama, PID çıktısı üretimi |
| Pan-Tilt Güncelleme | 60 Hz | Açı/hız/ivme entegrasyonu, aktüatör fiziği |

Kontrol döngüsünü diğer iki döngünün iki katı frekansta çalıştırmak bilinçli bir tasarım kararıdır: kontrolcüyü, aktüasyon ve hedef fiziği güncellemesinden daha sık örneklemek, daha kararlı ve düşük gecikmeli bir PID çıktısı üretir.

Zamanlama, sabit `dt` adımlarıyla ilerleyen bir accumulator modeliyle yürütülür (`DT_TARGET`, `DT_CONTROL`, `DT_PANTILT`). Pencere sürükleme, bir breakpoint veya GC duraklaması yüzünden gerçek geçen süre anormal şekilde sıçrarsa, bir `MAX_DT = 0.1s` tavanı ve `MAX_CATCHUP_STEPS = 10` sınırı **spiral-of-death** durumunu (döngünün yetişmeye çalışırken kilitlenmesi) engeller. Bu, gerçek zamanlı gömülü sistemlerde standart bir güvenlik önlemidir ve varlığı, simülasyonun rastgele bir demo olarak değil, gerçek zamanlı sistem disipliniyle yazıldığının bir göstergesidir.

### PID Kontrolcü

```
PID_KP = 0.34   PID_KI = 0.015   PID_KD = 0.060
PID_INTEGRAL_LIMIT = 12.0     (anti-windup)
PID_OUTPUT_LIMIT   = 2.0 derece/tick   (fine-mod hız tavanı)
PID_DERIVATIVE_FILTER_ALPHA = 0.25
```

- **Anti-windup:** integral terim `PID_INTEGRAL_LIMIT` değerinde sınırlanır, böylece sürekli büyük hatalar integral doygunluğuna yol açıp kontrolcüyü kararsızlaştıramaz.
- **Türev filtreleme:** hedefin random-walk hareketinden gelen küçük gürültüler D teriminde büyütülüp motor çıktısında titreme olarak ortaya çıkabilir. Bu yüzden türev terimi alçak geçiren filtreden geçirilir (`α = 0.25`).
- **Hedef önden kestirimi (lead/prediction):** kontrolcü, hedefin o anki konumuna değil, Kalman-filtrelenmiş hız kestirimi (aşağıda açıklanıyor) kullanılarak `LEAD_TIME_SEC = 0.35s` ileriye projekte edilmiş bir konuma nişan alır — hareketli hedeflere karşı faz gecikmesini azaltmak için standart bir teknik.

### Coarse/Fine Mod Geçişi ve Lock-On Mantığı

Sistem iki kontrol modu arasında geçiş yapar:

- **COARSE:** açısal hata büyükken sabit bir tavan hızda (`COARSE_MAX_SPEED_DEG_PER_TICK = 8.0`) hızlı yaklaşma
- **FINE:** hata `TRACK_ANGLE_THRESHOLD_DEG = 2.5°`'nin altına düşünce PID hassas yaklaşma için devreye girer
- **LOCKED:** hata `LOCK_ANGLE_THRESHOLD_DEG = 1.2°`'nin altına düşünce kilit durumuna geçilir

Tek bir eşiğe göre naif şekilde mod değiştirmek, hata bu eşik civarında salındığında her tick'te mod değişmesine (titreme/chatter) yol açar. Bu proje bunu iki katmanlı mekanizmayla çözer:

1. **Histerezis (Schmitt-trigger mantığı):** çıkış eşiği, giriş eşiğinden daha yüksek tutulur — `COARSE_REENTRY_THRESHOLD_DEG = 4.0°`, `LOCK_EXIT_THRESHOLD_DEG = 2.16°`
2. **Debounce (ardışık-tick onayı):** histerezis tek başına yeterli değildir, çünkü sürekli hareket eden bir hedef yine de eşik etrafında salınabilir. Bir durum değişikliği (mod veya kilit), yeni durum `MODE_SWITCH_CONFIRM_TICKS = 6` (~96 ms) / `LOCK_CONFIRM_TICKS = 6` ardışık tick boyunca sürdüğünde kalıcı hale gelir.

Bu iki mekanizmanın birlikte kullanılması, gerçek radar/takip sistemlerinin track kararlılığını korumak için kullandığı yaklaşımı yansıtır.

### İvme Sınırlama

PID/coarse çıktısı doğrudan açıya uygulansaydı, hız bir tick'ten diğerine anlık olarak sıçrayabilirdi — gerçek bir servo motorun fiziksel olarak yapamayacağı bir şey. Bunu hesaba katmak için hızın değişim oranı ayrıca sınırlanır (`MAX_ACCEL_AZ/EL_DEFAULT_DEG_S2 = 150.0°/s²`), her eksen için bağımsız olarak yapılandırılabilir ve arayüzden çalışma zamanında ayarlanabilir. Bu, coarse/fine hız hesaplamasından **sonra** ve açıya entegre edilmeden **hemen önce** uygulanan bir son-işlem adımıdır.

### Hedef Durum Kestirimi — Sabit Hız (Constant-Velocity) Kalman Filtresi

Her `Target`, kendi 2D Kalman filtresini taşır (`core/kalman.py`, `CVKalmanFilter2D`) — durum vektörü `[px, py, vx, vy]` ve sabit-hız süreç modeliyle. Filtre bilinçli olarak bağımlılıksız yazıldı (saf Python, NumPy yok) — 4×4 kovaryans yayılımı ve 2×2 innovation-kovaryans tersi elle açık şekilde yazılmıştır, böylece modülün hiçbir dış bağımlılığı yoktur.

```
predict:  x_k = F x_{k-1}                  (sabit hız modeli)
          P_k = F P_{k-1} F^T + Q
update:   y   = z - H x_k                   (innovation)
          S   = H P_k H^T + R
          K   = P_k H^T S^{-1}
          x_k = x_k + K y
          P_k = (I - K H) P_k
```

`Target.step()`, her tick'te filtreyi ground-truth konumla "ölçüm" olarak besler, `predicted_position(lead_time)` ise ham anlık hız yerine filtrenin durum kestiriminden ekstrapolasyon yapar. Süreç gürültüsü (`q_vel`), hedef profiline göre `TARGET_NOISE_SCALE` / `TARGET_ACCEL_DAMPING`'den ölçeklenir; böylece `aggressive` bir hedefin filtresi yeni hız bilgisine daha hızlı güvenir (daha az gecikme, manevralara daha duyarlı), `slow` bir hedefin filtresi ise daha agresif şekilde yumuşatır.

Bu, planlanan YOLO tabanlı görüntü girdisi simüle edilmiş ground-truth hedef konumunun yerini aldığında sistemin üzerine oturacağı kestirim katmanı olarak tasarlanmıştır — bunun neden önemli olduğu için [Sonuçlar](#sonuçlar) bölümüne bakın.

---

## Çoklu Hedef ve Rota Sistemi

### Çoklu Hedef

- `INITIAL_TARGET_COUNT = 4` ile başlar, çalışma zamanında `MIN_TARGETS = 1` ile `MAX_TARGETS = 12` arasında ayarlanabilir
- Üç hedef profili (`normal`, `aggressive`, `slow`), her biri kendi hız zarfı ve OU-tipi ivme gürültüsü büyüklüğüyle — farklı hedef/tehdit davranışlarını simüle etmek için kullanılır
- **Otomatik hedef seçimi:** `nearest` (platforma en yakın) veya `center` (en küçük azimuth) stratejisi
- **Flip-flop önleme:** iki hedefin skoru birbirine yakınken auto modun her tick'te hedef değiştirmesini engellemek için, yeni aday mevcut hedefi en az bir **marjla** (`0,6 m` / `4,0°`) geçmeli ve son geçişten bu yana en az `AUTO_SWITCH_COOLDOWN_SEC = 1,2s` geçmiş olmalıdır. Sentetik bir testte bu, 300 tick üzerinde geçiş sayısını 300'den (korumasızken her tick) 2'ye düşürdü

### Waypoint / Rota Sistemi

- Kullanıcı tanımlı waypoint dizileri boyunca hareket (`TRACKING_MODE_ROUTE`)
- Rota sonu için üç davranış: `loop` (başa dön), `stop` (son noktada dur), `pingpong` (yönü tersine çevir)
- Rota hızı çalışma zamanında `0,2–6,0 m/s` aralığında ayarlanabilir

---

## Mimari

Proje, kontrol mantığını arayüzden tamamen ayıran katmanlı bir mimari (UI → Core tek yönlü bağımlılık) kullanır. `core` katmanının `ui`'a hiçbir bağımlılığı yoktur; bu da kontrol/simülasyon mantığının arayüzden bağımsız olarak birim test edilebilmesini sağlar.

```
pantilt_tracker/
├── core/
│   ├── kalman.py       # CVKalmanFilter2D — sabit-hız Kalman filtresi (bağımlılıksız)
│   ├── target.py        # Hedef fiziği, hedef tarafı durum kestirimi, TargetManager
│   └── ...               # Kontrol döngüsü, PID, rota mantığı, durum makinesi
├── ui/                  # PyQt5 arayüz katmanı — aşağıdaki modül dökümüne bakın
├── visualization/        # OpenGL tabanlı 3D render hattı
├── models/               # STL 3D model varlıkları (servo, kamera, braketler)
├── config.py             # Tüm sistem/kontrol parametreleri (tek doğruluk kaynağı)
└── main.py               # Uygulama giriş noktası
```

### `ui/` Modül Dökümü

Arayüz katmanı, sorumluluğa göre alt paketlere ayrılmıştır; hiçbir dosya birden fazla sorumluluk taşımaz:

```
ui/
├── components/          # Paylaşılan/yeniden kullanılabilir UI bileşenleri
│
├── control_panel/        # Kontrol paneli (sol panel)
│   ├── api.py             # Panelin core'a dışa dönük arayüzü
│   ├── interactions.py    # Kullanıcı etkileşim mantığı (buton/slider callback'leri)
│   ├── panel.py           # Panel widget'ının kendisi / layout kurulumu
│   └── sections.py        # Panel içindeki alt bölümler (PID, rota, hedef, vb.)
│
├── main_window/           # Ana pencere ve uygulama döngüsü
│   ├── handlers.py         # Olay/sinyal işleyicileri
│   ├── layout.py           # Ana pencere yerleşimi
│   ├── loop.py             # Çok frekanslı simülasyon döngüsünü UI thread'ine bağlar (QTimer)
│   ├── selection.py        # Hedef/waypoint seçim mantığı
│   └── window.py           # QMainWindow tanımı
│
└── radar_widget/          # 2D radar/harita görselleştirme widget'ı
    ├── coordinates.py      # Dünya <-> ekran koordinat dönüşümleri
    ├── drawing_grid.py     # Izgara ve harita sınırı çizimi
    ├── drawing_hud.py      # HUD katmanı (açı, durum, telemetri overlay)
    ├── drawing_route.py    # Waypoint/rota çizimi
    ├── drawing_targets.py  # Hedef işaretçileri, kilit halkası, nişan çizgisi çizimi
    ├── interaction.py      # Hedefler ve waypoint'ler için mouse tıklama/sürükleme etkileşimi
    ├── style.py             # Renk/stil sabitlerini widget'a uygular
    └── widget.py             # Kök QWidget sınıfı, paint-event orkestrasyonu
```

Bu dökümün öne çıkan noktaları:

- **Çizim, koordinat dönüşümleri ve etkileşim, `radar_widget` içinde ayrı dosyalarda yaşar** — tek bir "god widget" yerine her sorumluluğun kendi dosyası vardır, böylece çizim stilindeki bir değişiklik (`style.py`) etkileşim mantığına (`interaction.py`) dokunmadan yapılabilir.
- **`control_panel` içinde, UI etkileşim mantığı (`interactions.py`) dışa dönük API'den (`api.py`) ayrılmıştır** — panelin `core` ile nasıl konuştuğu tek bir noktadan yönetilir, `core` tarafındaki herhangi bir arayüz değişikliğinin etki alanı tek bir dosyaya izole edilir.
- **`main_window/loop.py`**, çok frekanslı simülasyon döngüsünü Qt'nin olay döngüsüne (`QTimer`, `TICK_MS = 4`) bağlayan katmandır — simülasyon zamanlaması ile UI thread'i arasındaki köprü burada izole edilir.
- **`core/kalman.py`, `core/target.py`'dan ayrıştırılmıştır** — filtre, genel amaçlı, hedeften bağımsız bir modüldür ve `config.py` veya hedef profilleri hakkında hiçbir bilgisi yoktur; `target.py`, alana özgü kalibrasyonu (süreç gürültüsünün hedef tipine nasıl eşlendiğini) kendi sorumluluğunda tutar.

---

## Görselleştirme

- **2D Radar (`radar_widget`):** hedefler, aktif-hedef vurgulama, kilit halkası, nişan çizgisi, rota/waypoint çizimi ve gerçek zamanlı HUD overlay
- **3D Sahne (OpenGL):** STL-model tabanlı pan-tilt mekanizması (servo braketleri, kamera montajı), pan/tilt pivotları etrafında gerçek zamanlı rotasyon, laser/nişan çizgisi simülasyonu
- **Telemetri grafikleri:** açısal hata ve hız, `WINDOW = 100`-örneklik kayan pencere üzerinde `TRAIL = 50`-örneklik iz ile

---

## Sonuçlar

Hedef-önden-kestirim (target-lead prediction) hattını doğrulamak için iki benchmark çalıştırıldı: naif ekstrapolasyon (`konum + hız * LEAD_TIME_SEC`) ile CV Kalman filtresinin `predicted_position()` çıktısı, her hedefin `LEAD_TIME_SEC = 0,35s` sonraki gerçek konumuna göre RMSE olarak karşılaştırıldı; `DT_TARGET` adımında 3000 tick boyunca.

**1. Gürültüsüz ground-truth girdi** (simülasyonun şu anki hali — `Target.step()`, filtreyi kendi kusursuz konumuyla besliyor):

| Hedef profili | Naif RMSE | Kalman RMSE |
|---|---|---|
| normal | 0,175 m | 0,201 m |
| aggressive | 0,445 m | 0,489 m |
| slow | 0,050 m | 0,063 m |

Naif ekstrapolasyon burada hafif *daha iyi* — bu beklenen bir sonuç, çünkü filtre, temizlenecek ölçüm gürültüsü sıfır olan bir hız sinyalini yumuşatıyor; bu yumuşatmanın kendisi küçük bir gecikme maliyetine dönüşüyor. Bu, girdi zaten ground-truth olduğunda filtrenin bir kazanç sağlamadığını doğruluyor.

**2. Simüle edilmiş gürültülü konum ölçümü** (`σ = 0,15 m`, planlanan YOLO gibi görüntü-tabanlı bir dedektörün doğruluğunu modelliyor — bu senaryoda hız doğrudan gözlenemez, ardışık gürültülü konum okumalarından kestirilmesi gerekir):

| Hedef profili | Finite-difference RMSE | Kalman RMSE | İyileşme |
|---|---|---|---|
| normal | 6,465 m | 0,356 m | %94,5 |
| aggressive | 6,443 m | 0,787 m | %87,8 |
| slow | 6,442 m | 0,181 m | %97,2 |

Fark burada bu kadar büyük çünkü iki gürültülü konum örneğini tek bir tick üzerinden (`Δt = 1/60s`) türevlemek, ölçüm gürültüsünü `1/Δt` kadar büyütüyor — 15 cm'lik bir konum hatası, çok büyük bir hız hatasına dönüşüyor. Bu tam olarak planlanan görüntü-tabanlı girdinin çalışacağı rejim, ve Kalman filtresinin sonradan eklenmek yerine kestirim katmanına şimdiden dahil edilmiş olmasının nedeni budur.

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

Proje şunların somut, tamamen parametrize edilmiş uygulamalarını içerir:

- Çok frekanslı gerçek zamanlı sistem tasarımı (bağımsız-Hz alt döngüler, sabit-dt entegrasyon, spiral-of-death koruması)
- Kapalı-çevrim PID kontrol tasarımı (anti-windup, türev filtreleme, hedef önden kestirimi)
- Histerezis + debounce ile durum-makinesi kararlılığı (titreme önleme)
- Bağımlılıksız sabit-hız Kalman filtresi ile hedef durum kestirimi, profil bazlı gürültü kalibrasyonuyla
- Fiziksel kısıtları modelleyen aktüatör simülasyonu (hız/ivme sınırları)
- Çoklu hedef ortamında flip-flop önlemeli otomatik hedef seçimi
- Katmanlı, tek-sorumluluklu modüler yazılım mimarisi (özellikle `ui/` altındaki alt paketler)
- Gerçek zamanlı 2D/3D veri görselleştirme

---

## Planlanan İyileştirmeler

- YOLO ile gerçek zamanlı hedef algılama (görüntü-tabanlı girdi) — mevcut `CVKalmanFilter2D` kestirim katmanını simüle edilmiş ground-truth yerine gerçek, gürültülü ölçümlerle besleyecek
- Donanım entegrasyonu (servo motor sürücü / Raspberry Pi dağıtımı)
- Ağ tabanlı uzaktan kontrol arayüzü

---

## Geliştirici

**Ali İhsan Gökyer**
Elektrik-Elektronik Mühendisliği Öğrencisi

---