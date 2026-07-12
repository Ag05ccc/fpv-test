# Roadmap v2 — Hareketli Hedefi Görsel Takip Eden FPV Dron (SITL)

Tarih: 2026-07-04. Bu doküman `docs/virtual-rc-gazebo-roadmap.md`'nin (v1)
devamıdır. v1 "dron sanal RC ile temiz uçsun" problemini çözmek içindi; v2'nin
konusu asıl proje hedefidir: sabit kameralı bir FPV dronun, simülasyon
ortamında hareket eden bir hedefi tracker ile takip etmesi, hedefi ekran
ortasında tutması ve ona yaklaşması. Ölçüm/kabul kuralları
`docs/sitl-flight-readiness-criteria.md`'den aynen devralınır.

## Hedef ve Kapsam

Uçtan uca akış şudur ve v2'nin bitiş tanımı bu akışın ölçülebilir biçimde
çalışmasıdır:

1. Kullanıcı (veya sanal pilot scripti) dronu kaldırır, hedefi ekran ortasına
   getirir ve takip komutunu verir (3 konumlu AUX: IDLE → AI-ARMED → TRACKING).
2. TRACKING'e girişte tracker ekran ortasındaki kutuya kilitlenir.
3. Kontrolcü hedefi yatayda ekran ortasında tutmak için yaw, hedefe
   yaklaşmak/uzaklaşmak için pitch komutu üretir (bbox genişliği ≈ mesafe
   vekili). Throttle ve roll pilotta kalır (`msp_override_channels = 10`
   sözleşmesi).
4. Komutlar Betaflight'a gider, dron döner/yaklaşır, kamera görüntüsü değişir,
   tracker yeni kareyle devam eder — yani çevrim kapalıdır.
5. Hedef kaybolursa komut gönderimi durur, ~2 sn sonra AI-ARMED'a düşülür,
   pilot kontrolü geri alır.

Tasarım kararları (bilinçli, v2 boyunca sabit):

- Kamera sabittir, gimbal yok. Dikey eksende bağımsız bir kontrol yüzeyi
  yoktur (pitch mesafe kapatmada kullanılıyor); bu yüzden "ekran ortasında
  tutma" hedefi YATAY eksende ölçülür, dikey piksel hatası telemetri olarak
  kaydedilir ve yalnız geniş bir kabul bandıyla izlenir. Dikey hata bandın
  dışına taşmayı alışkanlık edinirse throttle bağlaşımı ayrı bir aşama olarak
  açılır (şimdilik kapsam dışı).
- Önce yavaş hedef: kapalı çevrim kapıları 0.5–1.0 m/s hedef hızıyla başlar,
  hız merdiveni ölçerek çıkılır. "Hızlı araç takibi" ancak yavaş merdiven
  basamakları 5/5 geçtikten sonra denenir.
- Fiziksel kumanda hiçbir kapının ön şartı değildir (kalıcı kullanıcı kararı,
  2026-07-02). Her şey sanal RC / scriptli MSP / klavye RC ile koşar.

## Dokuz Metrik — Bugünkü Ölçülmüş Durum ve Eksikler

Kullanıcının dokuz sorusu v2'nin gereksinim listesidir. Her biri için bugün ne
ölçülüyor, ne eksik:

1. **Arm/disarm düzgün çalışıyor mu?** ARM tarafı ölçülüyor:
   `sitl_virtual_takeoff_check.py` BOOTGRACE'in temizlenmesini bekliyor,
   `armed_angle_samples`'ı sayıyor (safe-yaw regresyonu 5/5 PASS, 2026-07-02;
   kamera-açık koşu 59 ARM+ANGLE örneğiyle PASS, 2026-07-03). DISARM hiçbir
   yerde doğrulanmıyor: RC scripti disarm komutu gönderiyor ama hiçbir analiz
   "pencere sonunda disarmed + motorlar ~1000" iddiasını kontrol etmiyor.
   → Kapanış: A0'daki koşu-sonu disarm kapısı.
2. **Arm olunca pervaneler dönüyor mu?** `gazebo_sitl_motor_smoke.py` PASS
   (MSP_MOTOR 1402, rotor eklemleri dönüyor). Eksik: arm sonrası motor
   çıkışının 1000'in üstüne çıktığını koşu içinde erken doğrulayan otomatik
   kontrol (sessiz hiç-arm-olmama sınıfı ancak koşu sonunda anlaşılıyor).
   → Kapanış: A0 geçerlilik kapısı (armed=0 → INVALID) + motor yükselme
   kontrolü.
3. **Throttle verince kalkıyor mu?** Ölçülüyor: irtifa kazancı kapısı
   (2026-07-02 ailesinde 23–31 m sınıfı PASS'ler). Eksik: kazanç pencere
   süresiyle normalize edilmiyor (bilinen tuzak: 75 örnek ≈ 40–50 s, 120 örnek
   ≈ 70–77 s; tırmanış ~0.6 m/s). → Kapanış: A0'daki tırmanış hızı metriği
   (m/s) ve pencere süresinin her kanıta yazılması.
4. **Roll/pitch/yaw komutlarına DOĞRU tepki veriyor mu?** En kritik açık. Spin
   probleminin mekanizması ölçüldü (aşağıdaki problem listesi) ve güvenli
   pencere kapıları var (safe-yaw nudge, video yaw5/pitch5 sınırları,
   sync-dünyası yaw2 5/5). Ama bugünkü kapıların HEPSİ büyüklük ölçüyor
   ("devrilmedi"), hiçbiri yön ölçmüyor ("komut verilen yöne döndü").
   `sitl_pid_sweep_summary.py` gereken işaretli veriyi (poz yaw deltası, eksen
   biası) zaten kaydediyor ama iddia üretmiyor. → Kapanış: A1 işaretli
   komut-tepki kapısı.
5. **Otopilota komut gönderebiliyor muyum?** MSP taşıma katmanı ölçülü:
   `kenet_msp_smoke.py --set-raw-rc` AETR→MSP eşlemeli okuma-yazma PASS.
   Eksikler: (a) uçuş kanıtlarının tamamı UDP 9004 sanal RC ile; Kenet'in
   üretim yolu olan MSP Override ile hiçbir Gazebo uçuş kapısı koşulmadı;
   (b) `pipeline._build_override_channels` giden kareyi pilot/AETR sırasında
   gönderiyor — kanal sıra dönüşümü yalnız okuma tarafında var, yazma tarafında
   ters dönüşüm yok (SITL varsayılan map'inde maskelenen gizli kusur).
   → Kapanış: A5 (yazım yolu düzeltmesi + MSP Override loopback kapısı +
   MSP-Override'lı uçuş kapısı).
6. **Hedef takibi için makul bir PID var mı, nasıl teyit ederim?** Kontrolcü
   var (yaw: yatay piksel hatası, pitch: bbox genişliği; anti-windup, slew
   limit). Yön sözleşmesi ölçülü (`sitl_pid_direction_check.py` PASS). Eksik:
   yakınsama kanıtı yok — hiçbir test kalıcı/hareketli bir hataya karşı
   oturma süresi, aşım veya "hedef gerçekten ortalanıyor" ölçmüyor; GCS
   `set_pid` komutu dokümante ama işlenmiyor (ölü komut); kazançlar iki yerde
   ayrı ayrı gömülü. → Kapanış: A6 (çevrimdışı yakınsama testi + sim içi adım
   cevabı kapısı).
7. **Tracker çalışıyor mu, yumuşak harekette stabil mi?** Gerçek CSRT yolu
   yalnız kayıt video kapısında ölçülü (`test-2.mp4` neutral + target-loss
   PASS). Hareket dayanıklılığı hiç ölçülmedi: hiçbir test hedefi kareler
   arasında hareket ettirip sürüklenme/kopma ölçmüyor. → Kapanış: A2'deki
   çevrimdışı tracker hareket bankı (bilinen yörünge → piksel hatası/IoU).
8. **Sim ortamındaki hedefler hareket ediyor mu?** HAYIR — %0. Dört manzara
   modeli de (`kenet_person*`, `kenet_car*`) `static`, populated dünyada actor
   veya hareket plugin'i yok. Görünen tüm hareket ego-hareket. → Kapanış: A3
   (set_pose tabanlı hedef sürücü + ölçülmüş yer-değiştirme kapısı).
9. **Sim kamera görüntüsünü tracker'a besleyebiliyor muyum?** HAYIR — köprü
   yok(tu). `/kenet/fpv_camera` konusunun tek tüketicisi Gazebo GUI paneli;
   `kenet/camera.py` yalnız cv2 VideoCapture biliyor(du). Algı çevrimi açık:
   tracker dronun gördüğünü hiç görmüyor. → Kapanış: A2 (gz-transport köprüsü;
   bu turda eklendi, aşağıda).

## Gazebo–Betaflight Bilinen Problemler (ölçülmüş durum listesi)

Geliştirme sırasında tekrar yaşanmaması gereken problemler ve durumları.
Kaynak kanıtlar `gazebo-bug1.md`, `docs/virtual-rc-gazebo-roadmap.md`,
`logs/sitl/` ailesi.

Çözüldü (regresyon kapısıyla korunuyor / korunacak):

- Plugin yaw jiroskop Z işareti (nötr kalkış flip'inin kök nedeni) — plugin'de
  düzeltildi; kanıt çifti 20260629-221238 (FAIL) → 20260629-221545 (PASS).
- Iris motor eşleme çifte-remap'i — `--fix-iris-motor-map` + kaynak SDF kimlik
  eşlemesi.
- Betaflight looptime/dT çarpıklığı (sanal jiroskop 8 kHz iddia ediyor, döngü
  FDM adımında koşuyor; adım bağımlılığının kök nedeni) —
  `tools/betaflight_looptime_sync.patch`, `KENET_SITL_LOOPTIME_US` ile kapılı,
  değişken atanmazsa bit-bit stok. DENEYSEL: yalnız ≥2 ms adım.
- Kayıplı lockstep uyandırma — `tools/betaflight_lockstep_sem.patch` (sayaçlı
  semafor); sync@0.0025 transient'siz.
- Sync-dünyası video yaw braketini öldüren varsayılan ROLL PID'i — tüm-eksen
  sim-tune (`configs/sync-tune-allaxis-p23.txt`); yaw2 5/5 PASS.
- Kamera render yükünün sync-KAPALI dünyayı flip'e itmesi —
  `launch-fpv-sim.sh` truthful-timing profili + `configs/fpv-sim.txt`
  (small_angle=180 arming bloğu dahil).
- MSP TCP tek-istemci çakışması ve BOOTGRACE yarışı — runner'da arming-grace
  beklemesi.

Açık (v2 içinde adreslenecek):

- **Plant yaw otorite uyumsuzluğu** (spin probleminin kalan mekanizması):
  Gazebo iris aktüatör modelinin P-only rotor hız döngüsü (`vel_p_gain=0.05`,
  838 rad/s sabit ölçek) ile Betaflight varsayılan yaw otoritesinin kapalı
  çevrim uyumsuzluğu. Arıza yaw P'sinde ve setpoint büyüklüğünde monoton;
  hafifletme `--safe-yaw-authority` (uçuş tune'u DEĞİL, kabul profili).
  Kalıcı düzeltme adayı: plugin rotor hız PID'inin dt-normalize edilmesi +
  SDF-parametreli maxRpm; ardından sync-ON yeniden braket. (A1)
- Sync@1 ms zamanlayıcı açlığı: bounded-wait faz yarışını çözdü, geç-stall
  sınıfı (~t+20 s) açık. Hiçbir kapı 0.001+sync gerektirmediği için bloklayıcı
  değil.
- Sync-dünyası yaw3–yaw5 metastabil satürasyon bandı (~%50, bir motor 2000'de
  satüre). "Ölçülmüş sınır" etiketiyle kayıtlı; bandı genişletme işi plant yaw
  otorite ölçeğine bağlı.
- Kanıt dünyası bölünmesi: kabul kanıtlarının çoğu sync-KAPALI 0.0025
  dünyasında; kamera yolu sync gerektiriyor. Kapılar 5/5 ölçüldükçe tek tek
  sync dünyasına taşınır (kriter dokümanındaki karar); iki dünyanın kanıtı
  asla karıştırılmaz, her kanıt satırı adımı ve dünyayı yazar.

## Çalışma Kuralları (her aşama için geçerli)

- Bir koşu önce GEÇERLİ olmalı: `armed_angle_samples > 0`, ilk diagnostik
  örnek temiz (|roll|,|pitch| < 5°, motorlar 1000, disarmed), MSP kesintisiz,
  port/süreç hijyeni. Arm olmayan koşu FAIL değil INVALID'dir ve kanıt olamaz.
- Kabul = art arda 5 geçerli koşu 5/5 PASS. Tek koşu hiçbir hükmü kurmaz;
  %100 çıkmayan nokta "ölçülmüş sınır" etiketi alır.
- Her kanıt satırı fizik adımını (`--max-step-size`) ve dünyayı
  (sync-ON/OFF, dünya dosyası) birlikte yazar.
- Kapı eşiği değiştirilerek "geçirme" yapılmaz; eşik değişikliği ayrı commit +
  gerekçe ister.
- Claude'un geliştirme döngüsü: her davranış değişikliğinden sonra önce
  çevrimdışı paket (`fpv_env/bin/python -m pytest -q`), sonra değişikliğin
  dokunduğu katmanın canlı kapısı koşulur; kapı çıktısı (PASS/FAIL/INVALID +
  metrikler) rapora aynen yazılır.

## Aşamalar

Her aşamanın çıkış kriteri ölçülebilir bir kapıdır; "yaptım" beyanı kapı
çıktısı olmadan geçersizdir.

### A0 — Koşu Geçerliliği ve Sağlık Otomasyonu (öncelik: hemen)

Bugün elle yapılan kontrolleri araca çevirir; bundan sonraki her aşamanın
güvenlik ağıdır.

Yapılacaklar:

- `tools/sitl_run_quality_check.py` (yeni): bir koşunun diagnostics +
  motor-UDP JSONL'ini alır ve şu hükümleri üretir:
  - Geçerlilik: armed_angle_samples>0, ilk örnek temizliği, MSP süreklilik →
    VALID/INVALID (nedenleriyle).
  - Katman-2 sağlık: RTF bandı (0.99–1.01), motor UDP kadansı (adımla
    tutarlı), disarm-öncesi >30 ms boşluk yokluğu.
  - Disarm kapısı: pencere sonunda disarmed + motorlar sıfır komutta (metrik
    1'in eksik yarısı).
  - Tırmanış hızı: irtifa kazancı / ölçülen pencere süresi (m/s) (metrik 3
    normalizasyonu).
  - İşaretli tepki (varsa nudge bilgisi): poz yaw deltasının işareti komutun
    işaretiyle uyumlu mu (metrik 4'ün ilk otomatik yarısı).
- `sitl_readiness_report.py`'nin bu araca delege etmesi; donmuş 2026-06-30
  kanıt yollarının parametreleşmesi.
- 5/5 tekrar sayacı: aynı profilin ardışık koşularını sayan basit zincir
  raporu.

Çıkış kriteri: geçmiş loglardan seçilmiş bilinen-PASS ve bilinen-INVALID
koşular araç tarafından doğru sınıflanır (altın örnek testleri pytest'te) ve
takeoff runner'ın yeni koşusu araçtan geçer.

### A1 — RPY Komut Doğruluğu: "Devrilmedi"den "Doğru Yöne Döndü"ye

Spin probleminin kullanıcıya görünen yüzü. İki iş paralel yürür:

- İşaretli komut-tepki kapısı: nudge koşusunda (örn. yaw 1504) Gazebo poz yaw
  deltasının işareti ve büyüklük bandı iddialaşır; pitch nudge için poz
  x/y-ekseni ilerlemesi. Mevcut safe-profil pencereleri (safe-yaw P23/yaw1504,
  pitch1522@0.001, sync yaw2) işaretli kapıyla YENİDEN koşulur; kanıtlar
  adım+dünya etiketiyle yenilenir.
- Kalıcı plant düzeltmesi: plugin rotor hız döngüsünün dt-normalize edilmesi
  (feedforward + SDF-parametreli maxRotorVelocity). Ölçüm sırası: (1) yalnız
  plugin değişikliğiyle sync-OFF regresyon (safe-yaw 5/5 korunmalı), (2)
  sync-ON 0.0025 yeniden braket (yaw bandının yaw2'den yukarı genişlemesi
  hedef), (3) genişleme ölçülürse kabul profillerinin sync dünyasına göçü.

Çıkış kriteri: işaretli yaw ve pitch komut-tepki kapıları güvenli profilde
5/5 PASS; yaw komut bandının sync dünyasında ölçülmüş yeni sınırı raporlanır.

### A2 — Sim Kamera → Tracker Köprüsü ve Tracker Hareket Bankı

Metrik 7 ve 9'u kapatır. (Köprü bu turda eklendi; canlı kapı koşusu bekliyor.)

- `kenet/gz_camera.py`: gz-transport görüntü aboneliği, `CameraCapture` ile
  aynı arayüz; kaynak adı `gz:/kenet/fpv_camera` (fabrika:
  `kenet.camera.create_camera_capture`). Pipeline ve mixer aynı fabrikayı
  kullanır; `kenet.py --camera gz:` ve `kenet_sitl_mixer.py --camera gz:` ile
  canlı sim kamerası tüketilir.
- `tools/sitl_gz_camera_probe.py`: kamera kapısı — N saniyede kare hızı,
  çözünürlük, kare bayatlığı ölçer; eşik altı FAIL (örn. ≥20 fps, 640x480).
- Çevrimdışı tracker hareket bankı: sentetik, dokulu bir hedefi bilinen
  yörüngede (yatay süzülme, ölçek değişimi) hareket ettiren kare dizisi
  üretilir; CSRT/KCF koşulur; merkez hatası ve IoU eşikleri iddialaşır
  ("yumuşak harekette kopmuyor" sorusunun ölçülebilir hali).

Çıkış kriteri: kamera probu canlı simde PASS; mixer canlı kamera kaynağıyla
neutral kapıyı (sıfır limit, tracker hatasız) geçer; hareket bankı pytest'te
deterministik PASS.

### A3 — Hareketli Hedef Altyapısı

Metrik 8'i kapatır.

- `tools/sitl_target_mover.py` (yeni): `/world/<dünya>/set_pose` servisiyle
  seçilen modeli (örn. `kenet_car`) parametrik yörüngede sürer (düz hat
  gidiş-dönüş, daire, hız m/s, güncelleme Hz). Yer gerçeği JSONL'e yazılır
  (t, x, y, z, yaw, komut hızı).
- Ölçülmüş hareket kapısı: sürücünün kendi komutuna güvenilmez —
  `/world/<dünya>/pose/info` aboneliğinden ölçülen yer değiştirme, komutlanan
  yörüngeyle karşılaştırılır (min yer değiştirme + rota hatası eşiği).
- Dünya eşdeğerliği: hedef hareket ederken safe-yaw kabul penceresi yeniden
  koşulur (populated dünya kanıtı 22.985 m sınıfındaydı); hareketin fizik
  bütçesini bozmadığı (RTF bandı) A0 aracıyla doğrulanır.

Çıkış kriteri: hareket kapısı PASS (ölçülmüş yer değiştirme, ground-truth
JSONL kayıtlı) ve hareketli-hedefli dünyada uçuş sağlık kapıları bozulmadan
PASS.

### A4 — Kapalı Çevrim Görsel Takip Kapısı (projenin kalbi)

A2 + A3 birleşir: dron kendi kamerasında gördüğü HAREKETLİ hedefi izler.

- Senaryo: sanal pilot kalkış + hedefe yönelim (mevcut mixer akışı), TRACKING
  gecikmeli başlar (`--kenet-delay-seconds` sınıfı), hedef aracı 0.5 m/s ile
  düz hatta sürülür.
- Ölçütler (mixer + tracker + mover ground-truth JSONL'lerinden): takip
  sürekliliği (pencere boyunca target_found oranı ≥ eşik, kopma sayısı),
  yatay piksel hatasının bandı (örn. |err| ortalama < 60 px, uç < 160 px),
  bbox genişliği ile mesafe kapatma eğilimi, uçuş sağlığı (A0 kapıları,
  attitude bandı, spread).
- Hız merdiveni: 0.5 → 1.0 → 2.0 m/s; her basamak 5/5 ister; kopan basamak
  "ölçülmüş sınır" olarak raporlanır ve A6 tune girdisi olur.

Çıkış kriteri: 0.5 m/s basamağı 5/5 PASS (sync dünyası + tüm-eksen tune
profili); rapor hız-merdiveni sınırını içerir.

### A5 — Üretim Yolu: MSP Override ile Uçuş

Metrik 5'in kalan yarısı. Bugün tüm uçuş kanıtı UDP sanal RC'de; Kenet'in
gerçek yolu MSP Override.

- Yazma yolu düzeltmesi: `_build_override_channels` çıktısının Betaflight'ın
  beklediği kanal sırasına dönüştürülmesi (okuma yönündeki eşlemenin tersi);
  `kenet/rc_channels.py`'ye ters eşleme + birim testleri.
- Loopback kapısı (Gazebo'suz): SITL'e Kenet gönderim yoluyla bilinen
  pitch/yaw değerleri yaz, `MSP_RC` geri-okumasında doğru iç kanalların
  kımıldadığını iddiala (mevcut `kenet_msp_smoke.py` deseninin override
  yönlüsü).
- Uçuş kapısı: A4 senaryosunun aynısı, ama komutlar mixer'ın UDP kanalı yerine
  `kenet.py`'nin MSP Override yolundan gider (sanal RC yalnız pilot
  kanallarını taşır). msp_override_channels=10 maskesi ve TRACKING dışında
  gönderim olmaması iddialaşır.

Çıkış kriteri: loopback kapısı PASS + MSP-Override'lı kapalı çevrim kapısı
0.5 m/s basamağında PASS.

### A6 — PID Yakınsama ve Ayar Kapıları

Metrik 6'yı "makul mü?"den "ölçülü" hale getirir.

- Çevrimdışı yakınsama testi (pytest): birinci dereceden basit bir ego-hareket
  modeliyle (yaw komutu → piksel hatasının orantılı azalması) kontrolcü
  çevrimi simüle edilir; sabit ofset hedefte oturma süresi ve aşım bandı,
  sabit hızlı hedefte kalıcı hata bandı iddialaşır. Bu test kazanç
  değişikliklerinde regresyon bekçisidir.
- Sim içi adım cevabı: kapalı çevrimde hedefi anlık +X piksel kaydır (mover
  ile), oturma süresi/aşım ölç; JSONL'den otomatik raporla.
- Temizlik: PID kazançlarının tek kaynağa inmesi (PipelineConfig), `set_pid`
  GCS komutunun ya işlenmesi ya silinmesi, kazançların CLI'dan verilebilmesi
  (`--yaw-kp` ailesi) — böylece tune koşuları config dosyasıyla izlenebilir.

Çıkış kriteri: çevrimdışı yakınsama testleri pakette; sim içi adım cevabı
raporu en az bir kazanç seti için 5/5 tekrarlı.

### A7 — Hız Merdiveni ve "Hızlı Araç" Hedefi

- A4 merdiveni A6 tune'uyla yukarı itilir; her basamak adım+dünya etiketli
  5/5 kanıt ister.
- Tracker sınırı ile kontrol sınırı ayrıştırılır: kopma tracker'dan mı
  (hareket bankındaki eşdeğer hız da kopuyor) yoksa kontrolcüden mi (banka
  geçiyor, çevrim kopuyor)?
- Gerekirse tracker aday listesi genişletilir (örn. detection destekli
  yeniden-kilitlenme) — ayrı karar noktası, bu roadmap'te taahhüt değil.

Çıkış kriteri: ulaşılan en yüksek 5/5 hız basamağı ve kopma modu raporu.

## Metrik → Kapı Eşlemesi (özet)

1. Arm/disarm → runner arming-grace + `sitl_run_quality_check.py` disarm
   hükmü (A0).
2. Pervane dönüşü → `gazebo_sitl_motor_smoke.py` + koşu içi motor yükselme
   kontrolü (A0).
3. Throttle→kalkış → tırmanış hızı (m/s) metriği (A0).
4. RPY doğru tepki → işaretli komut-tepki kapıları (A1) + mevcut
   nudge/attitude/spread kapıları.
5. Otopilota komut → `kenet_msp_smoke.py` (mevcut) + override loopback ve
   MSP-Override uçuş kapısı (A5).
6. Takip PID'i → yön sözleşmesi (mevcut) + yakınsama testleri ve adım cevabı
   (A6).
7. Tracker sağlamlığı → video neutral/target-loss (mevcut) + hareket bankı
   (A2).
8. Hedef hareketi → mover + ölçülmüş yer değiştirme kapısı (A3).
9. Kamera→tracker → gz köprüsü + kamera probu + canlı-kamera neutral kapısı
   (A2).

## Bu Turda Eklenenler ve Ölçülenler (2026-07-04)

Eklenen araçlar ve ilk canlı ölçümleri (dünya: populated, headless, kamera
enjekte, Betaflight bağlı değil — sim serbest koşuyor):

- `kenet/gz_camera.py` — gz-transport kamera köprüsü (`gz:<topic>` kaynağı);
  `kenet.camera.create_camera_capture` fabrikası, pipeline + mixer
  entegrasyonu (`kenet.py --camera gz:` ve `kenet_sitl_mixer.py --camera
  gz:/kenet/fpv_camera` çalışır durumda).
- `tools/sitl_gz_camera_probe.py` — metrik 9 kapısı. CANLI PASS: 150 kare /
  5.0 s = 30.0 fps, 640x480, 0 çözme hatası, son kare yaşı 6 ms. Köprüden
  alınan örnek kare populated sahneyi (insan modelleri + sarı araç) doğru
  BGR renkleriyle gösteriyor.
- `tools/sitl_target_mover.py` — metrik 8 kapısı. CANLI PASS: `car_front_1`,
  düz hat 0.5 m/s, 20 s: 395 komut @20 Hz, ölçülen yer değiştirme 7.999 m
  (komutlanan 8 m ping-pong ucu), ortalama komut-gözlem hatası 0.025 m,
  395 istekte 1 yanıt kaybı. Ground-truth JSONL:
  `logs/sitl/20260704-152855-target-mover-car_front_1.jsonl`.
  İki ölçülmüş gz-transport tuzağı kodda belgeli: (1) servis keşfi tembel —
  ilk istekler zaman aşımına düşer ama komutlar sunucuda GEÇ uygulanır
  (araç koşu sonunda son komuta ışınlanır); çözüm yeniden denemeli ısınma
  adımı. (2) Kaçan tiki telafi etmeye çalışan istek fırtınası yanıt kaybını
  %6'dan %56'ya katlar; çözüm mutlak-tik temposu + tik atlama. Yanıt kaybı
  artık FAIL nedeni değil tanılama sayacıdır; hüküm ölçülen harekete dayanır.
- `tools/sitl_run_quality_check.py` — A0 çekirdeği. Altın örnek doğrulaması
  geçmiş loglarla yapıldı: 2026-07-03 kamera-açık PASS koşusu → PASS
  (VALID, RTF 0.9959–1.0015, kadans 2.53 ms, tırmanış 0.436 m/s);
  `20260630-235335-bracket-pitch1515` → INVALID (armed=0 + ilk örnek roll
  −180, dokümante nedenlerle birebir); `flipfix-yaw1650` → FAIL (tırmanış
  0.010 m/s). Disarm hükmü şimdilik rapor-modunda (`--require-disarm` ile
  kapıya döner) çünkü mevcut 75-örnek diagnostik pencereleri disarm fazını
  kapsamıyor — runner penceresi uzatılınca zorunlu yapılacak (A0 kalan işi).
- `tools/tracker_motion_benchmark.py` — metrik 7 çevrimdışı bankı.
  Deterministik sentetik dizi (dokulu hedef, bilinen yörünge). Ölçüldü:
  KCF 3 px/kare süzülme → found 1.00, ort. hata 3.0 px, IoU 0.93 PASS;
  CSRT 3 px/kare + ölçek 1.003/kare → ort. hata 2.8 px, IoU 0.89 PASS.
- `tests/test_controller_convergence.py` — metrik 6 çevrimdışı yakınsama
  kapıları: sabit 200 px ofset ≤6 s'de deadband bandına oturuyor ve kalıyor;
  40 px/s süzülen hedefte kalıcı hata sınırlı; bbox-genişlik (yaklaşma)
  döngüsü 120 px hedefe yakınsıyor; hedef kaybında çubuklar merkeze dönüyor.
- Yeni çevrimdışı testler toplamda: köprü/probe/mover/quality-check/banka/
  yakınsama. Paket: **362 test, tümü geçiyor** (~3 s).

Sıradaki adımlar (öncelik sırası): A0 kalanı (runner penceresine disarm fazı +
`sitl_readiness_report.py` delegasyonu), A1 işaretli komut-tepki kapısının
mevcut safe-profil koşusuyla ilk ölçümü (`--expect-yaw-sign` kalibrasyonu),
A2 canlı-kamera neutral mixer kapısı, A4 ilk kapalı çevrim koşusu
(mover + gz kamera + mixer, 0.5 m/s).

## 2026-07-04 A1+A2 Yürütme Sonuçları (ölçüldü)

Bu turda A1 ve A2 canlı olarak koşuldu ve sonraki aşamaların önündeki KÖK
ENGEL ölçümle bulundu. Tüm koşular headless, populated/harmonic dünya,
`sitl_run_quality_check.py` + runner kapılarıyla değerlendirildi.

### KÖK ENGEL: plant yaw-otorite flip'i (metrik 4, spin problemi)

Ölçülmüş, tekrarlanabilir bulgu — kullanıcının "rpy komutu verince spin atıyor"
şikâyetinin tam karşılığı:

- **Komutsuz uçuş kusursuz**: no-nudge, 40 s hold, throttle 1500 → PASS,
  roll/pitch `0.000`, motor spread `0.000`, tırmanış 4.68 m
  (`logs/sitl/a1-nonudge-hold40-*`). Dron havada mükemmel stabil.
- **Herhangi bir sürekli yaw komutu flip yaptırıyor**: +4 µs (yaw 1504) VE +2 µs
  (yaw 1502, 12 s gecikmeli) nudge → 8/8 koşuda roll 180 flip, motor spread
  945, ilk büyük split saf yaw ekseninde (-349 … -402). Test edilen TÜM
  konfigürasyonlar flip yaptı:
  - lockstep-ON sync-OFF (`a1-yawpos-02`, populated 0.0025)
  - lockstep-ON sync-ON + tüm-eksen tune (`a1-sync-yawpos-01`)
  - lockstep-OFF (Betaflight yeniden derlendi, `a1-nolockstep-yawpos-01`)
  - velocity-control τ=30 ms (`a1-velctrl-t30-yawpos`)
  - rotor damping 0.0005 (`a1-damp0005-yawpos`, arıza pitch'e kaydı)
  - literal dokümante safe-yaw komutu (`a1-baseline-harmonic`)
- **İşaret DOĞRU, büyüklük ~40-150× fazla**: yaw-sağ nudge (1504) → burun saat
  yönü → ENU'da azalan pose_yaw (flip öncesi 90°→−88°, ~5 s). Yani yön
  sözleşmesi doğru; sorun saf otorite. Sürekli komutta dron ~9-35°/s yaw'a
  gidiyor (beklenen ~0.24°/s), motorlar 2000/1000'e satüre oluyor, roll/pitch
  kontrolü otorite kalmadığından çöküyor → flip.
- **Betaflight zamanlaması dışlandı**: lockstep on/off ve looptime sync on/off
  hiçbiri kurtarmadı. Mekanizma plant tarafında: rotor reaksiyon torku (yaw)
  LiftDrag `cda`=0.004 sürükleme × `cp`=0.084 moment kolundan geliyor; itki
  bağımsız `cla` lift teriminden. Yaw otoritesini düşürmek `cda`/`cp`/damping
  retune'u gerektiriyor ama düşük-damping koşusu flip'i pitch'e kaydırdı
  (kuplaj), yani bu tek-parametre değil koordineli bir sim-retune — ve şu an
  kusursuz olan hover'ı riske atıyor. Bu, kullanıcı yönlendirmesi gereken
  ayrı bir iş (harici Gazebo modelinde geri-alınabilir ama iteratif ölçüm
  gerektiren cerrahi).
- **Kanıt-binary kayması**: dokümante safe-yaw 5/5 kanıtı (2026-07-02) lockstep
  KAPALI bir binary'de ölçülmüştü; mevcut binary Jul-3'te lockstep AÇIK +
  semafor ile yeniden derlendi ve o günkü tüm başarılı koşular NO-NUDGE kamera
  uçuşlarıydı. Yani mevcut binary hiçbir zaman bir yaw nudge'ıyla test
  edilmemişti; ilk kez bu turda test edildi ve flip yaptı. Eski bracket
  sayıları mevcut binary'ye taşınamaz.

**ÖNEMLİ AYRIM — sürekli-tutulan komut vs geçici takip komutu (ölçüldü):**
Yukarıdaki flip'ler SÜREKLİ TUTULAN yaw komutuyla (nudge 40 s boyunca sabit)
oldu — bu gerçek takipte olmayan bir durum. Gerçek takip komutu GEÇİCİDİR
(hedefi ortalamak için yaw ver, ortalanınca sıfıra dön). Geçici, sınırlı takip
yaw'ı ölçüldü ve **STABIL**: canlı-kamera mixer, `--yaw-limit 3`, populated +
sync + fpv-sim.txt → dron takip için yaw komutu verdi (mixer max yaw delta 3,
toplam pose yaw −153° DOĞRU yönde), roll `1.7` pitch `0.7` ile **SEVİYEDE
KALDI, flip YOK**, 222 canlı kare izlendi, 0 tracker hatası
(`logs/sitl/20260704-170228-video-tracking-*`). Yani kapalı çevrim takip
VİYABIL — flip yalnız gerçekçi-olmayan sabit-tutulan komutta.

Ölçülen takip otorite bandı: `yaw-limit 3` seviyeyi korur ama sürüklenen
hedefi ortalamaya yetmez (yaw_error 0'dan 150-318 px'e büyüdü — hedef kaçtı).
Yani band "takip için çok zayıf (±3)" ile "flip (sabit büyük komut)" arasında;
bu bandı genişletmek plant yaw-otorite retune'unun (cda/cp) faydalanacağı
yer — ama takip için zorunlu değil, yalnız daha hızlı hedefler için.

Ek ölçülen uyarı: kamera-açık + sync koşularında RTF kararsız (0.46-1.10,
render yükü); uçuş seviyede kalıyor ama sim wall-clock zamanlaması dalgalanıyor.
`sitl_run_quality_check.py` RTF bandı (0.99-1.01) kamera-açık koşular için
sıkı; bu koşularda RTF ayrı bir kamera-yük metriği olarak raporlanmalı.

Sonuç: A4 kapalı çevrim takip TEMELDE ÇALIŞIYOR (dron seviyede kalarak hedefi
yaw ile takip ediyor). Açık iş: takip otorite bandını hedef hızına göre
ayarlamak (A7) ve gerekirse plant retune ile genişletmek. A1 flight 5/5
(sabit-tutulan nudge profili) mevcut ortamda "ölçülmüş sınır" olarak kalır —
ama bu takip için bloklayıcı değildir.

Ek ölçülmüş mekanizma (sabit vs reaktif yaw): ultra-düşük yaw otoritesi
(yaw-pid 8, rate 10, limit 30) sabit-tutulan nudge'ı KURTARMADI ama flip'i
ROLL eksenine kaydırdı (yaw ekseni ~0, roll ekseni 407). Yani sabit SÜREKLİ
aynı-yönlü yaw dönüşü, yaw otoritesinden BAĞIMSIZ olarak attitude'u
kararsızlaştırıyor (muhtemelen sürekli dönüşte tutum tahmini bozulması). Buna
karşılık A4 takip koşusunda dron toplam −319° yaw yaptı (hedef sağa-sola
geçerken reaktif, YÖN DEĞİŞTİREN yaw) ve roll `1.5` ile SEVİYEDE KALDI. Ders:
gerçek takip (yön değiştiren reaktif yaw) stabil; sabit tek-yön yaw
kararsız — proje takibi ilkine dayanır.

### ⚠️ 2026-07-04 DÜZELTME 2: A4 testinin KENDİSİ yanlıştı — pozisyon yakınsaması ölçülmeli

Kullanıcı kritik test-tasarım hatasını gösterdi: "dronun bir YÖNE dönmesi"
takip testi değildir. Doğru test = **dronun POZİSYONU hedefe yaklaşıyor mu**
(statik hedefte mesafe kapanıyor mu; hareketli hedefte mesafe sınırlı kalıp
dron hedefle birlikte hareket ediyor mu). Yerinde dönen/asılı kalan bir dron
"yön" testini geçer ama takip etmiyordur.

Eklenen doğru ölçüm (ikisi de Gazebo pose yer-gerçeğinden):
- `tools/sitl_track_truth.py`: 50 Hz'de hem dron (iris) hem hedef pozisyonuna
  abone; dron→hedef yatay mesafesini, dron yol uzunluğunu (yerinde
  dönme/asılı kalmayı eler), yaw rate'i (spin) ölçer. Statik/hareketli hedefi
  ayırıp doğru kriteri uygular.
- `evaluate_convergence` + 5 offline test: gerçek yaklaşma PASS, yerinde-spin
  FAIL, hareketli-hedef-takip PASS, uzaklaşma FAIL.

**Ölçülmüş sonuç (canlı, hareketli araç, forward+yaw açık):** araç 32.8 m
hareket etti; **dron 0.03 m hareket etti** (yerinde asılı), mesafe 11.4 m'de
kapanmadı → **FAIL: "drone did not move through space"**. Yani dron takip
ETMİYOR — hedef önünden geçerken yerinde duruyor. Eski yön-tabanlı testim bunu
"tracking works" diye geçiriyordu; pozisyon-yakınsama testi anında yakalıyor.
Kanıt: `logs/sitl/20260704-201222-track-truth-car_front_1.jsonl`.

Sonuç: A4 kabul kriteri artık POZİSYON YAKINSAMASI (`sitl_track_truth.py`).
Mevcut dron bunu geçmiyor (ya spin atıyor ya yerinde asılı kalıp yaklaşmıyor;
forward/approach ekseni de hareket üretmiyor). Bir sonraki iş: dronun hedefe
doğru fiilen HAREKET etmesini sağlamak (approach ekseni + spin kök nedeni).

### ⚠️ 2026-07-04 DÜZELTME 1: A4 "PASS" YANLIŞTI — dron yaw ekseninde SPIN atıyor

Kullanıcı GUI'de izleyince gördü: kalkıştan sonra dron yaw ekseninde çılgınca
dönüyor. Aşağıdaki "A4 PASS" iddiası YANLIŞTI ve nedeni bir ÖLÇÜM KÖRLÜĞÜdür:

- **50 Hz gerçek ölçüm (yeni `tools/sitl_yaw_monitor.py`, Gazebo pose'a doğrudan
  abone):** kalkış sonrası yaw rate tepe **7655 deg/s** (~21 tur/sn), toplam
  yaw 1490° (~4 tam tur), SPIN DETECTED. Dron takip ETMİYOR, dönüyor.
- **Neden göremedim (3 birleşik kusur):**
  1. Flip kapım yalnız roll/pitch ölçüyordu, yaw'ı DIŞLIYORDU — seviyede kalan
     bir spin (roll/pitch ~0) roll/pitch kapısından geçer.
  2. Yaw RATE'i hiç hesaplamadım; büyük toplam-yaw'ı (−319°) "aracı takip
     dönüşü" diye YANLIŞ etiketledim. "Hedef kutusu 0→640 süpürdü" de dronun
     dönmesiyle sahnenin kameradan geçmesiydi, aracı takip değil.
  3. Diagnostik yaw'ı ~2-3 Hz'de (MSP round-trip) örnekliyor; 7655 deg/s spin
     örnek başına ~2500-3800° döner → ALIASING: sinyal "90°'de sabit" gibi
     görünür. Bu boru hızlı yaw dinamiğini fiziksel olarak göremez.
- **Ölçüm düzeltmesi (bu turda, fix'ten ÖNCE — kullanıcı isteği):**
  - `tools/sitl_yaw_monitor.py`: 50 Hz pose aboneliği, gerçek-zaman yaw + yaw
    rate, SPIN eşiği; canlı koşuda 7655 deg/s'i yakaladı.
  - `sitl_run_quality_check.py` içine `check_yaw_spin` eklendi (Gazebo
    pose'dan yaw rate; MSP'den bağımsız). Eski "PASS" logları yeniden
    puanlandı: 1.0 m/s **SPIN 436 deg/s**, 2.0 m/s **SPIN 106 deg/s** (diagnostik
    aliaslı alt sınır; gerçek 50 Hz tepe 7655). Artık hiçbiri sessizce
    PASS olamaz. Offline testler pakette.
- **Durum:** A4 (kapalı çevrim takip) SAĞLANMADI. Aşağıdaki eski A4/A7 "PASS"
  satırları GEÇERSİZDİR ve yalnızca ölçüm körlüğünü gösteren tarihsel kayıt
  olarak bırakılmıştır. Bir sonraki iş: spin'in kök nedenini (artık 50 Hz'de
  görülebiliyor) bulup düzeltmek.

### A4 — [GEÇERSİZ — yukarıdaki düzeltmeye bakın] eski "İLK PASS" kaydı

`tools/sitl_moving_target_track_check.py` (yeni orkestratör): kamera-takip
checker'ını (Gazebo + FPV kamera + Betaflight + gz-kamera izleyen mixer) ve
hedef aracı dronun önünde yatay geçiren sürücüyü aynı anda koşturur, sonra
takip sürekliliği + işaretli yaw + uçuş sağlığı + ölçülmüş hedef hareketini
değerlendirir.

**İlk ölçülmüş PASS (0.5 m/s, populated + sync + fpv-sim.txt, step 0.0025):**
- Hedef (`car_front_1`, z=1.5'e yükseltilerek kamera optik eksenine
  hizalandı) ölçülen yanal hareket **5.99 m**.
- Tracker **446 kare** izledi, found ratio **1.0**, **0 tracker hatası**;
  izlenen bbox merkezi kare boyunca (0→622 px, 640 genişlik) süpürdü — yani
  HAREKETLİ aracı izledi, sabit sahneyi değil.
- Mixer takip için yaw komutu verdi (max yaw delta **6**, işaretli poz yaw
  −319° DOĞRU yönde — aracı takip ederek döndü).
- Uçuş **SEVİYEDE**: max roll `1.5°`, pitch `0.4°`, motor spread `793 µs`
  (945 flip imzasının altında — reaktif takipte geçici, flip değil).
- Kanıt: `logs/sitl/*-a4-track-*` + `logs/sitl/*-a4-mover-car_front_1.jsonl`.

Bu, projenin çekirdek yeteneğinin ölçülmüş kanıtıdır: dron KENDİ KAMERASINDA
gördüğü HAREKETLİ hedefi, sınırlı yaw ile seviyede kalarak takip ediyor.
Kabul için 5/5 tekrar zinciri gerekli (ilk PASS ölçüldü); A7 hız merdiveni
(1.0, 2.0 m/s) sıradaki.

Gate kalibrasyonu (takip senaryosuna özgü, `_QualityArgs`): roll/pitch birincil
flip kapısı (`max_abs_attitude` 25°), motor spread eşiği 900 µs (945 flip
imzası altında, agresif yaw geçicilerinin üstünde), MSP-süreklilik gevşetildi
(`msp_min_connected_fraction` 0.5 — kamera+mixer MSP çekişmesi diagnostik
poll'ünü düşürüyor ama uçuşu değil; Gazebo pose 120/120 sağlam), düşük-hover
takibinde tırmanış aranmaz. Offline testler pakette
(`test_sitl_moving_target_track_check.py`).

### A5/A6/A7 durum

- A5: yazma yolu DOĞRULANDI + loopback bekçisi (yukarıda). Override'lı Gazebo
  uçuş kapısı: A4 orkestratörü mixer UDP yolunu kullanıyor; kenet.py MSP
  Override yolunu kullanan varyant sıradaki (write-path zaten kanıtlı, düşük
  risk).
- A6: kod temizliği DONE. Sim içi adım cevabı: A4 koşusu ETKİN BİR ADIM
  CEVABIDIR (hedef yatay adım → dron yaw ile takip); mixer log'undan oturma
  ölçülebilir. Formalizasyon sıradaki.
- A7: hız merdiveni ölçüldü — **0.5 m/s PASS** (446 kare, found 1.0, roll
  1.5°), **1.0 m/s PASS** (452 kare, found 1.0, roll 2.1°), **2.0 m/s PASS**
  (found 0.92, roll 1.3°, yaw delta 6, hedef yanal 6.0 m). Her basamakta uçuş
  SEVİYEDE kaldı ve dron yaw ile takip etti. **Tracker-vs-kontrol ayrımı
  ölçüldü**: 2.0 m/s'te sınırlayan KONTROL değil TRACKER — found ratio
  1.0'dan 0.92'ye düştü (hızlı hedefte kare kaybı), uçuş kontrolü seviyede
  kaldı. Yani hızlı araç takibinde bir sonraki iyileştirme ekseni tracker
  (yeniden-kilitlenme / detection destekli), kontrol değil. Kabul için her
  basamakta 5/5 tekrar zinciri gerekir (ilk PASS'ler ölçüldü).

### A1 — Kısmen tamamlandı (kapı hazır, flight engelli)

- İşaretli komut-tepki kapısı `sitl_run_quality_check.py` içinde HAZIR ve
  DOĞRULANDI: flip koşularını doğru FAIL ediyor (climb 0.118 m/s, runaway yaw
  delta 485.6° yakalandı). Kalibrasyon: yaw-sağ nudge (1504) için beklenen
  işaret `-1` (ENU azalan), `--expect-yaw-sign -1` + `--max-yaw-delta-deg` ile
  runaway yakalanır. Çevrimdışı testler pakette.
- Flight PASS bloklu (yukarıdaki plant engeli). Durum: ölçülmüş sınır.

### A2 — TAMAMLANDI

- Kamera köprüsü + canlı prob: PASS (30.0 fps, 640x480, 0 çözme hatası).
- Tracker hareket bankı: offline PASS (KCF/CSRT, ort. hata ~3 px, IoU ~0.9).
- **Canlı-kamera mixer neutral kapısı: PASS** — populated dünya + FPV kamera +
  sync + fpv-sim.txt, `--camera gz:/kenet/fpv_camera --yaw-limit 0
  --forward-limit 0`: mixer 228 canlı kare (640x480) izledi, 0 tracker hatası,
  kenet delta `0`, dron temiz uçtu (roll `0.0`, pitch `0.3`, spread `3.0`).
  Kanıt: `logs/sitl/20260704-165507-video-tracking-*`. Bu, algı→PID→FC
  çevriminin canlı sim görüntüsüyle uçtan uca kapandığını kanıtlar; kamera
  render yükünün flip yapmadığını da gösterir (flip yalnız yaw komutu).

### A5 — Yazma yolu DOĞRULANDI (uçuş engelli)

- `tools/sitl_msp_override_loopback.py` (yeni): pipeline'ın override kare
  kurma yolunu birebir taklit edip gerçek SITL'e yazıyor, MSP_RC geri-okumada
  kontrolcü pitch/yaw'ın doğru iç kanallara düştüğünü, pilot throttle/roll'un
  korunduğunu iddia ediyor. CANLI PASS. Reader'ın işaretlediği kanal-sıra
  kusuru ÇÜRÜTÜLDÜ — üretim gönderim yolu doğru. Loopback artık kalıcı
  regresyon bekçisi (+ offline testler).
- Override'lı uçuş kapısı: plant engeline bağlı.

### A6 — Kod temizliği TAMAMLANDI (sim adım cevabı engelli)

- PID kazançları tek kaynağa indi (`PipelineConfig`); `kenet.py`'deki
  duplikasyon kaldırıldı, `--yaw-kp/ki/kd` ve `--forward-kp/ki/kd` CLI
  bayrakları eklendi.
- GCS `set_pid` komutu artık İŞLENİYOR (önceden ölü komuttu):
  `FlightController` PID'lerine canlı kazanç seti. Offline testler pakette.
- Sim içi adım cevabı: plant engeline bağlı (kapalı çevrim gerektiriyor).

### A0 — Araç HAZIR (regresyon bulgusu eklendi)

- `sitl_run_quality_check.py` altın örneklerle doğrulandı ve bu turda 11 canlı
  koşuyu doğru sınıfladı (no-nudge PASS, flip'ler FAIL, dirty-first INVALID).
- Kalan polish: runner penceresine disarm fazı + `sitl_readiness_report.py`
  delegasyonu.

Paket: **369 test PASS**. Betaflight checkout deneyden sonra pristine'e
döndürüldü (kaynak lockstep=1, binary orijinal yedekle bit-bit aynı).
