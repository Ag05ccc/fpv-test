# Virtual RC Gazebo Roadmap

Tarih: 2026-07-01 (güncelleme: 2026-07-02)

Amaç: Virtual RC, Gazebo + Betaflight + Kenet uçuş problemlerini fiziksel
kumandadan izole ederek çözmek için kullanılır. 2026-07-02 kararı: bütün
geliştirme ve test işlemleri sanal RC / scriptli MSP ile, kullanıcıdan fiziksel
kumanda beklenmeden yürütülür. Fiziksel RC bir roadmap aşaması değil, kullanıcı
kendi inisiyatifiyle istediğinde koşulacak opsiyonel bir donanım doğrulamasıdır;
hiçbir kapı ona bağlanmaz.

Ölçülebilir "SITL hazır / uçuş sorunsuz" hükmü için katmanlı kriter seti:
`docs/sitl-flight-readiness-criteria.md`. Tekrar kriteri (kabul profili 5/5),
koşu geçerliliği (armed>0, temiz ilk örnek) ve flip kapanış kriteri orada
tanımlıdır.

## Mevcut Hüküm

Fiziksel RC debug hattından kalıcı olarak ayrıdır ve hiçbir aşamanın ön şartı
değildir. `/dev/input/js*` görünse bile canlı external RC sender açmadan önce
virtual RC acceptance ve no-send physical preflight geçmeli; bu yalnız
kullanıcının kendi başlattığı opsiyonel donanım doğrulaması için geçerlidir.

Kanıtlı iyi durumlar:

- No-hardware readiness PASS.
- Safe-yaw virtual takeoff PASS.
- Real video target-loss PASS.
- PID direction ve mixer matrix gate PASS.
- Kenet mixer sözleşmesi sağlam: `TRACKING` durumunda yalnız pitch/yaw Kenet
  tarafından değişir; roll/throttle pilotta kalır.

Kanıtlı sorun alanları:

- Geniş yaw/pitch komutlarında Gazebo + Betaflight kapalı çevrim hâlâ runaway
  üretebiliyor.
- P22 yaw sınırı metastable: bazı koşular PASS, bazı koşular FAIL.
- P23/yaw1504 baseline FAIL; instrumented debug loglarında yaw gyro/error/P/Sum
  runaway görülüyor.
- Geniş provoke denemeleri safe-yaw acceptance penceresinden ayrı tutulmalı.

Ölçülmüş kök neden durumu (2026-07-02, log + kod doğrulaması):

- Zamanlama/jitter hipotezi dışlandı: RTF her koşuda ~1.000 (0.9935–1.0013),
  motor UDP kadansı metronomik ve pass/fail koşularında istatistiksel olarak
  aynı; hiçbir paket boşluğu split'e öncülük etmiyor. Lockstep commit'i
  (`f11faf414`, `ENABLE_SIMULATOR_GYROPID_SYNC`) hiçbir FAIL vakasını PASS
  yapmadı; neutral zaten lockstep öncesinde de PASS'ti. gazebo-bug1.md'deki
  "RTF ~1.31 / simRate kayması" hipotezi ölçümle çürütüldü.
- Lockstep mekanizması bloklamalı bekleme değil trylock-atla: FDM paketi
  başına en fazla 1 PID iterasyonu; motor paketi zaten her build'de FDM başına
  1 taneyle kapılı (`updateLock`). Gazebo durursa Betaflight'ın motor çıkışı
  süresiz donar ama saat/failsafe zamanlayıcıları ölçekli duvar saatiyle
  ilerlemeye devam eder — bu yapısal risk not edildi, mevcut flip'in nedeni
  değil.
- Fizik adımı eksen kararlılığını iki yönde birden değiştiriyor (2026-07-02
  ölçümü): 0.001'de pitch nudge kararlı hale gelirken yaw P sınırı P21/P22'den
  P19 PASS / P20 FAIL'e iniyor (P22 `5/5 FAIL` deterministik — eski
  "metastability" 0.0025 ayrıklaştırmasının ürünüymüş) ve video yaw4 kapısı
  FAIL oluyor (yaw2 PASS). 0.0025'te tersine: video/yaw kapıları PASS, pitch
  nudge (PID23 profili) FAIL. Hiçbir adım iki ekseni birden kararlı yapmıyor;
  adım bir kabul parametresidir, her kapı kendi adımını sabitler ve kanıtla
  kaydeder. Runner varsayılanı 0.0025'e geri alındı.
- Kalan mekanizma: Gazebo iris aktüatör modeli (P-only rotor hız döngüsü
  `vel_p_gain=0.05`, hız komutu 838 rad/s ölçeği, plugin'de sabit) ile
  Betaflight varsayılan yaw otoritesinin kapalı çevrim uyumsuzluğu. Arıza
  monoton: P21 PASS, P22 sınır, P23 FAIL; yaw1503 PASS / yaw1504 FAIL.
  Setpoint'i rampalayan `yaw_rate_limit=120` (safe-yaw) rotoru lineer bölgede
  tutup P23'ü PASS yapıyor.
- Motor ölçekleme şüphesi çürütüldü: SITL paket değeri `(PWM-1000)/1000` ile
  0.0–1.0 tam aralığı kullanıyor; "%20 otorite kaybı" yok.
- İkincil, gerçek ama tali: koşular arası durum sızıntısı ölçüldü (ilk örnekte
  roll `-180` ile başlayan, hiç arm olmayan koşular). Bu sınır okumalarındaki
  flakiness'i açıklıyor, sınırın kendisini değil. Koşu geçerliliği kuralları
  `docs/sitl-flight-readiness-criteria.md` içinde.

## Geçerli Kabul Penceresi

Bu pencere uçuş çözümü değil, Gazebo/SITL debug için ölçülmüş güvenli profildir.

```bash
fpv_env/bin/python tools/sitl_virtual_takeoff_check.py \
  --max-step-size 0.001 \
  --capture-motor-udp \
  --nudge-delay-seconds 18 \
  --nudge-yaw 1504 \
  --yaw-pid 23,0,0 \
  --safe-yaw-authority \
  --debug-mode AC_ERROR \
  --hold-seconds 40
```

Kabul:

- `PASS`
- ARM + ANGLE sample var.
- Max roll/pitch kabul eşiğinin altında.
- Raw motor spread runaway göstermiyor.
- Diagnostics, motor UDP ve virtual RC JSONL birlikte kaydediliyor.

Ölçülmüş command-response sınırları:

- Yaw-only real video: `yaw5 PASS`, `yaw6 FAIL`.
- Pitch-only real video: `pitch5 PASS`, `pitch6 FAIL` (`pitch-pid 23,0,0` ile).
- Combined real video: `yaw3+pitch3 PASS`, `yaw4+pitch4 FAIL`.

## Aşama 1 - Virtual RC Baseline'ı Dondur

Hedef: Her debug turunun aynı başlangıçtan koştuğunu garanti etmek.

Komutlar:

```bash
tools/check_sitl_env.sh
fpv_env/bin/python -m pytest -q
fpv_env/bin/python tools/sitl_pid_direction_check.py
fpv_env/bin/python tools/sitl_mixer_matrix_check.py
fpv_env/bin/python tools/sitl_readiness_report.py
```

Kabul:

- Env usable.
- Unit/no-hardware gate temiz.
- `sitl_readiness_report.py` no-hardware `PASS`.
- Physical RC gate `WAITING` olabilir; bu debug hattını bloklamaz.

## Aşama 2 - Safe-Yaw Virtual Takeoff Regression

Hedef: Gazebo + Betaflight temel kalkış ve küçük yaw nudge profili her zaman
PASS kalmalı.

Kabul komutu:

```bash
fpv_env/bin/python tools/sitl_virtual_takeoff_check.py \
  --max-step-size 0.001 \
  --capture-motor-udp \
  --safe-yaw-authority \
  --debug-mode AC_ERROR \
  --hold-seconds 40
```

Kabul:

- Altitude gain pozitif.
- Max roll/pitch düşük.
- Raw motor spread runaway yok.
- `debug_mode=AC_ERROR` loglanıyor.

Bu gate bozulursa fiziksel RC'ye geçilmez.

## Aşama 3 - Yaw/Pitch Authority Braketini Netleştir

Hedef: Uçuşu bozan komut büyüklüğünü ve hangi kontrol teriminin tetiklediğini
virtual RC ile ölçmek.

Öncelik sırası:

1. P22 tekrar matrisi: en az `5` koşu, PASS/FAIL oranı ve raw split zamanı.
2. P23/yaw1504 baseline ile `--safe-yaw-authority` A/B karşılaştırması.
3. I-only sınır: `I=1`, `I=2`, yaw1502/yaw1503/yaw1504 hold40 tekrarları.
4. Pitch axis için `pitch-pid 23,0,0` dışındaki güvenli/unsafe sınırın aynı
   şekilde ölçülmesi.

Örnek P sınır koşusu:

```bash
fpv_env/bin/python tools/sitl_virtual_takeoff_check.py \
  --max-step-size 0.001 \
  --capture-motor-udp \
  --nudge-delay-seconds 18 \
  --nudge-yaw 1504 \
  --yaw-pid 22,0,0 \
  --debug-mode AC_ERROR \
  --hold-seconds 40
```

Kabul:

- Her koşu için diagnostics + motor UDP + virtual RC logları saklanır.
- `tools/sitl_pid_sweep_summary.py` ile raw split, attitude eşik zamanı ve
  axis bias çıkarılır.
- P22 güvenli tune olarak kabul edilmez; ancak sınır davranışı açıklanır.

## Aşama 4 - Gerçek Video / Kenet Command-Response

Hedef: Kenet'in gerçek tracker çıktısı uçuşu bozmayacak sınırlarda komut
üretsin.

Yaw-only kabul:

```bash
fpv_env/bin/python tools/sitl_video_tracking_check.py \
  --camera test-2.mp4 \
  --run-id video-target-found-yaw4-p23-delay18 \
  --virtual-hold-seconds 40 \
  --yaw-limit 4 \
  --forward-limit 0 \
  --max-abs-delta 4 \
  --min-abs-yaw-delta 4 \
  --max-abs-yaw-delta 4 \
  --max-abs-pitch-delta 0 \
  --yaw-pid 23,0,0 \
  --kenet-delay-seconds 18
```

Pitch-only kabul:

```bash
fpv_env/bin/python tools/sitl_video_tracking_check.py \
  --camera test-2.mp4 \
  --run-id video-target-found-pitch5-p23-delay18-pitchpid23 \
  --virtual-hold-seconds 40 \
  --yaw-limit 0 \
  --forward-limit 5 \
  --max-abs-delta 5 \
  --min-abs-pitch-delta 5 \
  --max-abs-pitch-delta 5 \
  --max-abs-yaw-delta 0 \
  --yaw-pid 23,0,0 \
  --pitch-pid 23,0,0 \
  --kenet-delay-seconds 18
```

Combined kabul:

```bash
fpv_env/bin/python tools/sitl_video_tracking_check.py \
  --camera test-2.mp4 \
  --run-id video-target-found-yaw3-pitch3-p23-delay18-pitchpid23 \
  --virtual-hold-seconds 40 \
  --diagnostic-samples 120 \
  --diagnostic-interval 0.1 \
  --yaw-limit 3 \
  --forward-limit 3 \
  --max-abs-delta 3 \
  --min-abs-yaw-delta 3 \
  --min-abs-pitch-delta 3 \
  --max-abs-yaw-delta 3 \
  --max-abs-pitch-delta 3 \
  --yaw-pid 23,0,0 \
  --pitch-pid 23,0,0 \
  --kenet-delay-seconds 18
```

Kabul:

- PASS yalnızca stabil uçuş değil, beklenen eksende gerçek komut üretildiği
  anlamına gelir.
- Target-found takeoff/ramp sırasında değil, gecikmeli başlar.
- Target-loss sonrası `pilot-target-lost` ve `AI-ARMED` görülür; pitch/yaw
  pilot passthrough'a döner.

## Aşama 5 - Opsiyonel Fiziksel RC Doğrulaması (Kullanıcı İnisiyatifi)

Bu aşama roadmap'in tamamlanması için gerekli değildir ve hiçbir zaman
kullanıcıdan istenmez. Aşama 1-4 + `docs/sitl-flight-readiness-criteria.md`
kriterleri sanal RC ile kapandığında proje "SITL hazır" sayılır. Kullanıcı
kendi isteğiyle fiziksel kumandayı denemek isterse ön koşullar şunlardır:

- Aşama 1-4 PASS.
- Safe-yaw profile ve command-response sınırları dokümante.
- Geniş komut/provoke denemeleri acceptance'tan ayrı etiketlenmiş.
- No-send physical preflight CH5/CH6/CH7 davranışını gösteriyor.

Preflight:

```bash
fpv_env/bin/python tools/sitl_physical_rc_preflight.py \
  --device /dev/input/js0 \
  --force-mode-pwm 1500 \
  --verbose
```

Fiziksel RC canlı gate sadece bundan sonra:

```bash
fpv_env/bin/python tools/sitl_virtual_takeoff_check.py \
  --rc-driver external \
  --safe-yaw-authority
```

Ayrı terminalde tek RC sender:

```bash
fpv_env/bin/python tools/sitl_rc_bridge.py \
  --device /dev/input/js0 \
  --send \
  --force-mode-pwm 1500
```

Kabul:

- Fiziksel RC sonucu virtual RC gate ile aynı kriterlerle okunur.
- Yeni kabul kriteri icat edilmez.
- Physical RC FAIL olursa önce RC mapping/preflight farkı aranır; Gazebo
  physics/PID sorunu varsayılmaz.

## Açık Kararlar

1. Safe-yaw authority profili acceptance profili olarak kalacak mı, yoksa
   Betaflight default rate/PID'e daha yakın yeni bir güvenli profil mi
   üretilecek? (Aday yön: Gazebo iris rotor hız döngüsünü gerçekçi
   güçlendirmek — plugin `maxRpm` sabitini SDF'ten ayarlanabilir yapmak ve
   `vel_p_gain`'i feedforward'lı hale getirmek — böylece Betaflight tarafında
   yapay düşük otoriteye gerek kalmaz.)
2. Pitch axis için yaw kadar ayrıntılı P/I sweep gerekli mi, yoksa mevcut
   `pitch-pid 23,0,0` + command-response sınırı yeterli mi?
3. `analyze_sitl_log.py` / `sitl_readiness_report.py` içine INVALID verdict'i
   (armed=0, kirli ilk örnek) ve Katman 2 sağlık metrikleri (RTF bandı,
   kadans) ne zaman eklenecek?

Kapanan kararlar (2026-07-02):

- Zamanlama/lockstep hipotezi test edildi ve dışlandı (yukarıdaki "Ölçülmüş
  kök neden durumu"). Lockstep açık kalabilir; zararsız ve deterministik, ama
  çözüm o değil.
- P22 kararı: kaçınılacak; "ölçülmüş sınır" olarak etiketlenir, kabul
  profiline yazılmaz (`docs/sitl-flight-readiness-criteria.md` Katman 4).
- Fiziksel RC: hiçbir kapının ön şartı değil; tamamen opsiyonel, kullanıcı
  inisiyatifinde (CH7 sorusu ancak o gün gündeme gelir).

## 2026-07-02 Aşama Koşuları (ölçüldü, tamamı sanal RC)

Aşama 1 PASS: pytest `267 passed`, PID direction PASS, mixer matrix
P6.1–P6.8 PASS, readiness `no_hardware=PASS` (fiziksel RC `WAITING`,
bloklamıyor). Süreç/port hijyeni temizdi; bayat `bf1.pid` silindi.

Aşama 2 PASS `5/5`: safe-yaw regression (`--max-step-size 0.001`, hold 40)
beş ardışık koşuda PASS — spread `0.000`, roll/pitch `0.000/0.000`, ARM+ANGLE
65–66, MSP 75/75, RTF 0.998–1.006, kadans ~999 Hz. Kanıt:
`logs/sitl/20260702-0131xx..0136xx-takeoff-*`.

Aşama 3 ölçümleri (yaw1504 nudge, step 0.001, temp cwd, hepsi geçerli koşu):
P19 `2/2 PASS` (spread 0.838), P20 FAIL, P21 `2/2 FAIL`, P22 `5/5 FAIL`
(deterministik yaw-baskın split, bias −390..−399 µs), P23+safe-yaw PASS
(spread 0.682). I-only: I1 FAIL (roll-baskın split +416 µs!), I2 FAIL
(pitch-baskın split −394 µs) — sapma ekseni kazanca göre değişiyor; sorun
kazanç avıyla çözülmez, aktüatör modelindedir. Pitch profili (PID23 +
safe-manual + throttle 1500): step 0.001'de `1522/1530/1540 PASS`, step
0.0025'te `1522 FAIL`.

Aşama 4 PASS (bugünkü binary ile, step 0.0025 pinli): video yaw4 PASS
(irtifa 31.5 m, roll/pitch 0.5/5.5), pitch5 PASS, combined yaw3+pitch3 PASS,
real-video target-loss PASS (kenet→pilot-target-lost→AI-ARMED zinciri
doğru). Bilgi: yaw4 step 0.001'de FAIL, yaw2 step 0.001'de PASS. Video ve
synthetic checker'lara `--max-step-size` pass-through bayrağı eklendi
(testleriyle birlikte); step artık her kapıda kayıt altında.

Not: Önceki "0.001'de tırmanış ~%25 düşük" gözlemi yanlış alarmdı —
diagnostik pencere süresi koşudan koşuya değişiyor (75 örnek ≈ 40–50 s);
tırmanış hızları aynı. İrtifa-kazancı eşiği pencere hızına duyarlıdır; sınıra
yakın profillerde (throttle 1475) throttle 1500 kullanılır.

## 2026-07-02 Gece: Looptime-Sync Deneyi (ölçüldü)

Adım-bağımlılığın kök nedeni bulundu ve kısmen düzeltildi. Betaflight SITL'in
sanal jiroskopu 8 kHz örnekleme iddia ediyor (`gyro_sync.c` default
`GYRO_RATE_8_kHz`), PID dT'si ve TÜM filtreler 125 µs varsayımıyla kuruluyor;
lockstep'te gerçek döngü FDM adımı (0.0025 → 2500 µs, 20× sapma; 0.001 →
8× sapma). Yani filtreler yapılandırılanın ~1/20 kesim frekansında çalışıyor.

Düzeltme: `tools/betaflight_looptime_sync.patch` (env `KENET_SITL_LOOPTIME_US`
ile kapılı, unset iken bit-bit stok davranış; `gyro_init.c` içinde marker'lı
tek hunk). Runner bayrağı: `--sync-betaflight-looptime` (video/synthetic
checker pass-through dahil). Ölçülen A/B sonuçları:

- pitch1522 PID23 @0.0025: sync'siz FAIL (spread 233) → **sync ile PASS** —
  pitch kararsızlığının nedeni dT/filtre sapmasıydı.
- P22/yaw1504 @0.001 + sync: hâlâ FAIL (yaw-baskın split) — yaw@0.001
  kararsızlığı Betaflight dT'sinden değil.
- safe-yaw ve P23+safe-yaw penceresi @0.0025 + sync: PASS ama geçici 945 µs
  spread olayları görüldü (attitude bozulmadan toparlanıyor).
- **video yaw4 @0.0025 + sync: FAIL** (sync'siz PASS'ti). Kritik içgörü:
  bugüne kadarki 0.0025 kabul kararlılığı kısmen zamanlama hatasının yan
  etkisine yaslanıyordu — 20× fazla filtreleme etkin döngü kazancını düşürüp
  yapay sönümleme sağlıyordu. Gerçekçi zamanlamayla mevcut kazançlar + zayıf
  Gazebo rotor modeli her iki adımda da kararsız.
- Operasyonel sınır: sync @1000 µs'de Betaflight scheduler'ı FDM faz yarışına
  giriyor ve sim sürünüyor (RTF 0.001'e düştü); tasks.c'ye hızlı-deneme hack'i
  denendi ve GERİ ALINDI (Gazebo altında MSP'yi açlıktan öldürdü). Sync şimdilik
  yalnız ≥2 ms adımlarla kullanılır.
- Regresyon temiz: env unset iken yeniden derlenen binary sync'siz kabul
  hattında aynı sonuçları veriyor (safe-yaw @0.0025 PASS, spread 0).
- Harness sertleştirme (kalıcı, koşulsuz): runner artık virtual RC'den önce
  BOOTGRACE'in temizlenmesini bekliyor (`--arming-grace-timeout`, MSP tek
  istemci olduğu için diagnostics başlamadan önce koşulur); bu, uzun boot'ta
  ARM_SWITCH kilidiyle "sessizce hiç arm olmayan" koşuları bitirir.

Sonuç hükmü: kabul kapıları ŞİMDİLİK sync'siz 0.0025 kanıtıyla yürür (ölçülü,
tekrarlanabilir), ama bu profil "gerçekçi uçuş" kanıtı değil.

## 2026-07-02 Gece 2: Adım 1-2-3 Sonuçları (ölçüldü)

Adım 1 — Plant güçlendirme DENENDİ ve hüküm değişti. Üç mekanizma eklendi ve
ölçüldü (`tools/gazebo_plugin_velocity_control.patch` + launch/runner/video
knob'ları): (a) rotor hız-servo modu `--iris-velocity-control` +
`--iris-motor-time-constant` (dartsim'de JointForceCmd/JointVelocityCmd
çakışması bulunup düzeltildi: force bileşeni hız servosunu eziyordu ve rotor
hiç dönmüyordu), (b) `maxRotorVelocity` SDF parametresi, (c)
`--iris-rotor-damping` yaw otorite knobu. Ölçüm: hiçbiri sync dünyasının
kapılarını kurtarmıyor — velctrl(τ=20ms) safe-yaw penceresini kötüleştirdi
(rotor zaten τ≈0.3 ms ile neredeyse anlık; "yavaş aktüatör" hipotezi yanlış),
damping 0.001 çıplak P23'ü ve video yaw4'ü kurtarmadı. Kalan açıklama:
gerçekçi zamanlama yapay aşırı filtrelemeyi kaldırınca Betaflight
varsayılan-türevi kazançlar bu plant için fazla sıcak — çözüm plant cerrahisi
değil, SİM'E ÖZGÜ TUNE (aşağıda, açık iş).

Adım 2 — Lockstep kayıpsız uyandırma UYGULANDI:
`tools/betaflight_lockstep_sem.patch` (sitl.c, sayaçlı semafor; mutex
unlock'un kaybolan uyandırmaları yerine en fazla 2 bekleyen paket sayar).
Ölçüm: sync@0.0025 artık geçici 945 olayı olmadan temiz (safe-yaw 3/3 spread
0.000); sync'siz dünya bit-bit aynı davranıyor (regresyon PASS). sync@0.001
açlığı HÂLÂ açık (RTF ort. 0.56'ya çöküyor; sorun kayıp uyandırma değil,
scheduler'ın gyro-kilitli modunun 1 kHz sim-köle zamanlamayla etkileşimi) —
0.001 artık hiçbir kapı için gerekmediğinden bloklayıcı değil, açık konu.

Adım 3 — Sync dünyasının ilk braketi ÖLÇÜLDÜ (0.0025 + sync + semafor build,
tamamı sanal RC):

- Safe-yaw regression: `3/3 PASS`, spread `0.000`, roll/pitch `0.000`.
- P23/yaw1504 + safe-yaw penceresi: attitude PASS (roll 4.1) ama nudge anında
  geçici `945` spread olayı — marj ince, kabul penceresi olarak yazılmadı.
- Pitch PID23+throttle1500: `1522` ve `1530` PASS (spread 12/15 µs) — sync
  dünyası pitch'i 0.0025'te de kararlı (adım-bağımlılık çözüldü: pitch artık
  0.001'e mahkûm değil).
- Video neutral PASS; video target-loss PASS (kenet→pilot-target-lost→
  AI-ARMED zinciri doğru).
- Video yaw braketi: `yaw2 PASS / yaw3 FAIL` (sync'siz dünya: yaw5/yaw6).
  Sync dünyası daha dar ama GERÇEKÇİ pencere; genişletmek = sim-tune işi.

Harness sertleştirme (bu turda bulunan gerçek hatalar): arming-grace
beklemesi external RC moduna da genişletildi (video/mixer koşuları BOOTGRACE
kilidine denk gelip sessizce hiç arm olmadan "FAIL" yazabiliyordu — yaw3 ilk
koşusu böyle INVALID çıktı ve tekrarında gerçek ölçüm alındı).

Kalan açık işler (öncelik sırasıyla):

1. Sim'e özgü tune araştırması: sync dünyasında video yaw braketini yaw2'nin
   üstüne taşıyacak Betaflight profili (level/rate/PID kombinasyonu; roll PID
   ve level kazançlarını araçlara maruz bırakmak gerekebilir).
2. sync@1 kHz scheduler açlığı (gyro-kilitli scheduler'ın sim-köle modu).
3. Kabul profilini sync dünyasına taşıma kararı: safe-yaw + pitch + neutral +
   target-loss kapıları sync'te zaten yeşil; command-response kapıları tune
   işini bekliyor.

## 2026-07-03: Sim-Tune Bulundu, FPV Katmanı ve Populated World

### Açık iş 1 ÇÖZÜLDÜ: tüm-eksen sim-tune

Sync dünyasında (0.0025 + `--sync-betaflight-looptime`, semafor build) video
yaw braketini daraltan şey yaw ekseni değil, DEFAULT ROLL PID'iydi. Kanıt
zinciri (hepsi `test-2.mp4`, `--forward-limit 0 --kenet-delay-seconds 18`,
`--max-abs-delta N --min-abs-yaw-delta N`):

- T1 `--yaw-pid 19,0,0` (roll default 45/80/30): yaw3 FAIL — roll `180.0`,
  pitch `76.6`, MSP spread `945`. Yaw P'yi düşürmek kurtarmıyor; kırılan
  eksen roll.
- T2 `--yaw-pid 23,0,0 --pitch-pid 23,0,0` + `configs/sync-tune-allaxis-p23.txt`
  (config import: `p_roll=23 i_roll=0 d_roll=0`, roll/pitch `rc_rate=5
  srate=30`): yaw3 PASS — roll `0.7`, pitch `0.4`, irtifa `19.4 m`.

Tekrar zincirleri (aynı tune profili) üç ayrı rejim ölçtü:

- **yaw2: `5/5 PASS`, hepsi roll/pitch `0.000` VE MSP motor spread `0.000`** —
  tamamen lineer rejim, hiç diferansiyel yok. Herhangi bir dünyada ölçülmüş
  ilk tam-tekrarlanabilir video command-response kapısı (sync'siz dünyanın
  yaw5/yaw6 braketi tek koşuluk kanıttı, tekrarı hiç ölçülmemişti).
- yaw3–yaw5: metastabil satürasyon bandı, ~%50 — yaw3 `3/6`, yaw4 `4/6`,
  yaw5 `3/5`. PASS koşularında attitude temiz (`0.7/0.4`) ama TRACKING
  boyunca SÜREKLİ ~`777` MSP spread (bir motor `2000`'de satüre, diyagonal/
  yaw çifti deseni; değer koşudan koşuya `777-804` bandında deterministik).
  FAIL koşularında aynı rejim roll `180` flip'e tırmanıyor. `--yaw-pid
  19,0,0` + tune probe'u da (yaw4 PASS, spread `777`) aynı satürasyon
  rejiminde — yaw P'yi düşürmek rejimi değiştirmiyor.
- yaw6: FAIL (roll `180`, spread `945`).

Mekanizma özeti: yaw2→yaw3 geçişi plant'i otorite-tavanı rejimine sokuyor
(spread `0.000` → `777` fazı). Bandı genişletmenin ölçülmüş yolu Betaflight
tarafında değil; sıradaki şüpheli, ±2-3 µs'lik komutla tam ölçekli sürekli
yaw diferansiyeli talep ettiren plant yaw-otorite ölçeği (plugin/damping).
`set roll_rate_limit = 120` config'te sessizce reddediliyor (CLI min 200) —
MSP yolu 120'yi kabul ediyor; bilinçli bırakıldı çünkü roll/pitch stick'leri
bu kapılarda nötr.

Sonuç: gerçekçi zamanlama artık braketi daraltmıyor (tek koşu çözünürlüğünde
zarf sync'siz dünyayla aynı: yaw5'e kadar geçebiliyor, yaw6 FAIL) ve sync
dünyası ilk kez 5/5'lik bir command-response kabul noktasına sahip: yaw2.

### Açık iş 2: bounded-wait faz yarışını çözdü; geç durma sınıfı hâlâ açık

`lockMainPID` trylock'u env-kapılı sınırlı beklemeye çevrildi
(`KENET_SITL_LOCKSTEP_WAIT_US`, `sem_timedwait`; env yokken bit-bit stok
trylock — sync-OFF 0.0025 regresyonu yeni binary'de PASS, spread 0). Runner
bayrağı: `--lockstep-wait-us` (örn. step 0.001 için 4000).

Ölçüm (sync@0.001, safe-yaw profili, aynı binary, motor-UDP 45 s):

- Kontrol (trylock): RTF `0.82` → 5-10 s içinde `~0` çöküş, MSP `4/75`,
  hiç arm olmadı (armed_angle=0 ⇒ INVALID), irtifa `0`.
- Bounded-wait 4000: kalkış boyunca RTF `0.98–1.00` (5 sn'lik dilimler),
  arm + ANGLE + tırmanış (4.5 m), MSP `57/75`, runner verdict PASS —
  sonra ~t+20 s'de aynı durma sınıfı (FDM/motor-cevap karşılıklı bekleme;
  paketler ~0'a düşüyor).

Hüküm: scheduler faz-yarışı açlığı ÇÖZÜLDÜ (tasks.c hack'inin MSP açlığı
olmadan); kalan geç-durma, plugin tarafı lockstep beklemesinin kayıp-cevap
senaryosu sınıfında ve AYRI bir iş. 0.001'i hiçbir kapı gerektirmediği için
bloklamıyor. Sıradaki ipucu: BetaflightPlugin'in lockstep beklemesine
timeout+yeniden-gönderim veya sayaçlı eşleme eklemek.

### FPV katmanı (kullanıcı isteği): kamera + insanlı/araçlı dünya + elle uçuş

- `--iris-forward-camera` (run_gazebo_betaflight.sh): geçici model kopyasına
  ileri bakan kamera link+sensor enjeksiyonu; topic `/kenet/fpv_camera`
  640x480@30, kütle 1 g (plant'e etkisi ihmal), kabul koşuları kamerasız
  kalır. Headless doğrulama: topic yayında, karede insan/araç/ufuk görünür,
  RTF ~1.000.
- Yerel sahne modelleri (Fuel/internet bağımlılığı yok):
  `kenet_person`, `kenet_person_blue`, `kenet_car`, `kenet_car_white` +
  `betaloop_iris_betaflight_demo_populated.sdf` (5 insan + 3 araç önde +Y,
  1+1 arkada; world adı `betaloop_demo` korunur). Uçuş-eşdeğerlik ölçüldü:
  safe-yaw P23/yaw1504 kabul penceresi populated world'de PASS (irtifa
  `22.985`, roll/pitch `0.000`, spread `0.683`).
- `tools/sitl_keyboard_rc.py`: klavyeden sanal RC (UDP 9004, sitl_rc_bridge
  paket formatı bire bir; saf `KeyboardRcState` + 20 unit test; ok tuşu ESC
  dizileri quit sanılmaz; çıkışta disarm burst).
- `./launch-fpv-sim.sh`: Gazebo GUI (ImageDisplay FPV paneli,
  `tools/fpv_gui.config`) + Betaflight SITL (temp cwd, mode+safe-rate+PID23
  konfigi) + klavye RC (varsayılan) veya `--input joystick`.
- Video checker'a `--yaw-rc-rate/--yaw-rate/--yaw-rate-limit` ve
  `--betaflight-config-file` pass-through'ları eklendi (tarama bunlarla
  koşuldu).
- Kamera + zamanlama etkileşimi ÖLÇÜLDÜ: kamera render yükü sync'siz
  dünyayı uçuşta deviriyor (aynı sabah-PASS profili kamera açıkken 2/2
  roll-180 FAIL; kamerasız aynı harici-Gazebo akışı PASS 23.7 m/0.000).
  Sync+tune dünyasında kamera ile PASS (roll 1.0/pitch 0.5). Bu, sync'siz
  kararlılığın yapay zamanlamaya yaslandığı bulgusunun bağımsız bir
  doğrulaması. `launch-fpv-sim.sh` bu yüzden sync profilini kullanır
  (looptime export + tune eeprom import). Uçuş-içi kamera karesi kanıtı:
  temiz uçuşta ufuk düz, crash koşusunda görüntü ters — kamera gövdeye
  rijit bağlı ve dronla hareket ediyor.

## Kısa Sonuç

Virtual RC bu projenin birincil ve yeterli test yoludur; bütün geliştirme ve
kabul kapıları onunla koşulur ve Aşama 1–4 2026-07-02 itibarıyla ölçülmüş
durumda: Aşama 1–2 ve 4 PASS, Aşama 3 braketleri güncel adım bağımlılığıyla
birlikte ölçüldü. Şu an geçerli çalışma yolu safe-yaw + gecikmeli Kenet
command-response penceresidir. Hedef, bu pencereyi
`docs/sitl-flight-readiness-criteria.md` kriterleriyle (5/5 tekrar dahil)
kalıcı regression haline getirmek ve kalıcı çözüm olarak plugin rotor
PID'ini dt-normalize etmek; fiziksel kumanda yalnız kullanıcı isterse, en
sonda, aynı kriterlerle denenir.
