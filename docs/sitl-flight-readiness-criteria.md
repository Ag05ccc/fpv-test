# SITL Uçuşa Hazırlık ve Sağlık Kriterleri

Tarih: 2026-07-02

Amaç: "SITL ortamı doğru kuruldu ve sorunsuz uçuş gerçekleştirilebiliyor"
hükmünü ölçülebilir hale getirmek. Bu dokümandaki her kapı fiziksel kumanda
olmadan, sanal RC (UDP 9004) ve scriptli MSP ile koşulur. Fiziksel RC hiçbir
katmanın ön şartı değildir.

Eşiklerin tamamı bu depodaki ölçülmüş koşulardan gelir (2026-06-30 /
2026-07-01 log ailesi, `logs/sitl/`); tahmini eşik yoktur.

## Hüküm Tanımı

"SITL hazır" hükmü şu demektir: aşağıdaki Katman 0–5'in hepsi kendi ölçülebilir
eşikleriyle PASS ve Katman 4 tekrar kriteri sağlanmış. Tek bir PASS koşusu asla
yeterli değildir. Bu hüküm bir gerçek-uçuş tune kanıtı değildir: safe-yaw /
safe-manual profilleri SITL debug/acceptance profilidir.

## Koşu Geçerliliği (PASS/FAIL'den Önce)

Bir koşunun PASS veya FAIL sayılabilmesi için önce GEÇERLİ olması gerekir.
Ölçülmüş gerekçe: `flipfix-yaw1650-rev` ve `flipfix-yaw1650-fixed` koşuları hiç
arm olmadı (motorlar 1000'de kaldı) ve "split yok" görüntüsü sahte kanıttı;
`20260630-235335-bracket-pitch1515` ilk diagnostik örnekte roll `-180` ile
başladı (önceki koşudan devralınan durum) ve hiç arm olmadı.

Geçerlilik şartları:

- `armed_angle_samples > 0`. Arm olmayan koşu FAIL değil INVALID'dir; brakete
  kanıt olarak yazılamaz.
- İlk diagnostik örnek temiz: `msp.attitude` ve Gazebo pose için |roll| ve
  |pitch| < 5 derece, motorlar 1000, disarmed.
- `msp_connected_samples` kesintisiz (pencere boyunca MSP kopması yok).
- Koşu öncesi süreç/port hijyeni: 9002/9003/9004/5761 boş; bayat `gz sim` veya
  `betaflight_SITL.elf` süreci yok; repo kökünde pid/eeprom kalıntısı
  bırakılmaz.

## Katman 0 — Ortam Kurulumu

```bash
tools/check_sitl_env.sh
fpv_env/bin/python -m pytest -q
```

- `gz`, Betaflight plugin ve Betaflight SITL binary bulunur; port durumları
  raporlanır.
- Test paketi temiz.
- Betaflight temp cwd varsayılandır; repo-root `eeprom.bin` yalnız bilinçli
  deney için `--betaflight-cwd repo` ile kullanılır (Betaflight `eeprom.bin`'i
  bulunduğu çalışma dizininden yükler; `sitl.c` `EEPROM_FILENAME`).

## Katman 1 — Donanımsız Statik Kapılar

```bash
fpv_env/bin/python tools/sitl_pid_direction_check.py
fpv_env/bin/python tools/sitl_mixer_matrix_check.py
fpv_env/bin/python tools/sitl_configure_modes.py   # SITL açıkken
fpv_env/bin/python tools/sitl_mode_status_check.py # SITL açıkken
fpv_env/bin/python tools/kenet_msp_smoke.py --msp-tcp 127.0.0.1:5761
```

- PID yön sözleşmesi, mixer matrisi, mode range'leri, MSP okuma/yazma temiz.

## Katman 2 — Simülasyon Sağlık Metrikleri (Her Gazebo Koşusunda)

Bu metrikler uçuş sonucundan bağımsızdır; simülasyonun kendisinin sağlıklı
koştuğunu ölçer. Kaynak: diagnostics JSONL (`gazebo.rtf`) ve motor UDP JSONL
zaman damgaları.

- Real-time factor: min >= 0.99, max <= 1.01. (Ölçülen aralık: 0.9935–1.0013;
  pass ve fail koşularında aynı.)
- Motor UDP kadansı fizik adımıyla tutarlı: `--max-step-size 0.001` için
  ~1000 Hz (ölçülen 996–999 Hz), `0.0025` için 400 Hz. Ortalama paket arası
  süre adım süresine eşit, sapma küçük (ölçülen sd ~0.18–0.25 ms).
- Disarm öncesi pencerede > 30 ms'lik paket boşluğu yok. (Ölçülen: pass/fail
  koşularında aynı ofsetlerde deterministik ~13–27 ms harness boşlukları var;
  bunlar arızaya öncülük etmiyor. 2026-07-02 PASS koşusunda max 26.6 ms
  ölçüldü.)
- Fizik adımı (max_step_size) her kapıda AÇIKÇA sabitlenir ve kanıtla birlikte
  kaydedilir. 2026-07-02 ölçümü: eksen kararlılığı adıma bağlı ve iki yönde
  ters (0.0025: yaw kararlı / pitch nudge kararsız; 0.001: tersi). Adım
  değiştirerek "düzeltme" yapılmaz; farklı adım = farklı kabul kanıtı.
- Zamanlama gerçekçiliği notu (2026-07-02 gece): stok SITL'de Betaflight PID
  dT/filtreleri 8 kHz varsayar; gerçek lockstep döngüsü adım hızındadır (20×
  sapma @0.0025). `--sync-betaflight-looptime` +
  `tools/betaflight_looptime_sync.patch` bunu düzeltir ama dinamiği kökten
  değiştirir (pitch@0.0025 FAIL→PASS, video yaw4@0.0025 PASS→FAIL). Bu
  dokümandaki kabul kanıtları SYNC KAPALI dünyaya aittir; sync'li dünya için
  braketler yeniden ölçülmeden hiçbir eski kanıt taşınamaz. Sync şimdilik
  yalnız ≥2 ms adımlarla çalışır (deneysel).
- Lockstep sözleşmesi (kod garantisi, ölçümle uyumlu): FDM paketi başına en
  fazla 1 PID iterasyonu (`mainLoopLock` trylock) ve tam 1 motor paketi
  (`updateLock`). Kadans bozulursa önce Gazebo/plugin tarafına bakılır.

## Katman 3 — Uçuş Kalite Metrikleri

Standart kabul profili: `--max-step-size 0.001`, throttle 1750, `--hold-seconds
40`, temp cwd, `--capture-motor-udp`, `--debug-mode AC_ERROR`.

- İrtifa kazancı: pencereye duyarlıdır — diagnostik pencere süresi koşudan
  koşuya değişir (75 örnek ≈ 40–50 s, 120 örnek ≈ 70–77 s), tırmanış hızı ise
  sabittir (~0.6 m/s, throttle 1750). Bu yüzden irtifa kazancı MUTLAKA pencere
  süresiyle birlikte raporlanır. Ölçülen bantlar: 75-örnek pencerede 23.1–23.9
  m (2026-07-02, 5/5), 120-örnek video penceresinde 30.9–31.8 m. Runner eşiği
  (`--min-altitude-gain`, varsayılan 1.0) sınıra yakın profillerde (throttle
  1475) pencere hızına takılabilir; marj için throttle 1500 kullanılır.
- Max |roll| ve |pitch| <= 5 derece (neutral/nudge kabul koşularında; komut
  cevabı kapılarında ölçülmüş üst sınır neyse o).
- Raw motor spread: kararlı durumda <= 25 µs; PASS koşularında ölçülen tipik
  değer < 1–13 µs. `945 µs` spread satürasyon/flip imzasıdır ve tek başına
  FAIL demektir. Eksen bias eşikleri (25/100/200/400 µs) tetiklenmez.
- AC_ERROR telemetrisi (aktif uçuş penceresi): yaw setpoint/error ~1, gyro ~0,
  P/Sum ~0. Gyro/error'un yüzlere çıkması (ölçülen FAIL değerleri 610–1741)
  divergence imzasıdır.
- Değerlendirme araçları: `tools/sitl_pid_sweep_summary.py` (raw split zamanı,
  eksen bias, AC_ERROR özetleri) ve `tools/analyze_sitl_log.py --per-path`.

## Katman 4 — Tekrarlanabilirlik

- Kabul profili: art arda 5 geçerli koşu, 5/5 PASS. Bunun altındaki hiçbir şey
  "ortam sağlıklı ve uçuş sorunsuz" hükmü veremez. (İlk ölçülmüş 5/5:
  2026-07-02 safe-yaw regression, step 0.001.)
- Sınır noktaları: en az 5 geçerli koşu koşulur; %100 PASS çıkmıyorsa o nokta
  "ölçülmüş sınır" olarak etiketlenir, asla kabul profiline yazılmaz. Örnek:
  P22/yaw1504 step 0.0025'te `3 PASS / 1 FAIL` görünüyordu; step 0.001'de
  `5/5 FAIL` deterministik çıktı (2026-07-02) — eski "metastability" adım
  ayrıklaştırmasının ürünüydü. Sınır etiketi adımla birlikte yazılır.
- FAIL tekrar üretildiğinde raw split zamanı raporlanır ve deterministik
  pencerede kalması beklenir (örnek: P23/yaw1504 nudge sonrası 0.4–0.9 s).

## Katman 5 — Kenet Entegrasyon Kapıları

Ölçülmüş güncel sınırlarla (komutlar `docs/sitl-quickstart.md` ve
`docs/sitl-acceptance-procedure.md` içinde):

- Real video neutral kapısı PASS (delta 0/0).
- Command-response braketleri: yaw-only <= 5 PASS (yaw6 FAIL), pitch-only <= 5
  PASS (`--pitch-pid 23,0,0` ile; pitch6 FAIL), combined <= 3+3 PASS
  (yaw4+pitch4 FAIL). TRACKING takeoff/ramp bitmeden başlatılmaz
  (`--kenet-delay-seconds 18`).
- Target-loss passthrough: synthetic + real video kapıları PASS
  (`pilot-target-lost` ve `AI-ARMED` örnekleri görülür, final-pilot delta 0).

## Flip Bug Kapanış Kriteri

Flip incelemesi şu üçü birden sağlandığında SITL kapsamında kapanmış sayılır:

1. Mekanizma ölçümle gösterildi: arıza yaw ekseninde P >= 22 + küçük setpoint
   (yaw1504) ile AC_ERROR divergence olarak başlıyor; zamanlama/jitter
   dışlandı (RTF ~1.000, kadans metronomik ve pass/fail'de aynı, lockstep
   commit'i hiçbir FAIL'i PASS yapmadı, step 0.001'de P23/yaw1504 hâlâ FAIL).
   Fizik adımı sınırı kaydırabiliyor (0.0025→0.001 pitch braketini kurtardı)
   ama yaw P23 için yeterli değil: kök neden Gazebo iris aktüatör modelinin
   (P-only rotor hız döngüsü, `vel_p_gain=0.05`) Betaflight varsayılan yaw
   otoritesiyle uyumsuz kapalı çevrimi.
2. Ölçülmüş güvenli pencere (safe-yaw / safe-manual + command-response
   braketleri) Katman 4 tekrar kriteriyle regression olarak korunuyor.
3. Sınır dışı (provoke) koşuları acceptance loglarından ayrı etiketleniyor.

Bu kapanış SITL içindir; gerçek uçuş PID/rate tune'u ayrı bir çalışma olarak
planlanır ve bu dokümandaki profiller oraya kopyalanmaz.

## Sync Dünyasına Geçiş Kararı (2026-07-03)

Karar: kabul kapıları gerçekçi-zamanlama (sync) dünyasına KADEMELİ geçer.
Sync dünyası profili: step `0.0025` + `--sync-betaflight-looptime` + semafor
build + tüm-eksen sim-tune (`configs/sync-tune-allaxis-p23.txt` config
import'u + `--yaw-pid 23,0,0 --pitch-pid 23,0,0` + safe-yaw rates). Bir kapı
bu profille Katman 4 kriterini (5/5) sağladığında o kapının kanıt tabanı
sync dünyasına döner; o zamana kadar ilgili kapı için sync-OFF `0.0025`
kanıtı geçerli kalır. Sync-OFF ve sync-ON kanıtları asla karıştırılmaz;
her kanıt satırı hangi dünyada ölçüldüğünü yazar.

Ölçülmüş durum (2026-07-03):

- Video yaw command-response, sync + tüm-eksen tune: **yaw2 `5/5 PASS`,
  hepsi roll/pitch `0.000` ve motor spread `0.000`** — bu kapı SYNC
  DÜNYASINA GEÇTİ ve kabul noktası `--yaw-limit 2`'dir. (Herhangi bir
  dünyada 5/5 ölçülmüş ilk command-response kapısı; sync-OFF yaw5/yaw6
  braketi tek koşuluk kanıttı.)
- yaw3–yaw5 metastabil satürasyon bandı (~%50: `3/6`, `4/6`, `3/5`;
  TRACKING boyunca sürekli ~`777` spread, bir motor satüre), yaw6 FAIL —
  "ölçülmüş sınır" etiketiyle kayıtlı, kabul profiline yazılamaz.
- Safe-yaw regression, pitch1522/1530, video neutral ve target-loss sync
  dünyasında yeşil ama henüz 5/5 zincirleri koşulmadı — geçişleri kendi 5/5
  zincirlerini bekler.

## Otomasyon Durumu ve Sonraki Adımlar

- `tools/sitl_readiness_report.py` bugün Katman 0/1/5 kanıtlarını topluyor.
- 2026-07-04: `tools/sitl_run_quality_check.py` eklendi (roadmap-v2 A0):
  koşu geçerliliği (INVALID: armed=0, kirli ilk örnek, MSP kopması), Katman 2
  sağlığı (RTF bandı, motor UDP kadansı ve boşluk, adım sabitleme
  `--expected-step`), pencere-normalize tırmanış hızı, disarm hükmü
  (rapor-modunda; `--require-disarm` ile kapı) ve işaretli yaw tepkisi
  (`--expect-yaw-sign` verilmeden ölçüm, verilince kapı) tek araçta.
  Birden çok `--diagnostics` dosyası zincir (5/5) hükmü üretir; çıkış kodu
  0=PASS, 1=FAIL, 2=INVALID. Altın örneklerle doğrulandı: 2026-07-03
  kamera-açık koşusu PASS, `20260630-235335-bracket-pitch1515` INVALID
  (armed=0 + roll −180 ilk örnek), `flipfix-yaw1650` FAIL (tırmanış 0.010
  m/s).
- Kalan otomasyon: runner diagnostik penceresinin disarm fazını kapsaması
  (sonra `--require-disarm` varsayılan olur) ve `sitl_readiness_report.py`nin
  donmuş kanıt yolları yerine bu araca delege etmesi.
