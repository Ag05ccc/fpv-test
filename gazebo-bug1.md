# Gazebo Bug 1 - Iris Takeoff Flip / Ani Takla

Tarih: 2026-06-23

Bu not Claude'un da inceleyebilmesi için yazıldı. Amaç, Gazebo Iris world'de
ARM + takeoff sonrasında görülen ani takla problemini Kenet kontrolünden ayırıp
Betaflight/Gazebo fizik hattında sistematik olarak izole etmek.

## Kısa Özet

Gözlenen problem: Iris model Gazebo'da arm oluyor, throttle verilince kısa süre
sonra aniden takla atıyor.

2026-06-30 son durum: Virtual RC hattında ilk büyük kök neden doğrulandı. Gazebo
`BetaflightPlugin.cc` yaw gyro `z` işareti Betaflight SITL'in kendi `z`
negation'ı ile pozitif feedback yaratıyordu. Plugin patch + rebuild sonrası
tek komutluk virtual takeoff runner PASS verdi. Ardından fiziksel joystick
olmadan Kenet mixer `--pilot-source virtual` ile external RC kabul kapısından
da PASS alındı. Sonraki direct yaw 1504 nudge testlerinde ise normal yaw PID
ile flip geri geldi; raw motor UDP capture ilk ayrışmanın saf yaw cw/ccw motor
pair bias olduğunu gösterdi. Yaw PID `0/0/0` yapılınca aynı yaw 1504 virtual RC
koşusu PASS verdi. Güncel aday Betaflight yaw PID/rate feedback zinciri.

Şu ana kadarki en önemli bulgular:

- Takla anında Kenet `IDLE` durumunda. Yani Kenet target tracking, PID veya
  pitch/yaw override aktif değil.
- Fiziksel Tango/USB joystick izole edilip virtual RC ile aynı takla tekrar
  üretildi. Bu nedenle fiziksel kumanda bu bug için ana kök neden değil.
- Gazebo plugin'e doğrudan motor pulse gönderildiğinde motor->moment işaretleri
  simetrik görünüyor; kaba motor moment yönü tek başına bariz ters değil.
- Son birleşik virtual RC koşusunda Betaflight raw UDP motor çıkışı da 2000'e
  kadar çıktı ve Gazebo pose fiziksel olarak roll `-180` durumuna geçti.
- Tek komutluk baseline FAIL:
  `logs/sitl/20260629-221238-takeoff-diagnostics.jsonl`.
- Yaw gyro sign patch sonrası PASS:
  `logs/sitl/20260629-221545-takeoff-diagnostics.jsonl`.
- Kenet mixer sanal pilot + external checker PASS:
  `logs/sitl/20260629-223523-external-kenet-takeoff-diagnostics.jsonl` ve
  `logs/sitl/20260629-223523-kenet-mixer-virtual-external-takeoff.jsonl`.
- Direct yaw 1504 raw UDP FAIL:
  `logs/sitl/20260630-closedloop-yaw1504-off-motor-udp.jsonl` ve
  `logs/sitl/20260630-closedloop-yaw1504-on-motor-udp.jsonl`;
  ilk büyük spread `yaw_cw_minus_ccw ~= 405-409 us`.
- Direct yaw 1504 + zero yaw PID PASS:
  `logs/sitl/20260630-yaw1504-zero-yaw-pid-diagnostics.jsonl`,
  `logs/sitl/20260630-yaw1504-zero-yaw-pid-motor-udp.jsonl`,
  `logs/sitl/20260630-yaw1504-zero-yaw-pid-virtual-rc.jsonl`.
- İlk yaw P/I sweep:
  - P-only `10,0,0`, `15,0,0`, `17,0,0`, `18,0,0`, `19,0,0` PASS.
  - İlk P-only `20,0,0` koşusu FAIL, ancak iki repeat PASS; bu nokta stabil
    fail değil, sınır/flaky bölge.
  - P-only `25,0,0` iki koşuda da FAIL; `45,0,0` FAIL.
  - I-only `0,1,0` PASS; `0,2,0`, `0,5,0`, `0,20,0`, `0,80,0` FAIL.
  - Geçici eşik güncellendi: ilk repo-root cwd sweep'i `P20/P22` sınır/flaky
    gibi gösterdi; runner temp-cwd olduktan sonra yaw1504 için `P21` iki PASS,
    `P22` üç PASS / bir FAIL, `P23` iki FAIL verdi. Aynı `P23` yaw1502 ve
    yaw1503 hold40 koşularında PASS verdi. Temiz P-only braket artık yaw1504
    için `P21` güvenli taraf, `P22` sınır/flaky, `P23+` stabil FAIL; P23 için
    nudge eşiği yaw1503/yaw1504 arasında. Eski P20/P22 oynaklığı için kalıcı
    `eeprom.bin`/FC state sızıntısı güçlü aday. I-only
    tarafı `1 < I < 2`.
  - `tools/sitl_pid_sweep_summary.py` eklendi; aktif uçuş fazında P20 raw split
    nudge'dan `2.943 s`, attitude eşiği `4.621 s` sonra; I2 raw split
    `17.518 s`, attitude eşiği `19.157 s` sonra; I5 raw split `5.051 s`,
    attitude eşiği `6.070 s` sonra görünüyor.
- P-only repeat ve yeni braket:
  - `P19` iki koşuda da PASS: raw spread `0.760` ve `0.730`, attitude sakin.
  - `P20` üç koşuda `1 FAIL / 2 PASS`: ilk koşuda raw split `2.943 s`,
    active attitude `4.621 s`; repeat1/repeat2 raw spread `~0.77`, attitude
    sakin. Yorum: `P20` güvenilir fail değil, başlangıç/oturum hassasiyeti olan
    sınır bölge.
  - `P20 hold40` PASS: `logs/sitl/20260630-yaw1504-yawpid-20-0-0-hold40-*`.
    Raw spread `0.795`, max attitude `0.9/1.6`, altitude gain `31.556`.
    Bu, uzun hold süresinin tek başına P20'yi kırmadığını gösteriyor.
  - `P21` temp-cwd runner ile iki koşuda PASS: raw spread `0.839/0.821`, max
    attitude `4.6/0.5` ve `0.2/0.3`. Bu nokta temiz başlangıçta güvenli tarafta.
  - `P22` repo-root cwd koşularında bir PASS/bir FAIL: fail raw split `2.759 s`,
    active attitude `7.532 s`, ilk büyük axis `yaw_cw_minus_ccw ~= -382 us`.
    Temp-cwd runner sonrası `3 PASS / 1 FAIL`: PASS raw spread
    `0.836/0.847/0.846`, son PASS
    `logs/sitl/20260630-yaw1504-yawpid-22-0-0-tempcwd4-*`; FAIL raw split
    `2.832 s`, active attitude `8.200 s`,
    ilk büyük axis
    `yaw_cw_minus_ccw ~= -381 us`. Yorum: P22 temiz cwd'de de sınır/flaky.
  - `P23` repo-root cwd'de iki koşuda da FAIL; temp-cwd'de de iki FAIL:
    temp-cwd raw split `2.721/2.396 s`, active attitude `8.726/5.155 s`, ilk
    büyük axis `yaw_cw_minus_ccw ~= -386..-393 us`. Güncel temiz-cwd yaw1504
    üst braket `P=23`.
  - `P23 yaw1502 hold40` temp-cwd PASS:
    `logs/sitl/20260630-yaw1502-yawpid-23-0-0-hold40-*`. Raw spread `0.426`,
    max attitude `0.2/0.4`, altitude gain `31.611`. Aynı P değeri yaw1504'te
    iki FAIL iken yaw1502'de uzun hold'da sakin kaldı; bu P-only kırılımının
    setpoint/nudge genliğine bağlı olduğunu güçlendiriyor.
  - `P23 yaw1503 hold40` temp-cwd PASS:
    `logs/sitl/20260630-yaw1503-yawpid-23-0-0-hold40-*`. Raw spread `0.645`,
    max attitude `2.5/1.5`, altitude gain `31.651`. Yaw1502 ve yaw1503 sakin,
    yaw1504 iki koşuda FAIL olduğundan P23 için pratik nudge eşiği
    `1503/1504` arasında görünüyor.
  - `P25` iki koşuda da FAIL: raw split `2.396-2.402 s`, active attitude
    `6.831-7.300 s`, ilk büyük axis `yaw_cw_minus_ccw ~= -397..-402 us`.
- Analyzer güncellemesi:
  - `tools/sitl_pid_sweep_summary.py` artık nudge'a en yakın, nudge öncesi ve
    nudge sonrası diagnostic sample'ları da raporluyor (`nudge_z`, `before_z`,
    `after_z`, attitude/yaw).
  - P20/P22 PASS ve FAIL koşularında nudge anı roll/pitch/yaw temiz ve benzer:
    roll/pitch `0/0`, pose yaw `90 deg`, after-nudge altitude yaklaşık `6.06-6.27 m`.
    Yani P20/P22 flakiness'i nudge anında bariz farklı attitude/yaw veya irtifa
    ile açıklanmıyor.
- Debug görünürlüğü:
  - `tools/sitl_diagnostics.py` ve `tools/sitl_dashboard.py` artık
    `MSP_ADVANCED_CONFIG` + `MSP_DEBUG` okuyor. Yeni diagnostic/dashboard
    sample'larında `msp.advanced_config.debug_mode`, `msp.debug_mode` ve
    `msp.debug` alanları var.
  - `tools/sitl_debug_config.py` artık `debug_mode` ayarını MSP üzerinden
    yapıyor. `tools/sitl_virtual_takeoff_check.py --debug-mode ...` runner
    Betaflight'i yönetiyorsa ayarı kaydedip aynı temp-cwd'den restart ediyor;
    çünkü Betaflight runtime `debugMode` init'te yükleniyor. Sıradaki kapı
    fiziksel RC değil; `PIDLOOP` / `ANGLERATE` / `ANGLE_TARGET` debug
    koşularından gerçek iç sinyal alınıp alınamadığını doğrulamak.
  - Bu doğrulama için `tools/sitl_debug_calibration.py` eklendi. Standalone
    `PIDLOOP` yaw1700/yaw1504 nonzero verdi fakat Betaflight kaynakta bu mod
    yaw PID değil loop timing. `ANGLERATE` yaw1700 sıfır kaldı. `ANGLE_TARGET`
    Gazebo P23/yaw1504 koşusunda `debug[3]` sinyal verdi ama yaw setpoint slotu
    `debug[2]` sıfır kaldı.
  - Doğrudan Betaflight instrumentation için `tools/betaflight_yaw_debug_patch.py`
    eklendi ve `/home/gz/betaflight/src/main/flight/pid.c` içine marker'lı blok
    uygulandı. `DEBUG_AC_ERROR` artık geçici olarak yaw setpoint, gyroRate,
    errorRate, P/I/F/S/Sum değerlerini `MSP_DEBUG[0..7]` içine koyuyor.
- I2 repeat ve erken axis-bias analizi:
  - `tools/sitl_pid_sweep_summary.py` artık raw motor axis bias için
    25/50/100/200/400us ilk geçiş zamanlarını ve aktif uçuş attitude eşiğini
    ayrıca raporluyor.
  - `0,2,0-repeat1` aynı unified runner ile tekrar FAIL verdi:
    `logs/sitl/20260630-yaw1504-yawpid-0-2-0-repeat1-diagnostics.jsonl`,
    `logs/sitl/20260630-yaw1504-yawpid-0-2-0-repeat1-motor-udp.jsonl`,
    `logs/sitl/20260630-yaw1504-yawpid-0-2-0-repeat1-virtual-rc.jsonl`.
  - I2 iki koşuda da benzer zaman sırasına sahip: 25us axis drift nudge'dan
    yaklaşık `16.6-16.7 s` sonra, raw split yaklaşık `17.35-17.52 s` sonra,
    attitude eşiği yaklaşık `19.16-19.19 s` sonra geliyor. Bu, I2 FAIL'in tekil
    log şansı değil, geç gelişen integrator/rate-feedback bozulumuna benzediğini
    güçlendiriyor.
- I2 nudge ölçekleme:
  - `yaw 1502` + `I=2` + 30s hold PASS:
    `logs/sitl/20260630-yaw1502-yawpid-0-2-0-diagnostics.jsonl`,
    `logs/sitl/20260630-yaw1502-yawpid-0-2-0-motor-udp.jsonl`,
    `logs/sitl/20260630-yaw1502-yawpid-0-2-0-virtual-rc.jsonl`.
    Raw max spread `0.765`, attitude max `0.5/0.2`.
  - `yaw 1503` + 30s hold FAIL ama attitude eşiği aktif uçuşta değil, disarm
    sonrası görülüyor:
    `logs/sitl/20260630-yaw1503-yawpid-0-2-0-*`.
    Raw split nudge'dan `21.656 s` sonra, aktif max roll/pitch `8.2/29.7`.
  - `yaw 1503` + 40s hold FAIL ve aktif uçuşta da attitude eşiğini geçiyor:
    `logs/sitl/20260630-yaw1503-yawpid-0-2-0-hold40-*`.
    Raw split `21.781 s`, active attitude threshold `28.257 s`.
  - Güncel eşik yorumu: bu setup'ta `I=2` için yaw `1502` sakin, yaw `1503`
    yeterli hold verilince aktif uçuşta kırılıyor; genlik azalınca bozulum
    kaybolmak yerine gecikiyor.

Sonraki odak:

1. P-only tarafında temiz-cwd braket yaw1504 için `P21` PASS, `P22`
   sınır/flaky, `P23+` stabil FAIL; yaw1502 ve yaw1503'te `P23` hold40 PASS.
   Instrumented `DEBUG_AC_ERROR` karşılaştırması artık tamamlandı: P21 aktif
   debug sakin kalırken P23'te yaw gyro/error/P/Sum runaway oluyor; P22 ise bir
   tekrar PASS, bir tekrar FAIL ile bu iki rejim arasında metastable kaldı.
   Yaw gyro scale, rotor velocity P gain ve physics step tek başına çözmedi.
   Ana iş artık P23/P22-FAIL için Betaflight yaw P/rate authority veya
   airframe'e özel güvenli yaw limit kapısı üretmek.
2. I2 tarafında gerekirse `yaw 1502` uzun hold veya `yaw 1503` repeat ile
   sınırın deterministikliğini pekiştir.
3. Plugin patch'in regression runner ile korunması.
4. Fiziksel Tango/joystick RC yolunu ancak yaw feedback kapısı netleşince aynı
   kabul kriterleriyle geri eklemek.
5. Kenet mixer + Gazebo TRACKING davranışını ayrı test etmek.

### Codex kontrolü - Claude güncellemeleri (2026-06-30)

Claude tarafındaki yeni durum kontrol edildi:

- `/home/gz/aeroloop_gazebo/models/betaloop_iris_with_standoffs/model.sdf`
  kaynak seviyesi artık identity rotor listesine çekilmiş görünüyor. Bu,
  bizim corrected BF SITL remap yorumumuzla uyumlu: Betaflight SITL kendi
  logical motorlarını UDP packet slotlarına zaten `3,0,1,2` olarak remap ediyor;
  plugin rotor listesi packet slotu -> fiziksel rotor identity kalmalı.
- `/home/gz/aeroloop_gazebo/plugins/BetaflightPlugin.cc` içinde yaw gyro için
  `pkt.imuAngularVelocityRPY[2] = angularVel.Z();` yorumu eklenmiş. Bu da son
  FDM/yaw sign hükmümüzle uyumlu: plugin tekrar ters çevirmemeli, çünkü
  Betaflight SITL bu packet alanını kendi içinde negate ediyor.
- Yeni `logs/sitl/flipfix-*` logları incelendi. `flipfix-baseline-diag.jsonl`
  neutral throttle koşusunda PASS veriyor; yaw komutlu `flipfix-yaw*`,
  `flipfix-lockstep*` ve `flipfix-cda001*` koşuları çoğunlukla FAIL. Bazı
  `fixed/rev` loglarında ARM+ANGLE örneği yok veya araç zaten roll `180`
  durumunda göründüğü için bunlar acceptance kanıtı sayılmamalı.
- `flipfix-yawP15-diag.jsonl` FAIL görünse de bu unified
  `sitl_virtual_takeoff_check.py --yaw-pid 15,0,0 --capture-motor-udp` koşusuyla
  aynı kapı değil; metadata RC kaynağı/parçalı diagnostics akışı kullanıldığını
  gösteriyor. Bu yüzden mevcut P15 PASS hükmü için esas kanıt hâlâ
  `logs/sitl/20260630-yaw1504-yawpid-15-0-0-*` üçlüsüdür.

Güncel karar değişmedi: Claude'un kaynak SDF/plugin yönüyle yaptığı güncelleme
bizim motor-map ve yaw-gyro-sign düzeltmelerimizle uyumlu; yeni flipfix logları
ise PID/rate feedback araştırmasını kapatmıyor. P-only ve I-only sınırları artık
yeterince daraltıldı. Ek Codex debug denemelerinde `debug_mode` doğru
kaydedildi/restart edildi. Son kalibrasyonla MSP_DEBUG'in tamamen kırık
olmadığı, ama seçilen public debug modlarının yaw setpoint/rate/PID iç sinyalini
güvenilir açıklamadığı görüldü. Bu yüzden daha fazla kör `MSP_DEBUG` koşusu
yerine instrumented AC_ERROR ile P23 FAIL ve P21 PASS karşılaştırması alındı.
P21/yaw1504 instrumented koşusu PASS: altitude gain `31.686 m`, max roll/pitch
`0.8/0.6`, raw spread `0.802`, aktif debug max `setpoint=1`, `gyro=0`,
`error=1`, `P/Sum=0`. P23/yaw1504 instrumented koşusu FAIL: raw split
`0.399 s`, ilk nonzero yaw debug `0.688 s`, aktif debug max `setpoint=1`,
`gyro/error=610`, `P/Sum=450`, raw spread `945`.
P22/yaw1504 instrumented iki yüzü de gösterdi: ilk koşu FAIL
(`logs/sitl/20260630-debug-acerror-instrumented-yaw1504-p22-*`), raw split
`2.776 s`, attitude `7.668 s`, aktif debug max `gyro/error=1740/1741`,
`P/Sum=1243`; repeat PASS
(`logs/sitl/20260630-debug-acerror-instrumented-yaw1504-p22-repeat1-*`),
altitude gain `31.654 m`, max roll/pitch `4.4/3.7`, raw spread `0.843`,
aktif debug `gyro=0`, `error=1`, `P/Sum=0`.

P23 runaway için üç dar fix hipotezi denendi fakat hiçbiri tek başına kabul
kapısını açmadı:

- `--iris-yaw-gyro-scale 0.5` ve `0.25`: aktif yaw P/Sum azaldı
  (`166` ve `102`) ama fail pitch/attitude tarafına kaydı.
- `--iris-rotor-vel-p-gain 0.01`: eski motor velocity P gain'e yakın değer
  split'i geciktirdi fakat P23 hâlâ FAIL.
- `--max-step-size 0.001`: daha ince Gazebo step de P23 runaway'i kapatmadı.

Bu sonuçlar tek başına Gazebo yaw gyro scale, rotor joint velocity P gain veya
physics step düzeltmesinin yeterli olmadığını gösteriyor. Sıradaki kapı
Betaflight yaw P/rate authority veya airframe'e özel güvenli yaw limit.

Ek kayıt: `CLAUDE.md` artık eski socat/MSP ağırlıklı notun yanında güncel
virtual RC/Gazebo runner'ını, fiziksel RC'nin şimdilik izole edildiğini ve P/I
sweep sınırlarını özetliyor. Son doğrulama `./fpv_env/bin/python -m pytest -q`
ile `178 passed`; bu repo ve `/home/gz/aeroloop_gazebo` için `git diff --check`
temiz.

Takip kontrolü: Claude güncellemeleri yeniden okundu ve mevcut hüküm değişmedi.
SDF identity rotor listesi ile plugin yaw gyro `Z` as-is notu doğru yönde.
P21 instrumented koşusunda görülen MSP-ready timeout için runner'a Betaflight
start/debug restart sonrası kısa sıfır-motor bootstrap'i eklendi; amaç
`ENABLE_SIMULATOR_GYROPID_SYNC` açıkken Gazebo plugin/BF handshake'ini MSP
beklemesinden önce canlandırmak. İlk retry'da restart sonrası süreç çok erken
kapandığı için ayrıca `--betaflight-restart-settle-seconds` eklendi. Varsayılan
2 saniye bekleme ile aynı P21/yaw1504 `DEBUG_AC_ERROR` komutu PASS verdi.

## Kanıtlar

İncelenen loglar:

- `logs/sitl/20260623-221633-dashboard.jsonl`
- `logs/sitl/20260623-233630-kenet-mixer.jsonl`

Analyzer özeti:

```text
Dashboard / Betaflight
  attitude samples: 69
  motor samples: 69
  armed True: 5
  max roll: 180.0 deg
  max motor spread: 2000

Kenet Mixer
  states:
    IDLE: 1225
  sources:
    pilot: 1225
  max final-pilot deltas:
    all channels: 0
```

Takla civarındaki kritik snapshot:

```text
time: 2026-06-23T23:36:40+0300
kenet: IDLE
source: pilot
pilot roll/pitch: 1500 / 1500
pilot throttle: ~1375
pilot yaw: ~1549
CH5/AUX1 ARM: 2000
CH6/AUX2 Kenet: 1000
CH7/AUX3 Mode: 1000 / LOW
flight flags: 0x00000001 / ARMED only
attitude: roll 180.0 deg, pitch 6.9 deg
motors: 1376, 1481, 1055, 2000
```

Yorum:

- Kenet `TRACKING` değil, `IDLE`.
- Kenet pitch/yaw üretmiyor.
- Final RC ile pilot RC arasında fark yok.
- CH7/AUX3 `LOW`, yani ANGLE/HORIZON gibi stabilized mode aktif görünmüyor.
- Motor spread çok büyük; bu, FC'nin takla veya hızlı attitude sapmasına cevap
  vermeye çalıştığını gösteriyor.

## 2026-06-29 Codex Güncellemesi

Virtual RC ve otomatik probe hattı eklendi. Bu sayede fiziksel kumanda olmadan
aynı takeoff script'i tekrar tekrar koşulabiliyor.

Yeni araçlar:

```bash
python tools/sitl_virtual_rc.py --script takeoff --throttle 1750 --mode-pwm 1500 --send
python tools/sitl_configure_modes.py
python tools/sitl_diagnostics.py --rc-source virtual --direct-msp
python tools/sitl_motor_udp_probe.py --duration 30
python tools/gazebo_motor_moment_probe.py
```

Önemli yeni loglar:

```text
Virtual RC standalone RC doğrulama:
  logs/sitl/20260629-211401-diagnostics.jsonl
  sonuç: expected RC -> MSP_RC delta 0

Virtual RC ile ilk Gazebo flip:
  logs/sitl/20260629-212308-diagnostics.jsonl
  sonuç: roll -180; physical joystick elendi

Direct Gazebo motor moment:
  logs/sitl/20260629-213502-motor-moment.jsonl
  sonuç: tek motor pulse'larında roll/pitch/yaw işaretleri simetrik

Direct high thrust:
  logs/sitl/20260629-214237-motor-moment.jsonl
  sonuç: all_equal 0.95 motor speed z'yi +0.061 m yükseltti

Birleşik virtual RC + diagnostics + raw UDP:
  diagnostics: logs/sitl/20260629-215325-diagnostics.jsonl
  raw UDP:     logs/sitl/20260629-215315-motor-udp.jsonl
  sonuç: roll 180, pitch 68; raw motor UDP 2000'e kadar çıkıyor
```

Birleşik koşudan kritik sıra:

```text
sample 41:
  RC throttle 1634, motors [1634,1634,1634,1634], attitude roll=0 pitch=0
sample 42:
  RC throttle 1714, motors [2000,1055,1383,1708], attitude roll=5 pitch=5.5
sample 46:
  RC throttle 1750, motors [1275,1055,1265,2000], attitude roll=115.5 pitch=68
sample 52:
  RC throttle 1750, motors [1474,1055,1683,2000], attitude roll=-180 pitch=0
after flip Gazebo pose:
  roll=-180.0, pitch~0, z=0.035
```

Yorum:

- Virtual RC ile `MSP_RC` doğru; fiziksel kumanda/joystick mapping bu bug'ın
  ana nedeni değil.
- Runtime `--fix-iris-motor-map` aktifken de flip tekrarlandığı için dış SDF'deki
  statik çift-remap hatası tek başına yeterli açıklama değil.
- Direct motor moment probe kaba fiziksel motor moment yönlerinin tutarlı
  olduğunu gösterdi.
- Geriye en güçlü adaylar: BetaflightPlugin'in IMU/body/yaw frame dönüşümü,
  Betaflight mixer'in gördüğü attitude işareti ile Gazebo gerçek pose arasındaki
  kapalı çevrim uyumsuzluğu, veya runaway korumasını tetikleyen attitude
  zinciri.

Tek komutluk A/B runner:

```bash
python tools/sitl_virtual_takeoff_check.py \
  --throttle 1750 --hold-seconds 8 \
  --diagnostic-samples 75 --diagnostic-interval 0.1
```

Baseline, gyro patch öncesi:

```text
log: logs/sitl/20260629-221238-takeoff-diagnostics.jsonl
result: FAIL
max roll=-180.0, max pitch=-73.6, max motor spread=945
altitude gain=0.930 m
ilk sert ayrışma:
  sample 22: motors=[2000,1244,1234,1990], roll=0.1 pitch=0.0
  Gazebo pose yaw hızla dönerken diagonal motor çifti yaw düzeltmesi gibi ayrışıyor.
```

Düzeltme:

```text
../aeroloop_gazebo/plugins/BetaflightPlugin.cc
pkt.imuAngularVelocityRPY[2] = -angularVel.Z();
```

Yorum:

- `MSP_ATTITUDE` genel roll/pitch pose'u takip ediyor; quaternion attitude kaba
  olarak tamamen ters değil.
- İlk büyük motor spread roll/pitch büyümeden başlıyor ve diagonal motor çifti
  ayrışıyor; bu yaw rate feedback işaretine işaret ediyor.
- Betaflight SITL `sitl.c` paket yaw gyro değerini `virtualGyroSet()` öncesinde
  negatiflediği için plugin'in ham Gazebo `angularVel.Z()` göndermesi yaw
  ekseninde pozitif feedback yaratıyor.

Plugin rebuild sonrası:

```text
log: logs/sitl/20260629-221545-takeoff-diagnostics.jsonl
result: PASS
altitude gain=6.624 m
max roll/pitch=0.0/0.0
max motor spread=0
```

Yeni karar:

1. Fiziksel RC ana neden değil.
2. Runtime motor map overlay gerekli ama tek başına yeterli değildi.
3. Virtual RC takeoff için kök neden BetaflightPlugin yaw gyro sign hatası
   olarak doğrulandı.
4. Sıradaki iş fiziksel Tango/joystick yolunu aynı runner kriterleriyle geri
   eklemek ve gerçek hedef bulunan Kenet TRACKING davranışını Gazebo üzerinde
   ayrıca test etmek.
5. Bu oturumda `/dev/input/js*` cihazı görünmedi; canlı fiziksel RC retest
   koşturulamadı. Bunun yerine aynı kabul kapısı external RC gözlem moduna
   genişletildi.
6. External RC gözlem modu Kenet mixer'ın sanal pilot kaynağıyla doğrulandı;
   hedef yokken `TRACKING` durumunda bile final RC pilot passthrough kaldı.
7. Sentetik target-found harness eklendi; kamera/video olmadan `source=kenet`
   ve pitch/yaw override yolu ölçülebiliyor.
8. Tek komutluk synthetic target runner eklendi; centered target-found profil
   PASS verdi ve `source=kenet/target_found=True` iken final-pilot delta `0`
   kaldı.
9. Sentetik target-found offset acceptance henüz FAIL: küçük pitch/yaw offsetleri
   takeoff sonrası motor spread `945 us` ve flip üretiyor.
10. Micro/P-only synthetic profiller ve Kenet'siz direct virtual RC nudge eklendi.
    Direct yaw 1504 de Kenet olmadan FAIL; bu yüzden sıradaki aday Kenet değil,
    motor yaw torque/direction veya model/IMU eksen işareti.

## En Olası Sebep

Kesinleşmiş sebep:

```text
Takla Kenet PID/target override kaynaklı değil.
Virtual RC hattındaki takla BetaflightPlugin yaw gyro sign hatasından kaynaklandı.
```

Henüz kesinleşmeyen ama en olası kök neden grupları:

1. **Flight mode eksikliği**
   - CH7/AUX3 logda `LOW`.
   - Betaflight flight flags sadece `ARMED`.
   - ANGLE mode aktif değilse ilk takeoff testinde self-level davranışı
     beklenmemeli.
   - Tek başına ACRO mode, merkez roll/pitch ile normalde anında takla
     atmamalı; ama motor/IMU mapping hatasını daha kolay görünür yapar.

2. **Motor order / motor direction uyuşmazlığı**
   - `betaloop_iris_with_standoffs/model.sdf` içinde Betaflight motorları özel
     rotor joint mapping ile bağlanmış.
   - Betaflight QUADX sırası:
     - BF[0] = REAR_R
     - BF[1] = FRONT_R
     - BF[2] = REAR_L
     - BF[3] = FRONT_L
   - SDF yorumuna göre mapping bu sırayı hedefliyor, fakat fiziksel moment
     işareti ve `turningDirection` hâlâ testle doğrulanmalı.

3. **IMU frame / attitude dönüşümü uyuşmazlığı**
   - `BetaflightPlugin.cc` Gazebo pose/orientation bilgisini Betaflight NED
     frame'ine dönüştürüyor.
   - Eğer orientation işareti veya eksen dönüşümü ters ise FC doğru motorlara
     yanlış düzeltme uygular ve takla çok hızlı oluşur.

## Uygulanan İlk Çözüm / Teşhis Kolaylığı

Tango 2 üzerindeki ikinci 3-position switch henüz güvenilir şekilde CH7/AUX3
üretmediği için yazılımdan CH7 sabitleme opsiyonu eklendi.

Eklenen CLI:

```bash
--force-mode-pwm <1000-2000>
```

Etkilediği araçlar:

- `tools/sitl_rc_bridge.py`
- `tools/kenet_sitl_mixer.py`
- `tools/sitl_dashboard.py`

Amaç:

```text
CH7/AUX3 = 1500
```

Configurator Modes tab'de ANGLE range `AUX3 1300-1700` yapılırsa, bu komut
ANGLE mode'u fiziksel switch beklemeden test etmeyi sağlar.

Test komutları:

```bash
python tools/sitl_dashboard.py --open --force-mode-pwm 1500
```

veya dashboard kullanmadan:

```bash
python tools/kenet_sitl_mixer.py --device "$JOY_DEV" --camera test-2.mp4 \
  --send --force-mode-pwm 1500 --print-hz 5
```

Beklenen dashboard/Configurator gözlemi:

```text
Pilot CH7 / AUX3: 1500
FC CH7 / AUX3: 1500
Betaflight Modes: ANGLE aktif
Flight flags: ARMED + ANGLE
```

## Çözüm Planı

### Aşama 1 - ANGLE Mode Retest

1. Betaflight Configurator Modes tab:
   - ARM: `AUX1 / CH5`, range `1700-2100`
   - ANGLE: `AUX3 / CH7`, range `1300-1700`
   - Kenet state için `AUX2 / CH6` herhangi bir Betaflight mode'a
     bağlanmayacak.

2. Dashboard veya mixer şu argümanla başlatılacak:

   ```bash
   --force-mode-pwm 1500
   ```

3. Dashboard'da şu alanlar kontrol edilecek:
   - Kenet state: `IDLE`
   - CH7/AUX3 pilot ve FC: `1500`
   - Betaflight armed: `ARMED`
   - Arming flags: boş
   - Flight flags veya mode göstergesi: ANGLE aktif

4. Throttle düşükten yavaş artırılacak.

Kabul:

- ANGLE aktifken drone stabil kalkarsa, sorun büyük ölçüde mode/switch
  eksikliğidir.
- ANGLE aktifken yine anında takla varsa, motor/IMU mapping testine geçilecek.

### Aşama 2 - Motor Order / Direction Testi

Amaç: Betaflight'ın ürettiği motor düzeltmesinin Gazebo modelinde doğru fiziksel
rotora gittiğini görmek.

Test fikirleri:

1. Roll right/left komutu ver.
2. Pitch forward/back komutu ver.
3. `MSP_MOTOR` çıktısını kaydet.
4. Gazebo modelindeki rotor pozisyonlarıyla karşılaştır.

Beklenen QUADX mantığı:

```text
BF[0] REAR_R
BF[1] FRONT_R
BF[2] REAR_L
BF[3] FRONT_L
```

SDF'de mevcut hedef mapping:

```text
BF[0] -> rotor_3_joint / rear-right / cw
BF[1] -> rotor_0_joint / front-right / ccw
BF[2] -> rotor_1_joint / rear-left / ccw
BF[3] -> rotor_2_joint / front-left / cw
```

Kontrol:

- Roll correction sağ/sol momenti doğru yönde mi?
- Pitch correction ön/arka momenti doğru yönde mi?
- Yaw correction rotor direction ile doğru işaretli mi?

Eğer yanlışsa çözüm:

- `../aeroloop_gazebo/models/betaloop_iris_with_standoffs/model.sdf` içindeki
  BetaflightPlugin rotor `jointName` mapping'i düzeltilecek.
- Gerekirse `turningDirection` değerleri veya LiftDrag forward vector yönleri
  düzeltilecek.

### Aşama 3 - IMU Frame Testi

Amaç: Gazebo orientation bilgisinin Betaflight'a doğru frame ve işaretle
gittiğini doğrulamak.

Test fikirleri:

1. Gazebo'da modeli küçük roll açısıyla başlat veya elle döndür.
2. `MSP_ATTITUDE` roll/pitch değerini oku.
3. Gazebo görsel yönü ile Betaflight attitude işareti karşılaştır.

Beklenen:

```text
Gazebo'da sağa roll -> Betaflight roll aynı yönde değişmeli
Gazebo'da burun aşağı/üstü -> Betaflight pitch aynı yönde değişmeli
```

Eğer tersse çözüm:

- `../aeroloop_gazebo/plugins/BetaflightPlugin.cc` içindeki Gazebo -> NED /
  model orientation dönüşümü düzeltilecek.
- Değişiklik sonrası plugin rebuild gerekecek.

### Aşama 4 - Log ve Kabul

Her retest için:

```bash
python tools/analyze_sitl_log.py
```

Kabul kriterleri:

- Dashboard logunda MSP attitude ve motor sample var.
- Kenet `IDLE` testinde final-pilot delta yok.
- CH7/AUX3 forced testinde FC de `1500` görüyor.
- ANGLE aktifken takla yoksa workaround/çözüm doğrulanır.
- ANGLE aktifken takla devam ederse motor/IMU mapping düzeltmesi yapılır.

## Claude'dan Beklenen İnceleme

Claude aşağıdaki noktaları özellikle kontrol edebilir:

1. `betaloop_iris_with_standoffs/model.sdf` motor mapping'i Betaflight QUADX ile
   gerçekten uyumlu mu?
2. `turningDirection` ve LiftDrag forward vector kombinasyonu net upward thrust
   ve doğru yaw torque üretiyor mu?
3. `BetaflightPlugin.cc` içindeki orientation quaternion dönüşümü Betaflight
   SITL'in beklediği frame ile uyumlu mu?
4. ANGLE mode aktif değilken görülen takla tek başına mode eksikliğiyle
   açıklanabilir mi, yoksa motor/IMU mapping daha güçlü aday mı?

## Mevcut Durum

- `--force-mode-pwm` eklendi.
- İlgili testler geçti: `95 passed`.
- Henüz ANGLE forced retest yapılmadı.
- Nihai kök neden henüz kesinleşmedi; sadece Kenet PID/override dışlandı.

---

## Claude Yorumları Sonrası Codex Değerlendirmesi

Claude'un yorumlarında bu bug için özellikle haklı bulduğum noktalar:

1. **Mode/status decode kırılgan.**
   Dashboard ve smoke test `MSP_STATUS_EX` içinden bazı alanları pratik
   heuristiklerle çıkarıyor. Betaflight kaynakta `flightModeFlags`,
   `packFlightModeFlags()` ile aktif box sırasına göre paketleniyor; sabit bit
   anlamı gibi okunmamalı. Bu yüzden "ANGLE aktif mi?" sorusunu sadece mevcut
   dashboard etiketiyle kesinleştirmek doğru değil.

2. **`eeprom.bin` / kalıcı config testleri kirletebilir.**
   Betaflight SITL çalışma dizinine `eeprom.bin` yazıyor. Bu dosya eski mode
   range, mixer veya tuning ayarını sonraki run'a taşıyabilir. Gazebo takla
   analizinde hermetik/fresh config olmadan test sonucu belirsiz kalır.

3. **Motor smoke test motor-output hattını kanıtlıyor ama tam uçuş fiziğini
   kanıtlamıyor.**
   `MSP_MOTOR` yükselmesi ve rotor joint hareketi doğru yönde iyi bir smoke
   kanıtı; ancak motor order, motor direction, LiftDrag moment işareti ve IMU
   frame doğruluğunu tek başına kanıtlamıyor.

4. **Safe-exit RC frame / TX watchdog eksikliği debug'u zorlaştırabilir.**
   Bu muhtemelen ilk taklanın kök nedeni değil; ama mixer veya bridge kapanırsa
   SITL son RC paketini tutabilir. Crash sonrası "neden throttle kaldı?" gibi
   ikincil davranışları karıştırmamak için düzeltilmeli.

5. **State-machine drift ve AUX default riskleri büyük ölçüde kapatıldı.**
   Claude'un eski state-machine ve AUX default yorumları haklıydı; bu kısım
   artık ortak helper, CH6/AUX2 default ve safety testleriyle ilk turda
   toparlandı. Bu bug için ana odak artık Gazebo/Betaflight fizik hattı.

## Bug-Fix Plan v2

Bu planın amacı, "takla neden oluyor?" sorusunu tahminle değil, ayrıştırılmış
testlerle cevaplamak. Her aşama bir sonraki aşamaya geçiş için net kanıt
üretmeli.

### P0 - Gözlem ve Config Güvenilirliğini Düzelt

Önce dashboard'un verdiği mode/armed bilgisine güvenilebilir hale gelmesi
gerekiyor. Aksi halde ANGLE aktif mi, ARM gerçekten aktif mi, mode bitleri ne
anlama geliyor soruları bulanık kalır.

- [x] `tools/sitl_dashboard.py` içinde `MSP_STATUS_EX` parse sonucu
  `flight_mode_flags` için "raw bitmask" olarak gösterilmeye devam etsin, ama
  `armed=True/False` bu raw bitin sabit anlamına bağlanmasın.
- [x] Aktif mode isimlerini güvenilir çıkarmak için Betaflight MSP box bilgileri
  okunacak:
  - `MSP_BOXNAMES`
  - `MSP_BOXIDS`
  - `MSP_STATUS_EX` packed box flags
- [x] Dashboard `active_modes` gibi açık bir alan üretsin:
  - `ARM`
  - `ANGLE`
  - `HORIZON`
  - varsa diğer aktif box/mode isimleri
- [x] `tools/analyze_sitl_log.py`, `flight_mode_flags=0x1` gibi raw değeri
  tek başına yorumlamak yerine `active_modes` varsa onu raporlasın.
- [x] `tools/gazebo_sitl_motor_smoke.py` arming/mode flag decode için aynı
  yardımcıyı kullansın; ayrı magic-byte araması yapılmasın.

Kabul:

```text
Dashboard ve analyzer aynı log için active_modes içinde ANGLE/ARM bilgisini
aynı şekilde gösteriyor.
```

Uygulama:

- `tools/sitl_msp.py` eklendi; MSP frame/request yardımcıları ve
  `MSP_STATUS_EX` / `MSP_BOXNAMES` / `MSP_BOXIDS` parser'ları tek yere alındı.
- Dashboard artık `active_modes`, `active_modes_valid`, `active_mode_ids` ve
  `armed=unknown/true/false` üretir.
- Analyzer `active_modes` alanını raporlar; eski raw bit yorumuna dayanmaz.
- Motor smoke test arming flags için aynı shared parser'ı kullanır.

### P1 - Hermetik Betaflight SITL Başlat

Mode/mixer config'in önceki testlerden kalmadığından emin olunacak.

- [x] `tools/sitl_virtual_takeoff_check.py` Betaflight'i varsayılan olarak temp
  run directory içinde başlatır; repo-root `eeprom.bin` sadece
  `--betaflight-cwd repo` ile opt-in.
- [x] `tools/gazebo_sitl_motor_smoke.py` Betaflight'i varsayılan olarak temp
  run directory içinde başlatır; repo-root `eeprom.bin` sadece
  `--betaflight-cwd repo` ile opt-in.
- [x] `tools/gazebo_sitl_motor_smoke.py` PASS kararında `MSP_MOTOR` / motor PWM
  yükselişini ana sinyal kabul eder; rotor joint hareketi default'ta uyarı/ek
  kanıt, `--require-rotor-motion` ile katı kabul kapısıdır.
- [x] Dashboard process launcher Betaflight SITL'i temp çalışma diziniyle
  başlatır; snapshot/API `working_dir` ve `eeprom_path` alanlarını gösterir.
- [x] Dashboard process-control endpoint'leri default olarak loopback ile
  sınırlıdır; uzak istemci process start/stop için `--allow-remote-control`
  açıkça verilmelidir.
- [ ] Test log metadata içine şu bilgiler yazılsın:
  - Betaflight binary path
  - Betaflight cwd (`sitl_virtual_takeoff_check.py` ve
    `gazebo_sitl_motor_smoke.py` için tamam; dashboard process snapshot'ında
    mevcut)
  - eeprom path veya temp run dir (`sitl_virtual_takeoff_check.py` run dir'i,
    `gazebo_sitl_motor_smoke.py` `run_dir/eeprom_path` basıyor; dashboard
    process snapshot'ında mevcut)
  - world name
  - `--force-mode-pwm` değeri

Kabul:

```text
Yeni test run'ı repo root'ta eeprom.bin üretmiyor ve log metadata run dir'i
gösteriyor.
```

### P2 - ANGLE Forced Retest

Bu aşama sadece mode/switch belirsizliğini kapatır. Eğer ANGLE aktifken takla
devam ederse kök neden büyük olasılıkla motor/IMU/Gazebo fizik tarafındadır.

- [ ] Configurator Modes tab:
  - ARM: `AUX1 / CH5`, `1700-2100`
  - ANGLE: `AUX3 / CH7`, `1300-1700`
  - Kenet state: `AUX2 / CH6`, Betaflight mode'a bağlanmayacak
- [ ] Test `--force-mode-pwm 1500` ile çalıştırılsın.
- [ ] Dashboard/analyzer logunda şunlar kanıtlanacak:
  - `active_modes` içinde `ARM`
  - `active_modes` içinde `ANGLE`
  - CH7/AUX3 pilot ve FC tarafında `1500`
  - Kenet state `IDLE`
  - final-pilot delta `0`
- [ ] Throttle düşükten yavaş artırılacak; takla anı varsa `Mark Event`
  basılacak.

Kabul:

```text
ANGLE aktif + Kenet IDLE + pilot passthrough koşulunda takla var/yok sonucu
tek logla kanıtlandı.
```

Karar:

- Takla yoksa: ana sorun CH7/mode/switch config eksikliği veya yanlış mode
  görünürlüğüydü. Tango 2 CH7 mapping'i çözülür, forced mode sadece debug
  aracı olarak kalır.
- Takla devam ederse: P3/P4'e geçilir.

### P3 - Motor Order ve Direction İzolasyonu

Bu aşama Betaflight motor çıkışının doğru fiziksel rotora ve doğru dönüş
yönüne gittiğini kanıtlar.

- [ ] Iris modelde rotor pozisyonları tek tabloda çıkarılsın:
  - rotor joint
  - fiziksel konum: front/rear, left/right
  - `turningDirection`
  - LiftDrag forward vector yönleri
- [ ] Betaflight QUADX motor sırası aynı tabloda gösterilsin:
  - BF[0] REAR_R
  - BF[1] FRONT_R
  - BF[2] REAR_L
  - BF[3] FRONT_L
- [ ] Tek eksenli kontrollü komut testleri yapılacak:
  - roll sağ
  - roll sol
  - pitch ileri
  - pitch geri
  - yaw sağ/sol
- [ ] Her testte `MSP_MOTOR` ve mümkünse Gazebo rotor joint velocity/force
  değerleri loglanacak.
- [ ] Beklenen moment yönü ile gözlenen motor artışları karşılaştırılacak.

Kabul:

```text
Roll, pitch ve yaw düzeltmeleri fiziksel olarak doğru momenti üretiyor.
```

Eğer başarısızsa uygulanacak fix:

- `../aeroloop_gazebo/models/betaloop_iris_with_standoffs/model.sdf` içinde
  BetaflightPlugin `rotor id -> jointName` mapping'i düzeltilecek.
- Gerekirse `turningDirection` ve LiftDrag forward vector yönleri düzeltilecek.
- Plugin/world yeniden başlatılıp P2 tekrar edilecek.

### P4 - IMU Frame / Orientation İzolasyonu

Motor mapping doğruysa ama takla devam ediyorsa FC'nin gördüğü attitude işareti
yanlış olabilir.

- [ ] Gazebo'da model küçük roll/pitch açısıyla statik başlatılacak veya elle
  döndürülecek.
- [ ] Aynı anda dashboard `MSP_ATTITUDE` değerini kaydedecek.
- [ ] Görsel Gazebo yönü ile Betaflight roll/pitch işareti karşılaştırılacak.
- [ ] `BetaflightPlugin.cc` içindeki Gazebo -> NED / model orientation
  dönüşümü incelenecek.

Kabul:

```text
Gazebo'da sağ roll Betaflight'ta doğru roll işareti; burun aşağı/yukarı doğru
pitch işareti üretiyor.
```

Eğer başarısızsa uygulanacak fix:

- `../aeroloop_gazebo/plugins/BetaflightPlugin.cc` orientation dönüşümü
  düzeltilecek.
- Plugin rebuild yapılacak.
- P2 ve P3 tekrar koşulacak.

### P5 - RC Fail-Safe / Debug Hijyen Düzeltmeleri

Bu aşama ilk taklanın kök nedeni olmayabilir; ama testleri daha güvenilir yapar.

- [x] `tools/kenet_sitl_mixer.py` kapanırken throttle-low, roll/pitch/yaw center
  ve mevcut güvenli AUX değerleriyle son RC frame gönderecek.
- [x] RC TX watchdog eklenecek: gönderim periyodu nominalin 2 katını aşarsa log
  warning üretilecek.
- [x] Aynı anda birden fazla RC sender çalışıyorsa dashboard uyarı versin veya
  process launcher duplicate sender başlatmasın.

Kabul:

```text
Mixer kapanınca SITL son throttle komutunda kalmıyor; logda RC send stall
uyarıları görülebiliyor.
```

## Güncel Öncelik Sırası

1. P0: mode/armed decode güvenilirliği.
2. P1: hermetik SITL/eeprom çalışma dizini.
3. P2: `--force-mode-pwm 1500` ile ANGLE forced retest.
4. P3: takla devam ederse motor order/direction testi.
5. P4: motor doğruysa IMU frame testi.
6. P5: safe-exit RC frame ve TX watchdog.

Bu sıraya göre ilk uygulanacak gerçek kod işi dashboard/analyzer mode decode
fix'idir. Bundan önce yapılacak fizik retestleri yine bilgi verir, ama log
yorumları hâlâ tartışmalı kalır.

---

## #claude — Kök Neden: Gazebo ↔ Betaflight Zaman Senkronizasyonu (2026-06-23)

> #claude Bu bölümün tamamı benim incelemem. Olgu satırları (kod/veri) işaretsiz; benim
> yorum/önerilerim `#claude` ile başlar. Mevcut plandaki ANGLE/motor/IMU hipotezleri makul
> ve testleri yine yapılmalı — ama dokümanda eksik olan bir kök neden var ve veri/kod onu
> kuvvetle işaret ediyor: **otopilot ile fizik motoru ortak saatte senkron çalışmıyor.**

### #claude Kısa hüküm

> #claude Kullanıcının "Gazebo ile Betaflight arasında frekans/iletişim senkron sorunu var,
> otopilot ile fizik motoru uyumlu çalışmadığı için saçmalıyor" sezgisi kod ve veri tarafından
> destekleniyor. Aktif taklanın (düzeyken kendi motorlarıyla <1 sn'de 180°'ye gitmesi) en olası
> sürücüsü bir **rate-loop kararsızlığı**, ve bunun en güçlü kaynağı kaba+seğiren zaman adımı.
> ANGLE eksikliği, motor-order ve IMU-frame gerçek adaylar ama bunları ayırt etmenin en ucuz
> yolu **önce zamanlamayı düzeltip** retest etmek.

### Yeni kanıt (mevcut dokümanda yok)

1. **Taklayan dünya, fizik adımı olarak aykırı değer.**
   `../aeroloop_gazebo/worlds/betaloop_iris_betaflight_demo_harmonic.sdf:15-18` →
   `max_step_size = 0.004` (250 Hz). Repodaki diğer TÜM dünyalar 0.0025 (400 Hz), smoke
   testinin "çalışan" dünyası `test_betaflight.sdf` dahil. 250 Hz tek başına bu dünyada.

2. **Plugin gyro'yu adıma kilitli gönderiyor.** `../aeroloop_gazebo/plugins/BetaflightPlugin.cc:564-591`
   her fizik adımında bir kez `ReceiveMotorCommand()` + `SendState()` → Betaflight'a gyro
   250 Hz / 4 ms'de gidiyor. Lockstep yok.

3. **Betaflight SITL'in saati tahmini orana bağlı:**

   ```c
   // ../betaflight/src/platform/SIMULATOR/sitl.c
   :463  uint64_t micros64(void){ out += (now - last) * simRate; ... }   // FC saati = gerçek * simRate
   :256  if (deltaSim < 0.02 && deltaSim > 0) simRate = deltaSim / gercek_gecen_sure;  // her pakette TAHMİN
   :252  imuSetHasNewData(deltaSim*1e6);   // gyro dt = paketler arası sim zamanı (~0.004 s)
   :530  void delayMicroseconds(uint32_t us){ microsleep(us / simRate); }  // scheduler uykusu da simRate'e bağlı
   ```

4. **Canlı real-time factor 1.0'a kilitli değil.** `gz topic -e -t /stats` → anlık
   `real_time_factor: 1.31` (dalgalı; kümülatif sim 333 / real 342). Üstüne `gz sim gui` açık.

> #claude İki arıza modu aynı anda:
> #claude  (A) Kabalık — 250 Hz / 4 ms looptime, Betaflight'ın D-term ve gyro filtrelerinin
>           beklediği ~1 kHz'in çok altında → yetersiz sönümleme.
> #claude  (B) Seğirme — RTF sabit değil + GUI yükü → `simRate` dalgalanıyor → FC'nin saati
>           fizik motoruna göre kayıyor → PID'in dt'si tutarsız → kararlılık payı tükeniyor.
> #claude Bu yüzden takla bu dünyada çıkıyor, 400 Hz'lik dünyalarda ve headless smoke'ta çıkmıyor.

### #claude Mevcut hipotezlerle ilişki (reddetmiyorum, çerçeveliyorum)

> #claude **ANGLE eksikliği (mevcut P2):** Doğru bir gözlem ama tek başına aktif takla
> üretmez. ACRO'da merkezde stick = sıfır dönüş *hızı* komutu; küçük bir bozulmada drone yavaş
> sürüklenir, kendi motorlarıyla <1 sn'de 180° dönmez. Düzeyken hızlı ıraksama = rate döngüsü
> kararsızlığı. Ayrıca P0'ın kendisi "mode decode güvenilmez" diyor — yani "ANGLE kapalıydı"
> bilgisi de henüz kesin değil. ANGLE açık olsa bile, rate döngüsü dt yüzünden kararsızsa dış
> self-level döngüsü kurtaramaz.

> #claude **Motor-order / IMU-frame (mevcut P3/P4):** Meşru, ama aynı plugin + aynı model
> 400 Hz'lik dünyalarda ve headless smoke testinde taklamamış. Sert bir motor-sırası ya da
> gyro-işareti hatası olsaydı *deterministik* olurdu ve HER dünyada taklardı. Not: Betaflight
> SITL motor indekslerini zaten remap ediyor (`sitl.c:644-647`), ve IMU yolunda işaret
> çevirmeleri var (`sitl.c:199-201` gyro y,z negatif; `sitl.c:232` "pitch was inverted"
> yorumu; varsayılan yol `sitl.c:234` `imuSetAttitudeQuat`). Bu yüzden P3/P4 yine değerli —
> ama bunlar zamanlamadan bağımsız, sabit hatalar olur; tek bir dünyada çıkan takla zamanlamayı
> işaret ediyor. Sıralama: önce zamanlama, sonra (gerekirse) bunlar.

### #claude Bana sorulan 4 soruya cevabım

> #claude 1) *Motor mapping QUADX ile uyumlu mu?* — Statik okumadan kısmen: rotor yönleri
> (0,1 = ccw / 2,3 = cw) standart bir X-quad ile tutarlı, ama gerçek moment işaretini statik
> SDF okumasıyla kanıtlayamam; üstelik `sitl.c:644-647` BF motor indekslerini döndürüyor, yani
> efektif eşleme BF_motor → paket_slot → SDF_rotor zinciri. Bunu kesin doğrulamak için P3'teki
> tek-eksen testi şart. İstersen bu zinciri uçtan uca ben de izlerim.

> #claude 2) *turningDirection + LiftDrag net upward thrust ve doğru yaw torku veriyor mu?* —
> Yön kombinasyonu makul görünüyor ama yaw torku işaretini ancak P3'teki yaw komut testi
> kanıtlar; statik okuma yeterli değil.

> #claude 3) *Orientation dönüşümü BF SITL frame'iyle uyumlu mu?* — Varsayılan yol quaternion'u
> doğrudan veriyor (`sitl.c:234`), ama gyro ekseninde y/z negatiflenmiş (`:199-201`). Bu
> çevirmeler kasıtlı; yanlış olsalardı yine *tüm* dünyalarda deterministik takla olurdu. P4
> bunu canlı olarak doğrulamalı.

> #claude 4) *ANGLE eksikliği tek başına mı yoksa motor/IMU daha güçlü aday mı?* — Bence
> ikisi de tek başına bu *aktif/hızlı* taklayı açıklamıyor. En iyi desteklenen sürücü zamanlama
> kaynaklı rate-loop kararsızlığı; ANGLE eksikliği sadece kurtarma döngüsünü ortadan kaldırıyor.

### #claude Çözüm

> #claude Etki/efor sırasına göre:
> #claude  1. **Birincil (ucuz, yüksek olasılık):** demo dünyada `max_step_size` 0.004 → 0.001
>           (1 kHz; en azından diğerleriyle aynı 0.0025) + **headless** koş (`--headless`,
>           GUI'siz) ki RTF kararlı 1.0'a otursun.
> #claude  2. **Denge uyarısı:** makine seçilen adımı gerçek-zamanda sabit tutamazsa RTF düşer
>           ve seğirme geri gelir; "makinenin kararlı RTF ile sürdürebildiği en ince adımı" seç.
> #claude  3. **Sağlam/kalıcı:** lockstep senkronizasyonu — `sitl.c:272-274` içinde
>           `ENABLE_SIMULATOR_GYROPID_SYNC` / `mainLoopLock` altyapısı zaten kısmen var.
>           Açıksa ve plugin el sıkışmayı uyguluyorsa `simRate` tahminine hiç gerek kalmaz.
>           Birincil çözüm taklayı kesmezse buna geçilir.

### #claude Plana entegrasyon — yeni faz: "P0.5 Zamanlama İzolasyonu"

> #claude Mevcut P0/P1/P2'den bağımsız, paralel koşulabilir ve P3/P4'ten (motor/IMU) ÖNCE
> gelmeli, çünkü hem daha ucuz hem daha olası. Adımlar:
> #claude  1. Mevcut taklayı baseline olarak sakla (bu dokümandaki veri).
> #claude  2. Tek değişken: `max_step_size` 0.004 → 0.001 yap, dünyayı yeniden başlat, arm + gaz
>           senaryosunu tekrarla.
> #claude  3. Tek değişken: adımı geri al, sadece `--headless` koş, `/stats` RTF varyansını ölç.
> #claude  4. İkisi birlikte: ince adım + headless. Beklenen: düz kalır, motorlar simetriye
>           yakın, 180° roll yok.
> #claude  5. Hâlâ marjinalse: lockstep (`ENABLE_SIMULATOR_GYROPID_SYNC`) araştır.
> #claude  6. Zamanlama temizken hâlâ takla varsa → ANCAK O ZAMAN P3 (motor) ve P4 (IMU)
>           gerçek suçludur; onlara geç.
> #claude  7. Regresyon: `run_gazebo_betaflight.sh`, dünyanın `max_step_size`'ı 0.0025'ten
>           kabaysa uyarı bastırsın; demo dünya varsayılan headless olsun.

### #claude Doğrulama kriteri (mevcut araçlarla)

> #claude  - `tools/analyze_sitl_log.py` yeni logda "attitude exceeded 60 deg" uyarısı
>           BASMAMALI; max roll/pitch küçük kalmalı.
> #claude  - Dashboard'da arm + gaz sonrası roll/pitch ~0; motorlar birbirine yakın (büyük
>           spread yok).
> #claude  - `gz topic -e -t /stats` real_time_factor seçilen koşuda ~1.0'da KARARLI.
> #claude Üçü birlikte sağlanırsa düzeltme kanıtlanır; tek başına "takla görünmedi" yetmez,
> RTF'in kararlılığını da görmek gerekir çünkü asıl mekanizma odur.

### #claude Açık belirsizlik

> #claude MSP örnekleme hızı düşüktü (~1-2 Hz), ıraksamanın alt-saniye dinamiğini göremedim;
> motor-order/IMU-işareti ihtimalini teorik olarak %100 eleyemiyorum. Ama (i) çapraz-dünya
> kanıtı, (ii) lockstep'siz `simRate` tahmin mimarisi, (iii) canlı RTF seğirmesi birlikte ağır
> biçimde zamanlamayı gösteriyor. Bu yüzden P0.5'i motor/IMU testlerinden önce koydum.

> #claude Not: Bu bölümde hiçbir kod/konfig/çalışan sistem değiştirilmedi; yalnızca okuma ve
> canlı log/topic incelemesi yapıldı. Onay verirsen P0.5'i (adım + headless) ben uygularım.

### Codex Uygulaması - P0.5 Zamanlama İzolasyonu

Claude'un zamanlama hipotezini kritik ve ucuz doğrulanabilir buluyorum. Bu
nedenle P3/P4 motor/IMU testlerinden önce timing izolasyonu için şu araçlar
eklendi/güncellendi:

- `tools/run_gazebo_betaflight.sh`
  - Dünya dosyasındaki ilk `<max_step_size>` değeri `0.0025` üstündeyse uyarı
    basar.
  - `--max-step-size <value>` ile dış repo dosyasını kalıcı değiştirmeden
    geçici world kopyasında physics step override yapar.
  - `--headless` kullanıldığında varsayılan olarak `--headless-rendering` de
    geçirir.
- `tools/check_sitl_env.sh`
  - `test_betaflight.sdf` ve Iris demo world için `max_step_size` raporlar;
    kaba step değerini uyarı olarak gösterir.
- `tools/gazebo_stats_monitor.py`
  - `/stats` topic'inden `real_time_factor` örnekleri alır, min/mean/max/spread
    raporu üretir.
- `tools/sitl_dashboard.py`
  - `Start Gazebo` komutu `--gazebo-max-step-size` verilirse aynı temporary
    world override yolunu kullanır.

Timing retest komut seti:

```bash
tools/check_sitl_env.sh

tools/run_gazebo_betaflight.sh \
  --world betaloop_iris_betaflight_demo_harmonic.sdf \
  --headless \
  --max-step-size 0.001

python tools/gazebo_stats_monitor.py --samples 30 --interval 0.5

python tools/sitl_dashboard.py \
  --open \
  --gazebo-headless \
  --gazebo-max-step-size 0.001 \
  --force-mode-pwm 1500
```

Kabul:

```text
RTF spread düşük, analyzer 60 derece üstü attitude uyarısı basmıyor, motor
spread aşırı değil ve Kenet IDLE/passthrough iken takla tekrar etmiyor.
```

---

## #claude — KESİN KÖK NEDEN: Motor Eşleme (Mapping) Hatası (2026-06-24)

> ⚠️ #claude GEÇERSİZ KILINDI (2026-06-30): Bu bölümün "kesin kök neden = motor eşleme"
> başlığı artık aşırı iddialıdır. Motor eşleme GERÇEK bir hataydı ve düzeltildi (hem
> `--fix-iris-motor-map` overlay'i hem de kaynak `model.sdf` identity'ye çekildi), AMA
> motor-map düzeltildikten sonra bile flip SÜRDÜ. Yani motor eşleme taklanın tek/asıl
> sebebi DEĞİLDİ. Sonraki teşhisler (2026-06-30) asıl arıza imzasının yaw komutu altında
> bir ROLL taklası olduğunu ve muhtemel sebebin yaw tork işareti / dönüş yönü (yaw motors
> reversed) olduğunu gösteriyor — bkz. dosyanın en sonundaki en güncel #claude bölümü.
> Aşağıdaki analiz tarihsel kayıt için bırakılmıştır.

> #claude DÜRÜST DÜZELTME: Zamanlama düzeltmesi uygulandı (Codex P0.5; en yeni dashboard logu
> `gazebo_max_step_size: 0.001`, geçici dünyayla gerçek 1 kHz) ve takla SÜRÜYOR (yine 180° roll,
> 42 örneğin 8'i). Bu, zamanlamanın bu taklanın ASIL SEBEBİ OLMADIĞINI kanıtlıyor. Önceki bölümde
> zamanlamayı "en olası" diye öne koymuştum; veri beni düzeltti. Deterministik bir sebep
> aranmalıydı — statik iz sürme onu buldu: **Betaflight motor çıkışları yanlış fiziksel rotorlara
> gidiyor.** Bu uçmadan, sadece kodla kanıtlanabilir.

### Zincir (hepsi doğrulandı)

1. **Rotor fiziksel konumları** (`betaloop_iris_with_standoffs/model.sdf` pose'larından,
   x=ileri+, y=sol+):
   - `rotor_0` = (+0.13, −0.22) → **ön-sağ (FR)**, ccw
   - `rotor_1` = (−0.13, +0.20) → **arka-sol (RL)**, ccw
   - `rotor_2` = (+0.13, +0.22) → **ön-sol (FL)**, cw
   - `rotor_3` = (−0.13, −0.20) → **arka-sağ (RR)**, cw

2. **Betaflight motor remap** (`sitl.c:644-647`). Yorum (`sitl.c:637`) aynen:
   *"for gazebo8 ArduCopterPlugin remap"*. Yani BF0→slot3, BF1→slot0, BF2→slot1, BF3→slot2.
   Betaflight motorlarını **ArduCopterPlugin kanal düzenine** göre gönderiyor.

3. **Plugin** `pkt.motorSpeed[i]`'yi `rotors[i]`'ye uyguluyor (`BetaflightPlugin.cc:699-704`).
   aeroloop plugin'i ArduCopterPlugin türevi → çalışması için rotor `id` → `rotor_N` eşlemesi
   **kimlik (identity)** olmalı.

4. **AMA SDF kimlik DEĞİL** — fazladan bir permütasyon var (`model.sdf` satır 648-693):
   `id=0→rotor_3_joint, id=1→rotor_0_joint, id=2→rotor_1_joint, id=3→rotor_2_joint`.

**Net sonuç — her BF motoru hangi fiziksel rotora gidiyor:**

```text
BF motor 0 (RR olmalı)  → slot3 → id=3 → rotor_2 = ÖN-SOL    ✗
BF motor 1 (FR olmalı)  → slot0 → id=0 → rotor_3 = ARKA-SAĞ  ✗
BF motor 2 (RL olmalı)  → slot1 → id=1 → rotor_0 = ÖN-SAĞ    ✗
BF motor 3 (FL olmalı)  → slot2 → id=2 → rotor_1 = ARKA-SOL  ✗
```

> #claude DÖRT motorun DÖRDÜ de yanlış köşeye gidiyor. FC'nin roll/pitch düzeltmeleri yanlış
> eksende tork üretiyor → pozitif geri besleme → arm + gaz anında kaçınılmaz takla. Bu
> deterministik: her adım boyutunda, GUI veya headless farketmez, her seferinde aynı. "1 kHz'de
> bile sürüyor" gözlemi tam olarak bunun beklenen sonucu. Önceki ANGLE/IMU hipotezleri de bunun
> yanında ikincil kalıyor.

### #claude Düzeltme (plugin REBUILD gerekmez; sadece SDF + Gazebo restart)

> #claude `model.sdf`'teki 4 rotor bloğunu KİMLİĞE çek, turningDirection'ı fiziksel rotora göre ver:
> #claude   id=0 → rotor_0_joint, turningDirection ccw   (FR)
> #claude   id=1 → rotor_1_joint, turningDirection ccw   (RL)
> #claude   id=2 → rotor_2_joint, turningDirection cw    (FL)
> #claude   id=3 → rotor_3_joint, turningDirection cw    (RR)
> #claude Bununla zincir doğrulanıyor: BF0(RR)→slot3→rotor_3(RR) ✓, BF1(FR)→slot0→rotor_0(FR) ✓,
> BF2(RL)→slot1→rotor_1(RL) ✓, BF3(FL)→slot2→rotor_2(FL) ✓. `jointName`/`turningDirection` SDF
> yükleme anında okunduğu için `.so` yeniden derlenmez; sadece dünya yeniden başlatılır.

### #claude Kesin doğrulama (uçmadan, güvenli)

> #claude Betaflight Configurator → **Motors** sekmesi → sadece **Motor 1**'i hafifçe döndür
> (sim, güvenli). Doğrusu **arka-sağ** pervane dönmeli. Mevcut bug'da Motor 1 → slot3 → id=3 →
> rotor_2 = **ön-sol** döner. Ön-sol dönüyorsa eşleme hatası kesinleşir. 4 motoru tek tek deneyip
> tam haritayı çıkarabilirsin. Bu test BF tarafındaki olası özel motor remap'ini de hesaba katar
> (uçtan uca gerçek eşlemeyi ölçer), o yüzden statik izden bile güçlü kanıttır.

### #claude Zamanlama bulguları ne olacak?

> #claude 250 Hz aykırı adım ve RTF seğirmesi GERÇEK; P0.5 araçları (temp-world override, headless,
> stats monitor) değerli ve "iyi simülasyon hijyeni" olarak KALSIN — ama bu takla onların sebebi
> değil. Motor eşlemesi düzeldikten sonra ince adım + headless, kararlı uçuş kalitesi için yine işe
> yarar. Sıra net: önce motor eşlemesi (takla onunla gidecek), sonra zamanlama hijyeni.

> #claude Onay verirsen dış repo dosyasını (`../aeroloop_gazebo/.../model.sdf`) 4 rotor bloğunda
> kimliğe çeker, Gazebo'yu yeniden başlatır ve aynı arm+gaz senaryosuyla doğrularım. Dış repoyu
> değiştireceğim için önce onayını istiyorum; istersen önce Configurator motor testini sen yapıp
> teşhisi mührleyelim.

### Codex Uygulaması - Yapısal Diagnostic ve İlk Sonuç (2026-06-24)

Claude'un motor eşleme iddiasını kritik ve test edilebilir buluyorum. Bu yüzden
dış `aeroloop_gazebo` dosyasını hemen değiştirmeden önce, bu iddiayı ve canlı
SITL gecikmelerini tek JSONL logda toplayan diagnostic aracı eklendi:

```bash
python tools/sitl_diagnostics.py --samples 20 --interval 0.25
```

Araç şu alanları birlikte kaydeder:

- Gazebo stats topic, `real_time_factor`, `step_size`, iteration ve komut
  okuma süresi.
- Dashboard API yanıt süresi ve dashboard state yaşı.
- MSP poll süresi, RC, motor, attitude, active mode ve arming blocker'lar.
- Joystick poll süresi, raw axis/button ve pilot RC kanalları.
- Pilot RC ile FC RC arasındaki kanal deltaları.
- `8080`, `5761`, `6761`, `9002`, `9003`, `9004` port sahipleri.
- İlgili process'ler ve CPU/memory yüzdeleri.
- `betaloop_iris_with_standoffs/model.sdf` içindeki
  `BetaflightPlugin rotor id -> jointName` eşlemesi.

Kuru diagnostic sonucu, altta Gazebo/Betaflight/joystick kapalıyken bile motor
eşlemesi için şu yapısal uyarıyı verdi:

```text
motor mapping ok: False
mismatched BF motors: 0,1,2,3
SDF BetaflightPlugin rotor id->joint eşlemesi identity değil; Betaflight SITL
remap ile çift remap riski var.
Motor mapping analizi BF motorlarının beklenen fiziksel rotorlara gitmediğini
gösteriyor.
```

Bu bulgu Claude'un son yorumuyla uyumlu: Betaflight SITL zaten
`BF0->slot3, BF1->slot0, BF2->slot1, BF3->slot2` remap yapıyor. SDF içindeki
plugin ayrıca `id=0->rotor_3`, `id=1->rotor_0`, `id=2->rotor_1`,
`id=3->rotor_2` yapınca efektif zincirde dört motor da beklenen fiziksel
köşeden farklı rotora gidiyor.

Dashboard tarafında komut gecikmesini görünür yapmak için:

- Web refresh `500 ms` yerine `100 ms` yapıldı.
- Üst satıra `API <ms>` ve `age <ms>` eklendi.
- Eşzamanlı fetch birikmesini engelleyen `refreshInFlight` koruması eklendi.

Doğrulama:

```text
41 passed
```

### Güncel Karar

Şu an en güçlü yapısal kök neden motor mapping hatasıdır. Zamanlama/RTF hâlâ
izlenecek, ama takla ve arm davranışındaki ana düzeltme adayı:

```text
../aeroloop_gazebo/models/betaloop_iris_with_standoffs/model.sdf
BetaflightPlugin rotor id eşlemesini identity yapmak:
id=0 -> rotor_0_joint
id=1 -> rotor_1_joint
id=2 -> rotor_2_joint
id=3 -> rotor_3_joint
```

Kalıcı çözüm dış repo dosyasında yapılacak bir değişikliktir; ama ilk test için
repo dışını değiştirmeyen geçici overlay eklendi:

```bash
tools/run_gazebo_betaflight.sh \
  --world betaloop_iris_betaflight_demo_harmonic.sdf \
  --max-step-size 0.001 \
  --fix-iris-imu-pose \
  --fix-iris-motor-map
```

Dashboard `Start Gazebo` butonu artık varsayılan olarak geçici IMU pose ve
motor map fix'lerini kullanır. Gazebo GUI açık kalır; `--headless` verilmediği
sürece görsel uçuş testi yapılabilir.

Doğrulama:

```text
bash -n tools/run_gazebo_betaflight.sh
tools/run_gazebo_betaflight.sh ... --fix-iris-motor-map --dry-run
55 passed
```

Bir sonraki gerçek kabul testi: dashboard üzerinden Gazebo + Betaflight + Kenet
başlat, sonra `python tools/sitl_diagnostics.py --samples 20 --interval 0.25`
çalıştır. ARM blocker boşalmalı, FC RC kanalları pilotla uyuşmalı, takeoff
sonrası attitude 60 derece üstüne çıkmamalı.

---

## #claude — Yavaşlık (%3-4 RTF) ve Arm Olmama: Ölçümlü Yapısal Teşhis (2026-06-24)

> #claude İKİNCİ DÜRÜST DÜZELTME. Kullanıcı raporladı: Gazebo sağ altta RTF %3-4, kumanda
> gecikiyor, arm olmuyor, PC kapasitesi yeterli. Bunu CANLI ÖLÇTÜM (bir kez sistemi kazara
> bozdum, sonra durdum). Çıkan tablo önceki "blocking-recv çöküşü tek başına" çerçevemden daha
> nüanslı. Bu, takladan AYRI bir problem (takla = motor eşleme; bu = zamanlama/yük).

### Ölçülen RTF (gerçek veri)

```text
250 Hz (step 0.004), GUI açık (önceki gün)          RTF ~1.30        (gerçek-zamanın üstü)
1 kHz (step 0.001), Betaflight bağlı (probe ölçümü) RTF ~0.76        (degraded, < 1)
1 kHz (step 0.001), GUI açık (kullanıcı gözlemi)    RTF ~0.03-0.04   (ÇÖKMÜŞ)
```

> #claude 1 kHz'e geçince (benim önceki önerim) iş KÖTÜLEŞTİ: bu model+makine 1 kHz fiziği
> gerçek-zamanda zor yetiştiriyor (BF bağlıyken bile 0.76 < 1), GUI render yükü eklenince %3-4'e
> çöküyor. Tek bir quad için bu "PC zayıf" demek değil; 1 kHz + GUI + bu modelin fizik maliyeti
> gerçek-zaman bütçesini aşıyor.

### Neden bu wall-clock yavaşlığı her şeyi bozuyor (yapısal kısım)

> #claude Gazebo↔Betaflight kuplajı **lockstep değil**:
> #claude  - Plugin her fizik adımında motor paketi için bloklayan UDP recv yapıyor
>           (`BetaflightPlugin.cc:646-659`; BF online iken timeout 1 sn).
> #claude  - Betaflight saatini `gerçek_zaman × simRate` üretiyor, gecikmelerini
>           `microsleep(us/simRate)` veriyor (`sitl.c:463,530`); `simRate` = ölçülen RTF.
> #claude  - RTF < 1 olunca BF saati wall-clock'a göre yavaşlıyor → boot-grace, RXLOSS timeout,
>           throttle-low süresi gibi ZAMAN KAPILARI 25-30 kat geç doluyor, RC paketleri BF'nin
>           çarpık saatinde düzensiz görünüyor (RXLOSS). Sonuç: kumanda gecikmesi + ARM olmama.

### Neden arm olmuyor (canlı logdan)

`logs/sitl/20260624-005839-dashboard.jsonl` arming-disable bayrak zaman çizelgesi:

```text
01:01:01  RXLOSS, ANGLE, BOOTGRACE
01:01:06  RXLOSS, ANGLE          (BOOTGRACE temizlendi)
01:01:24  ANGLE                  (RXLOSS temizlendi)
01:01:40  ANGLE                  (kalıcı engel)
```

> #claude Kalıcı engel `ANGLE`: BF aracı arming açı limitinin ötesinde EĞİK görüyor — büyük
> olasılıkla motor-eşleme taklasından sonra ters/yatık (ya da IMU attitude yanlış). `RXLOSS`
> aralıklı (yavaş sim RC zamanlamasını bozuyor). Arm-olmama = motor-eşleme taklası + düşük RTF.

### #claude Eklenen test/loglama

> #claude  - `tools/sitl_timing_probe.py` — `/stats` RTF'ini JSONL'e loglar, `healthy/degraded/
>           collapsed` sınıflandırır, `--label` ile izolasyon. Mevcut `sitl_diagnostics.py`
>           (RTF+MSP+arming+motor-map) ile çakışmaz; bu MSP'siz, Gazebo-tek-başına fazına uygun
>           odaklı RTF aracı. Birim test: `tests/test_timing_probe.py` (14 test, suite 55 passed).
> #claude  - İzolasyon deneyi (reçete): aynı step ile (A) Gazebo TEK BAŞINA `--label gazebo-only`,
>           (B) Gazebo+BF `--label with-betaflight`, (C) headless vs GUI ve step 0.004/0.0025/0.001
>           süpürmesi. A hızlı, B/GUI çöküyorsa darboğaz kuplaj/GUI'dir, compute değil.

### #claude Önerilen yön

> #claude  1. Motor eşlemesi (Codex `--fix-iris-motor-map`) — takla bununla gider.
> #claude  2. Makinenin RTF≈1 ile sürdürebildiği step'i seç; ölçüm 1 kHz'in sub-realtime olduğunu
>           gösterdi → 0.0025 (400 Hz) muhtemelen daha iyi denge. Kontrol testinde GUI'yi kapat
>           (headless); GUI RTF'i ~20 kat düşürüyor.
> #claude  3. Kalıcı çözüm LOCKSTEP (`ENABLE_SIMULATOR_GYROPID_SYNC`): lockstep'te RTF<1 kontrol
>           doğruluğunu bozmaz (FC + fizik sim-zamanında birlikte adımlar). "250 Hz BF için fazla
>           kaba" ile "1 kHz gerçek-zamanda yetişmiyor" gerilimini ancak lockstep çözer.
> #claude  4. ANGLE engeli: sim'i upright resetle; motor eşlemesi düzelince araç düz kalırsa ANGLE
>           kendiliğinden temizlenir.

---

## Codex Güncel Durum Notu - 2026-06-24 Gece Kapanışı

Bu bölüm yarın aynı yerden devam edebilmek için yazıldı. Claude'un önceki
bulguları değerliydi, fakat elimizde artık daha güncel canlı log var.

### Son Canlı Test

Test koşusu:

```text
Dashboard: http://127.0.0.1:8080
Gazebo: GUI açık
world_max_step_size: 0.001
runtime flags: --fix-iris-imu-pose --fix-iris-motor-map
Betaflight SITL: 5761/tcp MSP online
Kenet: IDLE / pilot passthrough
```

İlgili loglar:

```text
logs/sitl/20260624-012120-dashboard.jsonl
logs/sitl/20260624-012336-diagnostics.jsonl
logs/sitl/20260624-011806-kenet-mixer.jsonl
```

Marker'lar:

```text
2026-06-24T01:23:36+0300 before_takeoff
2026-06-24T01:25:12+0300 after_takeoff_flip
```

Takeoff denemesi sırasında görülen kritik değerler:

```text
Kenet state: IDLE
Kenet source: pilot
Kenet final-pilot delta: 0
Active modes: ARM only
Autopilot mode channel: LOW / CH7=1000
Pilot roll/pitch/yaw: centered
Throttle: ~1560-1616
Max roll: -180.0 deg
Max pitch: -54.7 deg
Max motor spread: 2000
Örnek motorlar: 1544,1439,2000,1055
```

Analyzer özeti:

```text
Dashboard / Betaflight
  attitude samples: 2037
  motor samples: 2037
  msp_offline: 0
  active modes: ARM during armed period
  arming flags: ANGLE after crash / inverted state
  max roll: -180.0 deg
  max pitch: -54.7 deg
  max motor spread: 2000

Kenet Mixer
  mostly IDLE
  takeoff event is IDLE / pilot passthrough
  final-pilot deltas during takeoff: 0
```

### Claude Bulgularının Güncel Değerlendirmesi

1. **Zamanlama / RTF hipotezi**

   Claude'un zamanlama uyarısı debug açısından haklıydı. İlk dashboard
   gecikmesi ve MSP offline görüntüsü, bizim polling tasarımımız ve pahalı
   `gz topic -l` çağrıları yüzünden ölçümü bozuyordu. Bu düzeltildi.

   Güncel veri:

   ```text
   Gazebo RTF: mean ~1.0
   step_size: 0.001
   Dashboard state age: genelde <100 ms
   MSP offline: 0
   ```

   Sonuç:

   ```text
   Zamanlama problemi izlenmeye devam etmeli, ama 2026-06-24 takeoff flip'inin
   ana nedeni gibi görünmüyor.
   ```

2. **Motor mapping hipotezi**

   Claude'un "source SDF motor mapping çift-remap riski taşıyor" bulgusu
   doğru. `tools/sitl_diagnostics.py` source SDF için bunu hâlâ raporluyor:

   ```text
   motor mapping ok: False
   mismatched BF motors: 0,1,2,3
   ```

   Ancak son testte runtime komutu geçici düzeltmeyle başladı:

   ```text
   --fix-iris-motor-map
   runtime_motor_fix_active: True
   ```

   Buna rağmen takeoff flip devam etti. Bu iki anlama gelebilir:

   ```text
   A) Geçici overlay beklediğimiz modeli gerçekten kullanmıyor olabilir.
   B) Motor joint mapping düzelse bile direction / LiftDrag moment / IMU frame
      tarafında ikinci bir hata var.
   ```

   Bu yüzden motor mapping tek başına kapanmış kök neden kabul edilmemeli.
   Yarın ilk iş uçmadan, end-to-end motor testiyle gerçek runtime rotor
   davranışı doğrulanmalı.

3. **ANGLE mode / flight mode hipotezi**

   Kullanıcı yorumu doğru: FPV/Acro modda roll-pitch vermeden drone direkt
   takla atmamalı. ANGLE mode eksikliği tek başına bu davranışı açıklamıyor.
   ACRO'da merkez stick yaklaşık sıfır rate komutudur; doğru motor/IMU/fizik
   hattında gaz verince anında 180 derece flip beklenmez.

   Güncel yorum:

   ```text
   ANGLE mode retest faydalı olabilir ama ana kök neden adayı değil.
   ANGLE sadece dış self-level döngüsü; iç rate loop / motor moment / IMU
   işareti yanlışsa ANGLE da kurtarmayabilir.
   ```

4. **IMU frame / gyro sign hipotezi**

   Hâlâ güçlü aday. Flip sırasında FC motorları sert ayrıştırıyor ve bir motoru
   2000'e vuruyor. Bu, FC'nin gördüğü sapmaya agresif düzeltme uyguladığını
   gösteriyor. Eğer gyro/attitude işareti veya model frame dönüşümü ters ise bu
   düzeltme pozitif geri beslemeye dönüşür.

### Şu An Elenen / Güçlenen Adaylar

Elenen veya zayıflayan:

```text
Kenet PID / target tracking kaynaklı değil.
Dashboard lag / MSP offline kaynaklı değil.
Tek başına Gazebo RTF düşüklüğü kaynaklı değil.
Tek başına ANGLE mode eksikliği gibi görünmüyor.
```

Güçlü kalan adaylar:

```text
1. Runtime motor direction / yaw torque / LiftDrag moment yönü hatası.
2. Runtime motor joint mapping overlay gerçekten uygulanmadı veya etkisi
   beklenenden farklı.
3. IMU gyro / attitude frame işareti ters.
4. Betaflight mixer frame'i ile Gazebo model frame'i uyuşmuyor.
```

### Yarın Kaldığımız Yer

Mevcut sim durumu:

```text
Drone Gazebo içinde ters/yatık kaldı.
Betaflight disarmed.
Arming blocker: ANGLE
Attitude: roll yaklaşık -180 deg
```

Bu nedenle yeni testten önce Gazebo world/reset veya Gazebo restart gerekli.

Yarın ilk yapılacak güvenli sıra:

1. Gazebo + Betaflight + Kenet süreçlerini temiz restart et.
2. Dashboard'da şu temiz başlangıcı doğrula:

   ```text
   MSP connected: true
   roll/pitch: 0 / 0
   arming blockers: none
   Kenet: IDLE
   RC delta: 0
   RTF: ~1.0
   runtime motor map fix active: true
   ```

3. Takeoff yapma. Önce end-to-end motor testi yap:

   ```text
   Betaflight Configurator -> Motors tab
   Motor 1 düşük değerde çalıştırılır: hangi fiziksel rotor dönüyor?
   Motor 2 düşük değerde çalıştırılır: hangi fiziksel rotor dönüyor?
   Motor 3 düşük değerde çalıştırılır: hangi fiziksel rotor dönüyor?
   Motor 4 düşük değerde çalıştırılır: hangi fiziksel rotor dönüyor?
   ```

   Beklenen Betaflight QUADX fiziksel konum:

   ```text
   Motor 1 / BF0 -> rear-right
   Motor 2 / BF1 -> front-right
   Motor 3 / BF2 -> rear-left
   Motor 4 / BF3 -> front-left
   ```

4. Motor hangi rotora gidiyor doğruysa, aynı testte dönüş yönü / moment
   doğrulanacak:

   ```text
   Roll düzeltmesi doğru yönde moment üretiyor mu?
   Pitch düzeltmesi doğru yönde moment üretiyor mu?
   Yaw torque işareti doğru mu?
   ```

5. Motor tarafı doğruysa IMU frame testine geç:

   ```text
   Gazebo'da modeli küçük roll/pitch açısıyla döndür.
   Dashboard/MSP_ATTITUDE aynı yönde mi değişiyor?
   Gyro işareti ters mi?
   ```

### Yarın İçin Faydalı Komutlar

Canlı diagnostic:

```bash
python tools/sitl_diagnostics.py --samples 20 --interval 0.25 \
  --gazebo-timeout 1 --dashboard-timeout 0.5
```

RTF kontrol:

```bash
python tools/gazebo_stats_monitor.py --samples 10 --interval 0.25 --timeout 1
```

Log özeti:

```bash
python tools/analyze_sitl_log.py \
  logs/sitl/20260624-012120-dashboard.jsonl \
  logs/sitl/20260624-011806-kenet-mixer.jsonl
```

### Kısa Hüküm

Son veriyle benim güncel hükmüm:

```text
Takla artık "ANGLE kapalı" veya "dashboard/MSP/RTF yavaş" diye açıklanmamalı.
Bu, Betaflight-Gazebo fizik hattında motor moment yönü veya IMU/frame işareti
problemine çok benziyor. Yarın uçuş denemesi değil, uçmadan motor/moment/IMU
izolasyonu yapılmalı.
```

---

## #claude — İzolasyon Testi Tamamlandı + Codex Diagnostic Mutabakatı (2026-06-24)

> #claude Bu bölüm, yarım kalan izolasyon testini temiz ortamda (her şey kapalı, headless)
> tamamlamamın sonuçları ve Codex'in diagnostic çıktılarıyla mutabakat. Olgular işaretsiz,
> yorumlarım `#claude`.

### İzolasyon ölçümü (headless, step 0.001 / 1 kHz)

```text
Faz A  Gazebo TEK BAŞINA      RTF mean=0.768  min=0.739  max=0.787  spread=0.048  (KARARLI)
Faz B  Gazebo + Betaflight    RTF mean=0.455  min=0.001  max=0.793  spread=0.792  (VAHŞİ SEĞİRME)
```

> #claude Bu, yavaşlığı iki AYRI faktöre kesin ayırıyor:
> #claude  1. **Compute:** 1 kHz fizik bu model+makinede tek başına bile ~0.77 RTF — gerçek-zamanın
>           biraz altında ama KARARLI (spread 0.05). Yani 1 kHz real-time'a hafif ağır.
> #claude  2. **Kuplaj seğirmesi:** Betaflight eklenince RTF 0.001 ↔ 0.79 savruluyor; seğirme tek
>           başınakine göre ~16 kat arttı (spread 0.79 vs 0.05). Lockstep-olmayan bloklayan-recv +
>           `simRate` geri beslemesi. BF saatini bozan, kumandayı geciktiren, arm'ı engelleyen budur.
> #claude Codex'in tam-stack ölçümü bunu doğruluyor: mean ~1.0 ama **spread 0.92** — aynı seğirme
> imzası. İki bağımsız ölçüm aynı yere çıkıyor.

### Codex diagnostic'leriyle mutabakat

> #claude  - **Motor eşleme — bağımsız teyit.** Codex `analyze_motor_mapping` benim elle çıkardığım
>           zincirin aynısını buldu: BF0→rotor_2(ön-sol), BF1→rotor_3(arka-sağ), BF2→rotor_0(ön-sağ),
>           BF3→rotor_1(arka-sol) — **4/4 `ok=false`**. Kaynak SDF bozuk; `--fix-iris-motor-map`
>           runtime fix var. Takla bunun sonucu; bu artık iki yöntemle doğrulanmış kök neden.
> #claude  - **`--fix` aktifken bile roll=180:** büyük olasılıkla araç önceki takladan ters yatıyor
>           (kalıcı `ANGLE` engeli), yeni takla değil. AÇIK SORU: fix gerçekten yeni taklayı
>           durduruyor mu? Ancak aracı upright resetleyip arm ederek kanıtlanır.
> #claude  - **Yeni sinyal — pilot/FC RC delta 1000us:** Codex pilot ile FC RC arası 1000us'a varan
>           fark gördü. RC kanal eşlemesinde ayrı bir sorun ya da disarm/failsafe ikamesi olabilir;
>           takip edilmeli.

### #claude Net tablo — ÜÇ AYRI problem (artık karışmasın)

> #claude  1. **Takla** ← motor eşleme (4/4 yanlış; iki yöntemle teyitli). Fix: `--fix-iris-motor-map`,
>           upright reset + arm ile doğrula.
> #claude  2. **Yavaşlık tabanı** ← 1 kHz compute ağır (~0.77, kararlı). Fix: step 0.0025 (400 Hz) +
>           headless.
> #claude  3. **Seğirme + arm-olmama** ← BF lockstep-olmayan kuplaj (spread 0.79, min 0.001). Kalıcı
>           fix: lockstep (`ENABLE_SIMULATOR_GYROPID_SYNC`). 250 Hz "BF için kaba" / 1 kHz "real-time'a
>           ağır" gerilimini ancak lockstep çözer.

### #claude Sıradaki doğrulama

> #claude Sim'i upright resetle → 0.0025 + headless + `--fix-iris-motor-map` ile arm et → attitude
> düz kalıyor mu izle. Düz kalırsa motor-fix kanıtlanır ve takla kapanır; hâlâ takla varsa sıradaki
> şüpheli IMU/frame işaretidir (sitl.c:199-201 gyro y/z negatif, :232 pitch invert). Bu test, sistemi
> ben açıp koşabilirim.

---

## #claude — Uçuş Doğrulama Denemesi: Zamanlama Fix'i Kanıtlandı, Takla Fix'i Inconclusive (2026-06-24)

> #claude Yukarıdaki doğrulama testini bizzat koştum (scriptli RC göndericisi + MSP attitude/motor/
> irtifa izleyici, fiziksel joystick olmadan; `scratchpad/arm_test.py`). Her şey kapalı, temiz başlangıç.

### Kesinleşen (sağlam)

```text
Gazebo step 0.0025 + headless:  RTF mean=1.000  min=1.000  max=1.000  spread=0.000  (HEALTHY)
```

> #claude Zamanlama düzeltmem doğrulandı: 0.0025 (400 Hz) + headless tek başına kusursuz, kararlı
> real-time veriyor (1 kHz'in ~0.77 sub-realtime'ı ve GUI'nin %3-4 çöküşünün aksine). Yavaşlığın
> pratik çözümü budur. BF MSP standalone da çalışıyor (~15 sn boot + 3 sn timeout gerekiyor); eeprom'da
> ARM + ANGLE tanımlı.

### Uçuş/takla testi — 4 deneme, hepsi INCONCLUSIVE

```text
Koşu 1 (thr 1520):           ARM oldu, attitude 0.0/0.0 sabit -> büyük olasılıkla hiç kalkmadı
Koşu 2 (thr 1700):           ARM OLMADI -> ARM_SWITCH kilidi (BOOTGRACE temizlenirken switch zaten yüksekti)
Koşu 3 (thr 1700, geç toggle): MSP hiç cevap vermedi (ERR), plugin 40x "Broken Betaflight connection"
```

> #claude Hiçbir koşuda "arm + havada + motor dönüyor" durumuna GÜVENİLİR ulaşamadım; motorlar 1000
> (rölanti), irtifa 0.00 kaldı. Standalone scriptli yaklaşım, BF↔Gazebo kuplajının kararsızlığına
> çarpıyor.

### #claude Bunun kendisi bir bulgu

> #claude Plugin'in tek koşuda **40 kez "Broken Betaflight connection"** (`BetaflightPlugin.cc:674`)
> vermesi, BF↔Gazebo motor-paketi alışverişinin tekrar tekrar koptuğunu gösteriyor — lockstep-olmayan
> kuplaj kararsızlığının doğrudan kanıtı. Gazebo tek başına RTF 1.0'da sağlamken bile BF takılınca
> alışveriş güvenilmez oluyor. Bu, "seğirme + arm-olmama" probleminin (bkz. önceki bölüm) aynı kökten
> geldiğini pekiştiriyor.

### #claude Takla fix'inin durumu (dürüst)

> #claude Fix'in DOĞRU olduğuna dair güçlü STATİK kanıt var (benim elle izim + Codex'in
> `analyze_motor_mapping` aracı bağımsız olarak aynı doğru eşlemeyi veriyor). Ama uçuşta AMPİRİK
> olarak kanıtlanmadı — kuplaj kararsızlığı temiz bir uçuş denememe izin vermedi. Yani: "muhtemelen
> doğru ama uçuşta teyit bekliyor".

### #claude Kesin doğrulama için yol

> #claude  1. Orijinal repro yolu: **dashboard + fiziksel joystick**, `0.0025 + headless +
>           --fix-iris-motor-map`. Dashboard MSP/RC'yi kararlı tutuyor (takla ilk böyle görüldü,
>           Codex attitude'u böyle okudu); scriptli standalone'da kuplaj kopuyor.
> #claude  2. Ya da önce kuplajı sağlamlaştır (lockstep `ENABLE_SIMULATOR_GYROPID_SYNC`), sonra
>           scriptli otomatik test güvenilir olur.
> #claude Not: Bu denemede hiçbir kalıcı kod/konfig değişmedi; geçici gz/bf süreçleri açılıp kapatıldı,
> sistem temiz bırakıldı.

---

## Codex Plan Güncellemesi - Virtual RC Ön Aşaması (2026-06-29)

Yeni karar: fiziksel Tango 2 / USB joystick yolu şimdilik Gazebo takla
debug'ından izole edilecek. Amaç, Codex'in kendi başına tekrar edilebilir RC
sekansları üretebilmesi ve CH3 throttle / CH5 ARM / CH7 flight-mode değerlerini
kontrollü şekilde test edebilmesi.

Eklenen araç:

```bash
python tools/sitl_virtual_rc.py --script takeoff --throttle 1550 --mode-pwm 1500
python tools/sitl_virtual_rc.py --script takeoff --throttle 1550 --mode-pwm 1500 --send
```

Bu script fiziksel joystick okumaz. Betaflight SITL'in beklediği UDP `9004`
`rc_packet` formatında şu sanal kalkış sırasını gönderir:

```text
boot-low           CH3=1000 CH5=1000
arm-low-throttle   CH3=1000 CH5=2000
throttle-ramp      CH3 1000 -> hedef throttle
takeoff-hold       CH3=hedef throttle CH5=2000
disarm-low         CH3=1000 CH5=1000
```

Diagnostics de virtual RC beklentisiyle çalışabilir:

```bash
python tools/sitl_diagnostics.py --rc-source virtual \
  --virtual-throttle 1550 \
  --virtual-arm-pwm 2000 \
  --virtual-mode-pwm 1500 \
  --samples 20 --interval 0.25
```

Yeni öncelik:

1. Betaflight standalone açıkken virtual RC -> `MSP_RC` delta testini yap.
2. Gazebo Iris'i temiz restart ile `--headless --max-step-size 0.0025
   --fix-iris-imu-pose --fix-iris-motor-map` koş.
3. Virtual RC takeoff script'i ile arm/throttle ver.
4. Diagnostics/analyzer ile `MSP_RC`, `MSP_MOTOR`, `MSP_ATTITUDE`, RTF spread,
   arming blocker ve motor mapping uyarılarını kaydet.
5. Virtual RC ile hâlâ takla varsa fiziksel kumanda elenir; motor/moment/IMU
   veya Betaflight-Gazebo kuplajı ana şüpheli kalır.
6. Virtual RC ile takla kapanırsa fiziksel joystick/Tango path'i ayrı
   entegrasyon problemi olarak geri ele alınır.

Plan kaynağı: `plan-sitl.md` içinde `0.5 Ön Aşama P-1 - Virtual RC / Kumanda
İzolasyonu`.

---

## Codex Uygulama Sonucu - Virtual RC ile Takla Tekrar Üretildi (2026-06-29)

Bu oturumda fiziksel Tango 2 / USB joystick tamamen denklemden çıkarıldı.
Virtual RC ve doğrudan MSP diagnostics ile test yapıldı.

Eklenen/iyileştirilen araçlar:

```bash
python tools/sitl_virtual_rc.py --script takeoff --throttle 1550 --mode-pwm 1500 --send
python tools/sitl_configure_modes.py
python tools/sitl_diagnostics.py --rc-source virtual --direct-msp ...
python tools/analyze_sitl_log.py logs/sitl/20260629-212308-diagnostics.jsonl
```

Önemli ara bulgu:

- İlk denemede virtual RC, FC tarafında `MSP_RC` olarak doğru görünüyordu ama
  ARM mode aktif değildi; motorlar `1000`'de kalıyordu.
- Bunun nedeni RC yolu değil, çalışan Betaflight SITL oturumunda mode range
  konfigürasyonunun boş olmasıydı.
- `tools/sitl_configure_modes.py` ile Configurator açmadan şu range'ler canlı
  oturuma yazıldı:

```text
ARM          AUX1 / CH5 high 1600-2100
MSP OVERRIDE AUX2 / CH6 high 1700-2100
ANGLE        AUX3 / CH7 mid  1300-1700
```

Doğrulanan temiz pencereler:

```text
logs/sitl/20260629-211401-diagnostics.jsonl
  Standalone Betaflight virtual RC -> MSP_RC delta: 0

logs/sitl/20260629-212220-diagnostics.jsonl
  ARM low throttle:
  active modes: ARM, ANGLE
  arming blockers: -
  motors: 1055..1055
  attitude: roll=0 pitch=0

logs/sitl/20260629-212235-diagnostics.jsonl
  throttle 1520 hold:
  active modes: ARM, ANGLE
  arming blockers: -
  motors: 1521..1521
  attitude: roll=0 pitch=0
```

Tam scripted takeoff sonucu:

```text
log: logs/sitl/20260629-212308-diagnostics.jsonl
command: tools/sitl_virtual_rc.py --script takeoff --throttle 1550 --mode-pwm 1500 --send
Gazebo: --headless --max-step-size 0.0025 --fix-iris-imu-pose --fix-iris-motor-map
runtime motor-map overlay: active
RTF mean: ~1.000
active modes: ARM, ANGLE
```

Takla tekrar üretildi:

```text
first attitude >=60:
  sample 19, 2026-06-29T21:23:16+0300
  roll=72.6 pitch=30.8
  MSP RC RPYT=[1500,1500,1500,1156], AUX1=2000, AUX3=1500
  motors=[2000,1176,1055,1277]

max attitude:
  sample 28, 2026-06-29T21:23:20+0300
  roll=-180.0 pitch=0.0
  motors=[1151,1055,2000,1320]

analyzer:
  max motor spread: 945 us
  WARNING: attitude exceeded 60 deg; flip/tumble event
```

Karar:

1. Fiziksel Tango/USB joystick bu taklanın ana nedeni olarak elendi.
2. Virtual RC, Betaflight `MSP_RC` ve ARM/ANGLE mode tarafı doğrulandı.
3. `--fix-iris-motor-map` runtime overlay aktifken bile `1550` scripted
   takeoff'ta takla sürdü.
4. Sıradaki iş artık uçuş denemesi değil, motor/moment/IMU izolasyonu:
   hangi motor komutunun Gazebo'da hangi yönde fiziksel moment ürettiği ve
   FC'nin IMU roll/pitch işaretini nasıl gördüğü tek tek ölçülecek.

Buradan devam:

```text
1. ../aeroloop_gazebo/plugins/BetaflightPlugin.cc yaw gyro sign patch'i korunacak
   ve plugin rebuild edilmiş kalacak.
2. python tools/sitl_virtual_takeoff_check.py --throttle 1750 --hold-seconds 8
   komutu regression kabul testi olarak kullanılacak.
3. PASS kriterleri: altitude gain >=1 m, max roll/pitch <=35 deg, ARM+ANGLE
   görülür, pose/IMU loglanır, motor spread runaway yapmaz.
4. Kenet mixer sanal pilot + external checker kabul koşusu geçti:
   `logs/sitl/20260629-223523-external-kenet-takeoff-diagnostics.jsonl`.
   Sonuç: altitude gain 23.840 m, max roll/pitch 0.3/0.1, ARM+ANGLE 63 örnek,
   high throttle 53 örnek.
5. Mixer logu hedef-yok passthrough'u doğruladı:
   `logs/sitl/20260629-223523-kenet-mixer-virtual-external-takeoff.jsonl`;
   state/source `TRACKING` / `pilot-target-lost`, final-pilot delta 0.
6. Sentetik target-found koşuları:
   - centered target-found one-command runner:
     `logs/sitl/20260629-continued-synthetic-centered-runner-delay8-diagnostics.jsonl`
     + `logs/sitl/20260629-continued-synthetic-centered-runner-delay8-mixer.jsonl`
     -> PASS, `source=kenet`, `target_found=True`, final-pilot delta 0.
   - combined pitch+yaw:
     `logs/sitl/20260629-224953-external-synthetic-tracking-delayed-diagnostics.jsonl`
   - yaw only +26:
     `logs/sitl/20260629-225200-external-synthetic-yaw-only-diagnostics.jsonl`
   - yaw only -26:
     `logs/sitl/20260629-225340-external-synthetic-yaw-negative-diagnostics.jsonl`
   - pitch only +20:
     `logs/sitl/20260629-225520-external-synthetic-pitch-only-diagnostics.jsonl`
   - yaw micro +12 P-only:
     `logs/sitl/20260629-continued-yaw-micro-positive-ponly-diagnostics.jsonl`
   - yaw micro +4 P-only:
     `logs/sitl/20260629-continued-yaw-micro-positive-limit4-diagnostics.jsonl`
   - pitch micro +10 P-only:
     `logs/sitl/20260629-continued-pitch-micro-positive-ponly-diagnostics.jsonl`
   - direct virtual RC, Kenet yok, yaw 1504:
     `logs/sitl/20260629-continued-direct-virtual-yaw1504-clean-diagnostics.jsonl`
   Sonuç: centered target-found RC karışımı acceptance PASS; offsetli
   target-found komutları ve direct virtual RC nudge fizik acceptance FAIL.
7. 2026-06-30 son ölçüm: direct virtual RC `P23/yaw1504` baseline instrumented
   koşusu FAIL (`raw_spread=945`, aktif `gyro/error=610`, `P/Sum=450`). Aynı
   koşu Betaflight yaw authority profili `yaw_rc_rate=5`, `yaw_rate=30`,
   `yaw_rate_limit=120` ile iki kez PASS verdi:
   `logs/sitl/20260630-debug-acerror-yawauth-r5-s30-l120-yaw1504-p23-*` ve
   repeat1. PASS koşularında altitude gain `31.395/31.662 m`, max raw spread
   `0.616/0.614`, aktif debug sıfır rejimde kaldı. Bu profil runner'da
   `--safe-yaw-authority` olarak eklendi.
8. Fiziksel Tango/joystick yolu yeniden eklenecek; aynı kriterleri koruyup
   korumadığı ölçülecek.
9. Sonra gerçek kamera/video ile Kenet mixer + Gazebo target-found TRACKING
   testi yapılacak; pitch/yaw override sadece TRACKING + target found durumunda
   aktif olmalı.
10. Fiziksel RC veya Kenet mixer RC gönderirken kabul testi şu modla koşulacak:
   `python tools/sitl_virtual_takeoff_check.py --rc-driver external --throttle 1750`.
```

---

## 2026-07-02 — Zamanlama/Lockstep Hipotezinin Kapanışı (ölçüldü)

Bu dosyadaki "Kök Neden: Gazebo ↔ Betaflight Zaman Senkronizasyonu" hipotezi
(0.004 step + RTF ~1.31 + simRate kayması → PID dt tutarsızlığı) log ve kod
doğrulamasıyla test edildi ve **çürütüldü**:

- RTF incelenen her koşuda ~1.000 (min 0.9935, max 1.0013), lockstep öncesi
  `flipfix-baseline` dahil. RTF 1.31 hiçbir güncel logda yok.
- Motor UDP kadansı metronomik (0.0025 step → 400 Hz, 0.001 → ~997 Hz) ve
  PASS/FAIL koşularında istatistiksel olarak aynı (P21 PASS: mean 2.501 ms,
  sd 0.257; P23 FAIL: mean 2.500 ms, sd 0.243). Hiçbir paket boşluğu split'e
  öncülük etmiyor; büyük boşluklar crash SONRASI geliyor.
- Lockstep (`ENABLE_SIMULATOR_GYROPID_SYNC`, commit `f11faf414`) hiçbir FAIL
  vakasını PASS yapmadı; neutral zaten lockstep öncesinde de PASS'ti. Commit
  mesajındaki "primary driver" iddiası loglarla desteklenmiyor.
- Mekanizma notu: lockstep bloklamalı bekleme değil trylock-atla
  (`core.c` `lockMainPID`, `sitl.c` trylock/unlock). FDM başına en fazla 1 PID
  iterasyonu; motor paketi her build'de FDM başına 1 (`updateLock`). SITL
  saati simRate ile ölçeklenmiş duvar saatidir; Gazebo durursa motor çıkışı
  süresiz donar ama failsafe/MSP zamanlayıcıları ilerler (yapısal risk, mevcut
  flip'in nedeni değil).
- Step boyutu sınırı kaydırıyor ama yaw'ı kurtarmıyor: 0.001'de pitch1510/1522
  PASS (spread 3.1/13.0 µs), P23/yaw1504 hâlâ FAIL (nudge sonrası split
  0.877 s).
- Motor ölçekleme şüphesi de çürütüldü: paket değeri `(PWM-1000)/1000`,
  0.0–1.0 tam aralık; %20 otorite kaybı yok.

Kalan kök neden: Gazebo iris aktüatör modeli (P-only rotor hız döngüsü
`vel_p_gain=0.05`, plugin'de sabit `maxRpm=838` rad/s ölçeği) ile Betaflight
varsayılan yaw otoritesinin kapalı çevrim uyumsuzluğu. Arıza yaw P kazancında
ve setpoint büyüklüğünde monoton (P21 PASS / P22 sınır / P23 FAIL; yaw1503
PASS / yaw1504 FAIL); `yaw_rate_limit=120` setpoint'i rampalayıp rotor
döngüsünü lineer tutarak P23'ü PASS yapıyor.

Ayrıca ölçülen ikincil bulgu: koşular arası durum sızıntısı
(`20260630-235335-bracket-pitch1515` ilk örnekte roll `-180`, hiç arm olmadı;
`flipfix-yaw1650-rev/fixed` hiç arm olmadı → bu koşular kanıt değil INVALID).
Koşu geçerliliği kuralları ve ölçülebilir kabul kriterleri:
`docs/sitl-flight-readiness-criteria.md`. Bu dosyanın 8–10. maddelerindeki
"fiziksel RC yeniden eklenecek" planı da güncellendi: 2026-07-02 kararıyla
fiziksel RC tamamen opsiyoneldir ve hiçbir kapının ön şartı değildir.
