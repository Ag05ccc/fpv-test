# Codex Roadmap v2 - FPV Gorsel Takip SITL

Tarih: 2026-07-04

Bu roadmap'in amaci, "sim ortaminda sabit kamerali FPV dron ile kullanicinin
sectigi hedefi tracker ile takip etmek ve hedefi ekranda merkezde tutacak
otonom pitch/yaw komutlarini Betaflight'a gondermek" hedefini olculebilir
kapilara bolmektir.

Bu dosya mevcut repo durumuna gore yazildi. Detay komutlar ve guncel kanitlar
asagidaki dosyalarda tutulur:

- `docs/sitl-quickstart.md`
- `docs/sitl-acceptance-procedure.md`
- `docs/sitl-flight-readiness-criteria.md`
- `docs/virtual-rc-gazebo-roadmap.md`
- `gazebo-bug1.md`
- `latest-development.md`

## Sert Kural

Bir is "tamam" sayilmaz; yalnizca su dort parcadan olusan kanit paketi varsa
tamam sayilir:

1. Kosulan komut.
2. Verdict: `PASS`, `FAIL` veya `INVALID`.
3. Log/artifact path'leri.
4. Metrikler: arm/disarm, motor, attitude, altitude, RC/MSP command, tracker,
   PID ve sim health.

`INVALID` kosu kabul kaniti degildir. Ornek: hic arm olmamis kosu, kirli ilk
attitude ornegi, MSP kopmasi, eski Gazebo/Betaflight sureci veya port
kalintisi.

## Hedef Mimari

Aktif hedef stack:

```text
Gazebo world + ileri bakan FPV kamera
  -> Kenet CameraCapture
  -> ObjectTracker (CSRT/KCF veya ileride detection+tracking)
  -> FlightController PID
  -> Kenet SITL mixer / MSP_SET_RAW_RC / UDP 9004 RC packet
  -> Betaflight SITL stabilization
  -> Gazebo motor/plant
```

Mevcut guvenlik karari:

- Kenet su an sadece pitch ve yaw komutu uretir.
- Roll ve throttle pilotta kalir.
- Hedefe yaklasma ilk fazda pitch-forward ve pilot throttle ile yapilir.
- Throttle veya roll authority istenirse ayri RFC, bench gate ve Gazebo gate
  acilmadan mevcut kapsama eklenmez.

## Mevcut Hukum

Kaniti olan iyi durumlar:

- No-hardware unit/regression gate var.
- PID direction gate var.
- Mixer matrix gate var: `TRACKING` sirasinda yalniz pitch/yaw Kenet'ten,
  roll/throttle pilottan gelir.
- Virtual RC ile arm, motor spin ve throttle takeoff kosulari olculebiliyor.
- Real video neutral ve target-loss kapilari var.
- Kamera ve populated world icin `./launch-fpv-sim.sh` yolu var.
- Kamera acikken ucus icin sync + tune profili gerekiyor; sync'siz kamera
  kosulari daha once flip uretmis.

Kaniti olan sorun alanlari:

- Roll/pitch/yaw komutlari genislediginde Gazebo + Betaflight closed-loop
  spin/flip uretebiliyor.
- Yaw command-response icin sync + all-axis tune dunyasinda olculmus kabul
  noktasi `yaw2 5/5 PASS`; yaw3-yaw5 satirinda metastable/saturasyon bandi var,
  yaw6 FAIL.
- FPV kamera, render yuku yuzunden kabul profilinin parcasidir; kamerali ve
  kamerasiz kanitlar karistirilmaz.
- Fiziksel RC bugunku debug hattinin on kosulu degildir. Virtual RC once
  gecmeli; fiziksel RC yalniz ayni kriterlerle opsiyonel dogrulama olur.

## Kullanici Sorulari ve Gate Eslesmesi

| Soru | Cevap verecek gate | Minimum kanit |
| --- | --- | --- |
| 1. Arm/disarm calisiyor mu? | G2 Flight Health | ARM mode aktif, `armed_angle_samples > 0`, disarm sonrasi motor low |
| 2. Arm olunca pervaneler donuyor mu? | G2 Flight Health | `MSP_MOTOR` veya raw motor UDP > 1000, rotor/motor sample var |
| 3. Throttle verince kalkiyor mu? | G2 Flight Health | altitude gain pozitif, climb rate pencere suresiyle raporlu |
| 4. Roll/pitch/yaw komutlarina dogru tepki var mi? | G3 Axis Response Matrix | her eksen icin komut alindi, beklenen isaret, no runaway, raw motor bias raporu |
| 5. Otopilot kontrolcusune uygun komut gonderebiliyor muyum? | G1/G4 Command Transport | UDP 9004, MSP RC readback, MSP Override/mode status, mixer source loglari |
| 6. Makul PID kontrolcum var mi? | G5 PID Quality | unit step/trace testi, saturation/rate-limit, live bounded command-response |
| 7. Tracker calisiyor mu? | G6 Tracker Quality | synthetic/real sequence IoU/loss, basit hareketlerde kopmama |
| 8. Sim hedefleri hareket ediyor mu? | G7 Moving Targets | actor/vehicle pose degisimi, camera frame'de hedef hareketi |
| 9. Kamera goruntusunu tracker'a besleyebiliyor muyum? | G8 Camera-to-Tracker | `/kenet/fpv_camera` frame smoke, nonblank frame, tracker init/update |

## Gate Matrix

### G0 - Ortam ve Kod Sagligi

Amac: Temel repo ve SITL ortam hatalarini ucus/tracker hatasi gibi
yorumlamamak.

Komutlar:

```bash
tools/check_sitl_env.sh
fpv_env/bin/python -m pytest -q
fpv_env/bin/python tools/sitl_readiness_report.py --json
```

Kabul:

- Python test suite PASS.
- `gz`, Betaflight SITL binary, Aeroloop Gazebo world/plugin bulunur.
- 9002/9003/9004/5761/6761 portlari raporlanir.
- No-hardware readiness `PASS`.
- Physical RC `WAITING` olabilir; bu debug hattini bloklamaz.

Eksik otomasyon:

- `sitl_readiness_report.py` icine `INVALID` kosu sinifi, RTF/kadans
  ozetleri ve tekrar sayaci tasinacak.
- Her gate sonunda tek `acceptance_manifest.json` uretilmeli.

### G1 - RC, Mode ve Mixer Sozlesmesi

Amac: Betaflight'a giden kanal sozlesmesini Gazebo ucmadan once sabitlemek.

Komutlar:

```bash
fpv_env/bin/python tools/sitl_pid_direction_check.py
fpv_env/bin/python tools/sitl_mixer_matrix_check.py
fpv_env/bin/python tools/sitl_configure_modes.py
fpv_env/bin/python tools/sitl_mode_status_check.py
fpv_env/bin/python tools/kenet_msp_smoke.py --msp-tcp 127.0.0.1:5761
```

Kabul:

- Hedef saga gittiginde yaw artar, sola gittiginde yaw azalir.
- Hedef buyuk/kucuk oldugunda pitch yonu beklenen sekildedir.
- `IDLE` ve `AI-ARMED` tam pilot passthrough yapar.
- `TRACKING` + target-found durumunda yalniz pitch/yaw Kenet tarafindan
  degisir.
- Target lost veya tracker unavailable durumunda pitch/yaw pilota geri doner.
- CH5 ARM, CH6 Kenet state, CH7 mode ayrimi tek mapping kaynagindan gelir.

### G2 - Flight Health: Arm, Motor Spin, Throttle Takeoff

Amac: "Dron arm oluyor, pervaneler donuyor, throttle ile kalkiyor" cumlesini
tekil gozlem olmaktan cikarip regression gate yapmak.

Baseline komut:

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

- Kosu gecerliligi: temiz ilk attitude, motorlar disarmed low, MSP bagli.
- ARM + ANGLE sample var.
- Arm sonrasi motorlar 1000 ustune cikar.
- Throttle penceresinde altitude gain pozitif ve pencere suresiyle raporlu.
- Disarm sonrasi motorlar tekrar low olur.
- Max roll/pitch profile ozel esigin altinda kalir.
- Raw motor spread runaway gostermez.
- Kabul profili 5/5 gecmeden "ucus saglikli" denmez.

Eksik otomasyon:

- Bu gate arm, motor spin, takeoff ve disarm metriklerini ayri satirlar
  halinde verdict'e yazmali.
- Basarisizlik `ARM_FAIL`, `MOTOR_NO_SPIN`, `NO_CLIMB`, `ATTITUDE_RUNAWAY`,
  `RAW_SPREAD_RUNAWAY`, `INVALID_START` gibi alt nedenlere ayrilmali.

### G3 - Axis Response Matrix: Roll, Pitch, Yaw

Amac: Kullanici RC komutu verdiginde Betaflight/Gazebo kapali cevriminin
dogru eksende, dogru isarette ve runaway olmadan cevap verdigini olcmek.

Yeni/gelistirilecek arac:

```text
tools/sitl_axis_response_matrix.py
```

Bu arac `sitl_virtual_takeoff_check.py` uzerine ince bir matrix runner olabilir.

Ilk matrix:

| Eksen | Komutlar | Beklenen |
| --- | --- | --- |
| yaw | 1501, 1502, 1503, 1504 | yaw rate/attitude beklenen isaret, raw yaw bias runaway yok |
| pitch | 1501, 1502, 1504, 1506 | pitch response beklenen isaret, altitude/attitude temiz |
| roll | 1501, 1502, 1504, 1506 | roll response beklenen isaret, attitude runaway yok |

Kabul:

- Her kosuda command sent ve MSP/RC readback kaniti var.
- Ilk buyuk raw axis bias hangi eksende basliyor raporlanir.
- Max attitude profile esigini gecmez.
- Raw motor spread stabil bolgede kalir; 945 us saturasyon FAIL.
- Sinir noktasi icin 5 gecerli tekrar gerekir.

Guncel beklenti:

- Bu gate bugun tam yesil degil. Spin sacmalamasini izole edecek ana gate bu.
- `zero-yaw-pid` sadece teshis gate'idir; kalici ucus cozum olarak kabul
  edilmez.

### G4 - Command Transport: Kenet Komutu Betaflight'a Ulasiyor mu?

Amac: Kenet'in urettigi komutun sadece Python icinde degil, Betaflight tarafinda
da goruldugunu kanitlamak.

Kanallar:

- UDP 9004 virtual RC packet.
- `kenet_sitl_mixer.py` merged RC packet.
- Production MSP path: `MSP_SET_RAW_RC`.
- Betaflight mode/motor/attitude readback.

Komutlar:

```bash
fpv_env/bin/python tools/kenet_msp_smoke.py \
  --msp-tcp 127.0.0.1:5761 \
  --set-raw-rc 1500,1600,1120,1400,2000,2000,1500,1500

fpv_env/bin/python tools/sitl_video_tracking_check.py \
  --camera test-2.mp4 \
  --run-id video-target-found-neutral \
  --yaw-limit 0 \
  --forward-limit 0 \
  --max-abs-delta 0
```

Kabul:

- Gonderilen RC frame, Betaflight MSP readback'te dogru kanal sirasiyla gorulur.
- `TRACKING` durumunda mixer logunda `source=kenet` gorulur.
- Max pitch/yaw delta istenen sinirla uyumludur.
- Roll/throttle final-pilot delta 0 kalir.

### G5 - PID Quality: Hedefi Merkezde Tutacak Kontrolcu

Amac: PID'in dogru isarette komut uretmesi yetmez; hedef hatasini azaltmali,
saturation ve rate-limit altinda kararsizlasmamali.

Mevcut durum:

- `FlightController` yaw icin horizontal pixel error kullanir.
- Forward/pitch icin hedef bbox genisligi ile `desired_target_width` farkini
  kullanir.
- PID anti-windup ve RC rate limit vardir.
- Unit test bugun sadece pitch/yaw disindaki kanallarin degismedigini ve
  direction gate'i kontrol ediyor; kalite metrikleri henuz zayif.

Eklenecek testler:

- `tests/test_controller_pid_quality.py`
- Sentetik bbox trace: hedef sagdan merkeze gelir, PID yaw komutu azalarak
  merkeze doner.
- Sentetik size trace: hedef uzakta/yakinda, pitch komutu dogru yone gider.
- Deadband testi: merkez civarinda komut neutral kalir.
- Anti-windup testi: uzun sure saturasyonda kalan hata sonrasinda integral
  patlamaz.
- Rate-limit testi: tek karelik buyuk hata RC jump uretmez.
- Target lost testi: PID resetlenir ve pitch/yaw neutral'a doner.

Live kabul:

- `tools/sitl_video_tracking_check.py` ile once neutral, sonra yaw-only,
  pitch-only, combined komutlar kosulur.
- Sync + all-axis tune dunyasinda yaw2 kabul noktasi 5/5 PASS olarak korunur.
- Daha genis yaw/pitch degerleri "provoke/sinir" etiketiyle tutulur, acceptance
  profiline yazilmaz.

### G6 - Tracker Quality

Amac: Tracker'in basit/yumusak hareketlerde hedefi kaybetmeden bbox urettigini
olcmek.

Mevcut durum:

- OpenCV contrib tracker factory testleri var.
- Real video gate `test-2.mp4` uzerinden target-found ve target-loss yolunu
  kosabiliyor.
- Tracker kalitesi icin IoU/loss/latency metrikleri henuz ayrik gate degil.

Eklenecek arac:

```text
tools/tracker_sequence_check.py
```

Kabul metrikleri:

- Synthetic sequence: 640x480 uzerinde hareket eden kutu/nesne, ground-truth
  bbox ile IoU raporu.
- Smooth motion: en az 300 frame boyunca loss yok, mean IoU >= 0.6.
- Moderate motion: kisa hizlanma ve scale change ile loss rate <= 5%.
- Occlusion/exit: hedef kaybolunca `found=False`, rastgele bbox uretmez.
- Re-init: kullanici yeni hedef sectiginde eski tracker state'i temizlenir.

Not:

- `test-*.mp4` lokal veri olarak kalabilir; CI icin sentetik video/frame
  uretimi tercih edilir.

### G7 - Moving Targets in Sim

Amac: Sim ortaminda hedeflerin gercekten hareket ettigini ve hareketin kamera
goruntusune yansidigini kanitlamak.

Eklenecek kontroller:

- Gazebo actor/vehicle pose topic veya world state uzerinden hedef pozisyonu
  zamanla degisiyor mu?
- FPV camera frame'lerinde hedefin piksel konumu degisiyor mu?
- Headless kosuda bile goruntu nonblank ve hareketli mi?

Eklenecek arac:

```text
tools/sitl_moving_target_check.py
```

Kabul:

- En az bir hedef icin dunya koordinati 10 saniyede anlamli degisir.
- Kamera frame'lerinde optical-flow veya simple color/shape detector ile hedef
  displacement raporlanir.
- RTF/kadans saglik metrikleri kabul bandinda kalir.

### G8 - Camera-to-Tracker Pipeline

Amac: Gazebo'daki dron kamerasindan gelen goruntu Kenet tracker'ina fiilen
besleniyor mu?

Komut ornegi:

```bash
tools/run_gazebo_betaflight.sh \
  --world betaloop_iris_betaflight_demo_populated.sdf \
  --headless \
  --iris-forward-camera \
  --max-step-size 0.0025

gz topic -i -t /kenet/fpv_camera
```

Eklenecek arac:

```text
tools/sitl_camera_tracker_check.py
```

Kabul:

- `/kenet/fpv_camera` publisher var.
- En az N frame okunur.
- Frame boyutu ve FPS raporlu.
- Frame nonblank.
- Tracker init edilebilir.
- Tracker update loop `found=True` sample uretir veya hedef yoksa bunu
  acikca `NO_TARGET_VISIBLE` olarak raporlar; sessiz PASS yok.

### G9 - Visible FPV Manual Flight

Amac: "Sim calisiyor" ile "Gazebo penceresinde FPV ucusu gorulebiliyor" ayrimini
net tutmak.

Komut:

```bash
./launch-fpv-sim.sh
```

Kabul:

- Gazebo GUI acilir.
- FPV Camera paneli gorunur.
- Klavye veya opsiyonel joystick ile arm/disarm yapilabilir.
- `e` ARM, space panic disarm, `w/s/a/d/i/k/j/l` kontrolleri logda RC frame
  olarak gorulur.
- Dashboard veya loglar arm, motor, attitude ve arming disable flag'lerini
  gosterir.

Not:

- Headless acceptance kosusu GUI dogrulamasi yerine gecmez.
- GUI isteniyorsa `--gazebo-gui` veya launcher GUI yolu acikca kullanilir.

### G10 - End-to-End Visual Servo: Hedefi Ortada Tutma

Amac: Kullanici hedefi merkeze getirip "takip et" dedikten sonra tracker ve PID
hedefi merkezde tutabiliyor mu?

Ilk kabul senaryosu:

1. Dron stabil hover/forward kamera ile baslar.
2. Hedef ekrana merkez yakininda girer.
3. Kullanici/senaryo tracker'i bbox ile init eder.
4. Kenet `TRACKING` durumuna gecikmeli girer; takeoff/ramp sirasinda tracking
   baslamaz.
5. PID yalniz yaw ve pitch komutlari uretir.
6. Betaflight/Gazebo flight health metrikleri bozulmaz.

Metrikler:

- Center error RMS ve P95 piksel.
- Bbox found ratio.
- Lost duration.
- RC command saturation suresi.
- Max roll/pitch.
- Raw motor spread.
- Altitude gain veya altitude bandi.

Ilk hedef:

- Yavas hareket eden hedef.
- Yaw-only veya cok dusuk combined komut limiti.
- Sync + all-axis tune dunyasi.
- 5/5 gecerli PASS.

### G11 - Approach Behavior

Amac: Sabit kamerali FPV dron hedefe dogru yaklasirken hedefi merkezde tutuyor
mu?

Kapsam karari:

- Faz 1: pitch-forward ile yaklasma, throttle pilot/senaryo tarafinda sabit.
- Faz 2: gerekirse throttle veya roll authority icin ayri RFC.

Kabul:

- Bbox width hedef `desired_target_width` degerine dogru yaklasir.
- Yaw center error artmaz.
- Pitch komutu rate-limit ve command-response guvenli penceresinde kalir.
- Dron hedefe yaklasirken altitude/attitude runaway yok.
- Target too-close durumunda pitch neutral veya retreat davranisi tanimli.

### G12 - Target Loss ve Fail-safe

Amac: Tracker hedefi kaybettiginde dronun "rastgele son komutu tutmasini"
engellemek.

Mevcut gate'ler:

```bash
fpv_env/bin/python tools/sitl_synthetic_tracking_check.py \
  --profile centered \
  --synthetic-target-loss-after-seconds 20 \
  --virtual-hold-seconds 80 \
  --max-abs-delta 0 \
  --min-target-lost-samples 1 \
  --min-ai-armed-samples 1

fpv_env/bin/python tools/sitl_video_tracking_check.py \
  --camera test-2.mp4 \
  --run-id video-target-loss-p23-delay18 \
  --virtual-hold-seconds 40 \
  --diagnostic-samples 80 \
  --yaw-limit 0 \
  --forward-limit 0 \
  --max-abs-delta 0 \
  --yaw-pid 23,0,0 \
  --pitch-pid 23,0,0 \
  --kenet-delay-seconds 18 \
  --target-loss-after-seconds 24 \
  --min-target-lost-samples 1 \
  --min-ai-armed-samples 1
```

Kabul:

- `source=kenet` sonrasinda `source=pilot-target-lost` gorulur.
- Uzun kayipta state `AI-ARMED` durumuna duser.
- Final-pilot delta 0.
- Re-entry icin pilot/switch niyeti gerekir.

### G13 - Physical RC Optional Return Path

Amac: Virtual RC ile cozulmus sistemin fiziksel kumanda ile ayni kriterleri
korudugunu gostermek.

Bu gate roadmap tamamlanmasi icin zorunlu degildir. Kullanici fiziksel kumanda
ile denemek istediginde kosulur.

On kosul:

- G0-G12 ilgili sanal kapilari PASS.
- No-send preflight PASS.

Komut:

```bash
fpv_env/bin/python tools/sitl_physical_rc_preflight.py \
  --device /dev/input/js0 \
  --force-mode-pwm 1500 \
  --verbose

./launch-physical-rc-sim.sh
```

Kabul:

- CH5 ARM low/high gorulur.
- CH6 Kenet low/mid/high gorulur.
- CH7 mode yoksa forced ANGLE logda acikca gorulur.
- Tek RC sender vardir.
- Sonuc virtual RC ile ayni acceptance kriterleriyle okunur.

### G14 - Raporlama ve CI Yapisi

Amac: Codex veya baska bir gelistirici "gecirdim" dediginde ayni artifact
paketiyle kontrol edilebilmesi.

Eklenecek yapi:

- `logs/sitl/<run-id>-acceptance.json`
- `logs/sitl/<run-id>-diagnostics.jsonl`
- `logs/sitl/<run-id>-motor-udp.jsonl`
- `logs/sitl/<run-id>-virtual-rc.jsonl`
- `logs/sitl/<run-id>-mixer.jsonl`
- `logs/sitl/<run-id>-summary.md`

`summary.md` sablonu:

```text
Run:
Date:
Git status:
Command:
World:
Camera:
Timing profile:
Verdict:
Validity:
Flight health:
Command response:
Tracker/PID:
Logs:
Notes:
```

CI ayrimi:

- Fast CI: unit tests, parser tests, controller/tracker synthetic tests.
- Local slow gate: Gazebo/Betaflight headless acceptance.
- Manual visible gate: GUI FPV/manual flight.

## Gelistirme Sirasi

### Faz 0 - Kanit altyapisini kapat

Yapilacaklar:

- `sitl_readiness_report.py` icine `INVALID` verdict ve Katman 2 sim health
  metriklerini ekle.
- Tek acceptance manifest formati uret.
- Mevcut `docs/sitl-flight-readiness-criteria.md` esiklerini kodla esit tut.

Definition of Done:

- `fpv_env/bin/python tools/sitl_readiness_report.py --json` no-hardware,
  sim-health ve known gate ozetlerini tek JSON'da verir.
- Hic arm olmayan kosu PASS/FAIL degil, `INVALID` olur.

### Faz 1 - Temel ucus sagligi

Yapilacaklar:

- G2 flight health gate'i arm, motor spin, throttle climb, disarm alt
  verdict'lerine bol.
- Safe-yaw/safe-manual kabul profilini 5/5 regression olarak koru.

Definition of Done:

- Arm/disarm, motor spin ve takeoff icin ayri PASS satirlari var.
- Basarisizlik nedeni tek bakista gorulur.

### Faz 2 - Roll/Pitch/Yaw spin kok nedeni

Yapilacaklar:

- G3 axis response matrix runner'i ekle.
- Roll, pitch, yaw icin kucuk nudge bracket'leri olc.
- Sync + all-axis tune ve kamera acik/kapali durumlarini ayri ayri etiketle.
- Provoke kosularini acceptance'tan ayir.

Definition of Done:

- Her eksen icin guvenli komut penceresi var.
- Spin ureten ilk eksen, ilk raw axis bias ve attitude threshold zamani
  raporlu.
- Kabul penceresi 5/5 gecerli PASS.

### Faz 3 - PID kalite testleri

Yapilacaklar:

- Controller PID synthetic trace testlerini ekle.
- Command saturation, anti-windup, deadband ve rate-limit testlerini ekle.
- Live video command-response gate'lerini yaw-only, pitch-only ve combined
  olarak sync dunyasina tasimayi tamamla.

Definition of Done:

- PID sadece dogru yone komut vermiyor; hata azalimi ve saturation davranisi
  testle kanitli.
- Live komut limiti acceptance manifest'te yaziyor.

### Faz 4 - Tracker kalite testleri

Yapilacaklar:

- Sentetik video/frame generator ile tracker sequence gate ekle.
- Real video gate'te tracker found/lost sample sayilarini ve bbox davranisini
  raporla.
- User selected ROI akisini ayri test et: merkez bbox yerine disaridan verilen
  bbox ile init.

Definition of Done:

- Smooth target sequence kopmadan takip ediliyor.
- Moderate motion icin loss/IoU raporu var.
- Target lost durumunda fail-safe gate tetikleniyor.

### Faz 5 - Sim target ve kamera pipeline

Yapilacaklar:

- Populated world icinde hareketli hedef senaryosunu sabitle.
- `/kenet/fpv_camera` frame smoke + nonblank + motion check ekle.
- Camera-to-tracker gate'i Gazebo frame'leriyle kos.

Definition of Done:

- Hedefler hareket ediyor.
- Kamera topic'i yayin yapiyor.
- Kenet tracker bu frame'lerden update aliyor.

### Faz 6 - End-to-end gorsel servo

Yapilacaklar:

- Yavas hedef, yaw-only takip.
- Yavas hedef, pitch-only approach.
- Yavas hedef, dusuk combined yaw+pitch.
- Sonra hiz ve ivme artirilir.

Definition of Done:

- Center error RMS/P95 raporlu.
- Bbox found ratio raporlu.
- Flight health ayni kosuda PASS.
- 5/5 gecerli PASS olmadan "stabil takip" denmez.

### Faz 7 - Yaklasma ve operasyonel davranis

Yapilacaklar:

- `desired_target_width` icin approach/too-close state tanimla.
- Pitch komutunun hedefe yaklasmayi olctugunu kanitla.
- Throttle/roll authority gerekip gerekmedigine metrikle karar ver.

Definition of Done:

- Hedefe yaklasirken merkezleme bozulmuyor.
- Too-close veya target-loss durumunda guvenli cikis tanimli.

### Faz 8 - Fiziksel RC'ye donus

Yapilacaklar:

- No-send preflight.
- Visible GUI flight.
- Ayni G2/G3/G10 kriterleriyle fiziksel RC sonucu.

Definition of Done:

- Physical RC sonucu virtual RC kriterleriyle okunur.
- Yeni kriter icat edilmez.
- Mapping farki varsa `RC_MAPPING_FAIL` olarak ayrilir.

## Ilk Uygulama Backlog'u

1. `tools/sitl_acceptance_manifest.py` veya `sitl_readiness_report.py` icinde
   ortak manifest uretimi.
2. `tools/sitl_axis_response_matrix.py`.
3. `tests/test_controller_pid_quality.py`.
4. `tools/tracker_sequence_check.py` ve sentetik sequence fixture generator.
5. `tools/sitl_camera_tracker_check.py`.
6. `tools/sitl_moving_target_check.py`.
7. `tools/sitl_visual_servo_check.py`.
8. Dokuman sync: bu roadmap'teki gate id'leri `docs/sitl-quickstart.md` ve
   `docs/sitl-acceptance-procedure.md` icine linklenecek.

## Durma Kriterleri

Asagidaki durumlardan biri gorulurse yeni ozellik eklenmez, once gate duzeltilir:

- G2 flight health FAIL veya INVALID.
- G3 axis response matrix'te yeni unexplained spin.
- Camera acikken sync/tune profili disina cikilmasi.
- Tracker hedef kaybinda pitch/yaw komutunun pilota donmemesi.
- Roll/throttle Kenet tarafindan yanlislikla degismesi.
- Acceptance kosusunun log/artifact uretmemesi.

## Sonraki Somut Adim

Bir sonraki teknik is Faz 0 + Faz 2'dir:

1. Readiness/acceptance raporunu `PASS/FAIL/INVALID` ve flight-health alt
   nedenleriyle sertlestir.
2. Roll/pitch/yaw nudge matrix runner'i ekle.
3. Mevcut spin sorununu bu matrix ile tekrar uretip guvenli komut penceresini
   5/5 tekrar kriteriyle sabitle.

Bu kapanmadan tracker/PID'i daha agresif hale getirmek yaniltici olur; gorsel
takip dogru komut uretse bile Betaflight/Gazebo closed-loop halen ayni komutta
spin uretebilir.
