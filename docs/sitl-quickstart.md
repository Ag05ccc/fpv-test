# SITL Quickstart

Bu kisa runbook, Kenet + Betaflight SITL + Gazebo kabul kapilarini fiziksel
kumanda olmadan (sanal RC ile) tekrar kosmak icindir. Butun gelistirme ve test
isleri bu sanal yoldan yurur; fiziksel RC opsiyoneldir ve hicbir kapinin on
sarti degildir. Ayrintili kabul kriterleri icin
`docs/sitl-acceptance-procedure.md`, olculebilir "SITL hazir" hukmu ve tekrar
kriterleri icin `docs/sitl-flight-readiness-criteria.md` dosyasini kullan.

## 0. Ortam

```bash
export AEROLOOP_GAZEBO="${AEROLOOP_GAZEBO:-../aeroloop_gazebo}"
export BETAFLIGHT_ROOT="${BETAFLIGHT_ROOT:-../betaflight}"
export JOY_DEV="${JOY_DEV:-/dev/input/js0}"

tools/check_sitl_env.sh
```

Kabul:

- `gz`, Betaflight plugin ve Betaflight SITL binary bulunur.
- `9002/9003/9004/5761/6761` port durumlari raporlanir.
- Fiziksel joystick yoksa bu donanimsiz akisi durdurmaz; virtual RC kullanilir.

## 1. Hemen Kosulacak Donanimsiz Gate'ler

```bash
fpv_env/bin/python -m pytest -q
fpv_env/bin/python tools/sitl_pid_direction_check.py
fpv_env/bin/python tools/sitl_mixer_matrix_check.py
```

Kabul:

- Test paketi temiz.
- PID direction gate hedef yonlerine gore pitch/yaw isaretlerini dogrular.
- Mixer matrix gate IDLE, AI-ARMED, TRACKING, target-lost ve roll/throttle
  passthrough sozlesmesini dogrular.

## 2. Gazebo + Virtual RC Takeoff Gate

Fiziksel RC'yi dahil etmeden once safe-yaw virtual RC kapisini kos:

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

- Altitude gain pozitif ve yaklasik `30 m` sinifinda.
- Max roll/pitch `35 deg` altinda.
- Motor/raw spread buyuk ayrisma gostermiyor.
- Bu profil bir ucus tune'u degil; SITL acceptance/debug profilidir.
- Physics step bir kabul parametresidir ve her gate kendi step'ini acikca
  sabitler (runner default'u `0.0025`). 2026-07-02 olcumu: coupled Gazebo iris
  plant'inin eksen kararliligi step'e bagli ve iki yonde ters calisiyor.
  `0.0025`'te yaw kararli (video yaw4/pitch5/combined3/target-loss PASS) ama
  pitch nudge kararsiz (`pitch=1522` FAIL, spread 233us). `0.001`'de pitch
  kararli (`pitch=1522/1530/1540` PASS) ama yaw kararsiz: yaw1504 nudge P
  siniri P19 PASS / P20+ FAIL'e iner (P22 `5/5 FAIL` deterministik) ve video
  yaw4 FAIL olur (yaw2 PASS). Yukaridaki safe-yaw komutu `0.001` pinli ve
  olculmus PASS'tir; video gate'leri step'i pinlemez ve `0.0025` default'unda
  kosar. Kok cozum plugin rotor PID'inin dt-normalize edilmesidir; o zamana
  kadar step degistirilerek "duzeltme" yapilmaz, sadece pinlenip kaydedilir.

## 3. Real Video Neutral Gate

Gercek video/tracker yolu komut uretmeden guvenli kalmali:

```bash
fpv_env/bin/python tools/sitl_video_tracking_check.py \
  --camera test-2.mp4 \
  --run-id video-target-found-neutral \
  --yaw-limit 0 \
  --forward-limit 0 \
  --max-abs-delta 0
```

Kabul:

- `target_found=True` ve `source=kenet` sample'lari vardir.
- Max pitch/yaw delta `0/0`.
- Diagnostics attitude ve motor spread temizdir.

## 4. Command-Response Gate'leri

Yaw-only olculmus guvenli pencere:

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

Pitch-only olculmus guvenli pencere:

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

Combined olculmus guvenli pencere:

```bash
fpv_env/bin/python tools/sitl_video_tracking_check.py \
  --camera test-2.mp4 \
  --run-id video-target-found-yaw3-pitch3-p23-delay18-pitchpid23 \
  --virtual-hold-seconds 40 \
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

- PASS sadece stabil ucus degil, beklenen eksende gercek komut uretildigi
  anlamina gelir.
- Guncel sinirlar: yaw5 PASS / yaw6 FAIL; pitch5 PASS / pitch6 FAIL;
  combined yaw3+pitch3 PASS / yaw4+pitch4 FAIL.

## 5. Target-Loss Gate'leri

Sentetik target-loss:

```bash
fpv_env/bin/python tools/sitl_synthetic_tracking_check.py \
  --profile centered \
  --synthetic-target-loss-after-seconds 20 \
  --virtual-hold-seconds 80 \
  --max-abs-delta 0 \
  --min-target-lost-samples 1 \
  --min-ai-armed-samples 1
```

Real video target-loss:

```bash
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

- Once `target_found/source=kenet`, sonra `source=pilot-target-lost`, sonra
  `AI-ARMED` gorulur.
- Max pitch/yaw delta `0/0`; hedef kaybinda pilot passthrough korunur.
- Diagnostics penceresi disarm sonrasina sarkmamali.

## 6. Loglari Yorumlama

```bash
fpv_env/bin/python tools/analyze_sitl_log.py --per-path logs/sitl/*diagnostics.jsonl
fpv_env/bin/python tools/sitl_readiness_report.py
```

`PASS`/`FAIL` satirlari hizli triage icindir. Genis yaw provoke denemeleri
`yaw_delta=+150/-150`, motor spread `945` ve attitude `180 deg` imzasi verdigi
surece safe-yaw acceptance penceresinden ayri tutulur.
`sitl_readiness_report.py`, no-hardware kanitlarinin PASS oldugunu ve fiziksel
RC / baska makine gibi dis gate'lerin bekleyip beklemedigini tek raporda
gosterir. CI veya not alma icin `--json` kullan.

Onemli: bir kosunun PASS/FAIL sayilabilmesi icin once GECERLI olmasi gerekir
(`armed_angle_samples > 0`, ilk diagnostik ornek temiz — roll/pitch ~0,
disarmed). Hic arm olmayan veya onceki kosudan kirli attitude devralan kosular
INVALID'dir ve brakete kanit yazilamaz; olculen ornekler ve tam kurallar icin
`docs/sitl-flight-readiness-criteria.md`.

## 7. FPV Modu: Ileri Bakan Kamera + Insanli/Aracli Dunya + Elle Ucus

Dron modeline istege bagli ileri bakan bir FPV kamerasi ve icinde insan/arac
modelleri olan bir dunya vardir. Tek komutla FPV ucusu (fiziksel donanim
GEREKMEZ, klavye yeterli):

```bash
./launch-fpv-sim.sh
```

Bu launcher sunlari yapar:

- Gazebo GUI'yi `betaloop_iris_betaflight_demo_populated.sdf` dunyasiyla acar
  (5 insan + 3 arac dronun onunde +Y yonunde, 1 insan + 1 arac arkada).
- Iris modeline `--iris-forward-camera` ile ileri bakan kamera enjekte eder;
  goruntu `/kenet/fpv_camera` topic'inde (640x480@30) ve Gazebo penceresindeki
  "FPV Camera" panelinde (`tools/fpv_gui.config`).
- Betaflight SITL'i gecici cwd ile baslatir, mode range'leri, guvenli manuel
  rate profilini (5/30/120) ve olculmus PID profilini (pitch/yaw `23,0,0`)
  uygular.
- Varsayilan giris klavyedir (`tools/sitl_keyboard_rc.py`, UDP 9004):
  `w/s` throttle (kalici), `a/d` yaw, `i/k` pitch, `j/l` roll (anlik,
  otomatik merkeze doner), `e` ARM, bosluk PANIC disarm, `1/2/3` Kenet
  state, `q`/ESC guvenli cikis (disarm burst gonderir).
- `./launch-fpv-sim.sh --input joystick` fiziksel kumandayi kullanir
  (yalnizca kullanici inisiyatifi; once throttle dusuk + ARM kapali bekler).

Ucus icin: `e` ile ARM, `w` ile throttle'i yavasca ~1600-1700'e cikar
(hover), kucuk `a/d` dokunuslariyla don. Kamera dogrulamasi headless da
yapilabilir:

```bash
tools/run_gazebo_betaflight.sh --world betaloop_iris_betaflight_demo_populated.sdf \
  --headless --iris-forward-camera --max-step-size 0.0025 &
gz topic -i -t /kenet/fpv_camera   # publisher gorunmeli
```

Kabul kapilari kamerasiz kosmaya devam eder (render maliyeti kanit
tabanini degistirmesin diye kamera yalnizca bayrakla acilir). Populated
world'un ucus-esdegerligi olculmustur: safe-yaw P23/yaw1504 kabul penceresi
populated world'de PASS (irtifa 22.985, roll/pitch 0.000, spread 0.683,
2026-07-03).

ONEMLI (2026-07-03 olcumu): kamera ACIKKEN ucus, gercekci-zamanlama (sync)
profilini gerektirir. Kameranin render yuku sync'siz dunyanin yapay-zamanlama
dengesini bozar: ayni PASS profili kamera+sync'siz iki kosuda da roll 180
flip verdi, kamerasiz ayni akis PASS, kamera+sync+tune PASS (roll 1.0).
`launch-fpv-sim.sh` bu nedenle `KENET_SITL_LOOPTIME_US` export edip
`configs/fpv-sim.txt`'yi eeprom'a import eder; FPV ucuslarini baska yoldan
kurarsan ayni profili kullan.

Manuel arm engeli (2026-07-03): sadece tune profiliyle (varsayilan
`small_angle=25`) SITL ivmeolcer temiz yercekimini gec bildirdiginde
Betaflight `ANGLE` arming-disable'i tutar ve dron tamamen duz olsa bile arm
etmez (motorlar 0'da kalir, pervaneler donmez — MSP'den canli dogrulandi: acc
0/0/0 okurken attitude ve Gazebo duruşu ikisi de duzdu). `configs/fpv-sim.txt`
tune profilinin ustune SITL temel ayarlarini ekler (`small_angle=180` egim
kontrolunu atlatir, `feature -3D`, `motor_pwm_protocol=PWM`,
`runaway_takeoff_prevention=OFF`). Kanitlandi: tam launcher konfigi (populated
world + kamera + sync + fpv-sim.txt) headless kosuda arm oldu (59 ARM+ANGLE
ornegi), motorlar dondu, 10.2 m tirmandi, PASS. Ayni konfig
`sitl_virtual_takeoff_check.py --iris-forward-camera` ile headless tekrar
edilebilir.

## 8. Opsiyonel: Fiziksel RC (Kullanici Inisiyatifi)

Bu bolum hicbir kabul kapisinin on sarti degildir; 0-6 sanal RC ile kapaninca
SITL akisi tamamdir. Fiziksel kumanda yalniz kullanici kendi isterse denenir.

- Tek komutla GUI + Betaflight SITL + fiziksel RC bridge acmak icin:

```bash
./launch-physical-rc-sim.sh
```

  Bu launcher Gazebo GUI'yi acar, Betaflight SITL'i gecici cwd ile baslatir,
  mode/PID profilini ve roll/pitch/yaw icin guvenli manuel rate profilini
  (`rc_rate=5`, `rate=30`, `rate_limit=120`) uygular, `/dev/input/js0`
  kumandayi UDP `9004` uzerinden yollar ve loglari `logs/sitl/<RUN>-*` olarak
  kaydeder. RC
  gondermeden once throttle dusuk ve ARM kapali bekler; terminalde `READY`
  gorunmeden arm/throttle verme. Ayrica dashboard'u
  `http://127.0.0.1:8080` adresinde acar; RC kanallari, ARM/Kenet/mode state,
  MSP arming flags, motorlar, attitude, loglar ve process butonlari buradan
  izlenir. Dashboard acikken diagnostics MSP bilgisini dashboard snapshot'indan
  okur; Betaflight MSP portu icin ikinci dogrudan okuyucu acilmaz.
- `/dev/input/js*` gorunmeden fiziksel Tango gate'i kosulmaz.
- Cihaz gorundugunde once no-send preflight kosulur:
  `tools/sitl_physical_rc_preflight.py --device /dev/input/js0 --verbose`.
  Bu adim CH5/AUX1 ARM, CH6/AUX2 Kenet ve CH7/AUX3 mode hareketlerini
  gormeden canli external RC gate acilmaz.
- CH7/AUX3 fiziksel switch henuz hareket uretmiyorsa ANGLE profili
  `--force-mode-pwm 1500` ile forced olarak kaydedilir.
- Virtual RC, synthetic target, real video neutral, command-response ve
  target-loss gate'leri temiz olmalidir.
- Fiziksel RC geri eklendiginde ayni acceptance kriterleri kullanilir; yeni
  kriter icat edilmez.
- Fiziksel RC'deki kucuk pitch dususunu sanal olarak tekrar uretmek icin
  `sitl_virtual_takeoff_check.py --max-step-size 0.001
  --safe-manual-authority --throttle 1500 --nudge-delay-seconds 14
  --nudge-pitch 1522 --pitch-pid 23,0,0 --yaw-pid 23,0,0` profili kullanilir.
  Dikkat: `--pitch-pid/--yaw-pid 23,0,0` bu profilin parcasidir; onceki dokuman
  bu bayraklari yazmiyordu ve default PID'lerle ayni komut `0.001`'de bile
  FAIL olur (2026-07-02 olcumu). Throttle `1475` yerine `1500` kullanilir;
  `1475` tirmanisi min-altitude-gain esigine pencere hizina gore takilabiliyor.
  Guncel kanit (2026-07-02): `0.001` + PID23 + throttle1500 ile
  `pitch=1522/1530/1540` PASS (spread 7.6/10.3/16.1us); ayni profil `0.0025`
  step'te `pitch=1522` FAIL (spread 233us, pitch 36.2deg). Bu fiziksel RC
  yerine Gazebo/Betaflight closed-loop physics step hassasiyetine isaret eder.
- Betaflight'in tam SITL CLI profilini test etmek icin runner'a
  `--betaflight-config-file /home/gz/betaflight/sitl_config.txt` verilebilir.
  Bu config import tek basina `0.0025` pitch kopmasini kapatmadi; asil kabul
  farkini `--max-step-size 0.001` yaratti.
