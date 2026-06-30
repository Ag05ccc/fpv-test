# SITL Acceptance Procedure

Bu dokuman fiziksel RC izole durumdayken Kenet/Gazebo/Betaflight SITL
kapilarini tekrar kosmak icin kisa kontrol listesidir. README komutlari aktif
`fpv_env` varsayar; temiz shell icin `fpv_env/bin/python ...` kullan.

## Varsayimlar

- `BETAFLIGHT_ROOT` Betaflight checkout'una isaret eder.
- `AEROLOOP_GAZEBO` Aeroloop Gazebo checkout'una isaret eder.
- Fiziksel Tango/joystick yolu aksi soylenmedikce izoledir.
- Betaflight temp cwd kullanilir; repo-root `eeprom.bin` sadece
  `--betaflight-cwd repo` ile opt-in.

## Hizli No-Hardware Gate

```bash
fpv_env/bin/python -m pytest -q
fpv_env/bin/python tools/sitl_pid_direction_check.py
```

Kabul:

- Unit/regression suite temiz.
- PID direction gate'de hedef saga gittiginde yaw > 1500, sola gittiginde
  yaw < 1500; centered ve lost hedef neutral kalir.

## MSP Smoke Gate

Betaflight SITL calisirken:

```bash
fpv_env/bin/python tools/kenet_msp_smoke.py --msp-tcp 127.0.0.1:5761
fpv_env/bin/python tools/kenet_msp_smoke.py \
  --msp-tcp 127.0.0.1:5761 \
  --set-raw-rc 1500,1600,1120,1400,2000,2000,1500,1500
```

Kabul:

- `MSP_API_VERSION`, `MSP_RC`, `MSP_ATTITUDE` okunur.
- `--set-raw-rc` pilot/AETR girdisi paylasilan AETR->MSP mapping ile okunur.

## Betaflight Mode Status Gate

Betaflight SITL calisirken Configurator acmadan mode range'leri uygula:

```bash
fpv_env/bin/python tools/sitl_configure_modes.py
```

Bu varsayilan tabloyu yazar:

```text
ARM          AUX1 / CH5 high 1600-2100
MSP OVERRIDE AUX2 / CH6 high 1700-2100
ANGLE        AUX3 / CH7 mid  1300-1700
HORIZON      AUX3 / CH7 high 1700-2100
```

Sanal RC ile aktif mode ve arming disable flag'lerini MSP'den oku:

```bash
fpv_env/bin/python tools/sitl_mode_status_check.py
```

Kabul:

- `all-low` kosusunda ARM/MSP Override/ANGLE/HORIZON aktif degil.
- `angle-mid` kosusunda yalniz ANGLE aktif.
- `horizon-high` kosusunda yalniz HORIZON aktif.
- `arm-angle` kosusunda ARM ve ANGLE aktif.
- `kenet-tracking-angle` kosusunda ARM, MSP OVERRIDE ve ANGLE aktif.
- Arming disable flag'leri ciktiya kaydedilir; Configurator Modes tab'a bakmak
  zorunlu degildir.

## Mixer Matrix Gate

P6 receiver/mixer kontratini donanimsiz kos:

```bash
fpv_env/bin/python tools/sitl_mixer_matrix_check.py
```

Kabul:

- P6.1 AUX2 LOW -> `IDLE`, tum kanallar pilot.
- P6.2 AUX2 MID -> `AI-ARMED`, tum kanallar pilot.
- P6.3 AUX2 HIGH hedef yok -> `pilot-target-lost`, tum kanallar pilot.
- P6.4 AUX2 HIGH hedef var -> yalniz pitch/yaw Kenet.
- P6.5 ARM low/high -> CH5 `1000/2000`.
- P6.6/P6.7 TRACKING sirasinda throttle/roll pilotta kalir.
- P6.8 hedef kaybi pitch/yaw'u pilota geri verir.

## Main Pipeline MSP Gate

Bir terminalde Betaflight SITL'i temp cwd ile baslat. Diger terminalde virtual RC
ile CH5 ARM high, CH6 Kenet high ve throttle dusuk/guvenli deger gonder:

```bash
fpv_env/bin/python tools/sitl_virtual_rc.py \
  --send \
  --port 9004 \
  --throttle 1120 \
  --arm-pwm 2000 \
  --kenet-pwm 2000 \
  --mode-pwm 1500
```

Sonra ana pipeline'i MSP TCP ile calistir:

```bash
fpv_env/bin/python kenet.py \
  --msp-tcp 127.0.0.1:5761 \
  --aux-ch 5 \
  --loop-hz 10 \
  --headless \
  --no-gcs \
  --camera test-2.mp4
```

Kabul:

- `TRACKING` gorulur.
- MSP override `send=[...]` satirinda pilot throttle korunur.
- CH6 mid'e dusurulunce `AI-ARMED` gorulur ve yeni override frame'i durur.

## Gazebo Virtual RC Gate

Gazebo/Betaflight runner'i fiziksel RC olmadan kos:

```bash
fpv_env/bin/python tools/sitl_virtual_takeoff_check.py \
  --capture-motor-udp \
  --nudge-yaw 1504 \
  --yaw-pid 23,0,0
```

Bilinen baseline bu profilde FAIL'dir. Olculmus guvenli yaw-authority kapisi:

```bash
fpv_env/bin/python tools/sitl_virtual_takeoff_check.py \
  --capture-motor-udp \
  --nudge-yaw 1504 \
  --yaw-pid 23,0,0 \
  --safe-yaw-authority
```

Kabul:

- Baseline ve safe-profile loglari JSONL olarak `logs/sitl/` altina yazilir.
- Safe-profile P23/yaw1504 kosusunda altitude gain pozitif, max attitude esik
  altinda ve raw motor spread dusuk kalir.
- `tools/sitl_pid_sweep_summary.py` ile raw motor axis bias ve
  `active_att_dt` raporlanir.

Guncel kanit:

- 2026-06-30 safe-yaw P23/yaw1504 PASS:
  `logs/sitl/20260630-064350-takeoff-diagnostics.jsonl`,
  `logs/sitl/20260630-064350-takeoff-motor-udp.jsonl`,
  `logs/sitl/20260630-064350-takeoff-virtual-rc.jsonl`.
- Ozet: altitude gain `31.402 m`, max roll/pitch `0.000/0.000`, raw spread
  `0.618`, raw axis `25/100/200/400us` esikleri tetiklenmedi.

## Real Video Target-Found Gate

Fiziksel RC olmadan, gercek video/tracker yolunu virtual pilot ile kos:

```bash
fpv_env/bin/python tools/sitl_video_tracking_check.py \
  --camera test-2.mp4 \
  --run-id video-target-found-neutral
```

Bu gate varsayilan olarak `yaw_limit=0` ve `forward_limit=0` kullanir. Amac,
gercek OpenCV tracker/video yolunda `target_found=True` ve `source=kenet`
goruldugunu kanitlamak, fakat pitch/yaw komutu vermemektir. Nonzero komutlu
video gate'i takeoff sonrasi gecikmeli Kenet state ve olculmus yaw profile ile
kosulacak.

Kabul:

- External Gazebo checker PASS.
- Mixer logunda yeterli `TRACKING`, `target_found=True` ve `source=kenet`
  sample'i var.
- `tracker_error` yok.
- Default neutral gate'te max pitch/yaw delta `0/0`.

Guncel kanit:

- 2026-06-30 `test-2.mp4` neutral video gate PASS:
  `logs/sitl/20260630-video-target-found-neutral-diagnostics.jsonl`,
  `logs/sitl/20260630-video-target-found-neutral-mixer.jsonl`,
  `logs/sitl/20260630-video-target-found-neutral-motor-udp.jsonl`.
- Ozet: altitude gain `23.850 m`, max roll/pitch `0.000/0.100`, raw spread
  `0.000`, mixer `265` target-found/source=kenet sample, frame `1280x720`,
  max pitch/yaw delta `0/0`.

## Synthetic Target-Loss Gate

Fiziksel RC ve kamera/tracker olmadan, target-loss passthrough sozlesmesini
Gazebo acceptance kosusunda olc:

```bash
fpv_env/bin/python tools/sitl_synthetic_tracking_check.py \
  --profile centered \
  --synthetic-target-loss-after-seconds 20 \
  --virtual-hold-seconds 80 \
  --min-target-lost-samples 5 \
  --min-ai-armed-samples 1 \
  --max-abs-delta 0
```

Kabul:

- External Gazebo checker PASS.
- Mixer logunda once `target_found=True` / `source=kenet` sample'i var.
- Hedef kaybolunca `source=pilot-target-lost` sample'i var ve final-pilot delta
  `0`.
- Lost threshold dolunca `AI-ARMED` sample'i var.
- Re-entry lockout aktif: switch track esiginin altina inmeden tekrar TRACKING
  denenmez.
- `--virtual-hold-seconds`, diagnostics penceresinden uzun tutulur; mixer
  safe-exit/disarm sonrasi dusus acceptance penceresine girmemeli.

Guncel kanit:

- 2026-06-30 sentetik target-loss final gate PASS:
  `logs/sitl/20260630-synthetic-target-loss-live-final-diagnostics.jsonl`,
  `logs/sitl/20260630-synthetic-target-loss-live-final-mixer.jsonl`.
- Ozet: altitude gain `58.505 m`, max roll/pitch `0/0`, motor spread `0`,
  mixer `source=kenet 200`, `pilot-target-lost 19`, `AI-ARMED 650`,
  max final-pilot delta `0`.

Nonzero yaw command-response gate:

```bash
fpv_env/bin/python tools/sitl_video_tracking_check.py \
  --camera test-2.mp4 \
  --run-id video-target-found-yaw4-p23-delay18 \
  --virtual-hold-seconds 40 \
  --yaw-limit 4 \
  --forward-limit 0 \
  --max-abs-delta 4 \
  --yaw-pid 23,0,0 \
  --kenet-delay-seconds 18
```

Not: Gecikmesiz `yaw_limit=4`, `2`, `1` video kosulari FAIL oldu; Kenet state
takeoff/ramp sirasinda TRACKING'e girmemeli.

Guncel nonzero kanit:

- 2026-06-30 `test-2.mp4` yaw2/P23/delay18 gate PASS:
  `logs/sitl/20260630-video-target-found-yaw2-p23-delay18-diagnostics.jsonl`,
  `logs/sitl/20260630-video-target-found-yaw2-p23-delay18-mixer.jsonl`,
  `logs/sitl/20260630-video-target-found-yaw2-p23-delay18-motor-udp.jsonl`.
- Ozet: altitude gain `31.401 m`, max roll/pitch `0.300/0.300`, raw spread
  `0.000`, mixer `203` target-found/source=kenet sample, max pitch/yaw delta
  `0/2`.
- 2026-06-30 `test-2.mp4` yaw4/P23/delay18 gate PASS:
  `logs/sitl/20260630-video-target-found-yaw4-p23-delay18-diagnostics.jsonl`,
  `logs/sitl/20260630-video-target-found-yaw4-p23-delay18-mixer.jsonl`,
  `logs/sitl/20260630-video-target-found-yaw4-p23-delay18-motor-udp.jsonl`.
- Ozet: altitude gain `31.401 m`, max roll/pitch `1.300/5.400`, raw spread
  `0.000`, mixer `196` target-found/source=kenet sample, max pitch/yaw delta
  `0/4`.
- 2026-06-30 `test-2.mp4` yaw5/P23/delay18 gate PASS:
  `logs/sitl/20260630-video-target-found-yaw5-p23-delay18-diagnostics.jsonl`,
  `logs/sitl/20260630-video-target-found-yaw5-p23-delay18-mixer.jsonl`,
  `logs/sitl/20260630-video-target-found-yaw5-p23-delay18-motor-udp.jsonl`.
- Ozet: altitude gain `31.377 m`, max roll/pitch `3.300/4.300`, raw spread
  `0.000`, mixer `226` target-found/source=kenet sample, max pitch/yaw delta
  `0/5`.
- 2026-06-30 `test-2.mp4` yaw6/P23/delay18 gate FAIL:
  `logs/sitl/20260630-video-target-found-yaw6-p23-delay18-diagnostics.jsonl`,
  `logs/sitl/20260630-video-target-found-yaw6-p23-delay18-mixer.jsonl`,
  `logs/sitl/20260630-video-target-found-yaw6-p23-delay18-motor-udp.jsonl`.
- Ozet: altitude gain `7.158 m`, max roll/pitch `180.000/72.400`, MSP motor
  spread `945`; mixer `195` target-found/source=kenet sample, max yaw delta
  `6`.
- 2026-06-30 `test-2.mp4` yaw8/P23/delay18 gate FAIL:
  `logs/sitl/20260630-video-target-found-yaw8-p23-delay18-diagnostics.jsonl`,
  `logs/sitl/20260630-video-target-found-yaw8-p23-delay18-mixer.jsonl`,
  `logs/sitl/20260630-video-target-found-yaw8-p23-delay18-motor-udp.jsonl`.
- Ozet: max roll `180.0`, max pitch `45.1`, diagnostics motor spread `945`;
  mixer `202` target-found/source=kenet sample, max yaw delta `8`. Bu nedenle
  video yaw command-response braketinde güncel sınır yaw5 PASS / yaw6 FAIL.

Pitch-only command-response gate:

- 2026-06-30 `test-2.mp4` pitch2/P23/delay18 default-pitch-PID gate FAIL:
  `logs/sitl/20260630-video-target-found-pitch2-p23-delay18-diagnostics.jsonl`,
  `logs/sitl/20260630-video-target-found-pitch2-p23-delay18-mixer.jsonl`,
  `logs/sitl/20260630-video-target-found-pitch2-p23-delay18-motor-udp.jsonl`.
- Ozet: altitude gain `8.569 m`, max roll/pitch `180.000/77.700`, MSP motor
  spread `945`; mixer `217` target-found/source=kenet sample, max pitch/yaw
  delta `2/0`.
- 2026-06-30 `test-2.mp4` pitch1/P23/delay18 default-pitch-PID gate FAIL:
  `logs/sitl/20260630-video-target-found-pitch1-p23-delay18-diagnostics.jsonl`,
  `logs/sitl/20260630-video-target-found-pitch1-p23-delay18-mixer.jsonl`,
  `logs/sitl/20260630-video-target-found-pitch1-p23-delay18-motor-udp.jsonl`.
- Ozet: altitude gain `13.620 m`, max roll/pitch `180.000/44.000`, MSP motor
  spread `945`; mixer `211` target-found/source=kenet sample, max pitch/yaw
  delta `1/0`.
- 2026-06-30 `test-2.mp4` pitch1/P23/delay18 repeat1 FAIL:
  `logs/sitl/20260630-video-target-found-pitch1-p23-delay18-repeat1-diagnostics.jsonl`,
  `logs/sitl/20260630-video-target-found-pitch1-p23-delay18-repeat1-mixer.jsonl`,
  `logs/sitl/20260630-video-target-found-pitch1-p23-delay18-repeat1-motor-udp.jsonl`.
- Ozet: altitude gain `12.838 m`, max roll/pitch `180.000/58.900`, MSP motor
  spread `945`; mixer `202` target-found/source=kenet sample, max pitch/yaw
  delta `1/0`.
- 2026-06-30 `test-2.mp4` pitch1/P23/delay18 with pitch PID 23/0/0 PASS:
  `logs/sitl/20260630-video-target-found-pitch1-p23-delay18-pitchpid23-diagnostics.jsonl`,
  `logs/sitl/20260630-video-target-found-pitch1-p23-delay18-pitchpid23-mixer.jsonl`,
  `logs/sitl/20260630-video-target-found-pitch1-p23-delay18-pitchpid23-motor-udp.jsonl`.
- Ozet: altitude gain `31.528 m`, max roll/pitch `0.000/0.000`, motor spread
  `0.000`; mixer `247` target-found/source=kenet sample, max pitch/yaw delta
  `1/0`.
- 2026-06-30 pitch2/pitch4/pitch5 with pitch PID 23/0/0 PASS:
  `logs/sitl/20260630-video-target-found-pitch2-p23-delay18-pitchpid23-*`,
  `logs/sitl/20260630-video-target-found-pitch4-p23-delay18-pitchpid23-*`,
  `logs/sitl/20260630-video-target-found-pitch5-p23-delay18-pitchpid23-*`.
- Ozet: pitch2 altitude gain `31.485 m`, max roll/pitch `15.800/16.400`;
  pitch4 altitude gain `31.535 m`, max roll/pitch `10.700/11.300`; pitch5
  altitude gain `31.538 m`, max roll/pitch `0.900/0.800`; hepsinde max motor
  spread `0.000`.
- 2026-06-30 pitch6 with pitch PID 23/0/0 FAIL:
  `logs/sitl/20260630-video-target-found-pitch6-p23-delay18-pitchpid23-diagnostics.jsonl`,
  `logs/sitl/20260630-video-target-found-pitch6-p23-delay18-pitchpid23-mixer.jsonl`,
  `logs/sitl/20260630-video-target-found-pitch6-p23-delay18-pitchpid23-motor-udp.jsonl`.
- Ozet: altitude gain `31.523 m`, max roll/pitch `78.800/9.300`, max motor
  spread `0.000`, mixer max pitch/yaw delta `6/0`. Bu nedenle pitch
  command-response braketinde güncel sınır pitch5 PASS / pitch6 FAIL.

Combined pitch+yaw command-response gate:

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
  --yaw-pid 23,0,0 \
  --pitch-pid 23,0,0 \
  --kenet-delay-seconds 18
```

Guncel combined kanit:

- 2026-06-30 yaw2+pitch2/P23/delay18/pitchPID23 gate PASS:
  `logs/sitl/20260630-video-target-found-yaw2-pitch2-p23-delay18-pitchpid23-diagnostics.jsonl`,
  `logs/sitl/20260630-video-target-found-yaw2-pitch2-p23-delay18-pitchpid23-mixer.jsonl`,
  `logs/sitl/20260630-video-target-found-yaw2-pitch2-p23-delay18-pitchpid23-motor-udp.jsonl`.
- Ozet: altitude gain `31.441 m`, max roll/pitch `3.100/2.600`, max motor
  spread `0.000`; mixer `175` target-found/source=kenet sample, max pitch/yaw
  delta `2/2`.
- 2026-06-30 yaw3+pitch3/P23/delay18/pitchPID23 gate PASS:
  `logs/sitl/20260630-video-target-found-yaw3-pitch3-p23-delay18-pitchpid23-diagnostics.jsonl`,
  `logs/sitl/20260630-video-target-found-yaw3-pitch3-p23-delay18-pitchpid23-mixer.jsonl`,
  `logs/sitl/20260630-video-target-found-yaw3-pitch3-p23-delay18-pitchpid23-motor-udp.jsonl`.
- Ozet: altitude gain `31.483 m`, max roll/pitch `3.200/3.300`, max motor
  spread `0.000`; mixer `193` target-found/source=kenet sample, max pitch/yaw
  delta `3/3`.
- 2026-06-30 yaw4+pitch4/P23/delay18/pitchPID23 gate FAIL:
  `logs/sitl/20260630-video-target-found-yaw4-pitch4-p23-delay18-pitchpid23-diagnostics.jsonl`,
  `logs/sitl/20260630-video-target-found-yaw4-pitch4-p23-delay18-pitchpid23-mixer.jsonl`,
  `logs/sitl/20260630-video-target-found-yaw4-pitch4-p23-delay18-pitchpid23-motor-udp.jsonl`.
- Ozet: altitude gain `8.169 m`, max roll/pitch `180.000/55.400`, MSP motor
  spread `945`; mixer `186` target-found/source=kenet sample, max pitch/yaw
  delta `4/4`. Bu nedenle combined command-response braketinde guncel sinir
  yaw3+pitch3 PASS / yaw4+pitch4 FAIL.

## External / Physical RC Gate

Bu gate yalniz `/dev/input/js*` gorundugunde kosulacak.

Once switch ayrimini RC gondermeden kontrol et:

```bash
fpv_env/bin/python tools/sitl_physical_rc_preflight.py \
  --device /dev/input/js0
```

CH7/AUX3 fiziksel switch henuz USB joystick'te gorunmuyorsa, ANGLE/mode
kanalini sender'da zorlayacagin profili de ayni preflight ile kaydet:

```bash
fpv_env/bin/python tools/sitl_physical_rc_preflight.py \
  --device /dev/input/js0 \
  --force-mode-pwm 1500
```

```bash
fpv_env/bin/python tools/sitl_virtual_takeoff_check.py \
  --rc-driver external \
  --safe-yaw-authority
```

Ayrica ayri terminalde tek bir RC sender calisir:

```bash
fpv_env/bin/python tools/sitl_rc_bridge.py \
  --device /dev/input/js0 \
  --send \
  --force-mode-pwm 1500
```

Kabul:

- Preflight CH5/AUX1 ARM icin low/high, CH6/AUX2 Kenet state icin low/mid/high
  gorur. CH7/AUX3 yoksa `--force-mode-pwm 1500` raporda forced ANGLE olarak
  acikca gorunur.
- Tek RC sender vardir; dashboard duplicate sender'i reddeder.
- CH5 ARM, CH6 Kenet state, CH7 ANGLE/mode ayrimi logda net gorulur.
- Fiziksel RC sonucu virtual RC gate ile ayni acceptance kriterlerine gore
  okunur.

## Gercek FC Bench Gate

Pervaneler sokulu olmadan kosulmaz.

- Betaflight `msp_override_channels = 10`.
- Ilk test motor davranisi degil Receiver/MSP davranisidir.
- Roll ve throttle pilotta kalir; Kenet yalniz pitch/yaw override eder.
- Throttle/roll Kenet authority bu prosedurun parcasi degildir; gerekirse ayri
  RFC ve yeni bench gate acilacak.
- CH5 ARM, CH6 Kenet state, CH7 mode ayrimi Configurator ve logda dogrulanir.

## Kanit Kaydi

Her kosuda su alanlari not et:

- Komut ve tarih.
- Log path'leri.
- PASS/FAIL.
- Max roll/pitch, altitude gain, max raw motor spread.
- Ilk raw axis threshold zamani.
- State gecisleri: `IDLE`, `AI-ARMED`, `TRACKING`, target lost/drop.
