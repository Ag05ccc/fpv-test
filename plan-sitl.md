# Kenet / Betaflight SITL Yol Haritası

Amaç: Gerçek drone'a geçmeden önce Kenet'in kontrol zincirini masaüstünde
kanıtlamak. Plan mümkün olduğunca sade tutulur: önce RC ve Betaflight SITL
zinciri, sonra Kenet'in görsel takip çıktısı, en son gerekirse MSP ve fizik
simülasyonu.

Durum işaretleri: `[ ]` yapılacak · `[~]` kısmen tamam / beklemede · `[x]` tamam

Taşınabilir varsayılanlar:

```bash
export FPV_ROOT="$(pwd)"
export AEROLOOP_GAZEBO="${AEROLOOP_GAZEBO:-../aeroloop_gazebo}"
export BETAFLIGHT_ROOT="${BETAFLIGHT_ROOT:-../betaflight}"
export JOY_DEV="${JOY_DEV:-/dev/input/js0}"
```

Ana prensip:

```text
Önce kontrol zinciri, sonra protokol gerçekçiliği, en son fizik simülasyonu.
```

Bu yüzden Gazebo, aeroloop, betaloop ve benzeri ağır parçalar ilk aşamada
zorunlu değildir. Şu anki en değerli test, kumandadan gelen RC değerleri ile
Kenet'in ürettiği pitch/yaw komutlarının Betaflight SITL Receiver tarafında
doğru görünmesidir.

---

## 0. Nihai Hedef

Nihai SITL hedefi üç seviyeli olacak:

```text
Seviye 1 - Basit RC/SITL
RC kumanda -> Python bridge -> UDP 9004 -> Betaflight SITL -> Configurator

Seviye 2 - Kenet kontrollü RC mixer
RC kumanda + Kenet vision/PID -> tek RC packet -> UDP 9004 -> Betaflight SITL

Seviye 3 - Gerçek sisteme yakın MSP yolu
Kenet vision/PID -> MSP_SET_RAW_RC -> SITL UART/MSP 5761 -> Betaflight SITL
```

Opsiyonel son seviye:

```text
Seviye 4 - Fizik simülasyonu
Betaflight SITL -> motor/sensör/fizik bridge -> Gazebo veya uygun sim
```

Bu projede sırayla ilerleme kararı:

1. Gazebo/debug otomasyonu için fiziksel kumanda gerekmeden sanal RC yolu
   çalışacak.
2. Sonra Seviye 1 ve Seviye 2 sağlamlaşacak.
3. MSP sadece kontrol mantığı güvenilir olduktan sonra eklenecek.
4. Gazebo için portable launcher ve smoke test hazırlanabilir; tam fizik
   doğrulaması kontrol zinciri netleştikten sonra yapılacak.

---

## 0.1 Plan Sonunda Beklenen Hedef Çıktılar

Bu planın sonunda elimizde şunlar olmalı:

1. Fiziksel kumanda olmadan çalışan deterministic virtual RC acceptance hattı:
   Gazebo + Betaflight + mode setup + virtual takeoff/nudge + diagnostics tek
   komuttan koşulabilmeli.
2. Her kritik koşu için aynı kanıt paketi: diagnostics JSONL, raw motor UDP
   JSONL, virtual RC JSONL ve PASS/FAIL özeti.
3. Kenet'i dışarıda bırakan direct virtual RC testi ile Kenet mixer'ı kullanan
   external checker testi aynı kabul kriterlerini paylaşmalı.
4. Motor map, FDM/IMU sign, yaw mixer flag ve yaw PID/rate feedback ayrı ayrı
   izole edilmiş olmalı; hangi katmanın elendiği loglarla gösterilmeli.
5. Fiziksel Tango/joystick yolu geri eklendiğinde aynı runner `--rc-driver
   external` ile çalışmalı; virtual RC'de temiz olan kabul kriterleri fiziksel
   RC için de korunmalı.
6. Gerçek kamera/video ve target-found TRACKING testine geçmeden önce küçük
   pitch/yaw offsetlerinin Gazebo/Betaflight closed-loop cevabı anlaşılmış
   olmalı.

---

## 0.5 Ön Aşama P-1 - Virtual RC / Kumanda İzolasyonu

Amaç: Gazebo ve Betaflight SITL problemlerini çözerken fiziksel Tango 2
kumandasına, USB joystick event'lerine ve switch mapping belirsizliğine bağlı
kalmamak. Bu aşamada Codex/agent kendi başına deterministic RC frame'leri
üretebilir:

```text
Virtual RC script -> UDP 9004 rc_packet -> Betaflight SITL -> Gazebo plugin
```

Bu fiziksel kumanda doğrulamasının yerine geçmez; sadece bug izolasyonu ve
otomatik tekrar için kullanılır.

Araç:

```bash
python tools/sitl_virtual_rc.py --script takeoff --throttle 1550 --mode-pwm 1500
python tools/sitl_virtual_rc.py --script takeoff --throttle 1550 --mode-pwm 1500 --send
```

Kenet mixer yolunu da fiziksel joystick olmadan test etmek için:

```bash
python tools/kenet_sitl_mixer.py --pilot-source virtual --virtual-script takeoff \
  --virtual-throttle 1750 --virtual-kenet-pwm 2000 --virtual-mode-pwm 1500 \
  --no-vision --send
```

Kamera/video asset'i olmadan target-found yolunu test etmek için:

```bash
python tools/kenet_sitl_mixer.py --pilot-source virtual --virtual-script takeoff \
  --virtual-throttle 1750 --virtual-kenet-pwm 2000 --virtual-mode-pwm 1500 \
  --synthetic-target --synthetic-target-delay-seconds 18 --send
```

External checker ve sanal Kenet mixer'ı tek komutla koşturmak için:

```bash
python tools/sitl_synthetic_tracking_check.py \
  --profile centered --synthetic-target-delay-seconds 0
```

Kenet'i tamamen dışarıda bırakıp virtual RC ile takeoff sonrası küçük nudge
vermek için:

```bash
python tools/sitl_virtual_takeoff_check.py \
  --hold-seconds 30 --virtual-rc-timeout 80 \
  --nudge-delay-seconds 18 --nudge-yaw 1504
```

Betaflight mode range'lerini Configurator açmadan test oturumuna uygulamak için:

```bash
python tools/sitl_configure_modes.py
python tools/sitl_mode_status_check.py
```

Varsayılan test düzeni:

```text
ARM          AUX1 / CH5 high 1600-2100
MSP OVERRIDE AUX2 / CH6 high 1700-2100
ANGLE        AUX3 / CH7 mid  1300-1700
HORIZON      AUX3 / CH7 high 1700-2100
```

Takeoff script'i şu sırayı üretir:

```text
boot-low           CH3=1000 CH5=1000
arm-low-throttle   CH3=1000 CH5=2000
throttle-ramp      CH3 1000 -> hedef throttle
takeoff-hold       CH3=hedef throttle CH5=2000
disarm-low         CH3=1000 CH5=1000
```

ANGLE veya başka flight-mode retest için fiziksel CH7 switch beklenmeden:

```bash
python tools/sitl_virtual_rc.py --script takeoff --throttle 1550 --mode-pwm 1500 --send
```

Diagnostics, beklenen pilot RC kaynağını virtual alabilir:

```bash
python tools/sitl_diagnostics.py --rc-source virtual \
  --virtual-throttle 1550 \
  --virtual-arm-pwm 2000 \
  --virtual-mode-pwm 1500 \
  --samples 20 --interval 0.25
```

Betaflight'in MSP içinde raporladığı motor değeri ile simülasyon UDP çıkışını
ayırmak için raw motor probe kullanılabilir. Bu araç Betaflight'in `9001/udp`
raw servo paketlerini dinler; Gazebo'nun kullandığı `9002/udp` portuna müdahale
etmez:

```bash
python tools/sitl_motor_udp_probe.py --duration 30
```

Gazebo BetaflightPlugin'i doğrudan motor komutlarıyla test etmek için:

```bash
python tools/gazebo_motor_moment_probe.py \
  --base-speed 0.30 --pulse-speed 0.45 \
  --pulse-seconds 0.35 --settle-seconds 0.25
```

Kabul kriterleri:

- [x] `tools/sitl_virtual_rc.py` joystick olmadan AETR RC packet üretebiliyor.
- [x] CH3 throttle, CH5/AUX1 ARM, CH6/AUX2 Kenet ve CH7/AUX3 mode değerleri
  komuttan sabitlenebiliyor.
- [x] `tools/sitl_diagnostics.py --rc-source virtual`, joystick yokken yanlış
  "joystick bağlı değil" uyarısı üretmeden beklenen RC kanal setini kullanıyor.
- [x] `tools/sitl_configure_modes.py` ile Configurator açmadan
  ARM/ANGLE/HORIZON mode range'leri SITL oturumuna uygulanabiliyor.
- [x] `tools/sitl_motor_udp_probe.py` ile Betaflight'in gerçek raw UDP motor
  çıkışı `9001/udp` üzerinden kaydedilebiliyor.
- [x] `tools/gazebo_motor_moment_probe.py` ile Betaflight'i bypass edip
  Gazebo plugin motor->moment işaretleri ölçülebiliyor.
- [x] Betaflight çalışırken virtual RC -> `MSP_RC` delta testi yapıldı;
  standalone koşuda max pilot/FC RC delta `0`.
- [x] Gazebo Iris retest'i fiziksel joystick yerine virtual RC ile koşuldu.
  İlk sonuçta takla devam etti ve fiziksel kumanda/joystick kök neden olmaktan
  çıktı.
- [x] Tek komutluk virtual takeoff runner ile baseline takla tekrar üretildi;
  `../aeroloop_gazebo/plugins/BetaflightPlugin.cc` yaw gyro `z` sign patch'i ve
  rebuild sonrası aynı runner PASS verdi.
- [x] Kenet mixer artık `--pilot-source virtual` ile joystick açmadan sanal
  pilot frame'leri üretebiliyor.
- [x] `tools/sitl_virtual_takeoff_check.py --rc-driver external` ile Kenet
  mixer sanal pilot göndericisi aynı acceptance gate'te PASS verdi:
  `logs/sitl/20260629-223523-external-kenet-takeoff-diagnostics.jsonl`.
- [x] Bu koşuda `TRACKING` + hedef yok durumunda mixer tüm kanalları pilotta
  bıraktı: `logs/sitl/20260629-223523-kenet-mixer-virtual-external-takeoff.jsonl`,
  max final-pilot delta `0`.
- [x] `tools/kenet_sitl_mixer.py --synthetic-target` eklendi; kamera/video
  açmadan `TrackResult(found=True)` -> PID -> `source=kenet` zinciri test
  edilebiliyor.
- [x] `tools/sitl_synthetic_tracking_check.py` eklendi; external checker ve
  virtual-pilot synthetic-target mixer tek komutta koşturuluyor. Centered
  target-found profil PASS verdi. Runner artık mixer logunu da gate'e dahil
  ediyor: `TRACKING`, `target_found`, `source=kenet` ve opsiyonel
  `--max-abs-delta` limiti doğrulanmadan PASS dönmüyor.
- [~] Offsetli sentetik target-found acceptance henüz geçmedi; küçük pitch/yaw offset
  takeoff sonrası motor spread `945 us` ve flip üretiyor.
- [x] Micro sentetik profiller P-only ve düşük limitli hale getirildi; ayrıca
  direct virtual RC nudge eklendi. Kenet'siz yaw 1504 nudge da aynı flip'i
  üretti, bu yüzden bir sonraki kapı Betaflight/Gazebo closed-loop motor output
  zinciri.
- [x] Direct motor axis probe BF SITL motor output remap'iyle güncellendi:
  `tools/gazebo_motor_moment_probe.py --pattern-set axis` default olarak
  `--motor-map bf-sitl` (`3,0,1,2`) kullanıyor. Bu koşuda roll/pitch/yaw moment
  pattern'leri temiz ayrıştı; raw UDP slotlarını logical BF motoru sanan eski
  yorum yanıltıcıydı.
- [x] `tools/gazebo_fdm_probe.py` eklendi. Gazebo plugin'i Betaflight'a FDM yaw
  gyro'yu beklenen inverted işaretle gönderiyor:
  `logs/sitl/20260629-continued-fdm-yaw-cw-bfsitl.jsonl` ve
  `logs/sitl/20260629-continued-fdm-yaw-ccw-bfsitl.jsonl`.
- [~] `yaw_motors_reversed` A/B kapısı eklendi:
  `tools/sitl_mixer_config.py` ve
  `tools/sitl_virtual_takeoff_check.py --yaw-motors-reversed on|off`.
  ON koşusu direct yaw 1504 nudge'a kadar daha sakin kalıyor ama acceptance
  geçmiyor; motor spread `945` ve flip devam ediyor.
- [x] `tools/sitl_pid_config.py` ve runner'da `--zero-yaw-pid` /
  `--yaw-pid P,I,D` kapısı eklendi. Bu sayede Configurator açmadan Betaflight
  yaw P/I/D değerleri MSP üzerinden değiştirilebiliyor.
- [x] Direct virtual RC yaw 1504 koşusu yaw PID sıfırlanınca PASS verdi:
  `logs/sitl/20260630-yaw1504-zero-yaw-pid-diagnostics.jsonl`,
  `logs/sitl/20260630-yaw1504-zero-yaw-pid-motor-udp.jsonl`,
  `logs/sitl/20260630-yaw1504-zero-yaw-pid-virtual-rc.jsonl`.
  Sonuç: altitude gain `23.822 m`, max roll/pitch `0.2/0.3`, diagnostics motor
  spread `0`, raw UDP max spread `0.767`, first large raw spread yok.
- [~] Bu bir uçuş çözümü değil; yaw otoritesini kapatarak kök nedeni daraltan
  teşhis kapısı. Sıradaki iş yaw P ve I terimlerini/işaretini ayrı ayrı
  ölçmek.
- [x] İlk yaw P/I sweep tamamlandı. Aynı direct virtual RC yaw 1504 acceptance
  kapısında P-only `10,0,0`, `15,0,0`, `17,0,0`, `18,0,0` ve `19,0,0` PASS
  verdi; P-only `20,0,0` ve `45,0,0` FAIL. I-only `0,1,0` PASS; `0,2,0`,
  `0,5,0`, `0,20,0`, `0,80,0` FAIL.
- [~] Geçici sonuç güncellendi: ilk repo-root cwd sweep'i `P20/P22`
  sınır/flaky gibi gösterdi; runner temp-cwd olduktan sonra yaw1504 için `P21`
  iki PASS, `P22` üç PASS / bir FAIL, `P23` iki FAIL verdi. Aynı `P23`
  yaw1502 ve yaw1503 hold40 koşularında PASS verdi. Temiz P-only braket artık
  yaw1504 için `P21` güvenli taraf, `P22` sınır/flaky, `P23+` stabil FAIL;
  P23 için nudge eşiği yaw1503/yaw1504 arasında. Eski P20/P22 oynaklığı için
  kalıcı `eeprom.bin`/FC state sızıntısı güçlü aday. I-only
  güvenli aralık ise `I=1` civarına kadar; `I=2` 30 sn hold içinde acceptance'ı
  bozuyor.
- [x] `tools/sitl_pid_sweep_summary.py` eklendi. Bu araç diagnostics, raw motor
  UDP ve virtual RC loglarını birleştirip aktif uçuş fazındaki ilk nudge, ilk
  raw split, ilk yaw-rate/yaw-delta ve ilk attitude eşik zamanlarını raporluyor;
  I/P sweep yorumları artık bu tabloyla
  tekrar üretilebilir.
- [x] Sweep özeti erken raw motor axis-bias eşiklerini de raporlayacak şekilde
  genişletildi: 25/50/100/200/400us ilk geçiş zamanları ve aktif uçuş attitude
  eşiği tabloya eklendi.
  `0,2,0-repeat1` aynı unified runner ile tekrar FAIL verdi; iki I2 koşusunda
  25us axis drift nudge'dan yaklaşık `16.6-16.7 s`, raw split `17.35-17.52 s`,
  attitude eşiği `19.16-19.19 s` sonra geldi.
- [x] I2 nudge ölçekleme başladı:
  - yaw `1502`, hold30 PASS: raw spread `0.765`, max attitude `0.5/0.2`.
  - yaw `1503`, hold30 FAIL; raw split `21.656 s` sonra ama aktif attitude eşiği
    yok, post-disarm attitude büyüyor.
  - yaw `1503`, hold40 FAIL; raw split `21.781 s`, aktif attitude eşiği
    `28.257 s`. Yorum: genlik azalınca bozulum kaybolmuyor, gecikiyor.
- [x] P-only repeat braket güncellendi:
  - `P19` iki koşuda PASS: raw spread `0.760` ve `0.730`.
  - `P20` üç koşuda `1 FAIL / 2 PASS`; ilk fail raw split `2.943 s`, iki repeat
    raw spread `~0.77` ile PASS. Bu nokta stabil fail değil, sınır/flaky bölge.
  - `P20 hold40` PASS: raw spread `0.795`, max attitude `0.9/1.6`, altitude gain
    `31.556`. Uzun hold süresi tek başına P20'yi kırmadı.
  - `P21` temp-cwd runner ile iki koşuda PASS: raw spread `0.839/0.821`, max
    attitude `4.6/0.5` ve `0.2/0.3`. Bu nokta temiz başlangıçta güvenli tarafta.
  - `P22` repo-root cwd koşularında bir PASS/bir FAIL verdi; repeat raw split
    `2.759 s`, active attitude `7.532 s`. Temp-cwd runner sonrası `P22`
    `3 PASS / 1 FAIL`: PASS raw spread `0.836/0.847/0.846`, son PASS
    `logs/sitl/20260630-yaw1504-yawpid-22-0-0-tempcwd4-*`; FAIL raw split
    `2.832 s`, active attitude `8.200 s`, first large axis
    `yaw_cw_minus_ccw ~= -381 us`. Bu, P22'nin temiz cwd'de bile sınır/flaky
    olduğunu gösteriyor.
  - `P23` repo-root cwd'de iki koşuda FAIL; temp-cwd'de de iki FAIL: raw split
    `2.721/2.396 s`, active attitude `8.726/5.155 s`, first large axis
    `yaw_cw_minus_ccw ~= -386..-393 us`.
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
  - `P25` iki koşuda FAIL; raw split `2.396-2.402 s`, active attitude
    `6.831-7.300 s`. Güncel temiz-cwd P-only braket: yaw1504 için `P=21` PASS,
    `P=22` sınır/flaky (`3 PASS / 1 FAIL`), `P=23+` stabil FAIL; yaw1502 ve
    yaw1503'te `P=23` hold40 PASS. Eski P20/P22 flakiness'i kalıcı FC
    state/başlangıç sızıntısı ile açıklanabilir ama tek açıklama değil.
- [x] Sweep özeti nudge'a en yakın, nudge öncesi ve nudge sonrası diagnostic
  sample'ları da raporlayacak şekilde genişletildi. P20/P22 PASS ve FAIL
  koşularında nudge anı roll/pitch/yaw temiz ve benzer; after-nudge altitude
  yaklaşık `6.06-6.27 m`. Flakiness nudge anındaki bariz attitude/yaw veya
  irtifa farkıyla açıklanmıyor.
- [x] Betaflight debug görünürlüğü için ilk altyapı eklendi:
  `tools/sitl_diagnostics.py` ve `tools/sitl_dashboard.py` artık
  `MSP_ADVANCED_CONFIG` + `MSP_DEBUG` okuyor. Yeni diagnostic/dashboard
  sample'larında `msp.advanced_config.debug_mode`, `msp.debug_mode` ve
  `msp.debug` alanları var. Bu, `debug_mode` seçildikten sonra yaw setpoint /
  feedforward / PID loop iç sinyalini aynı runner loglarına bağlayacak kapı.
- [x] Betaflight `debug_mode` seçimi otomatikleştirildi:
  `tools/sitl_debug_config.py --debug-mode PIDLOOP|ANGLERATE|ANGLE_TARGET|N`
  mevcut advanced config alanlarını koruyup sadece `debug_mode` değiştiriyor.
  `tools/sitl_virtual_takeoff_check.py --debug-mode ...` Betaflight'i runner
  başlatıyorsa ayarı kaydedip aynı temp-cwd'den restart ediyor; `MSP_DEBUG`
  değerleri diagnostics JSONL içinde aynı koşuya yazılıyor.
- [~] PIDLOOP/ANGLERATE ile yapılan ilk restart'lı koşularda `debug_mode`
  doğrulandı; ardından `tools/sitl_debug_calibration.py` eklendi. Bu araç
  temp-cwd Betaflight'i save+restart ile başlatıp fixed virtual RC altında
  `MSP_RC` + `MSP_DEBUG` kalibrasyonu yapıyor.
- [~] Kalibrasyon sonucu: `PIDLOOP` okunabilir ama yaw PID değil loop timing;
  `ANGLERATE` bu setup'ta sıfır; `ANGLE_TARGET` P23/yaw1504 Gazebo koşusunda
  `debug[3]` verdi ama yaw setpoint slotu `debug[2]` sıfır kaldı. Sıradaki
  debug kapısı Blackbox veya Betaflight tarafına doğrudan instrumentation.
- [x] Doğrudan Betaflight instrumentation için
  `tools/betaflight_yaw_debug_patch.py` eklendi. Patch `DEBUG_AC_ERROR`
  modunda yaw setpoint/gyro/error/P/I/F/S/Sum değerlerini `MSP_DEBUG[0..7]`
  içine koyuyor ve marker ile geri alınabiliyor.
- [x] Instrumented P21/P23 yaw1504 karşılaştırması tamamlandı. P21
  `logs/sitl/20260630-debug-acerror-instrumented-yaw1504-p21-*` PASS verdi:
  altitude gain `31.686 m`, max roll/pitch `0.8/0.6`, raw spread `0.802`;
  aktif uçuşta debug max `setpoint=1`, `gyro=0`, `error=1`, `P/Sum=0`. P23
  `logs/sitl/20260630-debug-acerror-instrumented-yaw1504-p23-*` FAIL verdi:
  raw split nudge'dan `0.399 s` sonra, aktif debug max `setpoint=1`,
  `gyro/error=610`, `P/Sum=450`, raw spread `945`, attitude FAIL. Sınır artık
  genel RC/Gazebo değil, yaw rate feedback/P eşiği.
- [x] Instrumented P22/yaw1504 sınır davranışı tekrarlandı. İlk P22 koşusu
  `logs/sitl/20260630-debug-acerror-instrumented-yaw1504-p22-*` FAIL verdi:
  raw split `2.776 s`, attitude `7.668 s`, aktif debug max `gyro/error`
  `1740/1741`, `P/Sum=1243`. Repeat
  `logs/sitl/20260630-debug-acerror-instrumented-yaw1504-p22-repeat1-*` PASS:
  altitude gain `31.654 m`, max roll/pitch `4.4/3.7`, raw spread `0.843`,
  aktif debug `gyro=0`, `error=1`, `P/Sum=0`. P22 bu yüzden güvenli tune değil;
  P21/P23 arasında koşu-to-koşu başlangıç farklarına hassas metastable eşik.
- [x] P23 runaway için fizik/feedback A/B kapıları koşuldu; dar Gazebo
  probları acceptance açmadı ama Betaflight yaw authority profili açtı.
  `--iris-yaw-gyro-scale 0.5` ve `0.25` aktif yaw P/Sum'u düşürdü
  fakat fail'i pitch/attitude tarafına taşıdı:
  `logs/sitl/20260630-debug-acerror-yawscale05-yaw1504-p23-*`,
  `logs/sitl/20260630-debug-acerror-yawscale025-yaw1504-p23-*`.
  `--iris-rotor-vel-p-gain 0.01` raw split'i geciktirdi ama FAIL:
  `logs/sitl/20260630-debug-acerror-rotorp001-yaw1504-p23-*`.
  `--max-step-size 0.001` de FAIL:
  `logs/sitl/20260630-debug-acerror-step001-yaw1504-p23-*`.
  Buna karşılık aynı `P23/yaw1504` koşusu `yaw_rc_rate=5`,
  `yaw_rate=30`, `yaw_rate_limit=120` ile iki kez PASS verdi:
  `logs/sitl/20260630-debug-acerror-yawauth-r5-s30-l120-yaw1504-p23-*`
  ve repeat1. Baseline P23 raw spread `945` ve attitude FAIL iken safe yaw
  authority koşularında altitude gain `31.395/31.662 m`, max raw spread
  `0.616/0.614`, aktif debug sıfır rejimde kaldı. Runner'da bu ölçülmüş profil
  artık `--safe-yaw-authority` ile uygulanabiliyor; fiziksel RC hâlâ izole.
- [~] Claude güncellemeleri tekrar kontrol edildi. `/home/gz/aeroloop_gazebo`
  kaynak SDF identity rotor listesi ve plugin yaw gyro `Z` as-is yorumu mevcut
  motor-map/yaw-sign hükmümüzle uyumlu. P21 instrumented koşusundaki
  GYROPID_SYNC/MSP-ready takılmasını azaltmak için
  `tools/sitl_virtual_takeoff_check.py` artık Betaflight start ve debug restart
  sonrası MSP-ready beklemeden önce Gazebo plugin'ine kısa sıfır-motor
  bootstrap'i gönderiyor. İlk P21 tekrarında restart sonrası erken süreç çıkışı
  sürdüğü için `--betaflight-restart-settle-seconds` eklendi; varsayılan
  2 saniye bekleme ile P21 instrumented PASS koşusu tamamlandı. Hedefli testler
  bootstrap ve summary aktif-debug helper'larını kapsıyor.

Virtual RC ile ilk Gazebo kabul sırası:

1. Tüm Gazebo/Betaflight/Kenet süreçlerini kapat.
2. Gazebo'yu headless, `0.0025`, motor-map fix ve IMU pose fix ile başlat.
3. Betaflight SITL'i temiz başlangıçla başlat.
4. `sitl_virtual_rc.py --script takeoff --throttle 1520..1600 --mode-pwm 1500 --send`
   ile kontrollü sanal kalkış denemesi yap.
5. Aynı anda diagnostics/analyzer ile `MSP_RC`, `MSP_MOTOR`, `MSP_ATTITUDE`,
   arming blocker, RTF spread, FDM sign ve closed-loop motor output kanıtlarını
   kaydet.
6. Baseline takla varsa fiziksel kumanda elenir; şüphe
   motor/moment/IMU/Betaflight-Gazebo kuplajında kalır. 2026-06-29
   koşusunda bu oldu ve sonraki A/B test yaw gyro sign patch'i ile kapandı.

2026-06-29 ölçülen virtual RC sonuçları:

```text
Standalone Betaflight:
  log: logs/sitl/20260629-211401-diagnostics.jsonl
  CH3/CH5/CH7 expected -> MSP_RC delta: 0

Gazebo pre-arm:
  log: logs/sitl/20260629-211453-diagnostics.jsonl
  Gazebo RTF mean: 1.000
  arming blockers: -
  attitude: roll=0 pitch=0

Gazebo ARM low:
  log: logs/sitl/20260629-212220-diagnostics.jsonl
  active modes: ARM, ANGLE
  motors: 1055..1055
  attitude: roll=0 pitch=0

Gazebo throttle 1520 hold:
  log: logs/sitl/20260629-212235-diagnostics.jsonl
  active modes: ARM, ANGLE
  motors: 1521..1521
  attitude: roll=0 pitch=0

Gazebo takeoff script throttle 1550:
  log: logs/sitl/20260629-212308-diagnostics.jsonl
  result: FAIL / flip reproduced with virtual RC
  first attitude >=60: sample 19, roll=72.6 pitch=30.8
  first flip RC: MSP RPYT=[1500,1500,1500,1156], AUX1=2000, AUX3=1500
  first flip motors: [2000,1176,1055,1277]
  max attitude: roll=-180.0 pitch=0.0
  analyzer: max motor spread 945 us

Gazebo direct motor moment probe:
  log: logs/sitl/20260629-213502-motor-moment.jsonl
  result: plugin moment signs symmetric/sane at low pulse
  motor0 delta: roll=-2.91 pitch=-3.35 yaw=-8.75
  motor1 delta: roll=+2.70 pitch=+3.39 yaw=-8.75
  motor2 delta: roll=+2.97 pitch=-3.41 yaw=+8.75
  motor3 delta: roll=-2.65 pitch=+3.32 yaw=+8.75
  all_equal: roll/pitch/yaw ~0

Gazebo direct high-thrust sanity:
  log: logs/sitl/20260629-214237-motor-moment.jsonl
  result: all_equal 0.95 speed raises z by +0.061 m over 0.4 s
  yorum: Gazebo modeli lift üretebiliyor; problem sadece thrust eksikliği değil.

Gazebo virtual RC + raw UDP + diagnostics combined:
  diagnostics: logs/sitl/20260629-215325-diagnostics.jsonl
  raw UDP:     logs/sitl/20260629-215315-motor-udp.jsonl
  result: FAIL / flip reproduced with virtual RC
  max attitude: roll=180.0 pitch=68.0
  MSP_RC during hold: RPYT=[1500,1500,1500,1750], AUX1=2000, AUX3=1500
  MSP_MOTOR peak/spread examples:
    sample 42: [2000,1055,1383,1708], attitude roll=5.0 pitch=5.5
    sample 46: [1275,1055,1265,2000], attitude roll=115.5 pitch=68.0
    sample 52: [1474,1055,1683,2000], attitude roll=-180.0 pitch=0.0
  raw UDP max normalized by motor: [1.0,1.0,1.0,1.0]
  Gazebo pose after flip: roll=-180.0, pitch~0, z=0.035
  yorum: physical RC elendi; Gazebo motor momentleri temel işaret olarak
  simetrik; takla artık Betaflight-Gazebo closed-loop attitude/frame/mixer
  tepkisine indirgenmiş durumda.
```

---

## 1. Güncel Durum Özeti

2026-06-29 itibarıyla:

- [x] TBS Tango 2 USB joystick olarak okunuyor.
- [x] Roll/pitch/throttle/yaw kanalları okunuyor.
- [x] Arm için two-state switch okunuyor.
- [x] Kenet state için three-state switch okunuyor.
- [x] Betaflight SITL çalıştırıldı.
- [x] Betaflight Configurator SITL'e bağlandı.
- [x] Receiver tab'de ana kanallar görüldü.
- [x] ARM mode ayarı çalıştı.
- [x] `tools/sitl_rc_bridge.py` ile RC paketleri `9004/udp` üzerinden gönderildi.
- [x] `tools/sitl_virtual_rc.py` ile fiziksel kumanda olmadan virtual RC ve
  takeoff throttle script'i üretilebiliyor.
- [x] `tools/sitl_configure_modes.py` ile Betaflight SITL mode range'leri
  Configurator açmadan uygulanabiliyor.
- [x] Virtual RC + Betaflight standalone `MSP_RC` doğrulamasında max RC delta
  `0` ölçüldü.
- [x] Virtual RC + Gazebo Iris retest'i koşuldu; `--fix-iris-motor-map` ve
  `--fix-iris-imu-pose` aktifken `1550` scripted takeoff'ta roll `-180`
  takla tekrarlandı.
- [x] Direct motor moment probe, doğru BF SITL remap'iyle Gazebo plugin
  motor->moment işaretlerinin simetrik ve eksen olarak ayrışmış olduğunu gösterdi.
- [x] Raw UDP motor probe, aynı virtual RC koşusunda Betaflight'in Gazebo'ya
  gerçekten yüksek ve asimetrik motor komutları gönderdiğini doğruladı.
- [x] FDM probe, Gazebo IMU yaw rate ile Betaflight'a giden FDM yaw gyro
  işaretinin beklenen ilişkiyi verdiğini gösterdi.
- [~] `yaw_motors_reversed` ON/OFF A/B denendi; ON koşusu yaw nudge'a kadar
  daha sakin olsa da direct yaw 1504 acceptance geçmedi.
- [~] Sıradaki kök neden adayı statik motor map veya FDM yaw sign değil;
  Betaflight'in closed-loop sırasında ürettiği actual motor UDP çıktısının
  slot/moment karşılığı, yaw hold/rate cevabı veya runaway koruması.
- [x] `tools/kenet_sitl_mixer.py` ile TRACKING sırasında pitch/yaw otomatik geldi.
- [x] Roll/throttle pilotta bırakıldı.
- [x] Yanlış Python/OpenCV ortamında tracker yoksa mixer artık çökmeden pilot
  passthrough yapıyor.
- [~] İkinci fiziksel three-state switch Tango 2 USB joystick çıktısında henüz
  görünmüyor; bu kısım Tango 2 model/mixer ayarına bırakıldı.
- [x] MSP transport SITL'e native TCP ile bağlandı; canlı Betaflight MSP smoke,
  `MSP_SET_RAW_RC` readback ve production `kenet.py --msp-tcp` P11 smoke geçti.
  Configurator kabul koşusu artık şart değil; mode/status ve mixer sözleşmesi
  CLI gate'leriyle doğrulanıyor.
- [~] Gazebo portable launcher, env check, headless plugin smoke test ve
  virtual RC retest tamamlandı. Fiziksel kumanda artık bu takla için ana şüphe
  değil; sıradaki dal motor/moment/IMU/Betaflight-Gazebo kuplajı.

Claude notu sonrası Codex kritik değerlendirmesi:

- [x] SITL mixer ile ana `TrackingPipeline` state machine davranışı aynı hale
  getirildi; state helper, target-loss drop ve boundary testleri production/SITL
  driftini kilitliyor.
- [x] Kenet state default kanalı tekilleştirildi (`CH6 / AUX2 / index 5`).
- [x] `tools/` SITL güvenlik davranışları testlerle sabitlendi.
- [x] RC/mapping gözlem araçlarında tek kaynak prensibine geçildi.
- [~] Gazebo/Betaflight takla analizinde ANGLE mode/CH7 aktifliği,
  zamanlama, motor order/direction ve attitude logları birlikte
  değerlendirildi. `tools/sitl_configure_modes.py` sonrası ARM+ANGLE aktif,
  virtual RC delta `0`, Gazebo RTF `~1.0`. Buna rağmen `1550` scripted
  takeoff'ta roll `-180` oldu. Bu, fiziksel kumanda yerine motor/moment/IMU
  veya Betaflight-Gazebo kuplajını sıradaki ana dal yapıyor.

---

## 2. Bağımlılık Politikası

İlk aşamada kullanılacak minimumlar:

- Python stdlib joystick/UDP araçları
- Proje venv'i: `$FPV_ROOT/fpv_env`
- OpenCV contrib tracker desteği
- Betaflight SITL binary
- Betaflight Configurator
- Gerekirse `websockify`

İlk aşamada kaçınılacaklar:

- Gazebo / Ignition / Harmonic
- aeroloop / betaloop plugin zinciri
- ROS / PX4 / ArduPilot entegrasyonları
- Python tarafında gereksiz yeni paketler
- Otomatik kalibrasyon UI'ları

Kural: Yeni bağımlılık ancak bir aşamanın kabul kriteri için gerçekten gerekirse
eklenecek.

---

## 3. Kanal ve Switch Politikası

Betaflight kanal sırası AETR kabul edilir:

| İşlev | Kanal | Kaynak | Durum |
|---|---:|---|---|
| Roll | CH1 | Axis 0 | [x] |
| Pitch | CH2 | Axis 1, invert | [x] |
| Throttle | CH3 | Axis 2 | [x] |
| Yaw | CH4 | Axis 3 | [x] |
| ARM | CH5 / AUX1 | Axis 4, two-state | [x] |
| Kenet state | CH6 / AUX2 | Axis 6, three-state | [x] |
| Autopilot mode | CH7 / AUX3 | Axis 5, three-state | [~] |

Kenet state anlamı:

| AUX2 / CH6 | Kenet state | Davranış |
|---:|---|---|
| 1000 | IDLE | Pilot tam kontrolde |
| 1500 | AI-ARMED | Kamera hazır, pilot kontrolde |
| 2000 | TRACKING | Kenet pitch/yaw üretebilir |

Önemli tasarım kararı:

- `AUX1 / CH5`: Betaflight ARM için kullanılacak.
- `AUX2 / CH6`: Betaflight mode'a bağlanmayacak; Kenet state için ayrılacak.
- `AUX3 / CH7`: İleride Betaflight flight mode için kullanılabilir.

Codex kritik notu:

Ana `kenet.py` / `PipelineConfig` default'u ile SITL mapping'i aynı olmalı.
Kenet state için `CH6 / AUX2 / index 5` tek default olarak seçildi. Ana
`kenet.py`, `PipelineConfig`, SITL mixer ve dashboard bu kanalı kullanacak
şekilde senkronlanmıştır. Eski AUX4 setup kullanan denemelerde açıkça
`--aux-ch 7` verilmelidir.

---

## 4. Aşama P0 - Ortam ve Proje Venv

Amaç: Her komutun doğru Python ve doğru OpenCV ile çalıştığından emin olmak.

Durum:

- [x] `fpv_env` oluşturuldu.
- [x] `opencv-contrib-python` kurulu.
- [x] `pytest` çalışıyor.
- [x] `fpv-test` alias eklendi.
- [x] `eeprom.bin`, `fpv_env/`, `.claude/`, `.codex/`, `.agents/`, videolar
  `.gitignore` kapsamında tutuluyor.

Kontrol komutları:

```bash
fpv-test
which python
python -c "import cv2; print(cv2.__file__, hasattr(cv2, 'TrackerCSRT_create'))"
python -m pytest -q
```

Beklenen:

```text
$FPV_ROOT/fpv_env/bin/python
True
95 passed
```

Sorun belirtisi:

```text
TrackerCSRT is unavailable in this OpenCV build
```

Anlamı: Büyük ihtimalle yanlış venv aktiftir. Doğru ortam:

```bash
source "$FPV_ROOT/fpv_env/bin/activate"
```

Kabul kriteri:

- [x] `python` proje venv'ini gösteriyor.
- [x] CSRT tracker mevcut.
- [x] Testler geçiyor.

---

## 5. Aşama P1 - Kumanda / Joystick Doğrulama

Amaç: RC kumanda bilgisayardan okunuyor mu, hangi axis hangi kanala karşılık
geliyor, bunu paket bağımlılığı olmadan görmek.

Bilinen cihaz:

```text
${JOY_DEV}
/dev/input/by-id/usb-Team-BlackSheep_TBS_Joystick_00000000001B-joystick
```

Araçlar:

```text
tools/sitl_rc_bridge.py
tools/rc_monitor.py
tools/state_monitor.py
tools/sitl_dashboard.py
```

Komutlar:

```bash
python tools/sitl_rc_bridge.py --device "$JOY_DEV" --dry-run
python tools/sitl_rc_bridge.py --device "$JOY_DEV" --changes
python tools/sitl_physical_rc_preflight.py --device "$JOY_DEV"
python tools/rc_monitor.py --device "$JOY_DEV"
python tools/sitl_dashboard.py --open
python tools/state_monitor.py --open  # compatibility shim -> sitl_dashboard.py
```

Kabul kriteri:

- [x] CH1-CH4 stick hareketleri 1000-2000 aralığında görülüyor.
- [x] Throttle düşükte 1000, yüksekte 2000.
- [x] Roll/pitch/yaw merkezde 1500.
- [x] Axis 4 ARM için 1000/2000 üretiyor.
- [x] Axis 6 Kenet state için 1000/1500/2000 üretiyor.
- [x] Bench checklist no-send preflight komutuna alındı:
  `tools/sitl_physical_rc_preflight.py` CH5 ARM, CH6 Kenet state ve CH7
  autopilot/ANGLE ayrımını raporlar; CH7 yoksa `--force-mode-pwm 1500`
  forced ANGLE profili olarak kaydedilir.
- [~] İkinci fiziksel three-state switch USB joystick'te görünür hale getirilecek.

Tango 2 notu:

İkinci fiziksel three-state switch hareket ettirildiğinde
`tools/sitl_rc_bridge.py --changes` hiçbir çıktı üretmiyorsa, Python tarafında
yapılacak bir şey yoktur. Switch önce Tango 2 model/mixer ayarlarında bir
kanala atanmalı ve USB joystick çıktısına düşmelidir. Detay notu:

```text
TBS-Tango-2-configuration-notes.md
```

---

## 6. Aşama P2 - Betaflight SITL ve Configurator

Amaç: Betaflight SITL'i çalıştırmak ve Configurator ile bağlantıyı görmek.

Mevcut gözlem:

```text
$BETAFLIGHT_ROOT/obj/main/betaflight_SITL.elf
```

SITL çalıştırma:

```bash
"$BETAFLIGHT_ROOT/obj/main/betaflight_SITL.elf"
```

Beklenen portlar:

| Port | Protokol | Anlam |
|---:|---|---|
| 9004 | UDP | RC input |
| 5761 | TCP | SITL UART / MSP |
| 9002 | UDP | Gazebo motor/sim çıkışı |
| 9001 | UDP | RealFlight bridge çıkışı |

Configurator bağlantısı:

```bash
websockify 127.0.0.1:6761 127.0.0.1:5761
```

Configurator port:

```text
ws://127.0.0.1:6761
```

Not:

Raw `tcp://127.0.0.1:5761` bağlantısı tarayıcı tabanlı Configurator'da takılabilir.
Websocket köprüsü bu yüzden daha pratik.

Kabul kriteri:

- [x] SITL binary çalışıyor.
- [x] SITL logunda `start UDP server for RC input @9004` görülüyor.
- [x] Configurator bağlanıyor.
- [x] Receiver tab açılıyor.
- [x] Modes tab üzerinden ARM ayarı yapılabiliyor.

---

## 7. Aşama P3 - Minimal RC UDP Bridge

Amaç: Kumandadan okunan değerleri Betaflight SITL'e gerçek RC input gibi
göndermek.

Dosya:

```text
tools/sitl_rc_bridge.py
```

Betaflight SITL packet formatı:

```c
typedef struct {
    double timestamp;
    uint16_t channels[16];
} rc_packet;
```

Python karşılığı:

```python
struct.pack("<d16H", time.time(), *channels)
```

Komutlar:

```bash
python tools/sitl_rc_bridge.py --device "$JOY_DEV" --dry-run
python tools/sitl_rc_bridge.py --device "$JOY_DEV" --send --verbose
python tools/sitl_rc_bridge.py --device "$JOY_DEV" --send --once --verbose
```

Kabul kriteri:

- [x] Dry-run kanal değerleri doğru.
- [x] `--send` ile 40 baytlık RC packet üretiliyor.
- [x] SITL logunda `new rc 40` görülüyor.
- [x] Configurator Receiver tab'de CH1-CH6 hareket ediyor.
- [~] CH7/AUX3 ikinci switch Tango 2 tarafı çözülünce canlı doğrulanacak.

Debug komutu:

```bash
python tools/sitl_rc_probe.py
```

Not: `tools/sitl_rc_probe.py` çalışırken Configurator kapalı olmalı; SITL'in
MSP/TCP portu aynı anda tek client kabul eder.

---

## 8. Aşama P4 - Betaflight Mode ve Güvenlik Ayarı

Amaç: Betaflight tarafında hangi AUX kanalının ne işe yaradığını netleştirmek.

Önerilen mode düzeni:

| Betaflight işlevi | Kanal | Range | Durum |
|---|---:|---|---|
| ARM | AUX1 / CH5 | 1700-2100 | [x] |
| ANGLE | AUX3 / CH7 | 1300-1700 | [x] |
| HORIZON | AUX3 / CH7 | 1700-2100 | [x] |
| Kenet state | AUX2 / CH6 | Betaflight mode yok | [x] |

Kritik not:

`AUX2 / CH6` Betaflight mode'a bağlanırsa Kenet state switch'i ile autopilot
mode switch'i birbirine karışır. Bu yüzden AUX2 sadece Kenet state için
ayrılacak.

Tango 2 üzerindeki ikinci three-state switch henüz güvenilir değilse CH7/AUX3
test için yazılımdan sabitlenebilir:

```bash
python tools/kenet_sitl_mixer.py --device "$JOY_DEV" --camera test-2.mp4 \
  --send --force-mode-pwm 1500 --print-hz 5
```

Bu komutta CH7/AUX3 `1500` olur. Configurator Modes tab'inde ANGLE range
`AUX3 1300-1700` ise stabilized takeoff retest'i yapılabilir.

Kabul kriteri:

- [x] ARM switch high konumunda Betaflight ARM mode aktif oluyor.
- [x] ARM switch low konumunda ARM mode pasif oluyor.
- [x] CH7/AUX3 flight mode mapping'i Configurator açmadan
  `tools/sitl_mode_status_check.py` ile doğrulanabilir: ANGLE mid,
  HORIZON high.
- [x] Failsafe/arming flags Configurator yerine `MSP_STATUS_EX` üzerinden
  `tools/sitl_mode_status_check.py` çıktısına yazılır.

---

## 9. Aşama P5 - Kenet SITL Mixer / RC Arbiter

Amaç: Pilot kumanda girdisi ile Kenet görsel takip çıktısını tek bir SITL RC
paketinde birleştirmek.

Dosya:

```text
tools/kenet_sitl_mixer.py
```

Bu aşama MSP kullanmaz. Betaflight SITL'e yine `9004/udp` üzerinden RC packet
gönderir. Böylece Kenet'in PID davranışı, MSP karmaşıklığı olmadan hızlıca
test edilir.

Karışım kuralı:

| Kenet state | Target | Roll | Pitch | Throttle | Yaw | AUX |
|---|---|---|---|---|---|---|
| IDLE | fark etmez | pilot | pilot | pilot | pilot | pilot |
| AI-ARMED | fark etmez | pilot | pilot | pilot | pilot | pilot |
| TRACKING | yok | pilot | pilot | pilot | pilot | pilot |
| TRACKING | var | pilot | Kenet | pilot | Kenet | pilot |

Komutlar:

```bash
python tools/kenet_sitl_mixer.py --device "$JOY_DEV" --no-vision --duration 2
python tools/kenet_sitl_mixer.py --device "$JOY_DEV" --camera test-2.mp4 --duration 2
python tools/kenet_sitl_mixer.py --device "$JOY_DEV" --camera test-2.mp4 --send --print-hz 5
```

Geçici masaüstü test kolaylığı:

```bash
python tools/kenet_sitl_mixer.py --device "$JOY_DEV" --camera test-2.mp4 \
  --aux-track-threshold 1400
```

Beklenen çıktı örnekleri:

```text
state=AI-ARMED source=pilot target=none ...
state=TRACKING source=kenet target=found pilot=... final=...
state=TRACKING source=pilot-target-lost target=none ...
source=pilot-tracker-unavailable
```

Kabul kriteri:

- [x] IDLE/AI-ARMED durumlarında `pilot` ve `final` aynı.
- [x] TRACKING durumunda hedef varsa pitch/yaw Kenet'ten geliyor.
- [x] TRACKING durumunda roll/throttle pilotta kalıyor.
- [x] Configurator Receiver tab'de TRACKING sırasında pitch/yaw otomatik geldi.
- [x] Tracker yoksa mixer crash yerine pilot passthrough yapıyor.
- [x] PID yönleri bilinçli hedef hareketleriyle tek tek doğrulandı:
  `tools/sitl_pid_direction_check.py`.

Bu aşamada kanıtlanan şey:

```text
Kenet görsel takip + PID kontrol çıktısı Betaflight SITL Receiver input'una
ulaşabiliyor.
```

Bu aşamada henüz kanıtlanmayan şey:

```text
Gerçek FC üzerindeki MSP_SET_RAW_RC davranışı.
```

---

## 10. Aşama P5.5 - State Machine Parity ve SITL Safety Tests

Amaç: SITL mixer'ın gerçek Kenet pipeline'ından farklı bir sistem gibi
davranmasını engellemek. SITL testinin değeri, üretim state machine'iyle aynı
kuralları kullanmasına bağlıdır.

Claude notunda haklı bulunan kritik drift alanları:

- Ana pipeline AUX threshold karşılaştırmaları ile mixer karşılaştırmaları
  boundary değerlerde ayrışabilir.
- Hedef kaybında ana pipeline `TRACKING -> AI-ARMED` yaparken mixer otomatik
  merkez bbox re-init davranışına kayabilir.
- Mixer bazı private alanlara erişiyor: `tracker._initialized`,
  `controller._cx/_cy`.
- Bu logic unit testlerle sabitlendi; production/SITL state helper ve mixer
  sözleşmesi aynı kabul kapılarına bağlı.

Plan:

- [x] AUX→state kararını ortak helper/modüle taşı.
- [x] Hedef kaybı davranışını ana pipeline ve SITL mixer arasında aynı yap.
- [x] Re-init için pilot switch niyeti gerekiyor mu, tek tasarım kararı olarak
  yaz.
- [x] `tracker.is_initialized` public property ekle.
- [x] `controller.set_frame_center(width, height)` public method ekle.
- [x] Mixer shutdown sırasında güvenli son RC frame'i gönder.
- [x] RC send interval watchdog warning ekle.

Re-init kararı:

```text
Hedef uzun süre kaybolursa TRACKING -> AI-ARMED yapılır.
Switch hâlâ HIGH ise bir sonraki state değerlendirmesinde tekrar TRACKING'e
geçilebilir; ancak aynı loop içinde sessiz merkez bbox re-init yapılmaz.
```

Testler:

- [x] IDLE/AI-ARMED durumunda final RC tüm kanallarda pilotla aynı.
- [x] TRACKING + target found durumunda sadece pitch/yaw değişiyor.
- [x] TRACKING + target lost durumunda final RC pilot passthrough.
- [x] Tracker unavailable durumunda final RC pilot passthrough.
- [x] AUX threshold boundary testleri: 1299, 1300, 1699, 1700.
- [x] `pack_rc_packet` 40 byte ve `<d16H` format kontratı.
- [x] `axis_to_rc`, `axis_to_three_pos_rc`, `clamp_rc`, `make_channels`
  boundary ve invert testleri.

Kabul kriteri:

- [x] SITL mixer ve ana pipeline state geçişleri aynı dokümante edilmiş kurala
  bağlı.
- [x] Hedef kaybı davranışı masaüstü ve gerçek yol için aynı.
- [x] `pytest` içinde `tools/` safety glue testleri var.
- [x] Bu aşama tamamlanmadan MSP transport "uçuş davranışı doğru" kabul
  edilmeyecek. Bu gate kapandı: state parity, target-loss passthrough ve
  only-pitch/yaw override testleri `pytest` + `sitl_mixer_matrix_check.py` ile
  doğrulanıyor.

---

## 11. Aşama P6 - Tekrarlanabilir Test Matrisi

Amaç: "Çalıştı" demek yerine hangi durumda hangi kanalın ne yaptığını net
not etmek.

Test matrisi:

| Test | Giriş | Beklenen |
|---|---|---|
| P6.1 | AUX2 LOW | `state=IDLE`, tüm kanallar pilot |
| P6.2 | AUX2 MID | `state=AI-ARMED`, tüm kanallar pilot |
| P6.3 | AUX2 HIGH, hedef yok | `pilot-target-lost`, tüm kanallar pilot |
| P6.4 | AUX2 HIGH, hedef var | pitch/yaw Kenet, diğerleri pilot |
| P6.5 | ARM low/high | CH5 1000/2000 ve Betaflight ARM mode doğru |
| P6.6 | Throttle oynat | TRACKING sırasında throttle hâlâ pilotta |
| P6.7 | Roll oynat | TRACKING sırasında roll hâlâ pilotta |
| P6.8 | Hedef kaybı | Kenet komutu bırakır, pilot pitch/yaw geri gelir |

Kayıt yöntemi:

- Donanımsız regression için `tools/sitl_mixer_matrix_check.py` koşulur.
- Betaflight mode/arming tarafı için `tools/sitl_mode_status_check.py` koşulur.
- Fiziksel RC geri geldiğinde aynı matris `sitl_physical_rc_preflight.py` ve
  external checker ile tekrar izlenir.

Kabul kriteri:

- [x] Test matrisi bir kez baştan sona tamamlandı:
  `tools/sitl_mixer_matrix_check.py` P6.1-P6.8 PASS.
- [x] Anormal kanal davranışı varsa hangi testte olduğu not edilecek yapı hazır;
  güncel matrix koşusunda anormal davranış yok.
- [x] PID yönleri doğru/ters şeklinde işaretlendi; controller direction gate
  PASS, Receiver/Gazebo offset acceptance ayrı.

---

## 12. Aşama P7 - PID Yön ve Limit Doğrulaması

Amaç: Pitch/yaw komutlarının sadece geldiğini değil, doğru yönde geldiğini
kanıtlamak.

Kontrol edilecekler:

- Hedef görüntünün sağındaysa yaw komutu doğru yöne gidiyor mu?
- Hedef görüntünün solundaysa yaw komutu tersine dönüyor mu?
- Hedef küçük/büyük görünüyorsa pitch/forward komutu beklenen yönde mi?
- Komutlar limitlerde kalıyor mu?
- Hedef kaybolunca integral birikimi sıfırlanıyor mu?

Komut parametreleri:

```bash
python tools/sitl_pid_direction_check.py

python tools/kenet_sitl_mixer.py --camera test-2.mp4 --send \
  --yaw-kp 0.8 --yaw-ki 0.05 --yaw-kd 0.15 \
  --forward-kp 0.4 --forward-ki 0.02 --forward-kd 0.1
```

2026-06-30 hızlı controller/PID direction gate:

```text
tools/sitl_pid_direction_check.py -> PASS
target_right: yaw > 1500
target_left: yaw < 1500
target_small: pitch > 1500
target_large: pitch < 1500
centered / target_lost: pitch,yaw = 1500
roll/throttle sabit: 1500/1500
```

Kabul kriteri:

- [x] Yaw yönü doğru: target_right `1548`, target_left `1452`.
- [x] Pitch/forward yönü doğru: target_small `1514`, target_large `1486`.
- [x] Komutlar 1000-2000 dışına taşmıyor; direction gate ve controller clamp
  aynı üretim `FlightController` yolunu kullanıyor.
- [x] Hedef kaybında Kenet pilotu bloke etmiyor: direction gate neutral,
  P11 production target-loss gate yeni MSP frame göndermiyor.
- [~] İlk güvenli PID değerleri dokümante edildi: controller yönleri doğru,
  fakat Gazebo/Betaflight offsetli target-found acceptance hâlâ yaw feedback
  sınırına bağlı; gerçek uçuş PID tune kabulü değil.

---

## 13. Aşama P8 - Loglama ve GCS ile İzleme

Amaç: Test sırasında sadece Configurator'a bakmak yerine Kenet tarafındaki
state, target ve kanal kararlarını da kayıt altına almak.

Mevcut araçlar:

```text
tools/sitl_dashboard.py
tools/rc_monitor.py
tools/state_monitor.py  # compatibility shim
kenet/gcs.py
```

Plan:

- [x] `kenet_sitl_mixer.py` için JSONL flight log eklendi.
- [x] Her satırda zaman, state, source, target, pilot RC, final RC yazılıyor.
- [x] Dashboard MSP/Betaflight snapshot logu eklendi.
- [x] Bir test koşusunun çıktısı `tools/analyze_sitl_log.py` ile kısa
  raporlanabilir hale geldi.
- [x] Dashboard'a takla/kopma anı için `Mark Event` log işareti eklendi.

Basit log formatı önerisi:

```text
timestamp,state,source,target_found,pilot_ch1..pilot_ch8,final_ch1..final_ch8
```

Kabul kriteri:

- [x] Bir test koşusu sonradan incelenebiliyor.
- [x] Hedef kaybı ve state geçişleri logdan okunabiliyor.
- [x] Configurator'da görülen davranış ile log davranışı tutarlı.
  Configurator artık zorunlu gözlem aracı değil; aynı sözleşme
  `tools/sitl_mode_status_check.py`, `tools/sitl_mixer_matrix_check.py`,
  dashboard/diagnostics JSONL ve MSP status/RC parse ile doğrulanıyor.

Claude logging notu sonrası Codex ekleri:

Log sistemi değerli ve korunmalı; ancak analiz aracı veri yokluğunu güvenli
durum gibi göstermemeli. Bu özellikle takla/kopma analizi için kritik.

- [x] Dashboard/MSP sample yoksa `tools/analyze_sitl_log.py` attitude için
  "veri yok" raporu versin; "35 derece altında kaldı" demesin.
- [x] Motor sample yoksa motor spread için "veri yok" raporu versin.
- [x] Mixer flight log default sample rate 30 Hz yerine daha düşük ve Pi/Jetson
  dostu bir değer olacak şekilde ayarlansın.
- [x] JSONL writer için flush/rotation politikası eklensin; uzun testler SD
  kartı veya disk üzerinde sınırsız büyümesin.
- [x] `CHANNEL_LABELS`, AETR sırası ve throttle neutral bilgisi
  analyzer içinde tekrar hardcode edilmesin; production mapping
  `kenet/rc_channels.py`, SITL etiket/neutral wrapper'ı
  `tools/sitl_rc_channels.py` üzerinden ortaklaştırıldı.
- [x] Mixer ve dashboard JSONL record şeması dokümante edilsin:
  `docs/sitl-jsonl-schema.md`.
- [x] Analyzer, üretici alanları eksik/değişmişse sessiz geçmek yerine uyarı
  üretsin.

---

## 14. Aşama P9 - MSP Yoluna Geçiş Kararı

Amaç: Gerçek drone yoluna yaklaşmak. Bu aşamada UDP RC packet yerine Kenet'in
gerçek sistemde kullandığı MSP mesajları test edilir.

MSP neyi değiştirir?

Şu an:

```text
Kenet SITL mixer -> UDP 9004 rc_packet -> Betaflight SITL receiver input
```

MSP aşamasında:

```text
Kenet -> MSP_SET_RAW_RC -> SITL TCP 5761 / pseudo serial -> Betaflight SITL
```

MSP'ye geçmeden önce şartlar:

- [x] RC bridge çalıştı.
- [x] Configurator Receiver tab çalıştı.
- [x] Kenet mixer TRACKING sırasında pitch/yaw üretti.
- [x] PID yönleri doğrulandı: `tools/sitl_pid_direction_check.py` PASS.
- [~] Hedef kaybı ve state geçişleri test matrisi tamamlandı: production/SITL
  unit gate, P11 AUX drop canlı smoke ve sentetik target-loss runner gate'i
  geçti; uzun hedef kaybından sonra yeniden TRACKING için pilotun switch'i
  track eşiğinin altına indirip tekrar HIGH yapması gerekiyor. Gerçek
  kamera/video target-loss matrisi ayrı koşulabilir. Canlı sentetik target-loss
  PASS: `logs/sitl/20260630-synthetic-target-loss-live-final-diagnostics.jsonl`
  ve `logs/sitl/20260630-synthetic-target-loss-live-final-mixer.jsonl`.

Karar:

```text
Bu şartlar tamamlanmadan MSP'ye geçmek gereksiz karmaşıklık yaratır.
```

---

## 15. Aşama P10 - MSP SITL Transport

Amaç: `kenet/msp.py` kodunu Betaflight SITL'in `5761/tcp` MSP hattına bağlamak.

İki sade seçenek vardı; artık Seçenek B kod/test düzeyinde uygulanmış durumda.

### Seçenek A - socat ile pseudo serial

Kod değişikliği azdır. SITL TCP portu sahte serial porta bağlanır:

```bash
socat -d -d pty,raw,echo=0,link=/tmp/bf-sitl,mode=666 tcp:127.0.0.1:5761
```

Sonra Kenet MSP portu olarak:

```text
/tmp/bf-sitl
```

kullanılır.

Artı:

- `kenet/msp.py` seri port mantığına dokunmadan denenir.

Eksi:

- `socat` ek sistem aracı gerekir.
- Configurator aynı anda bağlanamayabilir.

### Seçenek B - msp.py içine TCP transport

Kodda seri port yerine TCP socket transport eklenir:

```text
serial transport: /dev/ttyUSB0
tcp transport   : 127.0.0.1:5761
```

Güncel durum:

- [x] `kenet/msp.py` `tcp://host:port` ve `host:port` endpointlerini tanıyor.
- [x] `kenet.py --msp-tcp 127.0.0.1:5761` ana pipeline konfigürasyonuna bağlandı.
- [x] `tools/kenet_msp_smoke.py` production `MSPConnection` ile
  `MSP_API_VERSION`, `MSP_RC`, `MSP_ATTITUDE` ve opsiyonel `MSP_SET_RAW_RC`
  smoke testi yapabiliyor.
- [x] Unit testler TCP endpoint parse, socket transport wrapper ve smoke CLI
  davranışını kapsıyor.
- [x] Canlı Betaflight SITL smoke geçti:
  `./fpv_env/bin/python tools/kenet_msp_smoke.py --msp-tcp 127.0.0.1:5761 --timeout 2.0`
  -> PASS, API `1.48`, `MSP_RC` ve `MSP_ATTITUDE` okundu.
- [x] Canlı `MSP_SET_RAW_RC` readback gate geçti:
  `./fpv_env/bin/python tools/kenet_msp_smoke.py --msp-tcp 127.0.0.1:5761 --timeout 2.0 --set-raw-rc 1500,1600,1000,1400,2000,2000,1500,1500`
  -> PASS, `set_raw_rc_check: PASS`; pilot/AETR gönderim
  `[1500,1600,1000,1400,...]` Betaflight `MSP_RC` readback'inde beklenen
  RPYT düzeniyle `[1500,1600,1400,1000,...]` olarak doğrulandı.

Artı:

- Ek sistem aracı gerekmez.
- SITL için daha doğrudan çalışır.

Eksi:

- `kenet/msp.py` içinde transport ayrımı yapılmalıdır.

İlk öneri:

- [x] Seçenek A aktif kapı değil; sadece native TCP yoluna karşı
  socat/pseudo-serial regresyonu istenirse opsiyonel fallback olarak
  kullanılacak.
- [x] Seçenek B ile temiz TCP transport eklendi.

MSP testleri:

- [x] `MSP_API_VERSION` production MSP sınıfından ve canlı Betaflight SITL'den okunabiliyor.
- [x] `MSP_RC` production MSP sınıfından ve canlı Betaflight SITL'den okunabiliyor.
- [x] `MSP_ATTITUDE` production MSP sınıfından ve canlı Betaflight SITL'den okunabiliyor.
- [x] `MSP_SET_RAW_RC` production MSP sınıfından gönderilebiliyor; canlı
  Betaflight SITL `MSP_RC` readback'i ortak AETR->MSP mapping ile doğrulanıyor.
- [x] Configurator kapalıyken doğrudan MSP testleri çalışıyor.

Kabul kriteri:

- [x] Kenet production MSP transport'u canlı Betaflight SITL TCP MSP hattına bağlanabiliyor.
- [x] MSP mesajları timeout olmadan dönüyor: canlı smoke'ta API/RC/attitude okundu.
- [x] `MSP_SET_RAW_RC` ile pitch/yaw gönderilebiliyor: üretim `send_rc` yolu
  canlı readback gate ile doğrulandı.
- [x] UDP mixer ile MSP yolunun kanal davranışı aynı: pilot/AETR sözleşmesi,
  MSP readback mapping ve ana `kenet.py` merged override frame'i canlı P11
  smoke ile doğrulandı.

---

## 16. Aşama P11 - Kenet Ana Pipeline ile SITL MSP

Amaç: Araç scriptleri yerine ana Kenet pipeline'ını SITL MSP hattına bağlamak.

Hedef komut örneği:

```bash
python kenet.py --camera test-2.mp4 --port /tmp/bf-sitl --aux-ch 5 --headless
```

veya TCP transport eklenirse:

```bash
python kenet.py --camera test-2.mp4 --msp-tcp 127.0.0.1:5761 --aux-ch 5 --headless
```

Kontrol edilecek farklar:

- UDP mixer pilot + Kenet'i tek RC packet'te birleştirir.
- Ana pipeline gerçek sistemdeki MSP override mantığını kullanır.
- Override maskesi ve throttle/roll güvenliği tekrar gözden geçirilmelidir.
- 2026-06-30 canlı P11 smoke: Betaflight temp cwd + virtual RC UDP `9004`
  + ana `kenet.py --msp-tcp 127.0.0.1:5761 --aux-ch 5 --headless --no-gcs`
  koşuldu. Virtual RC frame `1500,1500,1120,1500,1000,2000,1500,1500`
  iken pipeline `TRACKING` durumuna geçti ve MSP'ye gönderilen merged frame
  `send=[1500,1515,1120,1535,1000,2000,1500,1500]` biçiminde throttle'ı
  pilot değeri `1120` olarak korudu. İlk deneme `MSP_RC` readback'inin RPYT
  olduğunu gösterdi; production dönüşüm `kenet/rc_channels.py` içine alındı
  ve pipeline artık `MSP_RC` okumasını pilot/AETR düzenine çeviriyor.
- 2026-06-30 canlı AUX drop smoke: önce CH6/AUX2 High `2000`, sonra Mid
  `1500` sanal RC ile beslendi. Logda `[TRACKING] CH6:2000 ... send=[...]`
  sonrası `[AI-ARMED] CH6:1500` görüldü ve Mid'e indikten sonra yeni `send=`
  satırı oluşmadı.

Özel güvenlik maddesi:

Gerçek sistemde throttle'ın yanlışlıkla 1500'e kilitlenmesi riskine karşı:

- [x] MSP override maskesi sadece gereken kanalları kapsıyor mu? Doküman,
  üretim `_build_override_channels()` ve P6 matrix kuralı pitch/yaw-only:
  `msp_override_channels = 10`. Gerçek FC CLI config okuma ayrı bench gate.
- [x] Roll/throttle pilotta kalıyor mu? Outbound MSP frame son pilot RC'den
  türetiliyor ve sadece pitch/yaw Kenet'ten geliyor; canlı P11 smoke'ta
  throttle `1120` korunarak doğrulandı.
- [x] TRACKING düşerse pilot anında geri geliyor mu? Canlı AUX High -> Mid
  smoke'ta MSP override gönderimi kesildi.
- [x] Hedef kaybında override bırakılıyor mu? Production unit gate:
  `TrackingPipeline._track_control_step()` `TrackResult(found=False)` için
  yeni `MSP_SET_RAW_RC` göndermiyor; uzun kayıpta `TRACKING -> AI-ARMED`.

Kabul kriteri:

- [x] Ana Kenet pipeline SITL MSP hattında çalışıyor.
- [x] State machine davranışı UDP mixer ile aynı: AUX High -> TRACKING ve
  AUX Mid -> AI-ARMED canlı doğrulandı; target-lost passthrough production ve
  mixer unit testleriyle aynı sözleşmeye bağlandı.
- [x] Throttle pilot kontrolünden kopmuyor.
- [x] MSP timeout veya buffer reset sorunu görülmüyor.

---

## 17. Aşama P12 - Gazebo / Fizik Simülasyonu

Amaç: Kontrol zinciri ve MSP hattı kanıtlandıktan sonra drone'un fiziksel
cevabını sim ortamında gözlemek.

Gazebo desteği taşınabilir değişkenlerle yönetilecek; repo veya kullanıcı
makinesine özel home path yazılmayacak.

Araçlar:

```text
tools/check_sitl_env.sh
tools/run_gazebo_betaflight.sh
```

Desteklenen ilk hedef:

```text
Ubuntu Linux + Gazebo Harmonic / gz-sim8 + aeroloop_gazebo BetaflightPlugin
```

Henüz desteklenmeyenler:

```text
Gazebo Classic, farklı gz-sim ABI'ları ve Linux dışı joystick akışı
```

Ortam kontrolü:

```bash
tools/check_sitl_env.sh
```

Headless smoke test:

```bash
tools/run_gazebo_betaflight.sh --world test_betaflight.sdf --headless
```

GUI ile Iris world:

```bash
tools/run_gazebo_betaflight.sh --world betaloop_iris_betaflight_demo_harmonic.sdf
```

Gazebo + Betaflight + RC sırası ayrı terminallerde çalıştırılacak:

```bash
tools/run_gazebo_betaflight.sh --world test_betaflight.sdf --headless
"$BETAFLIGHT_ROOT/obj/main/betaflight_SITL.elf"
python tools/sitl_rc_bridge.py --device "$JOY_DEV" --send --verbose
python tools/kenet_sitl_mixer.py --device "$JOY_DEV" --camera test-2.mp4 --send --print-hz 5
```

Tam fizik doğrulaması şu şartlardan önce genişletilmeyecek:

- [x] RC/SITL zinciri çalıştı.
- [x] Kenet UDP mixer TRACKING pitch/yaw üretti.
- [x] PID yönleri doğrulandı.
- [x] MSP transport smoke test geçti.
- [x] Ana pipeline MSP ile çalıştı.

Neden sona bırakılıyor?

- Gazebo kurulum ve plugin zinciri bağımlılıkları ağırdır.
- İlk sorunları fizik simülasyonunda aramak hatayı büyütür.
- Bizim önce kanıtlamamız gereken şey RC/MSP kontrol zinciridir.

Gazebo için uygulama yolu:

- [x] Gazebo Harmonic / `gz-sim8` mevcut sürüm tespit edildi.
- [x] `aeroloop_gazebo` model/world/plugin yapısı envanterlendi.
- [x] Headless Gazebo smoke testinde BetaflightPlugin yüklendi.
- [x] `tools/check_sitl_env.sh` ile bu makinede ortam raporu alındı.
- [x] `tools/run_gazebo_betaflight.sh` ile headless smoke test tekrar edildi.
- [x] `tools/gazebo_sitl_motor_smoke.py` eklendi.
- [x] `test_betaflight.sdf` ile `MSP_MOTOR=1402` ve rotor joint hareketi görüldü.
- [x] `tools/check_sitl_env.sh` world `max_step_size` değerlerini raporlar.
- [x] `tools/run_gazebo_betaflight.sh` coarse `max_step_size` için uyarı verir.
- [x] `tools/run_gazebo_betaflight.sh --max-step-size` geçici world kopyasında
  physics step override yapabilir.
- [x] `tools/run_gazebo_betaflight.sh --headless` Gazebo
  `--headless-rendering` modunu varsayılan olarak kullanır.
- [x] `tools/gazebo_stats_monitor.py` ile `/stats` real-time-factor spread
  ölçümü yapılabilir.
- [x] Dashboard `Start Gazebo`, `--gazebo-max-step-size` ile aynı timing
  izolasyonunu kullanabilir.
- [ ] `tools/check_sitl_env.sh` ile başka makinelerde ortam raporu alınacak.
- [x] Hazır Iris world ile gövde/uçuş fiziği gözlendi: virtual takeoff,
  synthetic tracking, gerçek video tracking ve raw motor UDP acceptance
  koşuları bu world üzerinde loglandı.
- [x] Kenet kamera girdisi kararı: donanımsız/tekrarlanabilir acceptance için
  `test-2.mp4` ve `--synthetic-target` kullanılacak; sim kamera entegrasyonu
  bu kapının ön koşulu değil, sonraki genişletme olarak kalacak.
- [x] `tools/sitl_virtual_takeoff_check.py` Betaflight'i varsayılan olarak temp
  çalışma dizininde başlatır; repo-root `eeprom.bin` sadece
  `--betaflight-cwd repo` ile opt-in.
- [x] `tools/gazebo_sitl_motor_smoke.py` Betaflight'i varsayılan olarak temp
  çalışma dizininde başlatır; repo-root `eeprom.bin` sadece
  `--betaflight-cwd repo` ile opt-in.
- [x] Dashboard process launcher Betaflight'i varsayılan olarak temp çalışma
  dizininde başlatır ve snapshot/API içinde `working_dir` + `eeprom_path`
  gösterir; repo-root `eeprom.bin` sadece `--betaflight-cwd repo` ile opt-in.
- [x] Dashboard process-control endpoint'leri default olarak loopback ile
  sınırlıdır; uzak istemci process start/stop için `--allow-remote-control`
  açıkça verilmelidir.
- [x] Dashboard Kenet sender başlatmadan önce aktif `sitl_rc_bridge.py`,
  `sitl_virtual_rc.py` veya `kenet_sitl_mixer.py --send` süreçlerini kontrol
  eder; duplicate RC sender varsa başlatmayı reddeder.
- [x] Smoke test PASS kararı için ana sinyal `MSP_MOTOR` / motor PWM olacak;
  rotor hareketi ek doğrulama olarak kalacak. Katı rotor kontrolü
  `--require-rotor-motion` ile opt-in.
- [x] Iris takeoff testinde `AUX3 / CH7` ANGLE mode, motor order/direction,
  MSP attitude ve motor spread aynı log koşusunda kontrol ediliyor:
  `tools/sitl_virtual_takeoff_check.py`, `tools/sitl_synthetic_tracking_check.py`
  ve `tools/sitl_video_tracking_check.py` bu alanları JSONL diagnostics olarak
  kaydediyor.

Claude timing yorumu sonrası P0.5 test sırası:

1. Baseline takla logu saklandı.
2. Iris demo world önce GUI kapalı ve daha ince physics step ile koşulacak:

   ```bash
   tools/run_gazebo_betaflight.sh \
     --world betaloop_iris_betaflight_demo_harmonic.sdf \
     --headless \
     --max-step-size 0.001
   ```

3. Aynı koşuda RTF stabilitesi ölçülecek:

   ```bash
   python tools/gazebo_stats_monitor.py --samples 30 --interval 0.5
   ```

4. Dashboard ile tam retest:

   ```bash
   python tools/sitl_dashboard.py \
     --open \
     --gazebo-headless \
     --gazebo-max-step-size 0.001 \
     --force-mode-pwm 1500
   ```

Kabul: `tools/analyze_sitl_log.py` 60 derece üstü attitude uyarısı basmıyor,
RTF spread düşük kalıyor, motor spread aşırı değil ve Kenet `IDLE`/passthrough
iken gövde takla atmıyor.

Kabul kriteri:

- [x] Doküman ve komutlarda makineye özel home-directory absolute path yok.
- [x] Gazebo plugin'i world içinde yükleniyor.
- [x] SITL motor output sim tarafına ulaşıyor.
- [x] Test quad rotor joint'leri motor output ile hareket ediyor.
- [x] Sim drone gövdesi virtual RC ile anlamlı fiziksel tepki veriyor.
  Safe-yaw P23/yaw1504 gate'i 2026-06-30 tekrar PASS:
  altitude gain `31.402 m`, max roll/pitch `0.000/0.000`, raw spread `0.618`.
- [~] Küçük offset kabulü Betaflight yaw/rate feedback sınırı nedeniyle
  yalnız safe-yaw profili ve takeoff sonrası gecikmeli Kenet state altında
  temiz. Yaw tarafında yaw5 PASS sonrası yaw6 FAIL sınırı görüldü. Pitch
  tarafında default PID ile pitch1 FAIL, fakat pitch PID `23/0/0` ile pitch5
  PASS / pitch6 FAIL. Birleşik pitch+yaw tarafında aynı gecikmeli/P23 ve pitch
  PID profiliyle yaw3+pitch3 PASS, yaw4+pitch4 FAIL. Fiziksel RC ayrı.
- [x] Kenet TRACKING sırasında sim davranışı centered synthetic target ve gerçek
  video target-found ile gözlendi. Neutral video PASS:
  `20260630-video-target-found-neutral-*`, mixer `265` target-found/source=kenet,
  max pitch/yaw delta `0/0`. Gecikmeli nonzero yaw2 video PASS:
  `20260630-video-target-found-yaw2-p23-delay18-*`, max pitch/yaw delta `0/2`.
  Aynı profil yaw4 PASS: `20260630-video-target-found-yaw4-p23-delay18-*`,
  max pitch/yaw delta `0/4`. Aynı profil yaw5 PASS:
  `20260630-video-target-found-yaw5-p23-delay18-*`, altitude gain `31.377 m`,
  max roll/pitch `3.300/4.300`, raw spread `0.000`, max yaw delta `5`.
  Aynı gecikmeli/P23 profil yaw6 FAIL:
  `20260630-video-target-found-yaw6-p23-delay18-*`, max roll `180.0`,
  max pitch `72.4`, MSP motor spread `945`, max yaw delta `6`. Daha geniş
  yaw8 koşusu da FAIL: `20260630-video-target-found-yaw8-p23-delay18-*`.
  Pitch-only tarafında default PID ile `yaw_limit=0`, `forward_limit=1` iki koşuda FAIL:
  `20260630-video-target-found-pitch1-p23-delay18-*` ve
  `20260630-video-target-found-pitch1-p23-delay18-repeat1-*`; max pitch/yaw
  delta `1/0`, MSP motor spread `945`. `forward_limit=2` de FAIL:
  `20260630-video-target-found-pitch2-p23-delay18-*`. Pitch PID `23/0/0`
  eklendiğinde pitch1/pitch2/pitch4/pitch5 PASS:
  `20260630-video-target-found-pitch1-p23-delay18-pitchpid23-*`,
  `pitch2-*`, `pitch4-*`, `pitch5-*`; pitch6 FAIL:
  `20260630-video-target-found-pitch6-p23-delay18-pitchpid23-*`, max roll
  `78.8`, max pitch/yaw delta `6/0`. Birleşik pitch+yaw gate'inde
  `20260630-video-target-found-yaw2-pitch2-p23-delay18-pitchpid23-*` ve
  `20260630-video-target-found-yaw3-pitch3-p23-delay18-pitchpid23-*` PASS;
  `20260630-video-target-found-yaw4-pitch4-p23-delay18-pitchpid23-*` FAIL.
- [~] Sim kurulumu `README.md`, `plan-sitl.md` ve `latest-development.md`
  içinde dokümante edildi; ayrı kısa kullanım dokümanı ileride eklenebilir.

---

## 18. Aşama P13 - Donanım Öncesi Çıkış Kriterleri

SITL'den gerçek FC testine geçmeden önce:

- [~] UDP mixer/test matrisi virtual/synthetic ve gerçek video neutral hatlarda
  tamamlandı. Safe-yaw virtual RC acceptance, video target-found neutral ve
  gecikmeli video yaw2/yaw4/yaw5 command-response PASS; gecikmeli video yaw6
  ve yaw8 FAIL. Pitch-only video command-response default PID ile pitch1/pitch2
  FAIL; pitch PID `23/0/0` ile pitch1/pitch2/pitch4/pitch5 PASS, pitch6 FAIL.
  Birleşik pitch+yaw video command-response tarafında yaw3+pitch3 PASS,
  yaw4+pitch4 FAIL. Fiziksel RC açık.
- [x] PID yönleri doğru.
- [x] Hedef kaybı güvenli.
- [x] MSP transport testi başarılı.
- [x] Throttle/roll override riski kapatıldı.
- [~] ARM ve Kenet state switch'leri virtual/log hattında ayrı kanallarda net;
  fiziksel bench doğrulaması için no-send preflight hazır, canlı koşu
  `/dev/input/js*` bekliyor.
- [x] GCS/log ile state geçişleri izlenebiliyor.
- [x] Test prosedürü yazılı: `docs/sitl-acceptance-procedure.md`.

Gerçek FC bench test ilkeleri:

- Pervaneler sökük.
- USB/UART bağlantısı ayrı doğrulanır.
- İlk testte motor çıkışı değil Receiver/MSP davranışı izlenir.
- Throttle pilotta kalmadan uçuş testine geçilmez.

---

## 19. Kısa Komut Rehberi

Yeni terminal:

```bash
fpv-test
```

SITL:

```bash
"$BETAFLIGHT_ROOT/obj/main/betaflight_SITL.elf"
```

Configurator köprüsü:

```bash
websockify 127.0.0.1:6761 127.0.0.1:5761
```

RC bridge:

```bash
python tools/sitl_rc_bridge.py --device "$JOY_DEV" --dry-run
python tools/sitl_rc_bridge.py --device "$JOY_DEV" --send --verbose
```

Monitor:

```bash
python tools/rc_monitor.py --device "$JOY_DEV"
python tools/sitl_dashboard.py --open
python tools/state_monitor.py --open  # compatibility shim -> sitl_dashboard.py
```

Kenet joystick desk-test:

```bash
python kenet.py --joystick --aux-ch 5 --joy-axis 6 --joy-arm-axis 4 \
  --camera test-2.mp4 --no-gcs
```

Kenet SITL mixer:

```bash
python tools/kenet_sitl_mixer.py --device "$JOY_DEV" --camera test-2.mp4
python tools/kenet_sitl_mixer.py --device "$JOY_DEV" --camera test-2.mp4 --send --print-hz 5
```

Gazebo:

```bash
tools/check_sitl_env.sh
tools/run_gazebo_betaflight.sh --world test_betaflight.sdf --headless
tools/run_gazebo_betaflight.sh --world betaloop_iris_betaflight_demo_harmonic.sdf
tools/run_gazebo_betaflight.sh --world betaloop_iris_betaflight_demo_harmonic.sdf \
  --max-step-size 0.001 --fix-iris-imu-pose --fix-iris-motor-map
python tools/gazebo_sitl_motor_smoke.py
python tools/sitl_dashboard.py --open
```

Opsiyonel MSP pseudo serial fallback denemesi:

```bash
socat -d -d pty,raw,echo=0,link=/tmp/bf-sitl,mode=666 tcp:127.0.0.1:5761
```

---

## 20. Yakın Sıradaki İşler

En yakın pratik sıra:

1. [x] Virtual RC sender ekle; joystick olmadan CH3 throttle, CH5 ARM,
   CH6 Kenet ve CH7 mode değerleri üretilebilsin.
2. [x] Diagnostics'e `--rc-source virtual` ekle; virtual RC koşularında beklenen
   pilot RC ile FC `MSP_RC` delta kontrolü yapılabilsin.
3. [x] Configurator açmadan mode range uygulamak için
   `tools/sitl_configure_modes.py` ekle.
   Varsayılan range seti artık HORIZON AUX3 high satırını da yazar.
4. [x] Virtual RC + Betaflight standalone ile `MSP_RC` doğrulaması yap:
   CH3/CH5/CH7 beklenen değerleri FC tarafında görülmeli.
5. [x] Virtual RC + Gazebo Iris + `0.0025 --headless --fix-iris-motor-map`
   koşusunda arm/throttle/attitude/motor logu al.
6. [x] Virtual RC koşusunda takla sürerse motor/moment/IMU izolasyonuna geç;
   takla yoksa fiziksel joystick/Tango yolunu ayrı entegrasyon sorunu olarak
   geri ekle. Sonuç: takla sürdü, fiziksel RC elendi.
7. [x] Motor/moment/IMU izolasyonuna başlandı ve virtual RC takla kök nedeni
   A/B ile daraltıldı. Direct motor moment işaretleri simetrik çıktı; pose/IMU
   diagnostics ile ilk ayrışma yaw gyro feedback tarafına indi. Dış
   `../aeroloop_gazebo/plugins/BetaflightPlugin.cc` içinde yaw gyro `z` işareti
   çevrilip plugin rebuild edilince virtual takeoff check PASS verdi.
8. [x] Kenet state default kanalını tekilleştir (`CH6 / AUX2 / index 5`)
   ve `kenet.py` / `PipelineConfig` / docs uyumunu sağla.
9. [x] SITL mixer ile ana pipeline state machine davranışını ortak helper veya
   pluggable sink yaklaşımıyla eşitle.
10. [x] `tools/` safety glue testlerini ekle: passthrough, only pitch/yaw,
   target lost, tracker unavailable, AUX thresholds, RC packet format.
11. [x] `tools/analyze_sitl_log.py` veri yokken güvenli sonuç üretmesin:
   attitude/motor sample yoksa açıkça "veri yok" raporlasın; ayrıca
   `diagnostic_sample` kayıtlarını okuyup MSP attitude/motor özetleyebilsin.
12. [x] Log rate/flush/rotation ve JSONL schema hardening tamamlandı:
   `JsonlLogger` batch/time-based flush ve size rotation destekliyor;
   mixer/dashboard flight log default'u `10 Hz`; schema
   `docs/sitl-jsonl-schema.md` altında dokümante; analyzer tüm kanal setini
   okuyup legacy/eksik alanlarda schema warning üretiyor.
13. [x] `sitl_dashboard.py` ana gözlem aracı oldu; `state_monitor.py`
   compatibility shim olarak dashboard'a yönlendiriyor, `--legacy` eski
   state-only web UI'ı koruyor. `rc_monitor.py` dar joystick/raw RC monitor
   olarak kaldı ve bridge mapping'i kullanıyor.
14. [x] `sitl_mixer_matrix_check.py` ile Receiver/mixer test matrisi virtual
   pilotta tamamlandı; fiziksel joystick `/dev/input/js*` ayrı gate olarak
   bekliyor.
15. [x] PID yönlerini bilinçli hedef hareketleriyle doğrula:
   `tools/sitl_pid_direction_check.py` PASS.
16. [~] Hedef kaybı durumunda pilot passthrough davranışını doğrula. Production
   pipeline ve SITL mixer unit gate'leri geçti; sentetik target-loss runner
   `pilot-target-lost` ve `AI-ARMED` sample gate'leriyle eklendi. Target-loss
   sonrası re-entry lockout production/SITL ortak davranış: switch track
   eşiğinin altına inmeden tekrar TRACKING yok. Canlı final gate PASS:
   `20260630-synthetic-target-loss-live-final`; gerçek video/kamera target-loss
   matrisi hâlâ ayrı koşulabilir.
17. [x] JSONL log ve `tools/analyze_sitl_log.py` eklendi.
18. [x] MSP için `socat` pseudo serial yolu aktif ön koşul olmaktan çıkarıldı.
   Native TCP üretim yolu eklendi ve geçti; socat sadece opsiyonel regresyon
   fallback'i.
19. [x] Gerekirse `msp.py` içine TCP transport ekle.
20. [x] Ana `kenet.py` pipeline'ını SITL MSP hattında çalıştır.
21. [x] Gazebo'da Betaflight SITL motor output'unun sim tarafına ulaştığını doğrula.
22. [x] Canlı dashboard ile joystick, Kenet state, Betaflight armed, FC RC ve motor değerlerini izle.
23. [x] Dashboard'a Gazebo, Betaflight SITL ve Kenet mixer start/stop butonları ekle.
24. [x] Iris world ile virtual RC/gövde fizik tepkisini gözle; takeoff öncesi
   `tools/sitl_diagnostics.py` ile Gazebo RTF, MSP arm blocker, RC delta ve
   motor mapping loglansın.
25. [~] Fiziksel Tango/joystick RC yolu virtual RC sonucu netleşince tekrar
   doğrulanacak. Bu oturumda `/dev/input/js*` görünmediği için canlı fiziksel
   retest koşturulamadı; bunun yerine takeoff checker'a `--rc-driver external`
   ve no-send `tools/sitl_physical_rc_preflight.py` eklendi.
26. [x] Kenet mixer + Gazebo hattında sanal pilot / hedef-yok passthrough
   davranışını test et.
27. [~] Kenet mixer + Gazebo hattında sentetik target-found TRACKING davranışını
   test et; centered target-found harness PASS, offsetli kabul FAIL.
28. [x] Gerçek kamera/video target-found TRACKING kabul testini aynı kriterlerle
   koş. `tools/sitl_video_tracking_check.py --camera test-2.mp4 --run-id
   20260630-video-target-found-neutral` PASS: diagnostics altitude gain
   `23.850 m`, max roll/pitch `0.000/0.100`, raw spread `0.000`; mixer
   `265` target_found/source=kenet sample, max pitch/yaw delta `0/0`.
29. [~] Target-found küçük yaw/pitch RC offset'lerinin motor spread/attitude
   cevabını daha düşük limitlerle ve tek eksen halinde izole et. İlk izolasyon:
   +4 yaw ve +10 pitch flip üretiyor; direct virtual RC yaw1504 de Kenet
   olmadan aynı yönde FAIL. P23/yaw1504 için `--safe-yaw-authority` PASS gate'i
   eklendi. Gerçek video tarafında gecikmesiz yaw4/yaw2/yaw1 FAIL; sebep
   komutun takeoff/ramp sırasında gelmesi. `--virtual-kenet-delay-seconds 18`
   ve `--yaw-pid 23,0,0` ile video yaw2 PASS:
   `20260630-video-target-found-yaw2-p23-delay18-*`. Aynı gecikmeli/P23 profil
   video yaw4 PASS: `20260630-video-target-found-yaw4-p23-delay18-*`, altitude
   gain `31.401 m`, max roll/pitch `1.300/5.400`, raw spread `0.000`. Aynı
   profil video yaw5 PASS: `20260630-video-target-found-yaw5-p23-delay18-*`,
   altitude gain `31.377 m`, max roll/pitch `3.300/4.300`, raw spread `0.000`.
   Aynı gecikmeli/P23 profil video yaw6 FAIL:
   `20260630-video-target-found-yaw6-p23-delay18-*`, max roll `180.0`,
   max pitch `72.4`, MSP motor spread `945`, max yaw delta `6`. Daha geniş
   video yaw8 de FAIL: `20260630-video-target-found-yaw8-p23-delay18-*`.
   Pitch-only gerçek video koşusunda default PID ile `yaw_limit=0`, `forward_limit=1` iki kez
   FAIL: `20260630-video-target-found-pitch1-p23-delay18-*` ve repeat1; max
   pitch/yaw delta `1/0`, MSP motor spread `945`. `forward_limit=2` de FAIL:
   `20260630-video-target-found-pitch2-p23-delay18-*`. Pitch PID `23/0/0` ile
   pitch1/pitch2/pitch4/pitch5 PASS; pitch6 FAIL:
   `20260630-video-target-found-pitch6-p23-delay18-pitchpid23-*`, max roll
   `78.8`, max pitch/yaw delta `6/0`. Birleşik pitch+yaw tarafında aynı
   gecikmeli/P23 ve pitch PID profiliyle yaw2+pitch2 PASS, yaw3+pitch3 PASS,
   yaw4+pitch4 FAIL:
   `20260630-video-target-found-yaw2-pitch2-p23-delay18-pitchpid23-*`,
   `20260630-video-target-found-yaw3-pitch3-p23-delay18-pitchpid23-*`,
   `20260630-video-target-found-yaw4-pitch4-p23-delay18-pitchpid23-*`.
30. [x] Tek komutluk virtual takeoff kabul runner'ı eklendi:
   `tools/sitl_virtual_takeoff_check.py`.
31. [x] Aynı kabul kapısı external RC için hazırlandı:
   `tools/sitl_virtual_takeoff_check.py --rc-driver external`. Bu mod RC
   göndermez; fiziksel `sitl_rc_bridge.py` veya `kenet_sitl_mixer.py --send`
   çalışırken MSP/Gazebo pose kriterlerini ölçer.
32. [x] External checker + virtual pilot + synthetic target orchestration tek
   komuta alındı: `tools/sitl_synthetic_tracking_check.py`.
33. [x] Kenet'siz doğrudan virtual RC nudge eklendi:
   `tools/sitl_virtual_takeoff_check.py --nudge-delay-seconds 18 --nudge-yaw 1504`.

## 21. Güncel Gazebo Diagnostic Akışı

Amaç: "kumanda geç geliyor", "arm olmuyor" ve "takeoff sonrası takla" gibi
belirtileri tek yerde ölçmek.

Komut:

```bash
python tools/sitl_diagnostics.py --samples 20 --interval 0.25
```

Virtual RC koşusunda:

```bash
python tools/sitl_configure_modes.py

python tools/sitl_diagnostics.py --rc-source virtual \
  --virtual-throttle 1550 \
  --virtual-arm-pwm 2000 \
  --virtual-mode-pwm 1500 \
  --samples 20 --interval 0.25
```

Bu araç şunları JSONL olarak kaydeder:

- Gazebo `real_time_factor`, `step_size`, iteration ve stats okuma süresi.
- Dashboard API yanıt süresi ve state yaşı.
- MSP RC, motor, attitude, active modes ve arming disable flags.
- Joystick veya virtual RC kaynak bilgisi ve beklenen pilot RC kanalları.
- Pilot RC ile FC RC arasındaki kanal deltaları.
- SITL port sahipleri ve process CPU/memory snapshot'ı.
- `betaloop_iris_with_standoffs/model.sdf` motor mapping analizi.
- Çalışan Gazebo sürecindeki geçici `--fix-iris-motor-map` overlay'i aktif mi.

İlk kuru sonuç eski statik SDF kontrolünde alarm verdi:

```text
motor mapping ok: False
mismatched BF motors: 0,1,2,3
```

Güncel yorum:

```text
Betaflight SITL zaten motorları slot3/slot0/slot1/slot2 olarak remap ediyor.
Bu yüzden raw packet slotlarını logical BF motoru gibi yorumlayan statik kontrol
yanıltıcı olabilir. Axis probe artık logical BF motorlarını --motor-map bf-sitl
ile emüle ediyor ve roll/pitch/yaw moment eksenleri temiz ayrışıyor.
```

Source/runtime eşlemesi:

```text
BetaflightPlugin rotor mapping:
id=0 -> rotor_0_joint
id=1 -> rotor_1_joint
id=2 -> rotor_2_joint
id=3 -> rotor_3_joint
```

Bu source identity mapping artık dış model dosyasında da runtime
`--fix-iris-motor-map` overlay'iyle uyumlu. Sıradaki aday kaba motor mapping
değil; closed-loop sırasında Betaflight'in ürettiği actual motor UDP çıktısını
slot/moment karşılığıyla yakalamak.

Kalıcı dış repo değişikliği yapmadan denemek için:

```bash
tools/run_gazebo_betaflight.sh \
  --world betaloop_iris_betaflight_demo_harmonic.sdf \
  --max-step-size 0.001 \
  --fix-iris-imu-pose \
  --fix-iris-motor-map
```

Dashboard `Start Gazebo` butonu bu geçici fix'i varsayılan olarak kullanır ve
GUI modunda açar. Komut satırından virtual RC ile önerilen ilk retest:

```bash
tools/run_gazebo_betaflight.sh \
  --world betaloop_iris_betaflight_demo_harmonic.sdf \
  --headless \
  --max-step-size 0.0025 \
  --fix-iris-imu-pose \
  --fix-iris-motor-map

"$BETAFLIGHT_ROOT/obj/main/betaflight_SITL.elf"

python tools/sitl_configure_modes.py

python tools/sitl_virtual_rc.py \
  --script takeoff \
  --throttle 1550 \
  --mode-pwm 1500 \
  --send
```

Kabul testi:

```bash
python tools/sitl_diagnostics.py --rc-source virtual \
  --virtual-throttle 1550 \
  --virtual-arm-pwm 2000 \
  --virtual-mode-pwm 1500 \
  --samples 20 --interval 0.25
```

Beklenen:

- Gazebo RTF okunuyor.
- MSP bağlı ve arm blocker listesi anlamlı.
- Virtual pilot/FC RC deltaları küçük.
- Takeoff sonrası roll/pitch 60 derece üstüne çıkmıyor.

2026-06-29 sonucu:

```bash
python tools/analyze_sitl_log.py logs/sitl/20260629-212308-diagnostics.jsonl
```

```text
samples: 45
armed True: 28, False: 17
active modes: ANGLE 45, ARM 28
max roll: -180.0 deg
max pitch: -46.2 deg
max motor spread: 945 at 2026-06-29T21:23:16+0300 motors=2000,1176,1055,1277
heuristic: attitude exceeded 60 deg; flip/tumble event
```

Karar: Bu koşu beklenen "takla yok" kabulünü geçmedi. Ancak RC izolasyonu
başarılı oldu: ARM/ANGLE aktif, MSP RC doğru, RTF temiz ve fiziksel kumanda
yok. Bu nedenle fiziksel Tango/joystick yolu bu taklanın ana nedeni olarak
elenir; sıradaki iş motor/moment/IMU izolasyonudur.

2026-06-29 22:15 A/B sonucu:

```bash
python tools/sitl_virtual_takeoff_check.py \
  --throttle 1750 --hold-seconds 8 \
  --diagnostic-samples 75 --diagnostic-interval 0.1
```

Baseline, gyro patch öncesi:

```text
log: logs/sitl/20260629-221238-takeoff-diagnostics.jsonl
result: FAIL
max roll: -180.0 deg
max pitch: -73.6 deg
max motor spread: 945 us
altitude gain: 0.930 m
```

Düzeltme:

```text
../aeroloop_gazebo/plugins/BetaflightPlugin.cc
pkt.imuAngularVelocityRPY[2] = -angularVel.Z();
```

Yorum: Betaflight SITL `sitl.c` içinde paket yaw gyro değerini zaten
`virtualGyroSet()` öncesi negatifliyor. Gazebo IMU yaw rate ham gönderilince
ANGLE/rate loop yaw ekseninde pozitif feedback üretiyor; ilk büyük motor spread
roll/pitch hâlâ küçükken diagonal yaw düzeltmesi olarak başlıyor.

Plugin rebuild sonrası aynı runner:

```text
log: logs/sitl/20260629-221545-takeoff-diagnostics.jsonl
result: PASS
altitude gain: 6.624 m
max roll/pitch: 0.0 / 0.0 deg
max motor spread: 0 us
```

Kenet mixer sanal pilot + external checker:

```text
diagnostics: logs/sitl/20260629-223523-external-kenet-takeoff-diagnostics.jsonl
mixer log:   logs/sitl/20260629-223523-kenet-mixer-virtual-external-takeoff.jsonl
result: PASS
samples: 120, MSP connected: 120, pose ok: 120
ARM+ANGLE samples: 63, high throttle samples: 53
altitude gain: 23.840 m
max roll/pitch: 0.3 / 0.1 deg
mixer state/source: TRACKING / pilot-target-lost
max final-pilot delta: 0 on first 8 channels
```

Sentetik target-found izolasyon koşuları:

```text
centered target-found, one-command runner:
  diagnostics: logs/sitl/20260629-continued-synthetic-centered-runner-delay8-diagnostics.jsonl
  mixer log:   logs/sitl/20260629-continued-synthetic-centered-runner-delay8-mixer.jsonl
  result: PASS / source=kenet target_found=True with neutral pitch/yaw
  samples: diagnostic 120, mixer 1244
  ARM+ANGLE samples: 73
  altitude gain: 23.838 m
  max roll/pitch: 0.4 / 0.3 deg
  max motor spread: 0
  max final-pilot delta: 0 on first 8 channels

combined pitch+yaw:
  diagnostics: logs/sitl/20260629-224953-external-synthetic-tracking-delayed-diagnostics.jsonl
  mixer log:   logs/sitl/20260629-224953-kenet-mixer-synthetic-tracking-delayed.jsonl
  result: FAIL / delayed target-found command after takeoff triggers flip
  altitude gain: 5.116 m
  max roll/pitch: 180.0 / 63.6 deg
  max final-pilot delta: pitch +20, yaw +26

yaw only, positive:
  diagnostics: logs/sitl/20260629-225200-external-synthetic-yaw-only-diagnostics.jsonl
  result: FAIL
  altitude gain: 5.665 m
  max roll/pitch: 180.0 / 52.0 deg
  max final-pilot delta: yaw +26

yaw only, negative:
  diagnostics: logs/sitl/20260629-225340-external-synthetic-yaw-negative-diagnostics.jsonl
  result: FAIL
  altitude gain: 5.568 m
  max roll/pitch: 180.0 / 61.3 deg
  max final-pilot delta: yaw -26

pitch only:
  diagnostics: logs/sitl/20260629-225520-external-synthetic-pitch-only-diagnostics.jsonl
  result: FAIL
  altitude gain: 5.375 m
  max roll/pitch: 180.0 / 32.3 deg
  max final-pilot delta: pitch +20

yaw micro, P-only:
  diagnostics: logs/sitl/20260629-continued-yaw-micro-positive-ponly-diagnostics.jsonl
  mixer log:   logs/sitl/20260629-continued-yaw-micro-positive-ponly-mixer.jsonl
  result: FAIL
  max roll/pitch: 180.0 / 28.5 deg
  max motor spread: 945
  max final-pilot delta: yaw +12

yaw micro, P-only, hard limit 4:
  diagnostics: logs/sitl/20260629-continued-yaw-micro-positive-limit4-diagnostics.jsonl
  mixer log:   logs/sitl/20260629-continued-yaw-micro-positive-limit4-mixer.jsonl
  result: FAIL
  max roll/pitch: 180.0 / 46.2 deg
  max motor spread: 945
  max final-pilot delta: yaw +4

pitch micro, P-only:
  diagnostics: logs/sitl/20260629-continued-pitch-micro-positive-ponly-diagnostics.jsonl
  mixer log:   logs/sitl/20260629-continued-pitch-micro-positive-ponly-mixer.jsonl
  result: FAIL
  max roll/pitch: 180.0 / 68.7 deg
  max motor spread: 945
  max final-pilot delta: pitch +10

direct virtual RC, Kenet yok, yaw 1504:
  diagnostics: logs/sitl/20260629-continued-direct-virtual-yaw1504-clean-diagnostics.jsonl
  result: FAIL
  max roll/pitch: 180.0 / 76.8 deg
  max motor spread: 945
  max FC yaw delta: +4

BF SITL axis moment probe, corrected remap:
  log: logs/sitl/20260629-continued-axis-moment-default-bfsitl-after-identity.jsonl
  result: PASS as an isolation signal
  motor-map: bf-sitl / logical-to-packet 3,0,1,2
  right/left pair -> roll, front/rear pair -> pitch, cw/ccw pair -> yaw

FDM yaw sign probe:
  cw log:  logs/sitl/20260629-continued-fdm-yaw-cw-bfsitl.jsonl
  ccw log: logs/sitl/20260629-continued-fdm-yaw-ccw-bfsitl.jsonl
  result: PASS / Gazebo IMU yaw and FDM yaw gyro are inverted as expected
  cw example: gazebo z=+1.857, fdm z=-1.856
  ccw example: gazebo z=-1.857, fdm z=+1.856

yaw_motors_reversed A/B:
  off diagnostics: logs/sitl/20260629-continued-direct-yaw1504-yaw-motors-reversed-off-diagnostics.jsonl
  on diagnostics:  logs/sitl/20260629-continued-direct-yaw1504-yaw-motors-reversed-on-diagnostics.jsonl
  result: FAIL in both cases
  off: flip can begin before delayed yaw nudge
  on: nudge phase starts cleaner, but yaw 1504 still reaches motor spread 945 and flip

closed-loop raw motor UDP capture:
  neutral diagnostics: logs/sitl/20260630-neutral-hold30-motorudp-diagnostics.jsonl
  neutral raw UDP:     logs/sitl/20260630-neutral-hold30-motorudp.jsonl
  neutral result: PASS, raw max spread 0, roll/pitch/yaw axis bias 0

  yaw1504 off diagnostics: logs/sitl/20260630-closedloop-yaw1504-off-diagnostics.jsonl
  yaw1504 off raw UDP:     logs/sitl/20260630-closedloop-yaw1504-off-motor-udp.jsonl
  yaw1504 on diagnostics:  logs/sitl/20260630-closedloop-yaw1504-on-diagnostics.jsonl
  yaw1504 on raw UDP:      logs/sitl/20260630-closedloop-yaw1504-on-motor-udp.jsonl
  yaw result: FAIL in both cases
  first large raw spread:
    off: yaw_cw_minus_ccw 405.5 us, roll bias 2.0 us, pitch bias 0.0 us
    on:  yaw_cw_minus_ccw 408.7 us, roll bias -1.7 us, pitch bias ~0.0 us
  yorum: runaway ilk anda roll/pitch motor karışımı değil, saf yaw pair
  ayrışması olarak başlıyor. Motor axis ve FDM yaw sign temiz olduğu için
  sıradaki kapı yaw PID/rate feedback etkisini düşürüp aynı raw UDP ayrışmasının
  kaybolup kaybolmadığını ölçmek.
```

Yorum: target-found yolu artık tek komutla ölçülebiliyor ve centered hedefte RC
karışımı güvenli sınırda kalıyor: roll/throttle/AUX pilottan, sadece pitch/yaw
Kenet'ten; merkez hedefte pitch/yaw delta `0`. Offsetli target-found Gazebo
fizik kabulü artık tek eksenlerde ve birleşik küçük limitlerde braketlendi:
yaw5 PASS/yaw6 FAIL, pitch5 PASS/pitch6 FAIL, birleşik yaw3+pitch3 PASS ve
yaw4+pitch4 FAIL. Bu nedenle sıradaki açık sim işi daha büyük limit kovalamak
değil; bu güvenli pencereyi acceptance prosedüründe sabit tutup fiziksel RC
geri ekleme ve gerçek FC bench kapılarını ayrı doğrulamak.

Fiziksel RC / Kenet mixer geri ekleme komutu:

```bash
python tools/sitl_virtual_takeoff_check.py \
  --rc-driver external \
  --throttle 1750 \
  --diagnostic-samples 75 --diagnostic-interval 0.1
```

Bu mod Gazebo/Betaflight'i ve diagnostics'i yönetir ama RC göndermez. Test
penceresinde ayrı bir terminalden `tools/sitl_rc_bridge.py --send
--force-mode-pwm 1500` veya `tools/kenet_sitl_mixer.py --send
--force-mode-pwm 1500` çalıştırılmalı. Kabul kriterleri virtual runner ile aynı:
ARM+ANGLE görülür, FC throttle hedefe çıkar, pose/IMU loglanır, altitude gain
`>=1 m`, max roll/pitch `<=35 deg`, motor spread runaway yapmaz.
