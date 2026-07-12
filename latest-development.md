# Latest Development — 2026-06-30

Bu doküman Kenet / Betaflight SITL tarafındaki en son durumu özetler. Son
odak noktası, gerçek uçuş kontrolcüsü olmadan RC kumanda, Betaflight SITL ve
Kenet görsel takip/PID çıktısını aynı masaüstü test hattında doğrulamaktı.

---

## Kısa Sonuç

SITL geliştirmelerinde altı önemli eşik geçildi:

```text
RC kumanda + Kenet visual tracker/PID
        -> tools/kenet_sitl_mixer.py
        -> Betaflight SITL UDP 9004 rc_packet
        -> Betaflight Configurator Receiver tab
```

`TRACKING` aşamasında Kenet'in ürettiği pitch ve yaw komutları Betaflight
Receiver tarafında otomatik olarak görüldü. Roll ve throttle pilotta kalıyor.
Bu, SITL tarafındaki ilk güvenli ve anlaşılır Kenet kontrol doğrulaması oldu.

İkinci eşik: fiziksel Tango/USB joystick izole edilerek virtual RC ile Gazebo
takla problemi tekrar üretildi. Böylece bu takla için fiziksel kumanda ana kök
neden olmaktan çıktı; sıradaki araştırma motor/moment/IMU ve
Betaflight-Gazebo kuplajına daraltıldı.

Üçüncü eşik: motor/moment izolasyonu tamamlandı. Direct Gazebo motor pulse
testinde plugin'in temel motor->moment işaretleri simetrik çıktı; yüksek
all-equal motor komutu z ekseninde lift üretebildi. Tek komutluk runner ile
baseline takla tekrar üretildi, sonra `BetaflightPlugin.cc` yaw gyro `z` sign
patch'i ve rebuild sonrası aynı virtual RC takeoff PASS verdi. Kök neden virtual
RC hattında fiziksel kumanda veya kaba motor map değil, plugin yaw gyro feedback
işaretiydi.

Dördüncü eşik: fiziksel joystick olmadan Kenet mixer yolundan da RC
gönderilebiliyor. `kenet_sitl_mixer.py --pilot-source virtual` ile sanal
takeoff RC üretildi ve `sitl_virtual_takeoff_check.py --rc-driver external`
aynı Gazebo/Betaflight kabul kriterlerinde PASS verdi.

Beşinci eşik daha çok teşhis kapısı: `kenet_sitl_mixer.py --synthetic-target`
ile kamera/video olmadan target-found yolu ölçülebiliyor. Bu harness çalışıyor,
`tools/sitl_synthetic_tracking_check.py` ile external checker + virtual Kenet
mixer tek komutta koşturuluyor. Centered target-found PASS; küçük pitch/yaw RC
offsetleri ise eski sentetik/gecikmesiz koşularda takeoff sonrası flip'i geri
getirdi. Güncel gerçek video tarafında TRACKING takeoff sonrasına
geciktirilince safe/P23 profilde yaw2 ve yaw4 command-response PASS verdi;
yaw5 PASS, yaw6 FAIL ile güncel sınır daraldı.

Altıncı eşik kök nedeni daha da daralttı: closed-loop raw motor UDP capture,
direct yaw 1504 nudge sırasında ilk büyük motor ayrışmasının neredeyse saf
`yaw_cw_minus_ccw` bias olduğunu gösterdi. Yaw PID sıfırlanınca aynı virtual RC
yaw 1504 koşusu PASS verdi. Bu yüzden güncel aday fiziksel RC veya Kenet değil,
Betaflight yaw PID/rate feedback zinciri.

---

## Mevcut Test Seviyesi

Kenet mixer'ın en sade kontrol seviyesi hâlâ MSP'siz okunabilir:

```text
Joystick -> Python RC mapping -> Kenet SITL mixer -> UDP 9004 -> Betaflight SITL
```

Bu yolun amacı, MSP ve Gazebo karmaşıklığına girmeden önce Kenet'in kontrol
mantığını test etmek. Gazebo/Betaflight acceptance runner'ında ise MSP artık
aktif olarak kullanılıyor: mode range kurulumu, yaw PID/debug config, `MSP_RC`,
`MSP_MOTOR`, attitude/status ve `MSP_DEBUG` diagnostics bu katmanda ölçülüyor.

Taşınabilir SITL değişkenleri:

```bash
export FPV_ROOT="$(pwd)"
export AEROLOOP_GAZEBO="${AEROLOOP_GAZEBO:-../aeroloop_gazebo}"
export BETAFLIGHT_ROOT="${BETAFLIGHT_ROOT:-../betaflight}"
export JOY_DEV="${JOY_DEV:-/dev/input/js0}"
```

Kanıtlananlar:

- [x] TBS Tango 2 USB joystick olarak okunuyor.
- [x] CH1-CH4 stick değerleri 1000-2000 RC aralığına çevriliyor.
- [x] CH5/AUX1 ARM switch olarak kullanılabiliyor.
- [x] CH6/AUX2 Kenet state switch olarak kullanılabiliyor.
- [x] Betaflight SITL çalışıyor.
- [x] Configurator SITL'e bağlanıyor.
- [x] Receiver tab'de RC kanalları hareket ediyor.
- [x] ARM mode çalışıyor.
- [x] `TRACKING` durumunda Kenet pitch/yaw üretiyor.
- [x] `TRACKING` durumunda roll/throttle pilotta kalıyor.
- [x] `pytest` sonucu temiz: `237 passed` (`./fpv_env/bin/python -m pytest -q`,
  2026-06-30).
- [x] Portable Gazebo env check ve launcher scriptleri eklendi.
- [x] Gazebo Harmonic headless smoke testinde BetaflightPlugin yüklendi.
- [x] Gazebo + Betaflight SITL motor-output smoke test geçti.
- [x] `MSP_MOTOR` 1402 değerine çıktı ve Gazebo rotor joint'leri hareket etti.
- [x] Takla logunda Kenet'in `IDLE` olduğu görüldü; problem Kenet PID/target
  override kaynaklı görünmüyor.
- [x] CH7/AUX3 autopilot mode için `--force-mode-pwm` test override'ı eklendi.
- [x] `tools/sitl_diagnostics.py` eklendi; Gazebo RTF/step, dashboard API
  gecikmesi, MSP arm blocker'ları, joystick/RC delta, process/port durumu ve
  Gazebo SDF motor mapping'i tek JSONL logda toplanıyor.
- [x] İlk diagnostic statik motor mapping kontrolünde
  `mismatched BF motors: 0,1,2,3` buldu. Bu, Betaflight SITL iç remap'i ile
  SDF plugin remap'inin üst üste binip dört motoru da yanlış köşeye göndermesi
  hipotezini güçlendiriyor.
- [x] `tools/run_gazebo_betaflight.sh --fix-iris-motor-map` eklendi. Dış
  `aeroloop_gazebo` dosyasını değiştirmeden geçici model kopyasında
  BetaflightPlugin rotor eşlemesini identity yapar.
- [x] Dashboard `Start Gazebo` artık varsayılan olarak geçici IMU pose fix ve
  motor map fix ile başlatır; GUI modu korunur.
- [x] `tools/sitl_virtual_rc.py` eklendi; fiziksel Tango/joystick olmadan
  CH3 throttle rampası, CH5 ARM ve CH7 flight-mode değerleriyle sanal takeoff
  sekansı üretilebilir.
- [x] `tools/sitl_diagnostics.py --rc-source virtual` eklendi; sanal RC
  koşusunda beklenen pilot frame'i ile FC `MSP_RC` değeri karşılaştırılabilir.
- [x] `tools/sitl_configure_modes.py` eklendi; Configurator açmadan Betaflight
  SITL mode range'leri canlı oturuma uygulanabiliyor. Varsayılan set ARM,
  MSP Override, ANGLE ve HORIZON range'lerini yazar.
- [x] `tools/sitl_mode_status_check.py` eklendi; sanal RC case'leri gönderip
  `MSP_STATUS_EX` ile ARM/MSP Override/ANGLE/HORIZON aktif modlarını ve arming
  disable flag'lerini Configurator açmadan raporlar.
- [x] `tools/sitl_mixer_matrix_check.py` eklendi; P6.1-P6.8 receiver/mixer
  matrisini donanımsız çalıştırır. Güncel koşu PASS: AUX2 low/mid/high state
  davranışı, target-lost passthrough, target-found yalnız pitch/yaw override,
  CH5 ARM low/high ve TRACKING sırasında roll/throttle pilotta kalma doğrulandı.
- [x] Throttle/roll authority kararı kapatıldı: mevcut Kenet kapsamı
  pitch/yaw-only. `msp_override_channels = 10` korunacak; throttle veya roll
  Kenet'e verilirse ayrı RFC ve yeni bench acceptance gate gerekecek.
- [x] Eski "Configurator ile log davranışı tutarlı mı" maddesi otomatik kapıya
  taşındı: mode aktiflikleri `sitl_mode_status_check.py`, receiver/mixer
  davranışı `sitl_mixer_matrix_check.py`, uçuş koşuları dashboard/diagnostics
  JSONL ve MSP parse ile izleniyor.
- [x] Virtual RC + standalone Betaflight testinde `MSP_RC` delta `0` ölçüldü.
- [x] Virtual RC + Gazebo Iris + runtime `--fix-iris-motor-map` koşusunda
  `1550` scripted takeoff taklayı tekrar üretti:
  `logs/sitl/20260629-212308-diagnostics.jsonl`.
- [x] `tools/analyze_sitl_log.py` artık `diagnostic_sample` kayıtlarındaki MSP
  attitude/motor verisini özetleyebiliyor.
- [x] `tools/gazebo_motor_moment_probe.py` eklendi; Betaflight'i bypass edip
  Gazebo BetaflightPlugin'e direct motor pulse gönderiyor ve pose/IMU delta
  kaydediyor.
- [x] Direct motor moment probe sonucu simetrik:
  `logs/sitl/20260629-213502-motor-moment.jsonl`.
- [x] Direct high thrust sanity sonucu all-equal `0.95` motor speed z'yi
  yükseltiyor: `logs/sitl/20260629-214237-motor-moment.jsonl`.
- [x] `tools/sitl_motor_udp_probe.py` eklendi; Betaflight'in `9001/udp` raw
  servo/motor çıkışını JSONL olarak kaydediyor.
- [x] Birleşik virtual RC + diagnostics + raw UDP koşusunda takla yeniden
  üretildi:
  `logs/sitl/20260629-215325-diagnostics.jsonl` ve
  `logs/sitl/20260629-215315-motor-udp.jsonl`.
- [x] Bu birleşik koşuda raw motor UDP 2000'e kadar çıktı; Gazebo pose da
  fiziksel olarak roll `-180` durumuna geçti.
- [x] `tools/sitl_virtual_takeoff_check.py` eklendi. Gazebo + Betaflight'i
  başlatır, plugin handshake için kısa sıfır motor bootstrap'i gönderir, mode
  range'leri kurar, virtual RC takeoff'u ve pose/IMU diagnostics'i tek komutta
  pass/fail olarak koşturur.
- [x] Tek komutluk baseline koşu gyro patch olmadan taklayı tekrar üretti:
  `logs/sitl/20260629-221238-takeoff-diagnostics.jsonl`.
  Sonuç: max roll `-180`, max pitch `-73.6`, max motor spread `945`, altitude
  gain yalnız `0.93 m`.
- [x] Kök neden A/B ile daraltıldı: `../aeroloop_gazebo/plugins/BetaflightPlugin.cc`
  içinde yaw gyro `z` işareti ters gönderilmeliydi. Betaflight SITL zaten paket
  `z` gyro değerini `virtualGyroSet()` öncesinde negatiflediği için eski yol
  pozitif yaw feedback üretiyordu.
- [x] Plugin rebuild sonrası aynı virtual takeoff check PASS verdi:
  `logs/sitl/20260629-221545-takeoff-diagnostics.jsonl`. Sonuç: altitude gain
  `6.624 m`, max roll/pitch `0`, max motor spread `0`.
- [x] Takeoff checker external RC gözlem moduna genişletildi:
  `tools/sitl_virtual_takeoff_check.py --rc-driver external`. Bu mod RC
  göndermez; fiziksel `sitl_rc_bridge.py` veya `kenet_sitl_mixer.py --send`
  RC gönderirken aynı MSP/Gazebo pose kabul kriterlerini ölçer.
- [x] `tools/kenet_sitl_mixer.py --pilot-source virtual` eklendi; fiziksel
  joystick olmadan sanal pilot RC frame'leri Kenet mixer yolundan gönderilebilir.
- [x] External RC checker + Kenet mixer sanal pilot koşusu PASS:
  `logs/sitl/20260629-223523-external-kenet-takeoff-diagnostics.jsonl`.
  Sonuç: `120` sample, ARM+ANGLE `63`, high throttle `53`, altitude gain
  `23.840 m`, max roll/pitch `0.3/0.1`, max motor spread `0`.
- [x] Mixer logunda `TRACKING` + `pilot-target-lost` passthrough doğrulandı:
  `logs/sitl/20260629-223523-kenet-mixer-virtual-external-takeoff.jsonl`;
  ilk 8 kanalda final-pilot delta `0`.
- [x] `tools/kenet_sitl_mixer.py --synthetic-target` eklendi; kamera/video
  olmadan deterministic `target_found=True` ve `source=kenet` testi yapılabiliyor.
- [x] `tools/sitl_synthetic_tracking_check.py --profile centered` PASS:
  `logs/sitl/20260629-continued-synthetic-centered-runner-delay8-diagnostics.jsonl`
  ve `logs/sitl/20260629-continued-synthetic-centered-runner-delay8-mixer.jsonl`.
  Sonuç: ARM+ANGLE `73`, altitude gain `23.838 m`, max roll/pitch `0.4/0.3`,
  motor spread `0`, mixer `TRACKING/source=kenet/target_found=True`,
  final-pilot delta `0`. Runner artık bu mixer sözleşmesini otomatik kontrol
  ediyor; `target_found/source=kenet` veya opsiyonel delta limiti bozulursa
  PASS dönmüyor.
- [~] Offsetli sentetik target-found Gazebo koşuları acceptance geçmedi:
  - combined pitch+yaw:
    `logs/sitl/20260629-224953-external-synthetic-tracking-delayed-diagnostics.jsonl`
  - yaw only +26:
    `logs/sitl/20260629-225200-external-synthetic-yaw-only-diagnostics.jsonl`
  - yaw only -26:
    `logs/sitl/20260629-225340-external-synthetic-yaw-negative-diagnostics.jsonl`
  - pitch only +20:
    `logs/sitl/20260629-225520-external-synthetic-pitch-only-diagnostics.jsonl`
- [x] Micro/P-only ve direct RC nudge izolasyonu eklendi:
  `tools/sitl_synthetic_tracking_check.py` artık micro profilleri düşük limit ve
  P-only çalıştırabiliyor; `tools/sitl_virtual_takeoff_check.py` doğrudan
  `--nudge-delay-seconds/--nudge-yaw` destekliyor.
- [~] Yeni izolasyon sonucu: Kenet'siz direct virtual yaw 1504 bile FAIL:
  `logs/sitl/20260629-continued-direct-virtual-yaw1504-clean-diagnostics.jsonl`.
  Sonuç: ARM+ANGLE `63`, altitude gain `7.257 m`, max roll/pitch `180.0/76.8`,
  motor spread `945`, FC yaw delta `+4`. Synthetic P-only yaw `+4` ve pitch
  `+10` da FAIL. Sıradaki aday Kenet değil, Betaflight/Gazebo closed-loop
  motor output zinciri.
- [x] Direct motor axis probe BF SITL remap'iyle düzeltildi:
  `tools/gazebo_motor_moment_probe.py --pattern-set axis` artık
  `--motor-map bf-sitl` (`3,0,1,2`) kullanıyor. Bu koşuda roll/pitch/yaw moment
  pattern'leri temiz ayrıştı:
  `logs/sitl/20260629-continued-axis-moment-default-bfsitl-after-identity.jsonl`.
- [x] `tools/gazebo_fdm_probe.py` eklendi ve FDM yaw sign iki yönde doğrulandı:
  `logs/sitl/20260629-continued-fdm-yaw-cw-bfsitl.jsonl`,
  `logs/sitl/20260629-continued-fdm-yaw-ccw-bfsitl.jsonl`.
- [~] `yaw_motors_reversed` A/B otomatikleşti ama acceptance geçmedi:
  `tools/sitl_mixer_config.py` ve
  `tools/sitl_virtual_takeoff_check.py --yaw-motors-reversed on|off`.
  ON direct yaw 1504 nudge'a kadar daha sakin, ancak motor spread `945` ve flip
  devam ediyor.
- [x] Closed-loop raw motor UDP capture runner'a eklendi:
  `tools/sitl_virtual_takeoff_check.py --capture-motor-udp`.
  `tools/sitl_motor_udp_probe.py` artık raw BF logical motorları Gazebo packet
  slot sırasına (`3,0,1,2`) remap ediyor ve roll/pitch/yaw axis bias özetliyor.
  Aynı capture yolu virtual RC JSONL logunu da üretiyor.
- [x] Neutral hold30 raw UDP baseline PASS:
  `logs/sitl/20260630-neutral-hold30-motorudp-diagnostics.jsonl`,
  `logs/sitl/20260630-neutral-hold30-motorudp.jsonl`.
  Raw max spread `0`, axis bias `0`.
- [~] Yaw1504 raw UDP koşuları FAIL ama ilk runaway şekli netleşti:
  `logs/sitl/20260630-closedloop-yaw1504-off-motor-udp.jsonl` ve
  `logs/sitl/20260630-closedloop-yaw1504-on-motor-udp.jsonl`.
  İlk büyük raw spread saf yaw pair ayrışması: `yaw_cw_minus_ccw ~= 405-409 us`,
  roll/pitch bias yaklaşık sıfır. Sıradaki aday yaw PID/rate feedback zinciri.
- [x] `tools/sitl_pid_config.py` eklendi; MSP üzerinden Betaflight PID
  tablosunu okuyup yaw P/I/D değerlerini değiştirebiliyor.
- [x] `tools/sitl_virtual_takeoff_check.py` içine `--zero-yaw-pid` ve
  `--yaw-pid P,I,D` eklendi.
- [x] Direct virtual RC yaw 1504 koşusu yaw PID `0/0/0` iken PASS:
  `logs/sitl/20260630-yaw1504-zero-yaw-pid-diagnostics.jsonl`,
  `logs/sitl/20260630-yaw1504-zero-yaw-pid-motor-udp.jsonl`,
  `logs/sitl/20260630-yaw1504-zero-yaw-pid-virtual-rc.jsonl`.
  Sonuç: altitude gain `23.822 m`, max roll/pitch `0.2/0.3`, diagnostics motor
  spread `0`, raw UDP max spread `0.767`, first large raw spread yok.
- [x] İlk yaw P/I sweep koşuldu:
  - `10,0,0` PASS: altitude gain `23.693 m`, raw max spread `0.800`.
  - `15,0,0` PASS: altitude gain `23.795 m`, raw max spread `1.073`.
  - `17,0,0` PASS: altitude gain `23.898 m`, raw max spread `0.683`.
  - `18,0,0` PASS: altitude gain `23.959 m`, raw max spread `0.705`.
  - `19,0,0` PASS: altitude gain `23.658 m`, raw max spread `0.760`.
  - `20,0,0` FAIL: raw max spread `945`, first large yaw bias `382.5 us`.
  - `45,0,0` FAIL: raw max spread `945`, first large yaw bias `407.6 us`.
  - `0,1,0` PASS: altitude gain `23.862 m`, raw max spread `0.792`.
  - `0,2,0` FAIL: raw max spread `945`, first large pitch bias `400.7 us`.
  - `0,5,0` FAIL: raw max spread `945`, first large spread roll/pitch ağırlıklı
    (`roll=207.1`, `pitch=-196.8`, `yaw=-19.0`).
  - `0,20,0` FAIL: raw max spread `945`, first large yaw bias `386.9 us`.
  - `0,80,0` FAIL: raw max spread `945`, first large yaw bias `399.3 us`.
- [~] Güncel yorum: ilk repo-root cwd sweep'i `P20/P22` sınır/flaky gibi
  gösterdi; runner temp-cwd olduktan sonra yaw1504 için `P21` iki PASS, `P22`
  üç PASS / bir FAIL, `P23` iki FAIL verdi. Aynı `P23` yaw1502 ve yaw1503
  hold40 koşularında PASS verdi. Temiz P-only braket artık yaw1504 için `P21`
  güvenli taraf, `P22` sınır/flaky, `P23+` stabil FAIL; P23 için nudge eşiği
  yaw1503/yaw1504 arasında. Eski P20/P22 oynaklığı için kalıcı `eeprom.bin`/FC
  state sızıntısı güçlü aday. Yaw I-only tarafında `I=1` PASS,
  `I=2` FAIL. Bu kalıcı tune önerisi değil; sonraki teşhis yaw integrator/rate
  feedback zaman sırası.
- [x] `tools/sitl_pid_sweep_summary.py` eklendi. Aktif uçuş fazına göre P20 raw
  split nudge'dan `2.943 s`, attitude eşiği `4.621 s` sonra; I2 raw split
  `17.518 s`, attitude eşiği `19.157 s` sonra; I5 raw split `5.051 s`, attitude
  eşiği `6.070 s` sonra geliyor.
- [x] Sweep özeti erken raw motor axis-bias eşiklerini de raporlayacak şekilde
  genişletildi: 25/50/100/200/400us ilk geçiş zamanları ve aktif uçuş attitude
  eşiği tabloya eklendi.
  `0,2,0-repeat1` aynı unified runner ile tekrar FAIL verdi. İki I2 koşusunda
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
- [x] `tools/sitl_pid_sweep_summary.py` nudge'a en yakın, nudge öncesi ve nudge
  sonrası diagnostic sample'ları da raporlayacak şekilde genişletildi. P20/P22
  PASS ve FAIL koşularında nudge anı roll/pitch/yaw temiz ve benzer; after-nudge
  altitude yaklaşık `6.06-6.27 m`. Flakiness nudge anındaki bariz attitude/yaw
  veya irtifa farkıyla açıklanmıyor.
- [x] Betaflight debug görünürlüğü için ilk altyapı eklendi:
  `tools/sitl_diagnostics.py` ve `tools/sitl_dashboard.py` artık
  `MSP_ADVANCED_CONFIG` + `MSP_DEBUG` okuyor. Yeni diagnostic/dashboard
  sample'larında `msp.advanced_config.debug_mode`, `msp.debug_mode` ve
  `msp.debug` alanları var. Bu, `debug_mode` seçildikten sonra yaw setpoint /
  feedforward / PID loop iç sinyalini aynı runner loglarına bağlayacak kapı.
- [x] `tools/sitl_debug_config.py` eklendi ve runner'a bağlandı. Artık
  Betaflight `debug_mode` `PIDLOOP`, `ANGLERATE`, `ANGLE_TARGET` veya numeric
  değerle ayarlanabiliyor; `tools/sitl_virtual_takeoff_check.py --debug-mode
  ...` runner Betaflight'i yönetiyorsa ayarı kaydedip Betaflight'i yeniden
  başlattıktan sonra aynı acceptance koşusunda `MSP_DEBUG` örneklerini
  diagnostics loguna düşürüyor.
- [~] İlk PIDLOOP/ANGLERATE debug-mode koşuları `msp.debug_mode` değerini doğru
  gösterdi; bunun ardından `tools/sitl_debug_calibration.py` eklendi ve canlı
  smoke koşuları alındı. Standalone `PIDLOOP` yaw1700/yaw1504 nonzero verdi
  ama bu mod Betaflight kaynakta loop timing; `ANGLERATE` yaw1700 sıfır kaldı.
  `ANGLE_TARGET` Gazebo P23/yaw1504 koşusunda sadece angle/current-angle
  tarafında sinyal verdi; yaw setpoint slotu `debug[2]` sıfır kaldı. Bu nedenle
  MSP_DEBUG hattı henüz yaw setpoint/rate/PID iç sinyali için güvenilir kanıt
  değil; bu yüzden doğrudan Betaflight instrumentation aşamasına geçildi.
- [x] Doğrudan Betaflight instrumentation başladı:
  `tools/betaflight_yaw_debug_patch.py` marker'lı patch ile `DEBUG_AC_ERROR`
  modunu geçici yaw telemetry kanalına çeviriyor (`setpoint`, `gyroRate`,
  `errorRate`, `P/I/F/S/Sum`). Patch `/home/gz/betaflight` içine uygulandı ve
  `make TARGET=SITL` başarılı oldu.
- [x] Instrumented P21/P23 yaw1504 karşılaştırması alındı:
  `logs/sitl/20260630-debug-acerror-instrumented-yaw1504-p21-*` PASS,
  `logs/sitl/20260630-debug-acerror-instrumented-yaw1504-p23-*` FAIL. P21
  koşusunda altitude gain `31.686 m`, max roll/pitch `0.8/0.6`, raw spread
  `0.802`; aktif uçuşta yaw debug max `setpoint=1`, `gyro=0`, `error=1`,
  `P/Sum=0`. P23 koşusunda raw motor split nudge'dan `0.399 s` sonra geldi,
  aktif debug max `setpoint=1`, `gyro/error=610`, `P/Sum=450`, raw spread
  `945` ve flip. Bu artık P22/P23 sınırını Betaflight yaw rate feedback/P
  eşiğine çok güçlü bağlıyor.
- [x] Instrumented P22/yaw1504 iki tekrar ile sınır/flaky olarak doğrulandı.
  İlk koşu FAIL:
  `logs/sitl/20260630-debug-acerror-instrumented-yaw1504-p22-*`; raw split
  `2.776 s`, attitude `7.668 s`, aktif debug max `setpoint=1`,
  `gyro/error=1740/1741`, `P/Sum=1243`, raw spread `945`. Repeat PASS:
  `logs/sitl/20260630-debug-acerror-instrumented-yaw1504-p22-repeat1-*`;
  altitude gain `31.654 m`, max roll/pitch `4.4/3.7`, raw spread `0.843`,
  aktif debug max P21 gibi sakin: `setpoint=1`, `gyro=0`, `error=1`,
  `P/Sum=0`. Yani P22 tek bir temiz çözüm değil; P21 ile P23 arasında
  koşu-to-koşu küçük başlangıç farklarına hassas metastable eşik.
- [~] P23 runaway için üç fizik/feedback hipotezi A/B kapısından geçirildi,
  hiçbiri tek başına acceptance'ı açmadı:
  - `--iris-yaw-gyro-scale 0.5`:
    `logs/sitl/20260630-debug-acerror-yawscale05-yaw1504-p23-*` FAIL. Aktif
    yaw P/Sum `450 -> 166` düştü ve raw split gecikti (`4.559 s`) ama ilk
    400us axis pitch'e kaydı; attitude FAIL.
  - `--iris-yaw-gyro-scale 0.25`:
    `logs/sitl/20260630-debug-acerror-yawscale025-yaw1504-p23-*` FAIL. Aktif
    yaw P/Sum `102` seviyesine indi ama pitch ağırlıklı raw split ve attitude
    FAIL sürdü.
  - `--iris-rotor-vel-p-gain 0.01`:
    `logs/sitl/20260630-debug-acerror-rotorp001-yaw1504-p23-*` FAIL. Eski SDF
    motor velocity P gain'ine yakın değer raw split'i geciktirdi (`3.619 s`)
    fakat yaw ağırlıklı split ve attitude FAIL devam etti.
  - `--max-step-size 0.001`:
    `logs/sitl/20260630-debug-acerror-step001-yaw1504-p23-*` FAIL. Timing
    çözünürlüğü iyileşmesi P23 runaway'i kapatmadı.
  Geçici teşhis araçları eklendi: BetaflightPlugin artık SDF
  `<yawGyroScale>` okuyabiliyor; launcher/runner temp Iris modelinde
  `--iris-yaw-gyro-scale` ve `--iris-rotor-vel-p-gain` geçebiliyor.
- [~] Bu oturumda `/dev/input/js*` yoktu; bu yüzden fiziksel Tango/joystick
  canlı retest'i koşturulamadı. Canlı retest öncesi no-send bench preflight
  eklendi: `tools/sitl_physical_rc_preflight.py` CH5/AUX1 ARM, CH6/AUX2 Kenet
  state ve CH7/AUX3 mode ayrımını raporlar; CH7 fiziksel switch yoksa
  `--force-mode-pwm 1500` forced ANGLE profili olarak kaydedilir.

Henüz tamamlanmayanlar:

- [x] SITL mixer state machine'i ana `TrackingPipeline` ile aynı AUX helper'a
  bağlandı; hedef kaybında mixer da `AI-ARMED` durumuna düşüyor.
- [x] Kenet state default kanalı tekilleştirildi: `CH6 / AUX2 / index 5`.
- [x] `tools/` SITL safety glue testleri eklendi.
- [x] `tools/analyze_sitl_log.py` veri yokken güvenli sonuç üretmeyecek şekilde
  düzeltilecek; attitude/motor sample yoksa açıkça "veri yok" demeli.
- [x] Log default rate/flush/rotation ve JSONL şema hardening tamamlandı:
  `JsonlLogger` artık batch/time-based flush ve size rotation destekliyor;
  Kenet mixer ve dashboard-started mixer flight log default'u `10 Hz`;
  analyzer varsa tüm `pilot_channels/final_channels` setini okuyor, legacy
  `first8` veya eksik alanlarda schema warning üretiyor.
- [x] Ortak RC/mapping katmanı tamamlandı: production mapping artık
  `kenet/rc_channels.py` içinde; `tools/sitl_rc_channels.py` kanal etiketleri,
  throttle neutral ve SITL wrapper katmanı olarak bu mapping'i re-export ediyor.
  Analyzer, diagnostics, dashboard, RC bridge ve monitor etiketleri buradan
  besleniyor. `sitl_dashboard.py` ana gözlem aracı olarak kabul edildi;
  `state_monitor.py` artık compatibility shim olarak dashboard'a yönlendiriyor
  (`--legacy` eski state-only web UI için), `rc_monitor.py` ise dar
  joystick/raw RC monitor olarak kaldı ve bridge mapping'i kullanıyor.
- [x] Kısa JSONL schema dokümanı eklendi: `docs/sitl-jsonl-schema.md`.
  `JsonlLogger` base fields, `kenet_mixer_sample`, `dashboard_sample`,
  `diagnostic_sample`, virtual RC ve motor UDP event sözleşmeleri burada.
- [x] Monitor/mapping araçları tek kaynak prensibine göre sadeleştirildi.
- [x] PID yönleri bilinçli hedef hareketleriyle tek tek doğrulandı:
  `tools/sitl_pid_direction_check.py` gerçek `FlightController` ile
  centered/right/left/small/large/lost sentetik hedeflerini kontrol ediyor.
  Canlı hızlı çıktı PASS: target_right yaw `1548`, target_left yaw `1452`,
  target_small pitch `1514`, target_large pitch `1486`, centered/lost neutral.
  Bu controller işaret gate'idir; Gazebo offsetli target-found fizik kabulü
  hâlâ Betaflight yaw feedback sınırından ayrı değerlendiriliyor.
- [~] Hedef kaybı test matrisi production/SITL unit gate, P11 AUX drop canlı
  smoke ve sentetik target-loss runner gate'i ile sabitlendi. Runner artık
  `--synthetic-target-loss-after-seconds` ile hedefi kaybettirip
  `pilot-target-lost` ve `AI-ARMED` sample sayılarını kabul kriteri yapabiliyor;
  uzun hedef kaybından sonra production/SITL ortak re-entry lockout devrede:
  pilot switch'i track eşiğinin altına indirmeden tekrar TRACKING yok. Gerçek
  kamera/video target-loss matrisi ayrı açık. 2026-06-30 canlı final gate PASS:
  `logs/sitl/20260630-synthetic-target-loss-live-final-diagnostics.jsonl` ve
  `logs/sitl/20260630-synthetic-target-loss-live-final-mixer.jsonl`; checker
  altitude gain `58.505 m`, max roll/pitch `0/0`, motor spread `0`; mixer
  `source=kenet 200`, `pilot-target-lost 19`, `AI-ARMED 650`, max final-pilot
  delta `0`.
- [x] MSP transport SITL'e native TCP ile bağlandı: `kenet/msp.py` artık
  `tcp://host:port` / `host:port` endpointlerini destekliyor, `kenet.py`
  `--msp-tcp 127.0.0.1:5761` alıyor ve `tools/kenet_msp_smoke.py` production
  `MSPConnection` ile `MSP_API_VERSION`, `MSP_RC`, `MSP_ATTITUDE` ve opsiyonel
  `MSP_SET_RAW_RC` smoke testi yapıyor. Unit testler transport ve smoke CLI
  yolunu kapsıyor. Canlı Betaflight SITL smoke de geçti:
  `tools/kenet_msp_smoke.py --msp-tcp 127.0.0.1:5761 --timeout 2.0` -> PASS,
  API `1.48`, `MSP_RC` ve `MSP_ATTITUDE` okundu. Canlı
  `MSP_SET_RAW_RC` readback gate de geçti:
  `--set-raw-rc 1500,1600,1000,1400,2000,2000,1500,1500` -> `set_raw_rc_check: PASS`;
  pilot/AETR gönderim Betaflight `MSP_RC` RPYT readback'inde
  `[1500,1600,1400,1000,...]` olarak doğrulandı. Ana `kenet.py` pipeline
  SITL MSP smoke da geçti: virtual RC UDP frame
  `1500,1500,1120,1500,1000,2000,1500,1500` ile AUX High/TRACKING görüldü,
  outbound MSP frame throttle'ı pilot değeri `1120` olarak korudu
  (`send=[1500,1515,1120,1535,1000,2000,1500,1500]`). Bu smoke sırasında
  `MSP_RC` RPYT readback'i production pipeline içinde pilot/AETR düzenine
  çevrilmediği için ilk deneme hatayı açığa çıkardı; fix
  `kenet/rc_channels.py` + pipeline dönüşümüyle eklendi. İkinci canlı P11 smoke
  AUX High -> Mid düşüşünü doğruladı: `[TRACKING] CH6:2000 ... send=[...]`
  sonrası `[AI-ARMED] CH6:1500` geldi ve Mid'e indikten sonra yeni `send=`
  satırı oluşmadı. Target-lost için production unit gate eklendi:
  `TrackResult(found=False)` yeni `MSP_SET_RAW_RC` göndermiyor; uzun kayıpta
  state `TRACKING -> AI-ARMED`. Eski `socat` pseudo-serial yolu artık aktif
  ön koşul değil; sadece native TCP'ye karşı opsiyonel regresyon fallback'i.
- [x] Gazebo'da virtual RC ile gövde/uçuş fiziği retest edildi; sonuç takla.
- [x] Iris takla problemi virtual RC hattında çözüldü: motor map/RC kaynak
  değil, BetaflightPlugin yaw gyro işareti pozitif geri besleme yaratıyordu.
  Dış plugin patch'i ve rebuild sonrası virtual takeoff check PASS.
- [ ] Fiziksel Tango/joystick yolu bu PASS sonrası tekrar eklenecek; amaç aynı
  runner kriterlerini fiziksel RC ile de korumak. Bunun için external RC gözlem
  modu hazır.
- [~] Tango 2 üzerindeki ikinci fiziksel three-state switch USB joystick
  çıktısında görünür hale getirilecek veya şimdilik pas geçilecek.

---

## Claude Notu Sonrası Codex Kritik Değerlendirmesi

Haklı bulduğum ana eleştiri: SITL yolu artık çok faydalı, fakat gerçek sistemi
doğrulaması için `tools/kenet_sitl_mixer.py` ile `kenet/pipeline.py` aynı state
machine davranışını paylaşmalı. Şu an mixer gerçek controller/tracker kodunu
kullanıyor, bu iyi; fakat AUX threshold, hedef kaybı ve re-init mantığı ayrı
kopya olarak duruyor. Bu yüzden bir sonraki mühendislik adımı Gazebo'yu daha
fazla büyütmek değil, harness davranışını üretim davranışıyla eşitlemek olmalı.

Kritik yol haritası ekleri:

1. State machine parity: AUX→state, hedef kaybı ve tracker re-init tek kaynak.
   Bu madde ilk test setiyle kapatıldı.
2. AUX default parity: Kenet state için `CH6 / AUX2 / index 5` kararını ana
   pipeline, joystick, mixer, dashboard ve dokümanlarda aynı hale getirme.
   Bu madde kapatıldı; eski AUX4 setup için `--aux-ch 7` açık verilmeli.
3. SITL safety tests: passthrough, only pitch/yaw override, target lost,
   tracker unavailable, threshold boundary ve RC packet format testleri.
   Bu madde ilk test setiyle kapatıldı.
4. Logging hardening: analyzer veri yokken false all-clear vermiyor; sırada log
   rate/flush/rotation ve record schema netleşecek.
5. Monitor consolidation: `sitl_dashboard.py` ana gözlem aracı, diğer monitorlar
   mapping kopyalamayacak.
6. MSP observability: dashboard/analyzer artık `flight_mode_flags` ham bitini
   sabit anlamla yorumlamıyor; `MSP_BOXNAMES` + `MSP_BOXIDS` + `MSP_STATUS_EX`
   üzerinden `active_modes` üretilecek.
7. Gazebo hardening: smoke test temp eeprom, motor PWM merkezli PASS kriteri,
   headless rendering opsiyonu, timing/max-step izolasyonu ve ANGLE mode/CH7
   takeoff doğrulaması.
8. Takla analizi: Kenet `IDLE` ve pilot passthrough kanıtlandı. Zamanlama
   izolasyonu değerli kalıyor. Statik motor mapping alarmı raw packet slotlarını
   logical BF motoru sandığı için yanıltıcıydı; corrected BF SITL remap axis
   probe motor eksenlerini temiz gösterdi. Sıradaki ana aday closed-loop
   sırasında Betaflight'in actual motor UDP çıktısı ve yaw hold/rate cevabı.

Pratik sonuç:

```text
State-machine parity + safety tests ilk turda tamamlandı.
Log analyzer false all-clear düzeltmesi de tamamlandı.
Dashboard/analyzer active_modes decode düzeltmesi eklendi.
   Takla analizi için virtual RC + corrected motor/FDM izolasyon koşuları
   tamamlandı; motor axis ve FDM yaw sign temiz, yaw_motors_reversed ON/OFF ise
   direct yaw 1504 acceptance'ı kapatmadı. Raw motor UDP capture ilk runaway'in
   saf yaw pair ayrışması olarak başladığını gösterdi.
Genel logging tarafında sıradaki iş: diagnostic JSONL çıktısını canlı test loglarıyla kalıcı analiz akışına bağlamak.
```

---

## Son Eklenen / Güncellenen Araçlar

### `tools/kenet_sitl_mixer.py`

Yeni ana SITL entegrasyon aracı. Pilot RC girdisi ile Kenet'in görsel takip/PID
çıktısını tek RC packet içinde birleştirir.

Güvenlik: `--send` modunda kapanışta throttle-low, roll/pitch/yaw centered ve
ARM-low güvenli son RC frame'i gönderir. Ardışık RC gönderimleri nominal
periyodun 2 katını aşarsa `RC send interval ... exceeded watchdog ...` warning'i
ve JSONL `tx_warning` event'i üretir.

Davranış:

| Kenet state | Target | Final RC davranışı |
|---|---|---|
| IDLE | fark etmez | tüm kanallar pilottan |
| AI-ARMED | fark etmez | tüm kanallar pilottan |
| TRACKING | yok | tüm kanallar pilottan |
| TRACKING | var | pitch/yaw Kenet'ten, diğerleri pilottan |

Önemli güvenlik tarafı:

- Throttle Kenet tarafından sabitlenmiyor.
- Roll pilotta kalıyor.
- Hedef yoksa pitch/yaw da pilota geri düşüyor.
- Tracker/OpenCV hatası varsa script artık crash atmıyor; pilot passthrough
  yapıyor ve `source=pilot-tracker-unavailable` yazıyor.

Çalıştırma:

```bash
python tools/kenet_sitl_mixer.py --device "$JOY_DEV" --camera test-2.mp4
python tools/kenet_sitl_mixer.py --device "$JOY_DEV" --camera test-2.mp4 --send --print-hz 5
python tools/kenet_sitl_mixer.py --pilot-source virtual --virtual-script takeoff \
  --virtual-throttle 1750 --virtual-kenet-pwm 2000 --virtual-mode-pwm 1500 \
  --no-vision --send
python tools/kenet_sitl_mixer.py --pilot-source virtual --virtual-script takeoff \
  --virtual-throttle 1750 --virtual-kenet-pwm 2000 --virtual-mode-pwm 1500 \
  --synthetic-target --synthetic-target-delay-seconds 18 --send
python tools/sitl_synthetic_tracking_check.py \
  --profile centered --synthetic-target-delay-seconds 0
python tools/sitl_synthetic_tracking_check.py \
  --profile centered \
  --synthetic-target-loss-after-seconds 20 \
  --virtual-hold-seconds 80 \
  --min-target-lost-samples 5 \
  --min-ai-armed-samples 1 \
  --max-abs-delta 0
```

Virtual pilot modu test otomasyonu içindir: joystick açmaz, CH3 throttle,
CH5/AUX1 ARM, CH6/AUX2 Kenet state ve CH7/AUX3 mode değerlerini scriptli üretir.
Kamera kapalı veya hedef yokken `TRACKING` state'inde bile final RC pilot
kanallarıyla aynı kalmalıdır. `--synthetic-target` ise kamera/video açmadan
found-target yolunu çalıştırır; `--synthetic-target-delay-seconds`, takeoff
sırasında hedefi merkezde tutup offset'i havalandıktan sonra uygulamak içindir.
`--synthetic-target-loss-after-seconds` ise aynı sentetik hedefi uçuş sırasında
kaybettirip target-loss passthrough ve `TRACKING -> AI-ARMED` düşüşünü ölçer.
Bu gate'te `--virtual-hold-seconds`, diagnostics penceresinden uzun tutulmalı;
aksi halde mixer safe-exit/disarm sonrası düşüş/tumble acceptance sonucunu
kirletebilir.

### `tools/sitl_rc_bridge.py`

Minimal RC -> Betaflight SITL bridge. Python stdlib ile Linux joystick okur ve
Betaflight SITL `rc_packet` formatında UDP `9004` portuna gönderir.

Kullanım:

```bash
python tools/sitl_rc_bridge.py --device "$JOY_DEV" --dry-run
python tools/sitl_rc_bridge.py --device "$JOY_DEV" --send --verbose
```

### `tools/sitl_virtual_rc.py`

Fiziksel kumanda/joystick olmadan Betaflight SITL'e deterministic RC packet
gönderir. Gazebo takla debug'ında önce bu yol kullanılacak.

```bash
python tools/sitl_virtual_rc.py --script takeoff --throttle 1550 --mode-pwm 1500
python tools/sitl_virtual_rc.py --script takeoff --throttle 1550 --mode-pwm 1500 --send
```

Bu araç CH3 throttle rampası, CH5 ARM ve CH7/AUX3 mode değerini sanal olarak
üretir. Virtual RC temizleşmeden fiziksel Tango/joystick yolundaki gecikme veya
mapping sorunları ana kök neden kabul edilmeyecek.

### `tools/sitl_configure_modes.py`

Betaflight Configurator açmadan canlı SITL oturumuna mode range yazar.
Virtual RC Gazebo retest'lerinden önce çalıştırılmalı:

```bash
python tools/sitl_configure_modes.py
```

Varsayılanlar:

```text
ARM          AUX1 / CH5 high 1600-2100
MSP OVERRIDE AUX2 / CH6 high 1700-2100
ANGLE        AUX3 / CH7 mid  1300-1700
HORIZON      AUX3 / CH7 high 1700-2100
```

`tools/sitl_mode_status_check.py` aynı range setini sanal RC case'leriyle
doğrular ve `MSP_STATUS_EX` üzerinden aktif mode'ları ve arming disable
flag'lerini raporlar.

### `tools/sitl_motor_udp_probe.py`

Betaflight SITL'in raw servo/motor UDP çıkışını `9001/udp` üzerinden dinler.
Gazebo'nun motor komutu aldığı `9002/udp` portuna müdahale etmez.

```bash
python tools/sitl_motor_udp_probe.py --duration 30
```

Takla izolasyonunda MSP içinde görülen motor komutu ile simülasyona giden raw
motor çıkışını ayırmak için kullanılır.

### `tools/gazebo_motor_moment_probe.py`

Betaflight'i bypass edip Gazebo BetaflightPlugin'e direct motor speed paketi
gönderir. Motor pulse öncesi/sonrası Gazebo dynamic pose ve IMU örneği alarak
motor->moment işaretini ölçer.

```bash
python tools/gazebo_motor_moment_probe.py \
  --base-speed 0.30 --pulse-speed 0.45 \
  --pulse-seconds 0.35 --settle-seconds 0.25
```

### `tools/check_sitl_env.sh`

Portable SITL/Gazebo ortam kontrol aracı. Ubuntu sürümünü, `gz sim` sürümünü,
`$AEROLOOP_GAZEBO` ve `$BETAFLIGHT_ROOT` altındaki gerekli dosyaları, joystick
cihazını ve `9002/9003/9004/5761/6761` port durumunu raporlar.

```bash
tools/check_sitl_env.sh
```

### `tools/run_gazebo_betaflight.sh`

Gazebo Harmonic launcher. `$AEROLOOP_GAZEBO` altındaki model/world/plugin
path'lerini export eder ve seçilen world dosyasını çalıştırır.

```bash
tools/run_gazebo_betaflight.sh --world test_betaflight.sdf --headless
tools/run_gazebo_betaflight.sh --world betaloop_iris_betaflight_demo_harmonic.sdf
tools/run_gazebo_betaflight.sh --world betaloop_iris_betaflight_demo_harmonic.sdf --max-step-size 0.001 --fix-iris-imu-pose --fix-iris-motor-map
tools/run_gazebo_betaflight.sh --world betaloop_iris_betaflight_demo_harmonic.sdf --headless --max-step-size 0.001 --fix-iris-imu-pose --fix-iris-motor-map
```

Coarse `max_step_size` değerleri için uyarı verir. `--max-step-size` dış repo
dosyasını değiştirmeden geçici world kopyasında physics step override yapar.
`--fix-iris-motor-map` dış model dosyasını değiştirmeden geçici model kopyasında
Betaflight motor map'i düzeltir. `--headless` varsayılan olarak
`--headless-rendering` geçirir.

### `tools/gazebo_stats_monitor.py`

Gazebo `/stats` topic'inden `real_time_factor` örnekleri alır ve timing
jitter/spread raporu üretir.

```bash
python tools/gazebo_stats_monitor.py --samples 30 --interval 0.5
```

### `tools/sitl_diagnostics.py`

Yapısal SITL sorunlarını tek komutta loglar. Gazebo stats, dashboard API
gecikmesi, MSP arm/mode/attitude, joystick kanalları, FC-pilot RC deltaları,
port/process durumu ve Gazebo motor mapping analizi aynı JSONL dosyasına yazılır.

```bash
python tools/sitl_diagnostics.py --samples 20 --interval 0.25
```

İlk kuru çalıştırma sonucu eski statik SDF kontrolünde alarm verdi:

```text
motor mapping ok: False
mismatched BF motors: 0,1,2,3
```

Güncel yorum: Betaflight SITL motor output'u UDP packet slotlarına `3,0,1,2`
olarak remap ediyor. Bu yüzden raw packet slotlarını logical BF motoru gibi
yorumlayan statik mapping kontrolü yanıltıcı olabilir. Corrected axis probe
`--motor-map bf-sitl` ile roll/pitch/yaw moment eksenlerini temiz doğruladı.

### `tools/gazebo_sitl_motor_smoke.py`

Gazebo + Betaflight SITL motor hattını tek komutla doğrular. Script Gazebo'yu
headless başlatır, Betaflight SITL'i açar, AUX1 düşükteyken arming flag'lerinin
temizlenmesini bekler, AUX1'i yükseltir, düşük throttle uygular, `MSP_MOTOR`
okur ve Gazebo rotor joint orientation değerlerinin değiştiğini kontrol eder.
Betaflight varsayılan olarak temp cwd içinde başlar; eski repo-root `eeprom.bin`
durumu yalnız `--betaflight-cwd repo` ile yeniden üretilebilir. Script stdout'a
`run_dir`, `betaflight_cwd` ve `eeprom_path` metadata'sını yazar. PASS kararının
ana sinyali `MSP_MOTOR` motor PWM yükselişidir; rotor joint hareketi default'ta
uyarı/ek kanıt, `--require-rotor-motion` ile katı kabul kapısıdır.

```bash
python tools/gazebo_sitl_motor_smoke.py
```

Son doğrulama sonucu:

```text
flags_after_throttle=0x00000000 none
msp_motor=1402,1402,1402,1402
result=PASS motor output reached Gazebo and rotor joints moved
```

### `tools/sitl_dashboard.py`

Gazebo, Betaflight SITL ve Kenet testleri sırasında canlı izleme arayüzü.
Joystick pilot kanallarını, Kenet state'i, ARM komutunu, Betaflight armed
durumunu, FC'nin gördüğü RC kanallarını, MSP motor değerlerini, attitude
telemetrisini ve raw axis/button değerlerini aynı ekranda gösterir. Ayrıca
arayüzden Gazebo, Betaflight SITL ve Kenet SITL mixer process'leri başlatılıp
durdurulabilir; her process için log tail gösterilir. Dashboard-started
Betaflight varsayılan olarak `logs/sitl/` altındaki temp cwd içinde başlar;
snapshot/API `working_dir` ve `eeprom_path` alanlarını gösterir. Eski repo-root
`eeprom.bin` davranışı yalnız `--betaflight-cwd repo` ile opt-in. Process
start/stop API'si default olarak loopback istemcilerle sınırlıdır; uzak
istemciler için `--allow-remote-control` açıkça verilmelidir. Dashboard Kenet
sender başlatmadan önce aktif `sitl_rc_bridge.py`, `sitl_virtual_rc.py` veya
`kenet_sitl_mixer.py --send` süreçlerini kontrol eder; duplicate RC sender
bulursa başlatmayı reddeder.

```bash
python tools/sitl_dashboard.py --open
```

Dashboard varsayılan olarak `logs/sitl/` altında JSONL uçuş logu üretir.
Logda Kenet state'i, ARM komutu, autopilot mode, pilot RC, FC RC, MSP motor,
armed state, arming disable flags ve attitude örnekleri bulunur. Arayüzdeki
`Mark Event` butonu takla/kopma anını JSONL içine zaman damgası olarak işler.
Start Gazebo butonu varsayılan olarak GUI modunda, `--max-step-size 0.001`,
geçici IMU pose fix ve geçici motor map fix ile çalışacak şekilde ayarlandı.

Not: Betaflight Configurator açıksa MSP alanları offline kalabilir; SITL'in
`5761/tcp` MSP hattı aynı anda tek istemci kabul eder.

### `tools/analyze_sitl_log.py`

Dashboard ve Kenet mixer JSONL loglarını özetler. Takla veya kontrol kopması
sonrası attitude, motor spread, armed state, Kenet state/source ve kanal
sapmalarını hızlıca görmek için kullanılır.

```bash
python tools/analyze_sitl_log.py
```

Belirli logları incelemek için:

```bash
python tools/analyze_sitl_log.py logs/sitl/20260623-*.jsonl
```

### Kenet mixer flight log

`tools/kenet_sitl_mixer.py` artık varsayılan olarak ayrıntılı JSONL log yazar.
Her örnekte state, source, target bilgisi, pilot/final RC kanalları, ilk 8
kanal deltalari, PID hata/çıkış değerleri ve tracker hatası bulunur.

### `tools/rc_monitor.py`

Joystick ve mapped RC kanallarını görsel veya terminal modunda izler. Tk yoksa
terminal moduna düşebilir.

### `tools/state_monitor.py`

Compatibility shim. Varsayılan olarak `sitl_dashboard.py` başlatır ve
device/channel argümanlarını aktarır. Eski tek dosyalık stdlib web GUI yalnızca
`--legacy` ile çalışır.

```bash
python tools/state_monitor.py --open
python tools/state_monitor.py --legacy --open
```

### `tools/sitl_rc_probe.py`

SITL'e RC gönderip ardından MSP/TCP üzerinden `MSP_RC` okuyarak Betaflight'ın
ne gördüğünü doğrular. Configurator kapalıyken kullanılmalıdır.

---

## Switch / Kanal Düzeni

Güncel varsayılan mapping:

| İşlev | Betaflight kanal | Joystick kaynak | Not |
|---|---:|---|---|
| Roll | CH1 | Axis 0 | 1000-2000 |
| Pitch | CH2 | Axis 1 invert | 1000-2000 |
| Throttle | CH3 | Axis 2 | düşük 1000, yüksek 2000 |
| Yaw | CH4 | Axis 3 | 1000-2000 |
| ARM | CH5 / AUX1 | Axis 4 | 1000/2000 |
| Kenet state | CH6 / AUX2 | Axis 6 | 1000/1500/2000 |
| Autopilot mode | CH7 / AUX3 | Axis 5 veya `--force-mode-pwm` | Tango ayarı beklerken PWM sabitlenebilir |

Kenet state:

```text
AUX2 LOW  -> IDLE
AUX2 MID  -> AI-ARMED
AUX2 HIGH -> TRACKING
```

Tasarım kararı:

`AUX2 / CH6` Betaflight mode'a bağlanmayacak. Bu kanal sadece Kenet state için
ayrıldı. Betaflight ARM için `AUX1 / CH5` kullanılacak.

Takla teşhis komutu:

```bash
python tools/kenet_sitl_mixer.py --device "$JOY_DEV" --camera test-2.mp4 \
  --send --force-mode-pwm 1500 --print-hz 5
```

Bu komut CH7/AUX3'ü `1500` değerinde tutar. Betaflight Configurator Modes
tab'inde ANGLE range `AUX3 1300-1700` olarak ayarlanırsa ilk fizik retest'i
stabilized mode ile yapılır.

---

## OpenCV / Venv Hatası ve Çözüm

Bir testte şu hata görüldü:

```text
TrackerCSRT is unavailable in this OpenCV build
```

Kök neden: Komut yanlış Python ortamıyla çalışmıştı. Sistem `python3` komutu
başka bir projenin venv'ine gidiyordu. O ortamda OpenCV contrib tracker
modülleri yoktu.

Doğru ortam:

```bash
fpv-test
which python
python -c "import cv2; print(cv2.__file__, hasattr(cv2, 'TrackerCSRT_create'))"
```

Beklenen:

```text
$FPV_ROOT/fpv_env/bin/python
True
```

Kod tarafında yapılan iyileştirme:

- `tools/kenet_sitl_mixer.py` artık tracker yokken kapanmıyor.
- Hata mesajında doğru venv öneriliyor.
- RC çıkışı pilot passthrough olarak devam ediyor.

---

## Git / Dosya Hijyeni

Takip edilmemesi gerekenler `.gitignore` tarafında tutuluyor:

- `.claude/`
- `.codex/`
- `.agents/`
- `.venv/`
- `fpv_env/`
- `.pytest_cache/`
- `test-*.mp4`
- `test.mp4`
- `eeprom.bin`

Not: `eeprom.bin` Betaflight SITL runtime artifact'ıdır; commit edilmemeli.

---

## Güncellenen Plan

SITL yol haritası baştan sona sadeleştirildi ve genişletildi:

```text
plan-sitl.md
```

Yeni plan şu sırayı öneriyor:

1. Proje venv ve OpenCV doğrulaması.
2. Kumanda/joystick mapping doğrulaması.
3. Betaflight SITL + Configurator bağlantısı.
4. Minimal RC UDP bridge.
5. Betaflight mode ve güvenlik ayarı.
6. Kenet SITL mixer / RC arbiter.
7. Tekrarlanabilir test matrisi.
8. PID yön ve limit doğrulaması.
9. Loglama / GCS ile izleme.
10. MSP transport karar ve smoke test.
11. Ana Kenet pipeline'ı SITL MSP hattına bağlama. Canlı smoke ve AUX drop
    gate geçti; target-loss passthrough production unit gate ile sabitlendi.
12. Portable Gazebo/fizik simülasyonu.
13. Canlı SITL dashboard ile durum/komut izleme ve start/stop kontrol paneli.
14. Donanım öncesi çıkış kriterleri.

Planın temel kararı hafif güncellendi:

```text
Kontrol zinciri kanıtlandı; Gazebo portable setup hazırlandı.
Tam fizik davranışı ve MSP hattı ayrı ayrı doğrulanacak.
```

---

## Yakın Sıradaki İş

Virtual RC ile takla önce tekrar üretildi, sonra yaw gyro sign patch'i ile
neutral takeoff runner PASS verdi. Kenet mixer sanal pilotu da external checker
ile PASS aldı. Offsetli target-found ve direct yaw 1504 ise normal PID ile flip
üretti. Corrected motor axis ve FDM yaw sign kapıları temiz; `yaw_motors_reversed`
ON/OFF acceptance'ı kapatmadı. Closed-loop raw motor UDP capture, ilk runaway'in
saf yaw pair ayrışması olarak başladığını gösterdi. Yaw PID `0/0/0` yapılınca
aynı yaw 1504 virtual RC koşusu PASS verdi. İlk repo-root cwd sweep'te
P10/P15/P17/P18/P19 PASS, P20 ilk koşuda FAIL ama repeatlerde PASS, P22 bir
PASS/bir FAIL, P23/P25/P45 FAIL; temp-cwd runner sonrası yaw1504 için P21
`2 PASS / 0 FAIL`, P22 `3 PASS / 1 FAIL`, P23 `2 FAIL / 0 PASS`. Yeni P23
yaw1502 hold40 PASS (`raw_spread=0.426`) ve P23 yaw1503 hold40 PASS
(`raw_spread=0.645`) verdi. I1 PASS, I2/I5/I20/I80 FAIL görüldü. Yakın iş artık
fiziksel RC'yi geri eklemek değil; P22/P23 sınırını Betaflight yaw PID
scale/rate feedback içinde, özellikle nudge/setpoint ölçeği üstünden açıklamak
ve I2 düşük-genlik sınırını gerekirse uzun hold/repeat ile pekiştirmek.

2026-06-30 son kontrol: Claude'un kaynak SDF/plugin yönündeki güncellemeleri
mevcut motor-map/yaw-gyro-sign hükmümüzle uyumlu. `CLAUDE.md` otomatik yüklenen
proje notu olarak güncellendi; artık virtual RC/Gazebo runner'ı, aktif yaw PID
sweep sınırlarını ve fiziksel RC'nin şimdilik izole edildiğini açıkça söylüyor.
Doğrulama: o anki snapshot'ta hedefli suite temizdi; güncel tam suite
`237 passed`; hem bu repo hem
`/home/gz/aeroloop_gazebo` için `git diff --check` temiz.

2026-06-30 Codex takip kontrolü: Claude güncellemeleri tekrar okundu. Kaynak SDF
identity rotor listesi ve plugin yaw gyro `Z` as-is yorumu mevcut hükümle
uyumlu. `tools/sitl_virtual_takeoff_check.py` içinde GYROPID_SYNC/MSP-ready
kilitlenmesini azaltmak için Betaflight başlatıldıktan sonra MSP beklemeden
önce Gazebo plugin'ine kısa sıfır-motor bootstrap'i gönderiliyor; debug-mode
restart sonrası da aynı bootstrap tekrarlanıyor. İlk P21 tekrarında restart
sonrası süreç yine erken kapandığı için runner'a
`--betaflight-restart-settle-seconds` eklendi; varsayılan 2 saniye bekleme ile
P21 instrumented PASS koşusu tamamlandı. Hedefli doğrulama:
`./fpv_env/bin/python -m pytest -q tests/test_sitl_virtual_takeoff_check.py tests/test_sitl_pid_sweep_summary.py tests/test_betaflight_yaw_debug_patch.py`
-> `34 passed`; `git diff --check`, `/home/gz/aeroloop_gazebo` ve
`/home/gz/betaflight` diff check temiz. P21 instrumented PASS karşılaştırması
artık alındı; sıradaki gerçek kapı P22 sınır/flaky davranışını aynı aktif-debug
ölçümüyle açıklamak veya P23 runaway için Betaflight yaw rate ölçeği/işareti
üzerinde fix hipotezi geliştirmek.

2026-06-30 son kayıt: fiziksel RC hâlâ izole; virtual RC runner ile P23/yaw1504
runaway için ilk pratik fix kapısı bulundu. Baseline instrumented P23 FAIL
koşusunda raw motor split nudge'dan `0.399 s` sonra geldi, raw spread `945`,
aktif debug `gyro/error=610`, `P/Sum=450` ve attitude FAIL. Aynı P23/yaw1504
koşusu Betaflight rate profili `yaw_rc_rate=5`, `yaw_rate=30`,
`yaw_rate_limit=120` ile iki kez PASS verdi:
`logs/sitl/20260630-debug-acerror-yawauth-r5-s30-l120-yaw1504-p23-*` ve
`logs/sitl/20260630-debug-acerror-yawauth-r5-s30-l120-yaw1504-p23-repeat1-*`.
PASS koşularında altitude gain `31.395/31.662 m`, max roll/pitch
`0.3/0.1` ve `3.2/1.6`, raw spread `0.616/0.614`; aktif debug sıfır rejimde
kaldı. `tools/sitl_virtual_takeoff_check.py` artık bu ölçülmüş profili
`--safe-yaw-authority` ile uyguluyor. Hedefli doğrulama:
`./fpv_env/bin/python -m pytest -q tests/test_sitl_virtual_takeoff_check.py tests/test_sitl_rate_config.py`
-> `32 passed`; ayrıca P23 baseline + iki safe-profile logu
`tools/sitl_pid_sweep_summary.py` ile karşılaştırıldı.

2026-06-30 I2 repeat + ölçekleme: `tools/sitl_pid_sweep_summary.py` raw motor
axis bias için 25/50/100/200/400us erken geçiş zamanlarını ve aktif uçuş
attitude eşiğini raporlayacak şekilde genişletildi. `0,2,0-repeat1` aynı unified
runner ile tekrar FAIL verdi. Eski I2 ve repeat birlikte okununca 25us axis
drift nudge'dan yaklaşık `16.6-16.7 s`, raw split `17.35-17.52 s`, attitude
eşiği `19.16-19.19 s` sonra geliyor. Daha düşük yaw nudge ölçeğinde `1502`
PASS, `1503` ise hold30'da raw split üretip post-disarm attitude büyütüyor;
hold40'ta aktif uçuşta da attitude eşiğini geçiyor (`active_att_dt=28.257 s`).
Bu, I2 kırılımını tekil log şansından çıkarıp genlik ve süreye bağlı geç gelişen
integrator/rate-feedback bozulumuna daha güçlü bağlıyor.

Eski takla repro akışı:

```bash
tools/run_gazebo_betaflight.sh \
  --world betaloop_iris_betaflight_demo_harmonic.sdf \
  --headless \
  --max-step-size 0.0025 \
  --fix-iris-imu-pose \
  --fix-iris-motor-map
"$BETAFLIGHT_ROOT/obj/main/betaflight_SITL.elf"
python tools/sitl_configure_modes.py
python tools/sitl_virtual_rc.py --script takeoff --throttle 1550 --mode-pwm 1500 --send
python tools/sitl_diagnostics.py --rc-source virtual \
  --virtual-throttle 1550 --virtual-arm-pwm 2000 --virtual-mode-pwm 1500 \
  --samples 20 --interval 0.25
```

Eski takla repro kanıtı:

```bash
python tools/analyze_sitl_log.py logs/sitl/20260629-212308-diagnostics.jsonl
```

```text
max roll: -180.0 deg
max motor spread: 945 us
first large attitude sample motors: [2000,1176,1055,1277]
```

Bundan sonraki pratik sıra:

```text
1. P-only tarafında temiz-cwd braket yaw1504 için `P21` PASS, `P22`
   sınır/flaky, `P23+` stabil FAIL; yaw1502 ve yaw1503'te `P23` hold40 PASS.
   Instrumented `DEBUG_AC_ERROR` karşılaştırması artık tamamlandı: P21 aktif
   debug sakin kalırken P23'te yaw gyro/error/P/Sum runaway oluyor; P22 ise bir
   tekrar PASS, bir tekrar FAIL ile bu iki rejim arasında metastable kaldı.
   Yaw gyro scale, rotor velocity P gain ve physics step tek başına çözmedi.
   Betaflight yaw authority profili (`yaw_rc_rate=5`, `yaw_rate=30`,
   `yaw_rate_limit=120`) aynı P23/yaw1504 koşusunu iki kez PASS'a taşıdı.
   Artık ana iş bu profili runner default'u yapmadan açık bir `--safe-yaw-authority`
   acceptance profili olarak kullanmak ve gerçek Kenet/physical RC'yi bunun
   üstünde geri eklemek.
2. I2 tarafında gerekirse `yaw 1502` uzun hold veya `yaw 1503` repeat ile
   sınırın deterministikliğini pekiştir.
3. Bu gate temizlenmeden fiziksel /dev/input/js* yolunu geri ekleme.
4. Fiziksel RC geri döndüğünde aynı external checker'ı `sitl_rc_bridge.py` veya
   joystick kaynaklı `kenet_sitl_mixer.py` ile koş.
5. Kamera/test video ile gerçek target-found TRACKING koşusu artık ölçüldü;
   sıradaki video tarafı nonzero pitch/yaw limitli command-response gate'i.
6. MSP transport ve ana `kenet.py` hattına ancak bu RC/mixer/fizik kanıtları
   temiz kaldıktan sonra dön.
```

2026-06-30 Codex kontrol kaydı: Claude'un son safe-yaw-authority ve I2
ölçekleme güncellemeleri kod/test tarafında doğrulandı. `tools/sitl_virtual_takeoff_check.py`
`--safe-yaw-authority`, bootstrap ve restart settle bayraklarını taşıyor;
`tools/sitl_pid_sweep_summary.py` raw motor axis threshold ve `active_att_dt`
alanlarını raporluyor. Tam test paketi tekrar koşuldu:
`./fpv_env/bin/python -m pytest -q` -> `237 passed`. Donanım öncesi tekrar
koşulacak gate'ler `docs/sitl-acceptance-procedure.md` altında kayda alındı.
Fiziksel RC hâlâ izole; `/dev/input/js*` gelmeden fiziksel Tango retest'i
kapalı kalacak.

2026-06-30 canlı acceptance devam kaydı: prosedürdeki safe-yaw P23/yaw1504
virtual RC gate'i tekrar koşuldu. İlk deneme uçuş davranışından değil, runner
timeout'undan FAIL oldu: `sitl_virtual_rc.py` 40 saniyelik hold için yine 40
saniye subprocess timeout ile çağrılıyordu. `tools/sitl_virtual_takeoff_check.py`
artık `--virtual-rc-timeout` verilmezse takeoff script süresini ve güvenlik
payını hesaplıyor; hedefli test `tests/test_sitl_virtual_takeoff_check.py`
altında kilitlendi. Tekrar koşu PASS:
`logs/sitl/20260630-064350-takeoff-diagnostics.jsonl`,
`logs/sitl/20260630-064350-takeoff-motor-udp.jsonl`,
`logs/sitl/20260630-064350-takeoff-virtual-rc.jsonl`.
Komut: `tools/sitl_virtual_takeoff_check.py --capture-motor-udp
--nudge-delay-seconds 18 --nudge-yaw 1504 --yaw-pid 23,0,0
--safe-yaw-authority --debug-mode AC_ERROR --hold-seconds 40`.
Sonuç: altitude gain `31.402 m`, max roll/pitch `0.000/0.000`, raw motor
spread `0.618`, raw axis `25/100/200/400us` eşikleri tetiklenmedi,
`debug_mode=49/AC_ERROR`, ARM+ANGLE `67` sample. Bu, safe-yaw profilinin
virtual RC kabul kapısını güncel kodla da geçtiğini gösterir; fiziksel RC ve
gerçek kamera/video target-found gate'leri hâlâ ayrı açık.

2026-06-30 gerçek video target-found acceptance kaydı: `tools/sitl_video_tracking_check.py`
eklendi. Bu wrapper external Gazebo checker'ı, virtual-pilot
`kenet_sitl_mixer.py --camera test-2.mp4` koşusunu, raw motor UDP capture'ı ve
mixer JSONL target-found doğrulamasını tek komutta topluyor. İlk gerçek video
gate'i bilinçli olarak `yaw_limit=0`, `forward_limit=0` ile koşuldu; amaç gerçek
OpenCV tracker/video yolunun `source=kenet` ve `target_found=True` ürettiğini,
ama pitch/yaw komutu vermeden güvenli kaldığını kanıtlamak. PASS logları:
`logs/sitl/20260630-video-target-found-neutral-diagnostics.jsonl`,
`logs/sitl/20260630-video-target-found-neutral-mixer.jsonl`,
`logs/sitl/20260630-video-target-found-neutral-motor-udp.jsonl`. Sonuç:
diagnostics altitude gain `23.850 m`, max roll/pitch `0.000/0.100`, raw motor
spread `0.000`; mixer `376` sample, `374` TRACKING, `265` target-found,
`265` source=kenet, frame shape `1280x720`, max pitch/yaw delta `0/0`,
tracker error yok. Bu gerçek kamera/video target-found neutral acceptance
gate'ini kapatır; nonzero komutlu video gate'i hâlâ Betaflight yaw/rate
feedback sınırı nedeniyle ayrı aşama.

2026-06-30 gerçek video nonzero command-response braket kaydı: ilk denemeler
Kenet state'i baştan TRACKING yaptığı için yaw komutunu takeoff/ramp sırasında
verdi ve FAIL oldu. `yaw_limit=4`, `2`, `1` koşuları sırasıyla
`20260630-video-target-found-yaw4-*`, `yaw2-*`, `yaw1-*` loglarında raw spread
`945` ve attitude FAIL üretti. P23 ayarı eklenmeden koşulan bu braketlerden
sonra `tools/sitl_video_tracking_check.py` external checker'a `--yaw-pid`
geçirebilir hale getirildi ve `kenet_sitl_mixer.py` virtual pilot için
`--virtual-kenet-delay-seconds` / `--virtual-kenet-pre-pwm` aldı. Böylece Kenet
state'i takeoff sonrası açılabiliyor. Ölçülen kabul koşusu:
`tools/sitl_video_tracking_check.py --camera test-2.mp4 --run-id
20260630-video-target-found-yaw2-p23-delay18 --virtual-hold-seconds 40
--yaw-limit 2 --forward-limit 0 --max-abs-delta 2 --yaw-pid 23,0,0
--kenet-delay-seconds 18`. PASS logları:
`logs/sitl/20260630-video-target-found-yaw2-p23-delay18-diagnostics.jsonl`,
`logs/sitl/20260630-video-target-found-yaw2-p23-delay18-mixer.jsonl`,
`logs/sitl/20260630-video-target-found-yaw2-p23-delay18-motor-udp.jsonl`.
Sonuç: altitude gain `31.401 m`, max roll/pitch `0.300/0.300`, raw spread
`0.000`, mixer `478` sample, `310` TRACKING, `203` target-found/source=kenet,
max pitch/yaw delta `0/2`. Bu küçük gerçek-video yaw command-response gate'ini
kapattı. Aynı gecikmeli/P23 profilde `yaw_limit=4` de PASS verdi:
`logs/sitl/20260630-video-target-found-yaw4-p23-delay18-diagnostics.jsonl`,
`logs/sitl/20260630-video-target-found-yaw4-p23-delay18-mixer.jsonl`,
`logs/sitl/20260630-video-target-found-yaw4-p23-delay18-motor-udp.jsonl`.
Sonuç: altitude gain `31.401 m`, max roll/pitch `1.300/5.400`, raw spread
`0.000`, mixer `476` sample, `306` TRACKING, `196` target-found/source=kenet,
max pitch/yaw delta `0/4`. Bu, video yaw command-response için asıl güvenlik
ayrımının komut büyüklüğünden önce takeoff sonrası Kenet state zamanlaması
olduğunu gösteriyor. Aynı gecikmeli/P23 profilde `yaw_limit=5` de PASS verdi:
`logs/sitl/20260630-video-target-found-yaw5-p23-delay18-diagnostics.jsonl`,
`logs/sitl/20260630-video-target-found-yaw5-p23-delay18-mixer.jsonl`,
`logs/sitl/20260630-video-target-found-yaw5-p23-delay18-motor-udp.jsonl`.
Sonuç: altitude gain `31.377 m`, max roll/pitch `3.300/4.300`, raw spread
`0.000`, mixer `478` sample, `307` TRACKING, `226` target-found/source=kenet,
max pitch/yaw delta `0/5`.

Aynı gecikmeli/P23 profilde `yaw_limit=6` FAIL verdi:
`logs/sitl/20260630-video-target-found-yaw6-p23-delay18-diagnostics.jsonl`,
`logs/sitl/20260630-video-target-found-yaw6-p23-delay18-mixer.jsonl`,
`logs/sitl/20260630-video-target-found-yaw6-p23-delay18-motor-udp.jsonl`.
Sonuç: altitude gain `7.158 m`, max roll/pitch `180.000/72.400`, MSP motor
spread `945`, mixer `481` sample, `309` TRACKING, `195`
target-found/source=kenet, max pitch/yaw delta `0/6`. Daha geniş kırılma
kanıtı olarak `yaw_limit=8` de FAIL:
`logs/sitl/20260630-video-target-found-yaw8-p23-delay18-diagnostics.jsonl`,
`logs/sitl/20260630-video-target-found-yaw8-p23-delay18-mixer.jsonl`,
`logs/sitl/20260630-video-target-found-yaw8-p23-delay18-motor-udp.jsonl`.
Sonuç: max roll `180.0`, max pitch `45.1`, diagnostics motor spread `945`;
mixer `311` TRACKING, `202` target-found/source=kenet ve max yaw delta `8`
üretti. Güncel video yaw braket sonucu: yaw5 PASS, yaw6 FAIL. Pitch-only ve
birleşik pitch+yaw command-response ayrıca ölçüldü; bunların sınırları aşağıda
ayrı kaydedildi.

Pitch/forward-only gerçek video braketinde `yaw_limit=0` ve gecikmeli/P23 profil
korundu. `forward_limit=2` FAIL:
`logs/sitl/20260630-video-target-found-pitch2-p23-delay18-diagnostics.jsonl`,
`logs/sitl/20260630-video-target-found-pitch2-p23-delay18-mixer.jsonl`,
`logs/sitl/20260630-video-target-found-pitch2-p23-delay18-motor-udp.jsonl`.
Sonuç: altitude gain `8.569 m`, max roll/pitch `180.000/77.700`, MSP motor
spread `945`, mixer `480` sample, `311` TRACKING, `217`
target-found/source=kenet, max pitch/yaw delta `2/0`.

Default pitch PID ile `forward_limit=1` de iki koşuda FAIL verdi:
`logs/sitl/20260630-video-target-found-pitch1-p23-delay18-diagnostics.jsonl`,
`logs/sitl/20260630-video-target-found-pitch1-p23-delay18-mixer.jsonl`,
`logs/sitl/20260630-video-target-found-pitch1-p23-delay18-motor-udp.jsonl`,
ve repeat
`logs/sitl/20260630-video-target-found-pitch1-p23-delay18-repeat1-diagnostics.jsonl`,
`logs/sitl/20260630-video-target-found-pitch1-p23-delay18-repeat1-mixer.jsonl`,
`logs/sitl/20260630-video-target-found-pitch1-p23-delay18-repeat1-motor-udp.jsonl`.
İlk pitch1: altitude gain `13.620 m`, max roll/pitch `180.000/44.000`, MSP
motor spread `945`, max pitch/yaw delta `1/0`. Repeat pitch1: altitude gain
`12.838 m`, max roll/pitch `180.000/58.900`, MSP motor spread `945`, max
pitch/yaw delta `1/0`.

Pitch PID `23/0/0` eklendikten sonra aynı gerçek video pitch-only kapısı
açıldı. `forward_limit=1`, `2`, `4`, `5` PASS:
`logs/sitl/20260630-video-target-found-pitch1-p23-delay18-pitchpid23-diagnostics.jsonl`,
`logs/sitl/20260630-video-target-found-pitch2-p23-delay18-pitchpid23-diagnostics.jsonl`,
`logs/sitl/20260630-video-target-found-pitch4-p23-delay18-pitchpid23-diagnostics.jsonl`,
`logs/sitl/20260630-video-target-found-pitch5-p23-delay18-pitchpid23-diagnostics.jsonl`.
Özet: pitch1 altitude gain `31.528 m`, max roll/pitch `0.000/0.000`;
pitch2 altitude gain `31.485 m`, max roll/pitch `15.800/16.400`; pitch4
altitude gain `31.535 m`, max roll/pitch `10.700/11.300`; pitch5 altitude gain
`31.538 m`, max roll/pitch `0.900/0.800`; hepsinde max motor spread `0.000`.
`forward_limit=6` aynı pitch PID profiliyle FAIL:
`logs/sitl/20260630-video-target-found-pitch6-p23-delay18-pitchpid23-diagnostics.jsonl`.
Sonuç: altitude gain `31.523 m`, max roll/pitch `78.800/9.300`, max motor
spread `0.000`, max pitch/yaw delta `6/0`. Güncel pitch braket sonucu: pitch5
PASS, pitch6 FAIL.

Birleşik pitch+yaw gerçek video command-response kapısı aynı gecikmeli/P23
profil ve pitch PID `23/0/0` ile ayrıca ölçüldü. `yaw_limit=2`,
`forward_limit=2` PASS:
`logs/sitl/20260630-video-target-found-yaw2-pitch2-p23-delay18-pitchpid23-diagnostics.jsonl`,
`logs/sitl/20260630-video-target-found-yaw2-pitch2-p23-delay18-pitchpid23-mixer.jsonl`,
`logs/sitl/20260630-video-target-found-yaw2-pitch2-p23-delay18-pitchpid23-motor-udp.jsonl`.
Sonuç: altitude gain `31.441 m`, max roll/pitch `3.100/2.600`, max motor
spread `0.000`, mixer `482` sample, `314` TRACKING, `175`
target-found/source=kenet, max pitch/yaw delta `2/2`. `yaw_limit=3`,
`forward_limit=3` de PASS:
`logs/sitl/20260630-video-target-found-yaw3-pitch3-p23-delay18-pitchpid23-diagnostics.jsonl`,
`logs/sitl/20260630-video-target-found-yaw3-pitch3-p23-delay18-pitchpid23-mixer.jsonl`,
`logs/sitl/20260630-video-target-found-yaw3-pitch3-p23-delay18-pitchpid23-motor-udp.jsonl`.
Sonuç: altitude gain `31.483 m`, max roll/pitch `3.200/3.300`, max motor
spread `0.000`, mixer `481` sample, `307` TRACKING, `193`
target-found/source=kenet, max pitch/yaw delta `3/3`. `yaw_limit=4`,
`forward_limit=4` aynı profil ile FAIL:
`logs/sitl/20260630-video-target-found-yaw4-pitch4-p23-delay18-pitchpid23-diagnostics.jsonl`,
`logs/sitl/20260630-video-target-found-yaw4-pitch4-p23-delay18-pitchpid23-mixer.jsonl`,
`logs/sitl/20260630-video-target-found-yaw4-pitch4-p23-delay18-pitchpid23-motor-udp.jsonl`.
Sonuç: altitude gain `8.169 m`, max roll/pitch `180.000/55.400`, MSP motor
spread `945`, mixer `484` sample, `311` TRACKING, `186`
target-found/source=kenet, max pitch/yaw delta `4/4`. Güncel birleşik video
braketi: yaw3+pitch3 PASS, yaw4+pitch4 FAIL. Bu, tek eksenlerde yaw5 ve
pitch5 PASS olsa bile birleşik komut penceresinin daha dar olduğunu gösterir.

2026-06-30 Claude denemeleri sonrası Codex kontrolü: son `flipfix-*` logları
ayrı satırlı tabloyla tekrar analiz edildi. Bunun için `tools/analyze_sitl_log.py`
artık `--per-path` modunu destekliyor; aynı komut gelecekteki hızlı denemelerde
her logu ayrı PASS/FAIL/WARN satırı olarak özetleyecek. Kontrol edilen
`flipfix-final*`, `recheck`, `velp01`, `veli`, `veli05`, `fcp5`, `conf-*`,
`d40`, `trial1`, `trial2` koşularının tamamı FAIL: büyük yaw provoke
denemelerinde `yaw_delta=+150` veya negatif denemede `-150`, MSP motor spread
`945` ve çoğu koşuda max roll `180 deg` görülüyor. Bu sonuç safe-yaw
P23/yaw1504 kabul kapısını geri almıyor; yalnızca `yaw=1650/1350` sınıfı daha
geniş yaw authority penceresinin hâlâ çözülmediğini gösteriyor. Kaldığımız
yer: testleri fiziksel RC'den izole tut, kabul akışında safe-yaw + gecikmeli
Kenet state + küçük command-response limitleriyle ilerle, daha büyük yaw
authority istenirse bunu ayrı bir braket ve ayrı kabul eşiği olarak aç.

2026-06-30 command-response gate sertleştirmesi: `tools/sitl_video_tracking_check.py`
ve `tools/sitl_synthetic_tracking_check.py` artık eksen bazında
`--min-abs-yaw-delta`, `--min-abs-pitch-delta`, `--max-abs-yaw-delta` ve
`--max-abs-pitch-delta` kabul eşiklerini destekliyor. Bu, yaw-only, pitch-only
ve combined braketlerde PASS sonucunu "uçuş stabil kaldı ve beklenen eksen
komutu gerçekten üretildi" anlamına getiriyor; nötr veya yanlış eksenle
yanlışlıkla PASS alma riski azaltıldı.

2026-06-30 real-video target-loss hazırlığı: `tools/kenet_sitl_mixer.py`
gerçek camera/video yolu açıkken `--target-loss-after-seconds` ile target-loss
geçişini deterministik tetikleyebiliyor. `tools/sitl_video_tracking_check.py`
bu bayrağı geçiriyor ve `source=pilot-target-lost` ile `AI-ARMED` sample
sayılarını acceptance kriteri olarak sayıyor. Bu, sentetik target-loss gate'ini
gerçek video wrapper seviyesine taşıdı; canlı Gazebo real-video target-loss
kanıtı aşağıdaki final PASS ile alındı.

2026-06-30 real-video target-loss canlı final PASS:
`tools/sitl_video_tracking_check.py --camera test-2.mp4 --run-id
20260630-video-target-loss-p23-delay18-final --virtual-hold-seconds 40
--diagnostic-samples 80 --yaw-limit 0 --forward-limit 0 --max-abs-delta 0
--yaw-pid 23,0,0 --pitch-pid 23,0,0 --kenet-delay-seconds 18
--target-loss-after-seconds 24 --min-target-lost-samples 1
--min-ai-armed-samples 1`. PASS logları:
`logs/sitl/20260630-video-target-loss-p23-delay18-final-diagnostics.jsonl`,
`logs/sitl/20260630-video-target-loss-p23-delay18-final-mixer.jsonl`,
`logs/sitl/20260630-video-target-loss-p23-delay18-final-motor-udp.jsonl`.
Sonuç: diagnostics `80` sample, ARM+ANGLE `70`, altitude gain `30.175 m`,
max roll/pitch `0/0`, motor/raw spread `0`; mixer `511` sample,
`target_found/source=kenet 31`, `pilot-target-lost 40`, `AI-ARMED 265`,
max pitch/yaw delta `0/0`.

2026-06-30 quickstart kaydı: `docs/sitl-quickstart.md` eklendi. Bu dosya
fiziksel RC izole durumdayken env check, no-hardware unit gate'ler, safe-yaw
virtual takeoff, real-video neutral, axis-specific command-response,
synthetic/real-video target-loss ve `analyze_sitl_log.py --per-path` triage
sırasını tek kısa runbook içinde topluyor. README ve acceptance procedure bu
quickstart'a link veriyor.

2026-06-30 plan sınıflandırması: sentetik target-found TRACKING davranışı
centered PASS ve offsetli FAIL kanıtlarıyla "test edildi" olarak kapatıldı.
Offsetli sentetik koşular artık çözülmesi beklenen ana acceptance kapısı değil;
Betaflight/Gazebo yaw/rate sınırını gösteren negatif kanıt olarak tutuluyor.
Küçük komut kabul penceresi gerçek video + gecikmeli Kenet state + safe-yaw/PID
profilleriyle takip edilecek.

2026-06-30 P13 sınıflandırması: no-hardware UDP mixer/test matrisi virtual,
synthetic ve gerçek video hatlarda tamamlandı. Fiziksel RC bu matrisin içinde
gizli açık olarak tutulmuyor; aynı acceptance kriterleriyle ayrı external gate
olarak `/dev/input/js*` görünmesini bekliyor.

2026-06-30 readiness raporu: `tools/sitl_readiness_report.py` eklendi. Bu araç
quickstart dokümanını, safe-yaw virtual takeoff logunu ve real-video target-loss
final loglarını okuyup no-hardware readiness verdict'i üretir; fiziksel RC ve
başka makine env raporunu dış gate olarak listeler. Güncel çıktı:
no-hardware `PASS`, physical RC device `READY` (`/dev/input/js0`), other-machine
env report `WAITING`.

2026-06-30 fiziksel RC bağlantı notu: TBS joystick
`/dev/input/by-id/usb-Team-BlackSheep_TBS_Joystick_00000000001B-joystick`
üzerinden `/dev/input/js0` olarak göründü. `sitl_rc_bridge.py --dry-run
--events --show-init` init değerlerini okudu:
`a0=0, a1=0, a2=-32767, a3=0, a4=-32767, a5=-32767, a6=-32767`, mapped RC
`1500,1500,1000,1500,1000,1000,1000,1500`. Ancak 20s preflight ve 30s
`--changes` penceresinde hareket event'i yakalanmadı; bu yüzden fiziksel canlı
external gate henüz başlatılmadı. Sıradaki adım kullanıcı switch/stick hareketi
yaparken `tools/sitl_physical_rc_preflight.py --device /dev/input/js0
--duration 20 --verbose` koşup CH5/CH6/CH7 coverage almak; CH7 gelmezse
`--force-mode-pwm 1500` ile ANGLE profili forced olarak kaydedilecek.

2026-06-30 fiziksel RC tek launcher: kullanıcı doğrudan simülasyonda kumanda ile
kontrol testi istediği için repo köküne `launch-physical-rc-sim.sh` eklendi.
Komut tek dosya olarak Gazebo GUI'yi, geçici cwd ile Betaflight SITL'i, mode
range konfigürasyonunu, safe-yaw rate profilini, yaw/pitch PID `23/0/0`
profilini, `/dev/input/js0` fiziksel RC bridge'i ve kısa diagnostics/motor UDP
capture'larını aynı `logs/sitl/<RUN>-*` prefix'iyle başlatıyor. Launcher artık
RC göndermeden önce throttle düşük ve ARM kapalı bekliyor; MSP-ready yarışına
karşı Betaflight start kısmında retry var. Ayrıca `sitl_dashboard.py` web
arayüzünü `http://127.0.0.1:8080` üzerinde başlatıyor; arayüz RC kanalları,
ARM/Kenet/mode state, MSP arming flags, motor/attitude ve süreç butonlarını
gösteriyor. Dashboard açıkken diagnostics MSP bilgisini dashboard snapshot'ından
okuyor; böylece Betaflight MSP portunda dashboard ile direkt diagnostics okuyucusu
çakışmıyor. Launcher tarafından başlatılan dış süreçler dashboard'da `external
active` olarak işaretlenir. Kısa smoke koşuları:
`codex-launch-gui-smoke-212154` GUI yolunda READY'ye kadar geçti, config
logunda ARM, MSP Override, ANGLE/HORIZON mode range'leri, yaw safe-rate ve
yaw/pitch PID ayarları MSP üzerinden uygulandı; `codex-launch-final-smoke-212521`
headless/automation yolunda RC bridge'in UDP `9004` gönderdiğini logladı.

2026-06-30 fiziksel RC canlı kontrol bulgusu: kullanıcı koşusu
`20260630-225622-physical-rc-launch` RC kanal plumbing'inin doğru olduğunu
gösterdi (`pilot_channels` ile FC RC kanalları eşleşti, dashboard external RC
aktifti), ancak küçük pitch/ileri-geri hareketlerinden sonra ANGLE modda
osilasyon ve ters dönme oluştu. Logda pitch stick yaklaşık `1397-1407` iken
attitude hızla `pitch=80.6`, `roll=178.7` seviyesine gitti; son durumda araç
Gazebo'da ters (`roll=-180`) kaldı ve arming `ANGLE` nedeniyle bloklandı. Bu
koşuda launcher yalnız safe-yaw rate profilini uyguluyordu; roll/pitch
`rc_rate=7`, `rate=67`, `rate_limit=1998` default/agresif kaldı. Bu nedenle
`sitl_rate_config.py` roll alanlarını da CLI/update path'ine aldı ve fiziksel RC
launcher artık roll/pitch/yaw üç eksende güvenli manuel authority
(`rc_rate=5`, `rate=30`, `rate_limit=120`) uyguluyor. Bu kalıcı tune değil,
manuel sim testlerinin osilasyonsuz yapılabilmesi için containment profilidir.

2026-06-30 fiziksel RC ikinci canlı kontrol ve virtual repro: kullanıcı koşusu
`20260630-232217-physical-rc-launch` yeni safe manual authority profilini doğru
uyguladı (`roll/pitch/yaw rc_rate=5 rate=30 rate_limit=120`), fakat yine FAIL
verdi. RC plumbing yine temiz: roll/yaw nötr, Kenet `IDLE`, AUX2 düşük, pitch
sadece `1495..1522` aralığında oynadı. Motor UDP artık analiz aracında doğru
okunuyor; bu koşuda ilk büyük motor spread fiziksel RC'den değil saf pitch
ekseni düzeltmesinden geldi: `spread>200` anında bias `pitch_rear_minus_front`
yaklaşık `+200 us`, roll/yaw bias `0`; `spread>900` anında motorlar
`1055/1965/1055/1965` sınıfında, yine saf pitch saturasyonu. Roll saturasyonu
araç yere/ters duruma girdikten sonra ortaya çıktı. Aynı throttle penceresi
virtual RC ile ayrıştırıldı: `20260630-virtual-physical-repro-neutral1475`
PASS (`max motor spread 0`, max roll/pitch `0/0`, altitude gain `1.526 m`),
ama `20260630-virtual-physical-repro-pitch1522` FAIL (`max spread 945`, ilk
büyük spread `pitch=-400.756`, max roll/pitch `180/44.8`). Sonuç: fiziksel RC
bu düşüş için ana kök neden değil; açık sorun küçük pitch command-response /
ANGLE-level loop penceresi.

2026-07-01 virtual pitch bracket ve Betaflight doküman kontrolü: Kullanıcının
işaret ettiği resmi Betaflight Gazebo SITL dokümanı Ubuntu 24.04/Gazebo
Harmonic mimarisini `UDP 9002 PWM out`, `9003 FDM in`, `9004 RC in` olarak
tarif ediyor ve eski SITL notlarıyla aynı temel uyarıyı taşıyor: Gazebo
`max_step_size` `0.0025` üstüne çıkmamalı, ESC/Motor protokolü `PWM` olmalı,
"Motor PWM speed separated from PID speed" kapalı olmalı ve PID loop frekansı
yüksek tutulmalı. Bizim runner/launcher port mimarisi doğruydu; ancak iki
temel test ortamı hatası bulundu. Birincisi, `sitl_virtual_takeoff_check.py`
FAIL sonrası geçici world/model kullanan `run_gazebo_betaflight.sh` wrapper'ını
kapatıyor ama child `gz sim` bazen kalıp UDP `9002` bind'ini tutuyordu. Bu,
`1515/1520/1522` ilk bracket koşularını geçersiz `ARM+ANGLE=0` sonucuna
çevirdi; `gazebo.log` içinde `failed to bind with 127.0.0.1:9002` görüldü.
Runner süreçleri artık process group olarak başlatıp kapatıyor. İkinci ve asıl
uçuş bulgusu: `max_step_size=0.0025` resmi üst sınırın içinde olmasına rağmen
bizim Iris/Betaflight loop için hâlâ kaba kaldı. Bracket sonuçları:
`20260630-235042-bracket-pitch1505` PASS (`raw spread 0.741`, max attitude
`0/0.1`), `20260630-235208-bracket-pitch1510` FAIL (ilk büyük split saf pitch
`~404 us`, sonra raw spread `945`), clean tekrarlarla `1515/1520/1522` de
geçerli FAIL. Aynı profil `--max-step-size 0.001` ile düzeldi:
`20260701-000845-bracket-step001-pitch1510` PASS (`raw spread 3.126`, max
attitude `0/0.3`, altitude gain `1.472 m`) ve
`20260701-001028-bracket-step001-pitch1522` PASS (`raw spread 12.972`, max
attitude `0/1.3`, altitude gain `1.404 m`). Sonuç: hazır PID preset aramadan
önce acceptance ve fiziksel RC launcher default'u `0.001` physics step olmalı.
Runner'a ayrıca `--betaflight-config-file` eklendi; `/home/gz/betaflight/`
`sitl_config.txt` import testi `20260701-000647-bracket-config-pitch1510` aynı
`0.0025` step ile hâlâ FAIL verdi, yani tek başına eksik CLI config kök neden
değil.
