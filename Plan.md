# Kenet — Düzeltme Planı

İnceleme bulgularına dayalı yapılacaklar listesi. Sıralama önceliğe göre:
önce sistemi çalışır hale getir, sonra uçuş güvenliği, sonra iyileştirmeler.

Durum: `[ ]` yapılacak · `[~]` devam ediyor · `[x]` tamam

---

## P0 — Sistem hiç çalışmıyor (önce bu)

### 1. OpenCV tracker'ları mevcut değil
**Sorun:** Ortamda `opencv-python` 4.13.0 kurulu, `opencv-contrib-python` değil.
`cv2.TrackerCSRT` / `cv2.TrackerKCF` `cv2`'de (legacy'de bile) **yok**. TRACKING
durumuna geçince [tracker.py:42](kenet/tracker.py#L42) `AttributeError` ile patlar.
`opencv-python` ve `opencv-contrib-python` aynı anda kurulamaz.

- [x] Mevcut global/başka-proje ortamına dokunmadan izole `fpv_env` oluştur
- [x] `opencv-contrib-python>=4.8.0` kur (requirements.txt zaten bunu söylüyor)
- [x] Doğrula: `python -c "import cv2; cv2.TrackerCSRT.create(); cv2.TrackerKCF.create()"`
- [x] `tracker.py`'ye instantiate başarısız olursa anlaşılır hata mesajı ekle
  (ör. "opencv-contrib-python gerekli")

---

## P1 — Uçuş güvenliği

### 2. Throttle override maskesi yanlış (pilot gaz kontrolünü kaybediyor)
**Sorun:** README `msp_override_channels = 15` diyor (roll+pitch+throttle+yaw).
Ama Kenet yalnızca yaw ve pitch sürüyor; roll ve throttle 1500'de sabit kalıyor
([controller.py:144-159](kenet/controller.py#L144-L159)). 15 ile pilot, takip
sırasında throttle'ı (irtifa) ve roll'u kaybeder, drone sabit orta-gazda kalır.

Bitmask (repo konvansiyonu: roll=0, pitch=1, throttle=2, yaw=3):
- `15` = hepsi · `10` = pitch+yaw (Kenet'in sürdüğü) · `12` = throttle+yaw

- [x] [README.md:160](README.md#L160) `msp_override_channels = 15` → `10` yap
- [x] Neden 10 olduğunu README'de açıkla (pilot throttle+roll'u korur)
- [x] (Karar) Mevcut Kenet kapsamı pitch/yaw-only. Throttle/roll Kenet
  tarafından sürülmeyecek; `msp_override_channels = 10` korunacak. Throttle
  veya roll authority istenirse ayrı RFC ve yeni bench acceptance gate açılacak.
- [ ] Propeller sökülü gerçek FC/Betaflight bench test ile `msp_override_channels=10`
  doğrula

---

## P2 — Doğruluk / sağlamlık

### 3. Attitude telemetrisi her zaman 0
**Sorun:** `TelemetryPacket` roll/pitch/yaw taşıyor ([gcs.py:35-39](kenet/gcs.py#L35-L39)),
`MSPConnection.get_attitude()` var ([msp.py:92](kenet/msp.py#L92)), ama
`_send_telemetry()` bunları doldurmuyor ([pipeline.py:339](kenet/pipeline.py#L339)).
quick_gcs hep `att=[0,0,0]` gösteriyor.

- [x] `_send_telemetry`'de `get_attitude()` çağır, paketi doldur
- [x] **Nüans:** her kontrol döngüsünde (30 Hz) değil, GCS gönderim hızında (10 Hz)
  çağır — yoksa bloklayan MSP isteği seri hattı tıkar. Rate-limit kararını
  `gcs.send_telemetry` içinden pipeline'a taşımak gerekebilir.

### 4. MSP parser sağlam değil
**Sorun:** [msp.py:73](kenet/msp.py#L73) her request başında `reset_input_buffer()`
geç gelen yanıtı atabilir; checksum okunuyor ama **doğrulanmıyor**
([msp.py:79-82](kenet/msp.py#L79-L82)); `$M!` hata yanıtı ele alınmıyor.

- [x] Header arayan + checksum doğrulayan küçük MSP parser yaz
- [x] `$M!` (hata) yanıtını ele al
- [x] AUX polling'i 30 Hz kontrol döngüsünden ayır, daha düşük rate'e al

---

## P3 — Hijyen / dokümantasyon

### 5. Test videoları
**Sorun:** `test-1.mp4`, `test-2.mp4` untracked; `.gitignore` sadece `test.mp4`
içeriyor ([.gitignore:5](.gitignore#L5)). Yanlışlıkla commit riski (24 MB).

- [x] `.gitignore`'a `test-*.mp4` veya `*.mp4` ekle
- [x] (Karar) Videolar lokal test verisi olarak kalacak; repo fixture'ı olarak
  commit edilmeyecek

### 6. README ↔ kod tutarsızlığı
**Sorun:** README iki ayrı AUX switch anlatıyor ([README.md:118](README.md#L118)),
kod tek 3-pozisyonlu `aux_ch` kullanıyor ([pipeline.py:76](kenet/pipeline.py#L76)).
CLI referansı eski: README `--arm-ch/--track-ch/--show-preview/--no-gcs` diyor,
gerçek CLI `--aux-ch/--headless` ([kenet.py:32](kenet.py#L32)).

- [x] Architecture / Two-Stage AUX bölümlerini tek 3-pozisyonlu switch'e göre güncelle
- [x] CLI referansını gerçek argümanlarla senkronla

---

## Sıra
1 (OpenCV) → 2 (throttle maskesi) → 3 (attitude) → 4 (MSP parser) → 5 (.gitignore) → 6 (README)

---

## Codex değerlendirmesi — Claude SITL notu sonrası kritik ekler

Claude'un SITL/Gazebo notunda haklı bulduğum kritik nokta şu: artık iki ayrı
kontrol yolu var ve bunların aynı güvenlik davranışını verdiğini garanti
etmeden Gazebo/MSP/gerçek donanım tarafına fazla güvenmemeliyiz.

### P4 — SITL harness üretim davranışından drift etmesin

**Sorun:** `tools/kenet_sitl_mixer.py` gerçek `FlightController` ve
`ObjectTracker` sınıflarını kullanıyor; bu iyi. Ancak AUX→state geçişi, hedef
kaybı ve tracker re-init davranışı `kenet/pipeline.py` içindeki üretim state
machine'inden ayrı kopyalanmış durumda. Özellikle hedef kaybında üretim pipeline
`TRACKING -> AI-ARMED` yaparken mixer merkez kutuyla otomatik yeniden init
edebiliyor. Bu, SITL'in gerçek sistemi doğrulama değerini düşürür.

- [x] AUX threshold/state kararlarını tek bir paylaşılan yardımcıya taşı
  veya ana pipeline'ı pluggable output sink ile SITL UDP'ye bağla.
- [x] Hedef kaybı davranışını üretim ve SITL yolunda aynı hale getir.
- [x] Tracker re-init için pilot niyeti/switch döngüsü gerekip gerekmediğini
  tek tasarım kararı olarak yaz.
- [x] `>=` / `>` boundary davranışını testle sabitle.

### P5 — AUX kanal default'u tek kaynak olsun

**Sorun:** Ana pipeline default'u hâlâ AUX4 / index `7`; SITL planı ve Tango 2
mapping'i ise Kenet state için CH6 / AUX2 / index `5` kullanıyor. Bu karışıklık
masaüstünde sadece kafa karıştırır, gerçek MSP/override yolunda ise yanlış
kanaldan state okunması anlamına gelebilir.

- [x] Kenet state default kanalı için tek karar ver: `CH6 / AUX2 /
  index 5`.
- [x] `kenet.py`, `PipelineConfig`, SITL mixer, dashboard, README ve planları
  aynı default'a getir.
- [x] Eski AUX4 kullanan setup varsa migration notu ekle.
- [x] ARM `CH5`, Kenet state `CH6`, autopilot/ANGLE `CH7` ayrımını bench
  checklist'e ekle. `tools/sitl_physical_rc_preflight.py` no-send preflight
  eklendi; CH7 fiziksel switch yoksa `--force-mode-pwm 1500` forced ANGLE
  profili olarak kaydediliyor.

### P6 — SITL safety glue testleri eklensin

**Sorun:** Üretim MSP/controller testleri var; fakat yeni `tools/` SITL yolu,
özellikle mixer/failsafe/RC packet tarafı, henüz testlerle kilitlenmemiş.

- [x] `test_mix_passthrough_when_not_tracking`
- [x] `test_mix_overrides_only_pitch_yaw`
- [x] `test_mix_falls_back_on_target_lost`
- [x] `test_mix_falls_back_on_tracker_unavailable`
- [x] `test_state_from_aux_thresholds`
- [x] `test_sitl_rc_bridge` ile `axis_to_rc`, `axis_to_three_pos_rc`,
  `clamp_rc`, `make_channels`, `pack_rc_packet`
- [x] `pytest.ini` veya `pyproject.toml` içinde `testpaths=tests`

### P7 — Tek RC/mapping gözlem aracı ve tek mapping kaynağı

**Sorun:** `rc_monitor.py`, `state_monitor.py` ve `sitl_dashboard.py` aynı alanı
kısmen tekrar ediyor. Bir monitor kendi joystick/mapping kopyasını taşırsa,
gösterdiği değer ile bridge'in gönderdiği değer ayrışabilir.

- [x] `sitl_dashboard.py` ana gözlem aracı olarak kabul edilsin.
- [x] `state_monitor.py` ve `rc_monitor.py` ya ince terminal/text shim'e
  indirilsin ya da dashboard'a yönlendirsin. `state_monitor.py` varsayılan
  olarak dashboard'a yönlenen compatibility shim oldu (`--legacy` eski UI);
  `rc_monitor.py` dar joystick/raw RC monitor olarak kaldı.
- [x] Switch threshold, AETR kanal sırası ve Kenet state kuralı tek küçük
  modülde toplansın. Production kanal index ve AETR->MSP dönüşümü
  `kenet/rc_channels.py` içinde; `tools/sitl_rc_channels.py` etiket,
  throttle neutral ve forced mode metadata wrapper'ı olarak bunu kullanıyor.
  Analyzer, diagnostics, dashboard, RC bridge ve monitor etiketleri buradan
  besleniyor; `state_monitor.py` de RC dönüşümünü `sitl_rc_bridge.make_channels`
  üzerinden yapıyor.

### P8 — Orta öncelikli ama güvenlik değeri olan sağlamlaştırmalar

- [x] Mixer `tracker._initialized` okumak yerine `tracker.is_initialized`
  public API'sini kullansın.
- [x] Controller frame merkezi için `_cx/_cy` yazmak yerine
  `controller.set_frame_center(width, height)` eklensin.
- [x] Mixer kapanırken throttle-low/centered güvenli son RC frame'i göndersin.
- [x] RC gönderim aralığı 2x nominal periyodu aşarsa watchdog warning yazsın.
- [x] Dashboard process-control endpoint'leri default olarak loopback ile
  sınırlı kalsın; network'e açmak `--allow-remote-control` ile açık opt-in
  olsun.
- [x] Dashboard Kenet sender başlatmadan önce aktif `sitl_rc_bridge.py`,
  `sitl_virtual_rc.py` veya `kenet_sitl_mixer.py --send` süreçlerini kontrol
  etsin; duplicate RC sender varsa başlatmayı reddetsin.
- [x] `tools/sitl_virtual_takeoff_check.py` Betaflight'i varsayılan olarak temp
  çalışma dizininde başlatsın; eski repo-root `eeprom.bin` davranışı sadece
  `--betaflight-cwd repo` ile opt-in kalsın.
- [x] `tools/gazebo_sitl_motor_smoke.py` Betaflight'i varsayılan olarak temp
  çalışma dizininde başlatsın; eski repo-root `eeprom.bin` davranışı
  `--betaflight-cwd repo` ile opt-in kalsın.
- [x] Dashboard process launcher Betaflight'i varsayılan olarak temp çalışma
  dizininde başlatsın ve snapshot/API içinde `working_dir` + `eeprom_path`
  göstersin; eski repo-root davranışı `--betaflight-cwd repo` ile opt-in.
- [x] Smoke test PASS kararı motor PWM büyüklüğünü ana sinyal kabul etsin;
  rotor hareketi tamamlayıcı kontrol olsun. Katı rotor kontrolü
  `--require-rotor-motion` ile opt-in.
- [x] Gazebo launcher `--headless` modunda gerekirse `--headless-rendering`
  desteği versin. `tools/run_gazebo_betaflight.sh --headless` varsayılan
  olarak `--headless-rendering` ekliyor; `--no-headless-rendering` legacy
  debug için opt-out. `tests/test_run_gazebo_betaflight.py` dry-run komutunu
  pinliyor.
- [x] README ve `CLAUDE.md`, `fpv_env/bin/python` / `fpv-test` kullanımını aynı
  dille anlatsın; yanlış venv tekrar footgun olmasın. README örnekleri aktif
  `fpv_env` varsayar; taze shell için `fpv_env/bin/python ...` notu eklendi.

### P9 — Loglama/analiz sistemi yanıltıcı sonuç üretmesin

Claude'un yeni logging yorumunda haklı bulduğum kritik nokta: JSONL log sistemi
faydalı ve doğru yönde, fakat analiz aracı veri yokluğunu güvenli durum gibi
yorumlamamalı. Özellikle takla analizinde MSP/attitude datası yoksa "35 derece
altında kaldı" demek yanlış güven verir.

- [x] `tools/analyze_sitl_log.py`, dashboard/MSP attitude sample yoksa
  "attitude verisi yok" demeli; eşik altında kaldı sonucu üretmemeli.
- [x] Motor sample yoksa motor spread için de "veri yok" demeli.
- [x] Log default oranı companion board dostu hale getirilsin: mixer flight log
  default'u 30 Hz yerine daha düşük, örn. 10 Hz.
- [x] JSONL writer flush stratejisi gözden geçirilsin: her satır flush yerine
  N kayıt veya yaklaşık 1 saniyelik buffered flush opsiyonu.
- [x] Uzun testlerde log dosyası büyümesi için size/rotation cap eklensin.
- [x] Analyzer içindeki `CHANNEL_LABELS`, AETR sırası ve throttle neutral
  hardcode'u ortak RC/mapping modülünden gelsin.
- [x] Analyzer, mixer üretici alan adları eksik/değişmişse sessiz geçmek yerine
  schema warning üretsin.
- [x] Mixer ve dashboard JSONL record şeması kısa dokümante edilsin:
  `docs/sitl-jsonl-schema.md`.
- [x] Analyzer sadece ilk 8 kanal yerine gerektiğinde tüm kanal setini
  raporlayabilecek şekilde genişletilsin.

### P10 — Gazebo takla kök nedenini izole et

Son SITL/Gazebo logunda takla anında Kenet `IDLE` durumundaydı; yani hedef takip
PID'i veya Kenet pitch/yaw override'ı aktif değildi. Roll/pitch pilotta merkez,
throttle yaklaşık `1375`, CH7/AUX3 `LOW` ve Betaflight flight flags sadece
`ARMED` göründü. Bu yüzden ilk ayrım Kenet değil, Betaflight mode /
motor-order-direction / IMU-frame tarafında yapılacak.

- [x] Fiziksel kumandayı Gazebo debug denkleminden geçici olarak çıkarmak için
  `tools/sitl_virtual_rc.py` eklendi; CH3 throttle rampası, CH5 ARM ve
  CH7/AUX3 flight mode sanal olarak üretilebilir.
- [x] `tools/sitl_diagnostics.py --rc-source virtual` eklendi; joystick yokken
  beklenen virtual RC frame'i ile Betaflight `MSP_RC` çıktısı karşılaştırılabilir.
- [x] `sitl_rc_bridge.py`, `kenet_sitl_mixer.py` ve `sitl_dashboard.py` için
  `--force-mode-pwm` eklendi; CH7/AUX3 Tango switch beklemeden sabitlenebilir.
- [x] Forced mode PWM davranışı testlerle sabitlendi.
- [x] `tools/sitl_configure_modes.py` eklendi; Configurator açmadan ARM/ANGLE
  mode range'leri SITL oturumuna uygulanabiliyor. Varsayılan range seti artık
  HORIZON AUX3 high satırını da yazar.
- [x] `tools/sitl_mode_status_check.py` eklendi; sanal RC case'leri gönderip
  `MSP_STATUS_EX` üzerinden ARM/MSP Override/ANGLE/HORIZON aktif modlarını ve
  arming disable flag'lerini Configurator açmadan raporlar.
- [x] `tools/sitl_mixer_matrix_check.py` eklendi; P6.1-P6.8 receiver/mixer
  matrisini donanımsız çalıştırıp IDLE/AI-ARMED/TRACKING, target-lost
  passthrough, target-found pitch/yaw override ve roll/throttle pilotta kalma
  sözleşmelerini tek komutta doğrular.
- [x] İlk yeni retest fiziksel kumanda ile değil, virtual RC ile yapıldı:
  `--script takeoff --throttle 1550 --mode-pwm 1500`.
- [x] ANGLE `AUX3 1300-1700` canlı SITL oturumuna MSP üzerinden uygulandı.
- [x] Virtual RC + Gazebo + `--fix-iris-motor-map` koşusunda takla tekrarlandı:
  `logs/sitl/20260629-212308-diagnostics.jsonl`, max roll `-180`, max motor
  spread `945 us`.
- [x] Tek komutluk regression runner eklendi:
  `tools/sitl_virtual_takeoff_check.py`.
- [x] Runner baseline logu gyro patch öncesi taklayı tekrar üretti:
  `logs/sitl/20260629-221238-takeoff-diagnostics.jsonl`.
- [x] Dış Gazebo plugin kök nedeni doğrulandı:
  `../aeroloop_gazebo/plugins/BetaflightPlugin.cc` yaw gyro `z` işareti
  Betaflight SITL'in kendi `z` negation'ı ile pozitif feedback yaratıyordu.
- [x] Plugin patch + rebuild sonrası aynı runner PASS:
  `logs/sitl/20260629-221545-takeoff-diagnostics.jsonl`, altitude gain
  `6.624 m`, max roll/pitch `0`, max motor spread `0`.
- [x] Aynı runner external RC gözlem moduna genişletildi:
  `tools/sitl_virtual_takeoff_check.py --rc-driver external`.
- [x] `tools/kenet_sitl_mixer.py --pilot-source virtual` eklendi; fiziksel
  joystick olmadan Kenet mixer yolundan scripted takeoff RC gönderilebiliyor.
- [x] External RC runner + Kenet mixer sanal pilot koşusu PASS:
  `logs/sitl/20260629-223523-external-kenet-takeoff-diagnostics.jsonl`.
  Sonuç: altitude gain `23.840 m`, max roll/pitch `0.3/0.1`, ARM+ANGLE `63`
  örnek, high throttle `53` örnek.
- [x] Aynı koşunun mixer logu `TRACKING` + `pilot-target-lost` passthrough'u
  doğruladı:
  `logs/sitl/20260629-223523-kenet-mixer-virtual-external-takeoff.jsonl`;
  final-pilot delta tüm ilk 8 kanalda `0`.
- [x] Kenet mixer target-found yolunu fiziksel kamera/video olmadan ölçmek için
  `--synthetic-target` ve `--synthetic-target-delay-seconds` eklendi.
- [x] Aynı yol için tek komutluk no-joystick/no-camera runner eklendi:
  `tools/sitl_synthetic_tracking_check.py`. Centered target-found profil PASS:
  `logs/sitl/20260629-continued-synthetic-centered-runner-delay8-diagnostics.jsonl`
  ve `logs/sitl/20260629-continued-synthetic-centered-runner-delay8-mixer.jsonl`.
  Sonuç: `120` diagnostic sample, `1244` mixer sample, ARM+ANGLE `73`,
  altitude gain `23.838 m`, max roll/pitch `0.4/0.3`, motor spread `0`,
  mixer `TRACKING/source=kenet/target_found=True`, final-pilot delta `0`.
- [~] Offsetli sentetik target-found acceptance koşuları PASS değil: hedef-yok
  passthrough güvenli, ancak küçük pitch/yaw offset'leri takeoff sonrası
  Gazebo/Betaflight closed-loop'unda taklayı geri getiriyor.
  - combined pitch+yaw: `logs/sitl/20260629-224953-external-synthetic-tracking-delayed-diagnostics.jsonl`
  - yaw only +26: `logs/sitl/20260629-225200-external-synthetic-yaw-only-diagnostics.jsonl`
  - yaw only -26: `logs/sitl/20260629-225340-external-synthetic-yaw-negative-diagnostics.jsonl`
  - pitch only +20: `logs/sitl/20260629-225520-external-synthetic-pitch-only-diagnostics.jsonl`
- [x] Daha dar tek eksen izolasyonu eklendi:
  `tools/sitl_synthetic_tracking_check.py` micro profilleri P-only çalışabiliyor;
  `tools/sitl_virtual_rc.py` ve `tools/sitl_virtual_takeoff_check.py` gecikmeli
  direct RC nudge destekliyor.
- [~] Micro/direct izolasyon sonucu Kenet görüntü/mixer katmanını güçlü biçimde
  dışarı itti:
  - synthetic yaw +12 P-only: `logs/sitl/20260629-continued-yaw-micro-positive-ponly-diagnostics.jsonl`
    -> FAIL, max roll/pitch `180.0/28.5`, motor spread `945`, max yaw delta `+12`.
  - synthetic yaw +4 P-only: `logs/sitl/20260629-continued-yaw-micro-positive-limit4-diagnostics.jsonl`
    -> FAIL, max roll/pitch `180.0/46.2`, motor spread `945`, max yaw delta `+4`.
  - synthetic pitch +10 P-only: `logs/sitl/20260629-continued-pitch-micro-positive-ponly-diagnostics.jsonl`
    -> FAIL, max roll/pitch `180.0/68.7`, motor spread `945`, max pitch delta `+10`.
  - direct virtual RC yaw 1504, Kenet yok:
    `logs/sitl/20260629-continued-direct-virtual-yaw1504-clean-diagnostics.jsonl`
    -> FAIL, max roll/pitch `180.0/76.8`, motor spread `945`, FC yaw delta `+4`.
  Yorum: sıradaki kök neden adayı Kenet değil; Betaflight/Gazebo yaw/pitch RC
  offset cevabı ve closed-loop motor output zinciri.
- [x] Direct motor axis probe BF SITL remap'iyle düzeltildi:
  `tools/gazebo_motor_moment_probe.py --pattern-set axis` artık default
  `--motor-map bf-sitl` (`3,0,1,2`) kullanıyor. Eski raw packet index yorumu
  yanıltıcıydı; doğru remap ile roll/pitch/yaw pair pattern'leri temiz ayrışıyor.
  Kanıt: `logs/sitl/20260629-continued-axis-moment-default-bfsitl-after-identity.jsonl`.
- [x] Dış Gazebo modelindeki BetaflightPlugin rotor listesi source seviyesinde
  runtime identity overlay ile uyumlu hale getirildi:
  `../aeroloop_gazebo/models/betaloop_iris_with_standoffs/model.sdf`.
- [x] FDM/IMU sign kabul kapısı eklendi: `tools/gazebo_fdm_probe.py`.
  `bf_cw_pair_high` ve `bf_ccw_pair_high` koşularında Gazebo IMU yaw rate ile
  Betaflight'a giden FDM yaw gyro beklenen inverted ilişkiyi verdi:
  `logs/sitl/20260629-continued-fdm-yaw-cw-bfsitl.jsonl` ve
  `logs/sitl/20260629-continued-fdm-yaw-ccw-bfsitl.jsonl`.
- [~] `yaw_motors_reversed` A/B deneyi otomatikleşti ama sorunu kapatmadı:
  `tools/sitl_mixer_config.py` ve
  `tools/sitl_virtual_takeoff_check.py --yaw-motors-reversed on|off` eklendi.
  `off` koşusu `logs/sitl/20260629-continued-direct-yaw1504-yaw-motors-reversed-off-diagnostics.jsonl`
  ile FAIL, `on` koşusu
  `logs/sitl/20260629-continued-direct-yaw1504-yaw-motors-reversed-on-diagnostics.jsonl`
  ile yine FAIL. ON koşusu nudge gelene kadar daha sakin, fakat yaw 1504 sonrası
  motor spread `945` ve flip devam ediyor.
- [x] Closed-loop actual motor UDP capture runner'a bağlandı:
  `tools/sitl_virtual_takeoff_check.py --capture-motor-udp` artık
  `tools/sitl_motor_udp_probe.py` sürecini aynı koşuda başlatıyor. Raw UDP logu
  BF logical motor değerlerini Gazebo packet slot sırasına (`3,0,1,2`) remap
  ediyor ve roll/pitch/yaw bias özetliyor. Aynı bayrak virtual RC JSONL logunu
  da üretmeye başladı.
- [x] Neutral hover baseline raw motor UDP ile doğrulandı:
  `logs/sitl/20260630-neutral-hold30-motorudp-diagnostics.jsonl` ve
  `logs/sitl/20260630-neutral-hold30-motorudp.jsonl`.
  Sonuç: PASS, altitude gain `23.780 m`, max roll/pitch `0.2/0.2`,
  diagnostics motor spread `0`, raw UDP max spread `0`, axis bias `0`.
- [~] Direct yaw 1504 closed-loop raw UDP koşuları FAIL ama kök neden alanını
  daralttı:
  - `off`: `logs/sitl/20260630-closedloop-yaw1504-off-diagnostics.jsonl`,
    `logs/sitl/20260630-closedloop-yaw1504-off-motor-udp.jsonl`.
  - `on`: `logs/sitl/20260630-closedloop-yaw1504-on-diagnostics.jsonl`,
    `logs/sitl/20260630-closedloop-yaw1504-on-motor-udp.jsonl`.
  İlk büyük raw motor ayrışması iki koşuda da neredeyse saf yaw bias:
  `yaw_cw_minus_ccw ~= 405-409 us`, roll/pitch bias yaklaşık sıfır. Yorum:
  runaway rastgele slot/mapping hatası gibi başlamıyor; Betaflight yaw
  düzeltme/hold zinciri cw/ccw motor çiftlerini agresif ayırarak başlıyor.
- [x] Betaflight yaw PID teşhis kapısı eklendi:
  `tools/sitl_pid_config.py` MSP `MSP_PID` / `MSP_SET_PID` üzerinden yaw P/I/D
  okuyup değiştirebiliyor; `tools/sitl_virtual_takeoff_check.py` içinde
  `--zero-yaw-pid` ve `--yaw-pid P,I,D` runner seçenekleri var.
- [x] Direct virtual RC yaw 1504 + raw motor UDP koşusu yaw PID sıfırlanınca
  PASS verdi:
  `logs/sitl/20260630-yaw1504-zero-yaw-pid-diagnostics.jsonl`,
  `logs/sitl/20260630-yaw1504-zero-yaw-pid-motor-udp.jsonl`,
  `logs/sitl/20260630-yaw1504-zero-yaw-pid-virtual-rc.jsonl`.
  Sonuç: yaw RC gerçekten `1504` gönderildi; altitude gain `23.822 m`,
  max roll/pitch `0.2/0.3`, diagnostics motor spread `0`, raw UDP max spread
  `0.767`, first large raw spread yok.
- [~] Güncel kök neden hükmü: fiziksel RC, Kenet mixer, kaba motor mapping,
  corrected motor axis ve FDM yaw sign ana aday olmaktan çıktı. Runaway,
  Betaflight yaw PID/rate feedback zinciri cw/ccw motor çiftlerini ayırınca
  başlıyor. Yaw PID sıfırlamak çözüm değil; sıradaki kapı yaw P/I terimleri ve
  kontrol işaretini ayrı ayrı ölçmek.
- [x] Yaw P/I sweep'in ilk turu koşuldu. Direct virtual RC yaw 1504 ve raw UDP
  kabul kapısında:
  - `0,0,0` PASS: raw max spread `0.767`.
  - `10,0,0` PASS: raw max spread `0.800`.
  - `15,0,0` PASS: raw max spread `1.073`.
  - `17,0,0` PASS: raw max spread `0.683`.
  - `18,0,0` PASS: raw max spread `0.705`.
  - `19,0,0` PASS: raw max spread `0.760`.
  - `20,0,0` FAIL: raw max spread `945`, first large yaw bias `382.5 us`.
  - `45,0,0` FAIL: raw max spread `945`, first large yaw bias `407.6 us`.
  - `0,1,0` PASS: raw max spread `0.792`.
  - `0,2,0` FAIL: raw max spread `945`, first large pitch bias `400.7 us`.
  - `0,5,0` FAIL: raw max spread `945`; ilk büyük spread yaw'dan önce
    roll/pitch karışmış görünüyor (`roll=207.1`, `pitch=-196.8`, `yaw=-19.0`).
  - `0,20,0` FAIL: raw max spread `945`, first large yaw bias `386.9 us`.
  - `0,80,0` FAIL: raw max spread `945`, first large yaw bias `399.3 us`.
- [~] Güncel yorum: ilk repo-root cwd sweep'i `P20/P22` sınır/flaky gibi
  gösterdi; runner temp-cwd olduktan sonra yaw1504 için `P21` iki PASS, `P22`
  üç PASS / bir FAIL, `P23` iki FAIL verdi. Aynı `P23` yaw1502 ve yaw1503
  hold40 koşularında PASS verdi. Bu yüzden temiz P-only braket artık yaw1504
  için `P21` güvenli taraf, `P22` sınır/flaky, `P23+` stabil FAIL; P23 için
  nudge eşiği yaw1503/yaw1504 arasında. Eski P20/P22 oynaklığı için kalıcı
  `eeprom.bin`/FC state sızıntısı hâlâ güçlü aday. I-only tarafında
  `I=1` PASS, `I=2` FAIL; bu düşük I değerlerinde bile hızlı büyüyen
  integrator/rate feedback/attitude coupling problemi gibi davranıyor. Kalıcı
  çözüm için yaw PID'i kapatmak değil, yaw feedback ölçeği ve integrator
  davranışı ayrı incelenecek.
- [x] Sweep zaman sırası için `tools/sitl_pid_sweep_summary.py` eklendi. Bu araç
  diagnostics, raw motor UDP ve virtual RC JSONL loglarını birlikte okuyup ilk
  nudge, ilk raw motor split ve ilk attitude eşik zamanlarını aynı tabloda
  raporluyor. Aktif uçuş fazına göre P20 raw split nudge'dan `2.943 s`, attitude
  eşiği `4.621 s` sonra; I2 raw split `17.518 s`, attitude eşiği `19.157 s`
  sonra; I5 raw split `5.051 s`, attitude eşiği `6.070 s` sonra.
- [x] Sweep özeti erken raw motor axis-bias eşiklerini de raporlayacak şekilde
  genişletildi: 25/50/100/200/400us ilk geçiş zamanları ve aktif uçuş attitude
  eşiği tabloya eklendi.
  `0,2,0-repeat1` aynı unified runner ile tekrar FAIL verdi; iki I2 koşusunda
  25us axis drift nudge'dan yaklaşık `16.6-16.7 s`, raw split `17.35-17.52 s`,
  attitude eşiği `19.16-19.19 s` sonra geldi. Bu, I2 bozulumunun tekil log
  şansı değil, geç gelişen integrator/rate-feedback davranışı olduğunu
  güçlendiriyor.
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
    attitude `4.6/0.5` ve `0.2/0.3`. Bu, P21'in temiz başlangıçta güvenli
    tarafta kaldığını gösteriyor.
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
  mevcut `MSP_ADVANCED_CONFIG` alanlarını koruyarak sadece `debug_mode`
  ayarlıyor ve doğruluyor. `tools/sitl_virtual_takeoff_check.py --debug-mode
  ...` Betaflight'i kendi başlattığında ayarı `MSP_EEPROM_WRITE` ile kaydedip
  Betaflight'i aynı temp-cwd'den yeniden başlatıyor; çünkü runtime `debugMode`
  init sırasında yükleniyor. Diagnostics JSONL içinde `msp.debug_mode` ve
  `msp.debug` aynı koşunun parçası olarak düşüyor.
- [~] İlk PIDLOOP/ANGLERATE Gazebo koşuları `debug_mode` ayarının doğru
  kaydedildiğini gösterdi, fakat hangi debug modunun gerçekten faydalı olduğu
  belirsizdi. Bunun için `tools/sitl_debug_calibration.py` eklendi: temp-cwd
  Betaflight başlatıyor, `debug_mode` ayarını save+restart ile yükletiyor,
  fixed virtual RC gönderiyor ve `MSP_RC` + `MSP_DEBUG` logluyor.
- [~] Kalibrasyon sonucu: `PIDLOOP` standalone yaw1700 ve yaw1504'te nonzero
  değer veriyor, fakat Betaflight kaynakta bu mod yaw PID iç sinyali değil
  loop timing (`gyroUpdate`, `pidController`, `motorUpdate`, subprocess).
  `ANGLERATE` yaw1700'de sıfır kaldı. `ANGLE_TARGET` standalone sıfır; Gazebo
  P23/yaw1504 koşusunda `debug[3]` büyüdü ama yaw setpoint slotu `debug[2]`
  sıfır kaldı. Bu yüzden MSP_DEBUG hattı henüz yaw setpoint/rate/PID iç
  sinyalini açıklayan güvenilir kanıt değil; sıradaki kapı Blackbox veya
  doğrudan Betaflight instrumentation.
- [x] `tools/sitl_virtual_takeoff_check.py` Betaflight restart sonrası sadece
  `sleep` ile ilerlemiyor; `MSP_ADVANCED_CONFIG` yanıtı gelene kadar kısa
  readiness polling yapıyor. Bu, `--debug-mode ANGLE_TARGET` tekrarında görülen
  erken `ConnectionRefusedError` tipini kapattı.
- [x] Doğrudan Betaflight yaw instrumentation kapısı eklendi:
  `tools/betaflight_yaw_debug_patch.py` dış Betaflight ağacındaki
  `src/main/flight/pid.c` dosyasına marker'lı, geri alınabilir bir blok uygular.
  Geçici olarak `DEBUG_AC_ERROR` şu slotlara ayrıldı:
  `debug[0]=yaw setpoint`, `[1]=gyroRate`, `[2]=errorRate`, `[3]=P`,
  `[4]=I`, `[5]=F`, `[6]=S`, `[7]=Sum`.
- [~] Instrumented Betaflight `make TARGET=SITL` ile derlendi ve P23/yaw1504
  Gazebo koşusunda aynı flip tekrar üretildi:
  `logs/sitl/20260630-debug-acerror-instrumented-yaw1504-p23-*`. Özet:
  raw split nudge'dan `0.399 s` sonra, ilk nonzero yaw debug örneği `0.688 s`
  sonra geldi; setpoint yaklaşık `-1`, gyro `+398..+610`, error `-399..-610`,
  P/Sum `-185..-450`. Bu, runaway sırasında yaw setpoint'in küçük kalmasına
  rağmen ölçülen yaw rate/error ve P/Sum'un hızla büyüdüğünü gösteriyor.
- [x] Aynı instrumentation ile P21 PASS karşılaştırması alındı:
  `logs/sitl/20260630-debug-acerror-instrumented-yaw1504-p21-*`. P21 koşusu
  PASS verdi; altitude gain `31.686 m`, max roll/pitch `0.8/0.6`, raw spread
  `0.802`, aktif debug max `setpoint=1`, `gyro=0`, `error=1`, `P/Sum=0`.
  P23 FAIL koşusundaki aktif `gyro/error=610` ve `P/Sum=450` ile fark artık net.
- [x] P22 instrumented repeat sınır/flaky davranışı doğruladı. İlk P22 koşusu
  FAIL (`logs/sitl/20260630-debug-acerror-instrumented-yaw1504-p22-*`):
  raw split `2.776 s`, attitude `7.668 s`, aktif `gyro/error=1740/1741`,
  `P/Sum=1243`. Repeat PASS
  (`logs/sitl/20260630-debug-acerror-instrumented-yaw1504-p22-repeat1-*`):
  altitude gain `31.654 m`, max roll/pitch `4.4/3.7`, raw spread `0.843`,
  aktif `gyro=0`, `error=1`, `P/Sum=0`.
- [~] P23 için dar fix probları tek başına geçmedi: `--iris-yaw-gyro-scale`
  `0.5/0.25` aktif yaw P/Sum'u düşürdü ama pitch/attitude FAIL'e kaydı;
  `--iris-rotor-vel-p-gain 0.01` split'i geciktirdi fakat kaldırmadı;
  `--max-step-size 0.001` hâlâ FAIL. Sıradaki kapı Betaflight yaw P/rate
  authority veya Iris'e özel güvenli yaw limit.
- [~] Bu oturumda `/dev/input/js*` görünmedi; fiziksel RC canlı retest'i
  koşturulamadı.
- [ ] Fiziksel RC yoluna dönülürse retest `--force-mode-pwm 1500` ile
  tekrarlanacak.
- [ ] Fiziksel RC ve gerçek hedef bulunan Kenet mixer + Gazebo TRACKING kabul
  testleri aynı runner kriterleriyle geri eklenecek.
- [~] Target-found kabulünden önce yaw/pitch RC offset -> Betaflight ANGLE/rate
  response -> motor spread zinciri tek eksenli daha dar limitlerle izole edildi.
  Yaw PID sıfırlama ve P10/P15/P17/P18/P19 kapıları ayrışmayı kaldırdı; P20 ilk
  koşuda kırıldı ama repeatlerde geçti, P22 repo-root cwd'de bir PASS/bir FAIL,
  temp-cwd'de P21 iki PASS, P22 `3 PASS / 1 FAIL`; P23 temp-cwd dahil yaw1504'te
  iki FAIL, fakat P23 yaw1502 ve yaw1503 hold40 PASS; P25/P45 ve I2+ kapıları
  kırıldı. I2 tarafında yaw `1502` PASS, yaw `1503` uzun hold'da aktif FAIL.
  P-only tarafında temiz runner ile yaw1504 için `P21` PASS, `P22` sınır/flaky
  ve `P23+` stabil FAIL eşiği öne çıktı; P23 yaw1502/yaw1503 PASS sonucu
  kırılımın setpoint/nudge genliğine bağlı olduğunu ve P23 eşiğinin
  `1503/1504` arasında olduğunu gösterdi. Next gate fiziksel RC'yi geri eklemek
  değil; P21/P23 instrumented AC_ERROR farkını kullanarak P22 sınır/flaky
  davranışını açıklamak ve yaw gyro/error/P/Sum eşiği için fix hipotezi üretmek.

Önerilen yeni sıra:

```text
P4 state-machine parity
→ P5 AUX default tekilleştirme
→ P6 tools/ safety testleri
→ P7 log/analyzer güvenilirliği
→ P8 monitor/mapping konsolidasyonu
→ P9 sağlamlaştırmalar
→ MSP transport ve Gazebo tam fizik doğrulaması
```

## Codex yorumları / ek notlar

Genel kararım: Plan doğru yönde. Mevcut sistemi çöpe atmadan önce bu maddelerle
sertleştirmek mantıklı. Yalnız bazı maddelerde sıralama ve kapsam biraz
netleştirilmeli.

### A. P0 / P1 sıralaması kullanım senaryosuna göre ayrılmalı
OpenCV tracker problemi masaüstü test için gerçekten P0. Ama gerçek FC, prop veya
uçuş hazırlığına dokunulacaksa throttle/override maskesi de fiilen P0 kabul
edilmeli. Yani:

- Sadece video/kamera/debug: önce OpenCV.
- FC bağlı, MSP Override veya motor testi: önce throttle/override güvenliği.

### B. OpenCV düzeltmesini proje-local venv ile yap
Aktif Python şu anda başka bir projenin venv'inden geliyor olabilir. Bu repoda
paketleri doğrudan o ortama kurmak başka projeleri bozabilir. Daha güvenlisi:

- Bu proje için `.venv` oluştur.
- Preview/GCS masaüstü kullanılacaksa `opencv-contrib-python` seç.
- Sadece headless companion board hedefleniyorsa `opencv-contrib-python-headless`
  seçilebilir, ama ikisini aynı ortamda karıştırma.
- `tracker.py` factory'si sadece `cv2.TrackerCSRT.create()` varsaymasın; mümkünse
  `cv2.TrackerCSRT_create`, `cv2.legacy.TrackerCSRT_create` ve yeni class tabanlı
  API'leri sırayla denesin. Böylece OpenCV sürüm farkı tekrar sistemi kırmaz.

### C. Throttle maskesinde sadece README değil, test prosedürü de değişmeli
`msp_override_channels = 10` repo kanal eşlemesine göre mantıklı görünüyor
(pitch + yaw). Ama Betaflight tarafında firmware/config sürümüne göre bunu
mutlaka bench test ile doğrulamak gerekir:

- Propeller sökülü.
- `diff all` / `get msp_override_channels` çıktısı kaydedilmiş.
- TRACKING açıkken pilot throttle ve roll hâlâ gerçek alıcıdan geliyor mu kontrol
  edilmiş.
- AUX low olduğunda MSP override gerçekten devreden çıkıyor mu kontrol edilmiş.

Ek öneri: README değişikliğinin yanına küçük bir "bench safety checklist" ekle.
Bu, ileride yanlışlıkla `15`e dönülmesini de engeller.

### D. Plan bir eksik bulguyu içermiyor: GCS stop IDLE'da işlenmiyor
Şu an `_handle_gcs_commands()` sadece `self._state >= AI_ARMED` iken çağrılıyor.
Bu yüzden GCS `stop` komutu IDLE durumunda kuyruğa düşse bile işlenmeyebilir.
Bunu P2'ye eklemek iyi olur:

- Komut alma her döngüde çalışsın.
- Telemetri gönderimi yine state/rate-limit ile sınırlı kalabilir.
- `stop` komutu state'ten bağımsız olmalı.

### E. Headless davranışı da netleştirilmeli
MSP bağlanamazsa pipeline `show_preview=True` yapıyor. Kullanıcı `--headless`
vermişse bu şaşırtıcı ve headless cihazda sorun çıkarabilir. Planın README/kod
tutarsızlığı maddesine eklenebilir:

- `--headless` her koşulda GUI açmamalı.
- Preview-only mod isteniyorsa ayrı bir `--preview` veya `--force-preview`
  davranışı daha açık olur.

### F. Attitude telemetrisi MSP parser'dan önce eklenirse yükü artırır
Attitude doldurma doğru hedef, ama mevcut MSP request yapısı bloklayıcı ve
parser zayıf. Bu yüzden attitude işini yaparken şu sırayı tercih ederim:

1. GCS gönderim rate-limit kararını pipeline tarafında görünür hale getir.
2. Sadece gerçekten paket gönderilecek turda `get_attitude()` çağır.
3. MSP checksum/parser düzeltmesini en azından minimal seviyede aynı PR içinde
   veya hemen sonrasında yap.

### G. Kamera lifecycle dokümantasyonu da güncellenmeli
README "AUX on olunca kamera başlar" gibi anlatıyor, fakat kodda kamera
`start()` sırasında başlıyor ve her state'te frame okunuyor. Bu kötü olmak
zorunda değil, hatta tracker hazırlığı için pratik; ama README bunu doğru
anlatmalı: AUX sadece takip/override state'ini yönetiyor, kamera şu an pipeline
başlangıcında açılıyor.

### H. Küçük test katmanı eklemek planı güçlendirir
Bu repo gerçek drone'a dokunduğu için birkaç hızlı test çok değerli olur:

- `msp_encode` checksum testi.
- `FlightController` throttle/roll sabit, pitch/yaw değişiyor testi.
- Fake MSP + fake camera ile state transition testi.
- GCS `stop` komutu IDLE'da da `_running=False` yapıyor testi.
- Tracker factory başarısızsa okunur hata testi.

Bu testler uçuşu garanti etmez, ama en tehlikeli regressions'ları erken yakalar.

---

## Codex uygulama durumu

Bu turda uygulananlar:

- Tracker factory OpenCV API farklarına dayanıklı hale getirildi ve eksik contrib
  paketinde okunur `TrackerUnavailableError` eklendi.
- README tek 3-pozisyonlu AUX modeline, gerçek CLI argümanlarına ve
  `msp_override_channels = 10` güvenlik yönlendirmesine göre güncellendi.
- GCS `stop` komutu state'ten bağımsız işlenecek şekilde döngünün başına alındı.
- `--headless` verildiğinde MSP yok diye otomatik GUI açma davranışı kaldırıldı.
- Attitude telemetrisi sadece GCS rate-limit izin verdiğinde MSP'den okunup pakete
  dolduruluyor.
- MSP response okuması header tarayan, checksum doğrulayan ve `$M!` hata frame'ini
  ele alan parser'a taşındı; `reset_input_buffer()` kullanımı kaldırıldı.
- AUX polling 30 Hz kontrol döngüsünden ayrılıp varsayılan 15 Hz'e indirildi.
- `.gitignore` test videoları, lokal venv ve test cache'lerini kapsayacak şekilde
  güncellendi.
- `requirements-dev.txt` ve MSP/tracker/controller/pipeline için küçük regresyon
  testleri eklendi.
- `fpv_env` adlı izole virtualenv kuruldu; `opencv-contrib-python`, `pyserial`,
  `numpy` ve `pytest` bu ortama yüklendi.
- `fpv_env/bin/python -m pytest -q` ile testler doğrulandı (`237 passed`,
  2026-06-30).
- 2026-06-30 devamında fiziksel RC izole tutuldu ve virtual RC runner ile
  P23/yaw1504 runaway için ölçülmüş yaw authority profili üretildi:
  `yaw_rc_rate=5`, `yaw_rate=30`, `yaw_rate_limit=120`. Baseline P23 FAIL
  iken aynı koşu bu profille iki kez PASS verdi; runner'da bu artık
  `--safe-yaw-authority` bayrağıyla açıkça uygulanabiliyor.
- Gerçek video target-found neutral gate eklendi ve koşuldu:
  `tools/sitl_video_tracking_check.py --camera test-2.mp4 --run-id
  20260630-video-target-found-neutral` PASS. Diagnostics altitude gain
  `23.850 m`, max roll/pitch `0.000/0.100`, raw spread `0.000`; mixer
  `265` target-found/source=kenet sample ve max pitch/yaw delta `0/0`.
- Gerçek video küçük yaw command-response gate eklendi ve koşuldu. Gecikmesiz
  yaw4/yaw2/yaw1 koşuları FAIL olduktan sonra virtual Kenet state delay
  eklendi; `--kenet-delay-seconds 18 --yaw-pid 23,0,0 --yaw-limit 2`
  koşusu PASS verdi: altitude gain `31.401 m`, max roll/pitch `0.300/0.300`,
  raw spread `0.000`, mixer max pitch/yaw delta `0/2`.
- Aynı gecikmeli/P23 profil `--yaw-limit 4` ile de PASS verdi: altitude gain
  `31.401 m`, max roll/pitch `1.300/5.400`, raw spread `0.000`, mixer max
  pitch/yaw delta `0/4`.
- Aynı gecikmeli/P23 profil `--yaw-limit 5` ile de PASS verdi:
  `20260630-video-target-found-yaw5-p23-delay18-*`; altitude gain `31.377 m`,
  max roll/pitch `3.300/4.300`, raw spread `0.000`, mixer max yaw delta `5`.
- Aynı gecikmeli/P23 profil `--yaw-limit 6` ile FAIL verdi:
  `20260630-video-target-found-yaw6-p23-delay18-*`; max roll `180.0`, max
  pitch `72.4`, MSP motor spread `945`, mixer max yaw delta `6`. Yaw8 de FAIL
  kaldı. Güncel video yaw braket sonucu: yaw5 PASS, yaw6 FAIL.
- Pitch-only gerçek video tarafı ayrı ölçüldü: `--yaw-limit 0`,
  `--forward-limit 1` iki kez FAIL verdi
  (`20260630-video-target-found-pitch1-p23-delay18-*` ve repeat1), max pitch/yaw
  delta `1/0`, MSP motor spread `945`. `--forward-limit 2` de FAIL
  (`20260630-video-target-found-pitch2-p23-delay18-*`). Pitch PID `23/0/0`
  eklendiğinde pitch1/pitch2/pitch4/pitch5 PASS verdi; pitch6 FAIL
  (`20260630-video-target-found-pitch6-p23-delay18-pitchpid23-*`). Güncel pitch
  braket: pitch5 PASS, pitch6 FAIL.
- Birleşik pitch+yaw gerçek video kapısı aynı gecikmeli/P23 ve pitch PID
  profiliyle ölçüldü. `yaw2+pitch2` PASS: altitude gain `31.441 m`, max
  roll/pitch `3.100/2.600`, max motor spread `0.000`, max pitch/yaw delta
  `2/2`. `yaw3+pitch3` PASS: altitude gain `31.483 m`, max roll/pitch
  `3.200/3.300`, max motor spread `0.000`, max pitch/yaw delta `3/3`.
  `yaw4+pitch4` FAIL: altitude gain `8.169 m`, max roll/pitch
  `180.000/55.400`, MSP motor spread `945`, max pitch/yaw delta `4/4`. Güncel
  birleşik braket: yaw3+pitch3 PASS, yaw4+pitch4 FAIL.

Bu turda bilinçli olarak yapılmayan:

- Başka bir projenin aktif Python ortamından paket kaldırılmadı. Bunun yerine
  proje-local `fpv_env` kullanıldı.
- Gerçek FC/Betaflight bench testi yapılmadı; `msp_override_channels=10`,
  roll/throttle pilotta kalıyor mu ve AUX düşüşünde MSP Override kesiliyor mu
  propeller sökülü test edilmeli.
- Throttle/roll kararı kapatıldı: mevcut ürün kapsamı pitch/yaw-only. Roll veya
  throttle Kenet'e verilirse ayrı RFC ve yeni bench gate gerekecek.
