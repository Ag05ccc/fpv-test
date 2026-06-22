# Latest Development — 2026-06-23

Bu doküman Kenet / Betaflight SITL tarafındaki en son durumu özetler. Son
odak noktası, gerçek uçuş kontrolcüsü olmadan RC kumanda, Betaflight SITL ve
Kenet görsel takip/PID çıktısını aynı masaüstü test hattında doğrulamaktı.

---

## Kısa Sonuç

SITL geliştirmelerinde önemli eşik geçildi:

```text
RC kumanda + Kenet visual tracker/PID
        -> tools/kenet_sitl_mixer.py
        -> Betaflight SITL UDP 9004 rc_packet
        -> Betaflight Configurator Receiver tab
```

`TRACKING` aşamasında Kenet'in ürettiği pitch ve yaw komutları Betaflight
Receiver tarafında otomatik olarak görüldü. Roll ve throttle pilotta kalıyor.
Bu, SITL tarafındaki ilk güvenli ve anlaşılır Kenet kontrol doğrulaması oldu.

---

## Mevcut Test Seviyesi

Şu an MSP kullanılmıyor. Çalışan yol daha sade:

```text
Joystick -> Python RC mapping -> Kenet SITL mixer -> UDP 9004 -> Betaflight SITL
```

Bu yolun amacı, MSP ve Gazebo karmaşıklığına girmeden önce Kenet'in kontrol
mantığını test etmek.

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
- [x] `pytest` sonucu temiz: `11 passed`.

Henüz tamamlanmayanlar:

- [ ] PID yönleri bilinçli hedef hareketleriyle tek tek doğrulanacak.
- [ ] Hedef kaybı test matrisi tamamlanacak.
- [ ] MSP transport SITL'e bağlanacak.
- [ ] Gazebo/fizik simülasyonu en sona bırakılacak.
- [~] Tango 2 üzerindeki ikinci fiziksel three-state switch USB joystick
  çıktısında görünür hale getirilecek veya şimdilik pas geçilecek.

---

## Son Eklenen / Güncellenen Araçlar

### `tools/kenet_sitl_mixer.py`

Yeni ana SITL entegrasyon aracı. Pilot RC girdisi ile Kenet'in görsel takip/PID
çıktısını tek RC packet içinde birleştirir.

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
python tools/kenet_sitl_mixer.py --device /dev/input/js0 --camera test-2.mp4
python tools/kenet_sitl_mixer.py --device /dev/input/js0 --camera test-2.mp4 --send --print-hz 5
```

### `tools/sitl_rc_bridge.py`

Minimal RC -> Betaflight SITL bridge. Python stdlib ile Linux joystick okur ve
Betaflight SITL `rc_packet` formatında UDP `9004` portuna gönderir.

Kullanım:

```bash
python tools/sitl_rc_bridge.py --device /dev/input/js0 --dry-run
python tools/sitl_rc_bridge.py --device /dev/input/js0 --send --verbose
```

### `tools/rc_monitor.py`

Joystick ve mapped RC kanallarını görsel veya terminal modunda izler. Tk yoksa
terminal moduna düşebilir.

### `tools/state_monitor.py`

Tek dosyalık stdlib web GUI. Kenet state, ARM state, autopilot mode, raw axes
ve mapped RC kanallarını aynı ekranda gösterir.

```bash
python tools/state_monitor.py --open
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
| Autopilot mode | CH7 / AUX3 | Axis 5 | şimdilik Tango ayarı bekliyor |

Kenet state:

```text
AUX2 LOW  -> IDLE
AUX2 MID  -> AI-ARMED
AUX2 HIGH -> TRACKING
```

Tasarım kararı:

`AUX2 / CH6` Betaflight mode'a bağlanmayacak. Bu kanal sadece Kenet state için
ayrıldı. Betaflight ARM için `AUX1 / CH5` kullanılacak.

---

## OpenCV / Venv Hatası ve Çözüm

Bir testte şu hata görüldü:

```text
TrackerCSRT is unavailable in this OpenCV build
```

Kök neden: Komut yanlış Python ortamıyla çalışmıştı. Sistem `python3` komutu
başka bir venv'e, örneğin `/home/gz/VBN/.venv/bin/python3`, gidiyordu. O ortamda
OpenCV contrib tracker modülleri yoktu.

Doğru ortam:

```bash
fpv-test
which python
python -c "import cv2; print(cv2.__file__, hasattr(cv2, 'TrackerCSRT_create'))"
```

Beklenen:

```text
/home/gz/fpv-test/fpv_env/bin/python
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
11. Ana Kenet pipeline'ı SITL MSP hattına bağlama.
12. Opsiyonel Gazebo/fizik simülasyonu.
13. Donanım öncesi çıkış kriterleri.

Planın temel kararı değişmedi:

```text
Önce kontrol zinciri, sonra MSP, en son fizik simülasyonu.
```

---

## Yakın Sıradaki İş

En mantıklı sıradaki adım:

```bash
python tools/kenet_sitl_mixer.py --device /dev/input/js0 --camera test-2.mp4 --send --print-hz 5
```

Bu komut çalışırken Configurator Receiver tab'de şu test matrisi tamamlanmalı:

- AUX2 LOW: tüm kanallar pilotta.
- AUX2 MID: tüm kanallar pilotta.
- AUX2 HIGH + hedef var: pitch/yaw Kenet'ten.
- AUX2 HIGH + hedef yok: pitch/yaw pilota geri düşüyor.
- TRACKING sırasında throttle hâlâ pilotta.
- TRACKING sırasında roll hâlâ pilotta.

Bundan sonra PID yönleri doğrulanacak. MSP aşamasına geçiş, bu test matrisi
tamamlandıktan sonra daha temiz ve anlaşılır olacak.
