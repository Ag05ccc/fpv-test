# Kenet / Betaflight SITL Yol Haritası

Amaç: Gerçek drone'a geçmeden önce Kenet'in kontrol zincirini masaüstünde
kanıtlamak. Plan mümkün olduğunca sade tutulur: önce RC ve Betaflight SITL
zinciri, sonra Kenet'in görsel takip çıktısı, en son gerekirse MSP ve fizik
simülasyonu.

Durum işaretleri: `[ ]` yapılacak · `[~]` kısmen tamam / beklemede · `[x]` tamam

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

1. Önce Seviye 1 ve Seviye 2 sağlamlaşacak.
2. MSP sadece kontrol mantığı güvenilir olduktan sonra eklenecek.
3. Gazebo/fizik simülasyonu en sona bırakılacak.

---

## 1. Güncel Durum Özeti

2026-06-23 itibarıyla:

- [x] TBS Tango 2 USB joystick olarak okunuyor.
- [x] Roll/pitch/throttle/yaw kanalları okunuyor.
- [x] Arm için two-state switch okunuyor.
- [x] Kenet state için three-state switch okunuyor.
- [x] Betaflight SITL çalıştırıldı.
- [x] Betaflight Configurator SITL'e bağlandı.
- [x] Receiver tab'de ana kanallar görüldü.
- [x] ARM mode ayarı çalıştı.
- [x] `tools/sitl_rc_bridge.py` ile RC paketleri `9004/udp` üzerinden gönderildi.
- [x] `tools/kenet_sitl_mixer.py` ile TRACKING sırasında pitch/yaw otomatik geldi.
- [x] Roll/throttle pilotta bırakıldı.
- [x] Yanlış Python/OpenCV ortamında tracker yoksa mixer artık çökmeden pilot
  passthrough yapıyor.
- [~] İkinci fiziksel three-state switch Tango 2 USB joystick çıktısında henüz
  görünmüyor; bu kısım Tango 2 model/mixer ayarına bırakıldı.
- [ ] MSP transport SITL'e henüz bağlanmadı.
- [ ] Gazebo/fizik simülasyonu henüz başlatılmadı.

---

## 2. Bağımlılık Politikası

İlk aşamada kullanılacak minimumlar:

- Python stdlib joystick/UDP araçları
- Proje venv'i: `/home/gz/fpv-test/fpv_env`
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
/home/gz/fpv-test/fpv_env/bin/python
True
11 passed
```

Sorun belirtisi:

```text
TrackerCSRT is unavailable in this OpenCV build
```

Anlamı: Büyük ihtimalle yanlış venv aktiftir. Doğru ortam:

```bash
source /home/gz/fpv-test/fpv_env/bin/activate
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
/dev/input/js0
/dev/input/by-id/usb-Team-BlackSheep_TBS_Joystick_00000000001B-joystick
```

Araçlar:

```text
tools/sitl_rc_bridge.py
tools/rc_monitor.py
tools/state_monitor.py
```

Komutlar:

```bash
python tools/sitl_rc_bridge.py --device /dev/input/js0 --dry-run
python tools/sitl_rc_bridge.py --device /dev/input/js0 --changes
python tools/rc_monitor.py --device /dev/input/js0
python tools/state_monitor.py --open
```

Kabul kriteri:

- [x] CH1-CH4 stick hareketleri 1000-2000 aralığında görülüyor.
- [x] Throttle düşükte 1000, yüksekte 2000.
- [x] Roll/pitch/yaw merkezde 1500.
- [x] Axis 4 ARM için 1000/2000 üretiyor.
- [x] Axis 6 Kenet state için 1000/1500/2000 üretiyor.
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
/home/gz/betaflight/obj/main/betaflight_SITL.elf
```

SITL çalıştırma:

```bash
~/betaflight/obj/main/betaflight_SITL.elf
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
python tools/sitl_rc_bridge.py --device /dev/input/js0 --dry-run
python tools/sitl_rc_bridge.py --device /dev/input/js0 --send --verbose
python tools/sitl_rc_bridge.py --device /dev/input/js0 --send --once --verbose
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
| ANGLE | AUX3 / CH7 | 1300-1700 | [ ] |
| HORIZON | AUX3 / CH7 | 1700-2100 | [ ] |
| Kenet state | AUX2 / CH6 | Betaflight mode yok | [x] |

Kritik not:

`AUX2 / CH6` Betaflight mode'a bağlanırsa Kenet state switch'i ile autopilot
mode switch'i birbirine karışır. Bu yüzden AUX2 sadece Kenet state için
ayrılacak.

Kabul kriteri:

- [x] ARM switch high konumunda Betaflight ARM mode aktif oluyor.
- [x] ARM switch low konumunda ARM mode pasif oluyor.
- [ ] CH7/AUX3 görünür hale gelince flight mode mapping doğrulanacak.
- [ ] Failsafe/arming flags Configurator üzerinden not edilecek.

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
python tools/kenet_sitl_mixer.py --device /dev/input/js0 --no-vision --duration 2
python tools/kenet_sitl_mixer.py --device /dev/input/js0 --camera test-2.mp4 --duration 2
python tools/kenet_sitl_mixer.py --device /dev/input/js0 --camera test-2.mp4 --send --print-hz 5
```

Geçici masaüstü test kolaylığı:

```bash
python tools/kenet_sitl_mixer.py --device /dev/input/js0 --camera test-2.mp4 \
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
- [ ] PID yönleri bilinçli hedef hareketleriyle tek tek doğrulanacak.

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

## 10. Aşama P6 - Tekrarlanabilir Test Matrisi

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

- Terminalde `tools/kenet_sitl_mixer.py --print-hz 5` çıktısı izlenir.
- Configurator Receiver tab aynı anda açık tutulur.
- Gerekirse ekran kaydı veya kısa not alınır.

Kabul kriteri:

- [ ] Test matrisi bir kez baştan sona tamamlandı.
- [ ] Anormal kanal davranışı varsa hangi testte olduğu not edildi.
- [ ] PID yönleri doğru/ters şeklinde işaretlendi.

---

## 11. Aşama P7 - PID Yön ve Limit Doğrulaması

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
python tools/kenet_sitl_mixer.py --camera test-2.mp4 --send \
  --yaw-kp 0.8 --yaw-ki 0.05 --yaw-kd 0.15 \
  --forward-kp 0.4 --forward-ki 0.02 --forward-kd 0.1
```

Kabul kriteri:

- [ ] Yaw yönü doğru.
- [ ] Pitch/forward yönü doğru.
- [ ] Komutlar 1000-2000 dışına taşmıyor.
- [ ] Hedef kaybında Kenet pilotu bloke etmiyor.
- [ ] İlk güvenli PID değerleri dokümante edildi.

---

## 12. Aşama P8 - Loglama ve GCS ile İzleme

Amaç: Test sırasında sadece Configurator'a bakmak yerine Kenet tarafındaki
state, target ve kanal kararlarını da kayıt altına almak.

Mevcut araçlar:

```text
tools/state_monitor.py
tools/rc_monitor.py
kenet/gcs.py
```

Plan:

- [ ] `kenet_sitl_mixer.py` için opsiyonel CSV/JSONL log eklensin.
- [ ] Her satırda zaman, state, source, target, pilot RC, final RC yazılsın.
- [ ] GCS telemetrisi ile aynı state bilgisi karşılaştırılsın.
- [ ] Bir test koşusunun çıktısı kısa raporlanabilir hale gelsin.

Basit log formatı önerisi:

```text
timestamp,state,source,target_found,pilot_ch1..pilot_ch8,final_ch1..final_ch8
```

Kabul kriteri:

- [ ] Bir test koşusu sonradan incelenebiliyor.
- [ ] Hedef kaybı ve state geçişleri logdan okunabiliyor.
- [ ] Configurator'da görülen davranış ile log davranışı tutarlı.

---

## 13. Aşama P9 - MSP Yoluna Geçiş Kararı

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
- [ ] PID yönleri doğrulandı.
- [ ] Hedef kaybı ve state geçişleri test matrisi tamamlandı.

Karar:

```text
Bu şartlar tamamlanmadan MSP'ye geçmek gereksiz karmaşıklık yaratır.
```

---

## 14. Aşama P10 - MSP SITL Transport

Amaç: `kenet/msp.py` kodunu Betaflight SITL'in `5761/tcp` MSP hattına bağlamak.

İki sade seçenek var.

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

Artı:

- Ek sistem aracı gerekmez.
- SITL için daha doğrudan çalışır.

Eksi:

- `kenet/msp.py` içinde transport ayrımı yapılmalıdır.

İlk öneri:

- [ ] Önce Seçenek A ile hızlı MSP smoke test.
- [ ] Sonra gerekirse Seçenek B ile temiz TCP transport.

MSP testleri:

- [ ] `MSP_API_VERSION` okunabiliyor.
- [ ] `MSP_RC` okunabiliyor.
- [ ] `MSP_ATTITUDE` okunabiliyor.
- [ ] `MSP_SET_RAW_RC` gönderildiğinde Receiver tab etkileniyor.
- [ ] Configurator kapalıyken doğrudan MSP testleri çalışıyor.

Kabul kriteri:

- [ ] Kenet, SITL'e MSP üzerinden bağlanıyor.
- [ ] MSP mesajları timeout olmadan dönüyor.
- [ ] `MSP_SET_RAW_RC` ile pitch/yaw gönderilebiliyor.
- [ ] UDP mixer ile MSP yolunun kanal davranışı aynı.

---

## 15. Aşama P11 - Kenet Ana Pipeline ile SITL MSP

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

Özel güvenlik maddesi:

Gerçek sistemde throttle'ın yanlışlıkla 1500'e kilitlenmesi riskine karşı:

- [ ] MSP override maskesi sadece gereken kanalları kapsıyor mu?
- [ ] Roll/throttle pilotta kalıyor mu?
- [ ] TRACKING düşerse pilot anında geri geliyor mu?
- [ ] Hedef kaybında override bırakılıyor mu?

Kabul kriteri:

- [ ] Ana Kenet pipeline SITL MSP hattında çalışıyor.
- [ ] State machine davranışı UDP mixer ile aynı.
- [ ] Throttle pilot kontrolünden kopmuyor.
- [ ] MSP timeout veya buffer reset sorunu görülmüyor.

---

## 16. Aşama P12 - Opsiyonel Gazebo / Fizik Simülasyonu

Amaç: Kontrol zinciri ve MSP hattı kanıtlandıktan sonra drone'un fiziksel
cevabını sim ortamında gözlemek.

Bu aşama şu şartlardan önce başlatılmayacak:

- [x] RC/SITL zinciri çalıştı.
- [x] Kenet UDP mixer TRACKING pitch/yaw üretti.
- [ ] PID yönleri doğrulandı.
- [ ] MSP transport smoke test geçti.
- [ ] Ana pipeline MSP ile çalıştı.

Neden sona bırakılıyor?

- Gazebo kurulum ve plugin zinciri bağımlılıkları ağırdır.
- İlk sorunları fizik simülasyonunda aramak hatayı büyütür.
- Bizim önce kanıtlamamız gereken şey RC/MSP kontrol zinciridir.

Gazebo için olası yol:

- [ ] Betaflight'ın güncel SITL/Gazebo dokümanı tekrar kontrol edilecek.
- [ ] Gazebo Harmonic veya sistemde mevcut sürüm tespit edilecek.
- [ ] aeroloop / betaloop / aeroloop_gazebo mevcut kurulumları envanterlenecek.
- [ ] Hazır quad model/world ile motor output görülmeye çalışılacak.
- [ ] Kenet kamera girdisi sim kameradan mı, test videosundan mı gelecek
  karar verilecek.

Kabul kriteri:

- [ ] SITL motor output sim tarafına ulaşıyor.
- [ ] Sim drone RC kumanda ile tepki veriyor.
- [ ] Kenet TRACKING sırasında sim davranışı gözlenebiliyor.
- [ ] Sim kurulumu ayrı dokümante edildi.

---

## 17. Aşama P13 - Donanım Öncesi Çıkış Kriterleri

SITL'den gerçek FC testine geçmeden önce:

- [ ] UDP mixer test matrisi tamamlandı.
- [ ] PID yönleri doğru.
- [ ] Hedef kaybı güvenli.
- [ ] MSP transport testi başarılı.
- [ ] Throttle/roll override riski kapatıldı.
- [ ] ARM ve Kenet state switch'leri ayrı kanallarda net.
- [ ] GCS/log ile state geçişleri izlenebiliyor.
- [ ] Test prosedürü yazılı.

Gerçek FC bench test ilkeleri:

- Pervaneler sökük.
- USB/UART bağlantısı ayrı doğrulanır.
- İlk testte motor çıkışı değil Receiver/MSP davranışı izlenir.
- Throttle pilotta kalmadan uçuş testine geçilmez.

---

## 18. Kısa Komut Rehberi

Yeni terminal:

```bash
fpv-test
```

SITL:

```bash
~/betaflight/obj/main/betaflight_SITL.elf
```

Configurator köprüsü:

```bash
websockify 127.0.0.1:6761 127.0.0.1:5761
```

RC bridge:

```bash
python tools/sitl_rc_bridge.py --device /dev/input/js0 --dry-run
python tools/sitl_rc_bridge.py --device /dev/input/js0 --send --verbose
```

Monitor:

```bash
python tools/rc_monitor.py --device /dev/input/js0
python tools/state_monitor.py --open
```

Kenet joystick desk-test:

```bash
python kenet.py --joystick --aux-ch 5 --joy-axis 6 --joy-arm-axis 4 \
  --camera test-2.mp4 --no-gcs
```

Kenet SITL mixer:

```bash
python tools/kenet_sitl_mixer.py --device /dev/input/js0 --camera test-2.mp4
python tools/kenet_sitl_mixer.py --device /dev/input/js0 --camera test-2.mp4 --send --print-hz 5
```

MSP pseudo serial denemesi:

```bash
socat -d -d pty,raw,echo=0,link=/tmp/bf-sitl,mode=666 tcp:127.0.0.1:5761
```

---

## 19. Yakın Sıradaki İşler

En yakın pratik sıra:

1. [ ] `kenet_sitl_mixer.py --send` ile Receiver tab test matrisini tamamla.
2. [ ] PID yönlerini bilinçli hedef hareketleriyle doğrula.
3. [ ] Hedef kaybı durumunda pilot passthrough davranışını doğrula.
4. [ ] Basit CSV/JSONL log ekle.
5. [ ] MSP için önce `socat` pseudo serial smoke test yap.
6. [ ] Gerekirse `msp.py` içine TCP transport ekle.
7. [ ] Ana `kenet.py` pipeline'ını SITL MSP hattında çalıştır.
8. [ ] Gazebo/fizik simülasyonuna ancak bu aşamalardan sonra karar ver.
