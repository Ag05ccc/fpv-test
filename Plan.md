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
- [ ] (Karar) Kenet ileride throttle/roll sürecekse maske tekrar gözden geçirilsin
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
- `fpv_env/bin/python -m pytest -q` ile testler doğrulandı (`11 passed`).

Bu turda bilinçli olarak yapılmayan:

- Başka bir projenin aktif Python ortamından paket kaldırılmadı. Bunun yerine
  proje-local `fpv_env` kullanıldı.
- Gerçek FC/Betaflight bench testi yapılmadı; `msp_override_channels=10`,
  roll/throttle pilotta kalıyor mu ve AUX düşüşünde MSP Override kesiliyor mu
  propeller sökülü test edilmeli.
- Throttle/roll'un ileride Kenet tarafından sürülüp sürülmeyeceği ürün/tasarım
  kararı olarak açık bırakıldı.
