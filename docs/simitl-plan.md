# SimITL Kurulum ve Değerlendirme Planı

Tarih: 2026-07-04. ÖNCE `AGENT-STATUS.md` OKU — dürüst durum orada: kapalı
çevrim takip ÇALIŞMIYOR, engel plant'in sürekli-yaw spin kararsızlığı; eski
"A4 PASS" iddiaları geçersiz (yanlış ölçen kapılar). Bu plan, o kök engelin
SimITL adayıyla çözülmesini tanımlar ve KRİTİK YOLDADIR: plant düzelmeden
hiçbir dürüst takip kapısı geçemez. Karar felsefesi: tam geçiş taahhüdü YOK —
önce zaman kutulu bir deney (Faz 0–3, ~2-3 gün), karar ancak Faz 3'ün
ölçülmüş sonucuyla verilir.

## Neden SimITL

Ölçülmüş kök problem (AGENT-STATUS.md, 2026-07-04): sürekli aynı yönde
tutulan yaw komutu Gazebo iris plant'ini kararsızlaştırıyor — 50 Hz poz
izlemede tepe **7655 °/s** spin ölçüldü; en taze repro 21:34 koşusu
(yaw-limit 6): tracker hedefi -320 px hatayla yakalayıp komut limitte satüre
kalınca spin TAM o anda başladı (t=22.1 s), 10 s'de -1512 °/s. Kullanılabilir
band yok: yaw-limit 3 seviyede kalıyor ama hedef kaçıyor, 6'da spin. Yaw
otoritesini düşürmek kararsızlığı roll/pitch'e taşıyor; Betaflight zamanlaması
(lockstep/sync on/off) dışlandı — mekanizma plant tarafında (iris + LiftDrag).
İris retune'u açık uçlu cerrahi; SimITL ise aynı problemin başkası tarafından
çözülmüş hali:

- [SimITL](https://github.com/AJ92/SimITL) (GPL-3.0): gerçek Betaflight'ı
  (AJ92 fork'u, 2025.12.0-pre tabanı, `simITL` hedefi, scheduler yamalı)
  FPV-sınıfı bir fizik modeliyle TEK SÜREÇTE çalıştıran C++ kütüphanesi.
  pr0p.dev yarış simülatörünün açık çekirdeği.
- Fizik modeli (kaynaktan doğrulandı, `src/network/packets.h`): motor
  elektrik modeli (KV/direnç/termal/yanma), pervane tork+atalet+harmonikler,
  prop wash, zemin etkisi, gövde sürüklemesi, batarya voltaj çökmesi,
  jiro gürültüsü. İris'in P-only rotor döngüsüyle kıyaslanamaz.
- Arayüz üç fonksiyonluk C ABI (`src/main.cpp`): `simitl_init(StateInit)`,
  `simitl_update(StateInput)`, `simitl_get_state() → StateOutput`. UDP/lockstep
  YOK — bizim looptime-sync/semafor yama sınıfımızın tamamı mimariyle düşer.
- RC doğrudan `StateInput.rcData[8]` (normalize -1..1) ile verilir; MSP için
  kütüphane arka planda Configurator bağlantısı dinler (`serial_tcp.c`,
  `serial_ws.c` ağaçta; tester README "connect to the Betaflight
  Configurator" diyor).
- `tools/simitl-tester`: oyun motoru olmadan 60 Hz referans host döngüsü,
  HARDCODED quad parametre seti ve terminale OSD dökümü. Bizim Python
  host'umuzun şablonu.

Hedef mimari (Faz 3 PASS ise): SimITL = plant + Betaflight; Gazebo = salt
dünya/renderer (iris pozu `set_pose` ile kukla gibi basılır). Kamera köprüsü
(`kenet/gz_camera.py`), hedef sürücü, ground-truth ve populated dünya AYNEN
kalır. Ölçülmüş iki kök neden (looptime çarpıklığı, iris yaw otoritesi) tanım
gereği ortadan kalkar.

## Faz 0 — Kurulum (yarım gün)

```bash
git clone https://github.com/AJ92/SimITL ~/SimITL
cd ~/SimITL
# Tuzak 1: submodule URL'leri SSH (git@github.com:...). HTTPS'e çevir:
git config url."https://github.com/".insteadOf "git@github.com:"
git submodule update --init --recursive
# Tuzak 2: setup.sh win+linux birlikte configure eder; mingw yoksa win kısmı
# hata verir. Yalnız Linux:
cmake -S . -B build/linux -D CMAKE_BUILD_TYPE=Release
cmake --build build/linux -j"$(nproc)"
```

- Konum: `~/SimITL` (kardeş dizin; `~/betaflight` ve `~/aeroloop_gazebo`
  düzeniyle tutarlı). Bizim `~/betaflight` checkout'una DOKUNULMAZ — SimITL
  kendi betaflight submodule'ünü kullanır, mevcut kanıt dünyası etkilenmez.
- Kayıt: SimITL commit hash + betaflight submodule hash plana işlenir.
- Çıkış kriteri: shared lib + `simitl-tester` Linux binary'leri derlendi.
- Muhtemel pürüzler: libwebsockets bağımlılıkları (openssl dev paketi),
  eksik apt paketleri. Çözümleri bu dosyaya not edilir.

## Faz 1 — Tester ile smoke + arayüz haritası (yarım gün)

- `simitl-tester`'ı TEMP CWD'de çalıştır (eeprom `test.bin` oluşturur —
  repo-root eeprom sızıntısı dersi burada da geçerli).
- Çalışırken `ss -ltnp` ile hangi portların açıldığını haritala (beklenti:
  stok SITL 5761 şeması ve/veya websocket). `kenet_msp_smoke.py --msp-tcp`
  ile bağlan: API/STATUS/RC/ATTITUDE okunmalı.
- `sitl_configure_modes.py` ile ARM/ANGLE mode range yazmayı dene.
- Tester kaynağından çıkarılacak üç bilgi (Python host'un girdileri):
  1. Hardcoded `StateInit` quad parametre seti (bizim başlangıç setimiz),
  2. Gravity/pozisyon entegrasyonunun kimde olduğu (host mu lib mi),
  3. Zemin teması (`contact`) ve RC besleme deseni.
- Çıkış kriteri: MSP üzerinden canlı okuma PASS + arayüz haritası bu dosyaya
  işlendi. MSP hiç yoksa/uyumsuzsa bu bir KARAR bulgusudur (araç zincirimizin
  taşınabilirliği düşer) ve Faz 3 kararına girdi olur.

## Faz 2 — Python ctypes host (1 gün)

- Yeni araç: `tools/simitl_host.py`
  - `packets.h` POD struct'larının ctypes karşılıkları; bilinen-değer
    karşılaştırma testiyle ABI doğrulaması (tester çıktısı ↔ host çıktısı).
  - Sabit-dt döngü (60–200 Hz; tester'ın kendisi 60 Hz — Python için yeterli
    kanıt). Pozisyon entegrasyonu + basit zemin teması host'ta (Faz 1
    bulgusuna göre).
  - RC adaptörü: bizim 1000–2000 µs sanal RC sözleşmesi → -1..1 `rcData`;
    kanal sırası tek kaynaktan (`kenet/rc_channels.py`). Mevcut sanal RC
    scriptleri UDP yerine bu adaptörle beslenir.
  - JSONL state logu (attitude/motor RPM/batarya) — mevcut analiz araçlarının
    okuyabildiği şemaya yakın.
- Çıkış kriteri: headless arm + hover — attitude seviye, RPM makul, MSP ile
  eş zamanlı canlı okuma; kanıt JSONL.

## Faz 3 — KARAR DENEYİ: sürekli-yaw flip reprosu (yarım gün)

Gazebo'da 8/8 flip yaptıran senaryonun eşleniği SimITL'de koşulur:

- Hover + SÜREKLİ TUTULAN küçük yaw komutu, 40 s (yaw1504 eşleniği ≈
  normalize +0.008; ayrıca 1502 eşleniği, mixer satürasyon eşleniği ±6 µs ve
  daha büyük adımlar).
- PID merdiveni: Betaflight default, P19, P21, P23 (yaw P-only profillerimiz).
- Metrikler — AGENT-STATUS ölçüm tuzaklarına göre tanımlı: yaw HIZI kapısı
  şart (heading/roll-pitch kapısı yasak). SimITL'de `StateOutput`
  angularVelocity doğrudan okunur (MSP aliasing riski yok): yaw hızı komutla
  orantılı ve sınırlı (küçük komutta < 90 °/s, runaway yok), |roll|,|pitch|
  < 25°, motor satürasyon ayrışması yok.
- **PASS ⇒** plant-değişim yolu ONAYLI; Faz 4'e geçilir.
- **FAIL ⇒** bulgular roadmap-v2'ye işlenir, iris retune planına dönülür.
  Her iki sonuç da ölçümdür; "çalışmadı" bile değerli çıktıdır.

## Faz 4 — Gazebo kukla köprüsü (2-3 gün; YALNIZ Faz 3 PASS ise)

- Yeni araç: `tools/simitl_gazebo_bridge.py` — SimITL host + `set_pose`
  puppet: populated dünyada iris pozunu 50–100 Hz basar.
  `sitl_target_mover.py`'ın kanıtlı gz-transport desenleri aynen kullanılır
  (servis ısınma adımı, mutlak-tik temposu, tik atlama).
- İris'in Gazebo fiziği devre dışı bırakılır (statik/kinematik model
  varyantı). DOĞRULANACAK risk: statik modele bağlı kamera sensörünün
  `set_pose` ile taşınırken render etmeye devam ettiği (hedef modellerimiz
  statik ve render ediyor; sensörlü statik model ayrıca test edilir).
- Kamera yolu, hedef sürücü ve ground-truth DEĞİŞMEZ.
- Çıkış kriterleri: kamera probu 30 fps (kukla uçarken); neutral kapı PASS;
  ardından hareketli-hedef kapısı — NİHAİ hakem `sitl_track_truth.py` sınıfı
  POZİSYON YAKINSAMASI (dron hedefe fiziken yaklaşıyor/birlikte öteleniyor +
  spin yok); `target_found`/heading/roll-pitch metrikleri tek başına kanıt
  DEĞİLDİR (AGENT-STATUS tuzak listesi). Tüm kanıt satırları `world=simitl`
  etiketi taşır — Gazebo-plant ve sync-dünyası kanıtlarıyla ASLA karışmaz.

## Faz 5 — Kapı göçü ve nihai karar

- `docs/sitl-flight-readiness-criteria.md` merdiveni SimITL dünyasında
  koşulur; her kapı orada 5/5 ölçüldükçe tek tek göçer (sync-dünyası göç
  politikasının aynısı).
- Gazebo-plant yolu, tüm kapılar göçene kadar çalışır durumda tutulur.
- Beklenen kazançlar: gerçekçi manuel/klavye uçuş hissi, geniş yaw-otorite
  bandı (A7 hız merdiveni için), lockstep/RTF dert sınıfının kapanması.

## Risk listesi

- Quad parametre seti → kaynağı tester'ın hardcoded modeli (Faz 1'de çıkar).
- Gravity/entegrasyon sahipliği → Faz 1'de netleşir; yanlış varsayım hover'ı
  bozar, o yüzden Faz 2 kapısı hover'dır.
- Koordinat/işaret dönüşümleri (quaternion, ENU/FLU) — gyro-sign vakasından
  bilinen tuzak sınıfı → Faz 2'ye açık eksen-yön kapısı eklenir (sağa komut →
  poz sağa döner, işaretli iddia).
- MSP yüzeyi (port şeması, BF 2025.12 MSP farkları) → Faz 1'de erken doğrula;
  tüm araç zincirimiz MSP'ye yaslanıyor.
- ctypes ABI hizalaması → bilinen-değer karşılaştırma testi (Faz 2).
- Upstream genç ve tek kişilik (33 yıldız, 72 commit, release yok) → commit
  pinlenir; gerekirse vendor edilir. GPL-3.0 dahili kullanımda sorunsuz.
- Python host kadansı yetmezse → önce 60 Hz'te kal (tester emsali), gerekirse
  ince C shim / cffi.

## Zamanlama ve çalışma kuralları

- Faz 0–3 (karar noktasına kadar): ~2-3 gün. Faz 4–5: ~1-2 hafta.
- Bu iş kritik yoldadır (kapalı çevrim takip plant yüzünden çalışmıyor);
  alternatifi iris LiftDrag/`cda`/`cp` retune cerrahisidir ve o da kullanıcı
  yönlendirmesi ister. SimITL işi ayrı checkout'ta yürür, `~/betaflight` ve
  `~/aeroloop_gazebo` değiştirilmez.
- Canlı koşu hijyeni (AGENT-STATUS): önce `pkill -9 -f "gz sim|
  betaflight_SITL|run_gazebo_betaflight|moving_target|track_truth"` ve
  `ss -tlnp | grep -E "5761|900[234]"` boş olmalı; 5761'de takılı Betaflight
  sessizce boş log üretir.
- Her kanıt satırı dünya etiketi (`world=simitl` / `gazebo-plant` /
  `sync-0.0025`) ve adımı birlikte yazar; roadmap-v2 çalışma kuralları
  (INVALID tanımı, 5/5 kabul, eşik değişikliği ayrı commit) aynen geçerli.
  Eski "A4 PASS" sınıfı iddialar geçersizdir; hiçbir yeni kapı heading/
  roll-pitch-only ölçümle "takip çalışıyor" diyemez.
