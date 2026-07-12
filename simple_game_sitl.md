# Simple Game SITL - Screen Capture Visual Tracking Sandbox

Tarih: 2026-07-04

Bu dokuman, Steam/oyun/simulator ekrani uzerinden Kenet tracker + PID
gelistirmek icin basit ve izole bir deneme ortami planidir.

Buradaki "SITL" kelimesi klasik Betaflight SITL anlaminda degildir. Bu bir
"game/screen-in-the-loop" sandbox'tir:

```text
Oyun / FPV sim ekrani
  -> ekran yakalama
  -> Kenet tracker
  -> bbox merkezi / hedef boyutu
  -> PID
  -> virtual joystick / keyboard input
  -> oyun icindeki drone
  -> yeni ekran goruntusu
```

## Temel Karar

Bu sandbox'in amaci Gazebo/Betaflight sorununu cozmek degildir. Amac, gorsel
takip ve PID dongusunu daha hizli ve daha az kirilgan bir ortamda denemektir.

Bu deney iki izole hatta paralel yurur:

```text
experiments/game_screen_sandbox:
  tracker + PID + gecikmeli gorsel kapali cevrim davranisi

experiments/simitl_pr0p_probe:
  pr0p/SimITL kurulumu, pencere/capture, MSP websocket, input mapping ve
  pr0p uzerinde canli yuzeylerin safe probe kaniti
```

Gazebo / Betaflight hatti bu dokumanin disinda kalir ve varsayilan launcher
ve acceptance kapilarina baglanmaz.

Simple game sandbox'ta iyi calisan PID sayilari gercek drone'a aynen
kopyalanmaz. Kalici deger tasiyan ciktılar sunlardir:

- PID yapisi: deadband, rate limit, anti-windup, smoothing.
- Target-loss ve re-entry davranisi.
- Gecikmeye dayaniklilik.
- Basit hedef merkezleme ve yaklasma davranisi.

## Ne Kanitlar / Ne Kanitlamaz

Kanitlar:

- Kenet'in oyun/sim goruntusunden frame alabildigini.
- Tracker'in hedefi bulup takip ettigini.
- PID'in hedefi merkeze cekme davranisini.
- Komutlarin oyunda gorsel sonuca donusup donusmedigini.
- Ucta-uca gecikme altinda kapali cevrimin stabil kalip kalmadigini.

Kanitlamaz:

- Betaflight arm/disarm dogrulugunu.
- Gercek motor/ESC/pervane davranisini.
- Gazebo-Betaflight entegrasyonunun saglam oldugunu.
- Gercek drone PID/rate/filter tune'unu.
- MSP Override veya fiziksel FC protokolunun hatasiz oldugunu.
- Gazebo'daki surekli-yaw spin kok nedeninin cozuldugunu.

Bu yuzden bu sandbox, Gazebo/Betaflight acceptance kapilarinin yerine gecmez.

## Izolasyon Kurallari

- Ana Kenet/Gazebo/Betaflight launcher'lari degistirilmez.
- Oyun binary'leri, Steam dosyalari, kayitlar veya ekran goruntuleri commit
  edilmez.
- Deney kodu varsayilan runtime'a import edilmez.
- Screen-loop deney kodu su dizin altinda tutulur:

```text
experiments/game_screen_sandbox/
```

- pr0p/SimITL kurulum ve live-surface probe kodu su dizin altinda tutulur:

```text
experiments/simitl_pr0p_probe/
```

- Runtime loglari su dizinde tutulur:

```text
logs/game_screen_sandbox/
logs/simitl_pr0p/
```

Tek bakis icin son game-screen karari ve son pr0p safe-suite kanitini okuyan
read-only rapor:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/independent_sim_readiness.py \
  --run-id independent-sim
```

Bu komut sim, Gazebo, Betaflight, capture veya `uinput` baslatmaz; sadece hangi
hatta hangi kanitin eksik oldugunu raporlar. Varsayilan olarak 24 saatten eski
kanitlar `STALE` sayilir; gerekirse `--max-evidence-age-s` ile replay amacli
eski log okunabilir.

- Online/multiplayer/anti-cheat iceren modlarda otomasyon yapilmaz. Sadece
  offline/free-flight/local sandbox kullanilir.
- Deney reddedilirse `experiments/game_screen_sandbox/` ve ilgili log dizini
  silinerek ana repo eski davranisini korur.

## Oyun Secim Kriterleri

Baslangicta oyun/sim secimi ayri bir fazdir. Herhangi bir oyuna kod yazmadan
once su kriterler kontrol edilir:

- Offline/free-flight veya local test modu var.
- Gamepad/joystick input destekliyor.
- Pencere yakalama stabil.
- Mumkunse self-level / ANGLE / horizon benzeri mod var.
- OSD/HUD kapatilabiliyor veya hedef secimini bozmayacak kadar sade.
- Anti-cheat/multiplayer zorunlu degil.

Ilk aday sirasi:

1. `pr0p`: ucretsiz, SimITL/Betaflight baglantisi olasiligi nedeniyle en
   yuksek transfer degeri.
2. `SITL Forge`: hazir fizik ve Betaflight DLC opsiyonu nedeniyle guclu aday.
3. Liftoff/Uncrashed gibi FPV oyunlari: zengin sahne var, ama acro agirlikli
   olduklari icin otomatik yaw-only merkezleme daha zor olabilir.

Acro-only bir oyunda sabit throttle + yaw-only dongu ayakta kalmayabilir. Bu
durumda oyun elenir veya yalniz tracker smoke icin kullanilir.

## Hedef Mimari

```text
GameWindowCapture
  -> frame
  -> ObjectTracker
  -> TrackResult bbox/center/found
  -> FlightController PID
  -> VirtualInputAdapter
  -> game joystick/keyboard axes
```

Adapter'lar:

- `GameWindowCapture`: oyun penceresinden frame alir.
- `TrackingLoop`: frame -> tracker -> PID -> command dongusunu yonetir.
- `VirtualInputAdapter`: PID cikisini joystick/keyboard komutuna cevirir.
- `GameSandboxLogger`: frame timing, bbox, PID ve input loglarini JSONL yazar.

Kenet tarafindan yeniden kullanilacak parcalar:

- `kenet.tracker.ObjectTracker`
- `kenet.controller.FlightController`
- `kenet.tracker.TrackResult`
- `PipelineConfig` / CLI'dan gelen PID gain degerleri
- Uretimdeki target-loss / re-entry semantigi
- Mumkunse mevcut `tools/sitl_log.py` icindeki `JsonlLogger`

## Ilk Klasor Yapisi

```text
experiments/game_screen_sandbox/
  PLAN.md
  capture_window.py
  x11_window.py
  virtual_input.py
  screen_tracking_loop.py
  simple_target_game.py
  README.md
```

## Repo-Ici Basit Hedef Penceresi

Gazebo/Betaflight disinda kalmak icin sandbox icinde cok kucuk bir OpenCV
penceresi vardir:

```text
experiments/game_screen_sandbox/simple_target_game.py
```

Amac:

- Harici oyun kurmadan hareketli hedefli bir pencere uretmek.
- `x11grab`/window-title capture hattini gercek ekrandan test etmek.
- Tracker bbox secimini ve S4/S6/S7 dry-run kapilarini Chrome gibi rastgele
  pencerelere bagli kalmadan denemek.

Kanitlamaz:

- Drone fizigini.
- Gercek oyun/FPV simulator input tepkisini.
- Betaflight/Gazebo davranisini.

Headless smoke:

```bash
fpv_env/bin/python experiments/game_screen_sandbox/simple_target_game.py \
  --duration 2 \
  --fps 10 \
  --static-target \
  --save-frame logs/game_screen_sandbox/simple-game-frame.png \
  --report-path logs/game_screen_sandbox/simple-game-report.json
```

Pencere smoke:

Tek komut otomatik smoke:

```bash
fpv_env/bin/python experiments/game_screen_sandbox/simple_window_smoke.py \
  --run-id sandbox-simple-window
```

Bu komut hedef penceresini acar, 640x480 X11 icerik region'ini secer,
ffmpeg/x11grab ile frame alir ve S7 combined dry-run kapisini kosar.

Tek komut acceptance kosusu:

```bash
fpv_env/bin/python experiments/game_screen_sandbox/sandbox_acceptance_runner.py \
  --run-id sandbox-acceptance \
  --include-simple-window
```

Bu komut S0-S14 sentetik kapilarini, opsiyonel repo-local simple-window
smoke'unu ve final decision raporunu birlikte uretir. `--include-simple-window`
verilmezse sentetik kanit yine uretilir ama real-window kaniti eksik oldugu icin
decision `WAITING` kalir.

Izolasyon audit'i:

```bash
fpv_env/bin/python experiments/game_screen_sandbox/isolation_audit.py --json
```

Acceptance runner raporu `isolation_status` alanini yazar. Sandbox Python
import'lari Gazebo/Betaflight modulune baglanirsa veya kosu sirasinda yeni bir
Gazebo/Betaflight sureci baslarsa acceptance `REJECT` olur.

Manuel smoke:

```bash
fpv_env/bin/python experiments/game_screen_sandbox/simple_target_game.py \
  --show \
  --duration 60 \
  --fps 30 \
  --static-target \
  --report-path logs/game_screen_sandbox/simple-game-window-report.json
```

Bu kosunun JSON raporundaki `first_bbox`, sabit hedef smoke icin
`phase_runner.py --tracker-bbox` degeri olarak kullanilir. Hareketli hedef
davranisi ise S8 kapisinda ayrica olculur.

Harici oyun/sim pencere preflight:

```bash
fpv_env/bin/python experiments/game_screen_sandbox/external_window_preflight.py \
  --capture-window-title pr0p \
  --capture-backend ffmpeg \
  --tracker-bbox-file logs/game_screen_sandbox/pr0p-target-bbox.json \
  --run-id pr0p-window-preflight
```

Sonuc `EXTERNAL_WINDOW_READY_DRY`, `WAITING` veya `REJECT` olur. Bu komut
Betaflight/Gazebo hazirligini kanitlamaz; yalniz secilen pencerenin
capture/tracker/PID dry-run gelistirmesine uygunlugunu olcer.

Dry preflight PASS olduktan sonra ayni entrypoint ile RC channel/axis binding
kapisi da acilir:

```bash
fpv_env/bin/python experiments/game_screen_sandbox/external_window_preflight.py \
  --capture-window-title pr0p \
  --capture-backend ffmpeg \
  --tracker-bbox-file logs/game_screen_sandbox/pr0p-target-bbox.json \
  --include-axis-sweep \
  --ack-live-input \
  --axis-sweep-axes yaw,pitch \
  --run-id pr0p-window-preflight-axis
```

Bu kosu `EXTERNAL_WINDOW_READY_LIVE_INPUT` seviyesine yalniz S5-real de PASS
olursa cikar. S1/S2/S7 PASS, S5-real WAITING ise once oyun/sim icindeki
`Controls -> RC Channels` veya axis binding ayarlari kontrol edilir.

Harici pencere uzerinde gercek takip session dry-run:

```bash
fpv_env/bin/python experiments/game_screen_sandbox/external_dry_run_sequence.py \
  --capture-window-title pr0p \
  --capture-backend ffmpeg \
  --tracker-bbox-file logs/game_screen_sandbox/pr0p-target-bbox.json \
  --duration 2 \
  --hz 10 \
  --enable-pitch \
  --desired-target-width 120 \
  --run-id pr0p-dry-sequence
```

Bu sequence status -> preflight -> follow dry-run siralar ve `uinput` basmaz.
Rapor icinde `sequence_steps` ve `live_readiness` alanlari vardir; basarili
olursa `EXTERNAL_DRY_RUN_READY` ve `READY_FOR_OPTIONAL_LIVE_INPUT` yazar.
Eger pencere basligi degisken ise `external_operator_preflight.py`
`window_discovery` adaylarinda `dry_run_sequence_region_command` yazar. Bu komut
`--capture-region` kullandigi icin dogru crop secildikten sonra `target_window`
gate'ine takilmadan bbox ve process-boundary gate'leriyle dry-run dener.
Ayni raporda `candidate_action_plan` da vardir: bbox yoksa aday icin once
`capture_bbox_frame_region`, bbox hazirsa `dry_run_sequence_region` onerir. Bu
oneriler input basmaz; operator dogru sim/FPV goruntusu oldugunu elle teyit
etmeden calistirilmaz. VS Code gibi tooling/editor pencereleri
`excluded_candidates` altinda raporlanir ve `candidate_action_plan` icine
alinmaz.
Bu adimi elle kopyalamak yerine `external_candidate_action_runner.py` plan-only
calistirilabilir. Aday secilmezse sadece `WAITING` raporu yazar; dogru aday
teyit edilirse `--candidate-window-id 0x... --ack-candidate --execute-safe`
ile yalniz o aday icin tek guvenli non-`uinput` komutu calistirir.
`--candidate-index`, exact `--candidate-title` ve tekil
`--candidate-title-contains` de desteklenir; belirsiz substring secimi
reddedilir. VS Code gibi tooling/editor pencereleri de baslikta simulator metni
gecse bile reddedilir; dogru sim/FPV penceresi gozle teyit edildikten sonra
son candidate planindaki `--candidate-window-id` tercih edilmelidir.
Frame incelenip bbox secildikten sonra ayni seciciyle
`--candidate-bbox X,Y,W,H` verilebilir; runner secili region crop uzerinden
bbox dosyasini yazar ve pencere basligina geri donmez. Bbox koordinatlari secili
region icine gore verilir; region disina tasarsa runner komut calistirmadan
reddeder.
Komut gercekten calisirsa runner sonrasinda ikinci bir operator preflight raporu
yazar; boylece secilen region capture/dry-run sonrasi gate durumu ayni artefakt
icinden gorulur.

Dry sequence gectikten sonra canli virtual-RC icin ayri readiness kapisi:

```bash
fpv_env/bin/python experiments/game_screen_sandbox/external_live_input_readiness.py \
  --capture-window-title pr0p \
  --capture-backend ffmpeg \
  --tracker-bbox-file logs/game_screen_sandbox/pr0p-target-bbox.json \
  --duration 2 \
  --hz 10 \
  --enable-pitch \
  --desired-target-width 120 \
  --ack-live-input \
  --axis-sweep-axes yaw,pitch \
  --run-id pr0p-live-input-readiness
```

Bu gate once dry sequence'i tekrar kosar. Ancak `--ack-live-input` varsa S5-real
axis-response icin virtual RC gonderir. Basarili olursa
`EXTERNAL_LIVE_INPUT_READY` yazar; ack yoksa `WAITING` yazar ve input basmaz.
Status tarafinda bu ust seviye sonuc tek basina yeterli sayilmaz. Matching
raporda gomulu S5-real `UInputAdapter`, `real_input: true`, `axis_sweep: true`
ve istenmisse ayni signed-axis beklentisi bulunmalidir; aksi halde status
`WAITING` kalir.

Canli takip icin dogrudan follow session'a atlamak yerine bounded sequence:

```bash
fpv_env/bin/python experiments/game_screen_sandbox/external_live_follow_sequence.py \
  --capture-window-title pr0p \
  --capture-backend ffmpeg \
  --tracker-bbox-file logs/game_screen_sandbox/pr0p-target-bbox.json \
  --duration 1 \
  --hz 10 \
  --enable-pitch \
  --desired-target-width 120 \
  --ack-live-input \
  --live-max-duration 2 \
  --run-id pr0p-live-follow-sequence
```

Bu sequence once live-input readiness'i tekrar kosar, sonra kisa sureli live
follow baslatir. Basarili olursa `EXTERNAL_LIVE_FOLLOW_COMPLETE` yazar.

Sonrasinda status-only resume komutu live-input kanitini de zorunlu sayabilir:

```bash
fpv_env/bin/python experiments/game_screen_sandbox/sandbox_status_report.py \
  --window-title pr0p \
  --bbox-file logs/game_screen_sandbox/pr0p-target-bbox.json \
  --require-live-input
```

Status raporu mevcut `--bbox-file` icindeki bbox ile raporlardaki bbox'i
eslestirir. Hedef yeniden secilirse eski preflight/follow/readiness kanitlari
ayni pencere basliginda olsa bile `READY` sayilmaz.
Status raporu ayrica `resume_commands` alaninda siradaki komutlari yazar.
Her komutta `safety_class`, `expected_status` ve `unblocks` alanlari bulunur;
virtual RC gonderebilecek komutlarda `sends_uinput: true` gorunur.
BBox eksikse resume listesinde iki secim yolu bulunur: `--bbox X,Y,W,H` ile
manuel koordinat yazma veya `--interactive-select` ile OpenCV ROI UI'dan secme.
Interaktif komut `requires_user_interaction: true` tasir ve runner tarafindan
otomatik acilmaz.
Komutlar ayrica `requires_pass` listesi tasir. Runner bu listedeki status
item'lari PASS olmadan komutu calistirmaz; ornegin dry sequence icin pencere ve
bbox, live follow icin de live-input readiness kaniti gerekir.

Devam otomasyonu icin `sandbox_resume_runner.py` kullanilir. Varsayilan mod
plan-only'dir; capture veya input calistirmaz:

```bash
fpv_env/bin/python experiments/game_screen_sandbox/sandbox_resume_runner.py \
  --window-title pr0p \
  --bbox-file logs/game_screen_sandbox/pr0p-target-bbox.json \
  --require-live-follow
```

`--execute-safe` sadece `sends_uinput: false` ve edit gerektirmeyen komutlari
calistirir. Virtual RC komutlari icin ayrica `--execute-live-input` ve
`--ack-live-input` gerekir. Bir komut calisirsa runner post-status raporu da
yazar; istenen gate artik hazirsa sonucu ready seviyesine yukseltir.
Manuel UI isteyen komutlar, ornegin `interactive_bbox_select`, execute-safe
modunda da `BLOCKED` kalir; pr0p penceresi gorunurken terminalden bilerek
calistirilmalidir.
`--execute-live-input --ack-live-input` verilse bile dependency eksikse
komutlar `BLOCKED` kalir ve `unmet_requires_pass` alaninda hangi gate'in eksik
oldugu gorulur.
Runner JSON/Markdown raporundaki `resume_decision`, ilk operator aksiyonunu ve
ilk eksik dependency'yi Steps tablosunu okumadan gosterir.
Runner raporundaki `resume_step_summary`, adim status sayilarini,
`sends_uinput` komut sayisini, calisan komut sayisini ve ilk planned/blocked/
unmet/failure adimini yazar.
Status raporundaki `readiness_ladder`, S0-S5 asama durumunu verir:
local sandbox izolasyonu, hedef kurulumu, dry perception/control, control
binding kaniti, live-input readiness ve bounded live follow.
Status raporundaki `evidence_freshness`, eslesen kanit dosyalarinin yasini ve
`FRESH/STALE/MISSING` durumunu yazar; bu eski raporlarin sessizce guncel kanit
gibi okunmasini engeller.
Bu kontrol varsayilan olarak bilgilendiricidir. Eski veya eksik zorunlu kanit
`READY` sonucunu engellesin istenirse status veya resume komutuna
`--require-fresh-evidence --evidence-stale-after-s 3600` eklenir.

Gercek pr0p/SITL Forge denemesinden hemen once tek bakislik operator hazirlik
raporu icin:

```bash
fpv_env/bin/python experiments/game_screen_sandbox/external_operator_preflight.py \
  --window-title pr0p \
  --bbox-file logs/game_screen_sandbox/pr0p-target-bbox.json \
  --require-live-follow \
  --require-fresh-evidence \
  --evidence-stale-after-s 3600 \
  --axis-expected-shifts yaw:+x,pitch:-y \
  --axis-min-shift-px 1 \
  --max-window-candidates 5
```

Bu komut simulator, capture veya `uinput` baslatmaz; yalnizca
`operator_preflight_decision`, komut state sayilari ve onerilen siradaki komutu
yazar. Pencere basligi eslesmezse `window_discovery` altinda gorunen X11
uygulama penceresi adaylarini da listeler. Her aday icin exact rerun ve
bbox-frame capture komutlari da rapora yazilir; ayrica basliktan bagimsiz
`--region` tabanli bbox-frame komutu da bulunur.

Gercek pr0p/SITL Forge penceresi icin son kabul karari
`external_target_acceptance.py` ile okunur. Bu komut varsayilan olarak
`--mode live-follow` ister, fresh evidence'i zorunlu sayar ve simulator,
capture veya `uinput` baslatmadan `EXTERNAL_TARGET_READY`, `WAITING` ya da
`REJECT` yazar. Ayrica before/after Gazebo/Betaflight process-boundary audit'i
ve ilk blokaj / siradaki operator aksiyonu icin `acceptance_decision` ozetini
rapora ekler. Siradaki komutlar da `operator_command_queue` icinde
`safety_class`, `sends_uinput`, `requires_pass`, `command_state`,
`unmet_requires_pass` ve exact command ile gorunur. `operator_command_summary`
state sayilarini ve ilk available/blocked/ack-required komutu ozetler:

```bash
fpv_env/bin/python experiments/game_screen_sandbox/external_target_acceptance.py \
  --window-title pr0p \
  --bbox-file logs/game_screen_sandbox/pr0p-target-bbox.json \
  --mode live-follow \
  --axis-expected-shifts yaw:+x,pitch:-y \
  --axis-min-shift-px 1 \
  --evidence-stale-after-s 3600
```

`bbox_tool.py` istenen pencereyi bulamazsa traceback yerine `WAITING` raporu
doner; bu durumda once oyun/sim penceresi acilmali veya `--window-title`
duzeltilmelidir.

Dry evidence hazir ama live-input response eksikse `resume_commands` icinde
`rc_binding_assistant.py` de gorunur. Bu, oyun/sim `Controls -> RC Channels`
sayfasinda yaw/pitch/roll/throttle eksenlerini bind etmek icindir ve virtual RC
gonderdigi icin yine `--execute-live-input --ack-live-input` olmadan calismaz.
Dry-run binding raporu RC channel binding kaniti sayilmaz; status icin mevcut
pencere ve bbox ile eslesen, acknowledged live `uinput` ve neutralize edilmis
binding raporu gerekir.

`EXTERNAL_LIVE_FOLLOW_COMPLETE` sonrasi en guclu status kapisi:

```bash
fpv_env/bin/python experiments/game_screen_sandbox/sandbox_status_report.py \
  --window-title pr0p \
  --bbox-file logs/game_screen_sandbox/pr0p-target-bbox.json \
  --require-live-follow
```

```bash
fpv_env/bin/python experiments/game_screen_sandbox/external_follow_session.py \
  --capture-window-title pr0p \
  --capture-backend ffmpeg \
  --tracker-bbox-file logs/game_screen_sandbox/pr0p-target-bbox.json \
  --duration 2 \
  --hz 10 \
  --enable-pitch \
  --desired-target-width 120 \
  --run-id pr0p-follow-dry
```

Bu kosu basarili olursa `EXTERNAL_FOLLOW_DRY_RUN_READY` yazar. Gercek virtual
RC basmak icin ayrica `--uinput --ack-live-input --live-max-duration 2`
gerekir; aksi halde session `WAITING` doner.

OpenCV ayni baslikla hem dekorasyonlu dis pencere hem de 640x480 icerik
penceresi olusturabilir. Bu durumda capture region su sekilde secilir:

```bash
fpv_env/bin/python experiments/game_screen_sandbox/x11_window.py \
  --title "Kenet Simple Target Game" \
  --preferred-size 640x480
```

## Faz S0 - Ortam ve Oyun Secimi

Amac: hangi oyunla baslanacagini ve makinenin capture/input icin uygun olup
olmadigini belirlemek.

Kontroller:

```bash
echo "$XDG_SESSION_TYPE"
echo "$DISPLAY"
fpv_env/bin/python -m pytest -q
```

Bu makinede 2026-07-04 itibariyle gozlenen durum:

- `XDG_SESSION_TYPE=x11`
- `DISPLAY=:1`
- `ffmpeg` mevcut
- `mss` ve `evdev` fpv_env icinde kurulu degil
- `/dev/uinput` mevcut, ancak izin/modul/udev durumu ayrica cozulmeli

Kabul:

- Ilk oyun adayi secildi.
- Oyun offline/local modda aciliyor.
- Pencere gorunuyor.
- Capture yolu belirlendi: `mss`, `ffmpeg x11grab`, OpenCV, veya gerekirse
  v4l2loopback.
- Input yolu belirlendi: `uinput` virtual gamepad, fiziksel gamepad, veya
  sadece manuel smoke icin keyboard.

Not:

- `uinput` otomasyon icin idealdir, ama ilk smoke icin fiziksel gamepad veya
  klavye yeterli olabilir.
- v4l2loopback iyi bir alternatif olabilir: oyun penceresi sanal kamera olur,
  Kenet mevcut `--camera N` yolu ile okuyabilir.

## Faz S1 - Ekran Yakalama Smoke

Amac: oyun penceresinden stabil frame almak.

Arac:

```text
experiments/game_screen_sandbox/capture_window.py
experiments/game_screen_sandbox/x11_window.py
experiments/game_screen_sandbox/bbox_tool.py
```

Kabul:

- 30 saniye boyunca en az 20 FPS.
- Frame nonblank.
- Crop/viewport sabit.
- Frame boyutu loglanir.
- Ornek frame kaydedilir.

Mevcut uygulama notu:

- X11 ortaminda `x11_window.py --list` ile pencere basliklari listelenebilir.
- `capture_window.py --window-title <baslik> --backend ffmpeg` ile elle crop
  yazmadan pencere goruntusu alinabilir.
- `bbox_tool.py --window-title <baslik>` hedef bbox secimi icin tek frame
  kaydeder; `--bbox x,y,w,h` ile overlay ve rapor uretir. Pencere bilerek
  acikken `--interactive-select` ile bbox OpenCV ROI UI uzerinden secilebilir.
- `external_window_preflight.py --capture-window-title <baslik>
  --tracker-bbox-file <bbox.json>` secilen oyun/sim penceresi icin S1-real,
  S2-real ve S7-real dry-run hazirligini tek raporda verir.

Pratik karar:

- Tracker icin ilk hedef 640x480 veya benzer dusuk/cozumlenebilir boyuttur.
  1080p tam pencere CSRT icin gereksiz CPU maliyeti getirebilir.
- OSD/HUD hedef bbox'una girerse oyun ayarlarindan kapatilir veya hedef
  OSD'siz bolgeden secilir.

Fail nedenleri:

- `NO_WINDOW`
- `BLACK_FRAME`
- `FPS_TOO_LOW`
- `CROP_UNSTABLE`

## Faz S2 - Tracker Smoke

Amac: oyun goruntusundeki hedefin tracker tarafindan takip edildigini gormek.

Akis:

1. Oyun/sim free-flight modunda acilir.
2. Kullanici hedef uzerinde bbox secer.
3. Tracker init edilir.
4. 30 saniye boyunca sadece takip edilir, input gonderilmez.

Kabul:

- `found_ratio >= 0.90`
- Ortalama merkez hatasi raporlu.
- Bbox boyutu ve merkez izi JSONL'e yazilir.
- Hedef kaybi olursa zaman ve neden loglanir.

Bu fazda PID input yoktur. Sadece "goruntu -> tracker" kanitlanir.

Mevcut uygulama notu:

- `phase_runner.py --capture-window-title <baslik> --tracker-bbox x,y,w,h`
  ile gercek pencere goruntusunun tracker'a verildigi S2-real kapisi
  olculebilir.
- `bbox_tool.py` ile secilen bbox'in frame icinde oldugu S2-real oncesinde
  dogrulanabilir.
- `phase_runner.py --tracker-bbox-file ...` ile bbox_tool'un JSON ciktisi
  dogrudan S2-real/S4-real kapilarinda kullanilabilir.

## Faz S3 - Virtual Input Smoke

Amac: oyuna kontrollu joystick/keyboard komutu basabilmek.

Arac:

```text
experiments/game_screen_sandbox/virtual_input.py
```

Ilk komutlar:

- neutral
- small yaw left/right
- small pitch forward/back
- throttle sabit veya oyun icindeki hover hizina gore manuel

Kabul:

- Oyun input/ayar menusunde sanal pad veya secilen input gorunur.
- Neutral input drone'u belirgin sekilde saptirmiyor.
- Kucuk yaw komutu goruntuyu beklenen yonde hareket ettiriyor.
- Kucuk pitch komutu goruntude ileri/geri hareket etkisi yaratiyor.
- Input stop edilince neutral'a donuyor.

Minimum guvenlik:

- Her kosuda manuel veya programatik neutral-stop yolu olur.
- Bu asamada tam runaway guard zorunlu degildir; sadece input'un guvenli sekilde
  notrlenebildigi kanitlanir.

Fail nedenleri:

- `INPUT_NOT_DETECTED`
- `AXIS_ORDER_UNKNOWN`
- `NO_SAFE_NEUTRAL`
- `COMMAND_TOO_AGGRESSIVE`

## Faz S4 - Yaw-only Minimal Kapali Cevrim

Amac: ilk gercek visual-servo dongusunu en basit haliyle denemek.

Akis:

```text
capture frame
  -> tracker bbox
  -> horizontal error
  -> yaw PID
  -> virtual joystick yaw
```

Kapsam:

- Sadece yaw.
- Pitch, roll, throttle sabit/manual.
- Dusuk PID limitleri.
- Hedef once yavas veya statik olur.
- Minimum neutral-stop vardir.

Kabul:

- 30-60 saniye kosu.
- `found_ratio >= 0.85`
- Center error RMS raporlu.
- P95 horizontal error raporlu.
- PID saturation suresi raporlu.
- Komutlar bounded.
- Oyun kontrolden cikmiyor.

Bu fazda hedef, "en basit haliyle calisiyor mu?" sorusudur. Runaway guard bu
faz icin zorunlu baslangic sarti degildir. Eger saturation/runaway belirtisi
gorulurse Faz S4b devreye girer.

Mevcut uygulama notu:

- `phase_runner.py --include-real-loop --tracker-bbox x,y,w,h` gercek pencere
  frame'leri uzerinde tracker -> PID -> komut uretimini dry-run olarak olcer.
- Bu S4-real kapisi oyuna joystick komutu basmaz; gercek input ile kapali
  cevrim bir sonraki manuel guvenlik adimidir.
- `phase_runner.py --include-live-loop --ack-live-input` sadece S4-real
  basarili olduktan sonra denenir; sure limiti vardir ve gercek uinput komutu
  basar.

## Faz S4b - Yaw-only Sertlestirme

Bu faz yalniz S4'te sorun gorulurse uygulanir.

Tetikleyiciler:

- PID komutu uzun sure saturation'da kaliyor.
- Hata azalmak yerine artiyor.
- Oyun goruntusu hizla donuyor veya drone kontrolden cikiyor.
- Neutral-stop insan refleksine kalacak kadar gec kaliyor.

Eklenecekler:

- Runaway guard: komut N saniyeden uzun sature ve `abs(error)` artiyorsa
  otomatik FAIL + neutral.
- Tek tus veya sinyal ile kill-switch: tum eksenler neutral, dongu durur.
- Daha dusuk yaw limit.
- Daha buyuk deadband veya daha yumusak rate limit.
- Ucta-uca gecikme olcumu: yaw step komutundan goruntudeki hareketin basladigi
  kareye kadar gecen sure.

Kabul:

- Ayni S4 senaryosu guard ile 60 saniye kontrolden cikmadan kosar.
- Guard tetiklenirse bu PASS degil, acik FAIL kabul edilir; ama guvenli kapanma
  dogrulanmis olur.

## Faz S5 - Ucta Uca Gecikme Olcumu

Amac: capture -> tracker -> PID -> input -> render dongusunun gecikmesini
olcmek.

Bu faz S4 calistiktan sonra kosulur. S4 iyi calisiyorsa bu sadece metrik olur;
S4 kararsizsa S4b ayarlarinin girdisi olur.

Olcum:

1. Sabit hedef secilir.
2. Kucuk yaw step komutu verilir.
3. Komut timestamp'i loglanir.
4. Goruntude yatay hareketin basladigi ilk frame bulunur.
5. Gecikme ms cinsinden raporlanir.

Kabul:

- Gecikme olculebilir.
- Frame/input timestamp'leri ayni logda iliskilendirilebilir.
- Gecikme PID tuning notuna yazilir.

Mevcut uygulama notu:

- `latency_probe.py` komut zamanindan goruntudeki ilk anlamli frame farkina
  kadar gecen sureyi olcer.
- `phase_runner.py --include-latency-probe` S5-real kapisini ekler.
- `--latency-axis-sweep --latency-axes yaw,pitch,roll,throttle` secilen
  virtual RC eksenlerini tek tek dener ve `axis_statuses`, `passed_axes`,
  `waiting_axes` metriklerini raporlar.
- Eger eksen tepki veriyor ama yaw/pitch yonunun ters olma riski varsa
  `--latency-axis-expected-shifts yaw:+x,pitch:-y` ile signed direction
  kontrolu acilir. Bu kontrol `response_shift_x_px`, `response_shift_y_px` ve
  `direction_status` metriklerini yazar; ters yon FAIL olur.
- Gercek input ile olcum yapmak icin `--latency-uinput --ack-live-input`
  gerekir; tepki gorulmezse PASS yerine WAITING raporlanir.
- S3-real PASS oldugu halde axis sweep WAITING ise once oyun/sim icindeki
  `Controls -> RC Channels` veya axis binding ayarlari kontrol edilir.
- Binding icin once oyun/sim `Controls -> RC Channels` sayfasinda ilgili satir
  secilir, sonra tek eksen su aracla oynatilir:

```bash
fpv_env/bin/python experiments/game_screen_sandbox/rc_binding_assistant.py \
  --window-title pr0p \
  --tracker-bbox-file logs/game_screen_sandbox/pr0p-target-bbox.json \
  --axes yaw \
  --axis-value 0.6 \
  --hold-seconds 0.8 \
  --neutral-seconds 0.3 \
  --uinput \
  --ack-live-input \
  --run-id pr0p-bind-yaw
```

- Ayni komut `--axes pitch`, `--axes roll`, `--axes throttle` icin tekrarlanir.

Signed direction kontrolu canli readiness zincirinde de kullanilabilir:

```bash
fpv_env/bin/python experiments/game_screen_sandbox/external_live_input_readiness.py \
  --capture-window-title pr0p \
  --capture-backend ffmpeg \
  --tracker-bbox-file logs/game_screen_sandbox/pr0p-target-bbox.json \
  --duration 2 \
  --hz 10 \
  --enable-pitch \
  --ack-live-input \
  --axis-sweep-axes yaw,pitch \
  --axis-expected-shifts yaw:+x,pitch:-y \
  --axis-min-shift-px 1 \
  --run-id pr0p-live-input-readiness-signed
```

Ayni signed direction beklentisi status/resume komutlarinda da tasinabilir:

```bash
fpv_env/bin/python experiments/game_screen_sandbox/sandbox_resume_runner.py \
  --window-title pr0p \
  --bbox-file logs/game_screen_sandbox/pr0p-target-bbox.json \
  --require-live-follow \
  --axis-expected-shifts yaw:+x,pitch:-y \
  --axis-min-shift-px 1
```

Bu komut plan-only modda input basmaz; sadece sonraki live-input/follow
komutlarinin ayni signed-axis beklentisiyle uretilecegini raporlar.

## Faz S6 - Pitch / Approach Basit Deneme

Amac: hedef boyutunu mesafe vekili gibi kullanip yaklasma davranisini ilk kez
denemek.

Akis:

```text
bbox width error = desired_width - current_width
  -> pitch PID
  -> virtual joystick pitch
```

Ilk dry-run kabul:

- Tracker hedefi kaybetmez.
- Width error `desired_target_width` degerine gore raporlanir.
- Pitch komutu saturation yapmaz.
- Input dongu sonunda neutral'a doner.

Ilk canli/game-response kabul:

- Bbox width hedef degere yaklasma egilimi gosterir.
- Yaw center error belirgin bozulmaz.
- Pitch komutu bounded kalir.
- Hedef cok buyurse pitch neutral veya retreat davranisi icin not alinir.

Mevcut uygulama notu:

- `phase_runner.py --include-real-approach` S6-real kapisini ekler.
- Bu kapı ilk asamada dry-run calisir: gercek oyuna joystick komutu basmaz,
  ancak tracker bbox width -> forward PID -> bounded pitch komutu zincirini
  olcer.
- Rapor; `desired_target_width`, ilk/son bbox width, width error, pitch limit,
  `max_abs_pitch_axis`, `found_ratio` ve neutralizasyon metriklerini yazar.
- Bbox yoksa WAITING, bbox frame disindaysa FAIL, komut bounded ve tracker
  stabilse PASS verir.

Ornek:

```bash
fpv_env/bin/python experiments/game_screen_sandbox/phase_runner.py \
  --capture-window-title pr0p \
  --capture-backend ffmpeg \
  --tracker-bbox-file logs/game_screen_sandbox/pr0p-target-bbox.json \
  --include-real-approach \
  --desired-target-width 120 \
  --duration 2 \
  --hz 10 \
  --run-id sandbox-pr0p-approach-dry
```

Onemli ayrim:

- Bu fazda bbox width tek metrik olarak denenebilir. Eger basit ortamda
  sorunsuz calisirsa ek metrik zorunlu degildir.
- Eger bbox width gurultulu, yaniltici veya yaklasmayi aciklamayan bir sinyal
  gibi davranirsa Faz S6b devreye girer.

## Faz S6b - Approach Metrik Sertlestirme

Bu faz yalniz S6'da sorun gorulurse uygulanir.

Tetikleyiciler:

- Bbox width hedefe yaklasmadan da buyuyor/kuculuyor.
- Hedef donusu, isik, OSD veya tracker ziplama genislik sinyalini bozuyor.
- Pitch komutu var ama goruntude yaklasma hissi yok.
- Hedef cok buyuyunce retreat/neutral kuralinin belirsiz oldugu goruluyor.

Eklenecek ikinci sinyaller:

- Optical flow: zemin/kenar dokusu ileri akiyor mu?
- Oyunun hiz/telemetri gostergesi varsa ekran/OCR veya log ile hiz artiyor mu?
- pr0p/SimITL veya SITL Forge telemetri sunuyorsa forward velocity okunabiliyor
  mu?
- Hedefe gore frame icindeki scale trendi median/LPF ile daha stabil hale
  geliyor mu?

Kabul:

- Approach karari yalniz ham bbox width'e bagli kalmaz.
- Too-close durumunda neutral veya retreat kuralı yazilir.
- Bu kural daha sonra Kenet uretim controller kararlarina aday olarak tasinir.

## Faz S7 - Combined Yaw + Pitch

Amac: hedefi yatayda merkezde tutarken hedefe yaklasmak.

Bu faz S4 ve S6 kabul edilebilir calistiktan sonra kosulur.

Dry-run kabul:

- `phase_runner.py --include-real-combined` S7-real kapisini ekler.
- Gercek oyuna joystick komutu basmadan yaw ve pitch PID ayni kosuda
  calistirilir.
- `found_ratio >= 0.85`
- Yaw ve pitch komutlari ayri ayri bounded kalir.
- Horizontal error, width error, command magnitude ve neutralizasyon raporlanir.
- Target loss durumunda input neutral'a doner.

Ornek:

```bash
fpv_env/bin/python experiments/game_screen_sandbox/phase_runner.py \
  --capture-window-title pr0p \
  --capture-backend ffmpeg \
  --tracker-bbox-file logs/game_screen_sandbox/pr0p-target-bbox.json \
  --include-real-combined \
  --desired-target-width 120 \
  --duration 2 \
  --hz 10 \
  --run-id sandbox-pr0p-combined-dry
```

Canli/game-response kabul:

- 60 saniye kosu.
- `found_ratio >= 0.85`
- Horizontal center error kabul bandinda.
- Bbox width veya S6b ikinci metrik hedefe yaklasma egilimi gosteriyor.
- PID saturation dusuk.
- Input komutlari rate-limit icinde.
- Target loss durumunda input neutral'a donuyor.

Bu faz basarili olursa Kenet PID davranisi icin kullanisli bir sandbox elde
edilmis sayilir.

## Faz S8 - Hareketli Hedef

Amac: hedef oyun icinde veya sahnede hareket ederken takip davranisini olcmek.

Senaryo secenekleri:

1. Oyunun icindeki hareketli obje/araç.
2. pr0p gibi ortamda ghost replay: once kendi ucusunu kaydet, sonra ghost'u
   takip et.
3. Statik hedef + hareketli kamera: drone hedef etrafinda elle/scriptle
   yorungede ucarken tracker kilidi korunuyor mu?
4. Gerekirse lokal overlay hedefi; bu daha zayif kanittir ve ayri etiketlenir.

Kabul:

- Hedef en az 30 saniye takip edilir.
- Kopma sayisi raporlanir.
- Hedef merkez hatasi raporlanir.
- PID komutlari bounded kalir.

Mevcut uygulama notu:

- `phase_runner.py --include-moving-target` S8 sentetik hareketli hedef
  kapisini ekler.
- Bu kapı Gazebo/Betaflight ve gercek oyun gerektirmez; deterministic hareketli
  hedef uzerinde tracker -> yaw+pitch PID -> dry-run input zincirini olcer.
- Rapor; hedef merkez hareketi, `found_ratio`, loss event sayisi, yaw/pitch
  command bound ve neutralizasyon metriklerini yazar.
- Gercek oyun/sim hedefi secildiginde ayni metrikler game-specific S8 kosusuna
  tasinir; bu sentetik kapı sadece pre-game regresyon kapisidir.

Ornek:

```bash
fpv_env/bin/python experiments/game_screen_sandbox/phase_runner.py \
  --include-moving-target \
  --duration 2 \
  --hz 10 \
  --min-moving-target-motion 30 \
  --run-id sandbox-moving-target
```

## Faz S9+ - In-Process Kapali Cevrim Kanitlari

Mevcut ek kapilar:

- `S9-game`: PID yaw cikisi basit oyun kamerasina dogrudan uygulanir.
- `S10-range`: pitch/forward PID sentetik hedef genisligini degistirir.
- `S11-handoff`: manuel merkezleme -> takip komutu -> otonom kontrol devrini
  olcer.
- `S12-adapter`: `screen_tracking_loop` komutu `InputAdapter` uzerinden yollar,
  basit oyun frame source'u bu komutu bir sonraki frame'e uygular.
- `S13-binding-dry`: RC/channel binding adimlarini dry-run adapter ile olcer.
- `S14-objective`: manuel merkezleme, takip komutu, tracker init, PID yaw/pitch
  ve adapter uzerinden uygulanan kontrolu tek basit oyun akisi icinde olcer.
- `S9/S10/S11/S14` raporlari `yaw_initial_command_corrective` ve pitch kullanan
  fazlarda `pitch_initial_command_corrective` metriklerini yazar. Boylece ters
  eksen/isaret hatasi sadece "son hata azaldi mi?" metriğine kalmadan yakalanir.

Ornek:

```bash
fpv_env/bin/python experiments/game_screen_sandbox/phase_runner.py \
  --include-moving-target \
  --include-game-dynamics \
  --include-range-dynamics \
  --include-handoff-dynamics \
  --include-adapter-dynamics \
  --include-binding-dry \
  --include-objective-loop \
  --duration 6 \
  --hz 20 \
  --run-id sandbox-independent-closed-loop
```

## Raporlama

Her kosu su dosyalari uretir:

```text
logs/game_screen_sandbox/<run-id>-frames.jsonl
logs/game_screen_sandbox/<run-id>-tracking.jsonl
logs/game_screen_sandbox/<run-id>-commands.jsonl
logs/game_screen_sandbox/<run-id>-summary.md
```

`summary.md` alanlari:

```text
Run:
Date:
Game/sim:
Window/crop:
Tracker:
PID gains:
Input adapter:
Verdict:
Found ratio:
Center error RMS/P95:
Latency ms:
PID saturation:
Target-loss events:
Approach metric:
Notes:
```

Mevcut uygulama notu:

- `phase_runner.py` her kosuda paired JSON + Markdown raporu yazar.
- Markdown raporu artik uc katmanlidir:
  - `Run Summary`: kaynak/game, input adapter, izolasyon ve verdict.
  - `Evidence Summary`: phase bazinda found ratio, loss, hareket, yaw/pitch
    command bound, latency/forward metriği, real input ve log yolu.
  - `Raw Metrics`: JSON metriklerinin okunabilir kopyasi.
- Rapor ustunde `sandbox-only; no Gazebo/Betaflight launcher required` izolasyon
  notu bulunur. Bu, game/screen sandbox'in ana Gazebo yoluna baglanmadigini
  her kosuda gorunur hale getirir.

## Basari Kriteri

Bu sandbox basarili sayilir eger:

- Ekran yakalama stabil.
- Tracker oyun goruntusunde hedefi takip ediyor.
- Virtual input oyuna guvenli komut verebiliyor.
- Yaw-only dongu hedefi yatayda merkeze yaklastiriyor.
- Gecikme olculebiliyor.
- Combined yaw+pitch dongu 60 saniye kontrolden cikmadan kosuyor.
- Tum kosular log ve summary uretiyor.

## Durma Kriteri

Asagidaki durumlarda deney durdurulur:

- Capture 20 FPS altinda kaliyor.
- Input otomasyonu guvenli neutral'a donemiyor.
- Oyun multiplayer/anti-cheat zorunlu kiliyor.
- Tracker hedefi basit hareketlerde bile surekli kaybediyor.
- PID komutlari oyunu surekli kontrolden cikariyor ve S4b ile toparlanmiyor.
- Approach metrikleri S6b sonrasinda bile yorumsuz kaliyor.
- Deney ana repo launch/test yollarini degistirmeyi gerektiriyor.

## Ilk Somut Adimlar

1. Oyun sec: once pr0p denenir; olmazsa SITL Forge veya baska offline FPV oyun.
2. `experiments/game_screen_sandbox/` klasorunu olustur.
3. Gerekirse `fpv_env` icin capture/input paketlerini ekle (`mss`, `evdev` gibi).
4. `capture_window.py` ile oyun penceresinden 30 saniye frame al.
5. `virtual_input.py` ile neutral + kucuk yaw komutu bas.
6. `screen_tracking_loop.py` ile yaw-only minimal donguyu calistir.
7. Sorun gorulurse S4b guard fazini uygula; sorun yoksa S5 gecikme olcumune
   gec.
8. Pitch/approach dene; sorun gorulurse S6b ikinci metrik fazini uygula.
9. Sonucu `roadmap-v2.md` icindeki tracker/PID gelistirme hattina not et.

## Rollback

Silinecekler:

```text
experiments/game_screen_sandbox/
logs/game_screen_sandbox/
```

Kontrol:

```bash
git status --short
pgrep -af 'game_screen_sandbox|uinput|ffmpeg' || true
```
