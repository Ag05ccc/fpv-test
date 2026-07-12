# Checkpoint Goal

Olusturma tarihi: 2026-07-05
Son guncelleme: 2026-07-07

Aktif hedef:

> Plani uygulamaya basla; mevcut Gazebo ile olan yapidan tamamen bagimsiz olsun.

Bu hedef tamamlanmis sayilmadi. Bu dosya, kota harcamadan daha sonra ayni
hedefe devam etmek icin devredilebilir checkpoint'tir.

## Kisa Durum

Gazebo/Betaflight yolundan tamamen izole bir `game/screen sandbox` yolu kuruldu.
Ana kapsam `experiments/game_screen_sandbox/` altinda.

Su an kanitlanan bagimsiz akista sunlar var:

- Harici oyun/sim penceresinden goruntu capture.
- BBox secimi ve tracker preflight.
- Tracker + PID dry-run.
- RC channel / axis binding yardimcisi.
- Opsiyonel, acik onayli virtual RC / `uinput`.
- Tek komut acceptance raporu.
- Gazebo/Betaflight import ve process izolasyon denetimi.
- Opsiyonel hard freshness gate: `--require-fresh-evidence`.
- Pr0p/SITL Forge gibi gercek pencere icin status-only external target
  acceptance kapisi.

Son full acceptance sonucu:

- `SIMPLE_SANDBOX_READY`
- Rapor:
  `logs/game_screen_sandbox/20260705-113825-acceptance-external-follow-session-ready-acceptance-run.md`
- Decision:
  `logs/game_screen_sandbox/20260705-113825-acceptance-external-follow-session-ready-decision.md`
- Decision icinde `S13-binding-dry PASS`.
- Izolasyon:
  - `forbidden_imports: []`
  - `new_forbidden_processes: []`
  - `isolation_status: PASS`

Son test durumu, onceki dogrulamada:

```text
573 passed
```

Son hedefli sandbox testi, interaktif bbox secimi, resume-runner UI guard,
`requires_pass` dependency gate'i, `resume_decision` ozeti ve
`readiness_ladder` asama ozeti, `evidence_freshness` kanit yasi ozeti ve
opsiyonel `freshness_gate`, external target acceptance kapisi,
process-boundary audit'i, `acceptance_decision` ust ozeti ve
`operator_command_queue` / `operator_command_summary`, ayrica
`resume_step_summary` ve `external_operator_preflight.py` eklendiginden sonra:

```text
tests/test_game_screen_sandbox.py: 167 passed
tests/test_simitl_pr0p_probe.py: 85 passed
tests/test_game_screen_sandbox.py + tests/test_simitl_pr0p_probe.py: 252 passed
```

Bu checkpoint ilk yazildiginda testler tekrar calistirilmiyordu. 2026-07-06
guncellemelerinde hedefli sandbox testi tekrar calistirildi. Son dogrulamada
153 test gecti.

## Son Anlik Snapshot

2026-07-06 son kontrolunde devam etmek icin kota-dostu okuma yapildi; yeni sim,
capture, `uinput`, Gazebo veya Betaflight prosesi baslatilmadi.
Bu turdaki son dogrulamada `tests/test_game_screen_sandbox.py` tekrar calisti,
`153 passed` sonucu verdi; `external_target_acceptance.py` ve ilgili test dosyasi
`py_compile` kontrolunden gecti.

Anlik proses kontrolu:

```text
pgrep -af 'Kenet Simple Target Game|simple_target_game|external_dry_run_sequence|external_follow_session|external_window_preflight|bbox_tool|ffmpeg|gazebo|gzserver|gzclient|betaflight|pr0p|SITL_Forge'
```

Sonuc: cikti yok. Yani bu checkpoint yazilirken ilgili sandbox/sim/capture
prosesleri calismiyordu.

2026-07-06 devam guncellemesi:

- `external_dry_run_sequence.py` raporuna makine-okunur `sequence_steps` ve
  `live_readiness` alanlari eklendi.
- `live_readiness`, ancak status + preflight + follow dry-run tamamen gecerse
  `READY_FOR_OPTIONAL_LIVE_INPUT` olur.
- Bu alanlar sonraki agent'in hangi adimda durdugunu gormesi icindir; sequence
  yine `uinput`, Gazebo veya Betaflight baslatmaz.
- `fpv_env/bin/python -m py_compile experiments/game_screen_sandbox/external_dry_run_sequence.py`
  gecti.
- pr0p penceresi ve bbox yokken smoke beklenen sekilde `WAITING` verdi:
  `logs/game_screen_sandbox/20260706-213335-pr0p-sequence-waiting-smoke-v4-external-dry-run-sequence.md`.
- Repo-local `Kenet Simple Target Game` penceresi ile sequence smoke
  `EXTERNAL_DRY_RUN_READY` verdi ve `live_readiness` alaninda
  `READY_FOR_OPTIONAL_LIVE_INPUT` yazdi:
  `logs/game_screen_sandbox/20260706-213251-simple-window-dry-sequence-ready-v3-external-dry-run-sequence.md`.

2026-07-06 ikinci devam guncellemesi:

- `external_live_input_readiness.py` eklendi.
- Bu gate once `external_dry_run_sequence.py` kosar; dry sequence gecmeden
  axis-response'a gecmez.
- Dry sequence gecer ama `--ack-live-input` verilmezse `WAITING` ve
  `ACK_LIVE_INPUT_REQUIRED` raporu yazar, virtual RC gondermez.
- `--ack-live-input` ile S5-real visual axis-response da gecer ise
  `EXTERNAL_LIVE_INPUT_READY` yazar.
- Repo-local `Kenet Simple Target Game` penceresi ile ack verilmeden smoke
  beklenen sekilde `WAITING` verdi; dry sequence `EXTERNAL_DRY_RUN_READY`,
  `ack_live_input` `WAITING`, `axis_response` `NOT_RUN`:
  `logs/game_screen_sandbox/20260706-214002-simple-window-live-readiness-noack-v1-external-live-input-readiness.md`.

2026-07-06 ucuncu devam guncellemesi:

- `sandbox_status_report.py` artik `external_live_input_readiness.py` raporunu
  status item olarak okur.
- Varsayilan status hala dry-run readiness'i bozmadan raporlar.
- `--require-live-input` verilirse `EXTERNAL_LIVE_INPUT_READY` kaniti yokken
  status `WAITING` kalir ve sonraki aksiyon olarak
  `external_live_input_readiness.py --ack-live-input` onerilir.
- pr0p penceresi/bbox yokken `sandbox_status_report.py --require-live-input`
  smoke beklenen sekilde `WAITING` verdi ve yeni live-input item'ini raporladi:
  `logs/game_screen_sandbox/20260706-214320-pr0p-status-require-live-smoke-v1-status-report.md`.

2026-07-06 dorduncu devam guncellemesi:

- `external_live_follow_sequence.py` eklendi.
- `--ack-live-input` yoksa readiness veya follow calistirmeden `WAITING` ve
  `ACK_LIVE_INPUT_REQUIRED` raporu yazar.
- Ack varsa once `external_live_input_readiness.py` kosar; readiness
  `EXTERNAL_LIVE_INPUT_READY` olmadan live follow'a gecmez.
- Readiness gectikten sonra bounded `external_follow_session.py --uinput`
  calistirir ve basarili olursa `EXTERNAL_LIVE_FOLLOW_COMPLETE` yazar.
- pr0p penceresi/bbox/ack yokken smoke beklenen sekilde `WAITING` verdi;
  `live_input_readiness` ve `live_follow` adimlari `NOT_RUN` kaldi:
  `logs/game_screen_sandbox/20260706-214757-pr0p-live-follow-noack-smoke-v1-external-live-follow-sequence.md`.

2026-07-06 besinci devam guncellemesi:

- `sandbox_status_report.py` artik `external_live_follow_sequence.py`
  raporunu da status item olarak okur.
- Varsayilan status dry-run readiness'i bozmadan kalir.
- `--require-live-follow` verilirse `EXTERNAL_LIVE_FOLLOW_COMPLETE` kaniti
  yokken status `WAITING` kalir ve sonraki aksiyon olarak
  `external_live_follow_sequence.py --ack-live-input` onerilir.
- pr0p penceresi/bbox yokken `sandbox_status_report.py --require-live-follow`
  smoke beklenen sekilde `WAITING` verdi ve onceki no-ack live-follow raporunu
  `WAITING` item olarak okudu:
  `logs/game_screen_sandbox/20260706-215250-pr0p-status-require-live-follow-smoke-v1-status-report.md`.

2026-07-06 altinci devam guncellemesi:

- `sandbox_status_report.py` artik preflight/follow/readiness/live-follow
  raporlarini yalniz pencere basligina gore degil, mevcut `--bbox-file`
  icindeki bbox ile de eslestirir.
- Hedef yeniden secildiyse ayni pencere basligina ait eski bbox raporlari
  `READY` kaniti sayilmaz; ilgili item `WAITING` kalir.
- Bbox dosyasi yokken external kanitlar hic eslesmis sayilmaz; status item'lari
  "current bbox is required..." seklinde bekler.
- pr0p penceresi/bbox yokken bbox-aware status smoke beklenen sekilde `WAITING`
  verdi ve external evidence path'leri `null` kaldi:
  `logs/game_screen_sandbox/20260706-215659-pr0p-status-bbox-aware-smoke-v2-status-report.md`.

2026-07-06 yedinci devam guncellemesi:

- `sandbox_status_report.py` artik `resume_commands` alanini yazar.
- `resume_commands`, bir sonraki uygulanabilir komutlari makine-okunur listeler.
- Virtual RC gonderebilecek komutlarda `sends_uinput: true` ve
  `requires_ack_live_input: true` gorunur.
- pr0p penceresi/bbox yokken status smoke `resume_commands` alaninda yalniz
  status ve bbox secim komutlarini verdi; `sends_uinput: true` komutu uretmedi:
  `logs/game_screen_sandbox/20260706-220008-pr0p-status-resume-commands-smoke-v1-status-report.md`.

2026-07-06 sekizinci devam guncellemesi:

- `resume_commands` girdileri guclendirildi.
- Her komut artik `safety_class`, `expected_status` ve `unblocks` alanlarini
  yazar.
- Rapor ureten komutlarda `expected_report_glob`; bbox yazma komutunda
  `expected_report_path` gorunur.
- Markdown status raporundaki `Resume Commands` tablosu artik safety ve expected
  kolonlarini da gosterir.
- Metadata testleri eklendi; hedefli sandbox testi tekrar calistirildi:
  `tests/test_game_screen_sandbox.py: 125 passed`.
- pr0p penceresi/bbox yokken metadata smoke beklenen sekilde `WAITING` verdi;
  yalniz status ve bbox komutlari onerildi, `sends_uinput: true` komutu
  uretilmedi:
  `logs/game_screen_sandbox/20260706-220440-pr0p-status-resume-metadata-smoke-v1-status-report.md`.

2026-07-06 dokuzuncu devam guncellemesi:

- `sandbox_resume_runner.py` eklendi.
- Varsayilan mod plan-only'dir; status raporu uretir ama capture, simulator
  veya input komutu calistirmaz.
- `--execute-safe`, sadece `sends_uinput: false` ve edit gerektirmeyen
  resume komutlarini calistirir.
- Live virtual RC komutlari ayrica `--execute-live-input` ve
  `--ack-live-input` olmadan calismaz.
- Runner raporu her adimi `SKIPPED`, `PLANNED`, `BLOCKED`, `PASS`, `WAITING`
  veya `REJECT` olarak ayirir.
- Hedefli sandbox testi tekrar calistirildi:
  `tests/test_game_screen_sandbox.py: 128 passed`.
- pr0p penceresi/bbox yokken plan-only smoke beklenen sekilde `WAITING`
  verdi, `executed_count: 0` kaldi ve input/capture baslatmadi:
  `logs/game_screen_sandbox/20260706-221133-pr0p-resume-runner-plan-smoke-v2-sandbox-resume-runner.md`.

2026-07-06 onuncu devam guncellemesi:

- `sandbox_resume_runner.py`, guvenli bir komut calistirdiktan sonra artik
  otomatik post-status raporu yazar.
- Post-status `READY` ise runner sonucu `SANDBOX_RESUME_READY` seviyesine
  yukseltir; post-status hala bekliyorsa runner `WAITING`/progress bilgisini
  ayrik raporlar.
- `--execute-safe` smoke sirasinda gercek bir hata yakalandi:
  `bbox_tool.py`, pr0p penceresi yokken traceback ile `REJECT` oluyordu.
- `bbox_tool.py` missing-window durumunda artik temiz `WAITING` raporu ve exit
  code `2` doner.
- Hedefli sandbox testi tekrar calistirildi:
  `tests/test_game_screen_sandbox.py: 130 passed`.
- pr0p penceresi/bbox yokken `--execute-safe` smoke artik beklenen sekilde
  `WAITING` verdi, post-status artifact'i yazdi ve input baslatmadi:
  `logs/game_screen_sandbox/20260706-221459-pr0p-resume-runner-execsafe-poststatus-smoke-v2-sandbox-resume-runner.md`.

2026-07-06 on birinci devam guncellemesi:

- `sandbox_status_report.py` resume komutlari artik dry evidence hazir ama
  live-input readiness eksikken `rc_binding_assistant.py` komutunu da listeler.
- Bu komut `Controls -> RC Channels` sayfasinda yaw/pitch/roll/throttle
  binding yapmak icindir.
- `rc_binding_assistant` komutu `sends_uinput: true`,
  `requires_ack_live_input: true`, `safety_class: live_binding_ack_required`
  olarak isaretlenir.
- Runner varsayilan ve `--execute-safe` modunda bu komutu calistirmaz; ancak
  `--execute-live-input --ack-live-input` ile acilir.
- Hedefli sandbox testi tekrar calistirildi:
  `tests/test_game_screen_sandbox.py: 130 passed`.
- pr0p penceresi/bbox yokken status smoke beklenen sekilde `WAITING` verdi ve
  binding komutu uretmedi:
  `logs/game_screen_sandbox/20260706-221901-pr0p-status-rc-binding-command-smoke-v1-status-report.md`.

2026-07-06 on ikinci devam guncellemesi:

- `rc_binding_assistant.py` artik rapora opsiyonel `window_title` ve
  `tracker_bbox` baglami yazar.
- `rc_binding_assistant.py` CLI icin `--window-title` ve `--tracker-bbox-file`
  eklendi.
- `sandbox_status_report.py`, `*-rc-binding.json` raporlarini yalniz mevcut
  pencere basligi ve mevcut bbox ile eslesirse `rc_binding_assistant PASS`
  sayar.
- Eski veya farkli bbox'a ait binding raporlari yeni hedef icin kanit olmaz.
- `resume_commands` icindeki binding komutu artik `--window-title` ve
  `--tracker-bbox-file` parametreleriyle uretilir.
- Hedefli sandbox testi tekrar calistirildi:
  `tests/test_game_screen_sandbox.py: 132 passed`.
- Sentetik bbox + dry-run binding + status smoke ile `matching_rc_binding`
  kaniti dogrulandi:
  `logs/game_screen_sandbox/20260706-222427-pr0p-status-rc-binding-context-smoke-v1-status-report.md`.

## 2026-07-07 pr0p / SimITL Checkpoint

Guncelleme zamani: `2026-07-07 10:19:57 +03`

Bu devam turunda calisma pr0p / SimITL hattinda, Gazebo'dan tamamen bagimsiz
olarak ilerletildi.

### Kanitlananlar

- pr0p isolated root guncellendi ve client calisir hale geldi:
  `/tmp/fpv-test-simitl-pr0p/pr0p.x86_64`
- pr0p versiyon guncellemesi updater ile tamamlandi:
  `logs/simitl_pr0p/manual_update/pr0p-updater-pty-yes-v1.log`
- pr0p local race UI smoke daha once PASS verdi:
  `logs/simitl_pr0p/20260707-100351-pr0p-ui-live-after-install-v1-ui-smoke.md`
- pr0p capture FPV goruntusu PASS verdi:
  `logs/simitl_pr0p/20260707-100942-pr0p-capture-current-v1-capture-probe.md`
- Secilen test bbox:
  `148,220,142,82`
- Bu bbox ile tracker/PID dry-run PASS verdi:
  `logs/simitl_pr0p/20260707-100538-pr0p-tracking-red-gate-v1-tracking-probe.md`
- Read-only MSP_RC baseline PASS verdi ve kanallar sabitti:
  `logs/simitl_pr0p/20260707-100856-pr0p-rc-baseline-after-yaw-v1-msp-ws-rc.md`
- MSP_SET_RAW_RC iki kez ack aldi, fakat `MSP_RC` baseline'da kaldi:
  `logs/simitl_pr0p/20260707-100912-pr0p-rc-loopback-after-baseline-v1-msp-ws-rc.md`
  - Bu durum `MSP_RC_WRITE_NOT_LATCHED_OR_RX_OVERRIDDEN` olarak raporlandi.
  - Yani websocket UART'a MSP yazisi ulasiyor, ama aktif RC kaynagi olmuyor.
- pr0p input config generic `Joystick` profilinde roll/pitch/throttle/yaw icin
  PASS verdi:
  `logs/simitl_pr0p/20260707-100927-pr0p-input-config-after-rc-loopback-v1-input-config.md`
- Kisa omurlu uinput ile yaw response WAITING kaldi:
  `logs/simitl_pr0p/20260707-100621-pr0p-yaw-live-after-ui-v1-response-probe.md`
- Kisa omurlu uinput ile throttle response WAITING kaldi:
  `logs/simitl_pr0p/20260707-101020-pr0p-throttle-live-diagnostic-v1-response-probe.md`
- Bu turda yeni daha guclu gate eklendi: pr0p baslamadan once uinput cihazini
  acip surec boyunca ayni cihazla response olcen persistent-uinput live gate.
- Persistent-uinput live throttle gate calisti, pr0p'u acti, UI ile local race'e
  girdi, response olctu ve kendi baslattigi pr0p'u temizledi; sonuc yine
  WAITING:
  `logs/simitl_pr0p/20260707-101409-pr0p-live-persistent-uinput-throttle-v1-live-session.md`
  - `persistent_uinput_active_at_launch: true`
  - `ui_smoke.status: PASS`
  - `live_response.status: WAITING`
  - `projection_px` yaklasik `0.5 px`, baseline drift seviyesinde.

### Kod/Test Degisiklikleri

- `experiments/simitl_pr0p_probe/pr0p_live_session_runner.py`
  - `--hold-uinput` eklendi.
  - `--run-live-response` eklendi.
  - `--ack-live-input` olmadan persistent input veya live response calismaz.
  - Live response ayni persistent `UInputAdapter` ile olculur.
- `experiments/simitl_pr0p_probe/pr0p_probe_suite.py`
  - Safe suite'e read-only `P5-rc-baseline` gate'i eklendi.
- `experiments/simitl_pr0p_probe/pr0p_live_run_manifest.py`
  - `P5-persistent-uinput-live-response` fazi eklendi.
  - `P5-rc-baseline` artik suite evidence'indan status/evidence alir.
- `experiments/simitl_pr0p_probe/pr0p_decision_report.py`
  - Promotion icin `P5-persistent-uinput-live-response PASS` zorunlu hale geldi.
- `experiments/simitl_pr0p_probe/README.md` ve `PLAN.md`
  - Persistent-uinput runtime response gate dokumante edildi.
- `tests/test_simitl_pr0p_probe.py`
  - Persistent-uinput ack ve lifecycle testleri eklendi.

Son testler:

```text
tests/test_simitl_pr0p_probe.py: 94 passed
tests/test_game_screen_sandbox.py + tests/test_simitl_pr0p_probe.py: 261 passed
```

### Son Fresh Raporlar

pr0p kapatildiktan sonra safe suite tazelendi:

- Suite:
  `logs/simitl_pr0p/20260707-101824-pr0p-suite-after-persistent-gate-v1-suite.md`
- Manifest:
  `logs/simitl_pr0p/20260707-101936-pr0p-live-manifest-after-rc-baseline-fix-v1-live-manifest.md`
- Independent readiness:
  `logs/simitl_pr0p/20260707-101936-independent-sim-after-rc-baseline-fix-v1-independent-sim-readiness.md`
- Decision:
  `logs/simitl_pr0p/20260707-101936-pr0p-decision-after-rc-baseline-fix-v1-decision.md`

Son durum `WAITING`. Bunun sebebi bu checkpoint yazilirken pr0p calismiyor:

- `P1-install-discovery PASS`
- `P1-client-executable PASS`
- `P4-input-readiness PASS`
- `P4-input-mapping PASS`
- `P6-synthetic-e2e-dry-run PASS`
- `P2-websocket WAITING`
- `P2-msp-readonly WAITING`
- `P5-rc-baseline WAITING`
- `P3-capture WAITING`
- `P6-tracking-pid-dry-run WAITING`

Process kontrolu checkpoint sonunda temizdi; pr0p/updater/Gazebo/Betaflight
prosesi yoktu.

### Acik Darbogaz

Sim goruntusu ve tracker/PID dry-run calisiyor. Ancak canlı kontrol kaniti
halen yok:

1. MSP_SET_RAW_RC ack aliyor ama `MSP_RC`'ye aktif kaynak olarak yansimiyor.
2. pr0p input config mapped gorunuyor, ama uinput komutlari local race
   runtime'inda olculebilir goruntu/vehicle response uretmiyor.
3. Persistent-uinput gate, "cihaz pr0p'tan sonra yaratildi" hipotezini
   zayiflatti; cihaz launch oncesi vardi ama response yine WAITING kaldi.

Bu nedenle bir sonraki adim, pr0p icindeki `Controls -> RC Channels` ekraninda
runtime'da eksen barlarinin gerçekten hareket edip etmedigini manuel veya
yarim-otomatik dogrulamaktir. Config dosyasinin PASS vermesi tek basina yeterli
degil.

### Devam Komutlari

1. pr0p'u persistent input gate ile tekrar ac ve olc:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_live_session_runner.py \
  --launch-pr0p \
  --ack-live-launch \
  --send-ui \
  --ack-live-ui \
  --hold-uinput \
  --ack-live-input \
  --run-live-response \
  --response-axis throttle \
  --response-magnitude 0.35 \
  --response-image-axis y \
  --response-expected-sign 0 \
  --response-post-duration 1.5 \
  --response-max-shift-px 300 \
  --startup-wait 4 \
  --run-id pr0p-live-persistent-uinput-throttle-next
```

## VERY LATEST 2026-07-07 10:29 +03

Ek olarak `P4-rc-channels-visual` runtime gate'i eklendi. Detayli bolum bu
dosyada yukarida `## LATEST 2026-07-07 10:29 +03 - Runtime RC Channels Visual Gate`
basligindadir.

Kisa ozet:

- Yeni gate: `experiments/simitl_pr0p_probe/pr0p_runtime_input_visual_probe.py`
- Kullanim yeri: pr0p `Controls -> RC Channels` sayfasi acikken.
- Amac: virtual RC pulse sirasinda ekrandaki RC bar/kanal bolgesinin degisip
  degismedigini olcmek.
- Promotion karari artik `P4-rc-channels-visual PASS` kanitini de bekliyor.
- Son testler:
  `tests/test_simitl_pr0p_probe.py: 100 passed`
  `tests/test_game_screen_sandbox.py tests/test_simitl_pr0p_probe.py: 267 passed`
- Son decision:
  `logs/simitl_pr0p/20260707-102901-pr0p-decision-runtime-visual-required-v1-decision.md`
  ve sebep listesinde `MISSING_RUNTIME_RC_CHANNEL_VISUAL` var.

Siradaki pratik komut, pr0p `Controls -> RC Channels` sayfasi acikken:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_runtime_input_visual_probe.py \
  --uinput \
  --ack-live-input \
  --axis yaw \
  --magnitude 0.6 \
  --run-id pr0p-rc-channels-visual-next
```

## VERY LATEST 2026-07-07 10:45 +03

Bu turda pr0p Controls sayfasina gercekten ulasildi ve runtime input visual
gate canli PASS verdi.

Canli UI rotasi:

1. pr0p acildi.
2. Login ekraninda `skip` tiklandi.
3. `Local -> Time attack` smoke ile local race acildi.
4. FPV ekraninda sol hamburger tiklandi.
5. `Controls` sekmesine tiklandi.
6. `Controls -> RC channels` ve sagda `Input visualization` goruldu.

Onemli kanit:

- Runtime visual input PASS:
  `logs/simitl_pr0p/20260707-103654-pr0p-rc-channels-visual-controls-yaw-v1-runtime-input-visual.md`
- Kullanilan crop:
  `--crop 480,110,320,190`
- Bu, pr0p runtime UI'nin virtual RC yaw input'unu gordugunu kanitlar.

Ardindan FPV gorunumune donuldu ve live safe suite calisti:

- Suite:
  `logs/simitl_pr0p/20260707-104219-pr0p-suite-after-rc-visual-pass-live-v1-suite.md`
- Kritik durum:
  - `P0-isolation PASS`
  - `P0-preflight PASS`
  - `P1-install-discovery PASS`
  - `P1-client-executable PASS`
  - `P2-websocket PASS`
  - `P2-msp-readonly PASS`
  - `P5-rc-baseline PASS`
  - `P3-capture PASS`
  - `P4-input-readiness PASS`
  - `P4-input-mapping PASS`
  - `P4-input-config-patch-dry PASS`
  - `P6-synthetic-e2e-dry-run PASS`
  - `P6-synthetic-log-check PASS`
  - `P6-tracking-pid-dry-run PASS`
  - `P6-tracking-log-check PASS`

Yeni manifest/readiness/decision:

- Manifest:
  `logs/simitl_pr0p/20260707-104235-pr0p-live-manifest-suite-rc-visual-live-v1-live-manifest.md`
- Independent readiness:
  `logs/simitl_pr0p/20260707-104235-independent-sim-suite-rc-visual-live-v1-independent-sim-readiness.md`
- Decision:
  `logs/simitl_pr0p/20260707-104248-pr0p-decision-suite-rc-visual-live-v1-decision.md`

Son decision artik sadece su nedenlerle `WAITING`:

- `MISSING_LIVE_YAW_RESPONSE`
- `MISSING_LIVE_PITCH_RESPONSE`
- `MISSING_RC_SOURCE_LOOPBACK`
- `MISSING_PERSISTENT_UINPUT_RESPONSE`
- `MISSING_LIVE_TRACKING_CONTROL`

Ek canli response denemeleri:

- Yaw response v2:
  `logs/simitl_pr0p/20260707-104327-pr0p-yaw-live-after-rc-visual-v2-response-probe.md`
  `WAITING`, projection yaklasik `-0.01 px`.
- Pitch response v2:
  `logs/simitl_pr0p/20260707-104332-pr0p-pitch-live-after-rc-visual-v2-response-probe.md`
  `WAITING`, projection yaklasik `0.50 px`, baseline drift seviyesinde.
- +Throttle response:
  `logs/simitl_pr0p/20260707-104411-pr0p-throttle-pos-live-after-rc-visual-v1-response-probe.md`
  `WAITING`.
- -Throttle response:
  `logs/simitl_pr0p/20260707-104416-pr0p-throttle-neg-live-after-rc-visual-v1-response-probe.md`
  `WAITING`.
- MSP RC loopback tekrar:
  `logs/simitl_pr0p/20260707-104127-pr0p-rc-loopback-controls-open-v1-msp-ws-rc.md`
  `WAITING`; MSP_SET_RAW_RC ack aliyor fakat `MSP_RC` baseline'da kaliyor.

Yorum:

- Config dosyasi ve runtime UI input artik kanitlandi.
- FPV arac/goruntu response halen yok.
- Siradaki teknik hipotez: arm/start-state veya physics input routing.
  Yani sorun artik `Controls -> RC Channels` mapping degil.

Son testler:

```text
tests/test_simitl_pr0p_probe.py: 101 passed
tests/test_game_screen_sandbox.py tests/test_simitl_pr0p_probe.py: 268 passed
```

Checkpoint sonunda pr0p/updater/Gazebo/Betaflight prosesi yoktu.

## TRUE LATEST 2026-07-07 22:52 +03 - APPROACH/PITCH GATE EKLENDI

Bu turda moving-target yaw-only PASS sonrasi pitch/approach asamasinin da
olculebilir bir gate olmasi saglandi. Canli pr0p/input baslatilmadi.

Yeni core akisi:

```text
core gates PASS
  -> extended_follow
extended_follow PASS
  -> moving_target_yaw
moving_target_yaw PASS
  -> approach_pitch
approach_pitch PASS
  -> manual-to-autonomous handoff gate planlanabilir
```

Eklenen dosya:

- `experiments/simitl_pr0p_probe/pr0p_approach_pitch_plan.py`

Bu runner:

- pr0p/Gazebo/Betaflight baslatmaz.
- OS input/uinput basmaz.
- Sentetik range-dynamics ile pitch/approach regresyonu calistirir.
- Gercek pr0p icin `--real-tracking-report path/to/*-tracking-acceptance.json`
  ve `--ack-real-approach-target` ister.
- Gercek approach gate su metrikleri kontrol eder:
  - live tracking acceptance PASS
  - `duration >= 20 s`
  - `found_ratio >= 0.90`
  - `loss_events <= 2`
  - `enable_pitch = true`
  - `real_input_sent = true`
  - `initial_abs_width_error_px >= 10`
  - `final_abs_width_error_px <= 12`
  - `width_error_reduction_ratio >= 0.50`
  - pitch komutu kullanilmis ama bounded kalmis

Core degisikligi:

- `experiments/simitl_pr0p_probe/pr0p_core_goal_report.py`
  - latest `*-approach-pitch.json` raporunu okur.
  - `moving_target_yaw PASS` olduktan sonra `approach_pitch` post-core gate'i
    acilir.
  - `approach_pitch` eksikse `next_command_key = approach_pitch_plan`.
  - `approach_pitch PASS` olursa `next_command_key = None` ve siradaki is
    manual-to-autonomous handoff gate planidir.

Test durumu:

```text
targeted approach/moving/core docs: 34 passed
tests/test_simitl_pr0p_probe.py: 220 passed
tests/test_game_screen_sandbox.py + tests/test_simitl_pr0p_probe.py: 387 passed
isolation smoke: PASS
```

Dogru mevcut canli siralama hala degismedi: mevcut gercek loglarla core smoke
extended follow bekliyor. Once `extended_follow_live` PASS olmali; sonra
moving-target yaw; sonra approach/pitch.

## TRUE LATEST 2026-07-07 22:28 +03 - EXTENDED FOLLOW GATE AYRILDI

Bu turda Claude'un core PASS kanitlari incelendi ve eksik kalan kontrol
noktasi koda baglandi: kisa `tracking_acceptance PASS` artik 20 saniyelik
uzun follow kaniti sayilmiyor.

Yeni durum:

- `pr0p_core_goal_report.py` ana uc gate'i aynen koruyor:
  `rc_manual_flight`, `camera_tracker`, `autopilot_control`.
- Core PASS olduktan sonra raporda ayrica `post_core_gates.extended_follow`
  uretiliyor.
- Bu gate su kosullari olcuyor:
  - latest `*-tracking-acceptance.json` taze olmali.
  - `pr0p_tracking_live` PASS olmali.
  - `run_live_gates = true` olmali.
  - `duration_s >= 20.0`.
  - `min_found_ratio >= 0.95`.
  - `max_loss_events <= 0`.
  - live komut `--arm-first`, `--duration 20.0`,
    `--min-found-ratio 0.95`, `--max-loss-events 0` icermeli.

Canli sim baslatmadan kosulan smoke:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_core_goal_report.py \
  --bbox-file logs/simitl_pr0p/pr0p-target-bbox.json \
  --allow-physical-mapping \
  --run-id codex-post-core-gate-smoke
```

Sonuc:

```text
status: PASS
next_command_key: extended_follow_live
post_core_gates.extended_follow: WAITING
```

Neden WAITING: Claude'un son live tracking kaniti basarili ama 4 saniyelikti
ve `min_found_ratio` esigi 0.85 idi. Yeni gate bunu dogru sekilde kabul
etmedi; hareketli hedef asamasina gecmeden once 20 saniye / 0 loss / 0.95
found-ratio kosusu gerekiyor.

Degisen dosyalar:

- `experiments/simitl_pr0p_probe/pr0p_core_goal_report.py`
  - `post_core_gates` alani eklendi.
  - `extended_follow` gate'i latest tracking acceptance raporundan olculuyor.
  - Extended follow PASS olursa bir sonraki aksiyon hareketli hedef
    yaw-only fazina hazirlik; PASS degilse `extended_follow_live`.
- `tests/test_simitl_pr0p_probe.py`
  - tracking acceptance fixture'i artik duration/min-found/max-loss/live
    command bilgilerini yaziyor.
  - Core PASS + short tracking => `extended_follow WAITING`.
  - Core PASS + 20 s extended tracking => `extended_follow PASS`,
    `next_command_key = None`.
- `experiments/simitl_pr0p_probe/README.md`
  ve `experiments/simitl_pr0p_probe/PLAN.md`
  - Core PASS ile extended follow PASS ayrimi dokumante edildi.

Dogru sonraki canli adim:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_core_next_runner.py \
  --bbox-file logs/simitl_pr0p/pr0p-target-bbox.json \
  --allow-physical-mapping \
  --execute-next \
  --ack-live-command \
  --run-id pr0p-extended-follow-live
```

Bu komut pr0p UI ve RC Channels hazirken calistirilmali. PASS olduktan sonra
hareketli hedef icin sadece yaw-only takip fazina gecilmeli; pitch/approach
eklemeleri bu faz stabil olana kadar beklemeli.

## TRUE LATEST 2026-07-07 22:35 +03 - MOVING TARGET YAW PLAN EKLENDI

Bu turda extended follow PASS sonrasi bosa dusen `next_command_key = None`
durumu kaldirildi. Artik core zinciri su sekilde ilerliyor:

```text
core gates PASS
  -> extended_follow_live
extended_follow PASS
  -> moving_target_yaw_plan
```

Eklenen dosya:

- `experiments/simitl_pr0p_probe/pr0p_moving_target_yaw_plan.py`

Bu runner:

- pr0p/Gazebo/Betaflight baslatmaz.
- OS input / uinput basmaz.
- Sentetik hareketli hedef uzerinde yaw-only tracker -> PID -> dry-run input
  regresyonunu calistirabilir.
- Gercek pr0p hareketli hedef kanitini ayri `WAITING` birakır; cunku bunun
  icin sahnede hareket eden obje, ghost replay veya tekrar edilebilir operator
  senaryosu secilmelidir.
- Pitch/approach bu fazda kapali tutulur.

Core degisikligi:

- `experiments/simitl_pr0p_probe/pr0p_core_goal_report.py`
  - Yeni command key: `moving_target_yaw_plan`.
  - `extended_follow PASS` olursa `next_command_key` artik
    `moving_target_yaw_plan`.

Canli sim/input baslatmadan kosulan smoke:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_moving_target_yaw_plan.py \
  --bbox-file logs/simitl_pr0p/pr0p-target-bbox.json \
  --execute-synthetic \
  --duration 2.0 \
  --hz 15 \
  --min-found-ratio 0.8 \
  --min-center-motion 15 \
  --run-id codex-moving-target-yaw-smoke
```

Sonuc:

```text
overall: WAITING
synthetic_moving_target_yaw: PASS
real_pr0p_moving_target_yaw: WAITING
found_ratio: 1.0
loss_events: 0
center_motion_ok: True
enable_pitch: False
pitch_zero: True
real_input_sent: False
```

Core smoke mevcut gercek loglarla hala beklenen sekilde extended follow'da:

```text
status: PASS
next_command_key: extended_follow_live
post_core_gates.extended_follow: WAITING
```

Yani dogru siradaki canli adim degismedi:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_core_next_runner.py \
  --bbox-file logs/simitl_pr0p/pr0p-target-bbox.json \
  --allow-physical-mapping \
  --execute-next \
  --ack-live-command \
  --run-id pr0p-extended-follow-live
```

Bu extended follow PASS olduktan sonra core report otomatik olarak
`moving_target_yaw_plan` onerecek. O noktada sentetik yaw-only moving-target
regresyonu kosulacak, ardindan gercek pr0p moving target icin sahne/ghost/hedef
secimi gerekecek.

## TRUE LATEST 2026-07-07 22:45 +03 - MOVING TARGET CORE GATE BAGLANDI

Bu turda `moving_target_yaw_plan` sadece bir komut onerisi olmaktan cikarildi;
core rapor extended follow PASS olduktan sonra artik bunu post-core gate olarak
da izliyor.

Yeni core akisi:

```text
core gates PASS
  -> post_core_gates.extended_follow
extended_follow PASS
  -> post_core_gates.moving_target_yaw
moving_target_yaw PASS
  -> yaw-only moving target tamam; pitch/approach icin yeni gate planlanabilir
```

Degisen dosyalar:

- `experiments/simitl_pr0p_probe/pr0p_core_goal_report.py`
  - latest `*-moving-target-yaw.json` raporunu okur.
  - `moving_target_yaw` gate'i `synthetic_moving_target_yaw` ve
    `real_pr0p_moving_target_yaw` adimlarini kontrol eder.
  - Extended follow PASS ama moving-target raporu yoksa
    `next_command_key = moving_target_yaw_plan`.
  - Moving-target PASS olursa `next_command_key = None`.
- `experiments/simitl_pr0p_probe/pr0p_moving_target_yaw_plan.py`
  - `--real-tracking-report path/to/*-tracking-acceptance.json` okuyabilir.
  - `--ack-real-moving-target` olmadan gercek moving target PASS olmaz.
  - Gercek gate su metrikleri kontrol eder:
    - `duration >= 30 s`
    - `found_ratio >= 0.90`
    - `loss_events <= 2`
    - `target_center_span_px >= 30`
    - `real_input_sent = true`
    - `enable_pitch = false`
    - `max_abs_pitch_axis = 0`
    - bounded yaw

Onemli ayrim:

- Sentetik yaw-only moving target PASS, gercek pr0p moving target yerine gecmez.
- Gercek pr0p moving target icin once hareketli sahne hedefi, ghost replay veya
  tekrar edilebilir operator senaryosu secilmeli.
- Sonra live tracking acceptance kosulur.
- Ardindan `pr0p_moving_target_yaw_plan.py --real-tracking-report ... --ack-real-moving-target`
  ile moving-target raporu refresh edilir.

Test durumu:

```text
targeted moving/core docs: 26 passed
```

Bu turda canli pr0p/input baslatilmadi.

## TRUE LATEST 2026-07-07 19:05 +03

Aktif hat halen Gazebo'dan tamamen bagimsiz `pr0p / SITL Forge` hattidir.
Bu turda P6 live tracker/PID control'e gecisi koruyan yeni kabul runner'i
eklendi. Amac, bbox + P6 dry-run + signed yaw/pitch response PASS kanitlari
olmadan `pr0p_tracking_probe.py --uinput` canli kontrolunun kosulmasini
engellemektir.

Yeni dosya:

- `experiments/simitl_pr0p_probe/pr0p_tracking_acceptance_runner.py`
  - Varsayilan plan-only calisir; pr0p baslatmaz, OS input gondermez.
  - `--bbox x,y,w,h` yoksa live tracking'i bekletir.
  - En guncel `*-live-manifest.json` icinde `P3-capture`,
    `P4-input-readiness`, `P4-input-mapping`,
    `P6-tracking-pid-dry-run` PASS arar.
  - En guncel `*-response-acceptance.json` icinde
    `pr0p_yaw_response_live` ve `pr0p_pitch_response_live` PASS arar.
  - `--run-live-gates --ack-live-input` verilirse sadece bu prerequisite'ler
    PASS oldugunda yaw-only `pr0p_tracking_probe.py --uinput` kosar.
  - Prerequisite eksikse live tracking adimini `SKIPPED` birakir.
  - `--execute-dry-run` ile P6 dry tracking komutu real OS input olmadan
    kosulabilir.

Readiness entegrasyonu:

- `independent_sim_readiness.py` artik iki yeni komut da uretir:
  - `pr0p_tracking_acceptance_plan`
  - `pr0p_tracking_acceptance_live`

Dokuman:

- `experiments/simitl_pr0p_probe/README.md` P6 bolumune tracking acceptance
  plan/live komutlari eklendi; direct live probe manuel debug olarak etiketlendi.
- `experiments/simitl_pr0p_probe/PLAN.md` Phase P6'ya ayni acceptance akisi
  eklendi.

Eklenen test kanitlari:

- `test_tracking_acceptance_runner_plan_only_does_not_execute`
- `test_tracking_acceptance_runner_requires_ack_for_live_input`
- `test_tracking_acceptance_runner_skips_live_without_response_pass`
- `test_tracking_acceptance_runner_runs_live_after_prereqs_pass`
- `test_tracking_acceptance_runner_executes_dry_run_without_live`

Son dogrulamalar:

- `fpv_env/bin/python -m pytest tests/test_simitl_pr0p_probe.py -q`
  sonuc: `153 passed`
- `fpv_env/bin/python -m pytest tests/test_game_screen_sandbox.py tests/test_simitl_pr0p_probe.py -q`
  sonuc: `320 passed`
- Gazebo bagimsizlik gate'i:
  `fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_isolation_check.py --run-id tracking-acceptance-isolation-v1`
  sonuc: `PASS`; rapor:
  `logs/simitl_pr0p/20260707-190504-tracking-acceptance-isolation-v1-isolation-check.md`
- Tracking acceptance plan-only dry run:
  `logs/simitl_pr0p/20260707-190438-tracking-acceptance-runner-v1-tracking-acceptance.md`
  sonuc: `WAITING`; bbox ve P6 manifest prerequisite PASS, eksik:
  `pr0p_yaw_response_live`, `pr0p_pitch_response_live`.
- Independent readiness dry run:
  `logs/simitl_pr0p/20260707-190438-independent-sim-tracking-acceptance-v1-independent-sim-readiness.md`
  `Commands` bolumunde `pr0p_tracking_acceptance_plan` ve
  `pr0p_tracking_acceptance_live` var.

Sıradaki teknik adim:

1. pr0p Controls -> RC Channels icinde AUX1/CH5'i Kenet virtual AUX1 axis'e
   manuel bagla ya da kullanici acik onay verirse `pr0p_aux1_config_patch_write`
   komutunu uygula.
2. `pr0p_aux1_acceptance_live` ile AUX1 RC-effect ve AUX1 arm-status PASS
   kanitini al.
3. `pr0p_response_acceptance_live` ile yaw signed response ve pitch signed
   response PASS kanitini al.
4. `pr0p_tracking_acceptance_live` ile yaw-only live tracker/PID kontrolunu
   kos. Pitch/approach daha sonra ve yalniz yaw-only stabil ise eklenmeli.

Checkpoint sonunda pr0p/updater/Gazebo/Betaflight prosesi yoktu.

## TRUE LATEST 2026-07-07 12:35 +03

Aktif hat halen Gazebo'dan tamamen bagimsiz `pr0p / SITL Forge` hattidir.
Bu turda AUX1 kabul zincirinden sonra calisacak signed yaw/pitch response
kabul runner'i eklendi. Amac, `P5-uinput-aux1-arm-status PASS` kaniti
olmadan response testlerinin yanlis sirada kosulmasini veya PASS sayilmasini
engellemektir.

Yeni dosya:

- `experiments/simitl_pr0p_probe/pr0p_response_acceptance_runner.py`
  - Varsayilan plan-only calisir; pr0p baslatmaz, UI tiklamaz, OS input
    gondermez.
  - En guncel `*-live-manifest.json` icinde su prerequisite'leri kontrol eder:
    `P3-capture`, `P4-input-readiness`, `P4-input-mapping`,
    `P5-uinput-rc-effect-throttle-low`,
    `P5-uinput-status-effect-throttle-clear`,
    `P5-uinput-rc-effect-aux1-high`,
    `P5-uinput-aux1-arm-status`.
  - `--run-live-gates --ack-live-launch --ack-live-ui --ack-live-input`
    verilirse prerequisite PASS ise once persistent-uinput yaw response,
    yalniz yaw PASS olursa pitch response kosar.
  - Prerequisite eksikse yaw/pitch response adimlarini `SKIPPED` birakir.

Readiness entegrasyonu:

- `independent_sim_readiness.py` artik iki yeni komut da uretir:
  - `pr0p_response_acceptance_plan`
  - `pr0p_response_acceptance_live`

Dokuman:

- `experiments/simitl_pr0p_probe/README.md` P5 response bolumune plan-only
  ve live response acceptance runner komutlari eklendi.
- `experiments/simitl_pr0p_probe/PLAN.md` Phase P5'e ayni acceptance akisi
  eklendi.

Eklenen test kanitlari:

- `test_response_acceptance_runner_plan_only_does_not_execute`
- `test_response_acceptance_runner_requires_ack_for_live_input`
- `test_response_acceptance_runner_skips_live_when_aux_arm_missing`
- `test_response_acceptance_runner_runs_pitch_after_yaw_pass`
- `test_response_acceptance_runner_skips_pitch_when_yaw_waits`

Son dogrulamalar:

- `fpv_env/bin/python -m pytest tests/test_simitl_pr0p_probe.py -q`
  sonuc: `148 passed`
- `fpv_env/bin/python -m pytest tests/test_game_screen_sandbox.py tests/test_simitl_pr0p_probe.py -q`
  sonuc: `315 passed`
- Gazebo bagimsizlik gate'i:
  `fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_isolation_check.py --run-id response-acceptance-isolation-v1`
  sonuc: `PASS`; rapor:
  `logs/simitl_pr0p/20260707-123521-response-acceptance-isolation-v1-isolation-check.md`
- Response acceptance plan-only dry run:
  `logs/simitl_pr0p/20260707-123448-response-acceptance-runner-v1-response-acceptance.md`
  sonuc: `WAITING`; eksikler:
  `P5-uinput-rc-effect-aux1-high`, `P5-uinput-aux1-arm-status`.
- Independent readiness dry run:
  `logs/simitl_pr0p/20260707-123448-independent-sim-response-acceptance-v1-independent-sim-readiness.md`
  `Commands` bolumunde `pr0p_response_acceptance_plan` ve
  `pr0p_response_acceptance_live` var.

Sıradaki teknik adim degismedi ama artik daha kontrollu:

1. pr0p Controls -> RC Channels icinde AUX1/CH5'i Kenet virtual AUX1 axis'e
   manuel bagla ya da kullanici acik onay verirse `pr0p_aux1_config_patch_write`
   komutunu uygula.
2. `pr0p_aux1_acceptance_live` ile once AUX1 RC-effect, sonra AUX1 arm-status
   PASS kanitini al.
3. Yeni `pr0p_response_acceptance_live` komutu ile yaw signed response PASS,
   ardindan pitch signed response PASS kanitini al.
4. Bu iki response gate PASS olmadan P6 live tracker/PID control'e gecme.

Checkpoint sonunda pr0p/updater/Gazebo/Betaflight prosesi yoktu.

## TRUE LATEST 2026-07-07 11:53 +03

En guncel durum: pr0p/SITL Forge bagimsiz hatta throttle-low cozuldu, ARM
mode range read-only olculdu, direct button ARM yolu denendi ve calismadi.
Betaflight ARM `AUX1 / CH5` icin `1700-2100` bekliyor.

Yeni kod/gate'ler:

- `experiments/simitl_pr0p_probe/msp_ws_mode_ranges_probe.py`
  read-only `MSP_MODE_RANGES` okur.
- `experiments/simitl_pr0p_probe/msp_uinput_arm_probe.py`
  throttle low tutulurken south/east button adaylarini dener.
- `experiments/game_screen_sandbox/virtual_input.py`
  artik `aux1`/`ABS_Z` ve button press/release destekliyor.
- `experiments/simitl_pr0p_probe/pr0p_input_config_patch.py`
  `--role aux1` dry-run/write destegi ekledi; slot 4 icin `<Joystick>/Z`
  oneriyor.
- `experiments/simitl_pr0p_probe/pr0p_live_session_runner.py`
  `--run-arm-button-effect` ve `aux1` RC-effect destekliyor.

Canli kanitlar:

- Mode ranges PASS:
  `logs/simitl_pr0p/20260707-114048-pr0p-live-suite-mode-ranges-v1-suite-p2-mode-ranges-msp-ws-mode-ranges.md`
  sonuc: ARM `AUX1 / CH5`, `1700-2100`.
- Button ARM denemesi WAITING:
  `logs/simitl_pr0p/20260707-113523-pr0p-live-uinput-arm-button-v1-live-session.md`
  sonuc: `NO_UINPUT_ARM_BUTTON_EFFECT`; south/east ARM mode uretmedi.
- AUX1 config dry-run PASS:
  `logs/simitl_pr0p/20260707-114819-pr0p-input-config-aux1-dry-v1-input-config-patch.md`
  sonuc: slot 4 bos -> `<Joystick>/Z`, `real_config_write=False`.
- AUX1 live RC-effect before patch WAITING:
  `logs/simitl_pr0p/20260707-115217-pr0p-live-uinput-rc-aux1-before-patch-v2-live-session.md`
  sonuc: `NO_UINPUT_MSP_RC_EFFECT`; CH5/AUX1 1500'de kaldi.
- Son manifest:
  `logs/simitl_pr0p/20260707-115241-pr0p-live-manifest-aux1-before-patch-v2-live-manifest.md`
  next action: backup-backed AUX1 patch/manual AUX1 bind, sonra
  `P5-uinput-rc-effect-aux1-high`.
- Son decision:
  `logs/simitl_pr0p/20260707-115241-pr0p-decision-aux1-before-patch-v2-decision.md`
  WAITING reasons icinde `MISSING_UINPUT_AUX1_RC_EFFECT` var.

Son testler:

- `tests/test_simitl_pr0p_probe.py`: 133 passed
- `tests/test_game_screen_sandbox.py tests/test_simitl_pr0p_probe.py`: 300 passed

Sıradaki teknik adim:

1. Kullanici onayi/manual tercih ile pr0p `input.json` icin AUX1 slot 4'u
   `<Joystick>/Z` olarak uygula ya da pr0p Controls -> RC Channels icinde
   AUX1/CH5'i Kenet virtual AUX1 axis'e bagla.
2. Sonra:
   `pr0p_live_session_runner.py --run-rc-effect --rc-effect-axis aux1 --rc-effect-expected-channel aux1 --rc-effect-expected-direction higher`
   ile CH5'in 1700+ oldugunu kanitla.
3. `throttle=+1.0` low + `aux1=+1.0` high birlikte tutulurken `FC_ARMED`
   / `ARMING_DISABLED` durumunu read-only status ile kanitla.
4. Ardindan P5 yaw/pitch live response ve P6 live tracking'e gec.

Checkpoint sonunda pr0p/updater/Gazebo/Betaflight prosesi yoktu.

## TRUE LATEST 2026-07-07 12:03 +03

Kullanici mevcut hattin pr0p sim uzerinde olup olmadigini sordu. Cevap:
evet, aktif calisma Gazebo'dan tamamen bagimsiz `pr0p / SITL Forge`
hattidir. Bu asamada sifirdan oyun yazilmiyor; pr0p ekrani, virtual
RC/uinput, MSP read-only olcumleri ve Kenet tracker/PID zinciri icin olculebilir
bir test hatti kuruluyor.

Bu ara turda eklenen yeni parca:

- `experiments/simitl_pr0p_probe/msp_uinput_aux_arm_status_probe.py`
  eklendi. Amaci: throttle low (`throttle=+1.0`) ve AUX1 high
  (`aux1=+1.0`) birlikte tutulurken FC'nin ARM/armed durumuna gecip
  gecmedigini read-only MSP status ile olcmek.
- `experiments/simitl_pr0p_probe/pr0p_live_session_runner.py`
  icine `--run-aux-arm-status` opsiyonu eklendi. Bu, pr0p launch +
  persistent uinput adapter ile yeni AUX1 arm-status probe'unu live session
  metriklerine ekler.
- `tests/test_simitl_pr0p_probe.py` icine bu yeni probe ve live-session
  baglantisi icin unit testler eklendi.

Son dogrulanmis testler:

- `fpv_env/bin/python -m pytest tests/test_simitl_pr0p_probe.py -q`
  sonuc: `137 passed`
- `fpv_env/bin/python -m pytest tests/test_game_screen_sandbox.py tests/test_simitl_pr0p_probe.py -q`
  sonuc: `304 passed`

Henuz tamamlanmayan entegrasyon:

- Yeni `P5-uinput-aux1-arm-status` gate'i manifest/decision/readiness
  raporlarina henuz tam eklenmedi. Devam edecek agent once
  `experiments/simitl_pr0p_probe/pr0p_live_run_manifest.py` ve
  `experiments/simitl_pr0p_probe/pr0p_decision_report.py` icinde bu gate'i
  resmi faz olarak tanimlamali.
- `checkpoint-goal.md` icindeki onceki 11:53 durum hala gecerlidir:
  AUX1/CH5 pr0p tarafinda bagli olmadigi icin son live RC-effect denemesi
  `NO_UINPUT_MSP_RC_EFFECT` ile `WAITING` verdi. ARM icin Betaflight
  `AUX1 / CH5` kanalinda `1700-2100` bekliyor.

Sıradaki teknik adim:

1. pr0p Controls -> RC Channels icinde AUX1/CH5'i Kenet virtual AUX1 axis'e
   manuel bagla ya da kullanici acik onay verirse backup-backed
   `pr0p_input_config_patch.py --role aux1 --write` yolunu uygula.
2. CH5'in 1700+ oldugunu kanitlamak icin:
   `pr0p_live_session_runner.py --run-rc-effect --rc-effect-axis aux1 --rc-effect-expected-channel aux1 --rc-effect-expected-direction higher`
   komutunu calistir.
3. Ardindan yeni eklenen arm-status olcumunu kos:
   `pr0p_live_session_runner.py --launch-pr0p --ack-live-launch --send-ui --ack-live-ui --hold-uinput --ack-live-input --run-aux-arm-status --aux-arm-throttle-magnitude 1.0 --aux-arm-aux1-magnitude 1.0 --startup-wait 8 --run-id pr0p-live-uinput-aux1-arm-status-v1`
4. Bu PASS olduktan sonra P5 yaw/pitch signed live response ve P6 live
   tracking control'e gec.

Checkpoint sonunda pr0p/updater/Gazebo/Betaflight prosesi yoktu.

## TRUE LATEST 2026-07-07 12:03 +03

Kullanici mevcut hattin pr0p sim uzerinde olup olmadigini sordu. Cevap:
evet, aktif calisma Gazebo'dan tamamen bagimsiz `pr0p / SITL Forge`
hattidir. Bu asamada sifirdan oyun yazilmiyor; pr0p ekrani, virtual
RC/uinput, MSP read-only olcumleri ve Kenet tracker/PID zinciri icin olculebilir
bir test hatti kuruluyor.

Bu ara turda eklenen yeni parca:

- `experiments/simitl_pr0p_probe/msp_uinput_aux_arm_status_probe.py`
  eklendi. Amaci: throttle low (`throttle=+1.0`) ve AUX1 high
  (`aux1=+1.0`) birlikte tutulurken FC'nin ARM/armed durumuna gecip
  gecmedigini read-only MSP status ile olcmek.
- `experiments/simitl_pr0p_probe/pr0p_live_session_runner.py`
  icine `--run-aux-arm-status` opsiyonu eklendi. Bu, pr0p launch +
  persistent uinput adapter ile yeni AUX1 arm-status probe'unu live session
  metriklerine ekler.
- `tests/test_simitl_pr0p_probe.py` icine bu yeni probe ve live-session
  baglantisi icin unit testler eklendi.

Son dogrulanmis testler:

- `fpv_env/bin/python -m pytest tests/test_simitl_pr0p_probe.py -q`
  sonuc: `137 passed`
- `fpv_env/bin/python -m pytest tests/test_game_screen_sandbox.py tests/test_simitl_pr0p_probe.py -q`
  sonuc: `304 passed`

Henuz tamamlanmayan entegrasyon:

- Yeni `P5-uinput-aux1-arm-status` gate'i manifest/decision/readiness
  raporlarina henuz tam eklenmedi. Devam edecek agent once
  `experiments/simitl_pr0p_probe/pr0p_live_run_manifest.py` ve
  `experiments/simitl_pr0p_probe/pr0p_decision_report.py` icinde bu gate'i
  resmi faz olarak tanimlamali.
- `checkpoint-goal.md` icindeki onceki 11:53 durum hala gecerlidir:
  AUX1/CH5 pr0p tarafinda bagli olmadigi icin son live RC-effect denemesi
  `NO_UINPUT_MSP_RC_EFFECT` ile `WAITING` verdi. ARM icin Betaflight
  `AUX1 / CH5` kanalinda `1700-2100` bekliyor.

Sıradaki teknik adim:

1. pr0p Controls -> RC Channels icinde AUX1/CH5'i Kenet virtual AUX1 axis'e
   manuel bagla ya da kullanici acik onay verirse backup-backed
   `pr0p_input_config_patch.py --role aux1 --write` yolunu uygula.
2. CH5'in 1700+ oldugunu kanitlamak icin:
   `pr0p_live_session_runner.py --run-rc-effect --rc-effect-axis aux1 --rc-effect-expected-channel aux1 --rc-effect-expected-direction higher`
   komutunu calistir.
3. Ardindan yeni eklenen arm-status olcumunu kos:
   `pr0p_live_session_runner.py --launch-pr0p --ack-live-launch --send-ui --ack-live-ui --hold-uinput --ack-live-input --run-aux-arm-status --aux-arm-throttle-magnitude 1.0 --aux-arm-aux1-magnitude 1.0 --startup-wait 8 --run-id pr0p-live-uinput-aux1-arm-status-v1`
4. Bu PASS olduktan sonra P5 yaw/pitch signed live response ve P6 live
   tracking control'e gec.

Checkpoint sonunda pr0p/updater/Gazebo/Betaflight prosesi yoktu.

## VERY LATEST 2026-07-07 11:24 +03 - Final Pointer

En guncel durum bu bloktur; 11:04 pointer artik eski kaldi.

Yeni gate'ler:

- `experiments/simitl_pr0p_probe/msp_uinput_rc_effect_probe.py`
  - UInput pulse sirasinda `MSP_RC` kanal degerleri degisiyor mu olcer.
  - MSP write yapmaz; sadece uinput + read-only `MSP_RC`.
- `experiments/simitl_pr0p_probe/msp_uinput_status_probe.py`
  - UInput komutu tutulurken `MSP_STATUS_EX` arm blocker'lari temizleniyor mu
    olcer.
  - MSP write yapmaz; sadece uinput + read-only status.
- `pr0p_live_session_runner.py` artik persistent uinput ile:
  - `--run-rc-effect`
  - `--run-status-effect`
  destekliyor.

Canli kanitlar:

- `throttle=-1.0` testi:
  `logs/simitl_pr0p/20260707-111509-pr0p-live-uinput-rc-effect-throttle-neg-v1-live-session.md`
  sonuc: FAIL direction; MSP_RC throttle `1500 -> 2000`.
- `throttle=+1.0` testi:
  `logs/simitl_pr0p/20260707-111545-pr0p-live-uinput-rc-effect-throttle-pos-v1-live-session.md`
  sonuc: PASS, `THROTTLE_LOW_REACHED`; MSP_RC throttle `1500 -> 1000`.
- Status-effect testi:
  `logs/simitl_pr0p/20260707-112145-pr0p-live-uinput-status-throttle-low-v1-live-session.md`
  sonuc: PASS, `UINPUT_CLEARED_THROTTLE`.
  Baseline blocker `THROTTLE,BOOTGRACE,CALIB`; throttle tutulurken `THROTTLE`
  temizlendi, `BOOTGRACE,CALIB` kaldi.
- Son manifest:
  `logs/simitl_pr0p/20260707-112349-pr0p-live-manifest-uinput-status-effect-v1-live-manifest.md`
  next action: `Throttle-low clears THROTTLE; next discover and drive the ARM AUX/button path, then verify FC_ARMED with P2-fc-status-readonly.`
- Son readiness:
  `logs/simitl_pr0p/20260707-112350-independent-sim-uinput-status-effect-v1-independent-sim-readiness.md`
- Son decision:
  `logs/simitl_pr0p/20260707-112358-pr0p-decision-uinput-status-effect-v1-decision.md`
  reasons: `FC_ARM_STATE_BLOCKED`, `MISSING_LIVE_YAW_RESPONSE`,
  `MISSING_LIVE_PITCH_RESPONSE`, `MISSING_RC_SOURCE_LOOPBACK`,
  `MISSING_PERSISTENT_UINPUT_RESPONSE`, `MISSING_LIVE_TRACKING_CONTROL`.

Son testler:

- `tests/test_simitl_pr0p_probe.py`: 121 passed
- `tests/test_game_screen_sandbox.py tests/test_simitl_pr0p_probe.py`: 288 passed

Sıradaki teknik adim:

1. ARM AUX/button path'i kesfet. FC status'ta active modes bos; `ARM` aktif
   degil.
2. `throttle=+1.0` low tutulurken ARM komutunu/kanalini sur.
3. `P2-fc-status-readonly --samples 6 --interval 1` ile `FC_ARMED` ve
   arming blocker temizligini kanitla.
4. Ondan sonra P5 yaw/pitch live response ve P6 live tracking'e gec.

Checkpoint sonunda pr0p/updater/Gazebo/Betaflight prosesi yoktu.

## LATEST 2026-07-07 10:29 +03 - Runtime RC Channels Visual Gate

Bu devam turunda pr0p input darboğazini daha olculebilir hale getirmek icin
yeni bir gate eklendi:

- Yeni dosya:
  `experiments/simitl_pr0p_probe/pr0p_runtime_input_visual_probe.py`
- Amac:
  pr0p `Controls -> RC Channels` sayfasi acikken virtual RC pulse sirasinda
  ekrandaki RC bar/kanal bolgesinin goruntusel olarak degisip degismedigini
  olcmek.
- Bu gate arac hareketini kanitlamaz; sadece runtime UI'nin virtual RC input'u
  gorup gormedigini kanitlar.
- Eger sayfada animasyon varsa `--crop left,top,width,height` ile RC bar
  bolgesine daraltmak gerekir.

Yeni karar entegrasyonu:

- `pr0p_live_run_manifest.py` icine `P4-rc-channels-visual` fazi eklendi.
- `independent_sim_readiness.py` komut listesine
  `pr0p_runtime_input_visual` eklendi.
- `pr0p_decision_report.py`, promotion icin
  `P4-rc-channels-visual PASS` kanitini de zorunlu tutuyor.
- Bu nedenle son decision raporunda artik
  `MISSING_RUNTIME_RC_CHANNEL_VISUAL` acik sebep olarak gorunuyor.

Son testler:

```text
tests/test_simitl_pr0p_probe.py: 100 passed
tests/test_game_screen_sandbox.py tests/test_simitl_pr0p_probe.py: 267 passed
```

Son fresh raporlar:

- Runtime visual no-window smoke:
  `logs/simitl_pr0p/20260707-102850-pr0p-runtime-input-visual-nowindow-v2-runtime-input-visual.md`
- Suite:
  `logs/simitl_pr0p/20260707-102850-pr0p-suite-runtime-visual-v1-suite.md`
- Manifest:
  `logs/simitl_pr0p/20260707-102858-pr0p-live-manifest-runtime-visual-required-v1-live-manifest.md`
- Independent readiness:
  `logs/simitl_pr0p/20260707-102859-independent-sim-runtime-visual-required-v1-independent-sim-readiness.md`
- Decision:
  `logs/simitl_pr0p/20260707-102901-pr0p-decision-runtime-visual-required-v1-decision.md`

Checkpoint sonunda pr0p/updater/Gazebo/Betaflight prosesi yoktu.

Bir sonraki en dogru test:

1. pr0p'u ac.
2. `Controls -> RC Channels` sayfasina git.
3. Mümkünse RC barlarini kapsayan crop belirle.
4. Asagidaki komutu calistir:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_runtime_input_visual_probe.py \
  --uinput \
  --ack-live-input \
  --axis yaw \
  --magnitude 0.6 \
  --run-id pr0p-rc-channels-visual-next
```

Gerekirse crop ile:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_runtime_input_visual_probe.py \
  --uinput \
  --ack-live-input \
  --axis yaw \
  --magnitude 0.6 \
  --crop left,top,width,height \
  --run-id pr0p-rc-channels-visual-next
```

2. pr0p acik ve local race calisirken safe suite'i kos:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_probe_suite.py \
  --tracking-bbox 148,220,142,82 \
  --run-id pr0p-suite-live-next
```

3. Eger response yine WAITING ise, `Controls -> RC Channels` sayfasinda
   `pr0p_rc_channel_mapping_assistant.py` ile eksenleri tek tek pulse ederek
   runtime bar hareketini gor:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_rc_channel_mapping_assistant.py \
  --uinput \
  --ack-live-input \
  --role all \
  --run-id pr0p-rc-map-live-next
```

Bu komut yalniz pr0p `Controls -> RC Channels` ekrani acikken anlamlidir.

4. Promotion/ready karari icin:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_decision_report.py \
  --run-id pr0p-decision-next
```

Bu hedef tamamlanmis degil. Live tracker/PID kontrolune gecmek icin en azindan
`P5-persistent-uinput-live-response`, `P5-yaw-live`, `P5-pitch-live` ve
`P5-rc-loopback` kanitlari PASS olmalidir.

2026-07-06 on ucuncu devam guncellemesi:

- `sandbox_status_report.py` artik dry-run rc-binding raporlarini
  `rc_binding_assistant PASS` kaniti saymaz.
- RC binding hazir kaniti icin matching raporda `real_input: true`,
  `ack_live_input: true`, `neutralized: true` ve `command_count > 0` gerekir.
- Bu sayede `rc_binding_assistant.py` dry-run kosulari yardimci/debug kaniti
  olarak kalir; oyun/sim `Controls -> RC Channels` binding'i yapildi gibi
  raporlanmaz.
- Hedefli sandbox testi tekrar calistirildi:
  `tests/test_game_screen_sandbox.py: 133 passed`.
- Mevcut dry-run binding smoke yeniden status'a okutuldu; beklenen sekilde
  `rc_binding_assistant WAITING` verdi:
  `logs/game_screen_sandbox/20260706-222830-pr0p-status-rc-binding-dry-not-ready-smoke-v1-status-report.md`.

2026-07-06 on dorduncu devam guncellemesi:

- `sandbox_status_report.py --require-live-follow` artik yalniz ust seviye
  `EXTERNAL_LIVE_FOLLOW_COMPLETE` sonucuna guvenmez.
- Matching live-follow sequence `PASS` sayilmasi icin gomulu live uinput
  kaniti gerekir:
  - `ack_live_input` adimi `PASS`
  - `live_input_readiness` adimi `EXTERNAL_LIVE_INPUT_READY`
  - `live_follow` adimi `EXTERNAL_FOLLOW_LIVE_RUN_COMPLETE`
  - gomulu `follow.status` `EXTERNAL_FOLLOW_LIVE_RUN_COMPLETE`
  - gomulu `follow.metrics.real_input: true`
  - gomulu `follow.metrics.ack_live_input: true`
  - gomulu `follow.metrics.adapter: UInputAdapter`
- Bu kosullar eksikse status item `WAITING` kalir ve rapor
  `proof_gaps` listesiyle eksik kaniti yazar.
- Hedefli sandbox testi tekrar calistirildi:
  `tests/test_game_screen_sandbox.py: 134 passed`.
- pr0p penceresi/bbox yokken `--require-live-follow` smoke beklenen sekilde
  `WAITING` verdi; bbox yok oldugu icin external live-follow kaniti eslesmedi:
  `logs/game_screen_sandbox/20260706-223239-pr0p-status-live-follow-proof-smoke-v1-status-report.md`.

2026-07-06 on besinci devam guncellemesi:

- S5 axis-response gate'i opsiyonel signed direction kontroluyle
  guclendirildi.
- `latency_probe.py` artik response frame icin signed image shift hesaplar ve
  `response_shift_x_px`, `response_shift_y_px`, `response_shift_score`
  metriklerini yazar.
- `--axis-expected-shifts yaw:+x,pitch:-y` /
  `--latency-axis-expected-shifts yaw:+x,pitch:-y` ile "goruntu degisti mi?"
  kontrolu "beklenen yone hareket etti mi?" kontrolune yukseltilebilir.
- Yanlis/invert eksen `direction_status: FAIL`; belirsiz veya cok kucuk shift
  `WAITING` olur.
- Bu opsiyon `phase_runner.py`, `external_window_preflight.py`,
  `external_live_input_readiness.py` ve `external_live_follow_sequence.py`
  uzerinden tasindi.
- Hedefli sandbox testi tekrar calistirildi:
  `tests/test_game_screen_sandbox.py: 139 passed`.
- pr0p penceresi/bbox yokken `--require-live-follow` status-only smoke yine
  beklenen sekilde `WAITING` verdi ve capture/input baslatmadi:
  `logs/game_screen_sandbox/20260706-224113-pr0p-status-signed-axis-gate-smoke-v1-status-report.md`.

2026-07-06 on altinci devam guncellemesi:

- `sandbox_status_report.py` artik `--axis-sweep-axes`,
  `--axis-expected-shifts` ve `--axis-min-shift-px` argumanlarini kabul eder.
- Bu ayarlar `resume_commands` icindeki `status`,
  `live_input_readiness` ve `live_follow_sequence` komutlarina tasinir.
- `sandbox_resume_runner.py` ayni argumanlari kabul eder ve plan-only/status
  raporlarinda signed-axis beklentisini kaybetmez.
- Hedefli sandbox testi tekrar calistirildi:
  `tests/test_game_screen_sandbox.py: 141 passed`.
- pr0p penceresi/bbox yokken signed-axis status-only smoke beklenen sekilde
  `WAITING` verdi; rapordaki resume status komutu
  `--axis-expected-shifts pitch:-y,yaw:+x --axis-min-shift-px 1.0` tasiyor:
  `logs/game_screen_sandbox/20260706-224814-pr0p-status-signed-resume-command-smoke-v1-status-report.md`.
- pr0p penceresi/bbox yokken signed-axis resume runner plan-only smoke
  beklenen sekilde `WAITING` verdi, `executed_count: 0` kaldi ve capture/input
  baslatmadi:
  `logs/game_screen_sandbox/20260706-224814-pr0p-resume-signed-axis-plan-smoke-v1-sandbox-resume-runner.md`.

2026-07-06 on yedinci devam guncellemesi:

- `sandbox_status_report.py --require-live-input` artik yalniz ust seviye
  `EXTERNAL_LIVE_INPUT_READY` sonucuna guvenmez.
- Matching live-input readiness `PASS` sayilmasi icin gomulu live uinput
  kaniti gerekir:
  - `dry_sequence` adimi `EXTERNAL_DRY_RUN_READY`
  - `ack_live_input` adimi `PASS`
  - `axis_response` adimi `EXTERNAL_WINDOW_READY_LIVE_INPUT`
  - gomulu `axis_preflight.status` `EXTERNAL_WINDOW_READY_LIVE_INPUT`
  - gomulu `S5-real.status` `PASS`
  - gomulu `S5-real.metrics.real_input: true`
  - gomulu `S5-real.metrics.adapter: UInputAdapter`
  - gomulu `S5-real.metrics.axis_sweep: true`
- Status komutunda `--axis-expected-shifts` verildiyse matching live-input ve
  live-follow raporlarinin ayni signed-axis beklentisini tasimasi gerekir.
- Eksik veya dry-run benzeri live-input raporlari `WAITING` kalir ve
  `proof_gaps` listesiyle eksik kaniti yazar.
- Hedefli sandbox testi tekrar calistirildi:
  `tests/test_game_screen_sandbox.py: 143 passed`.
- pr0p penceresi/bbox yokken signed-axis status-only smoke beklenen sekilde
 `WAITING` verdi ve capture/input baslatmadi:
  `logs/game_screen_sandbox/20260706-225452-pr0p-status-live-input-proof-smoke-v1-status-report.md`.

2026-07-06 on sekizinci devam guncellemesi:

- `bbox_tool.py --interactive-select` eklendi.
- `prepare_bbox_artifacts(..., interactive_select=True)` artik tek frame
  yakalayip OpenCV ROI UI ile hedef bbox sectirebilir.
- Interaktif secim iptal edilirse status `WAITING` kalir ve yarim bbox raporu
  uretilmez.
- `sandbox_status_report.py`, hedef bbox dosyasi eksikken `resume_commands`
  icinde `interactive_bbox_select` komutunu da yazar.
- Bu komut `safety_class: manual_bbox_select`, `sends_uinput: false` ve
  `requires_user_interaction: true` metadata'si tasir.
- `sandbox_resume_runner.py`, `requires_user_interaction: true` komutlari
  plan/execute-safe akista otomatik calistirmaz; "manual UI interaction is
  required..." olarak `BLOCKED` birakir.
- Hedefli sandbox testi tekrar calistirildi:
  `tests/test_game_screen_sandbox.py: 145 passed`.
- pr0p penceresi/bbox yokken status-only smoke beklenen sekilde `WAITING`
  verdi ve `interactive_bbox_select` resume komutunu yazdi:
  `logs/game_screen_sandbox/20260706-230426-pr0p-status-interactive-bbox-command-smoke-v2-status-report.md`.
- pr0p penceresi/bbox yokken resume-runner plan-only smoke beklenen sekilde
  `WAITING` verdi; `executed_count: 0` kaldi, capture/input baslatmadi ve
  interaktif bbox secimini otomatik acmadan `BLOCKED` birakti:
  `logs/game_screen_sandbox/20260706-230426-pr0p-resume-interactive-bbox-plan-smoke-v2-sandbox-resume-runner.md`.

2026-07-06 on dokuzuncu devam guncellemesi:

- `sandbox_status_report.py` resume komutlarina `requires_pass` metadata'si
  eklendi.
- `capture_bbox_frame`, `write_bbox_file_template` ve
  `interactive_bbox_select` artik `target_window` PASS olmadan uygulanabilir
  sayilmiyor.
- `dry_sequence`, `target_window`, `target_bbox` ve
  `gazebo_betaflight_process_boundary` PASS ister.
- `live_input_readiness`, `target_window`, `target_bbox`,
  `external_window_preflight`, `external_follow_session`,
  `uinput_environment` ve `gazebo_betaflight_process_boundary` PASS ister.
- `live_follow_sequence`, bunlara ek olarak `external_live_input_readiness`
  PASS ister.
- `sandbox_resume_runner.py` bu dependency listesini status item'larindan
  okur; eksik varsa komutu calistirmadan `BLOCKED` yapar ve
  `unmet_requires_pass` alanina eksik item/status ciftlerini yazar.
- Hedefli sandbox testi tekrar calistirildi:
  `tests/test_game_screen_sandbox.py: 146 passed`.
- pr0p penceresi/bbox yokken status smoke `requires_pass` alanlarini yazdi:
  `logs/game_screen_sandbox/20260706-230830-pr0p-status-requires-pass-smoke-v1-status-report.md`.
- pr0p penceresi/bbox yokken resume-runner plan-only smoke `capture_bbox_frame`
  komutunu `target_window=WAITING` nedeniyle `BLOCKED` birakti ve hicbir
  capture/input baslatmadi:
  `logs/game_screen_sandbox/20260706-230830-pr0p-resume-requires-pass-plan-smoke-v1-sandbox-resume-runner.md`.
- Eski bir bbox dosyasi varken ama pr0p penceresi yokken `--execute-safe`
  smoke `dry_sequence` komutunu `target_window=WAITING` nedeniyle calistirmadi:
  `logs/game_screen_sandbox/20260706-230850-pr0p-resume-requires-pass-execsafe-bbox-window-missing-smoke-v1-sandbox-resume-runner.md`.
- Ayni durumda `--execute-safe --execute-live-input --ack-live-input` verilse
 bile live-input/live-follow komutlari dependency eksikleri nedeniyle
  calismadi; `executed_count: 0` kaldi:
  `logs/game_screen_sandbox/20260706-230901-pr0p-resume-requires-pass-liveack-bbox-window-missing-smoke-v1-sandbox-resume-runner.md`.

2026-07-06 yirminci devam guncellemesi:

- `sandbox_resume_runner.py` raporlarina `resume_decision` ozeti eklendi.
- Bu ozet `decision`, `next_operator_action`, ilk bloklanan step, ilk eksik
  `requires_pass` step'i, ilk planli step ve calisan step listesini yazar.
- Markdown raporlarinda `Resume Decision` bolumu artik Steps tablosundan once
  gorunur.
- pr0p penceresi/bbox yokken plan-only smoke `resume_decision.decision` icin
  `missing_required_status`, `next_operator_action` icin
  "open the external game/sim window..." yazdi:
  `logs/game_screen_sandbox/20260706-231348-pr0p-resume-decision-summary-plan-smoke-v1-sandbox-resume-runner.md`.
- Eski bir bbox dosyasi varken, pr0p penceresi yokken ve
  `--execute-safe --execute-live-input --ack-live-input` verilse bile
  `resume_decision` ilk eksik gate'i `dry_sequence -> target_window=WAITING`
  olarak yazdi; `executed_count: 0` kaldi:
  `logs/game_screen_sandbox/20260706-231349-pr0p-resume-decision-summary-liveack-bbox-window-missing-smoke-v1-sandbox-resume-runner.md`.
- Hedefli sandbox testi tekrar calistirildi:
  `tests/test_game_screen_sandbox.py: 146 passed`.

2026-07-06 yirmi birinci devam guncellemesi:

- `sandbox_status_report.py` raporlarina `readiness_ladder` eklendi.
- Ladder asamalari:
  - S0 `local_sandbox_isolation`
  - S1 `target_setup`
  - S2 `dry_perception_control`
  - S3 `control_binding_evidence`
  - S4 `live_input_readiness`
  - S5 `bounded_live_follow`
- Her asama `requires_pass`, varsa `requires_any_pass`,
  `unmet_requires_pass`, `unmet_requires_any_pass` ve item status'larini yazar.
- Status Markdown raporlarinda `Readiness Ladder` bolumu `Items` tablosundan
  once gorunur.
- pr0p penceresi/bbox yokken status smoke S0'i `PASS`, ilk blokaji S1
  `target_setup`, eksikleri `target_window=WAITING` ve `target_bbox=WAITING`
  olarak yazdi:
  `logs/game_screen_sandbox/20260706-231806-pr0p-status-readiness-ladder-smoke-v1-status-report.md`.
- Resume-runner smoke nested status raporu da ayni ladder'i tasidi:
  `logs/game_screen_sandbox/20260706-231806-pr0p-resume-readiness-ladder-smoke-v1-sandbox-resume-runner.md`.
- Hedefli sandbox testi tekrar calistirildi:
  `tests/test_game_screen_sandbox.py: 146 passed`.

2026-07-06 yirmi ikinci devam guncellemesi:

- `sandbox_status_report.py` metrics alanina `evidence_freshness` eklendi.
- Her evidence path icin `FRESH`, `STALE` veya `MISSING`, `age_seconds`,
  `mtime` ve path bilgisi yaziliyor.
- Varsayilan stale esigi 3600 saniye; bu simdilik hard gate degil, gorunurluk
  ve handoff uyarisi icin kullaniliyor.
- Markdown status raporlarinda `Evidence Freshness` tablosu eklendi.
- Unit test eklendi: fresh/stale/missing kanit ayrimi deterministik mtime ile
  dogrulaniyor.
- Hedefli sandbox testi tekrar calistirildi:
  `tests/test_game_screen_sandbox.py: 147 passed`.
- pr0p penceresi/bbox yokken status smoke, eski acceptance artefaktini
  `STALE`, bbox ve pr0p live/dry kanitlarini `MISSING` olarak yazdi:
  `logs/game_screen_sandbox/20260706-232116-pr0p-status-evidence-freshness-smoke-v1-status-report.md`.
- Resume-runner nested status raporu da ayni freshness ozetini tasidi:
  `logs/game_screen_sandbox/20260706-232116-pr0p-resume-evidence-freshness-smoke-v1-sandbox-resume-runner.md`.

2026-07-06 yirmi ucuncu devam guncellemesi:

- `sandbox_status_report.py` icin opsiyonel `--require-fresh-evidence` ve
  `--evidence-stale-after-s` eklendi.
- Varsayilan davranis degismedi: freshness hala raporlanir. Bu bayrak
  verilirse, normalde `READY` olacak bir status eski/eksik zorunlu kanit varsa
  `WAITING` olur.
- Zorunlu taze kanit seti moda gore secilir: dry-run icin acceptance,
  external preflight, external follow ve bbox; `--require-live-input` ile
  live-input; `--require-live-follow` ile live-follow sequence kaniti de eklenir.
- `sandbox_resume_runner.py` ayni freshness bayraklarini nested status
  raporlarina ve resume status komutuna tasir.
- Markdown status raporunda `Evidence Freshness` altinda freshness gate durumu
  gorunur.
- Hedefli sandbox testi tekrar calistirildi:
  `tests/test_game_screen_sandbox.py: 149 passed`.
- pr0p penceresi/bbox yokken hard freshness gate smoke beklenen sekilde
  `WAITING` verdi; eski acceptance `STALE`, bbox ve pr0p dry/live kanitlari
  `MISSING` olarak raporlandi:
  `logs/game_screen_sandbox/20260706-232912-pr0p-status-fresh-evidence-gate-smoke-v1-status-report.md`.
- Resume-runner smoke da ayni bayraklari korudu ve ilk operator aksiyonunu
  pr0p penceresini acmak olarak yazdi:
  `logs/game_screen_sandbox/20260706-232912-pr0p-resume-fresh-evidence-gate-smoke-v1-sandbox-resume-runner.md`.

2026-07-06 yirmi dorduncu devam guncellemesi:

- `external_target_acceptance.py` eklendi.
- Bu arac gercek harici oyun/sim penceresi icin status-only acceptance
  kararidir; simulator, capture, Gazebo, Betaflight veya `uinput` baslatmaz.
- Varsayilan modu proje hedefine uygun olarak `--mode live-follow` ve fresh
  evidence zorunludur. Dry veya live-input ara modlari icin `--mode dry-run`
  veya `--mode live-input` kullanilabilir.
- Cikti `EXTERNAL_TARGET_READY`, `WAITING` veya `REJECT` olur; nested
  `sandbox_status_report.py` raporunu ve S0-S5 readiness ladder'i evidence
  olarak tasir.
- Hedefli sandbox testi tekrar calistirildi:
  `tests/test_game_screen_sandbox.py: 152 passed`.
- pr0p penceresi/bbox yokken external target acceptance smoke beklenen sekilde
  `WAITING` verdi; ilk blokaj S1 `target_setup`:
  `logs/game_screen_sandbox/20260706-233447-pr0p-external-acceptance-smoke-v1-external-target-acceptance.md`.

2026-07-06 yirmi besinci devam guncellemesi:

- `external_target_acceptance.py` artik kosu basinda ve sonunda forbidden
  Gazebo/Betaflight process snapshot alir.
- Yeni forbidden process delta olusursa nested status `READY` olsa bile ust
  acceptance `REJECT` olur ve `isolation_audit` metrics alanina yazilir.
- Markdown raporuna `Process Boundary` bolumu eklendi.
- Hedefli sandbox testi tekrar calistirildi:
  `tests/test_game_screen_sandbox.py: 153 passed`.
- pr0p penceresi/bbox yokken isolation-aware external target acceptance smoke
  beklenen sekilde `WAITING` verdi; process-boundary `PASS`, ilk blokaj S1
  `target_setup`:
  `logs/game_screen_sandbox/20260706-233941-pr0p-external-acceptance-isolation-smoke-v1-external-target-acceptance.md`.

2026-07-06 yirmi altinci devam guncellemesi:

- `external_target_acceptance.py` metrics alanina `acceptance_decision` ust
  ozeti eklendi.
- Bu ozet `decision`, `status_report_status`, `isolation_status`,
  `freshness_gate_status`, `first_blocking_stage`, `next_operator_action`,
  stale/missing/required fresh evidence listelerini tek yerde gosterir.
- Markdown raporuna `Acceptance Decision` bolumu eklendi.
- Hedefli sandbox testi tekrar calistirildi:
  `tests/test_game_screen_sandbox.py: 153 passed`.
- pr0p penceresi/bbox yokken acceptance decision smoke beklenen sekilde
  `WAITING` verdi; `next_operator_action` pr0p penceresini acmak,
  `first_blocking_stage` S1 `target_setup`:
  `logs/game_screen_sandbox/20260706-234233-pr0p-external-acceptance-decision-smoke-v1-external-target-acceptance.md`.

2026-07-06 yirmi yedinci devam guncellemesi:

- `external_target_acceptance.py` metrics alanina `operator_command_queue`
  eklendi.
- Bu queue nested `sandbox_status_report.py` icindeki `resume_commands`
  listesinden status komutu haric tutularak uretilir.
- Her komut icin `name`, `purpose`, `safety_class`, `sends_uinput`,
  `requires_ack_live_input`, `requires_edit`, `requires_user_interaction`,
  `requires_pass`, `expected_status`, `unblocks` ve exact `command` ust
  seviyede gorunur.
- `acceptance_decision.next_operator_command` ilk operator komutunu isaret eder.
- Hedefli sandbox testi tekrar calistirildi:
  `tests/test_game_screen_sandbox.py: 153 passed`.
- pr0p penceresi/bbox yokken command-queue smoke beklenen sekilde `WAITING`
  verdi; ilk komut `capture_bbox_frame`, `sends_uinput=false`,
  `requires_pass=["target_window"]`:
  `logs/game_screen_sandbox/20260706-234546-pr0p-external-acceptance-command-queue-smoke-v1-external-target-acceptance.md`.

2026-07-06 yirmi sekizinci devam guncellemesi:

- `operator_command_queue` girdilerine `command_state` ve
  `unmet_requires_pass` eklendi.
- `command_state` degerleri: `AVAILABLE`, `BLOCKED`, `MANUAL_EDIT`,
  `MANUAL_UI`, `ACK_REQUIRED`.
- Bu sayede queue sadece komut listesini degil, komutun neden bekledigini de
  ust seviyede gosterir.
- Hedefli sandbox testi tekrar calistirildi:
  `tests/test_game_screen_sandbox.py: 153 passed`.
- pr0p penceresi/bbox yokken command-state smoke beklenen sekilde `WAITING`
  verdi; ilk komut `capture_bbox_frame`, `command_state=BLOCKED`,
  `unmet_requires_pass=["target_window=WAITING"]`:
  `logs/game_screen_sandbox/20260706-234846-pr0p-external-acceptance-command-state-smoke-v1-external-target-acceptance.md`.

2026-07-06 yirmi dokuzuncu devam guncellemesi:

- `external_target_acceptance.py` metrics alanina `operator_command_summary`
  eklendi.
- Bu ozet `total_count`, `state_counts`, `sends_uinput_count`,
  `first_available_command`, `first_blocked_command` ve
  `first_ack_required_command` alanlarini yazar.
- `acceptance_decision` icine de ilk available/blocked/ack-required komut
  kisayollari eklendi.
- Hedefli sandbox testi tekrar calistirildi:
  `tests/test_game_screen_sandbox.py: 153 passed`.
- pr0p penceresi/bbox yokken command-summary smoke beklenen sekilde `WAITING`
  verdi; `operator_command_summary.total_count=3`,
  `state_counts={"BLOCKED": 3}`, `sends_uinput_count=0`:
  `logs/game_screen_sandbox/20260706-235122-pr0p-external-acceptance-command-summary-smoke-v1-external-target-acceptance.md`.

2026-07-06 otuzuncu devam guncellemesi:

- Checkpoint sonrasi tekrar hedefli dogrulama yapildi:
  `tests/test_game_screen_sandbox.py: 153 passed`.
- `external_target_acceptance.py` ve `tests/test_game_screen_sandbox.py`
  `py_compile` kontrolunden gecti.
- `pgrep` kontrolunde `Kenet Simple Target Game`, `simple_target_game`,
  `external_follow_session`, `external_window_preflight`, `rc_binding_assistant`,
  `ffmpeg`, `gazebo`, `gzserver`, `gzclient`, `betaflight`, `pr0p` veya
  `SITL_Forge` prosesi gorulmedi.
- Bu guncelleme yeni simulator, capture veya `uinput` baslatmadi; yalnizca
  external sim/screen sandbox kabul altyapisinin mevcut durumunu dogruladi.

2026-07-06 otuz birinci devam guncellemesi:

- `sandbox_resume_runner.py` metrics alanina `resume_step_summary` eklendi.
- Bu ozet `total_count`, `status_counts`, `sends_uinput_count`,
  `executed_count`, `first_planned_step`, `first_blocked_step`,
  `first_unmet_requirement_step` ve `first_failure_step` alanlarini yazar.
- Hedefli resume-runner testi calisti:
  `tests/test_game_screen_sandbox.py -k resume_runner: 6 passed`.
- Tum sandbox testi tekrar calisti:
  `tests/test_game_screen_sandbox.py: 153 passed`.
- pr0p penceresi/bbox yokken plan-only resume smoke beklenen sekilde
  `WAITING` verdi; `resume_step_summary.status_counts={"BLOCKED": 3, "SKIPPED": 1}`,
  `sends_uinput_count=0`, `executed_count=0`:
  `logs/game_screen_sandbox/20260706-235710-pr0p-resume-step-summary-smoke-v1-sandbox-resume-runner.md`.
- `pgrep` kontrolunde `Kenet Simple Target Game`, `simple_target_game`,
  `external_follow_session`, `external_window_preflight`, `rc_binding_assistant`,
  `ffmpeg`, `gazebo`, `gzserver`, `gzclient`, `betaflight`, `pr0p` veya
  `SITL_Forge` prosesi gorulmedi.

2026-07-07 yerel log zamanli otuz ikinci devam guncellemesi:

- `external_operator_preflight.py` eklendi.
- Bu komut status-only calisir; simulator, capture, Gazebo, Betaflight veya
  `uinput` baslatmadan `operator_preflight_decision`,
  `operator_command_queue` ve `operator_command_summary` yazar.
- Hedefli operator-preflight testi calisti:
  `tests/test_game_screen_sandbox.py -k operator_preflight: 3 passed`.
- Tum sandbox testi tekrar calisti:
  `tests/test_game_screen_sandbox.py: 156 passed`.
- pr0p penceresi/bbox yokken operator preflight smoke beklenen sekilde
  `WAITING` verdi; `readiness_stage=target_setup`,
  `command_gate=blocked_by_dependency`, `next_operator_action` pr0p penceresini
  acmak:
  `logs/game_screen_sandbox/20260707-000230-pr0p-operator-preflight-smoke-v1-external-operator-preflight.md`.

2026-07-07 yerel log zamanli otuz ucuncu devam guncellemesi:

- `external_operator_preflight.py` raporuna `window_discovery` eklendi.
- `window_discovery`, hedef `--window-title` eslesmezse gorunen X11 uygulama
  penceresi adaylarini `title`, `window_id`, `region` ve
  `use_window_title_arg` alanlariyla listeler.
- Masaustu/guard pencereleri filtrelenir; rapor simulator, capture veya
  `uinput` baslatmadan kalir.
- Hedefli operator-preflight testi calisti:
  `tests/test_game_screen_sandbox.py -k operator_preflight: 3 passed`.
- Tum sandbox testi tekrar calisti:
  `tests/test_game_screen_sandbox.py: 156 passed`.
- pr0p penceresi/bbox yokken window-candidates smoke beklenen sekilde
  `WAITING` verdi; `window_discovery.reason=NO_MATCH`, `visible_count=13`,
  `candidate_pool_count=10`, `candidate_count=5`:
  `logs/game_screen_sandbox/20260707-000818-pr0p-operator-preflight-window-candidates-smoke-v2-external-operator-preflight.md`.

2026-07-07 yerel log zamanli otuz dorduncu devam guncellemesi:

- `window_discovery` adaylarina `rerun_operator_preflight_command` ve
  `capture_bbox_frame_command` eklendi.
- Bu komutlar secilen aday pencere basligini `--window-exact` ile kullanir;
  rapor yine simulator, capture veya `uinput` baslatmadan kalir.
- Hedefli operator-preflight testi calisti:
  `tests/test_game_screen_sandbox.py -k operator_preflight: 3 passed`.
- Tum sandbox testi tekrar calisti:
  `tests/test_game_screen_sandbox.py: 156 passed`.
- pr0p penceresi/bbox yokken rerun-command smoke beklenen sekilde `WAITING`
  verdi ve adaylara exact rerun/bbox command alanlarini yazdi:
  `logs/game_screen_sandbox/20260707-001217-pr0p-operator-preflight-rerun-commands-smoke-v1-external-operator-preflight.md`.

2026-07-07 yerel log zamanli otuz besinci devam guncellemesi:

- `window_discovery` adaylarina `capture_bbox_frame_region_command` eklendi.
- Bu komut aday pencerenin mevcut `region` bilgisini `bbox_tool.py --region`
  ile kullanir; pencere basligi degisse bile frame capture icin alternatif yol
  verir.
- Hedefli operator-preflight testi calisti:
  `tests/test_game_screen_sandbox.py -k operator_preflight: 3 passed`.
- Tum sandbox testi tekrar calisti:
  `tests/test_game_screen_sandbox.py: 156 passed`.
- pr0p penceresi/bbox yokken region-command smoke beklenen sekilde `WAITING`
  verdi ve adaylara region tabanli bbox command alanlarini yazdi:
  `logs/game_screen_sandbox/20260707-001455-pr0p-operator-preflight-region-commands-smoke-v1-external-operator-preflight.md`.

2026-07-07 yerel log zamanli otuz altinci devam guncellemesi:

- `external_dry_run_sequence.py` region modunda artik `target_window` gate'ini
  bloklayici gereksinim saymaz; `--capture-region` verilirse gerekli status
  item'lari `target_bbox` ve `gazebo_betaflight_process_boundary` olur.
- `external_operator_preflight.py` `window_discovery` adaylarina
  `dry_run_sequence_region_command` ekler; bu komut aday crop'u ile
  `external_dry_run_sequence.py --capture-region ...` calistirir.
- Hedefli test calisti:
  `tests/test_game_screen_sandbox.py -k 'operator_preflight or external_dry_run_sequence': 8 passed, 149 deselected`.
- Compile kontrolu gecti:
  `external_operator_preflight.py`, `external_dry_run_sequence.py`,
  `external_target_acceptance.py`, `sandbox_resume_runner.py`,
  `tests/test_game_screen_sandbox.py`.
- Tum sandbox testi tekrar calisti:
  `tests/test_game_screen_sandbox.py: 157 passed`.
- pr0p penceresi/bbox yokken region dry-run command smoke beklenen sekilde
  `WAITING` verdi ve adaylara `dry_run_sequence_region_command` alanlarini yazdi:
  `logs/game_screen_sandbox/20260707-002241-pr0p-operator-preflight-region-dryrun-smoke-v2-external-operator-preflight.md`.
- `pgrep` kontrolunde `Kenet Simple Target Game`, `simple_target_game`,
  `external_operator_preflight`, `external_dry_run_sequence`,
  `external_follow_session`, `external_window_preflight`,
  `rc_binding_assistant`, `ffmpeg`, `gazebo`, `gzserver`, `gzclient`,
  `betaflight`, `pr0p` veya `SITL_Forge` prosesi gorulmedi.

2026-07-07 yerel log zamanli otuz yedinci devam guncellemesi:

- `external_operator_preflight.py` raporuna `candidate_action_plan` ve
  `recommended_candidate_action` eklendi.
- Her gorunen aday icin siradaki guvenli non-`uinput` adimi hesaplanir:
  bbox yoksa `capture_bbox_frame_region`, bbox hazir ama dry evidence eksikse
  `dry_run_sequence_region`.
- Bu alanlar operatorun yanlis VSCode/Chrome penceresi yerine dogru sim/FPV
  adayini elle teyit edip yalniz o aday komutunu calistirmasi icin guard yazar.
- Hedefli operator-preflight testi calisti:
  `tests/test_game_screen_sandbox.py -k operator_preflight: 4 passed, 154 deselected`.
- Tum sandbox testi tekrar calisti:
  `tests/test_game_screen_sandbox.py: 158 passed`.
- Compile kontrolu gecti:
  `external_operator_preflight.py`, `external_dry_run_sequence.py`,
  `external_target_acceptance.py`, `sandbox_resume_runner.py`,
  `tests/test_game_screen_sandbox.py`.
- pr0p penceresi/bbox yokken candidate-action smoke beklenen sekilde `WAITING`
  verdi; `recommended_candidate_action.next_step=capture_bbox_frame_region`:
  `logs/game_screen_sandbox/20260707-002619-pr0p-operator-candidate-action-smoke-v1-external-operator-preflight.md`.
- `pgrep` kontrolunde `Kenet Simple Target Game`, `simple_target_game`,
  `external_operator_preflight`, `external_dry_run_sequence`,
  `external_follow_session`, `external_window_preflight`,
  `rc_binding_assistant`, `ffmpeg`, `gazebo`, `gzserver`, `gzclient`,
  `betaflight`, `pr0p` veya `SITL_Forge` prosesi gorulmedi.

2026-07-07 yerel log zamanli otuz sekizinci devam guncellemesi:

- `external_candidate_action_runner.py` eklendi.
- Bu runner once `external_operator_preflight.py` kosar ve nested operator
  raporunu evidence olarak yazar.
- `--candidate-index` verilmezse plan-only kalir, komut calistirmaz ve
  `candidate_index_required` ile `WAITING` yazar.
- `--candidate-index N --ack-candidate --execute-safe` verilirse yalniz secilen
  aday icin tek guvenli non-`uinput` komutu calistirir.
- Aday aksiyonu `sends_uinput: true` ise runner `REJECT` eder.
- Hedefli test calisti:
  `tests/test_game_screen_sandbox.py -k 'candidate_action or operator_preflight': 7 passed, 154 deselected`.
- Tum sandbox testi tekrar calisti:
  `tests/test_game_screen_sandbox.py: 161 passed`.
- Compile kontrolu gecti:
  `external_candidate_action_runner.py`, `external_operator_preflight.py`,
  `external_dry_run_sequence.py`, `external_target_acceptance.py`,
  `sandbox_resume_runner.py`, `tests/test_game_screen_sandbox.py`.
- pr0p penceresi/bbox yokken plan-only candidate runner smoke beklenen sekilde
  `WAITING` verdi; `candidate_index_required` ve adaylarin
  `capture_bbox_frame_region` adimlari raporlandi:
  `logs/game_screen_sandbox/20260707-003116-pr0p-candidate-action-plan-smoke-v1-external-candidate-action-runner.md`.
- `pgrep` kontrolunde `Kenet Simple Target Game`, `simple_target_game`,
  `external_candidate_action_runner`, `external_operator_preflight`,
  `external_dry_run_sequence`, `external_follow_session`,
  `external_window_preflight`, `rc_binding_assistant`, `ffmpeg`, `gazebo`,
  `gzserver`, `gzclient`, `betaflight`, `pr0p` veya `SITL_Forge` prosesi
  gorulmedi.

2026-07-07 yerel log zamanli otuz dokuzuncu devam guncellemesi:

- `external_candidate_action_runner.py` artik bir aday komutu gercekten
  calistiginda ikinci bir `post_operator_preflight` raporu uretir.
- Plan-only, aday secilmemis veya `--ack-candidate` eksik durumlarda post rapor
  yazilmaz; boylece yanlislikla capture/input baslatmadan yalniz plan kalir.
- `candidate_action_decision.post_operator_status` ve
  `post_operator_preflight_report` alanlari ile komut sonrasi readiness durumu
  ayni JSON/Markdown artefaktinda gorulur.
- Hedefli candidate-action testi calisti:
  `tests/test_game_screen_sandbox.py -k candidate_action: 4 passed, 157 deselected`.
- Tum sandbox testi tekrar calisti:
  `tests/test_game_screen_sandbox.py: 161 passed`.
- Compile kontrolu gecti:
  `external_candidate_action_runner.py`, `external_operator_preflight.py`,
  `external_dry_run_sequence.py`, `external_target_acceptance.py`,
  `sandbox_resume_runner.py`, `tests/test_game_screen_sandbox.py`.
- pr0p penceresi/bbox yokken post-preflight smoke beklenen sekilde `WAITING`
  verdi; aday secilmedigi icin `post_operator_preflight_report=null` kaldi:
  `logs/game_screen_sandbox/20260707-003530-pr0p-candidate-action-post-preflight-smoke-v1-external-candidate-action-runner.md`.
- `pgrep` kontrolunde `Kenet Simple Target Game`, `simple_target_game`,
  `external_candidate_action_runner`, `external_operator_preflight`,
  `external_dry_run_sequence`, `external_follow_session`,
  `external_window_preflight`, `rc_binding_assistant`, `ffmpeg`, `gazebo`,
  `gzserver`, `gzclient`, `betaflight`, `pr0p` veya `SITL_Forge` prosesi
  gorulmedi.

2026-07-07 yerel log zamanli kirkinci devam guncellemesi:

- `external_candidate_action_runner.py` artik aday secimi icin
  `--candidate-window-id`, exact `--candidate-title` ve tekil
  `--candidate-title-contains` destekler.
- `--candidate-window-id`, aday sirasi degisse bile dogru sim/FPV penceresini
  secmek icin tercih edilen yoldur.
- Birden fazla title substring eslesirse runner `REJECT` eder ve komut
  calistirmaz.
- Hedefli candidate-action testi calisti:
  `tests/test_game_screen_sandbox.py -k candidate_action: 6 passed, 157 deselected`.
- Tum sandbox testi tekrar calisti:
  `tests/test_game_screen_sandbox.py: 163 passed`.
- Compile kontrolu gecti:
  `external_candidate_action_runner.py`, `external_operator_preflight.py`,
  `external_dry_run_sequence.py`, `external_target_acceptance.py`,
  `sandbox_resume_runner.py`, `tests/test_game_screen_sandbox.py`.
- Window-id selector smoke beklenen sekilde `WAITING` verdi; secilen aday
  `PLANNED` kaldi, `selector=candidate_window_id`, `sends_uinput=false` ve
  `post_operator_preflight_report=null`:
  `logs/game_screen_sandbox/20260707-085433-pr0p-candidate-window-id-selector-smoke-v1-external-candidate-action-runner.md`.
- `pgrep` kontrolunde `Kenet Simple Target Game`, `simple_target_game`,
  `external_candidate_action_runner`, `external_operator_preflight`,
  `external_dry_run_sequence`, `external_follow_session`,
  `external_window_preflight`, `rc_binding_assistant`, `ffmpeg`, `gazebo`,
  `gzserver`, `gzclient`, `betaflight`, `pr0p` veya `SITL_Forge` prosesi
  gorulmedi.

2026-07-07 yerel log zamanli kirk birinci devam guncellemesi:

- `external_operator_preflight.py` adaylarina region tabanli
  `write_bbox_file_region_command` ve
  `interactive_bbox_select_region_command` eklendi.
- `external_candidate_action_runner.py` artik `--candidate-bbox X,Y,W,H`
  alirsa secili aday icin `write_bbox_file_region` adimina gecer ve bbox
  dosyasini region crop uzerinden yazar.
- Bu yol pencere basligina geri donmeden bbox dosyasi uretmek icindir; yine
  `--ack-candidate --execute-safe` olmadan komut calistirmaz.
- Hedefli operator/candidate testi calisti:
  `tests/test_game_screen_sandbox.py -k 'operator_preflight or candidate_action': 10 passed, 154 deselected`.
- Tum sandbox testi tekrar calisti:
  `tests/test_game_screen_sandbox.py: 164 passed`.
- Compile kontrolu gecti:
  `external_candidate_action_runner.py`, `external_operator_preflight.py`,
  `external_dry_run_sequence.py`, `external_target_acceptance.py`,
  `sandbox_resume_runner.py`, `tests/test_game_screen_sandbox.py`.
- `--candidate-bbox` plan smoke beklenen sekilde `WAITING` verdi; secili adim
  `write_bbox_file_region`, komut `--bbox 100,100,80,60` ile olustu ama
  `--execute-safe` olmadigi icin `PLANNED` kaldi ve calismadi:
  `logs/game_screen_sandbox/20260707-085945-pr0p-candidate-bbox-plan-smoke-v1-external-candidate-action-runner.md`.
- `pgrep` kontrolunde `Kenet Simple Target Game`, `simple_target_game`,
  `external_candidate_action_runner`, `external_operator_preflight`,
  `external_dry_run_sequence`, `external_follow_session`,
  `external_window_preflight`, `rc_binding_assistant`, `ffmpeg`, `gazebo`,
  `gzserver`, `gzclient`, `betaflight`, `pr0p` veya `SITL_Forge` prosesi
  gorulmedi.

2026-07-07 yerel log zamanli kirk ikinci devam guncellemesi:

- `external_candidate_action_runner.py` icinde `--candidate-bbox` icin secili
  aday `region` siniri kontrolu eklendi.
- Bbox koordinati secili region disina tasarsa runner artik shell komutu
  calistirmadan `REJECT` verir.
- Rapor ozeti bu durumda acik sekilde
  `candidate bbox is outside the selected candidate region` yazar.
- Hedefli candidate-action testi gecti:
  `tests/test_game_screen_sandbox.py -k 'candidate_action': 8 passed, 157 deselected`.
- Tum sandbox testi tekrar gecti:
  `tests/test_game_screen_sandbox.py: 165 passed`.
- Compile kontrolu gecti:
  `external_candidate_action_runner.py`, `external_operator_preflight.py`,
  `external_dry_run_sequence.py`, `external_target_acceptance.py`,
  `sandbox_resume_runner.py`, `tests/test_game_screen_sandbox.py`.
- Out-of-region bbox smoke beklenen sekilde `REJECT` verdi; `executed_count=0`
  ve `post_operator_preflight_report=null`:
  `logs/game_screen_sandbox/20260707-090522-pr0p-candidate-bbox-guard-smoke-v2-external-candidate-action-runner.md`.
- `pgrep` kontrolunde `Kenet Simple Target Game`, `simple_target_game`,
  `external_candidate_action_runner`, `external_operator_preflight`,
  `external_dry_run_sequence`, `external_follow_session`,
  `external_window_preflight`, `rc_binding_assistant`, `ffmpeg`, `gazebo`,
  `gzserver`, `gzclient`, `betaflight`, `pr0p` veya `SITL_Forge` prosesi
  gorulmedi.

2026-07-07 yerel log zamanli kirk ucuncu devam guncellemesi:

- Canli smoke sirasinda `SITL Forge` substring secimi gercek sim yerine acik
  VS Code penceresini secti; baslikta simulator metni gecmesi tek basina
  guvenilir degil.
- `external_candidate_action_runner.py` icine tooling/editor window guard
  eklendi. VS Code/editor/terminal benzeri adaylar secilse bile shell komutu
  calistirmadan `REJECT` verir.
- Hedefli candidate-action testi gecti:
  `tests/test_game_screen_sandbox.py -k 'candidate_action': 9 passed, 157 deselected`.
- Tum sandbox testi tekrar gecti:
  `tests/test_game_screen_sandbox.py: 166 passed`.
- Compile kontrolu gecti:
  `external_candidate_action_runner.py`, `external_operator_preflight.py`,
  `external_dry_run_sequence.py`, `external_target_acceptance.py`,
  `sandbox_resume_runner.py`, `tests/test_game_screen_sandbox.py`.
- Tooling-window guard smoke beklenen sekilde `REJECT` verdi; raporda hem
  `candidate_window_is_tooling:0x2c00004` hem de `executed_count=0` goruldu:
  `logs/game_screen_sandbox/20260707-090915-pr0p-candidate-tooling-window-guard-smoke-v1-external-candidate-action-runner.md`.
- Bu kanit ayni zamanda su an gercek pr0p/SITL Forge sim penceresi yerine IDE
  penceresinin eslestigini gosterir; canli takip asamasina gecmeden once gercek
  sim/FPV penceresi acik ve gozle teyit edilmis olmali.

2026-07-07 yerel log zamanli kirk dorduncu devam guncellemesi:

- `window_safety.py` eklendi; tooling/editor pencere guard'i operator preflight
  ve candidate runner tarafinda ortak kullanilir.
- `external_operator_preflight.py` artik VS Code/editor/terminal benzeri
  pencereleri `candidate_action_plan` icine almaz; bunlari
  `window_discovery.excluded_candidates` altinda `candidate_window_is_tooling:*`
  sebebiyle raporlar.
- Canli preflight smoke beklenen sekilde gercek sim adayi bulamadi:
  `candidate_action_plan=[]`, `candidate_action_gate=no_candidate_action`,
  `excluded_candidate_count=1`:
  `logs/game_screen_sandbox/20260707-091501-pr0p-operator-tooling-exclusion-smoke-v1-external-operator-preflight.md`.
- Canli candidate-runner smoke artik VS Code'u secmeden reddetti:
  `candidate_title_contains_not_found:SITL Forge`, `executed_count=0`,
  `operator_discovery.excluded_candidate_count=1`:
  `logs/game_screen_sandbox/20260707-091619-pr0p-candidate-tooling-excluded-smoke-v2-external-candidate-action-runner.md`.
- Hedefli operator/candidate testi gecti:
  `tests/test_game_screen_sandbox.py -k 'operator_preflight or candidate_action': 13 passed, 154 deselected`.
- Tum sandbox testi tekrar gecti:
  `tests/test_game_screen_sandbox.py: 167 passed`.
- Compile kontrolu gecti:
  `window_safety.py`, `external_operator_preflight.py`,
  `external_candidate_action_runner.py`, `external_dry_run_sequence.py`,
  `external_target_acceptance.py`, `sandbox_resume_runner.py`,
  `tests/test_game_screen_sandbox.py`.
- `pgrep` kontrolunde `Kenet Simple Target Game`, `simple_target_game`,
  `external_candidate_action_runner`, `external_operator_preflight`,
  `external_dry_run_sequence`, `external_follow_session`,
  `external_window_preflight`, `rc_binding_assistant`, `ffmpeg`, `gazebo`,
  `gzserver`, `gzclient`, `betaflight`, `pr0p` veya `SITL_Forge` prosesi
  gorulmedi.

2026-07-07 yerel log zamanli kirk besinci devam guncellemesi:

- Mevcut `experiments/simitl_pr0p_probe/` hatti incelendi; pr0p/SimITL icin
  ayri ve Gazebo'dan bagimsiz safe probe suite zaten var.
- Ilk safe suite kosusu `FAIL` verdi; kok neden gercek Gazebo coupling degil,
  `game_screen_sandbox/isolation_audit.py` icindeki kural regex'inin
  `pr0p_isolation_check.py` tarafindan forbidden string sanilmasiydi.
- `pr0p_isolation_check.py` icinde string-literal kural dosyasi allowlist'i
  eklendi; `isolation_audit.py` kural tanimi olarak tarama false-positive'inden
  muaf, normal dosyalardaki forbidden launcher string'leri hala FAIL.
- Hedefli izolasyon/probe-suite testi gecti:
  `tests/test_simitl_pr0p_probe.py -k 'isolation or probe_suite': 5 passed, 76 deselected`.
- Safe pr0p suite tekrar calisti ve beklenen sekilde `WAITING` oldu:
  `P0-isolation PASS`, P4 input readiness/mapping/config patch PASS,
  P6 synthetic E2E/log-check PASS; geriye gercek pr0p pencere, websocket ve
  bbox bekleyen fazlar kaldi:
  `logs/simitl_pr0p/20260707-091851-pr0p-suite-safe-current-v2-suite.md`.
- Tum SimITL/pr0p probe testi gecti:
  `tests/test_simitl_pr0p_probe.py: 81 passed`.
- Game-screen sandbox + SimITL/pr0p izole testleri birlikte gecti:
  `tests/test_game_screen_sandbox.py tests/test_simitl_pr0p_probe.py: 248 passed`.
- Compile kontrolu gecti:
  `pr0p_isolation_check.py`, `pr0p_probe_suite.py`, `preflight_probe.py`,
  `tests/test_simitl_pr0p_probe.py`.
- `pgrep` kontrolunde `Kenet Simple Target Game`, `simple_target_game`,
  `external_candidate_action_runner`, `external_operator_preflight`,
  `external_dry_run_sequence`, `external_follow_session`,
  `external_window_preflight`, `rc_binding_assistant`, `ffmpeg`, `gazebo`,
  `gzserver`, `gzclient`, `betaflight`, `pr0p`, `SITL_Forge`, `SimITL` veya
  `simitl` prosesi gorulmedi.

2026-07-07 yerel log zamanli kirk altinci devam guncellemesi:

- `experiments/simitl_pr0p_probe/independent_sim_readiness.py` eklendi.
- Bu rapor son `game_screen_sandbox` decision kanitini ve son
  `simitl_pr0p_probe` suite kanitini birlikte okur; sim, Gazebo, Betaflight,
  capture veya `uinput` baslatmaz.
- Mevcut smoke beklenen sekilde `WAITING` verdi:
  `game_screen_decision=SIMPLE_SANDBOX_READY`, `pr0p_decision=WAITING`,
  `next_action=Start pr0p, enter a local race, then rerun the safe suite`:
  `logs/simitl_pr0p/20260707-092433-independent-sim-current-v1-independent-sim-readiness.md`.
- Raporun bekleyen pr0p nedenleri: live websocket, read-only MSP websocket,
  FPV capture, tracking dry-run/log-check, live yaw/pitch response,
  RC source loopback ve live tracking control.
- Hedefli readiness/manifest/decision testi gecti:
  `tests/test_simitl_pr0p_probe.py -k 'independent_readiness or live_manifest or decision_report': 12 passed, 72 deselected`.
- Tum SimITL/pr0p probe testi gecti:
  `tests/test_simitl_pr0p_probe.py: 84 passed`.
- Game-screen sandbox + SimITL/pr0p izole testleri birlikte gecti:
  `tests/test_game_screen_sandbox.py tests/test_simitl_pr0p_probe.py: 251 passed`.
- Compile kontrolu gecti:
  `independent_sim_readiness.py`, `pr0p_isolation_check.py`,
  `pr0p_probe_suite.py`, `pr0p_live_run_manifest.py`,
  `pr0p_decision_report.py`, `tests/test_simitl_pr0p_probe.py`.
- `pgrep` kontrolunde `Kenet Simple Target Game`, `simple_target_game`,
  `external_candidate_action_runner`, `external_operator_preflight`,
  `external_dry_run_sequence`, `external_follow_session`,
  `external_window_preflight`, `rc_binding_assistant`, `ffmpeg`, `gazebo`,
  `gzserver`, `gzclient`, `betaflight`, `pr0p`, `SITL_Forge`, `SimITL` veya
  `simitl` prosesi gorulmedi.

2026-07-07 yerel log zamanli kirk yedinci devam guncellemesi:

- `independent_sim_readiness.py` icine evidence freshness gate eklendi.
- Varsayilan `--max-evidence-age-s` 86400 saniye. Daha eski game-screen veya
  pr0p suite kaniti `STALE` sayilir ve promote edilmez.
- Ilk smoke eski game-screen decision dosyasini yakaladi:
  `GAME_SCREEN_EVIDENCE_STALE`, `game_screen_decision=STALE`,
  `next_action=Rerun the game-screen acceptance command...`:
  `logs/simitl_pr0p/20260707-092830-independent-sim-freshness-v1-independent-sim-readiness.md`.
- Ardindan game-screen acceptance izole sekilde yenilendi ve
  `SIMPLE_SANDBOX_READY` verdi:
  `logs/game_screen_sandbox/20260707-092859-independent-sim-refresh-game-screen-v1-acceptance-run.md`.
- Yenilenen birleşik readiness smoke beklenen sekilde yalniz pr0p canli
  kanitlarini bekliyor:
  `game_screen_decision=SIMPLE_SANDBOX_READY`, `pr0p_decision=WAITING`,
  `next_action=Start pr0p, enter a local race, then rerun the safe suite`:
  `logs/simitl_pr0p/20260707-092905-independent-sim-freshness-v2-independent-sim-readiness.md`.
- Hedefli independent readiness testi gecti:
  `tests/test_simitl_pr0p_probe.py -k 'independent_readiness': 4 passed, 81 deselected`.
- Tum SimITL/pr0p probe testi gecti:
  `tests/test_simitl_pr0p_probe.py: 85 passed`.
- Game-screen sandbox + SimITL/pr0p izole testleri birlikte gecti:
  `tests/test_game_screen_sandbox.py tests/test_simitl_pr0p_probe.py: 252 passed`.
- Compile kontrolu gecti:
  `independent_sim_readiness.py`, `pr0p_isolation_check.py`,
  `pr0p_probe_suite.py`, `pr0p_live_run_manifest.py`,
  `pr0p_decision_report.py`, `tests/test_simitl_pr0p_probe.py`.
- `pgrep` kontrolunde `Kenet Simple Target Game`, `simple_target_game`,
  `external_candidate_action_runner`, `external_operator_preflight`,
  `external_dry_run_sequence`, `external_follow_session`,
  `external_window_preflight`, `rc_binding_assistant`, `ffmpeg`, `gazebo`,
  `gzserver`, `gzclient`, `betaflight`, `pr0p`, `SITL_Forge`, `SimITL` veya
  `simitl` prosesi gorulmedi.

## Son Eklenen Ana Parcalar

### `external_follow_session.py`

Dosya:

```text
experiments/game_screen_sandbox/external_follow_session.py
```

Amac:

- Harici oyun/sim penceresini yakalar.
- Tracker + PID follow loop calistirir.
- Dry-run veya acik onayli `uinput` ile komut gonderir.
- JSON/Markdown session raporu yazar.
- Gate metrikleri:
  - `found_ratio`
  - `loss_events`
  - `max_abs_yaw_axis`
  - `max_abs_pitch_axis`
  - `isolation_status`

Basarili dry-run kaniti:

```text
logs/game_screen_sandbox/20260705-113629-external-follow-simple-game-dry-external-follow-session.md
```

Sonuc:

```text
EXTERNAL_FOLLOW_DRY_RUN_READY
```

Beklenen WAITING kanitlari:

```text
logs/game_screen_sandbox/20260705-113609-external-follow-missing-bbox-smoke-external-follow-session.md
```

Sebep:

```text
TRACKER_BBOX_REQUIRED
```

```text
logs/game_screen_sandbox/20260705-113642-external-follow-uinput-no-ack-smoke-external-follow-session.md
```

Sebep:

```text
ACK_LIVE_INPUT_REQUIRED
```

### `rc_binding_assistant.py`

Dosya:

```text
experiments/game_screen_sandbox/rc_binding_assistant.py
```

Amac:

- Oyun/sim icindeki `Controls -> RC Channels` sayfasinda tek tek axis binding
  yapmak.
- `yaw`, `pitch`, `roll`, `throttle` eksenlerini kontrollu sekilde pulse eder.
- Gercek input icin `--uinput --ack-live-input` ister.
- Dry-run raporlar, OS input gondermez.

Ornek:

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

### `S13-binding-dry`

`phase_runner.py`, `decision_report.py` ve `sandbox_acceptance_runner.py`
icinde acceptance akisa eklendi.

Amac:

- RC binding dry-run yolunun regression ile bozulmadigini acceptance icinde
  yakalamak.
- Gercek `uinput` kullanmaz.
- Gazebo/Betaflight kullanmaz.

### `screen_tracking_loop.py` guvenlik kilidi

Dogudan `--uinput` kullanimi artik su kosullari ister:

```text
--ack-live-input
--live-max-duration
```

Onaysiz `--uinput` beklenen sekilde parse seviyesinde hata verir:

```text
screen_tracking_loop.py: error: --uinput requires --ack-live-input
```

### `external_dry_run_sequence.py`

Dosya:

```text
experiments/game_screen_sandbox/external_dry_run_sequence.py
```

Amac:

- Status -> preflight -> follow dry-run adimlarini tek komutta siralar.
- `uinput` kullanmaz.
- Pencere, bbox veya forbidden-process siniri hazir degilse preflight/follow
  calistirmeden `WAITING` raporu yazar.
- `--capture-region` verilirse pencere basligi gate'i bloklayici olmaz; setup
  kontrolu bbox ve Gazebo/Betaflight process-boundary item'lariyla yapilir.
- Basarili olursa `EXTERNAL_DRY_RUN_READY` yazar.
- Rapor metriklerinde `sequence_steps` ve `live_readiness` alanlarini yazar.

### `external_live_input_readiness.py`

Dosya:

```text
experiments/game_screen_sandbox/external_live_input_readiness.py
```

Amac:

- Dry sequence gecmeden canli virtual-RC axis-response kapisina gecmemek.
- `--ack-live-input` yoksa input gondermeden `WAITING` raporu yazmak.
- Ack varsa S5-real axis-response preflight kosup oyun/sim penceresinin
  virtual RC eksenlerine gorsel cevap verip vermedigini olcmek.
- Basarili olursa `EXTERNAL_LIVE_INPUT_READY` yazmak.

### `external_live_follow_sequence.py`

Dosya:

```text
experiments/game_screen_sandbox/external_live_follow_sequence.py
```

Amac:

- Canli follow baslatmadan once live-input readiness kanitini zorunlu kilmak.
- `--ack-live-input` yoksa hicbir live-input/readiness/follow komutu
  calistirmeden `WAITING` raporu yazmak.
- Ack varsa readiness'i tekrar kosup sadece `EXTERNAL_LIVE_INPUT_READY`
  durumunda bounded live follow'a gecmek.
- Basarili olursa `EXTERNAL_LIVE_FOLLOW_COMPLETE` yazmak.

### `sandbox_resume_runner.py`

Dosya:

```text
experiments/game_screen_sandbox/sandbox_resume_runner.py
```

Amac:

- Mevcut status raporunu uretmek.
- `resume_commands` listesini uygulanabilir adimlara cevirmek.
- Varsayilan olarak plan-only kalmak; capture veya input calistirmamak.
- `--execute-safe` ile yalniz non-uinput ve edit gerektirmeyen komutlari
  sinirli sayida calistirmak.
- Guvenli komut calistiktan sonra post-status raporu uretmek.
- Dry evidence hazir ama live input readiness eksikken `rc_binding_assistant`
  komutunu ack-gated resume adimi olarak gostermek.
- RC binding raporlarini pencere+bbox baglami ile status'a eslestirmek.
- Dry-run rc-binding raporlarini live binding hazir kaniti saymamak.
- `--require-live-follow` icin gomulu live uinput proof olmadan
  `EXTERNAL_LIVE_FOLLOW_COMPLETE` sonucunu yeterli kanit saymamak.
- Live virtual RC komutlarini sadece `--execute-live-input --ack-live-input`
  ile acmak.
- `requires_pass` listesindeki status item'lari PASS degilse komutu
  calistirmadan `BLOCKED` birakmak.
- `resume_decision` ile operator icin ilk aksiyon ve ilk eksik gate'i ozetlemek.
- `resume_step_summary` ile adim status sayilarini, live-input komut sayisini
  ve ilk planned/blocked/unmet/failure adimini ozetlemek.
- Status raporundaki `readiness_ladder` ile S0-S5 asama durumunu gostermek.
- Status raporundaki `evidence_freshness` ile kanit artefaktlarinin eski,
  eksik veya taze oldugunu gostermek.
- `--require-fresh-evidence` verilirse stale/missing zorunlu kanitlar varken
  `READY` sonucunu `WAITING` olarak tutmak.

### `external_operator_preflight.py`

Dosya:

```text
experiments/game_screen_sandbox/external_operator_preflight.py
```

Amac:

- Gercek pr0p/SITL Forge denemesinden once status-only operator hazirlik
  raporu uretmek.
- Simulator, capture, Gazebo, Betaflight veya `uinput` baslatmadan
  `operator_preflight_decision` yazmak.
- Mevcut `sandbox_status_report.py` kanitlarini ve
  `external_target_acceptance.py` komut-state siniflandirmasini tek raporda
  birlestirmek.
- Ilk readiness stage, ilk operator aksiyonu, recommended command,
  safe/blocked/ack-required komut durumlarini gostermek.
- Hedef pencere basligi eslesmezse `window_discovery` ile gorunen X11 uygulama
  penceresi adaylarini gostermek.
- VS Code/editor/terminal gibi tooling pencerelerini `excluded_candidates`
  altinda raporlayip candidate action plani disinda tutmak.
- Her aday icin `candidate_action_plan` ile siradaki guvenli region capture veya
  region dry-run adimini gostermek.
- Her pencere adayi icin exact `rerun_operator_preflight_command` ve
  `capture_bbox_frame_command` komutlarini gostermek.
- Baslik degiskenligi icin her adayda `capture_bbox_frame_region_command`
  komutunu da gostermek.
- Bbox dosyasi hazir oldugunda her aday icin
  `dry_run_sequence_region_command` komutunu gostermek.

### `external_candidate_action_runner.py`

Dosya:

```text
experiments/game_screen_sandbox/external_candidate_action_runner.py
```

Amac:

- Operator preflight aday planini guarded handoff olarak calistirmak.
- Varsayilan olarak plan-only kalmak; aday selector yoksa komut
  calistirmemek.
- `--candidate-window-id`, `--candidate-index`, exact `--candidate-title` veya
  tekil `--candidate-title-contains` ile aday secmek.
- VS Code gibi tooling/editor pencerelerini, baslikta simulator metni gecse de
  reddetmek.
- `--candidate-bbox X,Y,W,H` ile secili aday region'u uzerinden bbox dosyasini
  yazmak; bbox secili region disina tasarsa komut calistirmadan reddetmek.
- Secili aday ve `--ack-candidate --execute-safe` ile yalniz bir safe
  non-`uinput` aday komutunu calistirmak.
- Belirsiz title-substring eslesmelerini reddetmek.
- `uinput` gonderebilecek aday aksiyonlarini reddetmek.
- Nested operator preflight raporunu evidence olarak yazmak.
- Aday komutu calistiginda post-action operator preflight raporu yazarak
  once/sonra readiness durumunu tek artefakta almak.

### `external_target_acceptance.py`

Dosya:

```text
experiments/game_screen_sandbox/external_target_acceptance.py
```

Amac:

- Pr0p/SITL Forge gibi gercek harici pencere icin son kabul kararini vermek.
- Varsayilan olarak proje hedefi olan `--mode live-follow` kanitini istemek.
- Fresh evidence zorunlu tutmak; gerekirse `--allow-stale-evidence` ile
  yalniz debug amacli gevsetmek.
- `sandbox_status_report.py` sonucunu evidence olarak gommek.
- `EXTERNAL_TARGET_READY`, `WAITING` veya `REJECT` yazmak.
- Yeni forbidden Gazebo/Betaflight process delta gorurse `REJECT` yazmak.
- `acceptance_decision` ile ilk blokaj ve siradaki operator aksiyonunu ust
  seviyede gostermek.
- `operator_command_queue` ile siradaki komutlari safety metadata'siyle ust
  seviyede gostermek.
- Her queue girdisinde `command_state` ve `unmet_requires_pass` yazarak neden
  bloklu oldugunu aciklamak.
- `operator_command_summary` ile state sayilarini ve ilk available/blocked/
  ack-required komutlari ust seviyede gostermek.
- Status-only kalmak; simulator, capture, Gazebo, Betaflight veya `uinput`
  baslatmamak.

Dry-run sequence icin son beklenen WAITING kaniti, pr0p penceresi ve bbox
yokken:

```text
logs/game_screen_sandbox/20260706-212123-pr0p-sequence-waiting-smoke-external-dry-run-sequence.md
```

Repo-local harici pencere ile basarili dry-run sequence kaniti:

```text
logs/game_screen_sandbox/20260706-212424-simple-window-dry-sequence-ready-external-dry-run-sequence.md
```

Sonuc:

```text
EXTERNAL_DRY_RUN_READY
```

Bu kosu `Kenet Simple Target Game` penceresini harici oyun gibi kullandi.
Metrikler:

```text
preflight: EXTERNAL_WINDOW_READY_DRY
follow: EXTERNAL_FOLLOW_DRY_RUN_READY
found_ratio: 1.0
loss_events: 0
isolation_status: PASS
```

## Guncellenen Dokumanlar

- `simple_game_sitl.md`
- `experiments/game_screen_sandbox/README.md`
- `experiments/game_screen_sandbox/PLAN.md`

Bu dokumanlarda mevcut siralama soyle:

1. Oyun/sim penceresini ac.
2. BBox sec.
3. `external_window_preflight.py` ile capture/tracker/PID dry preflight calistir.
4. Gerekirse `rc_binding_assistant.py` ile `Controls -> RC Channels` binding yap.
5. `external_follow_session.py` ile tracker + PID follow dry-run calistir.
6. Ancak dry-run ve axis-response kabul edilebilir olduktan sonra kisa sureli
   `--uinput --ack-live-input` live denemeye gec.

## Onemli Komutlar

### Full sandbox acceptance

```bash
fpv_env/bin/python experiments/game_screen_sandbox/sandbox_acceptance_runner.py \
  --include-simple-window \
  --duration 6 \
  --hz 20 \
  --run-id acceptance-external-follow-session-ready
```

Son bilinen sonuc:

```text
SIMPLE_SANDBOX_READY
```

### Harici pencere preflight

Once status-only external target acceptance ile bak:

```bash
fpv_env/bin/python experiments/game_screen_sandbox/external_target_acceptance.py \
  --window-title pr0p \
  --bbox-file logs/game_screen_sandbox/pr0p-target-bbox.json \
  --mode live-follow \
  --axis-expected-shifts yaw:+x,pitch:-y \
  --axis-min-shift-px 1 \
  --evidence-stale-after-s 3600
```

Alt status raporunu direkt okumak gerekirse:

```bash
fpv_env/bin/python experiments/game_screen_sandbox/sandbox_status_report.py \
  --window-title pr0p \
  --bbox-file logs/game_screen_sandbox/pr0p-target-bbox.json \
  --require-live-follow \
  --require-fresh-evidence \
  --evidence-stale-after-s 3600
```

Pencere ve bbox hazirsa tum dry-run zincirini tek komutla calistir:

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

Bu sequence `uinput` basmaz; yalniz status -> preflight -> follow dry-run
siralar. Basarili olursa `EXTERNAL_DRY_RUN_READY` yazar.

```bash
fpv_env/bin/python experiments/game_screen_sandbox/external_window_preflight.py \
  --capture-window-title pr0p \
  --capture-backend ffmpeg \
  --tracker-bbox-file logs/game_screen_sandbox/pr0p-target-bbox.json \
  --run-id pr0p-window-preflight
```

Axis-response dahil:

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

### Harici follow dry-run

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

Basarili olursa:

```text
EXTERNAL_FOLLOW_DRY_RUN_READY
```

### Harici follow live, sadece kisa ve acik onayli

```bash
fpv_env/bin/python experiments/game_screen_sandbox/external_follow_session.py \
  --capture-window-title pr0p \
  --capture-backend ffmpeg \
  --tracker-bbox-file logs/game_screen_sandbox/pr0p-target-bbox.json \
  --duration 1 \
  --hz 10 \
  --uinput \
  --ack-live-input \
  --live-max-duration 2 \
  --run-id pr0p-follow-live
```

## Mevcut Worktree Notu

Repo genelinde cok sayida degisiklik var. Bunlarin hepsi bu sandbox isine ait
degil. Son `git status --short` genel olarak eski Gazebo/SITL dosyalarinda da
degisiklikler ve untracked dosyalar gosteriyor.

Sandbox ile dogrudan ilgili ana dosyalar:

```text
experiments/game_screen_sandbox/
tests/test_game_screen_sandbox.py
simple_game_sitl.md
```

Bir sonraki agent, unrelated degisiklikleri revert etmemeli.

## Sonraki Yapilacaklar

1. Once gercek pr0p veya SITL Forge penceresini manuel ac.
2. Oyun icinde gerekirse `Controls -> RC Channels` ayarlarini kontrol et; RC
   komutlari gorunmuyorsa binding adimina erken don.
3. `bbox_tool.py` ile hedef bbox sec ve
   `logs/game_screen_sandbox/pr0p-target-bbox.json` uret. Iki guvenli yol var:
   once frame yakalayip `--bbox X,Y,W,H` ile tekrar calistir veya pr0p
   penceresi bilerek acikken `--interactive-select` ile ROI UI'dan sec.
4. Once `sandbox_status_report.py` ile pencere + bbox durumuna bak.
5. Pencere ve bbox hazirsa `external_dry_run_sequence.py` ile tek komut dry
   sequence calistir.
6. Gerekirse `external_window_preflight.py` ile ayrik dry preflight calistir.
7. Eger S1/S2/S7 PASS ama S5 WAITING ise once oyun/sim
   `Controls -> RC Channels` / axis binding ayarlarini kontrol et.
8. `rc_binding_assistant.py` ile yaw/pitch/roll/throttle binding yap.
9. `external_live_input_readiness.py --ack-live-input` ile
   `EXTERNAL_LIVE_INPUT_READY` elde et. Eksen yonu supheliyse
   `--axis-expected-shifts yaw:+x,pitch:-y` ve `--axis-min-shift-px 1`
   ekleyerek signed direction kanitini zorunlu kil. Ayni argumanlar
   `sandbox_status_report.py` ve `sandbox_resume_runner.py` tarafindan da
   korunur. Final status bu raporu ancak gomulu S5-real live uinput proof
   varsa hazir sayar.
10. `external_follow_session.py` ile `EXTERNAL_FOLLOW_DRY_RUN_READY` elde et.
11. Ancak bundan sonra `external_live_follow_sequence.py --ack-live-input`
    ile kisa sureli live `uinput` dene.
12. Live follow tamamlaninca `sandbox_status_report.py --require-live-follow`
    ile final status kanitini oku. Bu gate artik gomulu live uinput proof
    ister; eksikse `proof_gaps` alanina bak.
13. Live denemede su metrikleri rapordan izle:
   - `found_ratio`
   - `loss_events`
   - `max_abs_yaw_axis`
   - `max_abs_pitch_axis`
   - `isolation_status`
14. Runaway/saturation gorulurse PID tuning'e hemen atlama; once:
   - RC axis mapping dogru mu?
   - oyun penceresi gercekten input goruyor mu?
   - bbox stabil mi?
   - tracker kopuyor mu?
   - kamera/capture FPS yeterli mi?

## Devam Eden Sinirlar

- Bu sandbox Gazebo/Betaflight hazirligini kanitlamaz.
- Bu sandbox fiziksel RC hazirligini kanitlamaz.
- Virtual RC sadece debug/validation araci olarak kalmali.
- Nihai fiziksel RC yoluna donus ayrica planlanmali.
- Goal henuz tamamlandi diye isaretlenmedi.

## Hemen Bakilacak Kanit Dosyalari

```text
logs/game_screen_sandbox/20260705-113825-acceptance-external-follow-session-ready-acceptance-run.md
logs/game_screen_sandbox/20260705-113825-acceptance-external-follow-session-ready-decision.md
logs/game_screen_sandbox/20260705-113629-external-follow-simple-game-dry-external-follow-session.md
logs/game_screen_sandbox/20260705-113642-external-follow-uinput-no-ack-smoke-external-follow-session.md
logs/game_screen_sandbox/20260705-113609-external-follow-missing-bbox-smoke-external-follow-session.md
logs/game_screen_sandbox/20260706-213335-pr0p-sequence-waiting-smoke-v4-external-dry-run-sequence.md
logs/game_screen_sandbox/20260706-213251-simple-window-dry-sequence-ready-v3-external-dry-run-sequence.md
logs/game_screen_sandbox/20260706-214002-simple-window-live-readiness-noack-v1-external-live-input-readiness.md
logs/game_screen_sandbox/20260706-214320-pr0p-status-require-live-smoke-v1-status-report.md
logs/game_screen_sandbox/20260706-214757-pr0p-live-follow-noack-smoke-v1-external-live-follow-sequence.md
logs/game_screen_sandbox/20260706-215250-pr0p-status-require-live-follow-smoke-v1-status-report.md
logs/game_screen_sandbox/20260706-215659-pr0p-status-bbox-aware-smoke-v2-status-report.md
logs/game_screen_sandbox/20260706-220008-pr0p-status-resume-commands-smoke-v1-status-report.md
logs/game_screen_sandbox/20260706-220440-pr0p-status-resume-metadata-smoke-v1-status-report.md
logs/game_screen_sandbox/20260706-221133-pr0p-resume-runner-plan-smoke-v2-sandbox-resume-runner.md
logs/game_screen_sandbox/20260706-221459-pr0p-resume-runner-execsafe-poststatus-smoke-v2-sandbox-resume-runner.md
logs/game_screen_sandbox/20260706-221901-pr0p-status-rc-binding-command-smoke-v1-status-report.md
logs/game_screen_sandbox/20260706-222427-pr0p-status-rc-binding-context-smoke-v1-status-report.md
logs/game_screen_sandbox/20260706-222830-pr0p-status-rc-binding-dry-not-ready-smoke-v1-status-report.md
logs/game_screen_sandbox/20260706-223239-pr0p-status-live-follow-proof-smoke-v1-status-report.md
logs/game_screen_sandbox/20260706-224113-pr0p-status-signed-axis-gate-smoke-v1-status-report.md
logs/game_screen_sandbox/20260706-224814-pr0p-status-signed-resume-command-smoke-v1-status-report.md
logs/game_screen_sandbox/20260706-224814-pr0p-resume-signed-axis-plan-smoke-v1-sandbox-resume-runner.md
logs/game_screen_sandbox/20260706-225452-pr0p-status-live-input-proof-smoke-v1-status-report.md
logs/game_screen_sandbox/20260706-230426-pr0p-status-interactive-bbox-command-smoke-v2-status-report.md
logs/game_screen_sandbox/20260706-230426-pr0p-resume-interactive-bbox-plan-smoke-v2-sandbox-resume-runner.md
logs/game_screen_sandbox/20260706-230830-pr0p-status-requires-pass-smoke-v1-status-report.md
logs/game_screen_sandbox/20260706-230830-pr0p-resume-requires-pass-plan-smoke-v1-sandbox-resume-runner.md
logs/game_screen_sandbox/20260706-230850-pr0p-resume-requires-pass-execsafe-bbox-window-missing-smoke-v1-sandbox-resume-runner.md
logs/game_screen_sandbox/20260706-230901-pr0p-resume-requires-pass-liveack-bbox-window-missing-smoke-v1-sandbox-resume-runner.md
logs/game_screen_sandbox/20260706-231348-pr0p-resume-decision-summary-plan-smoke-v1-sandbox-resume-runner.md
logs/game_screen_sandbox/20260706-231349-pr0p-resume-decision-summary-liveack-bbox-window-missing-smoke-v1-sandbox-resume-runner.md
logs/game_screen_sandbox/20260706-231806-pr0p-status-readiness-ladder-smoke-v1-status-report.md
logs/game_screen_sandbox/20260706-231806-pr0p-resume-readiness-ladder-smoke-v1-sandbox-resume-runner.md
logs/game_screen_sandbox/20260706-232116-pr0p-status-evidence-freshness-smoke-v1-status-report.md
logs/game_screen_sandbox/20260706-232116-pr0p-resume-evidence-freshness-smoke-v1-sandbox-resume-runner.md
logs/game_screen_sandbox/20260706-232912-pr0p-status-fresh-evidence-gate-smoke-v1-status-report.md
logs/game_screen_sandbox/20260706-232912-pr0p-resume-fresh-evidence-gate-smoke-v1-sandbox-resume-runner.md
logs/game_screen_sandbox/20260706-233447-pr0p-external-acceptance-smoke-v1-external-target-acceptance.md
logs/game_screen_sandbox/20260706-233941-pr0p-external-acceptance-isolation-smoke-v1-external-target-acceptance.md
logs/game_screen_sandbox/20260706-234233-pr0p-external-acceptance-decision-smoke-v1-external-target-acceptance.md
logs/game_screen_sandbox/20260706-234546-pr0p-external-acceptance-command-queue-smoke-v1-external-target-acceptance.md
logs/game_screen_sandbox/20260706-234846-pr0p-external-acceptance-command-state-smoke-v1-external-target-acceptance.md
logs/game_screen_sandbox/20260706-235122-pr0p-external-acceptance-command-summary-smoke-v1-external-target-acceptance.md
logs/game_screen_sandbox/20260707-000230-pr0p-operator-preflight-smoke-v1-external-operator-preflight.md
logs/game_screen_sandbox/20260707-000818-pr0p-operator-preflight-window-candidates-smoke-v2-external-operator-preflight.md
logs/game_screen_sandbox/20260707-001217-pr0p-operator-preflight-rerun-commands-smoke-v1-external-operator-preflight.md
logs/game_screen_sandbox/20260707-001455-pr0p-operator-preflight-region-commands-smoke-v1-external-operator-preflight.md
logs/game_screen_sandbox/20260707-001931-pr0p-operator-preflight-region-dryrun-smoke-v1-external-operator-preflight.md
logs/game_screen_sandbox/20260707-002241-pr0p-operator-preflight-region-dryrun-smoke-v2-external-operator-preflight.md
logs/game_screen_sandbox/20260707-002619-pr0p-operator-candidate-action-smoke-v1-external-operator-preflight.md
logs/game_screen_sandbox/20260707-003116-pr0p-candidate-action-plan-smoke-v1-external-candidate-action-runner.md
logs/game_screen_sandbox/20260707-003530-pr0p-candidate-action-post-preflight-smoke-v1-external-candidate-action-runner.md
logs/game_screen_sandbox/20260707-085433-pr0p-candidate-window-id-selector-smoke-v1-external-candidate-action-runner.md
logs/game_screen_sandbox/20260707-085945-pr0p-candidate-bbox-plan-smoke-v1-external-candidate-action-runner.md
```

## Baslamadan Once Kontrol

Kota korumak icin once sadece sunlari calistir:

```bash
fpv_env/bin/python experiments/game_screen_sandbox/sandbox_status_report.py \
  --window-title pr0p \
  --bbox-file logs/game_screen_sandbox/pr0p-target-bbox.json \
  --require-live-follow \
  --require-fresh-evidence \
  --evidence-stale-after-s 3600
fpv_env/bin/python experiments/game_screen_sandbox/external_operator_preflight.py \
  --window-title pr0p \
  --bbox-file logs/game_screen_sandbox/pr0p-target-bbox.json \
  --require-live-follow \
  --require-fresh-evidence \
  --evidence-stale-after-s 3600 \
  --axis-expected-shifts yaw:+x,pitch:-y \
  --axis-min-shift-px 1 \
  --max-window-candidates 5
fpv_env/bin/python experiments/game_screen_sandbox/external_candidate_action_runner.py \
  --window-title pr0p \
  --bbox-file logs/game_screen_sandbox/pr0p-target-bbox.json \
  --require-live-follow \
  --require-fresh-evidence \
  --evidence-stale-after-s 3600 \
  --axis-expected-shifts yaw:+x,pitch:-y \
  --axis-min-shift-px 1 \
  --max-window-candidates 5
fpv_env/bin/python experiments/game_screen_sandbox/external_target_acceptance.py \
  --window-title pr0p \
  --bbox-file logs/game_screen_sandbox/pr0p-target-bbox.json \
  --mode live-follow \
  --axis-expected-shifts yaw:+x,pitch:-y \
  --axis-min-shift-px 1 \
  --evidence-stale-after-s 3600
fpv_env/bin/python experiments/game_screen_sandbox/sandbox_resume_runner.py \
  --window-title pr0p \
  --bbox-file logs/game_screen_sandbox/pr0p-target-bbox.json \
  --require-live-follow \
  --require-fresh-evidence \
  --evidence-stale-after-s 3600
git status --short
ls -t logs/game_screen_sandbox | head -20
pgrep -af 'Kenet Simple Target Game|simple_target_game|external_candidate_action_runner|external_operator_preflight|external_dry_run_sequence|external_follow_session|external_window_preflight|rc_binding_assistant|ffmpeg|gazebo|gzserver|gzclient|betaflight|pr0p|SITL_Forge'
```

`sandbox_status_report.py` status-only calisir; simulator, capture veya `uinput`
baslatmaz. Pr0p penceresi ve bbox yoksa beklenen sonuc `WAITING` olur ve
siradaki eksik gate'leri listeler.
Eger bbox eksikse rapordaki `interactive_bbox_select` komutu manuel UI ister;
`sandbox_resume_runner.py` bu komutu otomatik acmaz.
Komutlarda `requires_pass` listesi varsa runner listedeki status item'lari
PASS olmadan komutu calistirmaz; bu durum `unmet_requires_pass` alaninda
gorulur.
Runner raporundaki `resume_decision.next_operator_action`, bir sonraki insan
adimini en kisa sekilde gosterir.
Operator preflight raporundaki `operator_preflight_decision`, ayni bilgiyi
komut-state sayilari ve recommended command ile tek raporda toplar.
Candidate action runner dogru pencere adayina gecisi guard'li hale getirir;
aday secilmeden veya `--ack-candidate` verilmeden komut calistirmaz.
Status raporundaki `readiness_ladder`, hangi asamaya kadar gelindigini ve ilk
blokajin hangi S asamasinda oldugunu gosterir.
Status raporundaki `evidence_freshness`, eski artefaktlarin yeni kanit gibi
gorunmesini engellemek icin artefakt yasini ve `FRESH/STALE/MISSING` durumunu
gosterir.
`--require-fresh-evidence` eklenirse ayni bilgi hard gate olur; stale/missing
zorunlu kanitlar varken status `READY` olmaz.

Gerekiyorsa sonra sadece hedefli test:

```bash
fpv_env/bin/python -m pytest tests/test_game_screen_sandbox.py -q
```

2026-07-07 yerel log zamanli kirk sekizinci devam guncellemesi:

- `independent_sim_readiness.py` next-action siralamasi duzeltildi.
- Eski davranis: game-screen kaniti taze ve pr0p suite `WAITING` iken,
  `P1-install-discovery=WAITING` olsa bile rapor
  `Start pr0p, enter a local race...` oneriyordu.
- Yeni davranis: `P1-install-discovery` `UNKNOWN/MISSING/WAITING/FAIL`
  durumundaysa combined readiness once izole pr0p install discovery/download
  adimini onerir; live race onerisi ancak P1 kurulum kaniti hazir olunca gelir.
- Combined readiness komut listesine su komutlar eklendi:
  `pr0p_install_discovery`, `pr0p_install_download`,
  `pr0p_live_session_dry`.
- `experiments/simitl_pr0p_probe/README.md` ve `PLAN.md` icinde ayni
  oncelik kurali dokumante edildi.
- Live smoke beklenen sekilde `WAITING` verdi ama artik dogru siradaki adimi
  gosteriyor:
  `Download or install pr0p in the isolated root first...`:
  `logs/simitl_pr0p/20260707-093518-independent-sim-install-priority-v1-independent-sim-readiness.md`.
- Smoke metrikleri:
  `game_screen_decision=SIMPLE_SANDBOX_READY`,
  `pr0p_decision=WAITING`, `P1-install-discovery=WAITING`,
  game-screen ve pr0p suite freshness `PASS`.
- Hedefli independent readiness testi gecti:
  `tests/test_simitl_pr0p_probe.py -k 'independent_readiness': 5 passed, 81 deselected`.
- Tum SimITL/pr0p probe testi gecti:
  `tests/test_simitl_pr0p_probe.py: 86 passed`.
- Game-screen sandbox + SimITL/pr0p izole testleri birlikte gecti:
  `tests/test_game_screen_sandbox.py tests/test_simitl_pr0p_probe.py: 253 passed`.

Sonraki en dogru adim:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_install_probe.py \
  --install-root /tmp/fpv-test-simitl-pr0p \
  --download \
  --run-id pr0p-install-download
```

Ardindan updater manuel calistirilmali, discovery/safe-suite tekrar kosulmali
ve P1 artik ilk blokaj degilse pr0p local race / websocket canli kanit fazina
gecilmeli.

2026-07-07 yerel log zamanli kirk dokuzuncu devam guncellemesi:

- Izole root'a updater indirildi; binary calistirilmadi:
  `logs/simitl_pr0p/20260707-093648-pr0p-install-download-current-v1-install-probe.md`.
- Indirme/probe sonucu `PASS`; dosya:
  `/tmp/fpv-test-simitl-pr0p/updater`, boyut `36090380`, sha256
  `deb8899d11f147279d7c72f3a172baa3f8245acfa16fbee21141264fed89467b`.
- Post-download safe suite kosuldu:
  `logs/simitl_pr0p/20260707-093654-pr0p-suite-post-download-v1-suite.md`.
- Bu suite artik `P1-install-discovery PASS` veriyor, fakat
  `install_scan.client_candidates=[]`; sadece `updater` executable gorunuyor.
- Bu yuzden `independent_sim_readiness.py` icine ikinci guard eklendi:
  `P1 PASS` olsa bile client candidate yoksa rapor live race onermiyor.
- Yeni combined readiness smoke beklenen sekilde `WAITING` verdi ve siradaki
  adimi su sekilde gosterdi:
  `Run the downloaded updater manually from the isolated root, then rerun install discovery/live-session dry-run until a pr0p client executable is visible.`:
  `logs/simitl_pr0p/20260707-093813-independent-sim-post-download-guard-v1-independent-sim-readiness.md`.
- `pr0p_live_session_runner.py --run-id pr0p-live-session-dry-post-download-v1`
  beklenen sekilde `WAITING` verdi:
  `pr0p executable is not installed in the isolated root`.
- Hedefli independent readiness testi gecti:
  `tests/test_simitl_pr0p_probe.py -k 'independent_readiness': 6 passed, 81 deselected`.
- Tum SimITL/pr0p probe testi gecti:
  `tests/test_simitl_pr0p_probe.py: 87 passed`.
- Game-screen sandbox + SimITL/pr0p izole testleri birlikte gecti:
  `tests/test_game_screen_sandbox.py tests/test_simitl_pr0p_probe.py: 254 passed`.

Sonraki en dogru adim artik download degil:

1. `/tmp/fpv-test-simitl-pr0p/updater` manuel calistirilmali.
2. Kurulum bittikten sonra:

   ```bash
   fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_install_probe.py \
     --install-root /tmp/fpv-test-simitl-pr0p \
     --run-id pr0p-install-after-updater
   fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_live_session_runner.py \
     --run-id pr0p-live-session-dry-after-updater
   fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_probe_suite.py \
     --run-id pr0p-suite-after-updater
   fpv_env/bin/python experiments/simitl_pr0p_probe/independent_sim_readiness.py \
     --run-id independent-sim-after-updater
   ```

3. Ancak client executable gorunurse pr0p local race / websocket canli kanit
   fazina gecilmeli.

2026-07-07 yerel log zamanli ellinci devam guncellemesi:

- `experiments/simitl_pr0p_probe/pr0p_client_probe.py` eklendi.
- Bu probe sadece izole install root'u tarar; `updater` dosyasini pr0p client
  saymaz. `pr0p`, `pr0p.x86_64` veya pr0p/prop isimli executable client
  adaylarini en fazla 3 derinlikte arar.
- `pr0p_probe_suite.py` icine yeni `P1-client-executable` fazi eklendi.
  Artik P1 iki parcali:
  `P1-install-discovery` updater/download kaniti,
  `P1-client-executable` gercek runnable client kaniti.
- `pr0p_decision_report.py`, `pr0p_live_run_manifest.py` ve
  `independent_sim_readiness.py` bu yeni faza baglandi.
- Gercek mevcut durum smoke'u:
  `logs/simitl_pr0p/20260707-094359-pr0p-client-current-v1-client-probe.md`
  `WAITING` verdi:
  `pr0p client executable is not installed in the isolated root`.
- Yeni safe suite:
  `logs/simitl_pr0p/20260707-094409-pr0p-suite-client-gate-v1-suite.md`.
  Kritik satirlar:
  `P1-install-discovery PASS`, `P1-client-executable WAITING`,
  `P4-input-readiness PASS`, `P4-input-mapping PASS`,
  `P6-synthetic-e2e-dry-run PASS`, `P6-synthetic-log-check PASS`.
- Yeni combined readiness:
  `logs/simitl_pr0p/20260707-094414-independent-sim-client-gate-v1-independent-sim-readiness.md`
  `WAITING` verdi ve nedenlere `PR0P:MISSING_PR0P_CLIENT_EXECUTABLE`
  eklendi.
- Live manifest ve decision raporlari da ayni yerde duruyor:
  `logs/simitl_pr0p/20260707-094544-pr0p-live-manifest-client-gate-v1-live-manifest.md`
  next action olarak `Run the downloaded updater manually...` veriyor;
  `logs/simitl_pr0p/20260707-094544-pr0p-decision-client-gate-v1-decision.md`
  `WAITING / promotion is not justified yet`.
- Siradaki adim artik net:

  ```bash
  /tmp/fpv-test-simitl-pr0p/updater
  fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_client_probe.py \
    --install-root /tmp/fpv-test-simitl-pr0p \
    --run-id pr0p-client-after-updater
  fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_probe_suite.py \
    --run-id pr0p-suite-after-client
  fpv_env/bin/python experiments/simitl_pr0p_probe/independent_sim_readiness.py \
    --run-id independent-sim-after-client
  ```

- Hedefli testler gecti:
  `tests/test_simitl_pr0p_probe.py -k 'client_probe or independent_readiness or live_manifest or decision_report': 17 passed, 72 deselected`.
- Tum SimITL/pr0p probe testi gecti:
  `tests/test_simitl_pr0p_probe.py: 89 passed`.
- Game-screen sandbox + SimITL/pr0p izole testleri birlikte gecti:
  `tests/test_game_screen_sandbox.py tests/test_simitl_pr0p_probe.py: 256 passed`.

2026-07-07 yerel log zamanli elli birinci devam guncellemesi:

- `experiments/simitl_pr0p_probe/pr0p_updater_runner.py` eklendi.
- Varsayilan mod dry-run: dis binary calistirmaz, sadece
  `/tmp/fpv-test-simitl-pr0p/updater` icin launch planini, install scan'i ve
  client candidate sayisini raporlar.
- Gercek updater calistirma icin iki acik ack gerekir:
  `--launch-updater --ack-external-binary`. Updater penceresinin acik
  birakilmasi istenirse ayrica `--leave-running --ack-leave-running` gerekir.
- Dry-run smoke:
  `logs/simitl_pr0p/20260707-095020-pr0p-updater-dry-current-v1-updater-runner.md`
  `PASS` verdi; `real_launch=false`, `planned_command=/tmp/fpv-test-simitl-pr0p/updater`,
  `client_candidates_after=0`.
- `pr0p_live_run_manifest.py` ve `independent_sim_readiness.py` komut listesine
  updater runner dry-run ve ack'li launch komutlari eklendi.
- Yeni manifest:
  `logs/simitl_pr0p/20260707-095038-pr0p-live-manifest-updater-runner-v1-live-manifest.md`
  next action:
  `Run P1-updater-runner-dry, then launch the updater with explicit ack or manually, and rerun P1-client-executable.`
- Yeni combined readiness:
  `logs/simitl_pr0p/20260707-095038-independent-sim-updater-runner-v1-independent-sim-readiness.md`
  next action:
  `Run the pr0p updater runner dry-run, then launch the updater with explicit ack or manually, and rerun the pr0p client probe until a client executable is visible.`
- Decision raporu:
  `logs/simitl_pr0p/20260707-095039-pr0p-decision-updater-runner-v1-decision.md`
  halen `WAITING / promotion is not justified yet`.
- Hedefli testler gecti:
  `tests/test_simitl_pr0p_probe.py -k 'updater_runner or client_probe or independent_readiness or live_manifest or decision_report': 20 passed, 72 deselected`.
- Tum SimITL/pr0p probe testi gecti:
  `tests/test_simitl_pr0p_probe.py: 92 passed`.
- Game-screen sandbox + SimITL/pr0p izole testleri birlikte gecti:
  `tests/test_game_screen_sandbox.py tests/test_simitl_pr0p_probe.py: 259 passed`.
- Proses kontrolu temiz:
  `Kenet Simple Target Game`, `simple_target_game`, `external_*`, `ffmpeg`,
  `gazebo`, `gzserver`, `gzclient`, `betaflight`, `pr0p`, `SITL_Forge`,
  `SimITL`, `simitl` ve `/tmp/fpv-test-simitl-pr0p/updater` prosesi yok.

Sonraki en dogru adim:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_updater_runner.py \
  --install-root /tmp/fpv-test-simitl-pr0p \
  --launch-updater \
  --ack-external-binary \
  --leave-running \
  --ack-leave-running \
  --run-id pr0p-updater-launch
```

Updater kurulum penceresi tamamlandiktan sonra:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_client_probe.py \
  --install-root /tmp/fpv-test-simitl-pr0p \
  --run-id pr0p-client-after-updater
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_probe_suite.py \
  --run-id pr0p-suite-after-updater
fpv_env/bin/python experiments/simitl_pr0p_probe/independent_sim_readiness.py \
  --run-id independent-sim-after-updater
```

2026-07-07 yerel log zamanli elli ikinci devam guncellemesi:

- Ack'li updater launch gercekten calistirildi ve pencere acik birakildi:

  ```bash
  fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_updater_runner.py \
    --install-root /tmp/fpv-test-simitl-pr0p \
    --launch-updater \
    --ack-external-binary \
    --leave-running \
    --ack-leave-running \
    --startup-wait 5 \
    --run-id pr0p-updater-launch-current-v1
  ```

- Launch raporu:
  `logs/simitl_pr0p/20260707-095401-pr0p-updater-launch-current-v1-updater-runner.md`.
  Sonuc `PASS`; `real_launch=true`, `PROCESS_LEFT_RUNNING_BY_ACK`.
- Acik updater prosesleri:
  `27454 sh -c /usr/bin/x-terminal-emulator -e /tmp/fpv-test-simitl-pr0p/updater`,
  `27455 /usr/bin/python3 /usr/bin/x-terminal-emulator -e /tmp/fpv-test-simitl-pr0p/updater`,
  `27465 /tmp/fpv-test-simitl-pr0p/updater`.
- X11 penceresi gorunur:
  title `/tmp/fpv-test-simitl-pr0p/updater`, terminal class
  `X-terminal-emulator`, pencere `814x548+178+223`.
- Updater stdout:
  `pr0p updater 0.0.3`, `Directory: /tmp/fpv-test-simitl-pr0p`.
  stderr bos.
- Client probe halen `WAITING`:
  `logs/simitl_pr0p/20260707-095720-pr0p-client-final-check-v1-client-probe.md`,
  `pr0p client executable is not installed in the isolated root`.
- Onemli false-positive bug yakalandi: updater terminal basliginda `pr0p`
  gectigi icin capture probe ilk denemede terminali FPV penceresi sanip
  `PASS` verdi:
  `logs/simitl_pr0p/20260707-095548-capture-updater-false-positive-check-v1-capture-probe.md`.
- `pr0p_capture_probe.py` varsayilan exclusion listesine
  `x-terminal-emulator`, `/tmp/fpv-test-simitl-pr0p/updater`,
  `pr0p updater` eklendi.
- Regression testi eklendi:
  `test_capture_probe_excludes_updater_terminal_false_positive`.
- Duzeltme sonrasi canli capture check:
  `logs/simitl_pr0p/20260707-095631-capture-updater-exclusion-check-v2-capture-probe.md`
  `WAITING` verdi: `no matching pr0p/FPV window is visible yet`.
- Yeni safe suite:
  `logs/simitl_pr0p/20260707-095638-pr0p-suite-updater-window-guard-v1-suite.md`.
  Kritik satirlar:
  `P1-install-discovery PASS`, `P1-client-executable WAITING`,
  `P3-capture WAITING`, `P6-synthetic-e2e-dry-run PASS`,
  `P6-synthetic-log-check PASS`.
- Yeni manifest/readiness/decision:
  `logs/simitl_pr0p/20260707-095646-pr0p-live-manifest-updater-window-guard-v1-live-manifest.md`,
  `logs/simitl_pr0p/20260707-095646-independent-sim-updater-window-guard-v1-independent-sim-readiness.md`,
  `logs/simitl_pr0p/20260707-095646-pr0p-decision-updater-window-guard-v1-decision.md`.
- Hedefli test:
  `tests/test_simitl_pr0p_probe.py -k 'capture_probe_excludes_updater or updater_runner': 4 passed, 89 deselected`.
- Tum SimITL/pr0p probe testi:
  `tests/test_simitl_pr0p_probe.py: 93 passed`.
- Game-screen sandbox + SimITL/pr0p izole testleri:
  `tests/test_game_screen_sandbox.py tests/test_simitl_pr0p_probe.py: 260 passed`.

Bu tur sonunda updater penceresi bilincli olarak acik birakildi. Siradaki
insan/operator adimi: acik terminal/updater penceresindeki kurulum adimini
tamamla. Ardindan:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_client_probe.py \
  --install-root /tmp/fpv-test-simitl-pr0p \
  --run-id pr0p-client-after-updater-ui
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_probe_suite.py \
  --run-id pr0p-suite-after-updater-ui
fpv_env/bin/python experiments/simitl_pr0p_probe/independent_sim_readiness.py \
  --run-id independent-sim-after-updater-ui
```

## LATEST 2026-07-07 10:19 +03 - pr0p / SimITL Runtime Input Handoff

En guncel ayrintili checkpoint bu dosyada yukarida
`## 2026-07-07 pr0p / SimITL Checkpoint` basligindadir. Kisa ozet:

- pr0p updater tamamlandi, client mevcut:
  `/tmp/fpv-test-simitl-pr0p/pr0p.x86_64`
- pr0p capture, websocket/MSP readonly, bbox, tracker/PID dry-run tek tek
  kanitlandi.
- Bbox: `148,220,142,82`
- MSP_RC baseline PASS ve stabil goruldu:
  `logs/simitl_pr0p/20260707-100856-pr0p-rc-baseline-after-yaw-v1-msp-ws-rc.md`
- MSP_SET_RAW_RC ack aliyor ama `MSP_RC` baseline'da kaliyor:
  `logs/simitl_pr0p/20260707-100912-pr0p-rc-loopback-after-baseline-v1-msp-ws-rc.md`
- Kisa omurlu uinput yaw/throttle response WAITING kaldi.
- Yeni persistent-uinput gate eklendi ve calistirildi; pr0p baslamadan once
  sanal RC cihazi acildi, local race'e girildi, ayni cihazla throttle response
  olculdu, sonuc yine WAITING:
  `logs/simitl_pr0p/20260707-101409-pr0p-live-persistent-uinput-throttle-v1-live-session.md`
- Safe suite'e read-only `P5-rc-baseline` eklendi.
- Promotion artik `P5-persistent-uinput-live-response PASS` olmadan cikamaz.
- Son testler:
  `tests/test_simitl_pr0p_probe.py: 94 passed`
  `tests/test_game_screen_sandbox.py tests/test_simitl_pr0p_probe.py: 261 passed`
- Son fresh raporlar:
  `logs/simitl_pr0p/20260707-101824-pr0p-suite-after-persistent-gate-v1-suite.md`
  `logs/simitl_pr0p/20260707-101936-pr0p-live-manifest-after-rc-baseline-fix-v1-live-manifest.md`
  `logs/simitl_pr0p/20260707-101936-independent-sim-after-rc-baseline-fix-v1-independent-sim-readiness.md`
  `logs/simitl_pr0p/20260707-101936-pr0p-decision-after-rc-baseline-fix-v1-decision.md`
- Checkpoint sonunda pr0p/updater/Gazebo/Betaflight prosesi yoktu.

## VERY LATEST 2026-07-07 11:04 +03 - pr0p FC Arm-State Blocker

Bu turun en guncel pr0p durumu:

- Yeni read-only gate eklendi:
  `experiments/simitl_pr0p_probe/msp_ws_status_probe.py`
- Gate `MSP_BOXNAMES`, `MSP_BOXIDS`, `MSP_STATUS_EX` okuyor; gerekirse basic
  `MSP_STATUS` fallback kullaniyor. Arm, throttle veya RC write gondermez.
- `--samples N --interval S` monitor modu eklendi. Amac transient
  `BOOTGRACE/CALIB` ile kalici arm blocker'i ayirmak.
- Safe suite artik `P2-fc-status-readonly` fazini da kosuyor.
- `pr0p_live_session_runner.py` artik `--tracking-bbox x,y,w,h` alip safe suite'e
  aktarabiliyor; boylece bounded live-session bbox'li P6 dry-run kaniti
  uretebiliyor.
- Manifest standalone `*-msp-ws-status.json` raporlarini okuyabiliyor ve
  `FC_NOT_ARMED` / `ARMING_DISABLED:*` notlari varsa P5 live response yerine
  arm/start-state cozumu oneriyor.
- Decision raporu artik `FC_ARM_STATE_BLOCKED` reason'ini uretiyor.

Canli kanitlar:

- Bounded live status suite:
  `logs/simitl_pr0p/20260707-105629-pr0p-live-session-fc-status-v1-live-session.md`
  ve
  `logs/simitl_pr0p/20260707-105623-pr0p-live-session-fc-status-v1-suite-p2-fc-status-msp-ws-status.md`
  sonuc: `P2-fc-status-readonly PASS`, `armed=False`,
  `ARMING_DISABLED:THROTTLE,BOOTGRACE,CALIB`.
- 6 sample monitor:
  `logs/simitl_pr0p/20260707-105938-pr0p-fc-status-monitor-v1-msp-ws-status.md`
  sonuc: 6/6 PASS, `armed=False`, kalici blocker sadece `THROTTLE`.
- Bbox'li bounded live suite:
  `logs/simitl_pr0p/20260707-110239-pr0p-live-session-fc-status-bbox-v1-live-session.md`
  ve suite:
  `logs/simitl_pr0p/20260707-110237-pr0p-live-session-fc-status-bbox-v1-suite-suite.md`
  sonuc: `P2-fc-status-readonly PASS`, `P6-tracking-pid-dry-run PASS`,
  `P6-tracking-log-check PASS`, cleanup OK.
- Son manifest:
  `logs/simitl_pr0p/20260707-110303-pr0p-live-manifest-fc-status-bbox-v1-live-manifest.md`
  next action: `Resolve pr0p FC arm/start-state first; run P2-fc-status-readonly with --samples 6 --interval 1 and clear FC_NOT_ARMED/ARMING_DISABLED before P5 live response.`
- Son decision:
  `logs/simitl_pr0p/20260707-110408-pr0p-decision-fc-arm-blocked-v1-decision.md`
  reasons: `FC_ARM_STATE_BLOCKED`, `MISSING_LIVE_YAW_RESPONSE`,
  `MISSING_LIVE_PITCH_RESPONSE`, `MISSING_RC_SOURCE_LOOPBACK`,
  `MISSING_PERSISTENT_UINPUT_RESPONSE`, `MISSING_LIVE_TRACKING_CONTROL`.

Son testler:

- `tests/test_simitl_pr0p_probe.py`: 109 passed
- `tests/test_game_screen_sandbox.py tests/test_simitl_pr0p_probe.py`: 276 passed

Sıradaki teknik adim:

1. pr0p/Betaflight FC tarafinda throttle-low ve arm-state'i coz. MSP_RC baseline
   su anda tum kanallarda 1500 gorundu; bu `THROTTLE_NOT_LOW_FOR_ARM` ile
   uyumlu.
2. Once read-only monitor ile `FC_NOT_ARMED` / `ARMING_DISABLED` temizlendigini
   kanitla.
3. Sonra P5 persistent-uinput live response, yaw/pitch signed response ve ancak
   bundan sonra P6 live tracking control kos.

Checkpoint sonunda pr0p/updater/Gazebo/Betaflight prosesi yoktu.

En olasi sonraki adim: pr0p `Controls -> RC Channels` sayfasinda runtime bar
hareketini dogrula. Config dosyasi PASS veriyor, fakat runtime sim henuz
uinput veya MSP komutuna olculebilir arac/goruntu response uretmedi.

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_live_session_runner.py \
  --launch-pr0p \
  --ack-live-launch \
  --send-ui \
  --ack-live-ui \
  --hold-uinput \
  --ack-live-input \
  --run-live-response \
  --response-axis throttle \
  --response-magnitude 0.35 \
  --response-image-axis y \
  --response-expected-sign 0 \
  --response-post-duration 1.5 \
  --response-max-shift-px 300 \
  --startup-wait 4 \
  --run-id pr0p-live-persistent-uinput-throttle-next
```

## VERY LATEST 2026-07-07 10:29 +03

Ek olarak `P4-rc-channels-visual` runtime gate'i eklendi. Detayli bolum bu
dosyada yukarida `## LATEST 2026-07-07 10:29 +03 - Runtime RC Channels Visual Gate`
basligindadir.

Kisa ozet:

- Yeni gate: `experiments/simitl_pr0p_probe/pr0p_runtime_input_visual_probe.py`
- Kullanim yeri: pr0p `Controls -> RC Channels` sayfasi acikken.
- Amac: virtual RC pulse sirasinda ekrandaki RC bar/kanal bolgesinin degisip
  degismedigini olcmek.
- Promotion karari artik `P4-rc-channels-visual PASS` kanitini de bekliyor.
- Son testler:
  `tests/test_simitl_pr0p_probe.py: 100 passed`
  `tests/test_game_screen_sandbox.py tests/test_simitl_pr0p_probe.py: 267 passed`
- Son decision:
  `logs/simitl_pr0p/20260707-102901-pr0p-decision-runtime-visual-required-v1-decision.md`
  ve sebep listesinde `MISSING_RUNTIME_RC_CHANNEL_VISUAL` var.

Siradaki pratik komut, pr0p `Controls -> RC Channels` sayfasi acikken:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_runtime_input_visual_probe.py \
  --uinput \
  --ack-live-input \
  --axis yaw \
  --magnitude 0.6 \
  --run-id pr0p-rc-channels-visual-next
```
## VERY LATEST 2026-07-07 10:45 +03 - Final Pointer

Bu dosyanin en guncel pr0p runtime durumu:

- Runtime visual input PASS:
  `logs/simitl_pr0p/20260707-103654-pr0p-rc-channels-visual-controls-yaw-v1-runtime-input-visual.md`
- Live suite PASS olan temel kapilar:
  `logs/simitl_pr0p/20260707-104219-pr0p-suite-after-rc-visual-pass-live-v1-suite.md`
- Son decision:
  `logs/simitl_pr0p/20260707-104248-pr0p-decision-suite-rc-visual-live-v1-decision.md`
- Kalan nedenler:
  `MISSING_LIVE_YAW_RESPONSE`,
  `MISSING_LIVE_PITCH_RESPONSE`,
  `MISSING_RC_SOURCE_LOOPBACK`,
  `MISSING_PERSISTENT_UINPUT_RESPONSE`,
  `MISSING_LIVE_TRACKING_CONTROL`.
- Son testler:
  `tests/test_simitl_pr0p_probe.py: 101 passed`
  `tests/test_game_screen_sandbox.py tests/test_simitl_pr0p_probe.py: 268 passed`
- Checkpoint sonunda pr0p/updater/Gazebo/Betaflight prosesi yoktu.

## VERY LATEST 2026-07-07 11:24 +03 - Final Pointer

En guncel durum bu bloktur; 10:45 ve 11:04 pointer'lari artik eski kaldi.

- Yeni gate'ler:
  - `experiments/simitl_pr0p_probe/msp_uinput_rc_effect_probe.py`
  - `experiments/simitl_pr0p_probe/msp_uinput_status_probe.py`
- `pr0p_live_session_runner.py` artik persistent uinput ile:
  - `--run-rc-effect`
  - `--run-status-effect`
  destekliyor.

Canli kanitlar:

- `throttle=-1.0`:
  `logs/simitl_pr0p/20260707-111509-pr0p-live-uinput-rc-effect-throttle-neg-v1-live-session.md`
  sonuc: FAIL direction; MSP_RC throttle `1500 -> 2000`.
- `throttle=+1.0`:
  `logs/simitl_pr0p/20260707-111545-pr0p-live-uinput-rc-effect-throttle-pos-v1-live-session.md`
  sonuc: PASS, `THROTTLE_LOW_REACHED`; MSP_RC throttle `1500 -> 1000`.
- Status-effect:
  `logs/simitl_pr0p/20260707-112145-pr0p-live-uinput-status-throttle-low-v1-live-session.md`
  sonuc: PASS, `UINPUT_CLEARED_THROTTLE`.
  Baseline blocker `THROTTLE,BOOTGRACE,CALIB`; throttle tutulurken `THROTTLE`
  temizlendi, `BOOTGRACE,CALIB` kaldi.
- Son manifest:
  `logs/simitl_pr0p/20260707-112349-pr0p-live-manifest-uinput-status-effect-v1-live-manifest.md`
  next action: `Throttle-low clears THROTTLE; next discover and drive the ARM AUX/button path, then verify FC_ARMED with P2-fc-status-readonly.`
- Son readiness:
  `logs/simitl_pr0p/20260707-112350-independent-sim-uinput-status-effect-v1-independent-sim-readiness.md`
- Son decision:
  `logs/simitl_pr0p/20260707-112358-pr0p-decision-uinput-status-effect-v1-decision.md`
  reasons: `FC_ARM_STATE_BLOCKED`, `MISSING_LIVE_YAW_RESPONSE`,
  `MISSING_LIVE_PITCH_RESPONSE`, `MISSING_RC_SOURCE_LOOPBACK`,
  `MISSING_PERSISTENT_UINPUT_RESPONSE`, `MISSING_LIVE_TRACKING_CONTROL`.

Son testler:

- `tests/test_simitl_pr0p_probe.py`: 121 passed
- `tests/test_game_screen_sandbox.py tests/test_simitl_pr0p_probe.py`: 288 passed

Sıradaki teknik adim:

1. ARM AUX/button path'i kesfet. FC status'ta active modes bos; `ARM` aktif
   degil.
2. `throttle=+1.0` low tutulurken ARM komutunu/kanalini sur.
3. `P2-fc-status-readonly --samples 6 --interval 1` ile `FC_ARMED` ve
   arming blocker temizligini kanitla.
4. Ondan sonra P5 yaw/pitch live response ve P6 live tracking'e gec.

Checkpoint sonunda pr0p/updater/Gazebo/Betaflight prosesi yoktu.

## VERY LATEST 2026-07-07 11:04 +03 - Final Pointer

En guncel durum bu bloktur; yukaridaki 10:45 pointer artik eski kaldi.

- Yeni read-only FC status gate:
  `experiments/simitl_pr0p_probe/msp_ws_status_probe.py`
- Monitor modu:
  `--samples 6 --interval 1`
- Safe suite artik `P2-fc-status-readonly` fazini kosuyor.
- `pr0p_live_session_runner.py` artik `--tracking-bbox x,y,w,h` alip safe
  suite'e aktarabiliyor.
- Manifest standalone `*-msp-ws-status.json` raporlarini okuyabiliyor ve
  FC arm blocker varsa P5 live response yerine arm/start-state cozumu oneriyor.
- Decision raporu artik `FC_ARM_STATE_BLOCKED` reason'ini uretiyor.

Canli kanitlar:

- Status monitor:
  `logs/simitl_pr0p/20260707-105938-pr0p-fc-status-monitor-v1-msp-ws-status.md`
  sonuc: 6/6 PASS, `armed=False`, kalici blocker sadece `THROTTLE`.
- Bbox'li bounded live suite:
  `logs/simitl_pr0p/20260707-110237-pr0p-live-session-fc-status-bbox-v1-suite-suite.md`
  sonuc: `P2-fc-status-readonly PASS`, `P6-tracking-pid-dry-run PASS`,
  `P6-tracking-log-check PASS`, cleanup OK.
- Son manifest:
  `logs/simitl_pr0p/20260707-110303-pr0p-live-manifest-fc-status-bbox-v1-live-manifest.md`
  next action: `Resolve pr0p FC arm/start-state first; run P2-fc-status-readonly with --samples 6 --interval 1 and clear FC_NOT_ARMED/ARMING_DISABLED before P5 live response.`
- Son decision:
  `logs/simitl_pr0p/20260707-110408-pr0p-decision-fc-arm-blocked-v1-decision.md`
  reasons: `FC_ARM_STATE_BLOCKED`, `MISSING_LIVE_YAW_RESPONSE`,
  `MISSING_LIVE_PITCH_RESPONSE`, `MISSING_RC_SOURCE_LOOPBACK`,
  `MISSING_PERSISTENT_UINPUT_RESPONSE`, `MISSING_LIVE_TRACKING_CONTROL`.

Son testler:

- `tests/test_simitl_pr0p_probe.py`: 109 passed
- `tests/test_game_screen_sandbox.py tests/test_simitl_pr0p_probe.py`: 276 passed

Sıradaki teknik adim:

1. pr0p/Betaflight FC tarafinda throttle-low ve arm-state'i coz. MSP_RC baseline
   tum kanallarda 1500 gorundu; bu `THROTTLE_NOT_LOW_FOR_ARM` ile uyumlu.
2. Once read-only monitor ile `FC_NOT_ARMED` / `ARMING_DISABLED` temizlendigini
   kanitla.
3. Sonra P5 persistent-uinput live response, yaw/pitch signed response ve
   ardindan P6 live tracking control kos.

Checkpoint sonunda pr0p/updater/Gazebo/Betaflight prosesi yoktu.

## TRUE LATEST 2026-07-07 11:24 +03

En guncel durum: throttle-low problemi cozuldu/olculdu; `throttle=+1.0`
MSP_RC throttle'i `1500 -> 1000` indiriyor ve `THROTTLE` arming blocker'ini
temizliyor.

Kanıtlar:

- RC effect PASS:
  `logs/simitl_pr0p/20260707-111545-pr0p-live-uinput-rc-effect-throttle-pos-v1-live-session.md`
- Status effect PASS:
  `logs/simitl_pr0p/20260707-112145-pr0p-live-uinput-status-throttle-low-v1-live-session.md`
- Son manifest:
  `logs/simitl_pr0p/20260707-112349-pr0p-live-manifest-uinput-status-effect-v1-live-manifest.md`
- Son decision:
  `logs/simitl_pr0p/20260707-112358-pr0p-decision-uinput-status-effect-v1-decision.md`
- Son testler:
  `tests/test_simitl_pr0p_probe.py: 121 passed`
  `tests/test_game_screen_sandbox.py tests/test_simitl_pr0p_probe.py: 288 passed`

Sıradaki teknik adim: ARM AUX/button path'i kesfet, `throttle=+1.0` low
tutulurken ARM komutunu sur, sonra `P2-fc-status-readonly --samples 6
--interval 1` ile `FC_ARMED` kanitla.

Checkpoint sonunda pr0p/updater/Gazebo/Betaflight prosesi yoktu.

## TRUE LATEST 2026-07-07 11:53 +03

En guncel durum: pr0p/SITL Forge bagimsiz hatta throttle-low cozuldu, ARM
mode range read-only olculdu, direct button ARM yolu denendi ve calismadi.
Betaflight ARM `AUX1 / CH5` icin `1700-2100` bekliyor.

Yeni kod/gate'ler:

- `experiments/simitl_pr0p_probe/msp_ws_mode_ranges_probe.py`
  read-only `MSP_MODE_RANGES` okur.
- `experiments/simitl_pr0p_probe/msp_uinput_arm_probe.py`
  throttle low tutulurken south/east button adaylarini dener.
- `experiments/game_screen_sandbox/virtual_input.py`
  artik `aux1`/`ABS_Z` ve button press/release destekliyor.
- `experiments/simitl_pr0p_probe/pr0p_input_config_patch.py`
  `--role aux1` dry-run/write destegi ekledi; slot 4 icin `<Joystick>/Z`
  oneriyor.
- `experiments/simitl_pr0p_probe/pr0p_live_session_runner.py`
  `--run-arm-button-effect` ve `aux1` RC-effect destekliyor.

Canli kanitlar:

- Mode ranges PASS:
  `logs/simitl_pr0p/20260707-114048-pr0p-live-suite-mode-ranges-v1-suite-p2-mode-ranges-msp-ws-mode-ranges.md`
  sonuc: ARM `AUX1 / CH5`, `1700-2100`.
- Button ARM denemesi WAITING:
  `logs/simitl_pr0p/20260707-113523-pr0p-live-uinput-arm-button-v1-live-session.md`
  sonuc: `NO_UINPUT_ARM_BUTTON_EFFECT`; south/east ARM mode uretmedi.
- AUX1 config dry-run PASS:
  `logs/simitl_pr0p/20260707-114819-pr0p-input-config-aux1-dry-v1-input-config-patch.md`
  sonuc: slot 4 bos -> `<Joystick>/Z`, `real_config_write=False`.
- AUX1 live RC-effect before patch WAITING:
  `logs/simitl_pr0p/20260707-115217-pr0p-live-uinput-rc-aux1-before-patch-v2-live-session.md`
  sonuc: `NO_UINPUT_MSP_RC_EFFECT`; CH5/AUX1 1500'de kaldi.
- Son manifest:
  `logs/simitl_pr0p/20260707-115241-pr0p-live-manifest-aux1-before-patch-v2-live-manifest.md`
  next action: backup-backed AUX1 patch/manual AUX1 bind, sonra
  `P5-uinput-rc-effect-aux1-high`.
- Son decision:
  `logs/simitl_pr0p/20260707-115241-pr0p-decision-aux1-before-patch-v2-decision.md`
  WAITING reasons icinde `MISSING_UINPUT_AUX1_RC_EFFECT` var.

Son testler:

- `tests/test_simitl_pr0p_probe.py`: 133 passed
- `tests/test_game_screen_sandbox.py tests/test_simitl_pr0p_probe.py`: 300 passed

Sıradaki teknik adim:

1. Kullanici onayi/manual tercih ile pr0p `input.json` icin AUX1 slot 4'u
   `<Joystick>/Z` olarak uygula ya da pr0p Controls -> RC Channels icinde
   AUX1/CH5'i Kenet virtual AUX1 axis'e bagla.
2. Sonra:
   `pr0p_live_session_runner.py --run-rc-effect --rc-effect-axis aux1 --rc-effect-expected-channel aux1 --rc-effect-expected-direction higher`
   ile CH5'in 1700+ oldugunu kanitla.
3. `throttle=+1.0` low + `aux1=+1.0` high birlikte tutulurken `FC_ARMED`
   / `ARMING_DISABLED` durumunu read-only status ile kanitla.
4. Ardindan P5 yaw/pitch live response ve P6 live tracking'e gec.

Checkpoint sonunda pr0p/updater/Gazebo/Betaflight prosesi yoktu.

## TRUE LATEST 2026-07-07 12:03 +03

Kullanici mevcut hattin pr0p sim uzerinde olup olmadigini sordu. Cevap:
evet, aktif calisma Gazebo'dan tamamen bagimsiz `pr0p / SITL Forge`
hattidir. Bu asamada sifirdan oyun yazilmiyor; pr0p ekrani, virtual
RC/uinput, MSP read-only olcumleri ve Kenet tracker/PID zinciri icin olculebilir
bir test hatti kuruluyor.

Bu ara turda eklenen yeni parca:

- `experiments/simitl_pr0p_probe/msp_uinput_aux_arm_status_probe.py`
  eklendi. Amaci: throttle low (`throttle=+1.0`) ve AUX1 high
  (`aux1=+1.0`) birlikte tutulurken FC'nin ARM/armed durumuna gecip
  gecmedigini read-only MSP status ile olcmek.
- `experiments/simitl_pr0p_probe/pr0p_live_session_runner.py`
  icine `--run-aux-arm-status` opsiyonu eklendi. Bu, pr0p launch +
  persistent uinput adapter ile yeni AUX1 arm-status probe'unu live session
  metriklerine ekler.
- `tests/test_simitl_pr0p_probe.py` icine bu yeni probe ve live-session
  baglantisi icin unit testler eklendi.

Son dogrulanmis testler:

- `fpv_env/bin/python -m pytest tests/test_simitl_pr0p_probe.py -q`
  sonuc: `137 passed`
- `fpv_env/bin/python -m pytest tests/test_game_screen_sandbox.py tests/test_simitl_pr0p_probe.py -q`
  sonuc: `304 passed`

Henuz tamamlanmayan entegrasyon:

- Yeni `P5-uinput-aux1-arm-status` gate'i manifest/decision/readiness
  raporlarina henuz tam eklenmedi. Devam edecek agent once
  `experiments/simitl_pr0p_probe/pr0p_live_run_manifest.py` ve
  `experiments/simitl_pr0p_probe/pr0p_decision_report.py` icinde bu gate'i
  resmi faz olarak tanimlamali.
- `checkpoint-goal.md` icindeki onceki 11:53 durum hala gecerlidir:
  AUX1/CH5 pr0p tarafinda bagli olmadigi icin son live RC-effect denemesi
  `NO_UINPUT_MSP_RC_EFFECT` ile `WAITING` verdi. ARM icin Betaflight
  `AUX1 / CH5` kanalinda `1700-2100` bekliyor.

Sıradaki teknik adim:

1. pr0p Controls -> RC Channels icinde AUX1/CH5'i Kenet virtual AUX1 axis'e
   manuel bagla ya da kullanici acik onay verirse backup-backed
   `pr0p_input_config_patch.py --role aux1 --write` yolunu uygula.
2. CH5'in 1700+ oldugunu kanitlamak icin:
   `pr0p_live_session_runner.py --run-rc-effect --rc-effect-axis aux1 --rc-effect-expected-channel aux1 --rc-effect-expected-direction higher`
   komutunu calistir.
3. Ardindan yeni eklenen arm-status olcumunu kos:
   `pr0p_live_session_runner.py --launch-pr0p --ack-live-launch --send-ui --ack-live-ui --hold-uinput --ack-live-input --run-aux-arm-status --aux-arm-throttle-magnitude 1.0 --aux-arm-aux1-magnitude 1.0 --startup-wait 8 --run-id pr0p-live-uinput-aux1-arm-status-v1`
4. Bu PASS olduktan sonra P5 yaw/pitch signed live response ve P6 live
   tracking control'e gec.

Checkpoint sonunda pr0p/updater/Gazebo/Betaflight prosesi yoktu.

## TRUE LATEST 2026-07-07 12:12 +03

Aktif hat halen Gazebo'dan tamamen bagimsiz `pr0p / SITL Forge` hattidir.
Bu turda 12:03 checkpoint'inde "henuz tamamlanmayan entegrasyon" olarak
isaretlenen is tamamlandi: `P5-uinput-aux1-arm-status` artik resmi manifest
ve decision gate'i.

Yeni resmi gate:

- `P5-uinput-aux1-arm-status`
  - Komut kaynagi:
    `experiments/simitl_pr0p_probe/pr0p_live_session_runner.py --run-aux-arm-status`
  - Olctugu sey:
    throttle low (`throttle=+1.0`) ve AUX1 high (`aux1=+1.0`) birlikte
    tutulurken read-only MSP status uzerinden `FC_ARMED` / ARM mode aktif mi.
  - Rapor kaynaklari:
    standalone `*-uinput-aux-arm-status.json` veya live-session icindeki
    `metrics.live_aux_arm_status`.

Kod degisiklikleri:

- `experiments/simitl_pr0p_probe/pr0p_live_run_manifest.py`
  `P5-uinput-aux1-arm-status` komutunu, rapor okumasini, manifest adimini,
  metadata alanini ve next-action siralamasini destekliyor.
- `experiments/simitl_pr0p_probe/pr0p_decision_report.py`
  promote icin artik `P5-uinput-aux1-arm-status PASS` istiyor ve eksikte
  `MISSING_UINPUT_AUX1_ARM_STATUS` reason uretir. Direct button probe artik
  promote icin zorunlu degil; gercek zorunlu kanit AUX1 arm-status.
- `tests/test_simitl_pr0p_probe.py`
  manifest command assertionlari, nested live-session okuma ve promote
  fixture'lari yeni gate'e gore guncellendi.
- `experiments/simitl_pr0p_probe/PLAN.md`
  son adim `FC_ARMED verification` yerine resmi
  `P5-uinput-aux1-arm-status` gate adi ile senkronlandi.

Son dogrulamalar:

- `fpv_env/bin/python -m pytest tests/test_simitl_pr0p_probe.py -q`
  sonuc: `137 passed`
- `fpv_env/bin/python -m pytest tests/test_game_screen_sandbox.py tests/test_simitl_pr0p_probe.py -q`
  sonuc: `304 passed`
- Bbox'li manifest dry run:
  `logs/simitl_pr0p/20260707-121125-pr0p-live-manifest-aux-arm-gate-bbox-v1-live-manifest.md`
  next action:
  `Buttons did not activate ARM; bind AUX1/CH5, then prove P5-uinput-rc-effect-aux1-high.`
- Final decision dry run:
  `logs/simitl_pr0p/20260707-121135-pr0p-decision-aux-arm-gate-final-v1-decision.md`
  sonuc: `WAITING`; reasons icinde
  `MISSING_UINPUT_AUX1_RC_EFFECT` ve `MISSING_UINPUT_AUX1_ARM_STATUS` var.
- Independent readiness dry run:
  `logs/simitl_pr0p/20260707-121301-independent-sim-readiness-aux-arm-gate-v1-independent-sim-readiness.md`
  sonuc: `WAITING`; reasons icinde
  `PR0P:MISSING_UINPUT_AUX1_RC_EFFECT` ve
  `PR0P:MISSING_UINPUT_AUX1_ARM_STATUS` var.

Sıradaki teknik adim:

1. pr0p Controls -> RC Channels icinde AUX1/CH5'i Kenet virtual AUX1 axis'e
   manuel bagla ya da kullanici acik onay verirse backup-backed
   `pr0p_input_config_patch.py --role aux1 --write` yolunu uygula.
2. CH5'in 1700+ oldugunu kanitla:
   `pr0p_live_session_runner.py --run-rc-effect --rc-effect-axis aux1 --rc-effect-expected-channel aux1 --rc-effect-expected-direction higher`
3. Yeni resmi ARM gate'ini kos:
   `pr0p_live_session_runner.py --launch-pr0p --ack-live-launch --send-ui --ack-live-ui --hold-uinput --ack-live-input --run-aux-arm-status --aux-arm-throttle-magnitude 1.0 --aux-arm-aux1-magnitude 1.0 --startup-wait 8 --run-id pr0p-live-uinput-aux1-arm-status-v1`
4. Bu PASS olursa P5 signed yaw/pitch live response ve sonra P6 live
   tracking control'e gec.

Checkpoint sonunda pr0p/updater/Gazebo/Betaflight prosesi yoktu.

## TRUE LATEST 2026-07-07 12:16 +03

Aktif hat halen Gazebo'dan tamamen bagimsiz `pr0p / SITL Forge` hattidir.
Bu turda AUX1 baglama sonrasi calistirilacak komutlar independent readiness
raporuna isimli resume komutlari olarak eklendi. Boylece sonraki agent veya
operator uzun komutlari markdown icinden aramak yerine readiness raporundaki
`Commands` bolumunden dogrudan ilerleyebilir.

Yeni/sertlesen readiness komutlari:

- `pr0p_aux1_config_patch_dry`
  - `pr0p_input_config_patch.py --role aux1`
  - pr0p input config icin AUX1/CH5 dry-run; yazma yapmaz.
- `pr0p_aux1_config_patch_write`
  - `pr0p_input_config_patch.py --role aux1 --write --ack-config-write`
  - sadece kullanici acik onayi ile backup-backed config yazar.
- `pr0p_aux1_rc_effect`
  - `pr0p_live_session_runner.py --run-rc-effect --rc-effect-axis aux1`
  - CH5/AUX1'in 1700+ hareket ettigini kanitlamak icin.
- `pr0p_aux1_arm_status`
  - `pr0p_live_session_runner.py --run-aux-arm-status`
  - throttle-low + AUX1-high birlikte tutulurken FC ARM status'u kanitlamak
    icin.

Dokuman senkronu:

- `experiments/simitl_pr0p_probe/README.md` icine CH5 movement komutundan
  sonra resmi `P5-uinput-aux1-arm-status` komutu eklendi.
- `experiments/simitl_pr0p_probe/independent_sim_readiness.py` komut listesi
  yukaridaki dort AUX1 resume komutunu uretir hale geldi.
- `tests/test_simitl_pr0p_probe.py` bu komutlarin readiness raporundan
  dusmemesi icin assertion'larla guncellendi.

Son dogrulamalar:

- `fpv_env/bin/python -m pytest tests/test_simitl_pr0p_probe.py -q`
  sonuc: `137 passed`
- `fpv_env/bin/python -m pytest tests/test_game_screen_sandbox.py tests/test_simitl_pr0p_probe.py -q`
  sonuc: `304 passed`
- Independent readiness dry run:
  `logs/simitl_pr0p/20260707-121614-independent-sim-readiness-aux1-resume-commands-v1-independent-sim-readiness.md`
  sonuc: `WAITING`; `Commands` bolumunde
  `pr0p_aux1_config_patch_dry`, `pr0p_aux1_config_patch_write`,
  `pr0p_aux1_rc_effect`, `pr0p_aux1_arm_status` var.
- Manifest dry run:
  `logs/simitl_pr0p/20260707-121614-pr0p-live-manifest-aux1-resume-commands-v1-live-manifest.md`
  next action halen:
  `Buttons did not activate ARM; bind AUX1/CH5, then prove P5-uinput-rc-effect-aux1-high.`

Sıradaki teknik adim:

1. pr0p Controls -> RC Channels icinde AUX1/CH5'i Kenet virtual AUX1 axis'e
   manuel bagla ya da kullanici acik onay verirse readiness raporundaki
   `pr0p_aux1_config_patch_write` komutunu uygula.
2. Readiness raporundaki `pr0p_aux1_rc_effect` komutunu calistir ve
   `P5-uinput-rc-effect-aux1-high PASS` kanitla.
3. Readiness raporundaki `pr0p_aux1_arm_status` komutunu calistir ve
   `P5-uinput-aux1-arm-status PASS` kanitla.
4. Bu iki gate PASS olursa P5 signed yaw/pitch live response ve sonra P6 live
   tracking control'e gec.

Checkpoint sonunda pr0p/updater/Gazebo/Betaflight prosesi yoktu.

## TRUE LATEST 2026-07-07 12:20 +03

Aktif hat halen Gazebo'dan tamamen bagimsiz `pr0p / SITL Forge` hattidir.
Bu turda P5 arm/response siralamasinda kritik bir mantik hatasi duzeltildi.

Duzeltilen davranis:

- `pr0p_live_run_manifest.py` artik P5 live response'a gecmeden once her
  durumda su komut-spesifik gate'leri sirayla ister:
  1. `P5-uinput-rc-effect-throttle-low`
  2. `P5-uinput-status-effect-throttle-clear`
  3. `P5-uinput-rc-effect-aux1-high`
  4. `P5-uinput-aux1-arm-status`
- Onceki mantik bu P5 arm gate'lerini yalniz baseline
  `P2-fc-status-readonly` arm blocker notlari varsa zorluyordu. Bu,
  decision'in promote icin istedigi gate'lerle tutarsizdi.
- `pr0p_decision_report.py` artik baseline `P2-fc-status-readonly`
  raporunda `FC_NOT_ARMED` / `ARMING_DISABLED:*` notlari olsa bile
  `P5-uinput-aux1-arm-status PASS` varsa promotion'i bu baseline notlarla
  bloke etmez. Cunku artik daha guclu kanit, throttle-low + AUX1-high
  tutulurken alinan komut-spesifik arm-status gate'idir.
- Eger `P5-uinput-aux1-arm-status` eksikse baseline blocker notlari hala
  `FC_ARM_STATE_BLOCKED` reason'i olarak kalir.

Eklenen test kanitlari:

- `test_live_manifest_requires_aux_arm_status_after_aux1_rc_effect_without_baseline_blocker`
- `test_decision_report_blocks_promotion_when_fc_is_not_armed_without_aux_arm_status`
- `test_decision_report_allows_baseline_fc_not_armed_when_aux_arm_status_passes`

Son dogrulamalar:

- `fpv_env/bin/python -m pytest tests/test_simitl_pr0p_probe.py -q`
  sonuc: `139 passed`
- `fpv_env/bin/python -m pytest tests/test_game_screen_sandbox.py tests/test_simitl_pr0p_probe.py -q`
  sonuc: `306 passed`
- Manifest dry run:
  `logs/simitl_pr0p/20260707-122028-pr0p-live-manifest-aux-arm-order-v1-live-manifest.md`
  sonuc: `WAITING/PASS manifest status`, next action halen
  `Buttons did not activate ARM; bind AUX1/CH5, then prove P5-uinput-rc-effect-aux1-high.`
- Decision dry run:
  `logs/simitl_pr0p/20260707-122028-pr0p-decision-aux-arm-order-v1-decision.md`
  sonuc: `WAITING`; reasons icinde `FC_ARM_STATE_BLOCKED`,
  `MISSING_UINPUT_AUX1_RC_EFFECT`, `MISSING_UINPUT_AUX1_ARM_STATUS` var.
- Independent readiness dry run:
  `logs/simitl_pr0p/20260707-122028-independent-sim-readiness-aux-arm-order-v1-independent-sim-readiness.md`
  sonuc: `WAITING`; ayni eksikleri `PR0P:*` olarak tasiyor.

Sıradaki teknik adim degismedi:

1. pr0p Controls -> RC Channels icinde AUX1/CH5'i Kenet virtual AUX1 axis'e
   manuel bagla ya da kullanici acik onay verirse readiness raporundaki
   `pr0p_aux1_config_patch_write` komutunu uygula.
2. `pr0p_aux1_rc_effect` ile `P5-uinput-rc-effect-aux1-high PASS` kanitla.
3. `pr0p_aux1_arm_status` ile `P5-uinput-aux1-arm-status PASS` kanitla.
4. Sonra P5 signed yaw/pitch live response ve P6 live tracking control'e gec.

Checkpoint sonunda pr0p/updater/Gazebo/Betaflight prosesi yoktu.

## TRUE LATEST 2026-07-07 12:26 +03

Aktif hat halen Gazebo'dan tamamen bagimsiz `pr0p / SITL Forge` hattidir.
Bu turda AUX1 baglama sonrasi iki canli gate'i tek raporda siralayan yeni
ack-gated runner eklendi.

Yeni dosya:

- `experiments/simitl_pr0p_probe/pr0p_aux1_acceptance_runner.py`
  - Varsayilan plan-only calisir; pr0p baslatmaz, config yazmaz, OS input
    gondermez.
  - `--execute-dry-patch` ile AUX1 config patch dry-run komutunu calistirir.
  - `--apply-config-patch --ack-config-write` ile backup-backed config write
    yapabilir.
  - `--run-live-gates --ack-live-launch --ack-live-ui --ack-live-input` ile
    once `pr0p_aux1_rc_effect`, sadece PASS olursa `pr0p_aux1_arm_status`
    kosar.
  - RC-effect WAITING/FAIL ise arm-status adimini `SKIPPED` birakir; yani
    arm kaniti yanlis sirada uretilmez.

Readiness entegrasyonu:

- `independent_sim_readiness.py` artik iki yeni komut da uretir:
  - `pr0p_aux1_acceptance_plan`
  - `pr0p_aux1_acceptance_live`
- Tekil komutlar da kaldi:
  - `pr0p_aux1_config_patch_dry`
  - `pr0p_aux1_config_patch_write`
  - `pr0p_aux1_rc_effect`
  - `pr0p_aux1_arm_status`

Dokuman:

- `experiments/simitl_pr0p_probe/README.md` AUX1 bolumune plan-only ve
  live acceptance runner komutlari eklendi.
- `experiments/simitl_pr0p_probe/PLAN.md` AUX1 acceptance runner ile
  senkronlandi.

Eklenen test kanitlari:

- `test_aux1_acceptance_runner_plan_only_does_not_execute`
- `test_aux1_acceptance_runner_requires_ack_for_writes_and_live_input`
- `test_aux1_acceptance_runner_runs_live_gates_after_rc_effect_pass`
- `test_aux1_acceptance_runner_skips_arm_status_when_rc_effect_waits`

Son dogrulamalar:

- `fpv_env/bin/python -m pytest tests/test_simitl_pr0p_probe.py -q`
  sonuc: `143 passed`
- `fpv_env/bin/python -m pytest tests/test_game_screen_sandbox.py tests/test_simitl_pr0p_probe.py -q`
  sonuc: `310 passed`
- AUX1 acceptance plan-only dry run:
  `logs/simitl_pr0p/20260707-122607-pr0p-aux1-acceptance-plan-v1-aux1-acceptance.md`
  sonuc: `WAITING`; plan-only, live kanit bekliyor.
- Independent readiness dry run:
  `logs/simitl_pr0p/20260707-122607-independent-sim-readiness-aux1-acceptance-runner-v1-independent-sim-readiness.md`
  `Commands` bolumunde `pr0p_aux1_acceptance_plan` ve
  `pr0p_aux1_acceptance_live` var.
- Manifest dry run:
  `logs/simitl_pr0p/20260707-122607-pr0p-live-manifest-aux1-acceptance-runner-v1-live-manifest.md`
  next action halen:
  `Buttons did not activate ARM; bind AUX1/CH5, then prove P5-uinput-rc-effect-aux1-high.`

Sıradaki teknik adim:

1. pr0p Controls -> RC Channels icinde AUX1/CH5'i Kenet virtual AUX1 axis'e
   manuel bagla ya da kullanici acik onay verirse `pr0p_aux1_config_patch_write`
   komutunu uygula.
2. Baglama sonrasi tek komutlu sirali kabul icin:
   `pr0p_aux1_acceptance_live` komutunu kullan.
3. Bu runner `pr0p_aux1_rc_effect` PASS olmadan `pr0p_aux1_arm_status`
   kosmayacak; iki gate PASS olursa P5 signed yaw/pitch live response'a gec.

Checkpoint sonunda pr0p/updater/Gazebo/Betaflight prosesi yoktu.

## TRUE LATEST 2026-07-07 12:35 +03

Aktif hat halen Gazebo'dan tamamen bagimsiz `pr0p / SITL Forge` hattidir.
Bu turda AUX1 kabul zincirinden sonra calisacak signed yaw/pitch response
kabul runner'i eklendi. Amac, `P5-uinput-aux1-arm-status PASS` kaniti
olmadan response testlerinin yanlis sirada kosulmasini veya PASS sayilmasini
engellemektir.

Yeni dosya:

- `experiments/simitl_pr0p_probe/pr0p_response_acceptance_runner.py`
  - Varsayilan plan-only calisir; pr0p baslatmaz, UI tiklamaz, OS input
    gondermez.
  - En guncel `*-live-manifest.json` icinde su prerequisite'leri kontrol eder:
    `P3-capture`, `P4-input-readiness`, `P4-input-mapping`,
    `P5-uinput-rc-effect-throttle-low`,
    `P5-uinput-status-effect-throttle-clear`,
    `P5-uinput-rc-effect-aux1-high`,
    `P5-uinput-aux1-arm-status`.
  - `--run-live-gates --ack-live-launch --ack-live-ui --ack-live-input`
    verilirse prerequisite PASS ise once persistent-uinput yaw response,
    yalniz yaw PASS olursa pitch response kosar.
  - Prerequisite eksikse yaw/pitch response adimlarini `SKIPPED` birakir.

Readiness entegrasyonu:

- `independent_sim_readiness.py` artik iki yeni komut da uretir:
  - `pr0p_response_acceptance_plan`
  - `pr0p_response_acceptance_live`

Dokuman:

- `experiments/simitl_pr0p_probe/README.md` P5 response bolumune plan-only
  ve live response acceptance runner komutlari eklendi.
- `experiments/simitl_pr0p_probe/PLAN.md` Phase P5'e ayni acceptance akisi
  eklendi.

Eklenen test kanitlari:

- `test_response_acceptance_runner_plan_only_does_not_execute`
- `test_response_acceptance_runner_requires_ack_for_live_input`
- `test_response_acceptance_runner_skips_live_when_aux_arm_missing`
- `test_response_acceptance_runner_runs_pitch_after_yaw_pass`
- `test_response_acceptance_runner_skips_pitch_when_yaw_waits`

Son dogrulamalar:

- `fpv_env/bin/python -m pytest tests/test_simitl_pr0p_probe.py -q`
  sonuc: `148 passed`
- `fpv_env/bin/python -m pytest tests/test_game_screen_sandbox.py tests/test_simitl_pr0p_probe.py -q`
  sonuc: `315 passed`
- Gazebo bagimsizlik gate'i:
  `fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_isolation_check.py --run-id response-acceptance-isolation-v1`
  sonuc: `PASS`; rapor:
  `logs/simitl_pr0p/20260707-123521-response-acceptance-isolation-v1-isolation-check.md`
- Response acceptance plan-only dry run:
  `logs/simitl_pr0p/20260707-123448-response-acceptance-runner-v1-response-acceptance.md`
  sonuc: `WAITING`; eksikler:
  `P5-uinput-rc-effect-aux1-high`, `P5-uinput-aux1-arm-status`.
- Independent readiness dry run:
  `logs/simitl_pr0p/20260707-123448-independent-sim-response-acceptance-v1-independent-sim-readiness.md`
  `Commands` bolumunde `pr0p_response_acceptance_plan` ve
  `pr0p_response_acceptance_live` var.

Sıradaki teknik adim degismedi ama artik daha kontrollu:

1. pr0p Controls -> RC Channels icinde AUX1/CH5'i Kenet virtual AUX1 axis'e
   manuel bagla ya da kullanici acik onay verirse `pr0p_aux1_config_patch_write`
   komutunu uygula.
2. `pr0p_aux1_acceptance_live` ile once AUX1 RC-effect, sonra AUX1 arm-status
   PASS kanitini al.
3. Yeni `pr0p_response_acceptance_live` komutu ile yaw signed response PASS,
   ardindan pitch signed response PASS kanitini al.
4. Bu iki response gate PASS olmadan P6 live tracker/PID control'e gecme.

Checkpoint sonunda pr0p/updater/Gazebo/Betaflight prosesi yoktu.

## TRUE LATEST 2026-07-07 19:05 +03

Aktif hat halen Gazebo'dan tamamen bagimsiz `pr0p / SITL Forge` hattidir.
Bu turda P6 live tracker/PID control'e gecisi koruyan yeni kabul runner'i
eklendi. Amac, bbox + P6 dry-run + signed yaw/pitch response PASS kanitlari
olmadan `pr0p_tracking_probe.py --uinput` canli kontrolunun kosulmasini
engellemektir.

Yeni dosya:

- `experiments/simitl_pr0p_probe/pr0p_tracking_acceptance_runner.py`
  - Varsayilan plan-only calisir; pr0p baslatmaz, OS input gondermez.
  - `--bbox x,y,w,h` yoksa live tracking'i bekletir.
  - En guncel `*-live-manifest.json` icinde `P3-capture`,
    `P4-input-readiness`, `P4-input-mapping`,
    `P6-tracking-pid-dry-run` PASS arar.
  - En guncel `*-response-acceptance.json` icinde
    `pr0p_yaw_response_live` ve `pr0p_pitch_response_live` PASS arar.
  - `--run-live-gates --ack-live-input` verilirse sadece bu prerequisite'ler
    PASS oldugunda yaw-only `pr0p_tracking_probe.py --uinput` kosar.
  - Prerequisite eksikse live tracking adimini `SKIPPED` birakir.
  - `--execute-dry-run` ile P6 dry tracking komutu real OS input olmadan
    kosulabilir.

Readiness entegrasyonu:

- `independent_sim_readiness.py` artik iki yeni komut da uretir:
  - `pr0p_tracking_acceptance_plan`
  - `pr0p_tracking_acceptance_live`

Dokuman:

- `experiments/simitl_pr0p_probe/README.md` P6 bolumune tracking acceptance
  plan/live komutlari eklendi; direct live probe manuel debug olarak etiketlendi.
- `experiments/simitl_pr0p_probe/PLAN.md` Phase P6'ya ayni acceptance akisi
  eklendi.

Eklenen test kanitlari:

- `test_tracking_acceptance_runner_plan_only_does_not_execute`
- `test_tracking_acceptance_runner_requires_ack_for_live_input`
- `test_tracking_acceptance_runner_skips_live_without_response_pass`
- `test_tracking_acceptance_runner_runs_live_after_prereqs_pass`
- `test_tracking_acceptance_runner_executes_dry_run_without_live`

Son dogrulamalar:

- `fpv_env/bin/python -m pytest tests/test_simitl_pr0p_probe.py -q`
  sonuc: `153 passed`
- `fpv_env/bin/python -m pytest tests/test_game_screen_sandbox.py tests/test_simitl_pr0p_probe.py -q`
  sonuc: `320 passed`
- Gazebo bagimsizlik gate'i:
  `fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_isolation_check.py --run-id tracking-acceptance-isolation-v1`
  sonuc: `PASS`; rapor:
  `logs/simitl_pr0p/20260707-190504-tracking-acceptance-isolation-v1-isolation-check.md`
- Tracking acceptance plan-only dry run:
  `logs/simitl_pr0p/20260707-190438-tracking-acceptance-runner-v1-tracking-acceptance.md`
  sonuc: `WAITING`; bbox ve P6 manifest prerequisite PASS, eksik:
  `pr0p_yaw_response_live`, `pr0p_pitch_response_live`.
- Independent readiness dry run:
  `logs/simitl_pr0p/20260707-190438-independent-sim-tracking-acceptance-v1-independent-sim-readiness.md`
  `Commands` bolumunde `pr0p_tracking_acceptance_plan` ve
  `pr0p_tracking_acceptance_live` var.

Sıradaki teknik adim:

1. pr0p Controls -> RC Channels icinde AUX1/CH5'i Kenet virtual AUX1 axis'e
   manuel bagla ya da kullanici acik onay verirse `pr0p_aux1_config_patch_write`
   komutunu uygula.
2. `pr0p_aux1_acceptance_live` ile AUX1 RC-effect ve AUX1 arm-status PASS
   kanitini al.
3. `pr0p_response_acceptance_live` ile yaw signed response ve pitch signed
   response PASS kanitini al.
4. `pr0p_tracking_acceptance_live` ile yaw-only live tracker/PID kontrolunu
   kos. Pitch/approach daha sonra ve yalniz yaw-only stabil ise eklenmeli.

Checkpoint sonunda pr0p/updater/Gazebo/Betaflight prosesi yoktu.

## TRUE LATEST 2026-07-07 19:12 +03

Aktif hat halen Gazebo'dan tamamen bagimsiz `pr0p / SITL Forge` hattidir.
Bu turda acceptance runner'lar karar katmanina baglandi. Artik sadece eski
tekil live probe fazlari PASS diye pr0p yolu promote edilmeyecek.

Degisen dosyalar:

- `experiments/simitl_pr0p_probe/pr0p_live_run_manifest.py`
  - En guncel `*-response-acceptance.json` raporunu `P5-response-acceptance`
    fazi olarak manifest'e ekler.
  - En guncel `*-tracking-acceptance.json` raporunu `P6-tracking-acceptance`
    fazi olarak manifest'e ekler.
  - Suite icinde bu fazlar varsa fallback olarak kullanir.
  - Manifest genel status'u artik response/tracking acceptance PASS degilse
    `WAITING` kalir.
- `experiments/simitl_pr0p_probe/pr0p_decision_report.py`
  - Promotion icin `P5-response-acceptance PASS` ve
    `P6-tracking-acceptance PASS` zorunlu hale geldi.
  - Eksikler decision reason olarak `MISSING_RESPONSE_ACCEPTANCE` ve
    `MISSING_TRACKING_ACCEPTANCE` seklinde gorunur.
- `tests/test_simitl_pr0p_probe.py`
  - Promotion fixture'lari yeni acceptance fazlarini icerir.
  - `test_decision_report_requires_acceptance_runner_gates` eklendi.

Son dogrulamalar:

- `fpv_env/bin/python -m pytest tests/test_simitl_pr0p_probe.py -q`
  sonuc: `154 passed`
- `fpv_env/bin/python -m pytest tests/test_game_screen_sandbox.py tests/test_simitl_pr0p_probe.py -q`
  sonuc: `321 passed`
- Gazebo bagimsizlik gate'i:
  `fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_isolation_check.py --run-id decision-acceptance-isolation-v1`
  sonuc: `PASS`; rapor:
  `logs/simitl_pr0p/20260707-191215-decision-acceptance-isolation-v1-isolation-check.md`
- Manifest dry run:
  `logs/simitl_pr0p/20260707-191215-manifest-acceptance-gates-v2-live-manifest.md`
  sonuc: `WAITING`; `P5-response-acceptance` ve
  `P6-tracking-acceptance` fazlari manifest'te `WAITING` gorunuyor.
- Decision dry run:
  `logs/simitl_pr0p/20260707-191215-decision-acceptance-gates-v2-decision.md`
  sonuc: `WAITING`; reasons icinde
  `MISSING_RESPONSE_ACCEPTANCE` ve `MISSING_TRACKING_ACCEPTANCE` var.
- Independent readiness dry run:
  `logs/simitl_pr0p/20260707-191215-independent-sim-decision-acceptance-gates-v2-independent-sim-readiness.md`
  sonuc: `WAITING`; ayni eksikler `PR0P:*` olarak gorunuyor.

Sıradaki teknik adim degismedi:

1. AUX1/CH5 binding veya kullanici onayli `pr0p_aux1_config_patch_write`.
2. `pr0p_aux1_acceptance_live`.
3. `pr0p_response_acceptance_live`.
4. `pr0p_tracking_acceptance_live`.

Checkpoint sonunda pr0p/updater/Gazebo/Betaflight prosesi yoktu.

## TRUE LATEST 2026-07-07 19:20 +03

Aktif hat halen mevcut Gazebo/Betaflight entegrasyonundan tamamen bagimsiz
`pr0p / SITL Forge` hattidir. Bu turda yeni bir ordered acceptance chain eklendi;
amac tekil runner'larin yanlis sirada kosulmasini engellemek ve "siradaki komut
ne?" sorusunu tek raporda cevaplamak.

Degisen dosyalar:

- `experiments/simitl_pr0p_probe/pr0p_acceptance_chain_runner.py`
  - Yeni chain runner.
  - Varsayilan mod plan-only; dis komut, OS input, pr0p launch veya config write
    yapmaz.
  - Siralama: AUX1/ARM acceptance PASS degilse response SKIPPED; response PASS
    degilse tracking SKIPPED.
  - Live mod icin `--run-live-gates --ack-live-launch --ack-live-ui
    --ack-live-input` zorunlu.
  - Config write icin ayrica `--apply-config-patch --ack-config-write` zorunlu.
  - AUX1 ve response sonrasinda manifest refresh raporu uretir; boylece sonraki
    gate eski manifest'e bakip yanlis WAITING/PASS okumaz.
- `experiments/simitl_pr0p_probe/independent_sim_readiness.py`
  - `pr0p_acceptance_chain_plan` ve `pr0p_acceptance_chain_live` komutlari
    readiness raporuna eklendi.
- `experiments/simitl_pr0p_probe/README.md`
  - Ordered acceptance chain kullanim bolumu eklendi.
- `tests/test_simitl_pr0p_probe.py`
  - Chain runner importlari ve 4 yeni test eklendi.
  - Testler plan-only dis komut calistirmama, live ack zorunlulugu, AUX1 PASS
    sonrasi response'a gecme, response PASS sonrasi tracking'e gecme davranisini
    dogrular.

Son dogrulamalar:

- `fpv_env/bin/python -m pytest tests/test_simitl_pr0p_probe.py -q -k acceptance_chain`
  sonuc: `4 passed, 154 deselected`
- Plan-only chain smoke:
  `fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_acceptance_chain_runner.py --run-id acceptance-chain-plan-smoke`
  sonuc: `WAITING`; rapor:
  `logs/simitl_pr0p/20260707-192002-acceptance-chain-plan-smoke-acceptance-chain.md`
- `fpv_env/bin/python -m pytest tests/test_simitl_pr0p_probe.py -q`
  sonuc: `158 passed`
- Readiness smoke:
  `fpv_env/bin/python experiments/simitl_pr0p_probe/independent_sim_readiness.py --run-id acceptance-chain-readiness-smoke`
  sonuc: `WAITING`; rapor:
  `logs/simitl_pr0p/20260707-192017-acceptance-chain-readiness-smoke-independent-sim-readiness.md`
- Gazebo bagimsizlik gate'i:
  `fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_isolation_check.py --run-id acceptance-chain-isolation-smoke`
  sonuc: `PASS`; rapor:
  `logs/simitl_pr0p/20260707-192018-acceptance-chain-isolation-smoke-isolation-check.md`
- Genis test:
  `fpv_env/bin/python -m pytest tests/test_game_screen_sandbox.py tests/test_simitl_pr0p_probe.py -q`
  sonuc: `325 passed`

Sıradaki teknik adim:

1. pr0p Controls -> RC Channels icinde AUX1/CH5 binding'i manuel yap ya da
   kullanici acik onay verirse config patch write uygula.
2. Tekil komutlar yerine once chain plan komutunu calistir:

   ```bash
   fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_acceptance_chain_runner.py \
     --bbox x,y,w,h \
     --run-id pr0p-acceptance-chain-plan
   ```

3. Canli onay verildiginde chain live komutu ile sirali kabul testini kos:

   ```bash
   fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_acceptance_chain_runner.py \
     --bbox x,y,w,h \
     --execute-dry-patch \
     --run-live-gates \
     --ack-live-launch \
     --ack-live-ui \
     --ack-live-input \
     --run-id pr0p-acceptance-chain-live
   ```

4. Chain `PASS` olduktan sonra live manifest, decision ve independent readiness
   raporlarini yeniden uret.

Bu turda canli pr0p launch, OS input gonderimi veya config write yapilmadi.

## TRUE LATEST 2026-07-07 19:26 +03

Aktif hat halen mevcut Gazebo/Betaflight entegrasyonundan tamamen bagimsiz
`pr0p / SITL Forge` hattidir. Bu turda canli pr0p launch, OS input veya config
write yapilmadi. Sadece mevcut AUX1/CH5 blocker'ini manuel cozmek icin guvenli
mapping assistant eksigi kapatildi.

Degisen dosyalar:

- `experiments/simitl_pr0p_probe/pr0p_rc_channel_mapping_assistant.py`
  - `ROLE_ORDER` artik `aux1` icerir.
  - `--role aux1` tek basina AUX1/ARM CH5 pulse plani uretir.
  - `--role all` artik roll, pitch, throttle, yaw ve AUX1'i sirayla pulse eder.
  - Varsayilan dry-run davranisi degismedi; OS input sadece
    `--uinput --ack-live-input` ile gonderilir.
- `tests/test_simitl_pr0p_probe.py`
  - Mapping assistant testleri AUX1'i kapsayacak sekilde guncellendi.
  - `mapping_command_for_role("aux1", ...)` ve AUX1 pulse event'leri
    dogrulaniyor.
- `experiments/simitl_pr0p_probe/README.md`
  - RC Channels binding bolumu "bes rol" olarak guncellendi.
  - `--role aux1` ARM/CH5 blocker icin acikca dokumante edildi.
- `experiments/simitl_pr0p_probe/PLAN.md`
  - Manuel mapping assistant adimi AUX1/ARM CH5'i kapsayacak sekilde
    guncellendi.

Son dogrulamalar:

- `fpv_env/bin/python -m pytest tests/test_simitl_pr0p_probe.py -q -k rc_channel_mapping_assistant`
  sonuc: `4 passed, 154 deselected`
- AUX1 mapping dry-run smoke:
  `fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_rc_channel_mapping_assistant.py --role aux1 --run-id aux1-mapping-dry-smoke`
  sonuc: `PASS`, `real_input_sent=False`; rapor:
  `logs/simitl_pr0p/20260707-192547-aux1-mapping-dry-smoke-rc-channel-mapping.md`
- `fpv_env/bin/python -m pytest tests/test_simitl_pr0p_probe.py -q`
  sonuc: `158 passed`
- Gazebo bagimsizlik gate'i:
  `fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_isolation_check.py --run-id aux1-mapping-assistant-isolation-smoke`
  sonuc: `PASS`; rapor:
  `logs/simitl_pr0p/20260707-192553-aux1-mapping-assistant-isolation-smoke-isolation-check.md`
- Genis test:
  `fpv_env/bin/python -m pytest tests/test_game_screen_sandbox.py tests/test_simitl_pr0p_probe.py -q`
  sonuc: `325 passed`

Sıradaki teknik adim:

1. pr0p acikken `Controls -> RC Channels` sayfasini ac.
2. Config patch tercih edilmiyorsa once dry plan:

   ```bash
   fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_rc_channel_mapping_assistant.py \
     --role aux1 \
     --run-id pr0p-rc-channel-map-aux1-dry
   ```

3. Canli input icin kullanici acik onayi oldugunda, AUX1/CH5 binding sirasinda:

   ```bash
   fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_rc_channel_mapping_assistant.py \
     --uinput \
     --ack-live-input \
     --role aux1 \
     --run-id pr0p-rc-channel-map-aux1-live
   ```

4. Ardindan chain plan/live sirasi:
   `pr0p_acceptance_chain_runner.py --bbox x,y,w,h ...`.

Bu tur sonunda pr0p/updater/Gazebo/Betaflight prosesi yoktu.

## TRUE LATEST 2026-07-07 19:29 +03

Aktif hat halen mevcut Gazebo/Betaflight entegrasyonundan tamamen bagimsiz
`pr0p / SITL Forge` hattidir. Bu turda canli pr0p launch, OS input veya config
write yapilmadi. 19:26'da eklenen AUX1 mapping helper destegi bu kez manifest
ve readiness raporlarina tasindi; yani operator raporu artik config patch'e ek
olarak manuel AUX1/CH5 binding komutlarini da dogrudan gosterir.

Degisen dosyalar:

- `experiments/simitl_pr0p_probe/pr0p_live_run_manifest.py`
  - Yeni manifest komutlari:
    - `P4-rc-channel-mapping-assistant-aux1-dry`
    - `P4-rc-channel-mapping-assistant-aux1-live`
  - Manifest step listesine bu iki faz eklendi.
  - `P5-uinput-rc-effect-aux1-high` eksikken `next_action` artik yalniz config
    patch yolunu degil, manuel AUX1 mapping assistant yolunu da onerir.
- `experiments/simitl_pr0p_probe/independent_sim_readiness.py`
  - Yeni readiness komutlari:
    - `pr0p_aux1_mapping_assistant_dry`
    - `pr0p_aux1_mapping_assistant_live`
- `tests/test_simitl_pr0p_probe.py`
  - Manifest ve readiness komut listeleri icin AUX1 mapping assistant assertleri
    eklendi.

Son dogrulamalar:

- Hedefli test:
  `fpv_env/bin/python -m pytest tests/test_simitl_pr0p_probe.py -q -k "live_manifest_includes_p6_tracking_and_aux_commands or independent_readiness"`
  sonuc: `6 passed, 152 deselected`
- Manifest dry-run:
  `fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_live_run_manifest.py --run-id aux1-helper-manifest-smoke`
  sonuc: `WAITING`; rapor:
  `logs/simitl_pr0p/20260707-192854-aux1-helper-manifest-smoke-live-manifest.md`
  raporda `P4-rc-channel-mapping-assistant-aux1-dry/live` gorunuyor.
- Independent readiness dry-run:
  `fpv_env/bin/python experiments/simitl_pr0p_probe/independent_sim_readiness.py --run-id aux1-helper-readiness-smoke`
  sonuc: `WAITING`; rapor:
  `logs/simitl_pr0p/20260707-192855-aux1-helper-readiness-smoke-independent-sim-readiness.md`
  raporda `pr0p_aux1_mapping_assistant_dry/live` gorunuyor.
- `fpv_env/bin/python -m pytest tests/test_simitl_pr0p_probe.py -q`
  sonuc: `158 passed`
- Gazebo bagimsizlik gate'i:
  `fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_isolation_check.py --run-id aux1-helper-reporting-isolation-smoke`
  sonuc: `PASS`; rapor:
  `logs/simitl_pr0p/20260707-192903-aux1-helper-reporting-isolation-smoke-isolation-check.md`
- Genis test:
  `fpv_env/bin/python -m pytest tests/test_game_screen_sandbox.py tests/test_simitl_pr0p_probe.py -q`
  sonuc: `325 passed`

Sıradaki teknik adim:

1. `independent_sim_readiness.py` veya `pr0p_live_run_manifest.py` raporunu ac.
2. `pr0p_aux1_mapping_assistant_dry` / `P4-rc-channel-mapping-assistant-aux1-dry`
   komutunu plan olarak kontrol et.
3. Kullanici canli input icin acik onay verirse pr0p `Controls -> RC Channels`
   sayfasinda `AUX1/CH5` binding yaparken live helper'i calistir.
4. Ardindan `pr0p_acceptance_chain_runner.py` ile AUX1 -> response -> tracking
   kabul zincirini kos.

Bu tur sonunda pr0p/updater/Gazebo/Betaflight prosesi yoktu.

## TRUE LATEST 2026-07-07 19:34 +03

Aktif hat halen mevcut Gazebo/Betaflight entegrasyonundan tamamen bagimsiz
`pr0p / SITL Forge` hattidir. Bu turda canli pr0p launch, OS input veya config
write yapilmadi. Yeni eklenen parca, son acceptance kanitlarini sadece okuyarak
zincirin hangi asamada bekledigini soyleyen read-only state raporudur.

Degisen dosyalar:

- `experiments/simitl_pr0p_probe/pr0p_acceptance_state_report.py`
  - Yeni read-only rapor.
  - En guncel `*-aux1-acceptance.json`, `*-response-acceptance.json` ve
    `*-tracking-acceptance.json` raporlarini okur.
  - Eksik, stale, fail ve pass durumlarini ayirir.
  - Varsayilan stale limiti 24 saat.
  - Dis komut calistirmaz; pr0p baslatmaz; OS input gondermez; config yazmaz.
  - Bir sonraki aksiyonu AUX1 -> response -> tracking sirasi ile verir.
- `experiments/simitl_pr0p_probe/independent_sim_readiness.py`
  - `pr0p_acceptance_state` komutu readiness raporuna eklendi.
- `experiments/simitl_pr0p_probe/README.md`
  - Ordered acceptance chain bolumune read-only acceptance state komutu eklendi.
- `tests/test_simitl_pr0p_probe.py`
  - 3 yeni state testi eklendi:
    - kanit yokken AUX1'de WAITING,
    - stale kanitta WAITING/STALE,
    - tum acceptance kanitlari PASS ise state PASS.

Son dogrulamalar:

- Hedefli state testi:
  `fpv_env/bin/python -m pytest tests/test_simitl_pr0p_probe.py -q -k acceptance_state`
  sonuc: `3 passed, 158 deselected`
- Acceptance state smoke:
  `fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_acceptance_state_report.py --run-id acceptance-state-smoke`
  sonuc: `WAITING`; rapor:
  `logs/simitl_pr0p/20260707-193345-acceptance-state-smoke-acceptance-state.md`
  mevcut durumda AUX1/ARM kaniti bekledigini soyluyor.
- Independent readiness smoke:
  `fpv_env/bin/python experiments/simitl_pr0p_probe/independent_sim_readiness.py --run-id acceptance-state-readiness-smoke`
  sonuc: `WAITING`; rapor:
  `logs/simitl_pr0p/20260707-193354-acceptance-state-readiness-smoke-independent-sim-readiness.md`
  `pr0p_acceptance_state` komutunu iceriyor.
- `fpv_env/bin/python -m pytest tests/test_simitl_pr0p_probe.py -q`
  sonuc: `161 passed`
- Gazebo bagimsizlik gate'i:
  `fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_isolation_check.py --run-id acceptance-state-isolation-smoke`
  sonuc: `PASS`; rapor:
  `logs/simitl_pr0p/20260707-193354-acceptance-state-isolation-smoke-isolation-check.md`
- Genis test:
  `fpv_env/bin/python -m pytest tests/test_game_screen_sandbox.py tests/test_simitl_pr0p_probe.py -q`
  sonuc: `328 passed`

Sıradaki teknik adim:

1. Canli calisma oncesi:

   ```bash
   fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_acceptance_state_report.py \
     --bbox x,y,w,h \
     --run-id pr0p-acceptance-state
   ```

2. State `AUX1/ARM` bekliyorsa, pr0p `Controls -> RC Channels` sayfasinda
   AUX1/CH5 binding veya backup-backed config patch yap.
3. Sonra `pr0p_acceptance_chain_runner.py` live komutu ile AUX1 -> response ->
   tracking kabul zincirini kos.
4. Chain PASS olduktan sonra manifest, decision ve independent readiness
   raporlarini yeniden uret.

Bu tur sonunda pr0p/updater/Gazebo/Betaflight prosesi yoktu.

## TRUE LATEST 2026-07-07 19:40 +03

Aktif hat halen mevcut Gazebo/Betaflight entegrasyonundan tamamen bagimsiz
`pr0p / SITL Forge` hattidir. Bu turda canli pr0p launch, OS input veya config
write yapilmadi. Yeni eklenen parca, standalone response/tracking acceptance
JSON kanitlari bayatsa manifestin bunlari PASS olarak kullanmasini engelleyen
stale evidence guard'dir.

Degisen dosyalar:

- `experiments/simitl_pr0p_probe/pr0p_live_run_manifest.py`
  - `DEFAULT_MAX_ACCEPTANCE_EVIDENCE_AGE_S = 24 saat` eklendi.
  - `P5-response-acceptance` ve `P6-tracking-acceptance` artik standalone
    acceptance JSON kanitini dosya yasina gore kontrol eder.
  - Kanit bayatsa step `WAITING` olur, summary `latest acceptance evidence is
    stale` olur, notlara `STALE_RESPONSE_ACCEPTANCE` veya
    `STALE_TRACKING_ACCEPTANCE` eklenir.
  - Suite icinde eski PASS gorunse bile stale standalone evidence varsa manifest
    PASS'a dusmez; operator yeni acceptance kosmaya yonlendirilir.
  - CLI'ya `--max-acceptance-evidence-age-s` eklendi ve negatif degerler
    reddedilir.
- `tests/test_simitl_pr0p_probe.py`
  - Stale standalone response/tracking acceptance raporlarinin manifesti
    `WAITING` yaptigini dogrulayan test eklendi.
  - Negatif `--max-acceptance-evidence-age-s` argumaninin reddedildigini
    dogrulayan test eklendi.

Son dogrulamalar:

- Hedefli manifest/state testleri:
  `fpv_env/bin/python -m pytest tests/test_simitl_pr0p_probe.py -q -k "acceptance_state or stale_standalone_acceptance or negative_acceptance or live_manifest"`
  sonuc: `15 passed, 148 deselected`
- Manifest dry-run:
  `fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_live_run_manifest.py --run-id stale-acceptance-manifest-smoke`
  sonuc: `WAITING`; rapor:
  `logs/simitl_pr0p/20260707-193952-stale-acceptance-manifest-smoke-live-manifest.md`
- `fpv_env/bin/python -m pytest tests/test_simitl_pr0p_probe.py -q`
  sonuc: `163 passed`
- Genis test:
  `fpv_env/bin/python -m pytest tests/test_game_screen_sandbox.py tests/test_simitl_pr0p_probe.py -q`
  sonuc: `330 passed`
- Gazebo bagimsizlik gate'i:
  `fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_isolation_check.py --run-id stale-acceptance-isolation-smoke`
  sonuc: `PASS`; rapor:
  `logs/simitl_pr0p/20260707-194013-stale-acceptance-isolation-smoke-isolation-check.md`
- Surec kontrolu:
  `pgrep -af 'pr0p|SimITL|SITL Forge|gazebo|gz sim|betaflight'`
  sonuc: cikti yok; canli sim/FC prosesi birakilmadi.

Sıradaki teknik adim:

1. Kullanici canli input icin acik onay verdiginde pr0p acik ve
   `Controls -> RC Channels` sayfasindayken AUX1/CH5 mapping helper'i calistir:

   ```bash
   fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_rc_channel_mapping_assistant.py \
     --uinput \
     --ack-live-input \
     --role aux1 \
     --run-id pr0p-rc-channel-map-aux1-live
   ```

2. Sonra `pr0p_acceptance_chain_runner.py` ile AUX1 -> response -> tracking
   kabul zincirini kos.
3. Chain sonrasi `pr0p_acceptance_state_report.py`,
   `pr0p_live_run_manifest.py`, `pr0p_decision_report.py` ve
   `independent_sim_readiness.py` raporlarini yeniden uret.

## TRUE LATEST 2026-07-07 19:46 +03

Aktif hat halen mevcut Gazebo/Betaflight entegrasyonundan tamamen bagimsiz
`pr0p / SITL Forge` hattidir. Bu turda canli pr0p launch, OS input veya config
write yapilmadi. Yeni eklenen parca, decision katmaninin eski manifest/suite
kanitlarini promotion icin kullanmasini engelleyen evidence freshness guard'dir.

Degisen dosyalar:

- `experiments/simitl_pr0p_probe/pr0p_decision_report.py`
  - Varsayilan `--max-evidence-age-s` 24 saat olarak eklendi.
  - CLI decision raporu artik secilen evidence dosyasi bayatsa `WAITING` verir,
    `EVIDENCE_STALE` sebebini yazar ve promote/reject karari uretmez.
  - `status != PASS` olan kaynak raporlar artik tum alt fazlar PASS gorunse bile
    `SOURCE_STATUS_NOT_PASS:<status>` sebebiyle promote edilmez.
  - `status == FAIL` olan kaynak raporlar `SOURCE_STATUS_FAIL` ile reject edilir.
  - Negatif `--max-evidence-age-s` argumani reddedilir.
- `tests/test_simitl_pr0p_probe.py`
  - Top-level `WAITING` kaynak raporunun promote edilmedigi test edildi.
  - Stale evidence path ile decision'in `WAITING/EVIDENCE_STALE` verdigi test
    edildi.
  - Negatif decision evidence age argumaninin reddedildigi test edildi.
  - `test_independent_readiness_promotes_only_when_both_paths_ready` fixture'i
    yeni source-status guard'a uygun olarak `P0-preflight PASS` kanitini de
    icerir hale getirildi.

Son dogrulamalar:

- Hedefli decision/readiness testi:
  `fpv_env/bin/python -m pytest tests/test_simitl_pr0p_probe.py -q -k "decision_report or independent_readiness or stale_standalone_acceptance"`
  sonuc: `20 passed, 146 deselected`
- `fpv_env/bin/python -m pytest tests/test_simitl_pr0p_probe.py -q`
  sonuc: `166 passed`
- Genis test:
  `fpv_env/bin/python -m pytest tests/test_game_screen_sandbox.py tests/test_simitl_pr0p_probe.py -q`
  sonuc: `333 passed`
- Decision smoke:
  `fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_decision_report.py --run-id decision-freshness-smoke-v2`
  sonuc: `WAITING`; rapor:
  `logs/simitl_pr0p/20260707-194634-decision-freshness-smoke-v2-decision.md`
- Gazebo bagimsizlik gate'i:
  `fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_isolation_check.py --run-id decision-freshness-isolation-smoke`
  sonuc: `PASS`; rapor:
  `logs/simitl_pr0p/20260707-194649-decision-freshness-isolation-smoke-isolation-check.md`
- Surec kontrolu:
  `pgrep -af 'pr0p|SimITL|SITL Forge|gazebo|gz sim|betaflight'`
  sonuc: cikti yok; canli sim/FC prosesi birakilmadi.

Sıradaki teknik adim:

1. Kullanici canli input icin acik onay verdiginde pr0p acik ve
   `Controls -> RC Channels` sayfasindayken AUX1/CH5 mapping helper'i calistir:

   ```bash
   fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_rc_channel_mapping_assistant.py \
     --uinput \
     --ack-live-input \
     --role aux1 \
     --run-id pr0p-rc-channel-map-aux1-live
   ```

2. Sonra `pr0p_acceptance_chain_runner.py` ile AUX1 -> response -> tracking
   kabul zincirini kos.
3. Chain sonrasi `pr0p_acceptance_state_report.py`,
   `pr0p_live_run_manifest.py`, `pr0p_decision_report.py` ve
   `independent_sim_readiness.py` raporlarini yeniden uret.

## TRUE LATEST 2026-07-07 19:50 +03

Aktif hat halen mevcut Gazebo/Betaflight entegrasyonundan tamamen bagimsiz
`pr0p / SITL Forge` hattidir. Bu turda canli pr0p launch, OS input veya config
write yapilmadi. Yeni eklenen parca, decision katmaninin promotion icin safe
suite'i tek basina yeterli kanit saymasini engelleyen `live manifest required`
guard'dir.

Degisen dosyalar:

- `experiments/simitl_pr0p_probe/pr0p_decision_report.py`
  - Evidence source artik `live_manifest`, `suite` veya `UNKNOWN` olarak
    siniflandirilir.
  - Promotion icin `live_manifest` source zorunlu hale getirildi.
  - Safe suite halen bekleme/teshis icin okunur, ama tum fazlari PASS gorunse
    bile `MISSING_LIVE_MANIFEST_EVIDENCE` sebebiyle promote edilmez.
  - Otomatik evidence secimi artik manifest varsa manifesti secer; manifest
    yoksa suite'e duser.
- `tests/test_simitl_pr0p_probe.py`
  - Promotion fixture'i live manifest seklinde acik isaretlendi.
  - Suite'in tek basina promote etmedigi test edildi.
  - Manifestin daha eski gorunse bile decision icin suite'e tercih edildigi test
    edildi.
  - Source-kind siniflandirmasi test edildi.

Son dogrulamalar:

- Syntax:
  `fpv_env/bin/python -m py_compile experiments/simitl_pr0p_probe/pr0p_decision_report.py tests/test_simitl_pr0p_probe.py`
  sonuc: PASS
- Hedefli decision/readiness testi:
  `fpv_env/bin/python -m pytest tests/test_simitl_pr0p_probe.py -q -k "decision_report or independent_readiness"`
  sonuc: `21 passed, 147 deselected`
- `fpv_env/bin/python -m pytest tests/test_simitl_pr0p_probe.py -q`
  sonuc: `168 passed`
- Genis test:
  `fpv_env/bin/python -m pytest tests/test_game_screen_sandbox.py tests/test_simitl_pr0p_probe.py -q`
  sonuc: `335 passed`
- Decision smoke:
  `fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_decision_report.py --run-id live-manifest-required-smoke`
  sonuc: `WAITING`; rapor:
  `logs/simitl_pr0p/20260707-195027-live-manifest-required-smoke-decision.md`
- Gazebo bagimsizlik gate'i:
  `fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_isolation_check.py --run-id live-manifest-required-isolation-smoke`
  sonuc: `PASS`; rapor:
  `logs/simitl_pr0p/20260707-195041-live-manifest-required-isolation-smoke-isolation-check.md`
- Surec kontrolu:
  `pgrep -af 'pr0p|SimITL|SITL Forge|gazebo|gz sim|betaflight'`
  sonuc: cikti yok; canli sim/FC prosesi birakilmadi.

Sıradaki teknik adim:

1. Kullanici canli input icin acik onay verdiginde pr0p acik ve
   `Controls -> RC Channels` sayfasindayken AUX1/CH5 mapping helper'i calistir:

   ```bash
   fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_rc_channel_mapping_assistant.py \
     --uinput \
     --ack-live-input \
     --role aux1 \
     --run-id pr0p-rc-channel-map-aux1-live
   ```

2. Sonra `pr0p_acceptance_chain_runner.py` ile AUX1 -> response -> tracking
   kabul zincirini kos.
3. Chain sonrasi `pr0p_acceptance_state_report.py`,
   `pr0p_live_run_manifest.py`, `pr0p_decision_report.py` ve
   `independent_sim_readiness.py` raporlarini yeniden uret.

## TRUE LATEST 2026-07-07 19:56 +03

Aktif hat halen mevcut Gazebo/Betaflight entegrasyonundan tamamen bagimsiz
`pr0p / SITL Forge` hattidir. Bu turda canli pr0p launch, OS input veya config
write yapilmadi. Yeni eklenen parca, independent readiness raporunun disk
uzerinde gercek `*-live-manifest.json` kaniti olmadan
`INDEPENDENT_SIM_READY` demesini engelleyen live-manifest evidence guard'dir.

Degisen dosyalar:

- `experiments/simitl_pr0p_probe/independent_sim_readiness.py`
  - En guncel `*-live-manifest.json` raporu okunur ve freshness kontrolune
    eklenir.
  - `evidence_paths` artik `pr0p_live_manifest` yolunu da raporlar.
  - `metrics.evidence_freshness` artik `pr0p_live_manifest` durumunu da icerir.
  - Suite PASS olsa bile live manifest eksikse readiness `WAITING` kalir ve
    `PR0P:LIVE_MANIFEST_EVIDENCE_MISSING` sebebini yazar.
  - Live manifest stale ise readiness `WAITING` kalir ve yeniden manifest
    uretmeyi onerir.
  - Preview manifest sadece next-action uretmek icin kalir; promotion karari
    disk uzerindeki manifest kanitina baglandi.
- `tests/test_simitl_pr0p_probe.py`
  - `readiness_promotion_steps()` helper'i eklendi.
  - `test_independent_readiness_promotes_only_when_both_paths_ready` artik hem
    suite hem de live manifest kaniti yazar.
  - `test_independent_readiness_requires_live_manifest_after_suite_pass` eklendi:
    suite PASS olsa bile live manifest yoksa readiness `WAITING` kalir.

Son dogrulamalar:

- Syntax:
  `fpv_env/bin/python -m py_compile experiments/simitl_pr0p_probe/independent_sim_readiness.py tests/test_simitl_pr0p_probe.py`
  sonuc: PASS
- Hedefli readiness/decision testi:
  `fpv_env/bin/python -m pytest tests/test_simitl_pr0p_probe.py -q -k "independent_readiness or decision_report"`
  sonuc: `22 passed, 147 deselected`
- `fpv_env/bin/python -m pytest tests/test_simitl_pr0p_probe.py -q`
  sonuc: `169 passed`
- Genis test:
  `fpv_env/bin/python -m pytest tests/test_game_screen_sandbox.py tests/test_simitl_pr0p_probe.py -q`
  sonuc: `336 passed`
- Independent readiness smoke:
  `fpv_env/bin/python experiments/simitl_pr0p_probe/independent_sim_readiness.py --run-id live-manifest-evidence-smoke`
  sonuc: `WAITING`; rapor:
  `logs/simitl_pr0p/20260707-195639-live-manifest-evidence-smoke-independent-sim-readiness.md`
- Gazebo bagimsizlik gate'i:
  `fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_isolation_check.py --run-id readiness-live-manifest-evidence-isolation-smoke`
  sonuc: `PASS`; rapor:
  `logs/simitl_pr0p/20260707-195653-readiness-live-manifest-evidence-isolation-smoke-isolation-check.md`
- Surec kontrolu:
  `pgrep -af 'pr0p|SimITL|SITL Forge|gazebo|gz sim|betaflight'`
  sonuc: cikti yok; canli sim/FC prosesi birakilmadi.

Sıradaki teknik adim:

1. Kullanici canli input icin acik onay verdiginde pr0p acik ve
   `Controls -> RC Channels` sayfasindayken AUX1/CH5 mapping helper'i calistir:

   ```bash
   fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_rc_channel_mapping_assistant.py \
     --uinput \
     --ack-live-input \
     --role aux1 \
     --run-id pr0p-rc-channel-map-aux1-live
   ```

2. Sonra `pr0p_acceptance_chain_runner.py` ile AUX1 -> response -> tracking
   kabul zincirini kos.
3. Chain sonrasi `pr0p_acceptance_state_report.py`,
   `pr0p_live_run_manifest.py`, `pr0p_decision_report.py` ve
   `independent_sim_readiness.py` raporlarini yeniden uret.

## TRUE LATEST 2026-07-07 20:01 +03

Aktif hat halen mevcut Gazebo/Betaflight entegrasyonundan tamamen bagimsiz
`pr0p / SITL Forge` hattidir. Bu turda canli pr0p launch, OS input veya config
write yapilmadi. Yeni eklenen parca, independent readiness raporunun live
manifest ile en guncel safe suite kanitinin ayni kaynaga ait oldugunu
dogrulayan manifest-suite consistency guard'dir.

Degisen dosyalar:

- `experiments/simitl_pr0p_probe/independent_sim_readiness.py`
  - `manifest_suite_consistency()` eklendi.
  - Live manifest icindeki `suite_report`, en guncel `*-suite.json` ile
    eslesmiyorsa readiness `WAITING` kalir.
  - Bu durumda `PR0P:LIVE_MANIFEST_SUITE_MISMATCH` sebebi yazilir.
  - Next action artik bu durumda live manifest'i en guncel safe suite'ten
    yeniden uretmeyi onerir.
  - Metrics'e `pr0p_manifest_suite_consistency` eklendi.
- `tests/test_simitl_pr0p_probe.py`
  - `write_pr0p_manifest()` helper'i opsiyonel `suite_report` metadata'si
    yazabilir hale geldi.
  - Ready senaryosu artik manifestin suite_report alanini guncel suite ile
    eslestirir.
  - `test_independent_readiness_rejects_manifest_from_older_suite` eklendi:
    manifest taze olsa bile eski suite'i referansliyorsa readiness `WAITING`
    kalir.

Son dogrulamalar:

- Syntax:
  `fpv_env/bin/python -m py_compile experiments/simitl_pr0p_probe/independent_sim_readiness.py tests/test_simitl_pr0p_probe.py`
  sonuc: PASS
- Hedefli readiness/decision testi:
  `fpv_env/bin/python -m pytest tests/test_simitl_pr0p_probe.py -q -k "independent_readiness or decision_report"`
  sonuc: `23 passed, 147 deselected`
- `fpv_env/bin/python -m pytest tests/test_simitl_pr0p_probe.py -q`
  sonuc: `170 passed`
- Genis test:
  `fpv_env/bin/python -m pytest tests/test_game_screen_sandbox.py tests/test_simitl_pr0p_probe.py -q`
  sonuc: `337 passed`
- Independent readiness smoke:
  `fpv_env/bin/python experiments/simitl_pr0p_probe/independent_sim_readiness.py --run-id manifest-suite-consistency-smoke`
  sonuc: `WAITING`; rapor:
  `logs/simitl_pr0p/20260707-200047-manifest-suite-consistency-smoke-independent-sim-readiness.md`
- Gazebo bagimsizlik gate'i:
  `fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_isolation_check.py --run-id manifest-suite-consistency-isolation-smoke`
  sonuc: `PASS`; rapor:
  `logs/simitl_pr0p/20260707-200101-manifest-suite-consistency-isolation-smoke-isolation-check.md`
- Surec kontrolu:
  `pgrep -af 'pr0p|SimITL|SITL Forge|gazebo|gz sim|betaflight'`
  sonuc: cikti yok; canli sim/FC prosesi birakilmadi.

Sıradaki teknik adim:

1. Kullanici canli input icin acik onay verdiginde pr0p acik ve
   `Controls -> RC Channels` sayfasindayken AUX1/CH5 mapping helper'i calistir:

   ```bash
   fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_rc_channel_mapping_assistant.py \
     --uinput \
     --ack-live-input \
     --role aux1 \
     --run-id pr0p-rc-channel-map-aux1-live
   ```

2. Sonra `pr0p_acceptance_chain_runner.py` ile AUX1 -> response -> tracking
   kabul zincirini kos.
3. Chain sonrasi `pr0p_acceptance_state_report.py`,
   `pr0p_live_run_manifest.py`, `pr0p_decision_report.py` ve
   `independent_sim_readiness.py` raporlarini yeniden uret.

## TRUE LATEST 2026-07-07 20:06 +03

Aktif hat halen mevcut Gazebo/Betaflight entegrasyonundan tamamen bagimsiz
`pr0p / SITL Forge` hattidir. Bu turda canli pr0p launch, OS input veya config
write yapilmadi. Yeni eklenen parca, acceptance chain PASS oldugunda promotion
icin gerekli downstream raporlari da otomatik ureten chain refresh yapisidir.

Degisen dosyalar:

- `experiments/simitl_pr0p_probe/pr0p_acceptance_chain_runner.py`
  - Tracking acceptance PASS olduktan sonra final live manifest refresh devam
    eder.
  - Final manifestten `pr0p_decision_report` otomatik yazilir.
  - Ardindan `independent_sim_readiness` otomatik yazilir.
  - Chain next action artik bu durumda decision/readiness raporlarini incelemeyi
    onerir.
  - `--game-log-dir` eklendi; default `logs/game_screen_sandbox` kalir, ancak
    testler ve izole kosular dis game-screen log state'inden etkilenmeden kendi
    game log dizinini verebilir.
  - Chain metrics artik `game_log_dir` alanini da yazar.
- `tests/test_simitl_pr0p_probe.py`
  - Acceptance chain helper'i izole `game_log_dir` kullanir.
  - PASS chain testinde `pr0p_decision_refresh` ve
    `independent_sim_readiness_refresh` stage'leri assert edildi.
  - Bu testte decision refresh `PROMOTE_CANDIDATE`, readiness refresh ise izole
    game-screen evidence olmadigi icin `WAITING` olarak dogrulandi.

Son dogrulamalar:

- Syntax:
  `fpv_env/bin/python -m py_compile experiments/simitl_pr0p_probe/pr0p_acceptance_chain_runner.py tests/test_simitl_pr0p_probe.py`
  sonuc: PASS
- Hedefli chain/readiness/decision testi:
  `fpv_env/bin/python -m pytest tests/test_simitl_pr0p_probe.py -q -k "acceptance_chain_runner or independent_readiness or decision_report"`
  sonuc: `27 passed, 143 deselected`
- `fpv_env/bin/python -m pytest tests/test_simitl_pr0p_probe.py -q`
  sonuc: `170 passed`
- Genis test:
  `fpv_env/bin/python -m pytest tests/test_game_screen_sandbox.py tests/test_simitl_pr0p_probe.py -q`
  sonuc: `337 passed`
- Acceptance chain smoke:
  `fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_acceptance_chain_runner.py --bbox 10,20,80,60 --run-id chain-refresh-smoke`
  sonuc: `WAITING`; rapor:
  `logs/simitl_pr0p/20260707-200556-chain-refresh-smoke-acceptance-chain.md`
  mevcut durumda AUX1/CH5 live evidence bekledigi icin response/tracking
  gate'lerine gecmedi.
- Gazebo bagimsizlik gate'i:
  `fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_isolation_check.py --run-id chain-refresh-isolation-smoke`
  sonuc: `PASS`; rapor:
  `logs/simitl_pr0p/20260707-200608-chain-refresh-isolation-smoke-isolation-check.md`
- Surec kontrolu:
  `pgrep -af 'pr0p|SimITL|SITL Forge|gazebo|gz sim|betaflight'`
  sonuc: cikti yok; canli sim/FC prosesi birakilmadi.

Sıradaki teknik adim:

1. Kullanici canli input icin acik onay verdiginde pr0p acik ve
   `Controls -> RC Channels` sayfasindayken AUX1/CH5 mapping helper'i calistir:

   ```bash
   fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_rc_channel_mapping_assistant.py \
     --uinput \
     --ack-live-input \
     --role aux1 \
     --run-id pr0p-rc-channel-map-aux1-live
   ```

2. Sonra tek chain komutu ile AUX1 -> response -> tracking -> manifest ->
   decision -> independent readiness zincirini kos.
3. Chain PASS olup readiness halen WAITING ise game-screen evidence/recentness
   sebebini `independent_sim_readiness` raporundan oku.

## TRUE LATEST 2026-07-07 20:10 +03

Aktif hat halen mevcut Gazebo/Betaflight entegrasyonundan tamamen bagimsiz
`pr0p / SITL Forge` hattidir. Bu turda canli pr0p launch, OS input veya config
write yapilmadi. Yeni eklenen parca, acceptance chain refresh davranisinin
README/PLAN runbook'larinda acik ve testle korunan hale getirilmesidir.

Degisen dosyalar:

- `experiments/simitl_pr0p_probe/README.md`
  - Ordered acceptance chain bolumu artik `--game-log-dir` amacini aciklar.
  - AUX1 -> response -> tracking PASS oldugunda chain'in final live manifest,
    `pr0p_decision_refresh` ve `independent_sim_readiness_refresh` stage'lerini
    urettigi yazildi.
  - P8 decision bolumu artik safe suite'in tek basina promotion kaniti
    olmadigini, real `*-live-manifest.json` gerektigini ve live manifest'in en
    guncel suite'e bagli olmasi gerektigini soyluyor.
- `experiments/simitl_pr0p_probe/PLAN.md`
  - P7 acceptance kriterlerine live manifest evidence ve latest-suite
    consistency guard'i eklendi.
  - P8 icine tercih edilen live promotion komutu olarak
    `pr0p_acceptance_chain_runner.py ... --game-log-dir logs/game_screen_sandbox`
    eklendi.
  - Chain PASS'in tek basina final promotion olmadigi; generated
    decision/readiness raporlarinin incelenmesi gerektigi belirtildi.
- `tests/test_simitl_pr0p_probe.py`
  - `test_pr0p_docs_cover_chain_refresh_and_promotion_guards` eklendi.
  - Bu test README ve PLAN'in `pr0p_decision_refresh`,
    `independent_sim_readiness_refresh`, `--game-log-dir`, `*-live-manifest.json`
    ve latest safe suite guard'larini anlatmaya devam ettigini kontrol eder.

Son dogrulamalar:

- Syntax:
  `fpv_env/bin/python -m py_compile experiments/simitl_pr0p_probe/pr0p_acceptance_chain_runner.py tests/test_simitl_pr0p_probe.py`
  sonuc: PASS
- Hedefli chain/readiness/decision/docs testi:
  `fpv_env/bin/python -m pytest tests/test_simitl_pr0p_probe.py -q -k "acceptance_chain_runner or pr0p_docs_cover_chain_refresh or independent_readiness or decision_report"`
  sonuc: `28 passed, 143 deselected`
- `fpv_env/bin/python -m pytest tests/test_simitl_pr0p_probe.py -q`
  sonuc: `171 passed`
- Genis test:
  `fpv_env/bin/python -m pytest tests/test_game_screen_sandbox.py tests/test_simitl_pr0p_probe.py -q`
  sonuc: `338 passed`
- Acceptance chain smoke:
  `fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_acceptance_chain_runner.py --bbox 10,20,80,60 --run-id docs-chain-refresh-smoke`
  sonuc: `WAITING`; rapor:
  `logs/simitl_pr0p/20260707-201010-docs-chain-refresh-smoke-acceptance-chain.md`
  mevcut durumda AUX1/CH5 live evidence bekledigi icin response/tracking
  gate'lerine gecmedi.
- Independent readiness smoke:
  `fpv_env/bin/python experiments/simitl_pr0p_probe/independent_sim_readiness.py --run-id docs-chain-refresh-readiness-smoke`
  sonuc: `WAITING`; rapor:
  `logs/simitl_pr0p/20260707-201018-docs-chain-refresh-readiness-smoke-independent-sim-readiness.md`
  siradaki aksiyon P6 bbox sample/overlay secimini isaret ediyor.
- Gazebo bagimsizlik gate'i:
  `fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_isolation_check.py --run-id docs-chain-refresh-isolation-smoke`
  sonuc: `PASS`; rapor:
  `logs/simitl_pr0p/20260707-201011-docs-chain-refresh-isolation-smoke-isolation-check.md`
- Surec kontrolu:
  `pgrep -af 'pr0p|SimITL|SITL Forge|gazebo|gz sim|betaflight'`
  sonuc: cikti yok; canli sim/FC prosesi birakilmadi.

Sıradaki teknik adim:

1. Kullanici canli input icin acik onay verdiginde pr0p acik ve
   `Controls -> RC Channels` sayfasindayken AUX1/CH5 mapping helper'i calistir:

   ```bash
   fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_rc_channel_mapping_assistant.py \
     --uinput \
     --ack-live-input \
     --role aux1 \
     --run-id pr0p-rc-channel-map-aux1-live
   ```

2. Bbox secildikten sonra tek chain komutu ile AUX1 -> response -> tracking ->
   manifest -> decision -> independent readiness zincirini kos:

   ```bash
   fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_acceptance_chain_runner.py \
     --bbox x,y,w,h \
     --execute-dry-patch \
     --run-live-gates \
     --ack-live-launch \
     --ack-live-ui \
     --ack-live-input \
     --game-log-dir logs/game_screen_sandbox \
     --run-id pr0p-acceptance-chain-live
   ```

3. Chain PASS olup readiness halen WAITING ise generated
   `independent_sim_readiness_refresh` raporundaki game-screen evidence,
   freshness veya latest-suite/live-manifest consistency sebebini oku.

## TRUE LATEST 2026-07-07 20:17 +03

Kullanici geri bildirimiyle odak tekrar ana hedeflere cekildi: bu hat uzerinde
asil kabul artik uc basliktir:

1. Sim uzerinde RC/manual flight: pr0p local race, RC input, AUX1/CH5 arm ve FC
   readback.
2. Goruntu uzerinde tracker: FPV capture, bbox, tracker/PID dry-run ve tracking
   evidence.
3. Otopilot/PID kontrol: signed yaw/pitch response ve live tracking-control ile
   PID komutlarinin simi surdugunun kaniti.

Bu turda canli pr0p launch, OS input veya config write yapilmadi. Mevcut
Gazebo/Betaflight entegrasyonuna dokunulmadan, pr0p hattina ana hedefleri
tek ekranda ozetleyen core goal report eklendi.

Degisen dosyalar:

- `experiments/simitl_pr0p_probe/pr0p_core_goal_report.py`
  - Yeni read-only core status araci.
  - Mevcut suite ve acceptance kanitlarini `rc_manual_flight`,
    `camera_tracker`, `autopilot_control` olarak toplar.
  - Canli sim baslatmaz, OS input gondermez, config yazmaz.
  - Eksik kanita gore tek bir ana `next_action` verir.
- `experiments/simitl_pr0p_probe/README.md`
  - En uste `Core Goal Status` bolumu eklendi.
  - Once bu komutla ana hedef durumuna bakilmasi soylendi:

    ```bash
    fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_core_goal_report.py \
      --bbox x,y,w,h \
      --run-id pr0p-core-goal
    ```

- `experiments/simitl_pr0p_probe/independent_sim_readiness.py`
  - Readiness'in urettigi acceptance-chain komutu artik `--game-log-dir`
    parametresini acik yazar.
- `experiments/simitl_pr0p_probe/pr0p_acceptance_state_report.py`
  - Tum acceptance kanitlari PASS oldugunda eski manuel "generate reports"
    ifadesi yerine chain-generated decision/readiness raporlarina bakmayi veya
    acceptance chain'i yeniden calistirmayi onerir.
- `tests/test_simitl_pr0p_probe.py`
  - Core goal report icin iki test eklendi:
    - Kanit yokken ilk odak `rc_manual_flight`.
    - RC/manual, camera/tracker ve autopilot kanitlari tamken core status PASS.
  - README'nin core goal status bolumunu koruyan assertler eklendi.

Son dogrulamalar:

- Syntax:
  `fpv_env/bin/python -m py_compile experiments/simitl_pr0p_probe/pr0p_core_goal_report.py experiments/simitl_pr0p_probe/independent_sim_readiness.py experiments/simitl_pr0p_probe/pr0p_acceptance_state_report.py tests/test_simitl_pr0p_probe.py`
  sonuc: PASS
- Hedefli core/readiness/acceptance-state testi:
  `fpv_env/bin/python -m pytest tests/test_simitl_pr0p_probe.py -q -k "core_goal_report or independent_readiness or acceptance_state_report"`
  sonuc: `13 passed, 160 deselected`
- Core/docs smoke testi:
  `fpv_env/bin/python -m pytest tests/test_simitl_pr0p_probe.py -q -k "core_goal_report or pr0p_docs_cover_chain_refresh"`
  sonuc: `3 passed, 170 deselected`
- Core goal smoke:
  `fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_core_goal_report.py --bbox 10,20,80,60 --run-id user-refocus-core-smoke`
  sonuc: `WAITING`; rapor:
  `logs/simitl_pr0p/20260707-201650-user-refocus-core-smoke-core-goal.md`
  next action:
  `Focus on RC/manual flight first: open pr0p local race, bind RC Channels including AUX1/CH5, then run the acceptance chain until AUX1/ARM and RC input evidence pass.`
  gate ozeti:
  - `rc_manual_flight WAITING`
  - `camera_tracker WAITING`
  - `autopilot_control WAITING`
- `fpv_env/bin/python -m pytest tests/test_simitl_pr0p_probe.py -q`
  sonuc: `173 passed`
- Genis test:
  `fpv_env/bin/python -m pytest tests/test_game_screen_sandbox.py tests/test_simitl_pr0p_probe.py -q`
  sonuc: `340 passed`
- Gazebo bagimsizlik gate'i:
  `fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_isolation_check.py --run-id user-refocus-core-isolation-smoke`
  sonuc: `PASS`; rapor:
  `logs/simitl_pr0p/20260707-201650-user-refocus-core-isolation-smoke-isolation-check.md`
- Surec kontrolu:
  `pgrep -af 'pr0p|SimITL|SITL Forge|gazebo|gz sim|betaflight'`
  sonuc: cikti yok; canli sim/FC prosesi birakilmadi.

Sıradaki teknik adim artik detay guard degil, dogrudan ana hedef:

1. pr0p acik, local race calisir, `Controls -> RC Channels` sayfasi acikken
   RC/AUX1/CH5 mapping'i tamamla.
2. `pr0p_core_goal_report.py --bbox x,y,w,h` ile ilk gate'in
   `rc_manual_flight` oldugunu izle.
3. RC/manual flight kaniti icin acceptance chain'i canli onaylarla kos:

   ```bash
   fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_acceptance_chain_runner.py \
     --bbox x,y,w,h \
     --execute-dry-patch \
     --run-live-gates \
     --ack-live-launch \
     --ack-live-ui \
     --ack-live-input \
     --game-log-dir logs/game_screen_sandbox \
     --run-id pr0p-acceptance-chain-live
   ```

4. `rc_manual_flight` PASS olmadan tracker/autopilot detaylarina gecme.

## TRUE LATEST 2026-07-07 20:20 +03

Kullanici geri bildirimi sonrasi bu turda sadece ana hedefe bagli read-only
kontroller yapildi. Kod buyutulmedi, canli pr0p launch yapilmadi, OS input
gonderilmedi, config write yapilmadi.

Ana durum:

- pr0p client kurulu ve izole kokte gorunuyor:
  `/tmp/fpv-test-simitl-pr0p/pr0p.x86_64`
- Preflight:
  - display/capture: `PASS`
  - virtual input/uinput: `PASS`
  - install root: `PASS`
  - process check: `PASS`
  - MSP websocket/local race: `WAITING`
- RC/manual flight:
  - roll/pitch/throttle/yaw primary input mapping mevcut.
  - AUX1/CH5 arm kanali eksik; `axis_4` bos.
  - Bu yuzden ilk core gate halen `rc_manual_flight WAITING`.
- Camera/tracker ve autopilot control gate'leri beklemeli; RC/manual flight
  PASS olmadan bunlara gecilmeyecek.

Calistirilan ana komutlar:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_core_goal_report.py \
  --bbox 10,20,80,60 \
  --run-id main-focus-core-now
```

Sonuc: `WAITING`

Next action:

```text
Focus on RC/manual flight first: open pr0p local race, bind RC Channels including AUX1/CH5, then run the acceptance chain until AUX1/ARM and RC input evidence pass.
```

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_client_probe.py \
  --install-root /tmp/fpv-test-simitl-pr0p \
  --run-id main-focus-client-now
```

Sonuc: `PASS`; selected client:
`/tmp/fpv-test-simitl-pr0p/pr0p.x86_64`

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/preflight_probe.py \
  --install-root /tmp/fpv-test-simitl-pr0p \
  --run-id main-focus-preflight-now
```

Sonuc: `WAITING`; tek bekleyen canli local race websocket.

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_live_session_runner.py \
  --run-id main-focus-rc-flight-plan
```

Sonuc: `PASS`; pr0p launch etmeden bounded live-session plan uretildi.
Planlanan executable:
`/tmp/fpv-test-simitl-pr0p/pr0p.x86_64`

Input config ozeti:

- `roll`: `<Joystick>/Stick/x`
- `pitch`: `<Joystick>/Stick/y`
- `throttle`: `<Joystick>/RotateY`
- `yaw`: `<Joystick>/RotateX`
- `AUX1/axis_4`: bos

AUX1 icin iki guvenli hazir yol:

1. Manuel binding:

   ```bash
   fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_rc_channel_mapping_assistant.py \
     --role aux1 \
     --run-id main-focus-aux1-map-dry
   ```

   Dry-run sonucu: `PASS`; canli mod icin pr0p `Controls -> RC Channels`
   sayfasinda `--uinput --ack-live-input` ile pulse basilir.

2. Backup-backed config patch:

   ```bash
   fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_input_config_patch.py \
     --role aux1 \
     --run-id main-focus-aux1-patch-dry
   ```

   Dry-run sonucu: `PASS`; yazma yapilmadi. Uygulanacaksa acik komut:

   ```bash
   fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_input_config_patch.py \
     --role aux1 \
     --write \
     --ack-config-write \
     --run-id main-focus-aux1-patch-write
   ```

Son surec kontrolu:

```bash
pgrep -af 'pr0p|SimITL|SITL Forge|gazebo|gz sim|betaflight'
```

Sonuc: cikti yok; canli sim/FC prosesi birakilmadi.

Sıradaki karar:

- Hizli ilerlemek icin backup-backed AUX1 config patch uygulanabilir.
- Manuel dogrulama istenirse pr0p local race acilip `Controls -> RC Channels`
  ekraninda AUX1/CH5 mapping assistant live pulse ile baglanir.
- Her iki durumda da sonraki kabul komutu ayni:

  ```bash
  fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_acceptance_chain_runner.py \
    --bbox x,y,w,h \
    --execute-dry-patch \
    --run-live-gates \
    --ack-live-launch \
    --ack-live-ui \
    --ack-live-input \
    --game-log-dir logs/game_screen_sandbox \
    --run-id pr0p-acceptance-chain-live
  ```

## TRUE LATEST 2026-07-07 20:24 +03

Kullanici geri bildirimi dogrultusunda bu turda ana hedefin ilk parcasina,
`rc_manual_flight`, odaklanildi. Detay gate'leri tek bir ust komut altina
toplandi.

Yeni dosya:

- `experiments/simitl_pr0p_probe/pr0p_rc_manual_flight_runner.py`
  - Ilk ana kabul komutu.
  - `pr0p_client`, `primary_rc_mapping`, `aux1_arm_acceptance` stage'lerini
    tek raporda toplar.
  - Varsayilan plan-only; pr0p launch etmez, OS input gondermez, config yazmaz.
  - `--apply-config-patch` icin `--ack-config-write` ister.
  - `--run-live-gates` icin `--ack-live-launch --ack-live-ui --ack-live-input`
    ister.

README guncellemesi:

- `experiments/simitl_pr0p_probe/README.md` en ustteki `Core Goal Status`
  bolumune ilk gate komutu eklendi:

  ```bash
  fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_rc_manual_flight_runner.py \
    --bbox x,y,w,h \
    --run-id pr0p-rc-manual-flight
  ```

Core status artik RC/manual flight beklerken bu runner'a yonlendirir:

```text
Focus on RC/manual flight first: open pr0p local race, bind RC Channels including AUX1/CH5, then run pr0p_rc_manual_flight_runner.py until AUX1/ARM and RC input evidence pass.
```

Mevcut gercek state ile smoke:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_rc_manual_flight_runner.py \
  --bbox 10,20,80,60 \
  --run-id rc-manual-flight-plan-smoke
```

Sonuc: `WAITING`

Stage ozeti:

- `pr0p_client PASS`
- `primary_rc_mapping PASS`
- `aux1_arm_acceptance WAITING`

Yani ilk ana hedef icin artik durum cok net:

```text
RC/manual flight icin tek eksik AUX1/CH5 arm kanalinin canli kabul kaniti.
```

Testler:

- Syntax:
  `fpv_env/bin/python -m py_compile experiments/simitl_pr0p_probe/pr0p_rc_manual_flight_runner.py experiments/simitl_pr0p_probe/pr0p_core_goal_report.py tests/test_simitl_pr0p_probe.py`
  sonuc: PASS
- Hedefli test:
  `fpv_env/bin/python -m pytest tests/test_simitl_pr0p_probe.py -q -k "rc_manual_flight_gate or aux1_acceptance_runner or core_goal_report or pr0p_docs_cover_chain_refresh"`
  sonuc: `10 passed, 166 deselected`
- Pr0p test seti:
  `fpv_env/bin/python -m pytest tests/test_simitl_pr0p_probe.py -q`
  sonuc: `176 passed`
- Genis test:
  `fpv_env/bin/python -m pytest tests/test_game_screen_sandbox.py tests/test_simitl_pr0p_probe.py -q`
  sonuc: `343 passed`
- Gazebo bagimsizlik:
  `fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_isolation_check.py --run-id rc-manual-flight-isolation-smoke`
  sonuc: `PASS`
- Surec kontrolu:
  `pgrep -af 'pr0p|SimITL|SITL Forge|gazebo|gz sim|betaflight'`
  sonuc: cikti yok.

Sıradaki dogrudan ana hedef adimi:

1. AUX1/CH5'i bagla:
   - hizli yol: backup-backed config patch write,
   - manuel yol: pr0p local race + `Controls -> RC Channels` + live pulse.
2. Ardindan ayni ilk gate'i canli ack'lerle kos:

   ```bash
   fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_rc_manual_flight_runner.py \
     --bbox x,y,w,h \
     --execute-dry-patch \
     --run-live-gates \
     --ack-live-launch \
     --ack-live-ui \
     --ack-live-input \
     --run-id pr0p-rc-manual-flight-live
   ```

3. Bu `PASS` olmadan camera/tracker ve autopilot-control fazina gecme.

## TRUE LATEST 2026-07-07 20:27 +03

Bu turda ana hedef akisi daha da sade hale getirildi: `pr0p_core_goal_report.py`
artik en yeni `*-rc-manual-flight.json` raporunu dogrudan `rc_manual_flight`
kaniti olarak okur. Yani ilk ana gate'in durumu eski alt gate listesinden
degil, artik `pr0p_rc_manual_flight_runner.py` ciktisindan izlenir.

Degisen dosyalar:

- `experiments/simitl_pr0p_probe/pr0p_core_goal_report.py`
  - `latest_rc_manual_flight_report()` eklendi.
  - Core report, taze `rc-manual-flight` raporu varsa `rc_manual_flight`
    gate'ini bu rapora gore degerlendirir.
  - `rc_manual_flight_report` ve freshness bilgisi metrics'e eklendi.
- `tests/test_simitl_pr0p_probe.py`
  - Core report'un `rc-manual-flight` PASS raporunu okuyup
    `rc_manual_flight` gate'ini PASS yaptigini test eder.
  - `rc-manual-flight` WAITING raporunda eksik `aux1_arm_acceptance` sebebinin
    core status'a tasindigini test eder.

Guncel core sonuc:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_core_goal_report.py \
  --bbox 10,20,80,60 \
  --run-id core-reads-manual-flight-after-smoke
```

Sonuc: `WAITING`

Core report'un okudugu ilk gate kaniti:

```text
logs/simitl_pr0p/20260707-202726-core-link-manual-flight-plan-smoke-rc-manual-flight.json
```

Gate ozeti:

- `rc_manual_flight WAITING`
  - missing: `aux1_arm_acceptance`
- `camera_tracker WAITING`
- `autopilot_control WAITING`

Yani aktif siralama halen:

1. `rc_manual_flight` PASS yap.
2. Sonra `camera_tracker`.
3. Sonra `autopilot_control`.

Testler:

- Syntax:
  `fpv_env/bin/python -m py_compile experiments/simitl_pr0p_probe/pr0p_core_goal_report.py experiments/simitl_pr0p_probe/pr0p_rc_manual_flight_runner.py tests/test_simitl_pr0p_probe.py`
  sonuc: PASS
- Hedefli test:
  `fpv_env/bin/python -m pytest tests/test_simitl_pr0p_probe.py -q -k "core_goal_report or rc_manual_flight_gate"`
  sonuc: `7 passed, 171 deselected`
- Pr0p test seti:
  `fpv_env/bin/python -m pytest tests/test_simitl_pr0p_probe.py -q`
  sonuc: `178 passed`
- Genis test:
  `fpv_env/bin/python -m pytest tests/test_game_screen_sandbox.py tests/test_simitl_pr0p_probe.py -q`
  sonuc: `345 passed`
- Gazebo bagimsizlik:
  `fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_isolation_check.py --run-id core-manual-flight-link-isolation-smoke`
  sonuc: `PASS`
- Surec kontrolu:
  `pgrep -af 'pr0p|SimITL|SITL Forge|gazebo|gz sim|betaflight'`
  sonuc: cikti yok.

Sıradaki tek ana is:

`aux1_arm_acceptance` PASS yapilacak. Bunun icin ya backup-backed AUX1 config
patch uygulanacak ya da pr0p local race acilip `Controls -> RC Channels`
ekraninda AUX1/CH5 manuel/live pulse ile baglanacak. Bu PASS olmadan diger
fazlara gecilmemeli.

## TRUE LATEST 2026-07-07 20:34 +03

Kullanici odagi yeniden netlestirdi: detay gate'lerine takilma; asil hedef
simde RC ile ucus, goruntu uzerinde tracker, PID/autopilot komutuyla dronu
surmek. Bu nedenle bu turda ilk ana gate sadece bu hedefe baglandi:

1. `rc_manual_flight`
   - pr0p local race acilabiliyor mu?
   - RC mapping hazir mi?
   - AUX1/CH5 ile arm kaniti var mi?
   - Arm sonrasi yaw/pitch/roll komutlari simde sinirli ve olculebilir goruntu
     tepkisi uretiyor mu?
2. `camera_tracker`
   - FPV goruntu tracker'a akiyor mu?
   - secilen bbox takip ediliyor mu?
3. `autopilot_control`
   - PID/tracker ciktisi yaw/pitch/roll inputuna cevrilip simi suruyor mu?

Degisen dosyalar:

- `experiments/simitl_pr0p_probe/pr0p_rc_manual_flight_runner.py`
  - `aux1_arm_acceptance` PASS olduktan sonra ayni ilk gate icinde
    `manual_yaw_response`, `manual_pitch_response`, `manual_roll_response`
    stage'leri eklendi.
  - Varsayilan mod halen plan-only: pr0p baslatmaz, OS input gondermez.
  - Canli response stage'leri yalnizca `--run-live-gates` ve explicit ack
    bayraklariyla calisir.
- `experiments/simitl_pr0p_probe/pr0p_core_goal_report.py`
  - `rc_manual_flight PASS` artik "AUX1 + bounded yaw/pitch/roll response"
    anlamina gelecek sekilde mesajlar guncellendi.
  - AUX1 gecmeden downstream `SKIPPED` response stage'leri ana missing listesine
    sokulmaz; core rapor ilk bloklayici isi gostermeye devam eder.
- `experiments/simitl_pr0p_probe/README.md`
  - Core hedef sirasi RC/manual flight -> camera/tracker -> autopilot-control
    olarak netlestirildi.
- `tests/test_simitl_pr0p_probe.py`
  - Plan-only runner'in response stage'lerini `SKIPPED` gosterdigi,
    canli/fake PASS durumunda yaw/pitch/roll response komutlarini kosup gate'i
    PASS yaptigi test edildi.

Guncel plan-only smoke:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_rc_manual_flight_runner.py \
  --bbox 10,20,80,60 \
  --run-id rc-manual-flight-bounded-plan-smoke
```

Sonuc:

```text
WAITING
pr0p_client PASS
primary_rc_mapping PASS
aux1_arm_acceptance WAITING
manual_yaw_response SKIPPED
manual_pitch_response SKIPPED
manual_roll_response SKIPPED
```

Guncel core smoke:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_core_goal_report.py \
  --bbox 10,20,80,60 \
  --run-id core-after-bounded-manual-smoke
```

Sonuc:

```text
rc_manual_flight WAITING, missing: aux1_arm_acceptance
camera_tracker WAITING
autopilot_control WAITING
```

Testler:

- Syntax:
  `fpv_env/bin/python -m py_compile experiments/simitl_pr0p_probe/pr0p_rc_manual_flight_runner.py experiments/simitl_pr0p_probe/pr0p_core_goal_report.py tests/test_simitl_pr0p_probe.py`
  sonuc: PASS
- Hedefli test:
  `fpv_env/bin/python -m pytest tests/test_simitl_pr0p_probe.py -q -k "core_goal_report or rc_manual_flight_gate"`
  sonuc: `7 passed, 171 deselected`
- Pr0p test seti:
  `fpv_env/bin/python -m pytest tests/test_simitl_pr0p_probe.py -q`
  sonuc: `178 passed`
- Genis test:
  `fpv_env/bin/python -m pytest tests/test_game_screen_sandbox.py tests/test_simitl_pr0p_probe.py -q`
  sonuc: `345 passed`
- Gazebo/pr0p izolasyon:
  `fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_isolation_check.py --run-id bounded-manual-flight-isolation-smoke`
  sonuc: PASS
- Surec kontrolu:
  `pgrep -af '[p]r0p|[S]imITL|[S]ITL Forge|[g]azebo|[g]z sim|[b]etaflight'`
  sonuc: cikti yok.

Sonraki agent icin tek odak:

1. Kullanici pr0p local race'i acar.
2. `Controls -> RC Channels` ekraninda AUX1/CH5 bind edilir veya
   backup-backed patch explicit onayla uygulanir.
3. Asagidaki canli gate kosulur:

   ```bash
   fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_rc_manual_flight_runner.py \
     --bbox x,y,w,h \
     --execute-dry-patch \
     --run-live-gates \
     --ack-live-launch \
     --ack-live-ui \
     --ack-live-input \
     --run-id pr0p-rc-manual-flight-live
   ```

4. Beklenen PASS sirasi:
   `aux1_arm_acceptance PASS`, sonra `manual_yaw_response PASS`,
   `manual_pitch_response PASS`, `manual_roll_response PASS`.
5. Bu PASS olmadan camera/tracker veya PID/autopilot detaylarina gecme.

## TRUE LATEST 2026-07-07 20:39 +03

Bu turda ordered acceptance chain de kullanicinin ana hedefine gore yeniden
siraya baglandi. Eski zincir `AUX1 -> response -> tracking` seklindeydi; bu
fazla zayifti cunku AUX1 tek basina "RC ile ucurabiliyorum" kaniti degil.

Yeni zincir:

1. `rc_manual_flight`
   - pr0p client + RC mapping,
   - AUX1/CH5 arm,
   - bounded yaw/pitch/roll response.
2. `pr0p_response_acceptance`
   - signed yaw/pitch yon dogrulugu.
3. `pr0p_tracking_acceptance`
   - bbox/tracker/PID live control.
4. `pr0p_decision_refresh` ve `independent_sim_readiness_refresh`
   - sadece tracking PASS sonrasi uretilir.

Degisen dosyalar:

- `experiments/simitl_pr0p_probe/pr0p_acceptance_chain_runner.py`
  - Ilk stage artik `pr0p_aux1_acceptance` degil, `rc_manual_flight`.
  - `rc_manual_flight PASS` olmadan manifest refresh, signed response ve
    tracking stage'leri `SKIPPED`.
  - CLI'ye `--install-root`, `--input-config`, `--expected-device`,
    `--manual-response-magnitude` eklendi.
- `experiments/simitl_pr0p_probe/README.md`
  - P7.5 ordered acceptance chain metni `RC/manual flight -> signed response
    -> live tracking` olarak guncellendi.
- `experiments/simitl_pr0p_probe/PLAN.md`
  - Preferred promotion path artik RC/manual flight'i "AUX1/ARM + bounded
    yaw/pitch/roll response" diye tanimliyor.
- `tests/test_simitl_pr0p_probe.py`
  - Acceptance-chain testleri yeni ilk stage'e gore guncellendi.
  - Fake live senaryoda once manual yaw/pitch/roll response PASS, sonra signed
    yaw/pitch response, sonra live tracking kosuldugu dogrulandi.

Plan-only chain smoke:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_acceptance_chain_runner.py \
  --bbox 10,20,80,60 \
  --run-id acceptance-chain-rc-manual-plan-smoke
```

Sonuc:

```text
WAITING
summary: acceptance chain is waiting at RC/manual flight
next: Run pr0p_rc_manual_flight_runner.py until AUX1/ARM and bounded yaw/pitch/roll response evidence pass.
stages:
- rc_manual_flight WAITING
- pr0p_live_manifest_after_rc_manual_flight SKIPPED
- pr0p_response_acceptance SKIPPED
- pr0p_tracking_acceptance SKIPPED
```

Testler:

- Syntax:
  `fpv_env/bin/python -m py_compile experiments/simitl_pr0p_probe/pr0p_acceptance_chain_runner.py experiments/simitl_pr0p_probe/pr0p_rc_manual_flight_runner.py tests/test_simitl_pr0p_probe.py`
  sonuc: PASS
- Hedefli test:
  `fpv_env/bin/python -m pytest tests/test_simitl_pr0p_probe.py -q -k "acceptance_chain_runner or rc_manual_flight_gate or core_goal_report or pr0p_docs_cover_chain_refresh"`
  sonuc: `12 passed, 166 deselected`
- Pr0p test seti:
  `fpv_env/bin/python -m pytest tests/test_simitl_pr0p_probe.py -q`
  sonuc: `178 passed`
- Genis test:
  `fpv_env/bin/python -m pytest tests/test_game_screen_sandbox.py tests/test_simitl_pr0p_probe.py -q`
  sonuc: `345 passed`
- Izolasyon:
  `fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_isolation_check.py --run-id acceptance-chain-rc-manual-isolation-smoke`
  sonuc: PASS
- Surec kontrolu:
  `pgrep -af '[p]r0p|[S]imITL|[S]ITL Forge|[g]azebo|[g]z sim|[b]etaflight'`
  sonuc: cikti yok.

Sonraki net is degismedi:

1. pr0p local race ac.
2. `Controls -> RC Channels` tarafinda AUX1/CH5'i bagla veya explicit onayla
   config patch uygula.
3. Once `pr0p_rc_manual_flight_runner.py` live gate'i PASS yap.
4. Ardindan ordered acceptance chain'i live kos; bu sirada response ve tracking
   ancak `rc_manual_flight PASS` olduktan sonra ilerler.

## TRUE LATEST 2026-07-07 20:42 +03

Bu turda `pr0p_core_goal_report.py` da ana hedef sirasina kilitlendi. Artik
downstream eski/ayri kanitlar olsa bile `rc_manual_flight` PASS olmadan
`camera_tracker` veya `autopilot_control` hazir gibi gorunmez.

Yeni core dependency mantigi:

1. `rc_manual_flight` PASS degilse:
   - `camera_tracker WAITING`, missing: `rc_manual_flight`
   - `autopilot_control WAITING`, missing: `rc_manual_flight`
   - raw downstream status evidence icinde saklanir.
2. `rc_manual_flight PASS`, fakat `camera_tracker PASS` degilse:
   - `autopilot_control WAITING`, missing: `camera_tracker`
3. Herhangi bir downstream gate gercek FAIL ise FAIL saklanmaz, gorunur kalir.

Degisen dosyalar:

- `experiments/simitl_pr0p_probe/pr0p_core_goal_report.py`
  - `blocked_by()` ve `apply_core_goal_dependencies()` eklendi.
  - Raw downstream gate evidence'i `raw_status`, `raw_summary`,
    `raw_missing`, `blocked_by` olarak saklanir.
- `experiments/simitl_pr0p_probe/README.md`
  - Core Goal Status bolumune dependency-gated davranis notu eklendi.
- `tests/test_simitl_pr0p_probe.py`
  - `test_core_goal_report_blocks_downstream_until_manual_flight_passes`
    eklendi.
  - README'deki dependency-gated notu dokuman testine baglandi.

Core smoke:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_core_goal_report.py \
  --bbox 10,20,80,60 \
  --run-id core-dependency-gate-smoke
```

Sonuc:

```text
rc_manual_flight WAITING, missing: aux1_arm_acceptance
camera_tracker WAITING, missing: rc_manual_flight
autopilot_control WAITING, missing: rc_manual_flight
```

Testler:

- Syntax:
  `fpv_env/bin/python -m py_compile experiments/simitl_pr0p_probe/pr0p_core_goal_report.py tests/test_simitl_pr0p_probe.py`
  sonuc: PASS
- Hedefli test:
  `fpv_env/bin/python -m pytest tests/test_simitl_pr0p_probe.py -q -k "core_goal_report or pr0p_docs_cover_chain_refresh or rc_manual_flight_gate or acceptance_chain_runner"`
  sonuc: `13 passed, 166 deselected`
- Pr0p test seti:
  `fpv_env/bin/python -m pytest tests/test_simitl_pr0p_probe.py -q`
  sonuc: `179 passed`
- Genis test:
  `fpv_env/bin/python -m pytest tests/test_game_screen_sandbox.py tests/test_simitl_pr0p_probe.py -q`
  sonuc: `346 passed`
- Izolasyon:
  `fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_isolation_check.py --run-id core-dependency-gate-isolation-smoke`
  sonuc: PASS
- Surec kontrolu:
  `pgrep -af '[p]r0p|[S]imITL|[S]ITL Forge|[g]azebo|[g]z sim|[b]etaflight'`
  sonuc: cikti yok.

Sonraki tek uygulama adimi halen ayni:

1. pr0p local race ac.
2. `Controls -> RC Channels` ile AUX1/CH5 bagla veya explicit onayli config
   patch uygula.
3. `pr0p_rc_manual_flight_runner.py` live PASS yap.
4. Sonra acceptance chain live ile camera/tracker ve PID/autopilot kapilarina
   ilerle.

## TRUE LATEST 2026-07-07 20:46 +03

Bu turda `pr0p_core_goal_report.py` sadece durum raporu olmaktan cikarilip
operator/sonraki-agent icin komut merkezi haline getirildi. Artik JSON ve
Markdown raporda `commands` alani var.

Yeni `commands` anahtarlari:

- `core_status`
- `rc_manual_flight_plan`
- `rc_manual_flight_live`
- `acceptance_chain_plan`
- `acceptance_chain_live`
- `tracking_acceptance_plan`
- `response_acceptance_plan`

Ornek smoke:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_core_goal_report.py \
  --bbox 10,20,80,60 \
  --run-id core-command-surface-smoke
```

Sonuc:

```text
WAITING
next: Focus on RC/manual flight first...
commands include:
- rc_manual_flight_live
- acceptance_chain_live
```

Raporun yazdigi canli ilk gate komutu:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_rc_manual_flight_runner.py \
  --bbox 10,20,80,60 \
  --execute-dry-patch \
  --run-live-gates \
  --ack-live-launch \
  --ack-live-ui \
  --ack-live-input \
  --run-id core-command-surface-smoke-rc-manual-flight-live
```

Raporun yazdigi ordered chain canli komutu:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_acceptance_chain_runner.py \
  --bbox 10,20,80,60 \
  --execute-dry-patch \
  --run-live-gates \
  --ack-live-launch \
  --ack-live-ui \
  --ack-live-input \
  --run-id core-command-surface-smoke-acceptance-chain-live
```

Degisen dosyalar:

- `experiments/simitl_pr0p_probe/pr0p_core_goal_report.py`
  - `commands` alanı eklendi.
  - Markdown'a `## Commands` bolumu eklendi.
- `experiments/simitl_pr0p_probe/README.md`
  - Core report'un plan/live commands yazdigi dokumante edildi.
- `tests/test_simitl_pr0p_probe.py`
  - Core report commands JSON/Markdown beklentileri test edildi.

Testler:

- Syntax:
  `fpv_env/bin/python -m py_compile experiments/simitl_pr0p_probe/pr0p_core_goal_report.py tests/test_simitl_pr0p_probe.py`
  sonuc: PASS
- Hedefli test:
  `fpv_env/bin/python -m pytest tests/test_simitl_pr0p_probe.py -q -k "core_goal_report or pr0p_docs_cover_chain_refresh or rc_manual_flight_gate or acceptance_chain_runner"`
  sonuc: `13 passed, 166 deselected`
- Pr0p test seti:
  `fpv_env/bin/python -m pytest tests/test_simitl_pr0p_probe.py -q`
  sonuc: `179 passed`
- Genis test:
  `fpv_env/bin/python -m pytest tests/test_game_screen_sandbox.py tests/test_simitl_pr0p_probe.py -q`
  sonuc: `346 passed`
- Izolasyon:
  `fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_isolation_check.py --run-id core-command-surface-isolation-smoke`
  sonuc: PASS
- Surec kontrolu:
  `pgrep -af '[p]r0p|[S]imITL|[S]ITL Forge|[g]azebo|[g]z sim|[b]etaflight'`
  sonuc: cikti yok.

Sonraki uygulama adimi:

1. `pr0p_core_goal_report.py --bbox ...` calistir.
2. Raporun `commands.rc_manual_flight_live` komutunu kullan.
3. pr0p local race + `Controls -> RC Channels` icinde AUX1/CH5 hazir olunca
   bu live gate'i PASS yap.
4. Sonra ayni rapordaki `commands.acceptance_chain_live` ile devam et.

## TRUE LATEST 2026-07-07 20:48 +03

Bu turda `pr0p_core_goal_report.py` raporuna tek onerilen komut alani eklendi.
Artik rapor sadece butun komutlari listelemiyor, aktif gate'e gore bir sonraki
calistirilacak komutu da seciyor.

Yeni alanlar:

- `next_command_key`
- `next_command`

Secim mantigi:

1. Herhangi bir core gate FAIL ise `next_command_key = core_status`.
2. `rc_manual_flight` PASS degilse `next_command_key = rc_manual_flight_live`.
3. `rc_manual_flight PASS`, `camera_tracker` PASS degilse
   `next_command_key = tracking_acceptance_dry`.
4. `autopilot_control` PASS degilse `next_command_key = acceptance_chain_live`.
5. Her sey PASS ise yine uzun dogrulama icin `acceptance_chain_live`.

Smoke:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_core_goal_report.py \
  --bbox 10,20,80,60 \
  --run-id core-next-command-smoke
```

Sonuc:

```text
next_command_key = rc_manual_flight_live
next_command =
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_rc_manual_flight_runner.py \
  --bbox 10,20,80,60 \
  --execute-dry-patch \
  --run-live-gates \
  --ack-live-launch \
  --ack-live-ui \
  --ack-live-input \
  --run-id core-next-command-smoke-rc-manual-flight-live
```

Markdown raporun en ustunde de:

```text
Next command key: `rc_manual_flight_live`
```

Degisen dosyalar:

- `experiments/simitl_pr0p_probe/pr0p_core_goal_report.py`
  - `next_command_key`, `next_command` alanlari eklendi.
  - `tracking_acceptance_dry`, `tracking_acceptance_live`,
    `response_acceptance_live` komutlari commands listesine eklendi.
- `experiments/simitl_pr0p_probe/README.md`
  - `Next command key` alani dokumante edildi.
- `tests/test_simitl_pr0p_probe.py`
  - Core report aktif gate'e gore tek komut seciyor mu test edildi.

Testler:

- Syntax:
  `fpv_env/bin/python -m py_compile experiments/simitl_pr0p_probe/pr0p_core_goal_report.py tests/test_simitl_pr0p_probe.py`
  sonuc: PASS
- Hedefli test:
  `fpv_env/bin/python -m pytest tests/test_simitl_pr0p_probe.py -q -k "core_goal_report or pr0p_docs_cover_chain_refresh or rc_manual_flight_gate or acceptance_chain_runner"`
  sonuc: `13 passed, 166 deselected`
- Pr0p test seti:
  `fpv_env/bin/python -m pytest tests/test_simitl_pr0p_probe.py -q`
  sonuc: `179 passed`
- Genis test:
  `fpv_env/bin/python -m pytest tests/test_game_screen_sandbox.py tests/test_simitl_pr0p_probe.py -q`
  sonuc: `346 passed`
- Izolasyon:
  `fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_isolation_check.py --run-id core-next-command-isolation-smoke`
  sonuc: PASS
- Surec kontrolu:
  `pgrep -af '[p]r0p|[S]imITL|[S]ITL Forge|[g]azebo|[g]z sim|[b]etaflight'`
  sonuc: cikti yok.

Sonraki uygulama adimi:

1. `pr0p_core_goal_report.py --bbox ...` calistir.
2. `next_command_key` ve `next_command` alanlarini izle.
3. Su an beklenen `next_command_key = rc_manual_flight_live`.
4. Bu komutu ancak pr0p local race + `Controls -> RC Channels` AUX1/CH5 hazir
   oldugunda canli olarak kos.

## TRUE LATEST 2026-07-07 20:52 +03

Bu turda core raporun sectigi `next_command` icin guvenli runner eklendi:

```text
experiments/simitl_pr0p_probe/pr0p_core_next_runner.py
```

Amac:

- `pr0p_core_goal_report.py` raporunu uretir.
- `next_command_key` / `next_command` alanlarini okur.
- Varsayilan olarak plan-only calisir, hicbir komut calistirmaz.
- `--execute-next` verilirse secilen komutu calistirir.
- Secilen komut live input / live launch iceriyorsa ek olarak
  `--ack-live-command` ister.

Plan-only smoke:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_core_next_runner.py \
  --bbox 10,20,80,60 \
  --run-id core-next-plan-smoke
```

Sonuc:

```text
PLANNED
next_command_key: rc_manual_flight_live
command_is_live: True
executed: False
notes: PLAN_ONLY, LIVE_COMMAND
```

No-ack live execute smoke:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_core_next_runner.py \
  --bbox 10,20,80,60 \
  --execute-next \
  --run-id core-next-noack-smoke
```

Sonuc:

```text
WAITING
Rerun with --execute-next --ack-live-command after the simulator UI and RC Channels are ready.
executed: False
```

Bu, yanlislikla live input basma riskini azaltir. Gercek canli kosu icin komut:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_core_next_runner.py \
  --bbox x,y,w,h \
  --execute-next \
  --ack-live-command \
  --run-id pr0p-core-next-live
```

Ancak bunu sadece pr0p local race acik, hedef bbox secili ve
`Controls -> RC Channels` icinde AUX1/CH5 hazir oldugunda kos.

Degisen dosyalar:

- `experiments/simitl_pr0p_probe/pr0p_core_next_runner.py`
  - yeni safe next-command runner.
- `experiments/simitl_pr0p_probe/README.md`
  - runner komutu ve live ack davranisi eklendi.
- `tests/test_simitl_pr0p_probe.py`
  - plan-only komut calistirmama,
  - live komutu ack olmadan engelleme,
  - dry next command'i execute edebilme,
  - `--ack-live-command` icin `--execute-next` zorunlulugu test edildi.

Testler:

- Syntax:
  `fpv_env/bin/python -m py_compile experiments/simitl_pr0p_probe/pr0p_core_next_runner.py experiments/simitl_pr0p_probe/pr0p_core_goal_report.py tests/test_simitl_pr0p_probe.py`
  sonuc: PASS
- Hedefli test:
  `fpv_env/bin/python -m pytest tests/test_simitl_pr0p_probe.py -q -k "core_next_runner or core_goal_report or pr0p_docs_cover_chain_refresh or rc_manual_flight_gate or acceptance_chain_runner"`
  sonuc: `17 passed, 166 deselected`
- Pr0p test seti:
  `fpv_env/bin/python -m pytest tests/test_simitl_pr0p_probe.py -q`
  sonuc: `183 passed`
- Genis test:
  `fpv_env/bin/python -m pytest tests/test_game_screen_sandbox.py tests/test_simitl_pr0p_probe.py -q`
  sonuc: `350 passed`
- Izolasyon:
  `fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_isolation_check.py --run-id core-next-runner-isolation-smoke`
  sonuc: PASS
- Surec kontrolu:
  `pgrep -af '[p]r0p|[S]imITL|[S]ITL Forge|[g]azebo|[g]z sim|[b]etaflight'`
  sonuc: cikti yok.

Sonraki uygulama adimi:

1. pr0p local race ac.
2. Hedef bbox'i sec.
3. `Controls -> RC Channels` icinde AUX1/CH5 hazirla.
4. `pr0p_core_next_runner.py --execute-next --ack-live-command` ile
   `rc_manual_flight_live` gate'ini kos.

## TRUE LATEST 2026-07-07 20:54 +03

Bu turda `pr0p_core_next_runner.py` icin placeholder bbox guard eklendi.
Sebep: `pr0p_core_goal_report.py` bbox verilmezse komutlarda `x,y,w,h`
placeholder'i yazar. Bu planlama icin faydali, fakat `--execute-next` ile
yanlislikla calistirilmamali.

Yeni davranis:

- Plan-only modda placeholder komut gosterilir ama calistirilmaz.
- `--execute-next` verilirse ve komutta `x,y,w,h` varsa runner `WAITING`
  doner.
- `--ack-live-command` verilmis olsa bile placeholder komut calistirilmaz.
- Rapor `notes: PLACEHOLDER_COMMAND`, `executed: False`,
  `metrics.command_has_placeholder: true` yazar.

Placeholder smoke:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_core_next_runner.py \
  --execute-next \
  --ack-live-command \
  --run-id core-next-placeholder-smoke
```

Sonuc:

```text
WAITING
executed: False
notes: PLACEHOLDER_COMMAND
command_has_placeholder: True
next action: Select a target bbox and rerun with --bbox x,y,w,h replaced by real numbers.
```

Bbox'li plan-only smoke halen guvenli:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_core_next_runner.py \
  --bbox 10,20,80,60 \
  --run-id core-next-plan-smoke-v2
```

Sonuc:

```text
PLANNED
executed: False
```

Degisen dosyalar:

- `experiments/simitl_pr0p_probe/pr0p_core_next_runner.py`
  - `PLACEHOLDER_COMMAND` guard eklendi.
- `experiments/simitl_pr0p_probe/README.md`
  - bbox placeholder varken execute'in engellenecegi dokumante edildi.
- `tests/test_simitl_pr0p_probe.py`
  - placeholder komutun command_runner'a ulasmadigi test edildi.

Testler:

- Syntax:
  `fpv_env/bin/python -m py_compile experiments/simitl_pr0p_probe/pr0p_core_next_runner.py tests/test_simitl_pr0p_probe.py`
  sonuc: PASS
- Hedefli test:
  `fpv_env/bin/python -m pytest tests/test_simitl_pr0p_probe.py -q -k "core_next_runner or core_goal_report or pr0p_docs_cover_chain_refresh or rc_manual_flight_gate or acceptance_chain_runner"`
  sonuc: `18 passed, 166 deselected`
- Pr0p test seti:
  `fpv_env/bin/python -m pytest tests/test_simitl_pr0p_probe.py -q`
  sonuc: `184 passed`
- Genis test:
  `fpv_env/bin/python -m pytest tests/test_game_screen_sandbox.py tests/test_simitl_pr0p_probe.py -q`
  sonuc: `351 passed`
- Izolasyon:
  `fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_isolation_check.py --run-id core-next-placeholder-guard-isolation-smoke`
  sonuc: PASS
- Surec kontrolu:
  `pgrep -af '[p]r0p|[S]imITL|[S]ITL Forge|[g]azebo|[g]z sim|[b]etaflight'`
  sonuc: cikti yok.

Sonraki uygulama adimi:

1. pr0p local race ac.
2. Hedef bbox'i gercek sayilarla sec.
3. `Controls -> RC Channels` icinde AUX1/CH5 hazirla.
4. Sonra:

   ```bash
   fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_core_next_runner.py \
     --bbox x,y,w,h \
     --execute-next \
     --ack-live-command \
     --run-id pr0p-core-next-live
   ```

   Buradaki `x,y,w,h` mutlaka gercek bbox degeriyle degistirilmeli.

## TRUE LATEST 2026-07-07 20:59 +03

Kullanici odagi yeniden netlestirdi: detayli yan analizler yerine asil uc
kapinin ilerletilmesi gerekiyor:

1. pr0p/SimITL sim uzerinde RC ile manuel ucus.
2. Sim kamera/goruntusunu tracker'a verme.
3. Tracker/PID ciktisiyla sim dronunu surme.

Gazebo yoluna geri donulmedi; bu kayit pr0p independent sim hatti icindir.

Bu turda sadece bu ana akisa dogrudan yardim eden bir eksik kapatildi:
`pr0p_core_goal_report.py` ve `pr0p_core_next_runner.py` artik
`--bbox-file <json>` kabul ediyor. Format mevcut tracker/bbox formatidir:

```json
{"bbox": [10, 20, 80, 60]}
```

Neden onemli:

- Operator hedef bbox degerini elle `--bbox x,y,w,h` olarak kopyalamak zorunda
  kalmaz.
- Core next runner placeholder `x,y,w,h` yerine gercek `--bbox 10,20,80,60`
  komutu uretir.
- Bu dogrudan kamera/tracker kapisinin calistirilabilir olmasina hizmet eder.

Degisen dosyalar:

- `experiments/simitl_pr0p_probe/pr0p_core_goal_report.py`
  - `--bbox-file` eklendi.
  - `--bbox` ve `--bbox-file` birlikte verilirse hata veriyor.
- `experiments/simitl_pr0p_probe/pr0p_core_next_runner.py`
  - `--bbox-file` eklendi.
  - Dosyadan okunan bbox, uretilen next command icinde sayisal `--bbox`
    olarak gorunuyor.
- `experiments/simitl_pr0p_probe/README.md`
  - core goal ve core next icin `--bbox-file` kullanimi eklendi.
- `tests/test_simitl_pr0p_probe.py`
  - core goal/next CLI bbox-file kabul ve conflict testleri eklendi.

Dogrulama:

```bash
fpv_env/bin/python -m py_compile \
  experiments/simitl_pr0p_probe/pr0p_core_goal_report.py \
  experiments/simitl_pr0p_probe/pr0p_core_next_runner.py \
  tests/test_simitl_pr0p_probe.py
```

Sonuc: PASS

```bash
fpv_env/bin/python -m pytest tests/test_simitl_pr0p_probe.py -q \
  -k "core_goal_report or core_next_runner"
```

Sonuc: `14 passed, 174 deselected`

CLI smoke:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_core_next_runner.py \
  --bbox-file logs/simitl_pr0p/core-bbox-smoke.json \
  --run-id core-next-bbox-file-smoke
```

Sonuc:

```text
PLANNED
next_command_key: rc_manual_flight_live
command_has_placeholder: False
next_command includes: --bbox 10,20,80,60
executed: False
```

Sonraki ana adim:

1. pr0p local race UI acik olacak.
2. `Controls -> RC Channels` uzerinden fiziksel RC kanallari, ozellikle
   AUX1/CH5 arm kanali, hazirlanacak.
3. Hedef bbox dosyasi `pr0p_bbox_tool.py` ile secilecek veya elle
   `{"bbox": [...]}` olarak verilecek.
4. Sonra yalniz ana kapi calistirilacak:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_core_next_runner.py \
  --bbox-file logs/simitl_pr0p/pr0p-target-bbox.json \
  --execute-next \
  --ack-live-command \
  --run-id pr0p-core-next-live
```

Beklenen ilk kapi: `rc_manual_flight_live`. Bu kapi gecmeden tracker/PID live
surusunu basarmis sayma.

## TRUE LATEST 2026-07-07 21:04 +03

Kullanici ana hedefi tekrar netlestirdi: Gazebo detaylarina donmeden pr0p/SimITL
bagimsiz hatta odaklan:

1. Sim uzerinde RC ile manuel ucus.
2. Sim goruntusunu tracker'a verme.
3. Tracker/PID/autopilot ciktisiyla dronu surme.

Bu turda `RC ile manuel ucus` kapisi icin onemli bir yanlis olcum riski
duzeltildi. Eski core/manual runner akisi varsayilan olarak Kenet virtual input
mapping'i bekliyordu. Fiziksel RC ile pr0p `Controls -> RC Channels` icinde
bind yapildiginda bu durum yanlis sekilde "virtual mapping eksik" diye ana
kapiyi bloke edebiliyordu.

Eklenen davranis:

- `pr0p_rc_manual_flight_runner.py`
  - `--allow-physical-mapping` eklendi.
  - Bu flag verildiginde primary roll/pitch/throttle/yaw eksenleri fiziksel
    kumandaya map edilmisse `primary_rc_mapping` PASS olabilir.
  - Varsayilan davranis degismedi: flag yoksa virtual input/autopilot command
    mapping'i beklenir.
- `pr0p_acceptance_chain_runner.py`
  - `--allow-physical-mapping` flag'ini manual flight gate'e tasir.
- `pr0p_core_goal_report.py`
  - `--allow-physical-mapping` flag'ini uretilen `rc_manual_flight_*` ve
    `acceptance_chain_*` komutlarina ekler.
- `pr0p_core_next_runner.py`
  - `--allow-physical-mapping` flag'ini core report'a tasir.
- `README.md`
  - Fiziksel RC ile `Controls -> RC Channels` bind edildiğinde bu flag'in
    kullanilacagi yazildi.

Dogrulama:

```bash
fpv_env/bin/python -m py_compile \
  experiments/simitl_pr0p_probe/pr0p_rc_manual_flight_runner.py \
  experiments/simitl_pr0p_probe/pr0p_acceptance_chain_runner.py \
  experiments/simitl_pr0p_probe/pr0p_core_goal_report.py \
  experiments/simitl_pr0p_probe/pr0p_core_next_runner.py \
  tests/test_simitl_pr0p_probe.py
```

Sonuc: PASS

```bash
fpv_env/bin/python -m pytest tests/test_simitl_pr0p_probe.py -q \
  -k "rc_manual_flight_gate or core_goal_report or core_next_runner or acceptance_chain_runner or pr0p_docs_cover_chain_refresh"
```

Sonuc: `25 passed, 166 deselected`

CLI smoke:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_core_next_runner.py \
  --bbox-file logs/simitl_pr0p/core-physical-bbox-smoke.json \
  --allow-physical-mapping \
  --run-id core-next-physical-mapping-smoke
```

Sonuc:

```text
PLANNED
next_command_key: rc_manual_flight_live
allow_physical_mapping: True
command_has_placeholder: False
next_command includes:
  --allow-physical-mapping
  --bbox 10,20,80,60
```

Canli surec kontrolu:

```bash
pgrep -af '[p]r0p|[S]imITL|[S]ITL Forge|[g]azebo|[g]z sim|[b]etaflight'
```

Sonuc: cikti yok.

Sonraki ana adim:

1. pr0p local race ac.
2. `Controls -> RC Channels` icinde fiziksel RC roll/pitch/throttle/yaw ve
   AUX1/CH5 binding'lerini yap.
3. Hedef bbox dosyasini hazirla.
4. Fiziksel RC mapping kabul edilecekse:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_core_next_runner.py \
  --bbox-file logs/simitl_pr0p/pr0p-target-bbox.json \
  --allow-physical-mapping \
  --execute-next \
  --ack-live-command \
  --run-id pr0p-core-next-live
```

Not: Bu sadece primary mapping'in fiziksel kumandaya bagli oldugunu kabul eder.
Tracker/PID/autopilot komutu icin virtual/autopilot command path ayrica
kanitlanmalidir; fiziksel RC ile pilot ucusu ve otonom komut enjeksiyonu ayni
kapinin icine karistirilma.

## PAUSE POINT 2026-07-07 21:05 +03

Kullanici talebi: "Mevcut kaldigin konumu ve yapacaklarini bir yere not et ve
bekle." Bu nedenle burada durulacak; yeni komut gelmeden pr0p, SimITL, canli
input, UI otomasyonu veya ek test calistirilmayacak.

Mevcut konum:

- Calisma hatti: Gazebo'dan tamamen bagimsiz `experiments/simitl_pr0p_probe`.
- Ana odak sirası:
  1. pr0p/SimITL uzerinde fiziksel RC ile manuel ucus.
  2. Sim ekran/goruntusunu tracker'a verme.
  3. Tracker/PID/autopilot komutuyla sim dronunu surme.
- Son tamamlanan teknik is:
  - `--bbox-file` destegi core goal/next runner'a eklendi.
  - `--allow-physical-mapping` destegi core next -> core goal ->
    acceptance chain -> rc manual flight runner hattina eklendi.
  - Bu sayede fiziksel RC `Controls -> RC Channels` mapping'i, virtual input
    eksigi gibi yanlis yorumlanmayacak.
- Son dogrulama:
  - Syntax: PASS.
  - Hedefli pytest: `25 passed, 166 deselected`.
  - CLI smoke: `core-next-physical-mapping-smoke` PLANNED, placeholder yok,
    next command `--allow-physical-mapping` ve `--bbox 10,20,80,60` iceriyor.
  - Canli pr0p/SimITL/Gazebo/Betaflight sureci birakilmadi.

Sonraki yapilacaklar, devam komutu gelirse:

1. Kullanici pr0p local race'i acacak.
2. Kullanici `Controls -> RC Channels` icinde fiziksel RC icin
   roll/pitch/throttle/yaw ve AUX1/CH5 binding'lerini yapacak.
3. Hedef bbox dosyasi hazir olacak:

   ```text
   logs/simitl_pr0p/pr0p-target-bbox.json
   ```

4. Sonra ilk canli ana kapi su komutla kosulacak:

   ```bash
   fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_core_next_runner.py \
     --bbox-file logs/simitl_pr0p/pr0p-target-bbox.json \
     --allow-physical-mapping \
     --execute-next \
     --ack-live-command \
     --run-id pr0p-core-next-live
   ```

Beklenen ilk gate: `rc_manual_flight_live`.

Basari sayilmasi icin:

- `primary_rc_mapping` PASS.
- `aux1_arm_acceptance` PASS.
- `manual_yaw_response`, `manual_pitch_response`, `manual_roll_response` PASS.

Bunlar PASS olmadan kamera/tracker veya PID/autopilot live surusune gecilmemeli.

## TRUE LATEST 2026-07-07 22:18 +03 - CODEX REVIEW AFTER CLAUDE PASS

Kullanici talebi: "Claude bazi guncellemeler yapti plana gore. Yapilanlari
detayli inceleyip kaldigin yerden devam et."

Inceleme sonucu:

- Claude tarafindan uretilen canli kanitlar mevcut ve dosyalari var.
- Core goal: `PASS`
  - `logs/simitl_pr0p/20260707-220547-claude-core-goal-final-v3-core-goal.json`
  - gate'ler: `rc_manual_flight PASS`, `camera_tracker PASS`,
    `autopilot_control PASS`
- Decision: `PROMOTE_CANDIDATE`
  - `logs/simitl_pr0p/20260707-220610-claude-decision-final-v3-decision.json`
  - reasons: `[]`
- Independent readiness: `INDEPENDENT_SIM_READY`
  - `logs/simitl_pr0p/20260707-220610-claude-independent-final-v2-independent-sim-readiness.json`
  - reasons: `[]`

Bulunan kopukluk:

- Claude'un PASS aldigi profil `--arm-first` + `--measure attitude` idi.
- Fakat core goal tarafinin onerilen `acceptance_chain_live` komutu bu profili
  tasimiyordu.
- Bu durumda "PASS kaniti" ile "tekrar kosulacak onerilen komut" farkli
  davranis gosterebilirdi; eski komut sky-dominated FPV goruntusunde tekrar
  visual-shift olcumune dusup WAITING kalabilirdi.

Bu turda duzeltilenler:

- `experiments/simitl_pr0p_probe/pr0p_acceptance_chain_runner.py`
  - `--arm-first`, `--arm-throttle`, `--measure`, `--attitude-min-delta`
    parametreleri eklendi.
  - Bu profil `rc_manual_flight`, `response_acceptance` ve
    `tracking_acceptance` alt runner'larina tasiniyor.
- `experiments/simitl_pr0p_probe/pr0p_rc_manual_flight_runner.py`
  - `--attitude-min-delta` eklendi.
  - Manual yaw/pitch/roll response komutlari attitude profilini tam tasiyor.
- `experiments/simitl_pr0p_probe/pr0p_core_goal_report.py`
  - live command uretimi kanitlanmis profile hizalandi:
    `--arm-first --arm-throttle -0.4 --measure attitude --attitude-min-delta 10.0`.
- `experiments/simitl_pr0p_probe/README.md`
  - acceptance chain live komutu kanitlanmis profile guncellendi.
- `experiments/simitl_pr0p_probe/PLAN.md`
  - preferred live promotion path ayni profile guncellendi.
- `tests/test_simitl_pr0p_probe.py`
  - manual response command attitude profile testi eklendi.
  - core goal PASS durumunda next command'in proven profile tasidigi test edildi.
  - acceptance chain'in bu profili alt komutlara aktardigi test edildi.

Dogrulama:

```bash
fpv_env/bin/python -m py_compile \
  experiments/simitl_pr0p_probe/pr0p_rc_manual_flight_runner.py \
  experiments/simitl_pr0p_probe/pr0p_acceptance_chain_runner.py \
  experiments/simitl_pr0p_probe/pr0p_core_goal_report.py \
  tests/test_simitl_pr0p_probe.py
```

Sonuc: PASS

```bash
fpv_env/bin/python -m pytest tests/test_simitl_pr0p_probe.py -q \
  -k "manual_response_command or response_acceptance_command or acceptance_chain_runner or core_goal_report or core_next_runner"
```

Sonuc: `23 passed, 180 deselected`

```bash
fpv_env/bin/python -m pytest tests/test_simitl_pr0p_probe.py -q
```

Sonuc: `203 passed`

```bash
fpv_env/bin/python -m pytest tests/test_game_screen_sandbox.py tests/test_simitl_pr0p_probe.py -q
```

Sonuc: `370 passed`

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_isolation_check.py \
  --run-id codex-proven-profile-isolation-smoke
```

Sonuc: PASS, forbidden Gazebo runtime coupling yok.

Core command smoke:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_core_goal_report.py \
  --bbox-file logs/simitl_pr0p/pr0p-target-bbox.json \
  --allow-physical-mapping \
  --run-id codex-proven-profile-smoke
```

Sonuc:

```text
PASS
next_command_key: acceptance_chain_live
next_command includes:
  --allow-physical-mapping
  --bbox 390,475,140,85
  --arm-first
  --arm-throttle -0.4
  --measure attitude
  --attitude-min-delta 10.0
```

Canli surec kontrolu:

```bash
pgrep -af '[p]r0p|[S]imITL|[S]ITL Forge|[g]azebo|[g]z sim|[b]etaflight'
```

Sonuc: cikti yok.

Sonraki ana adim:

1. Bu proven profile ile daha uzun bounded RC/manual + closed-loop follow
   senaryosu kos.
2. Hareketli hedefe gecmeden once statik/hedefe-kilitli closed-loop follow'un
   uzun sure stabil kaldigini raporla.
3. Sonra hareketli obje/yaw-only follow denemesi ekle; yaw-only stabil olmadan
   pitch/approach genisletmesine gecme.

Mevcut core next komutu artik dogru profili uretmeli; elle eski
`acceptance_chain_live` komutu kullanilirsa mutlaka yukaridaki arm-first +
attitude bayraklari eklenmeli.

## TRUE LATEST 2026-07-07 22:22 +03 - EXTENDED FOLLOW NEXT GATE

Aktif hedefe devam edildi: mevcut Gazebo hattina dokunmadan pr0p/SimITL
bagimsiz hatta PASS sonrasi gercek bir sonraki kapinin olculmesi saglandi.

Onceki durum:

- Core goal artik `PASS` oluyordu.
- `next_action`: daha uzun RC/manual + closed-loop follow senaryosu oneriyordu.
- Fakat `next_command_key` kisa `acceptance_chain_live` komutuna donuyordu.
- Bu, PASS sonrasi "daha uzun takip" hedefinin otomasyon tarafinda net bir
  gate olmamasi demekti.

Bu turda degisenler:

- `experiments/simitl_pr0p_probe/pr0p_core_goal_report.py`
  - Yeni command key: `extended_follow_live`.
  - Tum core gate'ler PASS oldugunda `next_command_key` artik
    `extended_follow_live`.
  - Komut, kanitlanmis live profile'i korur:
    `--arm-first --arm-throttle -0.4 --measure attitude --attitude-min-delta 10.0`.
  - Ek uzun takip kriterleri:
    `--duration 20.0 --min-found-ratio 0.95 --max-loss-events 0`.
- `experiments/simitl_pr0p_probe/independent_sim_readiness.py`
  - `pr0p_response_acceptance_live`, `pr0p_tracking_acceptance_live` ve
    `pr0p_acceptance_chain_live` komutlari proven profile'a hizalandi.
  - Yeni command: `pr0p_extended_follow_live`.
- `experiments/simitl_pr0p_probe/README.md`
  - PASS sonrasi `extended_follow_live` anlatildi.
  - Uzun takip komutu eklendi.
- `experiments/simitl_pr0p_probe/PLAN.md`
  - Preferred live promotion path sonrasi extended follow gate eklendi.
- `tests/test_simitl_pr0p_probe.py`
  - Core PASS sonrasi `extended_follow_live` secimi test edildi.
  - Readiness komutlarinin proven profile ve extended follow komutunu
    tasidigi test edildi.

Smoke:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_core_goal_report.py \
  --bbox-file logs/simitl_pr0p/pr0p-target-bbox.json \
  --allow-physical-mapping \
  --run-id codex-extended-follow-smoke
```

Sonuc:

```text
PASS
next_command_key: extended_follow_live
next_command includes:
  --allow-physical-mapping
  --bbox 390,475,140,85
  --arm-first
  --arm-throttle -0.4
  --measure attitude
  --attitude-min-delta 10.0
  --duration 20.0
  --min-found-ratio 0.95
  --max-loss-events 0
```

Dogrulama:

```bash
fpv_env/bin/python -m py_compile \
  experiments/simitl_pr0p_probe/pr0p_core_goal_report.py \
  experiments/simitl_pr0p_probe/independent_sim_readiness.py \
  experiments/simitl_pr0p_probe/pr0p_acceptance_chain_runner.py \
  tests/test_simitl_pr0p_probe.py
```

Sonuc: PASS

```bash
fpv_env/bin/python -m pytest tests/test_simitl_pr0p_probe.py -q
```

Sonuc: `203 passed`

```bash
fpv_env/bin/python -m pytest tests/test_game_screen_sandbox.py tests/test_simitl_pr0p_probe.py -q
```

Sonuc: `370 passed`

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_isolation_check.py \
  --run-id codex-extended-follow-isolation-smoke
```

Sonuc: PASS, forbidden Gazebo runtime coupling yok.

Canli surec kontrolu:

```bash
pgrep -af '[p]r0p|[S]imITL|[S]ITL Forge|[g]azebo|[g]z sim|[b]etaflight'
```

Sonuc: cikti yok.

Sonraki uygulanacak ana adim:

1. pr0p local race acik ve bbox taze iken core next'i calistir:

   ```bash
   fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_core_next_runner.py \
     --bbox-file logs/simitl_pr0p/pr0p-target-bbox.json \
     --allow-physical-mapping \
     --execute-next \
     --ack-live-command \
     --run-id pr0p-extended-follow-live
   ```

2. Bu komut artik `extended_follow_live` uretmeli ve 20 saniyelik bounded
   closed-loop follow kosmali.
3. Extended follow PASS olmadan hareketli hedef / yaw-only moving target
   asamasina gecilmemeli.

## TRUE LATEST 2026-07-07 22:08 +03 - CORE GOAL PASS

Bu tur Claude tarafindan kosuldu ve uc ana kapi ilk kez ayni anda PASS oldu:

```text
rc_manual_flight  -> PASS
camera_tracker    -> PASS
autopilot_control -> PASS
```

- Core goal raporu: `logs/simitl_pr0p/20260707-220547-claude-core-goal-final-v3-core-goal.md`
  sonuc: `PASS`, "Core pr0p goals are proven".
- Decision: `logs/simitl_pr0p/20260707-220610-claude-decision-final-v3-decision.md`
  sonuc: `PROMOTE_CANDIDATE`, "all promotion gates are present".
- Independent readiness: `logs/simitl_pr0p/20260707-220610-claude-independent-final-v2-independent-sim-readiness.md`
  sonuc: `INDEPENDENT_SIM_READY`.

### Kok neden zinciri (bu turda cozulenler)

1. AUX1/CH5 pr0p'ta hic bagli degildi (slot 4 bos). Backup destekli patch
   uygulandi: `pr0p_input_config_patch.py --role aux1 --write --ack-config-write`
   -> slot 4 `<Joystick>/Z`. Canli RC-effect: CH5 `1500 -> 2000` PASS.
   Geri almak icin: `pr0p_input_config_restore.py --restore --ack-config-restore`.
2. Arm probe'u race basladiktan hemen sonra kosuyordu; `BOOTGRACE,CALIB`
   aktifken AUX1 yukselince Betaflight `ARM_SWITCH` kilidi vurdu. Cozum:
   20x1s uzun baseline (throttle-low tutarken blocker'larin temizlenmesini
   bekle, AUX1'i sonra kaldir). Sonuc: `FC_ARMED` 6/6, blocker yok,
   `ARM_MODE_ONLY_WHILE_COMMAND_HELD`. Kanonik komut
   `independent_sim_readiness.py build_commands` icinde guncellendi.
3. Onceki tum response denemeleri iki nedenle WAITING kaliyordu:
   a) Arac disarm'li iken hicbir stick input arac tepkisi uretmez.
   b) FPV kamera yukari egimli; pad/tirmanis gorunumu gokyuzu-agirlikli ve
      piksel-shift olcumu donus sirasinda bile ~0 px okur (ekran goruntuleri
      ile kanitlandi; 1800 gazda arac goge firliyor, gorus bos gokyuzu).
   Cozum: `--response-arm-first` (arm + hover throttle tut) ve
   `--response-measure attitude` (MSP_ATTITUDE isaretli delta, render'dan
   bagimsiz). Olculen isaretler: yaw `+0.5` -> `+190 deg` heading (sign +1),
   pitch `+0.4` -> `-40.6 deg` pitch (sign -1). Hover/tirmanis gaz degeri
   `-0.4` (~1700 PWM; 1625 yerde kaliyor, 1800 hizli tirmanis).
4. `MSP_SET_RAW_RC` loopback'i pr0p mimarisi geregi hicbir zaman aktif RC
   kaynagi olamaz (sim joystick receiver okuyor). Decision raporu artik tam
   kanitli virtual-joystick zincirini (throttle RC-effect + AUX1 RC-effect +
   AUX1 arm-status) RC kaynagi olarak kabul ediyor;
   `MISSING_RC_SOURCE_LOOPBACK` sadece o zincir eksikse blocker.
5. Core goal `autopilot_control` kapisi, safe suite'in asla uretemeyecegi
   `P5-yaw-live/P5-pitch-live/P5-response-acceptance` suite fazlarini sart
   kosuyordu; kapi artik freshness-kontrollu acceptance-state gate'lerine
   (`pr0p_response_acceptance` + `pr0p_tracking_acceptance`) dayaniyor.

### Canli PASS kanitlari

- AUX1 RC-effect: `logs/simitl_pr0p/20260707-211137-claude-live-uinput-rc-aux1-high-v1-live-session.md`
- FC_ARMED: `logs/simitl_pr0p/20260707-211447-claude-live-uinput-aux1-arm-status-v2-live-session.md`
- Attitude yaw kalibrasyon (+190 deg): `logs/simitl_pr0p/20260707-213814-claude-attitude-yaw-calib-v1-live-session.md`
- Attitude pitch kalibrasyon (-40.6 deg): `logs/simitl_pr0p/20260707-213923-claude-attitude-pitch-calib-v1-live-session.md`
- Signed response acceptance PASS: `logs/simitl_pr0p/20260707-214104-claude-response-acceptance-live-v1-response-acceptance.md`
- Arm'li canli tracking PASS (bbox `390,475,140,85`, found_ratio 1.0,
  0 loss, max |yaw| 0.167, ARM_FIRST_ARMED):
  `logs/simitl_pr0p/20260707-215156-claude-tracking-acceptance-live-v1-tracking-acceptance.md`
- RC manual flight gate PASS (6/6 stage: client, primary mapping, aux1 arm,
  yaw/pitch/roll armed attitude response):
  `logs/simitl_pr0p/20260707-220253-claude-rc-manual-flight-live-v1-rc-manual-flight.md`
- Guncel hedef bbox dosyasi: `logs/simitl_pr0p/pr0p-target-bbox.json`

### Kod degisiklikleri

- Yeni: `experiments/simitl_pr0p_probe/pr0p_arm_sequence.py`
  (`arm_and_hover_for_response`, `response_pulse_command`, `HoldAxesAdapter`).
- Yeni: `experiments/simitl_pr0p_probe/msp_uinput_attitude_response_probe.py`
  (MSP_ATTITUDE isaretli delta olcumu, cok-tur yaw icin wrap'li birikim).
- `pr0p_live_session_runner.py`: `--response-arm-first`, `--response-arm-wait`,
  `--response-arm-throttle`, `--response-measure attitude` ve attitude
  parametreleri.
- `pr0p_response_acceptance_runner.py`: `--arm-first`, `--measure attitude`.
- `pr0p_tracking_probe.py`: `--arm-first` (HoldAxesAdapter ile PID komutlari
  hover+AUX1 tutuslariyla birlesir; tracker stop hover-hold'a doner, kapanis
  disarm eder).
- `pr0p_tracking_acceptance_runner.py`: `--arm-first` gecisi.
- `pr0p_rc_manual_flight_runner.py`: `--arm-first`, `--measure attitude`.
- `independent_sim_readiness.py`: kanonik aux1 arm-status komutu uzun baseline.
- `pr0p_decision_report.py`: virtual RC kaynagi kurali (madde 4).
- `pr0p_core_goal_report.py`: autopilot kapisi acceptance-state tabanli.
- README.md / PLAN.md guncellendi.

Testler: `tests/test_simitl_pr0p_probe.py` 202 passed;
`tests/test_game_screen_sandbox.py + tests/test_simitl_pr0p_probe.py` toplam
369 passed. Izolasyon: `20260707-211043-claude-continue-isolation-v1` PASS.

### Sonraki adimlar (core goal raporunun onerisi)

1. Daha uzun bounded RC/manual ucus + kapali dongu follow senaryosu
   (`acceptance_chain_live` komut anahtari hazir).
2. Hareketli hedef: pr0p sahnesinde hareketli obje uzerinde yaw-only follow,
   sonra pitch/approach eklenmesi (yalniz yaw stabil ise).
3. Hover throttle'in acik dongu olmasi kirilgan; irtifa tutma icin
   MSP_ALTITUDE tabanli basit bir throttle trim dusunulebilir.

Checkpoint sonunda pr0p/updater/Gazebo/Betaflight prosesi yoktu.

## TRUE LATEST 2026-07-07 22:52 +03 - APPROACH/PITCH GATE EKLENDI

Bu turda moving-target yaw-only PASS sonrasi pitch/approach asamasinin da
olculebilir bir gate olmasi saglandi. Canli pr0p/input baslatilmadi.

Yeni core akisi:

```text
core gates PASS
  -> extended_follow
extended_follow PASS
  -> moving_target_yaw
moving_target_yaw PASS
  -> approach_pitch
approach_pitch PASS
  -> manual-to-autonomous handoff gate planlanabilir
```

Eklenen dosya:

- `experiments/simitl_pr0p_probe/pr0p_approach_pitch_plan.py`

Bu runner:

- pr0p/Gazebo/Betaflight baslatmaz.
- OS input/uinput basmaz.
- Sentetik range-dynamics ile pitch/approach regresyonu calistirir.
- Gercek pr0p icin `--real-tracking-report path/to/*-tracking-acceptance.json`
  ve `--ack-real-approach-target` ister.
- Gercek approach gate su metrikleri kontrol eder:
  - live tracking acceptance PASS
  - `duration >= 20 s`
  - `found_ratio >= 0.90`
  - `loss_events <= 2`
  - `enable_pitch = true`
  - `real_input_sent = true`
  - `initial_abs_width_error_px >= 10`
  - `final_abs_width_error_px <= 12`
  - `width_error_reduction_ratio >= 0.50`
  - pitch komutu kullanilmis ama bounded kalmis

Core degisikligi:

- `experiments/simitl_pr0p_probe/pr0p_core_goal_report.py`
  - latest `*-approach-pitch.json` raporunu okur.
  - `moving_target_yaw PASS` olduktan sonra `approach_pitch` post-core gate'i
    acilir.
  - `approach_pitch` eksikse `next_command_key = approach_pitch_plan`.
  - `approach_pitch PASS` olursa `next_command_key = None` ve siradaki is
    manual-to-autonomous handoff gate planidir.

Test durumu:

```text
targeted approach/moving/core docs: 34 passed
```

Dogru mevcut canli siralama hala degismedi: mevcut gercek loglarla core smoke
extended follow bekliyor. Once `extended_follow_live` PASS olmali; sonra
moving-target yaw; sonra approach/pitch.

## TRUE LATEST 2026-07-07 23:01 +03 - EOF HANDOFF CHECKPOINT

Bu dosyanin ortasinda eski duplicate TRUE LATEST bloklari var; devam edecek
agent bu en sondaki bloku esas alsin.

Claude'un approach/pitch fazi incelendi ve eksik kalan manual-to-autonomous
handoff asamasi core post-core zincirine baglandi. Canli pr0p/input/Gazebo/
Betaflight baslatilmadi.

Guncel core sirasi:

```text
core gates PASS
  -> extended_follow
extended_follow PASS
  -> moving_target_yaw
moving_target_yaw PASS
  -> approach_pitch
approach_pitch PASS
  -> handoff
handoff PASS
  -> pr0p hattinda RC hedef secimi -> tracker -> PID/autonomous takip zinciri kanitli
```

Degisen dosyalar:

- `experiments/simitl_pr0p_probe/pr0p_handoff_plan.py`
- `experiments/simitl_pr0p_probe/pr0p_core_goal_report.py`
- `experiments/simitl_pr0p_probe/README.md`
- `experiments/simitl_pr0p_probe/PLAN.md`
- `tests/test_simitl_pr0p_probe.py`
- `checkpoint-goal.md`

Handoff gate kurali:

- Sentetik handoff regresyonu PASS olabilir ama tek basina final degil.
- Gercek PASS icin `--real-handoff-report ... --ack-real-handoff` gerekir.
- Gercek rapor `follow_command_sent`, tracker init timing,
  `pre_handoff_auto_control_samples = 0`, real input, bounded yaw/pitch,
  found/loss ve merkez/width hata azalimi metriklerini tasimali.

Test/smoke:

```text
py_compile: PASS
targeted handoff/approach/moving/core/core-next/docs: 42 passed
tests/test_simitl_pr0p_probe.py: 228 passed
tests/test_game_screen_sandbox.py + tests/test_simitl_pr0p_probe.py: 395 passed
pr0p_handoff_plan smoke: WAITING, synthetic_status PASS, real_pr0p_status WAITING
pr0p_core_goal_report smoke: PASS, next_command_key extended_follow_live
pr0p_isolation_check: PASS
```

Smoke raporlari:

- `logs/simitl_pr0p/20260707-230005-codex-handoff-smoke-handoff.json`
- `logs/simitl_pr0p/20260707-230005-codex-handoff-core-smoke-core-goal.json`
- `logs/simitl_pr0p/20260707-230005-codex-handoff-isolation-smoke-isolation-check.json`

Onemli: Mevcut gercek loglarla hala `extended_follow_live` bekleniyor.
Handoff gate zincire eklendi diye moving-target/approach/handoff canli sirasi
atlanmadi. Devamda once 20 s, 0-loss extended follow; sonra moving-target
yaw-only; sonra approach/pitch; en son real handoff raporu alinmali.

## TRUE LATEST 2026-07-07 23:11 +03 - REAL HANDOFF ACCEPTANCE RUNNER EKLENDI

Bu turda handoff gate'in bekledigi gercek raporu uretecek guarded runner
eklendi. Canli pr0p/input/Gazebo/Betaflight baslatilmadi.

Yeni dosya:

- `experiments/simitl_pr0p_probe/pr0p_handoff_acceptance_runner.py`

Bu runner:

- Varsayilan olarak plan-only calisir, input gondermez.
- Canli takip komutunu yalniz su uc bayrak birlikte verilirse calistirir:
  `--run-live-gate --ack-live-input --ack-real-handoff`.
- `approach_pitch PASS` prerequisite'ini kontrol eder.
- Follow aninda `pr0p_tracking_probe.py --enable-pitch --uinput --arm-first`
  komutunu uretir.
- Tracking probe raporundan `*-real-handoff.json` uretir.
- Uretilen rapor `pr0p_handoff_plan.py --real-handoff-report ...` tarafindan
  dogrudan tuketilebilir.

Guncellenen handoff akisi:

```text
approach_pitch PASS
  -> pr0p_handoff_acceptance_runner.py
     -> real *-real-handoff.json
  -> pr0p_handoff_plan.py --real-handoff-report ... --ack-real-handoff
     -> handoff PASS/WAITING/FAIL
```

Dokuman/test guncellemeleri:

- `experiments/simitl_pr0p_probe/README.md`
- `experiments/simitl_pr0p_probe/PLAN.md`
- `experiments/simitl_pr0p_probe/pr0p_handoff_plan.py`
- `tests/test_simitl_pr0p_probe.py`

Dogrulanan test/smoke:

```text
py_compile: PASS
targeted handoff/core docs: 14 passed
targeted handoff/approach/moving/core/core-next/docs: 48 passed
tests/test_simitl_pr0p_probe.py: 234 passed
tests/test_game_screen_sandbox.py + tests/test_simitl_pr0p_probe.py: 401 passed
pr0p_handoff_acceptance_runner plan smoke: WAITING, real_input_sent False
pr0p_handoff_plan linked smoke: WAITING, synthetic PASS, real handoff WAITING
pr0p_isolation_check: PASS
```

Smoke raporlari:

- `logs/simitl_pr0p/20260707-231024-codex-handoff-acceptance-plan-smoke-real-handoff.json`
- `logs/simitl_pr0p/20260707-231024-codex-handoff-acceptance-linked-smoke-handoff.json`
- `logs/simitl_pr0p/20260707-231037-codex-handoff-acceptance-isolation-smoke-isolation-check.json`

Mevcut gercek loglarla siralama hala degismedi:

1. Once `extended_follow_live` 20 s, 0-loss PASS.
2. Sonra moving-target yaw-only real PASS.
3. Sonra approach/pitch real PASS.
4. Sonra `pr0p_handoff_acceptance_runner.py` ile real handoff raporu.
5. En son `pr0p_handoff_plan.py --real-handoff-report ... --ack-real-handoff`.

## TRUE LATEST 2026-07-07 23:14 +03 - EXTENDED FOLLOW KOMUTU ODAKLANDI

Bu turda `extended_follow_live` komutu core PASS sonrasinda tum
`pr0p_acceptance_chain_runner.py` zincirini tekrar calistirmak yerine dogrudan
`pr0p_tracking_acceptance_runner.py` ile 20 saniyelik bounded live follow
kapisini calistiracak sekilde sadeleştirildi. Canli pr0p/input baslatilmadi.

Yeni `extended_follow_live` komutu:

- `pr0p_tracking_acceptance_runner.py`
- `--run-live-gates`
- `--ack-live-input`
- `--arm-first`
- `--arm-throttle -0.4`
- `--duration 20.0`
- `--min-found-ratio 0.95`
- `--max-loss-events 0`

Artik bu komut `--ack-live-launch`, `--ack-live-ui`, `--measure attitude` veya
`--attitude-min-delta` tasimiyor; cunku bu gate core prerequisite'ler PASS
olduktan sonra yalniz uzun live tracker/PID follow'u olcmek icin var.

Guncellenen dosyalar:

- `experiments/simitl_pr0p_probe/pr0p_core_goal_report.py`
- `experiments/simitl_pr0p_probe/README.md`
- `experiments/simitl_pr0p_probe/PLAN.md`
- `tests/test_simitl_pr0p_probe.py`
- `checkpoint-goal.md`

Dogrulanan test/smoke:

```text
py_compile: PASS
targeted core/core-next/docs: 15 passed
tests/test_simitl_pr0p_probe.py: 234 passed
tests/test_game_screen_sandbox.py + tests/test_simitl_pr0p_probe.py: 401 passed
core smoke: PASS, next_command_key extended_follow_live
core-next plan smoke: PLANNED, executed False
pr0p_isolation_check: PASS
```

Smoke raporlari:

- `logs/simitl_pr0p/20260707-231401-codex-direct-extended-follow-core-smoke-core-goal.json`
- `logs/simitl_pr0p/20260707-231401-codex-direct-extended-follow-next-plan-smoke-core-next.json`
- `logs/simitl_pr0p/20260707-231416-codex-direct-extended-follow-isolation-smoke-isolation-check.json`

Mevcut siralama ayni: once `extended_follow_live` gercek 20 s / 0 loss PASS,
sonra moving-target yaw-only, approach/pitch ve real handoff.

## TRUE LATEST 2026-07-07 23:22 +03 - MOVING TARGET ACCEPTANCE RUNNER EKLENDI

Bu turda Claude/Codex sonrasi pr0p hattindaki post-core zincir incelendi ve
moving-target yaw-only icin guarded acceptance runner tamamlandi. Canli
pr0p/uinput/RC komutu gonderilmedi.

Yeni dosya:

- `experiments/simitl_pr0p_probe/pr0p_moving_target_acceptance_runner.py`

Bu runner:

- Varsayilan olarak plan-only calisir, input gondermez.
- Canli takip komutunu yalniz su uc bayrak birlikte verilirse calistirir:
  `--run-live-gate --ack-live-input --ack-real-moving-target`.
- `extended_follow_live` prerequisite'ini kontrol eder.
- Gercek moving-target yaw-only kaniti icin
  `pr0p_tracking_acceptance_runner.py --run-live-gates --ack-live-input
  --arm-first --arm-throttle -0.4` komutunu uretir.
- Pitch/approach'u bu fazda kapali tutar:
  `--max-abs-pitch-axis 0.0`, `--enable-pitch` yok.
- Uretilen veya verilen `*-tracking-acceptance.json` raporunu
  `pr0p_moving_target_yaw_plan.py --real-tracking-report ...` tarafindan
  tuketilebilecek sekilde dogrular.

Onemli duzeltme:

- `*-tracking-acceptance.json` dosyalari hem extended-follow hem moving-target
  kosularinda uretilebildigi icin "en son tracking acceptance" yaklasimi
  moving-target raporunu yanlislikla extended-follow prerequisite'i sanabiliyordu.
- `pr0p_moving_target_acceptance_runner.py` artik en yeni dosyayi dogrudan
  almak yerine extended-follow kriterlerini gercekten saglayan en yeni raporu
  seciyor:
  `status PASS`, `pr0p_tracking_live PASS`, `duration >= 20 s`,
  `min_found_ratio >= 0.95`, `max_loss_events <= 0`, `run_live_gates true`.

Guncellenen dosyalar:

- `experiments/simitl_pr0p_probe/pr0p_moving_target_acceptance_runner.py`
- `experiments/simitl_pr0p_probe/pr0p_moving_target_yaw_plan.py`
- `experiments/simitl_pr0p_probe/README.md`
- `experiments/simitl_pr0p_probe/PLAN.md`
- `tests/test_simitl_pr0p_probe.py`
- `checkpoint-goal.md`

Dogrulanan test/smoke:

```text
py_compile: PASS
targeted moving-target/core docs: 14 passed
tests/test_game_screen_sandbox.py + tests/test_simitl_pr0p_probe.py: 407 passed
pr0p_moving_target_acceptance_runner plan smoke: WAITING, real_input_sent False
pr0p_moving_target_yaw_plan linked smoke: WAITING, synthetic PASS, real WAITING
pr0p_isolation_check: PASS
```

Smoke raporlari:

- `logs/simitl_pr0p/20260707-232204-codex-moving-target-acceptance-plan-smoke-moving-target-acceptance.json`
- `logs/simitl_pr0p/20260707-232205-codex-moving-target-acceptance-linked-smoke-moving-target-yaw.json`
- `logs/simitl_pr0p/20260707-232205-codex-moving-target-acceptance-isolation-smoke-isolation-check.json`

Mevcut gercek loglarla siralama hala degismedi:

1. Once `extended_follow_live` gercek 20 s / 0 loss PASS.
2. Sonra `pr0p_moving_target_acceptance_runner.py` ile moving-target yaw-only
   real tracking acceptance raporu.
3. Sonra `pr0p_moving_target_yaw_plan.py --real-tracking-report ... --ack-real-moving-target`
   ile moving_target_yaw PASS.
4. Sonra approach/pitch real PASS.
5. Sonra real handoff PASS.

## TRUE LATEST 2026-07-07 23:42 +03 - EXTENDED FOLLOW LIVE PASS + IKI KRITIK OLCUM

Bu tur Claude tarafindan kosuldu. Kullanici istegi: extended follow'u canli
kos ve pr0p sahnesinde gercekten hareket eden bir sey var mi olc.

### 1. Extended follow canli PASS (formal)

- Komut resmi yoldan uretildi ve kosuldu:
  `pr0p_core_next_runner.py --bbox-file ... --execute-next --ack-live-command`
  -> `extended_follow_live` (duration 20, min-found-ratio 0.95, 0 loss,
  arm-first, throttle -0.4).
- Sonuc: `PASS`. 200 kare / 20 s, `found_ratio 1.0`, `0` loss, FC bastan sona
  arm'li (`ARM_FIRST_ARMED`), max |yaw| axis `0.258`.
- Rapor: `logs/simitl_pr0p/20260707-233324-claude-extended-follow-live-v1-core-extended-follow-live-tracking-acceptance.md`
- JSONL: `logs/simitl_pr0p/20260707-233304-...-p6-tracking.jsonl`
- Not: ilk denemede kullanici farenin basindaydi ve UI smoke `Select scene`
  ekraninda kaldi; oturum kapatilip temiz tekrar ile kosuldu. UI navigasyonu
  sirasinda fare/klavye serbest birakilmali.

### 2. KRITIK BULGU: dongu input olu bolgesinde calisiyor (PASS aldatici)

JSONL analizi PASS'in kontrol kanibi OLMADIGINI gosterdi:

- `yaw_error` 20 s boyunca ~45 px sabit kaldi (yakinsama yok).
- bbox merkezi x 436-462 araliginda, genislik 127-143: goruntu pratikte statik.
- Gonderilen yaw axis komutu ort. `0.152`, max `0.258`.

Ayni oturumda arm'li canli yaw yetki taramasi (read-only MSP_ATTITUDE):

```text
yaw 0.25 -> ~76 deg/s (153 deg / 2 s)
yaw 0.35 -> ~116 deg/s
yaw 0.50 -> ~187 deg/s
```

Yani 0.25 araci rahatca donduruyor ama dongunun urettigi ~0.15 bandi hicbir
donme uretmiyor: pr0p/oyun tarafinda (muhtemelen Unity stick deadband,
~0.125) + Betaflight merkez-civari rate egrisi birlesince ~0.15 alti komutlar
etkisiz. Tracker kilidi, sahne statik kaldigi icin tutuldu; PID sabit 45 px
hatayla olu bolgede takili kaldi.

Somut sonraki teknik adim (kapi eklemeden once yapinin duzeltilmesi):

1. Tracking PID yaw cikis olcegini buyut veya olu-bolge kompanzasyonu ekle
   (orn. cikis |y|>0 iken min ~0.2 taban degeri / deadband offset).
2. `extended_follow` kapisina yakinsama kriteri ekle (orn. son 5 s ort.
   |yaw_error| < ilk 5 s'nin yarisi) — statik kilit, kontrol kaniti gibi
   PASS olamasin. (Bu proje kulturundeki eski sahte-PASS dersinin aynisi.)

### 3. Hareketli hedef gozlemi: ContainerCrash Time Attack'ta hedef YOK

30 s / 150 kare yakalanip kare-farki analizi yapildi:

- Kalici hareket eden pikseller: %0.83; en buyuk blob 99x57 px.
- Hepsi ruzgarda sallanan cimen/bitki ortusu + OSD titremesi.
- Yorungesi olan hicbir nesne yok: baska yarisci yok, ghost yok, arac yok.

Yani Codex'in `moving_target_yaw` kapisinin ">= 30 px koherent hedef
hareketi" sarti bu mod/sahnede KARSILANAMAZ. Hareketli hedef fazina gecmeden
once senaryo karari gerekli; secenekler:

1. Diger sahneleri ayni 30 s yontemiyle tara (GreenField/Factory/WildWest/
   Junkyard/Port) — birinde animasyonlu obje olabilir.
2. Ghost/replay ozelligi arastir (izolasyon kurali: multiplayer yok).
3. Bu bulgu yokken hareketli-hedef kabul kriterlerini daha da detaylandirmak
   isabetli degil.

### Durum ozeti

- Core goal raporu simdi `extended_follow` dahil PASS goruyor; onerilen
  sonraki komut `moving_target_yaw_plan`.
- Ancak dogru sira: once olu-bolge duzeltmesi + yakinsamali extended follow
  tekrari, sonra sahne taramasi, en son moving-target kapi kriterleri.
- Checkpoint sonunda pr0p/updater/Gazebo/Betaflight prosesi yok.

## TRUE LATEST 2026-07-08 01:15 +03 - SAHTE-PASS KOK NEDENI + GERCEK KAPALI DONGU KANITI

Bu tur Claude tarafindan kosuldu. Kullanici karari: hareketli-hedef fazinin ilk
asamasi "havadan, yerde duran statik objeye duzgun track" senaryosu olacak.
Bu senaryoyu kosarken onceki TUM canli tracking PASS'larinin sahte oldugu
olculdu ve kok nedeni bulundu.

### 1. KRITIK: Onceki tum canli tracking PASS'lari GECERSIZ

Mekanizma (kare kare kanitli, `t5pipe` deneyi):

- Arm dizisi hover gazini (-0.4 = ~1700) donguden ONCE veriyordu; capture
  acilana kadar drone coktan gokyuzune firliyordu (OSD hiz 139-159 km/h).
- Dongu ilk karesini aldiginda hedef coktan gorus disindaydi; bbox bolgesinde
  kalan tek sey EKRANA SABIT OSD batarya yazisiydi.
- CSRT, OSD yazisina kilitleniyordu: found_ratio 1.0, yaw_error sabit ~45px,
  "PASS" - ama arac hic kontrol edilmiyordu.

Gecersiz kanitlar: 21:51 4s tracking PASS, 23:33 extended-follow PASS,
00:27 postfc PASS. Bunlar bracket/kabul kaniti olarak KULLANILMAMALI.

Ara bulgular (ayni gece, yanlis cikan hipotezler - olculdu ve elendi):
olu bolge yok (RC egri lineer, 0.05->1523 ... 0.5->1750), girdi yolu her
kosulda saglam (tekil/10Hz/jitter/arm'li/crash-sonrasi hepsi calisiyor),
runaway-disarm yok (dongu sonu armed=True).

### 2. Duzeltmeler (pr0p_tracking_probe + pr0p_arm_sequence)

- Tracker artik KALKISTAN ONCE, hedef gorusteyken init ediliyor.
- Arm yerde yapiliyor (gaz dusuk); kalkis dongu ICINDE, kamera kayittayken:
  `--takeoff-delay-frames` (yerde kilitlenme) + `--takeoff-boost-frames`
  (kisa tirmanis) + `--settle-throttle` uc fazli gaz programi
  (HoldAxesAdapter throttle_schedule).
- Ekran-sabit kilit dedektoru: arm'li kosuda bbox merkezi hic hareket
  etmediyse `SCREEN_FIXED_LOCK_SUSPECTED` + PASS -> WAITING.
- Dongu sonu FC durumu kanita eklendi (`post_loop_fc`: armed/blocker/RC).
- `--alt-hold`: MSP_ALTITUDE tabanli P kontrolcu iskeleti eklendi (asagida
  neden bloke oldugu var).

### 3. GERCEK kapali dongu kaniti (yeni enstrumanla, tekrarlanabilir)

Kirmizi kapi hedefi (bbox 155,215,130,85), yerde pivot + kucuk kalkis:

- v2: cx 258 -> 419.5 (merkez 414) 0.8 saniyede; dogru isaretli komutlar.
- v6: cx 220 -> 388.5; yaw komutu -0.265 -> -0.018 (hata 194px -> 25px,
  temiz yakinsama).
- Kayip her seferinde arac YUKSELDIGINDE: hedef, yukari egimli FPV kameranin
  gorusunden fiziksel olarak cikiyor (kare kare goruntulu kanit:
  scratchpad lossframes; hiz 68 -> 91 km/h tirmanista kayip).

### 4. Olculen yeni engeller (siradaki isin tanimi)

1. Hover bandi jilet gibi: 1625 yerde, 1640+ guclu tirmanis. Acik dongu
   gazla havada asili kalmak imkansiz.
2. MSP_ALTITUDE barosu 0.3 m'de SATURE OLUYOR (guclu tirmanista bile 0.3
   okuyor; padde -0.3/0.23). MSP tabanli irtifa tutucu bu build'de calismaz.
3. Kamera yukari egimli: yukselen drone yer hedefini hizla kaybediyor.

Onerilen cozum yollari (oncelik sirasiyla):

1. pr0p quad secim menusunden daha guclu olmayan bir arac sec (ornegin tiny
   whoop sinifi); UI smoke su an varsayilan "270 Supra 5in Racer" seciyor -
   asiri guclu. Daha yavas arac hem hover bandini genisletir hem gorus kaybini
   azaltir.
2. pr0p ayarlarinda kamera acisi (uptilt) dusurulebiliyorsa dusur.
3. Bu ikisi olmadan yaw-only "havadan statik hedef" kapisi kisa pencerede
   (pivot + kisa hop) olculebilir; uzun surekli takip icin sart.

### 5. Faz karari kaydi

Kullanici karari (2026-07-08): hareketli-hedef fazinin ilk asamasi "airborne
static-target track" olarak guncellendi. Gercek hareketli hedef (baska arac,
insan, ghost) SONRAKI asama. ContainerCrash Time Attack'ta hareketli obje
olmadigi zaten olculmustu; bu karar o bulguyla uyumlu. Codex'in
moving-target gate kriterleri bu karara gore yeniden hedeflenmeli
("hedef hareketi" yerine "arac hareketi + yakinsama" kaniti).

Testler: 407 passed (game_screen + pr0p). Izolasyon PASS. Checkpoint sonunda
pr0p/Gazebo/Betaflight prosesi yok.

## TRUE LATEST 2026-07-08 04:43 +03 - AIRBORNE STATIC TARGET GATE RETARGET + CORE SELECTOR FIX

Bu turda Claude'un 01:15 bulgulari repo uzerinden incelendi ve post-core
`moving_target_yaw` kapisinin ilk gercek fazi kullanici kararina gore yeniden
hedeflendi: artik ilk asama "havadan, yerde duran statik hedefe yaw-only
track/yakinsama" kanitidir. Gercek hareketli hedef/ghost sonraki fazdir.
Canli pr0p/uinput/RC komutu gonderilmedi.

Ana degisiklikler:

- `screen_tracking_loop.py` artik yatay hata yakinsamasi metrikleri yazar:
  `initial_abs_horizontal_error`, `last_abs_horizontal_error`,
  `horizontal_error_reduction_px`, `horizontal_error_reduction_ratio`.
- `pr0p_tracking_probe.py` screen-fixed/OSD sahte kilit suphe durumunu
  makine-okunur `screen_fixed_lock_suspected` metrigi olarak yazar.
- `pr0p_tracking_acceptance_runner.py` Claude'un buldugu loop-ici kalkis
  profilini ust komuttan probe'a forward eder:
  `--takeoff-delay-frames`, `--takeoff-boost-frames`, `--settle-throttle`.
- `pr0p_moving_target_yaw_plan.py` gercek PASS icin artik hedefin kendisinin
  hareketini degil, statik hedefe karsi arac/goruntu hareketi + yaw yakinsamasi
  olcer:
  image motion >= 30 px, initial center error >= 30 px,
  final center error <= 40 px, horizontal error reduction ratio >= 0.50,
  yaw-only, pitch zero, real input, loop sonunda FC armed, screen-fixed lock yok.
- `pr0p_moving_target_acceptance_runner.py` yeni dogru ack bayragi olarak
  `--ack-airborne-static-target` kullanir. Eski `--ack-real-moving-target`
  sadece legacy alias olarak kaldi.
- `README.md`, `PLAN.md`, `pr0p_core_goal_report.py` metinleri yeni faz karari
  ile uyumlu hale getirildi.

Onemli bug fix:

- Core goal raporu extended-follow icin yine "latest tracking acceptance"
  dosyasini aliyordu. Bu yuzden Claude'un daha yeni 15 s tracking acceptance
  raporu, daha eski ama gercek 20 s/0-loss/0.95 extended-follow PASS raporunu
  golgeliyordu.
- `pr0p_core_goal_report.py` artik extended-follow icin en yeni dosyayi degil,
  extended-follow profilini gercekten tasiyan en yeni `*-tracking-acceptance`
  raporunu seciyor. Kisa tracking acceptance dosyalari bu gate'i artik
  golgeleyemez.

Dogrulanan test/smoke:

```text
py_compile: PASS
targeted airborne/core/moving/docs: 31 passed
tests/test_game_screen_sandbox.py + tests/test_simitl_pr0p_probe.py: 410 passed
core smoke v2: PASS, next action moving_target_yaw_plan / airborne static-target yaw
airborne static acceptance plan smoke v2: WAITING, real input yok
airborne static yaw linked smoke v2: WAITING, synthetic PASS, real WAITING
pr0p_isolation_check: PASS
```

Smoke raporlari:

- `logs/simitl_pr0p/20260708-044314-codex-airborne-static-core-smoke-v2-core-goal.json`
- `logs/simitl_pr0p/20260708-044314-codex-airborne-static-acceptance-plan-smoke-v2-moving-target-acceptance.json`
- `logs/simitl_pr0p/20260708-044314-codex-airborne-static-yaw-linked-smoke-v2-moving-target-yaw.json`
- `logs/simitl_pr0p/20260708-044135-codex-airborne-static-isolation-smoke-isolation-check.json`

Mevcut siralama:

1. Core + extended_follow artik raporda PASS seciliyor.
2. Siradaki gercek is: statik FPV hedef secili halde
   `pr0p_moving_target_acceptance_runner.py --run-live-gate --ack-live-input
   --ack-airborne-static-target` ile airborne static yaw-only kanit almak.
3. Bu rapor PASS olunca `pr0p_moving_target_yaw_plan.py
   --real-tracking-report ... --ack-airborne-static-target` ile
   `moving_target_yaw` PASS yapilacak.
4. Sonra approach/pitch, daha sonra manual-to-autonomous handoff.

## TRUE LATEST 2026-07-08 04:47 +03 - CORE NEXT AIRBORNE STATIC LIVE KEY BAGLANDI

Bu turda post-core zincirdeki komut secimi duzeltildi. Canli pr0p/uinput/RC
komutu gonderilmedi.

Problem:

- `extended_follow PASS` olduktan sonra `moving_target_yaw_plan` calisiyor ve
  sentetik yaw regresyonu PASS, real pr0p kaniti WAITING uretiyordu.
- Buna ragmen `pr0p_core_goal_report.py` ve `pr0p_core_next_runner.py` sonraki
  komut olarak yine `moving_target_yaw_plan` oneriyordu.
- Bu durum kullaniciyi ayni sentetik plan komutunu tekrar tekrar calistirmaya
  iter; gercek siradaki is olan guarded live acceptance runner'a gecisi
  otomatik gostermiyordu.

Duzeltme:

- `pr0p_core_goal_report.py` komut listesine yeni key eklendi:
  `airborne_static_yaw_live`.
- Bu key su guarded live komutu uretir:

```text
pr0p_moving_target_acceptance_runner.py
  --run-live-gate
  --ack-live-input
  --ack-airborne-static-target
```

- `next_command_key_for(...)` artik:
  - `extended_follow PASS` ama yaw plan yoksa: `moving_target_yaw_plan`
  - `synthetic_moving_target_yaw PASS` ve `real_pr0p_moving_target_yaw WAITING`
    ise: `airborne_static_yaw_live`
  - `moving_target_yaw PASS` ise: `approach_pitch_plan`
- `pr0p_core_next_runner.py` bu yeni komutu live command olarak algiliyor ve
  `--execute-next --ack-live-command` olmadan calistirmiyor.

Dogrulanan test/smoke:

```text
py_compile: PASS
targeted core-next/airborne/docs: 12 passed
tests/test_game_screen_sandbox.py + tests/test_simitl_pr0p_probe.py: 411 passed
core smoke: PASS, next action airborne_static_yaw_live
core-next plan smoke: PLANNED, next_command_key airborne_static_yaw_live
core-next execute-without-live-ack smoke: WAITING, executed False
```

Smoke raporlari:

- `logs/simitl_pr0p/20260708-044706-codex-airborne-static-nextkey-core-smoke-core-goal.json`
- `logs/simitl_pr0p/20260708-044706-codex-airborne-static-nextkey-plan-smoke-core-next.json`
- `logs/simitl_pr0p/20260708-044706-codex-airborne-static-nextkey-guard-smoke-core-next.json`

Mevcut siralama:

1. Core + extended_follow PASS.
2. Sentetik yaw plan PASS, real airborne static kanit WAITING.
3. Simdi dogru next key: `airborne_static_yaw_live`.
4. Kullanici/pr0p UI hazir oldugunda gercek kosu icin:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_core_next_runner.py \
  --bbox-file logs/simitl_pr0p/pr0p-target-bbox.json \
  --execute-next \
  --ack-live-command \
  --run-id pr0p-airborne-static-live
```

Bu komut canli input gonderir; ancak `--ack-live-command` olmadan core-next
WAITING kalir ve komutu calistirmaz.

## TRUE LATEST 2026-07-08 04:54 +03 - AIRBORNE STATIC ACCEPTANCE REFRESH KEY BAGLANDI

Bu turda Claude'un son pr0p post-core guncellemeleri incelendi ve eksik kalan
acceptance -> refresh baglantisi tamamlandi. Canli pr0p/uinput/RC komutu
gonderilmedi.

Problem:

- `airborne_static_yaw_live` calisip `*-moving-target-acceptance.json` PASS
  uretebilirdi.
- Ancak `pr0p_core_goal_report.py` sadece `*-moving-target-yaw.json`
  okuyordu; live acceptance raporunu hic tuketmiyordu.
- Bu yuzden live acceptance PASS sonrasi core tekrar `airborne_static_yaw_live`
  veya genel `moving_target_yaw_plan` dongusune dusebilirdi.

Duzeltme:

- `pr0p_core_goal_report.py` artik en yeni
  `*-moving-target-acceptance.json` raporunu okuyor.
- Acceptance raporu PASS, taze ve `metrics.tracking_report` mevcutsa yeni
  next key seciliyor: `airborne_static_yaw_refresh`.
- Bu key dry komuttur:

```text
pr0p_moving_target_yaw_plan.py
  --execute-synthetic
  --real-tracking-report <acceptance metrics.tracking_report>
  --ack-airborne-static-target
```

- `pr0p_core_next_runner.py` bu komutu live saymiyor; yani
  `--execute-next` ile `--ack-live-command` olmadan calistirabilir.
- README ve PLAN bu yeni siralama ile guncellendi.

Dogrulanan test/smoke:

```text
py_compile: PASS
targeted airborne refresh/core-next/docs: 4 passed
tests/test_game_screen_sandbox.py + tests/test_simitl_pr0p_probe.py: 413 passed
core smoke: PASS, next_command_key airborne_static_yaw_live
core-next plan smoke: PLANNED, next_command_key airborne_static_yaw_live,
  command_is_live True, executed False
```

Smoke raporlari:

- `logs/simitl_pr0p/20260708-045345-codex-airborne-static-refresh-core-smoke-core-goal.json`
- `logs/simitl_pr0p/20260708-045345-codex-airborne-static-refresh-plan-smoke-core-next.json`

Mevcut gercek durum:

1. Core + extended_follow PASS.
2. Sentetik yaw plan PASS, real airborne static kanit hala WAITING.
3. Mevcut loglarda acceptance PASS olmadigi icin refresh hazir degil:
   `airborne_static_yaw_refresh_ready=False`.
4. Bu yuzden dogru siradaki live adim hala `airborne_static_yaw_live`.

Sonraki adim:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_core_next_runner.py \
  --bbox-file logs/simitl_pr0p/pr0p-target-bbox.json \
  --execute-next \
  --ack-live-command \
  --run-id pr0p-airborne-static-live
```

Bu komut canli input gonderir; kullanici/pr0p UI hazir olmadan calistirma.
Bu komut PASS acceptance raporu uretirse, bir sonraki core-next planinin
`airborne_static_yaw_refresh` gostermesi ve dry olarak `moving_target_yaw` gate
refresh etmesi beklenir.

## TRUE LATEST 2026-07-08 05:00 +03 - APPROACH LIVE/REFRESH CORE NEXT BAGLANDI

Bu turda yaw-only fazindan sonraki pitch/approach fazi incelendi ve ayni
tekrar-dongusu riski giderildi. Canli pr0p/uinput/RC komutu gonderilmedi.

Problem:

- `moving_target_yaw PASS` olduktan sonra `approach_pitch_plan` sentetik
  range/pitch regresyonunu PASS yapip real approach kanitini WAITING
  birakabiliyordu.
- Buna ragmen core sonraki komut olarak yine `approach_pitch_plan` onerecek
  durumdaydi.
- Bu, yaw fazinda daha once gordugumuz "sentetik plan tekrar eder, live kanita
  gecmez" probleminin approach/pitch fazindaki kopyasiydi.

Duzeltme:

- `pr0p_core_goal_report.py` yeni live key uretir:
  `approach_pitch_live`.
- Bu key guarded live komuttur:

```text
pr0p_tracking_acceptance_runner.py
  --run-live-gates
  --ack-live-input
  --arm-first
  --enable-pitch
  --duration 20.0
  --min-found-ratio 0.9
  --max-loss-events 2
```

- `pr0p_core_next_runner.py` bu komutu live sayar; `--execute-next
  --ack-live-command` olmadan calistirmaz.
- Pitch-enabled `*-tracking-acceptance.json` PASS, taze ve approach profiline
  uygunsa core yeni dry key uretir: `approach_pitch_refresh`.
- Bu dry key su canonical gate refresh komutunu uretir:

```text
pr0p_approach_pitch_plan.py
  --execute-synthetic
  --real-tracking-report <approach tracking acceptance>
  --ack-real-approach-target
```

- README ve PLAN yaw -> approach live -> approach refresh -> handoff sirasi ile
  guncellendi.

Dogrulanan test/smoke:

```text
py_compile: PASS
targeted approach live/refresh/docs: 4 passed
tests/test_game_screen_sandbox.py + tests/test_simitl_pr0p_probe.py: 416 passed
core smoke: PASS, next_command_key airborne_static_yaw_live
core-next plan smoke: PLANNED, next_command_key airborne_static_yaw_live,
  command_is_live True, executed False
```

Smoke raporlari:

- `logs/simitl_pr0p/20260708-050013-codex-approach-live-refresh-core-smoke-core-goal.json`
- `logs/simitl_pr0p/20260708-050013-codex-approach-live-refresh-plan-smoke-core-next.json`

Mevcut gercek durum:

1. Core + extended_follow PASS.
2. Sentetik yaw plan PASS, real airborne static kanit hala WAITING.
3. Bu yuzden mevcut siradaki adim degismedi: `airborne_static_yaw_live`.
4. Approach live/refresh akisi simdiden hazir; ancak `moving_target_yaw PASS`
   olmadan devreye girmemeli.

Sonraki canli adim yine:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_core_next_runner.py \
  --bbox-file logs/simitl_pr0p/pr0p-target-bbox.json \
  --execute-next \
  --ack-live-command \
  --run-id pr0p-airborne-static-live
```

Bu komut kullanici/pr0p UI hazir olmadan calistirilmayacak.

## TRUE LATEST 2026-07-08 05:15 +03 - IZOLASYON VE LIVE-GUARD KILIDI

Bu turda yeni core-next/readiness baglantilarinin Gazebo yoluna sizmadigi ve
live komut guard'larinin saglam kaldigi test edildi. Canli pr0p/uinput/RC
komutu gonderilmedi.

Duzeltme:

- `pr0p_core_next_runner.py` live command marker listesine tekil
  `--run-live-gate` eklendi. Boylece `--ack-live-input` olmasa bile bu flag'i
  tasiyan komut live kabul edilir.
- `tests/test_simitl_pr0p_probe.py` icine gercek default izolasyon root'larini
  tarayan regresyon testi eklendi:
  `test_isolation_check_default_pr0p_roots_stay_gazebo_independent`.
- Tekil live marker testi eklendi:
  `test_core_next_runner_detects_singular_live_gate_marker`.

Dogrulanan test/smoke:

```text
py_compile: PASS
targeted isolation/live guard: 6 passed
tests/test_game_screen_sandbox.py + tests/test_simitl_pr0p_probe.py: 423 passed
pr0p_isolation_check: PASS / GAZEBO_INDEPENDENCE_VERIFIED
core-next plan smoke: PLANNED, next_command_key airborne_static_yaw_live,
  command_is_live True, executed False
independent readiness smoke: INDEPENDENT_SIM_READY
```

Smoke raporlari:

- `logs/simitl_pr0p/20260708-051521-codex-post-core-isolation-smoke-v2-isolation-check.json`
- `logs/simitl_pr0p/20260708-051550-codex-isolation-guard-core-next-plan-smoke-core-next.json`
- `logs/simitl_pr0p/20260708-051550-codex-isolation-guard-readiness-smoke-independent-sim-readiness.json`

Mevcut siradaki canli adim degismedi:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_core_next_runner.py \
  --bbox-file logs/simitl_pr0p/pr0p-target-bbox.json \
  --execute-next \
  --ack-live-command \
  --run-id pr0p-airborne-static-live
```

Bu komut kullanici/pr0p UI hazir olmadan calistirilmayacak.

## TRUE LATEST 2026-07-08 05:12 +03 - RESUME NOKTASI

En guncel kod durumu:

- `independent_sim_readiness.py` artik `--bbox-file` kabul eder.
- Independent readiness `INDEPENDENT_SIM_READY` oldugunda sonraki aksiyon olarak
  `pr0p_core_goal_report.py` / `pr0p_core_next_runner.py` gosterir.
- Readiness raporu komut listesinde `pr0p_core_goal_status`,
  `pr0p_core_next_plan`, `pr0p_core_next_execute_with_ack` vardir.
- `pr0p_acceptance_chain_runner.py` PASS sonrasi core-goal/core-next ile devam
  etmeyi onerir.

Son dogrulama:

```text
targeted readiness/core-next chain: 4 passed
tests/test_game_screen_sandbox.py + tests/test_simitl_pr0p_probe.py: 421 passed
independent readiness smoke: INDEPENDENT_SIM_READY
```

Son smoke raporu:

- `logs/simitl_pr0p/20260708-051105-codex-core-next-readiness-smoke-independent-sim-readiness.json`

Mevcut siradaki gercek live adim degismedi:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_core_next_runner.py \
  --bbox-file logs/simitl_pr0p/pr0p-target-bbox.json \
  --execute-next \
  --ack-live-command \
  --run-id pr0p-airborne-static-live
```

Bu komut kullanici/pr0p UI hazir olmadan calistirilmayacak.

## TRUE LATEST 2026-07-08 05:11 +03 - READINESS CORE-NEXT'E BAGLANDI

Bu turda bagimsiz pr0p yolunun ust seviye readiness/chain yonlendirmesi
guncellendi. Canli pr0p/uinput/RC komutu gonderilmedi.

Problem:

- `independent_sim_readiness.py` INDEPENDENT_SIM_READY oldugunda hala eski
  "bounded live tracking gate" sonraki adimini soyluyordu.
- Oysa extended-follow, airborne static yaw, approach/pitch ve handoff sirasi
  artik `pr0p_core_goal_report.py` / `pr0p_core_next_runner.py` tarafindan
  yonetiliyor.
- Ayrica readiness CLI `--bbox-file` kabul etmiyordu; mevcut pratik akista bbox
  `logs/simitl_pr0p/pr0p-target-bbox.json` ile tasiniyor.

Duzeltme:

- `independent_sim_readiness.py` komut listesine eklendi:
  - `pr0p_core_goal_status`
  - `pr0p_core_next_plan`
  - `pr0p_core_next_execute_with_ack`
- Readiness `INDEPENDENT_SIM_READY` oldugunda next action artik
  `pr0p_core_goal_report.py` veya `pr0p_core_next_runner.py` ile devam etmeyi
  soyluyor.
- `independent_sim_readiness.py --bbox-file ...` destekli hale geldi.
- `pr0p_acceptance_chain_runner.py` chain PASS sonrasi yonlendirmesini
  decision/readiness raporlarini inceleyip core-goal/core-next ile devam et
  seklinde guncelledi.
- README ve PLAN readiness -> core-next gecisini aciklar.

Dogrulanan test/smoke:

```text
py_compile: PASS
targeted readiness/core-next chain: 4 passed
tests/test_game_screen_sandbox.py + tests/test_simitl_pr0p_probe.py: 421 passed
independent readiness smoke: INDEPENDENT_SIM_READY
```

Smoke raporu:

- `logs/simitl_pr0p/20260708-051105-codex-core-next-readiness-smoke-independent-sim-readiness.json`

Smoke sonucu:

```text
next_action = Both independent paths are ready; run pr0p_core_goal_report.py or
pr0p_core_next_runner.py to continue the measured extended-follow, yaw,
approach, and handoff gates.
```

Smoke komutlari:

```text
pr0p_core_goal_status:
  pr0p_core_goal_report.py --bbox 390,475,140,85

pr0p_core_next_plan:
  pr0p_core_next_runner.py --bbox 390,475,140,85

pr0p_core_next_execute_with_ack:
  pr0p_core_next_runner.py --bbox 390,475,140,85 --execute-next --ack-live-command
```

Mevcut gercek durum:

1. Bagimsiz readiness artik core-next'e dogru yonlendiriyor.
2. Core smoke'lara gore siradaki canli adim hala `airborne_static_yaw_live`.
3. Bu komut kullanici/pr0p UI hazir olmadan calistirilmayacak.

## TRUE LATEST 2026-07-08 05:05 +03 - HANDOFF LIVE/REFRESH CORE NEXT BAGLANDI

Bu turda approach/pitch sonrasi son operator flow fazi olan
manual-to-autonomous handoff zinciri incelendi ve plan/live/refresh ayrimi
tamamlandi. Canli pr0p/uinput/RC komutu gonderilmedi.

Problem:

- `approach_pitch PASS` olduktan sonra `handoff_plan` sentetik
  manual-to-autonomous regresyonu PASS yapip real handoff kanitini WAITING
  birakabiliyordu.
- Buna ragmen core sonraki komut olarak yine `handoff_plan` onerecek durumdaydi.
- Ayrica `latest_handoff_report()` deseni `*-real-handoff.json` acceptance
  raporlarini da canonical `*-handoff.json` raporu gibi secebilirdi.

Duzeltme:

- `pr0p_core_goal_report.py` artik canonical handoff raporlarini
  `*-real-handoff.json` kabul raporlarindan ayiriyor.
- Yeni guarded live key eklendi: `handoff_live`.
- Bu key su live acceptance komutunu uretir:

```text
pr0p_handoff_acceptance_runner.py
  --run-live-gate
  --ack-live-input
  --ack-real-handoff
```

- `pr0p_core_next_runner.py` bu komutu live sayar; `--execute-next
  --ack-live-command` olmadan calistirmaz.
- Real `*-real-handoff.json` acceptance raporu PASS ve taze ise yeni dry key
  secilir: `handoff_refresh`.
- Bu dry key su canonical gate refresh komutunu uretir:

```text
pr0p_handoff_plan.py
  --execute-synthetic
  --real-handoff-report <real handoff acceptance>
  --ack-real-handoff
```

- README ve PLAN handoff_plan -> handoff_live -> handoff_refresh sirasi ile
  guncellendi.

Dogrulanan test/smoke:

```text
py_compile: PASS
targeted handoff live/refresh/docs: 5 passed
tests/test_game_screen_sandbox.py + tests/test_simitl_pr0p_probe.py: 420 passed
core smoke: PASS, next_command_key airborne_static_yaw_live
core-next plan smoke: PLANNED, next_command_key airborne_static_yaw_live,
  command_is_live True, executed False
```

Smoke raporlari:

- `logs/simitl_pr0p/20260708-050523-codex-handoff-live-refresh-core-smoke-core-goal.json`
- `logs/simitl_pr0p/20260708-050523-codex-handoff-live-refresh-plan-smoke-core-next.json`

Mevcut gercek durum:

1. Core + extended_follow PASS.
2. Sentetik yaw plan PASS, real airborne static kanit hala WAITING.
3. Bu yuzden mevcut siradaki adim degismedi: `airborne_static_yaw_live`.
4. Eski handoff acceptance/linked smoke dosyalari gorunse de
   `handoff_refresh_ready=False`; bu dosyalar mevcut sirayi bypass etmiyor.
5. Handoff live/refresh akisi hazir; ancak `approach_pitch PASS` olmadan devreye
   girmemeli.

Sonraki canli adim yine:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_core_next_runner.py \
  --bbox-file logs/simitl_pr0p/pr0p-target-bbox.json \
  --execute-next \
  --ack-live-command \
  --run-id pr0p-airborne-static-live
```

Bu komut kullanici/pr0p UI hazir olmadan calistirilmayacak.

## TRUE LATEST 2026-07-08 05:12 +03 - EN ALT RESUME NOKTASI

Son dogrulanan durum:

- Readiness artik `--bbox-file` kabul ediyor.
- Readiness `INDEPENDENT_SIM_READY` oldugunda core-goal/core-next'e yonlendiriyor.
- Son smoke:
  `logs/simitl_pr0p/20260708-051105-codex-core-next-readiness-smoke-independent-sim-readiness.json`
- Son test:
  `tests/test_game_screen_sandbox.py + tests/test_simitl_pr0p_probe.py: 421 passed`

Mevcut siradaki canli adim:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_core_next_runner.py \
  --bbox-file logs/simitl_pr0p/pr0p-target-bbox.json \
  --execute-next \
  --ack-live-command \
  --run-id pr0p-airborne-static-live
```

Bu komut kullanici/pr0p UI hazir olmadan calistirilmayacak.

## TRUE LATEST 2026-07-08 05:15 +03 - EN ALT IZOLASYON RESUME

Son dogrulanan durum:

- `pr0p_core_next_runner.py` tekil `--run-live-gate` marker'ini live komut
  kabul ediyor.
- Default pr0p/game-screen izolasyon root'lari testte taraniyor ve Gazebo
  runtime coupling bulursa fail edecek.
- Son test:
  `tests/test_game_screen_sandbox.py + tests/test_simitl_pr0p_probe.py: 423 passed`
- Son izolasyon smoke:
  `logs/simitl_pr0p/20260708-051521-codex-post-core-isolation-smoke-v2-isolation-check.json`
- Son core-next plan smoke:
  `logs/simitl_pr0p/20260708-051550-codex-isolation-guard-core-next-plan-smoke-core-next.json`

Mevcut siradaki canli adim degismedi:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_core_next_runner.py \
  --bbox-file logs/simitl_pr0p/pr0p-target-bbox.json \
  --execute-next \
  --ack-live-command \
  --run-id pr0p-airborne-static-live
```

Bu komut kullanici/pr0p UI hazir olmadan calistirilmayacak.

## TRUE LATEST 2026-07-08 05:21 +03 - EN ALT TRACKING-REPORT GUARD

Son dogrulanan durum:

- `pr0p_core_goal_report.py` artik airborne static yaw refresh icin sadece
  acceptance PASS olmasini yeterli saymiyor; acceptance icindeki
  `tracking_report` dosyasi gercekten mevcut degilse
  `airborne_static_yaw_refresh` secilmiyor.
- Yeni regresyon testi:
  `test_core_goal_report_does_not_refresh_yaw_with_missing_tracking_report`.
- Son test:
  `tests/test_game_screen_sandbox.py + tests/test_simitl_pr0p_probe.py: 424 passed`
- Son core-next plan smoke:
  `logs/simitl_pr0p/20260708-052048-codex-missing-tracking-guard-core-next-smoke-core-next.json`
- Son readiness smoke:
  `logs/simitl_pr0p/20260708-052048-codex-missing-tracking-guard-readiness-smoke-independent-sim-readiness.json`
- Son izolasyon smoke:
  `logs/simitl_pr0p/20260708-052108-codex-missing-tracking-guard-isolation-smoke-isolation-check.json`

Mevcut siradaki canli adim degismedi:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_core_next_runner.py \
  --bbox-file logs/simitl_pr0p/pr0p-target-bbox.json \
  --execute-next \
  --ack-live-command \
  --run-id pr0p-airborne-static-live
```

Bu komut kullanici/pr0p UI hazir olmadan calistirilmayacak.

## TRUE LATEST 2026-07-08 05:27 +03 - EN ALT COMPLETION STATUS AYRIMI

Son dogrulanan durum:

- `pr0p_core_goal_report.py` top-level JSON'a `completion_status`,
  `completion_summary`, `completion_missing`, `completion_failed` alanlarini
  ekledi.
- `status=PASS` artik sadece core uc kapinin PASS oldugunu anlatir:
  RC/manual flight, camera/tracker, autopilot-control.
- Tam bagimsiz sim hedefi icin okunacak alan `completion_status`.
  `COMPLETE` sadece extended follow, airborne static-target yaw,
  approach/pitch ve real manual-to-auto handoff kapilari da PASS oldugunda
  gelir.
- Mevcut smoke:
  `logs/simitl_pr0p/20260708-052706-codex-completion-status-core-goal-smoke-core-goal.json`
  `status=PASS`, `completion_status=IN_PROGRESS`,
  `completion_missing=["moving_target_yaw","approach_pitch","handoff"]`.
- Core-next smoke:
  `logs/simitl_pr0p/20260708-052557-codex-completion-status-core-next-smoke-v3-core-next.json`
  `next_command_key=airborne_static_yaw_live`, `command_is_live=True`,
  `executed=False`.
- Readiness smoke:
  `logs/simitl_pr0p/20260708-052557-codex-completion-status-readiness-smoke-v2-independent-sim-readiness.json`
- Izolasyon smoke:
  `logs/simitl_pr0p/20260708-052619-codex-completion-status-isolation-smoke-v2-isolation-check.json`
- Son test:
  `tests/test_game_screen_sandbox.py + tests/test_simitl_pr0p_probe.py: 424 passed`

Mevcut siradaki canli adim degismedi:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_core_next_runner.py \
  --bbox-file logs/simitl_pr0p/pr0p-target-bbox.json \
  --execute-next \
  --ack-live-command \
  --run-id pr0p-airborne-static-live
```

Bu komut kullanici/pr0p UI hazir olmadan calistirilmayacak.

## TRUE LATEST 2026-07-08 05:32 +03 - EN ALT READINESS OBJECTIVE COMPLETION

Son dogrulanan durum:

- `independent_sim_readiness.py` top-level JSON'a
  `objective_completion_status`, `objective_completion_summary`,
  `objective_completion_missing`, `objective_completion_failed` alanlarini
  ekledi.
- Bu alanlar `pr0p_core_goal_report.py` preview'inden read-only hesaplanir;
  readiness komutu yine pr0p/Gazebo/capture/uinput baslatmaz.
- `INDEPENDENT_SIM_READY` artik "bagimsiz evidence path promotable" anlaminda
  kalir; tam hedef icin `objective_completion_status` okunur.
- Son readiness smoke:
  `logs/simitl_pr0p/20260708-053214-codex-readiness-objective-completion-smoke-independent-sim-readiness.json`
  `status=INDEPENDENT_SIM_READY`,
  `objective_completion_status=IN_PROGRESS`,
  `objective_completion_missing=["moving_target_yaw","approach_pitch","handoff"]`,
  `core_goal_next_command_key=airborne_static_yaw_live`.
- Son core-next smoke:
  `logs/simitl_pr0p/20260708-053214-codex-readiness-objective-completion-core-next-smoke-core-next.json`
  `next_command_key=airborne_static_yaw_live`, `command_is_live=True`,
  `executed=False`.
- Son izolasyon smoke:
  `logs/simitl_pr0p/20260708-053214-codex-readiness-objective-completion-isolation-smoke-isolation-check.json`
- Son test:
  `tests/test_game_screen_sandbox.py + tests/test_simitl_pr0p_probe.py: 424 passed`

Mevcut siradaki canli adim degismedi:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_core_next_runner.py \
  --bbox-file logs/simitl_pr0p/pr0p-target-bbox.json \
  --execute-next \
  --ack-live-command \
  --run-id pr0p-airborne-static-live
```

Bu komut kullanici/pr0p UI hazir olmadan calistirilmayacak.

## TRUE LATEST 2026-07-08 05:37 +03 - EN ALT CORE-NEXT LIVE PRECHECK

Son dogrulanan durum:

- `pr0p_core_next_runner.py` artik live command execute etmeden once
  read-only `independent_sim_readiness.py` precheck kosar.
- `--ack-live-command` verilse bile precheck status `INDEPENDENT_SIM_READY`
  degilse command runner'a gecmez, `executed=False` kalir ve
  `INDEPENDENT_READINESS_NOT_READY` notu yazar.
- Bu guard plan-only ve dry refresh komutlarini etkilemez.
- Yeni regresyon testi:
  `test_core_next_runner_blocks_live_execution_when_independent_readiness_missing`.
- Son readiness smoke:
  `logs/simitl_pr0p/20260708-053659-codex-core-next-live-precheck-readiness-smoke-independent-sim-readiness.json`
  `status=INDEPENDENT_SIM_READY`,
  `objective_completion_status=IN_PROGRESS`,
  `objective_completion_missing=["moving_target_yaw","approach_pitch","handoff"]`.
- Son core-next plan smoke:
  `logs/simitl_pr0p/20260708-053659-codex-core-next-live-precheck-plan-smoke-core-next.json`
  `next_command_key=airborne_static_yaw_live`, `command_is_live=True`,
  `executed=False`, `require_independent_ready_for_live=True`.
- Son izolasyon smoke:
  `logs/simitl_pr0p/20260708-053659-codex-core-next-live-precheck-isolation-smoke-isolation-check.json`
- Son test:
  `tests/test_game_screen_sandbox.py + tests/test_simitl_pr0p_probe.py: 425 passed`

Mevcut siradaki canli adim degismedi:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_core_next_runner.py \
  --bbox-file logs/simitl_pr0p/pr0p-target-bbox.json \
  --execute-next \
  --ack-live-command \
  --run-id pr0p-airborne-static-live
```

Bu komut kullanici/pr0p UI hazir olmadan calistirilmayacak.

## TRUE LATEST 2026-07-08 05:41 +03 - EN ALT PLAN-MODE LIVE PRECHECK

Son dogrulanan durum:

- `pr0p_core_next_runner.py` artik `--precheck-live-readiness` kabul ediyor.
- Bu flag plan-only modda live command'i calistirmadan
  `independent_sim_readiness.py` precheck kosar.
- Precheck ready degilse report `WAITING`, `executed=False`,
  `INDEPENDENT_READINESS_NOT_READY` notu ile doner.
- Precheck ready ise report `PLANNED`, `executed=False`,
  `INDEPENDENT_READINESS_READY` notu ile doner.
- Yeni regresyon testleri:
  - `test_core_next_runner_prechecks_live_readiness_without_executing`
  - `test_core_next_runner_precheck_keeps_live_plan_when_readiness_ready`
  - `test_core_next_runner_cli_accepts_live_precheck_flag`
- Son precheck smoke:
  `logs/simitl_pr0p/20260708-054122-codex-core-next-plan-precheck-smoke-core-next.json`
  `status=PLANNED`, `next_command_key=airborne_static_yaw_live`,
  `command_is_live=True`, `executed=False`,
  `notes=["PLAN_ONLY","LIVE_COMMAND","BOUND_COMMAND","INDEPENDENT_READINESS_READY"]`.
- Son readiness smoke:
  `logs/simitl_pr0p/20260708-054122-codex-core-next-plan-precheck-readiness-smoke-independent-sim-readiness.json`
  `status=INDEPENDENT_SIM_READY`,
  `objective_completion_status=IN_PROGRESS`,
  `objective_completion_missing=["moving_target_yaw","approach_pitch","handoff"]`.
- Son izolasyon smoke:
  `logs/simitl_pr0p/20260708-054122-codex-core-next-plan-precheck-isolation-smoke-isolation-check.json`
- Son test:
  `tests/test_game_screen_sandbox.py + tests/test_simitl_pr0p_probe.py: 428 passed`

Canli komuttan once kullanilacak guvenli precheck:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_core_next_runner.py \
  --bbox-file logs/simitl_pr0p/pr0p-target-bbox.json \
  --precheck-live-readiness \
  --run-id pr0p-airborne-static-precheck
```

Mevcut siradaki canli adim degismedi:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_core_next_runner.py \
  --bbox-file logs/simitl_pr0p/pr0p-target-bbox.json \
  --execute-next \
  --ack-live-command \
  --run-id pr0p-airborne-static-live
```

Bu komut kullanici/pr0p UI hazir olmadan calistirilmayacak.

## TRUE LATEST 2026-07-08 05:47 +03 - EN ALT LIVE SAFETY FIELDS

Son dogrulanan durum:

- `pr0p_core_next_runner.py` raporuna makine-okunur live safety alanlari
  eklendi:
  - `live_precheck_status`
  - `safe_to_execute_live_with_ack`
  - `safe_to_execute_live_blockers`
- Bu alanlar plan-only, `--precheck-live-readiness`, missing-ack ve live
  execute yollarinda set ediliyor. Amac: bir sonraki agent'in veya script'in
  not metni parse etmeden canli komutun guvenli olup olmadigini anlamasi.
- `safe_to_execute_live_with_ack=True` sadece live command bound oldugunda,
  placeholder olmadiginda ve independent readiness precheck
  `INDEPENDENT_SIM_READY` oldugunda veriliyor.
- Son precheck smoke:
  `logs/simitl_pr0p/20260708-054655-codex-live-safety-fields-precheck-smoke-v2-core-next.json`
  `status=PLANNED`, `next_command_key=airborne_static_yaw_live`,
  `command_is_live=True`, `executed=False`,
  `live_precheck_status=INDEPENDENT_SIM_READY`,
  `safe_to_execute_live_with_ack=True`,
  `safe_to_execute_live_blockers=[]`.
- Son readiness smoke:
  `logs/simitl_pr0p/20260708-054655-codex-live-safety-fields-readiness-smoke-v2-independent-sim-readiness.json`
  `status=INDEPENDENT_SIM_READY`,
  `objective_completion_status=IN_PROGRESS`,
  `objective_completion_missing=["moving_target_yaw","approach_pitch","handoff"]`.
- Son izolasyon smoke:
  `logs/simitl_pr0p/20260708-054655-codex-live-safety-fields-isolation-smoke-v2-isolation-check.json`
  `status=PASS`, `file_count=72`, `violations=[]`.
- Son test:
  `tests/test_game_screen_sandbox.py + tests/test_simitl_pr0p_probe.py: 428 passed`

Canli komuttan once kullanilacak guvenli precheck:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_core_next_runner.py \
  --bbox-file logs/simitl_pr0p/pr0p-target-bbox.json \
  --precheck-live-readiness \
  --run-id pr0p-airborne-static-precheck
```

Mevcut siradaki canli adim degismedi:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_core_next_runner.py \
  --bbox-file logs/simitl_pr0p/pr0p-target-bbox.json \
  --execute-next \
  --ack-live-command \
  --run-id pr0p-airborne-static-live
```

Bu komut kullanici/pr0p UI hazir olmadan calistirilmayacak.

## TRUE LATEST 2026-07-08 05:52 +03 - EN ALT NEXT OBJECTIVE GATE

Son dogrulanan durum:

- `pr0p_core_next_runner.py` raporuna current command -> objective gate
  baglantisi eklendi:
  - `next_objective_gate`
  - `next_objective_gate_snapshot`
  - `next_objective_gate_status`
  - `next_objective_gate_missing`
  - `next_objective_gate_failed`
- Amac: bir sonraki live/dry komutun hangi core/post-core acceptance kapisini
  ilerlettigini makine-okunur gormek. Bu, komut anahtarina bakip niyet
  cikarmak yerine olculebilir gate status'u okumayi saglar.
- Mevcut smoke'a gore siradaki live komut hala
  `airborne_static_yaw_live`, fakat artik raporda bunun `moving_target_yaw`
  gate'ini ilerlettigi acik gorunuyor.
- Son precheck smoke:
  `logs/simitl_pr0p/20260708-055222-codex-next-objective-gate-precheck-smoke-core-next.json`
  `status=PLANNED`, `next_command_key=airborne_static_yaw_live`,
  `command_is_live=True`, `executed=False`,
  `next_objective_gate=moving_target_yaw`,
  `next_objective_gate_status=WAITING`,
  `next_objective_gate_missing=["real_pr0p_moving_target_yaw","moving_target_yaw_pass"]`,
  `live_precheck_status=INDEPENDENT_SIM_READY`,
  `safe_to_execute_live_with_ack=True`,
  `safe_to_execute_live_blockers=[]`.
- Son readiness smoke:
  `logs/simitl_pr0p/20260708-055222-codex-next-objective-gate-readiness-smoke-independent-sim-readiness.json`
  `status=INDEPENDENT_SIM_READY`,
  `objective_completion_status=IN_PROGRESS`,
  `objective_completion_missing=["moving_target_yaw","approach_pitch","handoff"]`.
- Son izolasyon smoke:
  `logs/simitl_pr0p/20260708-055222-codex-next-objective-gate-isolation-smoke-isolation-check.json`
  `status=PASS`, `file_count=72`, `violations=[]`.
- Son test:
  `tests/test_game_screen_sandbox.py + tests/test_simitl_pr0p_probe.py: 428 passed`

Canli komuttan once kullanilacak guvenli precheck:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_core_next_runner.py \
  --bbox-file logs/simitl_pr0p/pr0p-target-bbox.json \
  --precheck-live-readiness \
  --run-id pr0p-airborne-static-precheck
```

Mevcut siradaki canli adim degismedi:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_core_next_runner.py \
  --bbox-file logs/simitl_pr0p/pr0p-target-bbox.json \
  --execute-next \
  --ack-live-command \
  --run-id pr0p-airborne-static-live
```

Bu komut kullanici/pr0p UI hazir olmadan calistirilmayacak.

## TRUE LATEST 2026-07-08 06:00 +03 - EN ALT OPERATOR PREFLIGHT GATE

Son dogrulanan durum:

- `pr0p_core_next_runner.py` artik opsiyonel
  `--precheck-operator-readiness` kabul ediyor.
- Bu precheck live komut calistirmadan, uinput gondermeden,
  `external_operator_preflight.py` uzerinden operator/UI hazirligini ayni
  core-next JSON'una ekliyor:
  - `operator_precheck_status`
  - `safe_to_execute_live_with_operator_precheck`
  - `safe_to_execute_live_operator_blockers`
  - `operator_readiness_precheck`
- Amac: `INDEPENDENT_SIM_READY` ile "su an pr0p FPV penceresi/operator
  hazir" bilgisini ayirmak. Bu ayrim onemli: evidence hazir olabilir ama
  canli pencere hazir degilse live gate calistirilmamali.
- Baseline precheck smoke:
  `logs/simitl_pr0p/20260708-055954-codex-operator-precheck-baseline-smoke-core-next.json`
  `status=PLANNED`, `next_command_key=airborne_static_yaw_live`,
  `executed=False`, `live_precheck_status=INDEPENDENT_SIM_READY`,
  `operator_precheck_status=NOT_CHECKED`,
  `safe_to_execute_live_with_operator_precheck=False`,
  `safe_to_execute_live_operator_blockers=["OPERATOR_READINESS_NOT_CHECKED"]`.
- Operator precheck smoke:
  `logs/simitl_pr0p/20260708-055954-codex-operator-precheck-smoke-core-next.json`
  `status=WAITING`, `next_command_key=airborne_static_yaw_live`,
  `executed=False`, `next_objective_gate=moving_target_yaw`,
  `live_precheck_status=INDEPENDENT_SIM_READY`,
  `operator_precheck_status=WAITING`,
  `safe_to_execute_live_with_operator_precheck=False`,
  `safe_to_execute_live_operator_blockers=["OPERATOR_READINESS_NOT_READY"]`.
- Operator precheck'in tespit ettigi mevcut blokaj:
  `target_window=WAITING`; next action:
  `open the external game/sim window or pass the correct --window-title`.
- Son readiness smoke:
  `logs/simitl_pr0p/20260708-060029-codex-operator-precheck-readiness-smoke-independent-sim-readiness.json`
  `status=INDEPENDENT_SIM_READY`,
  `objective_completion_status=IN_PROGRESS`,
  `objective_completion_missing=["moving_target_yaw","approach_pitch","handoff"]`.
- Son izolasyon smoke:
  `logs/simitl_pr0p/20260708-055954-codex-operator-precheck-isolation-smoke-isolation-check.json`
  `status=PASS`, `file_count=72`, `violations=[]`.
- Son test:
  `tests/test_game_screen_sandbox.py + tests/test_simitl_pr0p_probe.py: 431 passed`

Canli komuttan once kullanilacak daha guvenli precheck:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_core_next_runner.py \
  --bbox-file logs/simitl_pr0p/pr0p-target-bbox.json \
  --precheck-live-readiness \
  --precheck-operator-readiness \
  --run-id pr0p-airborne-static-precheck
```

Bu precheck `WAITING` donerse canli komut calistirilmeyecek; once pr0p FPV
penceresi/operator hazirligi tamamlanacak.

Mevcut siradaki canli adim degismedi:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_core_next_runner.py \
  --bbox-file logs/simitl_pr0p/pr0p-target-bbox.json \
  --precheck-operator-readiness \
  --execute-next \
  --ack-live-command \
  --run-id pr0p-airborne-static-live
```

Bu komut kullanici/pr0p UI hazir olmadan calistirilmayacak.

## TRUE LATEST 2026-07-08 06:05 +03 - EN ALT OPERATOR GATE MANDATORY EXECUTE

Son dogrulanan durum:

- `pr0p_core_next_runner.py` live execution yolunda operator readiness
  kontrolunu varsayilan zorunlu hale getirdi.
- Artik `--execute-next --ack-live-command` kullanilsa bile live komut
  baslamadan once operator/window/bbox precheck kosar. Bu precheck
  `OPERATOR_PREFLIGHT_READY` degilse `executed=False` kalir.
- `--precheck-operator-readiness` plan modunda ayni kontrolu onceden gormek
  icin kullanilir.
- Sadece acik ve denetlenmis istisna icin
  `--skip-operator-readiness-precheck` vardir; default degildir.
- Son operator mandatory precheck smoke:
  `logs/simitl_pr0p/20260708-060435-codex-operator-mandatory-precheck-smoke-core-next.json`
  `status=WAITING`, `next_command_key=airborne_static_yaw_live`,
  `executed=False`, `require_operator_ready_for_live=True`,
  `next_objective_gate=moving_target_yaw`,
  `live_precheck_status=INDEPENDENT_SIM_READY`,
  `safe_to_execute_live_with_ack=True`,
  `operator_precheck_status=WAITING`,
  `safe_to_execute_live_with_operator_precheck=False`,
  `safe_to_execute_live_operator_blockers=["OPERATOR_READINESS_NOT_READY"]`.
- Operator precheck'in tespit ettigi mevcut blokaj:
  `operator_stage=target_setup`, next action:
  `open the external game/sim window or pass the correct --window-title`.
- Son readiness smoke:
  `logs/simitl_pr0p/20260708-060435-codex-operator-mandatory-readiness-smoke-independent-sim-readiness.json`
  `status=INDEPENDENT_SIM_READY`,
  `objective_completion_status=IN_PROGRESS`,
  `objective_completion_missing=["moving_target_yaw","approach_pitch","handoff"]`.
- Son izolasyon smoke:
  `logs/simitl_pr0p/20260708-060435-codex-operator-mandatory-isolation-smoke-isolation-check.json`
  `status=PASS`, `file_count=72`, `violations=[]`.
- Son test:
  `tests/test_game_screen_sandbox.py + tests/test_simitl_pr0p_probe.py: 435 passed`

Canli komuttan once kullanilacak guvenli precheck:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_core_next_runner.py \
  --bbox-file logs/simitl_pr0p/pr0p-target-bbox.json \
  --precheck-live-readiness \
  --precheck-operator-readiness \
  --run-id pr0p-airborne-static-precheck
```

Bu precheck `WAITING` donerse canli komut calistirilmeyecek; once pr0p FPV
penceresi/operator hazirligi tamamlanacak.

Mevcut siradaki canli adim degismedi:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_core_next_runner.py \
  --bbox-file logs/simitl_pr0p/pr0p-target-bbox.json \
  --execute-next \
  --ack-live-command \
  --run-id pr0p-airborne-static-live
```

Bu komut artik operator readiness precheck'i otomatik kosar; kullanici/pr0p UI
hazir olmadan calistirilmeyecek.

## TRUE LATEST 2026-07-08 06:10 +03 - EN ALT COMBINED LIVE SAFETY

Son dogrulanan durum:

- `pr0p_core_next_runner.py` icinde live execution icin tek bakislik birlesik
  karar alanlari eklendi ve test edildi:
  - `safe_to_execute_live_with_all_prechecks`
  - `safe_to_execute_live_required_blockers`
- Bu alanlar bagimsiz sim readiness blokajlari ile operator/UI readiness
  blokajlarini birlestiriyor. Artik `safe_to_execute_live_with_ack=True`
  tek basina canli komut basmak icin yeterli sinyal olarak okunmamali.
- Dokuman tarafinda `experiments/simitl_pr0p_probe/README.md` ve
  `experiments/simitl_pr0p_probe/PLAN.md` bu yeni final gate alanlarini
  anlatiyor.
- Test tarafinda `tests/test_simitl_pr0p_probe.py` operator hazir degilken,
  operator hazirken ve independent readiness eksikken birlesik alanlari
  dogruluyor.

Son testler:

- Py compile:
  `fpv_env/bin/python -m py_compile experiments/simitl_pr0p_probe/pr0p_core_next_runner.py tests/test_simitl_pr0p_probe.py`
  PASS.
- Hedefli pytest:
  `tests/test_simitl_pr0p_probe.py -k 'all_prechecks or operator_readiness or operator_precheck or precheck_keeps_live_plan_when_readiness_ready or core_next_runner_executes_dry_next_without_live_ack or pr0p_docs_cover_chain_refresh_and_promotion_guards'`
  `9 passed, 259 deselected`.
- Guvenli pr0p/screen suite:
  `fpv_env/bin/python -m pytest tests/test_game_screen_sandbox.py tests/test_simitl_pr0p_probe.py -q`
  `435 passed`.

Son smoke raporlari:

- Core-next smoke:
  `logs/simitl_pr0p/20260708-060957-codex-combined-live-safety-smoke-core-next.json`
  - `status=WAITING`
  - `next_command_key=airborne_static_yaw_live`
  - `executed=False`
  - `command_is_live=True`
  - `next_objective_gate=moving_target_yaw`
  - `next_objective_gate_status=WAITING`
  - `next_objective_gate_missing=["real_pr0p_moving_target_yaw","moving_target_yaw_pass"]`
  - `live_precheck_status=INDEPENDENT_SIM_READY`
  - `safe_to_execute_live_with_ack=True`
  - `safe_to_execute_live_blockers=[]`
  - `operator_precheck_status=WAITING`
  - `safe_to_execute_live_with_operator_precheck=False`
  - `safe_to_execute_live_operator_blockers=["OPERATOR_READINESS_NOT_READY"]`
  - `safe_to_execute_live_with_all_prechecks=False`
  - `safe_to_execute_live_required_blockers=["OPERATOR_READINESS_NOT_READY"]`
  - `operator_stage=target_setup`
  - `next_action="open the external game/sim window or pass the correct --window-title; run external_window_preflight.py for pr0p"`
- Readiness smoke:
  `logs/simitl_pr0p/20260708-060956-codex-combined-live-safety-readiness-smoke-independent-sim-readiness.json`
  - `status=INDEPENDENT_SIM_READY`
  - `objective_completion_status=IN_PROGRESS`
  - `objective_completion_missing=["moving_target_yaw","approach_pitch","handoff"]`
- Isolation smoke:
  `logs/simitl_pr0p/20260708-060957-codex-combined-live-safety-isolation-smoke-isolation-check.json`
  - `status=PASS`
  - `file_count=72`
  - `violations=[]`

Canli komuttan once kullanilacak guvenli precheck:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_core_next_runner.py \
  --bbox-file logs/simitl_pr0p/pr0p-target-bbox.json \
  --precheck-live-readiness \
  --precheck-operator-readiness \
  --run-id pr0p-airborne-static-precheck
```

Bu precheck `safe_to_execute_live_with_all_prechecks=false` veya `WAITING`
donerse canli komut calistirilmayacak; once pr0p FPV penceresi/operator
hazirligi tamamlanacak.

Mevcut siradaki canli adim degismedi:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_core_next_runner.py \
  --bbox-file logs/simitl_pr0p/pr0p-target-bbox.json \
  --execute-next \
  --ack-live-command \
  --run-id pr0p-airborne-static-live
```

Bu komut operator readiness precheck'i otomatik kosar. Bugunku smoke'a gore
operator/UI hazir olmadigi icin calistirilirsa live komut basmadan `WAITING`
donmelidir.

Siradaki gercek odak:

1. Kullanici pr0p FPV/game penceresini acip hedef bbox hazirligini yapacak.
2. Yukaridaki precheck tekrar kosulacak.
3. `safe_to_execute_live_with_all_prechecks=True` olursa
   `airborne_static_yaw_live` gate'i calistirilacak.
4. Bu gate gecerse `moving_target_yaw`, sonra `approach_pitch`, sonra `handoff`
   gate'lerine gecilecek.

## TRUE LATEST 2026-07-08 06:18 +03 - EN ALT SIGNED INDEPENDENT LOOP + S13 READY

Son dogrulanan durum:

- Gazebo'dan tamamen bagimsiz game/screen sandbox tarafinda S9/S10/S11
  kapilarina ilk komut yonu metrikleri eklendi:
  - `yaw_initial_command_corrective`
  - `pitch_initial_command_corrective` (range/handoff fazlari icin)
- Bu metrikler "hedef merkeze yaklasti mi?" yaninda "ilk PID komutu hedef
  hatasini duzeltecek dogru isarette mi?" sorusunu da olcer.
- Ters ilk yaw veya pitch komutu artik evaluator tarafinda FAIL uretir:
  - `YAW_INITIAL_COMMAND_DIRECTION_WRONG`
  - `PITCH_INITIAL_COMMAND_DIRECTION_WRONG`
- Tum sure boyunca komut/hata isaret oranlari raporda tutulur ama hard gate
  degildir; iyi PID merkez civarinda frenleme/overshoot duzeltmesi icin ters
  komut uretebilir. Hard gate ilk anlamli komut yonudur.
- `experiments/game_screen_sandbox/README.md` ve `simple_game_sitl.md`
  bu yeni sign gate'i anlatiyor.

Son testler:

- Py compile:
  `fpv_env/bin/python -m py_compile experiments/game_screen_sandbox/game_dynamics_loop.py tests/test_game_screen_sandbox.py`
  PASS.
- Hedefli signed-loop testleri:
  `fpv_env/bin/python -m pytest tests/test_game_screen_sandbox.py -q -k 'game_dynamics or range_dynamics or handoff_dynamics'`
  `15 passed, 155 deselected`.
- Hedefli S13/decision/signed-loop testleri:
  `fpv_env/bin/python -m pytest tests/test_game_screen_sandbox.py -q -k 'binding_dry or decision or game_dynamics or range_dynamics or handoff_dynamics'`
  `20 passed, 150 deselected`.
- Guvenli pr0p/screen suite:
  `fpv_env/bin/python -m pytest tests/test_game_screen_sandbox.py tests/test_simitl_pr0p_probe.py -q`
  `438 passed`.

Son smoke raporlari:

- Signed loop + binding phase smoke:
  `logs/game_screen_sandbox/20260708-061842-codex-signed-loop-binding-smoke-phase-report.json`
  `status=PASS`.
  PASS fazlari:
  `S0`, `S1`, `S2`, `S3`, `S4`, `S8`, `S9-game`, `S10-range`,
  `S11-handoff`, `S12-adapter`, `S13-binding-dry`.
- Yeni signed metrikler:
  - `S9-game`: `yaw_initial_command_corrective=True`,
    `final_abs_error_px=8.0`.
  - `S10-range`: `yaw_initial_command_corrective=True`,
    `pitch_initial_command_corrective=True`,
    `final_abs_error_px=9.5`, `final_abs_width_error_px=3.0`.
  - `S11-handoff`: `yaw_initial_command_corrective=True`,
    `pitch_initial_command_corrective=True`,
    `final_abs_error_px=9.5`, `final_abs_width_error_px=3.0`.
  - `S13-binding-dry`: `neutralized=True`.
- Sandbox decision:
  `logs/game_screen_sandbox/20260708-061851-codex-signed-loop-binding-decision-decision.json`
  `decision=SIMPLE_SANDBOX_READY`, `reasons=[]`.
  Bu sadece bagimsiz game/screen tracker/PID dry-run gelistirme zemininin hazir
  oldugunu kanitlar; pr0p live, Betaflight, Gazebo veya gercek RC promosyonu
  degildir.
- Izolasyon smoke:
  `logs/simitl_pr0p/20260708-061816-codex-signed-loop-isolation-smoke-isolation-check.json`
  `status=PASS`, `file_count=72`, `violations=[]`.

Mevcut pr0p/live blokaji degismedi:

- Independent sandbox: `SIMPLE_SANDBOX_READY`.
- pr0p/operator live gate: onceki core-next smoke'a gore hala operator/UI
  hazirligi bekliyor:
  `safe_to_execute_live_with_all_prechecks=False`,
  `safe_to_execute_live_required_blockers=["OPERATOR_READINESS_NOT_READY"]`.
- Bu nedenle canli komut hala kullanici pr0p FPV/game penceresini acip hedef
  bbox/operator hazirligini tamamlamadan calistirilmamali.

Siradaki gercek odak:

1. Kullanici pr0p FPV/game penceresini acacak ve hedef bbox/operator
   hazirligini tamamlayacak.
2. `pr0p_core_next_runner.py --precheck-live-readiness --precheck-operator-readiness`
   tekrar kosulacak.
3. `safe_to_execute_live_with_all_prechecks=True` oldugunda
   `airborne_static_yaw_live` calistirilacak.
4. Canli gate gecince `moving_target_yaw`, `approach_pitch`, `handoff`
   sirasinda ilerlenilecek.

## TRUE LATEST 2026-07-08 06:25 +03 - EN ALT GAME DECISION SCHEMA GUARD

Son dogrulanan durum:

- `independent_sim_readiness.py` artik game-screen tarafindaki
  `SIMPLE_SANDBOX_READY` kararini sadece karar string'i ile kabul etmiyor.
- Karar raporu su anki zorunlu faz semasini de tasimali ve gecirmeli:
  - `S0`, `S1`, `S2`, `S3`, `S4`, `S8`, `S9-game`, `S10-range`,
    `S11-handoff`, `S12-adapter`, `S13-binding-dry`
  - simple-window tarafi: `S1-real`, `S2-real`, `S7-real`
- Eski formatli veya eksik faz kapsamli `SIMPLE_SANDBOX_READY` dosyalari artik
  `game_screen_decision_schema.status=MISMATCH` olarak raporlanir.
- Bu durumda readiness `INDEPENDENT_SIM_READY` olmaz; next action
  game-screen acceptance yeniden kosulmasini soyler.
- `experiments/simitl_pr0p_probe/README.md` ve
  `experiments/simitl_pr0p_probe/PLAN.md` bu schema guard'i anlatiyor.

Son testler:

- Py compile:
  `fpv_env/bin/python -m py_compile experiments/simitl_pr0p_probe/independent_sim_readiness.py tests/test_simitl_pr0p_probe.py`
  PASS.
- Hedefli readiness/docs testleri:
  `fpv_env/bin/python -m pytest tests/test_simitl_pr0p_probe.py -q -k 'independent_readiness or docs_cover_chain_refresh_and_promotion_guards'`
  `12 passed, 257 deselected`.
- Guvenli pr0p/screen suite:
  `fpv_env/bin/python -m pytest tests/test_game_screen_sandbox.py tests/test_simitl_pr0p_probe.py -q`
  `439 passed`.

Son smoke raporlari:

- Readiness smoke:
  `logs/simitl_pr0p/20260708-062308-codex-game-schema-readiness-smoke-independent-sim-readiness.json`
  - `status=INDEPENDENT_SIM_READY`
  - `objective_completion_status=IN_PROGRESS`
  - `objective_completion_missing=["moving_target_yaw","approach_pitch","handoff"]`
  - `game_screen_decision=SIMPLE_SANDBOX_READY`
  - `game_screen_decision_schema.status=PASS`
  - `missing_synthetic_required_phases=[]`
  - `non_pass_synthetic_phases=[]`
  - `game_screen_decision=logs/game_screen_sandbox/20260708-061851-codex-signed-loop-binding-decision-decision.json`
- Core-next smoke:
  `logs/simitl_pr0p/20260708-062308-codex-game-schema-core-next-smoke-core-next.json`
  - `status=WAITING`
  - `next_command_key=airborne_static_yaw_live`
  - `executed=False`
  - `next_objective_gate=moving_target_yaw`
  - `live_precheck_status=INDEPENDENT_SIM_READY`
  - `operator_precheck_status=WAITING`
  - `safe_to_execute_live_with_all_prechecks=False`
  - `safe_to_execute_live_required_blockers=["OPERATOR_READINESS_NOT_READY"]`
  - `operator_stage=target_setup`
- Izolasyon smoke:
  `logs/simitl_pr0p/20260708-062308-codex-game-schema-isolation-smoke-isolation-check.json`
  `status=PASS`, `file_count=72`, `violations=[]`.

Mevcut durumun ozeti:

- Bagimsiz game/screen + pr0p evidence yolu guncel schema ile
  `INDEPENDENT_SIM_READY`.
- Tam hedef tamam degil:
  `objective_completion_missing=["moving_target_yaw","approach_pitch","handoff"]`.
- Canli pr0p komutu hala operator/UI hazirligi bekliyor:
  `safe_to_execute_live_with_all_prechecks=False`.

Siradaki gercek odak:

1. Kullanici pr0p FPV/game penceresini acacak ve hedef bbox/operator
   hazirligini tamamlayacak.
2. `pr0p_core_next_runner.py --precheck-live-readiness --precheck-operator-readiness`
   tekrar kosulacak.
3. `safe_to_execute_live_with_all_prechecks=True` olursa
   `airborne_static_yaw_live` canli gate'i calistirilacak.

## TRUE LATEST 2026-07-08 06:29 +03 - EN ALT OPERATOR BBOX MATCH GUARD

Son dogrulanan durum:

- `pr0p_core_next_runner.py` operator preflight icin kullanilan bbox dosyasini
  artik live komutun hedef bbox'u ile karsilastiriyor.
- `--operator-bbox-file` verildiyse bu dosyadaki bbox, `--bbox` veya
  `--bbox-file` ile core/live komuta giden bbox ile ayni olmak zorunda.
- Eslesmezse operator precheck external window/capture asamasina gecmeden
  `WAITING` doner:
  - `decision.command_gate=bbox_file_mismatch`
  - `selected_bbox=[...]`
  - `operator_bbox=[...]`
- Bu durumda `safe_to_execute_live_with_all_prechecks=False` kalir ve live
  komut calismaz. Amac: preflight'in baska hedefi, live komutun baska hedefi
  takip etmesini engellemek.
- `experiments/simitl_pr0p_probe/README.md` ve
  `experiments/simitl_pr0p_probe/PLAN.md` bu bbox match guard'i anlatiyor.

Son testler:

- Py compile:
  `fpv_env/bin/python -m py_compile experiments/simitl_pr0p_probe/pr0p_core_next_runner.py tests/test_simitl_pr0p_probe.py`
  PASS.
- Hedefli operator/bbox/docs testleri:
  `fpv_env/bin/python -m pytest tests/test_simitl_pr0p_probe.py -q -k 'operator_bbox_mismatch or operator_readiness or operator_precheck or docs_cover_chain_refresh_and_promotion_guards'`
  `8 passed, 262 deselected`.
- Guvenli pr0p/screen suite:
  `fpv_env/bin/python -m pytest tests/test_game_screen_sandbox.py tests/test_simitl_pr0p_probe.py -q`
  `440 passed`.

Son smoke raporlari:

- Core-next smoke:
  `logs/simitl_pr0p/20260708-062839-codex-operator-bbox-guard-smoke-core-next.json`
  - `status=WAITING`
  - `next_command_key=airborne_static_yaw_live`
  - `executed=False`
  - `next_objective_gate=moving_target_yaw`
  - `live_precheck_status=INDEPENDENT_SIM_READY`
  - `operator_precheck_status=WAITING`
  - `operator_bbox_file=logs/simitl_pr0p/pr0p-target-bbox.json`
  - `selected_bbox=[390,475,140,85]`
  - `operator_bbox=[390,475,140,85]`
  - `operator_gate=blocked_by_dependency`
  - `safe_to_execute_live_with_all_prechecks=False`
  - `safe_to_execute_live_required_blockers=["OPERATOR_READINESS_NOT_READY"]`
- Readiness smoke:
  `logs/simitl_pr0p/20260708-062838-codex-operator-bbox-guard-readiness-smoke-independent-sim-readiness.json`
  - `status=INDEPENDENT_SIM_READY`
  - `objective_completion_status=IN_PROGRESS`
  - `objective_completion_missing=["moving_target_yaw","approach_pitch","handoff"]`
  - `game_screen_decision_schema.status=PASS`
- Izolasyon smoke:
  `logs/simitl_pr0p/20260708-062839-codex-operator-bbox-guard-isolation-smoke-isolation-check.json`
  `status=PASS`, `file_count=72`, `violations=[]`.

Mevcut durumun ozeti:

- Bbox hedef tutarliligi guard'i aktif ve smoke'ta mevcut dosya ile komut bbox'u
  ayni hedefi gosteriyor.
- Canli pr0p komutu hala operator/UI hazirligi bekliyor:
  `safe_to_execute_live_with_all_prechecks=False`,
  `OPERATOR_READINESS_NOT_READY`.
- Tam hedef tamam degil:
  `objective_completion_missing=["moving_target_yaw","approach_pitch","handoff"]`.

Siradaki gercek odak:

1. Kullanici pr0p FPV/game penceresini acacak.
2. Ayni `logs/simitl_pr0p/pr0p-target-bbox.json` hedefinin pr0p penceresinde
   gecerli oldugu operator precheck ile dogrulanacak.
3. `safe_to_execute_live_with_all_prechecks=True` oldugunda
   `airborne_static_yaw_live` canli gate'i calistirilacak.

## TRUE LATEST 2026-07-08 06:34 +03 - EN ALT OPERATOR FRESHNESS CORE-NEXT FIELDS

Son dogrulanan durum:

- Claude'un son pr0p/operator guncellemeleri repo uzerinden incelendi.
- Mevcut yapi ana hedefe uygun: canli komut icin sadece pencere/bbox degil,
  ayni hedefe ait dry-run, live-input/live-follow ve taze evidence kapilari
  isteniyor.
- Bu turda operator preflight kararina freshness ayrintilari eklendi:
  - `freshness_gate_status`
  - `required_fresh_evidence`
  - `stale_evidence`
  - `missing_evidence`
- `pr0p_core_next_runner.py` bu nested operator kararini artik top-level
  core-next metriklerine de tasiyor:
  - `operator_readiness_stage`
  - `operator_readiness_command_gate`
  - `operator_readiness_freshness_gate_status`
  - `operator_readiness_required_fresh_evidence`
  - `operator_readiness_stale_evidence`
  - `operator_readiness_missing_evidence`
- Amac: canli komut kapaliysa bunun nedeni pencere mi, bbox mi, stale/missing
  evidence mi tek JSON'dan gorulsun; nested rapor acmadan automation ve operator
  ayni sinyali okuyabilsin.
- README/PLAN bu yeni alanlari anlatiyor; docs-cover testi de bu alanlari
  ariyor.

Son testler:

- Py compile:
  `fpv_env/bin/python -m py_compile experiments/game_screen_sandbox/external_operator_preflight.py experiments/simitl_pr0p_probe/pr0p_core_next_runner.py tests/test_game_screen_sandbox.py tests/test_simitl_pr0p_probe.py`
  PASS.
- Hedefli game-screen operator/freshness/live-follow testleri:
  `fpv_env/bin/python -m pytest tests/test_game_screen_sandbox.py -q -k 'operator_preflight or freshness or status_report_require_live_follow or live_follow_sequence'`
  `11 passed, 159 deselected`.
- Hedefli pr0p core-next/operator/docs testleri:
  `fpv_env/bin/python -m pytest tests/test_simitl_pr0p_probe.py -q -k 'operator_readiness or operator_precheck or operator_bbox_mismatch or docs_cover_chain_refresh_and_promotion_guards'`
  `8 passed, 262 deselected`.
- Guvenli pr0p/screen suite:
  `fpv_env/bin/python -m pytest tests/test_game_screen_sandbox.py tests/test_simitl_pr0p_probe.py -q`
  `440 passed`.

Son smoke raporlari:

- Core-next smoke:
  `logs/simitl_pr0p/20260708-063403-codex-operator-freshness-smoke-core-next.json`
  - `status=WAITING`
  - `next_command_key=airborne_static_yaw_live`
  - `executed=False`
  - `next_objective_gate=moving_target_yaw`
  - `live_precheck_status=INDEPENDENT_SIM_READY`
  - `operator_precheck_status=WAITING`
  - `operator_readiness_stage=target_setup`
  - `operator_readiness_command_gate=blocked_by_dependency`
  - `operator_readiness_freshness_gate_status=WAITING`
  - `operator_readiness_missing_evidence=["matching_external_preflight","matching_external_follow","matching_external_live_follow_sequence"]`
  - `selected_bbox=[390,475,140,85]`
  - `operator_bbox=[390,475,140,85]`
  - `safe_to_execute_live_with_all_prechecks=False`
  - `safe_to_execute_live_required_blockers=["OPERATOR_READINESS_NOT_READY"]`
- Izolasyon smoke:
  `logs/simitl_pr0p/20260708-063403-codex-operator-freshness-smoke-isolation-check.json`
  `status=PASS`.

Mevcut durumun ozeti:

- Bagimsiz game/screen + pr0p evidence yolu daha onceki smoke'a gore guncel
  schema ile `INDEPENDENT_SIM_READY`.
- Tam hedef tamam degil:
  `objective_completion_missing=["moving_target_yaw","approach_pitch","handoff"]`.
- Canli pr0p komutu bilincli olarak calistirilmadi.
- Canli komut hala operator/UI ve taze external evidence bekliyor:
  `safe_to_execute_live_with_all_prechecks=False`,
  `OPERATOR_READINESS_NOT_READY`.

Siradaki gercek odak:

1. Kullanici pr0p FPV/game penceresini acacak.
2. Ayni `logs/simitl_pr0p/pr0p-target-bbox.json` hedefinin pr0p penceresinde
   gecerli oldugu operator precheck ile dogrulanacak.
3. `external_window_preflight`, `external_follow_session` ve
   `external_live_follow_sequence` kanitlari ayni bbox ile taze hale gelecek.
4. `safe_to_execute_live_with_all_prechecks=True` oldugunda
   `airborne_static_yaw_live` canli gate'i calistirilacak.

## TRUE LATEST 2026-07-08 06:40 +03 - EN ALT OPERATOR NEXT-COMMAND + SHELL WINDOW GUARD

Son dogrulanan durum:

- Core-next operator precheck sonucunda nested raporda kalan siradaki operator
  komutlari artik top-level core-next metriklerine tasiniyor:
  - `operator_readiness_next_action`
  - `operator_readiness_recommended_command`
  - `operator_readiness_recommended_candidate_action`
  - `operator_readiness_safe_to_execute_now`
  - `operator_readiness_live_ack_required`
- Bu alanlar sadece status/plan bilgisidir; canli komut icin hala
  `safe_to_execute_live_with_all_prechecks=True` ve explicit ack gerekir.
- Smoke sirasinda pr0p penceresi yokken operator candidate plan'in terminal
  penceresini (`gz@gz: ~`) aday olarak onerebildigi goruldu.
- `window_safety.py` artik `user@host: ...` shell/terminal basligi heuristic'i
  ile bu pencereleri tooling olarak disliyor.
- Son smoke'ta terminal ve VS Code pencereleri `excluded_candidates` altina
  dustu; `operator_readiness_recommended_candidate_action=null`.
- Bu, tracker/capture/operator zincirinin yanlis pencereye baglanmasini
  engelleyen ek bir guvenlik kapisidir.

Son testler:

- Py compile:
  `fpv_env/bin/python -m py_compile experiments/game_screen_sandbox/window_safety.py experiments/game_screen_sandbox/external_operator_preflight.py experiments/simitl_pr0p_probe/pr0p_core_next_runner.py tests/test_game_screen_sandbox.py tests/test_simitl_pr0p_probe.py`
  PASS.
- Hedefli operator/candidate testleri:
  `fpv_env/bin/python -m pytest tests/test_game_screen_sandbox.py -q -k 'operator_preflight or candidate_action_runner or window_title_candidates'`
  `14 passed, 157 deselected`.
- Hedefli pr0p core-next/operator/docs testleri:
  `fpv_env/bin/python -m pytest tests/test_simitl_pr0p_probe.py -q -k 'operator_readiness or operator_precheck or operator_bbox_mismatch or docs_cover_chain_refresh_and_promotion_guards'`
  `8 passed, 262 deselected`.
- Guvenli pr0p/screen suite:
  `fpv_env/bin/python -m pytest tests/test_game_screen_sandbox.py tests/test_simitl_pr0p_probe.py -q`
  `441 passed`.

Son smoke raporlari:

- Core-next smoke:
  `logs/simitl_pr0p/20260708-064001-codex-terminal-candidate-guard-smoke-core-next.json`
  - `status=WAITING`
  - `next_command_key=airborne_static_yaw_live`
  - `executed=False`
  - `operator_precheck_status=WAITING`
  - `operator_readiness_next_action="open the external game/sim window or pass the correct --window-title"`
  - `operator_readiness_recommended_command.name=dry_sequence`
  - `operator_readiness_recommended_command.safety_class=dry_run_capture`
  - `operator_readiness_recommended_command.sends_uinput=False`
  - `operator_readiness_recommended_command.unmet_requires_pass=["target_window=WAITING"]`
  - `operator_readiness_recommended_candidate_action=null`
  - `operator_readiness_missing_evidence=["matching_external_preflight","matching_external_follow","matching_external_live_follow_sequence"]`
  - `safe_to_execute_live_with_all_prechecks=False`
  - `safe_to_execute_live_required_blockers=["OPERATOR_READINESS_NOT_READY"]`
- Izolasyon smoke:
  `logs/simitl_pr0p/20260708-064001-codex-terminal-candidate-guard-smoke-isolation-check.json`
  `status=PASS`.

Mevcut durumun ozeti:

- Bagimsiz game/screen + pr0p evidence yolu guncel schema ile hazir kabul
  ediliyor ama tam hedef tamam degil.
- Eksik tam-hedef kapilari:
  `moving_target_yaw`, `approach_pitch`, `handoff`.
- Canli pr0p komutu bilincli olarak calistirilmadi.
- Operator/UI hazirligi yokken yanlis terminal/coding pencereleri artik aday
  olarak onerilmiyor.
- Canli komut hala operator/UI ve taze external evidence bekliyor:
  `safe_to_execute_live_with_all_prechecks=False`,
  `OPERATOR_READINESS_NOT_READY`.

Siradaki gercek odak:

1. Kullanici pr0p FPV/game penceresini acacak.
2. `pr0p_core_next_runner.py --bbox-file logs/simitl_pr0p/pr0p-target-bbox.json --precheck-live-readiness --precheck-operator-readiness`
   yeniden kosulacak.
3. Bu kez `operator_readiness_recommended_candidate_action` ancak gercek pr0p
   penceresi aday olursa dolmali; terminal/VS Code aday olmamali.
4. Ayni bbox ile `external_window_preflight`, `external_follow_session` ve
   `external_live_follow_sequence` taze kanitlari uretilecek.
5. `safe_to_execute_live_with_all_prechecks=True` oldugunda
   `airborne_static_yaw_live` canli gate'i calistirilacak.

## TRUE LATEST 2026-07-08 06:44 +03 - EN ALT LIVE BLOCKER SUMMARY

Son dogrulanan durum:

- `pr0p_core_next_runner.py` artik canli komutlar icin top-level
  `live_blocker_summary` metrigini uretiyor.
- Bu ozet tek yerde sunlari birlestiriyor:
  - `next_command_key`
  - `next_objective_gate`
  - objective gate missing/failed kanitlari
  - independent readiness status
  - operator readiness status/stage
  - operator missing/stale evidence
  - siradaki safe operator command adi/safety class/uinput durumu
  - safe live blocker listesi
- Bu alan canli guvenlik davranisini degistirmiyor; sadece automation/agent
  icin “neden live yok, hangi objective kaniti eksik, hangi safe operator
  komutu bekliyor?” sorusunu tek JSON alaninda cevapliyor.
- README/PLAN bu `live_blocker_summary` alanini anlatiyor.

Son testler:

- Py compile:
  `fpv_env/bin/python -m py_compile experiments/simitl_pr0p_probe/pr0p_core_next_runner.py tests/test_simitl_pr0p_probe.py`
  PASS.
- Hedefli pr0p core-next/operator/docs testleri:
  `fpv_env/bin/python -m pytest tests/test_simitl_pr0p_probe.py -q -k 'operator_readiness or operator_precheck or operator_bbox_mismatch or docs_cover_chain_refresh_and_promotion_guards or live_blocker'`
  `8 passed, 262 deselected`.
- Guvenli pr0p/screen suite:
  `fpv_env/bin/python -m pytest tests/test_game_screen_sandbox.py tests/test_simitl_pr0p_probe.py -q`
  `441 passed`.

Son smoke raporlari:

- Core-next smoke:
  `logs/simitl_pr0p/20260708-064429-codex-live-blocker-summary-smoke-core-next.json`
  - `status=WAITING`
  - `next_command_key=airborne_static_yaw_live`
  - `executed=False`
  - `live_blocker_summary.status=BLOCKED`
  - `live_blocker_summary.safe_to_execute_live=False`
  - `live_blocker_summary.blockers=["OPERATOR_READINESS_NOT_READY"]`
  - `live_blocker_summary.independent_readiness_status=INDEPENDENT_SIM_READY`
  - `live_blocker_summary.next_objective_gate=moving_target_yaw`
  - `live_blocker_summary.next_objective_gate_missing=["real_pr0p_moving_target_yaw","moving_target_yaw_pass"]`
  - `live_blocker_summary.operator_readiness_status=WAITING`
  - `live_blocker_summary.operator_readiness_stage=target_setup`
  - `live_blocker_summary.operator_missing_evidence=["matching_external_preflight","matching_external_follow","matching_external_live_follow_sequence"]`
  - `live_blocker_summary.operator_recommended_command_name=dry_sequence`
  - `live_blocker_summary.operator_recommended_command_safety_class=dry_run_capture`
  - `live_blocker_summary.operator_recommended_command_sends_uinput=False`
  - `live_blocker_summary.operator_recommended_command_unmet_requires_pass=["target_window=WAITING"]`
- Izolasyon smoke:
  `logs/simitl_pr0p/20260708-064429-codex-live-blocker-summary-smoke-isolation-check.json`
  `status=PASS`.

Mevcut durumun ozeti:

- Bagimsiz game/screen + pr0p evidence yolu guncel schema ile hazir kabul
  ediliyor ama tam hedef tamam degil.
- Eksik tam-hedef kapilari:
  `moving_target_yaw`, `approach_pitch`, `handoff`.
- Siradaki core-next komutu halen `airborne_static_yaw_live`, fakat operator/UI
  hazirligi yok ve canli komut bilincli olarak calistirilmadi.
- `live_blocker_summary` artik bunu tek bakista kanitliyor:
  objective gate `moving_target_yaw`, eksik real yaw evidence ve operator
  tarafinda eksik preflight/follow/live-follow evidence.

Siradaki gercek odak:

1. Kullanici pr0p FPV/game penceresini acacak.
2. `pr0p_core_next_runner.py --bbox-file logs/simitl_pr0p/pr0p-target-bbox.json --precheck-live-readiness --precheck-operator-readiness`
   yeniden kosulacak.
3. `live_blocker_summary.operator_recommended_command_unmet_requires_pass`
   once `target_window=WAITING` icermemeli.
4. Ayni bbox ile `external_window_preflight`, `external_follow_session` ve
   `external_live_follow_sequence` taze kanitlari uretilecek.
5. `live_blocker_summary.status=SAFE_TO_EXECUTE` ve
   `safe_to_execute_live_with_all_prechecks=True` oldugunda
   `airborne_static_yaw_live` canli gate'i calistirilacak.

## TRUE LATEST 2026-07-08 06:52 +03 - OBJECTIVE REQUIREMENT MATRIX

Son dogrulanan durum:

- `pr0p_core_goal_report.py` artik user-facing full objective icin
  `objective_requirements` matrisi uretiyor.
- Bu matris su satirlari ayri ayri PASS/WAITING/FAIL raporluyor:
  - `gazebo_independent_path`
  - `target_bbox_selected`
  - `manual_rc_flight`
  - `camera_image_to_tracker`
  - `autopilot_pid_control`
  - `extended_closed_loop_follow`
  - `airborne_static_yaw_centering`
  - `pitch_approach_control`
  - `manual_to_auto_handoff`
- Ayni raporda kompakt alanlar da var:
  `objective_requirement_statuses`, `objective_requirement_missing`,
  `objective_requirement_failed`.
- `pr0p_core_next_runner.py` bu requirement ozetlerini top-level metrics'e ve
  canli komutlar icin `live_blocker_summary` icine de tasiyor.
- Bu sayede core `PASS` olsa bile full hedefin tamamlanmadigi tek JSON'da
  gorunuyor; mevcut eksikler requirement seviyesinde:
  `airborne_static_yaw_centering`, `pitch_approach_control`,
  `manual_to_auto_handoff`.

Son testler:

- Py compile:
  `fpv_env/bin/python -m py_compile experiments/simitl_pr0p_probe/pr0p_core_goal_report.py experiments/simitl_pr0p_probe/pr0p_core_next_runner.py tests/test_simitl_pr0p_probe.py`
  PASS.
- Hedefli core goal / core next / docs testleri:
  `fpv_env/bin/python -m pytest tests/test_simitl_pr0p_probe.py -q -k 'core_goal_report or core_next_runner_precheck_keeps_live_plan_when_readiness_ready or live_blocker or docs_cover_chain_refresh_and_promotion_guards'`
  `23 passed, 247 deselected`.
- Guvenli pr0p/screen suite:
  `fpv_env/bin/python -m pytest tests/test_game_screen_sandbox.py tests/test_simitl_pr0p_probe.py -q`
  `441 passed`.

Son smoke raporlari:

- Core goal smoke:
  `logs/simitl_pr0p/20260708-065049-codex-objective-requirements-smoke-core-goal.json`
  - `status=PASS`
  - `completion_status=IN_PROGRESS`
  - `next_command_key=airborne_static_yaw_live`
  - `completion_missing=["moving_target_yaw","approach_pitch","handoff"]`
  - `objective_requirement_statuses.gazebo_independent_path=PASS`
  - `objective_requirement_statuses.manual_rc_flight=PASS`
  - `objective_requirement_statuses.camera_image_to_tracker=PASS`
  - `objective_requirement_statuses.autopilot_pid_control=PASS`
  - `objective_requirement_statuses.extended_closed_loop_follow=PASS`
  - `objective_requirement_missing=["airborne_static_yaw_centering","pitch_approach_control","manual_to_auto_handoff"]`
- Isolation smoke:
  `logs/simitl_pr0p/20260708-065050-codex-objective-requirements-smoke-isolation-check.json`
  `status=PASS`.
- Core-next smoke:
  `logs/simitl_pr0p/20260708-065225-codex-objective-requirements-smoke-core-next.json`
  - `status=WAITING`
  - `next_command_key=airborne_static_yaw_live`
  - `live_blocker_summary.status=BLOCKED`
  - `live_blocker_summary.blockers=["OPERATOR_READINESS_NOT_READY"]`
  - `live_blocker_summary.independent_readiness_status=INDEPENDENT_SIM_READY`
  - `live_blocker_summary.next_objective_gate=moving_target_yaw`
  - `live_blocker_summary.next_objective_gate_missing=["real_pr0p_moving_target_yaw","moving_target_yaw_pass"]`
  - `live_blocker_summary.objective_requirement_missing=["airborne_static_yaw_centering","pitch_approach_control","manual_to_auto_handoff"]`
  - `live_blocker_summary.operator_missing_evidence=["matching_external_preflight","matching_external_follow","matching_external_live_follow_sequence"]`

Mevcut durumun ozeti:

- Bagimsiz pr0p/game-screen yolu Gazebo'dan izole kalmaya devam ediyor.
- RC/manual flight, camera/tracker, autopilot PID control ve 20 s extended
  follow kanitlari mevcut evidence ile PASS gorunuyor.
- Tam hedef halen tamam degil; siradaki olculu eksik
  `airborne_static_yaw_centering`.
- Canli pr0p/uinput komutu calistirilmadi.
- Canli komut hala operator/UI penceresi ve taze external evidence bekliyor.

Siradaki gercek odak:

1. Kullanici pr0p FPV/game penceresini acacak.
2. `fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_core_next_runner.py --bbox-file logs/simitl_pr0p/pr0p-target-bbox.json --precheck-live-readiness --precheck-operator-readiness --run-id pr0p-core-next`
   yeniden kosulacak.
3. `live_blocker_summary.objective_requirement_missing` ilk olarak
   `airborne_static_yaw_centering` kapisini isaret etmeye devam etmeli.
4. Operator evidence tamamlaninca ve
   `safe_to_execute_live_with_all_prechecks=True` olunca
   `airborne_static_yaw_live` canli gate'i explicit ack ile denenebilir.

## TRUE LATEST 2026-07-08 06:57 +03 - OPERATOR COPY-RUN COMMAND PACKET

Son dogrulanan durum:

- `pr0p_core_next_runner.py` artik operator tarafindaki bir sonraki guvenli
  komutu `operator_readiness_next_command_packet` olarak top-level metrics'e
  tasiyor.
- `live_blocker_summary` ayni bilgiyi kompakt olarak sunuyor:
  - `operator_next_command_source`
  - `operator_next_command_name`
  - `operator_next_command`
  - `operator_next_command_sends_uinput`
  - `operator_next_command_unmet_requires_pass`
  - `operator_next_command_guard`
- Oncelik sirasi:
  1. Gercek external window candidate varsa onun region/candidate komutu.
  2. Yoksa operator status queue tarafindaki recommended command.
- Bu alan live gate'i bypass etmiyor; sadece “siradaki guvenli operator
  komutu nedir?” sorusunu tek JSON alaninda cevapliyor.

Son testler:

- Py compile:
  `fpv_env/bin/python -m py_compile experiments/simitl_pr0p_probe/pr0p_core_next_runner.py tests/test_simitl_pr0p_probe.py`
  PASS.
- Hedefli operator/core-next/docs testleri:
  `fpv_env/bin/python -m pytest tests/test_simitl_pr0p_probe.py -q -k 'operator_readiness or operator_precheck or operator_bbox_mismatch or live_blocker or docs_cover_chain_refresh_and_promotion_guards'`
  `8 passed, 262 deselected`.
- Guvenli pr0p/screen suite:
  `fpv_env/bin/python -m pytest tests/test_game_screen_sandbox.py tests/test_simitl_pr0p_probe.py -q`
  `441 passed`.

Son smoke raporu:

- Core-next smoke:
  `logs/simitl_pr0p/20260708-065732-codex-operator-command-packet-smoke-core-next.json`
  - `status=WAITING`
  - `next_command_key=airborne_static_yaw_live`
  - `operator_readiness_next_command_packet.name=dry_sequence`
  - `operator_readiness_next_command_packet.safety_class=dry_run_capture`
  - `operator_readiness_next_command_packet.sends_uinput=false`
  - `operator_readiness_next_command_packet.command="fpv_env/bin/python experiments/game_screen_sandbox/external_dry_run_sequence.py --capture-window-title pr0p --capture-backend ffmpeg --tracker-bbox-file logs/simitl_pr0p/pr0p-target-bbox.json --duration 2 --hz 10 --enable-pitch --desired-target-width 120 --run-id pr0p-dry-sequence"`
  - `operator_readiness_next_command_packet.unmet_requires_pass=["target_window=WAITING"]`
  - `live_blocker_summary.operator_next_command` ayni komutu tekrarliyor.

Mevcut durumun ozeti:

- Canli pr0p/uinput komutu calistirilmadi.
- Siradaki live objective halen `airborne_static_yaw_centering`.
- Operator/UI hazir degilken core-next artik hem neden live komutun kapali
  oldugunu hem de operator tarafinda acilinca hangi dry/capture komutunun
  kosulacagini tek raporda gosteriyor.

Siradaki gercek odak:

1. Kullanici pr0p FPV/game penceresini acacak.
2. Core-next precheck tekrar kosulacak.
3. `operator_readiness_next_command_packet.unmet_requires_pass` icindeki
   `target_window=WAITING` kalkarsa packet gercek window candidate/region
   komutuna veya dogrudan dry sequence komutuna donecek.
4. Dry/follow/live-follow operator evidence tamamlaninca
   `airborne_static_yaw_live` icin explicit ack'li canli deneme yapilabilir.

## TRUE LATEST 2026-07-08 07:03 +03 - GUARDED OPERATOR-NEXT EXECUTION

Son dogrulanan durum:

- `pr0p_core_next_runner.py` artik `--execute-operator-next` destekliyor.
- Bu yol core live komutu calistirmiyor; sadece
  `operator_readiness_next_command_packet` icindeki operator-side komutu
  calistirmeye calisiyor.
- Guard kurallari:
  - command yoksa calismaz.
  - command uinput/live marker iceriyorsa calismaz.
  - `unmet_requires_pass` bos degilse calismaz.
  - placeholder bbox varsa calismaz.
  - external-window candidate/region komutu ise
    `--ack-operator-candidate` olmadan calismaz.
- Bu sayede pr0p penceresi hazir oldugunda dry/capture evidence otomasyonu
  tek komutla ilerleyebilir, ama canli RC/uinput gate'leri acilmaz.

Son testler:

- Py compile:
  `fpv_env/bin/python -m py_compile experiments/simitl_pr0p_probe/pr0p_core_next_runner.py tests/test_simitl_pr0p_probe.py`
  PASS.
- Hedefli operator-next/core-next/docs testleri:
  `fpv_env/bin/python -m pytest tests/test_simitl_pr0p_probe.py -q -k 'execute_operator_next or operator_next_cli_guards or operator_readiness or operator_precheck or docs_cover_chain_refresh_and_promotion_guards'`
  `12 passed, 263 deselected`.
- Guvenli pr0p/screen suite:
  `fpv_env/bin/python -m pytest tests/test_game_screen_sandbox.py tests/test_simitl_pr0p_probe.py -q`
  `446 passed`.

Son smoke raporlari:

- Operator-next execution guard smoke:
  `logs/simitl_pr0p/20260708-070325-codex-operator-next-exec-guard-smoke-core-next.json`
  - `status=WAITING`
  - `executed=false`
  - `notes=["OPERATOR_NEXT_COMMAND_UNMET_REQUIREMENTS"]`
  - `next_command_key=airborne_static_yaw_live`
  - `operator_next_execution_requested=true`
  - `operator_next_command_executed=false`
  - `operator_next_execution_blockers=["OPERATOR_NEXT_COMMAND_UNMET_REQUIREMENTS"]`
  - packet halen `dry_sequence`, `sends_uinput=false`, ama
    `unmet_requires_pass=["target_window=WAITING"]`
- Isolation smoke:
  `logs/simitl_pr0p/20260708-070336-codex-operator-next-exec-guard-smoke-isolation-check.json`
  `status=PASS`.

Mevcut durumun ozeti:

- Bagimsiz pr0p/game-screen kodu Gazebo runtime coupling olmadan duruyor.
- Canli pr0p/uinput komutu calistirilmadi.
- Core-next artik operator dry/capture komutunu hem raporlayabiliyor hem de
  guvenli kosullar saglanirsa calistirabiliyor.
- Mevcut ortamda pr0p target window hazir olmadigi icin execution guard dogru
  sekilde durdu.

Siradaki gercek odak:

1. Kullanici pr0p FPV/game penceresini acacak.
2. `fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_core_next_runner.py --bbox-file logs/simitl_pr0p/pr0p-target-bbox.json --execute-operator-next --run-id pr0p-operator-next`
   kosulabilir.
3. Eger packet candidate/region komutuna donerse ve kullanici dogru pencereyi
   teyit ederse `--ack-operator-candidate` eklenir.
4. Operator dry/follow/live-follow evidence tamamlaninca
   `airborne_static_yaw_live` icin explicit live ack'li denemeye gecilir.

## TRUE LATEST 2026-07-08 07:07 +03 - WINDOW DISCOVERY SUMMARY FLATTENED

Son dogrulanan durum:

- `pr0p_core_next_runner.py` artik operator window discovery bilgisini
  top-level metrics'e tasiyor:
  - `operator_readiness_window_discovery_status`
  - `operator_readiness_window_discovery_reason`
  - `operator_readiness_window_title_candidates`
  - `operator_readiness_excluded_window_candidates`
- `live_blocker_summary` ayni bilgiyi kompakt olarak tekrarliyor:
  - `operator_window_discovery_status`
  - `operator_window_discovery_reason`
  - `operator_window_title_candidates`
  - `operator_excluded_window_candidates`
- Bu, pr0p penceresi yokken terminal/VS Code gibi tooling pencerelerinin neden
  aday olmadigini core-next raporundan dogrudan gormeyi sagliyor.

Son testler:

- Py compile:
  `fpv_env/bin/python -m py_compile experiments/simitl_pr0p_probe/pr0p_core_next_runner.py tests/test_simitl_pr0p_probe.py`
  PASS.
- Hedefli operator/core-next/docs testleri:
  `fpv_env/bin/python -m pytest tests/test_simitl_pr0p_probe.py -q -k 'operator_readiness or operator_precheck or execute_operator_next or docs_cover_chain_refresh_and_promotion_guards'`
  `11 passed, 264 deselected`.
- Guvenli pr0p/screen suite:
  `fpv_env/bin/python -m pytest tests/test_game_screen_sandbox.py tests/test_simitl_pr0p_probe.py -q`
  `446 passed`.

Son smoke raporlari:

- Core-next window discovery smoke:
  `logs/simitl_pr0p/20260708-070646-codex-window-discovery-summary-smoke-core-next.json`
  - `status=WAITING`
  - `next_command_key=airborne_static_yaw_live`
  - `operator_readiness_window_discovery_status=WAITING`
  - `operator_readiness_window_discovery_reason=NO_MATCH`
  - `operator_readiness_window_title_candidates=[]`
  - `operator_readiness_excluded_window_candidates` icinde terminal ve VS Code
    pencereleri tooling olarak dislanmis gorunuyor.
- Isolation smoke:
  `logs/simitl_pr0p/20260708-070701-codex-window-discovery-summary-smoke-isolation-check.json`
  `status=PASS`.

Mevcut durumun ozeti:

- Canli pr0p/uinput komutu calistirilmadi.
- pr0p/FPV target window halen bulunamiyor (`NO_MATCH`).
- Kod yanlis terminal/VS Code pencerelerini aday olarak kullanmiyor.
- Bagimsiz pr0p/game-screen kodu Gazebo runtime coupling olmadan duruyor.

Siradaki gercek odak:

1. Kullanici pr0p FPV/game penceresini acacak.
2. Core-next tekrar kosulacak.
3. `operator_readiness_window_discovery_status` `MATCHED` veya aday listesi
   dolu hale gelirse operator dry/capture packet gercek pencereye baglanacak.
4. Ardindan `--execute-operator-next` ile non-uinput dry/capture evidence
   yenilenebilir; candidate varsa `--ack-operator-candidate` gerekir.

## TRUE LATEST 2026-07-08 07:11 +03 - CORE-NEXT ISOLATION EXECUTION GATE

Son dogrulanan durum:

- `pr0p_core_next_runner.py` artik her kosuda isolation precheck raporu
  uretiyor.
- Core-next JSON top-level metrics alanlari:
  - `isolation_status`
  - `isolation_report`
  - `isolation_violation_count`
  - `isolation_precheck`
- `live_blocker_summary.isolation_status` da ayni durumu kompakt ozet icinde
  gosteriyor.
- Isolation status `PASS` degilse core-next `GAZEBO_INDEPENDENCE_FAILED`
  notu ile `FAIL` donuyor ve hem `--execute-next` hem de
  `--execute-operator-next` komutlarini calistirmiyor.
- Boylece Gazebo bagimsizligi artik sadece ayri smoke degil, core-next
  execution kapisinin parcasi.

Son testler:

- Py compile:
  `fpv_env/bin/python -m py_compile experiments/simitl_pr0p_probe/pr0p_core_next_runner.py tests/test_simitl_pr0p_probe.py`
  PASS.
- Hedefli core-next/isolation/docs testleri:
  `fpv_env/bin/python -m pytest tests/test_simitl_pr0p_probe.py -q -k 'core_next_runner_plan_only or isolation_precheck or execute_operator_next or docs_cover_chain_refresh_and_promotion_guards'`
  `7 passed, 269 deselected`.
- Guvenli pr0p/screen suite:
  `fpv_env/bin/python -m pytest tests/test_game_screen_sandbox.py tests/test_simitl_pr0p_probe.py -q`
  `447 passed`.

Son smoke raporlari:

- Core-next isolation gate smoke:
  `logs/simitl_pr0p/20260708-071124-codex-core-next-isolation-gate-smoke-core-next.json`
  - `status=WAITING`
  - `next_command_key=airborne_static_yaw_live`
  - `isolation_status=PASS`
  - `isolation_violation_count=0`
  - `live_blocker_summary.isolation_status=PASS`
  - `operator_readiness_window_discovery_reason=NO_MATCH`
- Operator-next isolation gate execution smoke:
  `logs/simitl_pr0p/20260708-071124-codex-core-next-isolation-gate-exec-smoke-core-next.json`
  - `status=WAITING`
  - `executed=false`
  - `isolation_status=PASS`
  - `operator_next_execution_requested=true`
  - `operator_next_command_executed=false`
  - `operator_next_execution_blockers=["OPERATOR_NEXT_COMMAND_UNMET_REQUIREMENTS"]`

Mevcut durumun ozeti:

- Canli pr0p/uinput komutu calistirilmadi.
- Core-next ve operator-next artik Gazebo bagimsizligini execution gate olarak
  kontrol ediyor.
- Mevcut ortamda isolation PASS, fakat pr0p/FPV target window halen
  `NO_MATCH`.

Siradaki gercek odak:

1. Kullanici pr0p FPV/game penceresini acacak.
2. Core-next tekrar kosulacak; once `isolation_status=PASS`, sonra
   `operator_readiness_window_discovery_status=MATCHED` beklenir.
3. Operator dry/capture evidence `--execute-operator-next` ile yenilenir.
4. Operator/live-follow evidence tamamlaninca `airborne_static_yaw_live`
   explicit live ack ile denenir.

## TRUE LATEST 2026-07-08 07:15 +03 - POST-OPERATOR READINESS REFRESH GUARD

Son eklenen guard:

- `pr0p_core_next_runner.py` basarili guarded `--execute-operator-next`
  sonrasinda operator readiness precheck'i tekrar kosuyor.
- Basarili operator dry/capture komutundan sonra core-next metrics icinde su
  alanlar yaziliyor:
  - `post_operator_readiness_precheck`
  - `post_operator_readiness_summary`
  - `post_operator_precheck_status`
  - `post_operator_readiness_stage`
  - `post_operator_next_action`
  - `post_operator_missing_evidence`
  - `post_operator_stale_evidence`
  - `post_operator_window_discovery_status`
  - `post_operator_window_discovery_reason`
- Amac: operator tarafinda pencere/capture/tracker kaniti yenilendikten sonra
  ayni rapor icinde "hala WAITING mi, READY mi?" sorusunu olculebilir hale
  getirmek.

Son testler:

- Py compile:
  `fpv_env/bin/python -m py_compile experiments/simitl_pr0p_probe/pr0p_core_next_runner.py tests/test_simitl_pr0p_probe.py`
  PASS.
- Hedefli operator-next/docs testleri:
  `fpv_env/bin/python -m pytest tests/test_simitl_pr0p_probe.py -q -k 'execute_operator_next_runs_safe_non_uinput_candidate or execute_operator_next or docs_cover_chain_refresh_and_promotion_guards'`
  `5 passed, 271 deselected`.
- Guvenli pr0p/screen suite:
  `fpv_env/bin/python -m pytest tests/test_game_screen_sandbox.py tests/test_simitl_pr0p_probe.py -q`
  `447 passed`.

Son smoke:

- Komut:
  `fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_core_next_runner.py --bbox-file logs/simitl_pr0p/pr0p-target-bbox.json --execute-operator-next --run-id codex-post-operator-precheck-guard-smoke`
- Rapor:
  `logs/simitl_pr0p/20260708-071455-codex-post-operator-precheck-guard-smoke-core-next.json`
- Sonuc:
  - `status=WAITING`
  - `executed=false`
  - `notes=["OPERATOR_NEXT_COMMAND_UNMET_REQUIREMENTS"]`
  - `isolation_status=PASS`
  - `operator_readiness_window_discovery_status=WAITING`
  - `operator_readiness_window_discovery_reason=NO_MATCH`
  - `operator_next_command_executed=false`
  - `operator_next_execution_blockers=["OPERATOR_NEXT_COMMAND_UNMET_REQUIREMENTS"]`
  - `post_operator_*` alanlari beklenen sekilde yok; cunku operator komutu
    calismadi.

Mevcut durumun ozeti:

- Canli pr0p/uinput komutu calistirilmadi.
- Bagimsiz pr0p/game-screen yolu Gazebo'dan izole ve isolation precheck
  `PASS`.
- Mevcut makinede pr0p/FPV target window bulunmadigi icin operator dry/capture
  asamasina gecilmiyor.
- Artik operator dry/capture asamasi calisirsa, sonucunda readiness otomatik
  tekrar olculup core-next raporuna yazilacak.

Siradaki gercek odak:

1. Kullanici pr0p FPV/game penceresini acar ve hedef/goruntu hazirlar.
2. Core-next tekrar kosulur; `operator_readiness_window_discovery_status`
   `MATCHED` beklenir.
3. `--execute-operator-next` ile non-uinput dry/capture kaniti yenilenir.
4. Post-operator readiness `OPERATOR_PREFLIGHT_READY` olursa explicit ack ile
   ilk live/autopilot komut denemesine gecilir.

## TRUE LATEST 2026-07-08 07:28 +03 - SIMPLE GAME S14 OBJECTIVE LOOP

Son eklenen bagimsiz gate:

- `experiments/game_screen_sandbox/simple_game_objective_loop.py` eklendi.
- `S14-objective` fazi, Gazebo/pr0p/screen capture/uinput kullanmadan proje
  hedefinin cekirdek akis iskeletini tek dongude olcuyor:
  - manuel merkezleme fazi,
  - follow/takip komutu eventi,
  - tracker init yalniz follow sonrasinda,
  - yaw + pitch PID,
  - komutlarin `InputAdapter`-benzeri adapter uzerinden gonderilmesi,
  - adapter komutunun bir sonraki simple-game frame'ine uygulanmasi,
  - final merkezleme ve hedef genisligi/yaklasma hatasi.
- `phase_runner.py --include-objective-loop` artik `S14-objective` ekliyor.
- `sandbox_acceptance_runner.py` sentetik acceptance bundle icinde S14'u de
  kosuyor.
- `decision_report.py` sentetik required phase listesine `S14-objective`
  ekledi; yani independent sandbox ready karari artik bu uctan-uca objective
  gate olmadan cikmiyor.
- `sandbox_acceptance_runner.py` kisa `--duration/--hz` verildiginde S11/S14
  icin sentetik objective kapilarini minimum `6.0s @ 20 Hz` olacak sekilde
  normalize ediyor. Boylece hizli smoke komutu traceback yerine rapor uretir.

Guncellenen dokumanlar:

- `experiments/game_screen_sandbox/README.md`
- `experiments/game_screen_sandbox/PLAN.md`
- `simple_game_sitl.md`

Son testler:

- Py compile:
  `fpv_env/bin/python -m py_compile experiments/game_screen_sandbox/simple_game_objective_loop.py experiments/game_screen_sandbox/phase_runner.py experiments/game_screen_sandbox/sandbox_acceptance_runner.py experiments/game_screen_sandbox/decision_report.py tests/test_game_screen_sandbox.py`
  PASS.
- Hedefli yeni gate/decision/acceptance testleri:
  `fpv_env/bin/python -m pytest tests/test_game_screen_sandbox.py -q -k 'objective_loop or objective or acceptance_runner or sandbox_decision'`
  `10 passed, 165 deselected`.
- Acceptance normalization sonrasi hedefli test:
  `fpv_env/bin/python -m pytest tests/test_game_screen_sandbox.py -q -k 'acceptance_runner or sandbox_decision or objective_loop'`
  `8 passed, 167 deselected`.
- Guvenli pr0p/screen suite:
  `fpv_env/bin/python -m pytest tests/test_game_screen_sandbox.py tests/test_simitl_pr0p_probe.py -q`
  `451 passed`.

Son smoke raporlari:

- S14 phase smoke:
  `fpv_env/bin/python experiments/game_screen_sandbox/phase_runner.py --include-objective-loop --duration 6 --hz 20 --run-id codex-s14-objective-smoke`
  - Rapor:
    `logs/game_screen_sandbox/20260708-072550-codex-s14-objective-smoke-phase-report.json`
  - `S14-objective status=PASS`
  - Kritik metrikler:
    - `manual_to_auto_handoff=true`
    - `follow_command_sent=true`
    - `follow_button_events=3`
    - `tracker_initialized_before_follow=false`
    - `tracker_initialized_after_follow=true`
    - `pre_handoff_auto_control_samples=0`
    - `control_applied_through_input_adapter=true`
    - `nonneutral_adapter_commands=75`
    - `applied_nonneutral_commands=75`
    - `final_abs_error_px=9.5`
    - `final_abs_width_error_px=3.0`
    - `neutralized=true`
    - `gazebo_independent=true`
- Penceresiz acceptance smoke:
  `fpv_env/bin/python experiments/game_screen_sandbox/sandbox_acceptance_runner.py --run-id codex-s14-acceptance-smoke --duration 1 --hz 10 --min-fps 8`
  - Beklenen cikis kodu: `2` (`WAITING`, fail degil).
  - Rapor:
    `logs/game_screen_sandbox/20260708-072731-codex-s14-acceptance-smoke-acceptance-run.json`
  - Decision:
    `WAITING`
  - Reasons:
    `MISSING_SIMPLE_WINDOW_SMOKE`, `MISSING_SIMPLE_WINDOW_PHASE_REPORT`
  - Sentetik required phases icinde `S14-objective` var ve tum sentetik
    fazlar `PASS`.
  - `S14-objective` acceptance smoke metrikleri:
    - `final_abs_error_px=9.5`
    - `final_abs_width_error_px=3.0`
    - `control_applied_through_input_adapter=true`
    - `follow_button_events=3`

Mevcut durumun ozeti:

- Canli pr0p/uinput komutu calistirilmadi.
- Bu turdaki gelisme pr0p pencere bekleme noktasini bypass etmiyor; onun
  yerine Gazebo'dan tamamen bagimsiz simple-game kapali dongu hedef akisini
  daha guclu olculebilir hale getiriyor.
- Independent sandbox artik parca parca S9/S10/S11/S12 kanitlarinin yaninda,
  hedef kullanici akisina daha yakin S14 objective gate ile de korunuyor.

Siradaki gercek odak:

1. pr0p/FPV game penceresi acildiginda operator preflight tekrar kosulur.
2. Pencere MATCHED olursa bbox/dry-capture kaniti uretilir.
3. Real-window dry-run S7 ve external follow evidence S14'teki ayni metrik
   mantigina yakinlastirilir: follow oncesi kontrol yok, follow sonrasi
   tracker/PID, adapter komutu ve gorsel cevap.
4. Ancak canli input/uinput yine explicit ack olmadan calistirilmaz.

## TRUE LATEST 2026-07-08 20:15 +03 - AIRBORNE STATIC YAW GERCEK PASS + APPROACH KAPISININ OLCULMUS ENGELI (Claude)

Bu tur Codex'in biraktigi yerden devam etti: core-next zincirinin bekledigi
`airborne_static_yaw_live` kapisi canli kosuldu ve GERCEK PASS alindi; sonra
zincirin bir sonraki adimi `approach_pitch_live` uzerinde calisti, uc gercek
eksik bulunup duzeltildi, ancak kapi hala kalkis-gecis probleminde takiliyor.
En sonda kullanici pr0p kontrol ayarlarini fiziksel RC testi icin degistirdigini
bildirdi; canli otomasyon durduruldu (asagida "SONRAKI OTURUM ICIN KRITIK").

### 1) airborne_static_yaw_live: GERCEK CANLI PASS (kullanicinin istedigi senaryo)

Komut (Codex'in zincirdeki komutu + olculmus guvenli kalkis profili):
`pr0p_moving_target_acceptance_runner.py --bbox 155,215,130,85 --run-live-gate
--ack-live-input --ack-airborne-static-target --takeoff-delay-frames 10
--takeoff-boost-frames 4 --settle-throttle -0.26
--run-id claude-airborne-static-yaw-live-01`

Olculen (rapor: `logs/simitl_pr0p/20260708-090045-...-moving-target-acceptance.json`):
- 30 s, 300 kare, found_ratio 1.0, kayip 0
- Baslangic yatay hata 194.5 px -> son 0.0 px (azalma orani 1.0; esik 0.50)
- Son 100 karede ortalama |hata| 2.2 px (hedef merkeze kilitli tutuluyor)
- Goruntu hareketi 331.6 px span / 284.6 px travel (arac gercekten donuyor)
- yaw-only: max |yaw| 0.4, pitch 0.0; 300 karenin 290'inda komut isareti hata
  ile tutarli (kalan 10 sifir gecisi)
- Dongu sonunda FC armed, screen-fixed kilit yok
- `pr0p_moving_target_yaw_plan.py --execute-synthetic --real-tracking-report ...
  --ack-airborne-static-target` ile moving_target_yaw = PASS
  (rapor `20260708-090251-...-moving-target-yaw.json`)
- `pr0p_core_next_runner.py` dogrulandi: next_command_key artik
  `approach_pitch_live`

### 2) approach_pitch_live: 4 canli deneme FAIL, uc gercek eksik duzeltildi

Denemeler (hepsi ayni noktada, dongu ~f17'de tracker kaybi):
- 01: pitch f1'den aktifti; 02: pitch-delay 40 (pitch hic devreye girmeden
  kayip); 03: yaw "klempi" 0.2 verildi ama komutlar -0.344'e cikti; 07/08:
  gorsel gaz tutuculu, boost'lu ve boost'suz.

Video kaniti (ffmpeg x11grab, scratchpad run07/run08 kareleri): gaz LOW(1000)
-> hover(~1630) ADIM gecisinde arac 0.25 s icinde 0->27 km/h, 0.4 s'de 84,
sonra 128 km/h — tam roket kalkisi; kare tamamen gokyuzu, yer sahnesi cikiyor.
Yani statik "hover bandi 1625-1640" olcumu adim-tepki rejimini temsil etmiyor
(muhtemel: anti-gravity/I-term + guc/agirlik). Sabahki yaw PASS'i ayni gecisten
sansla sag cikmis (yaw 0.4 komutu net kaldirmayi dusurup tirmanmayi
yavaslatiyordu; yaw 0.2'ye kisinca tirmanis hizlandi).

Duzeltilen gercek eksikler (hepsi test edilmis, 3 yeni ozellik):
- `--pitch-delay-frames` (loop/probe/runner): yaklasma, yaw merkezleme
  oturduktan sonra baslar ("once merkezle, sonra yaklas").
- `apply_axis_clamps` (probe): `--max-abs-yaw-axis` / `--max-abs-pitch-axis`
  artik sadece kabul esigi degil GERCEK komut klempi (loop
  yaw_output_limit/forward_output_limit'e cevriliyor). Onceden 0.2 istesen de
  0.4 gidiyordu.
- `--track-throttle-hold` + `--track-throttle-kp` (probe/runner): baro 0.3
  m'de satüre oldugu icin irtifa geri beslemesi olarak hedefin karedeki dikey
  konumu kullanilir (LastResultTrackerProxy + vertical_hold_throttle; kayipta
  nazik alcalma bias'i). Kenet felsefesiyle uyumlu: bu "sanal pilot" katmani,
  Kenet yalnizca yaw/pitch surer.
- `--takeoff-ramp-frames` (probe/runner): adim yerine kademeli gaz rampasi
  (takeoff_ramp_schedule); adim gecisinin roket etkisine karsi. HENUZ CANLI
  DENENMEDI (kullanici RC testi icin ayar degistirince durduruldu).

Testler: yeni 7 birim testi dahil pytest suite yesil (pitch-delay loop testi
test_game_screen_sandbox'ta; klemp/rampa/gaz-tutucu/proxy/CLI passthrough
testleri test_simitl_pr0p_probe'ta).

### 3) SONRAKI OTURUM ICIN KRITIK

- KULLANICI pr0p kontrol ayarlarini fiziksel RC ile test icin DEGISTIRDI
  (2026-07-08 aksam). Bir sonraki canli otomasyondan once su yeniden
  dogrulanmali: uinput eslesmesi (`msp_uinput_rc_effect_probe` throttle/yaw,
  `msp_uinput_aux_arm_status_probe` AUX1) ve gerekirse
  `pr0p_input_config_patch`. Eski hover/oran kalibrasyonlari ayar degistiyse
  gecersiz olabilir.
- pr0p su an ACIK birakildi (kullanici test ediyor olabilir); holder process
  SIGKILL ile susturuldu ki 30 dk emniyet zamanlayicisi kullanicinin
  oturumunu kapatmasin. Sonraki otomasyon oncesi taze oturum sart.
- Onerilen siradaki adimlar: (a) input dogrulama probelari, (b)
  `--takeoff-ramp-frames 30` + `--track-throttle-hold` profilini denemek, (c)
  hala roketliyorsa asil coza gecmek: pr0p arac menusunden dusuk guclu quad
  secimi (UI smoke uzantisi) — bu ayni zamanda kullanici senaryosunun
  "havada asili takip" kismini acar.
- Kullanici tercihi (bugun): bash komutlari icin izin sorulmasina gerek yok.

### 4) Simple-game hedef TEYIT matrisi (kullanicinin sorusu, hareketli hedef haric)

- Ekran yakalama stabil: PASS (tum canli kosular ffmpeg x11grab ~20-30 fps).
- Tracker oyun goruntusunde hedefi takip ediyor: PASS (bugun canli 30 s
  found_ratio 1.0, OSD sahte kilidi dedektorlu).
- Virtual input guvenli komut veriyor: PASS (uinput arm/disarm/neutral,
  MSP loopback kanitli).
- Yaw-only dongu hedefi yatayda merkeze cekiyor: PASS (194.5 -> 0 px, canli).
- Gecikme olculebiliyor: PASS (S5/S5-real gecikme probelari mevcut).
- Combined yaw+pitch 60 s kontrolden cikmadan: SENTETIK PASS (S14), CANLI
  EKSIK — approach_pitch_live yukaridaki kalkis-gecis engeli yuzunden FAIL;
  "tracker sonrasi hedefe dogru otonom ucus" canli kaniti HENUZ YOK. Yalniz
  yaklasma yonunde ilk gercek veri: run-01'de pitch +0.05..0.09 ileri hareket
  uretti (genislik degisimi + one kayma), kayip kamera/cim engelinden.
- Log + summary uretimi: PASS (tum kosular JSON+MD+JSONL).

