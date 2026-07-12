# ArduPilot + Gazebo Takip Demosu — Kurulum ve Uygulama Plani

Tarih: 2026-07-05. Hedef: yeni, bagimsiz bir projede (dizini KULLANICI acacak)
ArduCopter SITL + Gazebo ile kisa bir demo: dunyada hareket eden 1 arac,
dron kalkar ve araci takip eder, kullanici istedigi an manuel kontrole gecer.

Bu plan bos kagittan yazilmadi: 2026-07-05 gecesi ayni demo bu makinede
prototip olarak fiilen kuruldu ve asagidaki "olculmus tuzaklar" o kurulumda
yasanip cozuldu. Yapi taslarinin cogu makinede HAZIR duruyor (asagida).

## Bu makinede su an hazir olanlar (yeniden kullanilabilir)

- `~/ardupilot` — guncel checkout, SITL board yapilandirilmis.
  **`~/ardupilot/build/sitl/bin/arducopter` derlendi ve mevcut.**
- `~/ardupilot_gazebo` — resmi eklenti klonlandi ve derlendi:
  **`~/ardupilot_gazebo/build/libArduPilotPlugin.so` mevcut** (ayrica
  CameraZoom + Parachute). Iki yerel degisiklikle: (1) rapidjson sistemde
  olmadigi icin `third_party/install` altina header-only kuruldu ve
  CMakeLists'e include yolu eklendi; (2) GStreamer dev paketi olmadigi icin
  `GstCameraPlugin` opsiyonel yapildi (demo icin gereksiz).
- Gazebo Harmonic (`gz sim` 8.11) sistemde kurulu; gz Python bindings
  `/usr/lib/python3/dist-packages` altinda (Python 3.12).
- `~/aeroloop_gazebo/models/kenet_car` — takip edilecek sari arac modeli
  (statik; `set_pose` ile suruluyor — fpv-test'te olculmus yontem).
- `~/fpv-test/tools/sitl_target_mover.py` — kanitli hedef surucu
  (`--world/--model/--trajectory circle` parametreli, gz-transport isinma ve
  tempo tuzaklari icinde cozulmus). Yeni proje bunu dogrudan cagirabilir.
- `~/kenet-ap-demo` — dun geceki PROTOTIP. Icinde calisan taslaklar var:
  `.venv` (pymavlink 2.4.49 + MAVProxy + empy), `worlds/ap_follow_demo.sdf`,
  `follow.py`, `manual_rc.py`, `launch_demo.sh`, `manual_control.sh`.
  Ilk canli deneme su asamaya geldi: Gazebo dunyayi yukledi, SITL baglanti
  bekledi, mover basladi; MAVProxy `future` modulu eksik diye dustu ve test
  orada kesildi. UCUS HENUZ DOGRULANMADI. Kullanici bu dizini isterse
  devralir, isterse siler — plan sifirdan kuruluma gore yazildi.

## Mimari

```text
Gazebo (ap_follow_demo dunyasi, GUI)
  iris_with_ardupilot  <-- ArduPilotPlugin (JSON FDM, UDP 9002, lockstep)
  kenet_car            <-- sitl_target_mover.py (set_pose, daire, 1-2 m/s)
        |
ArduCopter SITL (--model JSON)  <-- TCP 5760 (SERIAL0)
        |
MAVProxy hub: --master tcp:5760
  --out udp:14550  (Configurator/QGC/yedek GCS)
  --out udp:14551  (follow.py)
  --out udp:14552  (manuel kontrol)
        |
follow.py  : GUIDED'da 10 Hz hiz+yaw setpoint (SET_POSITION_TARGET_LOCAL_NED)
manual_rc  : klavye/kumanda -> RC_CHANNELS_OVERRIDE + mod anahtarlama
```

Kontrol sozlesmesi (manuel gecis): follow.py yalniz mod GUIDED iken setpoint
gonderir. Kullanici LOITER/ALT_HOLD'a gecince takip OTOMATIK duraklar (script
molaya girer), GUIDED'a donunce kaldigi yerden surer. RC override GUIDED'da
zaten yok sayilir — iki katman birbirini ezmez.

Cerceve notu: dunya ENU benzeri (x=Dogu, y=Kuzey, z=Yukari; plugin
`gazeboXYZToNED = 180/0/90 derece`). ENU->NED: N=y, E=x, D=-z.
Yaw(NED) = atan2(E, N). Ilk koşuda isaret dogrulamasi yapilmali (kuzeye hiz
ver, Gazebo'da +y'ye gittigini gor).

Konum kaynagi: bu demoda hedef ve dron pozu Gazebo ground-truth'undan
(`/world/<dunya>/pose/info`, gz.transport13). Kamera/tracker BILEREK kapsam
disi — demo ArduPilot kontrol yolunu kanitlar; Kenet algi katmani sonraki
adimda ayni dunyaya baglanir.

## Olculmus tuzaklar (kurulumda yasandi, cozumleri hazir)

1. **waf/empy**: ArduPilot waf `empy` ister ve 4.x ile KIRILIR —
   `pip install "empy==3.3.4" pexpect setuptools` (venv'e) ve derlemeyi
   `<venv-python> ./waf copter` ile yap. Sistem python'una paket kurma.
2. **MAVProxy/future**: pip MAVProxy'si calisirken `No module named future`
   verir — venv'e `future` (ve `lxml`) kur. (Prototipin kesildigi nokta.)
3. **rapidjson**: sistemde dev paketi yok, sudo parolasiz degil —
   header-only lokal kurulum + CMake degiskeni `RapidJSON_INCLUDE_DIRS`
   (buyuk/kucuk harfe dikkat). ~/ardupilot_gazebo'da COZULMUS halde.
4. **GStreamer**: ardupilot_gazebo varsayilan CMakeLists'i zorunlu ister;
   yalniz GstCameraPlugin icin gerekli — opsiyonellestirildi (cozuldu).
5. **Port cakismasi**: ArduPilotPlugin FDM'i UDP **9002** kullanir — Betaflight
   betaloop dunyasiyla AYNI port. Iki sim ayni anda calistirilamaz; launcher
   basta 5760/9002/9003/14550-14552 bos mu kontrol etmeli.
6. **eeprom hijyeni** (fpv-test dersi): arducopter'i her kosuda temiz bir
   `runs/<ts>/` cwd'sinde baslat — `eeprom.bin` ve loglar orada kalir,
   kosular birbirine sizmaz.
7. **pkill kendi kabugunu vurur**: temizlik desenlerini `"[a]rducopter"` gibi
   koseli parantezle yaz ve dogrulama `ps | grep`'ini ayri komutta yap.
8. **Ev konumu**: `iris_runway.sdf` kuresel koordinatlari SITL varsayilan
   evi (CMAC -35.363262, 149.165237) ile eslesir — dunya bu tabandan
   turetilirse `--home` vermeye gerek yok.
9. **gz bindings + venv**: venv Python 3.12 ise
   `sys.path.append("/usr/lib/python3/dist-packages")` ile gz.transport13
   import edilir (fpv-test araclarindaki desen).

## Kurulum adimlari (yeni projede)

Faz 0 — iskelet (15 dk):

```bash
mkdir <proje> && cd <proje>
python3 -m venv .venv
.venv/bin/pip install pymavlink MAVProxy future lxml "empy==3.3.4" pexpect setuptools
mkdir worlds logs runs
```

Yapi taslari hazir oldugundan derleme gerekmez; gerekirse recete:
copter: `cd ~/ardupilot && <venv-python> ./waf copter` — eklenti:
`cd ~/ardupilot_gazebo && cmake -S . -B build -DCMAKE_PREFIX_PATH=$PWD/third_party/install && cmake --build build -j$(nproc)`.

Faz 1 — dunya (30 dk): `~/ardupilot_gazebo/worlds/iris_runway.sdf` kopyasi;
dunya adi `ap_follow_demo`; `iris_with_gimbal` yerine
`iris_with_ardupilot` (include `<name>iris</name>`, poz `0 0 0.195, yaw 90`);
`model://kenet_car` include (orn. `10 15 0`). Kaynak yollari:
`GZ_SIM_SYSTEM_PLUGIN_PATH=~/ardupilot_gazebo/build`,
`GZ_SIM_RESOURCE_PATH=~/ardupilot_gazebo/models:~/ardupilot_gazebo/worlds:~/aeroloop_gazebo/models:<proje>/worlds`.
Kabul: `gz sim` acilir, `gz topic -l`'de `/world/ap_follow_demo/pose/info`
gorunur, iris ve sari arac sahnede.

Faz 2 — SITL + MAVProxy (30 dk):

```bash
(cd runs/<ts> && ~/ardupilot/build/sitl/bin/arducopter --model JSON \
   --defaults ~/ardupilot/Tools/autotest/default_params/copter.parm,~/ardupilot/Tools/autotest/default_params/gazebo-iris.parm -I0)
.venv/bin/mavproxy.py --master tcp:127.0.0.1:5760 \
   --out udp:127.0.0.1:14550 --out udp:127.0.0.1:14551 --out udp:127.0.0.1:14552
```

Kabul: MAVProxy "ArduCopter" heartbeat gorur; `mode guided` + `arm throttle`
+ `takeoff 8` elle calisir (EKF ~15-30 s ister; "PreArm" mesajlarini bekle).
Bu manuel smoke, follow.py'den ONCE yapilmali — katman katman.

Faz 3 — arac hareketi (15 dk):

```bash
/usr/bin/python3 ~/fpv-test/tools/sitl_target_mover.py \
  --world ap_follow_demo --model kenet_car \
  --trajectory circle --radius 8 --speed 1.5 --rate-hz 20 --duration 3600
```

Kabul: arac Gazebo'da daire cizer; mover'in olculen-hareket dogrulamasi
calisir (komut-gozlem hatasi kucuk).

Faz 4 — follow.py (1 saat): prototipteki tasarim aynen:
- gz pose feed (dron + arac; arac hizi sonlu-fark + LPF),
- arm dongusu (GUIDED + arm dene, STATUSTEXT bas, 120 s butce),
- `MAV_CMD_NAV_TAKEOFF` + irtifa bekleme,
- 10 Hz dongu: standoff noktasi (aracin 6 m gerisi), `v = Kp*hata + arac_hizi`,
  hiz kelepcesi (6 m/s), ENU->NED, `SET_POSITION_TARGET_LOCAL_NED`
  type_mask **2503** (pozisyon+ivme+yaw_rate ignore; hiz + yaw acisi aktif),
  yaw = araca bak,
- mod GUIDED degilse setpoint gonderme (duraklat/surdur olaylarini logla),
- JSONL log: t, dist, dron, arac, v_cmd.

Faz 5 — manuel kontrol (30 dk):
- Klavye: curses script (ok tuslari roll/pitch, a/d yaw, w/s gaz;
  l=LOITER, g=GUIDED, t=LAND, r=RTL; cikista override birak = kanallara 0).
  10 Hz `RC_CHANNELS_OVERRIDE`, kanal esleme RCMAP varsayilani (1 roll,
  2 pitch, 3 gaz, 4 yaw). Gaz baslangici 1500 (hover civari) — 1000'le
  LOITER'a gecmek dusurur.
- Fiziksel kumanda (opsiyonel, kullanici inisiyatifi): USB joystick takiliysa
  MAVProxy `module load joystick` denenir; `/dev/input/js0` su an makinede
  YOK, bu yuzden klavye birincil yol.

Faz 6 — launcher + kabul kosusu (30 dk): tek `launch_demo.sh`
(port kontrolu -> Gazebo -> SITL -> MAVProxy(daemon) -> mover -> follow;
`--headless` ve `--no-follow` bayraklari; Ctrl-C hepsini kapatir; her kosu
`runs/<ts>/` altina log).

## Kabul olcutleri (AGENT-STATUS olcum kulturuyle)

Demo "calisti" demek icin (tek kosuda, loglardan):
1. Arm + kalkis: 8 m'ye ulasildi (GLOBAL_POSITION_INT).
2. **Pozisyon yakinsamasi**: dron-arac yatay mesafesi, arac daire cizerken
   `standoff +/- 2 m` bandina oturuyor ve 60 s boyunca bantta kaliyor
   (heading/`target_found` DEGIL — fpv-test'in olcum-tuzagi dersi).
3. **Spin yok**: dron yaw hizi surekli < 90 deg/s (fpv-test
   `sitl_yaw_monitor.py` ayni dunyada calisir, dogrudan kullanilabilir).
4. Manuel gecis: LOITER'a gecince follow "paused" olayi yaziyor ve setpoint
   kesiliyor; klavyeyle dron kontrol edilebiliyor; GUIDED'a donunce "resumed"
   ve takip suruyor.
5. Kosu hijyeni: temiz cwd, portlar kosu sonunda bos.

Ilk kosu kaniti (follow.jsonl + mover jsonl + gazebo.log) `runs/` altinda
saklanir; Kenet acceptance kapilariyla KARISTIRILMAZ (ayri proje, ayri dunya).

## Kapsam disi (bilerek)

- Kamera/tracker baglantisi (Kenet algi katmani) — demodan sonraki adim:
  iris modeline kamera enjeksiyonu + `kenet/gz_camera.py` ayni desenle.
- ArduPilot parametre tune'u, gercek FC/donanim, failsafe matrisi.
- Coklu arac, engel, rota planlama.

## Tahmini sure

Hazir yapi taslariyla uctan uca ~2-3 saat (en buyuk belirsizlik: GUIDED'da
EKF/arm bekleme suresi ve ilk isaret/eksen dogrulamasi).
