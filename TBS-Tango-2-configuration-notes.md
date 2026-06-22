# TBS Tango 2 Configuration Notes

Amaç: TBS Tango 2 kumandadaki switch'leri Linux USB joystick, Betaflight SITL
ve Kenet test akışında doğru RC/AUX kanallarına taşımak.

## Mevcut durum

Sistem kumandayı USB joystick olarak görüyor:

```text
/dev/input/js0
Team-BlackSheep TBS Joystick
7 axis, 24 button
```

Şu an doğrulanan mapping:

| Fiziksel kontrol | Linux raw input | RC kanal | Kullanım |
|---|---:|---:|---|
| Roll | Axis 0 | CH1 | Roll |
| Pitch | Axis 1 | CH2 | Pitch, invert |
| Throttle | Axis 2 | CH3 | Throttle |
| Yaw | Axis 3 | CH4 | Yaw |
| 2-state switch | Axis 4 | CH5 / AUX1 | Betaflight ARM |
| 3-state switch | Axis 6 | CH6 / AUX2 | Kenet IDLE / AI-ARMED / TRACKING |

Gözlem: Sağ taraftaki ikinci 3-state switch oynatıldığında:

```bash
python tools/sitl_rc_bridge.py --device /dev/input/js0 --changes
```

komutu hiçbir `axis` veya `button` değişimi göstermedi.

Bu, Python/bridge mapping hatası değil. Linux joystick cihazı o fiziksel switch
için ham event üretmiyor. Yani switch, Tango 2 model/mixer tarafında aktif bir
output kanalına atanmış olmayabilir veya USB joystick moduna export edilmiyor
olabilir.

## Tango 2 üzerinde yapılacak ayar

Hedef:

```text
Sağdaki ikinci 3-state switch -> CH7 / AUX3
```

Genel FreedomTX/OpenTX mantığı:

1. Kumandada aktif modeli aç.
2. Model ayarlarına gir.
3. `Inputs` veya `Mixer` sayfasına git.
4. Boş bir kanal seç:

```text
CH7
```

5. Kaynak/source olarak sağdaki 3-position switch'i seç.
6. Mixer değerlerini sade bırak:

```text
Source: sağdaki 3-position switch
Weight: 100
Offset: 0
```

7. Output aralığı standart kalmalı:

```text
low  -> -100% -> USB raw -32767 -> RC 1000
mid  ->    0% -> USB raw      0 -> RC 1500
high -> +100% -> USB raw +32767 -> RC 2000
```

8. Crossfire/channel ayarlarında modelin en az 8 kanal çıkardığından emin ol.
   Gerekirse 8/12 kanal ayarını kontrol et.
9. USB kabloyu çıkarıp tekrar tak.
10. USB bağlantı modunda `Joystick` / `USB Joystick` seçili olduğundan emin ol.

## Doğrulama

Önce sadece raw event kontrolü:

```bash
python tools/sitl_rc_bridge.py --device /dev/input/js0 --changes
```

Sağdaki ikinci 3-state switch oynatıldığında buna benzer çıktı beklenir:

```text
changed axis  5 value=-32767
changed axis  5 value=     0
changed axis  5 value= 32767
```

Axis numarası `5` olmak zorunda değil. Hangi `axis N` görünürse bridge mapping
o numaraya göre güncellenecek.

Eğer hiçbir çıktı gelmezse:

- Switch hâlâ aktif bir model kanalına atanmadı.
- USB joystick moduna export edilmiyor.
- Farklı bir model profili aktif olabilir.
- Kumanda üzerindeki channel count / mixer / output ayarı eksik olabilir.

## Bridge mapping

Şu an `tools/sitl_rc_bridge.py` içinde varsayılan tahmin:

```python
"autopilot_mode_aux3": {"channel": 6, "source": "axis", "index": 5, "three_pos": True}
```

Bu şu anlama gelir:

```text
Axis 5 -> CH7 / AUX3 -> 1000 / 1500 / 2000
```

Doğrulamada başka axis numarası çıkarsa `index` değeri değiştirilecek.

## Betaflight Modes önerisi

Bu switch CH7/AUX3 olarak görünür hale geldikten sonra:

```text
AUX1 / CH5 high -> ARM
AUX2 / CH6      -> Kenet state, Betaflight mode'a bağlanmaz
AUX3 / CH7 low  -> Betaflight mode yok, Acro/default
AUX3 / CH7 mid  -> ANGLE mode
AUX3 / CH7 high -> HORIZON mode
```

Configurator `Modes` sekmesinde önerilen aralıklar:

```text
ARM    AUX1 high  1700-2100
ANGLE  AUX3 mid   1300-1700
HORIZON AUX3 high 1700-2100
```

## Bu adımdan sonraki plan

1. Tango 2 üzerinde sağdaki ikinci 3-state switch'i CH7/AUX3'e ata.
2. `tools/sitl_rc_bridge.py --changes` ile raw axis/button event geldiğini doğrula.
3. Gerekirse `CHANNEL_MAP["autopilot_mode_aux3"]["index"]` değerini düzelt.
4. `--send --verbose` ile Betaflight Receiver tab'de CH7/AUX3 hareketini doğrula.
5. Betaflight `Modes` sekmesinde ANGLE/HORIZON aralıklarını AUX3'e bağla.
6. ARM, ANGLE, HORIZON göstergelerinin doğru switch konumlarında aktif olduğunu doğrula.
7. Bu tamamlandıktan sonra Kenet + pilot RC arbiter aşamasına geç.
