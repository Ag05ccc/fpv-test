# Kritik Not — pr0p kapalı-döngü takip (yaw + pitch)

Tarih: 2026-07-12. Bu dosya, pr0p üzerinde Kenet'in kapalı-döngü takibini
kurarken **ölçerek** bulduğumuz ve ileride tekrar tekrar karşımıza çıkacak
kısıtları tutar. Tahmin değil, hepsi log/MSP ölçümüne dayanıyor.

## 1. Asıl tavan: FPV kamerası gövdeye sabit

Yaklaşma için dron öne yatmak zorunda, ama **kamera gövdeye sabit** olduğu için
dron yattıkça kamera da aşağı bakar. Ortadaki hedef görüntüde yukarı kayar ve
yeterince yatınca **kadrajdan çıkar**.

| Yatış | Kamera | Ortadaki hedef |
| --- | --- | --- |
| 0° | ileri | ortada |
| ~20° | biraz aşağı | yukarı kayar, kadrajda |
| ~50° | epey aşağı | **kadraj dışı** |

Bu, şu kısır döngüyü doğuruyor ve yaklaşmanın yakınsamamasının gerçek sebebi:

```
pitch doyar -> dron fırlar -> kamera aşağı bakar -> hedef kadrajdan çıkar
-> tracker kaybeder -> Kenet komutu 0'a düşer -> dron seviyelenir
-> hedefi yeniden bulur -> pitch yine doyar -> ...
```

Ölçüm (ANGLE, `pitch_authority 3.5`, `max_pitch 0.90` ≈ 50° yatış):
uygulanan pitch `-0.90`'a doydu, takla yok, ama **34 satır `found=False`**
(hedef kaybı) ve komut `-0.27 → 0.00 → -0.90` diye salındı.

**Sonuç: yaklaşma hızının tavanı uçuş modu ya da takla değil, hedefi kadrajda
tutabildiğimiz maksimum yatış.** Pratikte ~20–25°, yani `max_pitch ≈ 0.35–0.45`.

Doğrudan kaldıraç: **kamera uptilt açısı.** Uptilt arttıkça (ör. 30°) dron daha
fazla yatabilirken kamera hâlâ ileri/hedefe bakar — yani yaklaşma hızı tavanı
uptilt ile birlikte yükselir.

## 2. Uçuş modu: ANGLE doğru, HORIZON/ACRO değil

Pitch stickinin **anlamı** moda göre değişiyor; bu yüzden bir moddaki katsayı
diğerine taşınamıyor.

- **ACRO** — stick = dönüş HIZI. Sabit komut ⇒ yatış **birikir** ⇒ hızlı
  yaklaşma, ama dron dönmeye devam eder ve komut yeterince uzun tutulursa
  **takla atar**. Kararlı bir denge değil; "çalıştı" sanılan koşular, iş bitmeden
  yetişmiş koşulardı.
- **HORIZON** — küçük stickte self-level (açı gibi), büyük stickte rate'e döner.
  Sürekli büyük bir yatışı **tutamaz**. Ölçülen bant:

  | authority | uygulanan pitch | takla | yaklaşma | hedef kaybı |
  | --- | --- | --- | --- | --- |
  | 0.35 | 0.105 | yok | yok (düz) | – |
  | 0.60 | 0.18 | yok | yok (düz) | 2 |
  | 0.90 | 0.24 | yok | çok hafif | az |
  | 1.50 | 0.44 | **TAKLA** | – | 19 |

  Güvenli tavan ~0.9–1.0, ama orada yaklaşma zaten zayıf ⇒ **kullanılabilir
  bandı yok**.
- **ANGLE** — stick = **sınırlı açı**. Satürasyon **takla değil**, sadece
  "maksimum yatış" (~55° limit). Kenet'in ihtiyacı olan "yatışı tut" davranışını
  veren tek mod. Doğru mimari: **açı döngüsü Betaflight'ta (kHz), hedef seçimi ve
  yönlendirme Kenet'te (30 Hz).** ACRO'da roll'u kendimiz kapatmak, aslında
  ANGLE'ı 30 Hz'te ve uinput gecikmesiyle yeniden yazmak demek — çok daha kötü.

Betaflight'ta ANGLE tanımlı değildi (sadece ARM vardı). MSP ile eklendi:
`ANGLE (boxId 1) -> AUX3/CH7, 1300-1700` (CH7 sabit 1500 okur ⇒ sürekli açık).
`MSP_PID_ADVANCED` pr0p'de **stub** (1 byte döner) ⇒ `angle_limit` MSP'den
değiştirilemiyor.

## 3. Açık iş (sıradaki düzeltmeler)

1. `max_pitch`'i kadrajı koruyacak seviyeye sabitle (~0.40 ≈ 22°); kamera uptilt
   30°'ye çekilirse bu tavan yükseltilebilir — **yeniden ölç**.
2. **Hedef kaybında komutu sıfırlama, kısa süre TUT.** Şu an `found=False` olunca
   komut anında 0'a düşüyor; bu dronu seviyeleyip salınımı besliyor.
3. Tracker dayanıklılığı: çok küçük kilit kutusunda (34×28) CSRT'nin tutunacak
   dokusu azalır; kayıp sayısına bakarak boyutu dengele.

## 4. Araçlar

`experiments/simitl_pr0p_probe/live_kenet_send.py` — kapalı-döngü yaw+pitch mux
(pilot passthrough / CH6 TRACKING'de Kenet devralır), pencere-takipli yakalama,
CSRT + Kenet PID. Kazançlar `/tmp/kenet_tune.json`'dan **canlı** okunur
(uçarken restart gerekmez); işaret çevirme ve bağlama-titreşimi de bayrak
dosyalarıyla kontrol edilir — çünkü izleyici penceresine odaklanmak pr0p'nin
odağını çalar ve pr0p odakta değilken **RC okumayı bırakır**.
