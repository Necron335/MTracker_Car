# Protokół komunikacyjny MTracker

Dokument opisuje format ramek obsługiwany przez bieżącą wersję firmware robota
MTracker. Ten sam logiczny format treści jest używany przez USB/SCI oraz nRF24.

## Kolejność bajtów i typy danych

- Wszystkie liczby wielobajtowe w polu danych są przesyłane w kolejności little-endian.
- `int16` i `int24` są liczbami ze znakiem w kodzie uzupełnień do dwóch.
- `float32` oznacza IEEE-754 single precision, little-endian.
- Numer robota ma zakres `1..15` w sterowniku MATLAB.
- `SEQ` jest jednobajtowym numerem transakcji. W komunikacji USB bieżący sterownik
  MATLAB wysyła `0`. Radio używa go do skojarzenia odpowiedzi ACK/POLL z żądaniem.

## Ramka USB/SCI

```text
+--------+-----+-------+-----+-------+-----+-------------+--------+--------+
| 0xAA   | LEN | GROUP | CMD | ROBOT | SEQ | DATA        | CRC_H  | CRC_L  |
+--------+-----+-------+-----+-------+-----+-------------+--------+--------+
  1 B      1 B    1 B    1 B    1 B   1 B   LEN - 4 B      1 B      1 B
```

- `0xAA` — znacznik początku ramki.
- `LEN` — liczba bajtów od `GROUP` do końca `DATA`; nie obejmuje `0xAA`,
  samego `LEN` ani CRC.
- `GROUP` — obecnie zawsze `0x01`.
- `CMD` — kod polecenia.
- `ROBOT` — numer robota.
- `SEQ` — numer transakcji, kopiowany do odpowiedzi.
- CRC-16: wielomian `0x8005`, wartość początkowa `0xFFFF`, bez końcowego XOR.
  CRC jest liczone od bajtu `LEN` przez wszystkie `LEN` bajtów treści.
- Całkowity rozmiar ramki wynosi `LEN + 4` bajty.
- USB robota używa SCI-C: `921600 bit/s`, 8 bitów danych, brak parzystości,
  1 bit stopu, brak programowej kontroli przepływu.

## Transport nRF24

Pakiet radiowy nie zawiera znacznika `0xAA` ani CRC warstwy szeregowej. Pierwszym
bajtem payloadu jest `LEN`, po którym występują `GROUP`, `CMD`, `ROBOT`, `SEQ` i
`DATA`. Aplikacyjny payload nRF24 ma zawsze 32 bajty; znaczące jest pierwsze
`LEN + 1` bajtów.

Robot przyjmuje wyłącznie polecenia z pasującym `ROBOT`. Odpowiedź jest zapisywana
jako ACK payload. Nadajnik pobiera ją późniejszym pakietem POLL zawierającym ten
sam `SEQ`. CRC sprzętowe nRF24 pozostaje niezależne od CRC ramki USB.

## Bity statusu ruchu

Pole `STATUS` występuje w poleceniach ruchu i ma format `uint16`:

| Bit | Maska | Znaczenie |
|---:|---:|---|
| 0 | `0x0001` | włącz regulator prędkości |
| 1 | `0x0002` | włącz stopnie mocy silników |
| 2 | `0x0004` | ustaw odometrię z pól `X`, `Y`, `THETA` |
| 3 | `0x0008` | `CMD=5`: zadanie skrętu jest kątem w radianach |
| 15..4 | — | zarezerwowane, wysyłać jako zero |

## Polecenia

### `CMD = 1` — podstawowe zadanie prędkości

Żądanie: `LEN = 8`.

| Pole | Typ | Jednostka |
|---|---|---|
| `VL` | `int16` | starszy format skalowany wewnętrznie przez firmware |
| `VR` | `int16` | starszy format skalowany wewnętrznie przez firmware |

Polecenie nie generuje odpowiedzi. Jest utrzymywane wyłącznie dla zgodności.

### `CMD = 3` — ruch i odometria, format starszy

Żądanie zawiera `STATUS`, `WL`, `WR`, `X`, `Y`, `THETA`, gdzie prędkości są
`int16 Q8`, a pozycja i orientacja są `float32`. Odpowiedź używa również starszego
formatu opartego na `float32`. Nowe aplikacje powinny używać polecenia `6`.

### `CMD = 4` — parametry regulatora

Żądanie: `LEN = 20`, cztery wartości `float32`:

| Pole | Dopuszczalny zakres |
|---|---:|
| `feedbackBandwidth` | `1..200` |
| `observerBandwidth` | od `feedbackBandwidth` do `1000` |
| `damping` | `0.1..5` |
| `inertia` | `0.00001..1` |

Odpowiedź: `LEN = 22`: `STATUS_CONFIG` (`uint16`) oraz cztery aktywne wartości
`float32` w tej samej kolejności.

Dla aktualnej konfiguracji `CAR_VEHICLE` zweryfikowane nastawy początkowe to:

```text
feedbackBandwidth = 15.0 rad/s
observerBandwidth = 80.0 rad/s
damping            = 1.0
inertia            = 0.0025
```

`observerBandwidth` stroi ESO regulatora silnika. Niezależny obserwator
prędkości enkodera ma obecnie stałe pasmo `20 rad/s` i nie jest zmieniany przez
`CMD=4`. Nowe parametry działają natychmiast w RAM; trwały zapis wymaga
dwuetapowego `CMD=8`. Kompensacja strefy martwej napędu nie jest obecnie
zaimplementowana ani konfigurowana przez protokół.

### `CMD = 5` — sterowanie pojazdem typu car

Jest to podstawowe polecenie ruchu dla konfiguracji `CAR_VEHICLE`.

Żądanie: `LEN = 20`, 16 bajtów `DATA`:

| Offset DATA | Pole | Typ / skala | Jednostka |
|---:|---|---|---|
| 0 | `STATUS` | `uint16` | maska bitowa |
| 2 | prędkość tylnego koła | `int16 / 256` | rad/s |
| 4 | zadanie skrętu | `int16 / 4096` | surowe `[-1,1]` albo rad |
| 6 | `X` | `int24 / 32768` | m |
| 9 | `Y` | `int24 / 32768` | m |
| 12 | `THETA` | `int16 / 1024` | rad |
| 14 | reserved | `uint16 = 0` | — |

Publiczna dodatnia prędkość oznacza fizyczną jazdę do przodu. Ze względu na
zachowaną zgodność starszego formatu `CMD=5`, wartość na przewodzie jest
kodowana ze zmienionym znakiem:

```text
wireVelocityQ8 = round(-requestedRearVelocityRadPerSec * 256)
```

Bit 3 pola `STATUS` wybiera interpretację zadania skrętu:

- `0`: surowe położenie serwa `[-1, 1]` (tryb zgodny ze starszym protokołem),
- `1`: rzeczywisty kąt skrętu w radianach.

Dla trybu kątowego firmware oblicza
`raw = c0 + c1*a + c2*a^2 + c3*a^3`, gdzie `a` jest kątem w radianach, po czym
ogranicza wynik do `[-1, 1]`. Współczynniki ustawia polecenie `13`.

Odpowiedź: `LEN = 22`, 18 bajtów `DATA`:

| Offset DATA | Pole | Typ / skala | Jednostka |
|---:|---|---|---|
| 0 | status | `uint16` | `0` |
| 2 | zmierzona prędkość tylnego koła | `int16 / 256` | rad/s |
| 4 | `X` | `int24 / 32768` | m |
| 7 | `Y` | `int24 / 32768` | m |
| 10 | `THETA` | `int16 / 1024` | rad |
| 12 | sterowanie silnika | `int8 / 127` | zakres `-1..1` |
| 13 | `POWER[0..4]` | format jak w odpowiedzi `CMD=6` | — |

Zmierzona prędkość w odpowiedzi ma konwencję fizyczną bez dodatkowego
odwracania znaku: wartość dodatnia oznacza jazdę do przodu. Odbiornik dzieli
wartość `int16` przez `256`.

### `CMD = 6` — prędkości kół i odometria

Jest to podstawowe polecenie ruchu dla robota różnicowego.

Żądanie: `LEN = 20`, 16 bajtów `DATA`:

| Offset DATA | Pole | Typ / skala | Jednostka |
|---:|---|---|---|
| 0 | `STATUS` | `uint16` | maska bitowa |
| 2 | `WL` | `int16 / 256` | rad/s |
| 4 | `WR` | `int16 / 256` | rad/s |
| 6 | `X` | `int24 / 32768` | m |
| 9 | `Y` | `int24 / 32768` | m |
| 12 | `THETA` | `int16 / 1024` | rad |
| 14 | reserved | `uint16 = 0` | — |

Odpowiedź: `LEN = 30`, 26 bajtów `DATA`:

| Offset DATA | Pole | Typ / skala | Jednostka |
|---:|---|---|---|
| 0 | zmierzona `WL` | `int16 / 256` | rad/s |
| 2 | zmierzona `WR` | `int16 / 256` | rad/s |
| 4 | `X` | `int24 / 32768` | m |
| 7 | `Y` | `int24 / 32768` | m |
| 10 | `THETA` | `int16 / 1024` | rad |
| 12 | błąd pozycji lewego koła | `int16 / 1024` | jednostka regulatora |
| 14 | błąd pozycji prawego koła | `int16 / 1024` | jednostka regulatora |
| 16 | sterowanie lewego koła | `int8 / 127` | zakres `-1..1` |
| 17 | sterowanie prawego koła | `int8 / 127` | zakres `-1..1` |
| 18 | `POWER[0]` | starsze 8 bitów napięcia 12-bit | — |
| 19 | `POWER[1]` | młodsze 4 bity napięcia, starsze 4 bity prądu | — |
| 20 | `POWER[2]` | młodsze 8 bitów prądu | — |
| 21 | `POWER[3]` | starsze 8 bitów mocy | — |
| 22 | `POWER[4]` | młodsze 4 bity mocy, 4 bity flag | — |
| 23 | `LIGHT_INTENSITY` | `uint24 / 100`, little-endian | lx |

`LIGHT_INTENSITY = 0xFFFFFF` oznacza pomiar niedostępny lub nieważny.
Gdy `LIGHT_INTENSITY_BH1750` jest wyłączone, pole światła jest pomijane:
odpowiedź ma `LEN = 27` i 23 bajty DATA. Zero oznacza poprawny pomiar 0 lx.
Wartość pochodzi z ostatniego poprawnego odczytu BH1750 i uwzględnia wybrany tryb
pomiaru. Skala transmisji 0,01 lx nie zwiększa rozdzielczości czujnika.

Rozszerzenie zachowuje offsety poprzednich pól. Odbiornik musi akceptować nowe
`LEN = 30` zamiast `27` i odczytać trzy dodatkowe bajty. Ramka USB ma teraz
34 bajty; znacząca część payloadu radiowego ma 31 bajtów i mieści się w 32 bajtach.

Pola zasilania są pakowane kolejno jako `V[11:0]`, `I[11:0]`, `P[11:0]`,
`FLAGS[3:0]`. Napięcie jest bez znaku, w jednostkach `10 mV`. Prąd jest liczbą
12-bit ze znakiem w kodzie uzupełnień do dwóch, w jednostkach `2 mA`. Moc jest
bez znaku, w jednostkach `25 mW`. Zakresy wynoszą odpowiednio `0..40,95 V`,
`-4,096..4,094 A` i `0..102,375 W`.

| Bit FLAGS | Maska | Znaczenie |
|---:|---:|---|
| 0 | `0x01` | pomiar prądu i mocy jest ważny |
| 1 | `0x02` | napięcie pochodzi z INA219; w przeciwnym razie z ADC |
| 2 | `0x04` | moduł INA219 został poprawnie skonfigurowany |
| 3 | `0x08` | obsługa monitora mocy jest włączona w firmware |

Gdy INA219 nie jest dostępny albo jego pomiar nie jest jeszcze ważny, firmware
wysyła napięcie z ADC oraz zerowy prąd i moc.

Przykład żądania dla robota `1`, obu kół `0.5 rad/s`, regulatora i silników
włączonych, bez aktualizacji odometrii:

```text
AA 14 01 06 01 00 03 00 80 00 80 00 00 00 00 00 00 00 00 00 00 00 0D BA
```

### `CMD = 7` — geometria odometrii

Żądanie: `LEN = 16`, trzy wartości `float32` w milimetrach:
`leftWheelRadiusMm`, `rightWheelRadiusMm`, `wheelBaseMm`.

Odpowiedź: `LEN = 18`: `STATUS_CONFIG` (`uint16`) oraz trzy aktywne wartości
`float32` w tej samej kolejności.

### `CMD = 8` — zapis konfiguracji

Zapis całej konfiguracji do EEPROM M24C64 wymaga dwóch kolejnych zaakceptowanych
poleceń `CMD = 8`:

1. pierwsze zwraca `CONFIG_SAVE_ARMED`,
2. drugie zwraca `CONFIG_SAVE_ACCEPTED`, po czym zapis jest wykonywany dopiero
   po opróżnieniu toru odpowiedzi.

Jeżeli pierwszy bajt `DATA` ma wartość `1`, polecenie tylko odczytuje bieżący
status i nie inicjuje zapisu. Ze względu na minimalny rozmiar ramki USB sterownik
MATLAB dopełnia żądanie do czterech bajtów `DATA`.

Odpowiedź: `LEN = 6`, `STATUS_CONFIG` jako `uint16`.

### `CMD = 13` — kalibracja skrętu

Polecenie pozwala przesłać z zewnętrznego drivera kompletną kalibrację
wielomianową. Żądanie ma `LEN = 20` i 16 bajtów `DATA`:

| Offset DATA | Rozmiar | Pole | Znaczenie | Dopuszczalny zakres |
|---:|---:|---|---|---:|
| 0 | 4 | `c0` | wyraz stały, surowe położenie dla `0 rad` | `-1..1` |
| 4 | 4 | `c1` | współczynnik liniowy | `-1000..1000` |
| 8 | 4 | `c2` | współczynnik kwadratowy | `-1000..1000` |
| 12 | 4 | `c3` | współczynnik sześcienny | `-1000..1000` |

Wszystkie współczynniki są przesyłane jako IEEE-754 `float32`, little-endian.
Firmware wylicza dla kąta `a` w radianach:

```text
raw = clamp(c0 + c1*a + c2*a^2 + c3*a^3, -1, 1)
```

Odpowiedź: `LEN = 22`: `STATUS_CONFIG` (`uint16`) oraz aktywne `c0`, `c1`, `c2`,
`c3`. Status `CONFIG_OK` oznacza, że nowe dane zostały zaakceptowane i działają
od razu w RAM. Nie oznacza jeszcze zapisu trwałego. Dopiero poprawna, dwuetapowa
sekwencja `CMD = 8` zapisuje kalibrację wraz z geometrią i parametrami regulatora
do EEPROM.

## Status konfiguracji

| Wartość | Nazwa | Znaczenie |
|---:|---|---|
| 0 | `CONFIG_OK` | operacja poprawna |
| 1 | `CONFIG_INVALID_DATA` | dane lub długość niepoprawne |
| 2 | `CONFIG_STORAGE_ERROR` | błąd zapisu EEPROM |
| 3 | `CONFIG_SAVE_ARMED` | zapis uzbrojony, wymagane drugie polecenie |
| 4 | `CONFIG_SAVE_ACCEPTED` | zapis przyjęty do wykonania |
| 5 | `CONFIG_SAVE_NOT_REQUESTED` | zapis nie był żądany |
| 6 | `CONFIG_SAVE_CANCELLED` | oczekujący zapis anulowany |

## Walidacja odpowiedzi po stronie PC

Odpowiedź USB należy zaakceptować tylko wtedy, gdy:

1. znaleziono nagłówek `0xAA`,
2. `GROUP == 1`,
3. odebrano dokładnie `LEN + 4` bajty,
4. CRC jest poprawne,
5. `ROBOT` odpowiada aktualnie wybranemu robotowi,
6. kod `CMD` i — dla radia — `SEQ` odpowiadają oczekiwanej transakcji.

