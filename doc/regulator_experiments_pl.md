# Rejestracja i automatyczne strojenie lewego regulatora CAR

## Dane pomiarowe

Domyślnie `acquisition: cmd5_telemetry` korzysta z dostępnych odpowiedzi
sterownika, bez dodatkowego adaptera firmware. Temat `/regulator/drive_sample`
(`mtrackercar/msg/DriveSample`) zawiera prędkość zmierzoną i wyjście silnika
z odpowiedzi CMD=5 oraz prędkość z żądania dopasowanego po numerze sekwencji.
Czas oznacza odebranie odpowiedzi przez sterownik, a nie chwilę pomiaru MCU.
Komenda w tym temacie nie jest odczytem wewnętrznej referencji regulatora.

Ten tryb ocenia śledzenie prędkości i aktywność wyjścia. Nie odtwarza błędu
pozycji ani bufora RAM. Wymaga co najmniej 20 Hz, przerw poniżej 0.15 s
i pełnego zapisu przebiegu; opóźnienia PC ograniczają ocenę szybkiej dynamiki.
Nie służy do identyfikacji przebiegów z rozdzielczością 1 ms.

Opcjonalny `acquisition: firmware_buffer` używa próbek z RAM MTrackera.
Zgodnie z `trace_signal_map.md` wybierane
kanały CAR to `[1, 5, 13, 15]`:

| ID | Pole firmware | Zapis w module |
| --- | --- | --- |
| 1 | `drive.wRear` | prędkość robota `velocity_m_s = wRear * wheel_radius` |
| 5 | `regL.u` | wyjście lewego regulatora `left_output`, `[-1,1]` |
| 13 | `regL.e_phi` | **błąd pozycji** lewego regulatora `left_position_error_rad` |
| 15 | `RearWheelReference()` | rzeczywiste zadanie `reference_velocity_m_s = reference * wheel_radius` |

Kanały prawego regulatora nie są wykorzystywane. Błąd śledzenia prędkości
`reference_velocity_m_s - velocity_m_s` obliczany jest dodatkowo do oceny
jakości; nie jest zamiennikiem błędu pozycji z kanału 13.

Każdy sygnał zajmuje dwa słowa 16-bit, młodsze pierwsze. Kolejność kanałów
odpowiada żądaniu START. Czas próbki jest odtwarzany jako
`index * period_ms / 1000`. Dla 4 kanałów bufor 512 KiB mieści 32768 próbek,
czyli około 32.768 s przy okresie 1 ms. Moduł `trace_data.py` dekoduje ten
format, sprawdza pełne rekordy i konwertuje kanały CAR do jednostek CSV.

## Przebiegi zadane

`config/regulator_experiment.yaml` definiuje sygnały `training` i `validation`.
Dostępne rodzaje `kind`:

- `steps`: dodatnie skoki, zatrzymanie, ujemne skoki i zatrzymanie;
- `ramp`: narastanie i opadanie prędkości w obu kierunkach;
- `sine`: sinus o zadanej częstotliwości;
- `chirp`: sinus o liniowo zmienianej częstotliwości;
- `prbs`: powtarzalny pseudolosowy przebieg dwupoziomowy.

`amplitude` jest w m/s, `duration` i `quiet` w sekundach, częstotliwości w Hz.
Na początku i końcu każdego przebiegu występuje odcinek zerowej prędkości.
Sterowanie prędkością z PC nie ma gwarancji czasu rzeczywistego. Do oceny
używana jest referencja z żądania CMD=5 albo **rzeczywista referencja z bufora**,
zależnie od trybu, a nie wyłącznie teoretyczny przebieg zadany.
Zerwane indeksy, nieciągły czas, przepełnienie i brak końca
rejestracji dyskwalifikują próbę.

## Metoda automatyczna

Przeszukiwane są cztery parametry `CMD=4`:

1. `feedback_bandwidth` — pasmo sprzężenia, `1..200 rad/s`;
2. `observer_bandwidth` — pasmo ESO silnika, od pasma sprzężenia do `1000 rad/s`;
3. `damping` — `0.1..5`;
4. `inertia` — `0.00001..1`.

Punktem początkowym są znane, działające nastawy operatora. W pliku domyślnym
wpisano wartości CAR z dokumentacji: `[15.0, 80.0, 1.0, 0.0025]`. Nie oznacza
to odczytu nastaw z Twojego robota. Przed uruchomieniem strojenia potwierdź
`baseline_confirmed: true` i dopasuj `baseline` do sprawdzonej konfiguracji.

Zastosowano ograniczone przeszukiwanie współrzędnych w logarytmach parametrów.
Ta metoda nie wymaga różniczkowania zaszumionych danych. Każda próba zmienia
jedną współrzędną, pozostaje w zadanym przedziale i odtwarza ten sam sygnał
treningowy. Przy braku poprawy krok jest zmniejszany.

Przykładowe wąskie przedziały w pliku konfiguracyjnym są propozycją początkową,
a nie potwierdzonym obszarem stabilności robota. Cały zakres ESO musi pozostawać
powyżej całego zakresu pasma sprzężenia w tej implementacji.

Minimalizowana funkcja łączy:

- RMSE śledzenia prędkości, odniesione do amplitudy;
- tylko w trybie RAM: RMS błędu pozycji `regL.e_phi`, znormalizowany przez drogę przy amplitudzie
  prędkości przez 1 sekundę;
- aktywność sygnału sterującego;
- udział próbek z nasyconym wyjściem.

Jeśli zapisany przebieg nie spełni warunku ustalenia prędkości po końcowym
zatrzymaniu (`Robot did not settle after the final stop`), próba otrzymuje
status `quality_rejected`, a jej CSV pozostaje w wynikach. Program zadaje zero
i czeka do 5 s na prędkość nieprzekraczającą 0.015 m/s przez co najmniej 0.4 s.
Po potwierdzeniu zatrzymania idzie dalej: pomija taki punkt treningowy,
a pomiar walidacyjny powtarza jeden raz. Powtórka ma własny numer próby i CSV.
Jeżeli również ona zostanie odrzucona, wykonywane są pozostałe pomiary,
ale niepełna walidacja nie pozwala zastosować ani zapisać nowych nastaw.
`relative_improvement` ma wtedy wartość `null`, a wynik wraca do bazy.

Dalszy ruch po 5 s zatrzymywania, utrata komunikacji, przekroczenie limitów,
nadmierne nasycenie lub uszkodzone dane nadal przerywają przebieg z próbą
przywrócenia bazy. Odrzucony pomiar nie jest traktowany jako poprawny wynik.

Najlepszy kandydat jest następnie porównywany z bazą na osobnym sygnale
walidacyjnym. Domyślnie wykonuje się po 2 próby dla bazy i kandydata, ze
zmienianą kolejnością. Wymagana jest poprawa średniej oceny o co najmniej 3%
i brak pogorszenia w parach prób. Raport pokazuje **najlepszy znaleziony
zestaw w badanym zakresie**, nie dowód globalnego optimum.

## Zastosowanie i zapis

Domyślnie `apply_best: false` i `persist_best: false`: po próbach wraca baza,
a wynik jest propozycją w raporcie. Każda próba strojenia tymczasowo zmienia
nastawy w RAM, nawet gdy zastosowanie wyniku końcowego jest wyłączone.

Po udanej walidacji `apply_best: true` wysyła wszystkie cztery parametry przez
istniejącą usługę `/set_regulator`. `persist_best: true` następnie korzysta z
`/save_configuration`, która sprawdza dwustopniowy zapis EEPROM i jego
zakończenie. Zapis dotyczy całego rekordu konfiguracji, także kalibracji skrętu.

Jeżeli wynik nie przejdzie walidacji, program przywraca bazę bez zapisu EEPROM.
Przy przerwanym połączeniu przywrócenie może się nie udać; stan jest zapisany
w raporcie. Timeout zapisu EEPROM oznacza **nieznany wynik zapisu**, nawet gdy
później uda się przywrócić bazę w RAM. Nie jest to potwierdzenie przywrócenia
poprzedniej zawartości EEPROM.

## Uruchomienie

Tryb telemetrii działa przez istniejący protokół CMD=4 i CMD=5.
Nie wymaga analizy ani zmiany firmware. Opcjonalne pełne podłączenie usługi
`/controller_capture` do portu szeregowego wymaga stałych TRACE oraz semantyki
odpowiedzi z `MTrackerProtocol.h` / `MTrackerTrace.m`. Bez dostępnego dostawcy
tej usługi preflight trybu `firmware_buffer` kończy się błędem przed próbą ruchu
lub zmianą regulatora. Wybór telemetrii jest jawnym osobnym trybem, z innym CSV
i funkcją oceny; próbki RAM nie są zastępowane danymi CMD=5.

Przebuduj pakiet i zakończ wcześniej uruchomiony sterownik:

```bash
cd /home/student/ros2_workspace
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to mtrackercar --symlink-install
source /home/student/ros2_workspace/install/setup.bash
ros2 launch mtrackercar regulator_tuning.launch.py mode:=record
```

W konfiguracji eksperymentu ustaw jawnie `enable_motion: true`. Zapis danych
bez zmiany regulatora to `mode:=record`; pełne strojenie to `mode:=tune`.
Przed pełnym strojeniem ustaw sprawdzone `baseline`, granice `bounds`
i `baseline_confirmed: true`, a następnie uruchom launch z `mode:=tune`.
Konfiguracja jest odczytywana przy starcie procesu. Launch i usługa startowa
nie są potwierdzeniem zakończenia strojenia; wynik znajdziesz na temacie statusu
i w raporcie. Przed użyciem sprawdź zgodność kierunku ruchu z komendą i odom.

```bash
ros2 service call /regulator_tuning/start std_srvs/srv/Trigger '{}'
ros2 topic echo /regulator/status
# Przerwanie:
ros2 service call /regulator_tuning/cancel std_srvs/srv/Trigger '{}'
```

Launch wybiera w sterowniku `command_source=regulator_tuning`, dedykowane
wejście prędkości w m/s i zerowy kąt skrętu. Ignoruje `cmd_vel`. Sterownik
ogranicza zadanie do 0.10 m/s i zachowuje watchdog komend. Moduł dodatkowo
monitoruje świeżość odom, diagnostykę, prędkość i promień obszaru eksperymentu.
Limity `max_duration_s` i `max_total_distance_m` obejmują cały przebieg.
`max_trials` dotyczy treningu; dochodzi `2 × validation_repeats` prób walidacji.
Nie uruchamiaj równocześnie innego modułu strojenia ani usług zmieniających
nastawy. Anulowanie zatrzymuje ruch i podejmuje przywrócenie bazy; poczekaj
na końcowy status. `Ctrl+C` utrzymuje obsługę odpowiedzi na czas zakończenia
pracy i przywracania nastaw (do 20 s). Utrata połączenia może uniemożliwić rollback.

## Odpowiedzi konfiguracji i timeout

W transporcie USB/SCI `CMD=4` zwraca bezpośrednio `LEN=22`: status i cztery
aktywne wartości. Nie wymaga osobnego odczytu ani pakietu POLL (POLL dotyczy
transportu radiowego nRF24). Dokumentacja nie gwarantuje czasu odpowiedzi.

Sterownik kończy oczekującą transakcję ruchu, wysyła zatrzymanie i czeka na
jego odpowiedź, a dopiero potem wysyła konfigurację. `response_timeout_ms`
(domyślnie 50 ms) dotyczy cyklicznego ruchu, a `configuration_timeout_ms`
(domyślnie 500 ms, zakres 1..2000 ms) oczekiwania w transakcjach konfiguracji.
Po konfiguracji moduł czeka na powrót świeżej telemetrii przed następną próbą.

Timeout wskazuje oczekiwane `CMD`, `SEQ`, liczbę odebranych bajtów i liczbę
ramek niedopasowanych do żądania. `CMD=5` wskazuje etap ruchu/zatrzymania,
`CMD=4` — ustawianie regulatora. Brak odpowiedzi nie dowodzi, że firmware
nie zastosowało ustawień. Program przerywa strojenie, czeka do 5 s na powrót
potwierdzonej komunikacji i podejmuje jedną próbę przywrócenia bazy.
Sprawdź `baseline_restored` w `tuning.yaml`; samo `Connected to …`
potwierdza otwarcie portu, a nie przywrócenie nastaw. Moduł nie ponawia
automatycznie niepotwierdzonej próby strojenia ani zapisu EEPROM.

Po potwierdzonym uzbrojeniu zapisu sterownik wysyła drugie żądanie tylko raz.
Jeśli jego potwierdzenie zaginie, przechodzi do odczytu statusu. Timeout
pojedynczego odczytu statusu nie kończy operacji: zapytania tylko do odczytu
są ponawiane w ramach 5-sekundowego limitu oczekiwania na zakończenie.
Nie są ponawiane żądania inicjujące zapis. Brak potwierdzenia pierwszego
uzbrojenia nadal kończy operację jako niepotwierdzoną.

Błąd EEPROM nie usuwa ukończonego strojenia: sprawdź `validated`,
`selected_parameters` i `relative_improvement` w `tuning.yaml` oraz plik
`regulator_parameters.yaml`. Przywrócenie bazy w RAM po błędzie nie cofa
ewentualnego zapisu EEPROM. Aby ponownie zastosować wybrane nastawy,
wyślij wartości z tego pliku przez `/set_regulator`; nie trzeba powtarzać
wszystkich przejazdów. Zapis trwały to osobna operacja `/save_configuration`.

## Wyniki i wykresy

Wyniki powstają w `output_dir/<data_czas>/`, domyślnie `regulator_results`
względem katalogu uruchomienia. Każda próba trafia do `trial_NNN.csv`.
Tryb CMD=5 zapisuje:

```text
index,time_s,requested_velocity_m_s,velocity_m_s,motor_output
```

Tryb RAM zapisuje:

```text
index,time_s,reference_velocity_m_s,velocity_m_s,left_position_error_rad,left_output
```

`tuning.yaml` zawiera parametry każdej próby, miary jakości, błędy i końcowy
status walidacji, zastosowania i zapisu. Po zaakceptowanej walidacji powstaje
`regulator_parameters.yaml` z wybranymi czterema nastawami.

`report.html` jest samodzielnym raportem z osadzonym wykresem. Dostępne są
również `regulator.png` i `regulator.pdf`. Wykres pokazuje prędkość zadaną
i zmierzoną, błąd śledzenia prędkości (CMD=5) albo błąd pozycji (RAM),
wyjście silnika oraz oceny prób. Pole `acquisition` wskazuje źródło danych.
Ranking parametrów treningowych jest oddzielony od wyników walidacji.

Przykład bez robota:

```bash
ros2 run mtrackercar regulator_tuning \
  --config /home/student/ros2_workspace/src/MTrackerCar-ROS/config/regulator_experiment.yaml \
  --mode tune --demo
```

Tryb `--demo` używa syntetycznego obiektu testowego, **nie modelu ADRC MTrackera**.
Nie łączy się z robotem i nie zapisuje EEPROM. Raport jest oznaczony jako
syntetyczny; znalezione w nim nastawy nie są wynikiem strojenia Twojego robota.

## Kontrakt adaptera bufora

`mtrackercar/srv/ControllerCapture` udostępnia operacje START/STOP/READ.
START identyfikuje sesję, okres próbkowania, pojemność i wiek uruchomienia
rejestracji. STOP jest idempotentny i potwierdza liczbę próbek, kompletność
oraz brak przepełnienia. READ zwraca strony jednego niezmiennego przebiegu
z ciągłymi indeksami. Wartości `ControllerBufferSample` muszą pochodzić z
kanałów RAM, a nie z interpolacji telemetrii PC. Wiek rejestracji umożliwia
zgranie generatora zadania z początkiem czasu próbek.

Zobacz także [mapę sygnałów](trace_signal_map.md),
[pełny protokół](communication_protocol.md) i
[ustawianie regulatora](regulator_manual_pl.md).
