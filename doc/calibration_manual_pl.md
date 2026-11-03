# MTracker CAR — instrukcja kalibracji skrętu

Instrukcja obsługi modułu `steering_calibration` w pakiecie ROS 2 `mtrackercar`.

Ustawianie parametrów regulatora napędu opisuje osobna
[instrukcja regulatora](regulator_manual_pl.md)
([wersja HTML](regulator_manual_pl.html)).

## 1. Co wyznacza program

Program identyfikuje zależność między rzeczywistym kątem skrętu przednich kół a surowym sterowaniem serwem:

```text
rawServo = c0 + c1·a + c2·a² + c3·a³
```

`a` oznacza kąt w radianach, a `rawServo` jest bezwymiarową nastawą serwa. Wynikiem są cztery współczynniki `c0, c1, c2, c3` zgodne z kalibracją firmware `CAR_VEHICLE`.

Podczas pomiaru robot powoli jedzie do przodu i do tyłu przy różnych nastawach serwa. Moduł rejestruje **pozycję x, y i orientację theta z odom**. Nie potrzebuje osobnych prędkości tylnych kół.

Z kolejnych pozycji wyznacza drogę ze znakiem, a ze zmian orientacji — krzywiznę toru. Następnie oblicza kąt skrętu z modelu rowerowego:

```text
krzywizna = zmiana orientacji / droga ze znakiem
kąt skrętu = atan(rozstaw osi · krzywizna)
```

W implementacji krzywizna jest dopasowywana do całej serii pomiarowej, z odpornym ważeniem błędów. Odwijanie orientacji pozwala przechodzić przez granicę ±pi. Dla jazdy do tyłu droga ma znak ujemny, dzięki czemu ten sam skręt daje ten sam zidentyfikowany kąt.

**Warunek pomiaru:** odom musi opisywać rzeczywisty ruch robota, np. wyliczony z pomiarów obu tylnych kół. Odometria integrowana wyłącznie z zadanych komend nie pozwala wyznaczyć rzeczywistej charakterystyki serwa. Wynik zależy również od poprawności geometrii odometrii i poślizgu.

## 2. Przygotowanie środowiska

Polecenia poniżej zakładają workspace `/home/student/ros2_workspace` i ROS 2 Jazzy.

W pierwszym terminalu:

```bash
cd /home/student/ros2_workspace
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to mtrackercar --symlink-install
source install/setup.bash
```

Po aktualizacji kodu zakończ poprzedni launch i uruchom go ponownie.
Już działający proces sterownika nie przejmuje nowej wersji po przebudowie.

W każdym kolejnym terminalu załaduj środowisko:

```bash
cd /home/student/ros2_workspace
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

Zależności modułu obejmują `rclpy`, NumPy, PyYAML i Matplotlib. Firmware musi obsługiwać nowy protokół CAR: `CMD=5` z żądaniem `LEN=20`, odpowiedzią `LEN=22` i kalibracją `CMD=13`.

Zatrzymaj wcześniej uruchomiony sterownik, symulator, joystick i kontrolery jazdy. Launch kalibracji uruchamia własny sterownik. Robot powinien stać na podłożu pozwalającym mierzyć ruch; napęd i serwo będą aktywowane podczas pomiarów. Zapewnij miejsce na przejazdy i możliwość zatrzymania robota. Program nie wykrywa przeszkód.

## 3. Sprawdzenie konfiguracji

### Sterownik: `config/mtrackercar.yaml`

Sprawdź przede wszystkim:

| Parametr | Znaczenie |
| --- | --- |
| `serial_port` | Port robota, domyślnie `/dev/ttyUSB0`. |
| `baud_rate` | Prędkość transmisji, domyślnie `921600`. |
| `robot_address` | Jednobajtowy numer robota zgodny z firmware, `0…255`. Stara wartość `256` jest nieprawidłowa. |
| `enable_drive` | Musi być `true`, żeby wykonać przejazdy. |
| `wheel_radius` | Promień napędzanego koła w metrach. |
| `wheel_base` | Efektywny wzdłużny rozstaw osi modelu rowerowego w metrach. |

### Identyfikacja: `config/calibration.yaml`

`wheel_radius` i `wheel_base` muszą odpowiadać wartościom używanym dla tego samego robota. `wheel_base` oznacza odległość wzdłużną między osiami, a nie rozstaw lewego i prawego tylnego koła.

| Parametr | Domyślnie | Znaczenie |
| --- | --- | --- |
| `speed` | `0.05` | Wartość prędkości jazdy w m/s; moduł i sterownik ograniczają kalibrację do `0.10 m/s`. |
| `raw_values` | 13 wartości od `-1.00` do `+1.00` | Badane nastawy serwa. Co najmniej 5 różnych wartości obejmujących oba znaki; zakres programu `[-1.0, 1.0]`. Dobierz nastawy do mechaniki robota. |
| `repeats` | `3` | Liczba powtórzeń pełnego zestawu; minimum `2`. |
| `pause_duration` | `1.0 s` | Postój między odcinkami, z ustawioną nastawą serwa kolejnego odcinka. |
| `settle_duration` | `1.5 s` | Jazda pozwalająca ustalić skręt przed pomiarem. |
| `measure_duration` | `8.0 s` | Czas rejestracji pojedynczego odcinka. |
| `min_distance` | `0.15 m` | Minimalna zmierzona droga odcinka. |
| `pose_timeout` | `0.3 s` | Maksymalny dopuszczalny wiek/przerwa pomiarów pozycji. |
| `max_radius` | `2.0 m` | Maksymalna odległość od pozycji startowej według odom. |
| `max_total_distance` | `40.0 m` | Limit sumarycznej przebytej drogi, również poza fazą pomiaru. |
| `max_duration` | `900 s` | Limit czasu całego przebiegu. |
| `max_yaw_rms` | `0.025 rad` | Limit błędu modelu stałej krzywizny pojedynczego odcinka. |
| `max_fit_rmse` | `0.04` | Limit RMSE wielomianu w jednostkach `rawServo`. |
| `max_direction_difference` | `0.06 rad` | Limit różnicy średnich kątów przód–tył dla tej samej nastawy. |
| `seed` | `7` | Ziarno losowania kolejności nastaw. |

### Większa liczba przejazdów

Liczba odcinków wynosi:

```text
liczba nastaw × 2 kierunki × liczba powtórzeń
```

Domyślnie: `13 × 2 × 3 = 78` odcinki, około **13 min 39 s** bez dodatkowych opóźnień. Droga faz pomiarowych wynosi około `31.2 m`, a łącznie z ustalaniem skrętu około `37.05 m`.

Dla **130 odcinków** zmień w `calibration.yaml`:

```yaml
repeats: 5
max_duration: 1500.0
max_total_distance: 70.0
```

Przy pozostałych domyślnych ustawieniach potrwa to około **22 min 45 s**, a droga wyniesie około `61.75 m`. Zwiększenie liczby powtórzeń wymaga dostosowania limitów czasu i drogi. Limit promienia przestrzeni dobierz osobno do dostępnego miejsca.

Podczas pierwszego powtórzenia nastawy rosną, podczas drugiego maleją, a od trzeciego ich kolejność jest tasowana. Dla każdej nastawy wykonywana jest para przód–tył; kolejność kierunków zmienia się między powtórzeniami.

## 4. Uruchomienie pomiarów

W pierwszym terminalu:

```bash
ros2 launch mtrackercar calibration.launch.py enable_motion:=true
```

Sam launch **nie rozpoczyna przejazdów**. Sterownik pracuje w trybie `command_source=calibration_raw`, a moduł czeka na usługę startową, świeżą pozycję oraz potwierdzoną komunikację szeregową.

W tym launchu sterownik publikuje odometrię na **`odom`**, a moduł wykorzystuje z niej pozycję i orientację. Nie uruchamiaj drugiego sterownika publikującego na ten sam temat.

W drugim terminalu sprawdź dane:

```bash
ros2 topic echo /odom --once
ros2 topic echo /diagnostics --once
```

Odometria `nav_msgs/msg/Odometry` powinna mieć aktualny znacznik czasu, poprawną orientację oraz nazwy układu odniesienia i układu robota. Diagnostyka `mtrackercar/serial` powinna wskazywać potwierdzoną komunikację.

Rozpocznij przebieg:

```bash
ros2 service call /steering_calibration/start std_srvs/srv/Trigger '{}'
```

Oczekiwane potwierdzenie to `success: true` i ścieżka katalogu wynikowego. Od tej chwili robot wykonuje sekwencję jazdy.

Podgląd postępu:

```bash
ros2 topic echo /calibration/status
```

Komunikaty zawierają numer odcinka, fazę `pause`, `settle` lub `measure`, nastawę serwa i kierunek (`+1`: przód, `-1`: tył).

Pierwszy odcinek domyślnej sekwencji ma `raw=-1.00` i `direction=+1`,
więc robot powinien jechać do przodu. Sterownik wysyła prędkość napędu ze znakiem
zadanej komendy, bez dodatkowej negacji. Zgodność kierunku fizycznego i odometrii
wymaga sprawdzenia na robocie.

### Zatrzymanie i anulowanie

```bash
ros2 service call /steering_calibration/cancel std_srvs/srv/Trigger '{}'
```

Moduł zadaje zerową prędkość i zapisuje pomiary oraz raport częściowy. Komenda zatrzymania jest wysyłana przed generowaniem wykresów. Po zaniku komend sterownik ma dodatkowo własny timeout.

Nie zmieniaj kalibracji firmware, nie resetuj odometrii i nie uruchamiaj innych źródeł surowych komend podczas pomiaru. Tryb kalibracyjny sterownika ignoruje `cmd_vel`, lecz nadal przyjmuje wiadomości na `calibration/raw_command`.

## 5. Własne pliki i inne źródło odom

Możesz wskazać konkretne pliki konfiguracyjne oraz bezwzględny katalog wyników:

```bash
ros2 launch mtrackercar calibration.launch.py \
  config:=/home/student/ros2_workspace/src/MTrackerCar-ROS/config/mtrackercar.yaml \
  calibration_config:=/home/student/ros2_workspace/src/MTrackerCar-ROS/config/calibration.yaml \
  output_dir:=/home/student/ros2_workspace/calibration_results \
  enable_motion:=true
```

Jeżeli właściwy pomiar `nav_msgs/Odometry` pochodzi z innego tematu, np. `/measured_odom`:

```bash
ros2 launch mtrackercar calibration.launch.py \
  pose_topic:=/measured_odom driver_odometry_topic:=/firmware_odom \
  enable_motion:=true
```

Dla istniejącego strumienia `geometry_msgs/Pose2D` z polami `x, y, theta`:

```bash
ros2 launch mtrackercar calibration.launch.py \
  pose_type:=pose2d pose_topic:=/measured_pose \
  driver_odometry_topic:=/firmware_odom enable_motion:=true
```

`Pose2D` nie zawiera znacznika czasu. W tym trybie świeżość oceniana jest na podstawie chwili odebrania wiadomości; opóźnionego pomiaru źródłowego nie da się rozpoznać tak jak w `Odometry`.

Parametry launch `pose_topic`, `pose_type`, `output_dir` i `enable_motion` nadpisują odpowiadające im wartości w pliku `calibration.yaml`. Ustawiaj je przez argumenty launch. Pozostałe parametry, np. `speed` i `repeats`, zmieniaj w pliku przed uruchomieniem. Parametry przebiegu są odczytywane przy tworzeniu modułu.

Aby korzystać z już uruchomionego sterownika, dodaj `start_driver:=false`. Istniejący sterownik musi być uruchomiony z `command_source=calibration_raw`; ten parametr wybiera się przy jego starcie. Samo `steering_mode=raw` nie zastępuje wyboru źródła komend. Dopasuj również promień koła, temat odometrii i diagnostykę.

### Położenie punktu pomiarowego

Pozycja powinna dotyczyć środka tylnej osi, a theta — kierunku przodu pojazdu. Jeśli źródło opisuje inny punkt, ustaw:

- `pose_offset_x`, `pose_offset_y`: położenie mierzonego punktu względem środka tylnej osi, w osiach pojazdu, w metrach;
- `pose_yaw_offset`: orientacja raportowana przez źródło minus orientacja pojazdu, w radianach.

Przykład: punkt pomiarowy leży `0.10 m` przed środkiem tylnej osi i jest zgodnie zorientowany. Ustaw `pose_offset_x: 0.10`, `pose_offset_y: 0.0`, `pose_yaw_offset: 0.0`. Program przeliczy pomiary na środek tylnej osi przed identyfikacją i zapisem CSV.

## 6. Pliki wynikowe

Każdy przebieg tworzy nowy katalog pod `calibration_results`, domyślnie względem katalogu, z którego uruchomiono program. Jego ścieżka pojawia się w komunikatach modułu.

| Plik | Zawartość |
| --- | --- |
| `report.html` | Samodzielny raport z osadzonym wykresem; można otworzyć w przeglądarce bez serwera. |
| `calibration.png` | Wykres zbiorczy do podglądu i udostępniania. |
| `calibration.pdf` | Ten sam wykres w PDF. |
| `poses.csv` | Pomiary z faz `measure`: numer odcinka, powtórzenie, nastawa, kierunek, czas, x, y, theta. |
| `schedule.yaml` | Zaplanowana sekwencja ruchów. |
| `segments.csv` | Wyniki zaakceptowanych odcinków: kąt, krzywizna, droga, błąd orientacji i liczba próbek. |
| `identification.yaml` | Zaakceptowane współczynniki, zakresy pomiarowe, miary jakości i konfiguracja. |
| `steering_calibration.yaml` | Cztery współczynniki w formacie parametrów ROS 2. |
| `report.yaml` | Podsumowanie przebiegu i ewentualny powód błędu. |

Po anulowaniu lub odrzuceniu identyfikacji pozostają dane i raport częściowy. Pliki zaakceptowanej identyfikacji nie są wtedy eksportowane; `segments.csv` powstaje, jeśli zaakceptowano przynajmniej jeden odcinek.

Katalogi nazwane `synthetic_example_…` w tym workspace zawierają **przykład z danych syntetycznych**. Nie są wynikiem pomiarów Twojego robota i ich współczynników nie należy traktować jako jego kalibracji.

## 7. Jak czytać raport

### Trajektorie XY

Pokazują zarejestrowane odcinki pomiarowe; kolor odpowiada nastawie serwa. Sprawdź płynność łuków, brak skoków i rozsądną zgodność przejazdów. Wykres obejmuje fazy pomiarowe, a nie pełny zapis postojów i ustalania skrętu.

### Punkty kalibracji i wielomian

Oś pozioma przedstawia kąt wyznaczony z ruchu robota, a pionowa — zadaną surową nastawę serwa. Linia pokazuje dopasowany wielomian. Kąty na wykresie są w stopniach dla czytelności, ale współczynniki odnoszą się **do radianów**.

### Błędy dopasowania

Wykres pokazuje `przewidziane rawServo − zadane rawServo`. Wyniki powinny skupiać się wokół zera. Systematyczny kształt lub duży rozrzut może wskazywać na niedopasowany model, luzy, poślizg albo niestabilne warunki pomiaru.

### Zgodność przód–tył i powtarzalność

Dla każdej nastawy pokazane są średnie kąty i odchylenia standardowe między powtórzeniami, osobno dla obu kierunków. Duże różnice między kierunkami oznaczają, że jeden wielomian może nie opisywać wystarczająco dobrze całej charakterystyki.

### Miary jakości

| Pole | Interpretacja |
| --- | --- |
| `rmse_raw` | Błąd dopasowania wszystkich zaakceptowanych odcinków. |
| `cross_validation_rmse_raw` | Błąd predykcji po kolejnym pomijaniu całej nastawy serwa; sprawdza zachowanie poza punktami użytymi do dopasowania. Limit to dwukrotność `max_fit_rmse`. |
| `forward_reverse_difference_rad` | Największa różnica średnich kątów przód–tył dla tej samej nastawy. |
| `angle_range_rad` | Zakres kątów rzeczywiście objętych pomiarami. |
| `raw_range` | Zakres zbadanych nastaw serwa. |
| `condition_number` | Wskaźnik stabilności numerycznej dopasowania; program odrzuca wartość powyżej `1000`. |

Program wymaga co najmniej `0.20 rad` rozpiętości zidentyfikowanych kątów, pomiarów po obu stronach jazdy na wprost i monotonicznego wielomianu w zmierzonym zakresie. Nie ocenia poprawności modelu poza tym zakresem.

## 8. Zastosowanie i zapis kalibracji

Po zakończeniu pomiarów robot powinien stać, a moduł nie powinien prowadzić aktywnego przebiegu. Sprawdź, czy raport oznacza zakończenie identyfikacji bez błędu.

W terminalu ustaw zmienną na **rzeczywisty, zaakceptowany katalog pomiarowy**, np. zastąp poniższą ścieżkę właściwą nazwą przebiegu:

```bash
RUN_DIR=/home/student/ros2_workspace/calibration_results/NAZWA_PRZEBIEGU
ros2 param load /mtrackercar "$RUN_DIR/steering_calibration.yaml"
ros2 param get /mtrackercar steering_calibration
```

Samo załadowanie parametrów nie zmienia kalibracji firmware. Wyślij wszystkie współczynniki jedną transakcją:

```bash
ros2 service call /set_steering_calibration std_srvs/srv/Trigger '{}'
```

Wymagane jest `success: true`. Kalibracja działa teraz w RAM firmware. Aby zapisać ją na stałe:

```bash
ros2 service call /save_configuration std_srvs/srv/Trigger '{}'
```

Usługa zatrzymuje napęd, wykonuje dwa kolejne żądania `CMD=8`, a następnie odpytuje o zakończenie zapisu. `success: true` oznacza potwierdzony status zakończenia. Zapis obejmuje całą konfigurację firmware przechowywaną w EEPROM, także inne parametry znajdujące się w tym rekordzie.

Przy błędzie lub timeout sprawdź komunikat odpowiedzi; timeout może oznaczać nieznany wynik operacji. Nie zakładaj wtedy, że zapis na pewno się udał albo na pewno nie nastąpił.

### Powrót do normalnej jazdy

Zakończ launch kalibracji. W głównym `config/mtrackercar.yaml` ustaw:

- `steering_mode: angle`;
- `max_steering_angle` mieszczący się w zmierzonym zakresie po obu stronach zera oraz w ograniczeniach mechanicznych;
- opcjonalnie zapisz zaakceptowane `steering_calibration` w konfiguracji projektu, żeby zachować spójny zapis parametrów po stronie ROS.

Przykład: dla zakresu `[-0.50, +0.45] rad` symetryczny limit nie powinien przekraczać `0.45 rad`. Zmiana parametrów geometrii i trybu skrętu wymaga ponownego uruchomienia sterownika.

```bash
ros2 launch mtrackercar mtrackercar_master.launch.py
```

Normalny sterownik domyślnie korzysta z `drive_command`
(`ackermann_msgs/msg/AckermannDriveStamped`): `drive.speed` w m/s oraz
`drive.steering_angle` jako fizyczny kąt zastępczego koła przedniej osi w radianach.
Wymagany jest bieżący, niezerowy `header.stamp`; `frame_id` może być pusty
lub równy nazwie układu robota. Firmware stosuje swoją
zapisaną kalibrację. Surowe sterowanie jest dostępne wyłącznie przez
`command_source=calibration_raw` i `calibration/raw_command`.

Przykład publikacji komendy fizycznej (uruchamia napęd):

```bash
ros2 topic pub --rate 20 /drive_command ackermann_msgs/msg/AckermannDriveStamped \
  '{header: auto, drive: {speed: 0.05, steering_angle: 0.20}}'
```

Po zakończeniu publikacji sterownik zatrzymuje napęd przez timeout komend.
`header: auto` aktualizuje czas przy każdej publikacji w ROS 2 Jazzy.
Sterownik wykorzystuje tylko prędkość i kąt; pola przyspieszenia, jerk
i prędkości skrętu pozostaw zerowe — ich ograniczenia nie są realizowane.
Przyjęta kalibracja z 2026-10-31 objęła zakres `[-0.2630, +0.3289] rad`,
czyli około `[-15.1°, +18.8°]`. Domyślny limit jazdy wynosi symetrycznie
`±0.26 rad` (`±14.9°`). To zakres użytkowy mieszczący się w pomiarach,
a nie wyznaczenie mechanicznych ograniczników. Po zmianie konfiguracji
robota kalibrację i ten limit trzeba sprawdzić ponownie.
Dla istniejących kontrolerów publikujących `cmd_vel` wybierz jawnie tryb zgodności:

```bash
ros2 launch mtrackercar mtrackercar_master.launch.py command_source:=cmd_vel
```

W tym trybie `angular.z` nadal oznacza prędkość obrotu pojazdu w rad/s;
sterownik przelicza ją na fizyczny kąt. Zwykła jazda zawsze używa trybu
skalibrowanego kąta w firmware. Samo uruchomienie sterownika ani wpisanie
współczynników do YAML nie wysyła `CMD=13` i nie zapisuje EEPROM.

Trwałość kalibracji po wyłączeniu zasilania wymaga sprawdzenia na rzeczywistym robocie. Wykonane testy programowe nie zastępują tej próby.

## 9. Ponowna identyfikacja offline

Zapisane dane można przeliczyć ponownie bez jazdy i bez połączenia z robotem:

```bash
ros2 run mtrackercar steering_calibration \
  --offline "$RUN_DIR/poses.csv" \
  --wheel-base 0.190 \
  --output-dir /home/student/ros2_workspace/calibration_results/reanalysis
```

Użyj faktycznego `wheel_base` badanego robota. Powstanie nowy katalog wyników. Plik CSV zawiera już pozycję skorygowaną o offset punktu pomiarowego, dlatego nie należy przeliczać offsetu drugi raz.

Tryb CLI offline używa domyślnych progów jakości. Przy przeliczaniu danych zebranych z innymi progami należy dopasować argumenty funkcji w module `calibration_identification.py`. Niepełny przebieg może nie spełniać warunków identyfikacji.

## 10. Najczęstsze komunikaty

| Komunikat / objaw | Co sprawdzić |
| --- | --- |
| `Set enable_motion=true…` | Uruchom launch z `enable_motion:=true`; potem wywołaj usługę `start`. |
| `Fresh odom pose is required` | Temat, typ wiadomości, aktualność znaczników czasu, nazwy układów i quaternion. |
| `Confirmed driver serial communication is required` | Port szeregowy, numer robota, firmware i `/diagnostics`. |
| `Start driver with command_source=calibration_raw` | Czy sterownik jest uruchomiony w trybie dedykowanego wejścia surowego. |
| `Pose feedback lost` | Ciągłość publikacji odom i próg `pose_timeout`. |
| `Driver serial feedback lost` | Błędy komunikacji i aktualność diagnostyki sterownika. |
| `Insufficient travel…` | Czy robot się porusza, czy odom to rejestruje i czy odcinek jest wystarczająco długi. |
| Robot cofa tylko raz i zatrzymuje się | Sprawdź końcowy błąd w terminalu launch i `report.yaml` rzeczywistego przebiegu. Jeśli komunikat zawiera `motion opposite to commanded direction`, porównaj kierunek komendy, rzeczywisty ruch i znak drogi z odom. Po aktualizacji sterownika przebuduj pakiet i uruchom launch ponownie. |
| `Curvature is not constant…` | Czas ustalania skrętu, luzy mechaniczne, poślizg i stabilność pomiaru. |
| `Forward/reverse steering disagreement…` | Rozbieżność kierunków, geometrię odometrii i powtarzalność mechaniki. |
| `Calibration … budget exceeded` | Zaplanowaną liczbę odcinków oraz limity czasu, drogi i promienia. |
| `Cubic prediction error…` | Wykres błędów, zakres próbek i zgodność modelu z pomiarem. |

Nie zwiększaj progów jakości wyłącznie po to, aby otrzymać współczynniki. Najpierw ustal przyczynę odrzucenia i sprawdź raport częściowy.

Katalog wyników jest domyślnie względny wobec katalogu uruchomienia launch.
Jeśli launch uruchomiono z `/home/student`, dane znajdziesz w
`/home/student/calibration_results`, a nie w workspace. Pełna ścieżka może być
ustawiona argumentem `output_dir`. Raport `synthetic_example_…/report.html`
nie pokazuje ostatniego przejazdu robota.
