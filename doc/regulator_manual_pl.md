# Ustawianie regulatora napędu

Usługa `/set_regulator` sterownika `mtrackercar_drv` wysyła `CMD=4`.
Dotyczy regulatora napędu w firmware, a nie regulatora śledzenia trajektorii ROS.
Firmware udostępnia cztery parametry, a nie klasyczne Kp, Ki, Kd:

| Pole usługi | Pole firmware |
| --- | --- |
| `feedback_bandwidth` | `feedbackBandwidth` |
| `observer_bandwidth` | `observerBandwidth` |
| `damping` | `damping` |
| `inertia` | `inertia` |

Dokumentacja aktualnego firmware podaje domyślne nastawy CAR: pasmo sprzężenia
`15.0 rad/s`, pasmo ESO `80.0 rad/s`, tłumienie `1.0`, bezwładność `0.0025`.
Pełny protokół określa zakresy: sprzężenie `1..200`, ESO od pasma sprzężenia
do `1000`, tłumienie `0.1..5`, bezwładność `0.00001..1`. Moduł wymaga wartości skończonych i dodatnich; firmware może je
dodatkowo odrzucić. Nie wyznacza wzmocnień automatycznie.

Do automatycznego dobierania tych czterech parametrów służy osobny moduł
`regulator_tuning`: [instrukcja strojenia](regulator_experiments_pl.md)
([HTML](regulator_experiments_pl.html)). Domyślnie używa telemetrii CMD=5;
nie wymaga dostawcy próbek RAM.

## Uruchomienie i zmiana

Przebuduj pakiet (w tym nowy interfejs usługi):

```bash
cd /home/student/ros2_workspace
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to mtrackercar --symlink-install
source install/setup.bash
```

Zakończ poprzedni launch przez `Ctrl+C`. Uruchom zwykły sterownik:

```bash
ros2 launch mtrackercar mtrackercar_master.launch.py
```

Usługa jest również dostępna w sterowniku uruchomionym przez launch kalibracji;
nie uruchamiaj wtedy drugiego sterownika na tym samym porcie szeregowym.

W drugim terminalu załaduj środowisko i sprawdź interfejs:

```bash
source /opt/ros/jazzy/setup.bash
source /home/student/ros2_workspace/install/setup.bash
ros2 interface show mtrackercar/srv/SetRegulator
```

Zakończ aktywną kalibrację i zatrzymaj źródła komend jazdy przed zmianą.
Usługa zatrzymuje napęd przed transakcją, ale kolejne komendy jazdy mogą go
ponownie uruchomić.

Wstaw swoje cztery wartości w poniższe pola. Symbole są miejscami do zastąpienia,
nie proponowanymi nastawami:

```bash
ros2 service call /set_regulator mtrackercar/srv/SetRegulator \
  '{feedback_bandwidth: TWOJE_FB, observer_bandwidth: TWOJE_OBS, damping: TWOJE_D, inertia: TWOJE_J}'
```

`success: true` oznacza, że robot zaakceptował wszystkie cztery wartości i odesłał
te same nastawy. `active_values` zawiera wartości zwrócone przez firmware w kolejności
z tabeli; przy niepowodzeniu mogą być niedostępne i pozostać zerowe.
Timeout oznacza nieznany wynik zmiany.

Domyślne nastawy z dokumentacji można wysłać jawnie:

```bash
ros2 service call /set_regulator mtrackercar/srv/SetRegulator \
  '{feedback_bandwidth: 15.0, observer_bandwidth: 80.0, damping: 1.0, inertia: 0.0025}'
```

`observer_bandwidth` zmienia ESO regulatora silnika. Osobny obserwator prędkości
z pozycji enkodera ma stałe pasmo `20 rad/s`, działa z częstotliwością `1 kHz`
i nie jest konfigurowany przez `CMD=4`. Firmware nie udostępnia parametru
kompensacji martwej strefy ani tarcia statycznego.

Nie ma przycisku przywracania poprzednich wartości. Zachowaj znane, działające nastawy przed zmianą, aby móc ponownie
wysłać je tą samą usługą. `active_values` to wynik ostatniej transakcji,
nie niezależny odczyt ani historia ustawień.

Usługa zmienia ustawienia w RAM. Nie wysyła nic przy starcie sterownika.
Opisany protokół nie definiuje osobnego żądania odczytu regulatora, więc moduł
nie oferuje odczytu istniejących nastaw bez ich zmiany.

## Zapis trwały

Po sprawdzeniu działania zapisz konfigurację osobno:

```bash
ros2 service call /save_configuration std_srvs/srv/Trigger '{}'
```

Wymagane jest `success: true`. Operacja zapisuje całą konfigurację firmware,
w tym kalibrację skrętu. Testy programowe nie potwierdzają zachowania konkretnego
robota ani trwałości po wyłączeniu zasilania.

## Diagnostyka

| Objaw lub komunikat | Co zrobić |
| --- | --- |
| Nieznany typ `mtrackercar/srv/SetRegulator` | Przebuduj pakiet i wykonaj `source install/setup.bash` w terminalu klienta. |
| Oczekiwanie na `/set_regulator` | Sprawdź, czy działa nowa wersja sterownika; przy namespace dostosuj nazwę usługi. |
| `Regulator parameters must be finite and positive` | Podaj wszystkie cztery wartości liczbowe większe od zera. Żądanie nie zostało wysłane. |
| `No confirmed serial connection` | Sprawdź port, adres robota i `/diagnostics`. |
| `Regulator rejected, status=…` | Firmware odrzuciło żądanie; sprawdź wartości i opis statusów w `protocol.md`. |
| `Firmware returned different active regulator parameters` | Sprawdź `active_values`; moduł nie potwierdził żądanych nastaw. |
| Timeout lub nieprawidłowa odpowiedź | Wynik może być nieznany. Nie traktuj go jako potwierdzenia ani gwarancji pozostawienia poprzednich nastaw. |

Powiązane dokumenty: [kalibracja skrętu](calibration_manual_pl.md),
[protokół komunikacji](protocol.md).
