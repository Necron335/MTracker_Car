# Monitor zasilania PC NUC

**Ten dokument opisuje wcześniejszy wariant korzystający z ROS.** Dla
sprawdzonego NUC11TNKi5 dostępny jest już niezależny monitor C++ odczytujący
własny EC komputera. Dokumentacja docelowej usługi: `nuc_power_monitor/README.md`
w katalogu głównym workspace. Nie wymaga działającego drivera robota.

`pc_power_monitor` obserwuje napięcie z `/diagnostics` sterownika robota.
Ma to zastosowanie do ochrony PC tylko wtedy, gdy pomiar opisuje akumulator
zasilający NUC. Pomiar na wyjściu przetwornicy o stałym napięciu może nie
informować o rozładowaniu pakietu. NUC nie musi mieć własnego czujnika napięcia.

## Stan początkowy

Konfiguracja `config/pc_power_monitor.yaml` ma `shutdown_voltage: 0.0`
i `enable_shutdown: false`: próg nie jest ustalony i komputer nie zostanie
wyłączony. Około 17 V nie wystarcza do określenia typu pakietu ani progu.
Potwierdź oznaczenie akumulatora, wspólne zasilanie i sposób zasilania NUC.
Próg musi pozostawiać zapas na przetwornicę i czas zamknięcia systemu.

Monitor wymaga działającego drivera oraz zgodnych `ROS_DOMAIN_ID` i ustawień DDS.
Nie otwiera sam portu szeregowego. Wybiera nazwę diagnostyki i `hardware_id`
z konfiguracji. Nie uruchamiaj drugiego drivera tylko na potrzeby monitora.

## Uruchomienie obserwacji

```bash
source /opt/ros/jazzy/setup.bash
source /home/student/ros2_workspace/install/setup.bash
ros2 run mtrackercar pc_power_monitor --ros-args \
  --params-file /home/student/ros2_workspace/src/MTrackerCar-ROS/config/pc_power_monitor.yaml
```

Podgląd:

```bash
ros2 topic echo /power_monitor/status
```

Po ustawieniu właściwego progu pozostaw najpierw `enable_shutdown: false`.
Monitor zapisze `DRY RUN`, kiedy warunki nakazywałyby wyłączenie, bez
uruchamiania polecenia systemowego. Nie trzeba rozładowywać prawdziwego
akumulatora do testowania logiki: dostępne są testy syntetyczne w pakiecie.

## Decyzja o wyłączeniu

Świeże, dodatnie i skończone napięcie z potwierdzonej komunikacji musi spaść
poniżej `shutdown_voltage`. Stan niski jest podtrzymywany do wzrostu powyżej
`shutdown_voltage + hysteresis_voltage`. Po `low_duration_s` rozpoczyna się
odliczanie `shutdown_delay_s`. Powrót napięcia ponad próg z histerezą anuluje
odliczanie. Utrata pomiarów przerywa decyzję: brak danych nie oznacza 0 V.

Stany: `unconfigured`, `unknown`, `normal`, `low`, `countdown`, `shutdown`.
Ostatnio zmierzone napięcie może pozostać widoczne przy braku danych;
sprawdzaj także stan. Odłączenie USB nie wyłącza automatycznie komputera.

Po jawnym ustawieniu `enable_shutdown: true` końcowa decyzja uruchamia:
`sudo -n /usr/bin/systemctl poweroff`, bez wymuszania wyłączenia.
To zamknięcie systemu operacyjnego, a nie fizyczne odłączenie pakietu.
Ochrona ogniw nadal wymaga właściwego BMS/odcięcia zasilania.
Monitor nie zastępuje UPS; przy nagłym zaniku zasilania może zabraknąć czasu
na zamknięcie. Nie oblicza procentu naładowania ani napięć pojedynczych ogniw.

## Uprawnienie i autostart (do instalacji po doborze progu)

Przygotowany plik `config/pc-power-monitor.service` jest usługą użytkownika
`student`, nie jest obecnie zainstalowany ani włączony.
Do pracy bez logowania potrzebne jest `loginctl enable-linger student`.
W przypadku innego użytkownika zmień ścieżki w pliku.

Administrator może dodać przez `visudo` wyłącznie uprawnienie do konkretnego
polecenia (weryfikując lokalizację `systemctl`):

```sudoers
student ALL=(root) NOPASSWD: /usr/bin/systemctl poweroff
```

Nie nadawaj ogólnego `NOPASSWD: ALL`. Bez tego uprawnienia próba wyłączenia
zakończy się błędem w logu; monitor nie używa interaktywnego pytania o hasło.

Instalacja usługi użytkownika:

```bash
mkdir -p ~/.config/systemd/user
cp /home/student/ros2_workspace/src/MTrackerCar-ROS/config/pc-power-monitor.service ~/.config/systemd/user/
systemctl --user daemon-reload
systemctl --user enable --now pc-power-monitor.service
journalctl --user -u pc-power-monitor.service -f
```

Ustawienia są czytane przy starcie; po zmianie zrestartuj usługę.
Wyłączenie autostartu: `systemctl --user disable --now pc-power-monitor.service`.
