# Badanie pomiaru napięcia NUC11TNKi5 — 2026-10-31

## Odczyt potwierdzony i docelowa usługa

Wynik `nuc_ec_results/20261031_190006_734129.json` potwierdził sygnaturę
`NUC_EC` i typ 10 (`DC_IN`) w slocie `0x434`. Rzeczywiste odczyty z pola
`0x435..0x436` wyniosły 18.252, 18.588 oraz 18.596 V.
DIMM pokazał 1.200 V; kanał typu 11 pozostaje nierozpoznany i nie jest
wykorzystywany do decyzji o zasilaniu. Zewnętrzny czujnik nie jest potrzebny
do uzyskania odczytu EC na tym egzemplarzu. Zgodność napięcia z multimetrem
pozostaje do sprawdzenia, podobnie jak właściwy próg ochrony.

Docelowy monitor jest napisany w C++17, bez ROS i Pythona podczas pracy.
Pełna dokumentacja instalacji, autostartu i badania innych modeli znajduje się
w `nuc_power_monitor/README.md` w katalogu głównym workspace.
Poniższe sekcje dokumentują wcześniejsze etapy badania i ich ograniczenia.

## Fakty z tego komputera

- Komputer Intel NUC11TNKi5, płyta NUC11TNBi5.
- BIOS `TNTGL357.0064.2022.0217.1550`, data 2022-02-17.
- Kernel `7.0.0-42-generic`.
- Brak urządzeń w `/sys/class/power_supply`; UPower pokazuje wyłącznie
  wirtualne `DisplayDevice`, bez baterii i pomiaru napięcia.
- hwmon: `acpitz`, `nvme`, `coretemp`, `iwlwifi_1`; brak kanałów napięcia.
- Kontroler EC: `PNP0C09:00`, ścieżka ACPI `\_SB_.PC00.LPCB.H_EC`.
- Obecne magistrale DesignWare, SMBus I801 oraz magistrale ekranu i915.
  Ich obecność nie dowodzi obecności dostępnego ADC zasilania.
- Nie wykonywano skanowania I2C, zapisu do EC, zmiany BIOS ani wyłączania PC.

## Nowy trop: mapa EC VA

Projekt VirtualSMC wymienia dokładnie rodzinę `TNTGL357` i płytę NUC11TNBi5
jako `Intel_EC_VA`. W kodzie tej rodziny istnieją sześć slotów napięcia,
identyfikator pamięci `NUC_EC` oraz rozróżnienie typu napięcia `DC_IN = 10`.
To znacznie mocniejszy trop niż ogólne stwierdzenie specyfikacji o monitorowaniu
napięcia. Nadal nie jest dowodem, że konkretny firmware wystawia DC_IN.

Źródła pierwotne:

- [Powiązanie modelu i rodziny EC](https://github.com/acidanthera/VirtualSMC/blob/master/Docs/EmbeddedControllers.md)
- [Mapa rejestrów EC VA i typy napięć](https://github.com/acidanthera/VirtualSMC/blob/master/Sensors/SMCSuperIO/ECDeviceNUC.hpp)
- [Nazwanie kanału DC_IN jako VIN](https://github.com/acidanthera/VirtualSMC/blob/master/Sensors/SMCSuperIO/ECDeviceNUC.cpp)
- [Dostęp MMIO i rejestr LPC LGMR](https://github.com/acidanthera/VirtualSMC/blob/master/Sensors/SMCSuperIO/ECDevice.cpp)

## Co jest jeszcze niepotwierdzone

Nie wykonano odczytu identyfikatora EC, konfiguracji kanałów ani samego napięcia.
Nie znamy jeszcze poprawnej wartości napięcia wejściowego z tego interfejsu.
Nie należy podłączać niezweryfikowanych danych do automatycznego wyłączania.
Wcześniejszy wniosek, że zewnętrzny czujnik jest konieczny, był przedwczesny.

## Przygotowany następny krok

### Wynik analizy po zebraniu tablic

Zestaw `power_audit/20261031_185641_935430` zawiera DSDT i 10 tablic SSDT,
bez błędów odczytu. Rejestr LPC LGMR ma wartość `0xfe410001`: okno MMIO jest
włączone pod adresem `0xfe410000`. DSDT niezależnie deklaruje dokładnie ten
sam region: `H2RA`, `SystemMemory`, baza `0xFE410000`, długość `0x0F00`.
Definiuje pola bajtowe `R400` i następne, obejmujące sloty napięcia z mapy VA.

Próba wspólnej dekompilacji wszystkich tablic zgłosiła zduplikowane nazwy
firmware. Oddzielna dekompilacja DSDT przez ACPICA zakończyła się poprawnie.
Nie wykonywano żadnej metody AML.

Przygotowano niezależny od ROS czytnik `tools/nuc_ec_read.py`.
Weryfikuje model, rodzinę BIOS i dokładny LGMR, otwiera `/dev/mem` tylko
do odczytu i mapuje jedną stronę bez uprawnienia zapisu. Odczytuje wyłącznie
6 bajtów identyfikatora i sześć znanych 3-bajtowych pól napięcia, trzykrotnie.
Wymaga sygnatury `NUC_EC`; kanał wejściowy identyfikuje po typie 10,
nie po podobieństwie napięcia. Interpretacja wartości: big-endian, miliwolty.
Dekoder przeszedł trzy testy syntetyczne, w tym odrzucenie złej sygnatury
i nieprzypisywanie nieznanych kanałów do DC_IN.

Uruchomienie następnego odczytu lokalnie:

```bash
sudo python3 /home/student/ros2_workspace/tools/nuc_ec_read.py
```

Wynik trafi do nowego pliku JSON w `nuc_ec_results/`. Skrypt nie potrafi
wyłączyć PC ani zmienić ustawień EC. Jeśli kernel zablokuje `/dev/mem`,
zakończy się błędem; nie zmienia parametrów kernela ani zabezpieczeń.
Na razie odczyt sprzętowy pozostaje niewykonany: autoryzacja `sudo` z terminala
operatora nie jest dostępna w sesji agenta (`sudo -n` nadal wymaga hasła).
Po uzyskaniu DC_IN trzeba porównać pomiar z multimetrem, zanim użyje się go
do automatycznego wyłączania.

Tablice ACPI można przeanalizować offline: nie wymaga to wykonywania ich metod
ani skanowania magistral. Odczyt DSDT/SSDT jest ograniczony do administratora.
Próba `sudo -n` zakończyła się `sudo: a password is required`.

Przygotowano niezależny od ROS skrypt kopiujący tablice oraz inwentarz:

```bash
sudo python3 /home/student/ros2_workspace/tools/nuc_power_audit.py
```

Skrypt zapisze nowy katalog pod `/home/student/ros2_workspace/power_audit/`,
bez nadpisywania istniejących wyników. Czyta także rejestr konfiguracyjny LPC
LGMR opisujący okno MMIO; nie czyta pamięci EC ani `/dev/mem` i nic nie zapisuje
do sprzętu. Hasło wpisuje się lokalnie do `sudo`, nigdy do czatu.

Narzędzia ACPICA pobrano bez instalowania do `/tmp/nuc_power_audit/tools/`.
Po skopiowaniu tablic można zdekompilować kopie przez `iasl` i sprawdzić
definicje regionów EC/MMIO oraz metody ACPI/WMI. Dopiero po identyfikacji
interfejsu należy zaplanować minimalny odczyt znanych kanałów oraz porównanie
wyniku z multimetrem. Brak pola napięcia w ACPI nie wyklucza pomiaru w MMIO EC.
