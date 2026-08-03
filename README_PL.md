# Torque vectoring w C

[English](README.md) · **Polski**

Program rozdziela żądanie kierowcy między lewe i prawe koło tylnej osi. Przyjmuje:

- promień skrętu `[m]` — dodatni dla skrętu w lewo, ujemny dla skrętu w prawo,
- prędkość `[m/s]`,
- nacisk pedału jako liczbę całkowitą `0–256`.

Zwraca dwie całkowite komendy: dla lewego i prawego tylnego koła. Przepływ jest
prosty: program odczytuje skręt, prędkość i pedał, oblicza rozkład obciążenia,
a następnie skaluje żądanie pedału osobno dla obu kół.

## Konfiguracja zakresu

Domyślnie pedał i komendy kół używają tego samego zakresu `0–256`. Zmienia się
go w pliku `c_implementation/config.h`:

```c
#define TV_CONFIG_COMMAND_MIN 0
#define TV_CONFIG_COMMAND_MAX 100
```

Po tej zmianie wejście pedału i oba wyjścia działają w zakresie `0–100`.
`command_min` oznacza brak żądanego momentu, a `command_max` pełne żądanie.
Wartości spoza skonfigurowanego zakresu zwracają `TV_INVALID_ARGUMENT`.

## Uruchomienie

```sh
make
./build/torque-vectoring 6.5 5.0 128
make test
```

Przykładowy wynik dla skrętu w lewo:

```text
Rear left command:  66
Rear right command: 190
```

## Na czym polega problem

![Schemat pojazdu podczas skrętu i promieni toru kół](old/Screenshot%202026-03-09%20083548.png)

Na zakręcie koło zewnętrzne pokonuje dłuższą drogę i jest mocniej dociskane do
drogi. Koło wewnętrzne jest odciążane. Dlatego obu kołom nie należy zawsze
zadawać identycznej wartości.

## Obliczenia

Przyspieszenie i siła poprzeczna:

```text
a_y = v² / |R|
F_y = m · a_y
```

Przybliżone przeniesienie obciążenia:

```text
ΔF_z = F_y · h / t
F_z,inner = F_z,rear/2 - ΔF_z
F_z,outer = F_z,rear/2 + ΔF_z
```

Program wyznacza udział obu kół z ich dostępnego momentu `μ · F_z · r_w`.
Wartość pedału jest poziomem bazowym dla każdego koła podczas jazdy prosto.
Na zakręcie komenda koła wewnętrznego maleje, a zewnętrznego rośnie. Wynik jest
zaokrąglany i ograniczany do zakresu z konfiguracji. Po przekroczeniu limitu
`F_y > μmg` obie komendy przyjmują wartość `command_min`.

Jeżeli przy mocno wciśniętym pedale jedna z komend przekroczyłaby `command_max`,
program skaluje **obie** wartości tym samym współczynnikiem. Zachowuje dzięki
temu wyliczoną proporcję między kołami. Łączne żądanie momentu zostaje wtedy
zmniejszone, ponieważ ważniejsze jest nieprzekroczenie zakresu i zachowanie
podziału momentu.

## Pliki

- `c_implementation/torque_vectoring.h` — typy i publiczne funkcje,
- `c_implementation/config.h` — zakres wejścia i wyjść,
- `c_implementation/torque_vectoring.c` — obliczenia,
- `c_implementation/main.c` — wejście i wyjście programu,
- `c_implementation/test_torque_vectoring.c` — testy.

To uproszczony model edukacyjny. Nie uwzględnia m.in. koła tarcia opony,
dynamiki przejściowej, charakterystyki silników i poślizgu kół.
