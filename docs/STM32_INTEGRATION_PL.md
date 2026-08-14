# Integracja torque vectoringu ze STM32

## Co skopiować do projektu

Do projektu STM32CubeIDE należy dodać:

```text
c_implementation/torque_vectoring.c
c_implementation/torque_vectoring.h
c_implementation/config.h
```

`torque_vectoring.c` należy dodać do źródeł kompilowanych, a katalog
`c_implementation` do ścieżek nagłówków. Kod wymaga C11, ale nie wymaga HAL,
CMSIS-DSP, `math.h`, `libm`, sterty ani konkretnego modelu STM32.

## Jednostki interfejsu

| Wartość | Typ | Jednostka/przykład |
|---|---|---|
| Wychylenie maglownicy | `int32_t` | `+70 mm` w lewo, `-70 mm` w prawo |
| Dostępność maglownicy | `bool` | `false` oznacza brak świeżego pomiaru |
| Prędkość | `uint32_t` | `5000 mm/s` = `5 m/s` |
| Pedał | `int32_t` | domyślnie `0–256` |
| Komendy silników | `int32_t` | ten sam zakres co pedał |

Zakres `0–256` ma 257 możliwych wartości, więc nie mieści się w `uint8_t`.
Do transmisji należy użyć co najmniej `uint16_t` albo zmienić maksimum na `255`.

## Przykład użycia w pętli sterującej

```c
#include "torque_vectoring.h"

static VehicleParameters vehicle;

void torque_vectoring_init(void)
{
    vehicle = tv_default_vehicle();
}

void torque_vectoring_step(void)
{
    int32_t rack_mm = 0;
    uint32_t speed_mmps = 0U;
    int32_t pedal = 0;

    const bool rack_available = steering_sensor_read_mm(&rack_mm);
    speed_mmps = vehicle_speed_read_mmps();
    pedal = pedal_read_command();

    const WheelCommands result = tv_calculate_rear_commands_from_rack(
        &vehicle,
        rack_available,
        rack_mm,
        speed_mmps,
        pedal
    );

    if (result.status == TV_OK) {
        rear_left_motor_set(result.rear_left);
        rear_right_motor_set(result.rear_right);
    } else {
        rear_left_motor_set(vehicle.command_min);
        rear_right_motor_set(vehicle.command_min);
        torque_vectoring_report_fault(result.status);
    }
}
```

Funkcję należy wywoływać ze stałym okresem, np. co `10 ms`. Nie ma ona stanu
wewnętrznego, więc częstotliwość nie wpływa na sam wynik. Filtrowanie czujników,
kontrolę timeoutu i ograniczenie szybkości zmian komendy należy wykonać w warstwie
aplikacji.

## Fallback i błędy

- brak świeżego pomiaru maglownicy (`rack_available == false`) → oba koła dostają
  wartość pedału,
- wychylenie `0 mm` → oba koła dostają wartość pedału,
- `TV_LATERAL_GRIP_EXCEEDED` lub inny błąd → oba wyjścia w strukturze mają
  `command_min`; aplikacja nie powinna podawać momentu,
- wartość maglownicy pomiędzy `1–4 mm` nie jest fallbackiem, tylko błędem zakresu;
  dokładnie `0 mm` ma specjalne znaczenie.

## FPU, math.h i arm_math

Aktualna implementacja czasu rzeczywistego nie wykonuje obliczeń
zmiennoprzecinkowych. Nie zawiera `math.h` i nie linkuje `libm`, dlatego kwestia,
czy dana funkcja biblioteczna korzysta z FPU, nie dotyczy tej ścieżki.

`arm_math` z CMSIS-DSP nie jest potrzebne do dzielenia momentu. Może być przydatne
później do filtracji czujników lub bardziej rozbudowanego modelu, ale nie należy
zastępować nim obecnych prostych obliczeń całkowitoliczbowych bez pomiaru czasu
wykonania i błędu numerycznego na docelowym MCU.

## Geometria

![Geometria Perły](vehicle_geometry.svg)

- `L = 2750 mm` — odległość między środkami osi,
- `t = 1700 mm` — odległość między środkami lewego i prawego koła,
- `h = 511 mm` — wysokość środka masy nad nawierzchnią,
- `x_c = -40 mm` — środek masy jest 40 mm przed środkiem rozstawu osi.
