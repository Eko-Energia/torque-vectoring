# # def licznik_wywolan(func):
# #     def wrapper(*args, **kwargs):
# #         wrapper.licznik += 1
# #         return func(*args, **kwargs)
# #     wrapper.licznik = 0
# #     return wrapper

# # # Użycie dekoratora
# # @licznik_wywolan
# # def funkcja_testowa():
# #     print(funkcja_testowa.licznik)
    

# # # Przykład użycia
# # funkcja_testowa()
# # funkcja_testowa()
# # funkcja_testowa()

# # print("Liczba wywołań funkcji:", funkcja_testowa.licznik)


# # T=[[1,20,10],[1,3,4],[2,4,6],[5,8,9]]
# # T=sorted(T, key=lambda x: x[1])
# # print(*T, sep='\n')


# # def funkcja(**a):
# #     print(a)
# #     for i in a:
# #         print(i)
# #         print(a[i])
# #     return a

# # funkcja(a=1,b=2,c=3)

# # def funkcja_2(*a):
# #     print(a)
# #     return a


# # def suma(a,b):
# #     return a+b
# # def antysuma(a,b):
# #     return a-b+1

# # def cos(a,b,fun):
# #     return fun(a,b)
# # print(cos(1,2,suma))
# # print(cos(1,2,antysuma))


# # def read_txt_to_array(file_path):
# #     # Inicjalizacja pustej tablicy
# #     array_of_strings = []

# #     # Otwarcie pliku tekstowego
# #     with open(file_path, 'r') as file:
# #         # Iteracja po każdej linii w pliku
# #         for line in file:
# #             # Podział linii na elementy
# #             elements = line.strip().split(',')
# #             # Dodanie elementów do tablicy
# #             array_of_strings.append(elements)

# #     return array_of_strings

# # # Ścieżka do pliku tekstowego
# # file_path = 'sciezki_obieraki.txt'

# # def liczenie(file):
# #     T=read_txt_to_array(file_path)
# #     cunter=0
# #     for i,j in T:
# #         if (j[0]=="L" or j[1]=="L" or j[0]=="D" or j[1]=="D") and j[2]=="N" and i[0]=="A":
# #             cunter+=1
# #     return cunter



# # # print(liczenie(file_path))

# # T=[2,4,6]
# # print(3 in T)

# # A=[1,2]
# # A[0]
# # def find_cubic_solution(limit):
# #     for a in range(1, limit):
# #         for b in range(a, limit):
# #             c = round((a**3 + b**3)**(1/3))
# #             if a**3 + b**3 == c**3:
# #                 return a, b, c
# #     return None

# # result = find_cubic_solution(10000)
# # print(result)


# def hanoi(n, source, auxiliary, target):
#     if n == 1:
#         print(f"Przenieś dysk 1 z {source} do {target}")
#         return
#     hanoi(n - 1, source, target, auxiliary)
#     print(f"Przenieś dysk {n} z {source} do {target}")
#     hanoi(n - 1, auxiliary, source, target)

# # Przykład użycia
# n = 15  # liczba dysków
# #hanoi(n, 'A', 'B', 'C')

# def hanoi_moves(n):
#     if n <= 0:
#         return 0
#     # Tablica do przechowywania wyników dla różnych liczby dysków
#     dp = [0] * (n + 1)
    
#     # Dla 1 dysku potrzeba 1 ruchu
#     dp[1] = 1
    
#     for i in range(2, n + 1):
#         dp[i] = 2 * dp[i - 1] + 1
    
#     return dp[n]

# # Przykład użycia
# n = 12  # liczba dysków
# print(f"Liczba ruchów wymaganych do przeniesienia {n} dysków: {hanoi_moves(n)}")



# import neopixel
# from machine import Pin
# import time

# ws_pin = 0
# led_num = 8
# BRIGHTNESS = 0.1  # Adjust the brightness (0.0 - 1.0)

# neoRing = neopixel.NeoPixel(Pin(ws_pin), led_num)
# speed=10
# neoRing.fill((250,0,0))
# def set_brightness(color):
#     r, g, b = color
#     r = int(r * BRIGHTNESS)
#     g = int(g * BRIGHTNESS)
#     b = int(b * BRIGHTNESS)
#     return (r, g, b)
# def rgb(color):
#     r,g,b=color
#     if r!=0 and b==0:
#         r-=speed
#         g+=speed
#         return r,g,b
#     if g!=0 and r==0:
#         g-=speed
#         b+=speed
#         return r,g,b
#     if b!=0 and g==0:
#         b-=speed
#         r+=speed
#         return r,g,b
# def loop():

#     for j in range(8):
            
#         neoRing[j]=rgb(neoRing[j])
#     speed.sleep(0.1)



# while True:
#     loop()


#print((True+True+True)**(True+True+True))
#print(chr(sum(range(ord(min(str(not())))))))
#print(sum(range(ord(min(str(not()))))))
#print(chr(3486))

# import re
# n=1000000
# print(not re.match(r'^.?$|^(..+?)\1+$','1'*n))


# from random import uniform
# i=1
# słownik=set()
# while True:
#     rand=uniform(0,1)
#     if rand in słownik:
#         print(i)
#         break
#     słownik.add(rand)
#     i+=1
#     if i%10000000==0:print(i)

# import math

# # Parametry
# rozstaw_osi = 2.75  # w metrach
# rozstaw_kol = 1.7   # w metrach
# promien_skretu_min = 6.5  # w metrach

# # Promienie kół wewnętrznego i zewnętrznego
# promien_wewn = promien_skretu_min - rozstaw_kol / 2
# promien_zewn = promien_skretu_min + rozstaw_kol / 2

# # Kąty skrętu (w radianach)
# kat_wewn_radiany = math.atan(rozstaw_osi / promien_wewn)
# kat_zewn_radiany = math.atan(rozstaw_osi / promien_zewn)

# # Konwersja na stopnie
# kat_wewn_stopnie = math.degrees(kat_wewn_radiany)
# kat_zewn_stopnie = math.degrees(kat_zewn_radiany)

# print(kat_wewn_stopnie, kat_zewn_stopnie)

import math

def calculate_torque_with_front_rear(mass, h, xc, wheelbase, track_width, mu, turn_radius, velocity, wheel_diameter):
    """
    Oblicza momenty napędowe na wewnętrzne i zewnętrzne koło tylnej osi w zakręcie,
    uwzględniając rozkład sił przód-tył oraz siły odśrodkowe, z dodanym warunkiem poślizgu całego pojazdu.

    :param mass: Masa pojazdu (kg)
    :param h: Wysokość środka ciężkości (m)
    :param xc: Przesunięcie środka ciężkości wzdłuż osi pojazdu (m)
    :param wheelbase: Rozstaw osi (m)
    :param track_width: Rozstaw kół (m)
    :param mu: Współczynnik tarcia opony
    :param turn_radius: Promień zakrętu (m)
    :param velocity: Prędkość pojazdu (m/s)
    :param wheel_diameter: Średnica koła (m)
    :return: Moment na koło wewnętrzne (Nm), moment na koło zewnętrzne (Nm), Czy pojazd w poślizgu (True/False)
    """
    g = 9.81  # Przyspieszenie grawitacyjne (m/s^2)
    
    # Promień koła
    wheel_radius = wheel_diameter / 2
    
    # Siła odśrodkowa
    f_centrifugal = (mass * velocity ** 2) / turn_radius
    
    # Maksymalna siła tarcia całego pojazdu
    max_total_grip = mu * (mass * g)
    
    # Sprawdzenie, czy pojazd wpada w poślizg
    if f_centrifugal > max_total_grip:
        return 0, 0, False  # Pojazd w poślizgu, momenty ustawione na 0
    
    # Rozkład obciążenia przód-tył
    f_front = (mass * g / 2) - (mass * g * xc / wheelbase)
    f_rear = (mass * g / 2) + (mass * g * xc / wheelbase)
    
    # Maksymalna siła przyczepności osi tylnej
    f_rear_grip = f_rear * mu
    
    # Rozkład sił na lewe i prawe koła tylne
    f_left_rear = (f_rear / 2) + (f_centrifugal * h / track_width)
    f_right_rear = (f_rear / 2) - (f_centrifugal * h / track_width)
    
    # Maksymalne siły przyczepności dla kół
    f_max_left_rear = f_left_rear * mu
    f_max_right_rear = f_right_rear * mu
    
    # Maksymalne momenty na koła tylne
    m_left_rear = f_max_left_rear * wheel_radius
    m_right_rear = f_max_right_rear * wheel_radius
    
    return m_right_rear, m_left_rear, True  # Zwracamy momenty i brak poślizgu



m_right, m_left, is_safe = calculate_torque_with_front_rear(
    mass=700, h=0.511, xc=-0.04,
    wheelbase=2.75, track_width=1.7, mu=0.8,
    turn_radius=6.5, velocity=5, wheel_diameter=0.7
)

if not is_safe:
    print("Pojazd wpadł w poślizg! Zmniejsz prędkość.")
else:
    print(f"Moment na wewnętrzne koło: {m_left:.2f} Nm")
    print(f"Moment na zewnętrzne koło: {m_right:.2f} Nm " ,m_left+m_right)





