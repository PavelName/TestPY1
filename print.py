print('Hello!')

# from pioneer_sdk import Pioneer
# import time

# pioneer = Pioneer()

# def test_drone():
#     print("Проверка связи с дроном...")
#     # Можно добавить запрос телеметрии для проверки соединения
    
#     print("Арминг (включение моторов)...")
#     pioneer.arm()
#     time.sleep(1)
    
#     print("Взлёт на 1 метр...")
#     pioneer.takeoff()
    
#     # Ждём достижения высоты (в реальных миссиях лучше использовать point_reached)
#     time.sleep(3)
    
#     print("Повисание 2 секунды...")
#     time.sleep(2)
    
#     print("Посадка...")
#     pioneer.land()
#     time.sleep(2)
    
#     print("Дизарминг (выключение моторов)...")
#     pioneer.disarm()
#     print("Тест завершён.")

# if __name__ == "__main__":
#     try:
#         test_drone()
#     except KeyboardInterrupt:
#         print("Аварийная остановка, посадка...")
#         pioneer.land()
#         time.sleep(2)
#         pioneer.disarm()

# def mean(numbers):
#     if not numbers:
#         return None
#     return sum(numbers) / len(numbers)

# data = [10.2, 10.5, 9.8, 11.0, 10.3]
# avg = mean(data)
# print(avg)

# import cv2
# import numpy as np
# from pysimverse import Simulator, Drone

# # 1. Инициализация симулятора и дрона
# sim = Simulator()
# drone = Drone()
# sim.add_drone(drone)

# # Создадим простой объект-цель (красный шар) в сцене
# target_pos = (10, 10, 2)  # x, y, z
# sim.create_sphere(position=target_pos, radius=0.5, color=(0, 0, 255))  # BGR: красный

# sim.start()

# def find_target(frame):
#     """
#     Ищет красный объект на кадре с помощью цветовой сегментации.
#     Возвращает центр объекта (cx, cy) или None, если не найден.
#     """
#     hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
#     # Диапазон красного цвета (учитываем оба конца спектра HSV)
#     lower_red1 = np.array([0, 70, 50])
#     upper_red1 = np.array([10, 255, 255])
#     lower_red2 = np.array([170, 70, 50])
#     upper_red2 = np.array([180, 255, 255])
    
#     mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
#     mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
#     mask = cv2.bitwise_or(mask1, mask2)
    
#     # Морфологические операции для очистки шума
#     kernel = np.ones((5,5), np.uint8)
#     mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
#     mask = cv2.dilate(mask, kernel, iterations=2)
    
#     contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
#     if not contours:
#         return None
    
#     # Берём самый большой контур
#     c = max(contours, key=cv2.contourArea)
#     if cv2.contourArea(c) < 500:  # игнорируем слишком мелкие объекты
#         return None
        
#     x, y, w, h = cv2.boundingRect(c)
#     cx, cy = x + w//2, y + h//2
#     return (cx, cy), mask

# # 2. Основной цикл симуляции
# while sim.is_running():
#     frame = drone.get_camera_frame()  # получаем кадр с камеры дрона
    
#     result = find_target(frame)
    
#     if result is not None:
#         (cx, cy), _ = result
#         h, w = frame.shape[:2]
#         center_x, center_y = w // 2, h // 2
        
#         # Вычисляем смещение относительно центра кадра
#         dx = cx - center_x
#         dy = cy - center_y
        
#         # Пропорциональное управление (P-контроллер)
#         # Чем дальше объект от центра, тем сильнее корректируем движение
#         k = 0.002  # коэффициент усиления
#         vx = -k * dx  # движение влево/вправо
#         vy = k * dy   # движение вверх/вниз (инверсия оси)
#         vz = 0        # пока не меняем высоту
        
#         # Если объект почти в центре — снижаем скорость и зависаем
#         if abs(dx) < 20 and abs(dy) < 20:
#             vx = 0
#             vy = 0
#             vz = -0.1  # медленно снижаемся для «захвата»
#             print("Объект захвачен! Зависание над целью...")
#             # Здесь можно добавить логику сброса маркера или триггера
#         else:
#             drone.set_velocity(vx, vy, vz)
#     else:
#         # Объект не найден — продолжаем полёт по маршруту или ждём
#         print("Объект не обнаружен. Поиск...")
#         drone.set_velocity(0, 0, 0)  # можно добавить патрулирование
    
#     sim.step()  # шаг симуляции

# sim.stop()

# def find_all_position(text: str, substring: str) -> list[int]:
#     position = []
#     start = 0
#     while True:
#         ind = text.find(substring, start)
#         if ind == -1:
#             break
#         position.append(ind)
#         start = ind + 1
#     return position

# text = 'ejjgkljjgf'
# substring = 'jg'

# print(find_all_position(text, substring))

# def argenda(sums):
#     if not sums:
#         return 0
#     return sum(sums) / len(sums)

# numbers = [12, 34, 45, 67, 87]

# print(argenda([1]))
# print(argenda([6]))

# total_len = 0
# for _ in range(3):
#     word = input()
#     total_len = total_len + len(word)

# print(total_len)

# counter = 0
# for _ in range(5):
#     word = input()
#     if 'р' in word:
#         counter = counter + 1

# print(counter)

# temp = int(input())
# x = int(input())
# y = int(input())

# temp = x
# x = y
# y = temp

# print(temp)
# print(x)
# print(y)
# a = 15
# b = 26
# c = 17

# a, b, c = c, a, b

# print(a, b, c)

# total = 0
# for i in range(1, 10):
#     if i % 2 == 1:
#         total += i

# print(total)

# total1 = 0
# total2 = 0

# for _ in range(5):
#     num = int(input())

#     if 2 <= num < 5:
#         total1 += num

#     if abs(num) <= 3:
#         total2 += num

# print(total1, total2)

# num1 = 4
# num2 = 6
# num1 += num2
# num1 *= num1

# print(num1)

# total = 0
# for i in range(1, 6):
#     total += i
#     print(total, end='')

# flag = False
# for _ in range(5):
#     word = input()

#     if len(word) < 5:
#         flag = True

# print(flag)

# flag1 = False
# flag2 = False

# for _ in range(5):
#     word = input()
    
#     if word == 'Плутон':
#         flag1 = True
#     if 'тер' in word:
#         flag2 = True

# print(flag1)
# print(flag2)

# a = int(input())
# b = int(input())

# count = 0
# for num in range(a, b + 1):
#     if (num ** 3) % 10 in (4, 9):
#         count += 1

# print(count)

# n = int(input())
# count = 0

# for _ in range(n):
#     nums = int(input())
#     count += nums

# print(count)

# import math

# n = int(input())
# harm_sum = 0.0

# for i in range(1, n + 1):
#     harm_sum += 1 / i

# result = harm_sum - math.log(n)
# print(result)

# n = int(input())
# total = 0

# for i in range(5, n + 1, 10):
#     total += i
# print(total)

# import math

# n = int(input())

# print(math.factorial(n))

# prod = 1

# for _ in range(10):
#     num = int(input())
#     if num != 0:
#         prod *= num

# print(prod)

# n = int(input())
# total = 0

# for i in range(1, n + 1):
#     if n % i == 0:
#         total += i

# print(total)

all_even = True

# for _ in range(10):
#     num = int(input())
#     if num % 2 != 0:
#         all_even = False
#         break
# if all_even:
#     print("YES")
# else:
#     print("NO")

# n = int(input())
# totoal = 0

# for i in range(1, n + 1):
#     if i % 2 == 1:
#         totoal += i
#     else:
#         totoal -= i

# print(totoal)

# n = int(input())

# first = -1
# second = -1

# for _ in range(n):
#     x = int(input())
#     if x > first:
#      second = first
#      first = x
#     elif x > second:
#        second = x

# print(first)
# print(second)

# n = int(input())

# if n >= 1:
#     a = 1
#     print(a, end=' ')

# if n >= 2:
#     b = 1
#     print(b, end=' ')
# for _ in range(3, n + 1):
#     c = a + b
#     print(c, end=' ')
#     a, b = b, c

# print()
       
# i = 1
# num = int(input())
# while i <= num:
#     print(i)
#     i += 1

# count = 3

# print(input())

# while count > 0:
#     print(count)
#     count -= 1




# count = 6
# i = input()
# print(i)

# while count > 0:
#     print(count)
#     count -= 2N

# ansver = ''

# while ansver != 'хватит':
#     ansver = input("напиши чтонибудь (или 'хватит' , чтобы выйти!)")

# print("вход выполнен!")

# n = int(input())
# total = 0
# i = 1

# while i <= n:
#     total += i
#     i += 1
# print(f" сумма чисел от 1 до {n} равна {total}")


# Проверка пороля!


# name = input()
# counter = 0

# while 'S' not in name:
#     counter += 1
#     name = input()

# print(counter)
# num = int(input())
# total = 0

# while abs(num) <= 5:
#     total += num
#     num = int(input())

# print(total)

# i = 5
# while i <= 11:
#     print('Python awesome!')
#     i += 1
# num = int(input())
# counter = 0

# while '0' not in str(num):
#     counter += 1
#     num = int(input())

# print(counter)

# num = int(input())
# counter = 0

# while '0' not in str(num):
#     counter += 1

# print(counter)

# num = int(input())
# total = 0

# while num > -4:
#     num = int(input())
#     total += num

# print(total)

# i = 7
# a = 5
# while i <  11:
#     a += i
#     i += 2

# print(a)

# total = 1
# while total < 10:
#     num =int(input())
#     total *= num
#     print(total)

# import torch
# # x = torch.tensor([-1, 3, 2], dtype=torch.float32)
# # y = x + 5
# # z = y * 2

# # print(z)

# print(torch.__version__) 
# print("CUDA доступна:", torch.cuda.is_available())

# import torch

# # Создаём тензор
# x = torch.tensor([1, 2, 3], dtype=torch.float32)
# print("Тензор:", x)
# print("Устройство:", x.device)

# # Простая операция (как в твоём примере)
# y = x + 5
# z = y * 2
# print("Результат операций:", z)

# import torch
# x = torch.tensor([1, 2, 3])

# print(x)
# print(type(x))

# import numpy as np
# import torch

# np_array = np.array([7, 8, 9])

# print(f"NumPy array:, np.array")

# num = int(input())
# while num > 0:
#     last_digin = num % 10
#     num //= 10
#     print(last_digin, sep='=', end='')


# num = int(input())
# while num > 0:
#     last_digit = num % 10
#     if last_digit % 2 == 0:
#         print(last_digit)
#     num //= 10

# num = 12345
# prod = 1
# while num != 0:
#     last_digirt = num % 10
#     prod *= last_digirt
#     num //= 10

# print(prod)

# num = 123456789
# total = 0
# while num != 0:
#     last_digit = num % 10
#     if last_digit > 4:
#         total += 1

#     num = num // 10
# print(total)

# num = 725
# while num != 0:
#     last_digit = num % 10
#     num //= 10
#     print(last_digit, sep='', end='$')

# num = 586
# while num > 0:
#     last_digit = num % 10
#     print(last_digit, sep='*', end='#')
#     num //= 10
#     print()

# n = 12345
# while n > 0:
#     gigit = n % 10
#     print(gigit)
#     n //= 10

# n = int(input())
# reverse_n = 0
# while n > 0:
#     digits = n % 10
#     reverse_n = reverse_n * 10 + digits
#     n //= 10

# print(reverse_n)

# number = int(input())
# max_digit = -1
# min_digit = 10

# while number > 0:
#     digit = number % 10
#     if digit > max_digit:
#         max_digit = digit
#     if digit < min_digit:
#         min_digit = digit
#     number //= 10

# print('Максимальная цифра равна', max_digit )
# print('Минимальная цифра равна', min_digit )

# n = int(input())

# original_n = n

# sum_digit = 0
# count_digit = 0
# prod_digit = 1

# while n > 0:
#     digit = n % 10
#     sum_digit += digit
#     prod_digit *= digit
#     count_digit += 1
#     n //= 10

# average = sum_digit / count_digit

# first_digit = int(str(original_n)[0])
# last_digit = original_n % 10

# sum_first_last = first_digit + last_digit

# print(sum_digit)
# print(count_digit)
# print(prod_digit)
# print(average)
# print(first_digit)
# print(sum_first_last)

# n = int(input())

# first_digit = n % 10

# all_same = True
# while n > 0:
#     digin = n % 10
#     if digin != first_digit:
#         all_same = False
#         break
#     n //= 10

# if all_same:
#     print("Yes")
# else:
#     print("No") 

# n = int(input())
# prev = -1
# is_sorted = True

# while n > 0:
#     digit = n % 10
#     if prev != 0 and digit < prev:
#         is_sorted = False
#         break
#     prev = digit
#     n //= 10

# if is_sorted:
#     print('YES')
# else:
#     print("NO")

# n = input()

# count = 0

# for digit_char in n:
#     digit = int(digit_char)
#     if digit % 2 == 0 and digit != 0:
#         count += 1
#         print(f"{count}-я четная цифра равна {digit}")

# if count == 0:
#     print('Четных цифр в числе нет')

# num = 3281
# while num != 0:
#     print(num % 10, end= '')
#     num //= 100
#     if num != 0:
#         continue

# num = 364
# while num != 0:
#     print(num % 10, end='')
#     if num % 10 == '3':
#         break
#     num //= 10

# print()
# print('Program the end!')

# for i in range(10):
#     print(i, end='*')
#     if i > 6: 
# #break

# i = 100
# while i > 0:
#     if i == 40:
#         break
#     print(i, end='*')
#     i -= 20

# n = 10
# while n > 0:
#     n -= 1
#     if n == 2:
#         continue
#     print(n, end='*')

# result = 0
# for i in range(10):
#     if i % 2 == 0:
#         continue
#     result += i

# print(result)

# mult = 1
# for i in range(1, 11):
#     if i % 2 == 0:
#         continue
#     if i % 9 == 0:
#         break1
#     mult *= i

# print(mult)

# n = int(input())

# for i in range(2, n + 1):
#     if n % i == 0:
#         print(i)
#         break
# num = 3
# while num < 8:
#     num += 1
# else:
#     print('Цикл завершен.')

# print(num)

# a = "13356"
# d = 4
# #print(a > d)
# print(len(a))

# n = int(input())
# if n % 2 == 0:
#     print(n, '-четное число!')
# else:
#     print(n, '-нечетное число!')

# num = 3
# total = 0
# for i in range(num):
#     if i  % 2 == i:
#         total += 1
# else:
#     print(total)

# print(total + 1)

# num = 4
# while num < 10:
#     num += 2
#     print(num)
# else:
#     print('Loop the tnd')

num = 7
# while num < 12:
#     num +=2
#     if num == 11:
#         break
#     print(num)
# else:
#     print('Loop the end!')

# num = 6
# while True:
#     num += 1
#     if num >= 5:
#         break
#     print(num)
# else:
#     print('Loop the end!')

# for i in range(5):
#     print(str(i) * 2)
#     if i >= 2:
#         break
# f = range(2, 10, 5)
# print(list(f))

# saleries = {
#     'John': 1200,
#     'Vary': 500,
#     'Svetenr': 10000,
#     'Lizen': 8000

# }
# res = saleries['John']
# res_1 = saleries['Vary']

# print(res)
# print(res_1)

# n = int(input())
# total = 0
# for _ in range(n):
#     num = int(input())
#     total += num
# print(total)
# print(total / n if n > 0 else 0)

# 

# def scuare(number):
#     res = number ** 2
#     return res
# print(scuare(int(input())))

# n = int(input())
# lines = []
# for _ in range(n):
#     lines.append(input())

# for i in range(len(lines) -1, -1, -1):
#     print(lines[i])

# for i in range(2):
#     for j in range(3):
#         print('C' * i + '+' * j)      
  
# for i in range(1, 2):
#     print(i * 'C')

#     for j in range(2, 4):
#         print(j, '@')

#     for k in range(3, 1, -1):
#         print(k * '%')

# print()

# for i in range(1, 4):
#     for j in range(3, 6):
#         print(i, j)

for i in range(2):
    print(i, end='*')
    for j in range(2):
        print('/', end='+')
