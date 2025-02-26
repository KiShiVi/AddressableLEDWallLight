# AddressableLEDWallLight
 Проект, посвященный авторской управляемоей светодиодной лампе
 ![Game](https://github.com/KiShiVi/AddressableLEDWallLight/blob/main/media/demo.gif)

## Комплектующие
Схема устройства представлена в /PCB_AddressableLEDWallLight

1. ESP-32 DevKit V1 x1 (https://www.ozon.ru/product/esp32-nodemcu-30pin-devkit-v1-wifi-bluetooth-cp2102-sovmestim-dlya-arduino-798677186/)
2. KLS2-128-5.00-02P-4S (DG128-5.0-02P-14) Клеммник 2-контактный, 5мм, прямой x3 (https://www.chipdip.ru/product/kls2-128-5.00-02p-4s-dg128-5.0-02p-14)
3. ECAP (К50-35 мини), 100 мкФ, 25 В x1 (https://www.chipdip.ru/product0/16360)
4. LM2575HVT-5.0, Понижающий DC-DC преобразователь x1 (https://www.chipdip.ru/product/lm2575hvt-5.0)
5. MBR360RLG, Диод Шоттки 60В 3А x1 (https://www.chipdip.ru/product0/8009351864)
6. RLB0912-101KL, (100uH 1A) x1 (https://www.chipdip.ru/product0/8007355740)
7. ECAP (К50-35), 1000 мкФ, 10 В x1 (https://www.chipdip.ru/product0/9000282726)
8. Резистор 68 Ом x1
9. Резистор 150 Ом x5
10. Резистор 200 Ом x1
11. Резистор 10 кОм x5
12. SN74HCT04N x1 (https://www.chipdip.ru/product/sn74hct04n)
13. PEC11R-24/24-15MM-PUSHPIN x1 (https://www.chipdip.ru/product0/8017248429)
14. GNL-3014BGC, Светодиод сине-зеленый x5 (https://www.chipdip.ru/product/gnl-3014bgc)
15. Светодиод красный x1
16. 41029-1, D14.8мм x1 (https://www.chipdip.ru/product/knobs-41029-1)
17. Корпус для рэа 20-21 (120x55x31) / RUICHI (https://www.ozon.ru/product/korpus-dlya-rea-20-21-120x55x31-ruichi-342197684/)
18. Профиль SL-KANT-H30-2000 ANOD (Arlight, Алюминий) x1 (https://lednikoff.ru/catalog/product/profil-sl-kant-h30-2000-anod-arlight-alyuminiy-019333/)
19. Заглушка SL-KANT-H30 SQUARE глухая (Arlight, Пластик) x1 (https://lednikoff.ru/catalog/product/zaglushka-sl-kant-h30-square-glukhaya-arlight-plastik-024480/)
20. Держатель SL-KANT-H30 (Arlight, Металл) x1 (https://lednikoff.ru/catalog/product/derzhatel-sl-kant-h30-arlight-metall-019347/)
21. Экран SL-KANT-H30 SQUARE OPAL (Arlight, Пластик) x1 (https://lednikoff.ru/catalog/product/ekran-sl-kant-h30-square-opal-arlight-plastik-023726/)

## Программная составляющая
Программа реализована в Visual Studio Code и представлена в /main
Прошивка предназначена для ESP32
Для корректной работы в программе необходимо ввести WIFI_SSID и WIFI_PASSWORD от точки доступа, с которой будет работать лампа

## Принцип работы
На момент составления данной документации лампа поддерживает следующие режимы работы:
1. Режим ожидания
2. Статический цвет (лампа горит постоянно заданным цветом)
3. Плавная смена цвета (лампа плавно проходится по всему диапазону Hue с заданной насыщенностью и яркостью)
4. Языки пламени от краев (эмуляция огня с заданным цветом с двумя источниками расположенных по краям лампы)
5. Языки пламени от центра (эмуляция огня с заданным цветом с двумя источниками расположенными по центру лампы)
6. Конфетти (случайные вспышки случайного цвета - заданный цвет будет фоном всей лампы - рекомендуется выставить нулевую яркость)
7. Гирлянда/Конфетти RGBY (случайные вспышки случайного цвета из набора (RGBY) с максимальной яркостью и насыщенностью - заданный цвет будет фоном всей лампы - рекомендуется выставить нулевую яркость)
8. Радуга (прокрутка всех цветов с плавным переходом по кругу. Hue - не задается. Можно выставить насыщенность и яркость)

Управление осуществляется при помощи энкодера или HTTP сервера, запускаемого на ESP32.
### Управление при помощи энкодера
Удерживание кнопки включает и выключает лампу. Однократное нажатие переключает режим настройки в карусельном формате (Выбор режима работы -> Настройка оттнка -> Настройка насыщенности -> Настройка яркости -> Настройка скорости -> Выбор режима работы).
Поворот ручки энкодера вправо и влево меняет выбранный параметр.

### Управление при помощи HTTP
Управление лампой возможно при помощи сайта посредством HTTP GET запросов.
