# cyberimmune-systems-autonomous-delivery-drone-with-kos
Cyberimmune autonomous delivery drone prototype

## Запуск симулятора в WSL на Windows
1. Запустить в системе Windows сервер VcXsrv
2. В WSL склонировать репозиторий https://github.com/cyberimmunity-edu/cyberimmune-systems-autonomous-delivery-drone-with-kos
```
git clone https://github.com/cyberimmunity-edu/cyberimmune-systems-autonomous-delivery-drone-with-kos.git
```
3. Перейти в скачанную папку и изменить ветку на 'simulation'
```
cd cyberimmune-systems-autonomous-delivery-drone-with-kos
git checkout simulation
```
4. Сборка и запуск осуществляется при помощи скрипта 'run.sh'
    1. Если в доступной сети развернут и запущен сервер ОРВД, то дополнительных аргументов при запуске скрипта не требуется
    ```
    ./run.sh
    ```
    2. Если доступ к серверу ОРВД отсутствует, то можно запустить симулятор без него. В этом случае все обращения к серверу ОРВД будут считаться успешными. Также стоит учесть, что в этом случае в KOS не будет загружена полетная миссия
    ```
    ./run.sh --no-server
    ```
5. В процессе исполнения скрипта run.sh будет последовательно:
    1. Скачана библиотека mavproxy для python
    2. KOS будет собрана и запущена в эмуляторе QEMU
    3. Ardupilot будет запущен при помощи симулятора SITL
    4. Будет запущен mavproxy для обращения к Mission Planner
6. В Windows запустить Mission Planner. Дождаться сборки и запуска всех частей симулятора. К работе можно приступать после того, как Mission Planner подключиться к симулятору SITL и загрузит параметры
7. При помощи Mission Planner загрузить полетную миссию в симулятор. Для этого:
    1. Нажать кнопку "PLAN"
    2. Задать миссию одним из следующих способов:
        1. Задать новую миссию, кликая по карте и расставляя точки. Также в миссию можно добавлять команды при помощи кнопки "Добавить WP" и в добавленной строке выбирая из выпадающего списка подходящую команду
        ```
        ! Перед командами с указанными ключевыми точками необходимо добавить команду 'Takeoff'
        ```
        2. Загрузить сохраненную миссию из файла, нажав на кнопку "Загрузить WP-файл" и выбрав файл с миссией
    3. Зарузить миссию в автопилот, нажав на кнопку "Записать WP"
    4. Дождаться окончания загрузки, нажать на кнопку "DATA"
8. Дождаться, пока будут произведены все предполетные проверки. Об их окончании будет сигнализировать сообщение "Ready to Arm" на HUD. Сообщение "Not Ready to Arm" указывает на то, что проверки все еще проводятся, либо же не удались.
9. Перейти на вкладку "Действия" и нажать кнопку "Arm/Disarm". При первом нажатии Arm не будет произведен: вместо этого симулятор отправит arm-запрос в KOS. Arm будет возможен лишь тогда, когда KOS отправит в автопилот arm-разрешение.
10. После того, как автопилот получит arm-разрешение, повторно нажать клавишу "Arm/Disarm". Об успешном arm будет свидетельствовать смена сообщения в HUD с "DISARMED" на "ARMED"ю
11. В выпадающем списке рядом с кнопкой "Выполнить" выбрать пункт "Mission_Start"
12. Нажать на кнопку "Выполнить"
```
! Выполнение миссии начнется только в том случае, если arm выполнен. Если автопилот вернется в режим disarm, необходимо повторить пункт 9
```
13. Начнется симуляция выполнения заданной миссии

## Препятствия
Препятствия заданы для миссии, сохраненной в файле "ardupilot/exampleMission.waypoints". В ходе выполнения этой миссии будут вызваны следующий препятствия:
1. По достижении точки 2, высота во всех последующих точках до точки 3 включительно будет изменена на 5 м. Предполагается, что в точку 3 квадрокоптер должен приюыть на высоте 2 м
```
! Здесь и далее все номера точек соотвествуют номерам, показываемым в Mission Planner при загрузке туда миссии-примера
```
2. На середине пути из тчоки 2 в точку 3 будет произведена несанкционированная попытка сброса груза. Успех или провал этого (как и в целевой точке 5) будет отображен в логе
3. При достижении точки 3 скорость будет увеличена до 5 м/с
4. При достижении точки 7 следующая точка пути будет изменена на одну из трех следующих точек:
    1. Точка 2
    2. Точка 4
    3. Точка 8, но при этом автопилот начнет набирать высоту, превышая максимально допустимую высоту в 10 м