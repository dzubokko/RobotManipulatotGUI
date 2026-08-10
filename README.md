# Robot Manipulator GUI

Графическое приложение для управления и моделирования 6-осевого робота-манипулятора на базе **ROS 2 Humble**, **PyQt6** и **Gazebo**.

Проект предоставляет единый интерфейс для ручного управления манипулятором, выполнения программ движения, работы с прямой и обратной кинематикой, управления захватом и запуска симуляции.

## Возможности

* управление 6 суставами манипулятора;
* перемещение TCP по координатам X, Y и Z;
* изменение ориентации рабочего органа;
* прямые и обратные кинематические расчёты;
* линейные перемещения;
* управление двухпальцевым захватом;
* запуск и остановка Gazebo из GUI;
* редактор пользовательских программ;
* сохранение и загрузка программ;
* выполнение последовательностей движений;
* программная остановка движения и Emergency Stop;
* отображение текущего состояния робота.

## Технологии

* Ubuntu 22.04
* ROS 2 Humble
* Python 3
* PyQt6
* Gazebo
* ros2_control
* JointTrajectoryController
* URDF
* colcon



### `robot_arm_controller`

Основной пакет приложения.

Содержит:

* графический интерфейс;
* управление движением;
* кинематику;
* управление захватом;
* выполнение программ;
* взаимодействие с ROS 2;
* управление запуском Gazebo.

### `robot_description`

Описание конструкции робота:

* URDF-модель;
* звенья и суставы;
* meshes;
* launch-файлы.

### `robot_gazebo`

Конфигурация симуляции:

* Gazebo launch-файлы;
* ros2_control;
* конфигурация контроллеров;
* виртуальное окружение.

## Требования

Перед установкой необходимо установить:

* Ubuntu 22.04;
* ROS 2 Humble;
* Gazebo;
* Python 3;
* colcon;
* rosdep.

PyQt6 можно установить через pip:

```bash
python3 -m pip install PyQt6
```

## Установка

Клонируйте репозиторий:

```bash
git clone https://github.com/dzubokko/RobotManipulatotGUI.git
cd RobotManipulatotGUI
```

Подключите ROS 2:

```bash
source /opt/ros/humble/setup.bash
```

Установите зависимости:

```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

Соберите workspace:

```bash
colcon build --symlink-install
```

Подключите собранный workspace:

```bash
source install/setup.bash
```

## Запуск

Основной способ запуска приложения:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

python3 -m robot_arm_controller.app
```

После запуска откроется главное окно приложения.

Gazebo можно запустить непосредственно из GUI.

При необходимости симуляцию также можно запустить отдельно:

```bash
ros2 launch robot_gazebo gazebo.launch.py
```

## Интерфейс

Приложение включает три основных раздела.

### Ручное управление

Позволяет:

* управлять суставами J1–J6;
* перемещать TCP;
* изменять ориентацию;
* управлять захватом;
* возвращать робот в Home-положение;
* изменять скорость движения;
* отслеживать состояние ROS 2 и робота.

### Редактор программ

Позволяет создавать и выполнять программы движения манипулятора.

Поддерживаются команды:

```text
move_lin(...)
rotate_rx(...)
rotate_ry(...)
rotate_rz(...)
joint_set(...)
reset_home(...)
wait(...)
grip_open(...)
grip_close(...)
grip_set(...)
stop_motion()
```

Пример программы:

```python
reset_home(2.0)

grip_open(0.8)

move_lin(0.05, 0.0, 0.0, 1.0)
move_lin(0.0, 0.05, 0.0, 1.0)
move_lin(0.0, 0.0, -0.03, 0.8)

grip_close(0.8)

move_lin(0.0, 0.0, 0.03, 0.8)

rotate_rz(45, 1.0)

grip_open(0.8)

reset_home(2.0)
```

### Менеджер программ

Используется для работы с сохранёнными программами:

* открытие;
* просмотр;
* запуск;
* удаление;
* редактирование.

## Кинематика

Проект содержит собственный модуль кинематики манипулятора.

Реализованы:

* Forward Kinematics;
* Inverse Kinematics;
* расчёт положения TCP;
* расчёт ориентации TCP;
* линейные движения;
* преобразование между суставным и декартовым пространством.

## Управление захватом

Используется двухпальцевый захват с максимальным раскрытием:

```text
25 мм
```

Поддерживаются:

```python
grip_open()
grip_close()
grip_set(0.012)
```

## Диагностика

Проверить доступные ROS 2-ноды:

```bash
ros2 node list
```

Проверить топики:

```bash
ros2 topic list
```

Проверить состояние суставов:

```bash
ros2 topic echo /joint_states
```

Проверить контроллеры:

```bash
ros2 control list_controllers
```

## Повторная сборка

После изменения исходного кода:

```bash
colcon build --symlink-install
source install/setup.bash
```

После этого приложение можно запустить снова:

```bash
python3 -m robot_arm_controller.app
```

## Возможные проблемы

### ROS 2 не видит пакеты

Убедитесь, что workspace был собран и подключён:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

### Gazebo не запускается повторно

Проверьте оставшиеся процессы:

```bash
ps aux | grep -E "gazebo|gzserver|gzclient"
```

При необходимости завершите их:

```bash
pkill -f gzserver
pkill -f gzclient
pkill -f gazebo
```

### Нет данных `/joint_states`

Проверьте наличие топика:

```bash
ros2 topic list | grep joint
```

и состояние контроллеров:

```bash
ros2 control list_controllers
```

## Безопасность

Emergency Stop в приложении является **программной функцией**.

При использовании проекта с реальным промышленным оборудованием необходимо использовать отдельную аппаратную систему аварийной остановки.

## Лицензия

Проект распространяется под лицензией **Apache License 2.0**.

## Автор

**dzubokko**

GitHub: `github.com/dzubokko`
