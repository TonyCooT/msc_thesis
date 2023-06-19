## Планирование движения мобильного робота в социальной среде с обучением с подкреплением

Репозиторий содержит инструкции и файлы к практической части магистерской диссертации

### Добавление плагина в Gazebo

Ниже представлен порядок сборки плагина из исходного кода и его добавление в Gazebo.
Команду добавления необходимо выполнять в каждом новом терминале

```bash
    cd ~/gazebo_plugin_tutorial/model_move
    mkdir build
    cd build
    cmake ../
    make
    export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:~/gazebo_plugin_tutorial/model_move/build
```

### Обучение алгоритма

Ниже представлен порядок запуска обучения алгоритма в виртуальном окружении Conda в операционной системе Windows 10

```bash
    cd C:\test\rl_collision_avoidance\
    conda create --name test python=3.7
    conda activate test
    сd C:\test\rl_collision_avoidance\gym-collision-avoidance
    python -m pip install -e C:\test\rl_collision_avoidance\gym-collision-avoidance
    python -m pip install git+https://github.com/openai/baselines.git@ea25b9e8b234e6ee1bca43083f8f3cf974143998
    cd C:\test\rl_collision_avoidance\
    python -m pip install -r requirements.txt
    python -m pip install -e ga3c
    pip install protobuf==3.20.*
    cd C:\test\rl_collision_avoidance\ga3c\GA3C
    SET GYM_CONFIG_CLASS=TrainPhase1
    SET GYM_CONFIG_PATH=C:\test\rl_collision_avoidance\ga3c\GA3C\Config.py
    python Run.py
```

P.S. Для обеспечения полной совместимости и функционала необходимо доработать файлы
`Run.py` (функция), `NetworkVPCore.py` (wandb, пути), `ProcessStats.py` (txt), `vizualize.py` (пути, анимация) и `test_cases.py` (RVO)