#!/bin/bash

# Создаем новую сессию tmux с именем 'bcn3d_moveo_session' и запускаем первый процесс
tmux new-session -d -s bcn3d_moveo_session "source /home/danil/ros2_bcn3d_moveo/install/setup.bash; ros2 launch bcn3d_moveo_moveit bcn3d_moveo_joy.launch.py"

# Даем время на инициализацию первого процесса
sleep 12

# Создаем новую панель и запускаем второй процесс
tmux split-window -t bcn3d_moveo_session -h
tmux send-keys -t bcn3d_moveo_session "source /home/danil/ros2_bcn3d_moveo/install/setup.bash; ros2 run py_pubsub go_go_camera & ros2 run py_pubsub go_go_fsr_topic" C-m

# Даем время на инициализацию второго процесса
sleep 5

# Создаем еще одну панель и запускаем третий процесс
tmux split-window -t bcn3d_moveo_session -v
tmux send-keys -t bcn3d_moveo_session "source /home/danil/ros2_bcn3d_moveo/install/setup.bash; ros2 run py_pubsub go_go_bcn3d_tcp" C-m

# Даем время на инициализацию третьего процесса
sleep 3

# Создаем еще одну панель и запускаем четвертый процесс
tmux split-window -t bcn3d_moveo_session -v
tmux send-keys -t bcn3d_moveo_session "source /home/danil/ros2_bcn3d_moveo/install/setup.bash; ros2 run py_pubsub go_go_sub" C-m

# Даем время на инициализацию четвертого процесса
sleep 3

# Создаем последнюю панель и запускаем пятый процесс
tmux split-window -t bcn3d_moveo_session -v
tmux send-keys -t bcn3d_moveo_session "source /home/danil/ros2_bcn3d_moveo/install/setup.bash; ros2 run bcn3d_moveo_scripts go_go_to_the_point  " C-m

# Чтобы присоединиться к сессии tmux и увидеть все процессы, используйте:
# tmux attach-session -t bcn3d_moveo_session

# Чтобы закрыть сессию tmux и все ее процессы, используйте:
# tmux kill-session -t bcn3d_moveo_session