# Sim Bot

Pacote ROS 2 para simulacao de robos moveis diferenciais no Gazebo Sim. Esta versao foi ajustada para ROS 2 Jazzy, Ubuntu 24.04 e Gazebo Sim 8.

O pacote inclui:

- Robo diferencial de 2 rodas
- Robo diferencial de 4 rodas
- URDF/Xacro
- sensores simulados: lidar 2D, camera de profundidade e topicos de imagem/pontos
- bridge ROS 2 <-> Gazebo Sim
- SLAM com `slam_toolbox`
- navegacao com Nav2
- teleoperacao por joystick com `teleop_twist_joy`

## Requisitos

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Sim 8, instalado via pacotes ROS Jazzy

Antes de instalar este pacote, garanta que o ROS 2 Jazzy esteja instalado e disponivel:

```bash
source /opt/ros/jazzy/setup.bash
```

## Instalar

Crie um workspace, clone o pacote e instale as dependencias:

```bash
mkdir -p ~/sim_ws/src
cd ~/sim_ws/src
git clone -b humble-new-gazebo https://github.com/marlon655/sim_bot.git

cd ~/sim_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Se `rosdep` ainda nao estiver configurado no seu sistema:

```bash
sudo apt update
sudo apt install -y python3-rosdep python3-colcon-common-extensions
sudo rosdep init
rosdep update
```

Se `sudo rosdep init` informar que o arquivo ja existe, ignore esse passo e rode apenas:

```bash
rosdep update
```

## Rodar

Sempre que abrir um novo terminal, carregue o ROS 2 e o workspace:

```bash
cd ~/sim_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

Robo diferencial de 2 rodas:

```bash
ros2 launch sim_bot diff_bot.launch.py
```

Robo diferencial de 4 rodas:

```bash
ros2 launch sim_bot four_wheel.launch.py
```

Para confirmar que o pacote esta visivel para o ROS:

```bash
ros2 pkg prefix sim_bot
```

O retorno esperado deve apontar para:

```bash
~/sim_ws/install/sim_bot
```

## Argumentos De Launch

Os principais argumentos aceitos por `diff_bot.launch.py` e `four_wheel.launch.py`:

```text
world:=<path>       Mundo SDF a carregar. Padrao: test.world
headless:=True     Roda sem abrir a interface grafica do Gazebo
rviz:=False        Nao abre o RViz
joy:=False         Desativa teleoperacao por joystick
slam:=False        Desativa SLAM
nav:=False         Desativa Nav2
octomap:=True      Ativa mapeamento 3D com octomap_server
```

Exemplo rodando sem joystick e sem Nav2:

```bash
ros2 launch sim_bot diff_bot.launch.py joy:=False nav:=False
```

Exemplo rodando sem interface grafica:

```bash
ros2 launch sim_bot diff_bot.launch.py headless:=True
```

## Teleoperacao

Por padrao, o launch inicia teleoperacao por joystick. Para usar teclado, instale o pacote opcional:

```bash
sudo apt install -y ros-jazzy-teleop-twist-keyboard
```

Inicie a simulacao com joystick desativado:

```bash
ros2 launch sim_bot diff_bot.launch.py joy:=False
```

Em outro terminal:

```bash
cd ~/sim_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel_joy
```

## Octomap

O `octomap` fica desativado por padrao. Para usar mapeamento 3D, instale:

```bash
sudo apt install -y ros-jazzy-octomap-server
```

Depois rode:

```bash
ros2 launch sim_bot diff_bot.launch.py octomap:=True
```

## Observacoes

- Na primeira execucao, o Gazebo pode baixar modelos do Gazebo Fuel usados no mundo `test.world`.
- Se o pacote nao for encontrado, confira se o workspace correto foi carregado com `source ~/sim_ws/install/setup.bash`.
- Se outro workspace estiver no seu `.bashrc`, carregue primeiro `/opt/ros/jazzy/setup.bash` e depois `~/sim_ws/install/setup.bash`.
