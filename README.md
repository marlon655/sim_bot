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
map:=<path>        Mapa YAML para usar com Nav2 sem SLAM. Inicia map_server e AMCL
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
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Navegacao Com Mapa Existente

Para navegar usando um mapa `.yaml/.pgm` ja salvo, rode sem SLAM e passe o arquivo YAML no argumento `map`.

Exemplo usando o mundo e o mapa da aceleradora:

```bash
cd ~/sim_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch sim_bot diff_bot.launch.py \
  world:=$PWD/src/sim_bot/worlds/aceleradora.world \
  slam:=False \
  nav:=True \
  rviz:=True \
  joy:=False \
  map:=$PWD/src/sim_bot/config_map/aceleradora/aceleradora.yaml
```

O arquivo YAML deve apontar para o `.pgm`. Se os dois arquivos estiverem na mesma pasta, o caminho relativo e suficiente:

```yaml
image: aceleradora.pgm
resolution: 0.050000
origin: [-49.632501, -5.931761, 0.000000]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

Quando `map:=...` e informado, `nav.launch.py` inicia:

```text
map_server                  Carrega e publica o /map
amcl                        Localiza o robo no mapa e publica map -> odom
lifecycle_manager_localization  Ativa map_server e amcl automaticamente
```

No RViz, primeiro use `2D Pose Estimate` para informar a pose inicial do robo no mapa. Depois use `Nav2 Goal` para enviar o objetivo.

O Nav2 publica comandos diretamente em `/cmd_vel`, que e o topico usado pelo bridge ROS 2 <-> Gazebo. Os parametros `enable_stamped_cmd_vel: false` em `controller_server` e `behavior_server` mantem esse topico como `geometry_msgs/msg/Twist`, compativel com o Gazebo Sim.

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
- Novas alteracoes devem ser revisadas por Pull Request para a branch `dev` antes de serem integradas na `main`.

## Modificacoes

- Atualizado para ROS 2 Jazzy, Ubuntu 24.04 e Gazebo Sim 8.
- Dependencias do `package.xml` foram trocadas de pacotes fixos `ros-humble-*` para nomes genericos de pacotes ROS 2, permitindo resolucao correta via `rosdep` no Jazzy.
- Mundos SDF foram ajustados de plugins antigos `ignition-gazebo-*` para plugins `gz-sim-*`.
- Caminhos locais de modelos do Gazebo Fuel foram removidos do mundo `test.world` e substituidos por URIs publicas.
- Parametros do Nav2 foram atualizados para nomes de plugins compativeis com Jazzy.
- `octomap` passou a iniciar desativado por padrao, pois e opcional para a simulacao 2D com SLAM/Nav2.
- Frames das cameras foram ajustados para corrigir a exibicao do `/camera/points` no RViz: `camera_link_optical` agora fica alinhado com `camera_link`, e a depth camera publica a nuvem usando `camera_link_optical`.
- Variaveis locais de leitura dos `LaunchConfiguration` em `diff_bot.launch.py` foram renomeadas com prefixo `read_`, deixando mais claro que elas apenas leem os argumentos declarados no launch, sem alterar os nomes usados no terminal.
- Adicionado o modelo `aceleradora_world` com STL local, world `aceleradora.world` e configuracao do `GZ_SIM_RESOURCE_PATH` nos launch files para resolver URIs `model://` no Gazebo.
- Adicionado suporte a navegacao com mapa existente via argumento `map:=...`, iniciando `nav2_map_server`, `nav2_amcl` e `lifecycle_manager_localization`.
- Adicionadas dependencias explicitas `nav2_map_server` e `nav2_amcl` no `package.xml`.
- Ajustado o Nav2 para publicar `/cmd_vel` como `geometry_msgs/msg/Twist` com `enable_stamped_cmd_vel: false`, evitando conflito com `TwistStamped` no bridge do Gazebo.
- Removido o `twist_mux`; teleoperacao e Nav2 agora publicam diretamente em `/cmd_vel`. Use apenas um modo por vez, por exemplo `joy:=False` ao rodar com Nav2.
- Simplificado o `nav.launch.py`, removendo o modo composable (`use_composition`) e mantendo apenas a inicializacao direta dos nós Nav2 em processos separados.
