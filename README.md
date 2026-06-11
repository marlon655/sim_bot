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

Simulacao principal do Palmares:

```bash
ros2 launch sim_bot sim_manager.launch.py
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

Os principais argumentos aceitos por `sim_manager.launch.py`:

```text
world:=<path>       Mundo SDF a carregar. Padrao: test.world
headless:=True     Roda sem abrir a interface grafica do Gazebo
rviz:=False        Nao abre o RViz
joy:=False         Desativa teleoperacao por joystick
slam:=False        Desativa SLAM
nav:=False         Desativa Nav2
nav_params_file:=<path> Arquivo YAML de parametros do Nav2. Padrao: nav_hub/config/sim_nav_params.yaml
```

Exemplo rodando sem joystick e sem Nav2:

```bash
ros2 launch sim_bot sim_manager.launch.py joy:=False nav:=False
```

Exemplo rodando sem interface grafica:

```bash
ros2 launch sim_bot sim_manager.launch.py headless:=True
```

## Teleoperacao

Por padrao, o launch inicia teleoperacao por joystick. Para usar teclado, instale o pacote opcional:

```bash
sudo apt install -y ros-jazzy-teleop-twist-keyboard
```

Inicie a simulacao com joystick desativado:

```bash
ros2 launch sim_bot sim_manager.launch.py joy:=False
```

Em outro terminal:

```bash
cd ~/sim_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Navegacao Com Mapa Existente

Para navegar usando um mapa `.yaml/.pgm` ja salvo, rode sem SLAM e com Nav2
ativo. No fluxo atual, o mapa fica definido no arquivo de parametros do
`nav_hub`.

Exemplo usando o mundo e o mapa da aceleradora:

```bash
cd ~/sim_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch sim_bot sim_manager.launch.py \
  world:=$PWD/src/sim_bot/worlds/aceleradora.world \
  slam:=False \
  nav:=True \
  rviz:=True \
  joy:=False
```

O mapa usado no fluxo Oregon/Palmares fica definido no arquivo de parametros Nav2,
em `map_server.yaml_filename`. O arquivo YAML do mapa deve apontar para o `.pgm`.
Se os dois arquivos estiverem na mesma pasta, o caminho relativo e suficiente:

```yaml
image: aceleradora.pgm
resolution: 0.050000
origin: [-49.632501, -5.931761, 0.000000]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

No fluxo atual, `nav_hub/launch/route_graph/sim_nav_graph.launch.py` inicia a
localizacao usando o mapa definido no arquivo de parametros Nav2:

```text
map_server                  Carrega e publica o /map
amcl                        Localiza o robo no mapa e publica map -> odom
lifecycle_manager_localization  Ativa map_server e amcl automaticamente
```

No RViz, primeiro use `2D Pose Estimate` para informar a pose inicial do robo no mapa. Depois use `Nav2 Goal` para enviar o objetivo.

O Nav2 publica comandos diretamente em `/cmd_vel`, que e o topico usado pelo bridge ROS 2 <-> Gazebo. Os parametros `enable_stamped_cmd_vel: false` em `controller_server` e `behavior_server` mantem esse topico como `geometry_msgs/msg/Twist`, compativel com o Gazebo Sim.

## Fluxo Palmares/Oregon

Este fluxo usa o `palmares_bot`, mapa pronto, Nav2, route server, grafo de rotas
e filtro de velocidade. Ele e o fluxo principal para testar a adaptacao baseada
na arquitetura da Oregon.

### 1. Inicializar Simulacao

O `sim_manager.launch.py` carrega os valores padrao de:

```text
config/launch_params.yaml
```

Por isso, o fluxo Palmares/Oregon pode ser iniciado apenas com:

```bash
cd ~/sim_ws
source install/setup.bash

ros2 launch sim_bot sim_manager.launch.py
```

Arquivo de parametros carregado pelo launch:

```yaml
mode: simulation

# Simulation essentials
world: worlds/aceleradora.world
robot_name: palmares_bot
robot_urdf: description/palmares_bot.urdf.xacro
spawn_x: 18.34406852722168
spawn_y: 22.93574333190918
spawn_z: 0.15
bridge_config: config/gz_bridge.yaml
rviz_config: rviz/bot.rviz
rviz: true
joy: false

# Navigation
slam: false
nav: true
route: true
speed_filter: true
```

Os caminhos podem ser absolutos ou relativos ao pacote `sim_bot`.

Significado dos parametros:

```text
mode:=simulation   Usa a camada de simulacao baseada em Gazebo.
world:=...          Mundo Gazebo usado na simulacao.
robot_name:=...     Nome usado no spawn do robo no Gazebo.
robot_urdf:=...     Xacro/URDF usado pelo robot_state_publisher.
spawn_x/y/z:=...    Pose inicial do robo no Gazebo.
bridge_config:=...  YAML do ros_gz_bridge.
rviz_config:=...    Arquivo de configuracao do RViz.
slam:=False         Desliga SLAM, pois este fluxo usa mapa pronto.
nav:=True           Sobe a pilha Nav2.
rviz:=True          Abre o RViz.
joy:=False          Desliga teleoperacao por joystick para evitar conflito com Nav2.
route:=True         Sobe o nav2_route/route_server para calcular rotas pelo grafo.
speed_filter:=True  Sobe os servidores do filtro de velocidade.
```

Mapa, grafo de rota e mascara de velocidade ficam definidos no arquivo de
parametros Nav2 do pacote `nav_hub`:

```text
nav_hub/config/sim_nav_params.yaml
  -> map_server.yaml_filename
  -> route_server.graph_filepath
  -> speed_filter_mask_server.yaml_filename
```

No fluxo atual, esses arquivos ficam no `nav_hub`. O `sim_bot` nao mantem mais
copias locais de mapa, grafo, mascara ou Behavior Tree de navegacao.

Ainda e possivel sobrescrever qualquer parametro pelo terminal:

```bash
ros2 launch sim_bot sim_manager.launch.py rviz:=False joy:=True
```

O `sim_manager.launch.py` organiza a simulacao em duas camadas:

```text
sim_essentials.launch.py
  -> Gazebo, robot_state_publisher, spawn do robo, bridge, RViz e joystick

sim_navigation.launch.py
  -> slam opcional, nav_hub/launch/route_graph/sim_nav_graph.launch.py
  -> Nav2, route_server e speed_filter
```

O `config/launch_params.yaml` e lido apenas pelo `sim_manager.launch.py`.
Os launches de camada recebem os valores como argumentos. Portanto, para usar o
perfil completo, rode pelo orquestrador:

```bash
ros2 launch sim_bot sim_manager.launch.py
```

Com esse launch, o fluxo esperado ao clicar em `2D Goal Pose` no RViz e:

```text
RViz 2D Goal Pose
  -> bt_navigator
  -> nav_hub/btree/nav_on_route_graph_oregon.xml
  -> ComputeRoute
  -> route_server
  -> nav_hub/graphs/aceleradoras.json
  -> SmoothPath
  -> FollowPath
  -> /cmd_vel
```

### 2. Visualizar O Grafo No RViz

Em outro terminal:

```bash
cd ~/sim_ws
source install/setup.bash

ros2 run sim_bot graph_visualizer
```

Por padrao, o `graph_visualizer` usa o grafo do pacote `nav_hub`:

```text
nav_hub/graphs/aceleradoras.json
```

O pacote `nav_hub` precisa estar compilado no workspace, pois o `sim_bot` nao
mantem mais uma copia local do grafo.

No RViz, adicione:

```text
Add -> By topic -> /route_graph_markers -> MarkerArray
```

O visualizador mostra:

```text
linhas azuis: arestas do grafo
esferas amarelas: nos do grafo
texto branco: ID dos nos
```

### 3. Posicionar O Robo No No 1

Para testar `route_to_poses` com `start_id:=1`, primeiro envie o robo para a
posicao do no 1:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
"{pose: {header: {frame_id: 'map'}, pose: {position: {x: 17.41612434387207, y: 18.228958129882812, z: 0.0}, orientation: {w: 1.0}}}}"
```

Esse passo e util quando o teste usa IDs fixos no grafo e voce quer garantir
que a posicao real do robo esta coerente com o `start_id`.

### 4. Testar Rota Por ID

Depois que o robo estiver proximo do no 1:

```bash
ros2 run sim_bot route_to_poses --ros-args \
  -p use_start:=False \
  -p start_id:=1 \
  -p goal_id:=17
```

Esse comando chama `/compute_route` usando IDs do grafo e envia o caminho
retornado para `/follow_path`.

Arquivos principais deste fluxo:

```text
launch/sim_manager.launch.py
launch/sim_essentials.launch.py
launch/sim_navigation.launch.py
nav_hub/launch/route_graph/sim_nav_graph.launch.py
nav_hub/config/sim_nav_params.yaml
config/launch_params.yaml
nav_hub/btree/nav_on_route_graph_oregon.xml
nav_hub/graphs/aceleradoras.json
scripts/route_to_poses.py
scripts/graph_visualizer.py
nav_hub/maps/aceleradora_speed_mask.yaml
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
- Frames das cameras foram ajustados para corrigir a exibicao do `/camera/points` no RViz: `camera_link_optical` agora fica alinhado com `camera_link`, e a depth camera publica a nuvem usando `camera_link_optical`.
- Adicionado o modelo `aceleradora_world` com STL local, world `aceleradora.world` e configuracao do `GZ_SIM_RESOURCE_PATH` nos launch files para resolver URIs `model://` no Gazebo.
- Adicionado suporte a navegacao com mapa existente via `map_server.yaml_filename` no arquivo de parametros Nav2, iniciando `nav2_map_server`, `nav2_amcl` e `lifecycle_manager_localization`.
- Adicionadas dependencias explicitas `nav2_map_server` e `nav2_amcl` no `package.xml`.
- Ajustado o Nav2 para publicar `/cmd_vel` como `geometry_msgs/msg/Twist` com `enable_stamped_cmd_vel: false`, evitando conflito com `TwistStamped` no bridge do Gazebo.
- Removido o `twist_mux`; teleoperacao e Nav2 agora publicam diretamente em `/cmd_vel`. Use apenas um modo por vez, por exemplo `joy:=False` ao rodar com Nav2.
- Adicionado `nav_hub/launch/route_graph/sim_nav_graph.launch.py` como launch de navegacao da simulacao, mantendo a estrutura mais proxima da Oregon sem depender de hardware real.
