# Launch files

Esta pasta contem os arquivos de inicializacao ROS 2 do pacote `sim_bot`.

Um arquivo `.launch.py` define quais nos serao iniciados, quais argumentos podem ser passados no terminal e quais outros launch files serao chamados. No geral, os arquivos principais iniciam a simulacao no Gazebo e incluem arquivos auxiliares para robot state publisher, SLAM, navegacao e teleoperacao.

## Visao geral

Fluxo comum da simulacao:

```text
diff_bot.launch.py / four_wheel.launch.py / palmares_bot.launch.py
    -> rsp.launch.py
    -> teleop.launch.py        opcional
    -> slam.launch.py          opcional
    -> nav.launch.py           opcional
    -> Gazebo + bridges + RViz
```

Os launch files principais usam argumentos como:

```text
world:=...       caminho do mundo SDF usado pelo Gazebo
headless:=True   executa Gazebo sem interface grafica
rviz:=False      nao abre RViz
joy:=False       desativa teleoperacao por joystick
slam:=False      desativa SLAM
nav:=False       desativa Nav2
map:=...         mapa YAML usado pelo Nav2 com map_server e AMCL
```

## diff_bot.launch.py

Launch principal para o robo `diff_bot`.

Ele inicia:

- Gazebo server com o mundo definido em `world:=...`.
- Gazebo client, exceto quando `headless:=True`.
- `rsp.launch.py` usando `description/diff_bot.urdf.xacro`.
- Spawn do robo no Gazebo via `ros_gz_sim create`, lendo o topico `robot_description`.
- Bridge ROS 2 <-> Gazebo usando `config/gz_bridge.yaml`.
- Bridge de imagem para `/camera/image`.
- RViz usando `rviz/bot.rviz`, quando `rviz:=True`.
- Teleoperacao por joystick, quando `joy:=True`.
- SLAM, quando `slam:=True`.
- Nav2, quando `nav:=True`.

Exemplo com mapa existente:

```bash
ros2 launch sim_bot diff_bot.launch.py \
  world:=$PWD/src/sim_bot/worlds/aceleradora.world \
  slam:=False \
  nav:=True \
  rviz:=True \
  joy:=False \
  map:=$PWD/src/sim_bot/config_map/aceleradora/aceleradora.yaml
```

Use este arquivo como ponto de entrada principal para a simulacao do `diff_bot`.

## four_wheel.launch.py

Launch principal para o robo `four_wheel_bot`.

Ele tem estrutura parecida com `diff_bot.launch.py`, mas usa:

```text
description/four_wheel.urdf.xacro
```

E faz o spawn do robo com o nome:

```text
four_wheel_bot
```

Ele tambem pode iniciar Gazebo, RViz, teleoperacao, SLAM, Nav2 e bridges.

Observacao: este launch nao repassa o argumento `map` para o `nav.launch.py`. Portanto, no estado atual, ele e mais adequado para simulacao com SLAM do que para navegacao usando mapa YAML salvo.

## palmares_bot.launch.py

Launch principal para o robo `palmares_bot`.

Ele usa:

```text
description/palmares_bot.urdf.xacro
```

E faz o spawn do robo com o nome:

```text
palmares_bot
```

Ele inicia Gazebo, RViz, teleoperacao, SLAM e Nav2 de forma semelhante aos outros launches principais.

Precisa adicionar o argumento map:= , para que ele possa utilizar um mapa salvo.

## rsp.launch.py

Launch auxiliar do `robot_state_publisher`.

Ele recebe um arquivo Xacro/URDF pelo argumento:

```text
urdf:=...
```

Se nenhum `urdf` for informado, usa como padrao:

```text
description/diff_bot.urdf.xacro
```

O arquivo Xacro e processado com:

```text
xacro <arquivo>
```

O resultado e publicado no parametro:

```text
robot_description
```

Esse parametro e usado por:

- `robot_state_publisher`, para publicar a arvore de TF do robo.
- `ros_gz_sim create`, para spawnar o robo no Gazebo a partir do topico `robot_description`.

Sem este launch, o ROS 2 nao conhece corretamente os frames do robo, como `base_link`, sensores, rodas, camera e lidar.

## teleop.launch.py

Launch auxiliar de teleoperacao por joystick.

Ele inicia:

- `joy_node`, que le o controle/joystick.
- `teleop_twist_joy`, que converte comandos do joystick em velocidade.

Os parametros ficam em:

```text
config/joy_params.yaml
```

A saida de velocidade e publicada diretamente em:

```text
/cmd_vel
```

Como o `twist_mux` foi removido, evite usar teleoperacao e Nav2 ao mesmo tempo. Para usar Nav2, rode com:

```text
joy:=False
```

## slam.launch.py

Launch auxiliar do SLAM.

Ele inicia:

```text
slam_toolbox
```

usando o executavel:

```text
async_slam_toolbox_node
```

Os parametros ficam em:

```text
config/slam_params.yaml
```

Ele usa dados como `/scan`, odometria e TF para criar o mapa durante a movimentacao do robo.

Use SLAM quando ainda nao houver mapa salvo do ambiente:

```text
slam:=True
```

Quando ja existir um mapa `.yaml/.pgm`, use:

```text
slam:=False
map:=/caminho/do/mapa.yaml
```

## nav.launch.py

Launch auxiliar do Nav2.

Ele inicia a pilha de navegacao:

- `controller_server`
- `smoother_server`
- `planner_server`
- `behavior_server`
- `bt_navigator`
- `waypoint_follower`
- `velocity_smoother`
- `lifecycle_manager_navigation`

Os parametros principais ficam em:

```text
config/nav_params.yaml
```

Quando o argumento `map:=...` e informado, ele tambem inicia:

- `map_server`
- `amcl`
- `lifecycle_manager_localization`

Nesse caso, o Nav2 usa um mapa ja salvo e localiza o robo com AMCL.

Quando `map` fica vazio, `map_server` e `amcl` nao sao iniciados. Esse modo combina com SLAM, porque o mapa e a transformacao `map -> odom` ficam a cargo do `slam_toolbox`.

O `nav.launch.py` foi simplificado para usar apenas nos separados. O modo composable (`use_composition`) foi removido para manter o arquivo mais simples e facilitar depuracao na simulacao.

## Quando usar cada arquivo

Para iniciar uma simulacao completa do `diff_bot`:

```bash
ros2 launch sim_bot diff_bot.launch.py
```

Para usar mapa salvo com Nav2:

```bash
ros2 launch sim_bot diff_bot.launch.py \
  slam:=False \
  nav:=True \
  joy:=False \
  map:=/caminho/do/mapa.yaml
```

Para mapear um ambiente novo:

```bash
ros2 launch sim_bot diff_bot.launch.py \
  slam:=True \
  nav:=True
```

Para testar somente teleoperacao por joystick:

```bash
ros2 launch sim_bot diff_bot.launch.py \
  nav:=False \
  joy:=True
```

Para rodar apenas o robot state publisher com outro Xacro:

```bash
ros2 launch sim_bot rsp.launch.py \
  urdf:=$PWD/src/sim_bot/description/palmares_bot.urdf.xacro
```
