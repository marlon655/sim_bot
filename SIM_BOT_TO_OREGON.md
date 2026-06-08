# Sim Bot To Oregon

Este documento organiza as etapas para adaptar a simulacao atual do `sim_bot`
para usar configuracoes inspiradas no projeto Oregon.

O arquivo principal de teste e:

```bash
src/sim_bot/config/oregon_nav_params.yaml
```

Ele foi criado a partir do `config/nav_params.yaml` do simulador, mantendo
partes da Oregon comentadas para consulta e futura migracao.

## 1. Confirmar o Frame Base do Robo

Antes de testar os parametros da Oregon, confirme qual frame o robo usa no Xacro:

```text
base_link
base_footprint
```

O arquivo `oregon_nav_params.yaml` esta usando `base_link`.

Se o Xacro usar `base_footprint`, ajuste todos os pontos equivalentes:

```text
bt_navigator.robot_base_frame
amcl.base_frame_id
local_costmap.robot_base_frame
global_costmap.robot_base_frame
behavior_server.robot_base_frame
collision_monitor.base_frame_id, se for ativado depois
```

Comando util para conferir TF:

```bash
ros2 run tf2_tools view_frames
```

## 2. Testar Primeiro com Plugins Padrao

Primeiro teste apenas os plugins comuns do Nav2 que ja funcionam no simulador:

```text
DWBLocalPlanner
NavfnPlanner
ObstacleLayer
InflationLayer
StaticLayer
SimpleSmoother
AMCL
```

Comando de teste:

```bash
ros2 launch sim_bot diff_bot.launch.py \
  world:=$PWD/src/sim_bot/worlds/aceleradora.world \
  slam:=False \
  nav:=True \
  rviz:=True \
  joy:=False \
  map:=$PWD/src/sim_bot/config_map/aceleradora/aceleradora.yaml \
  params_file:=$PWD/src/sim_bot/config/oregon_nav_params.yaml
```

Se esse teste nao rodar limpo, nao avance para os plugins especificos da Oregon.

## 3. Ajustar o Footprint

O simulador esta usando raio simples:

```yaml
robot_radius: 0.35
```

A Oregon usava um footprint com o formato real do robo:

```yaml
footprint: "[[0.26, -0.25], [0.26, 0.25], [-0.26, 0.25], [-0.26, 0.085], [-0.54, 0.085], [-0.54, -0.085], [-0.26, -0.085], [-0.26, -0.25]]"
```

Para usar o footprint, remova ou comente o `robot_radius` e ative o
`footprint` em:

```text
local_costmap
global_costmap
```

## 4. Ajustar Velocidades

O simulador usa velocidades conservadoras:

```yaml
max_vel_x: 0.26
max_vel_theta: 1.0
```

A Oregon usava valores mais agressivos em alguns controladores, por exemplo:

```yaml
desired_linear_vel: 0.7
```

Suba as velocidades aos poucos. Primeiro valide navegacao lenta, depois ajuste
para se aproximar do comportamento da Oregon.

## 5. Avaliar o Controller da Oregon

A Oregon usava um controller customizado:

```yaml
nav2_regulated_pure_pursuit_controller::CustomRegulatedPurePursuitController
```

Antes de ativar, confirme se esse plugin existe no ambiente:

```bash
ros2 pkg list | grep pure
ros2 pkg prefix nav2_regulated_pure_pursuit_controller
```

Se o plugin customizado nao existir, mantenha o controller padrao do simulador:

```yaml
dwb_core::DWBLocalPlanner
```

## 6. Avaliar o Route Server

A Oregon usava navegacao por grafo:

```text
route_server
graph_filepath
nav2_route
```

Para ativar isso no simulador, sera necessario:

```text
ter o pacote nav2_route instalado ou compilado
copiar/adaptar o arquivo .json do grafo para o sim_bot
adaptar um launch para iniciar o route_server
usar uma Behavior Tree compativel com rota
```

Essa etapa deve ser feita depois que a navegacao basica com mapa estiver
funcionando.

## 7. Avaliar o Speed Filter

A Oregon usava filtro de velocidade no costmap:

```text
speed_filter_mask_server
speed_costmap_filter_info_server
speed_filter
```

Para usar no simulador, sera necessario:

```text
ter um mapa de mascara de velocidade .yaml/.pgm
iniciar os servidores do filtro
ativar filters: ["speed_filter"] no global_costmap
validar o topico /speed_limit
```

Deixe essa parte comentada ate o Nav2 basico estar estavel.

## 8. Deixar STVL e ToF para o Final

A Oregon tambem tinha configuracoes para sensor 3D/ToF:

```text
spatio_temporal_voxel_layer
/ground_segmentation/obstacle_points
PointCloud2
```

No simulador atual, o sensor principal e o LaserScan em:

```text
/scan
```

Por isso, mantenha STVL/ToF comentado ate existir uma simulacao real de sensor
3D ou uma ponte publicando `PointCloud2`.

## Comandos Uteis de Diagnostico

```bash
ros2 topic list
ros2 topic echo /scan --once
ros2 topic echo /odom --once
ros2 run tf2_ros tf2_echo map base_link
ros2 lifecycle nodes
ros2 node list
```

## Sequencia Recomendada

```text
1. Rodar o oregon_nav_params.yaml como esta
2. Conferir /scan, /odom, /tf e /map
3. Ajustar base_link/base_footprint
4. Ajustar footprint
5. Ajustar velocidades
6. Testar controller Oregon, se o plugin existir
7. Testar route_server
8. Testar speed_filter ou ToF apenas no final
```
