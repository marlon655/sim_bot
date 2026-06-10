# Integracao Palmares/Oregon

Este documento explica a adaptacao feita para usar o robo `palmares_bot` no
simulador com parametros inspirados no projeto Oregon.

Ele parte da ideia de que a pessoa conhece apenas o fluxo basico do
`diff_bot.launch.py`.

## Visao Geral

No projeto basico, o fluxo e:

```text
diff_bot.launch.py
  -> Gazebo
  -> robot_state_publisher
  -> bridge Gazebo/ROS
  -> RViz
  -> nav.launch.py
      -> map_server / AMCL
      -> Nav2
```

Na adaptacao Palmares/Oregon, o fluxo fica:

```text
palmares_bot.launch.py
  -> Gazebo
  -> robot_state_publisher com palmares_bot.urdf.xacro
  -> bridge Gazebo/ROS
  -> RViz
      -> nav.launch.py
          -> map_server / AMCL
          -> Nav2 com config/oregon_nav_params.yaml
          -> route_server, se route:=True
          -> speed_filter, se speed_filter:=True
          -> Behavior Tree de rota por grafo
```

O comando principal de teste e:

```bash
ros2 launch sim_bot palmares_bot.launch.py \
  world:=$PWD/src/sim_bot/worlds/aceleradora.world \
  slam:=False \
  nav:=True \
  rviz:=True \
  joy:=False \
  route:=True \
  speed_filter:=True \
  map:=$PWD/src/sim_bot/config_map/aceleradora/aceleradora.yaml \
  params_file:=$PWD/src/sim_bot/config/oregon_nav_params.yaml
```

## palmares_bot.launch.py

Arquivo:

```text
launch/palmares_bot.launch.py
```

Este e o launch principal para simular o robo Palmares.

Ele faz o mesmo papel geral do `diff_bot.launch.py`, mas usando:

```text
description/palmares_bot.urdf.xacro
```

### O Que Ele Sobe

```text
Gazebo server
Gazebo client, se headless:=False
robot_state_publisher
ros_gz_bridge
RViz
teleop, se joy:=True
slam.launch.py, se slam:=True
nav.launch.py, se nav:=True
```

### Por Que Ele Inclui nav.launch.py

O `palmares_bot.launch.py` nao repete todos os nos do Nav2 diretamente.
Em vez disso, ele inclui:

```text
launch/nav.launch.py
```

Isso evita duplicacao. Se outro robo tambem precisar de Nav2, ele pode incluir
o mesmo `nav.launch.py`.

### Argumentos Importantes

```text
world:=<path>
map:=<path>
params_file:=<path>
route:=True|False
slam:=True|False
nav:=True|False
rviz:=True|False
joy:=True|False
headless:=True|False
```

### Pose Inicial no Gazebo

O robo esta sendo spawnado no Gazebo em:

```text
x = 18.34406852722168
y = 22.93574333190918
z = 0.15
```

Isso fica no node `ros_gz_sim create`.

O valor de `z` no Gazebo e maior que zero para evitar que o robo nasca
enterrado ou colidindo com o chao.

## nav.launch.py

Arquivo:

```text
launch/nav.launch.py
```

Este arquivo sobe a parte de navegacao.

Ele e usado pelo `palmares_bot.launch.py` e tambem pode ser usado por outros
launchs do pacote.

### O Que Ele Sobe Sempre Que nav:=True

```text
controller_server
smoother_server
planner_server
behavior_server
bt_navigator
waypoint_follower
velocity_smoother
lifecycle_manager_navigation
```

### O Que Ele Sobe Quando map:=... E Informado

Quando o argumento `map` recebe um arquivo YAML de mapa, ele tambem sobe:

```text
map_server
amcl
lifecycle_manager_localization
```

Isso e usado quando ja existe mapa pronto, como:

```text
config_map/aceleradora/aceleradora.yaml
```

Nesse caso, o SLAM deve ficar desligado:

```text
slam:=False
```

### O Que Ele Sobe Quando route:=True

Quando `route:=True`, ele tambem sobe:

```text
route_server
lifecycle_manager_route
```

O `route_server` pertence ao pacote:

```text
nav2_route
```

Ele calcula rotas em um grafo, mas nao move o robo sozinho.

### O Que Ele Sobe Quando speed_filter:=True

Quando `speed_filter:=True`, ele tambem sobe:

```text
speed_filter_mask_server
speed_costmap_filter_info_server
lifecycle_manager_speed_filter
```

Esses nodes publicam a mascara de velocidade e a informacao do filtro usada
pelo `global_costmap`.

## oregon_nav_params.yaml

Arquivo:

```text
config/oregon_nav_params.yaml
```

Este arquivo e a configuracao Nav2 usada no teste Palmares/Oregon.

Ele foi criado a partir do `config/nav_params.yaml` do simulador e recebeu
parametros da Oregon de forma controlada.

### Por Que Criar Um Arquivo Novo

O `nav_params.yaml` original continua sendo a configuracao base do simulador.

O `oregon_nav_params.yaml` existe para testar a migracao Oregon sem quebrar o
comportamento conhecido do `diff_bot`.

### AMCL

O AMCL usa:

```text
map
odom
base_link
scan
```

O arquivo define uma pose inicial alinhada ao spawn do Gazebo:

```yaml
set_initial_pose: true
initial_pose:
  x: 18.34406852722168
  y: 22.93574333190918
  z: 0.0
  yaw: 0.0
```

O `z` no AMCL fica `0.0`, porque essa pose e no mapa 2D. O `z=0.15` e apenas
para spawn no Gazebo.

### Controller

A Oregon usava:

```yaml
nav2_regulated_pure_pursuit_controller::CustomRegulatedPurePursuitController
```

Esse plugin customizado nao esta disponivel no ambiente atual. Por isso, no
simulador foi usado o plugin padrao:

```yaml
nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
```

Isso permite testar comportamento parecido sem depender do Nav2 custom da
Oregon.

Configuracoes importantes:

```yaml
desired_linear_vel: 0.7
lookahead_dist: 1.5
use_velocity_scaled_lookahead_dist: true
use_collision_detection: true
max_allowed_time_to_collision_up_to_carrot: 1.0
use_rotate_to_heading: true
```

### Por Que use_rotate_to_heading Ficou True

Na Oregon, o controller custom usava:

```yaml
use_rotate_to_heading: false
```

No plugin padrao do Nav2, deixar `true` ajudou o robo diferencial a alinhar
antes de seguir o caminho.

### Collision Detection

Com:

```yaml
use_collision_detection: true
```

o controller projeta o movimento do robo alguns instantes a frente e verifica
se esse comando causaria colisao no `local_costmap`.

O horizonte esta em:

```yaml
max_allowed_time_to_collision_up_to_carrot: 1.0
```

Isso significa que o controller verifica aproximadamente ate 1 segundo a frente,
limitado pelo ponto de lookahead.

### Footprint

O footprint ativo foi ajustado para o `palmares_bot`:

```yaml
footprint: "[[0.27, -0.22], [0.27, 0.32], [-0.29, 0.32], [-0.29, -0.22]]"
```

O footprint original da Oregon ficou comentado como referencia.

### Inflation Layer

Os valores de inflacao foram alinhados com a Oregon:

```yaml
inflate_unknown: true
cost_scaling_factor: 10.0
inflation_radius: 0.3
```

Eles aparecem em:

```text
local_costmap
global_costmap
```

### route_server

O bloco `route_server` tambem esta configurado no mesmo YAML:

```yaml
route_server:
  ros__parameters:
    graph_filepath: "$(find-pkg-share sim_bot)/graphs/aceleradoras.json"
```

Esse bloco so e usado quando o `nav.launch.py` sobe o route server com:

```text
route:=True
```

### Behavior Tree De Rota Por Grafo

Arquivo:

```text
btree/nav_on_route_graph_sim.xml
```

Esta BT foi adaptada da logica usada pela Oregon/Navigation2 para aproximar o
fluxo do simulador do fluxo real por grafo.

Ela e ativada em:

```yaml
default_nav_to_pose_bt_xml: "$(find-pkg-share sim_bot)/btree/nav_on_route_graph_sim.xml"
```

Com isso, um `2D Goal Pose` no RViz nao usa apenas o planner global direto. O
fluxo passa a ser:

```text
RViz 2D Goal Pose
  -> bt_navigator
  -> ComputeRoute
  -> route_server
  -> path do grafo
  -> primeiro trecho ate o grafo, se necessario
  -> ultimo trecho ate o goal clicado, se necessario
  -> SmoothPath
  -> FollowPath
```

Por isso, quando este YAML estiver ativo, a simulacao deve subir com:

```text
route:=True
```

Se o `route_server` nao estiver ativo, o goal pelo RViz pode falhar porque a BT
vai tentar chamar `/compute_route`.

O smoother tambem foi alinhado com a Oregon:

```yaml
smoother_plugins: ["savitzky_golay_smoother"]
savitzky_golay_smoother:
  plugin: "nav2_smoother::SavitzkyGolaySmoother"
  do_refinement: true
  keep_start_orientation: true
```

O `simple_smoother` ficou comentado no YAML como fallback do simulador.

## graphs/aceleradoras.json

Arquivo:

```text
graphs/aceleradoras.json
```

Esse grafo foi copiado da Oregon para testar o `nav2_route` no simulador.

Ele possui:

```text
18 nos do tipo Point
34 arestas do tipo MultiLineString
```

Cada `Point` representa um no do grafo:

```json
{
  "geometry": {
    "coordinates": [19.189355850219727, 22.489364624023438],
    "type": "Point"
  },
  "properties": {
    "frame": "map",
    "id": 7
  }
}
```

Cada aresta conecta dois nos usando propriedades como:

```text
startid
endid
id
cost
```

### O Que O Grafo Faz

O grafo permite que o `route_server` calcule uma rota entre dois IDs de nos.

Exemplo de teste:

```bash
ros2 action send_goal /compute_route nav2_msgs/action/ComputeRoute \
"{start_id: 7, goal_id: 6, use_start: false, use_poses: false}"
```

Se aparecer:

```text
Goal finished with status: SUCCEEDED
```

significa que:

```text
route_server subiu
grafo foi carregado
rota entre os nos foi calculada
```

Isso nao significa que o robo vai andar.

## Diferenca Entre Calcular Rota E Navegar

O `route_server` calcula a rota no grafo.

Ele nao publica `/cmd_vel` e nao envia goal diretamente para o controller.

Fluxo atual:

```text
/compute_route
  -> route_server
  -> retorna uma rota/path
```

Fluxo necessario para navegar por grafo:

```text
RViz ou cliente
  -> pede rota ao route_server
  -> Behavior Tree ou node cliente usa essa rota
  -> Nav2 segue o path/waypoints
  -> controller publica /cmd_vel
```

Na Oregon, essa camada extra provavelmente era feita por:

```text
nav_hub
main_route_graph
btree/nav_on_route_graph_oregon.xml
```

No simulador atual, essa camada comecou a ser adaptada pelo node
`route_to_poses`.

## route_to_poses

Arquivo:

```text
scripts/route_to_poses.py
```

Este node e a primeira integracao simples entre o `route_server` e o Nav2.

Ele faz:

```text
1. chama /compute_route
2. recebe o Path calculado pelo route_server
3. envia esse Path para /follow_path
4. o controller_server move o robo seguindo o caminho
```

Este caminho foi escolhido antes de adaptar a Behavior Tree da Oregon porque e
mais facil de debugar. Primeiro validamos se o grafo gera um caminho que o Nav2
consegue seguir. Depois podemos transformar isso em BT ou em um node mais
parecido com o `nav_hub`.

### Parametros Do Node

```text
start_id     ID inicial do grafo
goal_id      ID final do grafo
use_start    true por padrao; usa a pose atual do robo via TF como inicio
use_poses    se false, usa IDs; se true, usa poses
mode         follow_path por padrao; tambem aceita navigate_through_poses
max_poses    limita a quantidade de poses enviadas ao Nav2. 0 envia todas
```

Uso basico com IDs:

```bash
ros2 run sim_bot route_to_poses --ros-args \
  -p goal_id:=6
```

Nesse modo, o inicio da rota vem da pose atual do robo. Isso evita mandar o
controller seguir um caminho que comeca em um ponto do grafo longe da posicao
real do robo.

Para forcar uma rota entre dois IDs especificos do grafo:

```bash
ros2 run sim_bot route_to_poses --ros-args \
  -p use_start:=False \
  -p start_id:=7 \
  -p goal_id:=6
```

Se a rota tiver poses demais, pode limitar:

```bash
ros2 run sim_bot route_to_poses --ros-args \
  -p goal_id:=6 \
  -p max_poses:=10
```

Esse node depende de dois action servers ativos:

```text
/compute_route
/follow_path
```

O modo antigo tambem pode ser testado com:

```bash
ros2 run sim_bot route_to_poses --ros-args \
  -p goal_id:=6 \
  -p mode:=navigate_through_poses
```

Nesse caso, o node usa `/navigate_through_poses` em vez de `/follow_path`.

Por isso, a simulacao deve subir com:

```text
route:=True
nav:=True
```

## Como Testar

Rebuild, porque foi adicionada a pasta `graphs`:

```bash
cd ~/sim_ws
colcon build --packages-select sim_bot --symlink-install
source install/setup.bash
```

Subir simulacao com route server:

```bash
ros2 launch sim_bot palmares_bot.launch.py \
  world:=$PWD/src/sim_bot/worlds/aceleradora.world \
  slam:=False \
  nav:=True \
  rviz:=True \
  joy:=False \
  route:=True \
  speed_filter:=True \
  map:=$PWD/src/sim_bot/config_map/aceleradora/aceleradora.yaml \
  params_file:=$PWD/src/sim_bot/config/oregon_nav_params.yaml
```
```
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
"{pose: {header: {frame_id: 'map'}, pose: {position: {x: 17.41612434387207, y: 18.228958129882812, z: 0.0}, orientation: {w: 1.0}}}}"
```

```
ros2 run sim_bot route_to_poses --ros-args \
  -p use_start:=False \
  -p start_id:=1 \
  -p goal_id:=17
```

Conferir nodes:

```bash
ros2 node list | grep route
```

Conferir action:

```bash
ros2 action list | grep compute_route
```

Testar rota:

```bash
ros2 action send_goal /compute_route nav2_msgs/action/ComputeRoute \
"{start_id: 7, goal_id: 6, use_start: false, use_poses: false}"
```

Testar rota e fazer o robo andar:

```bash
ros2 run sim_bot route_to_poses --ros-args \
  -p goal_id:=6
```

## Estado Atual

Funciona:

```text
Palmares no Gazebo
Mapa pronto com AMCL
Nav2 com Regulated Pure Pursuit padrao
Goals normais pelo RViz
route_server calculando rota no grafo
route_to_poses enviando rota para FollowPath
speed_filter configurado para teste inicial
Behavior Tree de rota por grafo para 2D Goal Pose
```

Ainda nao foi adaptado:

```text
nav_hub/main_route_graph
ToF/STVL/ground_segmentation
```

## Proximos Passos

Esta lista deve ser mantida como roteiro vivo da integracao. Conforme cada
parte for validada no simulador, atualizar esta secao ou mover a explicacao
para o estado atual.

### 1. Validar E Limpar O Grafo

Arquivo principal:

```text
graphs/aceleradoras.json
```

Antes de aproximar mais da Oregon, o grafo precisa representar bem o mapa do
simulador.

O teste mostrou que um ponto inicial mal posicionado faz o robo tentar seguir
um caminho invalido na pratica. O `route_server` pode calcular uma rota valida
entre nos, mas se esses nos ou conexoes atravessarem paredes, o controller vai
falhar ou colidir.

Checklist do grafo:

```text
nos em areas livres do mapa
arestas conectando apenas caminhos realmente navegaveis
start_id perto da posicao inicial do robo quando o teste for por ID fixo
sequencia de IDs coerente com o corredor/ambiente real
rotas testadas no RViz antes de mexer em outras camadas
```

### 2. Documentar O Fluxo Atual Funcionando

Fluxo validado ate agora:

```text
route_server
  -> /compute_route
  -> route_to_poses
  -> /follow_path
  -> controller_server
  -> /cmd_vel
```

Este fluxo ainda nao e igual ao da Oregon, mas e uma ponte simples para provar
que o grafo consegue gerar caminhos que o Nav2 consegue seguir.

### 3. Aproximar O Controller Da Oregon

Arquivo principal:

```text
config/oregon_nav_params.yaml
```

Comparar e ajustar, em pequenos testes:

```text
controller_server
FollowPath
goal_checker
progress_checker
velocity_smoother
```

O plugin customizado da Oregon nao esta disponivel no simulador atual, entao a
base continua sendo:

```text
nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
```

As diferencas precisam ficar comentadas no YAML para separar o que veio da
Oregon do que foi adaptado para funcionar no simulador.

### 4. Adicionar Speed Filter

Depois que grafo e controller estiverem estaveis, testar o filtro de velocidade.

Sequencia de integracao:

```text
1. localizar os parametros da Oregon no nav_routegraph.yaml
2. criar/copiar uma mascara de velocidade para o simulador
3. alinhar a mascara com resolucao e origem do mapa aceleradora.yaml
4. configurar speed_filter_mask_server
5. configurar speed_costmap_filter_info_server
6. ativar o filtro speed_filter no global_costmap
7. subir os servidores pelo nav.launch.py e testar no RViz/simulacao
```

Estado atual desta etapa:

```text
mascara criada em config_map/aceleradora/aceleradora_speed_mask.yaml
mascara PGM alinhada ao mapa aceleradora.yaml
speed_filter_mask_server configurado
speed_costmap_filter_info_server configurado
global_costmap com filters: ["speed_filter"]
nav.launch.py com argumento speed_filter:=True
palmares_bot.launch.py e diff_bot.launch.py repassam speed_filter
```

Arquivos adicionados:

```text
config_map/aceleradora/aceleradora_speed_mask.yaml
config_map/aceleradora/aceleradora_speed_mask.pgm
```

O YAML da mascara usa a mesma resolucao e origem do mapa principal:

```yaml
image: aceleradora_speed_mask.pgm
mode: scale
resolution: 0.050000
origin: [-24.675000, -31.875000, 0.000000]
negate: 0
occupied_thresh: 1.0
free_thresh: 0.0
```

Isso e importante porque a mascara precisa estar alinhada com:

```text
config_map/aceleradora/aceleradora.yaml
config_map/aceleradora/aceleradora.pgm
```

### Mascara Pintada

A primeira area de teste foi pintada na curva entre os nos:

```text
id5 -> id6 -> id7
```

Coordenadas dos pontos:

```text
id5: x=11.673667907714844, y=-17.095163345336914
id6: x=8.770400047302246, y=-19.400238037109375
id7: x=5.286102294921875, y=-19.418624877929688
```

Foi pintada uma faixa de aproximadamente `1.0 m` ao redor desse trecho.

Configuracao do filtro:

```yaml
base: 100.0
multiplier: -1.0
```

Com essa configuracao, o valor da mascara vira:

```text
velocidade (%) = base + multiplier * valor_da_mascara
```

Na area da curva foi usado valor de filtro `60`, que resulta em:

```text
100 + (-1 * 60) = 40%
```

No arquivo PGM, esse valor aparece como pixel `102`, porque a imagem em modo
`scale` usa escala de cinza.

Resumo dos valores atuais da mascara:

```text
255 = area sem reducao de velocidade
102 = area limitada para aproximadamente 40% da velocidade
```

Para subir com speed filter:

```bash
ros2 launch sim_bot palmares_bot.launch.py \
  world:=$PWD/src/sim_bot/worlds/aceleradora.world \
  slam:=False \
  nav:=True \
  rviz:=True \
  joy:=False \
  route:=True \
  speed_filter:=True \
  map:=$PWD/src/sim_bot/config_map/aceleradora/aceleradora.yaml \
  params_file:=$PWD/src/sim_bot/config/oregon_nav_params.yaml
```

Topicos/nos esperados:

```text
/speed_filter_mask
/speed_costmap_filter_info
/speed_limit
speed_filter_mask_server
speed_costmap_filter_info_server
lifecycle_manager_speed_filter
```

Para visualizar a mascara no RViz:

```text
Add -> Map -> Topic: /speed_filter_mask
```

Se necessario, ajustar a transparencia do display para enxergar a mascara sobre
o mapa principal.

Comandos uteis para conferir:

```bash
ros2 topic list | grep speed
ros2 topic echo --once /speed_costmap_filter_info
```

O topico `/speed_limit` pode aparecer quando o robo entra em uma area filtrada.

Essa etapa vem depois do grafo porque um caminho mal posicionado pode parecer
problema de velocidade ou controller mesmo quando o erro esta no JSON.

### 5. Avaliar Behavior Tree / nav_hub

Na Oregon, a camada que liga rota por grafo com navegacao provavelmente envolve:

```text
Behavior Tree
nav_hub
main_route_graph
btree/nav_on_route_graph_oregon.xml
```

No simulador, a primeira etapa foi feita com:

```text
btree/nav_on_route_graph_sim.xml
```

Essa BT deixa o clique do RViz mais proximo da Oregon, pois chama
`ComputeRoute` e usa o `route_server` antes de seguir o caminho.

O `route_to_poses.py` continua util como ferramenta de teste direto do grafo,
mas nao e mais o unico caminho para fazer o robo andar por rota.

Depois dos testes com grafo, controller e speed filter, decidir se vale:

```text
manter route_to_poses como ferramenta de teste
evoluir route_to_poses para um node mais completo
evoluir a BT para ficar ainda mais parecida com a Oregon
integrar clique do RViz com o grafo
```

### 6. Teste De Rota Alternativa E Colisao No Grafo

Foi adicionada uma rota alternativa entre os nos `6` e `7`:

```text
6 -> 18 -> 7
```

O no `18` foi criado em:

```text
x=6.513256072998047
y=-21.23345184326172
```

Para validar se o `route_server` conseguia escolher a alternativa, a aresta
direta `6 <-> 7` foi temporariamente configurada com:

```json
"cost": 100.0,
"overridable": false
```

Com isso, a rota passou pela alternativa, confirmando que o grafo alternativo
esta valido.

Depois foi testado:

```yaml
CostmapScorer:
  invalid_on_collision: true
```

Resultado:

```text
route_to_poses retornou error_code=405
```

No `ComputeRoute`, o erro `405` significa:

```text
NO_VALID_ROUTE
```

Esse parametro faz o `route_server` validar as arestas do grafo contra o
`global_costmap` no momento em que a rota e calculada.

Com `invalid_on_collision: true`, uma aresta pode ser rejeitada antes do robo
comecar a andar se a linha dela passar por:

```text
parede do mapa
obstaculo detectado
area inflada pela inflation_layer
celula com custo alto no costmap
```

Assim, o erro pode acontecer logo no inicio do comando:

```bash
ros2 run sim_bot route_to_poses --ros-args \
  -p use_start:=False \
  -p start_id:=1 \
  -p goal_id:=10
```

Nesse caso o robo nao sai do lugar porque a falha ocorre ainda no
`/compute_route`, antes de qualquer path ser enviado para `/follow_path`.

Conclusao:

```text
o route_server deixou de aceitar a rota em colisao,
mas o grafo atual ainda ficou restritivo demais para encontrar outra rota valida.
```

Isso pode acontecer quando:

```text
alguma aresta do grafo passa perto demais de parede
a inflation do costmap encosta na aresta
o obstaculo bloqueia tambem a alternativa
o no alternativo ainda nao da folga suficiente
```

No teste atual foi identificado que existe pelo menos uma aresta do grafo
pegando ou encostando em uma parede. Com `invalid_on_collision: false`, o
`route_server` ainda calcula a rota e o problema aparece depois, no controller.
Com `invalid_on_collision: true`, o proprio `route_server` detecta essa aresta
como invalida e retorna `NO_VALID_ROUTE`.

Por enquanto, o parametro voltou para:

```yaml
invalid_on_collision: false
```

Esse valor mantem a navegacao funcional enquanto o grafo e refinado.

Para retomar esse teste depois, o proximo passo e afastar melhor os pontos do
grafo das paredes e testar trechos pequenos com `/compute_route` antes de
ativar `invalid_on_collision: true` novamente.

### Reteste Apos Correcao Do Grafo

Depois de corrigir as arestas que encostavam em parede, o parametro foi
reativado para novo teste:

```yaml
CostmapScorer:
  invalid_on_collision: true
```

Foram testados trechos isolados do grafo:

```text
1 -> 2
2 -> 3
3 -> 4
...
16 -> 17
```

Resultado:

```text
todos os trechos isolados testados passaram
```

Mesmo assim, a rota completa:

```bash
ros2 run sim_bot route_to_poses --ros-args \
  -p use_start:=False \
  -p start_id:=1 \
  -p goal_id:=10
```

ainda retornou:

```text
error_code=405
NO_VALID_ROUTE
```

Como as arestas isoladas passaram, a suspeita atual deixou de ser uma colisao
simples em uma aresta individual e passou a ser a operacao de rota:

```yaml
operations: ["AdjustSpeedLimit", "ReroutingService", "CollisionMonitor"]

CollisionMonitor:
  max_collision_dist: 3.0
```

Hipotese:

```text
o CollisionMonitor pode estar restritivo demais para o mapa do simulador,
principalmente com max_collision_dist: 3.0.
```

Proximo teste:

```text
remover temporariamente CollisionMonitor das operations
manter invalid_on_collision: true
testar novamente a rota completa 1 -> 10
```

Se a rota completa funcionar sem o `CollisionMonitor`, o problema nao esta mais
no grafo nem no `CostmapScorer`, mas no criterio de monitoramento de colisao da
operacao de rota.

### Isolamento Do CostmapScorer

Mesmo com o `CollisionMonitor` removido das `operations`, a rota completa:

```bash
ros2 run sim_bot route_to_poses --ros-args \
  -p use_start:=False \
  -p start_id:=1 \
  -p goal_id:=17
```

continuou retornando:

```text
error_code=405
NO_VALID_ROUTE
```

Foi feita uma validacao local do arquivo `graphs/aceleradoras.json`:

```text
grafo conectado
sem referencias faltando
sem IDs duplicados
caminho existente de 1 ate 17
```

Ou seja, o erro nao era falta de conectividade no JSON.

Em seguida, o `CostmapScorer` foi removido temporariamente da lista de
`edge_cost_functions`:

```yaml
edge_cost_functions: ["DistanceScorer"]
```

O bloco do `CostmapScorer` permaneceu no arquivo como referencia, mas sem estar
ativo na lista.

Resultado:

```text
rota 1 -> 17 voltou a funcionar
```

Conclusao:

```text
o causador do NO_VALID_ROUTE nesta configuracao foi o CostmapScorer.
```

Isso significa que o `CostmapScorer` estava lendo o `global_costmap` e
considerando algum trecho da rota invalido ou caro demais, mesmo com o grafo
aparentemente livre no mapa.

Estado atual recomendado para manter a navegacao funcional:

```yaml
operations: ["AdjustSpeedLimit", "ReroutingService", "CollisionMonitor"]
edge_cost_functions: ["DistanceScorer", "CostmapScorer"]
CostmapScorer:
  invalid_on_collision: false
```

Essa configuracao fica mais proxima do arquivo da Oregon:

```yaml
edge_cost_functions: ["DistanceScorer", "CostmapScorer"]
CostmapScorer:
  invalid_on_collision: false
```

O erro do teste anterior ocorreu porque foi tentado:

```yaml
invalid_on_collision: true
```

Com `true`, o `CostmapScorer` deixa de apenas pontuar o custo das arestas e
passa a invalidar rotas em colisao/custo alto. Isso deixou o simulador mais
restritivo que a Oregon e causou `NO_VALID_ROUTE`.

Resumo do diagnostico:

```text
grafo puro funcionando
route_server funcionando
route_to_poses funcionando
CostmapScorer com invalid_on_collision:true gerou NO_VALID_ROUTE
CostmapScorer deve permanecer ativo com invalid_on_collision:false para ficar proximo da Oregon
```

Proximo teste recomendado:

```bash
ros2 run sim_bot route_to_poses --ros-args \
  -p use_start:=False \
  -p start_id:=1 \
  -p goal_id:=17
```

Se funcionar, manter essa configuracao. Se voltar a falhar, isolar novamente o
`CollisionMonitor`, mas sem remover o `CostmapScorer` antes de testar
`invalid_on_collision:false`.

## Visualizacao Do Grafo No RViz

Para validar se o robo esta seguindo o grafo, foi criado um node simples de
visualizacao:

```text
scripts/graph_visualizer.py
```

Ele le:

```text
graphs/aceleradoras.json
```

e publica:

```text
/route_graph_markers
```

Tipo da mensagem:

```text
visualization_msgs/MarkerArray
```

O visualizador mostra:

```text
linhas azuis para as arestas
esferas amarelas para os nos
texto branco com o ID de cada no
```

Comando:

```bash
ros2 run sim_bot graph_visualizer
```

Se quiser passar outro arquivo de grafo:

```bash
ros2 run sim_bot graph_visualizer --ros-args \
  -p graph_file:=$PWD/src/sim_bot/graphs/aceleradoras.json
```

No RViz:

```text
Add
By topic
/route_graph_markers
MarkerArray
```

Esse node nao altera a navegacao. Ele serve apenas para depurar o grafo e
confirmar visualmente:

```text
onde estao os nos
quais arestas existem
se alguma aresta passa perto de parede
se o caminho esperado faz sentido antes de testar o route_server
```

Como usa QoS `transient local`, o RViz deve receber os markers mesmo se o
display for adicionado depois que o node ja estiver rodando.

## Baseline Validado

Estado validado no simulador:

```text
Palmares no Gazebo
AMCL com mapa aceleradora
Nav2 usando config/oregon_nav_params.yaml
route_server carregando graphs/aceleradoras.json
BT btree/nav_on_route_graph_sim.xml ativa no 2D Goal Pose
CostmapScorer ativo com invalid_on_collision:false
CollisionMonitor ativo nas operations
speed_filter ativo e testado
graph_visualizer publicando /route_graph_markers
```

Fluxo validado pelo RViz:

```text
RViz 2D Goal Pose
  -> bt_navigator
  -> nav_on_route_graph_sim.xml
  -> ComputeRoute
  -> route_server
  -> graphs/aceleradoras.json
  -> SmoothPath
  -> FollowPath
  -> controller_server
  -> /cmd_vel
```

Comando principal:

```bash
ros2 launch sim_bot palmares_bot.launch.py \
  world:=$PWD/src/sim_bot/worlds/aceleradora.world \
  slam:=False \
  nav:=True \
  rviz:=True \
  joy:=False \
  route:=True \
  speed_filter:=True \
  map:=$PWD/src/sim_bot/config_map/aceleradora/aceleradora.yaml \
  params_file:=$PWD/src/sim_bot/config/oregon_nav_params.yaml
```

Comando para visualizar o grafo:

```bash
ros2 run sim_bot graph_visualizer
```

No RViz:

```text
Add -> By topic -> /route_graph_markers -> MarkerArray
```

Comando de diagnostico por IDs:

```bash
ros2 run sim_bot route_to_poses --ros-args \
  -p use_start:=False \
  -p start_id:=1 \
  -p goal_id:=17
```

## Comparacao Com Oregon

### route_server

Config Oregon:

```yaml
operations: ["AdjustSpeedLimit", "ReroutingService", "CollisionMonitor"]
ReroutingService:
  plugin: "nav2_route::ReroutingService"
AdjustSpeedLimit:
  plugin: "nav2_route::AdjustSpeedLimit"
CollisionMonitor:
  plugin: "nav2_route::CollisionMonitor"
  max_collision_dist: 3.0
edge_cost_functions: ["DistanceScorer", "CostmapScorer"]
DistanceScorer:
  plugin: "nav2_route::DistanceScorer"
CostmapScorer:
  plugin: "nav2_route::CostmapScorer"
  invalid_on_collision: false
```

Config atual do simulador:

```yaml
operations: ["AdjustSpeedLimit", "ReroutingService", "CollisionMonitor"]
ReroutingService:
  plugin: "nav2_route::ReroutingService"
AdjustSpeedLimit:
  plugin: "nav2_route::AdjustSpeedLimit"
CollisionMonitor:
  plugin: "nav2_route::CollisionMonitor"
  max_collision_dist: 3.0
edge_cost_functions: ["DistanceScorer", "CostmapScorer"]
DistanceScorer:
  plugin: "nav2_route::DistanceScorer"
CostmapScorer:
  plugin: "nav2_route::CostmapScorer"
  invalid_on_collision: false
```

Conclusao:

```text
route_server esta alinhado com a Oregon nessa parte.
```

A diferenca principal esta no arquivo de grafo:

```text
Oregon: grafo/mapa real do robo
sim_bot: graphs/aceleradoras.json adaptado para o mapa aceleradora
```

### Behavior Tree

Oregon:

```text
nav_hub/btree/nav_on_route_graph_oregon.xml
```

Simulador:

```text
btree/nav_on_route_graph_sim.xml
```

Objetivo da BT do simulador:

```text
manter a mesma ideia da Oregon:
calcular rota por grafo,
conectar primeiro/ultimo trecho quando necessario,
suavizar caminho,
seguir com FollowPath.
```

### Ajuste Da BT Para Ficar Mais Proxima Da Oregon

A BT do simulador foi ajustada para seguir a estrutura da
`nav_on_route_graph_oregon.xml`.

Estrutura atual:

```text
NavigateWithRoutes
  -> ControllerSelector
  -> PlannerSelector
  -> PlanningRecovery
      -> ComputeFullRoute
          -> ComputeRoute
          -> FirstMileCheck
          -> LastMileCheck
          -> EnsureGoalOnPath
          -> SmoothPath
  -> FollowPath
```

Isso substituiu a versao anterior que usava:

```text
NavigateRecovery
PipelineSequence
RateController
IsPathValid
FollowRoutePath recovery separado
```

A BT atual ficou mais proxima da Oregon em:

```text
nome da sequencia principal: NavigateWithRoutes
RecoveryNode: PlanningRecovery com number_of_retries=9999
ComputeFullRoute como sequencia principal de planejamento
EnsureGoalOnPath explicito
SmoothPath usando final_path
FollowPath direto depois do planejamento
```

Diferenças mantidas por causa do simulador:

```text
GetCurrentPose usa global_frame="map"
GetCurrentPose usa robot_base_frame="base_link"
ArePosesNear usa global_frame="map"
```

Na Oregon, o frame base esperado era `base_footprint`. No `palmares_bot` atual,
o frame de navegação validado e `base_link`, por isso essa diferença foi
mantida.

Proximo teste necessario depois dessa alteracao:

```text
subir palmares_bot.launch.py com route:=True e speed_filter:=True
abrir RViz
rodar graph_visualizer
enviar 2D Goal Pose
confirmar se o robo continua seguindo o grafo
```

### CostmapScorer

Foi testado:

```yaml
invalid_on_collision: true
```

Resultado:

```text
ComputeRoute retornou 405 NO_VALID_ROUTE
```

Decisao:

```text
manter invalid_on_collision:false, igual a Oregon.
```

Motivo:

```text
com false, o CostmapScorer participa do custo da rota,
mas nao invalida a rota inteira por custo/colisao no global_costmap.
```

## Upgrades Futuros

Lista de melhorias para aproximar ainda mais da Oregon ou transformar em
modelo generico para outros robos:

```text
1. Comparar nav_on_route_graph_sim.xml com nav_on_route_graph_oregon.xml linha por linha
2. Avaliar se o nav_hub/main_route_graph precisa ser portado ou substituido por node generico
3. Parametrizar grafo, mapa, mascara e BT por launch arguments
4. Separar parametros especificos do Palmares dos parametros genericos de rota
5. Criar visualizacao opcional do grafo direto no launch
6. Testar rotas alternativas reais no grafo, com custo e speed limit por aresta
7. Investigar uso seguro de invalid_on_collision:true apenas depois de validar costmap/grafo
8. Avaliar STVL/ToF somente quando houver sensor 3D equivalente no simulador
```

Ponto de atencao:

```text
na Oregon, alguns plugins e configuracoes podem depender de pacotes customizados.
No simulador, manter primeiro os plugins padrao do Nav2 funcionando antes de
substituir por implementacoes especificas do robo real.
```
