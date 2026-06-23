# Documento de Passagem de Conhecimento
## Projeto: palmares_bot — Docking Autônomo em Simulação

**Projeto:** `sim_ws/src/sim_bot`
**Stack:** ROS2 Jazzy + Gazebo Harmonic + Nav2 + slam_toolbox + opennav_docking
**Última atualização:** 2026-06-22
**Sessões de desenvolvimento:** 13 sessões (2026-06-11 a 2026-06-18)

---

## 1. Objetivo Geral

O robô **palmares_bot** realiza um ciclo completamente autônomo de carregamento em uma estação de docking (`charging_dock`) instalada na parede de um galpão de fábrica simulado. O ciclo completo é:

1. Robô nasce em `odom(0,0)` no centro do galpão
2. Após ~55s (Nav2 em t=20s, docking server em t=25s, AUTOSTART_DELAY=30s → total t=55s), navega autonomamente ~12m até a base de carregamento em `odom(13,0)`
3. Executa a aproximação precisa usando LiDAR (e fallback com câmera + ArUco)
4. Fica em `HOME_CHARGING` carregando a bateria simulada
5. Quando bateria ≥ 40%, aceita tarefas da fila (ir a um ponto, inspecionar, patrulhar, entregar)
6. Ao final de cada tarefa, retorna autonomamente ao dock
7. O ciclo se repete indefinidamente

---

## 2. Estrutura de Arquivos

```
sim_ws/src/sim_bot/
├── config/
│   ├── docking_params.yaml     # Parâmetros do opennav DockingServer + plugin
│   ├── gz_bridge.yaml          # Bridges Gazebo ↔ ROS2 (scan, odom, cmd_vel, camera)
│   ├── nav_params.yaml         # Nav2 completo (bt_navigator, costmaps, planner, etc.)
│   ├── slam_params.yaml        # slam_toolbox (online async)
│   ├── twist_mux_params.yaml   # Multiplexador de velocidade (nav vs teleop vs charging)
│   ├── usb_cam_params.yaml     # Driver usb_cam: C270 640×480 MJPG @ 30fps (robô real)
│   └── joy_params.yaml         # Parâmetros do joystick
├── description/
│   ├── palmares_bot.urdf.xacro # URDF principal do robô
│   ├── lidar.xacro             # Sensor LiDAR 2D (RPLidar-like)
│   ├── camera.xacro            # Câmera RGB frontal
│   ├── common.xacro            # Macros de inércia e materiais
│   └── ...
├── launch/
│   ├── palmares_bot.launch.py       # Launch principal (Gazebo + SLAM + Nav2 + docking)
│   ├── palmares_bot_real.launch.py  # Launch para hardware real (sem Gazebo)
│   ├── docking.launch.py            # Launch do docking (simulação, use_sim_time=True)
│   ├── docking_real.launch.py       # Launch do docking (hardware real, use_sim_time=False)
│   ├── camera_real.launch.py        # Launch da câmera USB C270 (robô real)
│   ├── slam.launch.py               # slam_toolbox isolado
│   ├── nav.launch.py                # Nav2 isolado
│   └── ...
├── models/
│   └── charging_dock/
│       ├── model.config        # Metadados do modelo Gazebo
│       └── model.sdf           # Modelo 3D da estação de carregamento
├── nodes/
│   ├── charging_manager.py     # Orquestrador principal: bateria + fila + estados
│   ├── dock_pose_estimator.py  # Estimador de pose do dock (ArUco + LiDAR)
│   └── odom_to_tf.py           # Relay /odom → TF odom→base_footprint
├── worlds/
│   ├── factory.world           # Galpão de fábrica 30×50m (mundo atual)
│   └── test.world              # Mundo pequeno de testes (backup)
├── meshes/
│   ├── palmares.stl            # Malha 3D do corpo do robô
│   └── base_carregamento.stl   # Malha da base de carregamento
├── rviz/bot.rviz                    # Configuração do RViz
├── SESSAO_PROGRESSO.txt             # Histórico detalhado de todas as 13 sessões
├── README.md                        # Visão geral, início rápido e tabela de documentação
├── INSTALACAO.md                    # Instalação completa de todas as dependências
├── PASSAGEM_CONHECIMENTO.md         # Arquitetura, parâmetros, decisões de design
├── TUTORIAL_CAMERA_REAL.md          # Configurar câmera C270 física no robô real
└── TROUBLESHOOTING.md               # Erros comuns e soluções rápidas
```

---

## 3. Stack de Software

### 3.1 ROS2 Jazzy

Distribuição ROS2 LTS (Ubuntu 24.04). Todos os nós usam rclpy (Python). O workspace é `~/sim_ws` com um único pacote `sim_bot`.

**Build:**
```bash
cd ~/sim_ws
colcon build --packages-select sim_bot
source install/setup.bash
```

### 3.2 Gazebo Harmonic

Simulador 3D integrado via `ros_gz_sim`. Usa o sistema de bridge `ros_gz_bridge` para converter tópicos Gazebo ↔ ROS2.

**Plugins utilizados no URDF:**
- `gz-sim-diff-drive-system` — Controle diferencial; publica `/odom` e TF `odom→base_footprint`
- `gz-sim-joint-state-publisher-system` — Publica `/joint_states` das rodas

**Bridge configurado em `gz_bridge.yaml`:**
- `/scan` — LaserScan do LiDAR
- `/odom` — Odometria do drive diferencial
- `/cmd_vel` — Comandos de velocidade → Gazebo
- `/camera/image` — Imagem RGB
- `/camera/camera_info` — Parâmetros intrínsecos da câmera

### 3.3 slam_toolbox

Mapeamento e localização simultâneos (SLAM) em modo `online_async`. Produz a TF `map→odom` e o tópico `/map`.

**Parâmetro importante:**
- `map_update_interval: 15.0` — Reduzido de 5s para 15s para diminuir TF jumps que causavam drift no odom

**Atenção ao SLAM drift:** Em ambientes grandes (>5m), o SLAM acumula drift no frame `odom`. Isso causa desvio cosmético no RViz mas **não** significa que o robô está fisicamente desalinhado. O `dock_pose_estimator` usa dados absolutos (LiDAR/ArUco) que são independentes do odom — portanto, confiar no status=4 do DockRobot é mais confiável que verificar Y/yaw via odom.

### 3.4 Nav2

Stack de navegação completa. Configurada em `nav_params.yaml`.

**Componentes ativos:**
- `bt_navigator` — Árvore de comportamento; `global_frame: odom` (não `map`)
- `controller_server` — DWBLocalPlanner para seguimento de trajetória
- `planner_server` — SmacPlanner2D (MOORE, allow_unknown) para planejamento global
- `local_costmap` — 3×3m, rolling window, sensor: LiDAR
- `global_costmap` — 25×25m fixo, origin(-5,-12), cobre X[-5,20] Y[-12,13]
- `behavior_server` — spin, backup, drive_on_heading, wait

**Por que `global_frame: odom`?** O frame `map` foi abandonado (sessão 6) porque TF jumps do SLAM causavam comportamentos erráticos no bt_navigator. Usando `odom` como frame global, a navegação fica estável e sem jumps.

**Tolerâncias críticas:**
- `xy_goal_tolerance: 0.25` m — Nav2 considera meta atingida dentro de 25cm
- `dock_prestaging_tolerance: 0.5` m — deve ser > `xy_goal_tolerance` para evitar falha intermitente na borda

### 3.5 opennav_docking

Pacote de docking autônomo. Fornece a action `dock_robot` (DockRobot) que:
1. Navega até o pose de staging (1m à frente do dock)
2. Usa o `dock_pose_estimator` para detectar o dock em tempo real
3. Executa a abordagem via `GracefulController`
4. Declara sucesso quando `docking_threshold: 0.05m` é atingido

**Plugin:** `opennav_docking::SimpleChargingDock`

**`GracefulController`** — Controlador de abordagem suave com parâmetros:
- `k_phi: 3.0` — Ganho de correção de yaw
- `k_delta: 2.0` — Ganho de correção lateral (alto = mais curva na abordagem)
- `v_linear_min: 0.06` — Velocidade mínima (importante: muito baixa causa loop se threshold muito pequeno)
- `slowdown_radius: 0.4` — Distância em que começa a desacelerar

---

## 4. Modelo do Robô (URDF)

### 4.1 Geometria

```
Dimensões do corpo (STL, escalado 0.001):  533 × 460 × 428 mm
Massa total:                               35 kg
Rodas: raio=100mm, largura=50mm, separação=305mm (centro a centro)

Frente do robô (base_link):  X = +0.267m (metade de 0.533 m)
Offset front (colisão):      X = -0.0145 + 0.533/2 ≈ +0.252 m
                             → robot_front_offset = 0.252 m (usado para calcular stop_distance)
```

### 4.2 Sensores

**LiDAR** (`lidar.xacro`):
- Frame: `laser_frame`
- Posição absoluta: X=0.194m, Y=0.050m, Z=0.275m (em relação a base_link)
- Alcance: 0.30m – 12m (min_range importante para zona cega)
- Frequência: 10 Hz
- Resolução: 360 pontos / 360° = 1° por raio

**Câmera RGB** (`camera.xacro`):
- Frame: `camera_link` / `camera_link_optical`
- Posição: chassis joint + offset → altura absoluta = 0.265m *(medir no robô físico e ajustar o joint Z em `camera.xacro`)*
- Resolução simulação: 1280×720 URDF atual (C720-like) — **atualizar para 640×480 e FOV real da C270** (ver `TUTORIAL_CAMERA_REAL.md` Seção 8)
- Publica: `/camera/image`, `/camera/camera_info`

### 4.3 Árvore TF

```
map
 └── odom              ← publicada pelo slam_toolbox
      └── base_footprint ← publicada pelo odom_to_tf.py (relay do /odom)
           └── base_link  ← fixed (joint base_footprint_joint)
                ├── chassis (virtual link para montar sensores)
                │    ├── laser_frame
                │    └── camera_link
                │         └── camera_link_optical
                ├── left_wheel
                └── right_wheel
```

**Por que `odom_to_tf.py`?** O plugin DiffDrive do Gazebo publica TF diretamente, mas com timestamps de física (não monotônicos), causando "jump back in time" no TF buffer. O `odom_to_tf.py` faz relay do `/odom` para TF com `get_clock().now()`, garantindo monotonia.

---

## 5. Mundo de Simulação (factory.world)

Galpão de fábrica baseado no modelo `Warehouse` da OpenRobotics/Gazebo Fuel.

**Dimensões:** ~30×50m  
**Obstáculos mapeados pelo Nav2:**
- Prateleiras grandes (shelf_big): linhas de 18m — SLAM mapeia como paredes
- Prateleiras simples (shelf): individuais nas bordas
- Barreiras de concreto (Jersey Barrier): corredor de segurança em X~-10

**Dock:**
- Posição no mundo: `(13.0, 0.0, 0.0)` — encostado na parede direita
- A face do ArUco aponta em direção `-X` (interior do galpão)
- O robô aborda em direção `+X`

**Iluminação:** Sol direcional + fill light focado no ArUco (para garantir detecção mesmo em luz fraca).

**Coordenadas importantes (odom = world — spawn em world origin):**
```
Spawn do robô:    odom(0.0,  0.0)
Dock:             odom(13.0, 0.0)
Staging:          odom(12.0, 0.0)  — 1m à frente da face
Face do dock:     odom(13.235, 0.0) — origem 13.0 + offset local 0.235m
Área de tarefas:  odom x < 10 (interior, longe do dock)
```

---

## 6. Nó: `dock_pose_estimator.py`

**Responsabilidade:** Publicar continuamente `/detected_dock_pose` (PoseStamped em `odom`) para o `opennav_docking` usar como referência de alvo durante a abordagem.

### 6.1 Duas Fases de Detecção

**Fase 1 — ArUco (longo alcance: ~0.5m a ~3m):**
- Detecta o marcador ArUco ID=771 (DICT_4X4_1000) via câmera RGB
- Usa `cv2.solvePnP` (método `IPPE_SQUARE`) para calcular pose 3D do marcador
- Transforma para frame `odom` via TF
- Publica target = centro_marcador - `stop_distance` na direção de abordagem

**Fase 2 — LiDAR (curto alcance: até ~0.5m, quando câmera perde o ArUco):**
- Analisa setor frontal do `/scan` (±30° normal, ±60° quando range < 0.5m)
- Calcula `face_x` como média dos pontos no centro (±20°) — evita subestimativa por raios oblíquos
- Calcula `center_y` pelo spread lateral dos pontos da face
- Para Y: usa `last_dock_odom[1]` do ArUco se disponível (mais estável que LiDAR edge)
- Publica target = `(face_x - stop_distance, dock_y_fixo, yaw=0)`

**Por que Y fixo do ArUco?** O LiDAR edge detection carrega offset Y do robô no staging (se robô chegou 3cm de lado, oy = +3cm → para 3cm fora do centro). Usar o Y do ArUco como verdade elimina esse bias.

### 6.2 Cache na Zona Cega

O LiDAR tem `min_range = 0.30m`. Quando o robô fica a menos de ~24cm da face, o LiDAR fica cego e para de publicar. A zona cega (~0.9s de duração, percorrendo ~9cm) é coberta pelo parâmetro `external_detection_timeout: 2.0s` no `docking_params.yaml`: o `docking_server` continua usando a última pose conhecida por até 2s antes de declarar falha. Se o `min_range` mudar para o LiDAR real, ajustar a distância de zona cega e verificar que está dentro do timeout.

### 6.3 Parâmetros Dinâmicos

`stop_distance` pode ser ajustado sem restart (padrão = 0.30m):
```bash
ros2 param set /dock_pose_estimator stop_distance 0.35
```

### 6.4 Parâmetros (docking.launch.py)

| Parâmetro | Valor | Descrição |
|-----------|-------|-----------|
| `marker_id` | 771 | ID do marcador ArUco |
| `marker_size` | 0.15m | Tamanho físico do marcador impresso |
| `stop_distance` | 0.30m | Offset da face → target (robot_front=0.252 + 5cm gap) |
| `aruco_timeout` | 2.0s | Tempo sem ArUco antes de LiDAR assumir |
| `sector_half_deg` | 30° | Setor normal do LiDAR |
| `close_sector_deg` | 60° | Setor largo (quando range < 0.5m) |
| `close_range_m` | 0.5m | Threshold para trocar para setor largo |

---

## 7. Nó: `charging_manager.py`

**Responsabilidade:** Orquestrar o ciclo completo — bateria simulada, fila de tarefas, e máquina de estados que controla quando navegar, undock, executar tarefas e retornar.

### 7.1 Máquina de Estados

```
STARTUP
  ↓ (após AUTOSTART_DELAY=30s)
RETURNING_HOME → DOCKING → HOME_CHARGING ←───────────┐
                                ↓ (bat≥40% e fila)    │
                            UNDOCKING                  │
                                ↓                      │
                          GOING_TO_TASK                │
                                ↓                      │
                          EXECUTING_TASK               │
                                ↓                      │
                          RETURNING_HOME ──────────────┘
```

**Estados especiais:**
- `NAVIGATING_TO_STAGING` / `CENTERING`: estados de abordagem manual (não usados atualmente com `navigate_to_staging_pose=True`)
- Qualquer estado pode receber `/go_charge` para forçar retorno imediato

### 7.2 Bateria Simulada

| Parâmetro | Valor | Resultado |
|-----------|-------|-----------|
| `BATT_DRAIN` | 0.5%/s | 100→0% em 200s (~3.3 min) |
| `BATT_CHARGE` | 1.0%/s | 0→100% em 100s (~1.7 min) |
| `MIN_FOR_TASK` | 40% | Aceita tarefa da fila quando acima |
| `LOW_AFTER` | 20% | Retorna ao dock após tarefa |
| `EMERGENCY` | 5% | Aborta tarefa imediatamente |

### 7.3 Fila de Tarefas (FIFO)

Quatro tipos de tarefa, publicados via tópico:

| Tópico | Tipo | Descrição | Espera |
|--------|------|-----------|--------|
| `/task/goto_pose` | `PoseStamped` | Vai a um ponto e espera | 2s |
| `/task/inspect` | `PoseStamped` | Vai a um ponto (inspeção) | 5s |
| `/task/deliver` | `PoseStamped` | Vai a um ponto (entrega) | 3s |
| `/task/patrol` | `String` ("A","B","C") | Percorre rota de waypoints | 0s |

**Rotas de patrulha (odom):**
```python
PATROL_ROUTES = {
    'A': [(3,7,0), (-4,7,1.57), (-4,-7,3.14)],   # corredor lateral longo
    'B': [(-2,3,0), (-7,0,0), (-2,-3,0)],          # corredor central esquerdo
    'C': [(5,-5,0), (-3,-5,1.57), (-3,5,0)],        # corredor inferior central
}
```

### 7.4 Retorno ao Dock

O undocking é simples (não usa `UndockRobot` action): publica `cmd_vel` com `linear.x = -0.15 m/s` por `2.0s` (~30cm de recuo), depois envia `DockRobot` com `navigate_to_staging_pose=True`.

`navigate_to_staging_pose=True` significa que o servidor:
1. Usa Nav2 para navegar até `staging = dock_pose - 1.0m em X`
2. A partir do staging, usa o `dock_pose_estimator` para abordagem precisa

### 7.5 State Guards (crítico para evitar race conditions)

Todos os callbacks assíncronos têm guards de estado antes de qualquer ação:

```python
def _task_nav_acc(self, f):
    if self._state != GOING_TO_TASK:
        return  # /go_charge mudou o estado antes deste callback

def _finish_task(self):
    if self._state != EXECUTING_TASK:
        return  # sem guards aqui → BUG: go_charge + timer = 2 DockRobots concorrentes
```

**Sem esses guards:** um `/go_charge` durante `EXECUTING_TASK` mudava o estado para `DOCKING`, mas o timer de `_finish_task` ainda disparava depois, chamando `_start_returning_home()` → segundo `DockRobot` concorrente → caos na abordagem (robô andando em zigue-zague ou em loop infinito).

### 7.6 Lógica de Falha no Docking

```python
# DockRobot retornou status != 4:
dock_retries += 1
if dock_retries >= DOCK_MAX_FAILS(4):
    dist = sqrt((robot_x - 12.7)^2 + robot_y^2)
    if dist <= ALIGN_TOL_XY(2.0m):
        → HOME_CHARGING (robô está perto, aceita sucesso)
    else:
        → retry em 15s (robô longe, não força HOME_CHARGING)
else:
    → retry em 3s
```

**Nunca forçar `HOME_CHARGING` longe do dock:** o bug clássico foi o robô em `odom(0,0)` passando no check de Y+yaw (ambos ≈0) e entrando em `HOME_CHARGING` a 13m do dock. Resultado: `/go_charge` era ignorado pois estado já era `HOME_CHARGING`.

---

## 8. Nó: `odom_to_tf.py`

**Problema resolvido:** O plugin `DiffDrive` do Gazebo publicava TF `odom→base_footprint` com timestamps de física (podem saltar para trás no tempo). Isso causava `NoDataForExtrapolationException` no slam_toolbox e outros nós.

**Solução:** Este nó assina `/odom` e re-publica como TF usando `get_clock().now()` como timestamp (tempo de sistema, monotônico). Garante que a TF está sempre "à frente" dos scans que chegam.

---

## 9. Sequência de Inicialização

O `palmares_bot.launch.py` usa `TimerAction` para garantir ordem de inicialização:

```
t=0s    Gazebo server, bridge, robot_state_publisher, odom_to_tf, joint_state_pub
t=0s    RViz (se habilitado), joystick (se habilitado), twist_mux
t=5s    slam_toolbox (aguarda TF odom→base_footprint do odom_to_tf)
t=20s   Nav2 (aguarda map→odom do slam_toolbox + scans do Gazebo)
t=25s   docking_server + dock_pose_estimator + charging_manager
t=55s   charging_manager dispara autostart → RETURNING_HOME → DOCKING
t≈100s  HOME_CHARGING (robô chegou e está no dock)
```

**Por que 30s de AUTOSTART_DELAY?** O `charging_manager` sobe junto com o docking server em t=25s (`TimerAction`). Com `AUTOSTART_DELAY=30s`, o autostart dispara em t=55s — quando Nav2 (~20s) e docking server (~25s) já estão estáveis há pelo menos 30s. No código: `charging_manager.py` linha 96.

---

## 10. Como Executar

### 10.1 Iniciar a simulação completa

```bash
# Terminal 1 — matar processos anteriores e lançar:
pkill -9 -f "gz sim|dock_pose_estimator|odom_to_tf|slam_toolbox|nav2|docking|twist_mux|robot_state_publisher|charging_manager" 2>/dev/null; sleep 5
cd ~/sim_ws && colcon build --packages-select sim_bot && source install/setup.bash
ros2 launch sim_bot palmares_bot.launch.py
```

### 10.2 Monitorar estado e bateria

```bash
# Terminal 2:
source ~/sim_ws/install/setup.bash
watch -n 2 "echo '=== ESTADO ===' && ros2 topic echo /charging_manager/state --once 2>/dev/null && echo '=== BATERIA ===' && ros2 topic echo /battery_level --once 2>/dev/null"
```

### 10.3 Enviar tarefas (aguardar HOME_CHARGING + bat ≥ 40%)

```bash
source ~/sim_ws/install/setup.bash

# Patrulha rota A (corredor lateral)
ros2 topic pub --once /task/patrol std_msgs/msg/String '{data: "A"}'

# Inspeção — área de prateleiras
ros2 topic pub --once /task/inspect geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: "odom"}, pose: {position: {x: -4.0, y: -10.0}, orientation: {w: 1.0}}}'

# Goto — ponto central do galpão
ros2 topic pub --once /task/goto_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: "odom"}, pose: {position: {x: 2.0, y: 0.0}, orientation: {w: 1.0}}}'

# Forçar retorno imediato ao dock:
ros2 topic pub --once /go_charge std_msgs/msg/Empty '{}'
```

### 10.4 Diagnóstico

```bash
# Verificar detecção do dock:
ros2 topic echo /detected_dock_pose

# Posição atual do robô:
ros2 topic echo /odom --once

# Frequência da câmera:
ros2 topic hz /camera/image

# Estado da máquina de estados:
ros2 topic echo /charging_manager/state --once

# Ajustar stop_distance sem restart (0.30 é o padrão; exemplo de ajuste fino):
ros2 param set /dock_pose_estimator stop_distance 0.35

# Árvore TF completa:
ros2 run tf2_tools view_frames
```

---

## 11. Parâmetros Críticos (Referência Rápida)

### Dock e Geometria

| Parâmetro | Valor | Local |
|-----------|-------|-------|
| Dock pose (odom) | `[13.0, 0.0, 0.0]` | `docking_params.yaml` |
| Staging (odom) | `(12.0, 0.0)` | Calculado: dock - 1.0m |
| Face do dock | `13.235m` | dock 13.0 + offset local 0.235m |
| `stop_distance` | `0.30m` | `docking.launch.py` |
| `robot_front_offset` | `0.252m` | URDF (`base_link` → frente) |
| Gap nominal | `≈16.8cm` | `13.235 - (12.985 + 0.252)` |
| `docking_threshold` | `0.05m` | `docking_params.yaml` |
| `dock_prestaging_tolerance` | `0.50m` | `docking_params.yaml` |

### LiDAR e Zona Cega

| Parâmetro | Valor | Impacto |
|-----------|-------|---------|
| `min_range` (LiDAR) | `0.30m` | LiDAR fica cego a < 0.30m |
| LiDAR posição (base_link) | `+0.194m` em X | |
| Zona cega começa | face < 0.494m do base_link | Robô ainda 24cm da face |
| Duração zona cega | ~0.9s (9cm percorridos) | Coberta por keepalive cache |

### Navegação

| Parâmetro | Valor | Local |
|-----------|-------|-------|
| `xy_goal_tolerance` | `0.25m` | `nav_params.yaml` |
| Global costmap | 25×25m, origin(-5,-12) | `nav_params.yaml` |
| `global_frame` | `odom` (não `map`) | `nav_params.yaml` |
| Planner | SmacPlanner2D MOORE | `nav_params.yaml` |

---

## 12. Bugs Conhecidos e Lições Críticas

### 12.1 SLAM Drift em Ambientes Grandes

**Problema:** Em galpões de 12m+, o SLAM acumula drift cosmético que faz o robô parecer torto no RViz, mas ele está fisicamente correto. Verificar Y/yaw via odom falha nesses casos.

**Solução:** Confiar **exclusivamente** em `DockRobot status=4` + `docking_threshold=0.05m` como critério de sucesso. O `dock_pose_estimator` usa LiDAR/ArUco (absolutos) que não dependem do odom.

### 12.2 filter_coef Baixo = Atropelamento

**Problema:** `filter_coef: 0.1` (EMA lento) demora >3s para sair do valor inicial (13.0m YAML) para o target real (~12.875m). Robô persegue target filtrado em ~12.96m → atropela o dock.

**Solução:** Usar `filter_coef ≥ 0.5`. Valor atual: `0.1` (no YAML), mas `stop_distance` foi ajustado para compensar.

### 12.3 docking_threshold Muito Pequeno = Loop Infinito

**Problema:** Com `docking_threshold: 0.02m`, o `GracefulController` com `v_linear_min=0.06 m/s` fisicamente não consegue parar exatamente a 2cm. `isDocked()` nunca retorna true → timeout → retry → loop.

**Solução:** `docking_threshold: 0.05m` (5cm).

### 12.4 navigate_to_staging_pose=True em Retry = CAOS

**Problema:** Se o DockRobot falha e o servidor tenta de novo com `navigate_to_staging_pose=True` quando o robô já está em X=13.1m (passou do staging em 12.0m), Nav2 planeja um path **para trás** → `GracefulController` empurra para frente sem parar.

**Mitigação:** `DOCK_MAX_FAILS=4` com pausa de 15s entre tentativas. Para robôs longe do dock, sempre reinicia do staging.

### 12.5 wait_for_server() em Callback Bloqueia Executor

**Problema:** Chamar `action_client.wait_for_server()` dentro de um callback ROS bloqueia o executor inteiro (single-thread), prevenindo outros callbacks e timers de rodar.

**Solução:** Usar `server_is_ready()` (não bloqueante) + `_once()` (timer de retry):
```python
if not self._nav.server_is_ready():
    self._once(3.0, self._start_going_to_task)
    return
```

### 12.6 dock_prestaging_tolerance ≤ xy_goal_tolerance

**Problema:** Se ambos são iguais (ex: 0.25m), o robô para exatamente na borda da tolerância. O docking server às vezes considera que o robô não chegou ao staging → falha intermitente.

**Solução:** `dock_prestaging_tolerance = 2 × xy_goal_tolerance` = 0.50m.

### 12.7 HOME_CHARGING Sem Estar Fisicamente no Dock

**Problema histórico (sessão 9):** Com spawn em `odom(0,0,yaw=0)`, o check antigo verificava apenas `|y_err| < 5cm` e `|yaw| < 10°`. Como o robô nsce em y=0, yaw=0, o check passava e o robô entrava em `HOME_CHARGING` a 13m do dock. Resultado: `/go_charge` era ignorado.

**Solução:** Check 2D de distância: `sqrt((robot_x - 12.7)^2 + robot_y^2) <= 2.0m`. Spawn em X=0 fica a 12.7m → bloqueado. Robô dockado fica em X≈12.9m → OK.

### 12.8 Race Conditions em Callbacks Async (Bug Principal da Sessão 10)

**Problema:** `/go_charge` durante `EXECUTING_TASK`:
1. `/go_charge` → `_task=None`, `_start_returning_home()` → `DOCKING`
2. Timer de 2s de `_finish_task` ainda pendente → dispara → chama `_start_returning_home()` novamente
3. Dois `DockRobot` concorrentes → robô em zigue-zague ou loop

**Solução:** State guard em `_finish_task`:
```python
def _finish_task(self):
    if self._state != EXECUTING_TASK:
        return
```
E em todos os outros callbacks: `_task_nav_acc`, `_patrol_wp_acc`, `_dock_acc`, etc.

---

## 13. Migração para a Câmera Real (C270)

> **Atenção:** O URDF atual simula uma câmera **C720-like** (1280×720 @ 30fps, FOV horizontal 68.8°). O robô físico usa uma **Logitech C270** (640×480 @ 30fps, FOV horizontal ~50°). Os parâmetros de simulação **não** correspondem ao hardware real — isso afeta diretamente o `solvePnP` do ArUco e precisa ser corrigido antes de qualquer teste em hardware.

### 13.1 O Que Mudar e Onde

**Arquivo: [description/camera.xacro](description/camera.xacro)**

| Linha | Parâmetro | Valor atual (C720-sim) | Valor correto (C270) |
|-------|-----------|------------------------|----------------------|
| 49 | `<horizontal_fov>` | `1.20` rad (68.8°) | `~0.87` rad (50°) |
| 53 | `<width>` | `1280` | `640` |
| 54 | `<height>` | `720` | `480` |

O trecho a ser alterado fica em `camera.xacro` entre as tags `<camera>`:

```xml
<!-- ANTES (C720-like) -->
<horizontal_fov>1.20</horizontal_fov>
<image>
    <format>R8G8B8</format>
    <width>1280</width>
    <height>720</height>
</image>

<!-- DEPOIS (C270) -->
<horizontal_fov>0.87</horizontal_fov>   <!-- ~50° horizontal -->
<image>
    <format>R8G8B8</format>
    <width>640</width>
    <height>480</height>
</image>
```

> **Atenção:** O valor `0.87 rad` é aproximado. O FOV real depende da calibração. Use o valor da calibração após o procedimento descrito na seção 13.2.

**Arquivo: [launch/docking.launch.py](launch/docking.launch.py)**

| Linha | Parâmetro | Valor atual | O que verificar |
|-------|-----------|-------------|-----------------|
| 21 | `'marker_size'` | `0.15` m | Medir o marcador ArUco impresso fisicamente (borda externa incluindo quiet zone) |

O `marker_size` é o lado do marcador incluindo a borda preta exterior (quiet zone). Se o marcador impresso tiver, por exemplo, 20cm de lado total, use `0.20`. Erro nesse valor causa estimativa de distância errada no `solvePnP`.

**Outros parâmetros a revisar após troca de câmera:**

- `stop_distance` em `docking.launch.py` — calibrar empiricamente (ver seção 11)
- `external_detection_translation_y` em `docking_params.yaml` — verificar se há bias lateral residual
- Altura do `camera_joint` em `camera.xacro` (atualmente `Z=0.09`) — confirmar alinhamento vertical com o ArUco físico

### 13.2 Calibração da Câmera (camera_calibration)

A calibração gera a matriz intrínseca `K` e os coeficientes de distorção `D` que o `dock_pose_estimator.py` usa via `/camera/camera_info`. Sem calibração, o `solvePnP` devolve posições erradas e o robô para no lugar errado.

**Passo 1 — Instalar:**
```bash
sudo apt install ros-jazzy-camera-calibration
```

**Passo 2 — Preparar tabuleiro de xadrez:**
- Tabuleiro padrão 8×6 quadrados internos
- Tamanho do quadrado: 25mm (ajustar conforme o tabuleiro que tiver)

**Passo 3 — Executar a calibração com a câmera publicando:**
```bash
source ~/sim_ws/install/setup.bash
ros2 run camera_calibration cameracalibrator \
  --size 8x6 \
  --square 0.025 \
  --ros-args \
  --remap image:=/camera/image \
  --remap camera:=/camera
```

**Passo 4 — Coletar imagens:** Mover o tabuleiro em diferentes posições, distâncias e inclinações até as barras de progresso ficarem cheias (X, Y, Size, Skew).

**Passo 5 — Calibrar e salvar:** Clicar em "CALIBRATE" → "SAVE" → "COMMIT". O arquivo YAML é salvo em `~/.ros/camera_info/`.

**Passo 6 — Configurar o nó de câmera** para carregar o arquivo de calibração:
```bash
# No lançamento da câmera (real ou simulada com parâmetros corretos):
--ros-args -p camera_info_url:=file:///home/robo02/.ros/camera_info/camera.yaml
```

**O que fazer com os valores calibrados:**
- `fx`, `fy` da matriz K → `horizontal_fov = 2 * atan(width / (2 * fx))`
- Usar esse FOV no `camera.xacro` para manter simulação consistente com hardware

---

## 14. Limitações do LiDAR Sem ArUco

O projeto tem um branch `carregamento-sem-cam` que tenta docking apenas com LiDAR. Esta seção documenta todos os problemas encontrados e **por que o sistema atual usa ArUco como âncora primária**, com LiDAR como fallback.

### 14.1 Sem Referência Absoluta de Y (Bias de Staging)

**O problema mais crítico.** O `dock_pose_estimator.py` tem este trecho:

```python
# dock_pose_estimator.py — _scan_cb()
if self.last_dock_odom is not None:
    oy = self.last_dock_odom[1]   # Y do ArUco (verdade absoluta)
else:
    oy = pt_odom.point.y          # Y do LiDAR (carrega offset do staging)
```

Quando o robô usa Nav2 para chegar ao staging em `odom(12,0)`, a tolerância `xy_goal_tolerance=0.25m` permite que ele pare em `y = ±0.25m`. Se parar em `y = +0.03m`, o LiDAR mede o dock em `y_laser ≈ -0.03m` (dock 3cm à direita relativo ao robô). Após transformar para odom: `oy ≈ +0.03m`. O estimador publica target em `y=+0.03m`. O robô para 3cm fora do centro do dock.

**Com ArUco:** O marcador fornece a posição absoluta do dock em odom. Mesmo que o robô chegue ao staging torto, o ArUco corrige o Y absolutamente.

**Sem ArUco:** Qualquer erro de staging contamina o Y da abordagem. Em pior caso (±0.25m de tolerância), o robô pode errar o dock por uma margem significativa.

### 14.2 Ausência de Detecção a Longo Alcance

O LiDAR começa a ver claramente a face do dock quando o robô está a menos de ~3m. Antes disso, a face de 60cm de largura subtende apenas ~10cm no setor frontal do LiDAR (a 12m de distância), confundindo-se facilmente com prateleiras, paredes e outros objetos.

O `opennav_docking` tem `initial_perception_timeout: 8.0s` — se o estimador não publicar nada em 8s após o DockRobot iniciar, a action falha. Com ArUco, a câmera detecta o marcador a partir de ~3-4m e começa a publicar imediatamente. Com LiDAR-only, pode demorar até o robô entrar no setor frontal limpo.

### 14.3 Confusão com Obstáculos ao Fundo

No galpão de fábrica, existem prateleiras (shelf_big) de 18m de comprimento no caminho entre o spawn e o dock. O LiDAR frontal (±30°) pode detectar uma prateleira frontal como se fosse a face do dock, especialmente quando o robô está navegando em direção ao dock.

O `dock_pose_estimator.py` filtra por range máximo de 2.5m no setor frontal para mitigar isso, mas obstáculos entre 1-2.5m na frente do robô durante a navegação ainda podem gerar detecções falsas que perturbam o `filter_coef` EMA.

**Com ArUco:** ID=771 é único — sem confusão com outros objetos.

### 14.4 Zona Cega Sem Âncora ArUco

Quando o robô entra na zona cega do LiDAR (< 0.30m da face do dock, que começa quando a face está a < 0.494m do `base_link`), o estimador ativa o keepalive cache com o último valor publicado.

- **Com ArUco como âncora:** O último `last_dock_odom` é a posição real do dock. O cache perpetua essa referência absoluta.
- **Sem ArUco:** O cache perpetua o último LiDAR, que pode ter erro de Y acumulado. Se durante os 9cm finais de abordagem o robô deriva lateralmente, ele para no lugar errado.

### 14.5 Estimativa de Yaw Mais Ruidosa

O `dock_pose_estimator.py` usa `yaw = 0` fixo para a face do dock (dock é sempre perpendicular ao eixo X no mundo). Mas há uma função `_face_normal_yaw()` via SVD que calcula o yaw a partir dos pontos LiDAR da face:

```python
def _face_normal_yaw(self, face_pts):
    # SVD nos pontos da face para achar a normal
    _, _, vt = np.linalg.svd(pts - centroid, full_matrices=False)
    normal = vt[1]
```

Com LiDAR-only, se quiser usar essa estimativa para corrigir desalinhamento angular, o SVD precisa de ≥3 pontos da face frontal. Com o setor de ±30°, a face de 60cm vista de 12m gera apenas ~2 raios. Vista de 0.5m, gera ~18 raios — só funciona na aproximação final, não no início.

**Com ArUco:** O `rvec` do `solvePnP` dá a orientação completa do marcador com erro típico de ~1°.

### 14.6 filter_coef e Convergência Inicial

O `SimpleChargingDock` aplica um filtro EMA na pose publicada pelo estimador:
```
pose_filtrada = (1 - filter_coef) * pose_anterior + filter_coef * pose_nova
```

Com `filter_coef: 0.1` (valor no YAML), o filtro começa em `pose_anterior = pose_do_YAML = [13.0, 0.0, 0.0]`. Se a primeira leitura do LiDAR publicar `x=12.875`, o filtro demora ~14 iterações (a 50Hz do servidor = ~0.28s) para chegar perto de 12.875. Nesse intervalo, o robô persegue um target em ~12.96m.

- **Com ArUco a 3m+ de distância:** O estimador começa a publicar cedo (antes do DockRobot iniciar a abordagem final), dando tempo para o filtro convergir.
- **Sem ArUco:** O LiDAR só enxerga o dock quando o robô já está próximo. O filtro começa a convergir tarde, com o robô já se movendo.

**Sintoma:** Robô que "passa" pelo target e precisa dar ré — o target filtrado estava 8-10cm à frente do target real durante os primeiros instantes.

### 14.7 Diagnóstico Visual Limitado

Com ArUco, é possível verificar no RViz o marcador detectado e a pose publicada. Com LiDAR-only, a única visualização é a pose publicada em `/detected_dock_pose`, sem confirmação de qual cluster LiDAR foi usado. Fica difícil diagnosticar se um docking falhou por problema de detecção ou de controle.

### 14.8 Resumo: Quando Usar Cada Modo

| Cenário | ArUco + LiDAR | LiDAR-only |
|---------|---------------|------------|
| Docking preciso (gap < 10cm) | Recomendado | Arriscado |
| Ambiente com obstáculos próximos ao dock | OK (ArUco ignora) | Problemático |
| Iluminação variável (dia/noite) | ArUco sensível | LiDAR imune |
| Dock sem marcador físico | Impossível | Única opção |
| Hardware com câmera limitada (C270) | Funciona (curto alcance) | Funciona (sem câmera) |
| Calibração necessária | Câmera + marker_size | Apenas stop_distance |

**Conclusão:** O sistema com ArUco + LiDAR é mais robusto porque o ArUco fornece a referência absoluta de Y (eliminando o bias de staging) e detecta a posição a longo alcance (dando tempo ao filtro EMA). LiDAR-only é viável, mas requer staging muito preciso (< 5cm de erro Y) e `filter_coef ≥ 0.5`.

---

## 15. Branches Git

O repositório em `sim_ws/src/sim_bot/.git` tem 4 branches de desenvolvimento:

| Branch | Descrição |
|--------|-----------|
| `carregamento-automatico-fabrica` | Branch atual — galpão de fábrica, docking completo |
| `carregamento-sem-cam` | Docking apenas com LiDAR (sem câmera/ArUco) |
| `dock-carregamento` | Branch inicial — ambiente pequeno (test.world) |
| `humble-new-gazebo` | Compatibilidade com ROS2 Humble + Gazebo Harmonic |

---

## 16. Extensões Futuras Sugeridas

1. **Hardware real:** O `dock_pose_estimator.py` usa `cv2.aruco` e `LaserScan` padrão ROS2 — portável diretamente para hardware real. Ajustar `stop_distance` e `marker_size` para o dock físico.

2. **Múltiplas estações de dock:** Adicionar mais entradas em `docks:` no `docking_params.yaml` e lógica de seleção no `charging_manager.py`.

3. **Bateria real:** Assinar tópico de bateria real (ex: `sensor_msgs/BatteryState`) em vez do timer simulado.

4. **Interface de tarefas:** Substituir `ros2 topic pub` por uma API REST ou interface gráfica que publica nos tópicos `/task/*`.

5. **Mapeamento persistente:** Salvar o mapa do slam_toolbox com `slam_toolbox/save_map` e carregar na próxima execução para evitar remapeamento.

6. **Abordagem em 3 estágios (sessão 11+):** Para ambientes com obstáculos próximos ao dock, usar PRE_ALIGN → STAGING → DockRobot ao invés de navegar direto ao staging. Isso garante yaw correto antes da abordagem final.

---

## 17. Referências de Documentação

- [ROS2 Jazzy](https://docs.ros.org/en/jazzy/)
- [Nav2 Docs](https://navigation.ros.org/)
- [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [opennav_docking](https://github.com/open-navigation/opennav_docking)
- [Gazebo Harmonic](https://gazebosim.org/docs/harmonic/)
- [ros_gz (bridge)](https://github.com/gazebosim/ros_gz)

---

*Histórico completo de todas as 13 sessões com bugs e soluções detalhadas: `SESSAO_PROGRESSO.txt`*
Use esses 2 arquivos para passar contexto para qualquer IA
