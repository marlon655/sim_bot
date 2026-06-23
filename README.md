# palmares_bot — Docking Autônomo em Simulação

Robô diferencial autônomo que realiza ciclo completo de carregamento em uma estação de docking
dentro de um galpão de fábrica simulado.

**Stack:** ROS2 Jazzy + Gazebo Harmonic + Nav2 + slam_toolbox + opennav_docking

---

## Demonstração do Ciclo

```
Robô nasce em odom(0,0) — centro do galpão
       ↓  (~55s: Nav2 t=20s, docking t=25s, AUTOSTART_DELAY=30s)
Navega 12m até a base de carregamento em odom(13,0)
       ↓
Docking preciso via LiDAR + ArUco → HOME_CHARGING
       ↓  (bateria ≥ 40%)
Aceita tarefas: goto_pose / inspect / patrol / deliver
       ↓
Executa tarefa → retorna ao dock → carrega → próxima tarefa
```

---

## Documentação

| Arquivo | Descrição |
|---------|-----------|
| [INSTALACAO.md](INSTALACAO.md) | Instalação de todas as dependências do zero |
| [PASSAGEM_CONHECIMENTO.md](PASSAGEM_CONHECIMENTO.md) | Arquitetura completa, parâmetros, decisões de design |
| [TUTORIAL_CAMERA_REAL.md](TUTORIAL_CAMERA_REAL.md) | Configurar câmera C270 física no robô real |
| [TROUBLESHOOTING.md](TROUBLESHOOTING.md) | Erros comuns e soluções rápidas |
| [SESSAO_PROGRESSO.txt](SESSAO_PROGRESSO.txt) | Histórico das 13 sessões de desenvolvimento |

---

## Início Rápido

```bash
# 1. Clonar (branch com docking completo no galpão de fábrica)
mkdir -p ~/sim_ws/src && cd ~/sim_ws/src
git clone -b carregamento-automatico-fabrica https://github.com/marlon655/sim_bot.git

# 2. Instalar dependências
cd ~/sim_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y

# 3. Compilar
colcon build --packages-select sim_bot && source install/setup.bash

# 4. Lançar
ros2 launch sim_bot palmares_bot.launch.py
```

Ver [INSTALACAO.md](INSTALACAO.md) para requisitos completos.

---

## Enviando Tarefas

Após o robô atingir `HOME_CHARGING` com bateria ≥ 40% (~100s após o launch):

```bash
source ~/sim_ws/install/setup.bash

# Patrulha rota A
ros2 topic pub --once /task/patrol std_msgs/msg/String '{data: "A"}'

# Ir a um ponto
ros2 topic pub --once /task/goto_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: "odom"}, pose: {position: {x: 2.0, y: 0.0}, orientation: {w: 1.0}}}'

# Forçar retorno imediato ao dock
ros2 topic pub --once /go_charge std_msgs/msg/Empty '{}'
```

---

## Branches

| Branch | Descrição |
|--------|-----------|
| `carregamento-automatico-fabrica` | **Atual** — galpão factory.world, docking completo |
| `carregamento-sem-cam` | Docking somente com LiDAR (sem câmera/ArUco) |
| `dock-carregamento` | Versão inicial — ambiente pequeno (test.world) |
| `humble-new-gazebo` | Compatibilidade com ROS2 Humble |

---

## Requisitos

- Ubuntu 24.04
- ROS2 Jazzy
- Gazebo Harmonic (`ros-jazzy-ros-gz`)
- `ros-jazzy-opennav-docking`
- `ros-jazzy-slam-toolbox`
- `ros-jazzy-nav2-bringup`
- `python3-opencv` (OpenCV 4.6.x com módulo aruco)

Ver [INSTALACAO.md](INSTALACAO.md) para a lista completa e comandos de instalação.
