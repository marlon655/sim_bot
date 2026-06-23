# Troubleshooting — palmares_bot Docking Autônomo

Referência rápida para os erros mais comuns. Organizado por categoria.

---

## Índice

1. [Gazebo não inicia / trava](#1-gazebo-não-inicia--trava)
2. [Erros de TF (transform)](#2-erros-de-tf-transform)
3. [Robô não navega / Nav2 falha](#3-robô-não-navega--nav2-falha)
4. [Docking falha ou fica em loop](#4-docking-falha-ou-fica-em-loop)
5. [ArUco não detectado](#5-aruco-não-detectado)
6. [LiDAR / dock_pose_estimator sem publicação](#6-lidar--dock_pose_estimator-sem-publicação)
7. [charging_manager travado num estado](#7-charging_manager-travado-num-estado)
8. [Robô colide com o dock](#8-robô-colide-com-o-dock)
9. [Robô para longe / torto no dock](#9-robô-para-longe--torto-no-dock)
10. [Erros de compilação / build](#10-erros-de-compilação--build)

---

## 1. Gazebo não inicia / trava

### Erro: `[gz sim] ... waiting for model to load` (trava indefinidamente)

**Causa:** Gazebo está baixando modelos do Fuel (internet) na primeira execução.
**Solução:** Aguardar. A primeira execução pode levar 2-5 minutos. Verificar progresso:
```bash
# Em outro terminal:
ls ~/.gz/fuel/fuel.gazebosim.org/
```

### Erro: `gz: command not found` ou `ros_gz_sim` não encontrado

**Causa:** Gazebo Harmonic não instalado ou variáveis de ambiente ausentes.
```bash
sudo apt install ros-jazzy-ros-gz
source /opt/ros/jazzy/setup.bash
```

### Gazebo abre mas o robô não aparece (sem erro visível)

**Causa:** O `spawn_entity` pode ter falhado silenciosamente.
```bash
# Verificar se o robô está no Gazebo:
gz topic -l | grep palmares_bot
# ou:
ros2 topic echo /odom --once
```
Se `/odom` não responder, o spawn falhou. Reiniciar com o comando completo de kill:
```bash
pkill -9 -f "gz sim|dock_pose_estimator|odom_to_tf|slam_toolbox|nav2|docking|twist_mux|robot_state_publisher|charging_manager" 2>/dev/null; sleep 5
cd ~/sim_ws && colcon build --packages-select sim_bot && source install/setup.bash
ros2 launch sim_bot palmares_bot.launch.py
```

### Erro: `[ERROR] [gz_bridge]: ... No match for topic`

**Causa:** Tópico Gazebo não existe ainda (plugin ainda subindo).
**Solução:** Ignorar — é transitório. Se persistir, verificar `gz_bridge.yaml`.

---

## 2. Erros de TF (transform)

### Erro: `NoDataForExtrapolationException` (slam_toolbox ou Nav2)

**Causa mais comum:** `odom_to_tf.py` não está rodando ou `/odom` não está sendo publicado.
```bash
ros2 topic hz /odom          # deve ser ~30 Hz
ros2 node list | grep odom   # deve mostrar /odom_to_tf
```
Se `/odom` estiver silencioso, o DiffDrive plugin do Gazebo não subiu. Reiniciar.

### Erro: `Lookup would require extrapolation into the future`

**Causa:** Timestamp de uma mensagem está ligeiramente à frente do TF disponível.
**Solução:** Já tratado pelo `odom_to_tf.py` (usa `get_clock().now()`). Se persistir, aumentar `transform_tolerance` em `nav_params.yaml`:
```yaml
transform_tolerance: 1.0   # aumentar de 0.5 para 1.0
```

### Erro: `map -> odom: could not find a connection`

**Causa:** `slam_toolbox` não subiu ainda ou está processando o primeiro scan.
**Solução:** Aguardar — o SLAM leva ~5-10s após o launch para publicar a primeira TF `map→odom`. Se persistir após 30s:
```bash
ros2 node list | grep slam
ros2 topic hz /map
```

### Erro: `camera_link_optical -> odom: could not find a connection`

**Causa:** `robot_state_publisher` não está publicando a TF da câmera.
```bash
ros2 topic echo /robot_description --once | grep camera
ros2 run tf2_tools view_frames && xdg-open /tmp/frames.pdf
```

---

## 3. Robô não navega / Nav2 falha

### Nav2 não responde (action `/navigate_to_pose` não existe)

**Causa:** Nav2 ainda não subiu (demoraa ~20s após launch).
```bash
ros2 action list | grep navigate
# Deve aparecer após ~20s:
#   /navigate_to_pose
#   /navigate_through_poses
```
Se não aparecer após 30s, verificar:
```bash
ros2 node list | grep bt_navigator
ros2 topic hz /scan   # scan deve estar chegando no Nav2
```

### Robô desvia de obstáculos que não existem / ignora obstáculos reais

**Causa:** Global costmap com tamanho/origem incorretos para o ambiente.
```bash
ros2 topic echo /global_costmap/costmap --once
```
Comparar com configuração em `config/nav_params.yaml`:
```yaml
width: 25
height: 25
origin_x: -5.0
origin_y: -12.0
```
Se o ambiente mudou, ajustar esses valores para cobrir toda a área de navegação.

### Nav2 planeja path mas robô fica girando no lugar

**Causa:** `xy_goal_tolerance` muito pequena ou plano inviável.
**Solução rápida:**
```bash
# Cancelar goal atual:
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{}" --cancel
```

### Robô "salta" ou rota incoerente

**Causa:** TF jump do SLAM (drift acumulado). O `bt_navigator.global_frame` está correto?
```bash
ros2 param get /bt_navigator global_frame
# Deve retornar: odom   (NÃO map)
```
Se retornar `map`, a nav foi lançada com parâmetro errado. Verificar `nav_params.yaml`.

---

## 4. Docking falha ou fica em loop

### Erro: `initial_perception_timeout` esgotado / "Dock not detected"

**Causa:** `/detected_dock_pose` não está sendo publicado.
```bash
ros2 topic hz /detected_dock_pose
```
Se não publicar, o `dock_pose_estimator` não está detectando nada:
- Câmera não vê o ArUco (iluminação, distância > 3m, marcador obstruído)
- LiDAR não detecta o dock no setor frontal (obstáculo na frente)
- `dock_pose_estimator` não está rodando: `ros2 node list | grep dock_pose`

### DockRobot status ≠ 4, retry infinito

**Causa 1:** `docking_threshold` muito pequeno (0.02m). O `v_linear_min` do GracefulController (0.06 m/s) não consegue parar a 2cm.
```yaml
# docking_params.yaml — já corrigido para:
docking_threshold: 0.05
```

**Causa 2:** `filter_coef` muito baixo. EMA demora a convergir para o target real.
```yaml
# docking_params.yaml — aumentar:
filter_coef: 0.5
```

**Causa 3:** `dock_prestaging_tolerance` ≤ `xy_goal_tolerance`. Robô para na borda.
```yaml
# Regra: dock_prestaging_tolerance = 2 × xy_goal_tolerance
dock_prestaging_tolerance: 0.5   # xy_goal_tolerance é 0.25
```

### Robô vai para staging mas GracefulController empurra para frente indefinidamente

**Causa:** `navigate_to_staging_pose=True` em retry quando robô já ultrapassou o staging (X > 12.0m). Nav2 manda robô de volta ao staging (X=12.0) estando em X=13.1m → GracefulController tenta ir para 12.0 mas dock está em 13.0 → caos.

**Diagnóstico:**
```bash
ros2 topic echo /odom --once  # verificar X atual do robô
```
Se X > 12.5m durante o retry, é esse bug. Já tratado pelo `DOCK_MAX_FAILS=4` + pausa de 15s.

### Dois DockRobot concorrentes (robô em zigue-zague ou loop)

**Causa:** `/go_charge` enviado durante `EXECUTING_TASK` sem state guard em `_finish_task`.
**Diagnóstico:**
```bash
ros2 topic echo /charging_manager/state
# Se alternar entre DOCKING e outros estados rapidamente, é race condition
```
Já corrigido nas sessões 9-10. Se o bug reaparecer após modificar `charging_manager.py`, verificar que os state guards estão em todos os callbacks async.

---

## 5. ArUco não detectado

### Nenhuma linha "ArUco:" no log do dock_pose_estimator

**Verificar câmera publicando:**
```bash
ros2 topic hz /camera/image      # deve ser ~30 Hz
ros2 topic echo /camera/camera_info --once  # K não deve ser zeros
```

**Verificar calibração:**
```bash
# Se K[0] (fx) = 0, a câmera não foi calibrada:
ros2 topic echo /camera/camera_info --once | grep -A 3 "k:"
```
Sem calibração, `solvePnP` retorna erro e a detecção não é publicada.

**Verificar iluminação e distância:**
- A câmera precisa ver o marcador claramente: sem reflexo, sem sombra forte
- Distância ideal: 0.5m a 3m
- No Gazebo: o `factory.world` tem um fill light focado no ArUco. Se a detecção falha na simulação, verificar se a luz está no arquivo `worlds/factory.world`

**Verificar marker_size:**
```bash
ros2 param get /dock_pose_estimator marker_size
# Deve corresponder ao tamanho físico do marcador impresso (incluindo borda)
```

### ArUco detectado mas posição errada (X muito diferente do esperado)

**Causa:** `marker_size` incorreto. O `solvePnP` usa o tamanho físico para calcular distância.
- Se `marker_size` for menor que o real → estima que está mais perto → X menor
- Se `marker_size` for maior que o real → estima que está mais longe → X maior

Medir o lado externo do marcador com régua e atualizar em `docking.launch.py`.

---

## 6. LiDAR / dock_pose_estimator sem publicação

### `/scan` não publicado

```bash
ros2 topic hz /scan
```
Se silencioso:
```bash
ros2 topic list | grep scan          # tópico existe?
ros2 node list | grep parameter_bridge  # bridge está rodando?
```
Verificar `config/gz_bridge.yaml` — o entry do `/scan` deve estar presente.

### dock_pose_estimator publica mas X está errado (muito longe ou muito perto)

**Causa:** Algoritmo de face_x usando pontos oblíquos ao invés do centro.
O `dock_pose_estimator.py` já tem correção (center_band ±20°). Verificar o log:
```bash
ros2 topic echo /detected_dock_pose | grep x
# Esperado: x ≈ 12.935m  (face_dock=13.235m - stop_distance=0.30m)
# Se stop_distance diferente: X_target = 13.235 - stop_distance
```

**Causa alternativa:** `filter_coef: 0.1` ainda convergindo (nos primeiros 3s após DockRobot iniciar).

### dock_pose_estimator para de publicar durante a abordagem final

**Causa:** Zona cega do LiDAR (min_range=0.30m). A zona cega (~0.9s) é coberta pelo `external_detection_timeout: 2.0s` no `docking_params.yaml` — o `docking_server` usa a última pose por até 2s.
Se a paragem durar mais de 2s, o docking server declara falha. Verificar se o `/scan` continua chegando durante a abordagem: `ros2 topic hz /scan`.

---

## 7. charging_manager travado num estado

### Travado em `STARTUP` (nunca vai para `RETURNING_HOME`)

**Causa:** `AUTOSTART_DELAY=30s` ainda não expirou, ou Nav2/docking server não subiram a tempo.
Aguardar até **t=55s** após o launch (docking server sobe em t=25s + AUTOSTART_DELAY=30s = t=55s). Se ainda não mudar:
```bash
ros2 node list | grep -E "charging_manager|docking_server|bt_navigator"
```
Todos os três devem aparecer.

### Travado em `RETURNING_HOME` (nunca chega a `DOCKING`)

**Causa:** `DockRobot` action não foi aceita (servidor não disponível).
```bash
ros2 action list | grep dock_robot
ros2 node list | grep docking_server
```

### Travado em `DOCKING` (nunca chega a `HOME_CHARGING`)

Ver seção 4 (Docking falha ou fica em loop).

### Tarefa aceita mas robô não sai do dock

**Causa:** Undocking simples (cmd_vel -0.15 m/s × 2s) falhou — talvez `twist_mux` bloqueando ou `/cmd_vel` não chegando no Gazebo.
```bash
ros2 topic echo /cmd_vel  # publicando durante UNDOCKING?
ros2 topic hz /cmd_vel
```
Verificar prioridade do `twist_mux`:
```bash
cat ~/sim_ws/src/sim_bot/config/twist_mux_params.yaml
```

### `/go_charge` não responde (estado não muda)

**Causas possíveis:**
```bash
ros2 topic echo /charging_manager/state --once
```
- Se estado for `HOME_CHARGING`, `DOCKING` ou `RETURNING_HOME`: comportamento correto — `/go_charge` é ignorado nesses estados
- Se estado for `EXECUTING_TASK` e não mudar: verificar state guards no código

---

## 8. Robô colide com o dock

### Colisão na primeira tentativa

**Causa principal:** `stop_distance` muito pequeno ou `filter_coef` muito baixo.

`stop_distance` correto = `robot_front_offset` (0.252m) + gap desejado (0.05m) = **0.302m ≈ 0.30m**

Se mesmo assim colide:
1. Verificar `filter_coef` — se for 0.1, o EMA demora a convergir e o target fica ~8cm à frente do real
2. Aumentar `stop_distance` gradualmente: `0.32`, `0.35`, `0.38`
3. Usar ajuste em runtime:
```bash
ros2 param set /dock_pose_estimator stop_distance 0.35
```

### Colisão apenas em retentativas (segunda tentativa em diante)

**Causa:** Na segunda tentativa, o robô já está a 0.5m do dock. O `navigate_to_staging_pose=True` manda para staging (12.0m), mas na abordagem seguinte o filtro EMA parte de um valor mais próximo do real (não 13.0m). A abordagem fica mais rápida e o robô não desacelera a tempo.
**Solução:** Já tratado pela pausa de 3s entre tentativas (`DOCK_MAX_FAILS` retries).

---

## 9. Robô para longe / torto no dock

### Para alinhado mas ~30cm longe demais

**Causa:** `stop_distance` muito grande ou `docking_threshold` muito pequeno.
```bash
ros2 param get /dock_pose_estimator stop_distance
```
Diminuir `stop_distance` em 2-3cm por vez e testar.

### Para com desvio lateral (não centrado no dock)

**Causa:** Y do target incorreto. Ver seção 14 do `PASSAGEM_CONHECIMENTO.md` para entender por que o Y pode estar errado.

**Diagnóstico:**
```bash
ros2 topic echo /detected_dock_pose | grep "y:"
# O y deve ser ~0.0 (dock está em odom Y=0.0)
# Se y ≠ 0, há bias lateral
```

**Solução:**
- Se ArUco foi detectado: verificar `external_detection_translation_y` em `docking_params.yaml` (está em -0.04 — ajustar para 0.0 como ponto de partida)
- Se LiDAR-only: bias vem do staging. Aumentar precisão de navegação ou usar pré-alinhamento.

### Para torto (yaw residual ≠ 0)

**Causa:** `k_delta` alto com ΔY ≠ 0. O GracefulController faz curva para corrigir Y → chega com yaw oblíquo.
```yaml
# docking_params.yaml — reduzir k_delta:
k_delta: 0.3     # era 2.0 — abordagem mais reta, menos correção lateral
```
**Mas:** reduzir `k_delta` só ajuda se o Y do target estiver correto. Corrigir o bias de Y primeiro.

---

## 10. Erros de compilação / build

### `ModuleNotFoundError: No module named 'cv2'`

```bash
sudo apt install python3-opencv
# ou:
pip3 install opencv-python
```

### `No rule to make target` ou arquivo não instalado

```bash
cd ~/sim_ws
colcon build --packages-select sim_bot --cmake-clean-cache
source install/setup.bash
```

### `SetupError: package directory ... does not exist`

Verificar que o workspace foi clonado corretamente:
```bash
ls ~/sim_ws/src/sim_bot/
# Deve mostrar: CMakeLists.txt package.xml nodes/ launch/ config/ ...
```

### Executável Python não encontrado pelo `ros2 run`

```bash
# Verificar que está instalado em lib/:
ls ~/sim_ws/install/sim_bot/lib/sim_bot/
# Deve listar: charging_manager.py dock_pose_estimator.py odom_to_tf.py alignment_test.py

# Se não estiver, recompilar:
colcon build --packages-select sim_bot
source install/setup.bash
```

### `AttributeError: module 'cv2.aruco' has no attribute 'Dictionary_get'`

**Causa:** OpenCV 4.8+ removeu a API legada do ArUco. O projeto usa `Dictionary_get` e `DetectorParameters_create`.

**Verificar versão:**
```bash
python3 -c "import cv2; print(cv2.__version__)"
```

Se for 4.8+, adaptar `dock_pose_estimator.py`:
```python
# API nova (OpenCV 4.7+)
self._aruco_dict   = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
self._aruco_params = cv2.aruco.DetectorParameters()
self._detector     = cv2.aruco.ArucoDetector(self._aruco_dict, self._aruco_params)
# ...
# Na detecção:
corners, ids, _ = self._detector.detectMarkers(gray)
```

---

## 11. Câmera Hardware Real (C270)

### `/dev/video*` não aparece após conectar a câmera

```bash
dmesg | tail -20 | grep -i "uvc\|video\|camera"
lsusb | grep -i logitech
```

Se `lsusb` não mostrar a câmera, problema é de hardware (cabo, porta USB). Se aparecer no `lsusb` mas não no `dmesg`, carregar o módulo:
```bash
sudo modprobe uvcvideo
```

### `[usb_cam]: Failed to open /dev/camera_c270`

**Causa 1:** Usuário não está no grupo `video`.
```bash
groups $USER | grep video   # se não aparecer video:
sudo usermod -aG video $USER
# Fazer logout/login para aplicar
```

**Causa 2:** A regra udev ainda não foi recarregada.
```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
ls -la /dev/camera_c270    # deve existir
```

### `[usb_cam]: Failed to set pixel format`

**Causa:** `pixel_format: "mjpeg2rgb"` não suportado por esta versão do driver.
```bash
# Verificar formatos disponíveis:
v4l2-ctl --device=/dev/camera_c270 --list-formats-ext
```
Tentar em ordem no `config/usb_cam_params.yaml`:
1. `pixel_format: "mjpeg2rgb"`
2. `pixel_format: "yuyv2rgb"`
3. `pixel_format: "mjpeg"`

### `/camera/camera_info` com K[0] = 0.0 (câmera não calibrada)

**Sintoma:** `dock_pose_estimator` recebe CameraInfo mas não detecta ArUco (silenciosamente — o código faz `if self.camera_matrix is None: return`).

**Diagnóstico:**
```bash
ros2 topic echo /camera/camera_info --once | grep -A 1 "k:"
# Se data: [0.0, 0.0, ...] → câmera não calibrada ou URL errada
```

**Verificar o caminho do arquivo de calibração:**
```bash
ls -la ~/.ros/camera_info/c270.yaml   # arquivo existe?
```

Se não existir, refazer a calibração seguindo `TUTORIAL_CAMERA_REAL.md` Seção 5.

Se existir mas K[0] ainda é 0.0, verificar que `camera_name` no YAML de calibração bate com o `camera_name` no `usb_cam_params.yaml` (ambos devem ser `"camera"`):
```bash
head -3 ~/.ros/camera_info/c270.yaml  # deve mostrar: camera_name: camera
```

### ArUco não detectado no hardware mas funciona na simulação

Verificar em ordem:
1. **Grupo video:** `groups $USER | grep video`
2. **K[0] ≠ 0:** `ros2 topic echo /camera/camera_info --once | grep -A 1 "k:"`
3. **Imagem chegando:** `ros2 topic hz /camera/image` → deve ser ~30 Hz
4. **Imagem não invertida:** `ros2 run rqt_image_view rqt_image_view`
5. **Iluminação:** marcador bem iluminado sem reflexo?
6. **marker_size correto:** medir o lado externo do ArUco impresso com régua
7. **TF completo:** `ros2 run tf2_ros tf2_echo odom camera_link_optical`
8. **Teste de distância:** segurar a 50cm, verificar `x ≈ 0.50` no `/detected_dock_pose`

---

## Comandos de Diagnóstico Rápido

```bash
# Estado geral (tópicos ativos):
ros2 topic list

# Frequência dos sensores principais:
ros2 topic hz /scan /odom /camera/image /detected_dock_pose

# Estado da máquina de estados + bateria:
watch -n 2 "ros2 topic echo /charging_manager/state --once 2>/dev/null; \
            ros2 topic echo /battery_level --once 2>/dev/null"

# Árvore TF:
ros2 run tf2_tools view_frames && xdg-open /tmp/frames.pdf

# Nós ativos:
ros2 node list

# Log de um nó específico (últimas 50 linhas):
ros2 node info /charging_manager
```
