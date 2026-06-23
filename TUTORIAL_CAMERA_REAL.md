# Tutorial: Câmera Física (Logitech C270) no Robô Real

**Contexto:** Este tutorial assume que o sistema de docking já funciona em simulação.
O objetivo é fazer a Logitech C270 publicar `/camera/image` e `/camera/camera_info`
no mesmo formato que o Gazebo usa, para que o `dock_pose_estimator.py` funcione sem alterações.

**Pré-requisito de hardware:** C270 conectada via USB no computador embarcado do robô.

---

## Índice

1. [Verificar a câmera no Linux](#1-verificar-a-câmera-no-linux)
2. [Instalar o driver usb_cam](#2-instalar-o-driver-usb_cam)
3. [Criar regra udev (endereço USB fixo)](#3-criar-regra-udev-endereço-usb-fixo)
4. [Criar o arquivo de configuração do driver](#4-criar-o-arquivo-de-configuração-do-driver)
5. [Calibrar a câmera](#5-calibrar-a-câmera)
6. [Validar a calibração e o marker_size](#6-validar-a-calibração-e-o-marker_size)
7. [Criar o launch file da câmera](#7-criar-o-launch-file-da-câmera)
8. [Atualizar o URDF para os parâmetros reais da C270](#8-atualizar-o-urdf-para-os-parâmetros-reais-da-c270)
9. [Montagem física do marcador ArUco no dock](#9-montagem-física-do-marcador-aruco-no-dock)
10. [Configurar TF estático para testes isolados](#10-configurar-tf-estático-para-testes-isolados)
11. [Ajustar controles da câmera (brilho/contraste)](#11-ajustar-controles-da-câmera-brilhocontraste)
12. [Atualizar docking.launch.py para hardware real](#12-atualizar-dockinglaunchy-para-hardware-real)
13. [Criar o launch file do robô real](#13-criar-o-launch-file-do-robô-real)
14. [Testar a detecção ArUco no sistema completo](#14-testar-a-detecção-aruco-no-sistema-completo)
15. [Checklist final de integração](#15-checklist-final-de-integração)
16. [Referências rápidas](#16-referências-rápidas)

---

## 1. Verificar a Câmera no Linux

Conecte a C270 via USB e verifique se o Linux a reconhece:

```bash
# Listar dispositivos de vídeo
ls -la /dev/video*
# Deve aparecer: /dev/video0 (ou video1, video2 se houver outras câmeras)

# Verificar informações detalhadas da C270
v4l2-ctl --device=/dev/video0 --info
# Deve aparecer "Logitech Webcam C270" no campo "Card type"

# Listar formatos e resoluções suportados
v4l2-ctl --device=/dev/video0 --list-formats-ext
```

**Saída esperada do último comando (trecho):**
```
[0]: 'YUYV' (YUYV 4:2:2)
    Size: Discrete 640x480
        Interval: Discrete 0.033s (30.000 fps)
    Size: Discrete 1280x720
        Interval: Discrete 0.100s (10.000 fps)   ← 720p só a 10fps!
[1]: 'MJPG' (Motion-JPEG, compressed)
    Size: Discrete 640x480
        Interval: Discrete 0.033s (30.000 fps)
    Size: Discrete 1280x720
        Interval: Discrete 0.033s (30.000 fps)
```

> **Atenção:** Use sempre **640×480 MJPG a 30fps**. O modo 1280×720 só chega a 10fps no YUYV, o que é inadequado para detecção ArUco em tempo real.

### 1.1 Instalar ferramentas v4l2

```bash
sudo apt install v4l-utils
```

### 1.2 Adicionar o usuário ao grupo `video`

Sem isso, o driver ROS2 não consegue abrir `/dev/video*` sem ser root:

```bash
sudo usermod -aG video $USER

# IMPORTANTE: fazer logout e login novamente para o grupo ter efeito
# Verificar que o grupo foi adicionado:
groups $USER
# Deve aparecer "video" na lista
```

Se não quiser fazer logout agora, testar com:
```bash
newgrp video   # abre um sub-shell com o grupo ativo na sessão atual
```

### 1.3 Diagnóstico se `/dev/video*` não aparecer

```bash
# Verificar se o kernel reconheceu a câmera
dmesg | grep -i "uvc\|video\|camera" | tail -20

# Carregar o módulo UVC manualmente (normalmente já é automático)
sudo modprobe uvcvideo

# Instalar módulos extra se necessário
sudo apt install linux-modules-extra-$(uname -r)
```

---

## 2. Instalar o Driver usb_cam

O `usb_cam` é o driver ROS2 padrão para câmeras USB UVC. Publica `/camera/image` e `/camera/camera_info` — exatamente os tópicos que o `dock_pose_estimator.py` assina.

```bash
sudo apt install ros-jazzy-usb-cam ros-jazzy-camera-calibration
```

**Verificar instalação:**
```bash
source /opt/ros/jazzy/setup.bash
ros2 pkg list | grep -E "usb_cam|camera_calibration"
# Deve aparecer ambos os pacotes
```

---

## 3. Criar Regra udev (Endereço USB Fixo)

Sem regra udev, `/dev/video0` pode virar `/dev/video1` após reiniciar ou reconectar a câmera. A regra cria um link simbólico estável `/dev/camera_c270`.

### 3.1 Identificar os atributos USB da câmera

```bash
udevadm info --name=/dev/video0 --attribute-walk | grep -E "idVendor|idProduct|serial"
```

Para a C270, os valores esperados são:
- `idVendor == "046d"` (Logitech)
- `idProduct == "0825"` (C270)

> Se tiver **duas C270 iguais**, elas terão o mesmo idVendor/idProduct. Nesse caso use o atributo `serial` para diferenciar (aparece na saída do `udevadm` como `ATTRS{serial}`).

### 3.2 Criar o arquivo de regra

```bash
sudo nano /etc/udev/rules.d/99-camera-c270.rules
```

Conteúdo (câmera única):
```
# Logitech C270 — link simbólico estável em /dev/camera_c270
SUBSYSTEM=="video4linux", ATTRS{idVendor}=="046d", ATTRS{idProduct}=="0825", \
    ATTR{index}=="0", SYMLINK+="camera_c270", MODE="0666", GROUP="video"
```

Conteúdo (duas C270 com seriais diferentes):
```
# C270 câmera frontal (serial: ABC123)
SUBSYSTEM=="video4linux", ATTRS{idVendor}=="046d", ATTRS{idProduct}=="0825", \
    ATTRS{serial}=="ABC123", ATTR{index}=="0", SYMLINK+="camera_c270_front", MODE="0666"

# C270 câmera traseira (serial: DEF456)
SUBSYSTEM=="video4linux", ATTRS{idVendor}=="046d", ATTRS{idProduct}=="0825", \
    ATTRS{serial}=="DEF456", ATTR{index}=="0", SYMLINK+="camera_c270_rear", MODE="0666"
```

### 3.3 Recarregar as regras

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger

# Reconectar a câmera e verificar:
ls -la /dev/camera_c270
# Deve apontar para /dev/video0 (ou video1)
```

A partir de agora use `/dev/camera_c270` nos arquivos de configuração.

---

## 4. Criar o Arquivo de Configuração do Driver

Crie o arquivo `config/usb_cam_params.yaml` no pacote `sim_bot`:

```bash
nano ~/sim_ws/src/sim_bot/config/usb_cam_params.yaml
```

**Conteúdo:**
```yaml
# Parâmetros do driver usb_cam para a Logitech C270
# Tópicos publicados após remaps no launch file:
#   /camera/image       — sensor_msgs/Image (RGB8)
#   /camera/camera_info — sensor_msgs/CameraInfo

/**:
  ros__parameters:
    video_device: "/dev/camera_c270"
    image_width: 640
    image_height: 480
    pixel_format: "mjpeg2rgb"     # MJPG → RGB8; menos CPU que yuyv2rgb
    camera_frame_id: "camera_link_optical"
    io_method: "mmap"
    framerate: 30.0

    # Auto-controles
    auto_white_balance: true
    auto_exposure: true
    autofocus: false              # C270 tem foco fixo — sem autofocus

    # Arquivo de calibração gerado na Seção 5
    # Usar $HOME para não hardcodar o usuário — funciona em qualquer máquina
    camera_info_url: "file:///home/${USER}/.ros/camera_info/c270.yaml"

    # Deve corresponder ao gz_frame_id do sensor Gazebo: "camera_link_optical"
    camera_name: "camera"
```

> **Sobre `camera_info_url`:** O `${USER}` na URL não é expandido automaticamente pelo driver — ele é uma string literal. Para evitar hardcodar o caminho, prefira criar um symlink ou usar o caminho absoluto com seu usuário. Uma alternativa segura é usar o parâmetro no launch file (veja Seção 7).

> **Sobre `pixel_format`:** Se `mjpeg2rgb` gerar erro `"Failed to set pixel format"`, tente alternativas em ordem:
> 1. `"mjpeg2rgb"` — MJPG decodificado para RGB8 (recomendado)
> 2. `"yuyv2rgb"` — YUYV convertido para RGB8 (mais CPU)
> 3. `"mjpeg"` — MJPG raw (requer conversão extra no pipeline)
>
> Verificar formatos disponíveis em tempo de execução:
> ```bash
> ros2 param describe /usb_cam pixel_format
> ```

---

## 5. Calibrar a Câmera

A calibração gera a matriz intrínseca `K` e os coeficientes de distorção `D` que o `dock_pose_estimator.py` passa para o `cv2.solvePnP`. **Sem calibração, o solvePnP retorna distâncias e posições erradas** — o robô para no lugar errado.

> **Por que a calibração é obrigatória:** O `dock_pose_estimator._image_cb` começa com `if self.camera_matrix is None: return`. Se `/camera/camera_info` chegar com `K = [0,0,0,...]` (sem calibração), `camera_matrix` nunca é setada e **toda a detecção ArUco é silenciosamente descartada** sem mensagem de erro.

### 5.1 Verificar se a câmera está publicando CameraInfo válida

```bash
ros2 topic echo /camera/camera_info --once | grep -A 3 "k:"
# Se k[0] = 0.0, a câmera NÃO está calibrada — prosseguir com a calibração
# Se k[0] ≠ 0.0, a câmera já foi calibrada anteriormente
```

### 5.2 Preparar o Tabuleiro de Calibração

- **Padrão:** tabuleiro de xadrez com **8×6 quadrados internos** (9×7 interseções visíveis)
- **Tamanho do quadrado:** 25mm ou 30mm — **medir com paquímetro após imprimir** (impressoras escalam ligeiramente)
- **Suporte rígido:** colar em papelão rígido ou MDF — tabuleiro que curva invalida a calibração
- **Impressão:** preto puro sobre branco puro, sem escala de cinza nos quadrados

### 5.3 Lançar o Driver da Câmera

```bash
# Terminal 1
source /opt/ros/jazzy/setup.bash
source ~/sim_ws/install/setup.bash
ros2 run usb_cam usb_cam_node_exe \
  --ros-args \
  --params-file ~/sim_ws/src/sim_bot/config/usb_cam_params.yaml \
  --remap /image_raw:=/camera/image \
  --remap /camera_info:=/camera/camera_info
```

### 5.4 Executar o Calibrador

```bash
# Terminal 2 — ajustar --square para o tamanho real do quadrado em METROS
source /opt/ros/jazzy/setup.bash
ros2 run camera_calibration cameracalibrator \
  --size 8x6 \
  --square 0.025 \
  --ros-args \
  --remap image:=/camera/image \
  --remap camera:=/camera
```

Uma janela abrirá mostrando a câmera com as interseções detectadas em círculos coloridos.

### 5.5 Coletar Amostras de Qualidade

Mover o tabuleiro lentamente em frente à câmera seguindo este roteiro:

| Posição | O que cobre |
|---------|-------------|
| Centro da imagem, face plana | cobertura central |
| Canto superior esquerdo | cobertura X e Y |
| Canto superior direito | cobertura X e Y |
| Canto inferior esquerdo | cobertura X e Y |
| Canto inferior direito | cobertura X e Y |
| Próximo (~25cm) | cobertura Size |
| Médio (~60cm) | cobertura Size |
| Longe (~1.5m) | cobertura Size |
| Inclinado ~30° horizontal | cobertura Skew |
| Inclinado ~30° vertical | cobertura Skew |

As barras de progresso devem encher gradualmente:
- **X** — cobertura horizontal (mover tabuleiro para esquerda/direita)
- **Y** — cobertura vertical (mover tabuleiro para cima/baixo)
- **Size** — variedade de distâncias (aproximar e afastar)
- **Skew** — variedade de inclinações (inclinar o tabuleiro)

Coletar **mínimo de 40 amostras** para boa calibração.

### 5.6 Finalizar e Salvar

Quando todas as barras estiverem cheias (ficam verdes), clicar em:
1. **CALIBRATE** — processa as amostras. Pode demorar 10-30s.
2. **Verificar o erro de reprojeção** — aparece no terminal como `RMS re-projection error`. Um valor **< 0.5 pixels** é excelente. Entre 0.5 e 1.0 é aceitável. Acima de 1.0 significa qualidade ruim — refazer com mais amostras ou tabuleiro mais rígido.
3. **SAVE** — salva em `/tmp/calibrationdata.tar.gz`
4. **COMMIT** — copia para `~/.ros/camera_info/camera.yaml`

```bash
# Renomear para identificar esta câmera específica
mkdir -p ~/.ros/camera_info
cp ~/.ros/camera_info/camera.yaml ~/.ros/camera_info/c270.yaml

# Verificar o conteúdo:
cat ~/.ros/camera_info/c270.yaml
```

**Exemplo de saída esperada (valores ilustrativos — os seus serão diferentes):**
```yaml
image_width: 640
image_height: 480
camera_name: camera
camera_matrix:
  rows: 3
  cols: 3
  data: [620.5, 0.0, 318.2,
          0.0, 621.3, 242.7,
          0.0,   0.0,   1.0]     # fx=620.5, fy=621.3, cx=318.2, cy=242.7
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [0.052, -0.121, 0.001, -0.002, 0.0]   # k1, k2, p1, p2, k3
rectification_matrix:
  rows: 3
  cols: 3
  data: [1, 0, 0, 0, 1, 0, 0, 0, 1]
projection_matrix:
  rows: 3
  cols: 4
  data: [625.0, 0.0, 317.8, 0.0,
          0.0, 625.0, 242.3, 0.0,
          0.0,   0.0,   1.0, 0.0]
```

### 5.7 Calcular o FOV Real para o URDF

Com `fx` da calibração, calcule o FOV horizontal real:

```python
import math
fx = 620.5    # substituir pelo valor real da sua calibração (data[0] da camera_matrix)
width = 640
hfov_rad = 2.0 * math.atan(width / (2.0 * fx))
hfov_deg = math.degrees(hfov_rad)
print(f"horizontal_fov = {hfov_rad:.4f} rad  ({hfov_deg:.1f}°)")
# Exemplo: horizontal_fov = 0.9197 rad  (52.7°)
```

Anote esse valor — será usado na Seção 8 para atualizar o URDF.

---

## 6. Validar a Calibração e o marker_size

Antes de integrar com o docking, faça estes dois testes para garantir que a calibração e o tamanho do marcador estão corretos.

### 6.1 Teste de Distância Conhecida (valida marker_size e calibração juntos)

Este é o teste mais importante. Um erro de `marker_size` ou de calibração causa erro proporcional na distância estimada.

**Procedimento:**

1. Colocar o marcador ArUco impresso em uma superfície vertical
2. Posicionar a câmera a exatamente **50cm** da face do marcador (usar régua)
3. Lançar câmera + estimador (sem docking server):

```bash
# Terminal 1
source ~/sim_ws/install/setup.bash
ros2 launch sim_bot camera_real.launch.py

# Terminal 2 — lançar estimador com stop_distance=0 para ver a posição bruta do marcador
source ~/sim_ws/install/setup.bash
ros2 run sim_bot dock_pose_estimator.py \
  --ros-args \
  -p use_sim_time:=false \
  -p marker_id:=771 \
  -p marker_size:=0.15 \
  -p stop_distance:=0.0     # ← zero para ver posição bruta do marcador

# Terminal 3
ros2 topic echo /detected_dock_pose | grep "x:"
```

4. Verificar o valor publicado de `x`:
   - **Esperado:** `x ≈ 0.50` (a câmera está a 50cm do marcador, stop_distance=0)
   - **Tolerância aceitável:** ± 2cm (ou seja, entre 0.48 e 0.52)

**Interpretação dos erros:**
- `x < 0.48` → `marker_size` está **maior** que o real. Medir o marcador novamente e diminuir o valor.
- `x > 0.52` → `marker_size` está **menor** que o real. Aumentar o valor.
- `x` correto mas oscila muito → calibração com erro alto (refazer com RMS < 0.5px)
- `x` correto mas com offset constante em `y` → câmera não está centralizada no URDF (ajustar joint)

### 6.2 Verificar que a CameraInfo Está Sendo Carregada

```bash
# Deve mostrar fx ≠ 0.0 (o valor calibrado)
ros2 topic echo /camera/camera_info --once | grep -A 1 "k:"
# Saída ruim (sem calibração):  data: [0.0, 0.0, 0.0, ...]
# Saída boa (com calibração):   data: [620.5, 0.0, 318.2, ...]
```

Se `k[0] = 0.0` mesmo após configurar `camera_info_url`:
- Verificar o caminho do arquivo (deve ser absoluto, sem `$USER`):
  ```bash
  ls -la ~/.ros/camera_info/c270.yaml    # arquivo existe?
  cat ~/.ros/camera_info/c270.yaml | head -5   # conteúdo válido?
  ```
- Verificar que `camera_name` no YAML de calibração bate com `camera_name` no `usb_cam_params.yaml` (ambos devem ser `"camera"`)

### 6.3 Verificar que a Imagem Não Está Invertida ou Espelhada

```bash
ros2 run rqt_image_view rqt_image_view
```

Selecionar `/camera/image`. A imagem deve mostrar o ambiente sem inversão. Se estiver:
- **Espelhada horizontalmente:** a montagem da câmera no robô está ao contrário. Ajustar o joint no URDF (`rpy` do `camera_joint`) ou adicionar `flip_horizontal: true` no driver.
- **De cabeça para baixo:** a câmera foi montada invertida. Ajustar o joint ou adicionar `flip_vertical: true`.

---

## 7. Criar o Launch File da Câmera

Crie o arquivo `launch/camera_real.launch.py`:

```bash
nano ~/sim_ws/src/sim_bot/launch/camera_real.launch.py
```

**Conteúdo:**
```python
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg = get_package_share_directory('sim_bot')
    cam_params = os.path.join(pkg, 'config', 'usb_cam_params.yaml')

    # Caminho absoluto para o arquivo de calibração — resolve problema do $USER hardcoded
    home = os.path.expanduser('~')
    calib_url = f'file://{home}/.ros/camera_info/c270.yaml'

    camera_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        output='screen',
        parameters=[
            cam_params,
            {'camera_info_url': calib_url},    # sobrescreve o caminho hardcoded do YAML
        ],
        remappings=[
            # Mapeia tópicos do driver para os nomes que dock_pose_estimator espera
            ('/image_raw',   '/camera/image'),
            ('/camera_info', '/camera/camera_info'),
        ]
    )

    return LaunchDescription([camera_node])
```

**Recompilar para registrar o novo arquivo:**
```bash
cd ~/sim_ws && colcon build --packages-select sim_bot && source install/setup.bash
```

**Testar:**
```bash
ros2 launch sim_bot camera_real.launch.py

# Em outro terminal:
ros2 topic list | grep camera          # /camera/image e /camera/camera_info
ros2 topic hz /camera/image            # deve ser ~30 Hz
ros2 run rqt_image_view rqt_image_view # ver imagem ao vivo
```

---

## 8. Atualizar o URDF para os Parâmetros Reais da C270

**Arquivo:** `description/camera.xacro`

Alterar as linhas marcadas — usar o FOV calculado na Seção 5.7:

```xml
<!-- ANTES (C720-like — valores de simulação) -->
<!-- C720: ~78° diagonal ≈ 1.20 rad horizontal FOV at 16:9 -->
<horizontal_fov>1.20</horizontal_fov>
<image>
    <format>R8G8B8</format>
    <width>1280</width>
    <height>720</height>
</image>

<!-- DEPOIS (C270 — substituir 0.92 pelo FOV calculado com seu fx real) -->
<!-- C270: ~53° horizontal ≈ 0.92 rad (calculado com fx=620.5 de calibração) -->
<horizontal_fov>0.92</horizontal_fov>
<image>
    <format>R8G8B8</format>
    <width>640</width>
    <height>480</height>
</image>
```

> O URDF só afeta a câmera simulada no Gazebo. No robô real, quem define o FOV são os parâmetros de calibração. Manter os valores consistentes facilita comparar comportamento sim vs real.

**Recompilar após alterar o URDF:**
```bash
cd ~/sim_ws && colcon build --packages-select sim_bot && source install/setup.bash
```

---

## 9. Montagem Física do Marcador ArUco no Dock

Esta seção é frequentemente ignorada mas causa problemas sérios se não for seguida.

### 9.1 Altura do Marcador

O `dock_pose_estimator.py` converte a posição do marcador de `camera_link_optical` para `odom` via TF. Para isso funcionar corretamente, **o marcador no dock físico deve estar na mesma altura que a câmera no robô**.

No projeto simulado:
- **Altura da câmera:** 0.265m (chassi 0.175m + joint Z 0.09m)
- **Altura do ArUco no dock:** 0.265m (configurado em `models/charging_dock/model.sdf`)

No robô real:
1. Medir a altura do centro da câmera ao chão (com o robô em terreno plano)
2. Colocar o centro do marcador ArUco no dock exatamente nessa mesma altura

```bash
# Para encontrar a altura atual da câmera no robô simulado:
grep -A 3 "camera_joint" ~/sim_ws/src/sim_bot/description/camera.xacro
# origin xyz: o Z absoluto = chassis_joint Z (0.175) + camera_joint Z (0.09) = 0.265m
```

### 9.2 Orientação do Marcador

O marcador deve estar:
- **Perpendicular** ao eixo de abordagem do robô (face voltada diretamente para o robô)
- **Sem inclinação** horizontal ou vertical (< 5° de tolerância)

O `dock_pose_estimator.py` usa `yaw = 0` fixo (face sempre perpendicular a X em odom). Se o marcador estiver inclinado, o target de abordagem ficará ligeiramente deslocado.

### 9.3 Tamanho do Marcador

O marcador impresso deve ser grande o suficiente para ser detectado a ~3m de distância. A C270 a 3m de distância e 640×480 tem ~7px por grau de visão.

Para detecção confiável a 3m com a C270:
- **Mínimo recomendado:** 15cm × 15cm (lado externo com quiet zone)
- **Ideal:** 20cm × 20cm ou maior

O arquivo disponível em `media/aruco-marker-ID=771(maior).png` já está em tamanho adequado.

### 9.4 Iluminação do Dock

A C270 tem sensor pequeno e sofre em ambientes mal iluminados. Para garantir detecção confiável:
- Adicionar uma LED direcional apontada para o marcador (similar ao `fill_dock` light no `factory.world`)
- Evitar luz de fundo (backlighting) atrás do marcador
- O marcador não deve ter reflexo especular (usar papel matte, não brilhante)

---

## 10. Configurar TF Estático para Testes Isolados

O `dock_pose_estimator.py` precisa da cadeia TF `odom → base_footprint → base_link → camera_link_optical` para transformar pontos da câmera para odom. Antes de ter o robô real completo rodando, você pode simular essa cadeia com transforms estáticos para testar a detecção ArUco de forma isolada.

### 10.1 Publicar Transforms Estáticos de Teste

```bash
# Terminal 1 — odom → base_footprint (robô parado em odom(0,0))
source /opt/ros/jazzy/setup.bash
ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z 0 \
  --roll 0 --pitch 0 --yaw 0 \
  --frame-id odom \
  --child-frame-id base_footprint

# Terminal 2 — base_footprint → base_link (fixed joint)
ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z 0 \
  --roll 0 --pitch 0 --yaw 0 \
  --frame-id base_footprint \
  --child-frame-id base_link

# Terminal 3 — base_link → chassis (usar valores do URDF: xyz="0.194 0.050 0.175")
ros2 run tf2_ros static_transform_publisher \
  --x 0.194 --y 0.050 --z 0.175 \
  --roll 0 --pitch 0 --yaw 0 \
  --frame-id base_link \
  --child-frame-id chassis

# Terminal 4 — chassis → camera_link (usar valores do URDF: xyz="0 -0.075 0.09")
ros2 run tf2_ros static_transform_publisher \
  --x 0 --y -0.075 --z 0.09 \
  --roll 0 --pitch 0 --yaw 0 \
  --frame-id chassis \
  --child-frame-id camera_link

# Terminal 5 — camera_link → camera_link_optical (rotação de convenção: rpy="-pi/2 0 -pi/2")
ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z 0 \
  --roll -1.5708 --pitch 0 --yaw -1.5708 \
  --frame-id camera_link \
  --child-frame-id camera_link_optical

# Terminal 6 — chassis → laser_frame (para o LiDAR; xyz="0 0 0.1")
ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z 0.1 \
  --roll 0 --pitch 0 --yaw 0 \
  --frame-id chassis \
  --child-frame-id laser_frame
```

### 10.2 Verificar a Cadeia TF

```bash
source /opt/ros/jazzy/setup.bash

# Ver a árvore TF completa:
ros2 run tf2_tools view_frames
xdg-open /tmp/frames.pdf

# Testar transform específico:
ros2 run tf2_ros tf2_echo odom camera_link_optical
# Deve mostrar a transform sem erros
```

### 10.3 Lançar o Estimador com TFs Estáticos

```bash
# Terminal 7 — câmera
ros2 launch sim_bot camera_real.launch.py

# Terminal 8 — estimador (use_sim_time=false)
ros2 run sim_bot dock_pose_estimator.py \
  --ros-args \
  -p use_sim_time:=false \
  -p marker_id:=771 \
  -p marker_size:=0.15 \
  -p stop_distance:=0.30

# Terminal 9 — verificar publicação
ros2 topic echo /detected_dock_pose
```

Neste modo, `/detected_dock_pose` reporta a posição do marcador relativa a odom como se o robô estivesse em odom(0,0). É útil para:
- Confirmar que ArUco é detectado
- Validar a calibração com o teste de distância (Seção 6.1)
- Ajustar `marker_size` antes de integrar com o robô real

---

## 11. Ajustar Controles da Câmera (Brilho/Contraste)

Em ambientes de fábrica com iluminação variável, pode ser necessário ajustar os controles da câmera via `v4l2-ctl`.

### 11.1 Listar controles disponíveis

```bash
v4l2-ctl --device=/dev/camera_c270 --list-ctrls
```

Controles típicos da C270:
```
brightness 0x00980900 (int) : min=0 max=255 step=1 default=128 value=128
contrast   0x00980901 (int) : min=0 max=255 step=1 default=32  value=32
saturation 0x00980902 (int) : min=0 max=255 step=1 default=32  value=32
gain       0x00980913 (int) : min=0 max=255 step=1 default=0   value=0 flags=inactive
exposure_auto 0x009a0901 (menu) : min=0 max=3 default=3 value=3
```

### 11.2 Ajustar para ambiente de docking

Se o dock estiver em área mal iluminada ou com backlighting:
```bash
# Desabilitar auto-exposure e definir manualmente
v4l2-ctl --device=/dev/camera_c270 \
  --set-ctrl=exposure_auto=1 \
  --set-ctrl=exposure_absolute=300

# Aumentar brilho se necessário
v4l2-ctl --device=/dev/camera_c270 --set-ctrl=brightness=150

# Aumentar contraste para melhorar a distinção preto/branco do ArUco
v4l2-ctl --device=/dev/camera_c270 --set-ctrl=contrast=50
```

### 11.3 Tornar os ajustes persistentes

Para que os ajustes sejam aplicados toda vez que a câmera conectar, adicionar ao arquivo udev:

```bash
sudo nano /etc/udev/rules.d/99-camera-c270.rules
```

Adicionar após a linha existente:
```
# Ajustes de imagem para o dock (executar após criar o device)
SUBSYSTEM=="video4linux", ATTRS{idVendor}=="046d", ATTRS{idProduct}=="0825", \
    ATTR{index}=="0", \
    RUN+="/usr/bin/v4l2-ctl --device=$env{DEVNAME} --set-ctrl=contrast=50"
```

Ou criar um script de inicialização em `/etc/rc.local`:
```bash
#!/bin/bash
v4l2-ctl --device=/dev/camera_c270 --set-ctrl=brightness=150 --set-ctrl=contrast=50
```

---

## 12. Atualizar docking.launch.py para Hardware Real

**Arquivo:** `launch/docking.launch.py`

Criar um arquivo separado para o robô real para não quebrar a simulação:

```bash
cp ~/sim_ws/src/sim_bot/launch/docking.launch.py \
   ~/sim_ws/src/sim_bot/launch/docking_real.launch.py
nano ~/sim_ws/src/sim_bot/launch/docking_real.launch.py
```

**Diferenças em relação ao docking.launch.py de simulação:**

```python
# docking_real.launch.py — dock_estimator node:
parameters=[{
    'use_sim_time':      False,       # ← CRÍTICO: False no robô real
    'marker_id':         771,
    'marker_size':       0.15,        # ← medir o marcador impresso (borda exterior)
    'stop_distance':     0.30,        # ← ajustar empiricamente (ver abaixo)
    'aruco_timeout':     2.0,
    'sector_half_deg':   30.0,
    'close_sector_deg':  60.0,
    'close_range_m':     0.5,
}]

# docking_real.launch.py — docking_server node:
parameters=[docking_params, {'use_sim_time': False}]   # ← False

# docking_real.launch.py — lifecycle_manager node:
parameters=[{
    'use_sim_time': False,             # ← False
    'autostart': True,
    'node_names': ['docking_server'],
}]

# docking_real.launch.py — charging_manager node:
parameters=[{'use_sim_time': False}]   # ← False
```

**Atualizar `docking_params.yaml` com a pose real do dock:**

```yaml
# config/docking_params.yaml — base_carregamento section:
base_carregamento:
  type: 'charging_dock'
  frame: 'odom'
  # Substituir [13.0, 0.0, 0.0] pela posição REAL do dock no ambiente físico
  # Medir com trena a partir do ponto de spawn do robô (odom origin)
  pose: [X_REAL, Y_REAL, YAW_REAL]
```

**Como calibrar `stop_distance` empiricamente no hardware real:**

1. Iniciar com `stop_distance=0.30`
2. Enviar `DockRobot` e observar onde o robô para
3. Medir o gap entre a frente do robô e a face do dock com uma régua
4. Gap desejado: **~5cm**
5. Ajustar dinamicamente sem restart:
   ```bash
   ros2 param set /dock_pose_estimator stop_distance 0.32
   ```
6. Testar novamente. Repetir em passos de 1cm.

---

## 13. Criar o Launch File do Robô Real

Este é o passo final — substituir o `palmares_bot.launch.py` (que lança Gazebo) por um launch file que usa hardware real.

Crie o arquivo `launch/palmares_bot_real.launch.py`:

```bash
nano ~/sim_ws/src/sim_bot/launch/palmares_bot_real.launch.py
```

**Conteúdo:**
```python
"""
palmares_bot_real.launch.py — lança o sistema completo para hardware real.

Diferenças em relação ao palmares_bot.launch.py (simulação):
  - SEM Gazebo (sem gz_sim, sem ros_gz_bridge)
  - use_sim_time: False em todos os nós
  - Câmera: usb_cam (não gz_bridge)
  - TF odom→base_footprint: vem do hardware (odom_to_tf.py assina /odom do encoder)
  - joint_state_publisher: publica estados das juntas sem sim_time
  - Sem TimerAction — tudo sobe de uma vez (hardware não precisa aguardar Gazebo)

Pré-requisitos:
  - /odom sendo publicado pelo hardware (encoder + diffdrive controller)
  - /scan sendo publicado pelo LiDAR
  - /cmd_vel chegando no hardware (motor controller)
"""

import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, DeclareLaunchArgument, GroupAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg = get_package_share_directory('sim_bot')
    home = os.path.expanduser('~')

    # ── Argumentos ────────────────────────────────────────────────────────
    declare_rviz = DeclareLaunchArgument(
        'rviz', default_value='True',
        description='Abrir RViz')
    declare_slam = DeclareLaunchArgument(
        'slam', default_value='True',
        description='Habilitar SLAM')
    declare_nav = DeclareLaunchArgument(
        'nav', default_value='True',
        description='Habilitar Nav2')

    read_rviz = LaunchConfiguration('rviz')
    read_slam = LaunchConfiguration('slam')
    read_nav  = LaunchConfiguration('nav')

    # ── Robot State Publisher (URDF → TF estático dos sensores) ──────────
    urdf_path = os.path.join(pkg, 'description', 'palmares_bot.urdf.xacro')
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'rsp.launch.py')),
        # use_sim_time: FALSE no hardware real
        launch_arguments={'use_sim_time': 'false', 'urdf': urdf_path}.items()
    )

    # ── odom → TF relay (converte /odom do encoder para TF odom→base_footprint) ──
    # Mesmo nó usado na simulação — funciona igual com hardware real
    odom_to_tf = Node(
        package='sim_bot',
        executable='odom_to_tf.py',
        name='odom_to_tf',
        output='screen',
        parameters=[{'use_sim_time': False}]    # ← False no hardware
    )

    # ── joint_state_publisher (TF das rodas, sem sim_time) ────────────────
    joint_state_pub = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': False}],   # ← False no hardware
        output='screen',
    )

    # ── Câmera USB (C270) ──────────────────────────────────────────────────
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'camera_real.launch.py'))
    )

    # ── Twist Mux (multiplexador: Nav2 vs teleop vs charging_manager) ────
    twist_mux_params = os.path.join(pkg, 'config', 'twist_mux_params.yaml')
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        parameters=[twist_mux_params, {'use_sim_time': False}],
        remappings=[('/cmd_vel_out', '/cmd_vel')]
    )

    # ── RViz ──────────────────────────────────────────────────────────────
    rviz_config = os.path.join(pkg, 'rviz', 'bot.rviz')
    rviz2 = GroupAction(
        condition=IfCondition(read_rviz),
        actions=[Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        )]
    )

    # ── SLAM ──────────────────────────────────────────────────────────────
    slam_node = GroupAction(
        condition=IfCondition(read_slam),
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, 'launch', 'slam.launch.py')),
            launch_arguments={'use_sim_time': 'false'}.items()
        )]
    )

    # ── Nav2 ──────────────────────────────────────────────────────────────
    nav_params = os.path.join(pkg, 'config', 'nav_params.yaml')
    nav_node = GroupAction(
        condition=IfCondition(read_nav),
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, 'launch', 'nav.launch.py')),
            launch_arguments={
                'use_sim_time': 'false',
                'params_file':  nav_params
            }.items()
        )]
    )

    # ── Sistema de Docking ─────────────────────────────────────────────────
    docking = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'docking_real.launch.py'))
    )

    return LaunchDescription([
        declare_rviz,
        declare_slam,
        declare_nav,

        rsp,
        odom_to_tf,
        joint_state_pub,
        camera,
        twist_mux,
        rviz2,
        slam_node,
        nav_node,
        docking,
    ])
```

**Recompilar:**
```bash
cd ~/sim_ws && colcon build --packages-select sim_bot && source install/setup.bash
```

**Lançar o sistema real:**
```bash
ros2 launch sim_bot palmares_bot_real.launch.py
```

> **Nota:** Diferente da simulação, não há `TimerAction`. O hardware não precisa aguardar Gazebo subir — todos os nós sobem ao mesmo tempo. O `AUTOSTART_DELAY` no `charging_manager.py` (30s) ainda serve como buffer para Nav2 e docking server ficarem prontos.

---

## 14. Testar a Detecção ArUco no Sistema Completo

Com o robô real rodando, execute este protocolo de teste antes do primeiro docking autônomo.

### 14.1 Verificar todos os tópicos

```bash
# Todos esses devem estar ativos:
ros2 topic list | grep -E "camera|scan|odom|detected_dock|charging"
```

Esperado:
```
/camera/camera_info
/camera/image
/scan
/odom
/detected_dock_pose
/charging_manager/state
/battery_level
```

### 14.2 Verificar frequências

```bash
ros2 topic hz /camera/image         # ~30 Hz
ros2 topic hz /scan                 # ~10 Hz
ros2 topic hz /odom                 # ~30 Hz
ros2 topic hz /detected_dock_pose   # ~10-30 Hz quando ArUco visível, ~10 Hz com LiDAR
```

### 14.3 Posicionar o robô e verificar detecção

1. Posicionar o robô a ~2m do dock, frente voltada para o ArUco
2. Verificar a detecção:

```bash
ros2 topic echo /detected_dock_pose
```

Esperado (robô a 2m do dock, stop_distance=0.30):
```yaml
header:
  frame_id: odom
pose:
  position:
    x: 1.70   # ≈ 2.00m - 0.30m de stop_distance
    y: 0.00   # ± 0.03m de tolerância
    z: 0.0
```

O log do nó mostrará:
```
[dock_pose_estimator]: ArUco: dock=(2.023, 0.008) target=(1.723, 0.008) yaw=0.0°
```

### 14.4 Testar o primeiro DockRobot manualmente

Antes de deixar o `charging_manager` controlar tudo, testar a action diretamente:

```bash
# Verificar que a action existe:
ros2 action list | grep dock
# Deve aparecer: /dock_robot

# Enviar o goal manualmente:
ros2 action send_goal /dock_robot nav2_msgs/action/DockRobot \
  '{use_dock_id: true, dock_id: "base_carregamento", navigate_to_staging_pose: true}'
```

Observar o robô navegar até o staging e depois abordar o dock.

### 14.5 Diagnóstico de problemas de detecção

| Sintoma | Causa provável | Solução |
|---------|---------------|---------|
| Sem detecção, câmera OK | `camera_matrix` zerada | Verificar Seção 6.2 |
| Sem detecção, CameraInfo OK | Iluminação ruim | Ver Seção 11 ou Seção 9.4 |
| Detecção intermitente | Câmera vibra com movimento | Fixar câmera no robô rigidamente |
| X errado (scale error) | `marker_size` incorreto | Refazer teste da Seção 6.1 |
| Y oscilando | Calibração ruim (RMS > 1px) | Refazer calibração |
| TF error no log | Cadeia TF quebrada | `ros2 run tf2_tools view_frames` |
| LiDAR vê dock mas ArUco não | Marker muito pequeno / longe | Aumentar marcador ou iluminação |

---

## 15. Checklist Final de Integração

### Hardware

- [ ] C270 conectada e `/dev/camera_c270` existe: `ls -la /dev/camera_c270`
- [ ] Usuário no grupo `video`: `groups $USER | grep video`
- [ ] LiDAR conectado e publicando: `ros2 topic hz /scan` → ~10 Hz
- [ ] Odometria publicando: `ros2 topic hz /odom` → ~30 Hz com `frame_id: odom`
- [ ] `/cmd_vel` controlando o robô: `ros2 topic pub --once /cmd_vel ...`

### Câmera

- [ ] `/camera/image` a ~30 Hz: `ros2 topic hz /camera/image`
- [ ] `CameraInfo.K[0] ≠ 0`: `ros2 topic echo /camera/camera_info --once | grep "k:"`
- [ ] Imagem não invertida/espelhada: `ros2 run rqt_image_view rqt_image_view`
- [ ] Teste de distância conhecida passou (Seção 6.1): `x ≈ 50cm - stop_distance`
- [ ] `marker_size` medido fisicamente e atualizado em `docking_real.launch.py`
- [ ] Erro de reprojeção da calibração < 0.5px

### Montagem Física

- [ ] Marcador ArUco na mesma altura que a câmera: `h_câmera = h_marcador`
- [ ] Marcador perpendicular ao eixo de abordagem do robô
- [ ] Iluminação do dock adequada (sem backlighting, sem reflexo)

### TF

- [ ] `odom → base_footprint` com timestamp recente: `ros2 run tf2_ros tf2_echo odom base_footprint`
- [ ] `base_footprint → laser_frame` existe
- [ ] `base_footprint → camera_link_optical` existe
- [ ] `ros2 run tf2_tools view_frames` mostra árvore completa sem quebras

### Parâmetros

- [ ] `use_sim_time: False` em TODOS os nós do `docking_real.launch.py`
- [ ] `stop_distance` calibrado empiricamente (gap ~5cm sem colisão)
- [ ] Pose do dock em `docking_params.yaml` com coordenadas reais do ambiente
- [ ] `AUTOSTART_DELAY` adequado (padrão 30s — verificar se Nav2 sobe antes)

### Docking

- [ ] `/detected_dock_pose` publicando a ~10-30 Hz quando dock visível
- [ ] `DockRobot` manual testado e bem-sucedido (status=4)
- [ ] `charging_manager` atingiu `HOME_CHARGING` no primeiro arranque
- [ ] Ciclo completo: tarefa → retorno → carregamento → nova tarefa

---

## 16. Referências Rápidas

```bash
# ── Câmera ───────────────────────────────────────────────────────────────
# Verificar câmera no Linux
v4l2-ctl --device=/dev/camera_c270 --list-formats-ext

# Verificar grupo video
groups $USER | grep video

# Lançar câmera isolada
ros2 launch sim_bot camera_real.launch.py

# Ver imagem ao vivo
ros2 run rqt_image_view rqt_image_view

# Verificar calibração carregada (K[0] ≠ 0.0)
ros2 topic echo /camera/camera_info --once | grep -A 1 "k:"

# ── Calibração ───────────────────────────────────────────────────────────
ros2 run camera_calibration cameracalibrator \
  --size 8x6 --square 0.025 \
  --ros-args --remap image:=/camera/image --remap camera:=/camera

# ── TF ───────────────────────────────────────────────────────────────────
# Ver árvore TF
ros2 run tf2_tools view_frames && xdg-open /tmp/frames.pdf

# Testar transformação específica
ros2 run tf2_ros tf2_echo odom camera_link_optical

# ── Detecção ArUco ────────────────────────────────────────────────────────
# Monitorar pose detectada
ros2 topic echo /detected_dock_pose

# Ajustar stop_distance sem restart
ros2 param set /dock_pose_estimator stop_distance 0.32

# ── Controles da câmera ───────────────────────────────────────────────────
v4l2-ctl --device=/dev/camera_c270 --list-ctrls
v4l2-ctl --device=/dev/camera_c270 --set-ctrl=brightness=150
v4l2-ctl --device=/dev/camera_c270 --set-ctrl=contrast=50

# ── Sistema completo ──────────────────────────────────────────────────────
# Lançar robô real
ros2 launch sim_bot palmares_bot_real.launch.py

# Estado da máquina de estados
ros2 topic echo /charging_manager/state --once

# Docking manual
ros2 action send_goal /dock_robot nav2_msgs/action/DockRobot \
  '{use_dock_id: true, dock_id: "base_carregamento", navigate_to_staging_pose: true}'

# Forçar retorno ao dock
ros2 topic pub --once /go_charge std_msgs/msg/Empty '{}'
```
