# Guia de Instalação — palmares_bot Docking Autônomo

Sistema operacional: **Ubuntu 24.04**
ROS: **ROS2 Jazzy**
Simulador: **Gazebo Harmonic (gz-sim 8)**

---

## 1. Instalar o ROS2 Jazzy

Se o ROS2 Jazzy já estiver instalado, pule esta seção.

```bash
# Locale
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Repositório ROS2
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Instalar ROS2 Jazzy Desktop (inclui RViz, rqt, etc.)
sudo apt update
sudo apt install -y ros-jazzy-desktop

# Ferramentas de desenvolvimento
sudo apt install -y python3-rosdep python3-colcon-common-extensions python3-pip
```

---

## 2. Instalar Gazebo Harmonic

```bash
# Repositório Gazebo
sudo curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
  -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
  http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt update

# Integração ROS2 Jazzy ↔ Gazebo Harmonic
sudo apt install -y ros-jazzy-ros-gz
```

---

## 3. Instalar Dependências do Pacote sim_bot

### 3.1 Navegação e SLAM

```bash
sudo apt install -y \
  ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox \
  ros-jazzy-twist-mux \
  ros-jazzy-teleop-twist-joy \
  ros-jazzy-joy
```

### 3.2 Docking Autônomo

```bash
sudo apt install -y \
  ros-jazzy-opennav-docking \
  ros-jazzy-opennav-docking-core
```

### 3.3 Visão Computacional (ArUco)

```bash
sudo apt install -y \
  ros-jazzy-cv-bridge \
  python3-opencv

# Verificar versão do OpenCV (deve ser 4.x com módulo aruco)
python3 -c "import cv2; print(cv2.__version__); import cv2.aruco; print('aruco OK')"
```

> **Atenção:** O projeto usa a API legada do ArUco (`cv2.aruco.Dictionary_get` e `cv2.aruco.DetectorParameters_create`). Isso funciona com OpenCV 4.6.x (padrão no Ubuntu 24.04). Se tiver OpenCV 4.8+ e a API nova, é necessário adaptar o `dock_pose_estimator.py`.

### 3.4 TF2 e Geometria

```bash
sudo apt install -y \
  ros-jazzy-tf2-ros \
  ros-jazzy-tf2-geometry-msgs \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-xacro
```

### 3.5 Câmera USB (apenas para robô real — não necessário para simulação)

```bash
sudo apt install -y ros-jazzy-usb-cam
```

### 3.6 Calibração de Câmera (apenas se for fazer calibração)

```bash
sudo apt install -y ros-jazzy-camera-calibration
# ou via pip se o pacote não existir no apt:
pip3 install camera-calibration
```

### 3.7 Dependências Python

```bash
pip3 install numpy
# numpy 1.26.x já vem com Ubuntu 24.04, mas garantir:
python3 -c "import numpy; print(numpy.__version__)"   # deve ser >= 1.20
```

---

## 4. Criar o Workspace e Clonar o Pacote

```bash
mkdir -p ~/sim_ws/src
cd ~/sim_ws/src

# Clonar o repositório (branch com docking completo no galpão de fábrica)
git clone -b carregamento-automatico-fabrica \
  https://github.com/marlon655/sim_bot.git
```

> **Nota sobre branches:**
> - `carregamento-automatico-fabrica` — branch atual, galpão factory.world, docking completo
> - `carregamento-sem-cam` — docking somente com LiDAR (sem câmera/ArUco)
> - `dock-carregamento` — versão inicial, ambiente pequeno (test.world)

---

## 5. Resolver Dependências com rosdep

```bash
# Inicializar rosdep (apenas na primeira vez)
sudo rosdep init
rosdep update

# Instalar dependências declaradas no package.xml
cd ~/sim_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

Se `sudo rosdep init` retornar "file already exists", ignore e execute apenas `rosdep update`.

---

## 6. Compilar o Pacote

```bash
cd ~/sim_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select sim_bot
source install/setup.bash
```

**Compilação rápida (sem recompilar dependências):**
```bash
colcon build --packages-select sim_bot --symlink-install
```

> `--symlink-install` cria links simbólicos para os arquivos Python e de configuração, de modo que alterações em `nodes/`, `config/`, `launch/`, etc., ficam disponíveis imediatamente sem recompilar.

---

## 7. Adicionar ao .bashrc (Opcional mas Recomendado)

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "source ~/sim_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 8. Verificar a Instalação

```bash
# Verificar que o pacote é visível
ros2 pkg prefix sim_bot
# Saída esperada: /home/<usuario>/sim_ws/install/sim_bot

# Verificar que os executáveis estão instalados
ros2 pkg executables sim_bot
# Saída esperada:
#   sim_bot alignment_test.py
#   sim_bot charging_manager.py
#   sim_bot dock_pose_estimator.py
#   sim_bot odom_to_tf.py

# Verificar que opennav_docking está disponível
ros2 pkg prefix opennav_docking
```

---

## 9. Primeira Execução

```bash
# Sempre que abrir um novo terminal:
source /opt/ros/jazzy/setup.bash
source ~/sim_ws/install/setup.bash

# Lançar a simulação completa:
ros2 launch sim_bot palmares_bot.launch.py
```

Ver o arquivo `PASSAGEM_CONHECIMENTO.md` para entender a sequência de inicialização e como enviar tarefas.

---

## 10. Dependências Completas (Resumo)

| Pacote | Tipo | Necessário Para |
|--------|------|-----------------|
| `ros-jazzy-desktop` | ROS2 | Base completa |
| `ros-jazzy-ros-gz` | ROS2 | Bridge Gazebo ↔ ROS2 |
| `ros-jazzy-nav2-bringup` | ROS2 | Navegação autônoma |
| `ros-jazzy-slam-toolbox` | ROS2 | Mapeamento SLAM |
| `ros-jazzy-opennav-docking` | ROS2 | Sistema de docking |
| `ros-jazzy-opennav-docking-core` | ROS2 | Plugin SimpleChargingDock |
| `ros-jazzy-cv-bridge` | ROS2 | Integração OpenCV ↔ ROS2 |
| `ros-jazzy-tf2-ros` | ROS2 | Transformadas de coordenadas |
| `ros-jazzy-tf2-geometry-msgs` | ROS2 | TF2 com PoseStamped |
| `ros-jazzy-twist-mux` | ROS2 | Multiplexador cmd_vel |
| `ros-jazzy-joy` | ROS2 | Joystick (opcional) |
| `ros-jazzy-teleop-twist-joy` | ROS2 | Teleop joystick (opcional) |
| `ros-jazzy-robot-state-publisher` | ROS2 | Publicar URDF/TF |
| `ros-jazzy-joint-state-publisher` | ROS2 | TF das rodas (monotônico) |
| `ros-jazzy-xacro` | ROS2 | Processar URDF xacro |
| `python3-opencv` | Python | Detecção ArUco |
| `python3-numpy` | Python | Cálculos numéricos |
| `ros-jazzy-usb-cam` | ROS2 | Câmera USB real (C270) |
| `ros-jazzy-camera-calibration` | ROS2 | Calibração da câmera |
