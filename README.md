[English](#english) | [Español](#español)

# English
## 🇬🇧 Sancho ROS2 Workspace 🤖

Welcome to the **Sancho ROS2 Workspace** repository! Here you’ll find everything needed to deploy, build, and run your Sancho robot—a modular system based on ROS 2 Humble and a modern web interface built with React and Vite.

---

## 📋 Table of Contents

1. [✨ Features](#-features)  
2. [🚀 Prerequisites](#-prerequisites)
3. [🦾 Required Hardware](#-required-hardware)
4. [📥 Repository Download](#-repository-download)  
5. [🔧 Installing Dependencies](#-installing-dependencies)  
   - 4.1 [ROS2 Dependencies (`package.xml`)](#41-ros2-dependencies-packagexml)  
   - 4.2 [Python Dependencies (`requirements.txt`)](#42-python-dependencies-requirementstxt)  
   - 4.3 [Node.js Dependencies (Web Client)](#43-nodejs-dependencies-web-client)  
6. [🏭 Workspace Compilation](#-workspace-compilation)  
7. [▶️ Getting Started](#️-getting-started)  
   - 6.1 [Robot Startup](#61-robot-startup)  
   - 6.2 [Interaction Modules](#62-interaction-modules)  
   - 6.3 [Navigation](#63-navigation)  
   - 6.4 [Web Server & Nginx Proxy](#64-web-server--nginx-proxy)  
   - 6.5 [Web Interface Development Mode](#65-web-interface-development-mode)  
8. [🗂️ Project Structure](#️-project-structure)  
9. [🛠️ Contributions](#️-contributions)  
10. [📄 License](#-license)  

---

## ✨ Features

- Modular: 13 custom ROS2 packages (`sancho_audio`, `sancho_navigation`, `sancho_web`, etc.).  
- Multimodal interaction: social vision, face detection and tracking, VAD-enabled audio, head movement.  
- Full Nav2 stack: mapping, localization, planning.  
- Modern web UI: React + Vite with WebSocket/HTTP proxy using Nginx.  
- Ready-to-use launch scripts (`launch.py`) for all components.  

---

## 🚀 Prerequisites

Make sure you have the following:

- **OS**: Ubuntu 22.04 LTS  
- **ROS 2**: Humble Hawksbill  
  ```bash
  sudo apt update
  sudo apt install ros-humble-desktop
  sudo rosdep init
  rosdep update
  ```

- **Basic tools**: `git`, `pip`, `colcon`  
  ```bash
  sudo apt install git python3-pip python3-colcon-common-extensions
  ```

- **Nginx** (WebSocket/HTTP reverse proxy)  
  ```bash
  sudo apt install nginx
  sudo systemctl enable --now nginx
  ```

- **Node.js** (LTS ≥18) and `npm` for React/Vite UI  
  ```bash
  curl -fsSL https://deb.nodesource.com/setup_20.x | sudo -E bash -
  sudo apt install -y nodejs
  node -v  # v20.x
  npm -v   # ≥10
  ```

---
## 🦾 Required Hardware

Below is a detailed list of the devices used in the robotic system **“Sancho.”**  
All components have been tested together; however, you can replace them with equivalents as long as they offer similar performance.

- **Mobile Base — AgileX Ranger Mini V3**  
  A 4×4 platform with independent steering, turning radius ≤ 0.42 m, and a 48 V Li-ion battery. It serves as the chassis, locomotion actuator, and power source.

- **Onboard Computer**  
  MinisForum HM80 Mini-PC with **AMD Ryzen 7 4800U**, **32 GB RAM**, and **512 GB NVMe SSD**; provides the computing power required for ROS 2, vision, and real-time planning.

- **Perception Sensors**
  - 2 × **Hokuyo UTM-30LX LIDARs** (270° @ 40 Hz) for mapping and safe navigation.
  - **Orbbec Astra RGB-D Camera** (640 × 480 @ 30 Hz) with integrated stereo MEMS microphones, used for user detection and TDOA computation.
  - **5 MP USB RGB Camera** (ELP-USB500W05G-L170, FoV ≈ 170°) mounted on the head for frontal interaction.

- **Pan-Tilt Unit**  
  Interbotix **WidowX Dual XM430** (±105° in pan/tilt, Dynamixel XM430-W350-T servos) to orient the front camera toward the speaker.
---

## 📥 Repository Download

Clone this repo into an empty workspace:

```bash
git clone https://github.com/antbaena/sancho_ws.git
cd ~/sancho_ws
```

> ➡️ `src/` contains 13 custom ROS2 packages plus third-party dependencies.

---

## 🔧 Installing Dependencies

### 4.1 ROS2 Dependencies (`package.xml`)

```bash
cd ~/sancho_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 4.2 Python Dependencies (`requirements.txt`)

- **Individually**:  
  ```bash
  pip install -r src/sancho_perception/requirements.txt
  pip install -r src/sancho_audio/requirements.txt
  # …
  ```

- **All at once**:  
  ```bash
  find src -name requirements.txt -exec pip install -r {} \;
  ```

### 4.3 Node.js Dependencies (Web Client)

```bash
cd src/sancho_web/ros-web-client
npm install
npm run build
```

---

## 🏭 Workspace Compilation

```bash
cd ~/sancho_ws
colcon build
source install/setup.bash
```

---

## ▶️ Getting Started

### 6.1 Robot Startup

- **View URDF in RViz2**:  
  ```bash
  ros2 launch sancho_description display.launch.py
  ```
- **Start base and sensors**:  
  ```bash
  ros2 launch sancho_bringup sancho_bringup.launch.py
  ```

### 6.2 Interaction Modules

| Module                         | Command                                                        |
|--------------------------------|----------------------------------------------------------------|
| Full perception stack          | `ros2 launch sancho_perception full_perception_launch.py`     |
| Social vision (faces)          | `ros2 launch sancho_vision detector_tracker.launch.py`        |
| Audio + VAD                    | `ros2 launch sancho_audio audio_player.launch.py`             |
| Head movement                  | `ros2 launch sancho_head_control sancho_head_launch.py`       |
| Interaction manager            | `ros2 launch sancho_interaction sancho_interaction_manager.launch.py` |
| Social orchestrator            | `ros2 run sancho_orchestrator orchestrator_node`              |

### 6.3 Navigation

```bash
ros2 launch sancho_navigation navigation_launch.py
```

### 6.4 Web Server & Nginx Proxy

```bash
ros2 launch sancho_web web.launch.py
scripts/start_nginx.sh
scripts/stop_nginx.sh
```

### 6.5 Web Interface Development Mode

```bash
cd src/sancho_web/ros-web-client
npm run dev      # http://localhost:5173
npm run preview  # http://localhost:4173
```

---

## 🗂️ Project Structure

```text
sancho_ws/
├── src/
│   ├── sancho_audio/
│   ├── sancho_bringup/
│   ├── sancho_description/
│   ├── sancho_demo/
│   ├─ …
└── install/
```

---

## 🛠️ Contributions

We welcome **issues** and **pull requests**! Please:

1. Open an **issue** describing your proposal or bug.  
2. Create a branch (`git checkout -b feature/your-feature`).  
3. Make changes and add tests if needed.  
4. Open a **pull request** describing your work.

---

## 📄 License

This project is under **GPL-3** license. See the [LICENSE](LICENSE) file for more.

---
# Español
## 🇪🇸 Sancho ROS2 Workspace 🤖

¡Bienvenido al repositorio **Sancho ROS2 Workspace**! Aquí encontrarás todo lo necesario para desplegar, compilar y ejecutar tu robot Sancho, un sistema modular basado en ROS 2 Humble y una interfaz web moderna con React y Vite.

## 📋 Tabla de Contenidos

1. [✨ Características](#-características)  
2. [🚀 Requisitos Previos](#-requisitos-previos)
3. [🦾 Hardware necesario](#-hardware-necesario)  
4. [📥 Descarga del Repositorio](#-descarga-del-repositorio)  
5. [🔧 Instalación de Dependencias](#-instalación-de-dependencias)  
   - 4.1 [Dependencias ROS2 (`package.xml`)](#41-dependencias-ros2-packagexml)  
   - 4.2 [Dependencias Python (`requirements.txt`)](#42-dependencias-python-requirementstxt)  
   - 4.3 [Dependencias Node.js (cliente web)](#43-dependencias-nodejs-cliente-web)  
6. [🏭 Compilación del Workspace](#-compilación-del-workspace)  
7. [▶️ Puesta en Marcha](#️-puesta-en-marcha)  
   - 6.1 [Arranque del Robot](#61-arranque-del-robot)  
   - 6.2 [Módulos de Interacción](#62-módulos-de-interacción)  
   - 6.3 [Navegación](#63-navegación)  
   - 6.4 [Servidor Web & Proxy Nginx](#64-servidor-web--proxy-nginx)  
   - 6.5 [Modo Desarrollo de la Interfaz Web](#65-modo-desarrollo-de-la-interfaz-web)  
8. [🗂️ Estructura del Proyecto](#️-estructura-del-proyecto)  
9. [🛠️ Contribuciones](#️-contribuciones)  
10. [📄 Licencia](#-licencia)  

---
## ✨ Características

- Modular: 13 paquetes ROS2 propios (`sancho_audio`, `sancho_navigation`, `sancho_web`, etc.).  
- Interacción multimodal: visión social, detección y seguimiento de rostros, audio con VAD, movimiento de cabeza.  
- Pila completa de navegación Nav2: mapeado, localización y planificación.  
- Interfaz web moderna: React + Vite con proxy WebSocket/HTTP mediante Nginx.  
- Scripts y lanzadores (`launch.py`) para todos los componentes.  

---

## 🚀 Requisitos Previos

Antes de empezar, asegúrate de contar con lo siguiente:

- **Sistema operativo**: Ubuntu 22.04 LTS 🐧  
- **ROS 2**: Humble Hawksbill 🦅  
  ```bash
  sudo apt update
  sudo apt install ros-humble-desktop
  sudo rosdep init
  rosdep update
  ```

- **Herramientas básicas**: `git`, `pip`, `colcon`  
  ```bash
  sudo apt install git python3-pip python3-colcon-common-extensions
  ```

- **Nginx** (proxy inverso WebSocket/HTTP)  
  ```bash
  sudo apt install nginx
  sudo systemctl enable --now nginx
  ```

- **Node.js** (versión LTS ≥18) y `npm` para compilar la interfaz React/Vite  
  ```bash
  curl -fsSL https://deb.nodesource.com/setup_20.x | sudo -E bash -
  sudo apt install -y nodejs
  node -v  # v20.x
  npm -v   # ≥10
  ```

---
## 🦾 Hardware necesario 

A continuación se detalla el conjunto de dispositivos empleados en el sistema robótico «Sancho».  
Todos los componentes han sido validados juntos; sin embargo, puedes sustituirlos por equivalentes si mantienes prestaciones similares.

- **Base móvil — AgileX Ranger Mini V3**  
  Plataforma 4 × 4 con dirección independiente, radio de giro ≤ 0,42 m y batería Li-ion 48 V. Sirve de chasis, actuador de locomoción y fuente de energía. :contentReference[oaicite:0]{index=0}

- **Ordenador de a bordo**  
  Mini-PC MinisForum HM80 con **AMD Ryzen 7 4800U**, **32 GB RAM** y **SSD NVMe 512 GB**; proporciona la potencia de cálculo necesaria para ROS 2, visión y planificación en tiempo real. 

- **Sensores de percepción**
  - 2 × **LIDAR Hokuyo UTM-30LX** (270 ° @ 40 Hz) para mapeo y navegación segura. :contentReference[oaicite:1]{index=1}
  - **Cámara RGB-D Orbbec Astra** (640 × 480 @ 30 Hz) con micrófonos MEMS estéreo integrados, usada para detección de usuarios y cálculo TDOA. :contentReference[oaicite:2]{index=2}
  - **Cámara USB RGB 5 MP** (ELP-USB500W05G-L170, FoV ≈ 170 °) montada en la cabeza para interacción frontal. :contentReference[oaicite:3]{index=3}

- **Unidad pan-tilt**  
  Interbotix **WidowX Dual XM430** (± 105 ° en pan/tilt, servos Dynamixel XM430-W350-T) que orienta la cámara frontal hacia el interlocutor. :contentReference[oaicite:4]{index=4}

---

## 📥 Descarga del Repositorio

Clona el repositorio dentro de un workspace vacío:

```bash
git clone https://github.com/antbaena/sancho_ws.git
cd ~/sancho_ws
```

> ➡️ El directorio `src/` contiene 13 paquetes ROS2 propios.

---

## 🔧 Instalación de Dependencias

### 4.1 Dependencias ROS2 (`package.xml`)

Instala las dependencias declaradas en cada `package.xml`:

```bash
cd ~/sancho_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 4.2 Dependencias Python (`requirements.txt`)

- **Individual**:  
  ```bash
  pip install -r src/sancho_perception/requirements.txt
  pip install -r src/sancho_audio/requirements.txt
  # …
  ```

- **Todas a la vez**:  
  ```bash
  find src -name requirements.txt -exec pip install -r {} \\\;
  ```

### 4.3 Dependencias Node.js (cliente web)

```bash
cd src/sancho_web/ros-web-client
npm install    # instala React, Vite y librerías auxiliares
npm run build  # genera producción en dist/
```

---

## 🏭 Compilación del Workspace

```bash
cd ~/sancho_ws
colcon build
source install/setup.bash
```

---

## ▶️ Puesta en Marcha

### 6.1 Arranque del Robot

- **Visualizar URDF en RViz2**:  
  ```bash
  ros2 launch sancho_description display.launch.py
  ```
- **Bring-up de base y sensores**:  
  ```bash
  ros2 launch sancho_bringup sancho_bringup.launch.py
  ```

### 6.2 Módulos de Interacción

| Módulo                         | Comando                                                        |
|--------------------------------|----------------------------------------------------------------|
| Percepción completa            | `ros2 launch sancho_perception full_perception_launch.py`    |
| Visión social (rostros)        | `ros2 launch sancho_vision detector_tracker.launch.py`       |
| Audio + VAD                    | `ros2 launch sancho_audio audio_player.launch.py`            |
| Movimiento de la cabeza        | `ros2 launch sancho_head_control sancho_head_launch.py`      |
| Gestor de interacción          | `ros2 launch sancho_interaction sancho_interaction_manager.launch.py` |
| Orquestador social             | `ros2 run sancho_orchestrator orchestrator_node`             |

### 6.3 Navegación

```bash
ros2 launch sancho_navigation navigation_launch.py
```

### 6.4 Servidor Web & Proxy Nginx

- **Arrancar servidor web**:  
  ```bash
  ros2 launch sancho_web web.launch.py
  ```
- **Control Nginx (opcional)**:  
  ```bash
  scripts/start_nginx.sh  # Inicia proxy WebSocket
  scripts/stop_nginx.sh   # Detiene Nginx
  ```

### 6.5 Modo Desarrollo de la Interfaz Web

Durante el desarrollo, sirve la SPA con Vite:

```bash
cd src/sancho_web/ros-web-client
npm run dev    # http://localhost:5173
# Para previsualizar producción:
npm run preview  # http://localhost:4173
```

> 🛠️ Ajusta el proxy de WebSocket en `vite.config.js` si el rosbridge está en otra máquina o puerto.

---

## 🗂️ Estructura del Proyecto

```text
sancho_ws/
├── src/
│   ├── sancho_audio/
│   ├── sancho_bringup/
│   ├── sancho_description/
│   ├── sancho_demo/
│   ├─ …
└── install/
```

---

## 🛠️ Contribuciones

¡Se aceptan **issues** y **pull requests**! Por favor:

1. Abre un **issue** detallando tu propuesta o bug.  
2. Crea una rama (`git checkout -b feature/nueva-funcionalidad`).  
3. Realiza tus cambios y añade tests si procede.  
4. Abre un **pull request** describiendo los cambios.

---

## 📄 Licencia

Este proyecto está bajo licencia **MIT**. Consulta el fichero [LICENSE](LICENSE) para más detalles.

---

¡Gracias por usar y contribuir al robot **Sancho**! 🌟
