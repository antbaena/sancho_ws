<p align="right">
  <a href="#english">🇬🇧 English</a> | <a href="#español">🇪🇸 Español</a>
</p>

#  Sancho ROS2 Workspace 🤖

Welcome to the **Sancho ROS2 Workspace** repository! Here you’ll find everything needed to deploy, build, and run your Sancho robot—a modular system based on ROS 2 Humble and a modern web interface built with React and Vite.

---

## 📋 Table of Contents

1. [✨ Features](#-features)  
2. [🚀 Prerequisites](#-prerequisites)  
3. [📥 Repository Download](#-repository-download)  
4. [🔧 Installing Dependencies](#-installing-dependencies)  
   - 4.1 [ROS2 Dependencies (`package.xml`)](#41-ros2-dependencies-packagexml)  
   - 4.2 [Python Dependencies (`requirements.txt`)](#42-python-dependencies-requirementstxt)  
   - 4.3 [Node.js Dependencies (Web Client)](#43-nodejs-dependencies-web-client)  
5. [🏗️ Workspace Compilation](#-workspace-compilation)  
6. [▶️ Getting Started](#️-getting-started)  
   - 6.1 [Robot Startup](#61-robot-startup)  
   - 6.2 [Interaction Modules](#62-interaction-modules)  
   - 6.3 [Navigation](#63-navigation)  
   - 6.4 [Web Server & Nginx Proxy](#64-web-server--nginx-proxy)  
   - 6.5 [Web Interface Development Mode](#65-web-interface-development-mode)  
7. [🗂️ Project Structure](#️-project-structure)  
8. [🛠️ Contributions](#️-contributions)  
9. [📄 License](#-license)  

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

## 📥 Repository Download

Clone this repo into an empty workspace:

```bash
mkdir -p ~/sancho_ws/src
cd ~/sancho_ws/src
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

## 🏗️ Workspace Compilation

```bash
cd ~/sancho_ws
colcon build --symlink-install
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

# 🇪🇸 Español

_Para ver esta sección en inglés, haz clic en la bandera 🇬🇧 arriba._

## Sancho ROS2 Workspace 🤖

¡Bienvenido al repositorio **Sancho ROS2 Workspace**! Aquí encontrarás todo lo necesario para desplegar, compilar y ejecutar tu robot Sancho, un sistema modular basado en ROS 2 Humble y una interfaz web moderna con React y Vite.

## 📋 Tabla de Contenidos

1. [✨ Características](#-características)  
2. [🚀 Requisitos Previos](#-requisitos-previos)  
3. [📥 Descarga del Repositorio](#-descarga-del-repositorio)  
4. [🔧 Instalación de Dependencias](#-instalación-de-dependencias)  
   - 4.1 [Dependencias ROS2 (`package.xml`)](#41-dependencias-ros2-packagexml)  
   - 4.2 [Dependencias Python (`requirements.txt`)](#42-dependencias-python-requirementstxt)  
   - 4.3 [Dependencias Node.js (cliente web)](#43-dependencias-nodejs-cliente-web)  
5. [🏗️ Compilación del Workspace](#-compilación-del-workspace)  
6. [▶️ Puesta en Marcha](#️-puesta-en-marcha)  
   - 6.1 [Arranque del Robot](#61-arranque-del-robot)  
   - 6.2 [Módulos de Interacción](#62-módulos-de-interacción)  
   - 6.3 [Navegación](#63-navegación)  
   - 6.4 [Servidor Web & Proxy Nginx](#64-servidor-web--proxy-nginx)  
   - 6.5 [Modo Desarrollo de la Interfaz Web](#65-modo-desarrollo-de-la-interfaz-web)  
7. [🗂️ Estructura del Proyecto](#️-estructura-del-proyecto)  
8. [🛠️ Contribuciones](#️-contribuciones)  
9. [📄 Licencia](#-licencia)  

---

¡Gracias por usar y contribuir al robot **Sancho**! 🌟
