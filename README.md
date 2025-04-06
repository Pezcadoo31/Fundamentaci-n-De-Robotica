# 🚀 Proyecto ROS 2 y micro-ROS | Fundamentación de Robótica

Este repositorio contiene el desarrollo completo de mis prácticas, desafíos y ejercicios realizados durante la materia **Fundamentación de Robótica**, con el objetivo de adquirir conocimientos sólidos en **ROS 2** y **micro-ROS**, aplicados a control de motores.

> Curso impartido con el acompañamiento de los profesores de la carrera y el apoyo de [Manchester Robotics Ltd.](https://github.com/ManchesterRoboticsLtd), quienes proporcionaron retos y clases introductorias a la robótica moderna con ROS 2.

---

## 🧭 Estructura del Repositorio

### 🔷 `ros2_ws/src`
Esta es la carpeta principal del espacio de trabajo ROS 2 (`ros2_ws`), dentro de la cual se encuentran diversos **paquetes ROS** que me permitieron comprender el funcionamiento de la arquitectura ROS 2, su comunicación y diseño modular.

#### Paquetes incluidos:
- **`basic_comms`**: Primeros pasos con la comunicación básica en ROS 2.
- **`challenge1`, `challenge2`, `challenge4`**: Miniretos prácticos propuestos por Manchester Robotics Ltd. para poner en práctica el conocimiento de tópicos, nodos, timers y más.
- **`custom_interfaces` / `my_interfaces`**: Diseño y uso de interfaces personalizadas para comunicación avanzada entre nodos.
- **`mini_challenge`**: Retos complementarios al aprendizaje, con enfoque en modularidad y lógica de control.
- **`motor_control`**: Implementación de control para un **motor DC de 12V con encoder**, usando ROS 2.
- **`signal_processing` / `step_signal_pkg`**: Generación y análisis de señales como escalón, seno y cuadrada, útiles para pruebas de respuesta y control.

---

### 🔶 `Arduino`

Incluye los proyectos realizados con **ESP32 y micro-ROS**, divididos en dos carpetas según su origen:  
#### 1. `Fundamentación de Robótica`  
Contiene ejercicios y pruebas realizados con el acompañamiento del profesor en clase, incluyendo:

- `CHALLENGES/` – Miniretos propuestos para aplicar micro-ROS y microcontroladores.
- `Encoder_microros_esp32` – Lectura de encoder y envío de datos a través de micro-ROS.
- `Motor_esp32` / `Motor_microros_esp32` – Control de un motor DC mediante ESP32 y comunicación ROS.
- `LED_ESP32` – Práctica básica de control de un LED con micro-ROS.
- `SumadorDeUnNumero` – Implementación de un nodo que suma valores recibidos.
- `REV_MIN_` y `REV_SEG_` – Revisión de funcionamiento con mensajes y temporización.
- `micro_ros_publisher` y `micro_ros_subscriber` – Implementación de nodos publicadores y suscriptores.

#### 2. `Manchester Robotics`
Proyectos realizados directamente con el contenido proporcionado por **Manchester Robotics Ltd.**, como ejemplos de conexión WiFi, reconexión, y nodos duales:

- `micro_ros_publisher`
- `micro_ros_reconnection_example_wifi_v2`
- `micro_ros_subscriber`
- `publisher_subscriber`

---

## 💡 Aprendizajes Clave

- Comprensión de la estructura y flujo de trabajo en ROS 2 (espacio de trabajo, nodos, tópicos, timers, interfaces personalizadas).
- Uso de **interfaces personalizadas** en ROS 2 para mejorar la comunicación entre nodos.
- Desarrollo de **paquetes funcionales** para control de motores con **ESP32 y ROS 2**.
- Introducción a **micro-ROS**, ejecutando nodos en microcontroladores para integrarlos en arquitecturas ROS completas.
- Aplicación del conocimiento en **retos prácticos** que simulan problemas reales de robótica.

---

## ⚙️ Requisitos

Para ejecutar los proyectos se requiere:
- **ROS 2 (Humble)**
- **Python3 y CMake**
- **ESP32 con firmware micro-ROS** cargado
- Dependencias de `micro_ros_setup` instaladas

---

## ✨ Créditos

- **Desarrollado por:** Abdiel Vicencio 🤖  
- **Contribución externa:** [Manchester Robotics Ltd.](https://github.com/ManchesterRoboticsLtd)  
- **Institución:** Ingeniería Robótica – Tecnologico De Monterrey CCM



