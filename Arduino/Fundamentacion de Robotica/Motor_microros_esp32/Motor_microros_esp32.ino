#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>  // Cambiamos a Float32 para valores decimales

// Definición de pines
const int IN1 = 18;      // Conectado a IN1 del L298N
const int IN2 = 15;      // Conectado a IN2 del L298N
const int ENA = 4;       // Conectado a ENA del L298N (Enable A)

// Configuración de PWM
const int freq = 5000;          // Frecuencia de PWM en Hz
const int resolution = 8;       // Resolución de 8 bits (0-255)
const int pwmChannel = 0;       // Canal PWM para ENA

// Variables de micro-ROS
rcl_subscription_t subscriber;
std_msgs__msg__Float32 msg;  // Usamos Float32 para valores decimales
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// Función para controlar el motor
void controlMotor(float value) {
  // Ajuste de valores recibidos para controlar el motor
  if (value >= -1 && value <= 1) {
    value = value * 255;
  } else if (value > 1) {
    value = 255;
  } else if (value < -1) {
    value = -255;
  }

  // Control de dirección y velocidad
  if (value > 0) {
    // Motor en una dirección (adelante)
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    ledcWrite(pwmChannel, value);  // Velocidad proporcional
  } else if (value < 0) {
    // Motor en la dirección opuesta (atrás)
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    ledcWrite(pwmChannel, -value);  // Velocidad proporcional
  } else {
    // Motor detenido
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    ledcWrite(pwmChannel, 0);  // Velocidad 0
  }
}

// Callback del subscriber
void subscription_callback(const void *msgin) {
  const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msgin;
  float value = msg->data;
  controlMotor(value);
}

void setup() {
  // Configura los pines como salida
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  // Configura el canal PWM para ENA
  ledcSetup(pwmChannel, freq, resolution);
  ledcAttachPin(ENA, pwmChannel);

  // Inicia micro-ROS
  set_microros_transports();
  delay(2000);

  // Configura el allocator
  allocator = rcl_get_default_allocator();

  // Inicia el soporte de micro-ROS
  rclc_support_init(&support, 0, NULL, &allocator);

  // Crea el nodo
  rclc_node_init_default(&node, "motor_control_node", "", &support);

  // Crea el subscriber
  rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),  // Usamos Float32
    "/motor_control");

  // Crea el executor
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA);
}

void loop() {
  // Ejecuta el executor
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
  delay(100);
}
