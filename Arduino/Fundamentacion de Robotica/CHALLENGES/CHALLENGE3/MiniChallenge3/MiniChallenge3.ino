#include <micro_ros_arduino.h>  
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>  // Usamos Float32 para valores decimales

// Definición de pines según el desafío
#define M1_IN1 18  // GPIO 18 para IN1 del controlador de motor
#define M1_IN2 15  // GPIO 15 para IN2 del controlador de motor
#define M1_PWM 4   // GPIO 4 para PWM del controlador de motor

// Configuración de PWM
#define PWM_FREQ 980        // Frecuencia de PWM en Hz (980 Hz según el desafío)
#define PWM_RESOLUTION 8    // Resolución de 8 bits (0-255)
#define PWM_CHANNEL 0       // Canal PWM para el motor

// Variables de micro-ROS
rcl_subscription_t subscriber;  // Suscriptor para el topico /cmd_pwm
std_msgs__msg__Float32 msg;     // Mensaje de tipo Float32 para recibir valores entre [-1, 1]
rclc_executor_t executor;       // Executor de micro-ROS
rclc_support_t support;         // Soporte de micro-ROS
rcl_allocator_t allocator;      // Allocator de micro-ROS
rcl_node_t node;                // Nodo de micro-ROS

// Función para controlar el motor
void controlMotor(float value) {
  // Asegurar que el valor esté entre -1 y 1
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
    digitalWrite(M1_IN1, HIGH);
    digitalWrite(M1_IN2, LOW);
    ledcWrite(PWM_CHANNEL, value * 255);  // Velocidad proporcional (0-255)
  } else if (value < 0) {
    // Motor en la dirección opuesta (atrás)
    digitalWrite(M1_IN1, LOW);
    digitalWrite(M1_IN2, HIGH);
    ledcWrite(PWM_CHANNEL, -value * 255);  // Velocidad proporcional (0-255)
  } else {
    // Motor detenido
    digitalWrite(M1_IN1, LOW);
    digitalWrite(M1_IN2, LOW);
    ledcWrite(PWM_CHANNEL, 0);  // Velocidad 0
  }
}

// Callback del suscriptor
void subscription_callback(const void *msgin) {
  const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msgin;
  float value = msg->data;  // Obtener el valor del mensaje
  controlMotor(value);      // Controlar el motor con el valor recibido
}

void setup() {
  // Configura los pines como salida
  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);
  pinMode(M1_PWM, OUTPUT);

  // Configura el canal PWM
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(M1_PWM, PWM_CHANNEL);

  // Inicia micro-ROS
  set_microros_transports();  // Configura los transportes de micro-ROS
  delay(2000);                // Espera para asegurar la inicialización

  // Configura el allocator
  allocator = rcl_get_default_allocator();

  // Inicia el soporte de micro-ROS
  rclc_support_init(&support, 0, NULL, &allocator);

  // Crea el nodo
  rclc_node_init_default(&node, "motor_node", "", &support);

  // Crea el suscriptor para el topico /cmd_pwm
  rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),  // Tipo de mensaje Float32
    "/cmd_pwm");  // Topico al que se suscribe

  // Crea el executor
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA);
}

void loop() {
  // Ejecuta el executor para procesar mensajes
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
  delay(100);  
}
