#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h> // Se incluye la librería para mensajes de tipo int32


rcl_publisher_t publisher;      // Manejador del publicador
std_msgs__msg__Int32 msg_pub;   // Mensaje de tipo Int32 que será enviado
rclc_executor_t executor;       // Manejador del ejecutor
rclc_support_t support;         // Estructura de soporte para inicialización
rcl_allocator_t allocator;      // Administrador de memoria para micro-ROS
rcl_node_t node;                // Nodo de ROS2 en micro-ROS
rcl_timer_t timer;              // Temporizador para ejecutar eventos periódicos

#define LED_PIN 13  // Pin del LED integrado

// Macros para verificar errores en llamadas a funciones de micro-ROS
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}  
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // Parpadeo del LED en caso de error
    delay(100);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, &msg_pub, NULL)); // Publica el mensaje
    msg_pub.data++; // Incrementa el valor enviado en cada ciclo
  }
}

void setup() {
  set_microros_transports(); // Configura la comunicación con micro-ROS
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  // Enciende el LED al inicio
  
  delay(2000); // Espera inicial para estabilidad
  
  allocator = rcl_get_default_allocator(); // Asigna el administrador de memoria

  // Inicializa el soporte de micro-ROS
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Crea el nodo de micro-ROS
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // Crea el publicador
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "micro_ros_arduino_node_publisher"));

  // Configura un temporizador con intervalo de 1 segundo
  const unsigned int timer_timeout = 1000;  
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // Inicializa el ejecutor y le agrega el temporizador
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  msg_pub.data = 0; // Inicializa el mensaje en 0
}

void loop() {
  delay(100); // Pequeña espera para evitar saturación de CPU
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100))); // Ejecuta el ciclo del ejecutor
}

