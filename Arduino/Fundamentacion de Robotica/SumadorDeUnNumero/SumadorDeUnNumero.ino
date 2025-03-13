#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32.h>

// Definición de nodos y variables de ROS
rcl_subscription_t subscriber;   // Suscriptor
rcl_publisher_t publisher;       // Publicador
std_msgs__msg__Float32 msg_sub;    // Mensaje recibido
std_msgs__msg__Float32 msg_pub;    // Mensaje a publicar
rclc_executor_t executor;        // Executor de ROS
rclc_support_t support;          // Soporte de ROS
rcl_allocator_t allocator;       // Administrador de memoria
rcl_node_t node;                 // Nodo de ROS
rcl_timer_t timer;               // Timer para publicación constante

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Función de error: entra en un bucle infinito
void error_loop(){
  while(1){
    delay(100);
  }
}

// Callback cuando llega un nuevo mensaje al suscriptor
void subscription_callback(const void * msgin)
{  
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
  
  // Sumar 3 al número recibido
  msg_pub.data = msg_sub.data + 3;

  // Publicar el resultado
  RCSOFTCHECK(rcl_publish(&publisher, &msg_pub, NULL));
}

// Callback del timer para publicar constantemente 0 si no hay nuevos datos
void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    // Si el último mensaje publicado no fue modificado por el suscriptor, publicar 0
    if (msg_pub.data != 3) {
      msg_pub.data = 0;
      RCSOFTCHECK(rcl_publish(&publisher, &msg_pub, NULL));
    }
  }
}

void setup() {
  set_microros_transports();  // Configura el transporte de micro-ROS
  
  delay(2000);  // Espera 2 segundos antes de continuar

  allocator = rcl_get_default_allocator();  // Inicializa el administrador de memoria

  // Inicializa el soporte de ROS
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Crea un nodo de ROS con nombre "micro_ros_arduino_node"
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // Crea el suscriptor que recibe datos del tópico "micro_ros_arduino_subscriber"
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "micro_ros_arduino_subscriber"));

  // Crea el publicador que enviará datos al tópico "micro_ros_arduino_publisher"
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "micro_ros_arduino_publisher"));

  // Crea el timer para publicar constantemente
  const unsigned int timer_timeout = 1000;  // Cada 1 segundo
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // Inicializa el executor con 2 tareas (suscripción y timer)
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));

  // Agrega la suscripción al executor
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg_sub, &subscription_callback, ON_NEW_DATA));

  // Agrega el timer al executor
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  // Inicializa el mensaje en 0
  msg_pub.data = 0;
}

void loop() {
  delay(100);  // Pequeña pausa para evitar sobrecarga
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));  // Ejecuta las tareas pendientes de ROS
}
