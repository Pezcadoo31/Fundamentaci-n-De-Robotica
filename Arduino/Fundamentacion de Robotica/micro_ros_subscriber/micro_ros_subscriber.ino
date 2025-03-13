#include <micro_ros_arduino.h>  // Biblioteca para usar micro-ROS en Arduino

// Bibliotecas necesarias para la comunicación con micro-ROS
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>  // Tipo de mensaje estándar de ROS para enteros de 32 bits

// Declaración del suscriptor
rcl_subscription_t subscriber;  
std_msgs__msg__Int32 msg_pub; // Variable para almacenar el mensaje recibido
rclc_executor_t executor;     // Executor para manejar tareas de ROS
rclc_support_t support;       // Soporte para la comunicación ROS
rcl_allocator_t allocator;    // Administrador de memoria para ROS
rcl_node_t node;              // Nodo de ROS que ejecutará el programa
rcl_timer_t timer;            // Temporizador (no utilizado en este código)

#define LED_PIN 2  // Pin donde está conectado el LED

// Macros para manejar errores en llamadas a funciones de ROS
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Función de error: parpadea el LED en caso de fallo y entra en un bucle infinito
void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

// Callback que se ejecuta cuando llega un nuevo mensaje
void subscription_callback(const void * msgin)
{  
  // Se castea el mensaje recibido al tipo correcto
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  
  // Enciende el LED si el dato recibido es diferente de 0, lo apaga si es 0
  digitalWrite(LED_PIN, (msg->data == 0) ? LOW : HIGH);  
}

void setup() {
  set_microros_transports();  // Configura el transporte de micro-ROS
  
  pinMode(LED_PIN, OUTPUT);  // Configura el pin del LED como salida
  digitalWrite(LED_PIN, HIGH);  // Enciende el LED inicialmente
  
  delay(2000);  // Espera 2 segundos antes de continuar

  allocator = rcl_get_default_allocator();  // Inicializa el administrador de memoria

  // Inicializa el soporte de ROS
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Crea un nodo de ROS con nombre "micro_ros_arduino_node"
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // Crea el suscriptor que escuchará mensajes del tópico "micro_ros_arduino_subscriber"
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "micro_ros_arduino_subscriber"));

  // Inicializa el executor con 1 tarea
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  
  // Agrega la suscripción al executor, vinculándola a la función de callback
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
}

void loop() {
  delay(100);  // Pequeña pausa para evitar sobrecarga
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));  // Ejecuta las tareas pendientes de ROS
}

