#include <micro_ros_arduino.h>  // Incluir la biblioteca de micro-ROS
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>  // Usamos Float32 para los mensajes

// Definición de pines según el desafío
#define M1_IN1 18  // GPIO 18 para IN1 del controlador de motor
#define M1_IN2 15  // GPIO 15 para IN2 del controlador de motor
#define M1_PWM 4   // GPIO 4 para PWM del controlador de motor

// Pines del encoder
#define ENCODER_A 34  // GPIO 34 para Fase A del encoder
#define ENCODER_B 35  // GPIO 35 para Fase B del encoder

// Configuración de PWM
#define PWM_FREQ 980        // Frecuencia de PWM en Hz (980 Hz según el desafío)
#define PWM_RESOLUTION 8    // Resolución de 8 bits (0-255)
#define PWM_CHANNEL 0       // Canal PWM para el motor

// Variables de micro-ROS
rcl_subscription_t subscriber;  // Suscriptor para el tema /motor_input
rcl_publisher_t publisher;      // Publicador para el tema /motor_output
std_msgs__msg__Float32 pwm_msg; // Mensaje para recibir valores de /motor_input
std_msgs__msg__Float32 output_msg; // Mensaje para publicar en /motor_output
rclc_executor_t executor;       // Executor de micro-ROS
rclc_support_t support;         // Soporte de micro-ROS
rcl_allocator_t allocator;      // Allocator de micro-ROS
rcl_node_t node;                // Nodo de micro-ROS
rcl_timer_t timer;              // Timer para publicar a 50 Hz

// Variables para el encoder
volatile int encoder_count = 0;  // Contador de las cuentas del encoder
int last_encoder_state = 0;      // Estado anterior del encoder

// Función para controlar el motor
void controlMotor(float value) {
  // Ajustar el valor recibido para controlar el motor
  if (value >= -1.0 && value <= 1.0) {
    value = value * 255;  // Mapear a [-255, 255]
  } else if (value > 1.0) {
    value = 255;  // Límite superior
  } else if (value < -1.0) {
    value = -255;  // Límite inferior
  }

  // Control de dirección y velocidad
  if (value > 0) {
    // Motor en una dirección (adelante)
    digitalWrite(M1_IN1, HIGH);
    digitalWrite(M1_IN2, LOW);
    ledcWrite(PWM_CHANNEL, value);  // Velocidad proporcional
  } else if (value < 0) {
    // Motor en la dirección opuesta (atrás)
    digitalWrite(M1_IN1, LOW);
    digitalWrite(M1_IN2, HIGH);
    ledcWrite(PWM_CHANNEL, -value);  // Velocidad proporcional
  } else {
    // Motor detenido
    digitalWrite(M1_IN1, LOW);
    digitalWrite(M1_IN2, LOW);
    ledcWrite(PWM_CHANNEL, 0);  // Velocidad 0
  }
}

// Función de interrupción para leer el encoder
void IRAM_ATTR readEncoder() {
  int stateA = digitalRead(ENCODER_A);
  int stateB = digitalRead(ENCODER_B);
  int current_state = (stateA << 1) | stateB;  // Combinar los estados de A y B

  // Lógica para determinar la dirección y actualizar el contador
  if (current_state != last_encoder_state) {
    if ((last_encoder_state == 0 && current_state == 2) ||
        (last_encoder_state == 2 && current_state == 3) ||
        (last_encoder_state == 3 && current_state == 1) ||
        (last_encoder_state == 1 && current_state == 0)) {
      encoder_count++;  // Giro en sentido horario
    } else {
      encoder_count--;  // Giro en sentido antihorario
    }
    last_encoder_state = current_state;
  }
}

// Callback del suscriptor
void subscription_callback(const void *msgin) {
  const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msgin;
  float value = msg->data;  // Obtener el valor del mensaje
  controlMotor(value);      // Controlar el motor con el valor recibido
}

// Callback del timer para publicar a 50 Hz
void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  (void)last_call_time;  // Evitar advertencia de parámetro no utilizado

  // Publicar las cuentas del encoder en el tema /motor_output
  output_msg.data = encoder_count;
  rcl_publish(&publisher, &output_msg, NULL);
}

void setup() {
  // Configura los pines como salida
  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);
  pinMode(M1_PWM, OUTPUT);

  // Configura el canal PWM
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(M1_PWM, PWM_CHANNEL);

  // Configura los pines del encoder como entrada
  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);

  // Configura la interrupción para leer el encoder
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), readEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), readEncoder, CHANGE);

  // Inicia micro-ROS
  set_microros_transports();  // Configura los transportes de micro-ROS
  delay(2000);                // Espera para asegurar la inicialización

  // Configura el allocator
  allocator = rcl_get_default_allocator();

  // Inicia el soporte de micro-ROS
  rclc_support_init(&support, 0, NULL, &allocator);

  // Crea el nodo
  rclc_node_init_default(&node, "motor_node", "", &support);

  // Crea el suscriptor para el tema /motor_input
  rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),  // Tipo de mensaje Float32
    "/motor_input");  // Tema al que se suscribe

  // Crea el publicador para el tema /motor_output
  rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),  // Tipo de mensaje Float32
    "/motor_output");  // Tema al que se publica

  // Crea el timer para publicar a 50 Hz (20 ms)
  rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(20),  // Intervalo de 20 ms (50 Hz)
    timer_callback);    // Función de callback del timer

  // Crea el executor
  rclc_executor_init(&executor, &support.context, 2, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber, &pwm_msg, &subscription_callback, ON_NEW_DATA);
  rclc_executor_add_timer(&executor, &timer);
}

void loop() {
  // Ejecuta el executor para procesar mensajes y el timer
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
  delay(10);  // Pequeño delay para evitar sobrecarga
}
