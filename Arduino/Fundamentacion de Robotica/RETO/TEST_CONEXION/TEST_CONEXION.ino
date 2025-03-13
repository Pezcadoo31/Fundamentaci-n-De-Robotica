#include <micro_ros_arduino.h>  // Incluir la biblioteca de micro-ROS
#include <WiFi.h>               // Incluir la biblioteca de Wi-Fi
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>  // Usamos Float32 para los mensajes

// Configuración de Wi-Fi
const char* ssid = "AVANS 9756";    // Reemplaza con el nombre de tu red Wi-Fi
const char* password = "kJ040$35";  // Reemplaza con la contraseña de tu red Wi-Fi

// Definición de pines según el desafío
#define M1_IN1 19  // GPIO 18 para IN1 del controlador de motor
#define M1_IN2 21  // GPIO 15 para IN2 del controlador de motor
#define M1_PWM 18   // GPIO 4 para PWM del controlador de motor

// Pines del encoder
#define ENCODER_A 22  // GPIO 34 para Fase A del encoder
#define ENCODER_B 23  // GPIO 35 para Fase B del encoder

// Configuración de PWM
#define PWM_FREQ 980        // Frecuencia de PWM en Hz (980 Hz según el desafío)
#define PWM_RESOLUTION 8    // Resolución de 8 bits (0-255)
#define PWM_CHANNEL 0       // Canal PWM para el motor

// Variables de micro-ROS
rcl_subscription_t subscriber;  // Suscriptor para el topico /motor_input
rcl_subscription_t kp_subscriber;  // Suscriptor para ajustar Kp
rcl_subscription_t ki_subscriber;  // Suscriptor para ajustar Ki
rcl_publisher_t publisher;      // Publicador para el topico /motor_output
rcl_publisher_t metrics_publisher;  // Publicador para métricas de desempeño
std_msgs__msg__Float32 pwm_msg; // Mensaje para recibir valores de /motor_input
std_msgs__msg__Float32 kp_msg;  // Mensaje para recibir Kp
std_msgs__msg__Float32 ki_msg;  // Mensaje para recibir Ki
std_msgs__msg__Float32 output_msg; // Mensaje para publicar en /motor_output
std_msgs__msg__Float32 metrics_msg; // Mensaje para publicar métricas
rclc_executor_t executor;       // Executor de micro-ROS
rclc_support_t support;         // Soporte de micro-ROS
rcl_allocator_t allocator;      // Allocator de micro-ROS
rcl_node_t node;                // Nodo de micro-ROS
rcl_timer_t timer;              // Timer para publicar

// Variables para el encoder
volatile int encoder_count = 0;  // Contador de las cuentas del encoder
int last_encoder_state = 0;      // Estado anterior del encoder

// Constantes para el cálculo de rev/s
const int ENCODER_PPR = 22;      // Resolución del encoder (11 PPR x 2 canales)
const int GEAR_RATIO = 34;       // Relación de reducción (1:34)

// Variables para el cálculo de rev/s
float rev_per_second = 0.0;      // Revoluciones por segundo
unsigned long last_time = 0;     // Tiempo de la última lectura

// Variables para el controlador PI
float Kp = 0.022;  // Ganancia proporcional
float Ki = 0.12;  // Ganancia integral
float error = 0.0;
float integral = 0.0;
float setpoint = 0.0;  // Valor deseado de velocidad

// Variables para métricas de desempeño
float Eee = 0.0;  // Error en estado estacionario
float ts = 0.0;   // Tiempo de establecimiento
float Mp = 0.0;   // Sobreimpulso
float tp = 0.0;   // Tiempo pico
float tr = 0.0;   // Tiempo de crecimiento
float ise = 0.0;  // Integral del error cuadrático
float iae = 0.0;  // Integral del error absoluto
float itse = 0.0; // Integral del tiempo y error cuadrático
float itae = 0.0; // Integral del tiempo y error absoluto

// Función para controlar el motor con un controlador PI
void controlMotorPI(float current_speed) {
  error = setpoint - current_speed;  // Calcular el error
  integral += error;                // Acumular el error para el término integral

  // Calcular la señal de control (PWM)
  float control_signal = Kp * error + Ki * integral;

  // Limitar la señal de control al rango [-1, 1]
  if (control_signal > 1.0) control_signal = 1.0;
  if (control_signal < -1.0) control_signal = -1.0;

  // Mapear la señal de control a PWM y dirección
  float pwm_duty_cycle = fabs(control_signal);  // Valor absoluto para el ciclo de trabajo
  int direction = (control_signal >= 0) ? 1 : -1;  // Dirección (1 = CCW, -1 = CW)

  // Aplicar la señal de control al motor
  if (direction == 1) {
    // Rotación en sentido CCW
    digitalWrite(M1_IN1, LOW);
    digitalWrite(M1_IN2, HIGH);
  } else if (direction == -1) {
    // Rotación en sentido CW
    digitalWrite(M1_IN1, HIGH);
    digitalWrite(M1_IN2, LOW);
  } else {
    // Motor detenido
    digitalWrite(M1_IN1, LOW);
    digitalWrite(M1_IN2, LOW);
  }

  // Convertir el ciclo de trabajo a un valor de PWM (0-255)
  int pwm_value = (int)(pwm_duty_cycle * 255);
  ledcWrite(PWM_CHANNEL, pwm_value);  // Aplicar el PWM
}

// Función de interrupción para leer el encoder
void IRAM_ATTR readEncoder() {
  int stateA = digitalRead(ENCODER_A);  // Leer el estado del pin A del encoder
  int stateB = digitalRead(ENCODER_B);  // Leer el estado del pin B del encoder
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
    last_encoder_state = current_state;  // Actualizar el estado anterior
  }
}

// Callback del suscriptor para el setpoint
void subscription_callback(const void *msgin) {
  const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msgin;
  float input_value = msg->data;  // Obtener el valor del mensaje

  // Limitar el valor de entrada al rango [-1, 1]
  if (input_value > 1.0) input_value = 1.0;
  if (input_value < -1.0) input_value = -1.0;

  setpoint = input_value;  // Actualizar el setpoint
}

// Callback del suscriptor para Kp
void kp_callback(const void *msgin) {
  const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msgin;
  Kp = msg->data;  // Actualizar Kp
}

// Callback del suscriptor para Ki
void ki_callback(const void *msgin) {
  const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msgin;
  Ki = msg->data;  // Actualizar Ki
}

// Función para calcular métricas de desempeño
void calculateMetrics(float current_speed) {
  // Error en estado estacionario
  Eee = setpoint - current_speed;

  // Integral del error cuadrático (ISE)
  ise += error * error;

  // Integral del error absoluto (IAE)
  iae += fabs(error);

  // Integral del tiempo y error cuadrático (ITSE)
  itse += micros() * error * error;

  // Integral del tiempo y error absoluto (ITAE)
  itae += micros() * fabs(error);

  // Publicar métricas
  metrics_msg.data = Eee;  // Publicar el error en estado estacionario
  rcl_publish(&metrics_publisher, &metrics_msg, NULL);
}

// Callback del timer para calcular y publicar rev/s
void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  RCL_UNUSED(last_call_time);

  // Calcular las revoluciones por segundo (rev/s)
  unsigned long current_time = micros();
  float time_elapsed = (current_time - last_time) / 1e6;  // Tiempo en segundos

  if (time_elapsed > 0) {  // Evitar división por cero
    rev_per_second = encoder_count / (ENCODER_PPR * GEAR_RATIO * time_elapsed);
    output_msg.data = rev_per_second;  // Asignar el valor calculado al mensaje
    rcl_publish(&publisher, &output_msg, NULL);  // Publicar el mensaje

    // Aplicar el controlador PI
    controlMotorPI(rev_per_second);

    // Calcular métricas de desempeño
    calculateMetrics(rev_per_second);

    // Reiniciar el contador y el tiempo
    encoder_count = 0;
    last_time = current_time;
  }
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
  // Conectar a la red Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Conectado a la red Wi-Fi");

  // Inicia micro-ROS con transporte Wi-Fi
  IPAddress agent_ip(192, 168, 137, 154);  // Reemplaza con la IP de tu agente micro-ROS
  char agent_ip_str[16];  // Buffer para almacenar la dirección IP como cadena
  snprintf(agent_ip_str, sizeof(agent_ip_str), "%d.%d.%d.%d", agent_ip[0], agent_ip[1], agent_ip[2], agent_ip[3]);

  set_microros_wifi_transports((char*)ssid, (char*)password, agent_ip_str, 8888);

  delay(2000);                // Espera para asegurar la inicialización

  // Configura el allocator
  allocator = rcl_get_default_allocator();

  // Inicia el soporte de micro-ROS
  rclc_support_init(&support, 0, NULL, &allocator);

  // Crea el nodo
  rclc_node_init_default(&node, "control_node", "", &support);

  // Crea el suscriptor para el topico /motor_input
  rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),  // Tipo de mensaje Float32
    "/motor_input");  // Topico al que se suscribe

  // Crea el suscriptor para el topico /kp
  rclc_subscription_init_default(
    &kp_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),  // Tipo de mensaje Float32
    "/kp");  // Topico para ajustar Kp

  // Crea el suscriptor para el topico /ki
  rclc_subscription_init_default(
    &ki_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),  // Tipo de mensaje Float32
    "/ki");  // Topico para ajustar Ki

  // Crea el publicador para el topico /motor_output
  rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),  // Tipo de mensaje Float32
    "/motor_output");  // Topico al que se publica

  // Crea el publicador para el topico /metrics
  rclc_publisher_init_default(
    &metrics_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),  // Tipo de mensaje Float32
    "/metrics");  // Topico para publicar métricas

  // Crea el timer para calcular rev/s cada 100 ms (10 Hz)
  rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(50),  // Intervalo de 100 ms
    timer_callback);      // Función de callback del timer

  // Crea el executor
  rclc_executor_init(&executor, &support.context, 4, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber, &pwm_msg, &subscription_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &kp_subscriber, &kp_msg, &kp_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &ki_subscriber, &ki_msg, &ki_callback, ON_NEW_DATA);
  rclc_executor_add_timer(&executor, &timer);
}

void loop() {
  // Ejecuta el executor para procesar mensajes, el timer y los parámetros
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
  delay(10);  // Pequeño delay para evitar sobrecarga
}

