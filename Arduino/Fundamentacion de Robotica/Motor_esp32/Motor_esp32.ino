// Definición de pines
const int IN1 = 4;      // Conectado a IN1 del L298N
const int IN2 = 5;      // Conectado a IN2 del L298N
const int ENA = 18;     // Conectado a ENA del L298N (Enable A)

// Configuración de PWM
const int freq = 5000;  // Frecuencia de PWM en Hz
const int resolution = 8;  // Resolución de 8 bits (0-255)
const int pwmChannel = 0;  // Canal PWM para ENA

void setup() {
  // Configura los pines como salida
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  // Configura el canal PWM para ENA
  ledcSetup(pwmChannel, freq, resolution);
  ledcAttachPin(ENA, pwmChannel);

  // Inicia la comunicación serial
  Serial.begin(9600);
  Serial.println("Ingresa un valor entre -1 y 1 para controlar el motor:");
}

void loop() {
  // Verifica si hay datos disponibles en el serial
  if (Serial.available() > 0) {
    // Lee el valor ingresado
    float motorValue = Serial.parseFloat();
    
    // Limpia el buffer del serial
    while (Serial.available() > 0) {
      Serial.read();
    }

    // Controla el motor con el valor ingresado
    controlMotor(motorValue);

    // Muestra el valor ingresado en el monitor serial
    Serial.print("Valor ingresado: ");
    Serial.println(motorValue);
  }
}

void controlMotor(float value) {
  // Asegúrate de que el valor esté entre -1 y 1
  value = constrain(value, -1, 1);

  // Control de dirección y velocidad
  if (value > 0) {
    // Motor en una dirección
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    ledcWrite(pwmChannel, value * 255);  // Control de velocidad en ENA
  } else if (value < 0) {
    // Motor en la dirección opuesta
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    ledcWrite(pwmChannel, -value * 255);  // Control de velocidad en ENA
  } else {
    // Motor detenido
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    ledcWrite(pwmChannel, 0);  // Velocidad 0 en ENA
  }
}
