#define LED_PIN2 2  // Definir el pin donde está conectado el LED

void setup() {
  pinMode(LED_PIN2, OUTPUT); // Configurar el pin como salida
}

void loop() {
  digitalWrite(LED_PIN2, HIGH); // Encender el LED
  delay(200);                  // Esperar 1 segundo
  digitalWrite(LED_PIN2, LOW);  // Apagar el LED
  delay(200);                  // Esperar 1 segundo
}

