/*
 * ROBOT SEGUIDOR DE LÍNEAS NEGRAS
 * ------------------------------
 * Este programa controla un robot seguidor de líneas con un proceso de calibración
 * en tres pasos y detección inteligente tanto de línea negra como de ausencia de superficie.
 * 
 * Autor: Fernández Hernán Gustavo
 * Última actualización: Abril 2025
 */

//---------------------------------------------------------------------------------
// DEFINICIÓN DE PINES
//---------------------------------------------------------------------------------

// Sensores de línea
#define S1            A2  // Sensor izquierda extrema
#define S2            A1  // Sensor izquierda media
#define S3            A3  // Sensor derecha media
#define S4            A4  // Sensor derecha extrema
#define SL            A0  // Sensor lateral izquierdo
#define SR            A5  // Sensor lateral derecho

// Control
#define SWITCH        12  // Pulsador de inicio
#define LED           11  // LED indicador de estado

// Motores
#define MOTOR_RIGHT   10  // Motor derecho (PWM)
#define MOTOR_LEFT    9   // Motor izquierdo (PWM)

//---------------------------------------------------------------------------------
// CONFIGURACIÓN DEL CONTROLADOR PID
//---------------------------------------------------------------------------------
float Kp = 35.0;          // Constante proporcional
float Kd = 15.0;          // Constante derivativa
float error = 0;          // Error actual
float lastError = 0;      // Error anterior
int BASE_SPEED = 100;     // Velocidad base
int MAX_SPEED = 255;      // Velocidad máxima

//---------------------------------------------------------------------------------
// VALORES DE CALIBRACIÓN
//---------------------------------------------------------------------------------

// Umbrales calculados
int umbralS1 = 500;       // Umbral para sensor S1
int umbralS2 = 500;       // Umbral para sensor S2
int umbralS3 = 500;       // Umbral para sensor S3
int umbralS4 = 500;       // Umbral para sensor S4
int umbralSL = 500;       // Umbral para sensor SL
int umbralSR = 500;       // Umbral para sensor SR

// Lecturas calibradas para superficie blanca
int blanco1, blanco2, blanco3, blanco4, blancoL, blancoR;

// Lecturas calibradas para línea negra
int negro1, negro2, negro3, negro4, negroL, negroR;

//---------------------------------------------------------------------------------
// ESTADOS DEL ROBOT
//---------------------------------------------------------------------------------
#define ESPERA 0              // Esperando primera pulsación para calibrar fondo blanco
#define CALIBRADO_BLANCO 1    // Calibrado fondo blanco, esperando pulsación para calibrar línea negra
#define CALIBRADO_NEGRO 2     // Calibrado completo, esperando pulsación para iniciar
#define ACTIVO 3              // Robot en funcionamiento

byte estadoRobot = ESPERA;    // Estado inicial
bool buttonPressed = false;   // Estado del botón

//---------------------------------------------------------------------------------
// CONFIGURACIÓN INICIAL
//---------------------------------------------------------------------------------
void setup() {
  // Configurar pines
  configurarPines();
  
  // Iniciar comunicación serial
  Serial.begin(9600);
  
  // Mostrar mensaje de bienvenida
  mostrarInstrucciones();
  
  // Parpadeo inicial para indicar que el sistema está listo
  parpadearLED(3, 100);
}

//---------------------------------------------------------------------------------
// BUCLE PRINCIPAL
//---------------------------------------------------------------------------------
void loop() {
  // Gestionar pulsador
  gestionarBoton();

  // Actuar según el estado actual del robot
  switch (estadoRobot) {
    case ESPERA:
      // Esperando calibrar fondo blanco - parpadeo lento
      apagarMotores();
      digitalWrite(LED, (millis() / 500) % 2);
      break;
      
    case CALIBRADO_BLANCO:
      // Esperando calibrar línea negra - parpadeo medio
      apagarMotores();
      digitalWrite(LED, (millis() / 250) % 2);
      break;
      
    case CALIBRADO_NEGRO:
      // Calibrado completo, esperando inicio - parpadeo rápido
      apagarMotores();
      digitalWrite(LED, (millis() / 100) % 2);
      break;
      
    case ACTIVO:
      // Robot en funcionamiento
      seguirLinea();
      break;
  }
}

//---------------------------------------------------------------------------------
// FUNCIONES DE CONFIGURACIÓN
//---------------------------------------------------------------------------------

/**
 * Configura todos los pines necesarios para el funcionamiento del robot
 */
void configurarPines() {
  // Configurar pines de sensores como entradas
  pinMode(S1, INPUT);
  pinMode(S2, INPUT);
  pinMode(S3, INPUT);
  pinMode(S4, INPUT);
  pinMode(SL, INPUT);
  pinMode(SR, INPUT);
  
  // Configurar pulsador con resistencia pull-up interna
  pinMode(SWITCH, INPUT_PULLUP);
  
  // Configurar LED y motores como salidas
  pinMode(LED, OUTPUT);
  pinMode(MOTOR_RIGHT, OUTPUT);
  pinMode(MOTOR_LEFT, OUTPUT);
  
  // Asegurar que los motores estén apagados al inicio
  apagarMotores();
}

/**
 * Muestra las instrucciones iniciales en el monitor serie
 */
void mostrarInstrucciones() {
  Serial.println(F("==== ROBOT SEGUIDOR DE LINEAS ===="));
  Serial.println(F("Calibracion en 3 pasos:"));
  Serial.println(F("1. Primera pulsacion: Calibrar FONDO BLANCO"));
  Serial.println(F("2. Segunda pulsacion: Calibrar LINEA NEGRA"));
  Serial.println(F("3. Tercera pulsacion: INICIAR seguimiento"));
  Serial.println(F("==================================="));
}

//---------------------------------------------------------------------------------
// FUNCIONES DE CONTROL
//---------------------------------------------------------------------------------

/**
 * Gestiona el pulsador y cambia el estado del robot según la secuencia
 */
void gestionarBoton() {
  bool buttonState = digitalRead(SWITCH);
  
  // Detectar pulsación (flanco descendente)
  if (buttonState == LOW && !buttonPressed) {
    buttonPressed = true;
  }
  // Detectar liberación (flanco ascendente)
  else if (buttonState == HIGH && buttonPressed) {
    buttonPressed = false;
    
    // Cambiar estado según secuencia
    switch (estadoRobot) {
      case ESPERA:
        // Primera pulsación - Calibrar fondo blanco
        Serial.println(F("Calibrando FONDO BLANCO..."));
        calibrarFondoBlanco();
        estadoRobot = CALIBRADO_BLANCO;
        Serial.println(F("FONDO BLANCO calibrado. Presione boton para calibrar LINEA NEGRA"));
        break;
        
      case CALIBRADO_BLANCO:
        // Segunda pulsación - Calibrar línea negra
        Serial.println(F("Calibrando LINEA NEGRA..."));
        calibrarLineaNegra();
        calcularUmbrales();
        estadoRobot = CALIBRADO_NEGRO;
        Serial.println(F("LINEA NEGRA calibrada. Presione boton para INICIAR"));
        break;
        
      case CALIBRADO_NEGRO:
        // Tercera pulsación - Activar
        estadoRobot = ACTIVO;
        Serial.println(F("Robot ACTIVADO"));
        break;
        
      case ACTIVO:
        // Cuarta pulsación - Detener y volver a espera calibrado
        estadoRobot = CALIBRADO_NEGRO;
        apagarMotores();
        Serial.println(F("Robot DETENIDO. Presione boton para reactivar"));
        break;
    }
    
    delay(50); // anti-rebote
  }
}

/**
 * Apaga ambos motores
 */
void apagarMotores() {
  analogWrite(MOTOR_LEFT, 0);
  analogWrite(MOTOR_RIGHT, 0);
}

/**
 * Hace parpadear el LED un número específico de veces
 * @param veces Número de veces a parpadear
 * @param duracion Duración de cada parpadeo en ms
 */
void parpadearLED(byte veces, int duracion) {
  for (byte i = 0; i < veces; i++) {
    digitalWrite(LED, HIGH);
    delay(duracion);
    digitalWrite(LED, LOW);
    delay(duracion);
  }
}

/**
 * Espera un tiempo determinado mientras parpadea el LED
 * @param ms Tiempo a esperar en ms
 */
void esperar(int ms) {
  unsigned long inicio = millis();
  while (millis() - inicio < ms) {
    digitalWrite(LED, (millis() / 100) % 2);
    delay(10);
  }
}

//---------------------------------------------------------------------------------
// FUNCIONES DE CALIBRACIÓN
//---------------------------------------------------------------------------------

/**
 * Calibra el robot para el fondo blanco
 */
void calibrarFondoBlanco() {
  // Indicación visual de inicio de calibración
  parpadearLED(3, 50);
  
  // Tomar múltiples muestras y promediar
  byte muestras = 10;
  int suma1 = 0, suma2 = 0, suma3 = 0, suma4 = 0, sumaL = 0, sumaR = 0;
  
  for (byte i = 0; i < muestras; i++) {
    suma1 += analogRead(S1);
    suma2 += analogRead(S2);
    suma3 += analogRead(S3);
    suma4 += analogRead(S4);
    sumaL += analogRead(SL);
    sumaR += analogRead(SR);
    digitalWrite(LED, !digitalRead(LED)); // Parpadeo durante la medición
    delay(10);
  }
  
  // Calcular promedios
  blanco1 = suma1 / muestras;
  blanco2 = suma2 / muestras;
  blanco3 = suma3 / muestras;
  blanco4 = suma4 / muestras;
  blancoL = sumaL / muestras;
  blancoR = sumaR / muestras;
  
  // Mostrar valores leídos
  Serial.println(F("Valores FONDO BLANCO:"));
  Serial.print(F("S1: ")); Serial.print(blanco1);
  Serial.print(F(" | S2: ")); Serial.print(blanco2);
  Serial.print(F(" | S3: ")); Serial.print(blanco3);
  Serial.print(F(" | S4: ")); Serial.println(blanco4);
  Serial.print(F("SL: ")); Serial.print(blancoL);
  Serial.print(F(" | SR: ")); Serial.println(blancoR);
}

/**
 * Calibra el robot para la línea negra
 */
void calibrarLineaNegra() {
  // Indicación visual de inicio de calibración
  parpadearLED(3, 50);
  
  // Tomar múltiples muestras y promediar
  byte muestras = 10;
  int suma1 = 0, suma2 = 0, suma3 = 0, suma4 = 0, sumaL = 0, sumaR = 0;
  
  for (byte i = 0; i < muestras; i++) {
    suma1 += analogRead(S1);
    suma2 += analogRead(S2);
    suma3 += analogRead(S3);
    suma4 += analogRead(S4);
    sumaL += analogRead(SL);
    sumaR += analogRead(SR);
    digitalWrite(LED, !digitalRead(LED)); // Parpadeo durante la medición
    delay(10);
  }
  
  // Calcular promedios
  negro1 = suma1 / muestras;
  negro2 = suma2 / muestras;
  negro3 = suma3 / muestras;
  negro4 = suma4 / muestras;
  negroL = sumaL / muestras;
  negroR = sumaR / muestras;
  
  // Mostrar valores leídos
  Serial.println(F("Valores LINEA NEGRA:"));
  Serial.print(F("S1: ")); Serial.print(negro1);
  Serial.print(F(" | S2: ")); Serial.print(negro2);
  Serial.print(F(" | S3: ")); Serial.print(negro3);
  Serial.print(F(" | S4: ")); Serial.println(negro4);
  Serial.print(F("SL: ")); Serial.print(negroL);
  Serial.print(F(" | SR: ")); Serial.println(negroR);
}

/**
 * Calcula los umbrales óptimos basados en las lecturas de calibración
 */
void calcularUmbrales() {
  // Calcular umbrales como punto medio entre blanco y negro
  umbralS1 = (blanco1 + negro1) / 2;
  umbralS2 = (blanco2 + negro2) / 2;
  umbralS3 = (blanco3 + negro3) / 2;
  umbralS4 = (blanco4 + negro4) / 2;
  umbralSL = (blancoL + negroL) / 2;
  umbralSR = (blancoR + negroR) / 2;
  
  // Determinar para cada sensor si el negro da valores más altos o más bajos que el blanco
  bool negroMasAltoS1 = (negro1 > blanco1);
  bool negroMasAltoS2 = (negro2 > blanco2);
  bool negroMasAltoS3 = (negro3 > blanco3);
  bool negroMasAltoS4 = (negro4 > blanco4);
  bool negroMasAltoSL = (negroL > blancoL);
  bool negroMasAltoSR = (negroR > blancoR);
  
  // Mostrar umbrales y comportamiento de sensores
  Serial.println(F("\nUmbrales calculados:"));
  Serial.print(F("S1: "));
  Serial.print(umbralS1);
  Serial.print(negroMasAltoS1 ? F(" (Negro>Blanco)") : F(" (Blanco>Negro)"));
  
  Serial.print(F(" | S2: "));
  Serial.print(umbralS2);
  Serial.print(negroMasAltoS2 ? F(" (Negro>Blanco)") : F(" (Blanco>Negro)"));
  
  Serial.print(F(" | S3: "));
  Serial.print(umbralS3);
  Serial.print(negroMasAltoS3 ? F(" (Negro>Blanco)") : F(" (Blanco>Negro)"));
  
  Serial.print(F(" | S4: "));
  Serial.print(umbralS4);
  Serial.println(negroMasAltoS4 ? F(" (Negro>Blanco)") : F(" (Blanco>Negro)"));
  
  Serial.print(F("SL: "));
  Serial.print(umbralSL);
  Serial.print(negroMasAltoSL ? F(" (Negro>Blanco)") : F(" (Blanco>Negro)"));
  
  Serial.print(F(" | SR: "));
  Serial.print(umbralSR);
  Serial.println(negroMasAltoSR ? F(" (Negro>Blanco)") : F(" (Blanco>Negro)"));
}

//---------------------------------------------------------------------------------
// FUNCIONES DE SEGUIMIENTO DE LÍNEA
//---------------------------------------------------------------------------------

/**
 * Función principal de seguimiento de línea
 */
void seguirLinea() {
  digitalWrite(LED, HIGH);  // LED encendido durante operación normal

  // Leer valores analógicos de los sensores
  int valorS1 = analogRead(S1);
  int valorS2 = analogRead(S2);
  int valorS3 = analogRead(S3);
  int valorS4 = analogRead(S4);
  int valorSL = analogRead(SL);
  int valorSR = analogRead(SR);
  
  // Verificar si se detecta superficie
  if (verificarAusenciaSuperficie(valorS1, valorS2, valorS3, valorS4, valorSL, valorSR)) {
    return;  // Salir si no se detecta superficie
  }
  
  // Determinar si cada sensor está detectando línea negra
  // La comparación es dinámica según los valores calibrados
  byte s1 = (valorS1 > umbralS1) == (negro1 > blanco1) ? 1 : 0;
  byte s2 = (valorS2 > umbralS2) == (negro2 > blanco2) ? 1 : 0;
  byte s3 = (valorS3 > umbralS3) == (negro3 > blanco3) ? 1 : 0;
  byte s4 = (valorS4 > umbralS4) == (negro4 > blanco4) ? 1 : 0;
  byte sl = (valorSL > umbralSL) == (negroL > blancoL) ? 1 : 0;
  byte sr = (valorSR > umbralSR) == (negroR > blancoR) ? 1 : 0;

  // Verificar si algún sensor detecta línea negra
  bool detectaLineaNegra = (s1 || s2 || s3 || s4 || sl || sr);
  
  // Mostrar estado de los sensores cada 100 iteraciones
  static int contador = 0;
  if (++contador >= 100) {
    contador = 0;
    Serial.print(F("Sensores: "));
    Serial.print(s1); Serial.print(s2); Serial.print(s3); Serial.print(s4);
    Serial.print(F(" SL:")); Serial.print(sl);
    Serial.print(F(" SR:")); Serial.println(sr);
  }
  
  // Si no detecta línea negra, detener
  if (!detectaLineaNegra) {
    apagarMotores();
    digitalWrite(LED, LOW);
    Serial.println(F("LINEA NEGRA PERDIDA - Robot detenido"));
    // Parpadear LED rápidamente para indicar detección
    parpadearLED(5, 100);
    return;
  }
  
  // Procesa los datos para seguir la línea
  procesarSeguimientoLinea(s1, s2, s3, s4, sl, sr);
}

/**
 * Verifica si hay ausencia de superficie debajo del robot
 * @return true si no se detecta superficie, false en caso contrario
 */
bool verificarAusenciaSuperficie(int valorS1, int valorS2, int valorS3, int valorS4, int valorSL, int valorSR) {
  // Definir umbrales para "no superficie" con un margen del 20%
  int margen = 20; // en porcentaje
  bool noHaySuperficie = false;
  
  // Verificar cada sensor para valores erroneos
  if (valorS1 < min(blanco1, negro1) - (abs(blanco1 - negro1) * margen / 100) || 
      valorS1 > max(blanco1, negro1) + (abs(blanco1 - negro1) * margen / 100)) {
    noHaySuperficie = true;
  }
  
  if (valorS2 < min(blanco2, negro2) - (abs(blanco2 - negro2) * margen / 100) || 
      valorS2 > max(blanco2, negro2) + (abs(blanco2 - negro2) * margen / 100)) {
    noHaySuperficie = true;
  }
  
  if (valorS3 < min(blanco3, negro3) - (abs(blanco3 - negro3) * margen / 100) || 
      valorS3 > max(blanco3, negro3) + (abs(blanco3 - negro3) * margen / 100)) {
    noHaySuperficie = true;
  }
  
  if (valorS4 < min(blanco4, negro4) - (abs(blanco4 - negro4) * margen / 100) || 
      valorS4 > max(blanco4, negro4) + (abs(blanco4 - negro4) * margen / 100)) {
    noHaySuperficie = true;
  }
  
  if (valorSL < min(blancoL, negroL) - (abs(blancoL - negroL) * margen / 100) || 
      valorSL > max(blancoL, negroL) + (abs(blancoL - negroL) * margen / 100)) {
    noHaySuperficie = true;
  }
  
  if (valorSR < min(blancoR, negroR) - (abs(blancoR - negroR) * margen / 100) || 
      valorSR > max(blancoR, negroR) + (abs(blancoR - negroR) * margen / 100)) {
    noHaySuperficie = true;
  }
  
  // Si no hay superficie, alertar y detener
  if (noHaySuperficie) {
    apagarMotores();
    
    // Parpadeo - tres parpadeos muy rápidos
    parpadearLED(3, 50);
    
    Serial.println(F("ALERTA! No se detecta superficie - Robot detenido"));
    return true;
  }
  
  return false;
}

/**
 * Procesa los datos de los sensores y controla los motores para seguir la línea
 */
void procesarSeguimientoLinea(byte s1, byte s2, byte s3, byte s4, byte sl, byte sr) {
  // Cálculo del error para línea negra
  error = (s1 * -3) + (s2 * -1) + (s3 * 1) + (s4 * 3);
  
  // Ajuste con sensores laterales
  if (sl) error -= 5;  // SL produce error negativo (giro a la izquierda) 
  if (sr) error += 5;  // SR produce error positivo (giro a la derecha)

  // Control PD
  float P = error;
  float D = error - lastError;
  lastError = error;

  float correction = (Kp * P) + (Kd * D);
  
  // Ajustar velocidad de motores según la corrección calculada
  int motorSpeedLeft = constrain(BASE_SPEED - correction, 0, MAX_SPEED);
  int motorSpeedRight = constrain(BASE_SPEED + correction, 0, MAX_SPEED);
  
  // Control para curvas cerradas detectadas con sensores laterales
  if (sl && !s1 && !s2) {
    // Curva cerrada izquierda
    motorSpeedLeft = MAX_SPEED;
    motorSpeedRight = 0;
  }
  
  if (sr && !s3 && !s4) {
    // Curva cerrada derecha
    motorSpeedLeft = 0;
    motorSpeedRight = MAX_SPEED;
  }

  // Aplicar velocidades a los motores
  analogWrite(MOTOR_LEFT, motorSpeedLeft);
  analogWrite(MOTOR_RIGHT, motorSpeedRight);
  
  delay(10); // pausa para estabilidad
}