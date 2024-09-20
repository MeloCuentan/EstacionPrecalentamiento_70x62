#include <Arduino.h>
#include <Wire.h>
#include <PID_v1.h>           // https://github.com/br3ttb/Arduino-PID-Library/tree/master
#include "sonda3950.h"        // https://github.com/MeloCuentan/termistor3950/
#include <RotaryEncoder.h>    // https://github.com/mathertel/RotaryEncoder
#include <Adafruit_SH110X.h>  // https://github.com/adafruit/Adafruit_SH110x/blob/master/Adafruit_SH110X.h
#include <Adafruit_GFX.h>     // https://github.com/adafruit/Adafruit-GFX-Library

// Valores de los pines utilizados
const uint8_t pinTermistor = A3;
const uint8_t pinSSR = 9;
const uint8_t pinEN1 = 3;
const uint8_t pinEN2 = 2;
const uint8_t pinBoton = 4;

// Valores del PID
double Setpoint = 0.0;
double Input = 0.0;
double Output = 0.0;

const double Kp = 1.35;
const double Ki = 0.02;
const double Kd = 2.0;

// Valores del display OLED
const uint8_t SCREEN_WIDTH = 128;
const uint8_t SCREEN_HEIGHT = 64;
const uint8_t SCREEN_ADDRESS = 0x3C;
const uint8_t TEXT_SIZE = 1;

// Valores generales de funcionamiento
const uint8_t temperaturaMinima = 50;
const uint16_t temperaturaMaxima = 250;
const uint16_t adcResolucion = 1023;
const uint16_t rPullUp = 4700;
const uint32_t rTermistor = 100000;
const float temperaturaInicio = 150.0;
uint32_t tiempoActual;

// Variables generales
bool estadoCalentador = false;
float valorSonda = 0;
uint16_t valorEncoder = 1;
uint16_t valorEncoderAnterior = 0;
bool lecturaBoton = false;

// Variables para generar PWM por software a baja frecuencia
const uint16_t frecuenciaPWM = 10;
const uint16_t duracionCiclo = 1000 / frecuenciaPWM;
uint16_t valorPWM = 0;

// Creamos objetos
sonda3950 termistor(pinTermistor, adcResolucion, rPullUp, rTermistor);
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
RotaryEncoder encoder(pinEN1, pinEN2, RotaryEncoder::LatchMode::FOUR3);
Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT);

// Declaramos funciones
void iniciarOLED();
void controlPWM(uint8_t duty);
void PID_Control();
void controlLimitesPID();
void pantallaInicio();
void medirTemperatura();
void comprobarEncoder();
String convertirNumeroAString(float numero, bool decimal);
void mostrarDisplay();

void setup() {
  pinMode(pinBoton, INPUT_PULLUP);
  pinMode(pinSSR, OUTPUT);
  Setpoint = temperaturaInicio;
  valorEncoder = Setpoint;

  analogReadResolution(10);

  encoder.setPosition(temperaturaInicio);
  myPID.SetMode(AUTOMATIC);

  termistor.begin();

  iniciarOLED();

  pantallaInicio();
}

void loop() {
  tiempoActual = millis();
  termistor.actualizar();
  comprobarEncoder();
  medirTemperatura();
  controlLimitesPID();
  PID_Control();
  mostrarDisplay();
}

void iniciarOLED() {
  display.begin(SCREEN_ADDRESS);
  display.clearDisplay();
  display.setRotation(0);
  display.setTextColor(SH110X_WHITE, SH110X_BLACK);
  display.setTextSize(TEXT_SIZE);
  display.setTextWrap(false);
  display.display();
}

void controlPWM(uint8_t duty) {
  static uint32_t tiempoAnterior;
  uint32_t onTime = (duracionCiclo * duty) / 255;
  uint32_t offTime = duracionCiclo - onTime;

  if (digitalRead(pinSSR) == HIGH && tiempoActual - tiempoAnterior >= onTime) {
    tiempoAnterior = tiempoActual;
    digitalWrite(pinSSR, LOW);
  } else if (digitalRead(pinSSR) == LOW && tiempoActual - tiempoAnterior >= offTime) {
    tiempoAnterior = tiempoActual;
    digitalWrite(pinSSR, HIGH);
  }
}

void PID_Control() {
  Input = valorSonda;
  myPID.Compute();

  if (estadoCalentador == true) {
    controlPWM(uint8_t(Output));
  } else {
    controlPWM(0);
    Output = 0;
  }
}

void controlLimitesPID() {
  int16_t diferencia = abs(int(Setpoint) - int(valorSonda));
  if (diferencia < 10) {
    myPID.SetOutputLimits(0, 15);
  } else if (diferencia < 50) {
    myPID.SetOutputLimits(0, 20);
  } else if (diferencia < 100) {
    myPID.SetOutputLimits(0, 30);
  } else {
    myPID.SetOutputLimits(0, 50);
  }
}

void pantallaInicio() {
  display.setCursor(13, 8);
  display.print("PLACA CALEFACTORA");
  display.setCursor(0, 24);
  display.print("POWER  -->");
  display.setCursor(0, 40);
  display.print("SET TEMP:");
  display.setCursor(0, 56);
  display.print("REAL TEMP:");
  display.drawCircle(96, 41, 2, SH110X_WHITE);
  display.drawCircle(108, 57, 2, SH110X_WHITE);
  display.setCursor(100, 40);
  display.print("C");
  display.setCursor(112, 56);
  display.print("C");
}

void medirTemperatura() {
  valorSonda = termistor.temperaturaLeida();
}

void comprobarEncoder() {
  static uint32_t tiempoDebounce, intervaloDebounce = 250;
  encoder.tick();

  valorEncoder = encoder.getPosition();
  if (valorEncoder < temperaturaMinima) {
    encoder.setPosition(temperaturaMinima);
    valorEncoder = temperaturaMinima;
  } else if (valorEncoder > temperaturaMaxima) {
    encoder.setPosition(temperaturaMaxima);
    valorEncoder = temperaturaMaxima;
  }

  if (valorEncoder != valorEncoderAnterior) {
    valorEncoderAnterior = valorEncoder;
    Setpoint = float(valorEncoder);
  }

  if (digitalRead(pinBoton) == LOW && lecturaBoton == true) {
    lecturaBoton = false;
    estadoCalentador = !estadoCalentador;
    tiempoDebounce = tiempoActual;
  }

  if (tiempoActual - tiempoDebounce >= intervaloDebounce && digitalRead(pinBoton) == HIGH) {
    lecturaBoton = true;
  }
}

String convertirNumeroAString(float numero, bool decimal) {
  String resultado = "";
  if (numero < 0) {
    resultado += "-";
    numero = -numero;
  } else {
    resultado += " ";
  }
  int parteEntera = (int)numero;
  if (parteEntera < 100) {
    resultado += " ";
  }
  resultado += String(parteEntera);
  if (decimal == true) {
    resultado += ".";
    int parteDecimal = (int)(numero * 10) % 10;
    resultado += String(parteDecimal);
  }
  return resultado;
}

void mostrarDisplay() {
  static uint32_t tiempoAnterior, intervalo = 250;

  if (tiempoActual - tiempoAnterior >= intervalo) {
    tiempoAnterior = tiempoActual;
    display.setCursor(72, 24);
    if (estadoCalentador)
      display.print("ON ");
    else
      display.print("OFF");

    display.setCursor(66, 40);
    display.print(convertirNumeroAString(Setpoint, false));

    display.setCursor(66, 56);
    display.print(convertirNumeroAString(valorSonda, true));

    display.display();
  }
}
