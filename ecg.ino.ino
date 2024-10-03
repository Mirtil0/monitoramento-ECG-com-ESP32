#include <TFT_eSPI.h>

#define pinLeitura 34 // Pino de leitura do ECG

// Instância do display
TFT_eSPI tft = TFT_eSPI();

unsigned long tempo = 0;
int tAmostra = 5; // Tempo de amostragem
bool atualiza = 1;

// Buffers para os filtros
float lowPassBuffer[9] = {0}; // Buffer para o filtro passa-baixa (x[n], x[n-1], ..., x[n-10], y[n-1], y[n-2])
float highPassBuffer[33] = {0}; // Buffer para o filtro passa-alta

// Variáveis de controle
float previousOutput = 0; // Para o filtro passa-alta

void setup() {
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  Serial.begin(9600);
  tempo = millis();
}

void loop() {
  if (millis() - tempo >= tAmostra) {
    tempo = millis();
    int leitura = analogRead(pinLeitura);

    float ecg = (float)leitura;  // Sinal de entrada
    processECGSignal(ecg);
    // Aplicar os filtros no sinal de ECG
    //float ecgSignal = lowPassFilter(ecg);  // Aplicar o filtro passa-baixa
    //ecgSignal = highPassFilter(ecgSignal);  // Aplicar o filtro passa-alta

    // Atualiza gráfico a cada dois ciclos
    atualiza = !atualiza;
    if (atualiza) {
      //int y = map((int)ecgSignal, 0, 4095, 0, 128);
      //attGraf(y);
    }

    // Enviar o sinal processado pela Serial
    //Serial.print(ecg);
    //Serial.print(",");
    //Serial.println(ecgS);

  }

  delay(1);
}

//########################## FILTRO PASSA-BAIXA ##########################
float lowPassFilter(float input, float *lowPassBuffer) {
  // Atualiza o buffer de entradas (x[n], ..., x[n-10])
  for (int i = 8; i > 0; i--) {
    lowPassBuffer[i] = lowPassBuffer[i - 1];
  }
  lowPassBuffer[0] = input; // Nova entrada

  // Calcula a saída do filtro (baseado na função de transferência)
  float result = lowPassBuffer[0] + 2 * lowPassBuffer[1] + 3 * lowPassBuffer[2] +
                 4 * lowPassBuffer[3] + 5 * lowPassBuffer[4] + 4 * lowPassBuffer[5] +
                 3 * lowPassBuffer[6] + 2 * lowPassBuffer[7] + lowPassBuffer[8];

  return result / 32.0;
}

//########################## FILTRO PASSA-ALTA ##########################
float highPassFilter(float input, float *highPassBuffer) {
  // Atualiza o buffer de entradas (x[n], ..., x[n-32])
  for (int i = 32; i > 0; i--) {
    highPassBuffer[i] = highPassBuffer[i - 1];
  }
  highPassBuffer[0] = input;  // Nova entrada

  // Calcula a saída do filtro baseado na função de transferência
  float result = (-(1 / 32.0) * highPassBuffer[0]) + highPassBuffer[16] - highPassBuffer[17]
                 + (highPassBuffer[32] / 32.0) + previousOutput;


  previousOutput = result;  // Atualiza y[n-1]

  return result;  // Saída do filtro
}

float derivativeFilter(float input, float *derivBuffer) {
  // Usando a função de transferência sugerida: 0.1(-z^-2 - 2z^-1 + 2z^1 + z^2)
  derivBuffer[0] = input;
  float result = (derivBuffer[0] + 2 * derivBuffer[1] -
                  2 * derivBuffer[3] - derivBuffer[4]) / 10;

  // Atualiza buffer
  for (int i = 4; i > 0; i--) {
    derivBuffer[i] = derivBuffer[i - 1];
  }

  return result;
}

float squareSignal(float input) {
  return input * input;
}
float movingAverage(float input, float *avgBuffer, int windowSize) {
  float sum = 0;
  for (int i = 0; i < windowSize; i++) {
    sum += avgBuffer[i];
  }

  // Atualiza buffer
  for (int i = windowSize - 1; i > 0; i--) {
    avgBuffer[i] = avgBuffer[i - 1];
  }
  avgBuffer[0] = input;

  return sum / windowSize;
}
bool detectQRS(float integratedSignal, float *signalLevel, float *noiseLevel, float *threshold) {
  // Atualizar os níveis de sinal e ruído
  if (integratedSignal > *threshold) {
    *signalLevel = 0.125 * integratedSignal + 0.875 * (*signalLevel);
    return true; // Pico detectado
  } else {
    *noiseLevel = 0.125 * integratedSignal + 0.875 * (*noiseLevel);
    return false; // Não é QRS
  }

  // Atualizar o limiar
  *threshold = *noiseLevel + 0.25 * (*signalLevel - *noiseLevel);
}
void processECGSignal(float ecgSignal) {
  static float lowPassBuffer[9] = {0};
  static float highPassBuffer[33] = {0};
  static float derivBuffer[5] = {0};
  static float avgBuffer[30] = {0};  // Janela de 150 ms para 200 Hz = 30 amostras

  static float signalLevel = 0;
  static float noiseLevel = 0;
  static float threshold = 0;

  // Aplicar filtros em sequência
  float filteredSignal = lowPassFilter(ecgSignal, lowPassBuffer);
  filteredSignal = highPassFilter(filteredSignal, highPassBuffer);
  float derivSignal = derivativeFilter(filteredSignal, derivBuffer);
  float squaredSignal = squareSignal(derivSignal);
  float integratedSignal = movingAverage(squaredSignal, avgBuffer, 30);

  // Detectar QRS
  bool qrsDetected = detectQRS(integratedSignal, &signalLevel, &noiseLevel, &threshold);
  Serial.print(ecgSignal);
  Serial.print(",");
  Serial.println(integratedSignal);
  if (qrsDetected) {
    //Serial.println("QRS Detected!");
  }
}

//########################## ATUALIZA O DISPLAY ##########################
void attGraf(int y) {
  static int x = 0;
  static int lastX = 0;
  static int lastY = 64;

  // Limpa a linha anterior e desenha a nova
  tft.drawLine(x, 0, x, 128, TFT_BLACK);
  tft.drawLine(lastX, lastY, x, y, TFT_GREEN);

  lastX = x;
  lastY = y;
  x++;

  // Se chegou ao final da tela, reinicia
  if (x >= 128) {
    x = 0;
    lastY = 64;
    lastX = 0;
  }
}
