float SignalLevelI = 0;  // Nível de sinal para o sinal integrado
float NoiseLevelI = 0;   // Nível de ruído para o sinal integrado
float ThresholdI = 50;    // Threshold para o sinal integrado

float SignalLevelF = 0;  // Nível de sinal para o sinal filtrado
float NoiseLevelF = 0;   // Nível de ruído para o sinal filtrado
float ThresholdF = 50;    // Threshold para o sinal filtrado

bool PEAKI = false;  // Pico no sinal integrado
bool PEAKF = false;  // Pico no sinal filtrado

const float alpha = 0.125;  // Constante de suavização


void setup() {
  Serial.begin(9600);
  pinMode(10, INPUT); // Configuração para detecção de derivações LO +
  pinMode(11, INPUT); // Configuração para detecção de leads off LO -
}

void loop() {
  static unsigned long tempo = millis();
  static float lowBuffer[9] = {0};
  static float highBuffer[33] = {0};
  static float derivBuffer[5] = {0};
  static float avBuffer[31] = {0}; //janela de 150 ms

  if (millis() - tempo >= 5) {//taxa de amostragem de 200Hz
    tempo = millis();
    int leitura = analogRead(A0);
    //Serial.print(leitura);
   // Serial.print(" ");
    float saida = lowPass(leitura, lowBuffer);
    saida = highPass(saida, highBuffer);
    saida = deriva(saida, derivBuffer);//já retorna elevado ao quadrado
    PEAKF = detectRPeak(saida, ThresholdF);
    saida = avFilter(saida, avBuffer);//filtro de media móvel
    PEAKI = detectRPeak(saida, ThresholdI);

    bool isQRS = classifyPeak(PEAKI, PEAKF);

    updateLevels(PEAKI, PEAKF, isQRS);

    if (isQRS) {
      Serial.println("QRS Detectado");
      // Calcular BPM e outras métricas
    }

   // Serial.print(saida);
    //Serial.print(" ");
   // Serial.println(ThresholdI);

  }
}

float lowPass(int input, float *lowBuffer) {
  for (int i = 8; i > 0; i-- ) lowBuffer[i] = lowBuffer[i - 1];
  lowBuffer[0] = input;

  float saida = lowBuffer[0] + 2 * lowBuffer[1] + 3 * lowBuffer[2] + 4 * lowBuffer[3] + 5 * lowBuffer[4] +
                4 * lowBuffer[5] + 3 * lowBuffer[6] + 2 * lowBuffer[7] + lowBuffer[8];

  return saida / 32.0;

}

float highPass(float input, float *highBuffer) {
  static float lastOut = 0;

  for (int i = 32; i > 0; i-- ) highBuffer[i] = highBuffer[i - 1];

  highBuffer[0] = input;

  float saida = -highBuffer[0] / 32.0 + highBuffer[16] - highBuffer[17] +  highBuffer[32] / 32.0 + lastOut;
  lastOut = saida;
  return saida;
}

float deriva(float input, float *derivBuffer) {

  for (int i = 4; i > 0; i-- ) derivBuffer[i] = derivBuffer[i - 1];

  derivBuffer[0] = input;

  float saida = -derivBuffer[4] - 2 * derivBuffer[3] + 2 * derivBuffer[1] + derivBuffer[0];

  return (saida * saida) / 100.0;

}

float avFilter(float input, float *avBuffer) {

  for (int i = 30; i > 0; i-- ) avBuffer[i] = avBuffer[i - 1];

  avBuffer[0] = input;
  float sum = 0;
  for (int i = 0; i <= 30; i++ ) {
    sum += avBuffer[i];
  }
  return sum / 30.0;

}

// Detecção de picos R
bool detectRPeak(float input, float threshold) {
  static float lastValue = 0;
  static bool peakDetected = false;

  if (input > threshold && lastValue < threshold && !peakDetected) {
    peakDetected = true;
    return true;  // Pico detectado
  } else if (input < threshold) {
    peakDetected = false;
  }

  lastValue = input;
  return false;
}

void calcLevels(float maxSignal, float avgNoise) {
  // Durante a fase de aprendizado (por exemplo, nos primeiros 2 segundos)
  SignalLevelI = 0.875 * maxSignal;  // Inicializar o nível de sinal com 75% do máximo
  NoiseLevelI = 0.125 * avgNoise;    // Inicializar o nível de ruído com 25% da média do ruído
  ThresholdI = NoiseLevelI + 0.125 * (SignalLevelI - NoiseLevelI);

  SignalLevelF = 0.875 * maxSignal;  // Para o sinal filtrado também
  NoiseLevelF = 0.125 * avgNoise;
  ThresholdF = NoiseLevelF + 0.125 * (SignalLevelF - NoiseLevelF);
}

void updateLevels(float peakI, float peakF, bool isSignalPeak) {
  if (isSignalPeak) {
    // Atualizar o nível de sinal com base no novo pico detectado
    SignalLevelI = alpha * peakI + (1 - alpha) * SignalLevelI;
    SignalLevelF = alpha * peakF + (1 - alpha) * SignalLevelF;
  } else {
    // Atualizar o nível de ruído se for classificado como ruído
    NoiseLevelI = alpha * peakI + (1 - alpha) * NoiseLevelI;
    NoiseLevelF = alpha * peakF + (1 - alpha) * NoiseLevelF;
  }

  // Atualizar os thresholds
  ThresholdI = NoiseLevelI + 0.125 * (SignalLevelI - NoiseLevelI);
  ThresholdF = NoiseLevelF + 0.125 * (SignalLevelF - NoiseLevelF);
}

bool classifyPeak(float peakI, float peakF) {
  // Verificar se o pico no sinal integrado está acima do threshold
  if (peakI > ThresholdI) {
    // Verificar também se o pico correspondente no sinal filtrado está acima do threshold filtrado
    if (peakF > ThresholdF) {
      return true;  // Pico de sinal (QRS detectado)
    }
  }
  return false;  // Pico de ruído
}
