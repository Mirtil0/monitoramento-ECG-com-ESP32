#include "BluetoothSerial.h" // Biblioteca para comunicação Bluetooth

#include <TFT_eSPI.h>
#include <SPI.h>
#include <Wire.h>

TFT_eSPI tft = TFT_eSPI();

// Constantes para o gráfico
#define GRAPH_X 0               // Posição inicial no eixo X
#define GRAPH_Y 240             // Posição inicial no eixo Y (inferior da tela)
#define GRAPH_WIDTH 320         // Largura do gráfico (largura da tela)
#define GRAPH_HEIGHT 125        // Altura do gráfico

#define TFT_DC 12 //A0
#define TFT_CS 13 //CS display
#define TFT_MOSI 14 //SDA
#define TFT_CLK 27 //SCK
#define TFT_RST 0 //direto no vcc
#define TFT_MISO 0 //direto no vcc

BluetoothSerial SerialBT;    // Instância de comunicação Bluetooth

// Define as variáveis de entrada e saída
int entrada = 15; // Sinal de entrada
int LP = 2;      // LO+
int LN = 4;     // LO-
int botao = 18;

// Estrutura para armazenar os dados calculados
struct DadosECG {
  bool pico;
  float filtrado;
  int BPM;
  float leitura;
  unsigned long tempoRelativo;
  bool LOP;
  bool LON;
};

DadosECG dadosECG;                // Instância da estrutura para os dados calculados

DadosECG bufferEnvio[5];
int indiceBuffer = 0;

bool dadosProntos = false;  // Flag para sincronização entre cores

float SignalLevelI = 0;
float NoiseLevelI = 0;
float ThresholdI = 50;
float SignalLevelF = 0;
float NoiseLevelF = 0;
float ThresholdF = 50;

bool LOP = 0, LON = 0;
unsigned long tempo = 0;
unsigned long tempo1 = 0;
unsigned long tZero = 0;
unsigned long lastTime = 0;

float filtrado = 0;
const double resolucao = (3.3 / 4095); //transforma o valor lido para mV
int cont = 0;
float leitura = 0;

bool isQRS = false;
bool PEAKI = false;

static float bandPassBuffer[20] = {0};

static int BPM = 60;

String coleta; // Variável global para armazenar mensagens Bluetooth

bool viaSerial = false;

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ECG_Device"); // Inicializa Bluetooth com o nome "ECG_Device"
  Serial.println("pode mandar");

  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);

  desenharBluetooth(20, 20, 10, TFT_BLUE); // Coordenadas, tamanho, cor
  desenharCoracao(160, 50, 30, TFT_RED);
  exibirBPM(BPM, TFT_WHITE, TFT_RED, 143, 33);

  desenharGrade();

  // Continua com o restante do setup
  //while (lerBluetooth() != "iniciado") delay(1); // Espera até que o sinal de inicialização seja recebido
  //SerialBT.println("HAHAHA");

  pinMode(LP, INPUT);
  pinMode(LN, INPUT);
  pinMode(botao, INPUT_PULLUP);
  tempo = millis();

  // Cria uma tarefa para comunicação Bluetooth no core 0
  xTaskCreatePinnedToCore(taskBluetooth, "BluetoothTask", 4096, NULL, 1, NULL, 0);
}

// Função principal
void loop() {
  static float lowBuffer[9] = {0};
  static float highBuffer[33] = {0};
  static float derivBuffer[5] = {0};
  static float avBuffer[31] = {0};
  static float peakBuffer[5] = {0};

  static const int peakSize = sizeof(peakBuffer) / sizeof(peakBuffer[0]);
  static const int bandPassSize = sizeof(bandPassBuffer) / sizeof(bandPassBuffer[0]);
  static const int lowSize = sizeof(lowBuffer) / sizeof(lowBuffer[0]);
  static const int highSize = sizeof(highBuffer) / sizeof(highBuffer[0]);
  static const int derivSize = sizeof(derivBuffer) / sizeof(derivBuffer[0]);
  static const int avSize = sizeof(avBuffer) / sizeof(avBuffer[0]);

  static int txAmostra = 5;
  static bool ler = 0;
  unsigned long agora = millis();

  if (coleta == "reset") ESP.restart();

  if (coleta == "begin") {
    ler = 1;
    tZero = agora;
    viaSerial = true;
  } else if (coleta == "finish") {
    ler = 0;
    viaSerial = false;
  }

  if (!digitalRead(botao) && !viaSerial) {
    ler = !ler;
    indiceBuffer = 0;
    while (!digitalRead(botao));
    delay(10);
  }

  if (ler) {
    if (agora - tempo1 >= 1) {
      tempo1 = agora;
      //leitura += analogRead(entrada); // Acumula a leitura
      //cont++;
    }
    if (agora - tempo >= txAmostra && !dadosProntos) {
      tempo = agora;
      leitura = analogRead(entrada) * resolucao; // Calcula a média das leituras

      float saida = lowPass(leitura, lowBuffer, lowSize);
      saida = highPass(saida, highBuffer, highSize);
      attBuffer(saida, bandPassBuffer, bandPassSize);
      saida = deriva(saida, derivBuffer, derivSize);
      saida = avFilter(saida, avBuffer, avSize);
      attBuffer(saida, peakBuffer, peakSize);

      isQRS = false;
      PEAKI = false;

      if (agora - lastTime >= 200) {
        PEAKI = detectRPeak(peakBuffer, peakSize);
        isQRS = classifyPeak(PEAKI, derivBuffer[1], peakBuffer[1]);
        updateLevels(derivBuffer[1], peakBuffer[1], isQRS, PEAKI);
      }

      if (isQRS) {
        BPM = 60000 / (agora - lastTime);
        lastTime = agora;
      }

      dadosECG = {isQRS, bandPassBuffer[5], BPM, leitura, tempo - tZero,
                  digitalRead(LP), digitalRead(LN)
                 };
      dadosProntos = true;  // Sinaliza que os dados estão prontos
      leitura = 0;
      cont = 0;
    }
  }
}

void taskBluetooth(void *parameter) {
  while (true) {
    coleta = lerBluetooth();

    if (dadosProntos) {
      // Armazena a amostra atual no buffer e incrementa o índice
      bufferEnvio[indiceBuffer] = dadosECG;
      indiceBuffer++;
      dadosProntos = false;  // Reseta a flag após armazenar os dados no buffer

      // Verifica se é hora de enviar as amostras (25 ms) ou se o buffer está cheio (5 amostras)
      if (indiceBuffer >= 5) {
        // Monta uma string para acumular todos os dados das 5 amostras
        String pacote = "";
        for (int i = 0; i < indiceBuffer; i++) {
          plotGraph(bufferEnvio[i].leitura); // Chama a função de plotar
          if (viaSerial) {
            pacote += String(bufferEnvio[i].leitura, 2) + "," +
                      String(bufferEnvio[i].filtrado, 2) + "," +
                      String(bufferEnvio[i].tempoRelativo) + "," +
                      String(bufferEnvio[i].LOP) + "," +
                      String(bufferEnvio[i].LON) + "," +
                      String(bufferEnvio[i].BPM) + "," +
                      String(bufferEnvio[i].pico);
            if (i < indiceBuffer - 1) {
              pacote += ";";
            }
          }
          if (bufferEnvio[i].pico)exibirBPM(bufferEnvio[i].BPM, TFT_WHITE, TFT_RED, 143, 33);

          // Adiciona o ";" somente se não for a última amostra

        }
        if (viaSerial) {
          SerialBT.println(pacote);  // Envia o pacote completo
        }
        indiceBuffer = 0;          // Reseta o índice do buffer

      }
    }
    delay(1);
  }
}

// Função de leitura Bluetooth
String lerBluetooth() {
  if (SerialBT.available()) {
    return SerialBT.readStringUntil('\n');
  }
  return "";
}

// Funções de filtragem e processamento
float lowPass(float input, float *lowBuffer, int tamanho) {
  attBuffer(input, lowBuffer, tamanho);
  float saida = lowBuffer[0] + 2 * lowBuffer[1] + 3 * lowBuffer[2] + 4 * lowBuffer[3] +
                5 * lowBuffer[4] + 4 * lowBuffer[5] + 3 * lowBuffer[6] +
                2 * lowBuffer[7] + lowBuffer[8];
  return saida;
}

float highPass(float input, float *highBuffer, int tamanho) {
  static float lastOut = 0;
  attBuffer(input, highBuffer, tamanho);
  float saida = -highBuffer[0] / 32.0 + highBuffer[16] - highBuffer[17] +
                highBuffer[32] / 32.0 + lastOut;
  lastOut = saida;
  return saida / 36.0;
}

float deriva(float input, float *derivBuffer, int tamanho) {
  attBuffer(input, derivBuffer, tamanho);
  float saida = -derivBuffer[4] - 2 * derivBuffer[3] + 2 * derivBuffer[1] + derivBuffer[0];
  return (saida * saida);
}

float avFilter(float input, float *avBuffer, int tamanho) {
  attBuffer(input, avBuffer, tamanho);
  float sum = 0;
  for (int i = 0; i < tamanho; i++) {
    sum += avBuffer[i];
  }
  return sum / (float)tamanho;
}

bool detectRPeak(float *input, int tamanho) {
  return input[1] > input[0] && input[1] > input[2];
}

void updateLevels(float F, float I, bool isSignalPeak, bool PEAK) {
  float sqrtF = sqrt(F * F);
  if (PEAK) {
    if (isSignalPeak) {
      SignalLevelI = 0.125 * I + 0.875 * SignalLevelI;
      SignalLevelF = 0.125 * sqrtF + 0.875 * SignalLevelF;
    } else {
      NoiseLevelI = 0.125 * I + 0.875 * NoiseLevelI;
      NoiseLevelF = 0.125 * sqrtF + 0.875 * NoiseLevelF;
    }
    ThresholdI = NoiseLevelI + 0.25 * (SignalLevelI - NoiseLevelI);
    ThresholdF = NoiseLevelF + 0.25 * (SignalLevelF - NoiseLevelF);
  }
}

bool classifyPeak(bool peak, float F, float I) {
  if (peak) {
    if (I > ThresholdI && sqrt(F * F) > ThresholdF) {
      return true;
    }
  }
  return false;
}

void attBuffer(float input, float *buff, int tamanho) {
  for (int i = tamanho - 1; i > 0; i--) {
    buff[i] = buff[i - 1];
  }
  buff[0] = input;
}
void desenharBluetooth(int x, int y, int tamanho, uint16_t cor) {
  int metade = tamanho / 2;

  // Linha vertical
  tft.drawLine(x, y - tamanho, x, y + tamanho, cor);

  // Linha diagonal superior direita
  tft.drawLine(x - metade , y + metade , x + metade, y - metade, cor);
  tft.drawLine(x + metade, y - metade, x, y - tamanho, cor);

  // Linha diagonal inferior direita
  tft.drawLine(x - metade, y - metade, x + metade, y + metade, cor);
  tft.drawLine(x + metade, y + metade, x, y + tamanho, cor);
}

void plotGraph(float value) {
  static int currentX = GRAPH_X;     // Posição atual no eixo X
  static int lastGraphX = GRAPH_X;   // Última posição X plotada
  static int lastGraphY = GRAPH_Y;   // Última posição Y plotada

  // Mapeia o valor para a altura do gráfico
  int graphY = GRAPH_Y - map(value * 1000, 0, 3300, 0, GRAPH_HEIGHT); // Mapeia de 0 a 3.3V

  // Redesenha a parte da grade para restaurar a aparência original
  restaurarGrade(currentX);

  // Desenha a linha conectando os pontos
  tft.drawLine(lastGraphX, lastGraphY, currentX, graphY, TFT_GREEN);

  // Atualiza as últimas posições
  lastGraphX = currentX;
  lastGraphY = graphY;

  // Incrementa a posição X
  currentX++;
  if (currentX >= GRAPH_X + GRAPH_WIDTH) {
    // Quando atingir o final da tela, reinicia na borda esquerda
    currentX = GRAPH_X;
    lastGraphX = GRAPH_X;
  }
}

void desenharGrade() {
  // Desenha o contorno do gráfico
  tft.drawRect(GRAPH_X, GRAPH_Y - GRAPH_HEIGHT, GRAPH_WIDTH, GRAPH_HEIGHT, TFT_DARKGREY);
  tft.drawLine(GRAPH_WIDTH - 1, GRAPH_Y - 1, GRAPH_WIDTH - 1, GRAPH_Y + 1 - GRAPH_HEIGHT, TFT_BLACK);
  // Define a largura de cada quadrado da grade
  int larguraQuadrado = 25; // Ajuste conforme necessário
  int numQuadradosX = GRAPH_WIDTH / larguraQuadrado;

  // Desenha os quadrados no eixo X
  for (int i = 0; i <= numQuadradosX; i++) {
    int xInicio = GRAPH_X + i * larguraQuadrado;

    // Linhas verticais da grade
    tft.drawLine(xInicio, GRAPH_Y, xInicio, GRAPH_Y - GRAPH_HEIGHT, TFT_DARKGREY);
  }

  // Linhas horizontais para completar a grade (opcional)
  int alturaQuadrado = 25; // Mesmo tamanho que no eixo X
  int numQuadradosY = GRAPH_HEIGHT / alturaQuadrado;

  for (int i = 0; i < numQuadradosY; i++) {
    int yInicio = GRAPH_Y - i * alturaQuadrado;

    // Linhas horizontais da grade
    tft.drawLine(GRAPH_X, yInicio, GRAPH_X + GRAPH_WIDTH, yInicio, TFT_DARKGREY);
  }
}

void restaurarGrade(int xPos) {
  // Apaga a coluna vertical na posição xPos
  tft.drawLine(xPos, GRAPH_Y - 1, xPos, GRAPH_Y - GRAPH_HEIGHT, TFT_BLACK);

  // Restaura a linha vertical da grade, se estiver alinhada
  int larguraQuadrado = 25; // Mesmo valor usado em desenharGrade()
  if ((xPos - GRAPH_X) % larguraQuadrado == 0) {
    tft.drawLine(xPos, GRAPH_Y, xPos, GRAPH_Y - GRAPH_HEIGHT, TFT_DARKGREY);
  }

  // Restaura os pontos de interseção das linhas horizontais
  int alturaQuadrado = 25; // Mesmo valor usado em desenharGrade()
  int numQuadradosY = GRAPH_HEIGHT / alturaQuadrado;

  for (int i = 0; i <= numQuadradosY; i++) { // Inclui bordas
    int yInicio = GRAPH_Y - i * alturaQuadrado;

    tft.drawPixel(xPos, yInicio, TFT_DARKGREY); // Pontos de interseção
  }
  tft.drawPixel(xPos, GRAPH_Y - 1, TFT_DARKGREY);
}

void desenharCoracao(int x, int y, int tamanho, uint16_t cor) {
  int w = tamanho;       // Largura do coração
  int h = w;             // Altura do coração (ajustável)
  int raio = w / 2;      // Raio para os círculos

  // Desenha o lado esquerdo do coração (metade superior do círculo)
  for (int i = -raio; i <= raio; i++) {
    for (int j = -raio; j <= 0; j++) {  // Apenas metade superior (j <= 0)
      if (i * i + j * j <= raio * raio) {
        tft.drawPixel(x - w / 2 + i, y - h / 2 + j, cor);
      }
    }
  }

  // Desenha o lado direito do coração (metade superior do círculo)
  for (int i = -raio; i <= raio; i++) {
    for (int j = -raio; j <= 0; j++) {  // Apenas metade superior (j <= 0)
      if (i * i + j * j <= raio * raio) {
        tft.drawPixel(x + w / 2 + i, y - h / 2 + j, cor);
      }
    }
  }

  // Desenha a parte inferior do coração (triângulo)
  tft.fillTriangle(x - w, y - w / 2,      // Ponto esquerdo da base do triângulo
                   x + w, y - w / 2,      // Ponto direito da base do triângulo
                   x, y + h / 2,          // Ponto superior do triângulo (apontando para baixo)
                   cor);
}

void exibirBPM(int BPM, uint16_t corTexto, uint16_t corFundo, int x, int y) {
  char bpmStr[4];  // Buffer para 3 dígitos + caractere nulo

  // Formata o BPM com 3 dígitos (zeros à esquerda, se necessário)
  sprintf(bpmStr, "%03d", BPM);

  // Configurações do texto
  tft.setTextColor(corTexto, corFundo); // Define as cores do texto e do fundo
  tft.setTextSize(2);                  // Define o tamanho do texto
  tft.setCursor(x, y);                 // Define a posição do cursor

  // Exibe o valor formatado
  tft.print(bpmStr);
}
