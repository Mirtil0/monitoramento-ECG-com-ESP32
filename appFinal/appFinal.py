import sys
import serial
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

import time

from PyQt5.QtWidgets import QApplication, QPushButton, QButtonGroup, QWidget, QVBoxLayout, QLineEdit, QLabel, QComboBox, QMessageBox, QDialog, QHBoxLayout, QProgressDialog, QRadioButton, QFormLayout
from PyQt5.QtWidgets import QMenuBar, QAction, QFileDialog, QSlider, QActionGroup, QDateEdit
from PyQt5.QtCore import QTimer, Qt, QSize, QDate
from PyQt5.QtGui import QPixmap, QPainter, QColor, QFont, QIntValidator 

import pyqtgraph as pg 

import serial.tools.list_ports

import os 
from threading import Thread

from scipy import signal

from flask import Flask, render_template, jsonify

import json

from datetime import datetime

data_vector = []
time_vector = []

class CircularButton(QPushButton):
    def __init__(self, text, parent=None):
        super().__init__(text, parent)
        self.setStyleSheet("background-color: green; color: white; border-radius: 10%;")
        self.setFixedSize(60, 60)  # Tamanho do botão

    def set_red(self):
        self.setStyleSheet("background-color: red; color: white; border-radius: 10%;")

    def set_green(self):
        self.setStyleSheet("background-color: green; color: white; border-radius: 10%;")

class MyApp(QWidget):
    def __init__(self):
        super().__init__()
        #diretório do programa
        if getattr(sys, 'frozen', False):
        # Se o script está sendo executado como um executável
            self.caminho = os.path.dirname(sys.executable)
        else:
            # Se o script está sendo executado como um arquivo Python normal
            self.caminho = os.path.dirname(__file__)
            
        # Configurações da janela
        self.setWindowTitle('Teste-ECG')
        self.setGeometry(100, 100, 720, 480)

        # Layout principal
        layout = QVBoxLayout()

        # Label para mostrar mensagens de status
        self.status_label = QLabel('Status: Nenhuma ação executada ainda.')
        self.status_label.setStyleSheet("font-size: 15px;")
        layout.addWidget(self.status_label)
        
        # Adiciona um QLabel clicável para mostrar o caminho do arquivo
        self.file_path_label = QLabel()
        self.file_path_label.setStyleSheet("font-size: 15px; color: blue; text-decoration: underline;")
        self.file_path_label.setText("Caminho do arquivo: Nenhum arquivo salvo.")
        self.file_path_label.mousePressEvent = self.open_directory  # Conecta o clique ao método
        layout.addWidget(self.file_path_label)

        # Botões
        self.button = CircularButton('Iniciar', self) #iniciar e finalizar coleta
        self.pausa = CircularButton('Pausar', self) #pausar e despausar gráfico
        self.filtro = CircularButton('Filtro', self) #ativa e desativa o fltro digital
        self.captura = CircularButton('Capturar', self) #capturar gráfico
        
        
        self.button.clicked.connect(self.toggle_reading) 
        self.pausa.clicked.connect(self.toggle_graf)
        self.filtro.clicked.connect(self.toggle_filtro)
        self.captura.clicked.connect(self.save_graph_image)

        # Layout para os botões
        h_layout_buttons = QHBoxLayout()
        h_layout_buttons.addWidget(self.button)
        h_layout_buttons.addWidget(self.pausa)
        h_layout_buttons.addWidget(self.filtro)
        h_layout_buttons.addWidget(self.captura)
        h_layout_buttons.setAlignment(Qt.AlignLeft)

        # Avisos de eletrodos
        # LO+
        self.timerLOP = QTimer()
        self.timerLOP.timeout.connect(self.piscarLOP)
        self.LOP = QLabel("LO+")
        self.LOP.setAlignment(Qt.AlignCenter)
        self.LOP.setFixedSize(60, 60)
        self.LOP.setStyleSheet("QLabel {background-color:yellow; color:black; font-size:12px; border-radius: 30%}")
        self.piscaLOP = True

        # LO-
        self.timerLON = QTimer()
        self.timerLON.timeout.connect(self.piscarLON)
        self.LON = QLabel("LO-")
        self.LON.setAlignment(Qt.AlignCenter)
        self.LON.setFixedSize(60, 60)
        self.LON.setStyleSheet("QLabel {background-color:red; color:black; font-size:12px; border-radius: 30%}")
        self.piscaLON = True

        # Adiciona os labels em um layout horizontal, para depois adicionar no layout principal
        h_layout_labels = QHBoxLayout()
        h_layout_labels.addWidget(self.LOP)
        h_layout_labels.addWidget(self.LON)
        h_layout_labels.setAlignment(Qt.AlignRight)

        # Cria um novo label para BPM
        self.bpm_label = ImageLabel(self.caminho + r"//coracao.png", "60", self)
        self.bpm_label.setAlignment(Qt.AlignCenter)

        # Layout principal para alinhar os dois layouts horizontalmente
        main_h_layout = QHBoxLayout()
        main_h_layout.addLayout(h_layout_buttons)      # Botões alinhados à esquerda
        main_h_layout.addWidget(self.bpm_label)         # Label BPM no meio
        main_h_layout.addLayout(h_layout_labels)         # Labels alinhados à direita

        # Adiciona o layout principal ao layout vertical
        layout.addLayout(main_h_layout)
        
        self.num_amostras = 1000 #visualizadas no grafico
        self.offset_amostras = 98 #offset dos buffers para melhor visualização
        self.numT_amostras = 2000 #número máximo de amostras na memória
        
        # Adiciona o widget do gráfico (PyQtGraph)
        self.plot_widget = pg.PlotWidget()
        self.plot_widget.showGrid(x=True, y=True, alpha=0.7)  # Ativa o grid em X e Y com uma leve transparência
        self.plot_widget.getAxis('bottom').setTickSpacing(major=200, minor=40)
        layout.addWidget(self.plot_widget)
        self.estadoGraf = True
        self.aguardar_plot = 0
        
        # Criar o Slider para o número de amostras
        self.slider_nAmostra = QSlider(Qt.Horizontal)
        self.slider_nAmostra.setMinimum(1)
        self.slider_nAmostra.setMaximum(int(self.numT_amostras/40)-1)
        self.slider_nAmostra.setValue(int(self.num_amostras/40))  # Valor inicial
        self.num_amostras = 55 - int(1000/40) #visualizadas no grafico
        self.slider_nAmostra.valueChanged.connect(self.update_slider)
        
        self.label_tempo = QLabel("a")
        self.label_tempo.setAlignment(Qt.AlignCenter)
        self.label_tempo.setStyleSheet("font-size: 15px; color: black;")
        self.label_tempo.setText(f'{self.num_amostras}mm/s')

        self.slider_nAmostra_layout = QHBoxLayout()
        self.slider_nAmostra_layout.addWidget(self.slider_nAmostra)
        self.slider_nAmostra_layout.setAlignment(Qt.AlignCenter)

        layout.addLayout(self.slider_nAmostra_layout)
        layout.addWidget(self.label_tempo)
        
        low = 0.5 / (0.5 * 200)  # Frequência de corte inferi or normalizada
        high = 15 / (0.5 * 200)  # Frequência de corte superior normalizada
        
        self.b, self.a = signal.iirnotch(60, 20, 200)

        # Inicializa o gráfico
        self.plot_curve = self.plot_widget.plot(pen='g')  # Linha verde
        self.data_buffer = []  # Buffer para armazenar as últimas 300 amostras
        self.time_buffer = []  # Buffer para armazenar o tempo das amostras
        
        
        #barras de menu
        self.menu_bar = QMenuBar(self)
        
        arquivo_menu = self.menu_bar.addMenu("Arquivo")
        abrir_graf = QAction("Visualizar coleta", self)
        abrir_graf.triggered.connect(self.visu_coleta)  # Conectar a ação
        arquivo_menu.addAction(abrir_graf)
        
        # Menu Visualização
        visualizacao_menu = self.menu_bar.addMenu("Visualização")
        
        # Submenu Ganho
        ganho_menu = visualizacao_menu.addMenu("Ganho")
        ganho_group = QActionGroup(self)
        
        #Selecionar paciente
        paciente_menu = QAction("Paciente", self)
        paciente_menu.triggered.connect(self.selecionarPaciente)
        self.menu_bar.addAction(paciente_menu)
        self.paciente = {"name": "teste", "age": "0"}
        
        # Adicionar as 3 opções de Ganho
        ganho_opcao1 = QAction("1mV/div", self, checkable=True)
        ganho_opcao1.setChecked(True)  # Marcar a primeira opção como selecionada
        ganho_opcao2 = QAction("2mV/div", self, checkable=True)
        ganho_opcao3 = QAction("0.5mV/div", self, checkable=True)
        
        # Conectar as ações
        ganho_opcao1.triggered.connect(lambda: self.alterar_ganho(1))
        ganho_opcao2.triggered.connect(lambda: self.alterar_ganho(2))
        ganho_opcao3.triggered.connect(lambda: self.alterar_ganho(0.5))
        
        self.ganho_vertical = 1
            
        ganho_menu.addAction(ganho_opcao1)
        ganho_menu.addAction(ganho_opcao2)
        ganho_menu.addAction(ganho_opcao3)
        
        # Adicionar as ações ao grupo, garantindo que apenas uma ação esteja selecionada
        ganho_group.addAction(ganho_opcao1)
        ganho_group.addAction(ganho_opcao2)
        ganho_group.addAction(ganho_opcao3)
        
        layout.setMenuBar(self.menu_bar)

        # Adiciona o layout à janela
        self.setLayout(layout)

        # Inicialmente não conectada
        self.serial_port = None

        # Lista para armazenar as linhas recebidas
        self.data_list = []
        
        # listas para armazenar os últimos estados dos eletrodos:
        self.listLOP = []
        self.listLON = []
        
        self.lastTime = 0

        # Timer para leitura da porta serial
        self.timer = QTimer()
        self.timer.timeout.connect(self.read_serial)
        self.is_reading = False  # Variável de controle da leitura

        # Variável para armazenar o tempo inicial
        self.start_time = None

        # Verificar portas seriais disponíveis e conectar
        self.find_serial_ports()
        self.serial_port.write(b'iniciado\n')
        self.sinal = 1
        
    def find_serial_ports(self):
        # Lista as portas seriais disponíveis
        ports = list(serial.tools.list_ports.comports())

        if len(ports) == 0:
            QMessageBox.critical(self, "Erro", "Nenhuma porta serial encontrada.")
            sys.exit()  # Sai do programa se não encontrar portas seriais

        elif len(ports) == 1:
            # Se apenas uma porta for encontrada, conectar automaticamente
            self.connect_serial(ports[0].device)

        else:
            # Se houver mais de uma porta, perguntar ao usuário qual deve ser usada
            port_selection_dialog = PortSelectionDialog(ports, self)
            if port_selection_dialog.exec_() == port_selection_dialog.Accepted:
                selected_port = port_selection_dialog.get_selected_port()
                self.connect_serial(selected_port)

    def connect_serial(self, port_name):
        try:
            self.serial_port = serial.Serial(port_name, 115200, timeout=1)
            self.status_label.setText(f"Conectado à porta {port_name}")
        except Exception as e:
            self.status_label.setText(f"Erro ao conectar à porta {port_name}: {e}")
            QMessageBox.critical(self, "Erro", f"Erro ao conectar à porta {port_name}: {e}")
            sys.exit()

    def toggle_reading(self):
        if not self.is_reading:
            self.start_reading()
        else:
            self.stop_reading()

    def start_reading(self):
        try:
            self.timerLOP.start(500)
            self.timerLON.start(500)
            
            #essas listas são responsáveis por armazenar todas as variáveis
            self.leitura_buffer = [0]*self.numT_amostras
            self.tempo_buffer = np.linspace(0, (self.numT_amostras - 1) * 5, self.numT_amostras).tolist()
            
            if self.serial_port and self.serial_port.is_open:
                # Limpa os buffers e o gráfico antes de começar uma nova leitura
                self.data_buffer.clear()  # Limpa o buffer de dados do gráfico
                self.time_buffer.clear()  # Limpa o buffer de tempo
                self.data_list.clear()  # Limpa o buffer de dados recebidos
                self.plot_curve.clear()  # Limpa o gráfico

                # Envia o comando para iniciar a leitura
                self.serial_port.write(b'begin\n')  
                self.status_label.setText("Leitura iniciada.")
                self.button.set_red()
                self.button.setText('Parar')
                self.is_reading = True
                self.start_time = time.time()  # Armazena o tempo inicial
                self.timer.start(30)  # Inicia o timer para leitura periódica
            else:
                self.status_label.setText("Porta serial não conectada.")
        except Exception as e:
            self.status_label.setText(f"Erro ao iniciar leitura: {e}")

    def stop_reading(self):
        try:
            self.timerLOP.stop()
            self.timerLON.stop()
            
            #limpa as listas para economizar memoria
            self.leitura_buffer = []
            self.tempo_buffer = []
            self.lastTime = 0
            
            self.LOP.setStyleSheet("QLabel {background-color:yellow; color:black; font-size:12px; border-radius: 30%}")  
            self.LON.setStyleSheet("QLabel {background-color:red; color:black; font-size:12px; border-radius: 30%}")  
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.write(b'finish\n')  # Envia o comando "finish"
                self.status_label.setText("Leitura finalizada.")
                self.save_to_csv()  # Salva os dados no CSV
                self.button.set_green()
                self.button.setText('Iniciar')
                self.pausa.setText('Pausar')
                self.pausa.set_green()
                self.estadoGraf = True
                self.is_reading = False
                self.timer.stop()  # Para a leitura periódica
        except Exception as e:
            self.status_label.setText(f"Erro ao parar leitura: {e}")

    def read_serial(self):
        if not self.serial_port or not self.serial_port.is_open:
            return  # Retorna se a porta serial não estiver aberta
        try:
            while self.serial_port.in_waiting > 0:
                bigline = self.serial_port.readline().decode('utf-8').strip().split(";")
                for shortline in bigline:
                    line = shortline.strip().split(",")
                    
                    if int(line[2]) > self.lastTime:
                        continue
                    if line:
                        self.data_list.append(line)
                        self.status_label.setText(f"Recebido: {line}")
                        
                        # Atualiza o gráfico e buffer apenas em intervalos maiores
                        self.update_graph(float(line[0]), int(line[2]))
                        self.lastTime = int(line[2]) + 6
                        
                        self.listLOP.append(line[4])
                        self.listLON.append(line[3])
                        
                        if len(self.listLOP) > 1000:
                            self.listLOP.pop(0)
                            self.listLON.pop(0)
                            
                        if '1' not in self.listLOP:#reinicia o contador de LO+ sempre que as últimas 1000 amostras forem todas zero
                            self.LOP.setStyleSheet("QLabel {background-color:yellow; color:black; font-size:12px; border-radius: 30%}")
                            self.timerLOP.start(500)
                            
                        if '1' not in self.listLON:#reinicia o contador de LO+ sempre que as últimas 1000 amostras forem todas zero
                            self.LON.setStyleSheet("QLabel {background-color:red; color:black; font-size:12px; border-radius: 30%}")
                            self.timerLON.start(500)
                        
                        self.bpm_label.updateText(line[5])
                        
        except Exception as e:
            self.status_label.setText(f"Erro ao ler da serial: {e}")

    def update_graph(self, new_value, new_time):
        
        global data_vector
        global time_vector

        #desativa
        self.plot_widget.setMouseEnabled(x=False, y=False)  # Desativa o zoom e o pan
        
        # Adiciona o novo valor ao buffer
        self.leitura_buffer.append(new_value)
        self.leitura_buffer.pop(0)

        # Calcula o tempo decorrido desde o início
        self.tempo_buffer.append(new_time + self.numT_amostras*5)
        self.tempo_buffer.pop(0)
        
        # Atualiza os buffers de plot apenas quandos está despausado
        self.aguardar_plot = self.aguardar_plot + 1
        if self.estadoGraf and self.aguardar_plot >= 25:
            self.aguardar_plot = 0
            # Mantém o tamanho dos buffer de plot
            #filtra 60hz
            if self.sinal:
                self.data_buffer = np.array(self.leitura_buffer.copy())
                self.data_buffer = signal.filtfilt(self.b, self.a, self.data_buffer)
                self.data_buffer = self.data_buffer.tolist()
            else:
                self.data_buffer = self.leitura_buffer.copy()
            self.time_buffer = self.tempo_buffer.copy()

        # Define o limite máximo e mínimo para o eixo y
        '''if self.ganho_vertical == 0.5:
            self.plot_widget.setYRange(1, 3.5)
        elif self.ganho_vertical == 1:
            self.plot_widget.setYRange(-1, 5)
        elif self.ganho_vertical == 2:
            self.plot_widget.setYRange(-4, 8)'''
            
        y_min = -3*self.ganho_vertical + 2
        y_max = 3*self.ganho_vertical + 2

        self.plot_widget.setYRange(y_min, y_max)

        # Define o espaçamento entre as linhas horizontais
        self.plot_widget.getAxis('left').setTickSpacing(major=self.ganho_vertical, minor=self.ganho_vertical/5) 
        data_vector = self.data_buffer[-self.num_amostras+self.offset_amostras:-self.offset_amostras]
        time_vector = self.time_buffer[-self.num_amostras+self.offset_amostras:-self.offset_amostras]
          
        self.plot_curve.setData(time_vector, data_vector)
        
    def save_graph_image(self):
        # Captura a imagem do widget de gráfico
        screenshot = self.plot_widget.grab()

        # Define o caminho e nome do arquivo para salvar
        screenshot.save(self.caminho + f'\\prints\\{self.paciente["name"]}-{self.paciente["age"]}-{self.time_buffer[0]}ms.png', 'PNG')

    def save_to_csv(self):
        # Verifica se há dados para salvar
        if self.data_list:
            # Obtém o nome do arquivo do campo de entrada
            filename = f"\\dados\\" + self.paciente["name"] + "-" + str(self.paciente["age"])
            
            # Se o nome estiver vazio, define um padrão
            if len(filename.replace(f"\\dados\\", "")) == 0:
                filename = f"\\dados\\" + 'dados_serial.csv'
                            # Garante que o arquivo tenha extensão .csv
            if not filename.endswith('.csv'):
                filename += '.csv'

            filename = self.caminho + filename
            
            # Cria um DataFrame do Pandas
            df = pd.DataFrame(self.data_list)
            df.columns = ['leitura','filtrado','tempoAmostra', 'LO+', 'LO-', 'BPM', 'pico']

            # Filtra as amostras anteriores da nova coleta
            indice_zero = df[df['tempoAmostra'] == '0'].index
            if not indice_zero.empty:
                df = df.iloc[indice_zero[0]:]
                
            df.to_csv(filename, index=False)
            self.status_label.setText(f"Dados salvos com sucesso!")
            self.file_path_label.setText(f"Salvo em: {filename}")  # Atualiza o caminho do arquivo
            self.data_list.clear()  # Limpa a lista após salvar
        else:
            self.status_label.setText("Nenhum dado para salvar.")

    def open_directory(self, event):
        # Abre o diretório onde o arquivo foi salvo
        file_path = self.file_path_label.text().replace("Salvo em: ", "")
        if os.path.exists(os.path.dirname(file_path)):
            os.startfile(os.path.dirname(file_path))  # Abre a pasta no Explorer4
    
    #piscar LOP
    def piscarLOP(self):
        
        if self.piscaLOP:
            self.piscaLOP = not self.piscaLOP
            self.LOP.setStyleSheet("QLabel {background-color:yellow; color:black; font-size:12px; border-radius: 30%}")        
        else:
            self.piscaLOP = not self.piscaLOP
            self.LOP.setStyleSheet("QLabel {background-color:transparent; color:black; font-size:12px; border-radius: 30%}")
            
    #piscar LON       
    def piscarLON(self):
        if self.piscaLON:
            self.piscaLON = not self.piscaLON
            self.LON.setStyleSheet("QLabel {background-color:red; color:black; font-size:12px; border-radius: 30%}")        
        else:
            self.piscaLON = not self.piscaLON
            self.LON.setStyleSheet("QLabel {background-color:transparent; color:black font-size:12px; border-radius: 30%}")
            
    #pausar e despausar gráfico        
    def toggle_graf(self):
        if self.is_reading: #apenas quando a leitura está sendo realizada      
            if self.estadoGraf: #troca o estado o gráfico
                self.pausa.setText('Seguir')
                self.pausa.set_red()
                self.estadoGraf = not self.estadoGraf
            else:
                self.pausa.setText('Pausar')
                self.pausa.set_green()
                self.estadoGraf = not self.estadoGraf
                
    #atualiza alguns objetos ao se redimensionar a janela
    def resizeEvent(self, event):
        super().resizeEvent(event)
        # Atualiza a largura do campo de entrada para 1/3 da largura atual da janela 
        self.slider_nAmostra.setFixedWidth(self.width() // 4)
        
    def closeEvent(self, event):
        """Esse método é chamado ao fechar a janela."""
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.write(b'finish\n')
            time.sleep(0.5)
            self.serial_port.write(b'reset\n')
            self.serial_port.close()  # Fecha a porta serial ao encerrar o programa
            print("Porta serial fechada com sucesso.")
        event.accept()  # Continua o encerramento do programa
        
    def toggle_filtro(self):
        if self.sinal: #troca o estado o gráfico
            #self.filtro.setText('Seguir')
            self.filtro.set_red()
            self.sinal = 0
        else:
            #self.filtro.setText('Pausar')
            self.filtro.set_green()
            self.sinal = 1
    
    # Função para coletar dados
    def visu_coleta(self):
        options = QFileDialog.Options()
        options |= QFileDialog.ReadOnly
        file_path, _ = QFileDialog.getOpenFileName(
            self, 
            "Selecione um arquivo", 
            self.caminho + "/dados", 
            "Arquivos CSV (*.csv);;Todos os Arquivos (*)", 
            options=options
        )
        
        if file_path:
            try:
                # Carrega o arquivo no formato pandas DataFrame
                self.visu_data = pd.read_csv(file_path)
                
                # Exibe uma mensagem de sucesso
                QMessageBox.information(self, "Arquivo Carregado", f"O arquivo foi carregado com sucesso!")
                
                # Chama a função para plotar os dados
                self.plotar_graf()
                
            except Exception as e:
                # Exibe mensagem de erro em caso de falha ao carregar o arquivo
                QMessageBox.critical(self, "Erro", f"Erro ao abrir o arquivo: {e}")
        else:
            QMessageBox.warning(self, "Nenhum Arquivo Selecionado", "Por favor, selecione um arquivo.")

    # Função para abrir uma janela de gráfico interativo com Matplotlib diretamente
    def plotar_graf(self):
        if self.visu_data is not None:
            try:
                # Inicializa o gráfico
                fig, ax = plt.subplots()
                ax.plot(self.visu_data['tempoAmostra'], self.visu_data['leitura'])
                ax.set_xlabel("Tempo [ms]")
                ax.set_ylabel("Amplitude [mV]")
                ax.legend()
                ax.grid(True)

                # Lista para armazenar as linhas que forem inseridas
                h_lines = []
                v_lines = []
                text_labels_h = []
                text_labels_v = []
                # Flag para controlar o estado interativo
                self.interactive_mode = False

                # Tolerância para a verificação de linhas existentes
                tolerancia = 0.05  # Ajuste esse valor conforme necessário

                # Função para adicionar/remover linhas com cliques do mouse
                def on_click(event):
                    if event.inaxes != ax or not self.interactive_mode:
                        return

                    # Verifica se o clique foi com o botão esquerdo (linha horizontal) ou direito (linha vertical)
                    if event.button == 1:  # Esquerdo - linha horizontal
                        # Checa se já existe uma linha na posição y com tolerância
                        for i, h in enumerate(h_lines):
                            # Usando [0] para pegar o valor escalar
                            if np.abs(h.get_ydata()[0] - event.ydata) < tolerancia:
                                h.remove()
                                text_labels_h[i].remove()  # Remove o texto correspondente
                                h_lines.pop(i)
                                text_labels_h.pop(i)
                                plt.draw()  # Atualiza o gráfico após a remoção
                                return

                        # Se não existe, adiciona a linha
                        h_line = ax.axhline(y=event.ydata, color='black', linestyle='--', linewidth=2)
                        h_lines.append(h_line)
                        # Adiciona o valor da linha horizontal no gráfico
                        text = ax.text(event.xdata, event.ydata, f'{event.ydata:.2f}', color='black', verticalalignment='bottom', fontsize=10)
                        text_labels_h.append(text)

                    elif event.button == 3:  # Direito - linha vertical
                        # Checa se já existe uma linha na posição x com tolerância
                        for i, v in enumerate(v_lines):
                            # Usando [0] para pegar o valor escalar
                            if np.abs(v.get_xdata()[0] - event.xdata) < tolerancia:
                                v.remove()
                                text_labels_v[i].remove()  # Remove o texto correspondente
                                v_lines.pop(i)
                                text_labels_v.pop(i)
                                plt.draw()  # Atualiza o gráfico após a remoção
                                return

                        # Se não existe, adiciona a linha
                        v_line = ax.axvline(x=event.xdata, color='red', linestyle='--', linewidth=2)
                        v_lines.append(v_line)
                        # Adiciona o valor da linha vertical no gráfico
                        text = ax.text(event.xdata, event.ydata, f'{event.xdata:.2f}', color='red', horizontalalignment='right', fontsize=10)
                        text_labels_v.append(text)

                    # Atualiza o gráfico para mostrar as novas linhas e os textos
                    plt.draw()

                # Função para ativar/desativar o modo interativo
                def toggle_interactive(event):
                    self.interactive_mode = not self.interactive_mode  # Alterna o modo interativo

                # Conecta os eventos de clique do mouse e teclas
                fig.canvas.mpl_connect('button_press_event', on_click)
                fig.canvas.mpl_connect('key_press_event', toggle_interactive)  # Usa tecla para alternar o modo

                # Mostra o gráfico
                plt.show()

            except IndexError:
                QMessageBox.warning(self, "Erro", "O arquivo selecionado não possui colunas suficientes.")
            except Exception as e:
                QMessageBox.critical(self, "Erro", f"Ocorreu um erro ao tentar plotar os dados: {e}")
        else:
            QMessageBox.warning(self, "Erro", "Nenhum dado foi carregado para plotar.")

    def update_slider(self, value):
        # Método que atualiza o label com o valor do slider
        self.num_amostras = 2200 - value*40
        self.label_tempo.setText(f'{55 - int(self.num_amostras/40)}mm/s')
        
    def alterar_ganho(self, opcao):
        self.ganho_vertical = opcao
        
    def selecionarPaciente(self):
        dialog = DataSelectionDialog(self.caminho + "\\dados.json")
        if dialog.exec_() == QDialog.Accepted:
            self.paciente = dialog.get_selected_data()
            self.setWindowTitle(f'{self.paciente["name"]}-ECG')
            print(self.paciente)


#janela de seleção da porta de comuniacação serial
class PortSelectionDialog(QDialog):
    def __init__(self, ports, parent=None):
        super().__init__(parent)
        
        self.setWindowTitle("Selecionar Porta Serial")
        self.resize(400, 200)
        layout = QVBoxLayout(self)

        self.port_combo = QComboBox(self)
        for port in ports:
            self.port_combo.addItem(port.device)

        layout.addWidget(self.port_combo)

        self.ok_button = QPushButton("OK", self)
        self.ok_button.clicked.connect(self.accept)
        layout.addWidget(self.ok_button)

    def get_selected_port(self):
        return self.port_combo.currentText()
  
# Defina a subclasse ImageLabel
class ImageLabel(QLabel):
    def __init__(self, image_path, text, parent=None):
        super().__init__(parent)
        self.image = QPixmap(image_path)  # Carrega a imagem
        self.text = text  # Texto a ser exibido
        self.setFixedSize(60, 60)  # Define um tamanho fixo para o label

    def paintEvent(self, event):
        painter = QPainter(self)
        
        # Desenha a imagem
        painter.drawPixmap(0, 0, self.image.scaled(self.size(), Qt.KeepAspectRatio))  # Ajusta a imagem ao tamanho do label

        # Define a cor e a fonte do texto
        painter.setPen(QColor(0, 0, 0))  # Cor do texto (vermelho, por exemplo)
        painter.setFont(QFont("Arial", 12))  # Fonte do texto

        # Desenha o texto no centro do QLabel
        text_rect = painter.boundingRect(self.rect(), Qt.AlignCenter, self.text)
        painter.drawText(text_rect, Qt.AlignCenter, self.text)
        
    def updateText(self, new_text):
        self.text = new_text  # Atualiza o texto
        self.update()  # Solicita a atualização da interface (redesenha o label)
        
# Classe para Janela de Carregamento
class ProgressBar:
    def __init__(self, title="ECG", message="Conectando a uma rede wifi...", maximum=100):
        self.progress_dialog = QProgressDialog(message, "Cancelar", 0, maximum)
        self.progress_dialog.setWindowTitle(title)
        self.progress_dialog.setModal(True)

    def start(self):
        self.progress_dialog.show()

    def update(self, value):
        self.progress_dialog.setValue(value)

    def was_canceled(self):
        return self.progress_dialog.wasCanceled()

    def close(self):
        self.progress_dialog.close()

class DataSelectionDialog(QDialog):
    selected_data = None

    def __init__(self, json_file, parent=None):
        super().__init__(parent)

        self.setWindowTitle("Selecionar paciente")
        self.resize(400, 300)

        # Arquivo JSON para armazenar os dados
        self.json_file = json_file
        self.existing_data = self.load_existing_data()

        # Layout principal
        layout = QVBoxLayout(self)

        # Opções: Novo ou Existente
        self.option_group = QButtonGroup(self)
        self.new_radio = QRadioButton("Novo", self)
        self.existing_radio = QRadioButton("Existente", self)
        self.new_radio.setChecked(True)  # Opção padrão
        self.option_group.addButton(self.new_radio)
        self.option_group.addButton(self.existing_radio)
        layout.addWidget(self.new_radio)
        layout.addWidget(self.existing_radio)

        # Layout para dados de "Novo"
        self.new_data_layout = QFormLayout()
        self.name_input = QLineEdit(self)

        # Substituir idade por calendário para data de nascimento
        self.birth_date_input = QDateEdit(self)
        self.birth_date_input.setCalendarPopup(True)
        self.birth_date_input.setDisplayFormat("dd/MM/yyyy")
        self.birth_date_input.setDate(QDate.currentDate())  # Data padrão: hoje

        self.new_data_layout.addRow("Nome:", self.name_input)
        self.new_data_layout.addRow("Data de nascimento:", self.birth_date_input)
        layout.addLayout(self.new_data_layout)

        # Combobox para "Existente"
        self.existing_combo = QComboBox(self)
        for entry in self.existing_data:
            entry_age = self.calculate_age(entry['birth_date'])
            self.existing_combo.addItem(
                f"{entry['name']} - {entry_age} anos", entry
            )
        layout.addWidget(self.existing_combo)
        self.existing_combo.setEnabled(False)  # Desabilitado inicialmente

        # Conexão para habilitar/desabilitar os campos com base na escolha
        self.option_group.buttonToggled.connect(self.toggle_inputs)

        # Botão OK
        self.ok_button = QPushButton("OK", self)
        self.ok_button.clicked.connect(self.save_data)
        layout.addWidget(self.ok_button)

    def toggle_inputs(self, button):
        if button == self.new_radio:
            self.name_input.setEnabled(True)
            self.birth_date_input.setEnabled(True)
            self.existing_combo.setEnabled(False)
        else:
            self.name_input.setEnabled(False)
            self.birth_date_input.setEnabled(False)
            self.existing_combo.setEnabled(True)

    def load_existing_data(self):
        try:
            with open(self.json_file, 'r') as file:
                return json.load(file)
        except (FileNotFoundError, json.JSONDecodeError):
            return []

    def calculate_age(self, birth_date):
        birth_date_obj = datetime.strptime(str(birth_date), "%d/%m/%Y")
        today = datetime.today()
        age = today.year - birth_date_obj.year - ((today.month, today.day) < (birth_date_obj.month, birth_date_obj.day))
        return age

    def save_data(self):
              
        if self.new_radio.isChecked():
            name = self.name_input.text().strip()
            birth_date = self.birth_date_input.date().toString("dd/MM/yyyy")

            if not name:
                QMessageBox.warning(self, "Erro", "Preencha todos os campos corretamente!")
                return

            new_entry = {'name': name, 'birth_date': birth_date}
            self.existing_data.append(new_entry)
            self.selected_data = {"name": name, "age": self.calculate_age(birth_date)}
        else:
            selected_entry = self.existing_combo.currentData()
            self.selected_data = {
                "name": selected_entry["name"],
                "age": self.calculate_age(selected_entry["birth_date"]),
            }

        # Salva no arquivo JSON
        with open(self.json_file, 'w') as file:
            json.dump(self.existing_data, file, indent=4)

        QMessageBox.information(self, "Sucesso", "Dados salvos com sucesso!")
        self.accept()

    def get_selected_data(self):
        return self.selected_data

    
if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MyApp()
    window.show()
    sys.exit(app.exec_())
