import serial
import time

# Constante de depuração
DEBUG = True

# Variável global para a conexão serial
arduino = None

def configurar_conexao(log_func, porta="COM7", baudrate=9600):
    """
    Tenta estabelecer a conexão serial ou inicializa o FakeArduino.
    Retorna o objeto Arduino (real ou fake).
    """
    global arduino, DEBUG
    
    class FakeArduino:
        def write(self, data):
            try:
                log_func(f"Enviando comando simulado: {data.decode().strip()}", "DEBUG")
            except Exception:
                pass
        def readline(self):
            return b"[DEBUG] resposta simulada\n"
        @property
        def in_waiting(self):
            return 0
    
    if not DEBUG:
        try:
            arduino = serial.Serial(porta, baudrate, timeout=1)
            time.sleep(2)
            log_func(f"Conexão serial estabelecida em {porta} @ {baudrate} baud.", "SUCESSO")
        except Exception as e:
            log_func(f"Falha ao conectar ao Arduino em {porta}: {e}. Ativando modo DEBUG.", "ERRO")
            DEBUG = True
            arduino = FakeArduino()
    
    if DEBUG and arduino is None:
        arduino = FakeArduino()
        log_func("Sistema inicializado em modo DEBUG.", "INFO")
        
    return arduino

def enviar_comando(motor, direcao, passos, delay, log_func):
    """Envia um comando para o Arduino (ou FakeArduino) e lida com a resposta."""
    global arduino, DEBUG
    
    if arduino is None:
        log_func("Arduino não inicializado.", "ERRO")
        return

    cmd = f"{motor},{direcao},{passos},{delay}\n"
    arduino.write(cmd.encode())
    time.sleep(0.05) 

    if not DEBUG:
        while arduino.in_waiting > 0:
            try:
                log_func(arduino.readline().decode().strip(), "RESPOSTA")
            except Exception as e:
                log_func(f"Erro ao ler resposta serial: {e}", "ERRO")
                break