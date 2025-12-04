import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import threading
import time
import math
from collections import deque # Usaremos uma deque para a fila, é mais eficiente

# --- Constantes físicas do braço (em mm) ---
L1 = 110.0
L2 = 80.0
L3 = 90.0
R_MAX = L2 + L3
R_MIN = abs(L2 - L3)

# --- Constantes da Fonte Vetorial ---
# Define a altura "segura" da caneta acima do papel
Z_PEN_UP = 5.0 
# Define a altura da caneta no papel (relativo à altura Z base)
Z_PEN_DOWN = 0.0 
# Define o tamanho padrão das letras
LETTER_HEIGHT = 10.0 # mm
LETTER_WIDTH = 6.0   # mm
LETTER_SPACING = 2.0 # mm entre as letras

# Define os traçados das letras como uma lista de pontos (x, y, z)
# Z é Z_PEN_UP ou Z_PEN_DOWN
# As coordenadas (x,y) são relativas ao canto inferior esquerdo da letra
FONT_LIBRARY = {
    'A': [
        (0, 0, Z_PEN_UP), # 1. Levanta, vai para o início
        (0, 0, Z_PEN_DOWN), # 2. Abaixa
        (LETTER_WIDTH / 2, LETTER_HEIGHT, Z_PEN_DOWN), # 3. Desenha para o topo
        (LETTER_WIDTH, 0, Z_PEN_DOWN), # 4. Desenha para baixo/direita
        (LETTER_WIDTH, 0, Z_PEN_UP), # 5. Levanta
        (LETTER_WIDTH * 0.25, LETTER_HEIGHT / 2, Z_PEN_UP), # 6. Levanta, move para a barra
        (LETTER_WIDTH * 0.25, LETTER_HEIGHT / 2, Z_PEN_DOWN), # 7. Abaixa
        (LETTER_WIDTH * 0.75, LETTER_HEIGHT / 2, Z_PEN_DOWN), # 8. Desenha barra
        (LETTER_WIDTH * 0.75, LETTER_HEIGHT / 2, Z_PEN_UP), # 9. Levanta
    ],
    'B': [
        (0, 0, Z_PEN_UP),
        (0, 0, Z_PEN_DOWN),
        (0, LETTER_HEIGHT, Z_PEN_DOWN), # Linha vertical
        (LETTER_WIDTH * 0.8, LETTER_HEIGHT * 0.75, Z_PEN_DOWN), # Curva de cima
        (0, LETTER_HEIGHT / 2, Z_PEN_DOWN),
        (LETTER_WIDTH, LETTER_HEIGHT * 0.25, Z_PEN_DOWN), # Curva de baixo
        (0, 0, Z_PEN_DOWN),
        (0, 0, Z_PEN_UP),
    ],
    'C': [
        (LETTER_WIDTH, LETTER_HEIGHT * 0.2, Z_PEN_UP),
        (LETTER_WIDTH, LETTER_HEIGHT * 0.2, Z_PEN_DOWN),
        (LETTER_WIDTH / 2, 0, Z_PEN_DOWN), # Base
        (0, LETTER_HEIGHT / 2, Z_PEN_DOWN), # Meio
        (LETTER_WIDTH / 2, LETTER_HEIGHT, Z_PEN_DOWN), # Topo
        (LETTER_WIDTH, LETTER_HEIGHT * 0.8, Z_PEN_DOWN),
        (LETTER_WIDTH, LETTER_HEIGHT * 0.8, Z_PEN_UP),
    ],
    'L': [
        (0, LETTER_HEIGHT, Z_PEN_UP),
        (0, LETTER_HEIGHT, Z_PEN_DOWN),
        (0, 0, Z_PEN_DOWN), # Linha vertical
        (LETTER_WIDTH, 0, Z_PEN_DOWN), # Linha horizontal
        (LETTER_WIDTH, 0, Z_PEN_UP),
    ],
    'I': [
        (LETTER_WIDTH / 2, LETTER_HEIGHT, Z_PEN_UP),
        (LETTER_WIDTH / 2, LETTER_HEIGHT, Z_PEN_DOWN),
        (LETTER_WIDTH / 2, 0, Z_PEN_DOWN), # Linha vertical
        (LETTER_WIDTH / 2, 0, Z_PEN_UP),
    ],
    'T': [
        (0, LETTER_HEIGHT, Z_PEN_UP),
        (0, LETTER_HEIGHT, Z_PEN_DOWN),
        (LETTER_WIDTH, LETTER_HEIGHT, Z_PEN_DOWN), # Barra horizontal
        (LETTER_WIDTH, LETTER_HEIGHT, Z_PEN_UP),
        (LETTER_WIDTH / 2, LETTER_HEIGHT, Z_PEN_UP),
        (LETTER_WIDTH / 2, LETTER_HEIGHT, Z_PEN_DOWN),
        (LETTER_WIDTH / 2, 0, Z_PEN_DOWN), # Barra vertical
        (LETTER_WIDTH / 2, 0, Z_PEN_UP),
    ],
    'O': [
        (LETTER_WIDTH / 2, 0, Z_PEN_UP),
        (LETTER_WIDTH / 2, 0, Z_PEN_DOWN),
        (0, LETTER_HEIGHT / 2, Z_PEN_DOWN),
        (LETTER_WIDTH / 2, LETTER_HEIGHT, Z_PEN_DOWN),
        (LETTER_WIDTH, LETTER_HEIGHT / 2, Z_PEN_DOWN),
        (LETTER_WIDTH / 2, 0, Z_PEN_DOWN),
        (LETTER_WIDTH / 2, 0, Z_PEN_UP),
    ]
}


def check_reachable(x, y, z):
    r = math.hypot(x, y)
    zprime = z - L1
    d = math.hypot(r, zprime)
    if d < (R_MIN - 0.1) or d > (R_MAX + 0.1): # Pequena margem de tolerância
        return False, f"Fora de alcance: (X:{x:.0f} Y:{y:.0f} Z:{z:.0f}) d={d:.1f} mm"
    return True, f"OK (r={r:.1f}, z'={zprime:.1f}, d={d:.1f})"

class App:
    def __init__(self, root):
        self.root = root
        root.title("Controle Braço Robô")
        frame = ttk.Frame(root, padding=10)
        frame.grid(row=0, column=0, sticky="nsew")

        # Frame de Controle Manual
        manual_frame = ttk.LabelFrame(frame, text="Controle Manual", padding=10)
        manual_frame.grid(row=0, column=0, sticky="ew", pady=5)

        # Frame de Escrita
        write_frame = ttk.LabelFrame(frame, text="Controle de Escrita", padding=10)
        write_frame.grid(row=1, column=0, sticky="ew", pady=5)
        
        # Frame de Conexão e Status
        status_frame = ttk.LabelFrame(frame, text="Conexão", padding=10)
        status_frame.grid(row=2, column=0, sticky="ew", pady=5)

        # --- Estado da Aplicação ---
        self.ser = None
        self.is_busy = False # Trava para handshaking
        # Fila de comandos para o sequenciador de escrita
        self.command_queue = deque() 

        # --- Widgets: Conexão ---
        ttk.Label(status_frame, text="Porta:").grid(row=0, column=0, sticky="w")
        self.port_cb = ttk.Combobox(status_frame, values=self.list_ports(), width=15)
        self.port_cb.grid(row=0, column=1, padx=5)
        self.b_connect = ttk.Button(status_frame, text="Conectar", command=self.connect)
        self.b_connect.grid(row=0, column=2, padx=5)

        self.status = tk.StringVar(value="Desconectado")
        ttk.Label(status_frame, textvariable=self.status, relief="sunken", padding=2).grid(row=1, column=0, columnspan=3, sticky="ew", pady=5)

        # --- Widgets: Controle Manual ---
        ttk.Label(manual_frame, text="X (mm):").grid(row=0, column=0, sticky="w")
        self.x_e = ttk.Entry(manual_frame); self.x_e.grid(row=0, column=1, padx=5)
        self.x_e.insert(0, "150.0")

        ttk.Label(manual_frame, text="Y (mm):").grid(row=1, column=0, sticky="w")
        self.y_e = ttk.Entry(manual_frame); self.y_e.grid(row=1, column=1, padx=5)
        self.y_e.insert(0, "0.0")

        ttk.Label(manual_frame, text="Z (mm):").grid(row=2, column=0, sticky="w")
        self.z_e = ttk.Entry(manual_frame); self.z_e.grid(row=2, column=1, padx=5)
        self.z_e.insert(0, "80.0") # Z é a altura do *papel*
        ttk.Label(manual_frame, text="(Altura do Papel)").grid(row=2, column=2, sticky="w")

        ttk.Label(manual_frame, text="Vel (%):").grid(row=3, column=0, sticky="w")
        self.v_e = ttk.Entry(manual_frame); self.v_e.grid(row=3, column=1, padx=5); self.v_e.insert(0,"80")

        self.elbow = tk.StringVar(value="U")
        ttk.Radiobutton(manual_frame, text="Elbow Up", variable=self.elbow, value="U").grid(row=4,column=0, padx=5, pady=5)
        ttk.Radiobutton(manual_frame, text="Elbow Down", variable=self.elbow, value="D").grid(row=4,column=1, padx=5, pady=5)

        self.b_move = ttk.Button(manual_frame, text="Mover", command=self.move_cmd)
        self.b_move.grid(row=5,column=0, pady=5)
        self.b_home = ttk.Button(manual_frame, text="Home", command=self.home_cmd)
        self.b_home.grid(row=5,column=1, pady=5)
        self.b_grip_open = ttk.Button(manual_frame, text="Abrir Garra", command=self.grip_open)
        self.b_grip_open.grid(row=6,column=0, pady=5)
        self.b_grip_close = ttk.Button(manual_frame, text="Fechar Garra", command=self.grip_close)
        self.b_grip_close.grid(row=6,column=1, pady=5)

        # --- Widgets: Controle de Escrita ---
        ttk.Label(write_frame, text="Texto para Escrever:").grid(row=0, column=0, sticky="w", padx=5)
        self.text_entry = ttk.Entry(write_frame, width=30)
        self.text_entry.grid(row=0, column=1, padx=5)
        self.text_entry.insert(0, "OLA")
        
        self.b_write = ttk.Button(write_frame, text="Escrever Texto", command=self.write_text_cmd)
        self.b_write.grid(row=0, column=2, padx=5)

        # --- Lista de todos os controles interativos ---
        self.controls = [
            self.b_move, self.b_grip_open, self.b_grip_close,
            self.b_home, self.b_write, self.x_e, self.y_e, self.z_e,
            self.v_e, self.text_entry
        ]

    def list_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        return ports

    def connect(self):
        port = self.port_cb.get()
        if not port:
            messagebox.showerror("Erro", "Selecione a porta serial")
            return
        try:
            self.ser = serial.Serial(port, 115200, timeout=0.1)
            self.status.set(f"Conectado: {port}")
            self.b_connect.config(text="Desconectar", command=self.disconnect)
            self.read_thread = threading.Thread(target=self.read_loop, daemon=True)
            self.read_thread.start()
        except Exception as e:
            messagebox.showerror("Erro", str(e))

    def disconnect(self):
        self.command_queue.clear() # Limpa a fila se estiver desconectando
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.ser = None
        self.status.set("Desconectado")
        self.b_connect.config(text="Conectar", command=self.connect)
        self.is_busy = False # Reseta o estado
        self.enable_controls() # Garante que os controles estão ativos

    def read_loop(self):
        buffer = ""
        while self.ser and self.ser.is_open:
            try:
                raw = self.ser.read_all().decode(errors='ignore')
                if raw:
                    buffer += raw
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        line = line.strip()
                        if line:
                            self.root.after_idle(self.process_message, line)
            except serial.SerialException:
                break
            except Exception as e:
                print("Erro leitura:", e)
            time.sleep(0.05) 
        
        if self.ser: 
            self.root.after_idle(self.disconnect)

    def process_message(self, msg):
        """
        Processa mensagens do Arduino na thread principal da GUI.
        Este é o coração do handshaking e do sequenciador.
        """
        print("Arduino:", msg) # Log no console

        if msg.startswith("ACK:"):
            # O Arduino recebeu o comando, atualiza o status
            if self.command_queue:
                self.status.set(f"Executando... {len(self.command_queue)} restantes.")
            else:
                self.status.set("Movimentando...")

        elif msg.startswith("NOTIFY: Movimento concluido"):
            # O Arduino terminou o último comando.
            # VERIFICA A FILA: Se houver mais comandos, envia o próximo.
            if self.command_queue:
                self.process_queue() # Envia o próximo comando da fila
            else:
                # A fila está vazia, o trabalho terminou.
                self.is_busy = False
                self.enable_controls()
                self.status.set("Pronto.")

        elif msg.startswith("ERR:"):
            # Ocorreu um erro no Arduino
            error_msg = msg.replace("ERR:", "").strip()
            messagebox.showerror("Erro do Arduino", error_msg)
            self.command_queue.clear() # Limpa a fila em caso de erro
            self.is_busy = False
            self.enable_controls()
            self.status.set(f"Erro: {error_msg}")

        elif msg.startswith("NNotify:"): 
            self.status.set(msg.replace("NNotify:", "").strip())
            
        # Ignora outras mensagens de log (Parsed, ANGLES, etc)
        
    def send(self, s):
        """
        Envia um comando MANUAL (Mover, Home, Grip).
        Trava a interface e define is_busy.
        """
        if not self.ser or not self.ser.is_open:
            messagebox.showerror("Erro","Não conectado")
            return False
        
        # Não permite comandos manuais se já estiver ocupado
        if self.is_busy:
            messagebox.showwarning("Ocupado", "Aguarde o movimento anterior terminar.")
            return False

        self.is_busy = True
        self.disable_controls()
        
        print("Enviando (Manual):", s)
        self.ser.write((s+"\n").encode())
        return True

    def disable_controls(self):
        for control in self.controls:
            control.config(state="disabled")

    def enable_controls(self):
        for control in self.controls:
            control.config(state="normal")

    # --- Funções de Comando Manual ---

    def move_cmd(self):
        try:
            x = float(self.x_e.get())
            y = float(self.y_e.get())
            z = float(self.z_e.get())
            v = int(self.v_e.get())
        except ValueError:
            messagebox.showerror("Erro", "Valores inválidos. Use números.")
            return

        ok, msg = check_reachable(x, y, z)
        if not ok:
            messagebox.showerror("Posição fora de alcance", msg)
            return

        cmd = f"MOV {x:.2f} {y:.2f} {z:.2f} {v} {self.elbow.get()}"
        self.send(cmd)

    def grip_open(self): 
        if self.send("GRIP OPEN"):
            self.status.set("Abrindo garra...")

    def grip_close(self): 
        if self.send("GRIP CLOSE"):
            self.status.set("Fechando garra...")

    def home_cmd(self):
        if self.send("HOME"):
            self.status.set("Iniciando 'Home'...")

    # --- Funções do Sequenciador de Escrita ---

    def write_text_cmd(self):
        """
        Função chamada pelo botão "Escrever Texto".
        Ela constrói a fila de comandos, mas não envia nada.
        Apenas inicia o processo chamando process_queue().
        """
        if self.is_busy:
            messagebox.showwarning("Ocupado", "Aguarde a ação anterior terminar.")
            return

        text = self.text_entry.get().upper()
        if not text:
            return

        try:
            # Ponto de origem (canto inferior esquerdo da primeira letra)
            x_start = float(self.x_e.get())
            y_start = float(self.y_e.get())
            z_paper = float(self.z_e.get()) # Altura do papel
            z_safe = z_paper + Z_PEN_UP # Altura de segurança
            v = int(self.v_e.get())
            elbow = self.elbow.get()
        except ValueError:
            messagebox.showerror("Erro", "Valores X, Y, Z e Vel inválidos.")
            return

        self.command_queue.clear()
        current_x_offset = 0.0

        # Passo 1: Construir a fila de movimentos
        for char in text:
            if char == ' ':
                current_x_offset += LETTER_WIDTH / 2 + LETTER_SPACING
                continue

            if char not in FONT_LIBRARY:
                print(f"Caractere '{char}' não definido na biblioteca.")
                continue

            path = FONT_LIBRARY[char]
            for (rel_x, rel_y, rel_z) in path:
                # Calcula coordenadas absolutas
                abs_x = x_start + current_x_offset + rel_x
                abs_y = y_start + rel_y
                abs_z = z_paper + rel_z # rel_z é 0 (DOWN) ou 5 (UP)

                # Verifica se o ponto é alcançável
                ok, msg = check_reachable(abs_x, abs_y, abs_z)
                if not ok:
                    messagebox.showerror("Erro na Escrita", f"Caractere '{char}' em posição inalcançável! {msg}")
                    self.command_queue.clear()
                    return
                
                # Adiciona o comando formatado à fila
                cmd = f"MOV {abs_x:.2f} {abs_y:.2f} {abs_z:.2f} {v} {elbow}"
                self.command_queue.append(cmd)
            
            # Move o "cursor" para a próxima letra
            current_x_offset += LETTER_WIDTH + LETTER_SPACING

        if not self.command_queue:
            self.status.set("Nada para escrever.")
            return

        # Passo 2: Iniciar a sequência
        self.is_busy = True
        self.disable_controls()
        self.status.set(f"Iniciando escrita... {len(self.command_queue)} movimentos.")
        
        # Envia o PRIMEIRO comando. O resto é tratado por process_message
        self.process_queue() 

    def process_queue(self):
        """
        Pega o próximo comando da fila e o envia para o Arduino.
        Esta função é chamada pelo write_text_cmd (para o primeiro comando)
        e pelo process_message (para todos os comandos subsequentes).
        """
        if not self.command_queue:
            # Fila está vazia, mas isso será tratado pelo process_message
            # quando o "NOTIFY" do último comando chegar.
            print("Process_queue chamada, mas fila vazia. Aguardando NOTIFY final.")
            return
        
        # Pega o próximo comando e o envia
        cmd = self.command_queue.popleft()
        print(f"Enviando (Sequencia): {cmd}")
        
        try:
            self.ser.write((cmd + "\n").encode())
        except Exception as e:
            messagebox.showerror("Erro Serial", f"Falha ao enviar comando: {e}")
            self.disconnect()


if __name__ == "__main__":
    root = tk.Tk()
    app = App(root)
    
    def on_closing():
        if app.ser and app.ser.is_open:
            app.ser.close()
        root.destroy()
    
    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.mainloop()