import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import threading
import time
import math

# --- Constantes físicas do braço (em mm) ---
L1 = 110.0
L2 = 80.0
L3 = 90.0
R_MAX = L2 + L3
R_MIN = abs(L2 - L3)

def check_reachable(x, y, z):
    r = math.hypot(x, y)
    zprime = z - L1
    d = math.hypot(r, zprime)
    if d < (R_MIN - 0.1) or d > (R_MAX + 0.1): # Pequena margem de tolerância
        return False, f"Fora de alcance: distância total={d:.1f} mm (permitido entre {R_MIN:.1f} e {R_MAX:.1f} mm)"
    return True, f"OK (r={r:.1f}, z'={zprime:.1f}, d={d:.1f})"

class App:
    def __init__(self, root):
        self.root = root
        root.title("Controle Braço Robô")
        frame = ttk.Frame(root, padding=10)
        frame.grid()

        # --- Estado da Aplicação ---
        self.ser = None
        self.is_busy = False # MUITO IMPORTANTE: Trava para handshaking

        # --- Widgets ---
        ttk.Label(frame, text="Porta:").grid(row=0, column=0)
        self.port_cb = ttk.Combobox(frame, values=self.list_ports(), width=15)
        self.port_cb.grid(row=0, column=1)
        self.b_connect = ttk.Button(frame, text="Conectar", command=self.connect)
        self.b_connect.grid(row=0, column=2)

        ttk.Label(frame, text="X (mm)").grid(row=1, column=0)
        self.x_e = ttk.Entry(frame); self.x_e.grid(row=1, column=1)
        ttk.Label(frame, text="Y (mm)").grid(row=2, column=0)
        self.y_e = ttk.Entry(frame); self.y_e.grid(row=2, column=1)
        ttk.Label(frame, text="Z (mm)").grid(row=3, column=0)
        self.z_e = ttk.Entry(frame); self.z_e.grid(row=3, column=1)

        ttk.Label(frame, text="Vel (%)").grid(row=4, column=0)
        self.v_e = ttk.Entry(frame); self.v_e.grid(row=4, column=1); self.v_e.insert(0,"80")

        self.elbow = tk.StringVar(value="U")
        ttk.Radiobutton(frame, text="Elbow Up", variable=self.elbow, value="U").grid(row=5,column=0)
        ttk.Radiobutton(frame, text="Elbow Down", variable=self.elbow, value="D").grid(row=5,column=1)

        # Armazena os botões como 'self' para poder desabilitá-los
        self.b_move = ttk.Button(frame, text="Mover", command=self.move_cmd)
        self.b_move.grid(row=6,column=0)
        self.b_grip_open = ttk.Button(frame, text="Abrir Garra", command=self.grip_open)
        self.b_grip_open.grid(row=6,column=1)
        self.b_grip_close = ttk.Button(frame, text="Fechar Garra", command=self.grip_close)
        self.b_grip_close.grid(row=6,column=2)
        self.b_home = ttk.Button(frame, text="Home", command=self.home_cmd)
        self.b_home.grid(row=7,column=0)

        self.status = tk.StringVar(value="Desconectado")
        ttk.Label(frame, textvariable=self.status).grid(row=8, column=0, columnspan=3)

    def list_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        return ports

    def connect(self):
        port = self.port_cb.get()
        if not port:
            messagebox.showerror("Erro", "Selecione a porta serial")
            return
        try:
            # Timeout baixo (0.1s) para não travar a leitura
            self.ser = serial.Serial(port, 115200, timeout=0.1)
            self.status.set(f"Conectado: {port}")
            self.b_connect.config(text="Desconectar", command=self.disconnect)
            # Inicia a thread de leitura
            self.read_thread = threading.Thread(target=self.read_loop, daemon=True)
            self.read_thread.start()
        except Exception as e:
            messagebox.showerror("Erro", str(e))

    def disconnect(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.ser = None
        self.status.set("Desconectado")
        self.b_connect.config(text="Conectar", command=self.connect)
        self.is_busy = False # Reseta o estado
        self.enable_controls() # Garante que os controles estão ativos

    def read_loop(self):
        """
        Loop de leitura em uma thread separada.
        NUNCA atualize a GUI diretamente daqui. Use self.root.after_idle().
        """
        buffer = ""
        while self.ser and self.ser.is_open:
            try:
                # Lê os dados que chegarem
                raw = self.ser.read_all().decode(errors='ignore')
                if raw:
                    buffer += raw
                    # Processa linha por linha
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        line = line.strip()
                        if line:
                            # Envia a linha para ser processada na thread principal da GUI
                            self.root.after_idle(self.process_message, line)
            except serial.SerialException:
                # Erro comum quando desconecta
                break
            except Exception as e:
                print("Erro leitura:", e)
            time.sleep(0.05) # Pequena pausa para não sobrecarregar
        
        # Se saiu do loop, provavelmente desconectou
        if self.ser: # Evita chamar after_idle se o root já foi destruído
            self.root.after_idle(self.disconnect)

    def process_message(self, msg):
        """
        Esta função é chamada pela self.root.after_idle()
        e roda na thread principal. É SEGURO atualizar a GUI daqui.
        """
        print("Arduino:", msg) # Log no console

        if msg.startswith("ACK:"):
            self.status.set("Movimentando...")
            # Não faz mais nada, só aguarda o "NOTIFY"

        elif msg.startswith("NOTIFY: Movimento concluido"):
            self.is_busy = False
            self.enable_controls()
            self.status.set("Pronto.")

        elif msg.startswith("ERR:"):
            # Ocorreu um erro no Arduino (ex: inalcançável)
            error_msg = msg.replace("ERR:", "").strip()
            messagebox.showerror("Erro do Arduino", error_msg)
            self.is_busy = False
            self.enable_controls()
            self.status.set(f"Erro: {error_msg}")

        elif msg.startswith("NNotify:"): # Mensagens de Notificação (ex: Mestre pronto!)
            self.status.set(msg.replace("NNotify:", "").strip())
            
        elif msg.startswith("Parsed") or msg.startswith("ANGLES") or msg.startswith("PASSOS"):
            # Apenas log de debug do Arduino, não atualiza status principal
            pass 

    def send(self, s):
        if not self.ser or not self.ser.is_open:
            messagebox.showerror("Erro","Não conectado")
            return False
        
        if self.is_busy:
            messagebox.showwarning("Ocupado", "Aguarde o movimento anterior terminar.")
            return False

        # Trava a interface ANTES de enviar
        self.is_busy = True
        self.disable_controls()
        
        print("Enviando:", s)
        self.ser.write((s+"\n").encode())
        return True

    def disable_controls(self):
        self.b_move.config(state="disabled")
        self.b_grip_open.config(state="disabled")
        self.b_grip_close.config(state="disabled")
        self.b_home.config(state="disabled")

    def enable_controls(self):
        self.b_move.config(state="normal")
        self.b_grip_open.config(state="normal")
        self.b_grip_close.config(state="normal")
        self.b_home.config(state="normal")

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
            print("ALERTA:", msg)
            return

        cmd = f"MOV {x} {y} {z} {v} {self.elbow.get()}"
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

if __name__ == "__main__":
    root = tk.Tk()
    app = App(root)
    # Garante que a desconexão serial ocorra ao fechar a janela
    def on_closing():
        if app.ser and app.ser.is_open:
            app.ser.close()
        root.destroy()
    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.mainloop()