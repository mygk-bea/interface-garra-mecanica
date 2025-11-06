import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import threading
import time

class App:
    def __init__(self, root):
        self.root = root
        root.title("Controle Braço Robô")
        frame = ttk.Frame(root, padding=10)
        frame.grid()

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

        ttk.Button(frame, text="Mover", command=self.move_cmd).grid(row=6,column=0)
        ttk.Button(frame, text="Abrir Garra", command=self.grip_open).grid(row=6,column=1)
        ttk.Button(frame, text="Fechar Garra", command=self.grip_close).grid(row=6,column=2)
        ttk.Button(frame, text="Home", command=self.home_cmd).grid(row=7,column=0)

        self.status = tk.StringVar(value="Desconectado")
        ttk.Label(frame, textvariable=self.status).grid(row=8, column=0, columnspan=3)

        self.ser = None

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
            threading.Thread(target=self.read_loop, daemon=True).start()
        except Exception as e:
            messagebox.showerror("Erro", str(e))

    def read_loop(self):
        while self.ser and self.ser.is_open:
            try:
                raw = self.ser.read_all().decode(errors='ignore')
                if raw.strip():
                    for line in raw.splitlines():
                        line = line.strip()
                        if line:
                            print("Arduino:", line)
            except Exception as e:
                print("Erro leitura:", e)
            time.sleep(0.05)

    def send(self, s):
        if not self.ser or not self.ser.is_open:
            messagebox.showerror("Erro","Não conectado")
            return
        print("Enviando:", s)
        self.ser.write((s+"\n").encode())

    def move_cmd(self):
        x = self.x_e.get(); y = self.y_e.get(); z = self.z_e.get(); v = self.v_e.get()
        try:
            float(x); float(y); float(z); int(v)
        except:
            messagebox.showerror("Erro","Valores inválidos")
            return
        cmd = f"MOV {x} {y} {z} {v} {self.elbow.get()}"
        self.send(cmd)

    def grip_open(self): self.send("GRIP OPEN")
    def grip_close(self): self.send("GRIP CLOSE")
    def home_cmd(self): self.send("HOME")

if __name__ == "__main__":
    root = tk.Tk()
    app = App(root)
    root.mainloop()
