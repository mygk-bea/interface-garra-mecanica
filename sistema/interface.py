import tkinter as tk
from tkinter import ttk
import tkinter.font as tkfont
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt

COR_PRIMARIA = '#188181'
COR_SECUNDARIA = '#FFD166'
COR_ERRO = '#b94444'
COR_FUNDO = '#181818'
COR_FUNDO_WIDGET = '#222222'
COR_TEXTO = '#EDEDED'
INPUT_BG = '#151515'
DISABLED_BG = '#555555'

def decorative_label(master, text, fg=COR_PRIMARIA, bg=COR_FUNDO, height=3, pad_top=6, pad_bottom=6):
    """Função auxiliar para criar rótulos decorativos."""
    wrap = tk.Frame(master, bg=bg)
    c = tk.Canvas(wrap, height=height+pad_top+pad_bottom, bg=bg, highlightthickness=0)
    c.pack(fill='x', expand=True)
    font = ("Segoe UI", 11, "bold")
    tkfont_obj = tkfont.Font(font=font)
    def redraw(event=None):
        c.delete("all")
        w = c.winfo_width()
        h = c.winfo_height()
        y = h // 2
        txt_w = tkfont_obj.measure(text)
        pad = 18
        left_x2 = max(12, (w - txt_w)//2 - pad)
        right_x1 = min(w-12, (w + txt_w)//2 + pad)
        line_thickness = 3
        c.create_line(8, y, left_x2, y, width=line_thickness, fill=fg)
        c.create_line(right_x1, y, w-8, y, width=line_thickness, fill=fg)
        c.create_rectangle(left_x2-8, y-16, right_x1+8, y+16, fill=bg, outline=bg)
        c.create_text(w//2, y, text=text, fill=COR_TEXTO, font=font)
    c.bind("<Configure>", redraw)
    return wrap

def criar_interface(root, callbacks, initial_coord):
    """
    Cria todos os widgets da interface e retorna um dicionário com referências.
    """
    widgets = {}
    
    root.configure(bg=COR_FUNDO)
    root.state("zoomed") 
    root.minsize(1024, 768) 
    
    style = ttk.Style()
    default_font = ("Segoe UI", 10)
    style.configure(".", background=COR_FUNDO, foreground=COR_TEXTO, font=default_font)
    style.configure("Card.TLabelframe", background=COR_FUNDO_WIDGET, foreground=COR_PRIMARIA, borderwidth=0)
    style.configure("Card.TLabelframe.Label", background=COR_FUNDO_WIDGET, foreground=COR_PRIMARIA, font=("Segoe UI", 11, "bold"))

    # --- Estrutura de Scroll ---
    container = tk.Frame(root, bg=COR_FUNDO)
    container.pack(fill="both", expand=True)

    canvas_scroll = tk.Canvas(container, bg=COR_FUNDO, highlightthickness=0)
    canvas_scroll.pack(side="left", fill="both", expand=True)
    widgets["canvas_scroll"] = canvas_scroll # Armazena para callbacks

    scrollbar = tk.Scrollbar(container, orient="vertical", command=canvas_scroll.yview)
    scrollbar.pack(side="right", fill="y")
    canvas_scroll.configure(yscrollcommand=scrollbar.set)

    content_frame = tk.Frame(canvas_scroll, bg=COR_FUNDO)
    canvas_window = canvas_scroll.create_window((0, 0), window=content_frame, anchor="nw")
    widgets["content_frame"] = content_frame # Armazena para grid

    # Handlers de scroll
    def resize_content(event):
        canvas_scroll.itemconfig(canvas_window, width=event.width)
    def update_scroll(event):
        canvas_scroll.configure(scrollregion=canvas_scroll.bbox("all"))
    def _on_mousewheel(event):
        canvas_scroll.yview_scroll(int(-1*(event.delta/120)), "units") 

    content_frame.bind("<Configure>", update_scroll)
    canvas_scroll.bind("<Configure>", resize_content)
    canvas_scroll.bind_all("<MouseWheel>", _on_mousewheel)

    # Configuração de Grid principal
    for i in range(5): content_frame.grid_columnconfigure(i, weight=1)
    content_frame.grid_columnconfigure(4, weight=6)
    for r in range(7): content_frame.grid_rowconfigure(r, weight=0)
    content_frame.grid_rowconfigure(4, weight=1) 

    # ---------- Título principal ----------
    title_lbl = tk.Label(content_frame, text="CONTROLADORA DO BRAÇO MECÂNICO",
    font=("Segoe UI", 26, "bold"), fg=COR_PRIMARIA, bg=COR_FUNDO)
    title_lbl.grid(row=0, column=0, columnspan=5, pady=(22, 12), padx=(20), sticky='w')

    # ---------- 1. Motores Individuais (1x4 cards) ----------
    frame_motores = tk.Frame(content_frame, bg=COR_FUNDO)
    frame_motores.grid(row=1, column=0, columnspan=4, sticky='nsew', padx=18, pady=(8,10))
    decor = decorative_label(frame_motores, "Motores Individuais")
    decor.pack(fill='x', pady=(0,10))

    inner_motors = tk.Frame(frame_motores, bg=COR_FUNDO)
    inner_motors.pack(fill='both', expand=True)
    for c in range(4): inner_motors.columnconfigure(c, weight=1) 

    sentido_var_list, passos_entry_list, delay_entry_list = [], [], []
    for i in range(4):
        card = tk.Frame(inner_motors, bg=COR_FUNDO_WIDGET, bd=0)
        card.grid(row=0, column=i, padx=10, pady=5, sticky='nsew') 

        # Título do Motor
        lbl = tk.Label(card, text=f"Motor {i+1}", bg=COR_FUNDO_WIDGET, fg=COR_TEXTO, font=("Segoe UI", 10, "bold"))
        lbl.pack(anchor='w', padx=12, pady=(10,4))

        # Radio Buttons Verticalizados
        sentido_var = tk.StringVar(value='H')
        sentido_var_list.append(sentido_var)
        rbf = tk.Frame(card, bg=COR_FUNDO_WIDGET)
        rbf.pack(anchor='w', padx=12)
        ttk.Radiobutton(rbf, text="Horário", variable=sentido_var, value='H').pack(anchor='w', pady=1) 
        ttk.Radiobutton(rbf, text="Anti-horário", variable=sentido_var, value='A').pack(anchor='w', pady=1)

        # Entradas de Passos e Delay
        tk.Label(card, text="Passos:", bg=COR_FUNDO_WIDGET, fg=COR_TEXTO).pack(anchor='w', padx=12, pady=(10,2))
        e_pass = tk.Entry(card, bg=INPUT_BG, fg="white", insertbackground="white", relief="flat", highlightthickness=2, highlightbackground="#333333", highlightcolor="#333333", bd=0, font=("Segoe UI", 10),)
        e_pass.pack(anchor='w', padx=12, ipady=5, fill='x', expand=True)
        passos_entry_list.append(e_pass)

        tk.Label(card, text="Delay (ms):", bg=COR_FUNDO_WIDGET, fg=COR_TEXTO).pack(anchor='w', padx=12, pady=(8,2))
        e_delay = tk.Entry(card, bg=INPUT_BG, fg="white", insertbackground="white", relief="flat", highlightthickness=2, highlightbackground="#333333", highlightcolor="#333333", bd=0, font=("Segoe UI", 10),)
        e_delay.pack(anchor='w', padx=12, ipady=5, pady=(0,8), fill='x', expand=True)
        e_delay.insert(0, "10") 
        delay_entry_list.append(e_delay)

        # Botões Verticalizados
        btn_f = tk.Frame(card, bg=COR_FUNDO_WIDGET)
        btn_f.pack(fill='x', padx=12, pady=(6,12))
        tk.Button(btn_f, text="Enviar", command=lambda m=i: callbacks["comando_motor"](m), font=("Segoe UI", 10, "bold"), bg=COR_PRIMARIA, fg=COR_TEXTO, activebackground="#444444", activeforeground=COR_FUNDO, borderwidth=0, relief="flat", padx=8, pady=4).pack(fill='x', pady=(0, 4))
        tk.Button(btn_f, text="Parar", command=lambda m=i: callbacks["parar_motor"](m), font=("Segoe UI", 10, "bold"), bg=COR_ERRO, fg=COR_TEXTO, activebackground="#C9302C", activeforeground=COR_FUNDO, borderwidth=0, relief="flat", padx=8, pady=4).pack(fill='x')

    # Armazena as listas de variáveis e entradas
    widgets["sentido_var_list"] = sentido_var_list
    widgets["passos_entry_list"] = passos_entry_list
    widgets["delay_entry_list"] = delay_entry_list

    # ---------- 2. Executar Testes (linha de botões) ----------
    frame_testes = tk.Frame(content_frame, bg=COR_FUNDO)
    frame_testes.grid(row=2, column=0, columnspan=4, sticky='ew', padx=18, pady=(4,8))
    decor_testes = decorative_label(frame_testes, "Executar Testes (5° por junta)")
    decor_testes.pack(fill='x', pady=(0,15))
    btns_testes_frame = tk.Frame(frame_testes, bg=COR_FUNDO)
    btns_testes_frame.pack(fill='x', padx=8)
    for i in range(4):
        tk.Button(btns_testes_frame, text=f"Junta {i+1}", command=lambda j=i: callbacks["mover_junta_temp"](j), 
            font=("Segoe UI", 10, "bold"), bg=COR_PRIMARIA, fg=COR_TEXTO, activebackground="#444444",
            activeforeground=COR_FUNDO, borderwidth=0, relief="flat", padx=8, pady=4).pack(side='left', expand=True, fill='x', padx=8)

    tk.Button(btns_testes_frame, text="CALIBRAR Z (Papel)", command=callbacks["calibrar_z_callback"],
        font=("Segoe UI", 10, "bold"), bg=COR_SECUNDARIA, fg=COR_FUNDO, activebackground=COR_FUNDO_WIDGET, 
        activeforeground=COR_TEXTO, borderwidth=0, relief="flat", padx=8, pady=4).pack(side='right', padx=8)

    # ---------- 3. Digite um texto (Comando atualizado) ----------
    frame_texto = tk.Frame(content_frame, bg=COR_FUNDO)
    frame_texto.grid(row=3, column=0, columnspan=4, sticky='ew', padx=18, pady=(4,8))
    decor_texto = decorative_label(frame_texto, "Envio de Texto (Escrita Vetorial)")
    decor_texto.pack(fill='x', pady=(10, 10))
    
    # Container para texto e escala
    txt_scale_inner = tk.Frame(frame_texto, bg=COR_FUNDO)
    txt_scale_inner.pack(fill='x', padx=10, pady=5)

    # Campo de texto (esquerda)
    txt_frame = tk.Frame(txt_scale_inner, bg=COR_FUNDO)
    txt_frame.pack(side='left', fill='x', expand=True, padx=(0, 10))
    tk.Label(txt_frame, text="Texto:", bg=COR_FUNDO, fg=COR_TEXTO).pack(side='left', padx=(0, 5))
    entry_texto = tk.Entry(txt_frame, bg=INPUT_BG, fg="white", insertbackground="white", relief="flat", highlightthickness=2, highlightbackground="#333333", highlightcolor="#333333", bd=0, font=("Segoe UI", 10),)
    entry_texto.pack(side='left', fill='x', expand=True, ipady=5)
    widgets["entry_texto"] = entry_texto

    # Campo de escala (direita)
    scale_frame = tk.Frame(txt_scale_inner, bg=COR_FUNDO)
    scale_frame.pack(side='left', padx=(10, 0))
    tk.Label(scale_frame, text="Escala (cm):", bg=COR_FUNDO, fg=COR_TEXTO).pack(side='left', padx=(0, 5))
    entry_scale = tk.Entry(scale_frame, width=5, bg=INPUT_BG, fg="white", insertbackground="white", relief="flat", highlightthickness=2, highlightbackground="#333333", highlightcolor="#333333", bd=0, font=("Segoe UI", 10),)
    entry_scale.insert(0, "3.0")
    entry_scale.pack(side='left', ipady=5)
    widgets["entry_scale"] = entry_scale
    
    # Botão de Escrita
    tk.Button(txt_scale_inner, text="Escrever!", command=callbacks["escrever_texto"], 
        font=("Segoe UI", 10, "bold"), bg=COR_PRIMARIA, fg=COR_TEXTO, activebackground="#444444", activeforeground=COR_FUNDO, borderwidth=0, relief="flat", padx=8, pady=4).pack(side='right', padx=(10, 0))

    # ---------- 4. Log (Log_text definido aqui) ----------
    frame_log = tk.Frame(content_frame, bg=COR_FUNDO)
    frame_log.grid(row=4, column=0, columnspan=4, sticky='nsew', padx=18, pady=(5,10))
    decor_log = decorative_label(frame_log, "Log")
    decor_log.pack(fill='x', pady=(0,8))
    log_box = tk.Frame(frame_log, bg=COR_FUNDO)
    log_box.pack(fill='both', expand=True, padx=10)

    log_scrollbar = tk.Scrollbar(log_box) 
    log_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

    log_text = tk.Text(log_box, height=20, bg=COR_FUNDO_WIDGET, fg=COR_TEXTO, 
                        insertbackground=COR_TEXTO, relief=tk.FLAT, yscrollcommand=log_scrollbar.set)
    log_text.pack(fill='both', expand=True)
    log_scrollbar.config(command=log_text.yview)
    widgets["log_text"] = log_text

    # ---------- 5. Painel 3D (direita) ----------
    frame_plot = tk.Frame(content_frame, bg=COR_FUNDO_WIDGET)
    frame_plot.grid(row=1, column=4, rowspan=4, sticky='nsew', padx=18, pady=12)
    frame_plot.grid_rowconfigure(0, weight=1)
    frame_plot.grid_columnconfigure(0, weight=1)
    plot_decor = decorative_label(frame_plot, "Modelo Tridimensional")
    plot_decor.pack(fill='x', pady=(0,10))

    fig = plt.figure(figsize=(10,5)) 
    fig.patch.set_facecolor(COR_FUNDO_WIDGET)
    ax = fig.add_subplot(111, projection='3d')
    canvas_plot = FigureCanvasTkAgg(fig, master=frame_plot) 
    canvas_plot.get_tk_widget().pack(fill='both', expand=True, padx=12, pady=(0,8))
    widgets["ax"] = ax
    widgets["canvas_plot"] = canvas_plot

    label_coord = tk.Label(frame_plot, text=f"Posição Atual: X = {initial_coord[0]:.1f} Y = {initial_coord[1]:.1f} Z = {initial_coord[2]:.1f}",
    bg=COR_FUNDO_WIDGET, fg=COR_TEXTO)
    label_coord.pack(pady=(0,20))
    widgets["label_coord"] = label_coord

    # ---------- 6. Coordenadas e botões de movimento (rodapé) ----------
    frame_coord = tk.Frame(content_frame, bg=COR_FUNDO)
    frame_coord.grid(row=5, column=0, columnspan=5, sticky='ew', padx=18, pady=(0,18))
    decor_coord = decorative_label(frame_coord, "Definir Coordenadas")
    decor_coord.pack(fill='x', pady=(0,10))
    coord_inner = tk.Frame(frame_coord, bg=COR_FUNDO)
    coord_inner.pack(fill='x', padx=10)

    for c in range(11): coord_inner.columnconfigure(c, weight=0)
    coord_inner.columnconfigure(1, weight=1); coord_inner.columnconfigure(3, weight=1); coord_inner.columnconfigure(5, weight=1)

    tk.Label(coord_inner, text="X:", bg=COR_FUNDO, fg=COR_TEXTO).grid(row=0, column=0, padx=(0,6), sticky='e')
    entry_x = tk.Entry(coord_inner, width=20, bg=INPUT_BG, fg="white", insertbackground="white", relief="flat", highlightthickness=2, highlightbackground="#333333", highlightcolor="#333333", bd=0, font=("Segoe UI", 10),); entry_x.grid(row=0, column=1, padx=(0,12), sticky='w', pady=5, ipady=4)
    widgets["entry_x"] = entry_x
    tk.Label(coord_inner, text="Y:", bg=COR_FUNDO, fg=COR_TEXTO).grid(row=0, column=2, padx=(0,6), sticky='e')
    entry_y = tk.Entry(coord_inner, width=20, bg=INPUT_BG, fg="white", insertbackground="white", relief="flat", highlightthickness=2, highlightbackground="#333333", highlightcolor="#333333", bd=0, font=("Segoe UI", 10),); entry_y.grid(row=0, column=3, padx=(0,12), sticky='w', pady=5, ipady=4,)
    widgets["entry_y"] = entry_y
    tk.Label(coord_inner, text="Z:", bg=COR_FUNDO, fg=COR_TEXTO).grid(row=0, column=4, padx=(0,6), sticky='e')
    entry_z = tk.Entry(coord_inner, width=20, bg=INPUT_BG, fg="white", insertbackground="white", relief="flat", highlightthickness=2, highlightbackground="#333333", highlightcolor="#333333", bd=0, font=("Segoe UI", 10),); entry_z.grid(row=0, column=5, padx=(0,12), pady=5, ipady=4, sticky='w')
    widgets["entry_z"] = entry_z

    tk.Button(coord_inner, text="Movimento Incremental", command=callbacks["mover_para_coordenada_seguro"], font=("Segoe UI", 10, "bold"), bg=COR_PRIMARIA, fg=COR_TEXTO, activebackground="#444444", activeforeground=COR_FUNDO, borderwidth=0, relief="flat", padx=8, pady=4).grid(row=0, column=6, padx=(20, 8)) 
    tk.Button(coord_inner, text="Movimento Absoluto (fsolve)", command=callbacks["mover_para_absoluto_fsolve"], font=("Segoe UI", 10, "bold"), bg=COR_PRIMARIA, fg=COR_TEXTO, activebackground="#444444", activeforeground=COR_FUNDO, borderwidth=0, relief="flat", padx=8, pady=4).grid(row=0, column=7, padx=(8, 8)) 
    tk.Button(coord_inner, text="Home", command=callbacks["mover_para_home"], font=("Segoe UI", 10, "bold"), bg=COR_ERRO, fg=COR_TEXTO, activebackground="#C9302C", activeforeground=COR_FUNDO, borderwidth=0, relief="flat", padx=8, pady=4).grid(row=0, column=8, padx=(50, 0)) 
    return widgets