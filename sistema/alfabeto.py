import numpy as np

# NOVO: Função auxiliar para gerar pontos em um arco de círculo
def gerar_arco(centro_x, centro_y, raio_x, raio_y, angulo_inicio, angulo_fim, num_segmentos):
    """Gera pontos em formato de elipse/círculo para suavizar curvas."""
    pontos = []
    angulos = np.linspace(np.radians(angulo_inicio), np.radians(angulo_fim), num_segmentos)
    for angulo in angulos:
        x = centro_x + raio_x * np.cos(angulo)
        y = centro_y + raio_y * np.sin(angulo)
        pontos.append((round(x, 3), round(y, 3)))
    return pontos

# Usamos 25 segmentos (24 passos) para garantir a suavidade das curvas
NUM_SEGMENTS_CURVE = 25 

# Geração do traço completo para 'O' (usando 360 graus)
pontos_circulo = gerar_arco(0.5, 0.5, 0.5, 0.5, 0, 360, NUM_SEGMENTS_CURVE)
# Inicia no ponto mais baixo (X=0.5, Y=0) e usa os pontos gerados
O_STROKE = [(0.5, 0.0)] + pontos_circulo 

# Geração de traços para 'C' (usando 270 graus)
# Centro: (0.5, 0.5), Raio: 0.5
C_CURVE = gerar_arco(0.5, 0.5, 0.5, 0.5, 45, 315, NUM_SEGMENTS_CURVE)


font = {
    'A': [
        [(0.0, 0.0), (0.5, 1.0), (1.0, 0.0)],
        [(0.25, 0.5), (0.75, 0.5)]           
    ],
    'B': [
        # Haste vertical
        [(0.0, 0.0), (0.0, 1.0)], 
        # Curva superior (de 1.0 a 0.5)
        [(0.0, 1.0)] + gerar_arco(0.5, 0.75, 0.5, 0.25, 90, 0, 10) + [(0.0, 0.5)],
        # Curva inferior (de 0.5 a 0.0)
        [(0.0, 0.5)] + gerar_arco(0.5, 0.25, 0.5, 0.25, 180, 270, 10) + [(0.0, 0.0)]
    ],
    'C': [
        # Utiliza a curva suave
        [(1.0, 0.5)] + C_CURVE + [(1.0, 0.5)]
    ],
    'L': [
        [(0.0, 1.0), (0.0, 0.0), (1.0, 0.0)] 
    ],
    'O': [
        # CORREÇÃO: Utiliza o traço de alta resolução
        O_STROKE 
    ],
    'X': [
        [(0.0, 1.0), (1.0, 0.0)], 
        [(1.0, 1.0), (0.0, 0.0)]
    ],
    ' ': [] 
}

def get_char(char):
    """Retorna a lista de traços para o caractere, ou uma lista vazia se não for encontrado."""
    return font.get(char.upper(), [])