#include <SoftwareSerial.h>

// --- Comunicação com Escravo ---
SoftwareSerial SlaveSerial(10, 11); // RX, TX
#define SLAVE_BAUD 9600

// --- MODO DE TESTE ---
// Mude para 'false' quando conectar o Arduino Escravo real
const bool DEBUG_NO_SLAVE = true;

// --- Parâmetros Físicos ---
const float L1 = 110.0;
const float L2 = 80.0;
const float L3 = 90.0;

// --- Parâmetros de Motor ---
const int NSTEP = 200;
const int MICROSTEP = 16;
const float R1 = 1.0;
const float R2 = 1.0;
const float R3 = 1.0;

// ângulos atuais (em radianos)
float theta_current[3] = {0, 0, 0};

void setup() {
  Serial.begin(115200);      // Comunicação com Python
  SlaveSerial.begin(SLAVE_BAUD); // Comunicação com Escravo
  
  Serial.println("NNotify: Mestre pronto!");
}

void loop() {
  if (Serial.available()) {
    processPythonCommand();
  }

  // Em modo de debug, não precisamos escutar o escravo
  if (!DEBUG_NO_SLAVE) {
    if (SlaveSerial.available()) {
      processSlaveAck();
    }
  }
}

// --- Processa comandos vindos do Python ---
void processPythonCommand() {
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();

  // --- COMANDO MOV (Cinemática) ---
  if (cmd.startsWith("MOV")) {
    cmd.replace("MOV", "");
    cmd.trim();

    float x=0, y=0, z=0; int v=0; char elbow='U';
    int idx1 = cmd.indexOf(' ');
    int idx2 = cmd.indexOf(' ', idx1 + 1);
    int idx3 = cmd.indexOf(' ', idx2 + 1);
    int idx4 = cmd.indexOf(' ', idx3 + 1);
    
    if (idx1 > 0 && idx2 > 0 && idx3 > 0) {
      x = cmd.substring(0, idx1).toFloat();
      y = cmd.substring(idx1 + 1, idx2).toFloat();
      z = cmd.substring(idx2 + 1, idx3).toFloat();
      v = cmd.substring(idx3 + 1, idx4).toInt();
      if (idx4 > 0 && idx4 < cmd.length()) elbow = cmd.substring(idx4 + 1).charAt(0);
    }

    Serial.print("Parsed -> X="); Serial.print(x);
    Serial.print(" Y="); Serial.print(y);
    Serial.print(" Z="); Serial.print(z);
    Serial.print(" V="); Serial.print(v);
    Serial.print(" ELBOW="); Serial.println(elbow);

    float t1, t2, t3;
    if (!inverseKinematics(x, y, z, elbow, t1, t2, t3)) {
      Serial.println("ERR: Posicao inalcancavel");
      return; 
    }
    
    float th_deg[3] = {t1*180.0/PI, t2*180.0/PI, t3*180.0/PI};
    Serial.print("ANGLES ");
    Serial.print(th_deg[0]); Serial.print(" ");
    Serial.print(th_deg[1]); Serial.print(" ");
    Serial.println(th_deg[2]);

    float dt_deg[3];
    dt_deg[0] = th_deg[0] - theta_current[0] * 180.0 / PI;
    dt_deg[1] = th_deg[1] - theta_current[1] * 180.0 / PI;
    dt_deg[2] = th_deg[2] - theta_current[2] * 180.0 / PI;

    long S1 = round((float)NSTEP * MICROSTEP * R1 * dt_deg[0] / 360.0);
    long S2 = round((float)NSTEP * MICROSTEP * R2 * dt_deg[1] / 360.0);
    long S3 = round((float)NSTEP * MICROSTEP * R3 * dt_deg[2] / 360.0);

    Serial.print("PASSOS -> S1="); Serial.print(S1);
    Serial.print(" S2="); Serial.print(S2);
    Serial.print(" S3="); Serial.println(S3);

    Serial.println("ACK: Comando recebido");
    int G = 0; // 0 = Sem ação da garra
    sendStepToSlave(S1, S2, S3, G, v);

    // MODO DEBUG: Simula resposta do escravo
    if (DEBUG_NO_SLAVE) {
      Serial.println("NOTIFY: Movimento concluido (DEBUG)");
    }

    theta_current[0] = t1;
    theta_current[1] = t2;
    theta_current[2] = t3;
  }

  // --- COMANDO GRIP ---
  else if (cmd.startsWith("GRIP")) {
    int G = 0;
    
    // MUDANÇA: Adiciona log "Parsed"
    Serial.print("Parsed -> CMD=GRIP, Action=");
    if (cmd.indexOf("OPEN") > 0) {
      G = 1; // 1 = Abrir
      Serial.println("OPEN");
    } else if (cmd.indexOf("CLOSE") > 0) {
      G = 2; // 2 = Fechar
      Serial.println("CLOSE");
    } else {
      Serial.println("NONE");
    }

    if (G > 0) {
      Serial.println("ACK: Comando recebido");
      int v = 50; // Velocidade padrão da garra

      // MUDANÇA: Adiciona log "PASSOS" para GRIP
      Serial.print("PASSOS -> S1=0 S2=0 S3=0 G="); Serial.println(G);
      
      sendStepToSlave(0, 0, 0, G, v); 

      // MODO DEBUG: Simula resposta do escravo
      if (DEBUG_NO_SLAVE) {
        Serial.println("NOTIFY: Movimento concluido (DEBUG)");
      }
    }
  }

  // --- COMANDO HOME ---
  else if (cmd.startsWith("HOME")) {
    // MUDANÇA: Adiciona log "Parsed"
    Serial.println("Parsed -> CMD=HOME");
    Serial.println("ACK: Comando recebido");

    // MUDANÇA: Calcular passos para voltar ao 'home' (0,0,0) para log
    float dt_deg[3];
    dt_deg[0] = 0.0 - theta_current[0] * 180.0 / PI;
    dt_deg[1] = 0.0 - theta_current[1] * 180.0 / PI;
    dt_deg[2] = 0.0 - theta_current[2] * 180.0 / PI;

    long S1 = round((float)NSTEP * MICROSTEP * R1 * dt_deg[0] / 360.0);
    long S2 = round((float)NSTEP * MICROSTEP * R2 * dt_deg[1] / 360.0);
    long S3 = round((float)NSTEP * MICROSTEP * R3 * dt_deg[2] / 360.0);

    Serial.print("PASSOS (para Home) -> S1="); Serial.print(S1);
    Serial.print(" S2="); Serial.print(S2);
    Serial.print(" S3="); Serial.println(S3);
    // Fim da MUDANÇA

    String payload = "HOME_CMD";
    int chk = 0;
    for (int i=0; i<payload.length(); i++) chk = (chk + payload[i]) & 0xFF;
    payload += " " + String(chk);
    
    SlaveSerial.println(payload);
    
    Serial.print("TO_SLAVE: "); Serial.println(payload);

    // MODO DEBUG: Simula resposta e reseta ângulos
    if (DEBUG_NO_SLAVE) {
      theta_current[0] = 0;
      theta_current[1] = 0;
      theta_current[2] = 0;
      // MUDANÇA: Adiciona log "ANGLES" para HOME
      Serial.println("ANGLES -> T1=0.00 T2=0.00 T3=0.00 (Reset)");
      Serial.println("NOTIFY: Movimento concluido (DEBUG_HOME)");
    }
  }
}

// --- Processa ACKs vindos do Escravo ---
void processSlaveAck() {
  String ack = SlaveSerial.readStringUntil('\n');
  ack.trim();

  if (ack.equals("ACK_STEP_DONE")) {
    Serial.println("NOTIFY: Movimento concluido");
  }

  if (ack.equals("ACK_HOME_DONE")) {
    theta_current[0] = 0;
    theta_current[1] = 0;
    theta_current[2] = 0;
    // MUDANÇA: Adiciona log "ANGLES" para HOME (quando não está em debug)
    Serial.println("ANGLES -> T1=0.00 T2=0.00 T3=0.00 (Reset)");
    Serial.println("NOTIFY: Movimento concluido");
  }
}

// --- Função Helper para enviar comando STEP ao Escravo ---
void sendStepToSlave(long S1, long S2, long S3, int G, int v) {
  int chk = 0;
  String payload = String("STEP ") + S1 + " " + S2 + " " + S3 + " " + G + " " + v;

  for (int i=0; i<payload.length(); i++) chk = (chk + payload[i]) & 0xFF;
  payload += " " + String(chk);

  // Envia para o escravo
  SlaveSerial.println(payload);

  Serial.print("TO_SLAVE: "); Serial.println(payload);
}


// --- Cinemática Inversa ---
bool inverseKinematics(float x, float y, float z, char elbow, float &t1, float &t2, float &t3) {
  t1 = atan2(y, x);
  float r = sqrt(x * x + y * y);
  float z_ = z - L1;
  float cos3 = (r * r + z_ * z_ - L2 * L2 - L3 * L3) / (2 * L2 * L3);

  Serial.print("r="); Serial.print(r);
  Serial.print(" z'="); Serial.print(z_);
  Serial.print(" cos3="); Serial.println(cos3);

  if (cos3 > 1.0 || cos3 < -1.0) {
    return false;
  }

  if (cos3 < -1.0) cos3 = -1.0;
  if (cos3 > 1.0)  cos3 = 1.0;
  
  t3 = acos(cos3);
  if (elbow == 'D') t3 = -t3;
  t2 = atan2(z_, r) - atan2(L3 * sin(t3), L2 + L3 * cos(t3));
  
  return true; 
}