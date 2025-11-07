void setup() {
  Serial.begin(115200);
  Serial.println("Mestre pronto!");
}

// parâmetros de motor
const int NSTEP = 200;       // passos por volta (motor sem microstepping)
const int MICROSTEP = 16;    // microstepping do driver (ex: 16)
const float R1 = 1.0;        // relação de engrenagem junta 1
const float R2 = 1.0;        // relação junta 2
const float R3 = 1.0;        // relação junta 3

// ângulos atuais (em radianos)
float theta_current[3] = {0, 0, 0};

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

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

      // === Cinemática inversa ===
      float t1, t2, t3;
      inverseKinematics(x, y, z, elbow, t1, t2, t3);
      float th_deg[3] = {t1*180.0/PI, t2*180.0/PI, t3*180.0/PI};

      Serial.print("ANGLES ");
      Serial.print(th_deg[0]); Serial.print(" ");
      Serial.print(th_deg[1]); Serial.print(" ");
      Serial.println(th_deg[2]);

      // === Conversão para passos ===
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

      // === Monta pacote STEP ===
      int G = 0; // sem garra
      int chk = 0;
      String payload = String("STEP ") + S1 + " " + S2 + " " + S3 + " " + G + " " + v;

      // calcula checksum (soma simples %256)
      for (int i=0; i<payload.length(); i++) chk = (chk + payload[i]) & 0xFF;
      payload += " " + String(chk);

      // envia para o escravo (simulado por enquanto)
      Serial.print("TO SLAVE: "); Serial.println(payload);
      Serial.println("ACK IK OK");

      // atualiza ângulos atuais
      theta_current[0] = t1;
      theta_current[1] = t2;
      theta_current[2] = t3;
    }

    else if (cmd.startsWith("GRIP")) {
      Serial.println("ACK GRIP OK");
    }

    else if (cmd.startsWith("HOME")) {
      Serial.println("ACK HOME OK");
      theta_current[0] = theta_current[1] = theta_current[2] = 0;
    }
  }
}

// Fazer medidas do braço real
const float L1 = 110.0;
const float L2 = 80.0; // ou 0 
const float L3 = 90.0;

bool inverseKinematics(float x, float y, float z, char elbow, float &t1, float &t2, float &t3) {
  t1 = atan2(y, x);
  float r = sqrt(x * x + y * y);
  float z_ = z - L1;
  float cos3 = (r * r + z_ * z_ - L2 * L2 - L3 * L3) / (2 * L2 * L3);

  Serial.print("r="); Serial.print(r);
  Serial.print(" z'="); Serial.print(z_);
  Serial.print(" cos3="); Serial.println(cos3);

  if (cos3 < -1) cos3 = -1;
  if (cos3 > 1)  cos3 = 1;
  t3 = acos(cos3);
  if (elbow == 'D') t3 = -t3;
  t2 = atan2(z_, r) - atan2(L3 * sin(t3), L2 + L3 * cos(t3));
  return true;
}
