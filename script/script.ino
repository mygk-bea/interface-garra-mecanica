void setup() {
  Serial.begin(115200);
  Serial.println("Mestre pronto!");
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd.startsWith("MOV")) {
      // Dividir a string em partes
      cmd.replace("MOV", "");
      cmd.trim();

      float x = 0, y = 0, z = 0;
      int v = 0;
      char elbow = 'U';

      // Divide pelos espaços
      int idx1 = cmd.indexOf(' ');
      int idx2 = cmd.indexOf(' ', idx1 + 1);
      int idx3 = cmd.indexOf(' ', idx2 + 1);
      int idx4 = cmd.indexOf(' ', idx3 + 1);

      if (idx1 > 0 && idx2 > 0 && idx3 > 0) {
        x = cmd.substring(0, idx1).toFloat();
        y = cmd.substring(idx1 + 1, idx2).toFloat();
        z = cmd.substring(idx2 + 1, idx3).toFloat();
        v = cmd.substring(idx3 + 1, idx4).toInt();
        if (idx4 > 0 && idx4 < cmd.length()) {
          elbow = cmd.substring(idx4 + 1).charAt(0);
        }
      }

      Serial.print("Parsed -> X="); Serial.print(x);
      Serial.print(" Y="); Serial.print(y);
      Serial.print(" Z="); Serial.print(z);
      Serial.print(" V="); Serial.print(v);
      Serial.print(" ELBOW="); Serial.println(elbow);

      float t1, t2, t3;
      inverseKinematics(x, y, z, elbow, t1, t2, t3);

      Serial.print("ANGLES ");
      Serial.print(t1 * 180 / PI, 2);
      Serial.print(" ");
      Serial.print(t2 * 180 / PI, 2);
      Serial.print(" ");
      Serial.println(t3 * 180 / PI, 2);

      Serial.println("ACK IK OK");
    }

    else if (cmd.startsWith("GRIP")) {
      Serial.println("ACK GRIP OK");
    }

    else if (cmd.startsWith("HOME")) {
      Serial.println("ACK HOME OK");
    }
  }
}

// Fazer medidas do braço real
const float L1 = 50.0;
const float L2 = 250.0;
const float L3 = 250.0;

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
