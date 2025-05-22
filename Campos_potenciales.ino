#define NUM_LECTURAS 1799
int motor1_pwm = 13;  // PWM para motor 1
int motor1_dir1 = 12; // Dirección motor 1
int motor1_dir2 = 14; // Dirección motor 1

int motor2_pwm = 25;  // PWM para motor 2
int motor2_dir1 = 27; // Dirección motor 2
int motor2_dir2 = 26; // Dirección motor 2

int motor3_pwm = 2;  // PWM para motor 4
int motor3_dir1 = 16; // Dirección motor 4
int motor3_dir2 = 4; // Dirección motor 4

int motor4_pwm = 18;  // PWM para motor 3
int motor4_dir1 = 5; // Dirección motor 3
int motor4_dir2 = 17; // Dirección motor 3

// Parámetros del robot
const float CPR = 22.0;  // Pulsos por revolución
const float r = 0.04;         // Radio de rueda (m)
const float l = 0.0485; //0.0475 //0.0485
const float b = 0.115; //0.1125 //0.115
const float L = l + b;
const float k = 1;          // Ganancia proporcional

// Estado actual
float x = 0, y = 0, theta = 0;
float Vx = 0;
float Vy = 0;
float Wz = 0;

float w_r[4] = {0, 0, 0, 0};  // Velocidades actuales de ruedas
float w_d[4] = {0, 0, 0, 0};
float rpm[4] = {0, 0, 0, 0};
float pwmL[4] = {0, 0, 0, 0};

// Referencia
float xd = 0.0, yd = 0.0, thetad = 0;

// Variables de tiempo
unsigned long lastTime = 0;
unsigned long startTime;

// ----- PID por rueda -----
float Kp = 1, Ki = 0.3, Kd = 0.05; //1 0.3 0.05

float e[4] = {0, 0, 0, 0};
float e_prev[4] = {0, 0, 0, 0};
float e_int[4] = {0, 0, 0, 0};
float control_signal[4] = {0, 0, 0, 0};

// ----- Campos potenciales -----
float Katrac = 0.5;
float Krep = 0.00097;
float Fatracx, Fatracy;
float Qd = 1;
float Frepx, Frepy=0;
float xs[NUM_LECTURAS], ys[NUM_LECTURAS], dist[NUM_LECTURAS];
int num_lecturas_recibidas = 0;

// ----- Encoder -----
const int encA[4] = {33, 35, 21, 36}; // Pines canal A
const int encB[4] = {32, 34, 19, 39}; // Pines canal B

volatile long encoderCount[4] = {0, 0, 0, 0}; // Conteo total
static long countsPrev[4] = {0, 0, 0, 0};


void IRAM_ATTR encoderISR0() {
  bool A = digitalRead(encA[0]);
  bool B = digitalRead(encB[0]);
  
  if (A == B)
    encoderCount[0]++;
  else
    encoderCount[0]--;
}
void IRAM_ATTR encoderISR1() {
  bool A = digitalRead(encA[1]);
  bool B = digitalRead(encB[1]);
  
  if (A == B)
    encoderCount[1]++;
  else
    encoderCount[1]--;
}
void IRAM_ATTR encoderISR2() {
  bool A = digitalRead(encA[2]);
  bool B = digitalRead(encB[2]);
  
  if (A == B)
    encoderCount[2]++;
  else
    encoderCount[2]--;
}
void IRAM_ATTR encoderISR3() {
  bool A = digitalRead(encA[3]);
  bool B = digitalRead(encB[3]);
  
  if (A == B)
    encoderCount[3]++;
  else
    encoderCount[3]--;
}

void motorWrite(int motorIndex, float pwmValue) {
  int pwm = constrain(abs(pwmValue), 0, 190);  // Limita PWM entre 0 y 255

  switch (motorIndex) {
    case 0: // Motor 1
      if (pwmValue >= 0) {
        digitalWrite(motor1_dir1, HIGH);
        digitalWrite(motor1_dir2, LOW);
      } else {
        digitalWrite(motor1_dir1, LOW);
        digitalWrite(motor1_dir2, HIGH);
      }
      analogWrite(motor1_pwm, pwm);
      break;

    case 1: // Motor 2
      if (pwmValue >= 0) {
        digitalWrite(motor2_dir1, HIGH);
        digitalWrite(motor2_dir2, LOW);
      } else {
        digitalWrite(motor2_dir1, LOW);
        digitalWrite(motor2_dir2, HIGH);
      }
      analogWrite(motor2_pwm, pwm);
      break;

    case 2: // Motor 3
      if (pwmValue >= 0) {
        digitalWrite(motor3_dir1, HIGH);
        digitalWrite(motor3_dir2, LOW);
      } else {
        digitalWrite(motor3_dir1, LOW);
        digitalWrite(motor3_dir2, HIGH);
      }
      analogWrite(motor3_pwm, pwm);
      break;

    case 3: // Motor 4
      if (pwmValue >= 0) {
        digitalWrite(motor4_dir1, HIGH);
        digitalWrite(motor4_dir2, LOW);
      } else {
        digitalWrite(motor4_dir1, LOW);
        digitalWrite(motor4_dir2, HIGH);
      }
      analogWrite(motor4_pwm, pwm);
      break;
  }
}

void C_atractivo(){
  float d = sqrt(pow((x - xd), 2) + pow((y - yd), 2));
  if(d<Qd){
    Fatracx = -Katrac * (x - xd);
    Fatracy = -Katrac * (y - yd);
  }else{
    Fatracx = (Qd *(-Katrac) * (x - xd))/d;
    Fatracy = (Qd *(-Katrac) * (y - yd))/d;
  }
}


void recibirDatosSerial() {
  static String inputString = "";
  static bool receiving = false;

  while (Serial.available()) {
    char inChar = (char)Serial.read();

    if (inChar == '<') {
      inputString = "";
      receiving = true;
    } else if (inChar == '>' && receiving) {
      receiving = false;

      // Separar los 5 valores esperados
      int indices[4];
      int start = 0;
      for (int i = 0; i < 4; i++) {
        indices[i] = inputString.indexOf(',', start);
        if (indices[i] == -1) return;
        start = indices[i] + 1;
      }

      // Extraer los valores
      Frepx   = inputString.substring(0, indices[0]).toFloat();
      Frepy   = inputString.substring(indices[0] + 1, indices[1]).toFloat();
      xd      = inputString.substring(indices[1] + 1, indices[2]).toFloat();
      yd      = inputString.substring(indices[2] + 1, indices[3]).toFloat();
      thetad  = inputString.substring(indices[3] + 1).toFloat();
    } else if (receiving) {
      inputString += inChar;
    }
  }
}


void setup() {
  Serial.begin(250000);
  lastTime = millis();
  startTime = millis();

  pinMode(motor1_pwm, OUTPUT);
  pinMode(motor1_dir1, OUTPUT);
  pinMode(motor1_dir2, OUTPUT);

  pinMode(motor2_pwm, OUTPUT);
  pinMode(motor2_dir1, OUTPUT);
  pinMode(motor2_dir2, OUTPUT);

  pinMode(motor3_pwm, OUTPUT);
  pinMode(motor3_dir1, OUTPUT);
  pinMode(motor3_dir2, OUTPUT);

  pinMode(motor4_pwm, OUTPUT);
  pinMode(motor4_dir1, OUTPUT);
  pinMode(motor4_dir2, OUTPUT);

  for (int i = 0; i < 4; i++) {
    pinMode(encA[i], INPUT);
    pinMode(encB[i], INPUT);
  }
  attachInterrupt(digitalPinToInterrupt(encA[0]), encoderISR0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encA[1]), encoderISR1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encA[2]), encoderISR2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encA[3]), encoderISR3, CHANGE);
}


void loop() {
  
  unsigned long currentTime = millis();
  float dt = currentTime - lastTime;
  recibirDatosSerial();
  if (dt >= 100) {

    for (int i = 0; i < 4; i++) {
      long deltaCounts = encoderCount[i];
      rpm[i] = (10.0 * deltaCounts * 60.0) / (CPR * 90);
      w_r[i] = ((rpm[i]*2*PI)/60);
      encoderCount[i] = 0;
      //Serial.print("x: ");
      //Serial.print(x, 2);
      //Serial.print(" | y: ");
      //Serial.print(y, 2);
      //Serial.print(" | Angulo: ");
      //Serial.print(theta, 2);
      //Serial.print(" | Motor: ");
      //Serial.print(i);
      //Serial.print(" | RPM: ");
      //Serial.print(rpm[i], 2);
      //Serial.print(" | RPM_D: ");
      //Serial.print(pwmL[i]);
      //Serial.print(" | w: ");
      //Serial.println(w_r[i]);      
    }
    Vx = ((r / 4.0) * (w_r[0] + w_r[1] + w_r[2] + w_r[3]));
    Vy = ((r / 4.0) * (-w_r[0] + w_r[1] + w_r[2] - w_r[3]));
    Wz = ((r / (4.0 * L)) * (-w_r[0] + w_r[1] - w_r[2] + w_r[3]));

    
    // Odometría
    theta += Wz * (dt/1000);
    x += (Vx * cos(theta) - Vy * sin(theta)) * (dt/1000);
    y += (Vx * sin(theta) + Vy * cos(theta)) * (dt/1000);

    // Control cinemático
    //float ex = xd - x;
    //float ey = yd - y;
    float et = thetad - theta;

    //float ux = k * ex;
    //float uy = k * ey;
    float uth = k * et;
    
    C_atractivo();
    
    float ux = Fatracx + Frepx;
    float uy = Fatracy + Frepy;

    float Vf = ux * cos(theta) + uy * sin(theta);   // forward
    float Vl = -ux * sin(theta) + uy * cos(theta);  // lateral
    float W = uth;

    // Cinemática inversa
    float w1_d = (1.0/r) * (Vf - Vl - L*W);
    float w2_d = (1.0/r) * (Vf + Vl + L*W);
    float w3_d = (1.0/r) * (Vf + Vl - L*W);
    float w4_d = (1.0/r) * (Vf - Vl + L*W);

    float w_d[4] = {w1_d, w2_d, w3_d, w4_d};
    for (int i = 0; i < 4; i++) {
      e[i] = w_d[i] - w_r[i];                    // Error actual
      e_int[i] += e[i] * (dt/1000);                     // Integral
      e_int[i] = constrain(e_int[i], -100, 100);
      float e_der = (e[i] - e_prev[i]) / (dt/1000);     // Derivada

      control_signal[i] = Kp * e[i] + Ki * e_int[i] + Kd * e_der;
      e_prev[i] = e[i];    

      pwmL[i] = ((control_signal[i]*60)/(2*PI));
      motorWrite(i, ((pwmL[i]*255)/110));
    }

    lastTime = currentTime; 
  }

}
