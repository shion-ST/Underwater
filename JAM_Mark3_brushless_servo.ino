#include "string.h"
#include <Servo.h>
#define Cont_threshold 3   // スティックの遊び（この値未満は0扱い）

// ===== ピン設定（J4 AUX） =====
#define ESC_L_PIN   4    // J4-7 : 左ブラシレス
#define ESC_R_PIN   7    // J4-8 : 右ブラシレス
#define SERVO1_PIN  8    // J4-9 : サーボ1（右スティック縦）
#define SERVO2_PIN  12   // J4-6 : サーボ2（X/Bボタン）
// DCモータ : M2枠（D5 + A1）

// ===== 調整用の値 =====
#define STICK_MAX     255   // スティックの最大値
#define ESC_NEUTRAL   1500  // ESCの停止位置(μs)
#define ESC_RANGE     500   // 停止位置からの振れ幅(μs) → 1000～2000μs
#define BUMPER_POWER  150   // LB/RBで旋回するときの出力(0～255)
#define SERVO2_STEP   2     // X/Bを押している間に1回で動く角度
#define DC_SPEED      255   // DCモータの出力(0～255)
#define FAILSAFE_MS   500   // この時間受信がなければモータ停止(ms)

char recieve_data[80];//受信可能文字数
int chkstr[17];//17入力 (X,Y,Z,rZ,B1~~~B12,POV)
int recieve_cont = 0;//文字カウント用変数
bool recieve_end = false;//命令が更新されているか（文字列の終端まで到達しているか）
bool USB_Communication = false;//電話線接続モード（USB時接続テスト時にはTrueにする）

Servo escL, escR, servo1, servo2;
int servo2Angle = 90;            // サーボ2の現在角度
unsigned long lastReceive = 0;   // 最後に受信した時刻

// 関数の前方宣言
void MotorDrive(char motornom, int motorspeed);
void EscDrive(Servo &esc, int power);
void SerialRead();

void setup() {
  pinMode(A1, OUTPUT); // Motor2（DCモータ）の方向ピン

  if(USB_Communication){
    Serial.begin(115200);
  }else{
    Serial1.begin(115200);
  }

  escL.attach(ESC_L_PIN);
  escR.attach(ESC_R_PIN);
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);

  escL.writeMicroseconds(ESC_NEUTRAL); // ESCは停止信号を送ってアーム
  escR.writeMicroseconds(ESC_NEUTRAL);
  servo1.write(90);
  servo2.write(servo2Angle);
  MotorDrive(2, 0);

  delay(2000); // ESCのアーム待ち
}

void loop() {
  SerialRead();

  if(recieve_end){
    recieve_end = false;       // 1回の受信につき1回だけ処理する
    lastReceive = millis();

  //chkstr[0]=左軸横（右が正）    //chkstr[1]=左軸縦（上が正）  //chkstr[2]=右軸縦（上が正）    //chkstr[3]=右軸横（右が正）
  //chkstr[4]=A                   //chkstr[5]=B                 //chkstr[6]=X                   //chkstr[7]=Y
  //chkstr[8]=LB                  //chkstr[9]=RB

    // ---------- ブラシレス（左スティック） ----------
    int lx = chkstr[0]; // 横：旋回
    int ly = chkstr[1]; // 縦：前進・後進
    if(abs(lx) < Cont_threshold) lx = 0;
    if(abs(ly) < Cont_threshold) ly = 0;

    int powerL = ly + lx;  // 右に倒すと左モータが強くなって右旋回
    int powerR = ly - lx;

    // LB/RB：その場で左右に旋回（スティックより優先）
    if(chkstr[8] == 1 && chkstr[9] != 1){        // LB → 左
      powerL = -BUMPER_POWER;
      powerR =  BUMPER_POWER;
    }else if(chkstr[9] == 1 && chkstr[8] != 1){  // RB → 右
      powerL =  BUMPER_POWER;
      powerR = -BUMPER_POWER;
    }

    EscDrive(escL, powerL);
    EscDrive(escR, powerR);

    // ---------- サーボ1（右スティック縦で0～180度） ----------
    int ry = chkstr[2];
    if(abs(ry) < Cont_threshold) ry = 0;
    int angle1 = map(ry, -STICK_MAX, STICK_MAX, 0, 180); // 中央で90度
    servo1.write(constrain(angle1, 0, 180));

    // ---------- サーボ2（X：角度を減らす / B：角度を増やす） ----------
    if(chkstr[6] == 1) servo2Angle -= SERVO2_STEP;
    if(chkstr[5] == 1) servo2Angle += SERVO2_STEP;
    servo2Angle = constrain(servo2Angle, 0, 180);
    servo2.write(servo2Angle);

    // ---------- DCモータ（Y：正転 / A：逆転） ----------
    int dcSpeed = 0;
    if(chkstr[7] == 1 && chkstr[4] != 1)      dcSpeed =  DC_SPEED;
    else if(chkstr[4] == 1 && chkstr[7] != 1) dcSpeed = -DC_SPEED;
    MotorDrive(2, dcSpeed);
  }

  // 通信が途切れたらモータを止める（サーボはその位置のまま）
  if(millis() - lastReceive > FAILSAFE_MS){
    EscDrive(escL, 0);
    EscDrive(escR, 0);
    MotorDrive(2, 0);
  }
}

// ESCに出力する関数（power：-255～255、0で停止）
void EscDrive(Servo &esc, int power){
  power = constrain(power, -255, 255);
  esc.writeMicroseconds(ESC_NEUTRAL + map(power, -255, 255, -ESC_RANGE, ESC_RANGE));
}

//DCモータを回転させる関数（今回はM2のみ使用）
//※M4(D9)・M5(D10)はServoライブラリと競合するため analogWrite は使わないこと
void MotorDrive(char motornom,int motorspeed){
  switch(motornom){
    case 2:
      analogWrite(5,(-1) * motorspeed);
      if(motorspeed <= 0){
        digitalWrite(A1,LOW);
      }else{
        digitalWrite(A1,HIGH);
      }
      break;
  }
}

//文字列を受信する関数（元のプログラムと同じ）
void SerialRead(){
  if(USB_Communication){
    if(Serial.available()){
        recieve_data[recieve_cont] = Serial.read();
        if(recieve_cont > 50 || recieve_data[recieve_cont] == '\n'){
          recieve_cont = 0;
          chkstr[0] = atoi(strtok(recieve_data,","));
          for(int counter1 = 1;counter1 < 16;counter1++) chkstr[counter1] = atoi(strtok(NULL,","));
          memset(( void * )recieve_data, 0, sizeof(recieve_data));
          recieve_end = true;
        }else{
          recieve_cont++;
          recieve_end = false;
        }
    }
  }else{
    if(Serial1.available()){
        recieve_data[recieve_cont] = Serial1.read();
        if(recieve_cont > 50 || recieve_data[recieve_cont] == '\n'){
          recieve_cont = 0;
          chkstr[0] = atoi(strtok(recieve_data,","));
          for(int counter1 = 1;counter1 < 16;counter1++) chkstr[counter1] = atoi(strtok(NULL,","));
          memset(( void * )recieve_data, 0, sizeof(recieve_data));
          recieve_end = true;
        }else{
          recieve_cont++;
          recieve_end = false;
        }
    }
  }
}
