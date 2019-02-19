int left_motor_speed, right_motor_speed;
float Kp = 35, Ki = 0.000015, Kd = 0;
float error = 0, P = 0, I = 0, D = 0, PID_value = 0;
float previous_error = 0, previous_I = 0;
int sensor[6] = {0, 0, 0, 0, 0, 0};
bool sensorRead[3] = {false, false, false};
int initial_motor_speed = 150, i = 0;
bool delayed = false;
bool Inversed = false;
void read_sensor_values(void);
void calculate_pid(void);
void motor_control(void);

void setup()
{
  pinMode(9, OUTPUT); //PWM Pin 1
  pinMode(10, OUTPUT); //PWM Pin 2
  pinMode(7, OUTPUT); //Left Motor Pin 1
  pinMode(8, OUTPUT); //Left Motor Pin 2
  pinMode(2, OUTPUT); //Right Motor Pin 1
  pinMode(3, OUTPUT); //Right Motor Pin 2
  pinMode(13, OUTPUT);
  Serial.begin(9600); //Enable Serial Communications
}

void loop()
{
  delayed = false;
  read_sensor_values();
  calculate_pid();
  read_junction();
  motor_control();
}

void ZTurnTransition()
{
  analogWrite(11, initial_motor_speed);
  analogWrite(5, initial_motor_speed);
  delay(400);
}

void ZTurnNormal() {
  analogWrite(11, 100);
  analogWrite(5, 100);
  delay(800);
  if (!Inversed)
  {
    sensor[0] = analogRead(A0);
    if (sensor[0] > 500 /*when sensor closer to ground400*/) sensor[0] = 1;
    else sensor[0] = 0;
    sensor[1] = analogRead(A1);
    if (sensor[1] > 500 /*when sensor closer to ground500*/) sensor[1] = 1;
    else sensor[1] = 0;
    sensor[2] = analogRead(A2);
    if (sensor[2] > 500 /*when sensor closer to ground500*/) sensor[2] = 1;
    else sensor[2] = 0;
    sensor[3] = analogRead(A3);
    if (sensor[3] > 500 /*when sensor closer to ground500*/) sensor[3] = 1;
    else sensor[3] = 0;
    sensor[4] = analogRead(A4);
    if (sensor[4] > 500 /*when sensor closer to ground500*/) sensor[4] = 1;
    else sensor[4] = 0;
    sensor[5] = analogRead(A5);
    if (sensor[5] > 500 /*when sensor closer to ground500*/) sensor[5] = 1;
    else sensor[5] = 0;
  }
  else
  {
    sensor[0] = analogRead(A0);
    if (sensor[0] > 500 /*when sensor closer to ground500*/) sensor[0] = 0;
    else sensor[0] = 1;
    sensor[1] = analogRead(A1);
    if (sensor[1] > 500 /*when sensor closer to ground500*/) sensor[1] = 0;
    else sensor[1] = 1;
    sensor[2] = analogRead(A2);
    if (sensor[2] > 500 /*when sensor closer to ground500*/) sensor[2] = 0;
    else sensor[2] = 1;
    sensor[3] = analogRead(A3);
    if (sensor[3] > 500 /*when sensor closer to ground500*/) sensor[3] = 0;
    else sensor[3] = 1;
    sensor[4] = analogRead(A4);
    if (sensor[4] > 500 /*when sensor closer to ground500*/) sensor[4] = 0;
    else sensor[4] = 1;
    sensor[5] = analogRead(A5);
    if (sensor[5] > 500 /*when sensor closer to ground500*/) sensor[5] = 0;
    else sensor[5] = 1;
  }

  Serial.print("Sensors ");
  Serial.print(sensor[0]);
  Serial.print(" ");
  Serial.print(sensor[1]);
  Serial.print(" ");
  Serial.print(sensor[2]);
  Serial.print(" ");
  Serial.print(sensor[3]);
  Serial.print(" ");
  Serial.print(sensor[4]);
  Serial.print(" ");
  Serial.println(sensor[5]);
  if ((sensor[0] == 1) && ((sensor[1] == 0) || (sensor[2] == 0) || (sensor[3] == 0) || (sensor[4] == 0)) && (sensor[5] == 1))
  {
    error = 0;
  }
  else
  {
    digitalWrite(7, HIGH);
    digitalWrite(8, LOW);
    digitalWrite(3, HIGH);
    digitalWrite(2, LOW);
    delay(400);
    if (!Inversed)
    {
      sensor[0] = analogRead(A0);
      if (sensor[0] > 500 /*when sensor closer to ground400*/) sensor[0] = 1;
      else sensor[0] = 0;
      sensor[1] = analogRead(A1);
      if (sensor[1] > 500 /*when sensor closer to ground500*/) sensor[1] = 1;
      else sensor[1] = 0;
      sensor[2] = analogRead(A2);
      if (sensor[2] > 500 /*when sensor closer to ground500*/) sensor[2] = 1;
      else sensor[2] = 0;
      sensor[3] = analogRead(A3);
      if (sensor[3] > 500 /*when sensor closer to ground500*/) sensor[3] = 1;
      else sensor[3] = 0;
      sensor[4] = analogRead(A4);
      if (sensor[4] > 500 /*when sensor closer to ground500*/) sensor[4] = 1;
      else sensor[4] = 0;
      sensor[5] = analogRead(A5);
      if (sensor[5] > 500 /*when sensor closer to ground500*/) sensor[5] = 1;
      else sensor[5] = 0;
    }
    else
    {
      sensor[0] = analogRead(A0);
      if (sensor[0] > 500 /*when sensor closer to ground500*/) sensor[0] = 0;
      else sensor[0] = 1;
      sensor[1] = analogRead(A1);
      if (sensor[1] > 500 /*when sensor closer to ground500*/) sensor[1] = 0;
      else sensor[1] = 1;
      sensor[2] = analogRead(A2);
      if (sensor[2] > 500 /*when sensor closer to ground500*/) sensor[2] = 0;
      else sensor[2] = 1;
      sensor[3] = analogRead(A3);
      if (sensor[3] > 500 /*when sensor closer to ground500*/) sensor[3] = 0;
      else sensor[3] = 1;
      sensor[4] = analogRead(A4);
      if (sensor[4] > 500 /*when sensor closer to ground500*/) sensor[4] = 0;
      else sensor[4] = 1;
      sensor[5] = analogRead(A5);
      if (sensor[5] > 500 /*when sensor closer to ground500*/) sensor[5] = 0;
      else sensor[5] = 1;
    }

    Serial.print("Sensors ");
    Serial.print(sensor[0]);
    Serial.print(" ");
    Serial.print(sensor[1]);
    Serial.print(" ");
    Serial.print(sensor[2]);
    Serial.print(" ");
    Serial.print(sensor[3]);
    Serial.print(" ");
    Serial.print(sensor[4]);
    Serial.print(" ");
    Serial.println(sensor[5]);
    if ((sensor[5] == 0) || (sensor[4] == 0) || (sensor[3] == 0) || (sensor[2] == 0))
    {
      error = 5;
    }
    else
    {
      Serial.println("IT'S A 90");
      analogWrite(11, 100);
      analogWrite(5, 100);
      digitalWrite(7, LOW);
      digitalWrite(8, HIGH);
      digitalWrite(3, LOW);
      digitalWrite(2, HIGH);
      delay(1000);
      analogWrite(11, 100);
      analogWrite(5, 100);
      digitalWrite(7, HIGH);
      digitalWrite(8, LOW);
      digitalWrite(3, LOW);
      digitalWrite(2, HIGH);
      error = -5;
    }
  }
  //  delay(2000);
}

void read_sensor_values()
{
  if (!Inversed)
  {
    sensor[0] = analogRead(A0);
    if (sensor[0] > 500 /*when sensor closer to ground400*/) sensor[0] = 1;
    else sensor[0] = 0;
    sensor[1] = analogRead(A1);
    if (sensor[1] > 500 /*when sensor closer to ground500*/) sensor[1] = 1;
    else sensor[1] = 0;
    sensor[2] = analogRead(A2);
    if (sensor[2] > 500 /*when sensor closer to ground500*/) sensor[2] = 1;
    else sensor[2] = 0;
    sensor[3] = analogRead(A3);
    if (sensor[3] > 500 /*when sensor closer to ground500*/) sensor[3] = 1;
    else sensor[3] = 0;
    sensor[4] = analogRead(A4);
    if (sensor[4] > 500 /*when sensor closer to ground500*/) sensor[4] = 1;
    else sensor[4] = 0;
    sensor[5] = analogRead(A5);
    if (sensor[5] > 500 /*when sensor closer to ground500*/) sensor[5] = 1;
    else sensor[5] = 0;
  }
  else
  {
    sensor[0] = analogRead(A0);
    if (sensor[0] > 500 /*when sensor closer to ground500*/) sensor[0] = 0;
    else sensor[0] = 1;
    sensor[1] = analogRead(A1);
    if (sensor[1] > 500 /*when sensor closer to ground500*/) sensor[1] = 0;
    else sensor[1] = 1;
    sensor[2] = analogRead(A2);
    if (sensor[2] > 500 /*when sensor closer to ground500*/) sensor[2] = 0;
    else sensor[2] = 1;
    sensor[3] = analogRead(A3);
    if (sensor[3] > 500 /*when sensor closer to ground500*/) sensor[3] = 0;
    else sensor[3] = 1;
    sensor[4] = analogRead(A4);
    if (sensor[4] > 500 /*when sensor closer to ground500*/) sensor[4] = 0;
    else sensor[4] = 1;
    sensor[5] = analogRead(A5);
    if (sensor[5] > 500 /*when sensor closer to ground500*/) sensor[5] = 0;
    else sensor[5] = 1;
  }

  Serial.print("Sensors ");
  Serial.print(sensor[0]);
  Serial.print(" ");
  Serial.print(sensor[1]);
  Serial.print(" ");
  Serial.print(sensor[2]);
  Serial.print(" ");
  Serial.print(sensor[3]);
  Serial.print(" ");
  Serial.print(sensor[4]);
  Serial.print(" ");
  Serial.println(sensor[5]);
  //    delay(500);

  if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 1) && (sensor[5] == 0)) //111110
    error = 5;
  else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 1) && (sensor[5] == 0)) //011110
    error = 5;
  else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 0) && (sensor[5] == 0)) //111100
    error = 4;
  else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 0) && (sensor[4] == 0) && (sensor[5] == 0)) //111000
  {
    error = 5;//previously 3
    //delayed=true;
  }
  else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0) && (sensor[5] == 1)) //110001
    error = 1;
  else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 0) && (sensor[4] == 0) && (sensor[5] == 1)) //111001
    error = 2;
  else if ((sensor[0] == 1) && (sensor[1] == 1) && ((sensor[2] == 0) || (sensor[3] == 0)) && (sensor[4] == 1) && (sensor[5] == 0)) //111010 or 110010
  {
    ZTurnTransition();
  }
  /*else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 1) && (sensor[5] == 1)) //010011 or 010111
    {
    ZTurnNormal();
    }*/
  //    error = 3;
  else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 0) && (sensor[4] == 1) && (sensor[5] == 1)) //111011
    error = 0;
  else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 1) && (sensor[5] == 1)) //110011
    error = 0;
  else if ((sensor[0] == 1) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 1) && (sensor[5] == 1)) //100011
    error = -1;
  else if ((sensor[0] == 1) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 1) && (sensor[4] == 1) && (sensor[5] == 1)) //100111
    error = -2;
  else if ((sensor[0] == 1) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 1) && (sensor[5] == 1)) //101111
    error = -3;
  else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 0) && (sensor[3] == 1) && (sensor[4] == 1) && (sensor[5] == 1)) //110111
    error = 0;
  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 1) && (sensor[4] == 1) && (sensor[5] == 1)) //000111
  {
    error = -5;//previously -3
    //delayed=true;
  }
  else if (/*((sensor[0] == 1) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0) && (sensor[5] == 1))||*/((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 1) && (sensor[5] == 1)) || ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 1) && (sensor[5] == 1))) //100001
  {
    error = 4;//previously -3
    //delayed=true;
    ZTurnNormal();
  }
  else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 1) && (sensor[5] == 0)) //000001
  {
    error = 5;

  }
  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 1) && (sensor[5] == 1)) //000011
  {
    error = -5;//new condition
    //delayed=true;
  }
  else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0) && (sensor[5] == 0)) //110000
  {
    error = 2;//new conditon
    //delayed=true;
  }
  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 1) && (sensor[5] == 1)) //001111
    error = -4;
  else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 1) && (sensor[5] == 1)) //011111
    error = -5;

  else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 1) && (sensor[5] == 1)) //111111
  {
    if ((error >= -5 && error < 0)) error = -6;    else if (error <= 5 && error > 0) error = 6;
    else if (error == 0) deadEndMechanics();
  }
  if ((sensor[0] == 0) && (sensor[1] == 0) && ((sensor[2] == 1) || (sensor[3] == 1)) && (sensor[4] == 0) && (sensor[5] == 0)) //001100
  {
    Inversed = !Inversed;
  }
}

void deadEndMechanics() {
  analogWrite(11, 100);
  analogWrite(5, 100);
  delay(2250);
  if (!Inversed)
  {
    sensor[0] = analogRead(A0);
    if (sensor[0] > 500 /*when sensor closer to ground500*/) sensor[0] = 1;
    else sensor[0] = 0;
    sensor[1] = analogRead(A1);
    if (sensor[1] > 500 /*when sensor closer to ground500*/) sensor[1] = 1;
    else sensor[1] = 0;
    sensor[2] = analogRead(A2);
    if (sensor[2] > 500 /*when sensor closer to ground500*/) sensor[2] = 1;
    else sensor[2] = 0;
    sensor[3] = analogRead(A3);
    if (sensor[3] > 500 /*when sensor closer to ground500*/) sensor[3] = 1;
    else sensor[3] = 0;
    sensor[4] = analogRead(A4);
    if (sensor[4] > 500 /*when sensor closer to ground500*/) sensor[4] = 1;
    else sensor[4] = 0;
    sensor[5] = analogRead(A5);
    if (sensor[5] > 500 /*when sensor closer to ground500*/) sensor[5] = 1;
    else sensor[5] = 0;
  }
  else
  {
    sensor[0] = analogRead(A0);
    if (sensor[0] > 500 /*when sensor closer to ground500*/) sensor[0] = 0;
    else sensor[0] = 1;
    sensor[1] = analogRead(A1);
    if (sensor[1] > 500 /*when sensor closer to ground500*/) sensor[1] = 0;
    else sensor[1] = 1;
    sensor[2] = analogRead(A2);
    if (sensor[2] > 500 /*when sensor closer to ground500*/) sensor[2] = 0;
    else sensor[2] = 1;
    sensor[3] = analogRead(A3);
    if (sensor[3] > 500 /*when sensor closer to ground500*/) sensor[3] = 0;
    else sensor[3] = 1;
    sensor[4] = analogRead(A4);
    if (sensor[4] > 500 /*when sensor closer to ground500*/) sensor[4] = 0;
    else sensor[4] = 1;
    sensor[5] = analogRead(A5);
    if (sensor[5] > 500 /*when sensor closer to ground500*/) sensor[5] = 0;
    else sensor[5] = 1;
  }
  Serial.print("Sensors ");
  Serial.print(sensor[0]);
  Serial.print(" ");
  Serial.print(sensor[1]);
  Serial.print(" ");
  Serial.print(sensor[2]);
  Serial.print(" ");
  Serial.print(sensor[3]);
  Serial.print(" ");
  Serial.print(sensor[4]);
  Serial.print(" ");
  Serial.println(sensor[5]);
  if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 1) && (sensor[5] == 1)) //111111
  {
    Serial.println("IT'S A 180");
    analogWrite(11, 254);
    analogWrite(5, 254);
    digitalWrite(7, LOW);
    digitalWrite(8, HIGH);
    digitalWrite(3, LOW);
    digitalWrite(2, HIGH);
    delay(1500);
    analogWrite(11, 100);
    analogWrite(5, 100);
    digitalWrite(7, HIGH);
    digitalWrite(8, LOW);
    digitalWrite(3, LOW);
    digitalWrite(2, HIGH);
  }

  error = -2;
}

void read_junction()
{
  if (!Inversed)
  {
    sensor[0] = analogRead(A0);
    if (sensor[0] > 500 /*when sensor closer to ground500*/) sensor[0] = 1;
    else sensor[0] = 0;
    sensor[1] = analogRead(A1);
    if (sensor[1] > 500 /*when sensor closer to ground500*/) sensor[1] = 1;
    else sensor[1] = 0;
    sensor[2] = analogRead(A2);
    if (sensor[2] > 500 /*when sensor closer to ground500*/) sensor[2] = 1;
    else sensor[2] = 0;
    sensor[3] = analogRead(A3);
    if (sensor[3] > 500 /*when sensor closer to ground500*/) sensor[3] = 1;
    else sensor[3] = 0;
    sensor[4] = analogRead(A4);
    if (sensor[4] > 500 /*when sensor closer to ground500*/) sensor[4] = 1;
    else sensor[4] = 0;
    sensor[5] = analogRead(A5);
    if (sensor[5] > 500 /*when sensor closer to ground500*/) sensor[5] = 1;
    else sensor[5] = 0;
  }
  else
  {
    sensor[0] = analogRead(A0);
    if (sensor[0] > 500 /*when sensor closer to ground500*/) sensor[0] = 0;
    else sensor[0] = 1;
    sensor[1] = analogRead(A1);
    if (sensor[1] > 500 /*when sensor closer to ground500*/) sensor[1] = 0;
    else sensor[1] = 1;
    sensor[2] = analogRead(A2);
    if (sensor[2] > 500 /*when sensor closer to ground500*/) sensor[2] = 0;
    else sensor[2] = 1;
    sensor[3] = analogRead(A3);
    if (sensor[3] > 500 /*when sensor closer to ground500*/) sensor[3] = 0;
    else sensor[3] = 1;
    sensor[4] = analogRead(A4);
    if (sensor[4] > 500 /*when sensor closer to ground500*/) sensor[4] = 0;
    else sensor[4] = 1;
    sensor[5] = analogRead(A5);
    if (sensor[5] > 500 /*when sensor closer to ground500*/) sensor[5] = 0;
    else sensor[5] = 1;
  }

  Serial.print("Sensors ");
  Serial.print(sensor[0]);
  Serial.print(" ");
  Serial.print(sensor[1]);
  Serial.print(" ");
  Serial.print(sensor[2]);
  Serial.print(" ");
  Serial.print(sensor[3]);
  Serial.print(" ");
  Serial.print(sensor[4]);
  Serial.print(" ");
  Serial.println(sensor[5]);

  if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 0) && (sensor[4] == 0) && (sensor[5] == 0)) //111000
  {
    error = 5;//new condition
    delayed = true;
    PID_value = 90;
    analogWrite(11, 0); //Left Motor Speed
    analogWrite(5, 0); //Right Motor Speed
    delay(300);
    analogWrite(11, 100);
    analogWrite(5, 100);
    delay(500);
    //    analogWrite(11, 0);
    //    analogWrite(5, 0);
    //    delay(500);
    if (!Inversed)
    {
      sensor[0] = analogRead(A0);
      if (sensor[0] > 500 /*when sensor closer to ground500*/) sensor[0] = 1;
      else sensor[0] = 0;
      sensor[1] = analogRead(A1);
      if (sensor[1] > 500 /*when sensor closer to ground500*/) sensor[1] = 1;
      else sensor[1] = 0;
      sensor[2] = analogRead(A2);
      if (sensor[2] > 500 /*when sensor closer to ground500*/) sensor[2] = 1;
      else sensor[2] = 0;
      sensor[3] = analogRead(A3);
      if (sensor[3] > 500 /*when sensor closer to ground500*/) sensor[3] = 1;
      else sensor[3] = 0;
      sensor[4] = analogRead(A4);
      if (sensor[4] > 500 /*when sensor closer to ground500*/) sensor[4] = 1;
      else sensor[4] = 0;
      sensor[5] = analogRead(A5);
      if (sensor[5] > 500 /*when sensor closer to ground500*/) sensor[5] = 1;
      else sensor[5] = 0;
    }
    else
    {
      sensor[0] = analogRead(A0);
      if (sensor[0] > 500 /*when sensor closer to ground500*/) sensor[0] = 0;
      else sensor[0] = 1;
      sensor[1] = analogRead(A1);
      if (sensor[1] > 500 /*when sensor closer to ground500*/) sensor[1] = 0;
      else sensor[1] = 1;
      sensor[2] = analogRead(A2);
      if (sensor[2] > 500 /*when sensor closer to ground500*/) sensor[2] = 0;
      else sensor[2] = 1;
      sensor[3] = analogRead(A3);
      if (sensor[3] > 500 /*when sensor closer to ground500*/) sensor[3] = 0;
      else sensor[3] = 1;
      sensor[4] = analogRead(A4);
      if (sensor[4] > 500 /*when sensor closer to ground500*/) sensor[4] = 0;
      else sensor[4] = 1;
      sensor[5] = analogRead(A5);
      if (sensor[5] > 500 /*when sensor closer to ground500*/) sensor[5] = 0;
      else sensor[5] = 1;
    }
    if ((sensor[0] == 0) && ((sensor[1] == 1) || (sensor[2] == 1) || (sensor[3] == 1) || (sensor[4] == 1)) && (sensor[5] == 0))
    {
      Inversed = !Inversed;
      error = -5;
    }
    else
    {
      analogWrite(11, 120); //Left Motor Speed
      analogWrite(5, 0); //Right Motor Speed
      delay(1000);
    }


    /*while(true)
      {
      if(sensor[0]==0)
        break;
      }*/
  }
  else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0) && (sensor[5] == 0)) //110000
  {
    Serial.println("JUnction 110000");
    error = 5;//new condition
    delayed = true;
    PID_value = 90;
    analogWrite(11, 0); //Left Motor Speed
    analogWrite(5, 0); //Right Motor Speed
    delay(300);
    analogWrite(11, 100);
    analogWrite(5, 100);
    delay(500);
    //    analogWrite(11, 0);
    //    analogWrite(5, 0);
    //    delay(500);
    if (!Inversed)
    {
      sensor[0] = analogRead(A0);
      if (sensor[0] > 500 /*when sensor closer to ground500*/) sensor[0] = 1;
      else sensor[0] = 0;
      sensor[1] = analogRead(A1);
      if (sensor[1] > 500 /*when sensor closer to ground500*/) sensor[1] = 1;
      else sensor[1] = 0;
      sensor[2] = analogRead(A2);
      if (sensor[2] > 500 /*when sensor closer to ground500*/) sensor[2] = 1;
      else sensor[2] = 0;
      sensor[3] = analogRead(A3);
      if (sensor[3] > 500 /*when sensor closer to ground500*/) sensor[3] = 1;
      else sensor[3] = 0;
      sensor[4] = analogRead(A4);
      if (sensor[4] > 500 /*when sensor closer to ground500*/) sensor[4] = 1;
      else sensor[4] = 0;
      sensor[5] = analogRead(A5);
      if (sensor[5] > 500 /*when sensor closer to ground500*/) sensor[5] = 1;
      else sensor[5] = 0;
    }
    else
    {
      sensor[0] = analogRead(A0);
      if (sensor[0] > 500 /*when sensor closer to ground500*/) sensor[0] = 0;
      else sensor[0] = 1;
      sensor[1] = analogRead(A1);
      if (sensor[1] > 500 /*when sensor closer to ground500*/) sensor[1] = 0;
      else sensor[1] = 1;
      sensor[2] = analogRead(A2);
      if (sensor[2] > 500 /*when sensor closer to ground500*/) sensor[2] = 0;
      else sensor[2] = 1;
      sensor[3] = analogRead(A3);
      if (sensor[3] > 500 /*when sensor closer to ground500*/) sensor[3] = 0;
      else sensor[3] = 1;
      sensor[4] = analogRead(A4);
      if (sensor[4] > 500 /*when sensor closer to ground500*/) sensor[4] = 0;
      else sensor[4] = 1;
      sensor[5] = analogRead(A5);
      if (sensor[5] > 500 /*when sensor closer to ground500*/) sensor[5] = 0;
      else sensor[5] = 1;
    }
    if ((sensor[0] == 0) && ((sensor[1] == 1) || (sensor[2] == 1) || (sensor[3] == 1) || (sensor[4] == 1)) && (sensor[5] == 0))
    {
      Inversed = !Inversed;
      error = -5;
    }
    else
    {
      analogWrite(11, 120); //Left Motor Speed
      analogWrite(5, 0); //Right Motor Speed
      delay(1000);
    }

    /*while(true)
      {
      if(sensor[0]==0)
        break;
      }*/
  }
  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0) && (sensor[5] == 0)) //000000
  {
    Serial.println("JUnction 000000");
    error = 0;//new condition
    delayed = true;
    PID_value = 90;
    analogWrite(11, 0); //Left Motor Speed
    analogWrite(5, 0); //Right Motor Speed
    delay(300);
    analogWrite(11, 100);
    analogWrite(5, 100);
    delay(500);
    //    analogWrite(11, 0);
    //    analogWrite(5, 0);
    //    delay(500);
    if (!Inversed)
    {
      sensor[0] = analogRead(A0);
      if (sensor[0] > 500 /*when sensor closer to ground500*/) sensor[0] = 1;
      else sensor[0] = 0;
      sensor[1] = analogRead(A1);
      if (sensor[1] > 500 /*when sensor closer to ground500*/) sensor[1] = 1;
      else sensor[1] = 0;
      sensor[2] = analogRead(A2);
      if (sensor[2] > 500 /*when sensor closer to ground500*/) sensor[2] = 1;
      else sensor[2] = 0;
      sensor[3] = analogRead(A3);
      if (sensor[3] > 500 /*when sensor closer to ground500*/) sensor[3] = 1;
      else sensor[3] = 0;
      sensor[4] = analogRead(A4);
      if (sensor[4] > 500 /*when sensor closer to ground500*/) sensor[4] = 1;
      else sensor[4] = 0;
      sensor[5] = analogRead(A5);
      if (sensor[5] > 500 /*when sensor closer to ground500*/) sensor[5] = 1;
      else sensor[5] = 0;
    }
    else
    {
      sensor[0] = analogRead(A0);
      if (sensor[0] > 500 /*when sensor closer to ground500*/) sensor[0] = 0;
      else sensor[0] = 1;
      sensor[1] = analogRead(A1);
      if (sensor[1] > 500 /*when sensor closer to ground500*/) sensor[1] = 0;
      else sensor[1] = 1;
      sensor[2] = analogRead(A2);
      if (sensor[2] > 500 /*when sensor closer to ground500*/) sensor[2] = 0;
      else sensor[2] = 1;
      sensor[3] = analogRead(A3);
      if (sensor[3] > 500 /*when sensor closer to ground500*/) sensor[3] = 0;
      else sensor[3] = 1;
      sensor[4] = analogRead(A4);
      if (sensor[4] > 500 /*when sensor closer to ground500*/) sensor[4] = 0;
      else sensor[4] = 1;
      sensor[5] = analogRead(A5);
      if (sensor[5] > 500 /*when sensor closer to ground500*/) sensor[5] = 0;
      else sensor[5] = 1;
    }
    if ((sensor[0] == 0) && ((sensor[1] == 1) || (sensor[2] == 1) || (sensor[3] == 1) || (sensor[4] == 1)) && (sensor[5] == 0))
    {
      Inversed = !Inversed;
      error = -5;
    }
    else
    {
      analogWrite(11, 120); //Left Motor Speed
      analogWrite(5, 0); //Right Motor Speed
      delay(1000);
    }

  }
  /*
    else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0) && (sensor[5] == 1)) //000001
    {
    error = 0;//new condition
    delayed = true;
    PID_value = 90;
    analogWrite(11, 0); //Left Motor Speed
    analogWrite(5, 0); //Right Motor Speed
    delay(300);
    analogWrite(11, 100);
    analogWrite(5, 100);
    delay(300);
    analogWrite(11, 120); //Left Motor Speed
    analogWrite(5, 40); //Right Motor Speed
    delay(400);
    }  */
  else if ((sensor[0] == 1) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0) && (sensor[5] == 0)) //100000
  {
    Serial.println("JUnction 110000");
    error = 5;//new condition
    delayed = true;
    PID_value = 90;
    analogWrite(11, 0); //Left Motor Speed
    analogWrite(5, 0); //Right Motor Speed
    delay(300);
    analogWrite(11, 100);
    analogWrite(5, 100);
    delay(500);
    //    analogWrite(11, 0);
    //    analogWrite(5, 0);
    //    delay(500);
    if (!Inversed)
    {
      sensor[0] = analogRead(A0);
      if (sensor[0] > 500 /*when sensor closer to ground500*/) sensor[0] = 1;
      else sensor[0] = 0;
      sensor[1] = analogRead(A1);
      if (sensor[1] > 500 /*when sensor closer to ground500*/) sensor[1] = 1;
      else sensor[1] = 0;
      sensor[2] = analogRead(A2);
      if (sensor[2] > 500 /*when sensor closer to ground500*/) sensor[2] = 1;
      else sensor[2] = 0;
      sensor[3] = analogRead(A3);
      if (sensor[3] > 500 /*when sensor closer to ground500*/) sensor[3] = 1;
      else sensor[3] = 0;
      sensor[4] = analogRead(A4);
      if (sensor[4] > 500 /*when sensor closer to ground500*/) sensor[4] = 1;
      else sensor[4] = 0;
      sensor[5] = analogRead(A5);
      if (sensor[5] > 500 /*when sensor closer to ground500*/) sensor[5] = 1;
      else sensor[5] = 0;
    }
    else
    {
      sensor[0] = analogRead(A0);
      if (sensor[0] > 500 /*when sensor closer to ground500*/) sensor[0] = 0;
      else sensor[0] = 1;
      sensor[1] = analogRead(A1);
      if (sensor[1] > 500 /*when sensor closer to ground500*/) sensor[1] = 0;
      else sensor[1] = 1;
      sensor[2] = analogRead(A2);
      if (sensor[2] > 500 /*when sensor closer to ground500*/) sensor[2] = 0;
      else sensor[2] = 1;
      sensor[3] = analogRead(A3);
      if (sensor[3] > 500 /*when sensor closer to ground500*/) sensor[3] = 0;
      else sensor[3] = 1;
      sensor[4] = analogRead(A4);
      if (sensor[4] > 500 /*when sensor closer to ground500*/) sensor[4] = 0;
      else sensor[4] = 1;
      sensor[5] = analogRead(A5);
      if (sensor[5] > 500 /*when sensor closer to ground500*/) sensor[5] = 0;
      else sensor[5] = 1;
    }
    if ((sensor[0] == 0) && ((sensor[1] == 1) || (sensor[2] == 1) || (sensor[3] == 1) || (sensor[4] == 1)) && (sensor[5] == 0))
    {
      Inversed = !Inversed;
      error = -5;
    }
    else
    {
      analogWrite(11, 120); //Left Motor Speed
      analogWrite(5, 0); //Right Motor Speed
      delay(1000);
    }

    /*while(true)
      {
      if(sensor[0]==0)
        break;
      }*/
  }
  else if (((sensor[0] == 1) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 1) && (sensor[4] == 0) && (sensor[5] == 0)) || ((sensor[0] == 1) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 0) && (sensor[4] == 0) && (sensor[5] == 0)) || ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 1) && (sensor[5] == 0))) //100100,101000
  {
    error = 0;//new condition
    delayed = true;
    PID_value = 90;
    analogWrite(11, 0); //Left Motor Speed
    analogWrite(5, 0); //Right Motor Speed
    delay(300);
    analogWrite(11, 100);
    analogWrite(5, 100);
    delay(500);
    //    analogWrite(11, 0);
    //    analogWrite(5, 0);
    //    delay(500);
    if (!Inversed)
    {
      sensor[0] = analogRead(A0);
      if (sensor[0] > 500 /*when sensor closer to ground500*/) sensor[0] = 1;
      else sensor[0] = 0;
      sensor[1] = analogRead(A1);
      if (sensor[1] > 500 /*when sensor closer to ground500*/) sensor[1] = 1;
      else sensor[1] = 0;
      sensor[2] = analogRead(A2);
      if (sensor[2] > 500 /*when sensor closer to ground500*/) sensor[2] = 1;
      else sensor[2] = 0;
      sensor[3] = analogRead(A3);
      if (sensor[3] > 500 /*when sensor closer to ground500*/) sensor[3] = 1;
      else sensor[3] = 0;
      sensor[4] = analogRead(A4);
      if (sensor[4] > 500 /*when sensor closer to ground500*/) sensor[4] = 1;
      else sensor[4] = 0;
      sensor[5] = analogRead(A5);
      if (sensor[5] > 500 /*when sensor closer to ground500*/) sensor[5] = 1;
      else sensor[5] = 0;
    }
    else
    {
      sensor[0] = analogRead(A0);
      if (sensor[0] > 500 /*when sensor closer to ground500*/) sensor[0] = 0;
      else sensor[0] = 1;
      sensor[1] = analogRead(A1);
      if (sensor[1] > 500 /*when sensor closer to ground500*/) sensor[1] = 0;
      else sensor[1] = 1;
      sensor[2] = analogRead(A2);
      if (sensor[2] > 500 /*when sensor closer to ground500*/) sensor[2] = 0;
      else sensor[2] = 1;
      sensor[3] = analogRead(A3);
      if (sensor[3] > 500 /*when sensor closer to ground500*/) sensor[3] = 0;
      else sensor[3] = 1;
      sensor[4] = analogRead(A4);
      if (sensor[4] > 500 /*when sensor closer to ground500*/) sensor[4] = 0;
      else sensor[4] = 1;
      sensor[5] = analogRead(A5);
      if (sensor[5] > 500 /*when sensor closer to ground500*/) sensor[5] = 0;
      else sensor[5] = 1;
    }
    if ((sensor[0] == 0) && ((sensor[1] == 1) || (sensor[2] == 1) || (sensor[3] == 1) || (sensor[4] == 1)) && (sensor[5] == 0))
    {
      Inversed = !Inversed;
      error = 0;
    }
    else
    {
      analogWrite(11, 120); //Left Motor Speed
      analogWrite(5, 0); //Right Motor Speed
      delay(1000);
    }

  }
}
void calculate_pid()
{
  P = error;
  I = I + error;
  D = error - previous_error;

  PID_value = (Kp * P) + (Ki * I) + (Kd * D);

  previous_I = I;
  previous_error = error;
}

void motor_control()
{
  Serial.print("PID Value");
  Serial.println(PID_value);
  left_motor_speed = initial_motor_speed + PID_value;//- then + is WRONG appearantly
  right_motor_speed = initial_motor_speed - PID_value;

  constrain(left_motor_speed, -200, 200);
  constrain(right_motor_speed, -200, 200);

  if (left_motor_speed > 255) left_motor_speed = 255;
  if (right_motor_speed > 255) right_motor_speed = 255;
  if (left_motor_speed < 0) left_motor_speed = 0;
  if (right_motor_speed < 0 ) right_motor_speed = 0;
  Serial.print("Motor Left and Right ");
  Serial.print(left_motor_speed);
  Serial.print(" ");
  Serial.println(right_motor_speed);
  analogWrite(11, left_motor_speed); //Left Motor Speed
  analogWrite(5, right_motor_speed); //Right Motor Speed

  digitalWrite(7, HIGH);
  digitalWrite(8, LOW);
  digitalWrite(3, LOW);
  digitalWrite(2, HIGH);
  if (delayed)
  {
    Serial.println("delayed");
    delay(440);
  }
  //  if (left_motor_speed <= 0  )
  //  {
  //    left_motor_speed = -120;
  //    digitalWrite(7, LOW);
  //    digitalWrite(8, HIGH);
  //  }
  //  else if (right_motor_speed <= 0)
  //  {
  //    right_motor_speed = -120;
  //    digitalWrite(2, LOW);
  //    digitalWrite(3, HIGH);
  //  }
  //  else {
  //    digitalWrite(7, HIGH);
  //    digitalWrite(8, LOW);
  //    digitalWrite(3, LOW);
  //    digitalWrite(2, HIGH);
  //  }

}
