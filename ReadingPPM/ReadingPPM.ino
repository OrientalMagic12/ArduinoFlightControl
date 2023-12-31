unsigned long ch[7], t[8];
int pulse = 0;

void IRAM_ATTR ISR(){
  t[pulse] = micros();
  
  switch(pulse){
    case 1:
    ch[1] = t[1]-t[0];
    pulse++;
    if (ch[1] > 3000){
      t[0] = t[1];
      pulse = 1;
    }
    break;

    case 2:
    ch[2] = t[2]-t[1];
    pulse++;
    if (ch[2] > 3000){
      t[0] = t[2];
      pulse = 1;
    }
    break;

    case 3:
    ch[3] = t[3]-t[2];
    pulse++;
    if (ch[3] > 3000){
      t[0] = t[3];
      pulse = 1;
    }
    break;

    case 4:
    ch[4] = t[4]-t[3];
    pulse++;
    if (ch[4] > 3000){
      t[0] = t[4];
      pulse = 1;
    }
    break;

    case 5:
    ch[5] = t[5]-t[4];
    pulse++;
    if (ch[5] > 3000){
      t[0] = t[5];
      pulse = 1;
    }
    break;

    case 6:
    ch[6] = t[6]-t[5];
    pulse++;
    if (ch[6] > 3000){
      t[0] = t[6];
      pulse = 1;
    }
    break;

    case 7:
    ch[7] = t[7]-t[6];
    pulse++;
    if (ch[7] > 3000){
      t[0] = t[7];
      pulse = 1;
    }
    break;
    default:
    pulse++;
    break;
    
    

  }
  
}
void setup(){
//pinMode(18, INPUT_PULLUP);

  attachInterrupt(18, ISR, RISING);
  Serial.begin(115200);

}
void loop(){
  printval();
  delay(100);
  ISR();
}


void printval(){
  Serial.print(ch[0]);
  Serial.print(" - ");
  Serial.print(ch[1]);
  Serial.print(" - ");
  Serial.print(ch[2]);
  Serial.print(" - ");
  Serial.print(ch[3]);
  Serial.print(" - ");
  Serial.print(ch[4]);
  Serial.print(" - ");
  Serial.print(ch[5]);
  Serial.print(" - ");
  Serial.println(ch[6]);
  
  
  
}