unsigned long int time1,previoustime,pulselength;
int x[15],ch1[15],ch[7],i;
//specifing arrays and variables to store values 

void IRAM_ATTR PPMSignalReadIn()  { //IRAM_ATTR = Internal RAM attribute, code placed in RAM instead of Flash so that the interrupt function can be proccesse as fast as possible

time1 = micros(); //store time value a when pin value falling
pulselength = time1 - previoustime;      //calculating time inbetween two peaks
previoustime = time1;        // 
x[i]= pulselength;     //storing 15 values in an array to be proccessed in the main loop because ESP32 interrupts are horrible
i=i+1;       
if(i==15){
  for(int j=0;j<15;j++) {
  ch1[j]=x[j];
   }
  i=0;
  }
}//copy store all values from temporary array to another array after 15 readings

void PPMSignalProccessing(){
int i,j,k=0;
  for(k=14;k>-1;k--){
    if(ch1[k]>3000){ //Seperation time for 1A6B reciever is around 3000us 
      j=k;
      }
    }  //detecting separation space 3000us in the temporary array, a seperation space of 3000us indicates its the start of a new set of values                   
  for(i=1;i<=6;i++){
    ch[i]=(ch1[i+j]);
    }
  }     //assigns the first 6 channel values after the separation space to the 6 channels

void setup() {
Serial.begin(9600);
  pinMode(2, INPUT_PULLUP);
  attachInterrupt(2, PPMSignalReadIn, FALLING); //(GPIO pin number, ISR function, Mode)
  // enabling interrupt at pin 2
}

void loop() {
PPMSignalProccessing();

Serial.print(ch[1]);Serial.print("\t"); 
Serial.print(ch[2]);Serial.print("\t");
Serial.print(ch[3]);Serial.print("\t");
Serial.print(ch[4]);Serial.print("\t");
Serial.print(ch[5]);Serial.print("\t");
Serial.print(ch[6]);Serial.print("\n");

delay(100); //delay time for printing out Channel values
}




