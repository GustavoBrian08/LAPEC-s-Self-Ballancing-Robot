//Definicoes pinos Arduino ligados a entrada da Ponte H
int forca = 30;
float contador_media = 0;
int indice = 0;
int IN3 = 6;
int IN4 = 7;
int ENB = 11;
int D0 = 2;
int rpm;
volatile int pulsos;
unsigned long timeold;

//Altere o numero abaixo de acordo com o seu disco encoder
unsigned int pulsos_por_volta = 20;

void contador()
{
  //Incrementa contador
  pulsos++;
}

void setup()
{

  Serial.begin(9600);
  //Pino do sensor como entrada
  pinMode(D0, INPUT);
 //Interrupcao 0 - pino digital 2
  //Aciona o contador a cada pulso
  attachInterrupt(0, contador, FALLING);
  pulsos = 0;
  rpm = 0;
  timeold = 0;
  //Define os pinos como saida
   
 pinMode(IN3, OUTPUT);
 pinMode(IN4, OUTPUT);
 pinMode(ENB, OUTPUT);
 analogWrite(ENB, 0);
 digitalWrite(IN3, HIGH);
 digitalWrite(IN4, LOW);

}


void loop()
{
  //Atualiza contador a cada 2segundo
  if (millis() - timeold >= 3000 && forca<256)
  {
    //Desabilita interrupcao durante o calculo
    detachInterrupt(0);
    rpm = (60 * 1000 / pulsos_por_volta ) / (millis() - timeold) * pulsos;
    timeold = millis();
    pulsos = 0;
    //Mostra o valor de RPM no serial monitor
    Serial.print(rpm);
    Serial.print(',');
    Serial.println(forca);
    //delay(300)
      forca+=1; 
    
    //Habilita interrupcao
    attachInterrupt(0, contador, FALLING);
  }
  //Gira o Motor B no sentido horario
  if(forca > 255){
    analogWrite(ENB, 0);
  } else{
    analogWrite(ENB, forca);
  }   
                                
}