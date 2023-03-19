//Definicoes pinos Arduino ligados a entrada da Ponte H
int forca = 0;
float contador_media = 0;
int indice = 0;
int IN3 = 6;
int IN4 = 7;
int ENB = 11;
int D0 = 2;
int rpm;
unsigned long t0;
unsigned long t1;
float resultado;
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
  
  rpm = 0;
  timeold = 0;
  //Define os pinos como saida
   
 pinMode(IN3, OUTPUT);
 pinMode(IN4, OUTPUT);
 pinMode(ENB, OUTPUT);
 analogWrite(ENB, 0);
 digitalWrite(IN3, HIGH);
 digitalWrite(IN4, LOW);
 delay(100);
 pulsos = 0;
}


void loop()
{
  t0 = millis();
  timeold = millis();
  analogWrite(ENB,255);
  while (true){
    //Atualiza contador a cada 2segundo
    if (millis() - timeold >= 50)
    {
      //Desabilita interrupcao durante o calculo
      detachInterrupt(0);
      rpm = (60 * 1000 / pulsos_por_volta ) / (millis() - timeold) * pulsos;
      timeold = millis();
      pulsos = 0;    
      //Habilita interrupcao
      attachInterrupt(0, contador, FALLING);
      if(rpm>=546){
        t1 = millis();                
        break;
      }    
    }
  }
  resultado = 2000/(t1 - t0);
  Serial.println(resultado);
  analogWrite(ENB, 0);
  delay(2000);             
}