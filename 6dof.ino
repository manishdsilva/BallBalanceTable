#include <Servo.h> 
#include <String.h>
 
Servo Servo1;    // create servo object to control a servo 
Servo Servo2;  
Servo Servo3;
Servo Servo4;
Servo Servo5;
Servo Servo6;

int x_angle = 10;
int y_angle = -10;
int i ;
String angle;
int a,b,c,d,e,f,u,v,w,x,y,z,k,l,m,p,q,r,an1,an2,an3,pr1,pr2,pr3;
unsigned long previousMillis = 0;
int interval = 100;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Servo1.attach(3);      // attach the signal pin of servo to pin9 of arduino
  Servo2.attach(5);
  Servo3.attach(6);      // attach the signal pin of servo to pin9 of arduino
  Servo4.attach(9);
  Servo5.attach(10);
  Servo6.attach(11);
  

  Servo1.write(90);
  Servo2.write(90);
  Servo3.write(90);
  Servo4.write(90);
  Servo5.write(90);
  Servo6.write(90);
 // delay(100);
 //Serial.setTimeout(50);
}

void loop() 
{  
  //unsigned long currentMillis = millis();
     i=0;
  if((Serial.available()>0)) //&& ((currentMillis-previousMillis)<interval))
  {   
      //Serial.println("Receiving Data at Arduino");
      angle = Serial.readStringUntil('*');
      //Serial.println(angle);
      //l = strlen(angle);
      
      //for(i=0; angle[i] !='\0';i=i+6)
      //{
      
        //Serial.println(angle);
        //Serial.println(angle[0]);
        //Serial.println(angle[1]);
        
        u = angle[i]-'0';
        v = angle[i+1]-'0';
        w = angle[i+2]-'0';
        x = angle[i+3]-'0';
        y = angle[i+4]-'0';
        z = angle[i+5]-'0';
        p = angle[i+6]-'0';
        q = angle[i+7]-'0';
        r = angle[i+8]-'0';
        k = angle[i+9]-'0';
        l = angle[i+10]-'0';
        m = angle[i+11]-'0';
        an1 = angle[i+12]-'0';
        an2 = angle[i+13]-'0';
        an3 = angle[i+14]-'0';
        pr1 = angle[i+15]-'0';
        pr2 = angle[i+16]-'0';
        pr3 = angle[i+17]-'0';
        
        a = (u*100)+ (v*10) + w;
        b = (x*100)+ (y*10) + z;
        c = (p*100)+ (q*10) + r;
        d = (k*100)+ (l*10) + m;
        e = (an1*100)+ (an2*10) + an3;
        f = (pr1*100)+ (pr2*10) + pr3;
        
        //Serial.println(a);
  
        /*Servo1.slowmove(a,255);
        Servo3.slowmove(180-a,255);                                     
        Servo2.slowmove(b,255);
        Servo4.slowmove(180-b,255);*/

        Servo1.write(180-a);
        Servo2.write(b);                                     
        Servo3.write(180-c);
        Servo4.write(d);
        Servo5.write(180-e);
        Servo6.write(f);                                     
    
      //}
      //angle = Serial.readString();
     
     

  /*if(i%2 == 0)
  {
    x_angle = -10;
    y_angle = 10;
    
  }
  else
  {
    x_angle = 10;
    y_angle = -10;
  }
  if(i<16)
  {
  Servo_yDir();
  Servo_xDir();
  delay(2000);
  i++ ;
  }*/
}
}
