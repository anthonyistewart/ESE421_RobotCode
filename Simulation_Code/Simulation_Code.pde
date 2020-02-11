import processing.serial.*;

Serial arduino;
String val;
int current_cone;

float conesX[] = {2000.0};
float conesY[] = {2000.0};

void setup(){
  String portName = Serial.list()[0];
  arduino = new Serial(this, portName, 115200);
  for(int i = 0; i < conesX.length; i++){
    ellipse(conesY[i], conesX[i], 2,2);
  }
  
}

void draw(){
  if(arduino.available() >0){
    
    //Format x,y,psi
    val = arduino.readStringUntil('\n');
    String[] list = split(val, ",");
    
    float x = float(list[0]);
    float y = float(list[1]);
    float psi = float(list[2]);
    
    point(y,x); //Reverse x and y because x goes North
    text("PSI: " + psi, width - 110, 30);
  }
}
