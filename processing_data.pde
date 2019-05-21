/*
* ENEE461 Final Project: Data Collection
*
* This sketch takes input from the serial monitor that was sent from Arduino
* and formats it into a .csv file. 
*/

import processing.serial.*;

Serial myPort;  // Create object from Serial class
Table data;     // Table for storing data from Arduino
String val;     // Data received from the serial port
String fileName = "controller_data.csv";


void setup()
{
  // Arduino is sending its serial output to port COM3
  String portName = Serial.list()[0]; 
  myPort = new Serial(this, portName, 9600);
  
  data = new Table();
  data.addColumn("Time", Table.FLOAT);
  data.addColumn("Target Distance", Table.FLOAT);
  data.addColumn("Current Distance", Table.FLOAT);
  data.addColumn("Dist Error", Table.FLOAT);
  data.addColumn("New Speed", Table.FLOAT);
}

void draw()
{
  if(myPort.available() > 0) {
    val = myPort.readStringUntil('\n');
    if(val != null) {
      val = trim(val);
      println(val);
      float vals[] = float(split(val, ','));
      
      TableRow newRow = data.addRow(); 
      newRow.setFloat("Time", vals[0]);
      newRow.setFloat("Target Distance", vals[1]);
      newRow.setFloat("Current Distance", vals[2]);
      newRow.setFloat("Dist Error", vals[3]);
      newRow.setFloat("New Speed", vals[4]);
    }
  }
  
} 

void keyPressed() {
  saveTable(data, fileName);
  exit();  
}
