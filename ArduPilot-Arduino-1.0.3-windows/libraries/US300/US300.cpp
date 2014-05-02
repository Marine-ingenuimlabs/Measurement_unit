

float US300::get_depth(void) 
{
	   k=0;
  // read the input on analog pin 0:
  
  
   for ( i=0 ; i<64 ; i++){
         
         k +=(float) analogRead(A8);
         
      }
      sensorValue = k/64.0;
  voltage = ((sensorValue * 5.0) / 1023.0);
  voltage = a * voltage + (1-a) * last_voltage;
  pressure = ((voltage-voltage_init) / (4.0));
  p = pressure*100.0;
  depth = 70.0 * p;
  last_voltage = voltage;
	return cos(roll) * omega.y - sin(roll) * omega.z;
}