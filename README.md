# AutonomousVehicle
Using a TIVA TM4CGH6PM microcontroller to control a vehicle. 
The vehicle has a mounted ultrasonic sensor that continuously measures the distance between the vehicle and any objects in front of it. If there is an object detected, a servo will rotate the sensor first 45 degrees counter clockwise, 45 degrees counter clockwise, 90 degrees clockwise, and 90 degrees counter clockwise, until it finds an unobstructed path for the vehicle to turn to. 
The vehicle will default to driving straight, but when an object is detected, the open path decides the PWM cycle of the 4 DC motors. The motor can turn to any of the degrees that the servo and ultrasonic sensor can turn to.
Using timer interrupts, the microcontroller perodically checks for bluetooth input from the user. If a valid command is received, movement is briefly paused to execute the user commands.
Possible commands are check for current temperature and check the current distance between the vehicle and any object straight forwards. 
