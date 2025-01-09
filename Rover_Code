import brickpi3  #import the BrickPi3 drivers
import time
import grovepi
import math
from MPU9250 import MPU9250

def movement(arrayMain, fileHazard):
  direction = 1
  rowCurrent = 25
  colCurrent = 1
  highIR = 0
  arrayMain[rowCurrent][0] = '5'
    #1 means moving in x
    #2 means moving in y
    #- means negative direction
  right(0)
  #the number inside () indicated turning angle - 0 used to make sure no error inside function
  [totalInit, rate] = BP.get_sensor(BP.PORT_4)
  #get initial heading, rate unused
  while True:
    while grovepi.ultrasonicRead(us_front) > 15 and grovepi.ultrasonicRead(us_front) != 255:
      #while space infront of robot
      if(grovepi.ultrasonicRead(us_right) < 13):
        BP.set_motor_power(BP.PORT_C, 0)
        BP.set_motor_power(BP.PORT_A, 30)
        time.sleep(.05)
        #micro turns for checking heading, left
      if(grovepi.ultrasonicRead(us_right) > 20 and grovepi.ultrasonicRead(us_right) != 255):
        BP.set_motor_power(BP.PORT_C, 30)
        BP.set_motor_power(BP.PORT_A, 0)
        time.sleep(.05)
        #micro turns for checking heading, right
      [total, rate] = BP.get_sensor(BP.PORT_4)
      #gets heading as total
      while total > totalInit + 3:
        BP.set_motor_power(BP.PORT_C, -20)
        BP.set_motor_power(BP.PORT_A, 20)
        [total, rate] = BP.get_sensor(BP.PORT_4)
        #if robot skews off heading suppsoed to correct
      while total < totalInit - 3:
        BP.set_motor_power(BP.PORT_C, 20)
        BP.set_motor_power(BP.PORT_A, -20)
        [total, rate] = BP.get_sensor(BP.PORT_4)
        #if robot skews off heading suppsoed to correct
      BP.set_motor_power(BP.PORT_A, 20)
      BP.set_motor_power(BP.PORT_C, 20)
      time.sleep(1)
      #time step to plot every 10 cm
      #below statments used to check direction, and then plot path accordingly
      if direction == 1:
        arrayMain[rowCurrent][colCurrent] = 1
        colCurrent += 1
        arrayMain[rowCurrent][colCurrent] = 4
      if direction == 2:
        arrayMain[rowCurrent][colCurrent] = 1
        rowCurrent -= 1
        arrayMain[rowCurrent][colCurrent] = 4
      if direction == -1:
        arrayMain[rowCurrent][colCurrent] = 1
        colCurrent -= 1
        arrayMain[rowCurrent][colCurrent] = 4
      if direction == -2:
        arrayMain[rowCurrent][colCurrent] = 1
        rowCurrent += 1
        arrayMain[rowCurrent][colCurrent] = 4
    valZ = magZ()
    #gets mag data
    if (valZ > 100.0 or valZ < 100):
      BP.set_motor_power(BP.PORT_C, 0)
      BP.set_motor_power(BP.PORT_A, 0)
      #plots and writes obstacle to file
      if direction == 1:
        arrayMain[rowCurrent][colCurrent + 1] = 3
        fileHazard.write("\nMRI, Field Strength(uT), %.2f, %d cm, %d cm" %(valZ, (colCurrent + 1) * 10, rowCurrent * 10))
      if direction == 2:
        arrayMain[rowCurrent - 1][colCurrent] = 3
        fileHazard.write("\nMRI, Field Strength(uT), %.2f, %d cm, %d cm" %(valZ, colCurrent * 10, (rowCurrent - 1) * 10))
      if direction == -1:
        arrayMain[rowCurrent][colCurrent - 1] = 3
        fileHazard.write("\nMRI, Field Strength(uT), %.2f, %d cm, %d cm" %(valZ, (colCurrent - 1) * 10, rowCurrent * 10))
      if direction == -2:
        arrayMain[rowCurrent + 1][colCurrent] = 3
        fileHazard.write("\nMRI, Field Strength(uT), %.2f, %d cm, %d cm" %(valZ, colCurrent * 10, (rowCurrent + 1) * 10))
      if grovepi.ultrasonicRead(us_right) > 25:    
        #-90
        right(90)
        [totalInit, rate] = BP.get_sensor(BP.PORT_4)
        direction = rightPos(direction)
        #turns right 90 if room to the right
      else:
        right(180)
        #turns 180 if no room to the right
        [totalInit, rate] = BP.get_sensor(BP.PORT_4)
        direction = fullTurnPos(direction)
    [leftIR, rightIR] = IR_Read(grovepi)
    if(leftIR >= rightIR):
      #IR reads 2 values, only uses highest value
      highIR = int(leftIR)
    else:
      highIR = int(rightIR)
    if(highIR >= 50):
      BP.set_motor_power(BP.PORT_C, 0)
      BP.set_motor_power(BP.PORT_A, 0)
      #does the same thing for IR as MAG values above
      if direction == 1:
        arrayMain[rowCurrent][colCurrent + 1] = 2
        fileHazard.write("\nCaesium-137, Radiated Power(W), %d, %d cm, %d cm" %(highIR, (colCurrent + 1) * 10, rowCurrent * 10))
      if direction == 2:
        arrayMain[rowCurrent - 1][colCurrent] = 2
        fileHazard.write("\nCaesium-137, Radiated Power(W), %d, %d cm, %d cm" %(highIR, colCurrent * 10, (rowCurrent - 1) * 10))
      if direction == -1:
        arrayMain[rowCurrent][colCurrent - 1] = 2
        fileHazard.write("\nCaesium-137, Radiated Power(W), %d, %d cm, %d cm" %(highIR, (colCurrent - 1) * 10, rowCurrent * 10))
      if direction == -2:
        arrayMain[rowCurrent + 1][colCurrent] = 2
        fileHazard.write("\nCaesium-137, Radiated Power(W), %d, %d cm, %d cm" %(highIR, colCurrent * 10, (rowCurrent + 1) * 10))
      if grovepi.ultrasonicRead(us_right) > 30:    
        #-90
        right(90)
        [totalInit, rate] = BP.get_sensor(BP.PORT_4)
        direction = rightPos(direction)
      else:
        right(180)
        [totalInit, rate] = BP.get_sensor(BP.PORT_4)
        direction = fullTurnPos(direction)
    
    BP.set_motor_power(BP.PORT_C, 0)
    BP.set_motor_power(BP.PORT_A, 0)
    if grovepi.ultrasonicRead(us_right) > 25 and grovepi.ultrasonicRead(us_right) != 255:    
      #-90
      right(90)
      [totalInit, rate] = BP.get_sensor(BP.PORT_4)
      direction = rightPos(direction)

    elif grovepi.ultrasonicRead(us_right) < 25:   
      #90
      left(90)
      [totalInit, rate] = BP.get_sensor(BP.PORT_4)
      direction = leftPos(direction)

def fullTurnPos(direction):
  #changes direction value based off of initial direction
  #180 turn
  if direction == 1:
    direction = -1
  elif direction == -2:
    direction = 2
  elif direction == -1:
    direction = 1
  else:
    direction = -2
  return direction

def rightPos(direction):
  #changes direction for a 90 right turn
  if direction == 1:
    direction = -2
  elif direction == -2:
    direction = -1
  elif direction == -1:
    direction = 2
  else:
    direction = 1
  return direction

def leftPos(direction):
  #changes direction for a 90 left turn
  if direction == 1:
    direction = 2
  elif direction == 2:
    direction = -1
  elif direction == -1:
    direction = -2
  else:
    direction = 1
  return direction

#right function
def right(x):
    while True:
        try:
            #this is used during that first right(0) to get sensor reading accurate values
            #after first pass, the try and except is skipped basically
            BP.get_sensor(BP.PORT_4)
        except brickpi3.SensorError as error:
            print(error)
            continue
        else:
            [totalInit, rateInit] = BP.get_sensor(BP.PORT_4)
            total = totalInit
            while total < totalInit + x:
                #turns until x value is acheived
                BP.set_motor_power(BP.PORT_C, 20)
                BP.set_motor_power(BP.PORT_A, -20)
                [total, rate] = BP.get_sensor(BP.PORT_4)
            BP.set_motor_power(BP.PORT_C, 0)
            BP.set_motor_power(BP.PORT_A, 0)
            break

#left function 
def left(x):
    #same as right function excpet left turn
    while True:
        try:
            BP.get_sensor(BP.PORT_4)
        except brickpi3.SensorError as error:
            print(error)
            continue
        else:
            [totalInit, rateInit] = BP.get_sensor(BP.PORT_4)
            total = totalInit
            while total > totalInit - x:
                BP.set_motor_power(BP.PORT_C, -20)
                BP.set_motor_power(BP.PORT_A, 20)
                [total, rate] = BP.get_sensor(BP.PORT_4)
            BP.set_motor_power(BP.PORT_C, 0)
            BP.set_motor_power(BP.PORT_A, 0)
            break



def magZ():
  mag = mpu9250.readMagnet()
  #reads z direction magnet
  return mag['z']
    
def IR_setup(grovepi):
  sensor1= 14		# Pin 14 is A0 Port.
  sensor2 = 15		# Pin 15 is A0 Port.
  grovepi.pinMode(sensor1,"INPUT")
  grovepi.pinMode(sensor2,"INPUT")


#Read Function		
def IR_Read(grovepi):
  try:
    sensor1= 14		# Pin 14 is A0 Port.
    sensor2 = 15		# Pin 15 is A0 Port.                
    sensor1_value = grovepi.analogRead(sensor1)
    sensor2_value = grovepi.analogRead(sensor2)
    #returns IR sensor value
    return [sensor1_value, sensor2_value]

  except IOError:
    print ("Error")

def cargoRelease():
  #call wherever we indicate robot is at end of the maze
  BP.set_motor_power(BP.PORT_B, -35)
  time.sleep(2)
  BP.set_motor_power(BP.PORT_B, 0)
  time.sleep(2)
    #releases cargo

  BP.set_motor_power(BP.PORT_A, 20)
  BP.set_motor_power(BP.PORT_C, 20)
  time.sleep(.5)
  BP.set_motor_power(BP.PORT_A, 0)
  BP.set_motor_power(BP.PORT_C, 0)
  #drives forward slightly

  BP.set_motor_power(BP.PORT_B, 25)
  time.sleep(2)
  BP.set_motor_power(BP.PORT_B, 0)
  #closes cargo hold
  
  
try:
  rows, cols = (50, 50)
  #used for 2D array
  arrayMain = [[0 for x in range(rows)] for y in range(cols)]
  #intializes array to all 0's
  BP = brickpi3.BrickPi3()
  BP.set_sensor_type(BP.PORT_4, BP.SENSOR_TYPE.EV3_GYRO_ABS_DPS)
  #gyro set up
  mpu9250 = MPU9250()
  #imu reading setup
  #pi wifi pass = team_55
  #ip 192.168.55.2
  #used for MAG
  us_front = 7
  #ultrasonic front port
  us_right = 4
  #ultrasonic right port
  map = int(input("Input map number"))
  fileMap = open("team55_map.csv",'w')
  fileHazard = open("team55_hazards.csv", 'w')
  fileHazard.write("Team: 55\nMap: %d \nNotes:\n\nResource Type, Parameter of Interest, Parameter, Resource X Coordinate, Resource Y Coordinate" %map)
  fileMap.write("Team: 55\nMap: %d \nUnit Length: 10\nUnit: cm\nOrigin: (0,24)\nNotes\n" %map)
  IR_setup(grovepi)
  movement(arrayMain, fileHazard)

except KeyboardInterrupt:
  #when exiting maze, writes the array to the file
  for x in range(50):
    for y in range(50):
      if y == 49:
        fileMap.write(str(arrayMain[x][y]))
      else:
        fileMap.write(str(arrayMain[x][y]))
        fileMap.write(',')
    fileMap.write("\n")
  cargoRelease()
  fileHazard.close()
  fileMap.close()
  BP.reset_all()
