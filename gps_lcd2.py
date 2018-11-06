import serial
import RPi.GPIO as GPIO
import time
import utm
import datetime
from Adafruit_CharLCD import Adafruit_CharLCD
from time import sleep

#initiate the GPIO Pins for LED Lights
GPIO.setmode(GPIO.BCM)
GPIO.setup(18, GPIO.OUT) #Green Light
GPIO.setup(12, GPIO.OUT) #Red Light

#iniitiate LCD & Specify Pins
lcd = Adafruit_CharLCD(rs=26, en=19, d4=13, d5=6, d6=5, d7=11, cols=16, lines=2)

#display text on LCD Display \n = new line
lcd.message('GPS & LCD Test\n Pablo JC')
sleep(3)

from shapely.geometry import Point, Polygon

#polygon for hampton building
#poly = Polygon(((40.43054,-86.91549), (40.43082,-86.91496), (40.43004,-86.91406), (40.42966,-86.91465)))
#coords =((40.43054,-86.91549), (40.43082,-86.91496), (40.43004,-86.91406), (40.42966,-86.91465))

#polygon for VOSS Structure
poly = Polygon(((40.42183,-86.92268),(40.42187,-86.92282),(40.42183,-86.92295),(40.42177,-86.92301),(40.42167,-86.923),(40.42162,-86.92296),(40.42159,-86.9229),(40.42158,-86.92279),(40.42162,-86.92268),(40.42172,-86.92262)))
coords = ((40.42183,-86.92268),(40.42187,-86.92282),(40.42183,-86.92295),(40.42177,-86.92301),(40.42167,-86.923),(40.42162,-86.92296),(40.42159,-86.9229),(40.42158,-86.92279),(40.42162,-86.92268),(40.42172,-86.92262))

#polygon for Chauncey Square
#poly = Polygon(((40.42451,-86.90571),(40.42452,-86.90688), (40.42398,-86.90684),(40.42395,-86.90573)))
#coords = ((40.42451,-86.90571),(40.42452,-86.90688), (40.42398,-86.90684),(40.42395,-86.90573))

#tup0=coords[0]
#tup1=coords[1]
#tup2=coords[2]
#tup3=coords[3]
coords_utm=[]

for a,b in coords:
	coords_utm_point=(utm.from_latlon(a,b))
	lat_poly_utm = coords_utm_point[0]
	long_poly_utm = coords_utm_point[1]
	print('first', a, 'then', b)
	coord_tup =(lat_poly_utm, long_poly_utm)
	print(coord_tup)
	coords_utm.append(coord_tup)
print(coords_utm)
#r = LinearRing(coords_utm)
poly_utm = Polygon(coords_utm)

#point = Point(40.430454,-86.914619)
#point from hampton lab for testing

ser = serial.Serial('/dev/ttyUSB0', 4800, timeout=5 )
lcd.clear()

while 1:
    line=ser.readline()
    splitline=line.split(',')

    if splitline[0]=='$GPGGA':
        lattitude=float(splitline[2])
        latDirec=splitline[3]
        longitude=float(splitline[4])
        longDirec=splitline[5]
	#long lat comes in Degrees, minutes Ex: 4025.8286 = 40d 25.8286'(minutes)

	#next portion converts from GPS Degrees, Minutes to GPS Degrees (minutes/60)
	deg_lat = float(str(lattitude)[:2])
	min_lat = (lattitude - deg_lat*100)/60
	lat_mod = deg_lat + min_lat
	
	# print (lattitude+deg_lat+min_lat+lat_mod)

	deg_long = float(str(longitude)[:2])
        min_long = (longitude - deg_long*100)/60
        long_mod = deg_long + min_long

	
	if latDirec == 'S':
		lat_mod=lat_mod*-1
#		print ('its S', lattitude)
#	else:
#		print (latDirec)
	if longDirec == 'W':
		long_mod=long_mod*-1
#		#print ('its W', longitude)
#	else:
#		print(longDirec)	
	print (lat_mod, long_mod)
	point = Point(lat_mod, long_mod)

## GET CURRENT TIME
	currentDT = datetime.datetime.now()

### CONVERT POINT TO UTM
        point_utm = utm.from_latlon(lat_mod, long_mod)
#       print('point utm')
#       print(point_utm)
        lat_p_utm = point_utm[0] #float(point_utm[0])
        long_p_utm = point_utm[1] ##float(point_utm[1])
        point_utm_done = Point(lat_p_utm, long_p_utm)
#       print(type(lat_p_utm))

##      calculate the distnace to the nearest boundary
        boundary_distance = 3.28084*poly_utm.boundary.distance(point_utm_done)
        exterior_distance = 3.28084*poly_utm.exterior.distance(point_utm_done)
        bound_dist = '%.3f' % boundary_distance
        ext_dist = '%.3f' % exterior_distance
        print(boundary_distance)
        print(exterior_distance)

#        lcd.lcd_byte(lcd.LCD_LINE_2, lcd.LCD_CMD)
#        lcd.lcd_string(bound_dist+' ft', 2)

### CHECK IF POLYGON CONTIANS POINT

	if poly.contains(point):
        	print 'inside'
		status = "inside"
#		lcd.lcd_byte(lcd.LCD_LINE_1, lcd.LCD_CMD)
#		lcd.lcd_string("Inside Area", 2)
		lcd.message('INSIDE AREA\n'+ bound_dist + ' Ft')
		GPIO.output(18, GPIO.HIGH)
		time.sleep(0.01)
		GPIO.output(18, GPIO.LOW)
		GPIO.output(12, GPIO.LOW)
		lcd.home()
	else:
        	print 'outside'
		status = "outsiide"
#		lcd.lcd_byte(lcd.LCD_LINE_1, lcd.LCD_CMD)
#               lcd.lcd_string("Outside Area", 2)
		lcd.message('OUTSIDE AREA\n'+ bound_dist + ' Ft')
		GPIO.output(12, GPIO.HIGH)
		time.sleep(0.01)
		GPIO.output(18, GPIO.LOW)
		GPIO.output(12, GPIO.LOW)
		lcd.home()


##	WRITE TO FILE
	
	file=open("testfile.txt","a")
	printline = "\n" + str(currentDT) + ", " + str(lat_mod) + ", " + str(long_mod) + ", " + status
	print(printline)
	file.write(printline)
	file.close()
#	display distance on LCD

GPIO.cleanup ()

