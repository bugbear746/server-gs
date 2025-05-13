import time
import requests
import board
import busio
import RPi.GPIO as GPIO
import logging
from io import BytesIO
from PIL import Image
import adafruit_bmp3xx
import adafruit_sht31d
from mpu6050 import mpu6050
import serial
import subprocess
import math


SERVER_URL = "https://15d677ac-b5e3-4d04-9dd1-469e293a8c86-00-1jttv25iv6uz3.sisko.replit.dev"
IMAGE_UPLOAD_INTERVAL = 10
TELEMETRY_UPLOAD_INTERVAL = 2
DEPLOY_PIN = 17
GPS_PORT = "/dev/ttyAMA0"
GPS_BAUDRATE = 9600
DEPLOY_ALTITUDE_THRESHOLD = 650
FALLING_THRESHOLD = 9.0
FALLING_DURATION = 0.5
REQUEST_TIMEOUT = 10  


logging.basicConfig(filename="cansat.log", level=logging.INFO, format='%(asctime)s %(message)s')


i2c = busio.I2C(board.SCL, board.SDA)
bmp = adafruit_bmp3xx.BMP3XX_I2C(i2c)
sht31 = adafruit_sht31d.SHT31D(i2c)
mpu = mpu6050(0x68)


GPIO.setmode(GPIO.BCM)
GPIO.setup(DEPLOY_PIN, GPIO.OUT)
servo = GPIO.PWM(DEPLOY_PIN, 50)
servo.start(0)


gps = serial.Serial(GPS_PORT, GPS_BAUDRATE, timeout=1)

def parse_gps(data):
    if "," in data and data.startswith("$GPGGA"):
        parts = data.split(",")
        if len(parts) > 5 and parts[2] and parts[4]:
            try:
                lat = float(parts[2]) / 100.0
                lon = float(parts[4]) / 100.0
                return lat, lon
            except:
                return None, None
    return None, None

def rotate_servo_once():
    servo.ChangeDutyCycle(12.5)
    time.sleep(0.5)
    servo.ChangeDutyCycle(0)
    logging.info("Parachute deployed via servo rotation")
    time.sleep(0.5)

def capture_image():
    try:
        subprocess.run([
            "libcamera-still", "-o", "/tmp/frame.jpg", "-n",
            "--width", "640", "--height", "480", "--timeout", "1000"
        ], check=True)
        with open("/tmp/frame.jpg", "rb") as f:
            buffer = BytesIO(f.read())
        return buffer
    except Exception as e:
        logging.error(f"Image capture failed: {e}")
        return None

def read_sensors():
    try:
        temperature = sht31.temperature
        humidity = sht31.relative_humidity
        pressure = bmp.pressure
        altitude = bmp.altitude
        accel_data = mpu.get_accel_data()

        gps_data = gps.readline().decode('ascii', errors='replace')
        lat, lon = parse_gps(gps_data)

        return {
            "temperature": round(temperature, 2),
            "humidity": round(humidity, 2),
            "pressure": round(pressure, 2),
            "altitude": round(altitude, 2),
            "acceleration_x": accel_data['x'],
            "acceleration_y": accel_data['y'],
            "acceleration_z": accel_data['z'],
            "latitude": lat,
            "longitude": lon
        }
    except Exception as e:
        logging.error(f"Sensor read error: {e}")
        return {}

def upload_telemetry():
    data = read_sensors()
    try:
        if data:
            response = requests.post(f"{SERVER_URL}/telemetry", json=data, timeout=REQUEST_TIMEOUT)
            if response.status_code == 200:
                logging.info("Telemetry uploaded")
            else:
                logging.warning(f"Telemetry upload returned {response.status_code}")
    except requests.RequestException as e:
        logging.error(f"Telemetry upload failed: {e}")
        reconnect_network()

def upload_image(frame):
    try:
        if frame:
            files = {'image': ('frame.jpg', frame, 'image/jpeg')}
            response = requests.post(f"{SERVER_URL}/upload_image", files=files, timeout=REQUEST_TIMEOUT)
            if response.status_code == 200:
                logging.info("Image uploaded")
            else:
                logging.warning(f"Image upload returned {response.status_code}")
    except requests.RequestException as e:
        logging.error(f"Image upload failed: {e}")
        reconnect_network()

def reconnect_network():
    logging.info("Attempting to reconnect network")
    try:
        # Simple reconnect by resolving DNS again
        requests.get(f"{SERVER_URL}", timeout=REQUEST_TIMEOUT)
        logging.info("Network reconnected")
    except requests.RequestException as e:
        logging.error(f"Network reconnect failed: {e}")

def check_deploy(altitude, accel_x, accel_y, accel_z):
    try:
        response = requests.get(f"{SERVER_URL}/deploy", timeout=REQUEST_TIMEOUT)
        if response.text.strip() == "DEPLOY":
            logging.info("DEPLOY command received from server")
            rotate_servo_once()
            return

        accel_magnitude = math.sqrt(accel_x**2 + accel_y**2 + accel_z**2)
        logging.debug(f"Accel: {accel_x}, {accel_y}, {accel_z}, Mag: {accel_magnitude} m/s²")

        global last_fall_check_time, fall_detected
        if not hasattr(check_deploy, 'last_fall_check_time'):
            check_deploy.last_fall_check_time = time.time()
            check_deploy.fall_detected = False
            rest_magnitude = math.sqrt(accel_x**2 + accel_y**2 + accel_z**2)
            logging.info(f"Calibrated rest magnitude: {rest_magnitude} m/s²")

        current_time = time.time()
        if (abs(accel_magnitude - 9.81) < 1.0) and not fall_detected:
            if current_time - check_deploy.last_fall_check_time > FALLING_DURATION:
                logging.info("Descent detected, deploying parachute")
                rotate_servo_once()
                check_deploy.fall_detected = True
            check_deploy.last_fall_check_time = current_time
        elif accel_magnitude > FALLING_THRESHOLD:
            check_deploy.fall_detected = False
            check_deploy.last_fall_check_time = current_time

        if altitude >= DEPLOY_ALTITUDE_THRESHOLD and not fall_detected:
            logging.info(f"Auto-deploy triggered at altitude {altitude}m")
            rotate_servo_once()

    except requests.RequestException as e:
        logging.error(f"Deploy check failed: {e}")
        reconnect_network()

last_telemetry_time = 0
last_img_time = 0

logging.info("Starting CanSat")

try:
    while True:
        now = time.time()

        if now - last_telemetry_time > TELEMETRY_UPLOAD_INTERVAL:
            upload_telemetry()
            last_telemetry_time = now

        if now - last_img_time > IMAGE_UPLOAD_INTERVAL:
            frame = capture_image()
            if frame:
                upload_image(frame)
                last_img_time = now

        sensor_data = read_sensors()
        if sensor_data:
            accel_x = sensor_data.get("acceleration_x", 0)
            accel_y = sensor_data.get("acceleration_y", 0)
            accel_z = sensor_data.get("acceleration_z", 0)
            check_deploy(sensor_data.get("altitude", 0), accel_x, accel_y, accel_z)
        time.sleep(0.5)

except KeyboardInterrupt:
    logging.info("Shutting down CanSat")
    servo.stop()
    GPIO.cleanup()
