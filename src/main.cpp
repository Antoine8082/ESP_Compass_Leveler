#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <MPU9250.h>
#include <Bounce2.h>
#include <EEPROM.h>
#include <ESP8266WiFi.h>
#include <D:\ESP32_Compass_Leveler\.pio\libdeps\esp_wroom_02\SimpleKalmanFilter\src\SimpleKalmanFilter.h>

MPU9250 imu(Wire, 0x68);
Adafruit_SSD1306 display(128, 64, &Wire, -1);
Bounce debouncer = Bounce();

SimpleKalmanFilter kalmanFilterPitch(1, 1, 0.01);
SimpleKalmanFilter kalmanFilterHeading(1, 1, 0.01);

const int LED1_PIN = 1;
const int LED2_PIN = 3;
const int CALIBRATION_BUTTON_PIN = 4;

const int NUM_READINGS = 20;
float pitchReadings[NUM_READINGS];
float headingReadings[NUM_READINGS];
int readIndex = 0;
float totalPitch = 0;
float totalHeading = 0;

int mode = 0; // 0 pour le mode de mesure, 1 pour le mode de calibration
int calibrationType = 0; // 0 pour la calibration du niveau, 1 pour la calibration de la boussole

void saveOffsets(float accX, float accY, float accZ, float magX, float magY, float magZ) {
    // Écriture des offsets dans la mémoire EEPROM
    EEPROM.put(0, accX);
    EEPROM.put(sizeof(float), accY);
    EEPROM.put(2 * sizeof(float), accZ);
    EEPROM.put(3 * sizeof(float), magX);
    EEPROM.put(4 * sizeof(float), magY);
    EEPROM.put(5 * sizeof(float), magZ);
    EEPROM.commit();
}

void loadOffsets(float &accX, float &accY, float &accZ, float &magX, float &magY, float &magZ) {
    // Lecture des offsets dans la mémoire EEPROM
    EEPROM.get(0, accX);
    EEPROM.get(sizeof(float), accY);
    EEPROM.get(2 * sizeof(float), accZ);
    EEPROM.get(3 * sizeof(float), magX);
    EEPROM.get(4 * sizeof(float), magY);
    EEPROM.get(5 * sizeof(float), magZ);
}

void setup() {
    Wire.begin();
    imu.begin();
    imu.setAccelRange(MPU9250::ACCEL_RANGE_2G);
    imu.setGyroRange(MPU9250::GYRO_RANGE_250DPS);
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    display.clearDisplay();
    display.display();
    pinMode(CALIBRATION_BUTTON_PIN, INPUT_PULLUP);
    pinMode(LED1_PIN, OUTPUT);
    pinMode(LED2_PIN, OUTPUT);
    debouncer.attach(CALIBRATION_BUTTON_PIN, INPUT_PULLUP);
    debouncer.interval(50);

    imu.calibrateGyro();
    imu.calibrateAccel();
    imu.calibrateMag();
    imu.begin();

    mode = 0;
}
void loop() {

    // Ajouter ce bloc pour détecter l'appui long sur le bouton de calibration
    debouncer.update();
    if (debouncer.fell()) {
        long startTime = millis();
        while (debouncer.read() == LOW) {
            if (millis() - startTime > 1000) {
                mode = 1;
                break;
            }
        }
    }

    if (mode == 0) {
        imu.readSensor();
        float x = imu.getAccelX_mss();
        float y = imu.getAccelY_mss();
        float z = imu.getAccelZ_mss();
        //float roll = atan2(y, z) * 180.0 / PI;
        float pitch = atan2(-x, sqrt(y * y + z * z)) * 180.0 / PI;

        // Ajout du filtre de Kalman et de la moyenne mobile pour le pitch
        float filteredPitch = kalmanFilterPitch.updateEstimate(pitch);
        totalPitch -= pitchReadings[readIndex];
        pitchReadings[readIndex] = filteredPitch;
        totalPitch += pitchReadings[readIndex];
        readIndex = (readIndex + 1) % NUM_READINGS;
        float averagePitch = totalPitch / NUM_READINGS;

        if (averagePitch >= 0 && averagePitch <= 0) {
            digitalWrite(LED1_PIN, HIGH);
        } else {
            digitalWrite(LED1_PIN, LOW);
        }
        display.clearDisplay();
        display.setCursor(0, 0);
        display.print("Pitch:");
        display.print(averagePitch, 1);
        display.display();
        delay(50);

        imu.readSensor();
        const float DECLINATION_ANGLE_NANTES = 0.0089; // 0°32' = (32/60)*3.14/180 = 0.0089 rad
        float heading = atan2(imu.getMagY_uT(), imu.getMagX_uT()) * 180.0 / PI - DECLINATION_ANGLE_NANTES;
        if (heading < 0) {
            heading += 360.0;
        }
        // Ajout du filtre de Kalman et de la moyenne mobile pour le cap
        float filteredHeading = kalmanFilterHeading.updateEstimate(heading);
        totalHeading -= headingReadings[readIndex];
        headingReadings[readIndex] = filteredHeading;
        totalHeading += headingReadings[readIndex];
        float averageHeading = totalHeading / NUM_READINGS;

        if (averageHeading >= 180 && averageHeading <= 180) {
            digitalWrite(LED2_PIN, HIGH);
        } else {
            digitalWrite(LED2_PIN, LOW);
        }
        display.setCursor(0, 10);
        display.print("Heading:");
        display.print(averageHeading, 1);
        display.print("   ");
        display.display();
        delay(50);
    }
    else {
        // Affichage pour sélectionner le type de calibration
        display.clearDisplay();
        display.setCursor(0, 0);
        display.print("Select Calibration:");
        display.setCursor(0, 10);
        if (calibrationType == 0) {
            display.setTextColor(WHITE, BLACK);
        } else {
            display.setTextColor(BLACK, WHITE);
        }
        display.print("Level");
        display.setTextColor(WHITE, BLACK);
        display.setCursor(64, 10);
        if (calibrationType == 1) {
            display.setTextColor(WHITE, BLACK);
        } else {
            display.setTextColor(BLACK, WHITE);
        }
        display.print("Compass");
        display.setTextColor(WHITE, BLACK);
        display.display();

        // Détection de l'appui court pour changer le type de calibration
        debouncer.update();
        if (debouncer.fell()) {
            if (calibrationType == 0) {
                calibrationType = 1;
            } else {
                calibrationType = 0;
            }
        }

        // Détection de l'appui long pour lancer la calibration
        if (debouncer.read() == LOW) {
            long startTime = millis();
            while (debouncer.read() == LOW) {
                if (millis() - startTime > 1000) {
                    if (calibrationType == 0) {
                        // Calibration du niveau
                        display.clearDisplay();
                        display.setCursor(0, 0);
                        display.print("Placez l'appareil");
                        display.setCursor(0, 10);
                        display.print("de niveau et");
                        display.setCursor(0, 20);
                        display.print("appuyez sur le");
                        display.setCursor(0, 30);
                        display.print("bouton");
                        display.display();
                        imu.calibrateAccel();
                        imu.readSensor();
                        float x_offset = -imu.getAccelX_mss();
                        float y_offset = -imu.getAccelY_mss();
                        float z_offset = -imu.getAccelZ_mss() - 9.81;
                        display.clearDisplay();
                        display.setCursor(0, 0);
                        display.print("Offset X:");
                        display.print(x_offset, 3);
                        display.setCursor(0, 10);
                        display.print("Offset Y:");
                        display.print(y_offset, 3);
                        display.setCursor(0, 20);
                        display.print("Offset Z:");
                        display.print(z_offset, 3);
                        display.display();
                        delay(2000);
                    } else {
                        // Calibration de la boussole
                        display.clearDisplay();
                        display.setCursor(0, 0);
                        display.print("Tournez l'appareil");
                        display.setCursor(0, 10);
                        display.print("autour de ses 3 axes");
                        display.setCursor(0, 20);
                        display.print("et appuyez sur le");
                        display.setCursor(0, 30);
                        display.print("bouton");
                        display.display();
                        imu.calibrateMag();
                        imu.readSensor();
                        float x_offset = -imu.getMagX_uT();
                        float y_offset = -imu.getMagY_uT();
                        float z_offset = -imu.getMagZ_uT();
                        display.clearDisplay();
                        display.setCursor(0, 0);
                        display.print("Offset X:");
                        display.print(x_offset, 3);
                        display.setCursor(0, 10);
                        display.print("Offset Y:");
                        display.print(y_offset, 3);
                        display.setCursor(0, 20);
                        display.print("Offset Z:");
                        display.print(z_offset, 3);
                        display.display();
                        delay(2000);
                    }
                    break;
                }
            }
        }
    }
}