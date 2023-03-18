#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <MPU9250.h>
#include <Bounce2.h>
#include <EEPROM.h>
#include <ESP8266WiFi.h>


MPU9250 imu(Wire, 0x68);
Adafruit_SSD1306 display(128, 32, &Wire, -1);
Bounce debouncer = Bounce();

const int LED1_PIN = 1;
const int LED2_PIN = 3;
const int CALIBRATION_BUTTON_PIN = 4;

int mode = 0; // 0 pour le mode de mesure, 1 pour le mode de calibration
int calibrationType = 0; // 0 pour la calibration du niveau, 1 pour la calibration de la boussole

void saveOffsets(float accX, float accY, float accZ, float magX, float magY, float magZ) {
    // Conversion des offsets en entiers signés sur 2 octets pour économiser de l'espace
    int16_t accOffsets[3] = {(int16_t)(accX * 1000), (int16_t)(accY * 1000), (int16_t)(accZ * 1000)};
    int16_t magOffsets[3] = {(int16_t)(magX * 1000), (int16_t)(magY * 1000), (int16_t)(magZ * 1000)};

    // Écriture des offsets dans la mémoire EEPROM
    EEPROM.put(0, accOffsets);
    EEPROM.put(sizeof(accOffsets), magOffsets);
    EEPROM.commit();
}

void loadOffsets(float &accX, float &accY, float &accZ, float &magX, float &magY, float &magZ) {
    // Lecture des offsets dans la mémoire EEPROM
    int16_t accOffsets[3];
    int16_t magOffsets[3];
    EEPROM.get(0, accOffsets);
    EEPROM.get(sizeof(accOffsets), magOffsets);

    // Conversion des entiers signés en floats
    accX = (float)accOffsets[0] / 1000.0;
    accY = (float)accOffsets[1] / 1000.0;
    accZ = (float)accOffsets[2] / 1000.0;
    magX = (float)magOffsets[0] / 1000.0;
    magY = (float)magOffsets[1] / 1000.0;
    magZ = (float)magOffsets[2] / 1000.0;
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

        if (pitch >= 0 && pitch <= 0) {
            digitalWrite(LED1_PIN, HIGH);
        } else {
            digitalWrite(LED1_PIN, LOW);
        }
        display.clearDisplay();
        display.setCursor(0, 0);
        display.print("Pitch:");
        display.print(pitch, 1);
        display.display();
        delay(100);

        imu.readSensor();
        float heading = atan2(imu.getMagY_uT(), imu.getMagX_uT()) * 180.0 / PI;
        if (heading < 0) {
            heading += 360.0;
        }
        if (heading >= 180 && heading <= 180) {
            digitalWrite(LED2_PIN, HIGH);
        } else {
            digitalWrite(LED2_PIN, LOW);
        }
        display.setCursor(0, 10);
        display.print("Heading:");
        display.print(heading, 1);
        display.print("   ");
        display.display();
        delay(100);
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
                        // Code pour la calibration du niveau
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
                        // Code pour la calibration de la boussole
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
