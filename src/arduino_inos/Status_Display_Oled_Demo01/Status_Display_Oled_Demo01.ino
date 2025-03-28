#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Frame data
const char* batteryLevels[] = {"Batt 30%", "Batt 70%", "Batt 90%", "Batt 100%"};
const char* fanStatus[] = {"Fan Off", "Fan Off", "Fan On", "Fan Off"};
const char* bottomText[] = {"Moving", "Charging", "Cleaning", "WAIT"};

int frameIndex = 0;

void setup() {
    Serial.begin(115200);
    Wire.begin(22, 23);  //  SDA,SCL

    // Initialize OLED
    if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println("SSD1306 allocation failed");
        while (true);
    }

    display.clearDisplay();
}

void loop() {
    display.clearDisplay();

    // **Top Right (Battery Level)**
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(70, 0);  // Align to top-right
    display.println(batteryLevels[frameIndex]);

    // **Top Left (Fan Status)**
    display.setCursor(5, 0);
    display.println(fanStatus[frameIndex]);

    // **Bottom (Full Width Status)**
    display.setTextSize(2);
    display.setCursor(10, 40);
    display.println(bottomText[frameIndex]);

    // Update display
    display.display();

    // Cycle to next frame
    frameIndex = (frameIndex + 1) % 4;

    delay(3000); // Change frame every second
}
