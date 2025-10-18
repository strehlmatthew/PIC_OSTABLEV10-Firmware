#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>
#include <LittleFS.h>
// ----------------------------
// TFT CONFIGURATION
// ----------------------------
#define TFT_CS 17
#define TFT_DC 16
#define TFT_RST 20
#define SCREEN_WIDTH 240
#define SCREEN_HEIGHT 240
#define CHAR_WIDTH 6
#define LINE_HEIGHT 9
#define MAX_LINES (SCREEN_HEIGHT / LINE_HEIGHT)
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
const int COLS = SCREEN_WIDTH / CHAR_WIDTH;
const int WRAP_COLS = COLS - 1; 
// ----------------------------
// SERIAL UPLOAD PROTOCOL CONFIG
// ----------------------------
const String ACK_MSG = "ACK\r\n";
const String READY_MSG = "READY\r\n\r\n"; 
const String UPLOAD_OK_MSG = "UPLOAD_OK";
const String FATAL_ERROR_MSG = "FATAL ERROR:";
const size_t BLOCK_SIZE = 512;
// ----------------------------
// SERIAL UPLOAD IMPLEMENTATION
// ----------------------------
bool fsReady = false;
int formatIndex = 0;
#define WDT_DISABLE() wdt_disable_platform()
#define WDT_ENABLE() wdt_enable_platform()
/**
 * @brief Reads a specific number of bytes from the serial port with a timeout.
 * * This is crucial for reading binary data. It blocks until the required bytes 
 * are available or the timeout is reached.
 * * @param buffer Pointer to the destination buffer.
 * @param count The number of bytes to read.
 * @param timeoutMs Timeout in milliseconds.
 * @return size_t The number of bytes successfully read.
 */
size_t serialBlockRead(uint8_t* buffer, size_t count, unsigned long timeoutMs = 3000) {
    size_t bytesRead = 0;
    unsigned long start = millis();
    
    while (bytesRead < count) {
        if (Serial.available()) {
            buffer[bytesRead++] = Serial.read();
            start = millis(); // Reset timeout on new data
        } else if (millis() - start > timeoutMs) {
            return bytesRead; // Timeout occurred
        }
        yield(); // Allow other tasks to run
    }
    return bytesRead;
}
// ----------------------------
// LED CONFIGURATION
// ----------------------------
#define STATUS_LED_PIN 25 // <--- CHANGE THIS TO YOUR ACTUAL LED PIN
#define LED_BLINK_DURATION_MS 50 // How long the LED stays on
unsigned long ledBlinkEndTime = 0; // When to turn the LED off
// ----------------------------
// BUTTONS (TTP223)
// ----------------------------
#define NUM_BUTTONS 4
const int buttonPins[NUM_BUTTONS] = {2, 3, 4, 5};
unsigned long lastPressTime[NUM_BUTTONS] = {0};
const unsigned long pressCooldown = 200;
#define IDX_PREV 0
#define IDX_NEXT 1
#define IDX_SELECT 2
#define IDX_BACK 3
// ----------------------------
// Keyboard layers
// ----------------------------
enum KMode { ALPHA, ALPHA_LOWER, NUM, SYM, CTRL, FUNC_VIEW };
const int NUM_PRIMARY_MODES = 4;
KMode kmode = ALPHA;
const char alphaChars[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ";
const char alphaLowerChars[] = "abcdefghijklmnopqrstuvwxyz";
const char numberChars[] = "0123456789";
const char symbolChars[] = ".,!?;:'\"-_=+()[]{}<>/\\|@#$%^&*`~";
const String ctrlKeys[] = {"SPACE","ENTER","DELETE","LEFT","RIGHT"}; 
const String funcKeys[] = {"F1", "F2", "F3", "F4", "F5", "F6", "F7", "F8", "F9", "F10", "F11", "F12"}; 
const char* modeLabels[] = {"ALPHA", "NUM", "SYM", "CTRL"}; 
const int CTRL_COUNT = 5;
const int FUNC_COUNT = 12;
const double PI_VALUE = 3.14159265358979323846; // High precision PI
int kbIndex = 0; // current character index (0 = mode label)
// ----------------------------
// F-Key Prompting States
// ----------------------------
enum FKeyState { F_INACTIVE, F2_AWAIT_CHAR, F4_AWAIT_CHAR, F7_AWAIT_INDEX, F9_AWAIT_INDEX, F_AWAIT_FORMAT_CONFIRM};
FKeyState fkeyState = F_INACTIVE;
String lastCommand = ""; 
int f1_copy_index = 0; 
extern bool awaitingFormatConfirm; 
// Helper function to get the current mode's name (for mode label and preview)
const char* kbGetModeName() {
    // FIX: Removed the check for fkeyState to allow the current mode name (ALPHA, NUM, etc.) to be displayed.
    // The visual cue for F-INPUT AWAIT is now done via coloring in drawCursorAndPreview().
    if (kmode == ALPHA || kmode == ALPHA_LOWER) return "ALPHA";
    if (kmode == NUM) return "NUM";
    if (kmode == SYM) return "SYM";
    if (kmode == CTRL) return "CTRL";
    if (kmode == FUNC_VIEW) return "FUNC";
    return "MODE";
}
// ----------------------------
// SETUP
// ----------------------------
unsigned long startMillis = 0;
String deviceVersion = "PICOS CLI v7.77"; // UPDATED VERSION
bool cursorVisible = true;
unsigned long lastBlink = 0;
const unsigned long BLINK_MS = 600;
// ----------------------------
// Terminal buffers
// ----------------------------
#define CMD_BUF 512
char cmdBuf[CMD_BUF];
int cmdLen = 0;
int cursorPos = 0;
bool inputWrapped = false;
int lastPreviewCols = 0;
int lastCursorCol = -1;
int lastCursorRowY = -1;
int lastGlobalCursorPos = -1;
// ----------------------------
// SHELL PROMPT
// ----------------------------
const String PROMPT = "PICOS> ";
inline int promptCols() { return PROMPT.length(); }
// ----------------------------
// SCROLLBACK DEFINITIONS (FIXED ORDER)
// ----------------------------
#define SCROLLBACK_SIZE 50 // MOVED UP
//Holds the text and the color for each entry
struct ScrollbackEntry {
    String text;
    uint16_t color; // Color for the message
};
ScrollbackEntry scrollback[SCROLLBACK_SIZE]; // Array of ScrollbackEntry structs
int scrollbackHead = 0;
int scrollbackCount = 0;
int terminalScrollOffset = 0;
#define HISTORY_SIZE 64
String history[HISTORY_SIZE];
int historyCount = 0;
int historyIndex = -1; // -1 means no command is loaded from history. historyCount means "new command" state.
int recallPos = 0;
// ----------------------------
// SHELL COLORS
// ----------------------------
const String SYS_PROMPT = "SYS> ";
// Standard 16-bit colors
#define ST77XX_GREEN 0x07E0
#define ST77XX_CYAN 0x07FF // For ALPHA mode
#define ST77XX_WHITE 0xFFFF
#define ST77XX_BLACK 0x0000
#define ST77XX_YELLOW 0xFFE0
#define ST77XX_RED   0xF800 // For FUNC mode & F-INPUT AWAIT
#define ST77XX_MAGENTA 0xF81F // For SYM mode
#define ST77XX_DARK_ORANGE 0xFC00
#define ST77XX_RAINBOW_ORANGE 0xFD20 
#define ST77XX_BLUE    0x001F // Blue
#define ST77XX_INDIGO  0x4810 // Indigo 
#define ST77XX_VIOLET  0x8010 // Violet 
// ----------------------------
// RAINBOW TEXT COLOR ARRAY
// ----------------------------
const uint16_t RAINBOW_COLORS[] = {
    ST77XX_RED,          // 1. Red
    ST77XX_RAINBOW_ORANGE,// 2. Orange (Using the new bright orange)
    ST77XX_YELLOW,       // 3. Yellow
    ST77XX_GREEN,        // 4. Green
    ST77XX_BLUE,         // 5. Blue (New True Blue)
    ST77XX_INDIGO,       // 6. Indigo (New Indigo)
    ST77XX_VIOLET        // 7. Violet (New Violet)
};
const int RAINBOW_COUNT = 7;
// Maximum number of characters to store color data for per line (e.g., S> Pi = 3.14...)
const int MAX_RAINBOW_CHARS = 30; 
// Parallel array to store per-character colors for rainbow lines.
uint16_t rainbowColors[SCROLLBACK_SIZE][MAX_RAINBOW_CHARS]; // SCROLLBACK_SIZE is now defined
// Helper to cycle through the global RAINBOW_COLORS array.
uint16_t getRainbowColor(int index) {
    // Uses your global RAINBOW_COLORS array and RAINBOW_COUNT constant 
    return RAINBOW_COLORS[index % RAINBOW_COUNT];
}
// Stores the rainbow data into the parallel array
void storeRainbowData(int scrollbackIndex, const String &s) {
    int charCount = s.length();
    for (int i = 0; i < charCount && i < MAX_RAINBOW_CHARS; ++i) {
        rainbowColors[scrollbackIndex][i] = getRainbowColor(i); 
    }
}
// ----------------------------
// 3D CUBE DEFINITIONS
// ----------------------------
struct Point3D {
    float x, y, z;
};

// FIX: Define the 2D Point struct with a constructor
struct Point {
    int x, y;
    // Default constructor
    Point() : x(0), y(0) {}
    // Constructor with initial values
    Point(int _x, int _y) : x(_x), y(_y) {}
};

// ----------------------------
// Rendering snapshots
// ----------------------------
String prevVisibleLines[MAX_LINES];
int prevVisibleCount = 0;
// ----------------------------
// Function prototypes
// ----------------------------
void pushScrollback(const String &s, uint16_t color = ST77XX_WHITE); // FIXED prototype
void invalidateTerminalCache();
void pushSystemMessage(const String &s);
void drawFullTerminal();
void drawInputArea();
void ensureCursorVisible();
void insertCharAtCursor(char c);
void insertStringAtCursor(const String& s);
void backspaceAtCursor();
void clearCmdBuffer();
void clearCurrentCommand();
void loadHistoryCommand(int index);
void historyRecallDown();
void historyRecallUp();
void addHistory(const String &line);
void executeCommandLine(const String &raw);
String trimStr(const String &s);
String evalCalc(const String &expr);
void tokenizeLine(const String &line, String tokens[], int &count, int maxTokens);
bool fsBegin();
String listFiles();
String readFile(const String &path);
bool writeFile(const String &path, const String &data, bool append); 
bool removeFile(const String &path);
void kbPrev(); 
void kbNext();
void kbConfirm();
void handleFKeyAction(int fKeyNumber);
void handleFKeyInput(char inputChar);
void drawCursorAndPreview(); 
void calculateFullWrapSegments(const String &input, String outLines[], int &count, int maxOut, bool startsAsContinuation);
const char* kbGetModeName();
void drawMultiColorString(const String &text, int lineNum, int x_start);
void executeCat(String filename);
void drawRotatingCube(Point* projected_points, uint16_t color);
void runCubeAnimation();
void mood();
void runMoonPhase();
void drawMoon(int day, int totalDays);
void drawStars(); // Add this prototype
Point3D rotateX(Point3D p, float angle);
Point3D rotateY(Point3D p, float angle);
Point3D rotateZ(Point3D p, float angle);
Point project(Point3D p);

uint16_t hsvToRgb565(int hue, uint8_t sat, uint8_t val);

void wdt_disable_platform() {
    // This is the correct function call for most RP2040 cores 
    // to stop the watchdog timer gracefully.
    watchdog_disable(); 
}
void wdt_enable_platform() {
    // If your WDT implementation requires setup, place it here. 
    // For now, an empty function is safe to prevent immediate re-triggering issues.
}

void invalidateTerminalCache() {
    prevVisibleCount = 0;
}

void drawStars() {
    const int numStars = 150;
    for (int i = 0; i < numStars; i++) {
        int x = random(SCREEN_WIDTH);
        int y = random(SCREEN_HEIGHT);
        // Randomly choose between white and yellow
        uint16_t color = (random(2) == 0) ? ST77XX_WHITE : ST77XX_YELLOW;
        tft.drawPixel(x, y, color);
    }
}

uint16_t hsvToRgb565(int hue, uint8_t sat, uint8_t val) {
    uint8_t r, g, b;
    hue = hue % 360; 
    
    if (val == 0) {
        r = g = b = 0;
    } else {
        uint8_t region = hue / 60;
        uint8_t remainder = (hue % 60) * 255 / 60;

        uint8_t p = (val * (255 - sat)) >> 8;
        uint8_t q = (val * (255 - ((sat * remainder) >> 8))) >> 8;
        uint8_t t = (val * (255 - ((sat * (255 - remainder)) >> 8))) >> 8;

        switch (region) {
            case 0: r = val; g = t; b = p; break;
            case 1: r = q; g = val; b = p; break;
            case 2: r = p; g = val; b = t; break;
            case 3: r = p; g = q; b = val; break;
            case 4: r = t; g = p; b = val; break;
            default: r = val; g = p; b = q; break;
        }
    }
    // Convert 8-bit R,G,B to 16-bit RGB565
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

// *** 3D CUBE IMPLEMENTATION ***

Point3D rotateX(Point3D p, float angle) {
    float rad = angle * PI / 180.0;
    float cosA = cos(rad);
    float sinA = sin(rad);
    return {p.x, p.y * cosA - p.z * sinA, p.y * sinA + p.z * cosA};
}

Point3D rotateY(Point3D p, float angle) {
    float rad = angle * PI / 180.0;
    float cosA = cos(rad);
    float sinA = sin(rad);
    return {p.x * cosA + p.z * sinA, p.y, -p.x * sinA + p.z * cosA};
}

Point3D rotateZ(Point3D p, float angle) {
    float rad = angle * PI / 180.0;
    float cosA = cos(rad);
    float sinA = sin(rad);
    return {p.x * cosA - p.y * sinA, p.x * sinA + p.y * cosA, p.z};
}

// --- 3D to 2D Projection ---
Point project(Point3D p) {
    // Simple orthographic projection
    return Point((int)(p.x + SCREEN_WIDTH / 2), (int)(p.y + SCREEN_HEIGHT / 2));
}

// --- Drawing the Cube ---
void drawRotatingCube(Point* projected_points, uint16_t color) {
    // Edges of the cube
    int edges[12][2] = {
        {0, 1}, {1, 2}, {2, 3}, {3, 0}, // Bottom face
        {4, 5}, {5, 6}, {6, 7}, {7, 4}, // Top face
        {0, 4}, {1, 5}, {2, 6}, {3, 7}  // Connecting edges
    };

    for (int i = 0; i < 12; ++i) {
        Point p1 = projected_points[edges[i][0]];
        Point p2 = projected_points[edges[i][1]];
        tft.drawLine(p1.x, p1.y, p2.x, p2.y, color);
    }
}
void runCubeAnimation() {
    tft.fillScreen(ST77XX_BLACK);
    
    // Define the 8 vertices of the cube
    Point3D vertices[8];
    float size = 50.0; // Use float to avoid conversion warnings
    vertices[0] = {-size, -size, -size};
    vertices[1] = { size, -size, -size};
    vertices[2] = { size,  size, -size};
    vertices[3] = {-size,  size, -size};
    vertices[4] = {-size, -size,  size};
    vertices[5] = { size, -size,  size};
    vertices[6] = { size,  size,  size};
    vertices[7] = {-size,  size,  size};

    float angleX = 0, angleY = 0, angleZ = 0;

    pushSystemMessage("Starting 3D animation...");
    delay(1000); 

    tft.fillScreen(ST77XX_BLACK);

    while (true) {
        Point projected_points[8];

        // --- ERASE OLD CUBE ---
        for(int i=0; i<8; ++i) {
            Point3D p = vertices[i];
            p = rotateX(p, angleX);
            p = rotateY(p, angleY);
            p = rotateZ(p, angleZ);
            projected_points[i] = project(p);
        }
        drawRotatingCube(projected_points, ST77XX_BLACK);

        // --- UPDATE ANGLES ---
        angleX += 1.0;
        angleY += 1.5;
        angleZ += 2.0;

        // --- DRAW NEW CUBE ---
        for(int i=0; i<8; ++i) {
            Point3D p = vertices[i];
            p = rotateX(p, angleX);
            p = rotateY(p, angleY);
            p = rotateZ(p, angleZ);
            projected_points[i] = project(p);
        }
        drawRotatingCube(projected_points, ST77XX_GREEN);
        
        // --- CHECK FOR EXIT ---
        if (digitalRead(buttonPins[IDX_BACK]) == HIGH) {
            pushSystemMessage("Exiting 3D animation...");
            break;
        }

        delay(15); // Animation speed
    }
    
    // --- RESTORE TERMINAL ---
    
    tft.fillScreen(ST77XX_BLACK);
    invalidateTerminalCache();
    drawFullTerminal();
}
void mood() {
    tft.fillScreen(ST77XX_BLACK);
    pushSystemMessage("Starting mood light!");
    drawFullTerminal();
    delay(1500); 
    
    float currentHue = 0.0; // Use a float for the hue
    int lastHueInt = -1;

    // This loop runs indefinitely until the BACK button is pressed.
    while (true) {
        // Check for the exit condition first.
        if (digitalRead(buttonPins[IDX_BACK]) == HIGH) {
            pushSystemMessage("Exiting mood light...");
            break; // Exit the infinite loop.
        }

        // Increment the hue by a small fractional amount for a smooth transition.
        currentHue += 0.5; 
        if (currentHue >= 360.0) {
            currentHue -= 360.0; // Wrap the hue value back to 0.
        }

        // Convert the float hue to an integer for comparison and color calculation.
        int currentHueInt = (int)currentHue;

        // Only redraw the screen if the integer part of the hue has changed.
        // This is an optimization to avoid unnecessary screen fills.
        if (currentHueInt != lastHueInt) {
            uint16_t color = hsvToRgb565(currentHueInt, 255, 255);
            tft.fillScreen(color);
            lastHueInt = currentHueInt;
        }
        
        delay(5); // A small delay to control the speed of the cycle.
    }

    // Restore the terminal interface.
    tft.fillScreen(ST77XX_BLACK);
    invalidateTerminalCache(); // Clear the visual cache
    drawFullTerminal();        // Force a full redraw of the terminal
}
/**
 * @brief Draws the moon phase for a given day using an overlapping circle approximation.
 * @param day The current day of the lunar cycle (0-29).
 * @param totalDays The total number of days in the cycle (e.g., 30).
 */
/**
 * @brief Draws the moon phase for a given day using an overlapping circle approximation.
 * @param day The current day of the lunar cycle (0-29).
 * @param totalDays The total number of days in the cycle (e.g., 30).
 */
void drawMoon(int day, int totalDays) {
    int cx = SCREEN_WIDTH / 2;
    int cy = SCREEN_HEIGHT / 2 - 20; // Move moon up to make space for text
    int r = 60;

    // Clear the drawing area to erase the previous frame.
    tft.fillRect(cx - r - 1, cy - r - 1, 2 * r + 2, 2 * r + 2, ST77XX_BLACK);

    // Calculate phase and shadow position.
    float phase = day / (float)(totalDays - 1);
    int terminator_x = cx - (2 * r) + (int)(phase * 4.0 * r);

    // --- Build the moon scanline by scanline with corrected logic ---
    for (int y = cy - r; y <= cy + r; y++) {
        int dy = y - cy;

        // Calculate the boundaries of the full moon shape for this line.
        int half_width_moon = round(sqrt(r * r - dy * dy));
        int moon_x1 = cx - half_width_moon;
        int moon_x2 = cx + half_width_moon;

        // Calculate the boundaries of the shadow shape for this line.
        int half_width_shadow = 0;
        if (abs(dy) < r) {
             half_width_shadow = round(sqrt(r * r - dy * dy));
        }
        int shadow_x1 = terminator_x - half_width_shadow;
        int shadow_x2 = terminator_x + half_width_shadow;
        
        // Find the actual start and end of the shadow's intersection with the moon.
        int intersection_start = max(moon_x1, shadow_x1);
        int intersection_end   = min(moon_x2, shadow_x2);

        // --- NEW, ROBUST LOGIC ---
        // First, check if there is any real overlap on this line.
        if (intersection_start >= intersection_end) {
            // NO OVERLAP: The shadow is not on the moon here. Draw the full moon slice.
            tft.drawFastHLine(moon_x1, y, moon_x2 - moon_x1, ST77XX_WHITE);
        } else {
            // OVERLAP EXISTS: "Punch out" the shadow by drawing the lit parts around it.
            // 1. Draw the lit part to the LEFT of the shadow intersection.
            if (moon_x1 < intersection_start) {
                tft.drawFastHLine(moon_x1, y, intersection_start - moon_x1, ST77XX_WHITE);
            }
            // 2. Draw the lit part to the RIGHT of the shadow intersection.
            if (intersection_end < moon_x2) {
                tft.drawFastHLine(intersection_end, y, moon_x2 - intersection_end, ST77XX_WHITE);
            }
        }
    }

    // --- Draw the day number underneath ---
    tft.setTextSize(2);
    tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    String dayText = "Day " + String(day + 1);
    
    int16_t x1, y1;
    uint16_t w, h;
    tft.getTextBounds(dayText, 0, 0, &x1, &y1, &w, &h);
    int text_x = (SCREEN_WIDTH - w) / 2;
    int text_y = cy + r + 10;

    tft.fillRect(0, text_y, SCREEN_WIDTH, h + 5, ST77XX_BLACK);
    tft.setCursor(text_x, text_y);
    tft.print(dayText);

    tft.setTextSize(1);
}
/**
 * @brief Enters a dedicated loop to display and control the moon phase viewer.
 * Exits when the BACK button is pressed.
 */
void runMoonPhase() {
    int currentDay = 0; // Start at Day 0 (New Moon)
    const int totalDays = 30;

    tft.fillScreen(ST77XX_BLACK);
    drawStars(); // Draw the starfield background once
    drawMoon(currentDay, totalDays);

    while (true) {
        unsigned long now = millis();
        bool dayChanged = false;

        // Check for LEFT button (IDX_PREV)
        if (digitalRead(buttonPins[IDX_PREV]) == HIGH && (now - lastPressTime[IDX_PREV] > pressCooldown)) {
            lastPressTime[IDX_PREV] = now;
            currentDay--;
            if (currentDay < 0) {
                currentDay = totalDays - 1; // Wrap around
            }
            dayChanged = true;
        }

        // Check for RIGHT button (IDX_NEXT)
        if (digitalRead(buttonPins[IDX_NEXT]) == HIGH && (now - lastPressTime[IDX_NEXT] > pressCooldown)) {
            lastPressTime[IDX_NEXT] = now;
            currentDay++;
            if (currentDay >= totalDays) {
                currentDay = 0; // Wrap around
            }
            dayChanged = true;
        }

        // Check for BACK button to exit
        if (digitalRead(buttonPins[IDX_BACK]) == HIGH && (now - lastPressTime[IDX_BACK] > pressCooldown)) {
            lastPressTime[IDX_BACK] = now;
            break; // Exit the loop
        }

        if (dayChanged) {
            drawMoon(currentDay, totalDays);
        }

        yield(); // Allow other processes to run
    }

    // Restore terminal interface upon exit
    tft.fillScreen(ST77XX_BLACK);
    invalidateTerminalCache();
    drawFullTerminal();
}
// ----------------------------
// Calculates the wrapped segments of the ENTIRE command buffer content.
// ----------------------------
void calculateFullWrapSegments(const String &input, String outLines[], int &count, int maxOut, bool startsAsContinuation) {
    count = 0;
    int pos = 0;
    int inputLength = input.length();
    
    if (inputLength == 0) {
        if (count < maxOut) outLines[count++] = "";
        return;
    }

    // --- 1. Handle the FIRST line ---
    int firstLineCapacity = startsAsContinuation ? WRAP_COLS : (WRAP_COLS - promptCols());
    if (count < maxOut) {
        int len = min(inputLength, firstLineCapacity);
        outLines[count++] = input.substring(pos, pos + len);
        pos += len;
    } else {
        return;
    }
    
    // --- 2. Handle subsequent lines (always Full WRAP_COLS capacity) ---
    while (pos < inputLength && count < maxOut) {
        int rem = inputLength - pos;
        int len = min(rem, WRAP_COLS); 
        outLines[count++] = input.substring(pos, pos + len);
        pos += len;
    }
}
// ----------------------------
// Scrollback / push helpers
// ----------------------------
void pushScrollback(const String &text, uint16_t color) { 
    int current = 0;
    int next = -1;
    const int SCROLLBACK_LINE_WIDTH = COLS; // 40 characters wide
    
    // Loop to find all explicit newline characters ('\n')
    do {
        next = text.indexOf('\n', current);
        String line;
        
        if (next == -1) {
            line = text.substring(current);
        } else {
            line = text.substring(current, next);
        }
        
        current = next + 1; // Advance for the next iteration
        
        // --- Line Segment Insertion Logic (Handles character-wrapping) ---
        if (line.length() > 0) {
            int linePos = 0;
            while (linePos < line.length()) {
                // Calculate the segment length (max of 40 characters or the remaining length)
                int segmentLength = min((int)line.length() - linePos, SCROLLBACK_LINE_WIDTH);
                
                String segment = line.substring(linePos, linePos + segmentLength);
                linePos += segmentLength;

                // Insert the segment into the scrollback buffer
                int idx;
                if (scrollbackCount < SCROLLBACK_SIZE) {
                    idx = (scrollbackHead + scrollbackCount) % SCROLLBACK_SIZE;
                    scrollbackCount++;
                } else {
                    idx = scrollbackHead;
                    scrollbackHead = (scrollbackHead + 1) % SCROLLBACK_SIZE;
                }

                // --- NEW LED BLINK TRIGGER ---
                ledBlinkEndTime = millis() + LED_BLINK_DURATION_MS;
                digitalWrite(STATUS_LED_PIN, HIGH); // Turn LED ON
                // -----------------------------    

                scrollback[idx].text = segment;
                scrollback[idx].color = color;
                terminalScrollOffset = 0;
            }
        } else if (line.length() == 0 && next != -1) {
            // Handle explicit empty line (two consecutive \n)
            int idx;
            if (scrollbackCount < SCROLLBACK_SIZE) {
                idx = (scrollbackHead + scrollbackCount) % SCROLLBACK_SIZE;
                scrollbackCount++;
            } else {
                idx = scrollbackHead;
                scrollbackHead = (scrollbackHead + 1) % SCROLLBACK_SIZE;
            }
            scrollback[idx].text = "";
            scrollback[idx].color = color;
            terminalScrollOffset = 0;
        }

        if (next == -1) break; 
        
    } while (true);
    
    // As before, redraw is handled by the caller (e.g., executeCommandLine)
}
// ----------------------------
// System messages: GREEN SYS LABEL
// ----------------------------
void pushSystemMessage(const String &s) {
    pushScrollback(SYS_PROMPT + s, ST77XX_GREEN);  // <-- CRITICAL: Set color to GREEN
}
// ----------------------------
// Draw only the scrollback area (top region) with bottom-up newest placement.
// ----------------------------
void drawScrollbackArea(int availableOutputRows) {
    if (availableOutputRows <= 0) {
        // Clear all previous lines
        for (int r = 0; r < prevVisibleCount; ++r) {
            if (prevVisibleLines[r] != "") {
                tft.fillRect(0, r * LINE_HEIGHT, SCREEN_WIDTH, LINE_HEIGHT, ST77XX_BLACK);
                prevVisibleLines[r] = "";
            }
        }
        prevVisibleCount = 0;
        return;
    }

    int newestGlobal = scrollbackCount - 1 - terminalScrollOffset;
    if (newestGlobal < 0) newestGlobal = 0;
    
    for (int slot = 0; slot < availableOutputRows; ++slot) {
        int visualRow = (availableOutputRows - 1) - slot;
        int globalIndex = newestGlobal - slot;
        
        String toDraw = "";
        uint16_t color = ST77XX_WHITE; // Default fallback

        if (globalIndex >= 0 && scrollbackCount > 0) {
            int idx = (scrollbackHead + globalIndex) % SCROLLBACK_SIZE;
            
            toDraw = scrollback[idx].text;
            color = scrollback[idx].color; 
        }

        // Optimization Check: Only redraw the row if the content has changed
        if (visualRow >= prevVisibleCount || prevVisibleLines[visualRow] != toDraw) {
            int y = visualRow * LINE_HEIGHT;
            
            // 1. Clear the line before drawing
            tft.fillRect(0, y, SCREEN_WIDTH, LINE_HEIGHT, ST77XX_BLACK);
            
            // 2. CHECK FOR RAINBOW SIGNAL (ST77XX_BLACK) - Unchanged
            if (color == ST77XX_BLACK && toDraw.length() > 0) {
                // ... (Rainbow logic, untouched)
                int idx = (scrollbackHead + globalIndex) % SCROLLBACK_SIZE;
                int x = 0;
                
                for (int i = 0; i < toDraw.length() && i < MAX_RAINBOW_CHARS; ++i) {
                    uint16_t charColor = rainbowColors[idx][i];
                    
                    // Override the SYS_PROMPT part to be solid GREEN 
                    if (i < SYS_PROMPT.length()) {
                        charColor = ST77XX_GREEN;
                    }
                    
                    tft.setCursor(x, y);
                    tft.setTextColor(charColor, ST77XX_BLACK);
                    tft.print(toDraw.charAt(i));
                    
                    x += CHAR_WIDTH;
                }
                prevVisibleLines[visualRow] = toDraw; // Update cache
                
            } else if (toDraw.length() > 0) {
                // 3. Draw standard single-color text using the color from the struct
                tft.setCursor(0, y);
                
                // --- Split-Color Logic for PROMPT and System Messages ---
                if (toDraw.startsWith(PROMPT)) {
                    tft.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
                    tft.print(PROMPT);
                    tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
                    tft.print(toDraw.substring(PROMPT.length()));
                } 
                // SCENARIO A: First line of a system message (starts with "S>")
                else if (toDraw.startsWith(SYS_PROMPT) && color == ST77XX_GREEN) { 
                    tft.setTextColor(ST77XX_GREEN, ST77XX_BLACK); // S> in Green
                    tft.print(SYS_PROMPT);
                    tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK); // Rest in White
                    tft.print(toDraw.substring(SYS_PROMPT.length()));
                }
                // SCENARIO B: Subsequent wrapped lines of a system message (NO "S>")
                else if (color == ST77XX_GREEN) {
                    // FIX: Draw the entire continuation line in White.
                    tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK); 
                    tft.print(toDraw);
                }
                // END FIX
                else {
                    // Default fallback: Use the color stored in the struct for all other lines
                    tft.setTextColor(color, ST77XX_BLACK);
                    tft.print(toDraw);
                }
                // --- End Single-Color Logic ---

                prevVisibleLines[visualRow] = toDraw; // Update cache
            } else {
                // Clear cache for empty lines
                prevVisibleLines[visualRow] = "";
            }
        }
    }

    // Clear any old scrollback lines that are no longer visible
    for (int r = availableOutputRows; r < prevVisibleCount; ++r) {
        tft.fillRect(0, r * LINE_HEIGHT, SCREEN_WIDTH, LINE_HEIGHT, ST77XX_BLACK);
        prevVisibleLines[r] = "";
    }
    prevVisibleCount = availableOutputRows;
}
// ----------------------------
// DRAWFULLTERMINAL- Draws everything once (scrollback + input lines).
// ----------------------------
void drawFullTerminal() {
    // 1. Calculate all visual line segments from the entire command buffer.
    String input = String(cmdBuf).substring(0, cmdLen); 
    const int MAX_CHUNKS = 16;
    String fwdSegments[MAX_CHUNKS];
    int fwdCount = 0;
    calculateFullWrapSegments(input, fwdSegments, fwdCount, MAX_CHUNKS, false);
    
    if (fwdCount == 0) fwdCount = 1;

    // Determine how many lines the input will occupy.
    const int INPUT_LINES_TO_DRAW = min(fwdCount, MAX_LINES);
    // 2. Determine screen layout, shrinking the history area to make room for the input.
    int availableOutputRows = MAX_LINES - INPUT_LINES_TO_DRAW; 
    
    // Redraw the history/scrollback area in its new, possibly smaller, space.
    drawScrollbackArea(availableOutputRows);

    // *** MODIFICATION ***
    // Redraw the input area using the new function
    drawInputArea();

    drawCursorAndPreview();

}
// ----------------------------
// drawInputArea() - Draws ONLY the command input area (no scrollback, no cursor)
// ----------------------------
void drawInputArea() {
    // 1. Calculate all visual line segments from the entire command buffer.
    String input = String(cmdBuf).substring(0, cmdLen); 
    const int MAX_CHUNKS = 16;
    String fwdSegments[MAX_CHUNKS];
    int fwdCount = 0;
    calculateFullWrapSegments(input, fwdSegments, fwdCount, MAX_CHUNKS, false);
    
    if (fwdCount == 0) fwdCount = 1; // Always at least one line

    // Determine how many lines the input will occupy.
    const int INPUT_LINES_TO_DRAW = min(fwdCount, MAX_LINES);
    // 2. Determine screen layout
    int availableOutputRows = MAX_LINES - INPUT_LINES_TO_DRAW; 
    
    // 3. Clear the entire input area before redrawing.
    int yStart = availableOutputRows * LINE_HEIGHT;
    if (yStart < SCREEN_HEIGHT) {
        tft.fillRect(0, yStart, SCREEN_WIDTH, SCREEN_HEIGHT - yStart, ST77XX_BLACK);
    }

    // 4. Loop to draw all *visible* segments of the command.
    for (int i = 0; i < INPUT_LINES_TO_DRAW; ++i) {
        int segmentIndex = fwdCount - INPUT_LINES_TO_DRAW + i; 
        if (segmentIndex >= 0 && segmentIndex < fwdCount) {
            int y = (availableOutputRows + i) * LINE_HEIGHT;
            int xOffset = 0;
            
            if (segmentIndex == 0) {
                tft.fillRect(0, y, promptCols() * CHAR_WIDTH, LINE_HEIGHT, ST77XX_BLACK);
                tft.setCursor(0, y);
                tft.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
                tft.print(PROMPT);
                xOffset = promptCols() * CHAR_WIDTH;
            }
            
            tft.setCursor(xOffset, y); 
            tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
            tft.print(fwdSegments[segmentIndex]);

            int textWidth = xOffset + fwdSegments[segmentIndex].length() * CHAR_WIDTH;
            tft.fillRect(textWidth, y, SCREEN_WIDTH - textWidth, LINE_HEIGHT, ST77XX_BLACK);
        }
    }
}
// ----------------------------
// drawCursorAndPreview()  Draws just the cursor and keyboard preview (Interaction point)
// ----------------------------
void drawCursorAndPreview() {
    
    // --- FORMAT CONFIRMATION DRAWING LOGIC ---
    if (fkeyState == F_AWAIT_FORMAT_CONFIRM) {
        const int PROMPT_LEN = 7;
        const int CURSOR_COL = PROMPT_LEN;
        int y_pos = (MAX_LINES - 1) * LINE_HEIGHT;
        int drawX = CURSOR_COL * CHAR_WIDTH;
        int drawY = y_pos;
        int padX = drawX - 1;
        int padY = drawY - 1;
        int padW = CHAR_WIDTH + 1;
        int padH = LINE_HEIGHT + 1;
        
        char displayChar = ' ';
        uint16_t fg_color = ST77XX_WHITE;
        uint16_t bg_color = ST77XX_BLACK;
        
        if (formatIndex == 0) { // User selected 'Y'
            displayChar = 'Y';
            if (cursorVisible) {
                fg_color = ST77XX_BLACK;
                bg_color = ST77XX_GREEN;
            } else {
                fg_color = ST77XX_GREEN;
                bg_color = ST77XX_BLACK;
            }
        } else if (formatIndex == 1) { // User selected 'N'
            displayChar = 'N';
            if (cursorVisible) {
                fg_color = ST77XX_BLACK;
                bg_color = ST77XX_RED;
            } else {
                fg_color = ST77XX_RED;
                bg_color = ST77XX_BLACK;
            }
        }

        tft.fillRect(padX, padY, padW, padH, bg_color);
        tft.setCursor(drawX, drawY);
        tft.setTextColor(fg_color, bg_color);
        tft.print(displayChar);
        tft.fillRect(padX + padW, padY, SCREEN_WIDTH - (padX + padW), padH, ST77XX_BLACK);
        tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
        return;
    }

    String fullInputLine = (inputWrapped ? "" : PROMPT) + String(cmdBuf).substring(0, cmdLen);
    // --- Select preview string ---
    String preview;
    const int ALPHA_CASE_KEY_INDEX = (int)strlen(alphaChars) + 1;
    const int NUM_ALPHA_KEY_INDEX = (int)strlen(numberChars) + 1;
    const int SYM_ALPHA_KEY_INDEX = (int)strlen(symbolChars) + 1;
    if (kbIndex == 0) { preview = kbGetModeName(); }
    else if (kmode == ALPHA) {
        if (kbIndex <= (int)strlen(alphaChars)) { preview = String(alphaChars[kbIndex - 1]);
        }
        else if (kbIndex == ALPHA_CASE_KEY_INDEX) { preview = "[SPACE]";
        }
        else if (kbIndex == ALPHA_CASE_KEY_INDEX + 1) { preview = "[ENTER]";
        }
        else if (kbIndex == ALPHA_CASE_KEY_INDEX + 2) { preview = "[CASE]";
        }
    }
    else if (kmode == ALPHA_LOWER) {
        if (kbIndex <= (int)strlen(alphaLowerChars)) { preview = String(alphaLowerChars[kbIndex - 1]);
        }
        else if (kbIndex == ALPHA_CASE_KEY_INDEX) { preview = "[SPACE]";
        }
        else if (kbIndex == ALPHA_CASE_KEY_INDEX + 1) { preview = "[ENTER]";
        }
        else if (kbIndex == ALPHA_CASE_KEY_INDEX + 2) { preview = "[case]";
        }
    }
    else if (kmode == NUM) {
        if (kbIndex <= (int)strlen(numberChars)) { preview = String(numberChars[kbIndex - 1]);
        }
        else if (kbIndex == NUM_ALPHA_KEY_INDEX) { preview = "[SPACE]";
        }
        else if (kbIndex == NUM_ALPHA_KEY_INDEX + 1) { preview = "[ENTER]";
        }
        else if (kbIndex == NUM_ALPHA_KEY_INDEX + 2) { preview = "[ALPHA]";
        }
    }
    else if (kmode == SYM) {
        if (kbIndex <= (int)strlen(symbolChars)) { preview = String(symbolChars[kbIndex - 1]);
        }
        else if (kbIndex == SYM_ALPHA_KEY_INDEX) { preview = "[SPACE]";
        }
        else if (kbIndex == SYM_ALPHA_KEY_INDEX + 1) { preview = "[ENTER]";
        }
        else if (kbIndex == SYM_ALPHA_KEY_INDEX + 2) { preview = "[ALPHA]";
        }
    }
    else if (kmode == CTRL) {
        if (kbIndex <= CTRL_COUNT) { preview = "[" + ctrlKeys[kbIndex - 1] + "]";
        }
        else if (kbIndex == CTRL_COUNT + 1) { preview = "[FUNC]";
        }
    }
    else if (kmode == FUNC_VIEW) {
        if (kbIndex <= FUNC_COUNT) { preview = "[" + funcKeys[kbIndex - 1] + "]";
        }
    }

    // --- Multi-Line Cursor Calculation Logic ---
    const int PROMPT_LEN_INTERNAL = PROMPT.length();
    const int LINE_1_CAPACITY = COLS - PROMPT_LEN_INTERNAL; 
    const int LINE_N_CAPACITY = WRAP_COLS; 

    int cursorCol = 0;
    int cursorRow = 0;
    int tempPos = cursorPos;
    if (tempPos < LINE_1_CAPACITY) {
        cursorCol = PROMPT_LEN_INTERNAL + tempPos;
        cursorRow = 0;
    } else {
        tempPos -= LINE_1_CAPACITY;
        cursorRow = 1 + (tempPos / LINE_N_CAPACITY);
        cursorCol = tempPos % LINE_N_CAPACITY;
    }

    String segments[16];
    int segmentCount = 0;
    calculateFullWrapSegments(String(cmdBuf).substring(0, cmdLen), segments, segmentCount, 16, false);
    
    if (segmentCount == 0) segmentCount = 1;

    const int INPUT_LINES_TO_DRAW = min(segmentCount, MAX_LINES);
    int availableOutputRows = MAX_LINES - INPUT_LINES_TO_DRAW;
    
    int cursorRowInInputArea = cursorRow - (segmentCount - INPUT_LINES_TO_DRAW);
    if (cursorRowInInputArea < 0) cursorRowInInputArea = 0;
    int cursorRowY = (availableOutputRows + cursorRowInInputArea) * LINE_HEIGHT;
    int globalCursorPos = PROMPT_LEN_INTERNAL + cursorPos;
    int previewCols = max(1, (int)preview.length());
    // --- Mode Text Color Logic ---
    uint16_t modeTextColor = ST77XX_WHITE;
    if (fkeyState != F_INACTIVE) { modeTextColor = ST77XX_RED; }
    else if (kmode == ALPHA || kmode == ALPHA_LOWER) { modeTextColor = ST77XX_CYAN;
    }
    else if (kmode == NUM) { modeTextColor = ST77XX_GREEN;
    }
    else if (kmode == SYM) { modeTextColor = ST77XX_MAGENTA;
    }
    else if (kmode == CTRL) { modeTextColor = ST77XX_DARK_ORANGE;
    }
    else if (kmode == FUNC_VIEW) { modeTextColor = ST77XX_RED;
    }
    
    // --- Clear Previous Cursor Highlight and Redraw Hidden Text ---
    if (lastGlobalCursorPos >= 0) {
        int prevGlobalCursorPos = lastGlobalCursorPos;
        int prevRowY = lastCursorRowY;
        int prevCol = lastCursorCol;
        
        for (int i = 0; i < lastPreviewCols; ++i) {
            if (prevCol >= COLS) {
                 prevCol = 0;
                 prevRowY += LINE_HEIGHT;
            }
            
            int drawX = prevCol * CHAR_WIDTH;
            int drawY = prevRowY;
            
            // --- TARGETED FIX FOR THE CHARACTER *AFTER* THE NEW CURSOR PREVIEW ---
            // The character being clipped is at the index equal to the old preview length (i == lastPreviewCols).
            // Since the loop runs from i=0 to lastPreviewCols-1, we need to check i == lastPreviewCols-1
            // AND ensure the character *after* this loop ends is also addressed.
            
            if (i == lastPreviewCols - 1) {
                // If the cursor is still on the same line, the character immediately following the new, 
                // shorter preview (which is the start of the old preview's clip area) is the target.
                
                // 1. First, clear the rest of the old cursor highlight using the standard wide clear.
                tft.fillRect(drawX - 1, drawY - 1, CHAR_WIDTH + 2, LINE_HEIGHT + 1, ST77XX_BLACK);
                
                // 2. NOW, check the position *after* this one for the clip artifact.
                // This requires logic *outside* the loop, but since we are modifying the loop...
                
                // For now, let's revert to the single-character fix at i=1, as that is the standard pattern, 
                // and assume the issue is related to the drawing of the character *after* the preview.
                
                // REVERTING TO THE LAST KNOWN BEST (BUT STILL FAILING) CODE, 
                // AND ADDING A FINAL CLEAR AFTER THE LOOP
            }
            
            // Reverting to the logic that uses your standard clear and redraw:
            tft.fillRect(drawX - 1, drawY - 1, CHAR_WIDTH + 2, LINE_HEIGHT + 1, ST77XX_BLACK);
            
            if (prevGlobalCursorPos + i < fullInputLine.length()) {
                char c = fullInputLine.charAt(prevGlobalCursorPos + i);
                
                tft.setCursor(drawX, drawY); 
                
                bool isPromptChar = !inputWrapped && (prevGlobalCursorPos + i) < PROMPT.length();
                tft.setTextColor(isPromptChar ? ST77XX_CYAN : ST77XX_WHITE);
                
                tft.print(c);
            }
            prevCol++;
        }
        
        // --- FINAL ATTEMPT AT TARGETED FIX FOR CLIPPING OUTSIDE THE LOOP ---
        // We calculate the column and row for the character *immediately following* the old preview area.
        int clipCol = prevCol;
        int clipRowY = prevRowY;
        
        // Adjust draw position for the character *after* the preview ends (if it wrapped).
        if (clipCol >= COLS) {
            clipCol = 0;
            clipRowY += LINE_HEIGHT;
        }

        int clipDrawX = clipCol * CHAR_WIDTH;
        int clipDrawY = clipRowY;
        
        // We target the *next* character's position for a surgical, unpadded clear and redraw.
        if (prevGlobalCursorPos + lastPreviewCols < fullInputLine.length()) {
            char c = fullInputLine.charAt(prevGlobalCursorPos + lastPreviewCols);
            
            // Clear the clipped area (unpadded clear).
            tft.fillRect(clipDrawX, clipDrawY, CHAR_WIDTH, LINE_HEIGHT, ST77XX_BLACK);
            
            // Redraw the character.
            tft.setCursor(clipDrawX, clipDrawY);
            bool isPromptChar = !inputWrapped && (prevGlobalCursorPos + lastPreviewCols) < PROMPT.length();
            tft.setTextColor(isPromptChar ? ST77XX_CYAN : ST77XX_WHITE);
            tft.print(c);
        }
    }
    
    int drawCol = cursorCol;
    int drawRowY = cursorRowY;

    for (int i = 0; i < preview.length(); ++i) {
        if (drawCol >= COLS) {
            drawCol = 0;
            drawRowY += LINE_HEIGHT;
        }

        int drawX = drawCol * CHAR_WIDTH;
        int padX = drawX - 1;
        int padY = drawRowY - 1;
        int padW = CHAR_WIDTH + 1; // Standard cursor width remains CHAR_WIDTH + 1
        int padH = LINE_HEIGHT + 1;

        if (padX < 0) {
            padX = 0;
        }


        if (cursorVisible) {
            // Apply modeTextColor only if it is the mode label (kbIndex == 0) OR if it is F-INPUT AWAIT mode (kbIndex > 0)
            uint16_t bgColor = ST77XX_WHITE;
            if (kbIndex == 0) {
                bgColor = modeTextColor; // Use the calculated mode color for the mode label
            } else if (fkeyState != F_INACTIVE) {
                 bgColor = ST77XX_RED; // Highlight the selected character itself when awaiting F-key input
            }
            
            tft.fillRect(padX, padY, padW, padH, bgColor);
            tft.setTextColor(ST77XX_BLACK);
        } else {
            tft.fillRect(padX, padY, padW, padH, ST77XX_BLACK);
            tft.setTextColor(kbIndex == 0 ? modeTextColor : ST77XX_WHITE);
        }

        tft.setCursor(drawX, drawRowY);
        tft.print(preview.charAt(i));
        drawCol++;
    }

    lastPreviewCols = previewCols;
    lastCursorCol = cursorCol;
    lastCursorRowY = cursorRowY;
    lastGlobalCursorPos = globalCursorPos;
}
void ensureCursorVisible() {
    drawCursorAndPreview();
}
void clearCmdBuffer() {
    memset(cmdBuf, 0, CMD_BUF);
    cmdLen = 0; cursorPos = 0;
    inputWrapped = false; 
    drawFullTerminal();
}
void redrawTrailingText() {
    // --- 1. Calculate the cursor's exact screen position (x, y) ---
    String fullInput = String(cmdBuf).substring(0, cmdLen);
    String segments[16];
    int segmentCount = 0;
    calculateFullWrapSegments(fullInput, segments, segmentCount, 16, inputWrapped);

    if (segmentCount == 0) segmentCount = 1; // Failsafe for empty buffer

    int linesToDraw = min(segmentCount, MAX_LINES);
    int startRow = MAX_LINES - linesToDraw;

    int tempCursorPos = cursorPos;
    int cursorSegmentIndex = -1;
    int cursorColInSegment = 0;
    for (int i = 0; i < segmentCount; ++i) {
        // Find the segment the cursor is on
        if (tempCursorPos <= segments[i].length()) {
            cursorSegmentIndex = i;
            cursorColInSegment = tempCursorPos;
            break;
        }
        tempCursorPos -= segments[i].length();
    }

    if (cursorSegmentIndex == -1) { // Failsafe
        // This can happen if the buffer is empty
        if (segmentCount > 0) {
             cursorSegmentIndex = 0;
        } else {
            drawFullTerminal();
            return;
        }
    }
    
    int promptOffset = (cursorSegmentIndex == 0) ? promptCols() : 0;
    int cursorScreenX = (promptOffset + cursorColInSegment) * CHAR_WIDTH;
    int cursorScreenY = (startRow + (cursorSegmentIndex - (segmentCount - linesToDraw))) * LINE_HEIGHT;
    
    // --- 2. Clear the screen area from the cursor to the end ---
    // Clear the rest of the current line
    tft.fillRect(cursorScreenX, cursorScreenY, SCREEN_WIDTH - cursorScreenX, LINE_HEIGHT, ST77XX_BLACK);
    // 2b. Clear any SUBSEQUENT LINES fully
    for (int i = (cursorSegmentIndex - (segmentCount - linesToDraw)) + 1; i < linesToDraw; ++i) {
        tft.fillRect(0, (startRow + i) * LINE_HEIGHT, SCREEN_WIDTH, LINE_HEIGHT, ST77XX_BLACK);
    }
    
    // --- 3. Redraw the trailing text (NEW LOGIC) ---
    tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);

    // 3a. Print the rest of the *current* segment
    String currentSegment = segments[cursorSegmentIndex];
    String restOfCurrentSegment = "";
    if (cursorColInSegment < currentSegment.length()) {
         restOfCurrentSegment = currentSegment.substring(cursorColInSegment);
    }
    tft.setCursor(cursorScreenX, cursorScreenY);
    tft.print(restOfCurrentSegment);

    // 3b. Print all *subsequent* segments on new lines
    int currentY = cursorScreenY + LINE_HEIGHT;
    for (int i = cursorSegmentIndex + 1; i < segmentCount; ++i) {
        // Check if this segment is actually visible on screen
        int segmentScreenRow = (startRow + (i - (segmentCount - linesToDraw)));
        if (segmentScreenRow >= MAX_LINES) break; // Stop if we're off-screen

        tft.setCursor(0, currentY); // Subsequent lines always start at X=0
        tft.print(segments[i]);
        currentY += LINE_HEIGHT;
    }
    
    // --- 4. Redraw the cursor preview over the new text ---
    drawCursorAndPreview();
}
void insertCharAtCursor(char c) {
    if (cmdLen + 1 >= CMD_BUF) return;

    // --- 1. Calculate BEFORE state ---
    const int MAX_CHUNKS = 16;
    String preSegments[MAX_CHUNKS];
    int preCount = 0;
    calculateFullWrapSegments(String(cmdBuf).substring(0, cmdLen), preSegments, preCount, MAX_CHUNKS, false);
    if (preCount == 0) preCount = 1; // Ensure at least one line

    // Calculate cursor's visual row BEFORE insertion
    int preCursorRow = 0;
    const int PROMPT_LEN_INTERNAL = PROMPT.length();
    const int LINE_1_CAPACITY = COLS - PROMPT_LEN_INTERNAL;
    const int LINE_N_CAPACITY = WRAP_COLS;
    int tempPrePos = cursorPos;
    if (tempPrePos >= LINE_1_CAPACITY) {
        tempPrePos -= LINE_1_CAPACITY;
        preCursorRow = 1 + (tempPrePos / LINE_N_CAPACITY);
    }

    // --- 2. Perform the insertion in memory ---
    for (int i = cmdLen; i > cursorPos; --i) {
        cmdBuf[i] = cmdBuf[i - 1];
    }
    cmdBuf[cursorPos] = c;
    cursorPos++; // Cursor moves forward after insertion
    cmdLen++;
    cmdBuf[cmdLen] = 0;

    // --- 3. Calculate AFTER state ---
    String postSegments[MAX_CHUNKS];
    int postCount = 0;
    calculateFullWrapSegments(String(cmdBuf).substring(0, cmdLen), postSegments, postCount, MAX_CHUNKS, false);
    if (postCount == 0) postCount = 1; // Ensure at least one line

    // Calculate cursor's visual row AFTER insertion
    int postCursorRow = 0;
    int tempPostPos = cursorPos; // Use the updated cursorPos
    if (tempPostPos >= LINE_1_CAPACITY) {
        tempPostPos -= LINE_1_CAPACITY;
        postCursorRow = 1 + (tempPostPos / LINE_N_CAPACITY);
    }

    // --- 4. MODIFIED Conditional Redraw Logic ---
    if (postCount > preCount || postCursorRow > preCursorRow) {
        // Redraw FULL terminal if:
        // a) The total number of lines increased (layout change)
        // OR
        // b) The cursor wrapped down onto a new line.

        // Hide cursor logic (optional but helps transition)
        bool oldVisibility = cursorVisible;
        cursorVisible = false;
        drawCursorAndPreview(); // Update internal state before full redraw
        cursorVisible = oldVisibility;

        // Perform the full terminal redraw.
        drawFullTerminal(); // This calls drawInputArea and drawCursorAndPreview internally
    } else {
        // No layout change AND cursor stayed on the same visual line.
        // Use the flicker-free redrawTrailingText.
        redrawTrailingText(); // Calls drawCursorAndPreview internally
    }

    // --- 5. Reset F1 sequence ---
    f1_copy_index = 0;
}
void insertStringAtCursor(const String& s) {
    for(int i = 0; i < s.length(); ++i) {
        insertCharAtCursor(s.charAt(i));
    }
}
void backspaceAtCursor() {
    // Exit if there is nothing to delete.
    if (cursorPos == 0 || cmdLen == 0) {
        return;
    }

    // --- 1. Get the current number of wrapped segments (lines) ---
    const int MAX_CHUNKS = 16;
    String preSegments[MAX_CHUNKS];
    int preCount = 0;
    calculateFullWrapSegments(String(cmdBuf).substring(0, cmdLen), preSegments, preCount, MAX_CHUNKS, false);

    // --- 2. Hide cursor IMMEDIATELY before deletion begins ---
    bool oldVisibility = cursorVisible;
    cursorVisible = false;
    drawCursorAndPreview(); // This also updates lastPreviewCols, etc.

    // --- 3. Perform the deletion in memory ---
    for (int i = cursorPos - 1; i < cmdLen - 1; ++i) {
        cmdBuf[i] = cmdBuf[i + 1];
    }
    cmdLen--;
    cursorPos--;
    cmdBuf[cmdLen] = 0;

    // --- 4. Get the new number of wrapped segments (lines) ---
    String postSegments[MAX_CHUNKS];
    int postCount = 0;
    calculateFullWrapSegments(String(cmdBuf).substring(0, cmdLen), postSegments, postCount, MAX_CHUNKS, false);
    if (cmdLen > 0 && postCount == 0) postCount = 1;

    // --- Flag if an un-wrap occurred ---
    bool didUnwrap = (postCount < preCount);

    // --- 5. HYBRID Conditional Redraw Logic ---
    if (didUnwrap) {
        // Un-wrap occurred.
        drawFullTerminal(); // Redraws layout, might glitch prompt.

        // Surgical prompt fix
        const int LINE_1_CAPACITY = COLS - PROMPT.length();
        if (cursorPos < LINE_1_CAPACITY) {
            int linesToDrawAfter = min(postCount > 0 ? postCount : 1, MAX_LINES);
            int firstInputLineY = (MAX_LINES - linesToDrawAfter) * LINE_HEIGHT;

            tft.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
            tft.setCursor(0, firstInputLineY);
            tft.print(PROMPT); // Redraw full prompt

            int charsToDraw = min(cmdLen, LINE_1_CAPACITY);
            String firstLineCmd = String(cmdBuf).substring(0, charsToDraw);
            tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
            tft.setCursor(PROMPT.length() * CHAR_WIDTH, firstInputLineY);
            tft.print(firstLineCmd);

            int endX = (PROMPT.length() + firstLineCmd.length()) * CHAR_WIDTH;
            if (endX < SCREEN_WIDTH) {
                 tft.fillRect(endX, firstInputLineY, SCREEN_WIDTH - endX, LINE_HEIGHT, ST77XX_BLACK);
            }
        }

    } else {
        // No un-wrap.
        redrawTrailingText();
    }

    // --- 6. Restore cursor visibility ---
    cursorVisible = oldVisibility;
    // Draw cursor at final position (this might cause the glitch on non-prompt un-wraps).
    drawCursorAndPreview(); // This updates lastPreviewCols for the *current* state

    // --- 7. TARGETED PATCH for non-prompt lines ---
    // Apply patch only if an un-wrap happened AND we are NOT on the first line.
    if (didUnwrap) {
        const int LINE_1_CAPACITY = COLS - PROMPT.length();
        if (cursorPos >= LINE_1_CAPACITY) { // Check cursor is NOT on the first line

            // Recalculate segment info
            String segments[16];
            int segmentCount = 0;
            calculateFullWrapSegments(String(cmdBuf).substring(0, cmdLen), segments, segmentCount, 16, false);
            if (segmentCount == 0) segmentCount = 1;

            int linesToDraw = min(segmentCount, MAX_LINES);
            int startRow = MAX_LINES - linesToDraw;

            // Find segment index cursor landed on
            int tempCursorPos = cursorPos;
            int currentSegmentIndex = -1;
            for (int i = 0; i < segmentCount; ++i) {
                 if (tempCursorPos <= segments[i].length()) {
                     currentSegmentIndex = i;
                     break;
                 }
                 tempCursorPos -= segments[i].length();
            }

            // Ensure valid segment index > 0
            if (currentSegmentIndex > 0) {
                int currentScreenY = (startRow + (currentSegmentIndex - (segmentCount - linesToDraw))) * LINE_HEIGHT;
                String lineText = segments[currentSegmentIndex];

                // *** CHANGE: Use PROMPT.length() + 1 for the number of chars to redraw ***
                int charsToPatch = PROMPT.length() + 1; // Explicitly set the number of chars
                String patchText = lineText.substring(0, min(charsToPatch, lineText.length()));

                // Redraw the calculated number of characters over the glitch at x=0.
                tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
                tft.setCursor(0, currentScreenY);
                tft.print(patchText);
            }
        }
    }
    // --- End of targeted patches ---
}
void clearCurrentCommand() {
    memset(cmdBuf, 0, CMD_BUF);
    cmdLen = 0; cursorPos = 0;
    inputWrapped = false;
}
void loadHistoryCommand(int index) {
    if (historyCount == 0 || index < 0 || index >= historyCount) {
        pushSystemMessage("Error: Invalid history index.");
        return;
    }

    clearCurrentCommand(); 

    String command = history[index % HISTORY_SIZE]; // Use modulo for wrap-around array
    if (command.length() >= CMD_BUF) {
        pushSystemMessage("Error: Command too long to recall.");
        return;
    }
    
    command.toCharArray(cmdBuf, CMD_BUF);
    cmdLen = command.length();
    cursorPos = cmdLen;
    inputWrapped = false;
    
    drawFullTerminal();
}
// Helper to load command based on historyIndex, cycling down (older)
void historyRecallDown() {
    if (historyCount == 0) return;
    
    // If we're at the latest command (or typing new), start from the newest history item
    if (historyIndex == historyCount) { 
        historyIndex = historyCount - 1;
    } else {
        // Move to the next older command
        if (historyIndex > 0) {
            historyIndex--;
        } else {
            // Stay at the oldest command (index 0)
            return; 
        }
    }
    
    // Load the command
    loadHistoryCommand(historyIndex % HISTORY_SIZE);
}
// Helper to load command based on historyIndex, cycling up (newer/back to blank)
void historyRecallUp() {
    if (historyCount == 0) return;
    
    // Move to the next newer command
    if (historyIndex < historyCount - 1) {
        historyIndex++;
        loadHistoryCommand(historyIndex % HISTORY_SIZE);
    } else if (historyIndex == historyCount - 1) {
        // Move from the newest command to the blank command line
        historyIndex = historyCount;
        clearCurrentCommand(); 
        drawFullTerminal(); 
    }
}
void addHistory(const String &line) {
    if (line.length() == 0) return;
    if (historyCount > 0 && history[(historyCount - 1) % HISTORY_SIZE] == line) return;
    history[historyCount % HISTORY_SIZE] = line;
    if (historyCount < HISTORY_SIZE) historyCount++;
    historyIndex = historyCount;
    lastCommand = line; // Tracks the last command for F1, F2, F3
}
// ----------------------------
// Keyboard Handlers
// ----------------------------
void kbPrev() {
    if (fkeyState == F_AWAIT_FORMAT_CONFIRM) {
        // Use the dedicated formatIndex instead of kbIndex
        formatIndex--;
        // Constrain the index to wrap back to 1 ('N') if it falls below 0 ('Y')
        if (formatIndex < 0) { 
            formatIndex = 1; 
        }
        // NOTE: drawCursorAndPreview() must be updated to use formatIndex
        drawCursorAndPreview(); 
        return; 
    }
    kbIndex--;
    if (kbIndex < 0) {
        // ALPHA modes wrap to the [CASE]/[case] key at X + 3
        if (kmode == ALPHA || kmode == ALPHA_LOWER) kbIndex = strlen(alphaChars) + 3;
        // NUM and SYM modes wrap to the [ENTER] key at X + 2
        else if (kmode == NUM) kbIndex = strlen(numberChars) + 2;
        else if (kmode == SYM) kbIndex = strlen(symbolChars) + 2;
        // Max index for CTRL is [FUNC] (index 6)
        else if (kmode == CTRL) kbIndex = CTRL_COUNT + 1;
        // Max index for FUNC_VIEW is [F12] (index 12)
        else if (kmode == FUNC_VIEW) kbIndex = FUNC_COUNT;
    }
}
void kbNext() {
    if (fkeyState == F_AWAIT_FORMAT_CONFIRM) {
        // Use the dedicated formatIndex instead of kbIndex
        formatIndex++;
        // Constrain the index to wrap back to 0 ('Y') if it exceeds 1 ('N')
        if (formatIndex > 1) { 
            formatIndex = 0; 
        }
        // NOTE: drawCursorAndPreview() must be updated to use formatIndex
        drawCursorAndPreview(); 
        return;
    }
    int maxIndex = 0;
    
    // ALPHA modes have the extra [CASE]/[case] key at X + 3
    if (kmode == ALPHA || kmode == ALPHA_LOWER) {
        maxIndex = strlen(alphaChars) + 3; 
    } 
    // NUM and SYM modes only have [SPACE] (X+1) and [ENTER] (X+2)
    else if (kmode == NUM) {
        maxIndex = strlen(numberChars) + 2;
    } else if (kmode == SYM) {
        maxIndex = strlen(symbolChars) + 2;
    } 
    // Max index for CTRL is [FUNC] (index 6)
    else if (kmode == CTRL) {
        maxIndex = CTRL_COUNT + 1;
    }
    // Max index for FUNC_VIEW is [F12] (index 12)
    else if (kmode == FUNC_VIEW) {
        maxIndex = FUNC_COUNT;
    }
    
    kbIndex++;
    if (kbIndex > maxIndex) {
        kbIndex = 0;
    }
    drawCursorAndPreview();
}
void kbConfirm() {
    char charToInsert = 0;
    String controlAction = "";
    
    // --- Constant Definitions ---
    const int ALPHA_CASE_KEY_INDEX = (int)strlen(alphaChars) + 1; 
    const int NUM_KEY_INDEX_SPACE = (int)strlen(numberChars) + 1; 
    const int SYM_KEY_INDEX_SPACE = (int)strlen(symbolChars) + 1; 
    
    // --- 1. Handle F-KEY Awaited Input (Character/Number Insertion) ---
    if (fkeyState != F_INACTIVE) {
        char inputChar = 0;
        // Check if the current selection is a character/number
        if (kmode == ALPHA && kbIndex <= (int)strlen(alphaChars) && kbIndex > 0) {
            inputChar = alphaChars[kbIndex - 1]; 
        } else if (kmode == ALPHA_LOWER && kbIndex <= (int)strlen(alphaLowerChars) && kbIndex > 0) {
            inputChar = alphaLowerChars[kbIndex - 1];
        } else if (kmode == NUM && kbIndex <= (int)strlen(numberChars) && kbIndex > 0) {
            inputChar = numberChars[kbIndex - 1]; 
        } else if (kmode == SYM && kbIndex <= (int)strlen(symbolChars) && kbIndex > 0) {
            inputChar = symbolChars[kbIndex - 1];
        }
        
        if (inputChar != 0) {
            // F7 is the ONLY multi-digit input mode that doesn't terminate immediately.
            if (fkeyState == F7_AWAIT_INDEX) {
                 // For F7, insert the digit and do NOT call the handler.
                 insertCharAtCursor(inputChar);
            } else {
                 // For F2/F4/F9, insert the char and then call the handler to execute the action and reset state.
                 insertCharAtCursor(inputChar);
                 handleFKeyInput(inputChar);
            }
            // Always return after inserting a character in F-key mode.
            return; 
        } 
    }


    // --- FIX: 1.5. Handle Format Confirmation Selection (The fixed block) ---
    if (fkeyState == F_AWAIT_FORMAT_CONFIRM) {
        
        // 1. CAPTURE the user's choice.
        int confirmedIndex = formatIndex; // This is 0 for Y, 1 for N

        // 2. STATE RESET (CRITICAL: MUST HAPPEN NOW TO AVOID CONFLICTS)
        // Resetting kbIndex to 0 here is SAFE because we captured confirmedIndex
        // and we are about to exit the confirmation state completely.
        fkeyState = F_INACTIVE;
        kmode = ALPHA; 
        kbIndex = 0; // The fix: kbIndex=0 (Mode Key) is harmless if fallthrough occurs.
        cmdLen = 0;
        cursorPos = 0; 
        cmdBuf[0] = '\0';
        cursorVisible = true;
        
        // 3. EXECUTE the action based on the captured choice.
        bool willFormat = (confirmedIndex == 0); // 'Y' is at index 0.

        if (willFormat) { 
            pushSystemMessage("Format selection confirmed. Executing format...");
            drawFullTerminal(); // Draw message before blocking call.
            formatFilesystem();
        } else { 
            pushSystemMessage("Format cancelled.");
            // NOTE: We no longer return here. We proceed to cleanup (step 4).
        }
        
        // 4. PERFORM FINAL STATE CLEANUP AND REDRAW (CRITICAL: Both Yes and No run this)
        
        // Reset CLI drawing state
        terminalScrollOffset = 0; 
        inputWrapped = false; 
        historyIndex = historyCount; 

        // 4a. SWAP ORDER: CLEAR ARTIFACTS FIRST 
        // ** EXPANDED ARTIFACT FIX to full line **
        const int y_pos_start = (MAX_LINES - 1) * LINE_HEIGHT; 
        const int start_y = y_pos_start - 1;
        const int y_height = LINE_HEIGHT + 1; 
        
        // Wipe the entire last line of the screen to remove Y/N remnants
        tft.fillRect(0, start_y, SCREEN_WIDTH, y_height, ST77XX_BLACK); 
        
        // 4b. Redraw the full terminal (Draws the clean PICOS> prompt)
        drawFullTerminal(); 

        // 4c. Redraw the new [ALPHA] mode label and cursor.
        drawCursorAndPreview(); 
        
        // 5. ENSURE the function exits and does not fall through.
        return;
    }

    // ... (rest of the kbConfirm function logic)
    // --- 2. Mode Switch Logic (for Mode Label key, kbIndex=0) ---
    if (kbIndex == 0) {
        if (fkeyState != F_INACTIVE && kmode == FUNC_VIEW) {
        // Original logic preserved
        } else if (kmode == FUNC_VIEW) {
            kmode = CTRL;
        } else {
            if (kmode == ALPHA || kmode == ALPHA_LOWER) {
                kmode = NUM;
            } else if (kmode == NUM) {
                kmode = SYM;
            } else if (kmode == SYM) {
                kmode = CTRL;
            } else if (kmode == CTRL) {
                kmode = ALPHA;
            }
        }
        f1_copy_index = 0; 
        kbIndex = 0;
        drawCursorAndPreview();
        return;
    }

    // --- 3. Character Insertion & Special Key Logic Determination ---
    if (kmode == ALPHA) {
        if (kbIndex <= (int)strlen(alphaChars)) {
            charToInsert = alphaChars[kbIndex - 1]; 
        } 
        else if (kbIndex == ALPHA_CASE_KEY_INDEX) { 
            controlAction = "SPACE"; 
        }
        else if (kbIndex == ALPHA_CASE_KEY_INDEX + 1) { 
            controlAction = "ENTER";
        }
        else if (kbIndex == ALPHA_CASE_KEY_INDEX + 2) { 
            kmode = ALPHA_LOWER; 
            kbIndex = ALPHA_CASE_KEY_INDEX + 2; 
            drawCursorAndPreview(); 
            return;
        }
    }
    else if (kmode == ALPHA_LOWER) {
        if (kbIndex <= (int)strlen(alphaLowerChars)) {
            charToInsert = alphaLowerChars[kbIndex - 1]; 
        }
        else if (kbIndex == ALPHA_CASE_KEY_INDEX) {
            controlAction = "SPACE"; 
        }
        else if (kbIndex == ALPHA_CASE_KEY_INDEX + 1) { 
            controlAction = "ENTER";
        }
        else if (kbIndex == ALPHA_CASE_KEY_INDEX + 2) {
            kmode = ALPHA; 
            kbIndex = ALPHA_CASE_KEY_INDEX + 2;
            drawCursorAndPreview();
            return;
        }
    }
    else if (kmode == NUM) {
        if (kbIndex <= (int)strlen(numberChars)) {
            charToInsert = numberChars[kbIndex - 1];
        } 
        else if (kbIndex == NUM_KEY_INDEX_SPACE) { 
            controlAction = "SPACE"; 
        } 
        else if (kbIndex == NUM_KEY_INDEX_SPACE + 1) { 
            controlAction = "ENTER"; 
        } 
    }
    else if (kmode == SYM) {
        if (kbIndex <= (int)strlen(symbolChars)) {
            charToInsert = symbolChars[kbIndex - 1];
        } 
        else if (kbIndex == SYM_KEY_INDEX_SPACE) {
            controlAction = "SPACE"; 
        }
        else if (kbIndex == SYM_KEY_INDEX_SPACE + 1) { 
            controlAction = "ENTER";
        }
    }
    else if (kmode == CTRL) {
        if (kbIndex <= CTRL_COUNT) {
            controlAction = ctrlKeys[kbIndex - 1];
        } else if (kbIndex == CTRL_COUNT + 1) { 
            kmode = FUNC_VIEW;
            f1_copy_index = 0; 
            kbIndex = 0;
            drawCursorAndPreview();
            return; 
        }
    }
    else if (kmode == FUNC_VIEW) {
        if (kbIndex <= FUNC_COUNT) {
            handleFKeyAction(kbIndex); 
            
            // FIX: Prevent mode switch back to CTRL if F1 (kbIndex=1) was pressed, 
            // but still switch if other F-keys (F2, F3, F4, etc.) are pressed and finished.
            if (kbIndex != 1 && fkeyState != F7_AWAIT_INDEX && fkeyState != F9_AWAIT_INDEX) {
                kmode = CTRL; 
            }
            
            kbIndex = 0;
            drawCursorAndPreview();
            return;
        }
    }

    // --- 4. Execute Character or Control Action ---
    if (charToInsert != 0) {
        insertCharAtCursor(charToInsert);
        // FIX: Reset F1 sequence when ANY normal character is inserted
        f1_copy_index = 0; 
    } 
    else if (controlAction.length() > 0) {
        if (controlAction == "SPACE") {
            insertCharAtCursor(' ');
            // FIX: Reset F1 sequence when SPACE is inserted
            f1_copy_index = 0; 
        } 
        
        // ****************************************
        // ** F7 HISTORY SELECTION MODE HANDLER **
        // ****************************************
        else if (controlAction == "ENTER" && fkeyState == F7_AWAIT_INDEX) {
            
            String fullInput = String(cmdBuf).substring(0, cmdLen);
            
            int lastSpace = fullInput.lastIndexOf(' ');
            String indexInput = fullInput.substring(lastSpace + 1);
            indexInput.trim(); 

            int index = -1;
            
            if (indexInput.length() > 0) {
                index = indexInput.toInt();
            } 
            
            if (index >= 0 && index < historyCount) {
                String commandToInsert = history[index % HISTORY_SIZE];
                
                int deleteCount = indexInput.length() + 1; 

                if (lastSpace == -1) {
                    deleteCount = indexInput.length();
                }

                for (int i = 0; i < deleteCount; ++i) {
                     backspaceAtCursor(); 
                }
                
                insertStringAtCursor(commandToInsert);
                
                pushSystemMessage("Inserted history item " + String(index) + ".");
                
            } else if (indexInput.length() == 0) {
                pushSystemMessage("History selection canceled.");
            } else {
                pushSystemMessage("Invalid index " + indexInput + ". Must be 0-" + String(historyCount - 1) + ".");
                
                int deleteCount = indexInput.length() + 1; 
                if (lastSpace == -1) deleteCount = indexInput.length();
                for (int i = 0; i < deleteCount; ++i) {
                     backspaceAtCursor(); 
                }
            }
            
            fkeyState = F_INACTIVE; 
            kmode = ALPHA; 
            kbIndex = 0;
            
            drawFullTerminal();
            return;
        }
        // ****************************************
        
        else if (controlAction == "ENTER") {
            f1_copy_index = 0; 
            String currentInput = String(cmdBuf).substring(0, cmdLen);
            String fullCommand = "";
            
            int i = scrollbackCount - 1;
            if (inputWrapped) { 
                while (i >= 0) {
                    int idx = (scrollbackHead + i) % SCROLLBACK_SIZE;
                    String line = scrollback[idx].text;
                    
                    if (line.startsWith(PROMPT)) {
                        fullCommand = line.substring(PROMPT.length()) + fullCommand;
                        scrollbackCount = i;
                        break;
                    } 
                    else if (!line.startsWith(SYS_PROMPT)) { 
                        fullCommand = line + fullCommand;
                        i--;
                    } else {
                        break;
                    }
                }
            } 
            
            fullCommand += currentInput;
            
            // --- START OF FIX: CHECK FOR EMPTY COMMAND ---
            String trimmedCommand = trimStr(fullCommand); 

            if (trimmedCommand.length() == 0) {
                // If the command is empty (or only whitespace), 
                // clear the input buffer and redraw the screen, then exit.
                clearCmdBuffer(); 
                drawFullTerminal(); 
                return; 
            }
            // --- END OF FIX ---
            
            // Proceed with scrollback and execution ONLY for non-empty commands
            
            const int MAX_CHUNKS = 16;
            String fwdFinal[MAX_CHUNKS];
            int fwdCountFinal = 0;
            calculateFullWrapSegments(fullCommand, fwdFinal, fwdCountFinal, MAX_CHUNKS, false);
            if (fwdCountFinal > 0) {
                 pushScrollback(PROMPT + fwdFinal[0]);
            }
            for (int j = 1; j < fwdCountFinal; j++) {
                pushScrollback(fwdFinal[j]);
            }
            
            addHistory(fullCommand);
            executeCommandLine(fullCommand);

            clearCmdBuffer(); 
            return;
        } 
        else if (controlAction == "DELETE") { 
            if (cursorPos < cmdLen) {
                for (int i = cursorPos; i < cmdLen - 1; ++i) cmdBuf[i] = cmdBuf[i + 1];
                cmdLen--;
                cmdBuf[cmdLen] = 0;
                drawFullTerminal(); 
            }
        } 
        else if (controlAction == "LEFT") {
            // New logic: function as a simple "move left one character" key.
            if (cursorPos > 0) {
                cursorPos--;
                drawCursorAndPreview();
            }
        }
        else if (controlAction == "RIGHT") {
            if (cursorPos < cmdLen) cursorPos++;
        }
        
        drawCursorAndPreview();
    }
}
// Function to handle the specific F-key actions
void handleFKeyAction(int fKeyNumber) {
    // Reset F1 index on new F-key press unless it is F1
    if (fKeyNumber != 1) {
        f1_copy_index = 0;
    }
    
    // All F-key actions should reset historyIndex to the "new command" state 
    // unless they specifically manipulate the history (F5, F8).
    if (fKeyNumber != 5 && fKeyNumber != 8) {
        historyIndex = historyCount;
    }

    // F-key specific actions
    switch (fKeyNumber) {
        case 1: // F1: Repeats the letters of the last command line, one by one.
            if (lastCommand.length() > 0) {
                if (f1_copy_index < lastCommand.length()) {
                    insertCharAtCursor(lastCommand.charAt(f1_copy_index));
                    f1_copy_index++;
                } else {
                    f1_copy_index = 0; // Wrap/restart
                }
            } else {
                pushSystemMessage("No last command for F1.");
            }
            break;
        case 2: // F2: Prompts the user to "enter the char to copy up to" from the last command line
            if (lastCommand.length() > 0) {
                fkeyState = F2_AWAIT_CHAR;
                pushSystemMessage("F2: Enter char to copy up to:");
            } else {
                 pushSystemMessage("No last command for F2.");
            }
            break;
        case 3: // F3: Repeats the entire last command line.
            if (lastCommand.length() > 0) {
                clearCurrentCommand();
                insertStringAtCursor(lastCommand);
            } else {
                pushSystemMessage("No last command for F3.");
            }
            break;
        case 4: // F4: Prompts the user to "enter the char to delete up to" from the current command line
            if (cmdLen > 0) {
                fkeyState = F4_AWAIT_CHAR;
                pushSystemMessage("F4: Enter char to delete up to:");
            } else {
                pushSystemMessage("Command line is empty for F4.");
            }
            break;
        case 5: // F5: Recalls the previous command line from the history (newest, no cycling).
            if (historyCount > 0) {
                // F5: Recall the absolute newest command
                historyIndex = historyCount - 1; 
                loadHistoryCommand(historyIndex % HISTORY_SIZE);
                historyIndex = historyCount; // Set back to new command state after loading
            } else {
                pushSystemMessage("History is empty for F5.");
            }
            break;
        case 6: // F6: Inserts the traditional CTRL+Z (^z) character.
            insertCharAtCursor('\x1A');
            break;
            
        case 7: // F7: Displays command history and awaits index for insertion.
            if (historyCount > 0) {
                pushSystemMessage("--- Command History (0-" + String(historyCount - 1) + ") ---");
                for (int i = 0; i < historyCount; ++i) { 
                    String h = String(i) + ": " + history[i % HISTORY_SIZE];
                    pushScrollback(h);
                }
                
                // 1. Set the new state to await numeric index input
                fkeyState = F7_AWAIT_INDEX; 
                
                // 2. FIX: Switch the keyboard to the NUMERIC layer for easy entry
                kmode = NUM; 
                kbIndex = 0; // Reset index for the new mode
                
                // 3. Push instruction prompt
                pushSystemMessage("Enter history index (0-" + String(historyCount - 1) + ") and press ENTER to insert:");
            } else { 
                pushSystemMessage("History is empty for F7."); 
            } 
            break;
        case 8: // F8: Cycles back through previous command lines. (Simplified to recall down)
            historyRecallDown();
            break;
            
        case 9: // F9: Prompts the user to enter a command number (0-9). (Simplified to single digit)
            if (historyCount > 0) {
                fkeyState = F9_AWAIT_INDEX;
                pushSystemMessage("F9: Enter history command number (0-9):");
            } else {
                pushSystemMessage("History is empty for F9.");
            }
            break;
            
        // F10, F11, F12 default to text insertion
        default:
            insertCharAtCursor('[');
            for (int i = 0; i < funcKeys[fKeyNumber - 1].length(); ++i) insertCharAtCursor(funcKeys[fKeyNumber - 1].charAt(i));
            insertCharAtCursor(']');
            break;
    }

    // Always redraw full terminal after an F-key action (except F1)
    if (fKeyNumber != 1) {
        drawFullTerminal(); 
    }
}
// Function to handle awaited input for F2, F4, F9
void handleFKeyInput(char inputChar) {
    if (fkeyState == F2_AWAIT_CHAR) {
        // F2: Copy up to char from last command
        int targetIndex = lastCommand.indexOf(inputChar);
        if (targetIndex != -1) {
            String sub = lastCommand.substring(0, targetIndex + 1);
            clearCurrentCommand();
            insertStringAtCursor(sub);
            pushSystemMessage("Copied last command up to '" + String(inputChar) + "'.");
        } else {
            pushSystemMessage("Char '" + String(inputChar) + "' not found in last command.");
        }
        fkeyState = F_INACTIVE;
    } else if (fkeyState == F4_AWAIT_CHAR) {
        // F4: Delete up to char from current command
        String currentCmd = String(cmdBuf).substring(0, cmdLen);
        int targetIndex = currentCmd.indexOf(inputChar);
        if (targetIndex != -1) {
            int deleteCount = targetIndex + 1;
            // Shift remaining characters left
            for (int i = 0; i < cmdLen - deleteCount; ++i) {
                cmdBuf[i] = cmdBuf[i + deleteCount];
            }
            cmdLen -= deleteCount;
            cmdBuf[cmdLen] = 0;
            cursorPos = 0; // Cursor moves to the start
            pushSystemMessage("Deleted current command up to '" + String(inputChar) + "'.");
        } else {
            pushSystemMessage("Char '" + String(inputChar) + "' not found in current command.");
        }
        fkeyState = F_INACTIVE;
    } else if (fkeyState == F9_AWAIT_INDEX) {
        // F9: Recall command by number (0-9)
        if (inputChar >= '0' && inputChar <= '9') {
            int index = inputChar - '0';
            if (index < historyCount) {
                loadHistoryCommand(index % HISTORY_SIZE);
                historyIndex = historyCount; // Set back to new command state after loading
                pushSystemMessage("Recalled history item " + String(index) + ".");
            } else {
                pushSystemMessage("Index " + String(index) + " is out of history range (0-" + String(historyCount > 0 ? historyCount - 1 : 0) + ").");
            }
        } else {
            pushSystemMessage("Invalid history index. Must be a digit 0-9.");
        }
        fkeyState = F_INACTIVE;
    }
    
    // Final state cleanup and redraw
    kmode = ALPHA;
    kbIndex = 0;
    drawFullTerminal(); 
}
// ----------------------------
// Command parsing & execution helpers
// ----------------------------
String trimStr(const String &s) {
    int i = 0;
    while (i < s.length() && isspace(s[i])) i++;
    int j = s.length() - 1; while (j >= 0 && isspace(s[j])) j--;
    if (j < i) return "";
    return s.substring(i, j + 1);
}
// basic arithmetic calculator
String evalCalc(const String &expr) {
    String s = expr;
    String t = "";
    for (unsigned int i = 0; i < s.length(); ++i) if (s[i] != ' ') t += s[i];
    s = t;

    struct Tok { String v; char type; };
    Tok *out = nullptr; int out_sz = 0;
    String *op = nullptr; int op_sz = 0;

    auto push_out = [&](Tok tk) { out = (Tok*)realloc(out, sizeof(Tok)*(out_sz+1));
        out[out_sz++] = tk; };
    auto push_op = [&](const String &o) { op = (String*)realloc(op, sizeof(String)*(op_sz+1)); op[op_sz++] = o; };
    auto pop_op = [&]()->String { String r = op[op_sz-1]; op_sz--; op = (String*)realloc(op, sizeof(String)*max(0,op_sz)); return r; };
    
    // FIX 1: Corrected typo: c=='*'||c=='/'
    auto isOp = [&](char c){ return c=='+'||c=='-'||c=='*'||c=='/'; }; 
    auto prec = [&](char c)->int { if (c=='+'||c=='-') return 1;
        if (c=='*'||c=='/') return 2; return 0; };

    for (unsigned int i = 0; i < s.length();) {
        char c = s[i];
        if ((c >= '0' && c <= '9') || c=='.') {
            String num="";
            while (i<s.length() && ((s[i]>='0'&&s[i]<='9') || s[i]=='.')) { num+=s[i]; i++; }
            push_out({num, 'n'});
        } else if (isOp(c)) {
            String opch(1,c);
            while (op_sz && isOp(op[op_sz-1][0]) && prec(op[op_sz-1][0]) >= prec(c)) {
                String pop = pop_op();
                push_out({pop,'o'});
            }
            push_op(opch);
            i++;
        } else if (c=='(') { push_op(String("(")); i++; }
        else if (c==')') {
            bool found=false;
            while (op_sz) {
                String top = pop_op();
                if (top=="(") { found=true; break; }
               
                push_out({top,'o'});
            }
            if (!found) { free(out); free(op); return "ERR"; }
            i++;
        } else return "ERR";
    }
    
    // FIX 2: Missing closing brace for the 'for' loop (Shunting-Yard)
    while (op_sz) {
        String top = pop_op();
        if (top=="("||top==")") { free(out); free(op); return "ERR"; }
        push_out({top,'o'});
    }
    
    double *st = nullptr; int st_sz = 0;
    auto push_st = [&](double v){ st = (double*)realloc(st, sizeof(double)*(st_sz+1)); st[st_sz++]=v; };
    auto pop_st = [&]()->double{ double v = st[st_sz-1]; st_sz--; st=(double*)realloc(st,sizeof(double)*max(0,st_sz)); return v; };

    for (int i = 0; i < out_sz; ++i) {
        Tok &tk = out[i]; 
        if (tk.type=='n') push_st(atof(tk.v.c_str())); 
        else if (tk.type=='o') { 
            if (st_sz < 2) { free(out); free(op); free(st); return "ERR"; } 
            double b = pop_st(); 
            double a = pop_st(); 
            char opch = tk.v[0]; 
            double res = 0; 
            if (opch=='+') res = a + b; 
            else if (opch=='-') res = a - b; 
            else if (opch=='*') res = a * b;
            else if (opch=='/') { 
                if (b == 0) { free(out); free(op); free(st); return "INF"; } 
                res = a / b;
            } 
            push_st(res); 
        } 
    } 
    
    if (st_sz != 1) { free(out); free(op); free(st); return "ERR"; } 
    char buf[64];
    dtostrf(st[0], 0, 6, buf); 
    String outstr = String(buf); 
    while (outstr.length()>1 && outstr.indexOf('.')>=0 && (outstr.endsWith("0") || outstr.endsWith("."))) { 
        if (outstr.endsWith("0")) outstr.remove(outstr.length()-1);
        else if (outstr.endsWith(".")) { outstr.remove(outstr.length()-1); break; }
    }
    
    free(out); free(op); free(st); 
    return outstr;
}
void tokenizeLine(const String &line, String tokens[], int &count, int maxTokens) {
    count = 0;
    int current = 0;
    bool inQuote = false;
    
    for (int i = 0; i < line.length() && count < maxTokens; ++i) {
        char c = line[i];
        
        if (c == '"') {
            inQuote = !inQuote;
        } else if (isspace(c) && !inQuote) {
            if (current > 0) {
                tokens[count++] = line.substring(i - current, i);
                current = 0;
            }
        } else {
            if (current == 0 && count < maxTokens) {
                // Start of a new token
                int start = i;
                while (i < line.length() && (inQuote || !isspace(line[i]))) {
                    if (line[i] == '"') inQuote = !inQuote;
                    i++;
                }
                tokens[count++] = line.substring(start, i);
                i--; // Back up one to re-check the space/next char
                current = 0; // Reset current, token is saved
            }
        }
    }

    if (current > 0 && count < maxTokens) {
        tokens[count++] = line.substring(line.length() - current);
    }
    
    // Final post-processing to strip surrounding quotes if present
    for (int i = 0; i < count; ++i) {
        if (tokens[i].startsWith("\"") && tokens[i].endsWith("\"") && tokens[i].length() >= 2) {
            tokens[i] = tokens[i].substring(1, tokens[i].length() - 1);
        }
    }
}
void executeCommandLine(const String &raw) {
    String line = trimStr(raw);
    if (line.length() == 0) {
        drawFullTerminal(); 
        clearCurrentCommand();
        return;         
    }

    // Add command to history
    addHistory(line); 

    const int MAX_TOKENS = 8;
    String tokens[MAX_TOKENS];
    int count = 0;
    tokenizeLine(line, tokens, count, MAX_TOKENS);
    
    if (count == 0) return;
    
    String cmd = tokens[0];
    cmd.toLowerCase(); 
    
    if (cmd == "help") {
        pushSystemMessage("Available commands:");
        pushScrollback("help         - Show this message.");
        pushScrollback("clear        - Clear terminal history.");
        pushScrollback("calc <expr>  - Evaluate simple math.");
        pushScrollback("pi           - Display Rainbow Pi"); 
        pushScrollback("ls           - List files on LittleFS.");
        pushScrollback("cat <file>   - Display file content.");
        pushScrollback("echo <text> >/>> <file> - Write file.");
        pushScrollback("rm <file>    - Delete a file.");
        pushScrollback("ver          - Display version info.");
        pushScrollback("time         - Show uptime since boot.");
        pushScrollback("fkey         - Show F-key functions.");
        pushScrollback("send <file>  - Send file to PC via USB.");
        pushScrollback("format       - Format LT-FS partition.");
        pushScrollback("cube         - 3D CUBE, back to exit.");
        pushScrollback("mood         - Cycle through RGB colors.");
        pushScrollback("moon         - Moon phases.");
    } else if (cmd == "fkey") {
        pushScrollback("F1: Print last command, char by char.");
        pushScrollback("--- F-Key functionality: ---");
        pushScrollback("F2: Copy last cmd up to char.");
        pushScrollback("F3: Repeat last cmd.");
        pushScrollback("F4: Delete current cmd up to char.");
        pushScrollback("F5: Recall last cmd. F6: Insert ^Z.");
        pushScrollback("F7: Show history.");
        pushScrollback("F8: Cycle back history.");  
        pushScrollback("F9: Recall by history index.");      

    } else if (cmd == "clear") {
        scrollbackCount = 0; 
        scrollbackHead = 0;
    } else if (cmd == "cube") {
        runCubeAnimation();
        return;
    } else if (cmd == "mood") {
        mood();
        return;
    } else if (cmd == "moon") { 
        runMoonPhase();
        return;
    } else if (cmd == "ver") {
        pushSystemMessage(deviceVersion);

    } else if (cmd == "echo") {
        String output = "";
        String filePath = "";
        bool appendMode = false;
        bool redirection = false;

        for (int i = 1; i < count; ++i) {
            if (tokens[i] == ">" || tokens[i] == ">>") {
                redirection = true;
                appendMode = (tokens[i] == ">>");
                if (i + 1 < count) filePath = tokens[i + 1];
                else { pushSystemMessage("Error: Missing filename write!"); return; }
                break;
            }
            if (output.length() > 0) output += " ";
            output += tokens[i];
        }

        if (redirection) {
            if (filePath.length() == 0) pushSystemMessage("Error: Missing filename for write!");
            else if (writeFile(filePath, output, appendMode))
                pushSystemMessage("Echo " + String(appendMode ? ">> " : "> ") + filePath + ": Success!");
            else pushSystemMessage("Error: Failed to write to " + filePath);
        } else {
            pushSystemMessage(output);
        }

    } else if (cmd == "time") {
        unsigned long uptime = (millis() - startMillis) / 1000;
        unsigned long seconds = uptime % 60;
        unsigned long minutes = (uptime / 60) % 60;
        unsigned long hours = (uptime / 3600);
        char buf[32];
        sprintf(buf, "Uptime: %lu:%02lu:%02lu", hours, minutes, seconds);
        pushSystemMessage(String(buf));

    } else if (cmd == "calc") {
        if (count < 2) {
            pushSystemMessage("Usage: calc <expression>");
        } else {
            String expr = raw.substring(cmd.length());
            expr.trim(); 
            String result = evalCalc(expr);
            if (result.startsWith("ERR"))
                pushSystemMessage("Error: Invalid expression or " + result.substring(4) + ".");
            else if (result.startsWith("INF"))
                pushSystemMessage("Error: Division by zero.");
            else
                pushScrollback("= " + result);
        }

    } else if (cmd == "ls") {
        pushScrollback(listFiles());

    } else if (cmd == "cat") {
        if (count < 2) pushSystemMessage("Usage: cat <filename>");
        else pushScrollback(readFile(tokens[1]));

    } else if (cmd == "rm") {
        if (count < 2) pushSystemMessage("Usage: rm <filename>");
        else {
            if (removeFile(tokens[1]))
                pushSystemMessage("Deleted " + tokens[1] + ".");
            else pushSystemMessage("Error: File not found or couldn't be deleted.");
        }
    } else if (cmd == "format") { 
        if (!fsReady) {
            pushSystemMessage("Error: LittleFS not available. Terminating...");
        } else if (fkeyState != F_INACTIVE) {
            pushSystemMessage("Error: Already in special input mode.");
        } else {
            // --- ENTER CONFIRMATION MODE ---
            pushSystemMessage("WARNING: Select Y/N to confirm FORMAT!");
            
            fkeyState = F_AWAIT_FORMAT_CONFIRM; 
            kbIndex = 0; // Start selection on 'Y' (kbIndex 0 = Y, 0 = N)
            // No need to change kmode, as kbConfirm/draw functions will ignore it.
            
            // Draw the new confirmation prompt immediately.
            drawFullTerminal(); 
        }
    } else if (cmd == "pi") {
        String piValue = String(PI_VALUE, 18);
        String piString = "Pi = " + piValue;
        String fullString = SYS_PROMPT + piString;
        pushScrollback(fullString, ST77XX_BLACK);
        int idx = (scrollbackHead + scrollbackCount - 1) % SCROLLBACK_SIZE;
        storeRainbowData(idx, fullString);
        int lineToDraw = MAX_LINES - 2;
        tft.fillRect(0, lineToDraw * LINE_HEIGHT, SCREEN_WIDTH, LINE_HEIGHT, ST77XX_BLACK);
        tft.setCursor(0, lineToDraw * LINE_HEIGHT);
        tft.setTextColor(ST77XX_GREEN, ST77XX_BLACK); 
        tft.print(SYS_PROMPT); 
        drawMultiColorString(piString, lineToDraw, SYS_PROMPT.length() * CHAR_WIDTH);
        tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);

    } else if (cmd == "send") {
        if (count < 2) pushSystemMessage("Usage: send <filename>");
        else {
            String filename = tokens[1];
            if (!LittleFS.exists(filename)) {
                pushSystemMessage("Error: File not found: " + filename);
            } else {
                File file = LittleFS.open(filename, "r");
                if (!file) pushSystemMessage("Error: Could not open file: " + filename);
                else {
                    size_t filesize = file.size();
                    Serial.printf("SEND %s %u\n", filename.c_str(), (unsigned)filesize);

                    const size_t blockSize = 512;
                    uint8_t buffer[blockSize];
                    size_t sent = 0;

                    while (file.available()) {
                        size_t toRead = min(file.available(), blockSize);
                        file.read(buffer, toRead);
                        Serial.write(buffer, toRead);
                        sent += toRead;
                    }
                    file.close();
                    Serial.println("\nEND");
                    pushSystemMessage("File sent: " + filename + " (" + String(sent) + " bytes)");
                }
            }
        }

    } else {
        pushSystemMessage("Error: Unknown command '" + cmd + "'. Type 'help'.");
    }

    drawFullTerminal();
    clearCurrentCommand();
}
void drawMultiColorString(const String &text, int lineNum, int x_start) {
    // MAX_LINES, LINE_HEIGHT, CHAR_WIDTH, and SCREEN_WIDTH are assumed to be defined
    if (lineNum < 0 || lineNum >= MAX_LINES) return;

    // Assumes 'tft' is the global Adafruit_ST7789 object
    tft.setTextWrap(false);
    tft.setFont(NULL);
    tft.setTextSize(1);
    
    int x = x_start;
    int y = lineNum * LINE_HEIGHT;
    
    // RAINBOW_COUNT and RAINBOW_COLORS are assumed to be defined globally
    for (int i = 0; i < text.length(); i++) {
        char c = text.charAt(i);
        uint16_t charColor = RAINBOW_COLORS[i % RAINBOW_COUNT];
        
        tft.setCursor(x, y);
        tft.setTextColor(charColor, ST77XX_BLACK);
        tft.write(c); 
        
        // CHAR_WIDTH is assumed to be defined
        x += CHAR_WIDTH;
        if (x + CHAR_WIDTH > SCREEN_WIDTH) {
            break;
        }
    }
}
// ----------------------------
// File System (LittleFS) Wrappers
// ----------------------------
bool fsBegin() {
    // FIX: LittleFS.begin() takes no arguments in the RP2040 core.
    if (!LittleFS.begin()) { 
        Serial.println("LittleFS Mount Failed - Attempting Format...");
        if (LittleFS.format()) {
            Serial.println("LittleFS Formatted. Attempting remount...");
            // FIX: LittleFS.begin() takes no arguments.
            return LittleFS.begin();
        } else {
            Serial.println("LittleFS Format Failed.");
            return false;
        }
    }
    return true;
}
String listFiles() {
    String output = "--- Files ---";
    // FIX: LittleFS.open() for root directory must specify mode "r".
    File root = LittleFS.open("/", "r");
    if (!root) {
        return "Error: Could not open FS root.";
    }

    File file = root.openNextFile();
    while (file) {
        output += "\n" + String(file.name()) + " (" + String(file.size()) + " bytes)";
        file.close();
        file = root.openNextFile();
    }
    root.close();
    if (output == "--- Files ---") return "--- No files on LittleFS ---";
    return output;
}
String readFile(const String &path) {
    if (!LittleFS.exists(path)) return "Error: File not found.";
    File file = LittleFS.open(path, "r");
    if (!file) return "Error: Could not open file.";

    String content = "--- " + path + " ---\n";
    while (file.available()) {
        content += (char)file.read();
    }
    file.close();
    return content;
}
bool writeFile(const String &path, const String &data, bool append) {
    File file = LittleFS.open(path, append ? "a" : "w");
    if (!file) return false;
    
    file.print(data);
    file.close();
    return true;
}
bool removeFile(const String &path) {
    if (!LittleFS.exists(path)) return false;
    return LittleFS.remove(path);
}
/**
 * @brief Formats the LittleFS filesystem, managing the Watchdog Timer (WDT).
 * @return true if format and remount were successful, false otherwise.
 */
bool formatFilesystem() {
    pushSystemMessage("Initiating file system format. LONG OPERATION - DO NOT POWER OFF!");
    
    // 1. Unmount the filesystem (required before formatting)
    LittleFS.end();
    
    // 2. WATCHDOG TIMER MANAGEMENT (CRITICAL)
    // IMPORTANT: Formatting takes several seconds and will trigger the WDT.
    // Replace WDT_DISABLE() and WDT_ENABLE() with your specific platform calls 
    // (e.g., ESP.wdtDisable(), watchDog.disable(), etc.)
    // If you do not have a WDT function, define a macro that does nothing (e.g., #define WDT_DISABLE() )
    WDT_DISABLE(); 

    // 3. Execute the actual format
    bool success = LittleFS.format(); 

    // 4. Re-enable WDT immediately
    WDT_ENABLE();

    if (success) {
        pushSystemMessage("LittleFS format complete.");
        
        // 5. Attempt to remount the newly formatted filesystem
        if (LittleFS.begin()) {
            pushSystemMessage("LittleFS remounted successfully.");
            fsReady = true;
            return true;
        } else {
            pushSystemMessage("Fatal Error: LittleFS remount failed after format!");
            fsReady = false;
            return false;
        }
    } else {
        pushSystemMessage("Error: LittleFS format failed!");
        fsReady = false;
        return false;
    }
}
// ----------------------------
// FILE SENDING (PC -> PICO)
// ----------------------------
/**
 * @brief Executes the flow-controlled binary UPLOAD process from PC to Pico.
 * This function is BLOCKING for the duration of the file transfer.
 * @param filename The name of the file to save.
 * @param fileSize The exact size of the file expected.
 */
// ----------------------------
// FILE SENDING (PC -> PICO) - FLOW CONTROL FIX
// ----------------------------
/**
 * @brief Executes the flow-controlled binary UPLOAD process from PC to Pico.
 * This function is BLOCKING for the duration of the file transfer.
 * @param filename The name of the file to save.
 * @param fileSize The exact size of the file expected.
 */
void executeUpload(String filename, size_t fileSize) {
    if (!fsReady) {
        Serial.println("FATAL ERROR: LittleFS not available.");
        pushSystemMessage("Error: LittleFS not available.");
        drawFullTerminal();
        return;
    }

    pushSystemMessage("DOWNLOADING: " + filename + " (" + String(fileSize) + " bytes)");
    drawFullTerminal(); 
    delay(50); // Pause briefly to ensure the message is fully drawn to the TFT

    if (filename.startsWith("/")) {
        filename = filename.substring(1);
    }

    // 1. Open the file for writing
    File outFile = LittleFS.open(filename, "w");
    if (!outFile) {
        Serial.println("FATAL ERROR: Could not open file for writing.");
        pushSystemMessage("DOWNLOAD FAILED: Cannot open file.");
        return;
    }

    // 2. Send READY signal to the PC application
    Serial.print(READY_MSG);
    
    size_t bytesRemaining = fileSize;
    uint8_t buffer[BLOCK_SIZE];
    bool success = true;

    // 3. Main data receiving loop with ACK flow control
    while (bytesRemaining > 0) {
        size_t bytesToRead = min((size_t)BLOCK_SIZE, bytesRemaining);
        
        // A. BLOCKING READ: Wait until the entire block is received in RAM
        if (!serialBlockRead(buffer, bytesToRead, 5000)) { 
            success = false;
            break; 
        }
        
        // B. CRITICAL FIX: Send ACK immediately after receiving the data, 
        //    *before* the slow LittleFS write operation.
        if (bytesRemaining > bytesToRead) { // Only send ACK if more data is coming
            Serial.print(ACK_MSG);
            // Force the ACK to leave the Pico buffer immediately
            Serial.flush(); 
        }

        // C. SLOW OPERATION: Now write the data to the slow filesystem
        if (outFile.write(buffer, bytesToRead) != bytesToRead) {
            Serial.println("FATAL ERROR: FS write error.");
            success = false;
            break;
        }

        bytesRemaining -= bytesToRead;
        
        // Progress update (optional, but good for large files)
        // pushSystemMessage("Received " + String(fileSize - bytesRemaining) + " / " + String(fileSize));
    }
    
    // 4. Finalize
    outFile.close();
    while (Serial.available()) Serial.read(); // Clean up any remaining serial garbage

    if (success && bytesRemaining == 0) {
        // Success confirmation
        Serial.print("UPLOAD_OK ");
        Serial.print(filename);
        Serial.print(" ");
        Serial.println(fileSize);
        pushSystemMessage("SUCCESS: " + filename + " saved.");
    } else {
        pushSystemMessage("DOWNLOAD FAILED. Removing file.");
        LittleFS.remove(filename);
    }
    drawFullTerminal(); 
    delay(50); // Pause briefly (50ms) to ensure the TFT completes the final draw
}
// ----------------------------
// FILE RECEIVING (PICO -> PC)
// ----------------------------
/**
 * @brief Reads the contents of a file and streams it to the serial port.
 * @param filename The name of the file to stream.
 */
void executeCat(String filename) {
    if (!fsReady) {
        pushSystemMessage("Error: LittleFS not available.");
        Serial.println("CAT_ERROR LittleFS not available.");
        return;
    }

    if (filename.startsWith("/")) {
        filename = filename.substring(1);
    }

    File file = LittleFS.open(filename, "r"); // Use "r" for read mode
    
    if (!file) {
        Serial.print("CAT_ERROR File not found: ");
        Serial.println(filename);
        pushSystemMessage("Error: File not found: " + filename);
        return;
    }

    size_t fileSize = file.size();
    
    // 1. Send START Marker and file information
    Serial.print("CAT_START ");
    Serial.print(filename);
    Serial.print(" ");
    Serial.println(fileSize);
    
    pushSystemMessage("Streaming file: " + filename + " (" + String(fileSize) + " bytes)");

    // 2. Stream raw binary data
    size_t bytesRead;
    uint8_t buffer[512];
    
    while ((bytesRead = file.read(buffer, sizeof(buffer))) > 0) {
        Serial.write(buffer, bytesRead);
        yield();
    }

    file.close();
    
    // 3. Send END Marker
    Serial.println("CAT_END");
    
    pushSystemMessage("Streaming complete: " + filename);
}
// ----------------------------
// Serial Command Handler (FIXED & CONSOLIDATED)
// ----------------------------
/**
 * @brief Handles incoming serial commands from a host PC.
 * * This function parses commands and delegates file transfer to blocking functions.
 * * It does NOT handle file reception via non-blocking state machines.
 */
void handleSerialCommands() {
    // Only static variables needed for command line parsing
    static char buffer[512];
    static size_t commandBufLen = 0;
    static bool sawCarriageReturn = false; 

    while (Serial.available()) {
        char c = Serial.read();

        // 1. Handle CR/LF and execute command
        if (c == '\n' || c == '\r') {
            
            // Handle the two-character newline sequence (\r\n or \n\r)
            sawCarriageReturn = (c == '\r');

            if (commandBufLen > 0) {
                buffer[commandBufLen] = '\0';
                String cmdLine = String(buffer);
                commandBufLen = 0;

                cmdLine.trim();
                if (cmdLine.length() == 0) continue;

                // Extract command word for logic
                int firstSpace = cmdLine.indexOf(' ');
                String command = (firstSpace == -1) ? cmdLine : cmdLine.substring(0, firstSpace);
                command.toUpperCase();

                // ----------------------------
                // UPLOAD command (Send file from PC to Pico)
                // ----------------------------
                if (command == "UPLOAD") {
                    
                    String args = cmdLine.substring(firstSpace + 1);
                    int secondSpace = args.indexOf(' ');
                    
                    if (secondSpace > 0) {
                        String filename = args.substring(0, secondSpace);
                        String sizeStr = args.substring(secondSpace + 1);
                        size_t fileSize = (size_t)sizeStr.toInt();
                        
                        // Clear any residual serial data before entering blocking routine
                        while (Serial.available()) Serial.read(); 

                        // CRITICAL: Call the flow-controlled, blocking upload function
                        executeUpload(filename, fileSize);
                        
                    } else {
                        pushSystemMessage("Error: UPLOAD command malformed (needs file & size).");
                        Serial.println("FATAL ERROR: UPLOAD syntax error."); 
                    }
                }
                
                // ----------------------------
                // CAT command (Receive file from Pico to PC)
                // ----------------------------
                else if (command == "CAT") {
                    if (firstSpace != -1) {
                        String filename = cmdLine.substring(firstSpace + 1);
                        filename.trim();
                        executeCat(filename); // Calls the file streaming function
                    } else {
                        pushSystemMessage("Error: CAT command requires a filename.");
                        Serial.println("ERROR: CAT requires filename.");
                    }
                }
                
                // ----------------------------
                // Other commands (e.g., LS, HELP, etc.)
                // ----------------------------
                
                else {
                    pushSystemMessage("Error: Unknown command: " + cmdLine);
                    Serial.println("ERROR: Unknown command.");
                }
            }
            continue; 
        }
        
        // 2. Collect Character
        sawCarriageReturn = false; 
        if (commandBufLen < sizeof(buffer) - 1) {
            buffer[commandBufLen++] = c;
        } else {
            pushSystemMessage("Error: Command buffer overflow.");
            Serial.println("FATAL ERROR: Command buffer overflow.");
            commandBufLen = 0;
        }
    }
}
// ----------------------------
// Setup / Loop
// ----------------------------
void setup() {
    // Setup serial for debugging (optional)
    Serial.begin(115200);
    delay(100);
    handleSerialCommands(); 
    randomSeed(analogRead(0)); // Add this line
    // TFT initialization
    tft.init(240, 240); 
    tft.setRotation(2); // Adjust as needed for your screen orientation
    tft.fillScreen(ST77XX_BLACK);
    tft.setTextWrap(false);
    tft.setFont(NULL); 
    tft.setTextSize(1);
    tft.setTextColor(ST77XX_WHITE);
    tft.setCursor(0, 0);

    // LED Initialization
    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, LOW); // Start off
    

    // Initialize button pins
    for (int i = 0; i < NUM_BUTTONS; ++i) {
        pinMode(buttonPins[i], INPUT_PULLDOWN);
        lastPressTime[i] = 0;
    }

    if (fsBegin()) { // fsBegin() should return true on success
        pushSystemMessage("LittleFS mounted.");
        fsReady = true; // Set the flag to true
    } else {
        pushSystemMessage("Warning: LittleFS mount/format failed. File commands disabled.");
        fsReady = false; // Ensure flag is false
    }

    pushSystemMessage("Welcome to PICOS!");
    pushSystemMessage("Type 'help' for commands.");
    drawFullTerminal();
    
    // Set initial history index to "new command" state
    historyIndex = historyCount; 
}
// ----------------------------
// Main loop
// ----------------------------
void loop() {
    unsigned long now = millis();

    // Cursor blinking
    if (now - lastBlink >= BLINK_MS) {
        lastBlink = now;
        cursorVisible = !cursorVisible;
        drawCursorAndPreview();
    }

    // LED timeout
    if (ledBlinkEndTime != 0 && now > ledBlinkEndTime) {
        digitalWrite(STATUS_LED_PIN, LOW);
        ledBlinkEndTime = 0;
    }

    // Button handling
    for (int i = 0; i < NUM_BUTTONS; ++i) {
        if (digitalRead(buttonPins[i]) == HIGH && (now - lastPressTime[i] > pressCooldown)) {
            lastPressTime[i] = now;
            switch (i) {
                case IDX_PREV: kbPrev(); drawCursorAndPreview(); break;
                case IDX_NEXT: kbNext(); break;
                case IDX_SELECT: kbConfirm(); break;
                case IDX_BACK: backspaceAtCursor(); break;
            }
        }
    }

    // Handle serial commands and automatic file reception
    handleSerialCommands();
}
