//#include <Adafruit_GFX.h>
//#include <Adafruit_ST7789.h>
#include <SPI.h>
#include <LittleFS.h>
#include <TFT_eSPI.h>
#include "pico/multicore.h"
#include <Adafruit_NeoPixel.h>
#include <Wire.h>

// ----------------------------
// TFT CONFIGURATION
// ----------------------------
TFT_eSPI tft = TFT_eSPI();

//#define TFT_CS    10  // KB2040 pin D10
//#define TFT_DC    9   // KB2040 pin D9
//#define TFT_RST   8   // KB2040 pin D8
//#define TFT_CS 17
//#define TFT_DC 16
//#define TFT_RST 20
#define SCREEN_WIDTH 480
#define SCREEN_HEIGHT 320
#define CHAR_WIDTH 6
#define LINE_HEIGHT 9
#define MAX_LINES (SCREEN_HEIGHT / LINE_HEIGHT)
//Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
const int COLS = SCREEN_WIDTH / CHAR_WIDTH;
const int WRAP_COLS = COLS - 1; 

// --- ADD THESE: CardKB (I2C Keyboard) ---
#define CARDKB_I2C_ADDR 0x5F
#define KB_POLL_INTERVAL_MS 50 // Poll every 50ms
unsigned long g_kb_last_poll_time = 0;
#define KEY_RIGHT_ARROW 0xB7
#define KEY_LEFT_ARROW  0xB4
#define KEY_UP_ARROW    0xB5
#define KEY_DOWN_ARROW  0xB6
#define KEY_ESCAPE      0x1B
// --- END CardKB ---

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
bool g_commandSerialError = false;
bool g_headless_mode = true; 
bool g_cli_needs_redraw = true;
#define WDT_DISABLE() wdt_disable_platform()
#define WDT_ENABLE() wdt_enable_platform()
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
//#define STATUS_LED_PIN 7 // KB2040 pin D7
#define SD_CS_PIN 21
#define TFT_BL 27
int g_bl_brightness = 255;
#define STATUS_LED_PIN 25 // <--- CHANGE THIS TO YOUR ACTUAL LED PIN
#define LED_BLINK_DURATION_MS 50 // How long the LED stays on
unsigned long ledBlinkEndTime = 0; // When to turn the LED off

// --- NEW: NEOPIXEL GLOBALS ---
#define NEOPIXEL_PIN 17 // KB2040 NeoPixel pin is D12
Adafruit_NeoPixel strip(1, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
bool g_rgb_enabled = false;
unsigned long g_rgb_last_rgb_update = 0;
uint16_t g_rgb_hue = 0; // Hue (0-65535)
// --- END NEOPIXEL ---
// ----------------------------
// BUTTONS (TTP223)
// ----------------------------
#define NUM_BUTTONS 4
const int buttonPins[NUM_BUTTONS] = {2, 3, 4, 5};
unsigned long lastPressTime[NUM_BUTTONS] = {0};
const unsigned long cooldown_nav = 200;    // Cooldown for PREV/NEXT
const unsigned long cooldown_action = 400; // Cooldown for SELECT/BACK
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
const String ctrlKeys[] = {"SPACE","ENTER","DELETE","LEFT","RIGHT", "UP", "DOWN"}; 
const String funcKeys[] = {"F1", "F2", "F3", "F4", "F5", "F6", "F7", "F8", "F9", "F10", "F11", "F12"}; 
const char* modeLabels[] = {"ALPHA", "NUM", "SYM", "CTRL"}; 
const int CTRL_COUNT = 7;
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
unsigned long timerEndTime = 0;
unsigned long lastTimerActiveBlink = 0;
const unsigned long TIMER_ACTIVE_BLINK_INTERVAL_MS = 1000;
unsigned long timerFinishedBlinkEndTime = 0; 
unsigned long lastFinishedBlinkToggle = 0; 
const unsigned long FINISHED_BLINK_INTERVAL_MS = 500;
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
// APP STATE MANAGER
// ----------------------------
enum AppState {
    APP_STATE_CLI,      // The main command line is active
    APP_STATE_NANO,     // The nano app is active
    APP_STATE_CUBE,     // The cube app is active
    APP_STATE_MOOD,     // The mood app is active
    APP_STATE_MOON,     // The moon app is active
    APP_STATE_PIC,      // The pic app is active
    APP_STATE_GEM,       // The gem app is active
    APP_STATE_PIX
};

AppState g_currentApp = APP_STATE_CLI; // We start at the CLI

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
#define ST77XX_RAINBOW_ORANGE 0xFBE0
#define ST77XX_BLUE    0x001F // Blue
#define ST77XX_INDIGO  0x4810 // Indigo 
#define ST77XX_VIOLET  0x8010 // Violet 
#define ST77XX_DARKGREY 0x18C3 // Dark grey
// ----------------------------
// RAINBOW TEXT COLOR ARRAY
// ----------------------------
const uint16_t RAINBOW_COLORS[] = {
    ST77XX_RED,          // 1. Red
    ST77XX_RAINBOW_ORANGE,// 2. Orange (Using the new bright orange)
    ST77XX_YELLOW,       // 3. Yellow
    ST77XX_GREEN,        // 4. Green
    ST77XX_CYAN,        
    ST77XX_BLUE,         // 5. Blue (New True Blue)
    ST77XX_MAGENTA       // 6. Magenta (New Magenta)
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
// NANO EDITOR STATE
// ----------------------------
#define NANO_MAX_LINES 1500      // Max lines the editor can hold (RAM limit)
#define NANO_MAX_LINE_LEN 120   // Max characters per line (adjust based on RAM/performance)
String nano_lines[NANO_MAX_LINES]; // Array to hold the text lines
int nano_lineCount = 0;           // Number of lines currently in the buffer
String nano_filename = "";        // Name of the file being edited
bool nano_isModified = false;     // Flag: true if file has unsaved changes
bool nano_exitRequested = false; // Add this near other nano_ state variables
// Cursor position *within the text buffer* (not screen position)
int nano_cursorLine = 0; // Index of the line the cursor is on
int nano_cursorCol = 0;  // Index of the character the cursor is before (0 = start)
// Viewport state (which part of the file is visible)
int nano_topLine = 0;    // Index of the first file line displayed at the top of the text area
bool isNanoActive = false;
// UI Focus state
enum NanoFocus {
    FOCUS_TEXT,     // Cursor is in the main text editing area
    FOCUS_FOOTER,   // Selection is on the bottom command bar
    FOCUS_HEADER,   // Selection is on the top status bar (optional)
    NANO_AWAIT_SAVE_CONFIRM    
};
NanoFocus nano_focus = FOCUS_TEXT; // Start focus in the text area
int nano_footerSelection = 0; // Index of the currently selected footer option (e.g., 0=Save, 1=Exit)
int nano_saveConfirmSelection = 0; // Index for Save Y/N (0=Y, 1=N)

// Screen layout constants for the editor UI (derived from existing defines)
const int NANO_HEADER_LINES = 1;      // Height of the status bar at the top
const int NANO_FOOTER_LINES = 1;      // Height of the command bar + virtual keyboard preview
const int NANO_TEXT_AREA_LINES = (SCREEN_HEIGHT / LINE_HEIGHT) - NANO_HEADER_LINES - NANO_FOOTER_LINES; // Height available for text
// ----------------------------
// 3D CUBE DEFINITIONS
// ----------------------------
struct Point3D {
    float x, y, z;
};
struct Point {
    int x, y;
    // Default constructor
    Point() : x(0), y(0) {}
    // Constructor with initial values
    Point(int _x, int _y) : x(_x), y(_y) {}
};
// --- Add these globals for the cube's persistent state ---
Point3D g_cube_vertices[8];
float g_cube_angleX = 0, g_cube_angleY = 0, g_cube_angleZ = 0;
// --- NEW: Global state for Mood app ---
float g_mood_currentHue = 0.0;
int g_mood_lastHueInt = -1;
unsigned long g_mood_lastFrameTime = 0;
// --- NEW: Global state for Moon app ---
int g_moon_currentDay = 0;
const int g_moon_totalDays = 30;
unsigned long g_cube_lastFrameTime = 0;
// =================================================================
// --- CURSOR DRAWING HELPER ---
// =================================================================

struct CursorDrawContext {
  String preview;
  uint16_t bgColor;
  uint16_t fgColor;
};

enum PixState {
    PIX_STATE_MAIN_MENU,
    PIX_STATE_RES_SELECT,
    PIX_STATE_FILE_SELECT,
    PIX_STATE_EDITOR,
    PIX_STATE_EDITOR_FOOTER,
    PIX_STATE_PALETTE
};

PixState g_pix_state = PIX_STATE_MAIN_MENU;
int g_pix_main_menu_selection = 0; // 0 = New, 1 = Edit
const char* PIX_FILE_EXTENSION = ".bmp";

const char* PIX_RESOLUTIONS[] = {
    "2x2", "4x4", "8x8", "16x16", "32x32", "64x64"
};
const int PIX_RES_COUNT = 6;
int g_pix_res_menu_selection = 0; // Index for PIX_RESOLUTIONS

// For the "Edit" screen file list
String g_pix_file_list[SCROLLBACK_SIZE]; // Re-use scrollback size
int g_pix_file_count = 0;
int g_pix_file_selection = 0;
int g_pix_file_top_index = 0;

uint16_t* g_pix_canvas = nullptr; // Pointer to our pixel data
int g_pix_canvas_width = 0;
int g_pix_canvas_height = 0;
int g_pix_cursor_x = 0;
int g_pix_cursor_y = 0;
int g_pix_palette_selection = 0; // Index for PIX_PALETTE
uint16_t g_pix_current_color = 0xFFFF; // Default to white
unsigned long g_pix_last_blink = 0;
bool g_pix_cursor_visible = true;
int g_pix_footer_selection = 0;

int g_pix_pixel_size = 0;
int g_pix_grid_x_offset = 0;
int g_pix_grid_y_offset = 0;

PixState g_pix_last_state; // To return to editor
char g_pix_filename_buffer[40];
int g_pix_filename_len = 0;


// --- REPLACE your old palette globals with this entire block ---

// Palette 1: "The one you just sent" (64-color ramp)
const uint16_t RAMP_64_PALETTE[64] = {
    // Row 1
    0x0000, 0x0841, 0x2945, 0x4A49, 0x5ACB, 0x8C71, 0xAEBA, 0xD69A,
    // Row 2
    0xFFFF, 0xFD16, 0xFBCB, 0xE800, 0xB0C0, 0x68C0, 0xFCF0, 0xFD88,
    // Row 3
    0xF280, 0xB280, 0x61E0, 0xFF76, 0xFE71, 0xFD40, 0xA420, 0x7340,
    // Row 4
    0xFFE0, 0xC720, 0x9E80, 0x6D20, 0x3B00, 0xCFE0, 0x3F60, 0x0600,
    // Row 5
    0x03E2, 0x0270, 0x7FFA, 0x078C, 0x0699, 0x04CC, 0x038B, 0x07FF,
    // Row 6
    0x0717, 0x05D6, 0x0413, 0x030D, 0x85FF, 0x3D1F, 0x3AFF, 0x001E,
    // Row 7
    0x100C, 0x9DBF, 0x7C5F, 0x615F, 0x580D, 0x380C, 0xD5FF, 0xD99F,
    // Row 8
    0xC81C, 0xA00A, 0x6808, 0xF5FF, 0xFB9F, 0xF8B1, 0xD0A9, 0x8003
};

// Palette 2: Vivid EGA 64
const uint16_t EGA_64_PALETTE[64] = {
    // Row 1
    0x0000, 0x0015, 0x0540, 0x0555, 0xA800, 0xA815, 0xAA80, 0xAD55,
    // Row 2
    0x52AA, 0x52BF, 0x57EA, 0x57FF, 0xFAAA, 0xFABF, 0xFFCA, 0xFFFF,
    // Row 3
    0x0000, 0x000A, 0x02A0, 0x02AA, 0x5000, 0x500A, 0x52A0, 0x52AA,
    // Row 4
    0x294A, 0x294F, 0x2BEA, 0x2BFF, 0x794A, 0x794F, 0x7BEA, 0x7BFF,
    // Row 5
    0x0000, 0x001F, 0x07E0, 0x07FF, 0xF800, 0xF81F, 0xFFE0, 0xFFFF,
    // Row 6
    0x0010, 0x001A, 0x0550, 0x055A, 0xA810, 0xA81A, 0xAD50, 0xAD5A,
    // Row 7
    0x52B0, 0x52BA, 0x57F0, 0x57FA, 0xFAB0, 0xFABA, 0xFFF0, 0xFFFA,
    // Row 8
    0x294A, 0x295F, 0x2BEA, 0x2BFF, 0xF800, 0xF81F, 0xFFE0, 0xFFFF
};

// Palette 3: AAP-64
const uint16_t AAP_64_PALETTE[64] = {
    // Row 1
    0x0021, 0x1082, 0x38A5, 0x70AD, 0xB105, 0xDFE4, 0xFD41, 0xFE83,
    // Row 2
    0xFEC8, 0xFFE8, 0xD78C, 0x9EC8, 0x5E06, 0x1505, 0x1BC7, 0x2297,
    // Row 3
    0x1104, 0x11B0, 0x2AF8, 0x251B, 0x2F5E, 0x2FF8, 0x2FE6, 0x2FEB,
    // Row 4
    0x9FC9, 0xFFC0, 0xF6A3, 0xF568, 0xF428, 0xED24, 0xC184, 0x8123,
    // Row 5
    0x48E3, 0x38E3, 0x59A8, 0x8A8B, 0xBBAE, 0xF4EE, 0xFDD4, 0xFFF9,
    // Row 6
    0xFF75, 0xFFCF, 0xFD6B, 0xE3E7, 0xB2A7, 0x79E6, 0x4124, 0x28E3,
    // Row 7
    0x18A2, 0x2104, 0x39E7, 0x5AEB, 0x7BEF, 0x9CF3, 0xC618, 0xFFFF,
    // Row 8
    0xE7BE, 0xBEE8, 0x8E94, 0x5DD1, 0x2CD0, 0x1BD6, 0x22B3, 0x29E8
};

// --- Master Palette Array ---
const int PIX_PALETTE_SET_COUNT = 3;
const int PIX_PALETTE_COLOR_COUNT = 64;
const uint16_t* PIX_PALETTES[PIX_PALETTE_SET_COUNT] = {
    RAMP_64_PALETTE,
    EGA_64_PALETTE,
    AAP_64_PALETTE
};
int g_pix_palette_index = 0; // 0, 1, or 2

// --- END REPLACEMENT ---
// =================================================================
// --- GEM QUEST APP - DATA MODELS ---
// =================================================================
enum QuestType {
    QT_STANDARD,
    QT_CRUSADE
};

enum QuestState {
  STATE_UNCOMPLETED, // 0
  STATE_STARTED,     // 1
  STATE_COMPLETED    // 2
};


struct Quest {
  String title;
  String description;
  QuestState questState;
  String notes;
  String skills; 
  bool isRepeatable;
  int difficulty;
  int gemReward;
  int desire;
  int xpReward;
  unsigned long timerStartTime;
  unsigned long timerDurationMinutes;
  unsigned long timerMinutesElapsed;
  unsigned long timerRemainderMs; 
  int gemBonusAwarded;
  QuestType type;
  int addCycleMinutes;
  String iconFile;
};

struct MerchantItem {
  String title;
  String description;
  int price;
  int rarity;
  int quality;
  int need;
  int desire;
  bool isConsumable;
  String iconFile; // <--- NEW: Stores "sword.bmp" etc.
};
// --- NEW: Merchant "Database" ---
#define MAX_MERCHANT_ITEMS 20
MerchantItem g_merchantDB[MAX_MERCHANT_ITEMS];
int g_merchantItemCount = 0;
bool g_gem_editor_isConsumable = false;

// --- Global Quest "Database" ---
#define MAX_QUESTS 10
Quest g_questDB[MAX_QUESTS];
int g_questCount = 0;


// --- Gem App State Management ---
enum GemAppState {
  GEM_STATE_QUEST_LIST,
  GEM_STATE_QUEST_DETAIL,
  GEM_STATE_CREATE_QUEST,
  GEM_STATE_STATS,
  GEM_STATE_TYPE_SELECT,
  GEM_STATE_MERCHANT,      
  GEM_STATE_MERCHANT_EDIT,
  GEM_STATE_INVENTORY,
  GEM_STATE_INVENTORY_VIEW,
  GEM_STATE_IMAGE_SELECT // <--- NEW STATE
};
GemAppState g_gemState = GEM_STATE_QUEST_LIST;

enum GemAppFocus {
  GEM_FOCUS_HEADER,         
  GEM_FOCUS_LIST,           
  GEM_FOCUS_DETAIL_EDITOR,  // Obsolete
  GEM_FOCUS_FOOTER,
  GEM_FOCUS_STATS_LIST,
  GEM_FOCUS_TYPE_SELECT_LIST,         

  // --- NEW STATES FOR CREATE QUEST ---
  GEM_FOCUS_CREATE_TITLE,
  GEM_FOCUS_CREATE_DESC,
  GEM_FOCUS_CREATE_SKILLS,
  GEM_FOCUS_CREATE_DIFF,
  GEM_FOCUS_CREATE_DESIRE, 
  GEM_FOCUS_CREATE_TIME,
  GEM_FOCUS_CREATE_MINS,
  GEM_FOCUS_CREATE_REPEATABLE,
  GEM_FOCUS_CREATE_ADD_CYCLE,
  GEM_FOCUS_MERCHANT_LIST,
  GEM_FOCUS_MERCHANT_CREATE_TITLE,
  GEM_FOCUS_MERCHANT_CREATE_DESC,
  GEM_FOCUS_MERCHANT_CREATE_PRICE,
  GEM_FOCUS_MERCHANT_CREATE_RARITY,
  GEM_FOCUS_MERCHANT_CREATE_QUALITY,
  GEM_FOCUS_MERCHANT_CREATE_NEED,
  GEM_FOCUS_MERCHANT_CREATE_DESIRE,
  GEM_FOCUS_MERCHANT_CREATE_CONSUMABLE,
  GEM_FOCUS_MERCHANT_CREATE_IMAGE,
  GEM_FOCUS_CREATE_IMAGE,
  GEM_FOCUS_INVENTORY_LIST
  // ---------------------------------
};
GemAppFocus g_gemFocus = GEM_FOCUS_LIST;

// --- NEW: Merchant State ---
int g_merchant_list_selection = 0;
int g_merchant_list_top_item = 0;
int g_merchant_currentItemID = -1; // Which item ID is open

#define MAX_INVENTORY_ITEMS 30
MerchantItem g_inventoryDB[MAX_INVENTORY_ITEMS];
int g_inventoryItemCount = 0;
int g_inventory_list_selection = 0;
int g_gem_list_top_item = 0;
int g_inventory_list_top_item = 0;
int g_inventory_currentItemID = -1;

int g_gem_list_selection = 0;   // Which quest is highlighted
int g_gem_stats_selection = 0;
int g_gem_stats_top_skill = 0;
int g_gem_currentQuestID = -1;    // Which quest ID is currently open
int g_gem_footer_selection = 0; // 0 = Left option, 1 = Right option
int g_gem_count = 0;            // Total gems the player has
bool g_forceCursorRedraw = false;
unsigned long g_gem_lastMinuteSave = 0; // For auto-saving timer progress
String g_gem_db_filename = "gem.db";

unsigned long g_gem_marquee_last_time = 0; // When we last scrolled
int g_gem_marquee_offset = 0;              // Current scroll position
bool g_gem_marquee_active = false;         // Is the marquee enabled?
bool g_gem_needs_bar_redraw = false;
bool g_gem_editor_isRepeatable = false;
int g_gem_level = 1;
unsigned long g_gem_total_xp = 0;

String g_gem_file_list[SCROLLBACK_SIZE]; 
int g_gem_file_count = 0;
int g_gem_file_selection = 0;
int g_gem_file_top_index = 0;
String g_gem_temp_icon_file = "";
String g_gem_temp_quest_icon = "";

// --- NEW: Global Skill Database (for Autocomplete) ---
#define MAX_SKILLS 100 // Max unique skill names
String g_skillDB[MAX_SKILLS];
int g_skillCount = 0;
// (g_skill_db_filename removed)

// --- NEW: Player's Personal Skill XP & Levels ---
struct PlayerSkill {
  String name;
  int level;
  unsigned long xp;
};
#define MAX_PLAYER_SKILLS 50 // Max skills a player can learn
PlayerSkill g_playerSkills[MAX_PLAYER_SKILLS];
int g_playerSkillCount = 0;
#define MAX_AUTOCOMPLETE 5 // Max suggestions to show
bool g_gem_autocomplete_active = false;
String g_gem_autocomplete_suggestions[MAX_AUTOCOMPLETE];
int g_gem_autocomplete_count = 0;
int g_gem_autocomplete_selection = 0;
int g_gem_autocomplete_word_start_col = 0;
int g_gem_autocomplete_word_start_line = 0;
int g_gem_last_ac_x = 0;
int g_gem_last_ac_y = 0;
int g_gem_last_ac_w = 0;
int g_gem_last_ac_h = 0;


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
bool executeCommandLine(const String &raw);
String trimStr(const String &s);
String evalCalc(const String &expr);
void tokenizeLine(const String &line, String tokens[], int &count, int maxTokens);
void runNanoEditor(String filename);
void nano_handleInput();
bool fsBegin();
String listFiles();
String readFile(const String &path);
bool writeFile(const String &path, const String &data, bool append); 
bool removeFile(const String &path);
String findFileCaseInsensitive(const String& filename);
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
void cube_start();
void cube_stop();
void cube_update();
void mood_start();
void mood_stop();
void mood_update();
void moon_start();
void moon_stop();
void moon_update();
void moon_handleInput(int buttonIndex);
void pic_start(const String& filename);
void pic_stop();
void pic_update();
void pix_start();
void pix_stop();
void pix_update(unsigned long now);
void pix_handleInput(int buttonIndex);
void pix_handleKey(uint8_t c);
void pix_drawMainMenu();
void pix_drawResSelect();
void pix_buildFileList(); 
void pix_drawFileSelect();
void pix_drawEditorFooter();
void pix_drawPixel(int x, int y);
void pix_drawCursor(int x, int y, bool isVisible);
void pix_drawPaletteCursor(int index, bool isVisible);
void pix_drawMainMenuItem(int index); // <-- ADD THIS
void pix_drawResSelectItem(int index); // <-- ADD THIS
void pix_drawFileSelectItem(int index);
bool pix_loadBMP(String filename);
bool pix_saveBMP(String filename);
void pix_drawPaletteBackground();
void write16_le(File &f, uint16_t val);
void write32_le(File &f, uint32_t val);
void pix_drawSavePrompt();
void nano_start(String filename); 
void nano_stop();              
void nano_update(unsigned long now); 
void nano_handleInput(int buttonIndex);
void drawMoon(int day, int totalDays);
void drawStars(); // Add this prototype
bool nano_insertChar_DataWorker(char c);
void splitAndWrapFile(const String& inputFilename);
void executeFormatConfirmation(bool didConfirm);
void gem_open_quest_detail(int questDB_Index);
String gem_escape_notes(String s);
String gem_unescape_notes(String s);
void gem_drawAllProgressBars();
void gem_drawListItem(int item_selection_index, bool drawBar = true);
void gem_deleteQuest(int db_index);
String gem_deleteSkill(int skill_index);
void gem_create_default_quests();
unsigned long gem_getXPForLevel(int level);
void gem_awardXP(int xp);
void gem_recalculateLevel();
void gem_add_skills_to_database(String skillsString);
void gem_award_skill_xp(String skillName, int xp);
void gem_recalculate_skill_level(PlayerSkill* skill, bool checkLevelUp);
unsigned long gem_getXPForSkillLevel(int level);
unsigned long gem_getCostForLevel(int level);
void gem_draw_autocomplete();
void gem_hide_autocomplete(bool needsRedraw);
void gem_drawStatisticsPage();
void gem_drawStatsList();
void gem_drawStatsItem(int skill_index);
void gem_drawTypeSelectList();
void gem_drawTypeSelectItem(int item_index, bool isSelected);
void gem_drawMerchantList();
void gem_drawMerchantItem(int item_index);
void gem_drawMerchantEditor(bool preserveData = false);
void gem_create_default_items();
void addMerchantItem(String title, String desc, int price);
void gem_deleteMerchantItem(int db_index);
void gem_refreshMerchantEditorView();
void gem_returnToMerchantList();
void gem_drawInventoryList();
void gem_drawInventoryItem(int item_index);
void gem_drawInventoryView();
void addInventoryItem(String title, String desc, int price);
void gem_deleteInventoryItem(int db_index);
int gem_getQuestItemHeight(int index);
int gem_getQuestItemY(int index);

enum QuestType; // Forward declaration
void addQuest(String title, String desc, String notes, String skills, bool isRepeatable, int diff, int desire, int timeEstimateMins, QuestType type, int addCycle, String icon);
uint16_t read16(File &f);
uint32_t read32(File &f);
Point3D rotateX(Point3D p, float angle);
Point3D rotateY(Point3D p, float angle);
Point3D rotateZ(Point3D p, float angle);
Point project(Point3D p);
struct CursorDrawContext; // Forward declaration of the struct
CursorDrawContext getCursorDrawContext(bool cursorVisibleState);
uint16_t hsvToRgb565(int hue, uint8_t sat, uint8_t val);

struct TextArea {
    // --- 1. Geometry & Size ---
    int x, y;
    int width, height;
    int numCols, numRows;

    // --- 2. Data Buffer ---
    String* lines;
    int lineCount;
    int maxLines;
    int maxLineLen;

    // --- 3. Viewport & Cursor State ---
    int topLine;
    int cursorLine;
    int cursorCol;

    // --- 4. State Flags ---
    bool isModified;
    bool isFocused;
    bool cursorVisible;

    // --- 5. Blink & Cursor Draw Tracking ---
    unsigned long lastBlinkTime;
    int _lastCursorScreenX;
    int _lastCursorScreenY;
    int _lastPreviewWidth;
    bool _wasCursorVisibleLast;

    /**
     * @brief Constructor for the text area.
     */
    TextArea(int x_pos, int y_pos, int w_chars, int h_chars, int buf_max_lines, int buf_max_line_len)
        : x(x_pos), y(y_pos), numCols(w_chars), numRows(h_chars),
          maxLines(buf_max_lines), maxLineLen(buf_max_line_len) 
    {
        width = numCols * CHAR_WIDTH;
        height = numRows * LINE_HEIGHT;
        lines = new String[maxLines]; // Allocate buffer
        lineCount = 1;
        lines[0] = "";
        topLine = 0;
        cursorLine = 0;
        cursorCol = 0;
        isModified = false;
        isFocused = false;
        cursorVisible = true;
        lastBlinkTime = 0;
        _lastCursorScreenX = -1;
        _lastCursorScreenY = -1;
        _lastPreviewWidth = 0;
        _wasCursorVisibleLast = false;
    }

    /**
     * @brief Destructor to free the line buffer.
     */
    ~TextArea() {
        delete[] lines;
    }

    // =================================================================
    // --- DRAWING METHODS ("View") ---
    // =================================S================================

    /**
     * @brief Draws the entire text area.
     */
    void draw(TFT_eSPI& tft) {
        tft.fillRect(x, y, width, height, ST77XX_BLACK);
        tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);

        for (int screenLine = 0; screenLine < numRows; ++screenLine) {
            int fileLineIndex = topLine + screenLine;
            if (fileLineIndex < lineCount) {
                String lineContent = lines[fileLineIndex];
                String displayString = lineContent.substring(0, numCols);
                tft.setCursor(x, y + screenLine * LINE_HEIGHT);
                tft.print(displayString);
            }
        }
    }
    
    /**
     * @brief (NEW) Redraws text from the cursor to the end of the line.
     * Ported from nano_redrawTrailingText.
     */
    void redrawTrailingText(TFT_eSPI& tft) {
        int screenLine = cursorLine - topLine;
        if (screenLine < 0 || screenLine >= numRows) {
            return; // Not visible
        }
        int cursorScreenX = (cursorCol * CHAR_WIDTH) + x;
        int cursorScreenY = (screenLine * LINE_HEIGHT) + y;

        // Clear from cursor to right edge of component
        tft.fillRect(cursorScreenX, cursorScreenY, (x + width) - cursorScreenX, LINE_HEIGHT, ST77XX_BLACK);

        // Redraw trailing text
        String currentLine = lines[cursorLine];
        if (cursorCol < currentLine.length()) {
            String trailingText = currentLine.substring(cursorCol, cursorCol + numCols - cursorCol);
            tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
            tft.setCursor(cursorScreenX, cursorScreenY);
            tft.print(trailingText);
        }
    }

    /**
     * @brief Draws the cursor/preview over the text.
     */
    void drawCursor(TFT_eSPI& tft, const String& preview, uint16_t bgColor, uint16_t fgColor) {
        
        int screenLine = cursorLine - topLine;
        int screenCol = cursorCol;
        int screenPixelX = (screenCol * CHAR_WIDTH) + x;
        int screenPixelY = (screenLine * LINE_HEIGHT) + y;
        
        if (_lastCursorScreenX != -1) {
            int prevDrawX = _lastCursorScreenX;
            int prevDrawY = _lastCursorScreenY;

            for (int i = 0; i < _lastPreviewWidth; ++i) {
                if (prevDrawX >= (x + width)) {
                    prevDrawX = x;
                    prevDrawY += LINE_HEIGHT;
                }
                
                int prevLineIndex = ((prevDrawY - y) / LINE_HEIGHT) + topLine;
                String prevLineText = "";
                if (prevLineIndex >= 0 && prevLineIndex < lineCount) {
                    prevLineText = lines[prevLineIndex];
                }
                int prevCol = (prevDrawX - x) / CHAR_WIDTH;

                tft.fillRect(prevDrawX, prevDrawY, CHAR_WIDTH, LINE_HEIGHT, ST77XX_BLACK);
                if (prevCol < prevLineText.length()) {
                    tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
                    tft.setCursor(prevDrawX, prevDrawY);
                    tft.print(prevLineText.charAt(prevCol));
                }
                prevDrawX += CHAR_WIDTH;
            }
        }

        if (screenLine < 0 || screenLine >= numRows || !isFocused) {
            _lastCursorScreenX = -1;
            return;
        }

        int previewWidth = max(1, (int)preview.length());
        int currentDrawX = screenPixelX;
        int currentDrawY = screenPixelY;

        for (int i = 0; i < previewWidth; ++i) {
            if (currentDrawX >= (x + width)) {
                currentDrawX = x;
                currentDrawY += LINE_HEIGHT;
            }
            if (currentDrawY >= (y + height)) {
                break;
            }
            
            tft.fillRect(currentDrawX, currentDrawY, CHAR_WIDTH, LINE_HEIGHT, bgColor);
            tft.setTextColor(fgColor, bgColor);
            tft.setCursor(currentDrawX, currentDrawY);
            tft.print(preview.charAt(i));
            currentDrawX += CHAR_WIDTH;
        }

        _lastCursorScreenX = screenPixelX;
        _lastCursorScreenY = screenPixelY;
        _lastPreviewWidth = previewWidth;
        _wasCursorVisibleLast = cursorVisible;
    }

    /**
     * @brief Handles the blink logic.
     */
    void updateBlink(TFT_eSPI& tft, unsigned long now, const String& preview, uint16_t modeColor) {
        // This is handled by the main gem_update loop
    }

    // =================================================================
    // --- "SMART" DATA & DRAWING METHODS ("Controller") ---
    // --- Return true = full redraw needed ---
    // =================================================================

    /**
     * @brief Inserts a character and redraws surgically.
     * @return bool True if a full UI redraw is needed (header change).
     */
    bool insertChar(TFT_eSPI& tft, char c) {
        bool wasMod = isModified;

        int screenLine = cursorLine - topLine;
        int cursorScreenX = (cursorCol * CHAR_WIDTH) + x;
        int cursorScreenY = (screenLine * LINE_HEIGHT) + y;
        
        bool wrapOccurred = _data_worker_insertChar(c);

        if (!wrapOccurred && screenLine >= 0 && screenLine < numRows) {
            // Flicker-free draw
            tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
            tft.setCursor(cursorScreenX, cursorScreenY);
            tft.print(c);
            redrawTrailingText(tft);
        }

        // Return true if first mod (for header) or a wrap occurred
        return (!wasMod && isModified) || wrapOccurred;
    }

    /**
     * @brief Performs backspace and redraws surgically.
     * @return bool True if a full UI redraw is needed.
     */
    bool backspace(TFT_eSPI& tft) {
        if (cursorLine == 0 && cursorCol == 0) return false;
        
        bool wasModified = isModified;
        String &currentLine = lines[cursorLine];
        bool isLastLine = (cursorLine == lineCount - 1);
        bool lineNotFull = (currentLine.length() < numCols);
        
        // --- FAST PATH ---
        if (cursorCol > 0 && (lineNotFull || isLastLine)) {
            currentLine = currentLine.substring(0, cursorCol - 1) + currentLine.substring(cursorCol);
            cursorCol--;
            isModified = true;
            redrawTrailingText(tft);
            return (!wasModified && isModified); // Redraw header if first mod
        }

        // --- FULL REFLOW or MERGE LOGIC ---
        bool runReflowLoop = false;
        if (cursorCol > 0) {
            currentLine = currentLine.substring(0, cursorCol - 1) + currentLine.substring(cursorCol);
            cursorCol--;
            isModified = true;
            runReflowLoop = true;
        } else {
            int prevLineIndex = cursorLine - 1;
            String &prevLine = lines[prevLineIndex];
            String &currLine = lines[cursorLine];

            if (prevLine.length() + currLine.length() > maxLineLen) return false;
            bool prevLineWasEmpty = (prevLine.length() == 0);
            cursorLine--;
            cursorCol = prevLine.length();
            isModified = true;
            
            prevLine += currLine;
            for (int j = cursorLine + 1; j < lineCount - 1; j++) lines[j] = lines[j + 1];
            lineCount--;
            lines[lineCount] = "";
            runReflowLoop = !prevLineWasEmpty;
        }

        if (runReflowLoop) {
            for (int i = cursorLine; i < lineCount - 1; i++) {
                String &lineAbove = lines[i];
                String &lineBelow = lines[i + 1];
                int spaceLeft = numCols - lineAbove.length();
                if (spaceLeft <= 0) break;
                int pullUpLen = min(spaceLeft, lineBelow.length());
                if (pullUpLen > 0) {
                    lineAbove += lineBelow.substring(0, pullUpLen);
                    lineBelow = lineBelow.substring(pullUpLen);
                }
                if (lineBelow.length() == 0) {
                    for (int j = i + 1; j < lineCount - 1; j++) lines[j] = lines[j + 1];
                    lineCount--;
                    lines[lineCount] = "";
                    i--; // Re-process
                }
            }
        }
        _scrollToCursor();
        return true; // Major change, full redraw needed
    }

    /**
     * @brief Inserts a new line.
     * @return bool True (always a major change).
     */
    bool insertLine(TFT_eSPI& tft) {
        if (lineCount >= maxLines) return false;
        if (cursorLine < 0 || cursorLine >= lineCount) return false;

        String &currentLine = lines[cursorLine];
        String textAfterCursor = currentLine.substring(cursorCol);
        currentLine = currentLine.substring(0, cursorCol);

        for (int i = lineCount; i > cursorLine + 1; --i) lines[i] = lines[i - 1];
        lines[cursorLine + 1] = textAfterCursor;
        lineCount++;
        cursorLine++;
        cursorCol = 0;
        isModified = true;
        _scrollToCursor();
        return true; // Major change, always needs full redraw
    }
    /**
     * @brief Moves the cursor and updates the viewport.
     * @return bool True if a scroll (page up/down) occurred.
     */
    bool moveCursor(int dx, int dy) {
        bool scrolled = false;
        
        // --- THIS IS THE CORRECTED LOGIC (from nano_moveCursor) ---
        if (dy > 0) { // Moving Down
            int bottomVisibleLine = topLine + numRows - 1;
            // Check if cursor is already at the bottom edge
            if (cursorLine >= bottomVisibleLine) {
                // --- Page Down ---
                int potentialNewTop = topLine + numRows;
                topLine = min(potentialNewTop, lineCount - numRows);
                if (topLine < 0) topLine = 0; 
                cursorLine = min(cursorLine + numRows, lineCount - 1);
                scrolled = true;
            } else {
                // --- Normal Line Down ---
                cursorLine = min(cursorLine + dy, lineCount - 1);
            }
        } else if (dy < 0) { // Moving Up
            // Check if cursor is already at the top edge
            if (cursorLine <= topLine) {
                // --- Page Up ---
                int potentialNewTop = topLine - numRows;
                topLine = max(0, potentialNewTop); // Clamp top line at 0
                cursorLine = max(0, cursorLine - numRows);
                scrolled = true;
            } else {
                // --- Normal Line Up ---
                cursorLine = max(0, cursorLine + dy);
            }
        }
        // --- END OF CORRECTED LOGIC ---

        if (dx != 0) {
            cursorCol = max(0, cursorCol + dx);
            int len = (cursorLine < lineCount) ? lines[cursorLine].length() : 0;
            cursorCol = min(cursorCol, len);
        }
        if (dy != 0) { // Vertical move, clamp column
            int len = (cursorLine < lineCount) ?
            lines[cursorLine].length() : 0;
            cursorCol = min(cursorCol, len);
        }
        
        return scrolled;
    }

    // ... (File I/O methods remain unchanged) ...
    bool loadFile(const String& filename) {
        lineCount = 0;
        String actualFilename = findFileCaseInsensitive(filename);
        if (!LittleFS.exists(actualFilename)) {
            lineCount = 1;
            lines[0] = "";
            return false;
        }
        File file = LittleFS.open(actualFilename, "r");
        if (!file) {
            lineCount = 1; lines[0] = "";
            return false;
        }
        while (file.available() && lineCount < maxLines) {
            String original_line = file.readStringUntil('\n');
            if (original_line.endsWith("\r")) {
                original_line.remove(original_line.length() - 1);
            }
            String cleaned_line = "";
            int len = original_line.length();
            for (int i = 0; i < len; ++i) {
                 unsigned char c1 = original_line.charAt(i);
                 if (c1 < 0x80) { cleaned_line += (char)c1; }
                 else if ((c1 & 0xE0) == 0xC0) { // 2-byte
                     if(i+1 < len){ unsigned char c2=original_line.charAt(i+1);
 if((c2 & 0xC0)==0x80){ uint16_t cp=((c1 & 0x1F)<<6)|(c2 & 0x3F); switch(cp){ case 0xA3: cleaned_line+='?'; break; case 0xA9: cleaned_line+="(c)"; break; default: cleaned_line+='?';
 break; } i++; } else { cleaned_line+='?';} } else { cleaned_line+='?';}
                 } else if ((c1 & 0xF0) == 0xE0) { // 3-byte
                     if(i+2 < len){ unsigned char c2=original_line.charAt(i+1);
 unsigned char c3=original_line.charAt(i+2); if(((c2 & 0xC0)==0x80)&&((c3 & 0xC0)==0x80)){ uint32_t cp=((c1 & 0x0F)<<12)|((c2 & 0x3F)<<6)|(c3 & 0x3F); switch(cp){ case 0x2018: cleaned_line+='\'';
 break; case 0x2019: cleaned_line+='\''; break; case 0x201C: cleaned_line+='"'; break; case 0x201D: cleaned_line+='"'; break; case 0x2013: cleaned_line+='-'; break; case 0x2014: cleaned_line+="--";
 break; case 0x2026: cleaned_line+="..."; break; case 0x20AC: cleaned_line+='?'; break; default: cleaned_line+='?'; break; } i+=2;
 } else { cleaned_line+='?';} } else { cleaned_line+='?';}
                 } else if ((c1 & 0xF8) == 0xF0) { // 4-byte
                     if(i+3 < len){ unsigned char c2=original_line.charAt(i+1);
 unsigned char c3=original_line.charAt(i+2); unsigned char c4=original_line.charAt(i+3); if(((c2&0xC0)==0x80)&&((c3&0xC0)==0x80)&&((c4&0xC0)==0x80)){ cleaned_line+='?'; i+=3; } else { cleaned_line+='?';} } else { cleaned_line+='?';}
                 } else { cleaned_line += '?';
 }
            }
            String line_to_wrap = cleaned_line;
            do {
                if (lineCount >= maxLines) break;
                if (line_to_wrap.length() > numCols) {
                    int wrapPoint = -1;
                    for (int i = numCols; i > 0; i--) {
                        if (line_to_wrap.charAt(i) == ' ') {
                            wrapPoint = i;
                            break;
                        }
                    }
                    if (wrapPoint > 0) {
                        lines[lineCount++] = line_to_wrap.substring(0, wrapPoint);
                        line_to_wrap = line_to_wrap.substring(wrapPoint + 1); 
                    } else {
                        lines[lineCount++] = line_to_wrap.substring(0, numCols);
                        line_to_wrap = line_to_wrap.substring(numCols);
                    }
                } else {
                    lines[lineCount++] = line_to_wrap;
                    line_to_wrap = "";
                }
            } while (line_to_wrap.length() > 0 && lineCount < maxLines);
        }
        if (file.available()) {
            pushSystemMessage("Warning: File exceeds max lines. Truncated.");
        }
        file.close();
        if (lineCount == 0) {
            lineCount = 1;
            lines[0] = "";
        }
        isModified = false;
        return true;
    }

    bool saveFile(const String& filename) {
        File file = LittleFS.open(filename, "w");
        if (!file) return false;
        for (int i = 0; i < lineCount; ++i) {
            if (file.println(lines[i]) == 0) {
                file.close();
                return false;
            }
        }
        file.close();
        isModified = false;
        return true;
    }

    // =================================================================
    // --- "PRIVATE" HELPER METHODS ---
    // =================================================================

    bool _data_worker_insertChar(char c) {
        bool didWrap = false;
        if (cursorLine < 0 || cursorLine >= lineCount) return false;
        String &currentLine = lines[cursorLine];
        if (currentLine.length() >= maxLineLen) return false;
        if (cursorCol == currentLine.length()) currentLine += c;
        else currentLine = currentLine.substring(0, cursorCol) + c + currentLine.substring(cursorCol);
        cursorCol++;
        isModified = true;
        if (currentLine.length() > numCols) {
            didWrap = true;
            for (int i = cursorLine; i < lineCount; i++) {
                String &line = lines[i];
                if (line.length() <= numCols) break;
                String overflowText = line.substring(numCols);
                line = line.substring(0, numCols);
                if (i + 1 >= maxLines) break;
                if (i + 1 == lineCount) {
                    if (lineCount < maxLines) {
                        lineCount++;
                        lines[i + 1] = overflowText;
                    } else break;
                } else {
                    String &nextLine = lines[i + 1];
                    nextLine = overflowText + nextLine;
                    if (nextLine.length() > maxLineLen) {
                        nextLine = nextLine.substring(0, maxLineLen);
                    }
                }
            }
            if (cursorCol > numCols) {
                cursorLine++;
                cursorCol = cursorCol - numCols;
                _scrollToCursor();
            }
        }
        return didWrap;
    }
    
    void _scrollToCursor() {
        if (cursorLine < topLine) {
            topLine = cursorLine;
        }
        if (cursorLine >= topLine + numRows) {
            topLine = cursorLine - numRows + 1;
        }
    }
};

// --- PASTE THE POINTERS *AFTER* THE DATA MODELS ---
TextArea* currentEditor = nullptr; 
TextArea* gemQuestNotesEditor = nullptr; // For the gem app

// --- NEW: Pointers for the Create Quest view ---
TextArea* gemCreateTitleEditor = nullptr;
TextArea* gemCreateDescEditor = nullptr;
TextArea* gemCreateDiffEditor = nullptr;
TextArea* gemCreateDesireEditor = nullptr;
TextArea* gemCreateTimeEditor = nullptr;
TextArea* gemCreateMinsEditor = nullptr;
TextArea* gemCreateSkillsEditor = nullptr;
TextArea* gemCreateAddCycleEditor = nullptr;
TextArea* gemCreateItemTitleEditor = nullptr;
TextArea* gemCreateItemDescEditor = nullptr;
TextArea* gemCreateItemPriceEditor = nullptr;
TextArea* gemCreateItemRarityEditor = nullptr;
TextArea* gemCreateItemQualityEditor = nullptr;
TextArea* gemCreateItemNeedEditor = nullptr;
TextArea* gemCreateItemDesireEditor = nullptr;
QuestType g_gem_editor_questType = QT_STANDARD;
int g_gem_type_select_selection = 0;
int g_gem_type_select_top_item = 0;
const String g_quest_type_names[] = {
    "Standard Quest",
    "Crusade (Progressive Timer)"
    // Add new quest types here
};
const int g_quest_type_count = 2;
// ---------------------------------------------

String gem_join_notes_from_editor(TextArea* editor);
void gem_load_notes_to_editor(TextArea* editor, String notes_string);
void gem_update_autocomplete(TextArea* editor);
void gem_select_autocomplete(TextArea* editor);

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
uint16_t read16(File &f) {
  uint16_t result;
  uint8_t buffer[2];
  if (f.read(buffer, 2) == 2) {
    result = buffer[0] | (buffer[1] << 8);
    return result;
  }
  return 0; // Error case
}
void nano_drawUI() {
    // Clear potentially unused areas (optional, but can help prevent artifacts)
    // tft.fillScreen(ST77XX_BLACK); // Or clear specific regions if needed

    nano_drawHeader();     // Draw the top status bar
    nano_drawTextArea();   // Draw the main text content
    nano_drawFooter();     // Draw the bottom command bar/keyboard preview
    // The cursor is drawn separately during the blink cycle or after input
}
void nano_start(String filename) {
    tft.startWrite();
    tft.fillScreen(ST77XX_BLACK);
    tft.endWrite();

    // --- SETUP ---
    g_currentApp = APP_STATE_NANO; // Set global state
    nano_filename = filename;
    nano_isModified = false;
    nano_cursorLine = 0;
    nano_cursorCol = 0;
    nano_topLine = 0;
    nano_focus = FOCUS_TEXT;
    nano_footerSelection = 0;
    nano_lineCount = 0;
    nano_exitRequested = false; 
    isNanoActive = true;
    kmode = ALPHA;
    kbIndex = 0;

    // Attempt to load the file
    if (!nano_loadFile(filename)) {
        nano_lineCount = 1;
        nano_lines[0] = "";
    }

    // Perform the initial full draw
    tft.startWrite();
    nano_drawUI();
    tft.endWrite();
}
void nano_stop() {
    g_currentApp = APP_STATE_CLI;
    isNanoActive = false;
    
    tft.startWrite();
    tft.fillScreen(ST77XX_BLACK);
    tft.endWrite();
    
    invalidateTerminalCache();
    drawFullTerminal();
    clearCurrentCommand();
}
void nano_update(unsigned long now) {
    // --- Handle Cursor/Focus Blinking ---
    if (now - lastBlink >= BLINK_MS) {
        lastBlink = now;
        cursorVisible = !cursorVisible; // Toggle blink state

        tft.startWrite();
        if (nano_focus == FOCUS_TEXT) {
            nano_drawEditorCursor();
        } else if (nano_focus == FOCUS_FOOTER || nano_focus == NANO_AWAIT_SAVE_CONFIRM) {
            nano_drawFooter();
        } else if (nano_focus == FOCUS_HEADER) {
            nano_drawHeader();
        }
        tft.endWrite();
    }
    yield();
}
void nano_drawHeader() {
    int y = 0; // Header is at the top row

    // --- 1. Calculate Page Numbers (Unchanged) ---
    int currentPage = 1;
    int totalPages = 1;
    if (NANO_TEXT_AREA_LINES > 0) {
        totalPages = (nano_lineCount + NANO_TEXT_AREA_LINES - 1) / NANO_TEXT_AREA_LINES;
        if (totalPages == 0) totalPages = 1;
        currentPage = (nano_topLine / NANO_TEXT_AREA_LINES) + 1;
        if (currentPage > totalPages) currentPage = totalPages;
    }

    // --- 2. Format Header Text Parts ---
    String pageInfo = "P. " + String(currentPage) + "/" + String(totalPages);
    String fileLabel = " File:"; // <-- SEPARATE LABEL
    // Filename part *without* the label initially
    String fileNamePart = " " + nano_filename;
    if (nano_isModified) {
        fileNamePart += " *";
    }

    // Calculate available space for file info (label + name + *)
    int availableWidthForFile = COLS - pageInfo.length() - fileLabel.length();

    // Truncate fileNamePart if necessary
    if (fileNamePart.length() > availableWidthForFile) {
        // Keep beginning, add "...", keep modification marker if needed
        String modMarker = nano_isModified ? " *" : "";
        int keepChars = availableWidthForFile - 3 - modMarker.length();
        if (keepChars < 1) keepChars = 1; // Ensure at least one char before "..."
        fileNamePart = fileNamePart.substring(0, keepChars) + "..." + modMarker;
    }

    // Calculate padding spaces needed
    String padding = "";
    int currentLength = fileLabel.length() + fileNamePart.length() + pageInfo.length();
    while (currentLength < COLS) {
        padding += " ";
        currentLength++;
    }

    // --- 3. Set Colors based on Focus (Unchanged) ---
    uint16_t bgColor = ST77XX_BLACK;
    uint16_t defaultFgColor = ST77XX_WHITE; // Default foreground
    uint16_t labelColor = ST77XX_GREEN;     // Specific color for the label

    if (nano_focus == FOCUS_HEADER && cursorVisible) {
        bgColor = ST77XX_WHITE;
        defaultFgColor = ST77XX_BLACK;
        // Keep label green even when inverted? Or make it black too?
        // Let's make it black when inverted for consistency.
        labelColor = ST77XX_BLACK;
    }

    // --- 4. Draw Background and Text in Parts ---
    tft.fillRect(0, y, SCREEN_WIDTH, LINE_HEIGHT * NANO_HEADER_LINES, bgColor); // Clear header

    // Part 1: Draw the " File:" label in green
    tft.setCursor(0, y);
    tft.setTextColor(labelColor, bgColor);
    tft.print(fileLabel);

    // Part 2: Draw the filename part in the default color
    tft.setTextColor(defaultFgColor, bgColor); // Switch to default FG
    tft.print(fileNamePart);

    // Part 3: Draw the padding spaces
    tft.print(padding);

    // Part 4: Draw the page info
    tft.print(pageInfo);
}
void nano_drawTextArea() {
    int startY = NANO_HEADER_LINES * LINE_HEIGHT; // Y position after the header
    int endY = startY + NANO_TEXT_AREA_LINES * LINE_HEIGHT;

    // Clear the entire text area first
    tft.fillRect(0, startY, SCREEN_WIDTH, NANO_TEXT_AREA_LINES * LINE_HEIGHT, ST77XX_BLACK);

    tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK); // Default text color

    // Loop through the screen lines allocated for the text area
    for (int screenLine = 0; screenLine < NANO_TEXT_AREA_LINES; ++screenLine) {
        int fileLineIndex = nano_topLine + screenLine; // Calculate the index in the nano_lines buffer

        // Check if this line index is valid within the loaded file content
        if (fileLineIndex < nano_lineCount) {
            String lineContent = nano_lines[fileLineIndex];
            // TODO: Add horizontal scrolling logic here if needed
            // For now, just display the start of the line
            tft.setCursor(0, startY + screenLine * LINE_HEIGHT);
            tft.print(lineContent.substring(0, COLS)); // Print chars that fit on screen
        } else {
            // Optionally draw a character like '~' for lines beyond the end of the file
            // tft.setCursor(0, startY + screenLine * LINE_HEIGHT);
            // tft.print("~");
        }
    }
}
void nano_drawFooter() {
    int textOffsetY = 1; // <-- NEW: Vertical offset for text
    int startY = SCREEN_HEIGHT - (NANO_FOOTER_LINES * LINE_HEIGHT); // Y position for the blue bar

    // 1. Clear the entire footer area first (unchanged)
    tft.fillRect(0, startY, SCREEN_WIDTH, NANO_FOOTER_LINES * LINE_HEIGHT, ST77XX_BLUE);

    // --- Draw based on current focus ---

    if (nano_focus == NANO_AWAIT_SAVE_CONFIRM) {
        // --- Draw Save Confirmation Prompt ---
        String promptText = " Save modified buffer? (Y/N)";
        tft.setTextColor(ST77XX_YELLOW, ST77XX_BLUE); // Prompt color over blue BG

        // --- MODIFIED: Added textOffsetY ---
        tft.setCursor(0, startY + textOffsetY);
        tft.print(promptText);

        int ynX = promptText.length() * CHAR_WIDTH;
        // --- MODIFIED: Added textOffsetY ---
        int ynY = startY + textOffsetY;

        // Draw 'Y' (index 0)
        uint16_t bgY = (nano_saveConfirmSelection == 0 && cursorVisible) ? ST77XX_GREEN : ST77XX_BLUE;
        uint16_t fgY = (nano_saveConfirmSelection == 0 && cursorVisible) ? ST77XX_BLUE : ST77XX_GREEN;
        // --- MODIFIED: Need to adjust fillRect for highlight position ---
        tft.fillRect(ynX, startY, CHAR_WIDTH, LINE_HEIGHT, bgY); // Fill background behind Y
        tft.setCursor(ynX, ynY);
        tft.setTextColor(fgY, bgY);
        tft.print("Y");

        int separatorX = tft.getCursorX(); // Get X after printing Y
        tft.setTextColor(ST77XX_WHITE, ST77XX_BLUE); // Separator color over blue
        // --- MODIFIED: Added textOffsetY ---
        tft.setCursor(separatorX, startY + textOffsetY);
        tft.print("/");

        int nX = tft.getCursorX(); // Get X after printing /
        // Draw 'N' (index 1)
        uint16_t bgN = (nano_saveConfirmSelection == 1 && cursorVisible) ? ST77XX_RED : ST77XX_BLUE;
        uint16_t fgN = (nano_saveConfirmSelection == 1 && cursorVisible) ? ST77XX_BLUE : ST77XX_RED;
        // --- MODIFIED: Need to adjust fillRect for highlight position ---
        tft.fillRect(nX, startY, CHAR_WIDTH, LINE_HEIGHT, bgN); // Fill background behind N
        tft.setCursor(nX, ynY);
        tft.setTextColor(fgN, bgN);
        tft.print("N");

        // Clear rest of the line
        int endX = tft.getCursorX();
        // --- MODIFIED: Clear from original startY ---
        tft.fillRect(endX, startY, SCREEN_WIDTH - endX, LINE_HEIGHT, ST77XX_BLUE);

    } else {
        // --- Draw Standard Command Options (Save/Exit) ---
        String opt1 = " Save";
        String opt2 = " Exit";
        int halfWidth = SCREEN_WIDTH / 2;

        // Determine colors based on focus
        uint16_t bg1 = (nano_focus == FOCUS_FOOTER && nano_footerSelection == 0 && cursorVisible) ? ST77XX_WHITE : ST77XX_BLUE;
        uint16_t fg1 = (nano_focus == FOCUS_FOOTER && nano_footerSelection == 0 && cursorVisible) ? ST77XX_BLUE : ST77XX_WHITE;
        uint16_t bg2 = (nano_focus == FOCUS_FOOTER && nano_footerSelection == 1 && cursorVisible) ? ST77XX_WHITE : ST77XX_BLUE;
        uint16_t fg2 = (nano_focus == FOCUS_FOOTER && nano_footerSelection == 1 && cursorVisible) ? ST77XX_BLUE : ST77XX_WHITE;

        // Draw Save option
        int opt1EndX = opt1.length() * CHAR_WIDTH;
        // --- MODIFIED: Fill background first, then draw text offset ---
        tft.fillRect(0, startY, halfWidth, LINE_HEIGHT, bg1); // Fill Save section BG
        tft.setTextColor(fg1, bg1);
        tft.setCursor(0, startY + textOffsetY); // <-- Add offset
        tft.print(opt1);

        // Draw Exit option
        int opt2StartX = halfWidth;
        // --- MODIFIED: Fill background first, then draw text offset ---
        tft.fillRect(opt2StartX, startY, SCREEN_WIDTH - opt2StartX, LINE_HEIGHT, bg2); // Fill Exit section BG
        tft.setTextColor(fg2, bg2);
        tft.setCursor(halfWidth, startY + textOffsetY); // <-- Add offset
        tft.print(opt2);
    }
}
void nano_drawEditorCursor() {
    // --- Static variables to track the PREVIOUS cursor state ---
    static int nano_lastCursorScreenX = -1;
    static int nano_lastCursorScreenY = -1;
    static int nano_lastPreviewWidth = 0; // NEW: Track the width of the last preview
    static String nano_lastStringUnderCursor = ""; // NEW: Track the full string that was under
    static bool nano_wasCursorVisibleLast = false;

    // --- 1. Calculate Current Screen Position ---
    if (nano_focus != FOCUS_TEXT) {
        // If focus isn't on the text, we still need to clear the last cursor drawing
        if (nano_lastCursorScreenX != -1) {
            // (The clearing logic below will handle this)
        } else {
            return; // Nothing to do if focus is elsewhere and no cursor was ever drawn
        }
    }
    
    int screenLine = nano_cursorLine - nano_topLine;
    int screenCol = nano_cursorCol;
    if (screenLine < 0 || screenLine >= NANO_TEXT_AREA_LINES) {
        // To prevent artifacts, ensure the old cursor is erased if it goes off-screen
        screenCol = -1; // Mark as invalid for drawing, but allow clearing to proceed
    }
    
    int screenX = screenCol * CHAR_WIDTH;
    int screenY = (NANO_HEADER_LINES + screenLine) * LINE_HEIGHT;
    // --- 2. COMPLETE LOGIC to Determine Preview String ---
    String preview = "";
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

    if (preview.length() == 0) preview = "?"; // Fallback for unhandled cases

    String stringUnderCursor = "";
    int previewWidth = max(1, (int)preview.length());
    if (nano_cursorLine < nano_lineCount) {
        stringUnderCursor = nano_lines[nano_cursorLine].substring(nano_cursorCol, nano_cursorCol + previewWidth);
    }
    while(stringUnderCursor.length() < previewWidth) {
        stringUnderCursor += " "; // Pad with spaces if at end of line
    }

    // --- 3. REFACTORED: Loop to clear the entire area of the *previous* preview ---
    if (nano_lastCursorScreenX != -1) {
        int prevDrawX = nano_lastCursorScreenX;
        int prevDrawY = nano_lastCursorScreenY;

        // Loop over the width of the OLD preview, redrawing the original text
        for (int i = 0; i < nano_lastPreviewWidth; ++i) {
            // --- ADD THIS WRAPPING LOGIC ---
            if (prevDrawX >= SCREEN_WIDTH) {
                prevDrawX = 0;
                prevDrawY += LINE_HEIGHT;
            }
            // --- END OF CHANGE ---

            // Find the line of text that was under this part of the OLD cursor
            int prevLineIndex = (prevDrawY / LINE_HEIGHT) - NANO_HEADER_LINES + nano_topLine;
            String prevLineText = "";
            if (prevLineIndex >= 0 && prevLineIndex < nano_lineCount) {
                prevLineText = nano_lines[prevLineIndex];
            }
            int prevCol = prevDrawX / CHAR_WIDTH;
            
            // Clear the old block
            tft.fillRect(prevDrawX, prevDrawY, CHAR_WIDTH, LINE_HEIGHT, ST77XX_BLACK);
            
            // Redraw the character that is NOW at the old location
            if (prevCol < prevLineText.length()) {
                tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
                tft.setCursor(prevDrawX, prevDrawY);
                tft.print(prevLineText.charAt(prevCol));
            }
            prevDrawX += CHAR_WIDTH;
        }
    }
    
    // Don't draw a new cursor if it's off-screen or focus is lost
    if (screenCol < 0 || nano_focus != FOCUS_TEXT) {
        // Invalidate last position so it doesn't try to clear again from a bad spot
        nano_lastCursorScreenX = -1; 
        return;
    }

    // --- 4. Determine Colors ---
    uint16_t bgColor = ST77XX_BLACK;
    uint16_t fgColor = ST77XX_WHITE;
    uint16_t modeTextColor = ST77XX_WHITE;

    if (kmode == ALPHA || kmode == ALPHA_LOWER) modeTextColor = ST77XX_CYAN;
    else if (kmode == NUM) modeTextColor = ST77XX_GREEN;
    else if (kmode == SYM) modeTextColor = ST77XX_MAGENTA;
    else if (kmode == CTRL) modeTextColor = ST77XX_DARK_ORANGE;
    
    if (cursorVisible) { // Cursor ON (Highlighted)
        bgColor = (kbIndex == 0) ? modeTextColor : ST77XX_WHITE;
        fgColor = ST77XX_BLACK;
    } else { // Cursor OFF (Hollow)
        bgColor = ST77XX_BLACK;
        fgColor = (kbIndex == 0) ? modeTextColor : ST77XX_WHITE;
    }

    // --- 5. REFACTORED: Loop to draw the new, potentially multi-character preview ---
    int currentDrawX = screenX;
    int currentDrawY = screenY;
    for (int i = 0; i < previewWidth; ++i) {
        if (currentDrawX >= SCREEN_WIDTH) {
            currentDrawX = 0;
            currentDrawY += LINE_HEIGHT;
        }
        if (currentDrawY >= (NANO_HEADER_LINES + NANO_TEXT_AREA_LINES) * LINE_HEIGHT) {
            break; // Stop if we draw past the text area
        }
        tft.fillRect(currentDrawX, currentDrawY, CHAR_WIDTH, LINE_HEIGHT, bgColor);
        tft.setTextColor(fgColor, bgColor);
        tft.setCursor(currentDrawX, currentDrawY);
        tft.print(preview.charAt(i));
        currentDrawX += CHAR_WIDTH;
    }

    // --- 6. Update State for Next Time ---
    nano_lastCursorScreenX = screenX;
    nano_lastCursorScreenY = screenY;
    nano_lastStringUnderCursor = stringUnderCursor;
    nano_lastPreviewWidth = previewWidth;
    nano_wasCursorVisibleLast = cursorVisible;
}
void nano_moveCursor(int dx, int dy) {
    bool scrolled = false;

    // --- 1. Handle Vertical Movement (Page Up/Down or Line Up/Down) ---
    if (dy > 0) { // Moving Down
        int bottomVisibleLine = nano_topLine + NANO_TEXT_AREA_LINES - 1;
        
        // Check if cursor is already at the bottom edge
        if (nano_cursorLine >= bottomVisibleLine) {
            // --- Page Down ---
            int potentialNewTop = nano_topLine + NANO_TEXT_AREA_LINES;
            // Clamp nano_topLine so the last line doesn't scroll above the bottom
            nano_topLine = min(potentialNewTop, nano_lineCount - NANO_TEXT_AREA_LINES);
            if (nano_topLine < 0) nano_topLine = 0; // Ensure it doesn't go negative

            // Also move the cursor down by a page (or to the last line)
            nano_cursorLine = min(nano_cursorLine + NANO_TEXT_AREA_LINES, nano_lineCount - 1);
            
            scrolled = true;
        } else {
            // --- Normal Line Down ---
            nano_cursorLine += dy;
            // Clamp cursor line within buffer bounds
            if (nano_cursorLine >= nano_lineCount) {
                nano_cursorLine = nano_lineCount - 1; 
            }
        }
    } else if (dy < 0) { // Moving Up
        // Check if cursor is already at the top edge
        if (nano_cursorLine <= nano_topLine) {
            // --- Page Up ---
            int potentialNewTop = nano_topLine - NANO_TEXT_AREA_LINES;
            nano_topLine = max(0, potentialNewTop); // Clamp top line at 0

            // Also move the cursor up by a page (or to the first line)
            nano_cursorLine = max(0, nano_cursorLine - NANO_TEXT_AREA_LINES);
            
            scrolled = true;
        } else {
            // --- Normal Line Up ---
            nano_cursorLine += dy;
            // Clamp cursor line within buffer bounds
            if (nano_cursorLine < 0) {
                nano_cursorLine = 0;
            }
        }
    }

    // --- 2. Handle Horizontal Movement ---
    if (dx != 0) {
        nano_cursorCol += dx;
        // Clamp column (initially, can go past end temporarily)
        if (nano_cursorCol < 0) {
            nano_cursorCol = 0;
        }
        // Max column depends on the (potentially new) current line's length
        int currentLineLen = (nano_cursorLine < nano_lineCount) ? nano_lines[nano_cursorLine].length() : 0;
        // Allow cursor to go one position PAST the end for typing
        if (nano_cursorCol > currentLineLen) { 
             nano_cursorCol = currentLineLen;
        }
    }

    // --- 3. Adjust Column for Vertical Movement ---
    // If we moved vertically (dy != 0), clamp the column to the length of the new line.
    if (dy != 0) {
        int currentLineLen = (nano_cursorLine < nano_lineCount) ? nano_lines[nano_cursorLine].length() : 0;
        // Allow cursor to go one position PAST the end for typing
        if (nano_cursorCol > currentLineLen) { 
            nano_cursorCol = currentLineLen;
        }
    }

    // --- 4. Redraw if Scrolled ---
    if (scrolled) {
        // If the viewport changed, redraw the entire text area
        nano_drawTextArea();
        nano_drawHeader();
    }

    // The cursor itself will be redrawn by nano_handleInput or the main blink cycle
}
void nano_backspace() {
    // 1. --- Check for 0,0 (cannot backspace) ---
    if (nano_cursorLine == 0 && nano_cursorCol == 0) {
        return;
    }

    bool wasModified = nano_isModified;

    // --- 2. Check for "Fast Path" (single-line delete) ---
    String &currentLine = nano_lines[nano_cursorLine];
    bool isLastLine = (nano_cursorLine == nano_lineCount - 1);
    bool lineNotFull = (currentLine.length() < WRAP_COLS);

    if (nano_cursorCol > 0 && (lineNotFull || isLastLine)) {
        // --- FAST PATH: Just delete one char and redraw this line ---
        String beginning = currentLine.substring(0, nano_cursorCol - 1);
        String end = currentLine.substring(nano_cursorCol);
        currentLine = beginning + end;
        
        nano_cursorCol--; // Move cursor back
        nano_isModified = true;

        nano_redrawTrailingText(); // Redraw *only* this line

        if (!wasModified) {
            nano_drawHeader();
        }
        return; // We are done. Do not fall through.
    }

    // --- 3. FULL REFLOW or MERGE LOGIC ---
    bool needsScrollCheck = false;
    bool runReflowLoop = false; // Flag to control reflow

    if (nano_cursorCol > 0) {
        // --- CASE A: Backspace in middle of a *full* line ---
        String &line = nano_lines[nano_cursorLine];
        String beginning = line.substring(0, nano_cursorCol - 1);
        String end = line.substring(nano_cursorCol);
        line = beginning + end;
        
        nano_cursorCol--;
        nano_isModified = true;
        runReflowLoop = true; // <-- We *always* reflow in this case

    } else {
        // --- CASE B: Backspace at the start of a line ---
        int prevLineIndex = nano_cursorLine - 1;
        String &prevLine = nano_lines[prevLineIndex];
        String &currLine = nano_lines[nano_cursorLine];

        // Check if merging will exceed the *hard data limit*
        if (prevLine.length() + currLine.length() > NANO_MAX_LINE_LEN) {
             return; // Cannot merge, lines are too long
        }

        // --- THIS IS THE FIX ---
        // 1. Check if the previous line was empty *before* the merge.
        bool prevLineWasEmpty = (prevLine.length() == 0);

        // 2. We are merging, so move the cursor
        nano_cursorLine--;
        nano_cursorCol = prevLine.length();
        nano_isModified = true;
        needsScrollCheck = true;
        
        // 3. Now, *actually merge the line data*
        prevLine += currLine;
        
        // 4. Delete the (now empty) current line
        for (int j = nano_cursorLine + 1; j < nano_lineCount - 1; j++) {
            nano_lines[j] = nano_lines[j + 1];
        }
        nano_lineCount--;
        nano_lines[nano_lineCount] = "";
        
        // 5. Set the reflow flag based on your rule
        if (prevLineWasEmpty) {
            runReflowLoop = false; // Don't reflow if merging into an empty line
        } else {
            runReflowLoop = true; // *Always* reflow if merging into a line that had text
        }
        // --- END OF FIX ---
    }
    
    // 4. --- CONDITIONAL UPWARD REFLOW ---
    if (runReflowLoop) {
        // This loop starts at the current cursor line and pulls text up.
        for (int i = nano_cursorLine; i < nano_lineCount - 1; i++) {
            String &lineAbove = nano_lines[i];
            String &lineBelow = nano_lines[i + 1];

            int spaceLeft = WRAP_COLS - lineAbove.length();
            if (spaceLeft <= 0) break; // Waterfall stops, line above is full

            int pullUpLen = min(spaceLeft, lineBelow.length());
            
            if (pullUpLen > 0) {
                lineAbove += lineBelow.substring(0, pullUpLen);
                lineBelow = lineBelow.substring(pullUpLen);
            }

            if (lineBelow.length() == 0) {
                for (int j = i + 1; j < nano_lineCount - 1; j++) {
                    nano_lines[j] = nano_lines[j + 1];
                }
                nano_lineCount--;
                nano_lines[nano_lineCount] = "";
                i--; // Re-process this index, which now holds a new line
            }
        }
    }

    // 5. --- FINAL REDRAW ---
    if (needsScrollCheck && nano_cursorLine < nano_topLine) {
        nano_topLine = nano_cursorLine;
    }

    // Because multiple lines could have changed, we must redraw the text area.
    if (!wasModified) {
        nano_drawUI(); // Full UI redraw if it's the first modification
    } else {
        nano_drawTextArea(); // Otherwise, just the text area and footer
        nano_drawFooter();
    }
    
    nano_drawEditorCursor(); // Explicitly draw cursor at its new final position
}
void nano_redrawVisibleLinesFrom(int fileLineIndex) {
    int screenLine = fileLineIndex - nano_topLine;
    
    // Check if the starting line is visible in the viewport
    if (screenLine < NANO_TEXT_AREA_LINES) {
        // Start clearing and redrawing from the current line's screen position.
        int startY = (NANO_HEADER_LINES + max(0, screenLine)) * LINE_HEIGHT;
        
        // Calculate the total height from this point to the bottom of the text area.
        int clearHeight = (NANO_TEXT_AREA_LINES - max(0, screenLine)) * LINE_HEIGHT;
        
        // 1. Clear the entire area once
        tft.fillRect(0, startY, SCREEN_WIDTH, clearHeight, ST77XX_BLACK);

        // 2. Loop through and redraw all subsequent visible lines
        for (int r = max(0, screenLine); r < NANO_TEXT_AREA_LINES; ++r) {
            int lineIndex = nano_topLine + r;
            
            if (lineIndex < nano_lineCount) {
                tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
                tft.setCursor(0, (NANO_HEADER_LINES + r) * LINE_HEIGHT);
                tft.print(nano_lines[lineIndex].substring(0, COLS));
            } else {
                break; // Stop when we run out of lines in the buffer
            }
        }
    }
}
void nano_insertLine() {
    // Check if we can add another line
    if (nano_lineCount >= NANO_MAX_LINES) {
        return;
    }

    // Ensure cursor line is valid
    if (nano_cursorLine < 0 || nano_cursorLine >= nano_lineCount) {
        return;
    }

    // --- OPTIMIZATION: Check modified status BEFORE changing it ---
    bool wasModified = nano_isModified;

    String &currentLine = nano_lines[nano_cursorLine];
    String textAfterCursor = currentLine.substring(nano_cursorCol); // Text to move to new line

    // Truncate the current line at the cursor position
    currentLine = currentLine.substring(0, nano_cursorCol);

    // Shift lines down to make space for the new line
    for (int i = nano_lineCount; i > nano_cursorLine + 1; --i) {
        nano_lines[i] = nano_lines[i - 1];
    }

    // Insert the text after the cursor as the new line
    nano_lines[nano_cursorLine + 1] = textAfterCursor;

    nano_lineCount++;    // Increment total line count
    nano_cursorLine++;   // Move cursor to the beginning of the new line
    nano_cursorCol = 0;

    nano_isModified = true; // Mark file as modified

    // Handle scrolling if necessary (cursor moved down)
    bool scrolled = false;
    if (nano_cursorLine >= nano_topLine + NANO_TEXT_AREA_LINES) {
        nano_topLine = nano_cursorLine - NANO_TEXT_AREA_LINES + 1;
        scrolled = true;
    }

    // --- OPTIMIZED REDRAW ---
    if (!wasModified || scrolled) {
        // If this is the FIRST modification OR the viewport SCROLLED, do a full UI redraw.
        nano_drawUI();
    } else {
        // It was already modified and the viewport didn't move.
        // We only need to redraw the affected text area and the footer.
        nano_redrawVisibleLinesFrom(nano_cursorLine - 1); // <-- NEW SURGICAL CALL
        nano_drawFooter();
    }
    
    // The main loop will handle the final cursor draw.
}
bool nano_insertChar_DataWorker(char c) {
    bool didWrap = false;
    
    // Safety check
    if (nano_cursorLine < 0 || nano_cursorLine >= nano_lineCount) {
        return didWrap;
    }

    String &currentLine = nano_lines[nano_cursorLine]; 

    // Check for hard character limit
    if (currentLine.length() >= NANO_MAX_LINE_LEN) {
        return didWrap;
    }

    // 2. Insert the character into the data
    if (nano_cursorCol == currentLine.length()) {
        currentLine += c;
    } else {
        String beginning = currentLine.substring(0, nano_cursorCol);
        String end = currentLine.substring(nano_cursorCol);
        currentLine = beginning + c + end;
    }

    nano_cursorCol++; 
    nano_isModified = true;

    // --- NEW ITERATIVE (LOOP-BASED) REFLOW LOGIC ---
    // 3. Check if the insertion made the line too long
    if (!didWrap && currentLine.length() > WRAP_COLS) {
        
        // This loop will start at the line we just changed
        // and run down the file as long as lines keep overflowing.
        for (int i = nano_cursorLine; i < nano_lineCount; i++) {
            
            String &line = nano_lines[i];
            
            // 4. If this line is fine, the waterfall stops.
            if (line.length() <= WRAP_COLS) {
                break; // <-- Stop the loop
            }
            
            // 5. This line is too long. Split it.
            String overflowText = line.substring(WRAP_COLS);
            line = line.substring(0, WRAP_COLS); // Truncate current line

            // 6. Now, add the overflow to the *next* line (i + 1)
            if (i + 1 >= NANO_MAX_LINES) {
                // We're at the max buffer size, text is lost.
                break; // <-- Stop the loop
            }

            // Check if we're on the last line of the file
            if (i + 1 == nano_lineCount) {
                // We need to add a new line
                if (nano_lineCount < NANO_MAX_LINES) {
                    nano_lineCount++;
                    nano_lines[i + 1] = overflowText;
                } else {
                    // No more room in the buffer, stop.
                    break; // <-- Stop the loop
                }
            } else {
                // We're in the middle of the file, prepend to the existing next line
                String &nextLine = nano_lines[i + 1];
                nextLine = overflowText + nextLine;

                // Enforce the hard data limit (NANO_MAX_LINE_LEN)
                if (nextLine.length() > NANO_MAX_LINE_LEN) {
                    nextLine = nextLine.substring(0, NANO_MAX_LINE_LEN);
                }
            }
            
            // The loop will now continue (i++) and check the line we just modified
        }
        
        // 7. After the loop, do a multi-line redraw and set the flag
        nano_redrawVisibleLinesFrom(nano_cursorLine);
        didWrap = true; // Tell nano_insertChar we handled the redraw
        if (nano_cursorCol > WRAP_COLS) {
            nano_cursorLine++; // Move cursor to the next line
            nano_cursorCol = nano_cursorCol - WRAP_COLS; // e.g., 41 - 40 = 1

            // Handle scrolling if the cursor just moved off-screen
            if (nano_cursorLine >= nano_topLine + NANO_TEXT_AREA_LINES) {
                nano_topLine++;
                // A scroll occurred, the partial redraw isn't enough.
                nano_drawTextArea();
            }
        }
    }
    // --- END OF NEW LOGIC ---

    return didWrap; // Return whether a redraw occurred
}
void nano_redrawTrailingText() {
    // 1. Calculate the cursor's exact screen position (x, y)
    int screenLine = nano_cursorLine - nano_topLine;
    if (screenLine < 0 || screenLine >= NANO_TEXT_AREA_LINES) {
        return; // Don't draw if the line isn't visible
    }
    int cursorScreenX = nano_cursorCol * CHAR_WIDTH;
    int cursorScreenY = (NANO_HEADER_LINES + screenLine) * LINE_HEIGHT;

    // 2. Clear the screen area only from the cursor to the right edge
    tft.fillRect(cursorScreenX, cursorScreenY, SCREEN_WIDTH - cursorScreenX, LINE_HEIGHT, ST77XX_BLACK);

    // 3. Redraw the trailing text on the current line
    String currentLine = nano_lines[nano_cursorLine];
    if (nano_cursorCol < currentLine.length()) {
        String trailingText = currentLine.substring(nano_cursorCol);
        tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
        tft.setCursor(cursorScreenX, cursorScreenY);
        tft.print(trailingText);
    }
    // Note: This function does NOT draw the cursor itself. That is handled by the main loop.
    
}
bool nano_insertChar(char c) {
    // 1. Call the data worker to update the text buffer
    // This function [cite: 292-315] handles all the line wrap logic
    // and returns 'true' if a full redraw (a "wrap") already occurred.
    bool wrapOccurred = nano_insertChar_DataWorker(c);
    
    // 2. We do NO drawing here. All drawing is
    //    handled by the "controller" (nano_handleInput).
    
    // 3. Return the important status
    return wrapOccurred;
}
String findFileCaseInsensitive(const String& filename) {
    if (!fsReady) {
        return filename; // FS not ready, just return original name
    }

    File root = LittleFS.open("/", "r");
    if (!root) {
        return filename; // Can't open root, return original name
    }

    File file = root.openNextFile();
    while (file) {
        String currentFileName = file.name();
        
        // Check if the names match, ignoring case
        if (currentFileName.equalsIgnoreCase(filename)) {
            file.close();
            root.close();
            return currentFileName; // Return the *actual* cased name
        }
        file.close();
        file = root.openNextFile();
    }
    root.close();
    return filename; // No match found, return original name
}
void splitAndWrapFile(const String& inputFilename) {
    const size_t TARGET_CHUNK_SIZE = 100 * 1024; // 100KB
    const size_t REQUIRED_SPACE = TARGET_CHUNK_SIZE; // Minimum space needed for next chunk

    String actualFilename = findFileCaseInsensitive(inputFilename);
    if (!LittleFS.exists(actualFilename)) {
        pushSystemMessage("Error: Input file not found: " + actualFilename);
        return;
    }

    File inputFile = LittleFS.open(actualFilename, "r");
    if (!inputFile) {
        pushSystemMessage("Error: Could not open input file: " + actualFilename);
        return;
    }

    pushSystemMessage("Splitting " + actualFilename + " into parts...");
    drawFullTerminal();

    int partNum = 1;
    bool errorOccurred = false;
    bool spaceErrorOccurred = false; // <-- NEW FLAG
    String baseName = actualFilename;
    int dotIndex = baseName.lastIndexOf('.');
    if (dotIndex != -1) {
        baseName = baseName.substring(0, dotIndex);
    }

    String pending_line_part = "";

    while (inputFile.available() || pending_line_part.length() > 0) {

        // --- Check Available Space ---
        if (fsReady) {
            size_t totalBytes = 0, usedBytes = 0;
            #ifdef ESP32
                totalBytes = LittleFS.totalBytes(); usedBytes = LittleFS.usedBytes();
            #else // RP2040
                FSInfo fs_info;
                if (!LittleFS.info(fs_info)) {
                    pushSystemMessage("Error: Could not get FS info to check space.");
                    errorOccurred = true; break;
                }
                totalBytes = fs_info.totalBytes; usedBytes = fs_info.usedBytes;
            #endif
            size_t freeBytes = totalBytes - usedBytes;

            if (freeBytes < REQUIRED_SPACE) {
                String nextOutputFilename = baseName + String(partNum) + ".txt";
                pushSystemMessage("Error: Not enough space for file: " + nextOutputFilename);
                pushSystemMessage(" Available: " + String(freeBytes / 1024) + "KB, Needed: ~" + String(REQUIRED_SPACE / 1024) + "KB");
                errorOccurred = true;
                spaceErrorOccurred = true; // <-- SET SPECIFIC FLAG
                break; // Exit outer loop
            }
        } else {
             pushSystemMessage("Error: Filesystem not ready, cannot check space.");
             errorOccurred = true; break;
        }
        // --- END Check Available Space ---

        String outputFilename = baseName + String(partNum) + ".txt";
        File outputFile = LittleFS.open(outputFilename, "w");
        if (!outputFile) {
            pushSystemMessage("Error: Could not create output file: " + outputFilename);
            errorOccurred = true; break;
        }

        size_t currentChunkSize = 0;
        bool chunkFull = false;

        // --- Process pending data ---
        if (pending_line_part.length() > 0) {
            if (pending_line_part.length() + 1 > TARGET_CHUNK_SIZE) {
                 pushSystemMessage("Error: Single wrapped line exceeds chunk size near part " + String(partNum));
                 errorOccurred = true; outputFile.close(); LittleFS.remove(outputFilename); break;
            }
            if (outputFile.println(pending_line_part) > 0) {
                currentChunkSize += pending_line_part.length() + 1;
                pending_line_part = "";
            } else {
                pushSystemMessage("Error: Write failed for pending data in " + outputFilename);
                errorOccurred = true; outputFile.close(); break;
            }
        }

        // --- Fill the current chunk ---
        while (currentChunkSize < TARGET_CHUNK_SIZE && inputFile.available()) {
            String original_line = inputFile.readStringUntil('\n');
            if (original_line.endsWith("\r")) { original_line.remove(original_line.length() - 1); }

            // --- Apply UTF-8 Substitution ---
            String cleaned_line = "";
            // [COPY THE ENTIRE UTF-8 SUBSTITUTION for loop and switch statement HERE]
             int len = original_line.length();
             for (int i = 0; i < len; ++i) {
                 unsigned char c1 = original_line.charAt(i);
                 if (c1 < 0x80) { cleaned_line += (char)c1; }
                  else if ((c1 & 0xE0) == 0xC0) { // 2-byte
                       if(i+1 < len){ unsigned char c2=original_line.charAt(i+1); if((c2 & 0xC0)==0x80){ uint16_t cp=((c1 & 0x1F)<<6)|(c2 & 0x3F); switch(cp){ case 0xA3: cleaned_line+='?'; break; case 0xA9: cleaned_line+="(c)"; break; default: cleaned_line+='?'; break; } i++; } else { cleaned_line+='?';} } else { cleaned_line+='?';}
                  } else if ((c1 & 0xF0) == 0xE0) { // 3-byte
                       if(i+2 < len){ unsigned char c2=original_line.charAt(i+1); unsigned char c3=original_line.charAt(i+2); if(((c2 & 0xC0)==0x80)&&((c3 & 0xC0)==0x80)){ uint32_t cp=((c1 & 0x0F)<<12)|((c2 & 0x3F)<<6)|(c3 & 0x3F); switch(cp){ case 0x2018: cleaned_line+='\''; break; case 0x2019: cleaned_line+='\''; break; case 0x201C: cleaned_line+='"'; break; case 0x201D: cleaned_line+='"'; break; case 0x2013: cleaned_line+='-'; break; case 0x2014: cleaned_line+="--"; break; case 0x2026: cleaned_line+="..."; break; case 0x20AC: cleaned_line+='?'; break; default: cleaned_line+='?'; break; } i+=2; } else { cleaned_line+='?';} } else { cleaned_line+='?';}
                  } else if ((c1 & 0xF8) == 0xF0) { // 4-byte
                       if(i+3 < len){ unsigned char c2=original_line.charAt(i+1); unsigned char c3=original_line.charAt(i+2); unsigned char c4=original_line.charAt(i+3); if(((c2&0xC0)==0x80)&&((c3&0xC0)==0x80)&&((c4&0xC0)==0x80)){ cleaned_line+='?'; i+=3; } else { cleaned_line+='?';} } else { cleaned_line+='?';}
                  } else { cleaned_line += '?'; }
             } // end UTF-8 sub loop


            // --- Apply Word Wrapping ---
            String line_to_wrap = cleaned_line;
            do {
                String segment;
                if (line_to_wrap.length() > WRAP_COLS) {
                    int wrapPoint = -1;
                    for (int i = WRAP_COLS; i > 0; i--) { if (line_to_wrap.charAt(i) == ' ') { wrapPoint = i; break; } }
                    if (wrapPoint > 0) { segment = line_to_wrap.substring(0, wrapPoint); line_to_wrap = line_to_wrap.substring(wrapPoint + 1); }
                    else { segment = line_to_wrap.substring(0, WRAP_COLS); line_to_wrap = line_to_wrap.substring(WRAP_COLS); }
                } else { segment = line_to_wrap; line_to_wrap = ""; }

                if (currentChunkSize + segment.length() + 1 > TARGET_CHUNK_SIZE) {
                    pending_line_part = segment;
                     if (line_to_wrap.length() > 0) {
                          pending_line_part += "\n";
                          if (pending_line_part.length() + line_to_wrap.length() < (2 * TARGET_CHUNK_SIZE)) {
                               pending_line_part += line_to_wrap;
                          } else {
                               pushSystemMessage("Warning: Truncating extremely long pending line part.");
                          }
                     }
                    chunkFull = true; break; // Exit wrapping loop
                }

                if (outputFile.println(segment) == 0) {
                    pushSystemMessage("Error: Write failed for " + outputFilename);
                    errorOccurred = true; chunkFull = true; break; // Exit wrapping loop
                }
                currentChunkSize += segment.length() + 1;

            } while (line_to_wrap.length() > 0);

            if (chunkFull || errorOccurred) { break; } // Exit middle while loop
        } // end while (currentChunkSize < TARGET_CHUNK_SIZE...)

        // --- Finalize this chunk ---
        outputFile.close();
        if (currentChunkSize > 0 && !errorOccurred) {
             pushSystemMessage("Created: " + outputFilename + " (" + String(currentChunkSize) + " bytes)");
             drawFullTerminal();
        }

        if (errorOccurred) break;

        partNum++;
        delay(10); yield();

    } // end outer while

    inputFile.close();

    // --- MODIFIED FINAL MESSAGES ---
    if (!errorOccurred) {
        pushSystemMessage("File splitting complete.");
    } else if (!spaceErrorOccurred) { // <-- Check specific flag
        // Only show generic error if it wasn't the space error
        pushSystemMessage("File splitting stopped due to errors.");
    }
    // If spaceErrorOccurred is true, the specific message was already printed.
}
bool nano_loadFile(const String& filename) {
    nano_lineCount = 0; // Reset line count

    // Use findFileCaseInsensitive to get the actual filename
    String actualFilename = findFileCaseInsensitive(filename);

    if (!LittleFS.exists(actualFilename)) {
        // File doesn't exist, treat as a new empty file
        nano_lineCount = 1; nano_lines[0] = "";
        return false;
    }

    File file = LittleFS.open(actualFilename, "r");
    if (!file) {
        pushSystemMessage("Error: Failed to open file: " + actualFilename);
        nano_lineCount = 1; nano_lines[0] = "";
        return false;
    }

    // Read file line by line
    while (file.available() && nano_lineCount < NANO_MAX_LINES) {
        String original_line = file.readStringUntil('\n');
        
        if (original_line.endsWith("\r")) {
            original_line.remove(original_line.length() - 1);
        }

        // --- NEW UTF-8 DECODING & ASCII SUBSTITUTION LOGIC ---
        String cleaned_line = ""; // Build the ASCII-fied line here
        int len = original_line.length();
        for (int i = 0; i < len; ++i) {
            unsigned char c1 = original_line.charAt(i);

            if (c1 < 0x80) { // Standard ASCII (0xxxxxxx)
                cleaned_line += (char)c1;
            } else if ((c1 & 0xE0) == 0xC0) { // Start of 2-byte sequence (110xxxxx)
                if (i + 1 < len) {
                    unsigned char c2 = original_line.charAt(i + 1);
                    if ((c2 & 0xC0) == 0x80) { // Check for continuation byte (10xxxxxx)
                        uint16_t code_point = ((c1 & 0x1F) << 6) | (c2 & 0x3F);
                        // Add substitutions for 2-byte sequences if needed
                        switch (code_point) {
                           case 0xA3: cleaned_line += '?'; break; // £ Pound
                           case 0xA9: cleaned_line += "(c)"; break; // © Copyright
                           // Add more common Latin-1 extended chars if needed
                           default: cleaned_line += '?'; break; // Default for other 2-byte
                        }
                        i++; // Skip the next byte
                    } else {
                        cleaned_line += '?'; // Invalid sequence
                    }
                } else {
                    cleaned_line += '?'; // Incomplete sequence
                }
            } else if ((c1 & 0xF0) == 0xE0) { // Start of 3-byte sequence (1110xxxx)
                if (i + 2 < len) {
                    unsigned char c2 = original_line.charAt(i + 1);
                    unsigned char c3 = original_line.charAt(i + 2);
                    if (((c2 & 0xC0) == 0x80) && ((c3 & 0xC0) == 0x80)) {
                        uint32_t code_point = ((c1 & 0x0F) << 12) | ((c2 & 0x3F) << 6) | (c3 & 0x3F);
                        // Add specific substitutions
                        switch (code_point) {
                            case 0x2018: cleaned_line += '\''; break; // ‘ Left single quote
                            case 0x2019: cleaned_line += '\''; break; // ’ Right single quote
                            case 0x201C: cleaned_line += '"'; break; // “ Left double quote
                            case 0x201D: cleaned_line += '"'; break; // ” Right double quote
                            case 0x2013: cleaned_line += '-'; break; // – En dash
                            case 0x2014: cleaned_line += "--"; break; // — Em dash
                            case 0x2026: cleaned_line += "..."; break; // … Ellipsis
                            case 0x20AC: cleaned_line += '?'; break; // € Euro sign
                            // Add more 3-byte substitutions if desired
                            default: cleaned_line += '?'; break; // Default for other 3-byte
                        }
                        i += 2; // Skip next two bytes
                    } else {
                        cleaned_line += '?'; // Invalid sequence
                    }
                } else {
                    cleaned_line += '?'; // Incomplete sequence
                }
            } else if ((c1 & 0xF8) == 0xF0) { // Start of 4-byte sequence (11110xxx)
                 if (i + 3 < len) {
                     unsigned char c2 = original_line.charAt(i + 1);
                     unsigned char c3 = original_line.charAt(i + 2);
                     unsigned char c4 = original_line.charAt(i + 3);
                     if (((c2 & 0xC0) == 0x80) && ((c3 & 0xC0) == 0x80) && ((c4 & 0xC0) == 0x80)) {
                         cleaned_line += '?'; // Substitute all 4-byte sequences
                         i += 3; // Skip next three bytes
                     } else {
                         cleaned_line += '?'; // Invalid sequence
                     }
                 } else {
                    cleaned_line += '?'; // Incomplete sequence
                 }
            } else {
                // Invalid start byte or continuation byte found unexpectedly
                cleaned_line += '?';
            }
        } // end for loop iterating through bytes
        // --- END UTF-8 DECODING & ASCII SUBSTITUTION LOGIC ---


        // --- WORD-WRAPPING LOGIC (uses the 'cleaned_line') ---
        String line_to_wrap = cleaned_line; // Use the processed line
        do {
            if (nano_lineCount >= NANO_MAX_LINES) break; // Buffer is full

            if (line_to_wrap.length() > WRAP_COLS) {
                int wrapPoint = -1;
                for (int i = WRAP_COLS; i > 0; i--) {
                    if (line_to_wrap.charAt(i) == ' ') {
                        wrapPoint = i;
                        break;
                    }
                }
                if (wrapPoint > 0) {
                    nano_lines[nano_lineCount++] = line_to_wrap.substring(0, wrapPoint);
                    line_to_wrap = line_to_wrap.substring(wrapPoint + 1); 
                } else {
                    nano_lines[nano_lineCount++] = line_to_wrap.substring(0, WRAP_COLS);
                    line_to_wrap = line_to_wrap.substring(WRAP_COLS);
                }
            } else {
                nano_lines[nano_lineCount++] = line_to_wrap;
                line_to_wrap = ""; // Stop the loop
            }
        } while (line_to_wrap.length() > 0 && nano_lineCount < NANO_MAX_LINES);
        // --- END WORD-WRAPPING LOGIC ---
    } // end while loop reading lines

    if (file.available()) {
        pushSystemMessage("Warning: File exceeds max lines (" + String(NANO_MAX_LINES) + "). Truncated.");
    }

    file.close();
    
    if (nano_lineCount == 0) {
        nano_lineCount = 1; nano_lines[0] = "";
    }

    return true; // Loading successful
}
bool nano_saveFile() {
    // Attempt to open the file for writing (overwrite mode)
    File file = LittleFS.open(nano_filename, "w");
    if (!file) {
        // Could show an error message in the status bar or via pushSystemMessage
        return false; // Failed to open file for writing
    }

    // Write each line from the buffer to the file
    for (int i = 0; i < nano_lineCount; ++i) {
        if (file.println(nano_lines[i]) == 0) {
            // Write failed (e.g., filesystem full?)
            file.close();
            // Show error message
            return false;
        }
    }

    file.close(); // Ensure file is closed after writing
    nano_isModified = false; // Mark file as saved (not modified)
    // Optional: Update header immediately to remove '*'
    nano_drawHeader();
    return true; // Saving successful
}
void nano_handleInput(int buttonIndex) {
    bool redrawUI = false;
    bool redrawTextArea = false; 
    bool redrawCursor = true; 
    bool requiresKeyboardRedraw = false; 

    // --- 1. START THE TRANSACTION ---
    // All drawing commands will be buffered until we call endWrite().
    tft.startWrite();

    // --- Action based on current focus ---
    switch (nano_focus) {
        case FOCUS_TEXT:
            if (buttonIndex == IDX_PREV) {
                kbPrev();
                requiresKeyboardRedraw = true;
                redrawCursor = false;
            }
            else if (buttonIndex == IDX_NEXT) {
                kbNext();
                requiresKeyboardRedraw = true;
                redrawCursor = false;
            }
            else if (buttonIndex == IDX_BACK) {
                tft.endWrite(); // End the transaction *before* calling backspace
                nano_backspace(); // Backspace now handles its own transactions
                
                // --- FIX 1: Prevent double tft.endWrite() crash for BACKSPACE ---
                return; //

            }
            else if (buttonIndex == IDX_SELECT) {
                char charToInsert = 0;
                String controlAction = "";
                bool isCtrlAction = false; 
                const int ALPHA_CASE_KEY_INDEX = (int)strlen(alphaChars) + 1;
                const int NUM_KEY_INDEX_SPACE = (int)strlen(numberChars) + 1;
                const int SYM_KEY_INDEX_SPACE = (int)strlen(symbolChars) + 1;

                if (kbIndex == 0) {
                    if (kmode == ALPHA || kmode == ALPHA_LOWER) kmode = NUM;
                    else if (kmode == NUM) kmode = SYM;
                    else if (kmode == SYM) kmode = CTRL;
                    else if (kmode == CTRL) kmode = ALPHA;
                    else kmode = ALPHA;
                    kbIndex = 0;
                    requiresKeyboardRedraw = true;
                    redrawCursor = false;
                    f1_copy_index = 0;
                }
                 else if (kmode == ALPHA) {
                    if (kbIndex <= (int)strlen(alphaChars)) charToInsert = alphaChars[kbIndex - 1];
                    else if (kbIndex == ALPHA_CASE_KEY_INDEX) controlAction = "SPACE";
                    else if (kbIndex == ALPHA_CASE_KEY_INDEX + 1) controlAction = "ENTER";
                    else if (kbIndex == ALPHA_CASE_KEY_INDEX + 2) {
                        kmode = ALPHA_LOWER; kbIndex = ALPHA_CASE_KEY_INDEX + 2;
                        requiresKeyboardRedraw = true; redrawCursor = false;
                    }
                 }
                 else if (kmode == ALPHA_LOWER) {
                     if (kbIndex <= (int)strlen(alphaLowerChars)) charToInsert = alphaLowerChars[kbIndex - 1];
                     else if (kbIndex == ALPHA_CASE_KEY_INDEX) controlAction = "SPACE";
                     else if (kbIndex == ALPHA_CASE_KEY_INDEX + 1) controlAction = "ENTER";
                     else if (kbIndex == ALPHA_CASE_KEY_INDEX + 2) {
                         kmode = ALPHA; kbIndex = ALPHA_CASE_KEY_INDEX + 2;
                         requiresKeyboardRedraw = true; redrawCursor = false;
                     }
                 }
                 else if (kmode == NUM) {
                     if (kbIndex <= (int)strlen(numberChars)) charToInsert = numberChars[kbIndex - 1];
                     else if (kbIndex == NUM_KEY_INDEX_SPACE) controlAction = "SPACE";
                     else if (kbIndex == NUM_KEY_INDEX_SPACE + 1) controlAction = "ENTER";
                 }
                 else if (kmode == SYM) {
                     if (kbIndex <= (int)strlen(symbolChars)) charToInsert = symbolChars[kbIndex - 1];
                     else if (kbIndex == SYM_KEY_INDEX_SPACE) controlAction = "SPACE";
                     else if (kbIndex == SYM_KEY_INDEX_SPACE + 1) controlAction = "ENTER";
                 }
                 else if (kmode == CTRL) {
                    if (kbIndex > 0 && kbIndex <= CTRL_COUNT) {
                        controlAction = ctrlKeys[kbIndex - 1];
                        isCtrlAction = true;
                    }
                 }
                
                // --- Execute Action ---
                if (charToInsert != 0) {
                    bool wasModified = nano_isModified;
                    
                    // --- THIS IS THE "ELEVATED" LOGIC ---
                    // 1. Get cursor pos BEFORE insert
                    int screenLine = nano_cursorLine - nano_topLine;
                    int cursorScreenX = nano_cursorCol * CHAR_WIDTH;
                    int cursorScreenY = (NANO_HEADER_LINES + screenLine) * LINE_HEIGHT;
                    
                    // 2. Call the "gutted" function and get its status
                    bool wrapOccurred = nano_insertChar(charToInsert);
                    
                    // 3. We now run the smart logic that *was* in nano_insertChar
                    if (!wasModified) {
                        nano_drawHeader(); // Draw the '*'
                    }
                    
                    if (!wrapOccurred && screenLine >= 0 && screenLine < NANO_TEXT_AREA_LINES) {
                        tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
                        tft.setCursor(cursorScreenX, cursorScreenY);
                        tft.print(charToInsert);
                        nano_redrawTrailingText();
                    }
                    redrawCursor = true; // We need to draw the cursor over the char
                    // --- END OF "ELEVATED" LOGIC ---
                    
                    f1_copy_index = 0;
                    
                } else if (controlAction.length() > 0) {
                    if (isCtrlAction) { 
                        if (controlAction == "LEFT") {
                            nano_moveCursor(0, -1);
                        } else if (controlAction == "RIGHT") {
                            nano_moveCursor(1, 0);
                        } else if (controlAction == "UP") {
                            if (nano_cursorLine == 0 && nano_topLine == 0) {
                                nano_focus = FOCUS_HEADER;
                                redrawUI = true;
                                redrawCursor = false;
                            } else {
                                nano_moveCursor(0, -1);
                            }
                        } else if (controlAction == "DOWN") {
                            if (nano_cursorLine == nano_lineCount - 1) {
                                 nano_focus = FOCUS_FOOTER;
                                 nano_footerSelection = 0;
                                 redrawUI = true;
                                 redrawCursor = false;
                             } else {
                                 nano_moveCursor(0, 1);
                             }
                        } else if (controlAction == "DELETE") {
                             redrawCursor = false;
                        } else if (controlAction == "SPACE") { 
                           isCtrlAction = false;
                        } else if (controlAction == "ENTER") {
                           isCtrlAction = false;
                        }
                    }

                    if (!isCtrlAction) {
                        if (controlAction == "SPACE") {
                            // --- ALSO APPLY "ELEVATED" LOGIC HERE ---
                            // 1. Get cursor pos
                            int screenLine = nano_cursorLine - nano_topLine;
                            int cursorScreenX = nano_cursorCol * CHAR_WIDTH;
                            int cursorScreenY = (NANO_HEADER_LINES + screenLine) * LINE_HEIGHT;
                            
                            // 2. Call "gutted" function
                            bool wrapOccurred = nano_insertChar(' ');

                            // 3. Run smart logic
                            if (!wrapOccurred && screenLine >= 0 && screenLine < NANO_TEXT_AREA_LINES) {
                                tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
                                tft.setCursor(cursorScreenX, cursorScreenY);
                                tft.print(' ');
                                nano_redrawTrailingText();
                            }
                            redrawCursor = true;
                            // --- END OF FIX ---
                            f1_copy_index = 0;
                        } else if (controlAction == "ENTER") {
                            tft.endWrite(); // End the transaction *before* insertLine
                            nano_insertLine(); // This function calls drawUI, so no redraw needed
                            
                            // --- FIX 2: Prevent double tft.endWrite() crash for ENTER ---
                            return; //
                        }
                    }
                }
            }
            break; 

        // --- (All other 'case' blocks for HEADER, FOOTER, etc. remain the same) ---
        case FOCUS_HEADER:
            if (buttonIndex == IDX_NEXT) {
                nano_focus = FOCUS_TEXT;
                redrawUI = true;
                redrawCursor = true;
            } else {
                 redrawCursor = false;
            }
            break;

        case FOCUS_FOOTER:
            redrawCursor = false;
            if (buttonIndex == IDX_PREV) {
                nano_focus = FOCUS_TEXT;
                redrawUI = true;
                redrawCursor = true;
            } else if (buttonIndex == IDX_BACK) {
                nano_footerSelection--;
                if (nano_footerSelection < 0) nano_footerSelection = 1;
                nano_drawFooter();
            } else if (buttonIndex == IDX_NEXT) {
                 nano_footerSelection++;
                 if (nano_footerSelection > 1) nano_footerSelection = 0;
                 nano_drawFooter();
            } else if (buttonIndex == IDX_SELECT) { 
                if (nano_footerSelection == 0) { // Action: SAVE
                    tft.endWrite(); // End before save
                    if (nano_saveFile()) {
                        nano_isModified = false;
                        pushSystemMessage("File Saved: " + nano_filename);
                    } else {
                         pushSystemMessage("Error Saving File!");
                    }
                    tft.startWrite(); // Start new transaction
                    nano_focus = FOCUS_TEXT;
                    redrawUI = true; redrawCursor = true;
                } else if (nano_footerSelection == 1) { // Action: EXIT
                     if (nano_isModified) {
                         nano_focus = NANO_AWAIT_SAVE_CONFIRM;
                         nano_saveConfirmSelection = 0;
                         redrawUI = true; 
                         redrawCursor = false;
                     } else {
                         tft.endWrite(); // <-- Must end before returning!
                         nano_stop();
                         return;     
                     }
                }
            }
            break;

        case NANO_AWAIT_SAVE_CONFIRM:
            redrawCursor = false;
            if (buttonIndex == IDX_BACK) {
                nano_saveConfirmSelection--;
                if (nano_saveConfirmSelection < 0) nano_saveConfirmSelection = 1;
                nano_drawFooter();
            } else if (buttonIndex == IDX_NEXT) {
                nano_saveConfirmSelection++;
                if (nano_saveConfirmSelection > 1) nano_saveConfirmSelection = 0;
                nano_drawFooter();
            } else if (buttonIndex == IDX_SELECT) { 
                if (nano_saveConfirmSelection == 0) { // User selected 'Y'
                    tft.endWrite(); // End before save
                    if (!nano_saveFile()) {
                        pushSystemMessage("Error Saving File! Exit aborted.");
                        nano_focus = FOCUS_TEXT; 
                        redrawUI = true;
                        redrawCursor = true;
                        tft.startWrite(); // Start new transaction
                    } else {
                        nano_isModified = false;
                        pushSystemMessage("File Saved. Exiting.");
                        nano_stop(); 
                        
                        // --- FIX 3: Remove redundant tft.endWrite() and use return ---
                        // The original code had a second tft.endWrite() here, which caused a double-end crash.
                        // tft.endWrite(); // REMOVED BUG
                        
                        return; //
                    }
                } else { // User selected 'N'
                    pushSystemMessage("Exiting without saving.");
                    tft.endWrite(); // <-- Must end before returning!
                    nano_stop(); 
                    return;
                }
            }
            break;
    } // End switch(nano_focus)

    // --- Perform Redraws based on flags ---
    if (redrawUI) {
        nano_drawUI();
        if (nano_focus == FOCUS_TEXT) nano_drawEditorCursor();
    } else if (redrawTextArea) {
        nano_drawTextArea();
        if (nano_focus == FOCUS_TEXT) nano_drawEditorCursor();
    } else if (requiresKeyboardRedraw) {
        nano_drawFooter();
        if (nano_focus == FOCUS_TEXT) nano_drawEditorCursor(); 
    } else if (redrawCursor && nano_focus == FOCUS_TEXT) {
        nano_drawEditorCursor();
    }
    
    // --- 2. END THE TRANSACTION ---
    tft.endWrite();
}
uint32_t read32(File &f) {
  uint32_t result;
  uint8_t buffer[4];
  if (f.read(buffer, 4) == 4) {
    result = buffer[0] | (buffer[1] << 8) | (buffer[2] << 16) | (buffer[3] << 24);
    return result;
  }
  return 0; // Error case
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
Point project(Point3D p) {
    // Simple orthographic projection
    return Point((int)(p.x + SCREEN_WIDTH / 2), (int)(p.y + SCREEN_HEIGHT / 2));
}
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
void cube_start() {
    g_currentApp = APP_STATE_CUBE; // Set the global state

    tft.startWrite();
    tft.fillScreen(ST77XX_BLACK);
    tft.endWrite();
    
    // Initialize vertices (from runCubeAnimation) [cite: 575-580]
    float size = 50.0;
    g_cube_vertices[0] = {-size, -size, -size};
    g_cube_vertices[1] = { size, -size, -size};
    g_cube_vertices[2] = { size,  size, -size};
    g_cube_vertices[3] = {-size,  size, -size};
    g_cube_vertices[4] = {-size, -size,  size};
    g_cube_vertices[5] = { size, -size,  size};
    g_cube_vertices[6] = { size,  size,  size};
    g_cube_vertices[7] = {-size,  size,  size};
    
    g_cube_angleX = 0; g_cube_angleY = 0; g_cube_angleZ = 0;
    
    pushSystemMessage("Starting 3D animation...");
    drawFullTerminal(); // Draw the message
    delay(1000); 

    tft.startWrite();
    tft.fillScreen(ST77XX_BLACK);
    tft.endWrite();
}
void cube_stop() {
    g_currentApp = APP_STATE_CLI; // Go back to the CLI
    
    pushSystemMessage("Exiting 3D animation...");
    
    tft.startWrite();
    tft.fillScreen(ST77XX_BLACK);
    tft.endWrite();

    invalidateTerminalCache();
    drawFullTerminal(); // Redraw the terminal
}
void cube_update() {
    // Non-blocking timer. We want a 15ms delay, so 1000/15 = ~66 FPS
    if (millis() - g_cube_lastFrameTime < 15) {
        return; // Not time to draw a new frame yet
    }
    g_cube_lastFrameTime = millis(); // Reset the timer

    Point projected_points[8];
    tft.startWrite();

    // --- ERASE OLD CUBE ---
    for(int i=0; i<8; ++i) {
        Point3D p = g_cube_vertices[i];
        p = rotateX(p, g_cube_angleX);
        p = rotateY(p, g_cube_angleY);
        p = rotateZ(p, g_cube_angleZ);
        projected_points[i] = project(p);
    }
    drawRotatingCube(projected_points, ST77XX_BLACK);

    // --- UPDATE ANGLES ---
    g_cube_angleX += 1.0;
    g_cube_angleY += 1.5;
    g_cube_angleZ += 2.0;

    // --- DRAW NEW CUBE ---
    for(int i=0; i<8; ++i) {
        Point3D p = g_cube_vertices[i];
        p = rotateX(p, g_cube_angleX);
        p = rotateY(p, g_cube_angleY);
        p = rotateZ(p, g_cube_angleZ);
        projected_points[i] = project(p);
    }
    drawRotatingCube(projected_points, ST77XX_GREEN);
    
    tft.endWrite();
    
    // The delay(15) is now gone.
}
void mood_start() {
    g_currentApp = APP_STATE_MOOD;
    
    tft.startWrite();
    tft.fillScreen(ST77XX_BLACK);
    tft.endWrite();
    
    pushSystemMessage("Starting mood light!");
    drawFullTerminal();
    delay(1500); // Keep setup delay
    
    // Initialize state
    g_mood_currentHue = 0.0;
    g_mood_lastHueInt = -1;
}
void mood_stop() {
    g_currentApp = APP_STATE_CLI;
    pushSystemMessage("Exiting mood light...");
    
    tft.startWrite();
    tft.fillScreen(ST77XX_BLACK);
    tft.endWrite();

    invalidateTerminalCache();
    drawFullTerminal();
}
void mood_update() {
    // Non-blocking timer. We want a 15ms delay, so 1000/15 = ~66 FPS
    if (millis() - g_mood_lastFrameTime < 5) {
        return; // Not time yet
    }
    g_mood_lastFrameTime = millis(); // Reset the timer
    // Increment the hue
    g_mood_currentHue += 0.5; 
    if (g_mood_currentHue >= 360.0) {
        g_mood_currentHue -= 360.0;
    }

    int currentHueInt = (int)g_mood_currentHue;

    // Only redraw the screen if the integer part of the hue has changed
    if (currentHueInt != g_mood_lastHueInt) {
        uint16_t color = hsvToRgb565(currentHueInt, 255, 255);
        tft.startWrite();
        tft.fillScreen(color);
        tft.endWrite();
        g_mood_lastHueInt = currentHueInt;
    }
}
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
    String dayText = "Phase: " + String(day + 1);
    
    //int16_t x1, y1;
    //uint16_t w, h;
    //tft.getTextBounds(dayText, 0, 0, &x1, &y1, &w, &h);
    uint16_t w = tft.textWidth(dayText);
    uint16_t h = tft.fontHeight(); // Gets height of current font (size 2)
    int text_x = (SCREEN_WIDTH - w) / 2;
    int text_y = cy + r + 10;

    // THE FIX: Clear only a small, centered box for the text, not the full width.
    int clear_width = 120; // A fixed width that's large enough for "Day 30"
    int clear_x = (SCREEN_WIDTH - clear_width) / 2;
    tft.fillRect(clear_x, text_y, clear_width, h + 5, ST77XX_BLACK);
    
    tft.setCursor(text_x, text_y);
    tft.print(dayText);

    tft.setTextSize(1);
}
void moon_start() {
    g_currentApp = APP_STATE_MOON;
    g_moon_currentDay = 0; // Start at Day 0

    tft.startWrite();
    tft.fillScreen(ST77XX_BLACK);
    drawStars(); // Draw the starfield background once
    drawMoon(g_moon_currentDay, g_moon_totalDays);
    tft.endWrite();
}
void moon_stop() {
    g_currentApp = APP_STATE_CLI;
    
    tft.startWrite();
    tft.fillScreen(ST77XX_BLACK);
    tft.endWrite();
    
    invalidateTerminalCache();
    drawFullTerminal();
}
void moon_update() {
    // This app is 100% input-driven, so the update loop is idle.
    yield();
}
void moon_handleInput(int buttonIndex) {
    // This is the logic moved from handleButtonPresses
    bool dayChanged = false;

    if (buttonIndex == IDX_PREV) {
        g_moon_currentDay--;
        if (g_moon_currentDay < 0) g_moon_currentDay = g_moon_totalDays - 1;
        dayChanged = true;
    } else if (buttonIndex == IDX_NEXT) {
        g_moon_currentDay++;
        if (g_moon_currentDay >= g_moon_totalDays) g_moon_currentDay = 0;
        dayChanged = true;
    } else if (buttonIndex == IDX_BACK) {
        moon_stop();
    }
    
    // If a button changed the day, redraw the moon
    if (dayChanged) {
        tft.startWrite();
        drawMoon(g_moon_currentDay, g_moon_totalDays);
        tft.endWrite();
    }
}
void pic_start(const String& filename) {
    g_currentApp = APP_STATE_PIC;
    
    // --- FIX: REMOVED tft.setSwapBytes(false); ---
    // We will let the library use its default byte swapping.
    
    tft.startWrite();
    tft.fillScreen(ST77XX_BLACK);

    File bmpFile;
    int bmpWidth, bmpHeight;
    uint8_t bmpDepth;
    uint32_t bmpImageoffset;
    uint8_t rowBuffer[SCREEN_WIDTH * 3];
    uint16_t screenBuffer[SCREEN_WIDTH];

    bmpFile = LittleFS.open(filename, "r");
    if (!bmpFile) {
        tft.endWrite(); 
        pushSystemMessage("Error opening file: " + filename);
        drawFullTerminal();
        clearCurrentCommand();
        g_currentApp = APP_STATE_CLI; // Go back to CLI
        return;
    }

    if (read16(bmpFile) != 0x4D42) { bmpFile.close(); tft.endWrite(); return; }
    read32(bmpFile); // filesize
    read32(bmpFile); // reserved
    bmpImageoffset = read32(bmpFile); // pixel data offset
    read32(bmpFile); // DIB header size
    bmpWidth = read32(bmpFile);
    bmpHeight = read32(bmpFile);
    if (read16(bmpFile) != 1) { bmpFile.close(); tft.endWrite(); return; } // planes
    bmpDepth = read16(bmpFile); // bits per pixel
    if ((bmpDepth != 24) || (read32(bmpFile) != 0)) { bmpFile.close(); tft.endWrite(); return; } // uncompressed

    uint32_t rowSize = (bmpWidth * 3 + 3) & ~3;
    int drawWidth = min(bmpWidth, SCREEN_WIDTH);
    int drawHeight = min(bmpHeight, SCREEN_HEIGHT);
    int bmpXOffset = 0;
    if (bmpWidth > SCREEN_WIDTH) bmpXOffset = (bmpWidth - SCREEN_WIDTH) / 2;
    int bmpYStartRow = 0;
    if (bmpHeight > SCREEN_HEIGHT) bmpYStartRow = (bmpHeight - SCREEN_HEIGHT) / 2;
    int bmpYEndRow = bmpYStartRow + drawHeight;
    int screenXStart = (SCREEN_WIDTH - drawWidth) / 2;
    int screenYStart = (SCREEN_HEIGHT - drawHeight) / 2;
    
    for (int bmpRow = bmpYStartRow; bmpRow < bmpYEndRow; bmpRow++) {
        uint32_t filePos = bmpImageoffset + (bmpHeight - 1 - bmpRow) * rowSize + bmpXOffset * 3;
        bmpFile.seek(filePos);
        if (bmpFile.read(rowBuffer, drawWidth * 3) != drawWidth * 3) {
             break; 
        }
        int bufIdx = 0;
        for (int col = 0; col < drawWidth; col++) {
            uint8_t b = rowBuffer[bufIdx++]; // Blue byte from file
            uint8_t g = rowBuffer[bufIdx++]; // Green byte from file
            uint8_t r = rowBuffer[bufIdx++]; // Red byte from file
            
            // --- THIS IS THE FIX ---
            // Create the 16-bit RGB color word (R, G, B)
            // The library will handle all byte swapping.
            screenBuffer[col] = tft.color565(r, g, b);
            // --- END OF FIX ---
        }
        int screenY = screenYStart + (bmpRow - bmpYStartRow);
        
        // Use pushImage() to draw the 16-bit buffer
        tft.pushImage(screenXStart, screenY, drawWidth, 1, screenBuffer);
        
        yield();
    }

    bmpFile.close();
    tft.endWrite();
}
void pic_stop() {
    g_currentApp = APP_STATE_CLI;
    
    // --- FIX: REMOVED tft.setSwapBytes(true); ---
    // We are no longer changing the swap state.
    
    // Clean up the screen
    tft.startWrite();
    tft.fillScreen(ST77XX_BLACK);
    tft.endWrite();
    
    // Redraw the terminal
    invalidateTerminalCache();
    drawFullTerminal();
    clearCurrentCommand();
}
void pic_update() {
    // This app is 100% input-driven, so the update loop is idle.
    yield();

}
void pix_start() {
    g_currentApp = APP_STATE_PIX;
    g_pix_state = PIX_STATE_MAIN_MENU;
    g_pix_main_menu_selection = 0;
    
    pix_drawMainMenu();
}
void pix_stop() {
    
    if (g_pix_canvas != nullptr) {
        delete[] g_pix_canvas;
        g_pix_canvas = nullptr;
    }
    // --- ADD THIS ---
    // Clear the file list cache
    for(int i = 0; i < g_pix_file_count; i++) {
        g_pix_file_list[i] = "";
    }
    g_pix_file_count = 0;
    // --- END ADD ---

    g_currentApp = APP_STATE_CLI;
    tft.startWrite();
    tft.fillScreen(ST77XX_BLACK);
    tft.endWrite();
    invalidateTerminalCache();
    drawFullTerminal();
    clearCurrentCommand();
}
void pix_update(unsigned long now) {
    if (now - g_pix_last_blink > BLINK_MS) {
        g_pix_last_blink = now;
        g_pix_cursor_visible = !g_pix_cursor_visible; // Toggle visibility

        if (g_pix_state == PIX_STATE_EDITOR) {
            tft.startWrite();
            pix_drawCursor(g_pix_cursor_x, g_pix_cursor_y, g_pix_cursor_visible); // Draw toggled cursor
            tft.endWrite();
        } 
        else if (g_pix_state == PIX_STATE_EDITOR_FOOTER) {
            tft.startWrite();
            pix_drawEditorFooter(); // Redraw footer for its own blink
            tft.endWrite();
        }
        else if (g_pix_state == PIX_STATE_FILE_SELECT) {
            if (g_pix_last_state == PIX_STATE_EDITOR) {
                pix_drawSavePrompt(); // Redraw save prompt for its blink
            }
        }
    }
    yield();
}
void pix_buildFileList() {
    // Clear the old list first
    for(int i = 0; i < g_pix_file_count; i++) {
        g_pix_file_list[i] = "";
    }
    g_pix_file_count = 0;
    g_pix_file_selection = 0;
    g_pix_file_top_index = 0;

    if (!fsReady) {
        pushSystemMessage("Error: LittleFS not ready.");
        return;
    }

    File root = LittleFS.open("/", "r");
    if (!root) {
        pushSystemMessage("Error: Could not open FS root.");
        return;
    }

    File file = root.openNextFile();
    while (file && g_pix_file_count < SCROLLBACK_SIZE) {
        String filename = file.name();
        if (filename.endsWith(PIX_FILE_EXTENSION)) {
            g_pix_file_list[g_pix_file_count] = filename;
            g_pix_file_count++;
        }
        file.close();
        file = root.openNextFile();
    }
    root.close();
}
bool pix_loadBMP(String filename) {
    if (!fsReady) {
        pushSystemMessage("Error: LittleFS not ready.");
        return false;
    }

    File bmpFile = LittleFS.open(filename, "r");
    if (!bmpFile) {
        pushSystemMessage("Error opening file: " + filename);
        return false;
    }

    // --- Read BMP Header ---
    if (read16(bmpFile) != 0x4D42) { // 'BM'
        bmpFile.close(); 
        pushSystemMessage("Error: Not a BMP file.");
        return false; 
    }
    
    read32(bmpFile); // filesize
    read32(bmpFile); // reserved
    uint32_t bmpImageoffset = read32(bmpFile);
    read32(bmpFile); // DIB header size
    g_pix_canvas_width = read32(bmpFile);
    g_pix_canvas_height = read32(bmpFile);
    
    if (read16(bmpFile) != 1) { // planes
        bmpFile.close(); 
        pushSystemMessage("Error: BMP planes not 1.");
        return false; 
    }
    
    uint8_t bmpDepth = read16(bmpFile);
    if (bmpDepth != 24) {
        bmpFile.close(); 
        pushSystemMessage("Error: BMP must be 24-bit.");
        return false; 
    }
    
    if (read32(bmpFile) != 0) { // uncompressed
        bmpFile.close(); 
        pushSystemMessage("Error: BMP must be uncompressed.");
        return false; 
    }

    // --- Check if resolution is too big ---
    if (g_pix_canvas_width > 64 || g_pix_canvas_height > 64) {
        bmpFile.close();
        pushSystemMessage("Error: BMP max size is 64x64.");
        return false;
    }

    // --- Allocate memory ---
    if (g_pix_canvas != nullptr) {
        delete[] g_pix_canvas;
    }
    g_pix_canvas = new uint16_t[g_pix_canvas_width * g_pix_canvas_height];
    if (g_pix_canvas == nullptr) {
        bmpFile.close();
        pushSystemMessage("Error: Not enough memory!");
        return false;
    }

    // --- Read Pixel Data ---
    uint32_t rowSize = (g_pix_canvas_width * 3 + 3) & ~3; // BMP row padding
    uint8_t rowBuffer[g_pix_canvas_width * 3]; // Buffer for one BGR row

    for (int y = 0; y < g_pix_canvas_height; y++) {
        // BMPs are stored bottom-to-top, so we read them in reverse
        int bmpRow = (g_pix_canvas_height - 1) - y;
        uint32_t filePos = bmpImageoffset + (bmpRow * rowSize);
        bmpFile.seek(filePos);
        
        if (bmpFile.read(rowBuffer, g_pix_canvas_width * 3) != g_pix_canvas_width * 3) {
             break; // Read error
        }
        
        int bufIdx = 0;
        for (int x = 0; x < g_pix_canvas_width; x++) {
            uint8_t b = rowBuffer[bufIdx++];
            uint8_t g = rowBuffer[bufIdx++];
            uint8_t r = rowBuffer[bufIdx++];
            
            // Convert 24-bit BGR to 16-bit RGB565 and store in canvas
            g_pix_canvas[y * g_pix_canvas_width + x] = tft.color565(r, g, b);
        }
    }

    bmpFile.close();
    pushSystemMessage("Loaded " + filename);
    return true;
}
bool pix_saveBMP(String filename) {
    if (g_pix_canvas == nullptr) return false;
    if (!fsReady) {
        pushSystemMessage("Error: LittleFS not ready.");
        return false;
    }
    
    if (!filename.endsWith(PIX_FILE_EXTENSION)) {
        filename += PIX_FILE_EXTENSION;
    }

    File f = LittleFS.open(filename, "w");
    if (!f) {
        pushSystemMessage("Error: Could not create file.");
        return false;
    }

    // --- 1. Calculate BMP File Sizes ---
    int w = g_pix_canvas_width;
    int h = g_pix_canvas_height;
    
    // Each row must be padded to a multiple of 4 bytes
    uint32_t rowSize = (w * 3 + 3) & ~3;
    uint32_t imageSize = rowSize * h;
    uint32_t fileSize = 54 + imageSize; // 54-byte header

    // --- 2. Write BMP Header (54 bytes) ---
    write16_le(f, 0x4D42);     // 0: 'BM'
    write32_le(f, fileSize);  // 2: File size
    write32_le(f, 0);         // 6: Reserved
    write32_le(f, 54);        // 10: Pixel data offset
    
    write32_le(f, 40);        // 14: DIB Header size (40)
    write32_le(f, w);         // 18: Width
    write32_le(f, h);         // 22: Height
    write16_le(f, 1);         // 26: Planes
    write16_le(f, 24);        // 28: Bits per pixel (24)
    write32_le(f, 0);         // 30: Compression (0 = none)
    write32_le(f, imageSize); // 34: Image size
    write32_le(f, 0);         // 38: X Pixels/Meter (0)
    write32_le(f, 0);         // 42: Y Pixels/Meter (0)
    write32_le(f, 0);         // 46: Colors in palette (0)
    write32_le(f, 0);         // 50: Important colors (0)

    // --- 3. Write Pixel Data (BGR, Bottom-to-Top) ---
    // Create a buffer for one padded row
    uint8_t rowBuffer[rowSize];
    
    for (int y = h - 1; y >= 0; y--) { // Iterate from bottom row to top row
        int bufIdx = 0;
        for (int x = 0; x < w; x++) {
            // Get the 16-bit RGB565 color
            uint16_t color = g_pix_canvas[y * w + x];
            
            // Convert to 24-bit R, G, B
            uint8_t r = (color >> 11) & 0x1F;
            uint8_t g = (color >> 5) & 0x3F;
            uint8_t b = color & 0x1F;
            
            // Scale from 5/6-bit to 8-bit
            r = (r * 255) / 31;
            g = (g * 255) / 63;
            b = (b * 255) / 31;

            // Write as BGR
            rowBuffer[bufIdx++] = b;
            rowBuffer[bufIdx++] = g;
            rowBuffer[bufIdx++] = r;
        }
        
        // Add padding bytes
        while(bufIdx < rowSize) {
            rowBuffer[bufIdx++] = 0;
        }
        
        // Write the full row to the file
        f.write(rowBuffer, rowSize);
    }

    f.close();
    pushSystemMessage("Saved " + filename);
    return true;
}
void pix_initEditor(int res_index, String filename) {
    // Free any old canvas
    if (g_pix_canvas != nullptr) {
        delete[] g_pix_canvas;
        g_pix_canvas = nullptr;
    }

    if (filename.length() > 0) {
        // --- LOAD FROM FILE ---
        // pix_loadBMP will set the width/height and alloc memory
        if (!pix_loadBMP(filename)) {
            // FAILED TO LOAD
            // Go back to the file select screen
            g_pix_state = PIX_STATE_FILE_SELECT;
            pix_buildFileList();
            pix_drawFileSelect();
            return;
        }
    } else {
        // --- NEW FILE ---
        String res_str = PIX_RESOLUTIONS[res_index]; // e.g., "16x16"
        int x_pos = res_str.indexOf('x');
        g_pix_canvas_width = res_str.substring(0, x_pos).toInt();
        g_pix_canvas_height = res_str.substring(x_pos + 1).toInt();

        // Allocate memory for the new canvas
        g_pix_canvas = new uint16_t[g_pix_canvas_width * g_pix_canvas_height];
        if (g_pix_canvas == nullptr) {
             pushSystemMessage("Error: Not enough memory!");
             pix_stop();
             return;
        }
        
        // Clear the new canvas to black
        memset(g_pix_canvas, 0, g_pix_canvas_width * g_pix_canvas_height * sizeof(uint16_t));
    }

    // Reset editor state
    g_pix_cursor_x = 0;
    g_pix_cursor_y = 0;
    g_pix_palette_selection = 1; // Default to white
    g_pix_current_color = PIX_PALETTES[g_pix_palette_index][g_pix_palette_selection];
    g_pix_last_blink = millis();
    g_pix_cursor_visible = true;
    
    g_pix_state = PIX_STATE_EDITOR;
    pix_drawEditor();
}
void pix_drawMainMenu() {
    tft.startWrite();

    // --- NEW BACKGROUND LOGIC ---
    bool backgroundDrawn = false;
    if (fsReady) {
        String bg_filename = findFileCaseInsensitive("background.bmp");
        if (LittleFS.exists(bg_filename)) {
            
            if (pix_loadBMP(bg_filename)) { // This loads width/height into globals
                
                // --- FIX: REMOVED the strict "if (width == 64...)" check ---
                // The drawing logic will now work for any size BMP (<= 64x64)
                
                int pixel_size_w = SCREEN_WIDTH / g_pix_canvas_width;
                int pixel_size_h = SCREEN_HEIGHT / g_pix_canvas_height;
                int g_pix_pixel_size = min(pixel_size_w, pixel_size_h); // Get best fit scale

                int grid_width = g_pix_canvas_width * g_pix_pixel_size;
                int grid_height = g_pix_canvas_height * g_pix_pixel_size;

                int g_pix_grid_x_offset = (SCREEN_WIDTH - grid_width) / 2; // Center
                int g_pix_grid_y_offset = (SCREEN_HEIGHT - grid_height) / 2; // Center

                tft.fillScreen(ST77XX_BLACK); // Black background for centering

                for (int y = 0; y < g_pix_canvas_height; y++) {
                    for (int x = 0; x < g_pix_canvas_width; x++) {
                        int draw_x = g_pix_grid_x_offset + (x * g_pix_pixel_size);
                        int draw_y = g_pix_grid_y_offset + (y * g_pix_pixel_size);
                        uint16_t color = g_pix_canvas[y * g_pix_canvas_width + x];
                        tft.fillRect(draw_x, draw_y, g_pix_pixel_size, g_pix_pixel_size, color);
                    }
                }
                backgroundDrawn = true; // <-- This will now be set
                
                // --- END OF FIX ---
                
                // IMPORTANT: Free the canvas memory
                delete[] g_pix_canvas;
                g_pix_canvas = nullptr;
                g_pix_canvas_width = 0;
                g_pix_canvas_height = 0;

            } // end if pix_loadBMP
        } // end if exists
    } // end if fsReady

    if (!backgroundDrawn) {
        // Default: Draw the 64-color "att" palette background
        pix_drawPaletteBackground();
    }
    // --- END NEW BACKGROUND LOGIC ---
    
    // --- Draw UI elements OVER the background (transparent) ---
    const int text_y = 5;
    const int instructions_y = SCREEN_HEIGHT - (LINE_HEIGHT * 2) - 2;
    
    tft.setTextColor(ST77XX_WHITE);
    tft.setCursor(5, text_y);
    tft.print("Pixel Art Editor");

    tft.setTextColor(ST77XX_GREEN);
    tft.setCursor(10, instructions_y);
    tft.print("Use UP/DOWN to select.");
    tft.setCursor(10, instructions_y + LINE_HEIGHT);
    tft.print("Press ENTER to confirm, ESC to exit.");
    
    tft.endWrite();
    // Draw items surgically
    pix_drawMainMenuItem(0);
    pix_drawMainMenuItem(1);
}
void pix_drawPaletteBackground() {
    const int items_per_row = 8;
    // Calculate box size to fill the screen (240 / 8 = 30)
    const int box_size = SCREEN_WIDTH / items_per_row;
    const uint16_t* palette = RAMP_64_PALETTE; // The "att" palette
    const int color_count = 64;

    for (int i = 0; i < color_count; i++) {
        int row = i / items_per_row;
        int col = i % items_per_row;
        int draw_x = col * box_size;
        int draw_y = row * box_size;
        // Draw the 30x30 scaled pixel
        tft.fillRect(draw_x, draw_y, box_size, box_size, palette[i]);
    }
}
void pix_drawEditor() {
    if (g_pix_canvas == nullptr) return; // Safety check

    tft.startWrite();
    tft.fillScreen(ST77XX_BLACK); // Clear screen

    // --- 1. Calculate and SET GLOBAL pixel/grid size ---
    
    // Total available height is the screen height minus the footer
    int grid_area_height = SCREEN_HEIGHT - LINE_HEIGHT; 
    
    // Calculate the largest square pixel that fits
    int pixel_size_w = SCREEN_WIDTH / g_pix_canvas_width;
    int pixel_size_h = grid_area_height / g_pix_canvas_height;
    g_pix_pixel_size = min(pixel_size_w, pixel_size_h); // Use the smaller dimension
    
    int grid_width = g_pix_canvas_width * g_pix_pixel_size;
    int grid_height = g_pix_canvas_height * g_pix_pixel_size;
    
    // Center the grid horizontally
    g_pix_grid_x_offset = (SCREEN_WIDTH - grid_width) / 2;
    // Center the grid vertically in the available area
    g_pix_grid_y_offset = (grid_area_height - grid_height) / 2;

    // --- 2. Draw all the pixels from the canvas ---
    for (int y = 0; y < g_pix_canvas_height; y++) {
        for (int x = 0; x < g_pix_canvas_width; x++) {
            // Use the global offsets we just calculated
            int draw_x = g_pix_grid_x_offset + (x * g_pix_pixel_size);
            int draw_y = g_pix_grid_y_offset + (y * g_pix_pixel_size);
            uint16_t color = g_pix_canvas[y * g_pix_canvas_width + x];
            
            if (color == ST77XX_BLACK) {
                tft.fillRect(draw_x, draw_y, g_pix_pixel_size, g_pix_pixel_size, (x+y)%2 ? 0x2104 : 0x4208);
            } else {
                tft.fillRect(draw_x, draw_y, g_pix_pixel_size, g_pix_pixel_size, color);
            }
        }
    }

    // --- 3. Draw the Footer ---
    pix_drawEditorFooter(); 

    // --- 4. Draw the Cursor (Initial Draw) ---
    if (g_pix_state == PIX_STATE_EDITOR && g_pix_cursor_visible) {
        // <--- MODIFIED BLOCK ---
        pix_drawCursor(g_pix_cursor_x, g_pix_cursor_y, true);
        // <--- END MODIFIED BLOCK ---
    }
    
    tft.endWrite();
}
void pix_drawEditorFooter() {
    int startY = SCREEN_HEIGHT - LINE_HEIGHT;
    int w1 = (SCREEN_WIDTH * 2) / 5; // 40%
    int w2 = (SCREEN_WIDTH * 2) / 5; // 40%
    int w3 = SCREEN_WIDTH - w1 - w2; // 20%
    int x2 = w1;
    int x3 = w1 + w2;

    // --- Button 0: Save ---
    uint16_t bg1 = (g_pix_state == PIX_STATE_EDITOR_FOOTER && g_pix_footer_selection == 0 && g_pix_cursor_visible) ? ST77XX_CYAN : ST77XX_GREEN;
    tft.fillRect(0, startY, w1, LINE_HEIGHT, bg1);
    tft.setTextColor(ST77XX_BLACK, bg1);
    tft.setCursor(5, startY + 1);
    tft.print("Save");

    // --- Button 1: Palette ---
    uint16_t bg2 = (g_pix_state == PIX_STATE_EDITOR_FOOTER && g_pix_footer_selection == 1 && g_pix_cursor_visible) ? ST77XX_CYAN : ST77XX_GREEN;
    tft.fillRect(x2, startY, w2, LINE_HEIGHT, bg2);
    
    // --- MODIFIED BLOCK ---
    // Draw the color preview box
    int preview_x = x2 + 5;
    int preview_y = startY + 1;
    int preview_w = 20; // <-- Increased width from (LINE_HEIGHT - 2)
    int preview_h = LINE_HEIGHT - 2;
    tft.fillRect(preview_x, preview_y, preview_w, preview_h, g_pix_current_color);
    tft.drawRect(preview_x, preview_y, preview_w, preview_h, ST77XX_BLACK);
    
    // Draw text next to preview
    tft.setTextColor(ST77XX_BLACK, bg2);
    tft.setCursor(preview_x + preview_w + 4, startY + 1); // Adjusted cursor position
    tft.print("Palette");
    // --- END MODIFIED BLOCK ---

    // --- Button 2: Exit ---
    uint16_t bg3 = (g_pix_state == PIX_STATE_EDITOR_FOOTER && g_pix_footer_selection == 2 && g_pix_cursor_visible) ? ST77XX_CYAN : ST77XX_RED;
    uint16_t fg3 = (g_pix_state == PIX_STATE_EDITOR_FOOTER && g_pix_footer_selection == 2 && g_pix_cursor_visible) ? ST77XX_BLACK : ST77XX_WHITE;
    tft.fillRect(x3, startY, w3, LINE_HEIGHT, bg3);
    tft.setTextColor(fg3, bg3);
    tft.setCursor(x3 + 5, startY + 1);
    tft.print("Exit");
}
void pix_drawPalette() {
    tft.startWrite();
    tft.fillScreen(0x2104); // Dark grey background

    // --- 8x8 GEOMETRY ---
    int items_per_row = 8;
    int box_size = 20; // Size of each color swatch
    int box_padding = 8; // Padding between swatches
    
    int grid_width = (items_per_row * box_size) + ((items_per_row - 1) * box_padding); // 216
    int grid_height = grid_width; // It's a square
    
    int grid_x_offset = (SCREEN_WIDTH - grid_width) / 2; // 12
    int grid_y_offset = (SCREEN_HEIGHT - grid_height) / 2; // 12

    // --- MODIFIED BLOCK ---
    const uint16_t* current_palette = PIX_PALETTES[g_pix_palette_index];
    for (int i = 0; i < PIX_PALETTE_COLOR_COUNT; i++) {
    // --- END MODIFIED BLOCK ---
        int row = i / items_per_row;
        int col = i % items_per_row;
        
        int draw_x = grid_x_offset + (col * (box_size + box_padding));
        int draw_y = grid_y_offset + (row * (box_size + box_padding));
        
        // Draw the color swatch
        tft.fillRect(draw_x, draw_y, box_size, box_size, current_palette[i]); // <-- Use current_palette
    }

    // --- Instructions and Title Removed ---
    
    tft.endWrite();
    
    // Draw the initial cursor surgically AFTER the main draw
    pix_drawPaletteCursor(g_pix_palette_selection, true);
}
void pix_drawResSelect() {
    tft.startWrite();

    // --- NEW BACKGROUND LOGIC ---
    bool backgroundDrawn = false;
    if (fsReady) {
        String bg_filename = findFileCaseInsensitive("background.bmp");
        if (LittleFS.exists(bg_filename)) {
            if (pix_loadBMP(bg_filename)) {
                
                // --- FIX: REMOVED the strict "if (width == 64...)" check ---
                
                int pixel_size_w = SCREEN_WIDTH / g_pix_canvas_width;
                int pixel_size_h = SCREEN_HEIGHT / g_pix_canvas_height;
                int g_pix_pixel_size = min(pixel_size_w, pixel_size_h);
                int grid_width = g_pix_canvas_width * g_pix_pixel_size;
                int grid_height = g_pix_canvas_height * g_pix_pixel_size;
                int g_pix_grid_x_offset = (SCREEN_WIDTH - grid_width) / 2;
                int g_pix_grid_y_offset = (SCREEN_HEIGHT - grid_height) / 2;
                
                tft.fillScreen(ST77XX_BLACK);
                for (int y = 0; y < g_pix_canvas_height; y++) {
                    for (int x = 0; x < g_pix_canvas_width; x++) {
                        int draw_x = g_pix_grid_x_offset + (x * g_pix_pixel_size);
                        int draw_y = g_pix_grid_y_offset + (y * g_pix_pixel_size);
                        uint16_t color = g_pix_canvas[y * g_pix_canvas_width + x];
                        tft.fillRect(draw_x, draw_y, g_pix_pixel_size, g_pix_pixel_size, color);
                    }
                }
                backgroundDrawn = true;
                
                // --- END OF FIX ---

                delete[] g_pix_canvas;
                g_pix_canvas = nullptr;
                g_pix_canvas_width = 0;
                g_pix_canvas_height = 0;
            }
        }
    }

    if (!backgroundDrawn) {
        // Default: Draw the 64-color "att" palette background
        pix_drawPaletteBackground();
    }
    // --- END NEW BACKGROUND LOGIC ---

    // --- Draw UI elements OVER the background (transparent) ---
    tft.setTextColor(ST77XX_WHITE);
    tft.setCursor(5, 5);
    tft.print("Select Canvas Resolution");
    
    tft.setTextColor(ST77XX_GREEN);
    tft.setCursor(10, SCREEN_HEIGHT - (LINE_HEIGHT * 2) - 2);
    tft.print("Use UP/DOWN to select.");
    tft.setCursor(10, SCREEN_HEIGHT - LINE_HEIGHT - 2);
    tft.print("Press ENTER to confirm, ESC to go back.");
    
    tft.endWrite();
    
    // Draw items surgically (they have their own backgrounds)
    for (int i = 0; i < PIX_RES_COUNT; i++) {
        pix_drawResSelectItem(i);
    }
}
void pix_drawFileSelect() {
    tft.startWrite();

    // --- NEW BACKGROUND LOGIC ---
    bool backgroundDrawn = false;
    if (fsReady) {
        String bg_filename = findFileCaseInsensitive("background.bmp");
        if (LittleFS.exists(bg_filename)) {
            if (pix_loadBMP(bg_filename)) {

                // --- FIX: REMOVED the strict "if (width == 64...)" check ---

                int pixel_size_w = SCREEN_WIDTH / g_pix_canvas_width;
                int pixel_size_h = SCREEN_HEIGHT / g_pix_canvas_height;
                int g_pix_pixel_size = min(pixel_size_w, pixel_size_h);
                int grid_width = g_pix_canvas_width * g_pix_pixel_size;
                int grid_height = g_pix_canvas_height * g_pix_pixel_size;
                int g_pix_grid_x_offset = (SCREEN_WIDTH - grid_width) / 2;
                int g_pix_grid_y_offset = (SCREEN_HEIGHT - grid_height) / 2;
                
                tft.fillScreen(ST77XX_BLACK);
                for (int y = 0; y < g_pix_canvas_height; y++) {
                    for (int x = 0; x < g_pix_canvas_width; x++) {
                        int draw_x = g_pix_grid_x_offset + (x * g_pix_pixel_size);
                        int draw_y = g_pix_grid_y_offset + (y * g_pix_pixel_size);
                        uint16_t color = g_pix_canvas[y * g_pix_canvas_width + x];
                        tft.fillRect(draw_x, draw_y, g_pix_pixel_size, g_pix_pixel_size, color);
                    }
                }
                backgroundDrawn = true;

                // --- END OF FIX ---

                delete[] g_pix_canvas;
                g_pix_canvas = nullptr;
                g_pix_canvas_width = 0;
                g_pix_canvas_height = 0;
            }
        }
    }

    if (!backgroundDrawn) {
        // Default: Draw the 64-color "att" palette background
        pix_drawPaletteBackground();
    }
    // --- END NEW BACKGROUND LOGIC ---

    // --- Draw UI elements OVER the background (transparent) ---
    tft.setTextColor(ST77XX_WHITE);
    tft.setCursor(5, 5);
    tft.print("Select File to Edit");

    int y_pos = 20;
    int item_height = LINE_HEIGHT + 4;
    int footer_y = SCREEN_HEIGHT - (LINE_HEIGHT * 2) - 2;
    int max_visible_items = (footer_y - y_pos) / (item_height + 2);

    if (g_pix_file_count == 0) {
        // Add dark background for readability
        tft.setTextColor(ST77XX_YELLOW, ST77XX_DARKGREY);
        tft.setCursor(10, 50);
        tft.print("No .bmp files found.");
    } 
    
    // Instructions (transparent)
    tft.setTextColor(ST77XX_GREEN);
    tft.setCursor(10, footer_y);
    tft.print("Use UP/DOWN to select.");
    tft.setCursor(10, footer_y + LINE_HEIGHT);
    tft.print("Press ENTER to open, ESC to go back.");
    
    tft.endWrite();
    
    // Draw items surgically (they have their own backgrounds)
    if (g_pix_file_count > 0) {
        for (int i = 0; i < max_visible_items; i++) {
            int file_index = g_pix_file_top_index + i;
            if (file_index >= g_pix_file_count) {
                break;
            }
            pix_drawFileSelectItem(file_index);
        }
    }
}
void pix_drawPixel(int x, int y) {
    if (g_pix_canvas == nullptr || g_pix_pixel_size == 0) return;
    if (x < 0 || x >= g_pix_canvas_width || y < 0 || y >= g_pix_canvas_height) return;
    // Use global geometry
    int draw_x = g_pix_grid_x_offset + (x * g_pix_pixel_size);
    int draw_y = g_pix_grid_y_offset + (y * g_pix_pixel_size);
    uint16_t color = g_pix_canvas[y * g_pix_canvas_width + x];
    
    // tft.startWrite(); // <--- REMOVED
    // Draw checkerboard for black pixels
    if (color == ST77XX_BLACK) {
        tft.fillRect(draw_x, draw_y, g_pix_pixel_size, g_pix_pixel_size, (x+y)%2 ? 0x2104 : 0x4208);
    } else {
        tft.fillRect(draw_x, draw_y, g_pix_pixel_size, g_pix_pixel_size, color);
    }
    // tft.endWrite(); // <--- REMOVED
}
void pix_drawCursor(int x, int y, bool isVisible) { 
    if (g_pix_canvas == nullptr || g_pix_pixel_size == 0) return;
    
    // Use global geometry for grid offset, but passed-in x/y for pixel index
    int draw_x = g_pix_grid_x_offset + (x * g_pix_pixel_size); // <--- MODIFIED
    int draw_y = g_pix_grid_y_offset + (y * g_pix_pixel_size); // <--- MODIFIED

    // tft.startWrite(); // <--- REMOVED
    if (isVisible) {
        // Draw a blinking rectangle using the *currently selected* color
        tft.drawRect(draw_x, draw_y, g_pix_pixel_size, g_pix_pixel_size, g_pix_current_color);
        if (g_pix_pixel_size > 4) { // Draw a 2nd inner border
             tft.drawRect(draw_x+1, draw_y+1, g_pix_pixel_size-2, g_pix_pixel_size-2, ST77XX_BLACK);
        }
    } else {
        // Erase the cursor by redrawing the pixel underneath it
        uint16_t color = g_pix_canvas[y * g_pix_canvas_width + x]; // <--- MODIFIED
        if (color == ST77XX_BLACK) {
            tft.fillRect(draw_x, draw_y, g_pix_pixel_size, g_pix_pixel_size, (x+y)%2 ? 0x2104 : 0x4208); // <--- MODIFIED
        } else {
            tft.fillRect(draw_x, draw_y, g_pix_pixel_size, g_pix_pixel_size, color);
        }
    }
    // tft.endWrite(); // <--- REMOVED
}
void pix_drawPaletteCursor(int index, bool isVisible) {
    if (index < 0 || index >= PIX_PALETTE_COLOR_COUNT) return; // <-- Use new constant

    // --- 8x8 GEOMETRY (must match pix_drawPalette) ---
    int items_per_row = 8;
    int box_size = 20;
    int box_padding = 8;
    
    int grid_width = (items_per_row * box_size) + ((items_per_row - 1) * box_padding);
    int grid_x_offset = (SCREEN_WIDTH - grid_width) / 2;
    int grid_height = (items_per_row * box_size) + ((items_per_row - 1) * box_padding);
    int grid_y_offset = (SCREEN_HEIGHT - grid_height) / 2;

    int row = index / items_per_row;
    int col = index % items_per_row;
    
    int draw_x = grid_x_offset + (col * (box_size + box_padding));
    int draw_y = grid_y_offset + (row * (box_size + box_padding));
    
    tft.startWrite();
    if (isVisible) {
        // Draw cursor
        tft.drawRect(draw_x - 2, draw_y - 2, box_size + 4, box_size + 4, ST77XX_CYAN);
        tft.drawRect(draw_x - 1, draw_y - 1, box_size + 2, box_size + 2, ST77XX_WHITE);
    } else {
        // Erase cursor by redrawing the swatch and the background around it
        uint16_t bg_color = 0x2104; // Dark grey
        tft.fillRect(draw_x - 2, draw_y - 2, box_size + 4, box_size + 4, bg_color);
        // --- MODIFIED LINE ---
        tft.fillRect(draw_x, draw_y, box_size, box_size, PIX_PALETTES[g_pix_palette_index][index]);
        // --- END MODIFIED LINE ---
    }
    tft.endWrite();
}
void pix_drawMainMenuItem(int index) {
    if (index < 0 || index > 1) return;

    const char* labels[] = {"New Pixel Art!", "Edit Pixel Art!"};
    int y_pos = (index == 0) ? 30 : 50;
    
    uint16_t bg = (g_pix_main_menu_selection == index) ? ST77XX_CYAN : ST77XX_DARKGREY;
    uint16_t fg = (g_pix_main_menu_selection == index) ? ST77XX_BLACK : ST77XX_WHITE;

    tft.startWrite();
    tft.fillRect(10, y_pos, SCREEN_WIDTH - 20, LINE_HEIGHT + 4, bg);
    tft.setTextColor(fg, bg);
    tft.setCursor(15, y_pos + 2);
    tft.print(labels[index]);
    tft.endWrite();
}
void pix_drawResSelectItem(int index) {
    if (index < 0 || index >= PIX_RES_COUNT) return;

    int y_pos = 30;
    int item_height = LINE_HEIGHT + 4;
    y_pos += (index * (item_height + 6)); // 6px padding

    uint16_t bg = (g_pix_res_menu_selection == index) ? ST77XX_CYAN : ST77XX_DARKGREY;
    uint16_t fg = (g_pix_res_menu_selection == index) ? ST77XX_BLACK : ST77XX_WHITE;
    
    tft.startWrite();
    tft.fillRect(10, y_pos, SCREEN_WIDTH - 20, item_height, bg);
    tft.setTextColor(fg, bg);
    tft.setCursor(15, y_pos + 2);
    tft.print(PIX_RESOLUTIONS[index]);
    tft.endWrite();
}
void pix_drawFileSelectItem(int index) {
    if (index < 0 || index >= g_pix_file_count) return;

    // Calculate geometry
    int y_pos_start = 20;
    int item_height = LINE_HEIGHT + 4;
    int footer_y = SCREEN_HEIGHT - (LINE_HEIGHT * 2) - 2;
    int max_visible_items = (footer_y - y_pos_start) / (item_height + 2); // 2px padding

    // Check if this index is visible
    if (index < g_pix_file_top_index || index >= g_pix_file_top_index + max_visible_items) {
        return; // Not visible, do nothing
    }

    int screen_row = index - g_pix_file_top_index;
    int y_pos = y_pos_start + (screen_row * (item_height + 2)); // 2px padding

    uint16_t bg = (g_pix_file_selection == index) ? ST77XX_CYAN : ST77XX_DARKGREY;
    uint16_t fg = (g_pix_file_selection == index) ? ST77XX_BLACK : ST77XX_WHITE;
    
    tft.startWrite();
    tft.fillRect(10, y_pos, SCREEN_WIDTH - 20, item_height, bg);
    tft.setTextColor(fg, bg);
    tft.setCursor(15, y_pos + 2);
    tft.print(g_pix_file_list[index]);
    tft.endWrite();
}
void write16_le(File &f, uint16_t val) {
    uint8_t buf[2];
    buf[0] = (uint8_t)val;
    buf[1] = (uint8_t)(val >> 8);
    f.write(buf, 2);
}
void write32_le(File &f, uint32_t val) {
    uint8_t buf[4];
    buf[0] = (uint8_t)val;
    buf[1] = (uint8_t)(val >> 8);
    buf[2] = (uint8_t)(val >> 16);
    buf[3] = (uint8_t)(val >> 24);
    f.write(buf, 4);
}
void pix_drawSavePrompt() {
    int w = SCREEN_WIDTH - 40;
    int h = 40;
    int x = (SCREEN_WIDTH - w) / 2;
    int y = (SCREEN_HEIGHT - h) / 2;

    tft.startWrite();
    // Draw the dialog box
    tft.fillRect(x, y, w, h, 0x4208); // Dark Brown
    tft.drawRect(x - 1, y - 1, w + 2, h + 2, ST77XX_WHITE);
    tft.drawRect(x - 2, y - 2, w + 4, h + 4, ST77XX_BLACK);
    
    tft.setTextColor(ST77XX_WHITE, 0x4208);
    tft.setCursor(x + 5, y + 5);
    tft.print("Save as:");

    // Draw the text field background
    tft.fillRect(x + 5, y + 20, w - 10, LINE_HEIGHT + 4, ST77XX_BLACK);
    
    // Draw the text
    tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    tft.setCursor(x + 8, y + 22);
    tft.print(g_pix_filename_buffer);

    // Draw the cursor
    if (g_pix_cursor_visible) {
        int cursor_x = x + 8 + (g_pix_filename_len * CHAR_WIDTH);
        tft.fillRect(cursor_x, y + 21, CHAR_WIDTH, LINE_HEIGHT + 2, ST77XX_WHITE);
    }
    
    tft.endWrite();
}
void pix_handleInput(int buttonIndex) {
    // 4-Button input handler
    if (g_pix_state == PIX_STATE_MAIN_MENU) {
        if (buttonIndex == IDX_PREV) { // Up
            if (g_pix_main_menu_selection != 0) {
                int old_selection = g_pix_main_menu_selection;
                g_pix_main_menu_selection = 0;
                pix_drawMainMenuItem(old_selection);
                pix_drawMainMenuItem(g_pix_main_menu_selection);
            }
        } else if (buttonIndex == IDX_NEXT) { // Down
            if (g_pix_main_menu_selection != 1) {
                int old_selection = g_pix_main_menu_selection;
                g_pix_main_menu_selection = 1;
                pix_drawMainMenuItem(old_selection);
                pix_drawMainMenuItem(g_pix_main_menu_selection);
            }
        } else if (buttonIndex == IDX_BACK) { // Back
            pix_stop();
        } else if (buttonIndex == IDX_SELECT) { // Enter
            if (g_pix_main_menu_selection == 0) {
                g_pix_state = PIX_STATE_RES_SELECT;
                g_pix_res_menu_selection = 0;
                pix_drawResSelect();
            } else {
                g_pix_state = PIX_STATE_FILE_SELECT;
                g_pix_last_state = PIX_STATE_MAIN_MENU;
                pix_buildFileList();
                pix_drawFileSelect();
            }
        }
    }
    else if (g_pix_state == PIX_STATE_RES_SELECT) {
        if (buttonIndex == IDX_PREV) { // Up
            if (g_pix_res_menu_selection > 0) {
                int old_selection = g_pix_res_menu_selection;
                g_pix_res_menu_selection--;
                pix_drawResSelectItem(old_selection);
                pix_drawResSelectItem(g_pix_res_menu_selection);
            }
        } else if (buttonIndex == IDX_NEXT) { // Down
            if (g_pix_res_menu_selection < PIX_RES_COUNT - 1) {
                int old_selection = g_pix_res_menu_selection;
                g_pix_res_menu_selection++;
                pix_drawResSelectItem(old_selection);
                pix_drawResSelectItem(g_pix_res_menu_selection);
            }
        } else if (buttonIndex == IDX_BACK) { // Back
            g_pix_state = PIX_STATE_MAIN_MENU;
            pix_drawMainMenu();
        } else if (buttonIndex == IDX_SELECT) { // Enter
            pix_initEditor(g_pix_res_menu_selection, "");
        }
    }
    else if (g_pix_state == PIX_STATE_FILE_SELECT) {
        if (g_pix_last_state == PIX_STATE_EDITOR) {
            // --- In "Save" mode ---
            if (buttonIndex == IDX_PREV) { // Up is Cancel
                g_pix_state = PIX_STATE_EDITOR;
                pix_drawEditor();
            } else if (buttonIndex == IDX_NEXT) { // Down is Confirm
                if (g_pix_filename_len > 0) {
                    g_pix_filename_buffer[g_pix_filename_len] = 0;
                    String filename = String(g_pix_filename_buffer);
                    pix_saveBMP(filename);
                    g_pix_state = PIX_STATE_EDITOR;
                    pix_drawEditor();
                }
            } else if (buttonIndex == IDX_BACK) { // Back is Backspace
                if (g_pix_filename_len > 0) {
                    g_pix_filename_len--;
                    g_pix_filename_buffer[g_pix_filename_len] = 0;
                    pix_drawSavePrompt();
                }
            } else if (buttonIndex == IDX_SELECT) { // Enter is Confirm
                if (g_pix_filename_len > 0) {
                    g_pix_filename_buffer[g_pix_filename_len] = 0;
                    String filename = String(g_pix_filename_buffer);
                    pix_saveBMP(filename);
                    g_pix_state = PIX_STATE_EDITOR;
                    pix_drawEditor();
                }
            }
        } else {
            // --- In "Load" mode ---
            if (buttonIndex == IDX_PREV) { // Up
                if (g_pix_file_selection > 0) {
              
                    int old_selection = g_pix_file_selection;
                    g_pix_file_selection--;
                    if (g_pix_file_selection < g_pix_file_top_index) {
                        g_pix_file_top_index = g_pix_file_selection;
                        pix_drawFileSelect();
                    } else {
                        pix_drawFileSelectItem(old_selection);
                        pix_drawFileSelectItem(g_pix_file_selection);
                    }
                }
            } else if (buttonIndex == IDX_NEXT) { // Down
                if (g_pix_file_selection < g_pix_file_count - 1) {
                    
                    int old_selection = g_pix_file_selection;
                    g_pix_file_selection++;
                    
                    int y_pos = 20;
                    int item_height = LINE_HEIGHT + 4;
                    int footer_y = SCREEN_HEIGHT - (LINE_HEIGHT * 2) - 2;
                    int max_visible_items = (footer_y - y_pos) / (item_height + 2);
                    if (g_pix_file_selection >= g_pix_file_top_index + max_visible_items) {
                        g_pix_file_top_index++;
                        pix_drawFileSelect();
                    } else {
                        pix_drawFileSelectItem(old_selection);
                        pix_drawFileSelectItem(g_pix_file_selection);
                    }
                }
            } else if (buttonIndex == IDX_BACK) { // Back
                g_pix_state = PIX_STATE_MAIN_MENU;
                pix_drawMainMenu();
            } else if (buttonIndex == IDX_SELECT) { // Enter
                if (g_pix_file_count > 0 && g_pix_file_selection < g_pix_file_count) {
                    String filename = g_pix_file_list[g_pix_file_selection];
                    pix_initEditor(-1, filename);
                }
            }
        }
    }
    else if (g_pix_state == PIX_STATE_EDITOR) {
        tft.startWrite();
        
        int old_cursor_x = g_pix_cursor_x;
        int old_cursor_y = g_pix_cursor_y;
        
        pix_drawCursor(old_cursor_x, old_cursor_y, false);

        // --- HANDLE MOVEMENT ---
        if (buttonIndex == IDX_PREV) { // Up
            g_pix_cursor_y--;
            if (g_pix_cursor_y < 0) g_pix_cursor_y = g_pix_canvas_height - 1;
        } else if (buttonIndex == IDX_NEXT) { // Down
            g_pix_cursor_y++;
            if (g_pix_cursor_y >= g_pix_canvas_height) {
                g_pix_cursor_y = g_pix_canvas_height - 1;
                g_pix_state = PIX_STATE_EDITOR_FOOTER;
                g_pix_footer_selection = 0;
            }
        } else if (buttonIndex == IDX_BACK) { // Back (remap to Left)
            g_pix_cursor_x--;
            if (g_pix_cursor_x < 0) g_pix_cursor_x = g_pix_canvas_width - 1;
        } else if (buttonIndex == IDX_SELECT) { // Select (Draw Pixel)
            g_pix_canvas[g_pix_cursor_y * g_pix_canvas_width + g_pix_cursor_x] = g_pix_current_color;
            pix_drawPixel(g_pix_cursor_x, g_pix_cursor_y);
        }
        
        // --- THIS IS THE FIX ---
        // Force the cursor to be visible and reset the blink timer
        g_pix_cursor_visible = true;
        g_pix_last_blink = millis();
        // --- END FIX ---

        if (g_pix_state == PIX_STATE_EDITOR) {
            // Draw new cursor (it's now visible)
            pix_drawCursor(g_pix_cursor_x, g_pix_cursor_y, true); 
        } else {
            // Moving to footer
            pix_drawEditorFooter();
        }
        
        tft.endWrite();
    }
    else if (g_pix_state == PIX_STATE_EDITOR_FOOTER) {
        if (buttonIndex == IDX_PREV) { // Up
            // --- NAVIGATE TO GRID ---
            g_pix_state = PIX_STATE_EDITOR;

            // --- THIS IS THE FIX ---
            g_pix_cursor_visible = true;
            g_pix_last_blink = millis();
            tft.startWrite();
            pix_drawEditorFooter(); // Redraw footer to hide focus
            pix_drawCursor(g_pix_cursor_x, g_pix_cursor_y, true);
            tft.endWrite();
            // --- END FIX ---

        } else if (buttonIndex == IDX_NEXT) { // Down (acts as Right)
            g_pix_footer_selection++;
            if (g_pix_footer_selection > 2) g_pix_footer_selection = 0;
            tft.startWrite();
            pix_drawEditorFooter(); // Just redraw footer for blink
            tft.endWrite();
        } else if (buttonIndex == IDX_BACK) { // Back (acts as Left)
            g_pix_footer_selection--;
            if (g_pix_footer_selection < 0) g_pix_footer_selection = 2;
            tft.startWrite();
            pix_drawEditorFooter(); // Just redraw footer for blink
            tft.endWrite();
        } else if (buttonIndex == IDX_SELECT) { // Enter
            // --- Activate Footer Button ---
            if (g_pix_footer_selection == 0) { // Save
                g_pix_last_state = PIX_STATE_EDITOR;
                g_pix_state = PIX_STATE_FILE_SELECT;
                g_pix_filename_len = 0;
                g_pix_filename_buffer[0] = 0;
                pix_drawSavePrompt();
            } else if (g_pix_footer_selection == 1) { // Palette
                g_pix_state = PIX_STATE_PALETTE;
                pix_drawPalette();
            } else if (g_pix_footer_selection == 2) { // Exit
                g_pix_state = PIX_STATE_MAIN_MENU;
                pix_drawMainMenu();
            }
        }
    }
    else if (g_pix_state == PIX_STATE_PALETTE) {
        int old_selection = g_pix_palette_selection;
        int items_per_row = 8;
        bool selection_changed = false;

        if (buttonIndex == IDX_PREV) { // Up
            g_pix_palette_selection -= items_per_row;
            if (g_pix_palette_selection < 0) g_pix_palette_selection += PIX_PALETTE_COLOR_COUNT;
            selection_changed = true;
        } else if (buttonIndex == IDX_NEXT) { // Down
            g_pix_palette_selection += items_per_row;
            if (g_pix_palette_selection >= PIX_PALETTE_COLOR_COUNT) g_pix_palette_selection -= PIX_PALETTE_COLOR_COUNT;
            selection_changed = true;
        } else if (buttonIndex == IDX_BACK) { // Back (remap to Left)
            int old_col = old_selection % items_per_row;
            if (old_col > 0) {
                // Move left within palette
                g_pix_palette_selection--;
                selection_changed = true;
            } else {
                // Wrap to previous palette
                int row = old_selection / items_per_row;
                g_pix_palette_index = (g_pix_palette_index - 1 + PIX_PALETTE_SET_COUNT) % PIX_PALETTE_SET_COUNT;
                g_pix_palette_selection = (row * items_per_row) + (items_per_row - 1);
                pix_drawPalette();
            }
        } else if (buttonIndex == IDX_SELECT) { // Enter (remap to "Select Color")
            g_pix_current_color = PIX_PALETTES[g_pix_palette_index][g_pix_palette_selection];
            g_pix_state = PIX_STATE_EDITOR;
            pix_drawEditor(); // Full redraw to return to editor
        }

        if (selection_changed && old_selection != g_pix_palette_selection) {
            pix_drawPaletteCursor(old_selection, false);
            pix_drawPaletteCursor(g_pix_palette_selection, true);
        }
    }
}
void pix_handleKey(uint8_t c) {
    // CardKB input handler
    if (g_pix_state == PIX_STATE_MAIN_MENU) {
        if (c == KEY_UP_ARROW) {
            if (g_pix_main_menu_selection != 0) {
                int old_selection = g_pix_main_menu_selection;
                g_pix_main_menu_selection = 0;
                pix_drawMainMenuItem(old_selection); // Deselect
                pix_drawMainMenuItem(g_pix_main_menu_selection);
            }
        } else if (c == KEY_DOWN_ARROW) {
            if (g_pix_main_menu_selection != 1) {
                int old_selection = g_pix_main_menu_selection;
                g_pix_main_menu_selection = 1;
                pix_drawMainMenuItem(old_selection); // Deselect
                pix_drawMainMenuItem(g_pix_main_menu_selection);
            }
        } else if (c == KEY_ESCAPE) {
            pix_stop();
        } else if (c == 0x0D) { // Enter
            if (g_pix_main_menu_selection == 0) {
                g_pix_state = PIX_STATE_RES_SELECT;
                g_pix_res_menu_selection = 0;
                pix_drawResSelect();
            } else {
                g_pix_state = PIX_STATE_FILE_SELECT;
                g_pix_last_state = PIX_STATE_MAIN_MENU;
                pix_buildFileList();
                pix_drawFileSelect();
            }
        }
    }
    else if (g_pix_state == PIX_STATE_RES_SELECT) {
        if (c == KEY_UP_ARROW) {
            if (g_pix_res_menu_selection > 0) {
                int old_selection = g_pix_res_menu_selection;
                g_pix_res_menu_selection--;
                pix_drawResSelectItem(old_selection);
                pix_drawResSelectItem(g_pix_res_menu_selection);
            }
        } else if (c == KEY_DOWN_ARROW) {
            if (g_pix_res_menu_selection < PIX_RES_COUNT - 1) {
                int old_selection = g_pix_res_menu_selection;
                g_pix_res_menu_selection++;
                pix_drawResSelectItem(old_selection);
                pix_drawResSelectItem(g_pix_res_menu_selection);
            }
        } else if (c == KEY_ESCAPE) {
            g_pix_state = PIX_STATE_MAIN_MENU;
            pix_drawMainMenu();
        } else if (c == 0x0D) { // Enter
            pix_initEditor(g_pix_res_menu_selection, "");
        }
    }
    else if (g_pix_state == PIX_STATE_FILE_SELECT) {
        if (g_pix_last_state == PIX_STATE_EDITOR) {
            // --- We are in the "Save As:" prompt ---
            if (c == KEY_ESCAPE) {
                g_pix_state = PIX_STATE_EDITOR;
                pix_drawEditor();
            } else if (c == 0x08) { // Backspace
                if (g_pix_filename_len > 0) {
                    g_pix_filename_len--;
                    g_pix_filename_buffer[g_pix_filename_len] = 0;
                    pix_drawSavePrompt();
                }
            } else if (c == 0x0D) { // Enter
                if (g_pix_filename_len > 0) {
                    g_pix_filename_buffer[g_pix_filename_len] = 0;
                    String filename = String(g_pix_filename_buffer);
                    pix_saveBMP(filename);
                    g_pix_state = PIX_STATE_EDITOR;
                    pix_drawEditor();
                }
            } else if (c >= 0x20 && c <= 0x7E) { // Printable ASCII
                if (g_pix_filename_len < sizeof(g_pix_filename_buffer) - 1) {
                    g_pix_filename_buffer[g_pix_filename_len] = c;
                    g_pix_filename_len++;
                    g_pix_filename_buffer[g_pix_filename_len] = 0;
                    pix_drawSavePrompt();
                }
            }
            g_pix_cursor_visible = true;
            g_pix_last_blink = millis();
            
        } else {
            // --- We are in the "Load File" list ---
            if (c == KEY_UP_ARROW) {
                if (g_pix_file_selection > 0) {
                    int old_selection = g_pix_file_selection;
                    g_pix_file_selection--;
                    if (g_pix_file_selection < g_pix_file_top_index) {
                        g_pix_file_top_index = g_pix_file_selection;
                        pix_drawFileSelect();
                    } else {
                        pix_drawFileSelectItem(old_selection);
                        pix_drawFileSelectItem(g_pix_file_selection);
                    }
                }
            } else if (c == KEY_DOWN_ARROW) {
                if (g_pix_file_selection < g_pix_file_count - 1) {
                    int old_selection = g_pix_file_selection;
                    g_pix_file_selection++;
                    
                    int y_pos = 20;
                    int item_height = LINE_HEIGHT + 4;
                    int footer_y = SCREEN_HEIGHT - (LINE_HEIGHT * 2) - 2;
                    int max_visible_items = (footer_y - y_pos) / (item_height + 2);
                    if (g_pix_file_selection >= g_pix_file_top_index + max_visible_items) {
                        g_pix_file_top_index++;
                        pix_drawFileSelect();
                    } else {
                        pix_drawFileSelectItem(old_selection);
                        pix_drawFileSelectItem(g_pix_file_selection);
                    }
                }
            } else if (c == KEY_ESCAPE) {
                g_pix_state = PIX_STATE_MAIN_MENU;
                pix_drawMainMenu();
            } else if (c == 0x0D) { // Enter
                if (g_pix_file_count > 0 && g_pix_file_selection < g_pix_file_count) {
                    String filename = g_pix_file_list[g_pix_file_selection];
                    pix_initEditor(-1, filename);
                }
            }
        }
    }
    else if (g_pix_state == PIX_STATE_EDITOR) {
        tft.startWrite();
        int old_cursor_x = g_pix_cursor_x;
        int old_cursor_y = g_pix_cursor_y;

        pix_drawCursor(old_cursor_x, old_cursor_y, false); // Erase old cursor

        // --- THIS IS THE REAL FIX (MOVED UP) ---
        // Force visibility and reset blink *before* any logic.
        g_pix_cursor_visible = true;
        g_pix_last_blink = millis();
        // --- END FIX ---
        
        bool stateChanged = false; // <-- NEW FLAG

        // --- HANDLE MOVEMENT / ACTIONS ---
        if (c == KEY_UP_ARROW) {
            g_pix_cursor_y--;
            if (g_pix_cursor_y < 0) g_pix_cursor_y = g_pix_canvas_height - 1;
        } else if (c == KEY_DOWN_ARROW) {
            g_pix_cursor_y++;
            if (g_pix_cursor_y >= g_pix_canvas_height) {
                // --- NAVIGATE TO FOOTER ---
                g_pix_cursor_y = g_pix_canvas_height - 1;
                g_pix_state = PIX_STATE_EDITOR_FOOTER;
                g_pix_footer_selection = 0;
                stateChanged = true; // <-- SET FLAG
                // (REMOVED return)
            }
        } else if (c == KEY_LEFT_ARROW) {
            g_pix_cursor_x--;
            if (g_pix_cursor_x < 0) g_pix_cursor_x = g_pix_canvas_width - 1;
        } else if (c == KEY_RIGHT_ARROW) {
            g_pix_cursor_x++;
            if (g_pix_cursor_x >= g_pix_canvas_width) g_pix_cursor_x = 0;
        } else if (c == 0x0D) { // Enter
            g_pix_canvas[g_pix_cursor_y * g_pix_canvas_width + g_pix_cursor_x] = g_pix_current_color;
            pix_drawPixel(g_pix_cursor_x, g_pix_cursor_y); // Surgical draw
        } else if (c == ' ') { // Space bar
            g_pix_state = PIX_STATE_PALETTE;
            stateChanged = true; // <-- SET FLAG
            // (REMOVED return)
        } else if (c == 's' || c == 'S') { // 'S' for Save
            g_pix_last_state = PIX_STATE_EDITOR;
            g_pix_state = PIX_STATE_FILE_SELECT;
            g_pix_filename_len = 0;
            g_pix_filename_buffer[0] = 0;
            stateChanged = true; // <-- SET FLAG
            // (REMOVED return)
        } else if (c == KEY_ESCAPE) {
            g_pix_state = PIX_STATE_MAIN_MENU;
            stateChanged = true; // <-- SET FLAG
            // (REMOVED return)
        }
        
        // --- NEW DRAW LOGIC ---
        if (stateChanged) {
            // A state change occurred. End this transaction and draw the new screen.
            tft.endWrite(); 
            
            if (g_pix_state == PIX_STATE_EDITOR_FOOTER) {
                pix_drawEditor(); // This redraws the footer with focus
            } else if (g_pix_state == PIX_STATE_PALETTE) {
                pix_drawPalette();
            } else if (g_pix_state == PIX_STATE_FILE_SELECT) {
                pix_drawSavePrompt();
            } else if (g_pix_state == PIX_STATE_MAIN_MENU) {
                pix_drawMainMenu();
            }
            return; // Now we return, *after* handling the new screen draw

        } else {
            // No state change, just draw the new cursor
            pix_drawCursor(g_pix_cursor_x, g_pix_cursor_y, true);
        }

        tft.endWrite();
    }
    else if (g_pix_state == PIX_STATE_EDITOR_FOOTER) {
        if (c == KEY_UP_ARROW) {
            g_pix_state = PIX_STATE_EDITOR;

            // --- THIS IS THE FIX ---
            g_pix_cursor_visible = true;
            g_pix_last_blink = millis();
            tft.startWrite();
            pix_drawEditorFooter(); // Redraw footer to hide focus
            pix_drawCursor(g_pix_cursor_x, g_pix_cursor_y, true);
            tft.endWrite();
            // --- END FIX ---

        } else if (c == KEY_DOWN_ARROW) {
            // Wrap to top of grid
            g_pix_state = PIX_STATE_EDITOR;
            g_pix_cursor_y = 0;

            // --- THIS IS THE FIX ---
            g_pix_cursor_visible = true;
            g_pix_last_blink = millis();
            tft.startWrite();
            pix_drawEditorFooter(); // Redraw footer to hide focus
            pix_drawCursor(g_pix_cursor_x, g_pix_cursor_y, true);
            tft.endWrite();
            // --- END FIX ---

        } else if (c == KEY_LEFT_ARROW) {
            g_pix_footer_selection--;
            if (g_pix_footer_selection < 0) g_pix_footer_selection = 2;
            tft.startWrite();
            pix_drawEditorFooter(); // Redraw footer for blink
            tft.endWrite();
        } else if (c == KEY_RIGHT_ARROW) {
            g_pix_footer_selection++;
            if (g_pix_footer_selection > 2) g_pix_footer_selection = 0;
            tft.startWrite();
            pix_drawEditorFooter(); // Redraw footer for blink
            tft.endWrite();
        } else if (c == KEY_ESCAPE) {
            g_pix_state = PIX_STATE_EDITOR;
            
            // --- THIS IS THE FIX ---
            g_pix_cursor_visible = true;
            g_pix_last_blink = millis();
            tft.startWrite();
            pix_drawEditorFooter(); // Redraw footer to hide focus
            pix_drawCursor(g_pix_cursor_x, g_pix_cursor_y, true);
            tft.endWrite();
            // --- END FIX ---

        } else if (c == 0x0D) { // Enter
            // --- Activate Footer Button ---
            if (g_pix_footer_selection == 0) { // Save
                g_pix_last_state = PIX_STATE_EDITOR;
                g_pix_state = PIX_STATE_FILE_SELECT;
                g_pix_filename_len = 0;
                g_pix_filename_buffer[0] = 0;
                pix_drawSavePrompt();
            } else if (g_pix_footer_selection == 1) { // Palette
                g_pix_state = PIX_STATE_PALETTE;
                pix_drawPalette();
            } else if (g_pix_footer_selection == 2) { // Exit
                g_pix_state = PIX_STATE_MAIN_MENU;
                pix_drawMainMenu();
            }
        }
    }
    else if (g_pix_state == PIX_STATE_PALETTE) {
        int old_selection = g_pix_palette_selection;
        int items_per_row = 8;
        bool selection_changed = false;
        bool palette_changed = false;
        if (c == KEY_UP_ARROW) {
            g_pix_palette_selection -= items_per_row;
            if (g_pix_palette_selection < 0) g_pix_palette_selection += PIX_PALETTE_COLOR_COUNT;
            selection_changed = true;
        } else if (c == KEY_DOWN_ARROW) {
            g_pix_palette_selection += items_per_row;
            if (g_pix_palette_selection >= PIX_PALETTE_COLOR_COUNT) g_pix_palette_selection -= PIX_PALETTE_COLOR_COUNT;
            selection_changed = true;
        } else if (c == KEY_LEFT_ARROW) {
            int old_col = old_selection % items_per_row;
            if (old_col > 0) {
                // Move left within palette
                g_pix_palette_selection--;
                selection_changed = true;
            } else {
                // Wrap to previous palette
                int row = old_selection / items_per_row;
                g_pix_palette_index = (g_pix_palette_index - 1 + PIX_PALETTE_SET_COUNT) % PIX_PALETTE_SET_COUNT;
                g_pix_palette_selection = (row * items_per_row) + (items_per_row - 1);
                palette_changed = true;
            }
        
        } else if (c == KEY_RIGHT_ARROW) { 
            int old_col = old_selection % items_per_row;
            if (old_col < items_per_row - 1) {
                // Move right within palette
                g_pix_palette_selection++;
                selection_changed = true;
            } else {
                // Wrap to next palette
                int row = old_selection / items_per_row;
                g_pix_palette_index = (g_pix_palette_index + 1) % PIX_PALETTE_SET_COUNT;
                g_pix_palette_selection = (row * items_per_row) + 0;
                palette_changed = true;
            }

        } else if (c == 0x09) { // 0x09 is TAB
            // Cycle to next palette, keep selection index
            g_pix_palette_index = (g_pix_palette_index + 1) % PIX_PALETTE_SET_COUNT;
            palette_changed = true;

        } else if (c == KEY_ESCAPE) {
            g_pix_state = PIX_STATE_EDITOR;
            pix_drawEditor();
        
        } else if (c == 0x0D || c == ' ') { // Enter or Spacebar
            g_pix_current_color = PIX_PALETTES[g_pix_palette_index][g_pix_palette_selection];
            g_pix_state = PIX_STATE_EDITOR;
            pix_drawEditor(); // Full redraw to return
        }
        
        if (palette_changed) {
            pix_drawPalette();
        } else if (selection_changed && old_selection != g_pix_palette_selection) {
            pix_drawPaletteCursor(old_selection, false);
            pix_drawPaletteCursor(g_pix_palette_selection, true);
        }
    }
}
CursorDrawContext getCursorDrawContext(bool cursorVisibleState) {
    CursorDrawContext ctx;

    // --- 1. Get Preview String ---
    const int ALPHA_CASE_KEY_INDEX = (int)strlen(alphaChars) + 1;
    const int NUM_ALPHA_KEY_INDEX = (int)strlen(numberChars) + 1;
    const int SYM_ALPHA_KEY_INDEX = (int)strlen(symbolChars) + 1;
    
    if (kbIndex == 0) { ctx.preview = kbGetModeName(); }
    else if (kmode == ALPHA) {
        if (kbIndex <= (int)strlen(alphaChars)) { ctx.preview = String(alphaChars[kbIndex - 1]); }
        else if (kbIndex == ALPHA_CASE_KEY_INDEX) { ctx.preview = "[SPACE]"; }
        else if (kbIndex == ALPHA_CASE_KEY_INDEX + 1) { ctx.preview = "[ENTER]"; }
        else if (kbIndex == ALPHA_CASE_KEY_INDEX + 2) { ctx.preview = "[CASE]"; }
    }
    else if (kmode == ALPHA_LOWER) {
         if (kbIndex <= (int)strlen(alphaLowerChars)) { ctx.preview = String(alphaLowerChars[kbIndex - 1]); }
         else if (kbIndex == ALPHA_CASE_KEY_INDEX) { ctx.preview = "[SPACE]"; }
         else if (kbIndex == ALPHA_CASE_KEY_INDEX + 1) { ctx.preview = "[ENTER]"; }
         else if (kbIndex == ALPHA_CASE_KEY_INDEX + 2) { ctx.preview = "[case]"; }
    }
    else if (kmode == NUM) {
        if (kbIndex <= (int)strlen(numberChars)) { ctx.preview = String(numberChars[kbIndex - 1]); }
        else if (kbIndex == NUM_ALPHA_KEY_INDEX) { ctx.preview = "[SPACE]"; }
        else if (kbIndex == NUM_ALPHA_KEY_INDEX + 1) { ctx.preview = "[ENTER]"; }
    }
    else if (kmode == SYM) {
        if (kbIndex <= (int)strlen(symbolChars)) { ctx.preview = String(symbolChars[kbIndex - 1]); }
        else if (kbIndex == SYM_ALPHA_KEY_INDEX) { ctx.preview = "[SPACE]"; }
        else if (kbIndex == SYM_ALPHA_KEY_INDEX + 1) { ctx.preview = "[ENTER]"; }
    }
    else if (kmode == CTRL) {
        if (kbIndex > 0 && kbIndex <= CTRL_COUNT) { ctx.preview = "[" + ctrlKeys[kbIndex - 1] + "]"; }
    }
    else if (kbIndex == CTRL_COUNT + 1) {
            ctx.preview = "[FUNC]";
    }
    else { ctx.preview = "?"; }

    // --- 2. Get Mode Color ---
    uint16_t modeTextColor = ST77XX_WHITE;
    if (kmode == ALPHA || kmode == ALPHA_LOWER) modeTextColor = ST77XX_CYAN;
    else if (kmode == NUM) modeTextColor = ST77XX_GREEN;
    else if (kmode == SYM) modeTextColor = ST77XX_MAGENTA;
    else if (kmode == CTRL) modeTextColor = ST77XX_DARK_ORANGE;
    
    // --- 3. Get Final Colors ---
    bool isModeLabel = (kbIndex == 0);
    if (cursorVisibleState) {
        ctx.bgColor = isModeLabel ? modeTextColor : ST77XX_WHITE;
        ctx.fgColor = ST77XX_BLACK;
    } else {
        ctx.bgColor = ST77XX_BLACK;
        ctx.fgColor = isModeLabel ? modeTextColor : ST77XX_WHITE;
    }

    return ctx;
}
String gem_escape_notes(String s) {
    s.replace("|", "\\p");
    s.replace("\n", "\\n");
    return s;
}
String gem_unescape_notes(String s) {
    s.replace("\\p", "|");
    s.replace("\\n", "\n");
    return s;
}
String gem_join_notes_from_editor(TextArea* editor) {
    if (!editor) return "";
    String full_notes = "";
    for (int i = 0; i < editor->lineCount; i++) {
        full_notes += editor->lines[i];
        if (i < editor->lineCount - 1) {
            full_notes += "\n";
        }
    }
    return full_notes;
}
void gem_load_notes_to_editor(TextArea* editor, String notes_string) {
    if (!editor) return;
    
    editor->lineCount = 0;
    int current = 0;
    int next = -1;
    
    if (notes_string.length() == 0) {
        editor->lines[0] = "";
        editor->lineCount = 1;
        return;
    }

    do {
        next = notes_string.indexOf('\n', current);
        String line;
        
        if (next == -1) {
            line = notes_string.substring(current);
        } else {
            line = notes_string.substring(current, next);
        }
        current = next + 1;

        if (editor->lineCount < editor->maxLines) {
            editor->lines[editor->lineCount++] = line;
        } else {
            break; // Editor is full
        }

    } while (next != -1 && editor->lineCount < editor->maxLines);
    
    if (editor->lineCount == 0) {
        editor->lines[0] = "";
        editor->lineCount = 1;
    }
}
bool gem_cycle_quest_state(int db_index) {
    if (db_index < 0 || db_index >= g_questCount) return false;
    Quest& quest = g_questDB[db_index];
    QuestState nextState; 
    
    if (quest.questState == STATE_UNCOMPLETED) {
        // --- 1. UNCOMPLETED -> STARTED ---
        quest.questState = nextState = STATE_STARTED;
        quest.timerStartTime = millis(); // START timer
        pushSystemMessage("Quest Started!");
        
    } else if (quest.questState == STATE_STARTED) {
        // --- 2. STARTED -> COMPLETED ---
        gem_update_all_timers();
        if (quest.isRepeatable) {
            // --- REPEATABLE QUEST ---
            if (quest.timerMinutesElapsed < quest.timerDurationMinutes) {
                pushSystemMessage("Timer not complete! Cannot turn in yet.");
                return false; // <-- TELL CALLER TO ABORT
            }
            quest.timerStartTime = 0; // Stop timer
            quest.questState = nextState = STATE_COMPLETED;
            pushSystemMessage("Quest ready to turn in.");
            
        } else {
            // --- NORMAL QUEST ---
            quest.timerStartTime = 0; // Stop timer
            quest.questState = nextState = STATE_COMPLETED;
            
            // MAIN XP/GEM REWARD
            const float BONUS_MULTIPLIER = 0.1;
            int bonusGems = 0;
            if (quest.timerMinutesElapsed < quest.timerDurationMinutes) {
                unsigned long minutesSaved = quest.timerDurationMinutes - quest.timerMinutesElapsed;
                bonusGems = (int)(minutesSaved * BONUS_MULTIPLIER);
            }
            
            quest.gemBonusAwarded = bonusGems;
            g_gem_count += (quest.gemReward + quest.gemBonusAwarded);
            gem_awardXP(quest.xpReward); 
            
            pushSystemMessage("Quest complete! +" + String(quest.gemReward) + 
                              " Gems (+" + String(bonusGems) + " Time Bonus) & +" + String(quest.xpReward) + " XP!");
            
            // AWARD SKILL XP
            float timeHours = (float)quest.timerDurationMinutes / 60.0f;
            int skillXPAward = (int)((quest.difficulty + quest.desire) * timeHours);
            String skillsString = quest.skills;
            if (skillsString.length() > 0 && skillXPAward > 0) {
                int current = 0;
                int next = -1;
                String skillName;
                do {
                    next = skillsString.indexOf(' ', current);
                    if (next == -1) {
                        skillName = skillsString.substring(current);
                    } else {
                        skillName = skillsString.substring(current, next);
                    }
                    current = next + 1;
                    skillName.trim();
                    
                    if (skillName.length() > 0) {
                        gem_award_skill_xp(skillName, skillXPAward);
                    }
                } while (next != -1);
            }
        }

    } else if (quest.questState == STATE_COMPLETED) {
        // --- 3. COMPLETED -> UNCOMPLETED ---
        
        if (quest.isRepeatable) {
            // --- REPEATABLE: This is the "Turn In" step ---
            quest.questState = nextState = STATE_UNCOMPLETED;
            quest.timerStartTime = 0; 
            quest.timerMinutesElapsed = 0; // Reset timer
            quest.gemBonusAwarded = 0; // No bonus for repeatables

            // --- THIS IS THE CRUSADE FIX ---
            if (quest.type == QT_CRUSADE && quest.addCycleMinutes > 0) {
                quest.timerDurationMinutes += quest.addCycleMinutes;
                pushSystemMessage("Crusade extended by " + String(quest.addCycleMinutes) + "m!");
            }
            // --- END OF FIX ---
            
            // Award standard Gems and XP
            g_gem_count += quest.gemReward;
            gem_awardXP(quest.xpReward); 
            
            pushSystemMessage("Quest Turned In! +" + String(quest.gemReward) + 
                              " Gems & +" + String(quest.xpReward) + " XP!");
            
            // Award Skill XP
            float timeHours = (float)quest.timerDurationMinutes / 60.0f;
            int skillXPAward = (int)((quest.difficulty + quest.desire) * timeHours);
            String skillsString = quest.skills;
            if (skillsString.length() > 0 && skillXPAward > 0) {
                int current = 0;
                int next = -1;
                String skillName;
                do {
                    next = skillsString.indexOf(' ', current);
                    if (next == -1) { skillName = skillsString.substring(current); }
                    else { skillName = skillsString.substring(current, next); }
                    current = next + 1;
                    skillName.trim();
                    if (skillName.length() > 0) {
                        gem_award_skill_xp(skillName, skillXPAward);
                    }
                } while (next != -1);
            }

        } else {
            // --- NORMAL: This is the "Un-complete" step ---
            quest.questState = nextState = STATE_UNCOMPLETED;
            quest.timerStartTime = 0; 
            quest.timerMinutesElapsed = 0; // Reset timer progress
            
            // --- NEW FIX: Calculate gems to subtract, but cap at 0 ---
            int gemsToSubtract = quest.gemReward + quest.gemBonusAwarded;
            int gemsLost = 0;

            if (g_gem_count < gemsToSubtract) {
                // Only subtract what the player has
                gemsLost = g_gem_count;
                g_gem_count = 0;
            } else {
                // Player has enough, subtract the full amount
                gemsLost = gemsToSubtract;
                g_gem_count -= gemsToSubtract;
            }
            // --- END FIX ---
            
            // Subtract all XP (this is already handled correctly)
            gem_awardXP(-quest.xpReward); 
            
            // Update system message to reflect actual gems lost
            pushSystemMessage("Quest reset! -" + String(gemsLost) + " Gems & -" + String(quest.xpReward) + " XP.");
            quest.gemBonusAwarded = 0;

            // SUBTRACT SKILL XP
            float timeHours = (float)quest.timerDurationMinutes / 60.0f;
            int skillXPAward = (int)((quest.difficulty + quest.desire) * timeHours);
            String skillsString = quest.skills;
            if (skillsString.length() > 0 && skillXPAward > 0) {
                int current = 0;
                int next = -1;
                String skillName;
                do {
                    next = skillsString.indexOf(' ', current);
                    if (next == -1) { skillName = skillsString.substring(current); }
                    else { skillName = skillsString.substring(current, next); }
                    current = next + 1;
                    skillName.trim();
                    if (skillName.length() > 0) {
                        gem_award_skill_xp(skillName, -skillXPAward);
                    }
                } while (next != -1);
            }
        }
    }
    
    // --- 4. SORT AND SAVE ---
    gem_sortQuests();
    gem_save_database(g_gem_db_filename);
    return true; // <-- RETURN SUCCESS
}
void gem_drawHeader(bool cursorVisible) {
    // --- 1-LINE HEADER ---
    const int HEADER_HEIGHT = LINE_HEIGHT + 2;
    uint16_t bgColor = ST77XX_GREEN;
    uint16_t titleFgColor = ST77XX_BLACK;
    uint16_t gemFgColor = ST77XX_MAGENTA;
    if (g_gemFocus == GEM_FOCUS_HEADER && cursorVisible) {
        bgColor = ST77XX_CYAN;
        titleFgColor = ST77XX_BLACK;
        gemFgColor = ST77XX_BLACK;   
    }
    
    // 1. Draw Header Background
    tft.fillRect(0, 0, SCREEN_WIDTH, HEADER_HEIGHT, bgColor);
    int y_pos = 2; // Y-padding

    // --- 1. Title (Left-aligned) ---
    String title = "ERROR";
    if (g_gemState == GEM_STATE_QUEST_LIST) {
        title = "QUESTS";
    } else if (g_gemState == GEM_STATE_CREATE_QUEST) {
        // --- THIS IS THE FIX ---
        if (g_gem_currentQuestID == -1) {
            // Creating a new quest
            title = (g_gem_editor_questType == QT_CRUSADE) ? "NEW CRUSADE" : "NEW QUEST";
        } else {
            // Editing an existing quest
            title = (g_gem_editor_questType == QT_CRUSADE) ? "EDIT QUEST" : "EDIT QUEST";
        }
    } else if (g_gemState == GEM_STATE_TYPE_SELECT) {
        title = "QUEST TYPE";                  
    
    } else if (g_gemState == GEM_STATE_STATS) {
        title = "PLAYER STATS";

    } else if (g_gemState == GEM_STATE_MERCHANT) {
        title = "MERCHANT";
    } else if (g_gemState == GEM_STATE_MERCHANT_EDIT) {
        title = (g_merchant_currentItemID == -1) ? "NEW ITEM" : "EDIT ITEM";
    }
    else if (g_gemState == GEM_STATE_INVENTORY) {
        title = "INVENTORY";
    }
    else if (g_gemState == GEM_STATE_INVENTORY_VIEW) {
        title = "VIEW ITEM";
    }
    tft.setTextColor(titleFgColor, bgColor);
    tft.setCursor(5, y_pos);
    tft.print(title);

    // --- RIGHT-ALIGNED BLOCK (Lvl, XP, Gems) ---
    // We calculate positions from right-to-left

    // --- ITEM 1: Gems (Right-most) ---
    String gemCountStr = "GEMS:" + String(g_gem_count);
    int gemX = SCREEN_WIDTH - (gemCountStr.length() * CHAR_WIDTH) - 5;
    tft.setCursor(gemX, y_pos);
    tft.setTextColor(gemFgColor, bgColor);
    tft.print(gemCountStr);

    // --- ITEM 2: XP Bar (Left of Gems) ---
    String xpLabel = "XP:";
    int bar_width_chars = 6; // 6-char wide bar
    int bar_pixel_width = bar_width_chars * CHAR_WIDTH;
    int bar_h = LINE_HEIGHT - 4;
    
    // Calculate X positions for the XP block
    int bar_draw_x = gemX - bar_pixel_width - 4;
    int label_x = bar_draw_x - (xpLabel.length() * CHAR_WIDTH) - 2;
    
    // --- ITEM 3: Lvl (Left of XP Bar) ---
    String levelStr = "LVL:" + String(g_gem_level);
    int lvlX = label_x - (levelStr.length() * CHAR_WIDTH) - 4;
    
    // --- Now draw Lvl and XP in the correct order ---
    tft.setTextColor(titleFgColor, bgColor);
    tft.setCursor(lvlX, y_pos);
    tft.print(levelStr);
    
    tft.setCursor(label_x, y_pos);
    tft.print(xpLabel);
    
    // Calculate XP percentage
    unsigned long xp_start_of_level = gem_getXPForLevel(g_gem_level);
    unsigned long xp_for_next_level = gem_getXPForLevel(g_gem_level + 1);
    unsigned long xp_needed = xp_for_next_level - xp_start_of_level;
    unsigned long xp_progress = g_gem_total_xp - xp_start_of_level;
    
    float progressPercent = 0.0;
    if (xp_needed > 0) {
        progressPercent = (float)xp_progress / (float)xp_needed;
    }
    if (progressPercent > 1.0) progressPercent = 1.0;
    if (g_gem_level == 1000) progressPercent = 1.0; // Max level

    int filled_width = (int)(progressPercent * bar_pixel_width);

    // --- ADD THIS LINE ---
    // Draw 1px border around the bar
    tft.drawRect(bar_draw_x - 1, y_pos, bar_pixel_width + 2, bar_h + 2, ST77XX_DARKGREY);
    // --- END ADD ---

    // Draw bar background (empty part)
    tft.fillRect(bar_draw_x, y_pos + 1, bar_pixel_width, bar_h, 0x3186); 
    
    // Draw filled part (Red to Green)
    uint16_t bar_color = hsvToRgb565((int)(progressPercent * 120), 255, 255);
    tft.fillRect(bar_draw_x, y_pos + 1, filled_width, bar_h, bar_color);
}
void gem_drawFooter(bool cursorVisible) {
    int startY = SCREEN_HEIGHT - LINE_HEIGHT;

    // --- Standard Footer Logic (2 or 3 buttons) ---
    String opt1, opt2, opt3;
    bool has_third_button = false;

    if (g_gemState == GEM_STATE_QUEST_LIST) {
        opt1 = " Profile";
        opt2 = " Merchant";
        opt3 = " Exit";
        has_third_button = true;
    } else if (g_gemState == GEM_STATE_STATS) {
        // --- THIS BLOCK IS MODIFIED ---
        opt1 = " Inventory";
        opt2 = "Settings";
        opt3 = " Exit";
        has_third_button = true;
        // --- END OF MODIFICATION ---
    } else if (g_gemState == GEM_STATE_TYPE_SELECT) {
        opt1 = " New Type";
        opt2 = " Back";
    } else if (g_gemState == GEM_STATE_MERCHANT) {
        opt1 = " New Item";
        opt2 = " Exit";
    } else if (g_gemState == GEM_STATE_MERCHANT_EDIT) {
        opt1 = " Save";
        opt2 = " Back";
        if (g_merchant_currentItemID != -1) { // Only show "Delete" in Edit mode
            opt3 = " Delete";
            has_third_button = true;
        }
    }
    // --- ADD THESE NEW STATES ---
    else if (g_gemState == GEM_STATE_INVENTORY) {
        opt1 = " Back";
        opt2 = " Exit";
    }
    else if (g_gemState == GEM_STATE_INVENTORY_VIEW) {
        opt1 = " Sell";
        opt2 = " Back";
        opt3 = " Delete";
        has_third_button = true;
    }
    // --- END OF ADDITION ---
    else { // GEM_STATE_CREATE_QUEST
        opt1 = " Save";
        opt2 = " Back";
        if (g_gem_currentQuestID != -1) { // Only show "Delete" in Edit mode
            opt3 = " Delete";
            has_third_button = true;
        }
    }

    if (has_third_button) {
        // --- Draw 3 Buttons ---
        int w1 = (SCREEN_WIDTH * 2) / 5; // 40%
        int w2 = (SCREEN_WIDTH * 2) / 5; // 40%
        int w3 = SCREEN_WIDTH - w1 - w2; // 20%
        int x2 = w1;
        int x3 = w1 + w2;
        // Colors for 0
        uint16_t bg1 = (g_gemFocus == GEM_FOCUS_FOOTER && g_gem_footer_selection == 0 && cursorVisible) ? ST77XX_CYAN : ST77XX_GREEN;
        // Colors for 1
        uint16_t bg2 = (g_gemFocus == GEM_FOCUS_FOOTER && g_gem_footer_selection == 1 && cursorVisible) ? ST77XX_CYAN : ST77XX_GREEN;
        // Colors for 2
        uint16_t bg3 = (g_gemFocus == GEM_FOCUS_FOOTER && g_gem_footer_selection == 2 && cursorVisible) ? ST77XX_CYAN : ST77XX_RED;
        uint16_t fg3 = (g_gemFocus == GEM_FOCUS_FOOTER && g_gem_footer_selection == 2 && cursorVisible) ? ST77XX_BLACK : ST77XX_WHITE;
        
        // --- Special Case for Inventory View Delete ---
        if (g_gemState == GEM_STATE_INVENTORY_VIEW) {
            bg1 = (g_gemFocus == GEM_FOCUS_FOOTER && g_gem_footer_selection == 0 && cursorVisible) ? ST77XX_CYAN : ST77XX_YELLOW; // Sell
            fg3 = (g_gemFocus == GEM_FOCUS_FOOTER && g_gem_footer_selection == 2 && cursorVisible) ? ST77XX_BLACK : ST77XX_WHITE; // Delete
        }

        // Draw 1
        tft.fillRect(0, startY, w1, LINE_HEIGHT, bg1);
        tft.setTextColor(ST77XX_BLACK, bg1);
        tft.setCursor(0, startY + 1);
        tft.print(opt1);
        // Draw 2
        tft.fillRect(x2, startY, w2, LINE_HEIGHT, bg2);
        tft.setTextColor(ST77XX_BLACK, bg2);
        tft.setCursor(x2, startY + 1);
        tft.print(opt2);
        // Draw 3
        tft.fillRect(x3, startY, w3, LINE_HEIGHT, bg3);
        tft.setTextColor(fg3, bg3);
        tft.setCursor(x3, startY + 1);
        tft.print(opt3);
        
    } else {
        // --- Draw 2 Buttons (Unchanged) ---
        int halfWidth = SCREEN_WIDTH / 2;
        uint16_t bg1 = (g_gemFocus == GEM_FOCUS_FOOTER && g_gem_footer_selection == 0 && cursorVisible) ? ST77XX_CYAN : ST77XX_GREEN;
        uint16_t fg1 = ST77XX_BLACK;
        uint16_t bg2 = (g_gemFocus == GEM_FOCUS_FOOTER && g_gem_footer_selection == 1 && cursorVisible) ? ST77XX_CYAN : ST77XX_GREEN; 
        uint16_t fg2 = ST77XX_BLACK;

        tft.fillRect(0, startY, halfWidth, LINE_HEIGHT, bg1);
        tft.setTextColor(fg1, bg1);
        tft.setCursor(0, startY + 1);
        tft.print(opt1);

        tft.fillRect(halfWidth, startY, SCREEN_WIDTH - halfWidth, LINE_HEIGHT, bg2);
        tft.setTextColor(fg2, bg2);
        tft.setCursor(halfWidth, startY + 1);
        tft.print(opt2);
    }
}
void gem_drawAllProgressBars() {
    // We do not use tft.startWrite() here because this is usually called 
    // from inside an existing transaction in the main loop or gem_update.
    const int BAR_HEIGHT = 4;
    int footerY = SCREEN_HEIGHT - LINE_HEIGHT; // Do not draw over footer
    
    // Start from top visible item. Iterate <= g_questCount to include the last item.
    for (int i = g_gem_list_top_item; i <= g_questCount; i++) {
        
        // 1. Skip Index 0 ([New Quest] has no bar)
        if (i == 0) continue;

        // 2. Get screen position
        int y_pos = gem_getQuestItemY(i);
        // Stop if scrolled off bottom
        if (y_pos == -1 || y_pos >= footerY) break;

        // 3. Map List Index to Database Index
        int db_index = i - 1;

        // 4. Get dimensions for THIS specific item
        int height = gem_getQuestItemHeight(i);
        int bar_y = y_pos + height - 5;

        Quest& quest = g_questDB[db_index];
        
        float progressPercent = 0.0;
        if (quest.questState == STATE_COMPLETED) {
            progressPercent = 1.0;
        } else {
            if (quest.timerDurationMinutes > 0) {
                float current = (float)quest.timerMinutesElapsed;
                if (quest.questState == STATE_STARTED && quest.timerStartTime != 0) 
                    current += (float)(millis() - quest.timerStartTime) / 60000.0f;
                progressPercent = current / (float)quest.timerDurationMinutes;
            }
        }

        if (progressPercent > 1.0) progressPercent = 1.0;
        if (progressPercent < 0.0) progressPercent = 0.0;

        int bar_x = 5;
        int bar_width = SCREEN_WIDTH - 10;
        
        if (quest.iconFile.length() > 0) {
            // FIX: Shifted to 44 to match new Icon Mode alignment
            bar_x = 44; 
            bar_width = SCREEN_WIDTH - 44 - 5; 
        }

        int filled_width = (int)(progressPercent * bar_width);

        // Draw Track (0x3186 matches theme)
        tft.fillRect(bar_x, bar_y, bar_width, BAR_HEIGHT, 0x3186);

        // Draw Filled Portion (Color)
        uint16_t bar_color = hsvToRgb565((int)(progressPercent * 120), 255, 255);
        if (filled_width > 0) {
            tft.fillRect(bar_x, bar_y, filled_width, BAR_HEIGHT, bar_color);
        }
    }
}
void gem_drawListItem(int item_selection_index, bool drawBar) { 
    int y_pos = gem_getQuestItemY(item_selection_index);
    if (y_pos == -1) return; // Off screen

    int height = gem_getQuestItemHeight(item_selection_index);
    if (item_selection_index < 0) return;

    // --- [New Quest] (Index 0) ---
    if (item_selection_index == 0) {
        uint16_t fg, bg;
        if (g_gem_list_selection == 0 && g_gemFocus == GEM_FOCUS_LIST) {
            fg = ST77XX_BLACK;
            bg = ST77XX_CYAN; g_gem_marquee_active = true; 
        } else {
            fg = ST77XX_GREEN;
            bg = ST77XX_DARKGREY; 
        }
        
        // Draw Background (5px margin Left/Right)
        tft.fillRect(5, y_pos, SCREEN_WIDTH - 10, LINE_HEIGHT + 3, bg);
        
        // Calculate Text Start (5px Margin + 6px Padding)
        int start_x = 5 + CHAR_WIDTH;

        tft.setCursor(start_x, y_pos + 3); 
        tft.setTextColor(fg, bg);
        
        if (g_gem_list_selection == 0 && g_gemFocus == GEM_FOCUS_LIST) {
            String fullText = "[New Quest] - Create a new task";
            String scrollText = fullText + "   "; 
            if (g_gem_marquee_offset >= scrollText.length()) g_gem_marquee_offset = 0;
            String marquee = scrollText.substring(g_gem_marquee_offset) + 
                             scrollText.substring(0, g_gem_marquee_offset);
            
            // Width Calculation: Screen - StartX - RightMargin(5)
            int available_pixels = SCREEN_WIDTH - start_x - 5;
            int max_chars = available_pixels / CHAR_WIDTH + 1;
            
            tft.print(marquee.substring(0, max_chars)); 
        } else {
            tft.print("[New Quest]");
        }
        return;
    } 

    // --- Quest Item (Index > 0) ---
    int db_index = item_selection_index - 1;
    if (db_index < 0 || db_index >= g_questCount) return; 
    Quest& quest = g_questDB[db_index];

    // Determine Colors
    uint16_t fg, bg;
    String check = "[?]";
    if (quest.questState == STATE_COMPLETED) { check = "[X]"; fg = ST77XX_GREEN; }
    else if (quest.questState == STATE_STARTED) { check = "[>]"; fg = ST77XX_YELLOW; }
    else { check = "[ ]"; fg = RAINBOW_COLORS[db_index % RAINBOW_COUNT]; }

    bool isSelected = (g_gem_list_selection == item_selection_index && g_gemFocus == GEM_FOCUS_LIST);
    if (isSelected) { fg = ST77XX_BLACK; bg = ST77XX_CYAN; } 
    else { bg = ST77XX_DARKGREY; }

    // --- DRAW CONTENT ---
    if (height >= 34) { 
        // === ICON MODE (34px) ===
        
        // 1. Draw Highlight Background
        // FIX: Shifted from 40 to 44 (4px right) to maintain gap from moved icon
        // Width adjusted: Screen - 44 - 5 (Margin) = Screen - 49
        tft.fillRect(44, y_pos + 2, SCREEN_WIDTH - 49, LINE_HEIGHT + 2, bg);

        // 2. Draw Icon and Border
        // FIX: Icon Bitmap x moved from 2 to 6
        if (quest.iconFile.length() > 0) drawItemBitmap(quest.iconFile, 6, y_pos + 1, 32, 32);
        // FIX: Border x moved from 1 to 5 (Aligns with standard bar start)
        tft.drawRect(5, y_pos, 34, 34, isSelected ? ST77XX_BLACK : ST77XX_WHITE);
        
        // FIX: Text Start moved from 40 to 44
        int textX = 44; 
        
        // 3. Print the Checkbox (Static)
        tft.setCursor(textX, y_pos + 4);
        tft.setTextColor(fg, bg);
        tft.print(check);

        // 4. Calculate space for scrolling text
        int check_width_px = (check.length() + 1) * CHAR_WIDTH; 
        int marquee_x = textX + check_width_px;
        
        int available_pixels = SCREEN_WIDTH - marquee_x - 5;
        int available_cols = available_pixels / CHAR_WIDTH;

        // 5. Move Cursor for Title/Marquee
        tft.setCursor(marquee_x, y_pos + 4);

        // --- SHARED DATA LOGIC ---
        String reward = " (G:" + String(quest.gemReward) + " XP:" + String(quest.xpReward) + ")";
        String timeStr = "";
        
        if (quest.questState == STATE_COMPLETED) {
            timeStr = " (Done)";
        } else {
            float currentProgressMinutes = (float)quest.timerMinutesElapsed;
            if (quest.questState == STATE_STARTED && quest.timerStartTime != 0) 
                currentProgressMinutes += (float)(millis() - quest.timerStartTime) / 60000.0f;
            
            float minutesToShow;
            String timeLabel;

            if (quest.questState == STATE_STARTED) {
                minutesToShow = quest.timerDurationMinutes - currentProgressMinutes;
                timeLabel = " Rem:";
            } else {
                minutesToShow = (float)quest.timerDurationMinutes;
                timeLabel = " Total:";
            }

            if (minutesToShow <= 0) {
                timeStr = timeLabel + " 0m";
            } else if (minutesToShow >= 60) {
                int hours = (int)(minutesToShow / 60.0f);
                int mins = (int)minutesToShow % 60;
                if (mins == 0) timeStr = timeLabel + " " + String(hours) + "h";
                else timeStr = timeLabel + " " + String(hours) + "h" + String(mins) + "m";
            } else {
                int mins = (int)minutesToShow;
                int secs = (int)((minutesToShow - mins) * 60.0f);
                timeStr = timeLabel + " " + String(mins) + "m" + String(secs) + "s";
            }
        }
        // --- END DATA LOGIC ---

        if (isSelected) { 
            String fullText = quest.title + reward + timeStr + " - " + quest.description;
            String scrollText = fullText + "   "; 
            if (g_gem_marquee_offset >= scrollText.length()) g_gem_marquee_offset = 0;
            String marquee = scrollText.substring(g_gem_marquee_offset) + scrollText.substring(0, g_gem_marquee_offset);
            tft.print(marquee.substring(0, available_cols)); 
        } else {
            tft.print(quest.title);
        }

        // Row 2 (Info)
        tft.setCursor(textX, y_pos + 18);
        tft.setTextColor(isSelected ? ST77XX_BLACK : ST77XX_YELLOW, ST77XX_DARKGREY);
        String infoStr = (quest.questState == STATE_COMPLETED) ? "Done!" : "XP:" + String(quest.xpReward);
        tft.print(infoStr);

    } else {
        // === STANDARD MODE (22px) ===
        
        // 1. Draw Highlight Background (5px margin Left/Right)
        tft.fillRect(5, y_pos, SCREEN_WIDTH - 10, LINE_HEIGHT + 2, bg);

        // 2. Set Cursor for Checkbox (Added CHAR_WIDTH padding on Left)
        tft.setCursor(5 + CHAR_WIDTH, y_pos + 2);
        tft.setTextColor(fg, bg);
        tft.print(check);
        
        // 3. Calculate Text Start Position
        int text_x = 5 + CHAR_WIDTH + (check.length() * CHAR_WIDTH);
        
        // 4. Calculate Available Columns for Marquee
        int available_pixels = SCREEN_WIDTH - text_x - 5;
        int available_cols = available_pixels / CHAR_WIDTH;

        tft.setCursor(text_x, y_pos + 2);
        
        // --- SHARED DATA LOGIC (Same as above) ---
        String reward = " (G:" + String(quest.gemReward) + " XP:" + String(quest.xpReward) + ")";
        String timeStr = "";

        if (quest.questState == STATE_COMPLETED) {
            timeStr = " (Done)";
        } else {
            float currentProgressMinutes = (float)quest.timerMinutesElapsed;
            if (quest.questState == STATE_STARTED && quest.timerStartTime != 0) 
                currentProgressMinutes += (float)(millis() - quest.timerStartTime) / 60000.0f;
            
            float minutesToShow;
            String timeLabel;

            if (quest.questState == STATE_STARTED) {
                minutesToShow = quest.timerDurationMinutes - currentProgressMinutes;
                timeLabel = " Rem:";
            } else {
                minutesToShow = (float)quest.timerDurationMinutes;
                timeLabel = " Total:";
            }

            if (minutesToShow <= 0) {
                timeStr = timeLabel + " 0m";
            } else if (minutesToShow >= 60) {
                int hours = (int)(minutesToShow / 60.0f);
                int mins = (int)minutesToShow % 60;
                if (mins == 0) timeStr = timeLabel + " " + String(hours) + "h";
                else timeStr = timeLabel + " " + String(hours) + "h" + String(mins) + "m";
            } else {
                int mins = (int)minutesToShow;
                int secs = (int)((minutesToShow - mins) * 60.0f);
                timeStr = timeLabel + " " + String(mins) + "m" + String(secs) + "s";
            }
        }
        // --- END DATA LOGIC ---

        if (isSelected) { // Marquee
            String fullText = quest.title + reward + timeStr + " - " + quest.description;
            String scrollText = fullText + "   "; 
            if (g_gem_marquee_offset >= scrollText.length()) g_gem_marquee_offset = 0;
            String marquee = scrollText.substring(g_gem_marquee_offset) + scrollText.substring(0, g_gem_marquee_offset);
            tft.print(marquee.substring(0, available_cols)); 
        } else { // Static
            String title = quest.title;
            String fullLine = title + reward; 
            
            if (fullLine.length() > available_cols) {
                fullLine = fullLine.substring(0, available_cols - 3) + "...";
            }
            tft.print(fullLine);
        }
    }

    // --- PROGRESS BAR (Anchored to Bottom) ---
    if (drawBar) {
        int bar_y = y_pos + height - 5;
        float progressPercent = 0.0;
        if (quest.questState == STATE_COMPLETED) progressPercent = 1.0;
        else if (quest.timerDurationMinutes > 0) {
            float current = (float)quest.timerMinutesElapsed;
            if (quest.questState == STATE_STARTED && quest.timerStartTime != 0) 
                current += (float)(millis() - quest.timerStartTime) / 60000.0f;
            progressPercent = current / (float)quest.timerDurationMinutes;
        }
        
        if (progressPercent > 1.0) progressPercent = 1.0;
        if (progressPercent < 0.0) progressPercent = 0.0;

        int bar_x = 5;
        int bar_width = SCREEN_WIDTH - 10;
        
        if (quest.iconFile.length() > 0) {
            // FIX: Shifted to 44 to match new text alignment
            bar_x = 44; 
            bar_width = SCREEN_WIDTH - 44 - 5;
        }

        int filled_width = (int)(progressPercent * bar_width);

        // Draw Track
        tft.fillRect(bar_x, bar_y, bar_width, 4, 0x3186);
        
        // Draw Fill
        uint16_t bar_color = hsvToRgb565((int)(progressPercent * 120), 255, 255);
        if (filled_width > 0) {
            tft.fillRect(bar_x, bar_y, filled_width, 4, bar_color);
        }
    }
}
void gem_drawQuestEditor(bool preserveData = false) { // <-- Added parameter
    kmode = ALPHA;
    kbIndex = 0;

    tft.startWrite();
    tft.fillScreen(ST77XX_DARKGREY); 
    gem_drawHeader(true);

    tft.setTextColor(ST77XX_WHITE, ST77XX_DARKGREY);
    tft.setCursor(5, (LINE_HEIGHT * 2));  tft.print("Title: (1 line max)");
    tft.setCursor(5, (LINE_HEIGHT * 4));  tft.print("Description: (3 lines max)");
    tft.setCursor(5, (LINE_HEIGHT * 8));  tft.print("Skills: (Space-separated, 4 lines max)");
    tft.setCursor(5, (LINE_HEIGHT * 13)); tft.print("Difficulty (1-7):");
    tft.setCursor( (COLS/2) * CHAR_WIDTH, (LINE_HEIGHT * 13)); tft.print("Desire (1-7):");
    
    tft.setCursor(5, (LINE_HEIGHT * 15)); tft.print("Time (Hours, 0-100):"); 
    tft.setCursor( (COLS/2) * CHAR_WIDTH, (LINE_HEIGHT * 15)); tft.print("Time (Mins, 0-100):");

    tft.setCursor( 5, (LINE_HEIGHT * 17));
    if (g_gem_editor_questType == QT_STANDARD) {
        tft.print("Repeatable:");
    } else {
        tft.print("Add Cycle (Mins):");
    }

    // --- Ensure Editors Exist ---
    if (!gemCreateTitleEditor) gemCreateTitleEditor = new TextArea(5, (LINE_HEIGHT * 3), COLS - 2, 1, 2, 40);
    if (!gemCreateDescEditor) gemCreateDescEditor = new TextArea(5, (LINE_HEIGHT * 5), COLS - 2, 3, 50, 500);
    if (!gemCreateSkillsEditor) gemCreateSkillsEditor = new TextArea(5, (LINE_HEIGHT * 9), COLS - 2, 4, 10, 200);
    if (!gemCreateDiffEditor) gemCreateDiffEditor = new TextArea(5, (LINE_HEIGHT * 14), 10, 1, 2, 3);
    if (!gemCreateDesireEditor) gemCreateDesireEditor = new TextArea((COLS/2) * CHAR_WIDTH, (LINE_HEIGHT * 14), 10, 1, 2, 3);
    if (!gemCreateTimeEditor) gemCreateTimeEditor = new TextArea(5, (LINE_HEIGHT * 16), 10, 1, 2, 5);
    if (!gemCreateMinsEditor) gemCreateMinsEditor = new TextArea((COLS/2) * CHAR_WIDTH, (LINE_HEIGHT * 16), 10, 1, 2, 5);
    if (!gemCreateAddCycleEditor) gemCreateAddCycleEditor = new TextArea(5, (LINE_HEIGHT * 18), 10, 1, 2, 5);
    
    // --- DATA LOADING LOGIC (Only runs if NOT preserving data) ---
    if (!preserveData) { // <--- FIX START
        if (g_gem_currentQuestID != -1 && g_gem_currentQuestID < g_questCount) {
            // Editing existing quest
            Quest& quest = g_questDB[g_gem_currentQuestID];
            g_gem_editor_questType = quest.type;
            
            gemCreateTitleEditor->lines[0] = quest.title; gemCreateTitleEditor->lineCount = 1;
            gem_load_notes_to_editor(gemCreateDescEditor, quest.notes);
            gem_load_notes_to_editor(gemCreateSkillsEditor, quest.skills);
            gemCreateDiffEditor->lines[0] = String(quest.difficulty); gemCreateDiffEditor->lineCount = 1;
            gemCreateDesireEditor->lines[0] = String(quest.desire); gemCreateDesireEditor->lineCount = 1;
            
            gemCreateTimeEditor->lines[0] = String(quest.timerDurationMinutes / 60);
            gemCreateTimeEditor->lineCount = 1; 
            gemCreateMinsEditor->lines[0] = String(quest.timerDurationMinutes % 60);
            gemCreateMinsEditor->lineCount = 1;
            
            g_gem_editor_isRepeatable = quest.isRepeatable;
            gemCreateAddCycleEditor->lines[0] = String(quest.addCycleMinutes); gemCreateAddCycleEditor->lineCount = 1;
            g_gem_temp_quest_icon = quest.iconFile; // Load existing icon
        } else {
            // Creating NEW quest (Clear fields)
            gemCreateTitleEditor->lines[0] = ""; gemCreateTitleEditor->lineCount = 1;
            gemCreateDescEditor->lines[0] = ""; gemCreateDescEditor->lineCount = 1;
            gemCreateSkillsEditor->lines[0] = ""; gemCreateSkillsEditor->lineCount = 1;
            gemCreateDiffEditor->lines[0] = ""; gemCreateDiffEditor->lineCount = 1;
            gemCreateDesireEditor->lines[0] = ""; gemCreateDesireEditor->lineCount = 1;
            gemCreateTimeEditor->lines[0] = ""; gemCreateTimeEditor->lineCount = 1;
            gemCreateMinsEditor->lines[0] = ""; gemCreateMinsEditor->lineCount = 1;
            gemCreateAddCycleEditor->lines[0] = ""; gemCreateAddCycleEditor->lineCount = 1;
            g_gem_editor_isRepeatable = (g_gem_editor_questType == QT_CRUSADE);
            g_gem_temp_quest_icon = ""; // Clear icon for new quest
        }
        
        // Reset cursors
        gemCreateTitleEditor->cursorCol = 0; gemCreateTitleEditor->cursorLine = 0;
        gemCreateDescEditor->cursorCol = 0;     gemCreateDescEditor->cursorLine = 0;
        gemCreateSkillsEditor->cursorCol = 0;   gemCreateSkillsEditor->cursorLine = 0;
        gemCreateDiffEditor->cursorCol = 0;     gemCreateDiffEditor->cursorLine = 0;
        gemCreateDesireEditor->cursorCol = 0;   gemCreateDesireEditor->cursorLine = 0;
        gemCreateTimeEditor->cursorCol = 0;     gemCreateTimeEditor->cursorLine = 0;
        gemCreateMinsEditor->cursorCol = 0;     gemCreateMinsEditor->cursorLine = 0;
        gemCreateAddCycleEditor->cursorCol = 0; gemCreateAddCycleEditor->cursorLine = 0;
    } // <--- FIX END (Close preserveData check)

    // Draw Shared Editors
    gemCreateTitleEditor->draw(tft);
    gemCreateDescEditor->draw(tft);
    gemCreateSkillsEditor->draw(tft);
    gemCreateDiffEditor->draw(tft);
    gemCreateDesireEditor->draw(tft);
    gemCreateTimeEditor->draw(tft);
    gemCreateMinsEditor->draw(tft);

    // Draw Type-Specific Field
    if (g_gem_editor_questType == QT_STANDARD) {
        int repeatX = 5;
        int repeatY = (LINE_HEIGHT * 18);
        String text = g_gem_editor_isRepeatable ? "[ON]" : "[OFF]";
        int textWidth = text.length() * CHAR_WIDTH;
        uint16_t repeat_fg = g_gem_editor_isRepeatable ? ST77XX_BLACK : ST77XX_WHITE;
        uint16_t repeat_bg = g_gem_editor_isRepeatable ? ST77XX_GREEN : ST77XX_RED;
        tft.fillRect(repeatX, repeatY - 1, textWidth, LINE_HEIGHT, repeat_bg);
        tft.setCursor(repeatX, repeatY);
        tft.setTextColor(repeat_fg, repeat_bg);
        tft.print(text);
    } else {
        gemCreateAddCycleEditor->draw(tft);
    }

    int iconY = (LINE_HEIGHT * 19);
    tft.setTextColor(ST77XX_WHITE, ST77XX_DARKGREY);
    tft.setCursor(5, iconY); 
    tft.print("Icon:");
    
    int iconBoxX = 40;
    String displayIcon = (g_gem_temp_quest_icon.length() > 0) ? g_gem_temp_quest_icon : "None";
    
    uint16_t imgBg = (g_gemFocus == GEM_FOCUS_CREATE_IMAGE) ? ST77XX_CYAN : ST77XX_BLACK;
    uint16_t imgFg = (g_gemFocus == GEM_FOCUS_CREATE_IMAGE) ? ST77XX_BLACK : ST77XX_WHITE;
    
    tft.fillRect(iconBoxX, iconY, 100, LINE_HEIGHT, imgBg);
    tft.setTextColor(imgFg, imgBg);
    tft.setCursor(iconBoxX + 2, iconY);
    tft.print(displayIcon);

    if (displayIcon != "None") {
        int imgX = SCREEN_WIDTH - 34 - 5;
        int imgY = (LINE_HEIGHT * 16);
        drawItemBitmap(g_gem_temp_quest_icon, imgX, imgY, 32, 32);
        tft.drawRect(imgX, imgY, 32, 32, ST77XX_WHITE);
    }

    // Set focus flags
    gemCreateTitleEditor->isFocused = (g_gemFocus == GEM_FOCUS_CREATE_TITLE);
    gemCreateDescEditor->isFocused  = (g_gemFocus == GEM_FOCUS_CREATE_DESC);
    gemCreateSkillsEditor->isFocused = (g_gemFocus == GEM_FOCUS_CREATE_SKILLS);
    gemCreateDiffEditor->isFocused  = (g_gemFocus == GEM_FOCUS_CREATE_DIFF);
    gemCreateDesireEditor->isFocused = (g_gemFocus == GEM_FOCUS_CREATE_DESIRE);
    gemCreateTimeEditor->isFocused = (g_gemFocus == GEM_FOCUS_CREATE_TIME);
    gemCreateMinsEditor->isFocused = (g_gemFocus == GEM_FOCUS_CREATE_MINS);
    if (gemCreateAddCycleEditor) gemCreateAddCycleEditor->isFocused = (g_gemFocus == GEM_FOCUS_CREATE_ADD_CYCLE);
    
    gem_drawFooter(true);
    tft.endWrite();
}
void gem_refreshCreateQuestView() {
    // 1. Redraw all Text Editors
    if (gemCreateTitleEditor) gemCreateTitleEditor->draw(tft);
    if (gemCreateDescEditor) gemCreateDescEditor->draw(tft);
    if (gemCreateSkillsEditor) gemCreateSkillsEditor->draw(tft);
    if (gemCreateDiffEditor) gemCreateDiffEditor->draw(tft);
    if (gemCreateDesireEditor) gemCreateDesireEditor->draw(tft);
    if (gemCreateTimeEditor) gemCreateTimeEditor->draw(tft);
    if (gemCreateMinsEditor) gemCreateMinsEditor->draw(tft);

    // 2. Redraw Type-Specific Toggle (Repeatable OR Add Cycle)
    if (g_gem_editor_questType == QT_STANDARD) {
        // Draw Repeatable Toggle
        int repeatX = 5;
        int repeatY = (LINE_HEIGHT * 18); 
        String text = g_gem_editor_isRepeatable ? "[ON]" : "[OFF]";
        int textWidth = text.length() * CHAR_WIDTH;
        
        uint16_t repeat_bg, repeat_fg;
        
        // Determine highlight based on focus
        if (g_gemFocus == GEM_FOCUS_CREATE_REPEATABLE) {
            if (g_gem_editor_isRepeatable) {
                repeat_bg = ST77XX_GREEN; 
                repeat_fg = ST77XX_BLACK;
            } else {
                repeat_bg = ST77XX_RED;   
                repeat_fg = ST77XX_WHITE;
            }
        } else {
            repeat_bg = g_gem_editor_isRepeatable ? ST77XX_GREEN : ST77XX_RED;
            repeat_fg = g_gem_editor_isRepeatable ? ST77XX_BLACK : ST77XX_WHITE;
        }
        
        tft.fillRect(repeatX, repeatY - 1, textWidth, LINE_HEIGHT, repeat_bg);
        tft.setCursor(repeatX, repeatY);
        tft.setTextColor(repeat_fg, repeat_bg);
        tft.print(text);

    } else { 
        // QT_CRUSADE: Redraw Add Cycle Editor
        if (gemCreateAddCycleEditor) gemCreateAddCycleEditor->draw(tft);
    }

    // 3. Redraw Image Selector Box
    int iconY = (LINE_HEIGHT * 19);
    int iconBoxX = 40;
    String displayIcon = (g_gem_temp_quest_icon.length() > 0) ? g_gem_temp_quest_icon : "None";
    
    // Determine colors based on focus
    uint16_t imgBg = (g_gemFocus == GEM_FOCUS_CREATE_IMAGE) ? ST77XX_CYAN : ST77XX_BLACK;
    uint16_t imgFg = (g_gemFocus == GEM_FOCUS_CREATE_IMAGE) ? ST77XX_BLACK : ST77XX_WHITE;
    
    tft.fillRect(iconBoxX, iconY, 100, LINE_HEIGHT, imgBg);
    tft.setTextColor(imgFg, imgBg);
    tft.setCursor(iconBoxX + 2, iconY);
    tft.print(displayIcon);
}
void gem_drawQuestList() {
    tft.startWrite();
    tft.fillScreen(ST77XX_DARKGREY); 
    gem_drawHeader(true);

    int startY = (LINE_HEIGHT * 2);
    int footerY = SCREEN_HEIGHT - LINE_HEIGHT;
    
    // Explicitly clear list area
    tft.fillRect(0, startY, SCREEN_WIDTH, footerY - startY, ST77XX_DARKGREY);

    if (g_questCount == 0) {
         gem_drawListItem(0, false);
    } else {
        // Draw visible items dynamically
        for (int i = g_gem_list_top_item; i <= g_questCount; i++) {
            int y = gem_getQuestItemY(i);
            if (y == -1) {
                if (i > g_gem_list_top_item) break; // Stop if off bottom
                continue;
            }
            gem_drawListItem(i, true); // Draw bar initially
        }
    }
    gem_drawFooter(true);
    tft.endWrite();
}
void gem_drawTypeSelectPage() {
    g_gemState = GEM_STATE_TYPE_SELECT;
    // Focus should be set by the caller

    tft.startWrite();
    tft.fillScreen(ST77XX_DARKGREY);
    gem_drawHeader(true);
    tft.endWrite(); // End header transaction

    gem_drawTypeSelectList(); // Draw the list content

    // Start new transaction for footer
    tft.startWrite();
    gem_drawFooter(true);
    tft.endWrite();
}
void gem_drawTypeSelectItem(int item_index) {
    if (item_index < 0 || item_index >= g_quest_type_count) return;

    // --- 1. Calculate Y position ---
    int startY = (LINE_HEIGHT * 3); // Base Y
    int item_height_px = LINE_HEIGHT + 4 + 6; // item + padding (19px)
    
    int screen_list_index = item_index - g_gem_type_select_top_item;
    int y_pos = startY + (screen_list_index * item_height_px);

    // --- 2. Check if visible ---
    int footerY = SCREEN_HEIGHT - LINE_HEIGHT;
    if (y_pos >= footerY) {
        return; // Off-screen
    }

    // --- 3. Start Transaction ---
    tft.startWrite();

    // --- 4. Determine Colors ---
    bool isSelected = (g_gemFocus == GEM_FOCUS_TYPE_SELECT_LIST && item_index == g_gem_type_select_selection);
    uint16_t fg = isSelected ? ST77XX_BLACK : ST77XX_WHITE;
    uint16_t bg = isSelected ? ST77XX_CYAN : ST77XX_DARKGREY;

    // --- 5. Draw the item ---
    tft.fillRect(0, y_pos, SCREEN_WIDTH, LINE_HEIGHT + 4, bg);
    tft.setTextColor(fg, bg);
    tft.setCursor(5, y_pos + 2);
    tft.print(g_quest_type_names[item_index]);

    tft.endWrite();
}
void gem_drawTypeSelectList() {
    // 1. Calculate Y position and clear area
    int startY = (LINE_HEIGHT * 3);
    int footerY = SCREEN_HEIGHT - LINE_HEIGHT;
    tft.fillRect(0, startY, SCREEN_WIDTH, footerY - startY, ST77XX_DARKGREY);

    // 2. Define layout
    int item_height_px = LINE_HEIGHT + 4 + 6; // 19px
    int max_visible_items = (footerY - startY) / item_height_px;

    // 3. Loop and draw visible items
    // (This calls the item-drawer which handles its own transactions)
    for (int i = 0; i < max_visible_items; i++) {
        int item_index = g_gem_type_select_top_item + i;
        if (item_index >= g_quest_type_count) {
            break; // No more items to draw
        }
        gem_drawTypeSelectItem(item_index);
    }
}
void gem_drawStatisticsPage() {
    g_gemState = GEM_STATE_STATS;

    tft.startWrite();
    tft.fillScreen(ST77XX_DARKGREY);
    gem_drawHeader(true); // Draw "Statistics" header

    // --- 1. Draw Main Level Info ---
    tft.setTextColor(ST77XX_WHITE, ST77XX_DARKGREY);
    tft.setTextWrap(false);
    tft.setCursor(5, (LINE_HEIGHT * 2));
    tft.print("Overall Level: " + String(g_gem_level));

    // --- 2. Draw Main XP Bar ---
    int bar_y = (LINE_HEIGHT * 3);
    int bar_h = 6;
    unsigned long xp_start_of_level = gem_getXPForLevel(g_gem_level);
    unsigned long xp_for_next_level = gem_getXPForLevel(g_gem_level + 1);
    unsigned long xp_needed = xp_for_next_level - xp_start_of_level;
    unsigned long xp_progress = g_gem_total_xp - xp_start_of_level;
    
    float progressPercent = 0.0;
    if (xp_needed > 0) {
        progressPercent = (float)xp_progress / (float)xp_needed;
    }
    if (progressPercent > 1.0) progressPercent = 1.0;
    if (g_gem_level == 1000) progressPercent = 1.0; // Max level

    int bar_pixel_width = SCREEN_WIDTH - 10;
    int filled_width = (int)(progressPercent * bar_pixel_width);
    tft.fillRect(5, bar_y, bar_pixel_width, bar_h, 0x3186); // Dark Grey BG
    uint16_t bar_color = hsvToRgb565((int)(progressPercent * 120), 255, 255);
    tft.fillRect(5, bar_y, filled_width, bar_h, bar_color);
    
    String xpText = String(xp_progress) + "/" + String(xp_needed) + " XP";
    tft.setCursor(10, bar_y + bar_h + 3);
    tft.print(xpText);

    // --- 3. Draw Skills List ---
    int y_pos = bar_y + bar_h + (LINE_HEIGHT * 2); // Start list here
    tft.drawFastHLine(5, y_pos, SCREEN_WIDTH - 10, ST77XX_WHITE);
    y_pos += 5;

    if (g_playerSkillCount == 0) {
        tft.setCursor(5, y_pos);
        tft.setTextColor(ST77XX_WHITE, ST77XX_DARKGREY);
        tft.print("No skills learned yet.");
    }

    // --- NEW SKILL LAYOUT ---
    int skill_bar_h = 4; // Make skill bars thinner
    int skill_bar_width = SCREEN_WIDTH - 10;

    for (int i = g_gem_stats_top_skill; i < g_playerSkillCount; i++) {
        PlayerSkill& skill = g_playerSkills[i];

        uint16_t skill_fg, skill_bg;
        if (g_gemFocus == GEM_FOCUS_STATS_LIST && i == g_gem_stats_selection) {
            // This skill is selected
            skill_fg = ST77XX_BLACK;
            skill_bg = ST77XX_CYAN;
        } else {
            // Not selected
            skill_fg = ST77XX_YELLOW;
            skill_bg = ST77XX_DARKGREY;
        }

        // 1. Draw Skill Name
        int skill_name_y = y_pos; // Store Y position
        tft.fillRect(0, skill_name_y, SCREEN_WIDTH, LINE_HEIGHT, skill_bg); // <-- ADD THIS
        tft.setCursor(5, skill_name_y);
        tft.setTextColor(skill_fg, skill_bg); // <-- THIS IS THE FIX
        tft.print(skill.name + " (Lvl " + String(skill.level) + ")");
        y_pos += LINE_HEIGHT;

        // 2. Calculate Skill XP
        unsigned long xp_next = gem_getXPForSkillLevel(skill.level + 1);
        unsigned long xp_prev = gem_getXPForSkillLevel(skill.level);
        unsigned long xp_needed_skill = xp_next - xp_prev;
        unsigned long xp_progress_skill = skill.xp - xp_prev;

        // 3. Draw Skill Progress Bar
        float skillProgressPercent = 0.0;
        if (xp_needed_skill > 0) {
            skillProgressPercent = (float)xp_progress_skill / (float)xp_needed_skill;
        }
        if (skillProgressPercent > 1.0) skillProgressPercent = 1.0;

        int skill_filled_width = (int)(skillProgressPercent * skill_bar_width);
        tft.fillRect(5, y_pos, skill_bar_width, skill_bar_h, 0x3186); // Dark Grey BG
        uint16_t skill_bar_color = hsvToRgb565((int)(skillProgressPercent * 120), 255, 255);
        tft.fillRect(5, y_pos, skill_filled_width, skill_bar_h, skill_bar_color);

        y_pos += skill_bar_h + 3; // Move down past bar

        // 4. Draw Skill XP Text
        String skillXpText = String(xp_progress_skill) + "/" + String(xp_needed_skill) + " XP";
        tft.setCursor(10, y_pos);
        tft.setTextColor(ST77XX_WHITE, ST77XX_DARKGREY);
        tft.print(skillXpText);
        
        y_pos += LINE_HEIGHT + 4; // Extra padding for next skill
        
        if (y_pos > SCREEN_HEIGHT - (LINE_HEIGHT * 2)) break; // Stop if off screen
    }
    // --- END NEW LAYOUT ---

    tft.endWrite();

    gem_drawStatsList(); // This draws the list and ends its own transaction

    // Now start a new transaction just to draw the footer
    tft.startWrite();
    gem_drawFooter(true);
    tft.endWrite();
}
void gem_drawStatsList() {
    tft.startWrite();

    // 1. Calculate Y position where the list starts
    int bar_y = (LINE_HEIGHT * 3);
    int bar_h = 6;
    int y_pos = bar_y + bar_h + (LINE_HEIGHT * 2);
    int footerY = SCREEN_HEIGHT - LINE_HEIGHT;

    // 2. Clear the entire list area (from the H-line down to the footer)
    tft.fillRect(0, y_pos, SCREEN_WIDTH, footerY - y_pos, ST77XX_DARKGREY);

    // 3. Redraw the H-line
    tft.drawFastHLine(5, y_pos, SCREEN_WIDTH - 10, ST77XX_WHITE);
    y_pos += 5; 
    
    if (g_playerSkillCount == 0) {
        tft.setCursor(5, y_pos);
        tft.setTextColor(ST77XX_WHITE, ST77XX_DARKGREY);
        tft.print("No skills learned yet.");
    }
    
    // 4. End this transaction. The item drawers will handle their own.
    tft.endWrite();

    // 5. Loop and draw each visible item
    // This is now outside the transaction
    int item_height_px = LINE_HEIGHT + 4 + 3 + LINE_HEIGHT + 4; // 29px
    for (int i = g_gem_stats_top_skill; i < g_playerSkillCount; i++) {
        gem_drawStatsItem(i); // This function starts/ends its own transaction
        
        // Check if the next item will be off-screen
        int screen_list_index = i - g_gem_stats_top_skill;
        int next_y_pos = (y_pos + 5) + ((screen_list_index + 1) * item_height_px);
        if (next_y_pos >= footerY) {
            break;
        }
    }
}
void gem_drawStatsItem(int skill_index) {
    // --- 1. Calculate Y position ---
    // This logic MUST match the start of the loop in gem_drawStatisticsPage
    int bar_y_start = (LINE_HEIGHT * 3);
    int bar_h_start = 6;
    int y_pos = bar_y_start + bar_h_start + (LINE_HEIGHT * 2) + 5; // STATS_LIST_START_Y
    
    // This is the height of one item, calculated from the loop in gem_drawStatisticsPage
    int skill_bar_h = 4;
    int item_height_px = LINE_HEIGHT + (skill_bar_h + 3) + LINE_HEIGHT + 4; // 29px

    int screen_list_index = skill_index - g_gem_stats_top_skill;
    y_pos += (screen_list_index * item_height_px);

    // --- 2. Check if visible ---
    int footerY = SCREEN_HEIGHT - (LINE_HEIGHT * 2); // Check against bottom of drawable area
    if (y_pos >= footerY || skill_index < 0 || skill_index >= g_playerSkillCount) {
        return; // Off-screen or invalid
    }

    // --- 3. Start Transaction ---
    tft.startWrite();

    PlayerSkill& skill = g_playerSkills[skill_index];
    uint16_t skill_fg, skill_bg;
    if (g_gemFocus == GEM_FOCUS_STATS_LIST && skill_index == g_gem_stats_selection) {
        skill_fg = ST77XX_BLACK;
        skill_bg = ST77XX_CYAN;
    } else {
        skill_fg = ST77XX_YELLOW;
        skill_bg = ST77XX_DARKGREY;
    }

    // --- 4. Draw the item (MODIFIED) ---
    
    // 1. Draw Skill Name
    int skill_name_y = y_pos;
    
    // --- FIX 1: Only fill the background for the NAME line ---
    tft.fillRect(0, skill_name_y, SCREEN_WIDTH, LINE_HEIGHT, skill_bg); 
    
    tft.setCursor(5, skill_name_y);
    tft.setTextColor(skill_fg, skill_bg);
    tft.print(skill.name + " (Lvl " + String(skill.level) + ")");
    y_pos += LINE_HEIGHT;

    // --- FIX 2: Clear the rest of the item area to the default BG color ---
    // This ensures that when deselecting, the cyan is erased
    int bar_and_xp_height = item_height_px - LINE_HEIGHT;
    tft.fillRect(0, y_pos, SCREEN_WIDTH, bar_and_xp_height, ST77XX_DARKGREY);

    // 2. Calculate Skill XP
    unsigned long xp_next = gem_getXPForSkillLevel(skill.level + 1);
    unsigned long xp_prev = gem_getXPForSkillLevel(skill.level);
    unsigned long xp_needed_skill = xp_next - xp_prev;
    unsigned long xp_progress_skill = skill.xp - xp_prev;
    
    // 3. Draw Skill Progress Bar
    float skillProgressPercent = 0.0;
    if (xp_needed_skill > 0) {
        skillProgressPercent = (float)xp_progress_skill / (float)xp_needed_skill;
    }
    if (skillProgressPercent > 1.0) skillProgressPercent = 1.0;
    int skill_bar_width = SCREEN_WIDTH - 10;
    int skill_filled_width = (int)(skillProgressPercent * skill_bar_width);
    
    // This fillRect draws the "empty" part of the bar
    tft.fillRect(5, y_pos, skill_bar_width, skill_bar_h, 0x3186); 
    uint16_t skill_bar_color = hsvToRgb565((int)(skillProgressPercent * 120), 255, 255);
    // This fillRect draws the "filled" part
    tft.fillRect(5, y_pos, skill_filled_width, skill_bar_h, skill_bar_color);

    y_pos += skill_bar_h + 3;

    // 4. Draw Skill XP Text
    String skillXpText = String(xp_progress_skill) + "/" + String(xp_needed_skill) + " XP";
    tft.setCursor(10, y_pos);
    // --- FIX 3: Always draw text on the default BG color ---
    tft.setTextColor(ST77XX_WHITE, ST77XX_DARKGREY);
    tft.print(skillXpText);
    
    tft.endWrite();
}
void gem_draw_autocomplete() {
    if (!g_gem_autocomplete_active || g_gem_autocomplete_count == 0) return;

    TextArea* editor = gemCreateSkillsEditor;
    if (!editor) return;

    // 1. Find cursor screen position
    int screenLine = editor->cursorLine - editor->topLine;
    int screenPixelX = (editor->cursorCol * CHAR_WIDTH) + editor->x;
    int screenPixelY = (screenLine * LINE_HEIGHT) + editor->y;

    // 2. Define dropdown geometry
    int drawX = editor->x;
    int drawY = screenPixelY + LINE_HEIGHT + 1; // 1px below cursor line
    int drawW = SCREEN_WIDTH - (editor->x * 2);
    int drawH = (g_gem_autocomplete_count * LINE_HEIGHT) + 2; // +2 for border

    // --- FIX: Store the geometry ---
    g_gem_last_ac_x = drawX;
    g_gem_last_ac_y = drawY;
    g_gem_last_ac_w = drawW;
    g_gem_last_ac_h = drawH;
    // --- END FIX ---

    // 3. Draw box
    tft.drawRect(drawX, drawY, drawW, drawH, ST77XX_WHITE);
    tft.fillRect(drawX + 1, drawY + 1, drawW - 2, drawH - 2, ST77XX_BLACK);

    // 4. Draw suggestions
    for (int i = 0; i < g_gem_autocomplete_count; i++) {
        int y_pos = drawY + 1 + (i * LINE_HEIGHT);
        uint16_t fg, bg;
        if (i == g_gem_autocomplete_selection) {
            fg = ST77XX_BLACK;
            bg = ST77XX_CYAN;
        } else {
            fg = ST77XX_WHITE;
            bg = ST77XX_BLACK;
        }
        tft.fillRect(drawX + 1, y_pos, drawW - 2, LINE_HEIGHT, bg);
        tft.setTextColor(fg, bg);
        tft.setCursor(drawX + 3, y_pos);
        tft.print(g_gem_autocomplete_suggestions[i]);
    }
}
void gem_hide_autocomplete(bool needsRedraw) {
    if (!g_gem_autocomplete_active && !needsRedraw) return;
    
    g_gem_autocomplete_active = false;
    g_gem_autocomplete_count = 0;
    
    if (needsRedraw) {
        // --- THIS IS THE FIX ---
        // This is now a "raw" function.
        // It MUST be wrapped in tft.startWrite() / tft.endWrite() by its caller.
        
        if (g_gem_last_ac_h == 0) return; // Nothing was ever drawn

        // 1. Clear the exact rectangle of the old box
        tft.fillRect(g_gem_last_ac_x, g_gem_last_ac_y, g_gem_last_ac_w, g_gem_last_ac_h, ST77XX_DARKGREY);

        // 2. Define the Y bounds of the cleared rectangle
        int clearTop = g_gem_last_ac_y;
        int clearBottom = g_gem_last_ac_y + g_gem_last_ac_h;
        
        // 3. Surgically redraw *only* the components that were overlapped
        // Y positions are hard-coded based on gem_drawQuestEditor
        
        // Skills Editor (Y: 81, H: 36. Bottom: 117)
        if (gemCreateSkillsEditor && clearTop < (81 + 36) && clearBottom > 81) {
            gemCreateSkillsEditor->draw(tft);
        }

        // "Difficulty" labels (Y: 117)
        if (clearTop <= 117 && clearBottom >= 117) {
            tft.setTextColor(ST77XX_WHITE, ST77XX_DARKGREY);
            tft.setCursor(5, (LINE_HEIGHT * 13)); tft.print("Difficulty (1-7):");
            tft.setCursor( (COLS/2) * CHAR_WIDTH, (LINE_HEIGHT * 13)); tft.print("Desire (1-7):");
        }
        
        // "Difficulty/Desire" fields (Y: 126, H: 9. Bottom: 135)
        if (clearTop < (126 + 9) && clearBottom > 126) {
            if (gemCreateDiffEditor) gemCreateDiffEditor->draw(tft);
            if (gemCreateDesireEditor) gemCreateDesireEditor->draw(tft);
        }
        
        // "Time" labels (Y: 135)
        if (clearTop <= 135 && clearBottom >= 135) {
            tft.setTextColor(ST77XX_WHITE, ST77XX_DARKGREY);
            tft.setCursor(5, (LINE_HEIGHT * 15)); tft.print("Time (Hours, 0-100):");
            tft.setCursor( (COLS/2) * CHAR_WIDTH, (LINE_HEIGHT * 15)); tft.print("Time (Mins, 0-100):");
        }
        
        // "Time" fields (Y: 144, H: 9. Bottom: 153)
        if (clearTop < (144 + 9) && clearBottom > 144) {
            if (gemCreateTimeEditor) gemCreateTimeEditor->draw(tft);
            if (gemCreateMinsEditor) gemCreateMinsEditor->draw(tft);
        }
        
        // "Repeatable/Cycle" label (Y: 153)
        if (clearTop <= 153 && clearBottom >= 153) {
            tft.setTextColor(ST77XX_WHITE, ST77XX_DARKGREY);
            tft.setCursor( 5, (LINE_HEIGHT * 17));
            if (g_gem_editor_questType == QT_STANDARD) {
                 tft.print("Repeatable:");
            } else {
                 tft.print("Add Cycle (Mins):");
            }
        }
        
        // "Repeatable/Cycle" field (Y: 162, H: 9. Bottom: 171)
        if (clearTop < (162 + 9) && clearBottom > 162) {
             if (g_gem_editor_questType == QT_STANDARD) {
                int repeatX = 5;
                int repeatY = (LINE_HEIGHT * 18);
                String text = g_gem_editor_isRepeatable ? "[ON]" : "[OFF]";
                int textWidth = text.length() * CHAR_WIDTH;
                uint16_t repeat_fg = g_gem_editor_isRepeatable ? ST77XX_BLACK : ST77XX_WHITE;
                uint16_t repeat_bg = g_gem_editor_isRepeatable ? ST77XX_GREEN : ST77XX_RED;
                tft.fillRect(repeatX, repeatY - 1, textWidth, LINE_HEIGHT, repeat_bg);
                tft.setCursor(repeatX, repeatY);
                tft.setTextColor(repeat_fg, repeat_bg);
                tft.print(text);
             } else {
                if (gemCreateAddCycleEditor) gemCreateAddCycleEditor->draw(tft);
             }
        }

    }
}
void gem_clear_exposed_background(int old_h, int new_h) {
    if (old_h <= new_h) return; // Only clear if shrinking

    // The area to clear is the difference in height, starting from the new height's Y position
    int clearY = g_gem_last_ac_y + new_h;
    int clearH = old_h - new_h;
    
    // 1. Clear the exposed rectangle to background color
    tft.fillRect(g_gem_last_ac_x, clearY, g_gem_last_ac_w, clearH, ST77XX_DARKGREY);

    // 2. Surgically redraw *only* the UI elements that were just exposed
    int clearBottom = clearY + clearH;
    
    // Y positions are hard-coded based on gem_drawQuestEditor
    
    // Skills Editor (Y: 81, H: 36. Bottom: 117)
    if (gemCreateSkillsEditor && clearY < (81 + 36) && clearBottom > 81) {
        gemCreateSkillsEditor->draw(tft);
    }

    // "Difficulty" labels (Y: 117)
    if (clearY <= 117 && clearBottom >= 117) {
        tft.setTextColor(ST77XX_WHITE, ST77XX_DARKGREY);
        tft.setCursor(5, (LINE_HEIGHT * 13)); tft.print("Difficulty (1-7):");
        tft.setCursor( (COLS/2) * CHAR_WIDTH, (LINE_HEIGHT * 13)); tft.print("Desire (1-7):");
    }
    
    // "Difficulty/Desire" fields (Y: 126, H: 9. Bottom: 135)
    if (clearY < (126 + 9) && clearBottom > 126) {
        if (gemCreateDiffEditor) gemCreateDiffEditor->draw(tft);
        if (gemCreateDesireEditor) gemCreateDesireEditor->draw(tft);
    }
    
    // ... (Continue this surgical logic for every overlapping UI element below the skills editor)
    // You must ensure all relevant UI elements in gem_hide_autocomplete are covered here, 
    // replacing the hardcoded checks with the new 'clearY' and 'clearBottom'.
    // Here are the remaining checks from your gem_hide_autocomplete:

    // "Time" labels (Y: 135)
    if (clearY <= 135 && clearBottom >= 135) {
        tft.setTextColor(ST77XX_WHITE, ST77XX_DARKGREY);
        tft.setCursor(5, (LINE_HEIGHT * 15)); tft.print("Time (Hours, 0-100):");
        tft.setCursor( (COLS/2) * CHAR_WIDTH, (LINE_HEIGHT * 15)); tft.print("Time (Mins, 0-100):");
    }
    
    // "Time" fields (Y: 144, H: 9. Bottom: 153)
    if (clearY < (144 + 9) && clearBottom > 144) {
        if (gemCreateTimeEditor) gemCreateTimeEditor->draw(tft);
        if (gemCreateMinsEditor) gemCreateMinsEditor->draw(tft);
    }
    
    // "Repeatable/Cycle" label (Y: 153)
    if (clearY <= 153 && clearBottom >= 153) {
        tft.setTextColor(ST77XX_WHITE, ST77XX_DARKGREY);
        tft.setCursor( 5, (LINE_HEIGHT * 17));
        if (g_gem_editor_questType == QT_STANDARD) {
             tft.print("Repeatable:");
        } else {
             tft.print("Add Cycle (Mins):");
        }
    }
    
    // "Repeatable/Cycle" field (Y: 162, H: 9. Bottom: 171)
    if (clearY < (162 + 9) && clearBottom > 162) {
         if (g_gem_editor_questType == QT_STANDARD) {
            int repeatX = 5;
            int repeatY = (LINE_HEIGHT * 18);
            String text = g_gem_editor_isRepeatable ? "[ON]" : "[OFF]";
            int textWidth = text.length() * CHAR_WIDTH;
            uint16_t repeat_fg = g_gem_editor_isRepeatable ? ST77XX_BLACK : ST77XX_WHITE;
            uint16_t repeat_bg = g_gem_editor_isRepeatable ? ST77XX_GREEN : ST77XX_RED;
            tft.fillRect(repeatX, repeatY - 1, textWidth, LINE_HEIGHT, repeat_bg);
            tft.setCursor(repeatX, repeatY);
            tft.setTextColor(repeat_fg, repeat_bg);
            tft.print(text);
         } else {
            if (gemCreateAddCycleEditor) gemCreateAddCycleEditor->draw(tft);
         }
    }
}
void gem_update_autocomplete(TextArea* editor) {
    if (!editor) return;

    String currentLine = editor->lines[editor->cursorLine];
    int cursor = editor->cursorCol;
    
    int wordStart = cursor;
    while (wordStart > 0 && currentLine.charAt(wordStart - 1) != ' ') {
        wordStart--;
    }
    
    String currentWord = currentLine.substring(wordStart, cursor);
    currentWord.trim();
    
    bool was_active = g_gem_autocomplete_active;
    
    // 1. Capture old state and geometry
    int old_ac_h = g_gem_last_ac_h; 
    
    // 2. Calculate new matches
    g_gem_autocomplete_word_start_line = editor->cursorLine;
    g_gem_autocomplete_word_start_col = wordStart;
    
    g_gem_autocomplete_count = 0;
    String wordUpper = currentWord;
    wordUpper.toUpperCase();
    for (int i = 0; i < g_skillCount && g_gem_autocomplete_count < MAX_AUTOCOMPLETE; i++) {
        String skillUpper = g_skillDB[i];
        skillUpper.toUpperCase();

        if (skillUpper.startsWith(wordUpper)) {
            g_gem_autocomplete_suggestions[g_gem_autocomplete_count++] = g_skillDB[i];
        }
    }
    
    // Determine the new state and size
    bool should_be_active = (g_gem_autocomplete_count > 0);
    int new_ac_h = should_be_active ? (g_gem_autocomplete_count * LINE_HEIGHT) + 2 : 0;

    // 3. Redraw Logic: Execute within a single transaction
    if (was_active || should_be_active) {
        tft.startWrite();
        
        if (was_active && !should_be_active) {
            // Case: ON -> OFF (Definitive Hide)
            gem_hide_autocomplete(true); // Full surgical hide
            g_gem_last_ac_y = 0; 
            g_gem_last_ac_h = 0;
            g_gem_last_ac_x = 0;
            g_gem_last_ac_w = 0;
        } 
        else if (should_be_active) {
            
            if (was_active) {
                // Case: ON -> ON
                
                // --- FIX: Always fast clear the old box first to remove artifacts ---
                if (old_ac_h > 0) {
                   tft.fillRect(g_gem_last_ac_x, g_gem_last_ac_y, g_gem_last_ac_w, old_ac_h, ST77XX_DARKGREY);
                }
                
                // --- FIX: If the box shrank, surgically redraw the exposed background. ---
                if (new_ac_h < old_ac_h) {
                    gem_clear_exposed_background(old_ac_h, new_ac_h);
                }
            }

            // Draw the new box (this saves the new geometry)
            g_gem_autocomplete_active = true;
            g_gem_autocomplete_selection = 0;
            gem_draw_autocomplete();
        }

        tft.endWrite();
    }
    
    // Final state update
    if (!should_be_active) {
        g_gem_autocomplete_active = false;
    }
    
    g_forceCursorRedraw = true;
}
void gem_select_autocomplete(TextArea* editor) {
    // Safety checks
    if (!editor || !g_gem_autocomplete_active || g_gem_autocomplete_selection >= g_gem_autocomplete_count) return;

    // 1. Get the selected suggestion
    String suggestion = g_gem_autocomplete_suggestions[g_gem_autocomplete_selection];
    
    // 2. Start a single transaction for all drawing/writing
    tft.startWrite();

    // 3. Clear the current partial word
    // Calculate how many characters to backspace, starting from the cursor
    int chars_to_delete = editor->cursorCol - g_gem_autocomplete_word_start_col;
    for (int i = 0; i < chars_to_delete; i++) {
        editor->backspace(tft);
    }
    
    // 4. Insert the full suggested word
    for (int i = 0; i < suggestion.length(); i++) {
        editor->insertChar(tft, suggestion.charAt(i));
    }

    // 5. Insert a space to separate the next skill
    editor->insertChar(tft, ' ');

    // 6. Hide the dropdown and clear the artifacts
    // Since we are inside tft.startWrite(), we call the raw hide function
    gem_hide_autocomplete(true);
    
    // 7. Reset geometry variables (THE KEY FIX for the subsequent word)
    g_gem_last_ac_y = 0; 
    g_gem_last_ac_h = 0;
    g_gem_last_ac_x = 0;
    g_gem_last_ac_w = 0;

    // 8. Reset state flags
    g_gem_autocomplete_active = false;
    g_gem_autocomplete_count = 0;

    tft.endWrite();
    
    g_forceCursorRedraw = true;
}
void addQuest(String title, String desc, String notes, String skills, bool isRepeatable, int diff, int desire, int timeEstimateMins, QuestType type, int addCycle, String icon) {
  if (g_questCount >= MAX_QUESTS) return; 

  Quest& newQuest = g_questDB[g_questCount];
  newQuest.title = title;
  newQuest.description = desc;
  newQuest.questState = STATE_UNCOMPLETED;
  newQuest.notes = notes;
  newQuest.skills = skills;
  newQuest.isRepeatable = isRepeatable;
  newQuest.difficulty = diff;
  newQuest.desire = desire;
  newQuest.timerDurationMinutes = (unsigned long)timeEstimateMins; // <-- FIX 1: Save total minutes directly
  newQuest.type = type;
  newQuest.addCycleMinutes = addCycle;
  newQuest.iconFile = icon;
  
  // --- REWARD CALC FIX ---
  float timeHours = (float)timeEstimateMins / 60.0f; // <-- FIX 2: Use proportional float
  float desire_multiplier = 1.0 + ( (7.0 - (float)desire) / 6.0 );
  float base_reward = (diff * 10) + (timeHours * 2.0f); // <-- FIX 3: Use correct formula (diff * 10)
  newQuest.gemReward = (int)(base_reward * desire_multiplier);
  newQuest.xpReward = (diff * 5) + (int)(desire_multiplier * 5) + (int)(timeHours * 2.0f); // <-- FIX 4: Cast new float
  // --- END REWARD CALC FIX ---
  
  newQuest.timerMinutesElapsed = 0;
  newQuest.timerStartTime = 0; 
  newQuest.gemBonusAwarded = 0;
  
  g_questCount++;
  gem_sortQuests();
}
void gem_create_default_quests() {
    addQuest("The Crystal Cave", "Find the 3 hidden gems.", "Find the 3 hidden gems.", "Exploration Caving", false, 3, 5, 60, QT_STANDARD, 0, "");
    addQuest("The Missing Blacksmith", "Ask the town guard.", "Ask the town guard for clues.", "Investigation", false, 5, 7, 240, QT_STANDARD, 0, "");
    addQuest("A Rat Problem", "Clear the tavern cellar.", "Clear the tavern cellar.", "Combat", false, 1, 3, 60, QT_STANDARD, 0, "");
    // ...
}
void gem_sortQuests() {
    // Bubble sort is fine for MAX_QUESTS=10 and provides stability.
    // Order: STATE_UNCOMPLETED(0), STATE_STARTED(1), STATE_COMPLETED(2)
    for (int i = 0; i < g_questCount - 1; i++) {
        for (int j = 0; j < g_questCount - i - 1; j++) {
            Quest& q1 = g_questDB[j];
            Quest& q2 = g_questDB[j + 1];

            // Swap if q1 has a lower priority (higher enum value) than q2
            if (q1.questState > q2.questState) {
                Quest temp = q1;
                q1 = q2;
                q2 = temp;
            }
        }
    }
}
void gem_deleteQuest(int db_index) {
    if (db_index < 0 || db_index >= g_questCount) return;

    // Shift all quests after this one up by one
    for (int i = db_index; i < g_questCount - 1; i++) {
        g_questDB[i] = g_questDB[i + 1];
    }
    
    g_questCount--; // Decrement the total number of quests
    
    pushSystemMessage("Quest deleted.");
    gem_save_database(g_gem_db_filename);
}
String gem_remove_skill_from_quest_string(String skillList, String skillToRemove) {
    skillToRemove.trim();
    skillList.trim();
    
    if (skillList.length() == 0 || skillToRemove.length() == 0) {
        return skillList;
    }

    // ⭐ CORRECTED: Initializing the return variable
    String newSkillList = "";
    int current = 0;
    int next = -1;
    bool firstSkillAdded = false;

    // Convert skill to remove to lowercase for comparison
    skillToRemove.toLowerCase();

    do {
        next = skillList.indexOf(' ', current);
        String skillName;

        if (next == -1) {
            skillName = skillList.substring(current);
        } else {
            skillName = skillList.substring(current, next);
        }
        current = next + 1;
        
        skillName.trim();
        
        if (skillName.length() > 0) {
            // Check if this is the skill we want to remove (case-insensitive)
            if (!skillName.equalsIgnoreCase(skillToRemove)) {
                
                // This is a skill we want to KEEP. Add it back to the list.
                if (firstSkillAdded) {
                    // ⭐ CORRECTED VARIABLE NAME
                    newSkillList += " "; 
                }
                // ⭐ CORRECTED VARIABLE NAME
                newSkillList += skillName;
                firstSkillAdded = true;
            }
        }
    } while (next != -1);

    // ⭐ CORRECTED VARIABLE NAME
    return newSkillList; 
}
String gem_deleteSkill(int index) {
    if (index < 0 || index >= g_playerSkillCount) {
        return ""; // Invalid index
    }
    
    // 1. CRITICAL: Get the name of the skill being deleted before array manipulation
    String skillToDelete = g_playerSkills[index].name;
    
    // 2. PURGE SKILL NAME FROM ALL ACTIVE QUESTS
    for (int i = 0; i < g_questCount; i++) {
        g_questDB[i].skills = gem_remove_skill_from_quest_string(
            g_questDB[i].skills, 
            skillToDelete
        );
    }

    // 3. SHIFT THE g_playerSkills ARRAY (Move everything after 'index' forward)
    // The loop iterates up to the second-to-last element (g_playerSkillCount - 2)
    for (int i = index; i < g_playerSkillCount - 1; i++) {
        g_playerSkills[i] = g_playerSkills[i + 1];
    }
    
    // ⭐ CRITICAL CHANGE: Clear the ghost element at the last occupied index
    // This is the index that now holds a duplicate of the element before it.
    // We clear it while the count is still correct.
    if (g_playerSkillCount > 0) {
        int lastIndexToClear = g_playerSkillCount - 1;
        g_playerSkills[lastIndexToClear].name = "";
        g_playerSkills[lastIndexToClear].level = 0; 
        g_playerSkills[lastIndexToClear].xp = 0;    
        // TODO: Clear any other fields in your PlayerSkill struct here
    }

    // 4. DECREMENT COUNT
    g_playerSkillCount--; 
    
    // 5. REBUILD GLOBAL SKILL LIST (Fixes autocomplete)
    gem_rebuildGlobalSkills(); 

    // 6. FINAL CLEANUP AND SAVE
    gem_recalculateLevel();
    gem_sortQuests();
    gem_save_database(g_gem_db_filename); 

    return skillToDelete;
}
void gem_rebuildGlobalSkills() {
    // 1. Clear the current global database
    g_skillCount = 0;
    // Clearing strings is good practice to free memory
    for (int i = 0; i < MAX_SKILLS; i++) { 
        g_skillDB[i] = "";
    }

    // 2. Add skills from all active quests (g_questDB)
    for (int i = 0; i < g_questCount; i++) {
        // This function must handle the comma-separated string
        gem_add_skills_to_database(g_questDB[i].skills); 
    }
    
    // 3. Add skills from all active player skills (g_playerSkills)
    for (int i = 0; i < g_playerSkillCount; i++) {
        // PlayerSkill names are single skills, no CSV parsing needed here
        gem_add_skills_to_database(g_playerSkills[i].name);
    }
}
void gem_close_quest_detail() {
    if (g_gem_currentQuestID == -1 || !gemQuestNotesEditor) return;
    
    // 2. Destroy the component
    delete gemQuestNotesEditor;
    gemQuestNotesEditor = nullptr;
    // 3. Reset state
    g_gem_currentQuestID = -1;
    g_gemState = GEM_STATE_QUEST_LIST;
    g_gemFocus = GEM_FOCUS_LIST;
    g_gem_list_selection = 0;

    // 4. Redraw the list
    gem_drawQuestList();
}
unsigned long gem_getCostForLevel(int level) {
    if (level <= 0) return 0; // No cost to complete "level 0"
    if (level >= 1000) return 0; // Max level, no more cost

    // Formula: (Level^2 * 10) + 100
    // Lvl 1->2 costs: (1*1*10) + 100 = 110 XP
    // Lvl 2->3 costs: (2*2*10) + 100 = 140 XP
    // Lvl 22->23 costs: (22*22*10) + 100 = 4940 XP
    unsigned long L = (unsigned long)level;
    return (L * L * 10) + 100;
}
unsigned long gem_getXPForLevel(int level) {
    if (level <= 1) return 0;
    if (level > 1000) level = 1000; // Cap

    unsigned long total_xp = 0;
    // Sum the *cost* of all previous levels (from 1 up to level-1)
    for (int i = 1; i < level; i++) {
        total_xp += gem_getCostForLevel(i);
    }
    return total_xp;
}
unsigned long gem_getXPForSkillLevel(int level) {
    if (level <= 1) return 0;
    // Formula: 10 * L^2
    // Lvl 2: 40 XP
    // Lvl 3: 90 XP
    // Lvl 4: 160 XP
    return (unsigned long)level * (unsigned long)level * 10;
}
void gem_recalculate_skill_level(PlayerSkill* skill, bool checkLevelUp) {
    if (!skill) return;

    int oldLevel = skill->level;

    // Recalculate level from base (safest way)
    skill->level = 1;
    while (skill->level < 100) { // Hard cap at Lvl 100
        if (skill->xp >= gem_getXPForSkillLevel(skill->level + 1)) {
            skill->level++;
        } else {
            break; // Not enough XP for the next level
        }
    }

    // If we are checking for level-ups (xp > 0) and level changed
    if (checkLevelUp && skill->level > oldLevel) {
        int levelsGained = skill->level - oldLevel;
        
        // Award 100 main XP *per level gained*
        gem_awardXP(100 * levelsGained);
        
        pushSystemMessage("Skill Up! " + skill->name + " is now Lvl " + String(skill->level) + "!");
        pushSystemMessage("You gain " + String(100 * levelsGained) + " main XP!");
    }
    
    // If we are checking for level-downs (xp < 0), we also need to subtract
    // the main XP bonus that was awarded.
    if (!checkLevelUp && skill->level < oldLevel) {
        int levelsLost = oldLevel - skill->level;
        
        // Subtract 100 main XP *per level lost*
        gem_awardXP(-100 * levelsLost); 
        
        pushSystemMessage("Skill " + skill->name + " decreased to Lvl " + String(skill->level) + ".");
        pushSystemMessage("You lose " + String(100 * levelsLost) + " main XP.");
    }
}
void gem_award_skill_xp(String skillName, int xp) {
    // 1. Find the skill in the player's database
    PlayerSkill* skill = nullptr;
    for (int i = 0; i < g_playerSkillCount; i++) {
        if (g_playerSkills[i].name.equalsIgnoreCase(skillName)) {
            skill = &g_playerSkills[i];
            break;
        }
    }

    // 2. If not found, create it (if there's space)
    if (skill == nullptr) {
        if (xp <= 0) return; // Don't create a skill just to subtract XP
        if (g_playerSkillCount >= MAX_PLAYER_SKILLS) return; // No space
        
        skill = &g_playerSkills[g_playerSkillCount];
        skill->name = skillName;
        skill->level = 1;
        skill->xp = 0;
        g_playerSkillCount++;
    }

    // 3. Add or Subtract XP
    if (xp > 0) {
        skill->xp += xp;
    } else {
        // Don't let XP drop below 0
        if ((unsigned long)(-xp) > skill->xp) {
            skill->xp = 0;
        } else {
            skill->xp += xp; // (xp is already negative)
        }
    }

    // 4. Recalculate level (up or down)
    gem_recalculate_skill_level(skill, (xp > 0));
}
void gem_add_skills_to_database(String skillsString) {
    if (skillsString.length() == 0) return;

    int current = 0;
    int next = -1;
    String skillName;

    // Use ' ' as the delimiter
    do {
        next = skillsString.indexOf(' ', current);
        if (next == -1) {
            skillName = skillsString.substring(current);
        } else {
            skillName = skillsString.substring(current, next);
        }
        current = next + 1;
        skillName.trim();
        
        if (skillName.length() > 0) {
            // --- Part 1: Update Global DB (for autocomplete) ---
            bool found_global = false;
            for (int i = 0; i < g_skillCount; i++) {
                if (g_skillDB[i].equalsIgnoreCase(skillName)) {
                    found_global = true;
                    break;
                }
            }
            if (!found_global && g_skillCount < MAX_SKILLS) {
                g_skillDB[g_skillCount++] = skillName;
            }

            // --- FIX: Update Player Skill DB (for stats page) ---
            bool found_player = false;
            for (int i = 0; i < g_playerSkillCount; i++) {
                if (g_playerSkills[i].name.equalsIgnoreCase(skillName)) {
                    found_player = true;
                    break;
                }
            }
            // If not found in the player's list, add it with 0 XP
            if (!found_player && g_playerSkillCount < MAX_PLAYER_SKILLS) {
                PlayerSkill& newSkill = g_playerSkills[g_playerSkillCount];
                newSkill.name = skillName;
                newSkill.level = 1;
                newSkill.xp = 0;
                g_playerSkillCount++;
            }
        }
    } while (next != -1);
    
    // Note: No save is needed here. gem_save_database() is called
    // right after this function in the "Save" button logic.
}
void gem_recalculateLevel() {
    g_gem_level = 1;
    while (g_gem_level < 1000) {
        if (g_gem_total_xp >= gem_getXPForLevel(g_gem_level + 1)) {
            g_gem_level++;
        } else {
            break; // Not enough XP for the next level
        }
    }
}
void gem_awardXP(int xp) {
    if (xp > 0) {
        g_gem_total_xp += xp;
    } else {
        // Don't let XP drop below 0
        if ((unsigned long)(-xp) > g_gem_total_xp) {
            g_gem_total_xp = 0;
        } else {
            g_gem_total_xp += xp; // (xp is already negative)
        }
    }
    // Update the level based on the new XP total
    gem_recalculateLevel();
}
void addMerchantItem(String title, String desc, int rarity, int quality, int need, int desire, bool isConsumable) {
    if (g_merchantItemCount >= MAX_MERCHANT_ITEMS) return;
    MerchantItem& newItem = g_merchantDB[g_merchantItemCount];
    newItem.title = title;
    newItem.description = desc;
    newItem.rarity = rarity;
    newItem.quality = quality;
    newItem.need = need;
    newItem.desire = desire;
    newItem.isConsumable = isConsumable;
    // Calculate and store the final price
    newItem.price = gem_calculate_item_price(rarity, quality, need, desire, isConsumable);
    
    g_merchantItemCount++;
}
int gem_getMerchantItemHeight(int index) {
    if (index < 0 || index >= g_merchantItemCount) return 0;
    // Use 34px for items with icons (Fits 32px image + 1px border)
    if (g_merchantDB[index].iconFile.length() > 0) return 34; 
    return LINE_HEIGHT + 4;
}
int gem_getMerchantItemY(int index) {
    if (index < g_merchant_list_top_item) return -1;
    int y = (LINE_HEIGHT * 2); 
    int footerY = SCREEN_HEIGHT - LINE_HEIGHT;
    
    for (int i = g_merchant_list_top_item; i < index; i++) {
        y += gem_getMerchantItemHeight(i);
        if (y >= footerY) return -1; 
    }
    if (y + gem_getMerchantItemHeight(index) > footerY) return -1;
    return y;
}
int gem_getInventoryItemHeight(int index) {
    if (index < 0 || index >= g_inventoryItemCount) return 0;
    if (g_inventoryDB[index].iconFile.length() > 0) return 34; // 34px Height
    return LINE_HEIGHT + 4;
}
int gem_getInventoryItemY(int index) {
    if (index < g_inventory_list_top_item) return -1;
    int y = (LINE_HEIGHT * 2);
    int footerY = SCREEN_HEIGHT - LINE_HEIGHT;
    
    for (int i = g_inventory_list_top_item; i < index; i++) {
        y += gem_getInventoryItemHeight(i);
        if (y >= footerY) return -1;
    }
    if (y + gem_getInventoryItemHeight(index) > footerY) return -1;
    return y;
}
int gem_getQuestItemHeight(int index) {
    if (index == 0) return (LINE_HEIGHT * 2); // Index 0 [New Quest] is 18px
    
    int db_index = index - 1;
    if (db_index < 0 || db_index >= g_questCount) return 0;

    // If icon exists, use 34px
    if (g_questDB[db_index].iconFile.length() > 0) return 34;
    
    // Standard items: 2 lines (18px) + 4px for bar = 22px
    return 22; 
}
int gem_getQuestItemY(int index) {
    if (index < g_gem_list_top_item) return -1; // Scrolled off top
    
    int y = (LINE_HEIGHT * 2); // Start Y (below header)
    int footerY = SCREEN_HEIGHT - LINE_HEIGHT;
    
    for (int i = g_gem_list_top_item; i < index; i++) {
        y += gem_getQuestItemHeight(i);
        if (y >= footerY) return -1; // Scrolled off bottom
    }
    
    if (y + gem_getQuestItemHeight(index) > footerY) return -1;
    return y;
}
void gem_create_default_items() {
    // addMerchantItem(Title, Desc, Rarity, Quality, Need, Desire, isConsumable)
    addMerchantItem("Health Potion", "A simple potion that restores 10 HP.", 1, 2, 7, 5, true);
    addMerchantItem("Mana Potion", "A simple potion that restores 10 MP.", 1, 2, 6, 5, true);
    addMerchantItem("Iron Sword", "A basic, sturdy sword.", 2, 3, 5, 4, false);
}
void gem_deleteMerchantItem(int db_index) {
    if (db_index < 0 || db_index >= g_merchantItemCount) return;
    for (int i = db_index; i < g_merchantItemCount - 1; i++) {
        g_merchantDB[i] = g_merchantDB[i + 1];
    }
    g_merchantItemCount--;
    pushSystemMessage("Item deleted.");
    gem_save_database(g_gem_db_filename);
}
void gem_drawMerchantItem(int item_index) {
    if (item_index < 0 || item_index >= g_merchantItemCount) return;

    int y_pos = gem_getMerchantItemY(item_index);
    if (y_pos == -1) return; 

    int height = gem_getMerchantItemHeight(item_index);
    MerchantItem& item = g_merchantDB[item_index];

    tft.startWrite();
    bool isSelected = (g_gemFocus == GEM_FOCUS_MERCHANT_LIST && item_index == g_merchant_list_selection);
    uint16_t fg = isSelected ? ST77XX_BLACK : ST77XX_WHITE;
    uint16_t bg = isSelected ? ST77XX_CYAN : ST77XX_DARKGREY;

    tft.fillRect(0, y_pos, SCREEN_WIDTH, height, bg);
    tft.setTextColor(fg, bg);

    if (height > (LINE_HEIGHT + 4)) {
        // --- 34px ROW MODE (32px Image) ---
        // Draw Icon (32x32) at offset (2, y+1)
        drawItemBitmap(item.iconFile, 2, y_pos + 1, 32, 32);
        
        // Draw Border (34x34) at offset (1, y) to frame it
        tft.drawRect(1, y_pos, 34, 34, isSelected ? ST77XX_BLACK : ST77XX_WHITE);
        
        // Text offset (Border 34 + 6px padding)
        int textX = 40;

        // Row 1: Title
        tft.setCursor(textX, y_pos + 4);
        tft.print(item.title);
        
        // Row 1 Right: Price
        String price = "G:" + String(item.price);
        int priceX = SCREEN_WIDTH - (price.length() * CHAR_WIDTH) - 2;
        tft.setCursor(priceX, y_pos + 4);
        tft.print(price);
        
        // Row 2: Info
        tft.setCursor(textX, y_pos + 18); 
        tft.setTextColor(isSelected ? ST77XX_BLACK : ST77XX_YELLOW, bg);
        String info = "Lvl:" + String(item.need) + " R:" + String(item.rarity);
        tft.print(info);
        
    } else {
        // --- 1-ROW MODE (Text Only) ---
        tft.setCursor(5, y_pos + 2);
        String title = item.title;
        String price = "G: " + String(item.price);
        int available_cols = COLS - price.length() - 2;
        if (title.length() > available_cols) {
            title = title.substring(0, available_cols) + "...";
        }
        tft.print(title);
        
        int price_x = (COLS - price.length() - 1) * CHAR_WIDTH;
        tft.setCursor(price_x, y_pos + 2);
        tft.print(price);
    }
    tft.endWrite();
}
void gem_drawMerchantList() {
    g_gemState = GEM_STATE_MERCHANT;
    int startY = (LINE_HEIGHT * 2);
    int footerY = SCREEN_HEIGHT - LINE_HEIGHT;

    tft.startWrite();
    tft.fillRect(0, startY, SCREEN_WIDTH, footerY - startY, ST77XX_DARKGREY);
    tft.endWrite();

    if (g_merchantItemCount == 0) {
        tft.startWrite();
        tft.setCursor(5, startY + 2);
        tft.setTextColor(ST77XX_WHITE, ST77XX_DARKGREY);
        tft.print("Merchant has no items. Select [New Item].");
        tft.endWrite();
    } else {
        // Draw visible items starting from top_item
        for (int i = g_merchant_list_top_item; i < g_merchantItemCount; i++) {
            // The draw function handles visibility checking internally now
            int y = gem_getMerchantItemY(i);
            if (y == -1) break; // Stop drawing if we go off screen
            gem_drawMerchantItem(i);
        }
    }
}
void gem_drawMerchantEditor(bool preserveData) {
    g_gemState = GEM_STATE_MERCHANT_EDIT;
    kmode = ALPHA;
    kbIndex = 0;

    tft.startWrite();
    tft.fillScreen(ST77XX_DARKGREY); 
    gem_drawHeader(true); 

    tft.setTextColor(ST77XX_WHITE, ST77XX_DARKGREY);
    tft.setCursor(5, (LINE_HEIGHT * 2));  tft.print("Item Title: (1 line max)");
    tft.setCursor(5, (LINE_HEIGHT * 4));  tft.print("Description: (3 lines max)");
    
    tft.setCursor(5, (LINE_HEIGHT * 8));  tft.print("Rarity (1-7):");
    tft.setCursor( (COLS/2) * CHAR_WIDTH, (LINE_HEIGHT * 8)); tft.print("Quality (1-7):");

    tft.setCursor(5, (LINE_HEIGHT * 10)); tft.print("Need (1-7):");
    tft.setCursor( (COLS/2) * CHAR_WIDTH, (LINE_HEIGHT * 10)); tft.print("Desire (1-7):");

    tft.setCursor(5, (LINE_HEIGHT * 12));  tft.print("Consumable:");

    // --- Ensure Editors Exist ---
    if (!gemCreateItemTitleEditor) gemCreateItemTitleEditor = new TextArea(5, (LINE_HEIGHT * 3), COLS - 2, 1, 2, 40);
    if (!gemCreateItemDescEditor) gemCreateItemDescEditor = new TextArea(5, (LINE_HEIGHT * 5), COLS - 2, 3, 50, 500);
    if (!gemCreateItemRarityEditor) gemCreateItemRarityEditor = new TextArea(5, (LINE_HEIGHT * 9), 10, 1, 2, 3);
    if (!gemCreateItemQualityEditor) gemCreateItemQualityEditor = new TextArea((COLS/2) * CHAR_WIDTH, (LINE_HEIGHT * 9), 10, 1, 2, 3);
    if (!gemCreateItemNeedEditor) gemCreateItemNeedEditor = new TextArea(5, (LINE_HEIGHT * 11), 10, 1, 2, 3);
    if (!gemCreateItemDesireEditor) gemCreateItemDesireEditor = new TextArea((COLS/2) * CHAR_WIDTH, (LINE_HEIGHT * 11), 10, 1, 2, 3);

    // --- DATA LOADING LOGIC ---
    if (!preserveData) {
        if (g_merchant_currentItemID != -1 && g_merchant_currentItemID < g_merchantItemCount) {
            MerchantItem& item = g_merchantDB[g_merchant_currentItemID];
            gemCreateItemTitleEditor->lines[0] = item.title; gemCreateItemTitleEditor->lineCount = 1;
            gem_load_notes_to_editor(gemCreateItemDescEditor, item.description);
            gemCreateItemRarityEditor->lines[0] = String(item.rarity); gemCreateItemRarityEditor->lineCount = 1;
            gemCreateItemQualityEditor->lines[0] = String(item.quality); gemCreateItemQualityEditor->lineCount = 1;
            gemCreateItemNeedEditor->lines[0] = String(item.need); gemCreateItemNeedEditor->lineCount = 1;
            gemCreateItemDesireEditor->lines[0] = String(item.desire); gemCreateItemDesireEditor->lineCount = 1;
            g_gem_editor_isConsumable = item.isConsumable;
            g_gem_temp_icon_file = item.iconFile; 
        } else {
            // New Item defaults
            gemCreateItemTitleEditor->lines[0] = ""; gemCreateItemTitleEditor->lineCount = 1;
            gemCreateItemDescEditor->lines[0] = ""; gemCreateItemDescEditor->lineCount = 1;
            gemCreateItemRarityEditor->lines[0] = ""; gemCreateItemRarityEditor->lineCount = 1;
            gemCreateItemQualityEditor->lines[0] = ""; gemCreateItemQualityEditor->lineCount = 1;
            gemCreateItemNeedEditor->lines[0] = ""; gemCreateItemNeedEditor->lineCount = 1;
            gemCreateItemDesireEditor->lines[0] = ""; gemCreateItemDesireEditor->lineCount = 1;
            g_gem_editor_isConsumable = false;
            g_gem_temp_icon_file = ""; 
        }
        
        gemCreateItemTitleEditor->cursorCol = 0; gemCreateItemTitleEditor->cursorLine = 0;
        gemCreateItemDescEditor->cursorCol = 0; gemCreateItemDescEditor->cursorLine = 0;
        gemCreateItemRarityEditor->cursorCol = 0; gemCreateItemRarityEditor->cursorLine = 0;
        gemCreateItemQualityEditor->cursorCol = 0; gemCreateItemQualityEditor->cursorLine = 0;
        gemCreateItemNeedEditor->cursorCol = 0; gemCreateItemNeedEditor->cursorLine = 0;
        gemCreateItemDesireEditor->cursorCol = 0; gemCreateItemDesireEditor->cursorLine = 0;
    }

    gemCreateItemTitleEditor->draw(tft);
    gemCreateItemDescEditor->draw(tft);
    gemCreateItemRarityEditor->draw(tft);
    gemCreateItemQualityEditor->draw(tft);
    gemCreateItemNeedEditor->draw(tft);
    gemCreateItemDesireEditor->draw(tft);

    // Draw Toggle
    int repeatX = 5;
    int repeatY = (LINE_HEIGHT * 13);
    String text = g_gem_editor_isConsumable ? "[ON]" : "[OFF]";
    int textWidth = text.length() * CHAR_WIDTH;
    uint16_t repeat_fg = g_gem_editor_isConsumable ? ST77XX_BLACK : ST77XX_WHITE;
    uint16_t repeat_bg = g_gem_editor_isConsumable ? ST77XX_GREEN : ST77XX_RED;
    tft.fillRect(repeatX, repeatY - 1, textWidth, LINE_HEIGHT, repeat_bg);
    tft.setCursor(repeatX, repeatY);
    tft.setTextColor(repeat_fg, repeat_bg);
    tft.print(text);

    // --- DRAW IMAGE SELECTOR ---
    tft.setTextColor(ST77XX_WHITE, ST77XX_DARKGREY);
    tft.setCursor(5, (LINE_HEIGHT * 14)); tft.print("Image:");
    
    int imgBoxX = 50; 
    int imgBoxY = (LINE_HEIGHT * 14);
    String displayIcon = (g_gem_temp_icon_file.length() > 0) ? g_gem_temp_icon_file : "None";
    
    uint16_t imgBg = (g_gemFocus == GEM_FOCUS_MERCHANT_CREATE_IMAGE) ? ST77XX_CYAN : ST77XX_BLACK;
    uint16_t imgFg = (g_gemFocus == GEM_FOCUS_MERCHANT_CREATE_IMAGE) ? ST77XX_BLACK : ST77XX_WHITE;
    
    tft.fillRect(imgBoxX, imgBoxY, 100, LINE_HEIGHT, imgBg);
    tft.setTextColor(imgFg, imgBg);
    tft.setCursor(imgBoxX + 2, imgBoxY);
    tft.print(displayIcon);

    // --- DRAW PREVIEW IMAGE (Centered at Bottom) ---
    if (displayIcon != "None") {
        int footerY = SCREEN_HEIGHT - LINE_HEIGHT; // Top of footer
        int contentBottomY = imgBoxY + LINE_HEIGHT; // Bottom of Image filename box
        int availableHeight = footerY - contentBottomY;
        int imgHeight = 64;
        
        if (availableHeight > imgHeight) {
            int imgX = (SCREEN_WIDTH - 64) / 2; // Center horizontally
            int imgY = contentBottomY + (availableHeight - imgHeight) / 2; // Center vertically
            
            drawItemBitmap(g_gem_temp_icon_file, imgX, imgY, 64, 64);
            tft.drawRect(imgX, imgY, 64, 64, ST77XX_WHITE);
        }
    }

    // Set focus flags
    gemCreateItemTitleEditor->isFocused = (g_gemFocus == GEM_FOCUS_MERCHANT_CREATE_TITLE);
    gemCreateItemDescEditor->isFocused  = (g_gemFocus == GEM_FOCUS_MERCHANT_CREATE_DESC);
    gemCreateItemRarityEditor->isFocused = (g_gemFocus == GEM_FOCUS_MERCHANT_CREATE_RARITY);
    gemCreateItemQualityEditor->isFocused = (g_gemFocus == GEM_FOCUS_MERCHANT_CREATE_QUALITY);
    gemCreateItemNeedEditor->isFocused = (g_gemFocus == GEM_FOCUS_MERCHANT_CREATE_NEED);
    gemCreateItemDesireEditor->isFocused = (g_gemFocus == GEM_FOCUS_MERCHANT_CREATE_DESIRE);
    
    gem_drawFooter(true);
    tft.endWrite();
}
void gem_refreshMerchantEditorView() {
    // 1. Redraw Text Editors
    if (gemCreateItemTitleEditor) gemCreateItemTitleEditor->draw(tft);
    if (gemCreateItemDescEditor) gemCreateItemDescEditor->draw(tft);
    if (gemCreateItemRarityEditor) gemCreateItemRarityEditor->draw(tft);
    if (gemCreateItemQualityEditor) gemCreateItemQualityEditor->draw(tft);
    if (gemCreateItemNeedEditor) gemCreateItemNeedEditor->draw(tft);
    if (gemCreateItemDesireEditor) gemCreateItemDesireEditor->draw(tft);

    // 2. Redraw Toggle
    int repeatX = 5;
    int repeatY = (LINE_HEIGHT * 13);
    String text = g_gem_editor_isConsumable ? "[ON]" : "[OFF]";
    int textWidth = text.length() * CHAR_WIDTH;
    uint16_t repeat_bg, repeat_fg;
    
    if (g_gemFocus == GEM_FOCUS_MERCHANT_CREATE_CONSUMABLE) {
        if (g_gem_editor_isConsumable) {
            repeat_bg = cursorVisible ? ST77XX_GREEN : ST77XX_DARKGREY;
            repeat_fg = cursorVisible ? ST77XX_BLACK : ST77XX_GREEN;
        } else {
            repeat_bg = cursorVisible ? ST77XX_RED : ST77XX_DARKGREY;
            repeat_fg = cursorVisible ? ST77XX_WHITE : ST77XX_RED;
        }
    } else {
        repeat_bg = g_gem_editor_isConsumable ? ST77XX_GREEN : ST77XX_RED;
        repeat_fg = g_gem_editor_isConsumable ? ST77XX_BLACK : ST77XX_WHITE;
    }
    
    tft.fillRect(repeatX, repeatY - 1, textWidth, LINE_HEIGHT, repeat_bg);
    tft.setCursor(repeatX, repeatY);
    tft.setTextColor(repeat_fg, repeat_bg);
    tft.print(text);

    // 3. Redraw Image Selector Box
    tft.setTextColor(ST77XX_WHITE, ST77XX_DARKGREY);
    tft.setCursor(5, (LINE_HEIGHT * 14)); 
    tft.print("Image:");
    
    int imgBoxX = 50; 
    int imgBoxY = (LINE_HEIGHT * 14);
    String displayIcon = (g_gem_temp_icon_file.length() > 0) ? g_gem_temp_icon_file : "None";
    
    uint16_t imgBg = (g_gemFocus == GEM_FOCUS_MERCHANT_CREATE_IMAGE) ? ST77XX_CYAN : ST77XX_BLACK;
    uint16_t imgFg = (g_gemFocus == GEM_FOCUS_MERCHANT_CREATE_IMAGE) ? ST77XX_BLACK : ST77XX_WHITE;
    
    tft.fillRect(imgBoxX, imgBoxY, 100, LINE_HEIGHT, imgBg);
    tft.setTextColor(imgFg, imgBg);
    tft.setCursor(imgBoxX + 2, imgBoxY);
    tft.print(displayIcon);

    // 4. Redraw Preview Image (Centered at Bottom)
    if (displayIcon != "None") {
        int footerY = SCREEN_HEIGHT - LINE_HEIGHT; // Top of footer
        int contentBottomY = imgBoxY + LINE_HEIGHT; // Bottom of Image filename box
        int availableHeight = footerY - contentBottomY;
        int imgHeight = 64;
        
        if (availableHeight > imgHeight) {
            int imgX = (SCREEN_WIDTH - 64) / 2; // Center horizontally
            int imgY = contentBottomY + (availableHeight - imgHeight) / 2; // Center vertically
            
            // Draw image from TEMP variable (since we are editing)
            drawItemBitmap(g_gem_temp_icon_file, imgX, imgY, 64, 64);
            tft.drawRect(imgX, imgY, 64, 64, ST77XX_WHITE);
        }
    }
}
void gem_returnToMerchantList() {
    g_gemState = GEM_STATE_MERCHANT;
    g_gemFocus = GEM_FOCUS_MERCHANT_LIST;
    // Reset selection to the top of the merchant list
    g_merchant_list_selection = 0;
    g_merchant_list_top_item = 0;

    // Redraw the entire merchant list screen
    tft.startWrite();
    gem_drawHeader(true);
    gem_drawMerchantList(); // This clears and draws the list
    gem_drawFooter(true);
    tft.endWrite();
    
    g_forceCursorRedraw = true; // Force cursor update in main loop
}
int gem_calculate_item_price(int rarity, int quality, int need, int desire, bool isConsumable) {
    // 1. Clamp values (1-7)
    if (rarity < 1) rarity = 1;
    if (quality < 1) quality = 1;
    if (need < 1) need = 1;
    if (desire < 1) desire = 1;

    // 2. Base price from Rarity and Quality
    // Min (1,1): (1*12.5) + (1*12.5) = 25
    // Max (7,7): (7*12.5) + (7*12.5) = 175
    float base_price = (rarity * 12.5f) + (quality * 12.5f);

    // 3. Multipliers from Need and Desire
    // Min (1,1): (1 + 1/5) = 1.2
    // Max (7,7): (1 + 7/5) = 2.4
    float need_multiplier = 1.0f + (need / 5.0f);
    float desire_multiplier = 1.0f + (desire / 5.0f);

    // 4. Calculate final price
    // Min (all 1s): 25 * 1.2 * 1.2 = 36
    // Max (all 7s): 175 * 2.4 * 2.4 = 1008
    float final_price = base_price * need_multiplier * desire_multiplier;

    // 5. Consumables are 50% cheaper
    if (isConsumable) {
        final_price *= 0.5f;
        // Min (all 1s, consumable): 36 * 0.5 = 18
        // Max (all 7s, consumable): 1008 * 0.5 = 504
        
        // --- FIX: Subtract the CONSUMABLE minimum (18) ---
        final_price = final_price - 18.0f;
    } else {
        // --- FIX: Subtract the NON-CONSUMABLE minimum (36) ---
        final_price = final_price - 36.0f;
    }

    // Return the final integer price, ensuring it's at least 1.
    return max(1, (int)final_price);
}
void addInventoryItem(String title, String desc, int rarity, int quality, int need, int desire, bool isConsumable) {
  if (g_inventoryItemCount >= MAX_INVENTORY_ITEMS) {
      pushSystemMessage("Inventory is full!");
      return;
  }
  MerchantItem& newItem = g_inventoryDB[g_inventoryItemCount];
  newItem.title = title;
  newItem.description = desc;
  
  // --- ADD NEW FIELDS ---
  newItem.rarity = rarity;
  newItem.quality = quality;
  newItem.need = need;
  newItem.desire = desire;
  newItem.isConsumable = isConsumable;
  // Recalculate and store the price
  newItem.price = gem_calculate_item_price(rarity, quality, need, desire, isConsumable);
  // --- END ADD ---

  g_inventoryItemCount++;
}
void gem_drawInventoryItem(int item_index) {
    if (item_index < 0 || item_index >= g_inventoryItemCount) return;

    int y_pos = gem_getInventoryItemY(item_index);
    if (y_pos == -1) return;

    int height = gem_getInventoryItemHeight(item_index);
    MerchantItem& item = g_inventoryDB[item_index];

    tft.startWrite();
    bool isSelected = (g_gemFocus == GEM_FOCUS_INVENTORY_LIST && item_index == g_inventory_list_selection);
    uint16_t fg = isSelected ? ST77XX_BLACK : ST77XX_WHITE;
    uint16_t bg = isSelected ? ST77XX_CYAN : ST77XX_DARKGREY;
    
    tft.fillRect(0, y_pos, SCREEN_WIDTH, height, bg);
    tft.setTextColor(fg, bg);

    if (height > (LINE_HEIGHT + 4)) {
        // --- 34px ROW MODE (32px Image) ---
        // Draw Icon (32x32) at offset (2, y+1)
        drawItemBitmap(item.iconFile, 2, y_pos + 1, 32, 32);
        
        // Draw Border (34x34) at offset (1, y)
        tft.drawRect(1, y_pos, 34, 34, isSelected ? ST77XX_BLACK : ST77XX_WHITE);
        
        int textX = 40;

        // Row 1
        tft.setCursor(textX, y_pos + 4);
        tft.print(item.title);
        
        String price = "S:" + String(max(1, (int)(item.price * 0.75)));
        int priceX = SCREEN_WIDTH - (price.length() * CHAR_WIDTH) - 2;
        tft.setCursor(priceX, y_pos + 4);
        tft.print(price);
        
        // Row 2
        tft.setCursor(textX, y_pos + 18);
        tft.setTextColor(isSelected ? ST77XX_BLACK : ST77XX_YELLOW, bg);
        String info = "Lvl:" + String(item.need) + " R:" + String(item.rarity);
        tft.print(info);
        
    } else {
        // --- 1-ROW MODE ---
        tft.setCursor(5, y_pos + 2);
        String title = item.title;
        String price = "Sell: " + String(max(1, (int)(item.price * 0.75)));
        int available_cols = COLS - price.length() - 2;
        if (title.length() > available_cols) title = title.substring(0, available_cols) + "...";
        tft.print(title);

        int price_x = (COLS - price.length() - 1) * CHAR_WIDTH;
        tft.setCursor(price_x, y_pos + 2);
        tft.print(price);
    }
    tft.endWrite();
}
void gem_drawInventoryList() {
    g_gemState = GEM_STATE_INVENTORY;
    int startY = (LINE_HEIGHT * 2); 
    int footerY = SCREEN_HEIGHT - LINE_HEIGHT;

    tft.startWrite();
    tft.fillRect(0, startY, SCREEN_WIDTH, footerY - startY, ST77XX_DARKGREY);
    tft.endWrite();
    
    if (g_inventoryItemCount == 0) {
        tft.startWrite();
        tft.setCursor(5, startY + 2);
        tft.setTextColor(ST77XX_WHITE, ST77XX_DARKGREY);
        tft.print("Inventory is empty.");
        tft.endWrite();
    } else {
        for (int i = g_inventory_list_top_item; i < g_inventoryItemCount; i++) {
            int y = gem_getInventoryItemY(i);
            if (y == -1) break;
            gem_drawInventoryItem(i);
        }
    }
}
void gem_deleteInventoryItem(int db_index) {
    if (db_index < 0 || db_index >= g_inventoryItemCount) return;
    
    // Shift items down
    for (int i = db_index; i < g_inventoryItemCount - 1; i++) {
        g_inventoryDB[i] = g_inventoryDB[i + 1];
    }
    
    g_inventoryItemCount--;
    
    // Clear the last item (ghost data cleanup)
    if (g_inventoryItemCount < MAX_INVENTORY_ITEMS) {
         g_inventoryDB[g_inventoryItemCount].title = "";
         g_inventoryDB[g_inventoryItemCount].description = "";
         g_inventoryDB[g_inventoryItemCount].iconFile = "";
         g_inventoryDB[g_inventoryItemCount].price = 0;
    }

    pushSystemMessage("Item discarded.");
    gem_save_database(g_gem_db_filename);
}
void gem_drawInventoryView() {
    g_gemState = GEM_STATE_INVENTORY_VIEW;
    g_gemFocus = GEM_FOCUS_FOOTER;
    kmode = ALPHA;
    kbIndex = 0;
    if (g_inventory_currentItemID < 0 || g_inventory_currentItemID >= g_inventoryItemCount) {
        g_gemState = GEM_STATE_INVENTORY;
        g_gemFocus = GEM_FOCUS_INVENTORY_LIST;
        gem_drawInventoryList();
        return;
    }

    MerchantItem& item = g_inventoryDB[g_inventory_currentItemID];

    tft.startWrite();
    tft.fillScreen(ST77XX_DARKGREY); 
    gem_drawHeader(true);

    // --- 1. Item Name/Title ---
    // Item Label: Set foreground color to CYAN (as requested)
    tft.setTextColor(ST77XX_CYAN, ST77XX_DARKGREY); 
    tft.setCursor(5, (LINE_HEIGHT * 2)); tft.print("Item:");
    
    // Item Title Value: Set foreground color to WHITE
    tft.setCursor(8, (LINE_HEIGHT * 3) + 2);
    tft.setTextColor(ST77XX_WHITE, ST77XX_DARKGREY);
    tft.print(item.title);

    // --- 2. Description Print ---
    tft.setCursor(5, (LINE_HEIGHT * 5));  
    // Description Label (White foreground)
    tft.setTextColor(ST77XX_CYAN, ST77XX_DARKGREY); 
    tft.print("Description:");
    
    tft.setCursor(8, (LINE_HEIGHT * 6) + 2); 
    
    // Description Text Value (White, Transparent Mode)
    tft.setTextColor(ST77XX_WHITE); 
    
    tft.setTextWrap(true); 
    tft.print(item.description);
    tft.setTextWrap(false); 
    
    // Reset to explicit background color mode for subsequent fields
    tft.setTextColor(ST77XX_WHITE, ST77XX_DARKGREY); 

    // --- Anchor the fixed value list starting at Row 11 ---
    const int Y_ORIGINAL_ANCHOR = (LINE_HEIGHT * 11); 
    const int X_VALUE_START = 150; 
    
    // Targeted Cleanup: Clear space between description area and fixed anchor
    int y_cleanup_start = (LINE_HEIGHT * 10);
    if (y_cleanup_start < Y_ORIGINAL_ANCHOR) {
        tft.fillRect(0, y_cleanup_start, SCREEN_WIDTH, Y_ORIGINAL_ANCHOR - y_cleanup_start, ST77XX_DARKGREY);
    }
    
    // --- 3. Original Value (Seamless, Fixed Y) ---
    int y_original_line = Y_ORIGINAL_ANCHOR; 
    
    // Print Label
    tft.setTextColor(ST77XX_WHITE, ST77XX_DARKGREY); 
    tft.setCursor(5, y_original_line); 
    tft.print("Original Value (Gems):");
    
    // Print Value (Same line, fixed X offset)
    tft.setCursor(X_VALUE_START, y_original_line);
    tft.setTextColor(ST77XX_YELLOW, ST77XX_DARKGREY);
    tft.print(item.price);
    
    // --- 4. Sell Value (Seamless, Relative to Original Value) ---
    int y_sell_line = y_original_line + LINE_HEIGHT + 4; 
    
    // Print Label
    tft.setCursor(5, y_sell_line); 
    tft.setTextColor(ST77XX_WHITE, ST77XX_DARKGREY); 
    tft.print("Sell Value (3/4):");
    
    // Print Value (Same line, fixed X offset)
    tft.setCursor(X_VALUE_START, y_sell_line);
    tft.setTextColor(ST77XX_GREEN, ST77XX_DARKGREY);
    tft.print(String(max(1, (int)(item.price * 0.75))));
    
    // --- FINAL CLEANUP and IMAGE DRAWING ---
    int y_current_bottom = y_sell_line + LINE_HEIGHT;
    int y_image_start = (LINE_HEIGHT * 16);
    if (y_current_bottom < y_image_start) {
         tft.fillRect(0, y_current_bottom, SCREEN_WIDTH, y_image_start - y_current_bottom, ST77XX_DARKGREY);
    }
    
    // --- 5. Draw Image (Centered Vertically in remaining space) ---
    if (item.iconFile.length() > 0) {
        int footerY = SCREEN_HEIGHT - LINE_HEIGHT; 
        int contentBottomY = y_image_start;
        
        int availableHeight = footerY - contentBottomY;
        int imgHeight = 64;
        
        if (availableHeight > imgHeight) {
            int imgX = (SCREEN_WIDTH - 64) / 2; 
            int imgY = contentBottomY + (availableHeight - imgHeight) / 2;
            
            drawItemBitmap(item.iconFile, imgX, imgY, 64, 64);
            tft.drawRect(imgX, imgY, 64, 64, ST77XX_WHITE);
        }
    }

    gem_drawFooter(true);
    tft.endWrite();
}
void gem_start() {
    // --- NEW: RESET ALL GLOBALS AT THE START ---
    g_questCount = 0;
    g_merchantItemCount = 0; 
    g_inventoryItemCount = 0;
    g_gem_count = 0;
    g_gem_level = 1;
    g_gem_total_xp = 0;
    g_skillCount = 0;
    g_playerSkillCount = 0;
    // --- END OF NEW RESET LOGIC ---

    g_currentApp = APP_STATE_GEM;
    g_gemState = GEM_STATE_QUEST_LIST;
    g_gemFocus = GEM_FOCUS_LIST;
    g_gem_list_selection = 0;
    g_gem_currentQuestID = -1;

    // --- DB Loading Logic ---
    int db_index = 0;
    bool loaded_successfully = false;
    const int MAX_DB_ATTEMPTS = 10; 

    while (!loaded_successfully && db_index < MAX_DB_ATTEMPTS) {
        if (db_index == 0) {
            g_gem_db_filename = "gem.db";
        } else {
            g_gem_db_filename = "gem" + String(db_index) + ".db";
        }

        if (!fsReady || !LittleFS.exists(g_gem_db_filename)) {
            pushSystemMessage("Creating new DB: " + g_gem_db_filename);
            gem_create_default_quests();
            gem_create_default_items();
            gem_save_database(g_gem_db_filename); 
            loaded_successfully = true;
            
        } else {
            pushSystemMessage("Attempting load: " + g_gem_db_filename);
            
            // --- THIS IS THE FIX ---
            if (gem_load_database(g_gem_db_filename)) { // Was g_grid_filename
            // --- END OF FIX ---
                
                pushSystemMessage("Successfully loaded: " + g_gem_db_filename);
                loaded_successfully = true;
            } else {
                pushSystemMessage("Warning: " + g_gem_db_filename + " corrupt/empty.");
                db_index++;
            }
        }
    }
    
    if (!loaded_successfully) {
        pushSystemMessage("All DBs failed! Creating default gem.db");
        g_gem_db_filename = "gem.db";
        gem_create_default_quests();
        gem_save_database(g_gem_db_filename);
    }
    // --- END DB Loading Logic ---
    
    gem_drawQuestList();
}
void gem_stop() {
    // --- NEW: RESET ALL GLOBALS ON EXIT ---
    // This clears the app's memory footprint when it closes.
    g_questCount = 0; 
    g_merchantItemCount = 0;
    g_inventoryItemCount = 0;
    g_gem_count = 0;
    g_gem_level = 1;
    g_gem_total_xp = 0;
    g_skillCount = 0;
    g_playerSkillCount = 0;
    g_gem_currentQuestID = -1; // Reset the active quest ID
    g_merchant_currentItemID = -1;
    g_inventory_currentItemID = -1;
    // --- END OF NEW RESET LOGIC ---

    g_currentApp = APP_STATE_CLI;

    // --- Clean up create-quest editors ---
    if (gemCreateTitleEditor) delete gemCreateTitleEditor;
    if (gemCreateDescEditor) delete gemCreateDescEditor;
    if (gemCreateSkillsEditor) delete gemCreateSkillsEditor;
    if (gemCreateDiffEditor) delete gemCreateDiffEditor;
    if (gemCreateDesireEditor) delete gemCreateDesireEditor;
    if (gemCreateTimeEditor) delete gemCreateTimeEditor;
    if (gemCreateAddCycleEditor) delete gemCreateAddCycleEditor;
    if (gemCreateMinsEditor) delete gemCreateMinsEditor;
    if (gemCreateItemTitleEditor) delete gemCreateItemTitleEditor;
    if (gemCreateItemDescEditor) delete gemCreateItemDescEditor;
    if (gemCreateItemPriceEditor) delete gemCreateItemPriceEditor;
    gemCreateTitleEditor = nullptr;
    gemCreateDescEditor = nullptr;
    gemCreateSkillsEditor = nullptr;
    gemCreateDiffEditor = nullptr;
    gemCreateDesireEditor = nullptr;
    gemCreateTimeEditor = nullptr;
    gemCreateAddCycleEditor = nullptr;
    gemCreateMinsEditor = nullptr;
    gemCreateItemTitleEditor = nullptr;
    gemCreateItemDescEditor = nullptr;
    gemCreateItemPriceEditor = nullptr;
    // ----------------------------------------

    tft.startWrite();
    tft.fillScreen(ST77XX_BLACK);
    tft.endWrite();

    invalidateTerminalCache();
    drawFullTerminal();
    clearCurrentCommand();
}
void gem_update(unsigned long now) {
    // --- Static variables for this function's timers ---
    static unsigned long lastBlink = 0;
    static bool cursorVisible = true;
    static unsigned long g_gem_lastBlinkCheck = 0;
    static int g_gem_blinks_remaining = 0;
    static unsigned long g_gem_next_blink = 0;
    
    bool needsDraw = false;
    // --- 1. 10-Second Auto-Save ---
    if (now - g_gem_lastMinuteSave > 10000) { 
        g_gem_lastMinuteSave = now;
        gem_save_database(g_gem_db_filename); 
        
        if (g_gemState == GEM_STATE_QUEST_LIST) {
            g_gem_needs_bar_redraw = true;
        }
    }
    
    // --- 2. 30-Second Blink Check (Unchanged) ---
    if (now - g_gem_lastBlinkCheck > 30000) {
        g_gem_lastBlinkCheck = now;
        int count = 0;
        for (int i = 0; i < g_questCount; i++) {
            if (g_questDB[i].questState == STATE_STARTED) {
                count++;
            }
        }
        if (count > 0) {
            g_gem_blinks_remaining = count;
            g_gem_next_blink = now; 
        }
    }
    
    // --- 3. Blink Sequence Handler (Unchanged) ---
    if (g_gem_blinks_remaining > 0 && now >= g_gem_next_blink) {
        g_gem_blinks_remaining--;
        g_gem_next_blink = now + 500; 
        digitalWrite(STATUS_LED_PIN, HIGH);
        ledBlinkEndTime = now + LED_BLINK_DURATION_MS;
    }

    // --- 4. Marquee Scroll Timer (Unchanged from your working version) ---
    if (g_gemState == GEM_STATE_QUEST_LIST && 
        g_gemFocus == GEM_FOCUS_LIST && 
        g_gem_marquee_active) 
    {
        const int MARQUEE_SPEED_MS = 300;
        if (now - g_gem_marquee_last_time > MARQUEE_SPEED_MS) {
            g_gem_marquee_last_time = now;
            g_gem_marquee_offset++; 
            
            tft.startWrite();
            // --- FIX: Pass 'true' so the bar persists while text scrolls ---
            gem_drawListItem(g_gem_list_selection, true);
            tft.endWrite();
        }
    }
    
    // --- 5. Cursor Blink Logic ---
    if (g_forceCursorRedraw) {
        lastBlink = now;
        cursorVisible = true;
        g_forceCursorRedraw = false;
        needsDraw = true;
    }
    else if (now - lastBlink >= BLINK_MS) { 
        lastBlink = now;
        cursorVisible = !cursorVisible;
        needsDraw = true;
    }

    // --- 6. Draw Cursor (AND 10-SECOND BAR UPDATE) ---
    if (needsDraw) {
        if (g_gemFocus == GEM_FOCUS_HEADER || g_gemFocus == GEM_FOCUS_FOOTER) {
            g_gem_marquee_active = false;
        }
        
        tft.startWrite();
        if (g_gem_needs_bar_redraw && g_gemState == GEM_STATE_QUEST_LIST) {
            gem_drawAllProgressBars();
            g_gem_needs_bar_redraw = false;
        }

        if (g_gemFocus == GEM_FOCUS_HEADER) {
            gem_drawHeader(cursorVisible);
        } else if (g_gemFocus == GEM_FOCUS_FOOTER) {
            gem_drawFooter(cursorVisible);
        } 
        else if (g_gemState == GEM_STATE_CREATE_QUEST) {
            TextArea* focusedEditor = nullptr;
            if (g_gemFocus == GEM_FOCUS_CREATE_TITLE) focusedEditor = gemCreateTitleEditor;
            else if (g_gemFocus == GEM_FOCUS_CREATE_DESC) focusedEditor = gemCreateDescEditor;
            else if (g_gemFocus == GEM_FOCUS_CREATE_SKILLS) focusedEditor = gemCreateSkillsEditor; 
            else if (g_gemFocus == GEM_FOCUS_CREATE_DIFF) focusedEditor = gemCreateDiffEditor;
            else if (g_gemFocus == GEM_FOCUS_CREATE_DESIRE) focusedEditor = gemCreateDesireEditor;
            else if (g_gemFocus == GEM_FOCUS_CREATE_TIME) focusedEditor = gemCreateTimeEditor;
            else if (g_gemFocus == GEM_FOCUS_CREATE_MINS) focusedEditor = gemCreateMinsEditor; // <-- FIX
            else if (g_gemFocus == GEM_FOCUS_CREATE_ADD_CYCLE) focusedEditor = gemCreateAddCycleEditor;
            
            if (gemCreateTitleEditor) gemCreateTitleEditor->isFocused = false;
            if (gemCreateDescEditor) gemCreateDescEditor->isFocused = false;
            if (gemCreateSkillsEditor) gemCreateSkillsEditor->isFocused = false;
            if (gemCreateDiffEditor) gemCreateDiffEditor->isFocused = false;
            if (gemCreateDesireEditor) gemCreateDesireEditor->isFocused = false;
            if (gemCreateTimeEditor) gemCreateTimeEditor->isFocused = false;
            if (gemCreateMinsEditor) gemCreateMinsEditor->isFocused = false;
            if (gemCreateAddCycleEditor) gemCreateAddCycleEditor->isFocused = false;

            // --- THIS IS THE FIX ---
            if (g_gemFocus == GEM_FOCUS_CREATE_REPEATABLE) {
                // Draw cursor for the repeatable toggle
                int repeatX = 5;
                int repeatY = (LINE_HEIGHT * 18);
                String text = g_gem_editor_isRepeatable ? "[ON]" : "[OFF]";
                int textWidth = text.length() * CHAR_WIDTH;
                uint16_t repeat_bg;
                uint16_t repeat_fg;
                if (g_gem_editor_isRepeatable) {
                    // [ON] State: Blink Green/Dark
                    repeat_bg = cursorVisible ? ST77XX_GREEN : ST77XX_DARKGREY;
                    repeat_fg = cursorVisible ? ST77XX_BLACK : ST77XX_GREEN;
                } else {
                    // [OFF] State: Blink Red/Dark
                    repeat_bg = cursorVisible ? ST77XX_RED : ST77XX_DARKGREY;
                    repeat_fg = cursorVisible ? ST77XX_WHITE : ST77XX_RED;
                }
                
                tft.fillRect(repeatX, repeatY - 1, textWidth, LINE_HEIGHT, repeat_bg);
                tft.setCursor(repeatX, repeatY);
                tft.setTextColor(repeat_fg, repeat_bg);
                tft.print(text);
                
                // Clear any old, long bar artifact
                int clearX = repeatX + textWidth;
                tft.fillRect(clearX, repeatY - 1, (COLS * CHAR_WIDTH) - clearX, LINE_HEIGHT, ST77XX_DARKGREY);
            } else if (focusedEditor) {
            // --- END OF FIX ---
            
                // Draw cursor for text areas
                focusedEditor->isFocused = true;
                focusedEditor->cursorVisible = cursorVisible;
                CursorDrawContext ctx = getCursorDrawContext(cursorVisible);
                
                focusedEditor->drawCursor(tft, ctx.preview, ctx.bgColor, ctx.fgColor);
            }
        }
        
        // --- ADD THIS ENTIRE BLOCK ---
        else if (g_gemState == GEM_STATE_MERCHANT_EDIT) {
            TextArea* focusedEditor = nullptr;
            if (g_gemFocus == GEM_FOCUS_MERCHANT_CREATE_TITLE) focusedEditor = gemCreateItemTitleEditor;
            else if (g_gemFocus == GEM_FOCUS_MERCHANT_CREATE_DESC) focusedEditor = gemCreateItemDescEditor;
            else if (g_gemFocus == GEM_FOCUS_MERCHANT_CREATE_RARITY) focusedEditor = gemCreateItemRarityEditor;
            else if (g_gemFocus == GEM_FOCUS_MERCHANT_CREATE_QUALITY) focusedEditor = gemCreateItemQualityEditor;
            else if (g_gemFocus == GEM_FOCUS_MERCHANT_CREATE_NEED) focusedEditor = gemCreateItemNeedEditor;
            else if (g_gemFocus == GEM_FOCUS_MERCHANT_CREATE_DESIRE) focusedEditor = gemCreateItemDesireEditor;

            // Set all to unfocused first
            if (gemCreateItemTitleEditor) gemCreateItemTitleEditor->isFocused = false;
            if (gemCreateItemDescEditor) gemCreateItemDescEditor->isFocused = false;
            if (gemCreateItemRarityEditor) gemCreateItemRarityEditor->isFocused = false;
            if (gemCreateItemQualityEditor) gemCreateItemQualityEditor->isFocused = false;
            if (gemCreateItemNeedEditor) gemCreateItemNeedEditor->isFocused = false;
            if (gemCreateItemDesireEditor) gemCreateItemDesireEditor->isFocused = false;
            if (gemCreateItemPriceEditor) gemCreateItemPriceEditor->isFocused = false; 

            if (g_gemFocus == GEM_FOCUS_MERCHANT_CREATE_CONSUMABLE) {
                // Draw cursor for the consumable toggle
                int repeatX = 5;
                int repeatY = (LINE_HEIGHT * 13); // Y position from draw function
                String text = g_gem_editor_isConsumable ? "[ON]" : "[OFF]";
                int textWidth = text.length() * CHAR_WIDTH;
                uint16_t repeat_bg;
                uint16_t repeat_fg;
                if (g_gem_editor_isConsumable) {
                    repeat_bg = cursorVisible ? ST77XX_GREEN : ST77XX_DARKGREY;
                    repeat_fg = cursorVisible ? ST77XX_BLACK : ST77XX_GREEN;
                } else {
                    repeat_bg = cursorVisible ? ST77XX_RED : ST77XX_DARKGREY;
                    repeat_fg = cursorVisible ? ST77XX_WHITE : ST77XX_RED;
                }
                
                tft.fillRect(repeatX, repeatY - 1, textWidth, LINE_HEIGHT, repeat_bg);
                tft.setCursor(repeatX, repeatY);
                tft.setTextColor(repeat_fg, repeat_bg);
                tft.print(text);
                
                // Clear any old, long bar artifact
                int clearX = repeatX + textWidth;
                tft.fillRect(clearX, repeatY - 1, (COLS * CHAR_WIDTH) - clearX, LINE_HEIGHT, ST77XX_DARKGREY);
            }
            else if (focusedEditor) {
                // Now set the correct one to focused and draw its cursor
                focusedEditor->isFocused = true;
                focusedEditor->cursorVisible = cursorVisible;
                CursorDrawContext ctx = getCursorDrawContext(cursorVisible);
                focusedEditor->drawCursor(tft, ctx.preview, ctx.bgColor, ctx.fgColor);
            }
        }
        // --- END OF ADDED BLOCK ---
        
        tft.endWrite();
    }
}
void gem_handleKey(uint8_t c) {
    TextArea* focusedEditor = nullptr;
    if (g_gemFocus == GEM_FOCUS_CREATE_TITLE) focusedEditor = gemCreateTitleEditor;
    else if (g_gemFocus == GEM_FOCUS_CREATE_DESC) focusedEditor = gemCreateDescEditor;
    else if (g_gemFocus == GEM_FOCUS_CREATE_SKILLS) focusedEditor = gemCreateSkillsEditor;
    else if (g_gemFocus == GEM_FOCUS_CREATE_DIFF) focusedEditor = gemCreateDiffEditor;
    else if (g_gemFocus == GEM_FOCUS_CREATE_DESIRE) focusedEditor = gemCreateDesireEditor;
    else if (g_gemFocus == GEM_FOCUS_CREATE_TIME) focusedEditor = gemCreateTimeEditor;
    else if (g_gemFocus == GEM_FOCUS_CREATE_MINS) focusedEditor = gemCreateMinsEditor;
    else if (g_gemFocus == GEM_FOCUS_CREATE_ADD_CYCLE) focusedEditor = gemCreateAddCycleEditor;
    else if (g_gemFocus == GEM_FOCUS_MERCHANT_CREATE_TITLE) focusedEditor = gemCreateItemTitleEditor;
    else if (g_gemFocus == GEM_FOCUS_MERCHANT_CREATE_DESC) focusedEditor = gemCreateItemDescEditor;
    else if (g_gemFocus == GEM_FOCUS_MERCHANT_CREATE_RARITY) focusedEditor = gemCreateItemRarityEditor;
    else if (g_gemFocus == GEM_FOCUS_MERCHANT_CREATE_QUALITY) focusedEditor = gemCreateItemQualityEditor;
    else if (g_gemFocus == GEM_FOCUS_MERCHANT_CREATE_NEED) focusedEditor = gemCreateItemNeedEditor;
    else if (g_gemFocus == GEM_FOCUS_MERCHANT_CREATE_DESIRE) focusedEditor = gemCreateItemDesireEditor;
    
    // --- 2. Image Selection State Input ---
    if (g_gemState == GEM_STATE_IMAGE_SELECT) {
        int itemsPerCol = (SCREEN_HEIGHT - (LINE_HEIGHT + 6) - LINE_HEIGHT) / LINE_HEIGHT;
        int maxPerPage = itemsPerCol * 2;
        
        if (c == KEY_UP_ARROW) {
            if (g_gem_file_selection > 0) {
                g_gem_file_selection--;
                if (g_gem_file_selection < g_gem_file_top_index) g_gem_file_top_index -= maxPerPage;
                if (g_gem_file_top_index < 0) g_gem_file_top_index = 0;
                gem_drawImageSelect();
            }
        } else if (c == KEY_DOWN_ARROW) {
            if (g_gem_file_selection < g_gem_file_count - 1) {
                g_gem_file_selection++;
                if (g_gem_file_selection >= g_gem_file_top_index + maxPerPage) g_gem_file_top_index += maxPerPage;
                gem_drawImageSelect();
            }
        } else if (c == KEY_LEFT_ARROW) {
             if (g_gem_file_selection >= itemsPerCol) {
                 g_gem_file_selection -= itemsPerCol;
                 if (g_gem_file_selection < g_gem_file_top_index) g_gem_file_top_index -= maxPerPage;
                 gem_drawImageSelect();
             }
        } else if (c == KEY_RIGHT_ARROW) {
             if (g_gem_file_selection + itemsPerCol < g_gem_file_count) {
                 g_gem_file_selection += itemsPerCol;
                 if (g_gem_file_selection >= g_gem_file_top_index + maxPerPage) g_gem_file_top_index += maxPerPage;
                 gem_drawImageSelect();
             }
        } else if (c == KEY_ESCAPE) {
            // Cancel selection, return to editor, PRESERVE DATA
            // FIX: Check FOCUS, not STATE (State is currently IMAGE_SELECT)
            if (g_gemFocus == GEM_FOCUS_CREATE_IMAGE) {
                g_gemState = GEM_STATE_CREATE_QUEST;
                gem_drawQuestEditor(true); // Pass true to keep text
            } else {
                g_gemState = GEM_STATE_MERCHANT_EDIT;
                g_gemFocus = GEM_FOCUS_MERCHANT_CREATE_IMAGE; 
                gem_drawMerchantEditor(true); 
            }
        } else if (c == 0x0D) { // Enter
            // Confirm selection: Update Temp Variable
            if (g_gem_file_selection >= 0 && g_gem_file_selection < g_gem_file_count) {
                 g_gem_temp_icon_file = g_gem_file_list[g_gem_file_selection];
                 g_gem_temp_quest_icon = g_gem_file_list[g_gem_file_selection]; // Update quest temp
            }
            // Return to editor, PRESERVE DATA
            // FIX: Check FOCUS, not STATE
            if (g_gemFocus == GEM_FOCUS_CREATE_IMAGE) {
                g_gemState = GEM_STATE_CREATE_QUEST;
                gem_drawQuestEditor(true); // Pass true to keep text
            } else {
                g_gemState = GEM_STATE_MERCHANT_EDIT;
                g_gemFocus = GEM_FOCUS_MERCHANT_CREATE_IMAGE;
                gem_drawMerchantEditor(true); 
            }
        }
        return;
    }

    // --- 3. Handle ESCAPE (Global Back/Exit) ---
    if (c == KEY_ESCAPE) {
        if (g_gem_autocomplete_active) {
            tft.startWrite();
            gem_hide_autocomplete(true);
            tft.endWrite();
        }

        if (g_gemState == GEM_STATE_QUEST_LIST) {
            gem_stop();
        } 
        else if (g_gemState == GEM_STATE_MERCHANT_EDIT) {
            g_gemState = GEM_STATE_MERCHANT;
            g_gemFocus = GEM_FOCUS_MERCHANT_LIST;
            tft.startWrite(); 
            gem_drawHeader(true);
            gem_drawMerchantList(); 
            gem_drawFooter(true);
            tft.endWrite();
            g_forceCursorRedraw = true;
        } 
        else if (g_gemState == GEM_STATE_INVENTORY) {
            g_gemFocus = GEM_FOCUS_FOOTER;
            g_gem_footer_selection = 0;
            gem_drawStatisticsPage();
            g_forceCursorRedraw = true;
        }
        else if (g_gemState == GEM_STATE_INVENTORY_VIEW) {
            g_gemState = GEM_STATE_INVENTORY;
            g_gemFocus = GEM_FOCUS_INVENTORY_LIST;
            g_inventory_list_selection = 0; 
            g_inventory_list_top_item = 0;
            tft.startWrite();
            gem_drawHeader(true);
            gem_drawInventoryList();
            gem_drawFooter(true);
            tft.endWrite();
            g_forceCursorRedraw = true;
        }
        else if (g_gemState == GEM_STATE_STATS ||
                 g_gemState == GEM_STATE_MERCHANT ||
                 g_gemState == GEM_STATE_TYPE_SELECT ||
                 g_gemState == GEM_STATE_CREATE_QUEST) 
        {
            gem_start();
        }
        else {
            gem_start();
        }
        return;
    } 
    
    // --- 4. Handle BACKSPACE (Delete / Back Navigation) ---
    else if (c == 0x08) { 
        if (focusedEditor) {
            tft.startWrite();
            focusedEditor->backspace(tft);
            tft.endWrite();
            if (g_gemFocus == GEM_FOCUS_CREATE_SKILLS) {
                gem_update_autocomplete(focusedEditor);
            }
        } 
        else if (g_gemState == GEM_STATE_STATS && g_gemFocus == GEM_FOCUS_STATS_LIST) {
            if (g_playerSkillCount > 0 && g_gem_stats_selection < g_playerSkillCount) {
                int indexToDelete = g_gem_stats_selection;
                String deletedSkillName = gem_deleteSkill(indexToDelete);
                
                if (g_gem_stats_selection >= g_playerSkillCount) g_gem_stats_selection = g_playerSkillCount - 1;
                if (g_gem_stats_selection < 0) g_gem_stats_selection = 0;
                if (g_gem_stats_top_skill >= g_playerSkillCount) g_gem_stats_top_skill = g_playerSkillCount - 1;
                if (g_gem_stats_top_skill < 0) g_gem_stats_top_skill = 0;
                
                gem_drawStatisticsPage();
                if (deletedSkillName.length() > 0) {
                    pushSystemMessage("Skill '" + deletedSkillName + "' deleted.");
                }
            }
        }
        else if (g_gemState == GEM_STATE_MERCHANT && g_gemFocus == GEM_FOCUS_MERCHANT_LIST) {
             if (g_merchantItemCount > 0 && g_merchant_list_selection < g_merchantItemCount) {
                int indexToDelete = g_merchant_list_selection;
                gem_deleteMerchantItem(indexToDelete);

                if (g_merchant_list_selection >= g_merchantItemCount) g_merchant_list_selection = g_merchantItemCount - 1;
                if (g_merchant_list_selection < 0) g_merchant_list_selection = 0;
                if (g_merchant_list_top_item >= g_merchantItemCount) g_merchant_list_top_item = g_merchantItemCount - 1;
                if (g_merchant_list_top_item < 0) g_merchant_list_top_item = 0;
                
                tft.startWrite();
                gem_drawHeader(true);
                gem_drawMerchantList();
                gem_drawFooter(true);
                tft.endWrite();
            }
        }
        else if (g_gemState == GEM_STATE_INVENTORY && g_gemFocus == GEM_FOCUS_INVENTORY_LIST) {
             if (g_inventoryItemCount > 0 && g_inventory_list_selection < g_inventoryItemCount) {
                int indexToDelete = g_inventory_list_selection;
                gem_deleteInventoryItem(indexToDelete);

                if (g_inventory_list_selection >= g_inventoryItemCount) g_inventory_list_selection = g_inventoryItemCount - 1;
                if (g_inventory_list_selection < 0) g_inventory_list_selection = 0;
                if (g_inventory_list_top_item >= g_inventoryItemCount) g_inventory_list_top_item = g_inventoryItemCount - 1;
                if (g_inventory_list_top_item < 0) g_inventory_list_top_item = 0;

                tft.startWrite();
                gem_drawHeader(true);
                gem_drawInventoryList();
                gem_drawFooter(true);
                tft.endWrite();
            }
        }
        return;
    }

    // --- 5. Handle Text Editor Navigation & Typing ---
    if (focusedEditor) {
        bool navigateUp = false;
        bool navigateDown = false;
        bool didType = false;
        bool autoAdvance = false;
        
        if (c == KEY_UP_ARROW) {
            if (g_gem_autocomplete_active) {
                g_gem_autocomplete_selection--;
                if (g_gem_autocomplete_selection < 0) g_gem_autocomplete_selection = g_gem_autocomplete_count - 1;
                tft.startWrite();
                gem_draw_autocomplete();
                tft.endWrite();
                g_forceCursorRedraw = true;
            } else {
                bool isMultiLine = (g_gemFocus == GEM_FOCUS_CREATE_DESC || g_gemFocus == GEM_FOCUS_CREATE_SKILLS || g_gemFocus == GEM_FOCUS_MERCHANT_CREATE_DESC);
                if (isMultiLine && (focusedEditor->cursorLine > focusedEditor->topLine)) {
                    tft.startWrite();
                    focusedEditor->moveCursor(0, -1);
                    tft.endWrite();
                } else {
                    navigateUp = true;
                }
            }
        }
        else if (c == KEY_DOWN_ARROW) {
            if (g_gem_autocomplete_active) {
                g_gem_autocomplete_selection++;
                if (g_gem_autocomplete_selection >= g_gem_autocomplete_count) g_gem_autocomplete_selection = 0;
                tft.startWrite();
                gem_draw_autocomplete();
                tft.endWrite();
                g_forceCursorRedraw = true;
            } else {
                bool isMultiLine = (g_gemFocus == GEM_FOCUS_CREATE_DESC || g_gemFocus == GEM_FOCUS_CREATE_SKILLS || g_gemFocus == GEM_FOCUS_MERCHANT_CREATE_DESC);
                int lastVisibleLine = focusedEditor->topLine + focusedEditor->numRows - 1;
                
                if (isMultiLine && focusedEditor->cursorLine < (focusedEditor->lineCount - 1) && focusedEditor->cursorLine < lastVisibleLine) {
                    tft.startWrite();
                    focusedEditor->moveCursor(0, 1);
                    tft.endWrite();
                } else {
                    navigateDown = true;
                }
            }
        }
        else if (c == 0x0D) { // Enter
            bool isMultiLine = (g_gemFocus == GEM_FOCUS_CREATE_DESC || g_gemFocus == GEM_FOCUS_CREATE_SKILLS || g_gemFocus == GEM_FOCUS_MERCHANT_CREATE_DESC);
            if (g_gem_autocomplete_active) {
                gem_select_autocomplete(focusedEditor);
            } else if (isMultiLine) {
                tft.startWrite();
                if (focusedEditor->lineCount >= focusedEditor->numRows) {
                    navigateDown = true;
                } else if (!focusedEditor->insertLine(tft)) {
                    navigateDown = true;
                }
                tft.endWrite();
            } else {
                navigateDown = true;
            }
        } 
        else if (c == KEY_LEFT_ARROW) {
            tft.startWrite();
            focusedEditor->moveCursor(-1, 0);
            tft.endWrite();
        }
        else if (c == KEY_RIGHT_ARROW) {
            tft.startWrite();
            focusedEditor->moveCursor(1, 0);
            tft.endWrite();
        }
        else if (c >= 0x20 && c <= 0x7E) { // Printable ASCII
            tft.startWrite();
            didType = true;

            // Numeric-only fields
            if (g_gemFocus == GEM_FOCUS_CREATE_DIFF || g_gemFocus == GEM_FOCUS_CREATE_DESIRE ||
                g_gemFocus == GEM_FOCUS_MERCHANT_CREATE_RARITY ||
                g_gemFocus == GEM_FOCUS_MERCHANT_CREATE_QUALITY ||
                g_gemFocus == GEM_FOCUS_MERCHANT_CREATE_NEED ||
                g_gemFocus == GEM_FOCUS_MERCHANT_CREATE_DESIRE) {
                if ((char)c >= '1' && (char)c <= '7' && focusedEditor->lines[0].length() < 1) {
                     focusedEditor->insertChar(tft, (char)c);
                     autoAdvance = true; 
                } else {
                     didType = false;
                }
            } 
            else if (g_gemFocus == GEM_FOCUS_CREATE_TIME || g_gemFocus == GEM_FOCUS_CREATE_MINS || g_gemFocus == GEM_FOCUS_CREATE_ADD_CYCLE) {
                if (isDigit((char)c)) {
                    String newNum = focusedEditor->lines[0] + (char)c;
                    if (newNum.length() <= 3 && newNum.toInt() <= 100) {
                        focusedEditor->insertChar(tft, (char)c);
                    } else {
                        didType = false;
                    }
                } else {
                     didType = false;
                }
            } 
            else if (g_gemFocus == GEM_FOCUS_MERCHANT_CREATE_PRICE) {
                if (isDigit((char)c) && focusedEditor->lines[0].length() < 6) {
                     focusedEditor->insertChar(tft, (char)c);
                } else {
                     didType = false;
                }
            }
            else { // Title, Desc, Skills
                 focusedEditor->insertChar(tft, (char)c);
            }
            tft.endWrite();
        }
        
        // Handle Auto-Advance / Navigation
        if (navigateUp) {
            GemAppFocus oldFocus = g_gemFocus;
            if (g_gemFocus == GEM_FOCUS_FOOTER) {
               if (g_gemState == GEM_STATE_CREATE_QUEST) {
                   // --- FIX: Nav Up from Footer goes to Image ---
                   g_gemFocus = GEM_FOCUS_CREATE_IMAGE;
               } else if (g_gemState == GEM_STATE_MERCHANT_EDIT) {
                   g_gemFocus = GEM_FOCUS_MERCHANT_CREATE_IMAGE;
               }
            }
            // Quest Creator Up Navigation
            else if (g_gemFocus == GEM_FOCUS_CREATE_IMAGE) { // --- ADDED ---
                 if (g_gem_editor_questType == QT_STANDARD) g_gemFocus = GEM_FOCUS_CREATE_REPEATABLE;
                 else g_gemFocus = GEM_FOCUS_CREATE_ADD_CYCLE;
            }
            else if (g_gemFocus == GEM_FOCUS_CREATE_REPEATABLE) { g_gemFocus = GEM_FOCUS_CREATE_MINS;
            } 
            else if (g_gemFocus == GEM_FOCUS_CREATE_ADD_CYCLE) { g_gemFocus = GEM_FOCUS_CREATE_MINS;
            } 
            else if (g_gemFocus == GEM_FOCUS_CREATE_MINS) { g_gemFocus = GEM_FOCUS_CREATE_TIME;
            } 
            else if (g_gemFocus == GEM_FOCUS_CREATE_TIME) { g_gemFocus = GEM_FOCUS_CREATE_DESIRE;
            } 
            else if (g_gemFocus == GEM_FOCUS_CREATE_DESIRE) { g_gemFocus = GEM_FOCUS_CREATE_DIFF;
            } 
            else if (g_gemFocus == GEM_FOCUS_CREATE_DIFF) { g_gemFocus = GEM_FOCUS_CREATE_SKILLS;
            }
            else if (g_gemFocus == GEM_FOCUS_CREATE_SKILLS) { g_gemFocus = GEM_FOCUS_CREATE_DESC;
            }
            else if (g_gemFocus == GEM_FOCUS_CREATE_DESC) { g_gemFocus = GEM_FOCUS_CREATE_TITLE;
            }
            else if (g_gemFocus == GEM_FOCUS_CREATE_TITLE) { g_gemFocus = GEM_FOCUS_HEADER;
            } 
            
            // Merchant Editor Up Navigation
            else if (g_gemFocus == GEM_FOCUS_MERCHANT_CREATE_IMAGE) { g_gemFocus = GEM_FOCUS_MERCHANT_CREATE_CONSUMABLE;
            }
            else if (g_gemFocus == GEM_FOCUS_MERCHANT_CREATE_CONSUMABLE) { g_gemFocus = GEM_FOCUS_MERCHANT_CREATE_DESIRE;
            }
            else if (g_gemFocus == GEM_FOCUS_MERCHANT_CREATE_DESIRE) { g_gemFocus = GEM_FOCUS_MERCHANT_CREATE_NEED;
            }
            else if (g_gemFocus == GEM_FOCUS_MERCHANT_CREATE_NEED) { g_gemFocus = GEM_FOCUS_MERCHANT_CREATE_QUALITY;
            }
            else if (g_gemFocus == GEM_FOCUS_MERCHANT_CREATE_QUALITY) { g_gemFocus = GEM_FOCUS_MERCHANT_CREATE_RARITY;
            }
            else if (g_gemFocus == GEM_FOCUS_MERCHANT_CREATE_RARITY) { g_gemFocus = GEM_FOCUS_MERCHANT_CREATE_DESC;
            }
            else if (g_gemFocus == GEM_FOCUS_MERCHANT_CREATE_DESC) { g_gemFocus = GEM_FOCUS_MERCHANT_CREATE_TITLE;
            }
            
            if (oldFocus != g_gemFocus) {
                 if (oldFocus == GEM_FOCUS_CREATE_SKILLS) {
                     tft.startWrite();
                     gem_hide_autocomplete(true);
                     focusedEditor->isFocused = false;
                     CursorDrawContext ctx = getCursorDrawContext(false);
                     focusedEditor->drawCursor(tft, ctx.preview, ctx.bgColor, ctx.fgColor);
                     g_gem_last_ac_y = 0; g_gem_last_ac_h = 0;
                     g_gem_last_ac_x = 0; g_gem_last_ac_w = 0;
                     tft.endWrite();
                 } else if (g_gemState == GEM_STATE_CREATE_QUEST) {
                     gem_refreshCreateQuestView();
                 } else if (g_gemState == GEM_STATE_MERCHANT_EDIT) {
                     tft.startWrite();
                     gem_refreshMerchantEditorView(); 
                     tft.endWrite();
                 }
                g_forceCursorRedraw = true;
            }
        } 
        else if (navigateDown || autoAdvance) {
            GemAppFocus oldFocus = g_gemFocus;
            if (g_gemFocus == GEM_FOCUS_HEADER) {
                if (g_gemState == GEM_STATE_CREATE_QUEST) g_gemFocus = GEM_FOCUS_CREATE_TITLE;
                else if (g_gemState == GEM_STATE_MERCHANT_EDIT) g_gemFocus = GEM_FOCUS_MERCHANT_CREATE_TITLE;
            }
            // Quest Creator Down Logic
            else if (g_gemFocus == GEM_FOCUS_CREATE_TITLE) { g_gemFocus = GEM_FOCUS_CREATE_DESC;
            }
            else if (g_gemFocus == GEM_FOCUS_CREATE_DESC) { g_gemFocus = GEM_FOCUS_CREATE_SKILLS;
            }
            else if (g_gemFocus == GEM_FOCUS_CREATE_SKILLS) { g_gemFocus = GEM_FOCUS_CREATE_DIFF;
            }
            else if (g_gemFocus == GEM_FOCUS_CREATE_DIFF) { g_gemFocus = GEM_FOCUS_CREATE_DESIRE;
            }
            else if (g_gemFocus == GEM_FOCUS_CREATE_DESIRE) { g_gemFocus = GEM_FOCUS_CREATE_TIME;
            } 
            else if (g_gemFocus == GEM_FOCUS_CREATE_TIME) { g_gemFocus = GEM_FOCUS_CREATE_MINS;
            }
            else if (g_gemFocus == GEM_FOCUS_CREATE_MINS) { 
                if (g_gem_editor_questType == QT_STANDARD) g_gemFocus = GEM_FOCUS_CREATE_REPEATABLE;
                else g_gemFocus = GEM_FOCUS_CREATE_ADD_CYCLE;
            }
            // --- FIX: Navigate to Image instead of Footer ---
            else if (g_gemFocus == GEM_FOCUS_CREATE_REPEATABLE) { g_gemFocus = GEM_FOCUS_CREATE_IMAGE;
            }
            else if (g_gemFocus == GEM_FOCUS_CREATE_ADD_CYCLE) { g_gemFocus = GEM_FOCUS_CREATE_IMAGE;
            }
            else if (g_gemFocus == GEM_FOCUS_CREATE_IMAGE) { g_gemFocus = GEM_FOCUS_FOOTER;
            }
            
            // Merchant Editor Down Logic
            else if (g_gemFocus == GEM_FOCUS_MERCHANT_CREATE_TITLE) { g_gemFocus = GEM_FOCUS_MERCHANT_CREATE_DESC;
            }
            else if (g_gemFocus == GEM_FOCUS_MERCHANT_CREATE_DESC) { g_gemFocus = GEM_FOCUS_MERCHANT_CREATE_RARITY;
            }
            else if (g_gemFocus == GEM_FOCUS_MERCHANT_CREATE_RARITY) { g_gemFocus = GEM_FOCUS_MERCHANT_CREATE_QUALITY;
            }
            else if (g_gemFocus == GEM_FOCUS_MERCHANT_CREATE_QUALITY) { g_gemFocus = GEM_FOCUS_MERCHANT_CREATE_NEED;
            }
            else if (g_gemFocus == GEM_FOCUS_MERCHANT_CREATE_NEED) { g_gemFocus = GEM_FOCUS_MERCHANT_CREATE_DESIRE;
            }
            else if (g_gemFocus == GEM_FOCUS_MERCHANT_CREATE_DESIRE) { g_gemFocus = GEM_FOCUS_MERCHANT_CREATE_CONSUMABLE;
            }
            else if (g_gemFocus == GEM_FOCUS_MERCHANT_CREATE_CONSUMABLE) { g_gemFocus = GEM_FOCUS_MERCHANT_CREATE_IMAGE;
            }
            else if (g_gemFocus == GEM_FOCUS_MERCHANT_CREATE_IMAGE) { g_gemFocus = GEM_FOCUS_FOOTER;
            }

            if (oldFocus != g_gemFocus) {
                if (oldFocus == GEM_FOCUS_CREATE_SKILLS) {
                    tft.startWrite();
                    gem_hide_autocomplete(true);
                    focusedEditor->isFocused = false;
                    CursorDrawContext ctx = getCursorDrawContext(false);
                    focusedEditor->drawCursor(tft, ctx.preview, ctx.bgColor, ctx.fgColor);
                    g_gem_last_ac_y = 0; g_gem_last_ac_h = 0;
                    g_gem_last_ac_x = 0; g_gem_last_ac_w = 0;
                    tft.endWrite();
                } else if (g_gemState == GEM_STATE_CREATE_QUEST) {
                    gem_refreshCreateQuestView();
                } else if (g_gemState == GEM_STATE_MERCHANT_EDIT) {
                     tft.startWrite();
                     gem_refreshMerchantEditorView(); 
                     tft.endWrite();
                }

                if (g_gemFocus == GEM_FOCUS_FOOTER) {
                    g_gem_footer_selection = 0;
                    tft.startWrite();
                    gem_drawFooter(true);
                    tft.endWrite();
                }
                g_forceCursorRedraw = true;
            }
        } else {
            g_forceCursorRedraw = true;
        }
        
        if (didType && g_gemFocus == GEM_FOCUS_CREATE_SKILLS) {
            gem_update_autocomplete(focusedEditor);
        }
    } 
    // --- 6. Handle MENUS / LISTS / TOGGLES (Non-Text Editor) ---
    else {
        // --- A. Repeatable Toggle ---
        if (g_gemFocus == GEM_FOCUS_CREATE_REPEATABLE) {
            if (c == 0x0D || c == KEY_RIGHT_ARROW || c == KEY_LEFT_ARROW) {
                g_gem_editor_isRepeatable = !g_gem_editor_isRepeatable;
                gem_refreshCreateQuestView();
                g_forceCursorRedraw = true;
            } else if (c == KEY_UP_ARROW) {
                g_gemFocus = GEM_FOCUS_CREATE_MINS;
                gem_refreshCreateQuestView();
                g_forceCursorRedraw = true;
            } else if (c == KEY_DOWN_ARROW) {
                // --- FIX: Go to Image ---
                g_gemFocus = GEM_FOCUS_CREATE_IMAGE;
                gem_refreshCreateQuestView();
                g_forceCursorRedraw = true;
            }
        }
        // --- B. Consumable Toggle ---
        else if (g_gemFocus == GEM_FOCUS_MERCHANT_CREATE_CONSUMABLE) {
            if (c == 0x0D || c == KEY_RIGHT_ARROW || c == KEY_LEFT_ARROW) {
                 g_gem_editor_isConsumable = !g_gem_editor_isConsumable;
                 tft.startWrite();
                 gem_refreshMerchantEditorView();
                 tft.endWrite();
                 g_forceCursorRedraw = true;
            } else if (c == KEY_UP_ARROW) {
                g_gemFocus = GEM_FOCUS_MERCHANT_CREATE_DESIRE;
                tft.startWrite();
                gem_refreshMerchantEditorView();
                tft.endWrite();
                g_forceCursorRedraw = true;
            } else if (c == KEY_DOWN_ARROW) {
                g_gemFocus = GEM_FOCUS_MERCHANT_CREATE_IMAGE;
                tft.startWrite();
                gem_refreshMerchantEditorView();
                tft.endWrite();
                g_forceCursorRedraw = true;
            }
        }
        // --- C. Image Selection Button (Merchant) ---
        else if (g_gemFocus == GEM_FOCUS_MERCHANT_CREATE_IMAGE) {
            if (c == 0x0D) { // Enter -> Open Selector
                g_gemState = GEM_STATE_IMAGE_SELECT;
                gem_buildBMPList();
                gem_drawImageSelect();
            }
            else if (c == KEY_UP_ARROW) {
                g_gemFocus = GEM_FOCUS_MERCHANT_CREATE_CONSUMABLE;
                tft.startWrite();
                gem_refreshMerchantEditorView();
                tft.endWrite();
            }
            else if (c == KEY_DOWN_ARROW) {
                g_gemFocus = GEM_FOCUS_FOOTER;
                g_gem_footer_selection = 0;
                tft.startWrite();
                gem_refreshMerchantEditorView();
                gem_drawFooter(true);
                tft.endWrite();
            }
        }
        // --- D. Image Selection Button (Quest) ---
        else if (g_gemFocus == GEM_FOCUS_CREATE_IMAGE) {
            if (c == 0x0D) { // Enter -> Open Selector
                g_gemState = GEM_STATE_IMAGE_SELECT;
                gem_buildBMPList();
                gem_drawImageSelect();
            }
            else if (c == KEY_UP_ARROW) {
                if (g_gem_editor_questType == QT_STANDARD) g_gemFocus = GEM_FOCUS_CREATE_REPEATABLE;
                else g_gemFocus = GEM_FOCUS_CREATE_ADD_CYCLE;
                gem_refreshCreateQuestView();
                g_forceCursorRedraw = true;
            }
            else if (c == KEY_DOWN_ARROW) {
                g_gemFocus = GEM_FOCUS_FOOTER;
                g_gem_footer_selection = 0;
                gem_refreshCreateQuestView();
                tft.startWrite();
                gem_drawFooter(true);
                tft.endWrite();
            }
        }
        // --- E. Standard List/Menu Navigation ---
        else {
            if (c == KEY_UP_ARROW) {
                if (g_gemFocus == GEM_FOCUS_FOOTER) {
                    if (g_gem_footer_selection == 0) {
                        if (g_gemState == GEM_STATE_QUEST_LIST) {
                            g_gemFocus = GEM_FOCUS_LIST;
                            g_gem_list_selection = g_questCount; 
                            g_gem_marquee_active = true;
                            tft.startWrite();
                            gem_drawFooter(true); 
                            gem_drawListItem(g_gem_list_selection, true);
                            tft.endWrite();
                        } else if (g_gemState == GEM_STATE_CREATE_QUEST) {
                            // --- FIX: Go to Image ---
                            g_gemFocus = GEM_FOCUS_CREATE_IMAGE;
                            gem_refreshCreateQuestView();
                            tft.startWrite(); gem_drawFooter(true); tft.endWrite();
                        } else if (g_gemState == GEM_STATE_TYPE_SELECT) { 
                            g_gemFocus = GEM_FOCUS_TYPE_SELECT_LIST;
                            g_gem_type_select_selection = g_quest_type_count - 1; 
                            gem_drawTypeSelectList();
                            tft.startWrite(); gem_drawFooter(true); tft.endWrite();
                        } else if (g_gemState == GEM_STATE_STATS) {
                            g_gemFocus = GEM_FOCUS_STATS_LIST;
                            const int visible_count = 6; 
                            int lastPossibleIndex = g_playerSkillCount - 1;
                            if (lastPossibleIndex < 0) lastPossibleIndex = 0;
                            if (g_playerSkillCount <= visible_count) g_gem_stats_top_skill = 0;
                            else g_gem_stats_top_skill = g_playerSkillCount - visible_count;
                            g_gem_stats_selection = lastPossibleIndex;
                            
                            gem_drawStatsList();
                            tft.startWrite(); gem_drawFooter(true); tft.endWrite();
                        }
                        else if (g_gemState == GEM_STATE_MERCHANT) {
                            g_gemFocus = GEM_FOCUS_MERCHANT_LIST;
                            g_merchant_list_selection = g_merchantItemCount - 1;
                            if (g_merchant_list_selection < 0) g_merchant_list_selection = 0;
                            // Check visibility and scroll if needed
                            if (gem_getMerchantItemY(g_merchant_list_selection) == -1) {
                                while(gem_getMerchantItemY(g_merchant_list_selection) == -1 && g_merchant_list_top_item < g_merchantItemCount) {
                                    g_merchant_list_top_item++;
                                }
                            }
                            
                            tft.startWrite();
                            gem_drawMerchantList();
                            gem_drawFooter(true);
                            tft.endWrite();
                        }
                        else if (g_gemState == GEM_STATE_MERCHANT_EDIT) {
                            g_gemFocus = GEM_FOCUS_MERCHANT_CREATE_IMAGE;
                            tft.startWrite();
                            gem_refreshMerchantEditorView();
                            gem_drawFooter(true);
                            tft.endWrite();
                        }
                        else if (g_gemState == GEM_STATE_INVENTORY) {
                            g_gemFocus = GEM_FOCUS_INVENTORY_LIST;
                            g_inventory_list_selection = g_inventoryItemCount - 1;
                            if (g_inventory_list_selection < 0) g_inventory_list_selection = 0;
                            
                            tft.startWrite();
                            gem_drawInventoryList();
                            gem_drawFooter(true);
                            tft.endWrite();
                        }
                        g_gem_marquee_offset = 0;
                        g_forceCursorRedraw = true; 
                    } else {
                        g_gem_footer_selection--;
                        tft.startWrite();
                        gem_drawFooter(true);
                        tft.endWrite();
                    }
                }
                else if (g_gemState == GEM_STATE_QUEST_LIST && g_gemFocus == GEM_FOCUS_LIST) {
                    int old_selection = g_gem_list_selection;
                    g_gem_list_selection--;
                    if (g_gem_list_selection < 0) g_gem_list_selection = 0;
                    
                    // --- FIX: Scroll Up Logic for Quest List ---
                    if (g_gem_list_selection < g_gem_list_top_item) {
                        g_gem_list_top_item = g_gem_list_selection;
                        tft.startWrite();
                        gem_drawQuestList();
                        tft.endWrite();
                    }
                    else if (old_selection != g_gem_list_selection) {
                        g_gem_marquee_offset = 0;
                        tft.startWrite();
                        gem_drawListItem(old_selection, false); 
                        gem_drawListItem(g_gem_list_selection, false);
                        tft.endWrite();
                    }
                }
                else if (g_gemState == GEM_STATE_STATS && g_gemFocus == GEM_FOCUS_STATS_LIST) {
                    int old_sel = g_gem_stats_selection;
                    int old_top = g_gem_stats_top_skill;
                    g_gem_stats_selection--;
                    if (g_gem_stats_selection < 0) g_gem_stats_selection = 0;
                    else if (g_gem_stats_selection < g_gem_stats_top_skill) g_gem_stats_top_skill--;
                    if (old_sel != g_gem_stats_selection) {
                        if (old_top != g_gem_stats_top_skill) {
                            gem_drawStatsList();
                        } else {
                             gem_drawStatsItem(old_sel);
                             gem_drawStatsItem(g_gem_stats_selection);
                        }
                    }
                }
                else if (g_gemState == GEM_STATE_TYPE_SELECT && g_gemFocus == GEM_FOCUS_TYPE_SELECT_LIST) {
                    int old_sel = g_gem_type_select_selection;
                    int old_top = g_gem_type_select_top_item;
                    g_gem_type_select_selection--;
                    if (g_gem_type_select_selection < 0) g_gem_type_select_selection = 0;
                    else if (g_gem_type_select_selection < g_gem_type_select_top_item) g_gem_type_select_top_item--;
                    if (old_sel != g_gem_type_select_selection) {
                        if (old_top != g_gem_type_select_top_item) {
                            gem_drawTypeSelectList();
                        } else {
                             gem_drawTypeSelectItem(old_sel);
                             gem_drawTypeSelectItem(g_gem_type_select_selection);
                        }
                    }
                }
                else if (g_gemState == GEM_STATE_MERCHANT && g_gemFocus == GEM_FOCUS_MERCHANT_LIST) {
                    int old_selection = g_merchant_list_selection;
                    g_merchant_list_selection--;
                    if (g_merchant_list_selection < 0) g_merchant_list_selection = 0;
                    
                    // Scroll Up Logic
                    if (g_merchant_list_selection < g_merchant_list_top_item) {
                        g_merchant_list_top_item = g_merchant_list_selection;
                        tft.startWrite();
                        gem_drawMerchantList(); 
                        tft.endWrite();
                    }
                    else if (old_selection != g_merchant_list_selection) {
                         tft.startWrite();
                         gem_drawMerchantItem(old_selection); 
                         gem_drawMerchantItem(g_merchant_list_selection);
                         tft.endWrite();
                    }
                }
                else if (g_gemState == GEM_STATE_INVENTORY && g_gemFocus == GEM_FOCUS_INVENTORY_LIST) {
                    int old_selection = g_inventory_list_selection;
                    g_inventory_list_selection--;
                    if (g_inventory_list_selection < 0) g_inventory_list_selection = 0;
                    
                    // Scroll Up Logic
                    if (g_inventory_list_selection < g_inventory_list_top_item) {
                        g_inventory_list_top_item = g_inventory_list_selection;
                        tft.startWrite();
                        gem_drawInventoryList();
                        tft.endWrite();
                    }
                    else if (old_selection != g_inventory_list_selection) {
                         tft.startWrite();
                         gem_drawInventoryItem(old_selection); 
                         gem_drawInventoryItem(g_inventory_list_selection);
                         tft.endWrite();
                    }
                }

            } else if (c == KEY_DOWN_ARROW) {
                if (g_gemFocus == GEM_FOCUS_FOOTER) {
                    tft.startWrite();
                    int max_selection = 1;
                     if (g_gemState == GEM_STATE_QUEST_LIST) max_selection = 2;
                     else if (g_gemState == GEM_STATE_STATS) max_selection = 2;
                     else if (g_gemState == GEM_STATE_CREATE_QUEST && g_gem_currentQuestID != -1) max_selection = 2;
                     else if (g_gemState == GEM_STATE_MERCHANT_EDIT && g_merchant_currentItemID != -1) max_selection = 2;
                     else if (g_gemState == GEM_STATE_INVENTORY_VIEW) max_selection = 2;
                     if (g_gem_footer_selection < max_selection) g_gem_footer_selection++;
                     gem_drawFooter(true);
                    tft.endWrite();
                }
                else if (g_gemState == GEM_STATE_QUEST_LIST && g_gemFocus == GEM_FOCUS_LIST) {
                    int old_selection = g_gem_list_selection;
                    g_gem_list_selection++;
                    if (g_gem_list_selection >= (g_questCount + 1)) {
                        g_gem_list_selection = g_questCount;
                        g_gemFocus = GEM_FOCUS_FOOTER;    
                        g_gem_footer_selection = 0;
                        g_gem_marquee_offset = 0;
                        tft.startWrite();
                        gem_drawListItem(old_selection, false); 
                        gem_drawFooter(true);
                        tft.endWrite();
                    } else {
                        // --- FIX: Scroll Down Logic for Quest List ---
                        if (gem_getQuestItemY(g_gem_list_selection) == -1) {
                             while (gem_getQuestItemY(g_gem_list_selection) == -1 && g_gem_list_top_item < g_gem_list_selection) {
                                   g_gem_list_top_item++;
                             }
                             tft.startWrite();
                             gem_drawQuestList();
                             tft.endWrite();
                        } else {
                            g_gem_marquee_offset = 0;
                            tft.startWrite();
                            gem_drawListItem(old_selection, false); 
                            gem_drawListItem(g_gem_list_selection, false);
                            tft.endWrite();
                        }
                    }
                }
                else if (g_gemState == GEM_STATE_STATS && g_gemFocus == GEM_FOCUS_STATS_LIST) {
                     const int visible_count = 6;
                     int lastPossibleIndex = g_playerSkillCount - 1;
                     if (lastPossibleIndex < 0) lastPossibleIndex = 0;
                     int old_sel = g_gem_stats_selection;
                     int old_top = g_gem_stats_top_skill;

                    g_gem_stats_selection++;
                    if (g_gem_stats_selection > lastPossibleIndex) {
                        g_gem_stats_selection = lastPossibleIndex;
                        g_gemFocus = GEM_FOCUS_FOOTER;
                        g_gem_footer_selection = 0;
                        tft.startWrite();
                        if (old_sel >= 0) gem_drawStatsItem(old_sel);
                        gem_drawFooter(true);
                        tft.endWrite();
                    } else {
                        int item_height_px = LINE_HEIGHT + 4 + 3 + LINE_HEIGHT + 4;
                        int startY = (LINE_HEIGHT * 3) + 6 + (LINE_HEIGHT * 2) + 5;
                        int footerY = SCREEN_HEIGHT - (LINE_HEIGHT * 2);
                        int max_visible_items = (footerY - startY) / item_height_px;
                        int lastVisibleIndex = g_gem_type_select_top_item + max_visible_items - 1;
                        if (g_gem_stats_selection > lastVisibleIndex) g_gem_stats_top_skill++;
                        if (old_sel != g_gem_stats_selection || old_top != g_gem_stats_top_skill) {
                            if (old_top != g_gem_stats_top_skill) {
                                gem_drawStatsList();
                            } else {
                                 if (old_sel >= 0) gem_drawStatsItem(old_sel);
                                 gem_drawStatsItem(g_gem_stats_selection);
                            }
                        }
                    }
                }
                else if (g_gemState == GEM_STATE_TYPE_SELECT && g_gemFocus == GEM_FOCUS_TYPE_SELECT_LIST) {
                     int lastPossibleIndex = g_quest_type_count - 1;
                    int old_sel = g_gem_type_select_selection;
                    int old_top = g_gem_type_select_top_item;
                    g_gem_type_select_selection++;
                    if (g_gem_type_select_selection > lastPossibleIndex) {
                        g_gem_type_select_selection = lastPossibleIndex;
                        g_gemFocus = GEM_FOCUS_FOOTER;
                        g_gem_footer_selection = 0;
                        gem_drawTypeSelectItem(old_sel);
                        tft.startWrite();
                        gem_drawFooter(true); 
                        tft.endWrite();
                    } else {
                        int item_height_px = LINE_HEIGHT + 4 + 6;
                        int startY = (LINE_HEIGHT * 2); 
                        int footerY = SCREEN_HEIGHT - LINE_HEIGHT;
                        int max_visible_items = (footerY - startY) / item_height_px;
                        int lastVisibleIndex = g_gem_type_select_top_item + max_visible_items - 1;
                        if (g_gem_type_select_selection > lastVisibleIndex) g_gem_type_select_top_item++;
                        if (old_sel != g_gem_type_select_selection || old_top != g_gem_type_select_top_item) {
                            if (old_top != g_gem_type_select_top_item) {
                                gem_drawTypeSelectList();
                            } else {
                                 gem_drawTypeSelectItem(old_sel);
                                 gem_drawTypeSelectItem(g_gem_type_select_selection);
                            }
                        }
                    }
                }
                else if (g_gemState == GEM_STATE_MERCHANT && g_gemFocus == GEM_FOCUS_MERCHANT_LIST) {
                      int old_selection = g_merchant_list_selection;
                     g_merchant_list_selection++;
                     if (g_merchant_list_selection >= g_merchantItemCount) {
                        // Footer
                        g_merchant_list_selection = g_merchantItemCount - 1;
                        if (g_merchant_list_selection < 0) g_merchant_list_selection = 0;
                        g_gemFocus = GEM_FOCUS_FOOTER; 
                        g_gem_footer_selection = 0; 
                        tft.startWrite();
                        if (old_selection >= 0 && old_selection < g_merchantItemCount) gem_drawMerchantItem(old_selection); 
                        gem_drawFooter(true);
                        tft.endWrite();
                    } else {
                        // Check if we need to scroll
                        if (gem_getMerchantItemY(g_merchant_list_selection) == -1) {
                             while (gem_getMerchantItemY(g_merchant_list_selection) == -1 && g_merchant_list_top_item < g_merchant_list_selection) {
                                  g_merchant_list_top_item++;
                             }
                             tft.startWrite();
                             gem_drawMerchantList(); 
                             tft.endWrite();
                        } else {
                            tft.startWrite();
                            gem_drawMerchantItem(old_selection); 
                            gem_drawMerchantItem(g_merchant_list_selection);
                            tft.endWrite();
                        }
                    }
                }
                else if (g_gemState == GEM_STATE_INVENTORY && g_gemFocus == GEM_FOCUS_INVENTORY_LIST) {
                     int old_selection = g_inventory_list_selection;
                     g_inventory_list_selection++;
                    
                    if (g_inventory_list_selection >= g_inventoryItemCount) {
                        // Footer
                        g_inventory_list_selection = g_inventoryItemCount - 1;
                        if (g_inventory_list_selection < 0) g_inventory_list_selection = 0;
                        g_gemFocus = GEM_FOCUS_FOOTER; 
                        g_gem_footer_selection = 0; 
                        tft.startWrite();
                        if (old_selection >= 0 && old_selection < g_inventoryItemCount) gem_drawInventoryItem(old_selection); 
                        gem_drawFooter(true);
                        tft.endWrite();
                    } else {
                        // Check if we need to scroll
                        if (gem_getInventoryItemY(g_inventory_list_selection) == -1) {
                             while (gem_getInventoryItemY(g_inventory_list_selection) == -1 && g_inventory_list_top_item < g_inventory_list_selection) {
                                  g_inventory_list_top_item++;
                             }
                             tft.startWrite();
                             gem_drawInventoryList(); 
                             tft.endWrite();
                        } else {
                            tft.startWrite();
                            gem_drawInventoryItem(old_selection); 
                            gem_drawInventoryItem(g_inventory_list_selection);
                            tft.endWrite();
                        }
                    }
                }
            } else if (c == KEY_LEFT_ARROW) {
                if (g_gemState == GEM_STATE_QUEST_LIST && g_gemFocus == GEM_FOCUS_LIST) {
                    if (g_gem_list_selection > 0) { 
                        int db_index = g_gem_list_selection - 1;
                        if (gem_cycle_quest_state(db_index)) {
                            g_gem_marquee_offset = 0;
                            gem_drawQuestList();
                            g_forceCursorRedraw = true;
                        } else {
                            g_gem_marquee_active = true;
                        }
                    }
                } 
                else if (g_gemFocus == GEM_FOCUS_FOOTER) {
                    tft.startWrite();
                    if (g_gem_footer_selection > 0) g_gem_footer_selection--;
                    gem_drawFooter(true);
                    tft.endWrite();
                }
            } else if (c == KEY_RIGHT_ARROW) {
                if (g_gemState == GEM_STATE_QUEST_LIST && g_gemFocus == GEM_FOCUS_LIST) {
                    if (g_gem_list_selection > 0) { 
                         int db_index = g_gem_list_selection - 1;
                         if (gem_cycle_quest_state(db_index)) {
                            g_gem_marquee_offset = 0;
                            gem_drawQuestList();
                            g_forceCursorRedraw = true;
                        } else {
                            g_gem_marquee_active = true;
                        }
                    }
                }
                else if (g_gemFocus == GEM_FOCUS_FOOTER) {
                    tft.startWrite();
                    int max_selection = 1;
                     if (g_gemState == GEM_STATE_QUEST_LIST) max_selection = 2;
                     else if (g_gemState == GEM_STATE_STATS) max_selection = 2;
                     else if (g_gemState == GEM_STATE_CREATE_QUEST && g_gem_currentQuestID != -1) max_selection = 2;
                     else if (g_gemState == GEM_STATE_MERCHANT_EDIT && g_merchant_currentItemID != -1) max_selection = 2;
                     else if (g_gemState == GEM_STATE_INVENTORY_VIEW) max_selection = 2;
                     if (g_gem_footer_selection < max_selection) g_gem_footer_selection++;
                     gem_drawFooter(true);
                    tft.endWrite();
                }
            } else if (c == 0x0D) { // Enter
                if (g_gemState == GEM_STATE_QUEST_LIST && g_gemFocus == GEM_FOCUS_LIST) {
                    if (g_gem_list_selection == 0) {
                        // "New Quest" selected
                        g_gem_currentQuestID = -1;
                        g_gemState = GEM_STATE_TYPE_SELECT; 
                        g_gemFocus = GEM_FOCUS_TYPE_SELECT_LIST;
                        g_gem_type_select_selection = 0;
                        gem_drawTypeSelectPage();
                        g_forceCursorRedraw = true;
                    } else {
                        // Existing quest selected
                        g_gem_currentQuestID = g_gem_list_selection - 1;
                        if (g_gem_currentQuestID != -1) g_gem_editor_questType = g_questDB[g_gem_currentQuestID].type;
                        else g_gem_editor_questType = QT_STANDARD;
                        g_gemState = GEM_STATE_CREATE_QUEST;
                        g_gemFocus = GEM_FOCUS_CREATE_TITLE;
                        gem_drawQuestEditor();
                        g_forceCursorRedraw = true;
                    }
                    return;
                } 
                else if (g_gemState == GEM_STATE_MERCHANT && g_gemFocus == GEM_FOCUS_MERCHANT_LIST) {
                    if (g_merchantItemCount > 0 && g_merchant_list_selection < g_merchantItemCount) {
                        MerchantItem& item = g_merchantDB[g_merchant_list_selection];
                        if (g_gem_count >= item.price) {
                            g_gem_count -= item.price;
                            addInventoryItem(item.title, item.description, item.rarity, item.quality, item.need, item.desire, item.isConsumable);
                            g_inventoryDB[g_inventoryItemCount-1].iconFile = item.iconFile; 
                            gem_deleteMerchantItem(g_merchant_list_selection);
                            // This saves
                            pushSystemMessage("Purchased " + item.title + "!");
                            if (g_merchant_list_selection >= g_merchantItemCount) g_merchant_list_selection = g_merchantItemCount - 1;
                            if (g_merchant_list_selection < 0) g_merchant_list_selection = 0;
                            tft.startWrite();
                            gem_drawHeader(true);
                            gem_drawMerchantList();
                            gem_drawFooter(true);
                            tft.endWrite();
                            g_forceCursorRedraw = true;
                        } else {
                            pushSystemMessage("Not enough gems!");
                        }
                        return;
                    }
                }
                else if (g_gemState == GEM_STATE_INVENTORY && g_gemFocus == GEM_FOCUS_INVENTORY_LIST) {
                    if (g_inventoryItemCount > 0 && g_inventory_list_selection < g_inventoryItemCount) {
                        g_inventory_currentItemID = g_inventory_list_selection;
                        g_gemState = GEM_STATE_INVENTORY_VIEW;
                        g_gemFocus = GEM_FOCUS_FOOTER;
                        gem_drawInventoryView();
                        g_forceCursorRedraw = true;
                        return;
                    }
                }
                else if (g_gemFocus == GEM_FOCUS_FOOTER) {
                    // Trigger footer buttons
                    String action;
                    if (g_gemState == GEM_STATE_QUEST_LIST) {
                        if (g_gem_footer_selection == 0) action = "Stats";
                        else if (g_gem_footer_selection == 1) action = "Merchant";
                        else if (g_gem_footer_selection == 2) action = "Exit";
                    }
                    else if (g_gemState == GEM_STATE_STATS) {
                        if (g_gem_footer_selection == 0) action = "Inventory";
                        else if (g_gem_footer_selection == 1) action = "Settings";
                        else if (g_gem_footer_selection == 2) action = "Exit";
                    }
                    else if (g_gemState == GEM_STATE_TYPE_SELECT) action = (g_gem_footer_selection == 0) ? "NewType" : "Back";
                    else if (g_gemState == GEM_STATE_MERCHANT) {
                        action = (g_gem_footer_selection == 0) ? "NewItem" : "Exit";
                    } 
                    else if (g_gemState == GEM_STATE_MERCHANT_EDIT) {
                        if (g_gem_footer_selection == 0) action = "SaveItem";
                        else if (g_gem_footer_selection == 1) action = "Back";
                        else if (g_gem_footer_selection == 2) action = "DeleteItem";
                    }
                    else if (g_gemState == GEM_STATE_INVENTORY) {
                         action = (g_gem_footer_selection == 0) ? "Back" : "Exit";
                    }
                    else if (g_gemState == GEM_STATE_INVENTORY_VIEW) {
                        if (g_gem_footer_selection == 0) action = "Sell";
                        else if (g_gem_footer_selection == 1) action = "Back";
                        else if (g_gem_footer_selection == 2) action = "Delete";
                    }
                    else { // CREATE_QUEST
                        if (g_gem_footer_selection == 0) action = "Save";
                        else if (g_gem_footer_selection == 1) action = "Back";
                        else if (g_gem_footer_selection == 2) action = "Delete";
                    }

                    if (action == "Stats") {
                        g_gemFocus = GEM_FOCUS_FOOTER;
                        g_gem_footer_selection = 0;
                        gem_drawStatisticsPage();
                        g_forceCursorRedraw = true;
                        return;
                    }
                    else if (action == "Inventory") {
                        g_gemState = GEM_STATE_INVENTORY;
                        g_gemFocus = GEM_FOCUS_INVENTORY_LIST;
                        g_inventory_list_selection = 0;
                        g_inventory_list_top_item = 0;
                        tft.startWrite();
                        gem_drawHeader(true);
                        gem_drawInventoryList();
                        gem_drawFooter(true);
                        tft.endWrite();
                        g_forceCursorRedraw = true;
                        return;
                    }
                    else if (action == "Merchant") {
                         g_gemState = GEM_STATE_MERCHANT;
                         g_gemFocus = GEM_FOCUS_MERCHANT_LIST;
                        g_merchant_list_selection = 0;
                        g_merchant_list_top_item = 0;
                        tft.startWrite();
                        gem_drawHeader(true);
                        gem_drawMerchantList();
                        gem_drawFooter(true);
                        tft.endWrite();
                        g_forceCursorRedraw = true;
                        return;
                    }
                    else if (action == "Settings") {
                         pushSystemMessage("Settings page not yet implemented.");
                         gem_drawStatisticsPage(); 
                    } else if (action == "NewType") { 
                        pushSystemMessage("New quest type editor not yet implemented.");
                        gem_drawTypeSelectPage(); 
                    } else if (action == "Exit") {
                        if (g_gemState == GEM_STATE_STATS || g_gemState == GEM_STATE_MERCHANT || g_gemState == GEM_STATE_INVENTORY) {
                             gem_start();
                             return;
                         } else {
                            gem_stop();
                            return;
                        }
                    } 
                    else if (action == "NewItem") {
                         g_merchant_currentItemID = -1;
                         g_gemState = GEM_STATE_MERCHANT_EDIT;
                        g_gemFocus = GEM_FOCUS_MERCHANT_CREATE_TITLE;
                        gem_drawMerchantEditor();
                        g_forceCursorRedraw = true;
                        return;
                    } 
                    else if (action == "SaveItem") {
                        if (g_gemState == GEM_STATE_MERCHANT_EDIT) {
                             String title = gemCreateItemTitleEditor->lines[0];
                             String desc = gem_join_notes_from_editor(gemCreateItemDescEditor);
                            int r = gemCreateItemRarityEditor->lines[0].toInt();
                            int q = gemCreateItemQualityEditor->lines[0].toInt();
                            int n = gemCreateItemNeedEditor->lines[0].toInt();
                            int d = gemCreateItemDesireEditor->lines[0].toInt();
                            bool isConsumable = g_gem_editor_isConsumable;
                            if (title.length() == 0) { pushSystemMessage("Title cannot be empty.");
                            }
                            else if (r < 1 || r > 7) { pushSystemMessage("Rarity must be 1-7.");
                            }
                            else if (q < 1 || q > 7) { pushSystemMessage("Quality must be 1-7.");
                            }
                            else if (n < 1 || n > 7) { pushSystemMessage("Need must be 1-7.");
                            }
                            else if (d < 1 || d > 7) { pushSystemMessage("Desire must be 1-7.");
                            }
                            else {
                                if (g_merchant_currentItemID == -1) {
                                    addMerchantItem(title, desc, r, q, n, d, isConsumable);
                                    // Icon is set in image selector logic
                                    // But we must ensure it's saved if we used the selector:
                                    g_merchantDB[g_merchantItemCount - 1].iconFile = g_gem_temp_icon_file;
                                    pushSystemMessage("New item created!");
                                } else {
                                    MerchantItem& item = g_merchantDB[g_merchant_currentItemID];
                                    item.title = title;
                                    item.description = desc;
                                    item.rarity = r;
                                    item.quality = q;
                                    item.need = n;
                                    item.desire = d;
                                    item.isConsumable = isConsumable;
                                    item.price = gem_calculate_item_price(r, q, n, d, isConsumable);
                                    item.iconFile = g_gem_temp_icon_file;
                                    pushSystemMessage("Item updated!");
                                }
                                gem_save_database(g_gem_db_filename);
                                g_gemState = GEM_STATE_MERCHANT;
                                g_gemFocus = GEM_FOCUS_MERCHANT_LIST;
                                tft.startWrite(); 
                                gem_drawHeader(true);
                                gem_drawMerchantList(); 
                                gem_drawFooter(true);
                                tft.endWrite();
                                g_forceCursorRedraw = true;
                                return;
                            }
                            tft.startWrite();
                            gem_drawMerchantEditor();
                            tft.endWrite();
                        }
                    }
                    else if (action == "DeleteItem") {
                        if (g_gemState == GEM_STATE_MERCHANT_EDIT && g_merchant_currentItemID != -1) {
                             gem_deleteMerchantItem(g_merchant_currentItemID);
                            g_gemState = GEM_STATE_MERCHANT;
                            g_gemFocus = GEM_FOCUS_MERCHANT_LIST;
                            tft.startWrite(); 
                            gem_drawHeader(true);
                            gem_drawMerchantList(); 
                            gem_drawFooter(true);
                            tft.endWrite();
                            g_forceCursorRedraw = true;
                            return;
                        }
                        else if (g_gemState == GEM_STATE_INVENTORY_VIEW && g_inventory_currentItemID != -1) {
                            // --- FIX: Robust Inventory Delete Logic ---
                            tft.endWrite(); // End transaction before logic
                            gem_deleteInventoryItem(g_inventory_currentItemID);
                            g_inventory_currentItemID = -1;
                            if (g_inventoryItemCount == 0) {
                                g_inventory_list_selection = 0;
                                g_inventory_list_top_item = 0;
                            } else {
                                if (g_inventory_list_selection >= g_inventoryItemCount) g_inventory_list_selection = g_inventoryItemCount - 1;
                                if (g_inventory_list_top_item > g_inventory_list_selection) g_inventory_list_top_item = g_inventory_list_selection;
                            }

                            g_gemState = GEM_STATE_INVENTORY;
                            g_gemFocus = GEM_FOCUS_INVENTORY_LIST;
                            tft.startWrite();
                            gem_drawHeader(true);
                            gem_drawInventoryList();
                            gem_drawFooter(true);
                            tft.endWrite();
                            g_forceCursorRedraw = true;
                            return;
                        }
                    }
                    else if (action == "Sell") {
                        if (g_gemState == GEM_STATE_INVENTORY_VIEW && g_inventory_currentItemID != -1) {
                             MerchantItem& item = g_inventoryDB[g_inventory_currentItemID];
                            int gemsGained = max(1, (int)(item.price * 0.75));
                            g_gem_count += gemsGained;
                            addMerchantItem(item.title, item.description, item.rarity, item.quality, item.need, item.desire, item.isConsumable);
                            g_merchantDB[g_merchantItemCount-1].iconFile = item.iconFile; 
                            
                            gem_deleteInventoryItem(g_inventory_currentItemID);
                            
                            // --- FIX: Robust Pointer Logic after Sell ---
                            g_inventory_currentItemID = -1;
                            if (g_inventoryItemCount == 0) {
                                g_inventory_list_selection = 0;
                                g_inventory_list_top_item = 0;
                            } else {
                                if (g_inventory_list_selection >= g_inventoryItemCount) g_inventory_list_selection = g_inventoryItemCount - 1;
                                if (g_inventory_list_top_item > g_inventory_list_selection) g_inventory_list_top_item = g_inventory_list_selection;
                            }

                            pushSystemMessage("Sold " + item.title + " for " + String(gemsGained) + " G.");
                            g_gemState = GEM_STATE_INVENTORY;
                            g_gemFocus = GEM_FOCUS_INVENTORY_LIST;
                            tft.startWrite();
                            gem_drawHeader(true);
                            gem_drawInventoryList();
                            gem_drawFooter(true);
                            tft.endWrite();
                            g_forceCursorRedraw = true;
                            return;
                        }
                    }
                    else if (action == "Save") {
                         String title = gemCreateTitleEditor->lines[0];
                         String desc = gemCreateDescEditor->lines[0]; 
                        String notes = gem_join_notes_from_editor(gemCreateDescEditor);
                        String skills = gem_join_notes_from_editor(gemCreateSkillsEditor);
                        skills.replace("\n", "");
                        int diff = gemCreateDiffEditor->lines[0].toInt();
                        int desire = gemCreateDesireEditor->lines[0].toInt();
                        
                        int timeEstHours = gemCreateTimeEditor->lines[0].toInt();
                        int timeEstMins_field = gemCreateMinsEditor->lines[0].toInt();
                        int totalTimeMins = (timeEstHours * 60) + timeEstMins_field;
                        int addCycleMins = 0;
                        bool isRepeatable;
                         if (g_gem_editor_questType == QT_CRUSADE) {
                            isRepeatable = true;
                            addCycleMins = gemCreateAddCycleEditor->lines[0].toInt();
                        } else {
                            isRepeatable = g_gem_editor_isRepeatable;
                        }
                        
                        if (title.length() == 0) { pushSystemMessage("Title cannot be empty.");
                        }
                        else if (diff < 0 || diff > 7) { pushSystemMessage("Difficulty must be 0-7.");
                        } // Allow 0
                        else if (desire < 0 || desire > 7) { pushSystemMessage("Desire must be 0-7.");
                        } // Allow 0
                        else if (timeEstHours < 0 || timeEstHours > 100) { pushSystemMessage("Hours must be 0-100.");
                        }
                        else if (timeEstMins_field < 0 || timeEstMins_field > 100) { pushSystemMessage("Mins must be 0-100.");
                        }
                        else if (totalTimeMins == 0) { pushSystemMessage("Total time cannot be zero.");
                        } // Can't be 0h 0m
                        else if (g_gem_editor_questType == QT_CRUSADE && (addCycleMins < 0 || addCycleMins > 100)) { // Allow 0
                            pushSystemMessage("Add Cycle must be 0-100.");
                        }
                        else {
                            gem_add_skills_to_database(skills);
                            if (g_gem_currentQuestID == -1) {
                                // --- FIX: Pass g_gem_temp_quest_icon to addQuest ---
                                addQuest(title, desc, notes, skills, isRepeatable, diff, desire, totalTimeMins, g_gem_editor_questType, addCycleMins, g_gem_temp_quest_icon);
                                pushSystemMessage("New quest created!");
                            } else {
                                Quest& quest = g_questDB[g_gem_currentQuestID];
                                quest.title = title;
                                quest.description = desc;
                                quest.notes = notes; 
                                quest.skills = skills;
                                quest.isRepeatable = isRepeatable;
                                quest.difficulty = diff;
                                quest.desire = desire;
                                quest.timerDurationMinutes = (unsigned long)totalTimeMins; // Save TOTAL MINS
                                quest.type = g_gem_editor_questType;
                                quest.addCycleMinutes = addCycleMins;
                                quest.iconFile = g_gem_temp_quest_icon; // --- FIX: Update Icon ---
                                
                                float timeHours = (float)totalTimeMins / 60.0f;
                                float base_reward = (diff * 10) + (timeHours * 2.0f);
                                float desire_multiplier = 1.0 + ( (7.0 - (float)desire) / 6.0 );
                                quest.gemReward = (int)(base_reward * desire_multiplier);
                                quest.xpReward = (quest.difficulty * 5) + (int)(desire_multiplier * 5) + (int)(timeHours * 2.0f);
                                
                                pushSystemMessage("Quest updated!");
                            }
                            
                            gem_save_database(g_gem_db_filename);
                            gem_start(); 
                            return;
                        }
                        gem_refreshCreateQuestView();
                        gem_drawFooter(true);

                    } else if (action == "Back") {
                        gem_start();
                        return;
                    } else if (action == "Delete") {
                        gem_deleteQuest(g_gem_currentQuestID);
                        gem_start();
                        return;
                    }
                }
                else if (g_gemState == GEM_STATE_STATS && g_gemFocus == GEM_FOCUS_STATS_LIST) {
                   // Do nothing on Enter in Stats List
                }
                else if (g_gemState == GEM_STATE_TYPE_SELECT && g_gemFocus == GEM_FOCUS_TYPE_SELECT_LIST) {
                    // Activate Type selection
                     if (g_gem_type_select_selection == 0) g_gem_editor_questType = QT_STANDARD;
                     else g_gem_editor_questType = QT_CRUSADE;
                     
                     g_gemState = GEM_STATE_CREATE_QUEST;
                     g_gemFocus = GEM_FOCUS_CREATE_TITLE;
                     gem_drawQuestEditor();
                     g_forceCursorRedraw = true;
                     return;
                }
            }
        }
    }
}
void gem_handleInput(int buttonIndex) {
    
    tft.startWrite();
    bool needsFullRedraw = false; 
    bool needsFooterRedraw = false;
    TextArea* focusedEditor = nullptr;
    
    // Map focus to editor pointers
    if (g_gemFocus == GEM_FOCUS_DETAIL_EDITOR) focusedEditor = gemQuestNotesEditor;
    else if (g_gemFocus == GEM_FOCUS_CREATE_TITLE) focusedEditor = gemCreateTitleEditor;
    else if (g_gemFocus == GEM_FOCUS_CREATE_DESC) focusedEditor = gemCreateDescEditor;
    else if (g_gemFocus == GEM_FOCUS_CREATE_SKILLS) focusedEditor = gemCreateSkillsEditor;
    else if (g_gemFocus == GEM_FOCUS_CREATE_DIFF) focusedEditor = gemCreateDiffEditor;
    else if (g_gemFocus == GEM_FOCUS_CREATE_DESIRE) focusedEditor = gemCreateDesireEditor;
    else if (g_gemFocus == GEM_FOCUS_CREATE_TIME) focusedEditor = gemCreateTimeEditor;
    else if (g_gemFocus == GEM_FOCUS_CREATE_MINS) focusedEditor = gemCreateMinsEditor;
    else if (g_gemFocus == GEM_FOCUS_CREATE_ADD_CYCLE) focusedEditor = gemCreateAddCycleEditor;
    // Merchant fields
    else if (g_gemFocus == GEM_FOCUS_MERCHANT_CREATE_TITLE) focusedEditor = gemCreateItemTitleEditor;
    else if (g_gemFocus == GEM_FOCUS_MERCHANT_CREATE_DESC) focusedEditor = gemCreateItemDescEditor;
    else if (g_gemFocus == GEM_FOCUS_MERCHANT_CREATE_RARITY) focusedEditor = gemCreateItemRarityEditor;
    else if (g_gemFocus == GEM_FOCUS_MERCHANT_CREATE_QUALITY) focusedEditor = gemCreateItemQualityEditor;
    else if (g_gemFocus == GEM_FOCUS_MERCHANT_CREATE_NEED) focusedEditor = gemCreateItemNeedEditor;
    else if (g_gemFocus == GEM_FOCUS_MERCHANT_CREATE_DESIRE) focusedEditor = gemCreateItemDesireEditor;
    
    g_gem_marquee_active = false; // Reset marquee state

    // --- HIDE AUTOCOMPLETE if not in skills field ---
    if (g_gemFocus != GEM_FOCUS_CREATE_SKILLS && g_gem_autocomplete_active) {
        gem_hide_autocomplete(true);
        needsFullRedraw = true;
    }
    
    switch (g_gemFocus) {

        case GEM_FOCUS_HEADER:
            if (buttonIndex == IDX_PREV) {} 
            else if (buttonIndex == IDX_NEXT) { 
                if (g_gemState == GEM_STATE_QUEST_LIST) g_gemFocus = GEM_FOCUS_LIST;
                else if (g_gemState == GEM_STATE_TYPE_SELECT) g_gemFocus = GEM_FOCUS_TYPE_SELECT_LIST; 
                else if (g_gemState == GEM_STATE_CREATE_QUEST) g_gemFocus = GEM_FOCUS_CREATE_TITLE;
                else if (g_gemState == GEM_STATE_STATS) {
                    g_gemFocus = GEM_FOCUS_STATS_LIST;
                    g_gem_stats_selection = 0; // Select first skill
                    g_gem_stats_top_skill = 0;
                }
                gem_drawHeader(false);
                
                needsFullRedraw = true;
                g_forceCursorRedraw = true; 
            }
            break;

        case GEM_FOCUS_LIST:
            if (buttonIndex == IDX_PREV) { // Button 0: UP
                int old_selection = g_gem_list_selection;
                g_gem_list_selection--;
                if (g_gem_list_selection < 0) {
                    g_gem_list_selection = 0;
                }

                // --- SCROLL UP LOGIC ---
                if (g_gem_list_selection < g_gem_list_top_item) {
                    g_gem_list_top_item = g_gem_list_selection;
                    needsFullRedraw = true; 
                }
                else if (old_selection != g_gem_list_selection) {
                    g_gem_marquee_offset = 0;
                    // --- FIX: Pass 'true' to keep bars visible ---
                    gem_drawListItem(old_selection, true); 
                    gem_drawListItem(g_gem_list_selection, true);
                }
            }
            else if (buttonIndex == IDX_NEXT) { // Button 1: DOWN
                int old_selection = g_gem_list_selection;
                g_gem_list_selection++;
                
                if (g_gem_list_selection >= (g_questCount + 1)) {
                    g_gem_list_selection = g_questCount;
                    g_gemFocus = GEM_FOCUS_FOOTER; 
                    g_gem_footer_selection = 0; 
                    g_gem_marquee_offset = 0;
                    // --- FIX: Pass 'true' here too ---
                    gem_drawListItem(old_selection, true); 
                    needsFooterRedraw = true;
                } else {
                    // --- SCROLL DOWN LOGIC ---
                    if (gem_getQuestItemY(g_gem_list_selection) == -1) {
                         while (gem_getQuestItemY(g_gem_list_selection) == -1 && g_gem_list_top_item < g_gem_list_selection) {
                                g_gem_list_top_item++;
                         }
                         needsFullRedraw = true;
                    } else {
                        g_gem_marquee_offset = 0;
                        // --- FIX: Pass 'true' to keep bars visible ---
                        gem_drawListItem(old_selection, true); 
                        gem_drawListItem(g_gem_list_selection, true); 
                    }
                }
            }
            else if (buttonIndex == IDX_BACK) { // Button 3: BACK (Edit)
                tft.endWrite();
                if (g_gem_list_selection == 0) {
                    g_gem_currentQuestID = -1;
                } else {
                    g_gem_currentQuestID = g_gem_list_selection - 1;
                }
                
                if (g_gem_currentQuestID != -1) {
                    g_gem_editor_questType = g_questDB[g_gem_currentQuestID].type;
                } else {
                    g_gem_editor_questType = QT_STANDARD;
                }
                
                g_gemState = GEM_STATE_CREATE_QUEST;
                g_gemFocus = GEM_FOCUS_CREATE_TITLE;
                gem_drawQuestEditor();
                g_forceCursorRedraw = true;
                return; 
            }
            else if (buttonIndex == IDX_SELECT) { // Button 2: CONFIRM
                if (g_gem_list_selection == 0) {
                    // --- NEW QUEST CLICKED: Go to Type Select ---
                    tft.endWrite();
                    g_gem_currentQuestID = -1; 
                    g_gemState = GEM_STATE_TYPE_SELECT; 
                    g_gemFocus = GEM_FOCUS_TYPE_SELECT_LIST;
                    g_gem_type_select_selection = 0;
                    gem_drawTypeSelectPage();
                    g_forceCursorRedraw = true;
                    return; 
                } else {
                    int db_index = g_gem_list_selection - 1;
                    if (gem_cycle_quest_state(db_index)) {
                        g_gem_marquee_offset = 0;
                        needsFullRedraw = true; 
                        g_forceCursorRedraw = true;
                    } else {
                        g_gem_marquee_active = true;
                    }
                }
            }
            break;
            
        case GEM_FOCUS_TYPE_SELECT_LIST:
            if (buttonIndex == IDX_PREV) { // UP
                g_gem_type_select_selection--;
                if (g_gem_type_select_selection < 0) {
                    g_gem_type_select_selection = 0;
                    g_gemFocus = GEM_FOCUS_HEADER; // Go to header
                    needsFullRedraw = true;
                }
                needsFullRedraw = true;
            } else if (buttonIndex == IDX_NEXT) { // DOWN
                g_gem_type_select_selection++;
                if (g_gem_type_select_selection > 1) { // Only 2 options (0, 1)
                    g_gem_type_select_selection = 1;
                    g_gemFocus = GEM_FOCUS_FOOTER;
                    g_gem_footer_selection = 0;
                    needsFooterRedraw = true;
                } else {
                    needsFullRedraw = true;
                }
            } else if (buttonIndex == IDX_SELECT) { // CONFIRM
                 if (g_gem_type_select_selection == 0) g_gem_editor_questType = QT_STANDARD;
                 else g_gem_editor_questType = QT_CRUSADE;
                 
                 tft.endWrite();
                 g_gemState = GEM_STATE_CREATE_QUEST;
                 g_gemFocus = GEM_FOCUS_CREATE_TITLE;
                 gem_drawQuestEditor();
                 g_forceCursorRedraw = true;
                 return;
            } else if (buttonIndex == IDX_BACK) { // BACK
                 tft.endWrite();
                 gem_start(); // Back to list
                 return;
            }
            break;
            
        case GEM_FOCUS_STATS_LIST:
            {
                const int visible_count = 6; 
                int lastPossibleIndex = g_playerSkillCount - 1;
                
                if (buttonIndex == IDX_PREV) { // Button 0: UP
                    g_gem_stats_selection--;
                    
                    if (g_gem_stats_selection < 0) {
                        g_gem_stats_selection = 0; // Stop at top
                    } else if (g_gem_stats_selection < g_gem_stats_top_skill) {
                        g_gem_stats_top_skill--; // Scroll up
                    }
                    needsFullRedraw = true;
                }
                else if (buttonIndex == IDX_NEXT) { // Button 1: DOWN
                    g_gem_stats_selection++;
                    
                    if (g_gem_stats_selection > lastPossibleIndex) {
                        // At the very last item, move to footer
                        g_gem_stats_selection = lastPossibleIndex;
                        g_gemFocus = GEM_FOCUS_FOOTER;
                        g_gem_footer_selection = 0;
                    } else {
                        // Check if we need to scroll down
                        int lastVisibleIndex = g_gem_stats_top_skill + visible_count - 1;
                        if (g_gem_stats_selection > lastVisibleIndex) {
                            g_gem_stats_top_skill++;
                        }
                    }
                    needsFullRedraw = true;
                }
                else if (buttonIndex == IDX_BACK) { // Button 3: BACK (to quest list)
                    tft.endWrite();
                    gem_start(); // Go back to quest list
                    return;
                }
                else if (buttonIndex == IDX_SELECT) { // Button 2: CONFIRM
                    // Do nothing
                }
            }
            break;
        case GEM_FOCUS_DETAIL_EDITOR:
             break;
             
        // --- ALL TEXT EDITOR FIELDS ---
        case GEM_FOCUS_CREATE_TITLE:
        case GEM_FOCUS_CREATE_DESC:
        case GEM_FOCUS_CREATE_SKILLS: 
        case GEM_FOCUS_CREATE_DIFF:
        case GEM_FOCUS_CREATE_DESIRE:
        case GEM_FOCUS_CREATE_TIME:
        case GEM_FOCUS_CREATE_MINS: 
        case GEM_FOCUS_CREATE_ADD_CYCLE:
        // Merchant Fields
        case GEM_FOCUS_MERCHANT_CREATE_TITLE:
        case GEM_FOCUS_MERCHANT_CREATE_DESC:
        case GEM_FOCUS_MERCHANT_CREATE_RARITY:
        case GEM_FOCUS_MERCHANT_CREATE_QUALITY:
        case GEM_FOCUS_MERCHANT_CREATE_NEED:
        case GEM_FOCUS_MERCHANT_CREATE_DESIRE:
            if (!focusedEditor) break;
            
            if (buttonIndex == IDX_PREV) { // Cycle virtual key (left)
                if (kmode == CTRL) {
                    kbIndex--;
                    if (kbIndex < 0) { 
                        kbIndex = CTRL_COUNT;
                    }
                } else {
                    kbPrev();
                }
                g_forceCursorRedraw = true;
            }
            else if (buttonIndex == IDX_NEXT) { // Cycle virtual key (right)
                if (kmode == CTRL) {
                    int maxIndex = CTRL_COUNT;
                    kbIndex++;
                    if (kbIndex > maxIndex) {
                        kbIndex = 0;
                    }
                } else {
                    kbNext();
                }
                g_forceCursorRedraw = true;
            }
            else if (buttonIndex == IDX_BACK) { // BACKSPACE
                if (g_gem_autocomplete_active) {
                    gem_hide_autocomplete(false);
                }
                if (focusedEditor->backspace(tft)) {
                    needsFullRedraw = true;
                }
                g_forceCursorRedraw = true;
                if (g_gemFocus == GEM_FOCUS_CREATE_SKILLS) {
                    gem_update_autocomplete(focusedEditor);
                }
            }
            else if (buttonIndex == IDX_SELECT) { // CONFIRM KEY
                char charToInsert = 0;
                String controlAction = "";
                
                if (g_gem_autocomplete_active && kbIndex != 0) {
                     String controlActionCheck = "";
                     if (kmode == CTRL && kbIndex > 0) controlActionCheck = ctrlKeys[kbIndex - 1];
                     if (controlActionCheck == "ENTER") {
                         gem_select_autocomplete(focusedEditor);
                         needsFullRedraw = true;
                         break; 
                     }
                }

                const int ALPHA_CASE_KEY_INDEX = (int)strlen(alphaChars) + 1;
                const int NUM_KEY_INDEX_SPACE = (int)strlen(numberChars) + 1;
                const int SYM_KEY_INDEX_SPACE = (int)strlen(symbolChars) + 1;
                if (kbIndex == 0) {
                    if (kmode == ALPHA || kmode == ALPHA_LOWER) kmode = NUM;
                    else if (kmode == NUM) kmode = SYM;
                    else if (kmode == SYM) kmode = CTRL;
                    else if (kmode == CTRL) kmode = ALPHA;
                    kbIndex = 0;
                    needsFooterRedraw = true;
                    g_forceCursorRedraw = true;
                }
                else if (kmode == ALPHA) {
                    if (kbIndex <= (int)strlen(alphaChars)) charToInsert = alphaChars[kbIndex-1];
                    else if (kbIndex == ALPHA_CASE_KEY_INDEX) controlAction = "SPACE";
                    else if (kbIndex == ALPHA_CASE_KEY_INDEX + 1) controlAction = "ENTER";
                    else if (kbIndex == ALPHA_CASE_KEY_INDEX + 2) { kmode = ALPHA_LOWER; kbIndex = 0; }
                }
                else if (kmode == ALPHA_LOWER) {
                     if (kbIndex <= (int)strlen(alphaLowerChars)) charToInsert = alphaLowerChars[kbIndex-1];
                     else if (kbIndex == ALPHA_CASE_KEY_INDEX) controlAction = "SPACE";
                     else if (kbIndex == ALPHA_CASE_KEY_INDEX + 1) controlAction = "ENTER";
                     else if (kbIndex == ALPHA_CASE_KEY_INDEX + 2) { kmode = ALPHA; kbIndex = 0; }
                }
                else if (kmode == NUM) {
                     if (kbIndex <= (int)strlen(numberChars)) charToInsert = numberChars[kbIndex-1];
                     else if (kbIndex == NUM_KEY_INDEX_SPACE) controlAction = "SPACE";
                     else if (kbIndex == NUM_KEY_INDEX_SPACE + 1) controlAction = "ENTER";
                }
                else if (kmode == SYM) {
                     if (kbIndex <= (int)strlen(symbolChars)) charToInsert = symbolChars[kbIndex-1];
                     else if (kbIndex == SYM_KEY_INDEX_SPACE) controlAction = "SPACE";
                     else if (kbIndex == SYM_KEY_INDEX_SPACE + 1) controlAction = "ENTER";
                }
                else if (kmode == CTRL) {
                    if (kbIndex > 0 && kbIndex <= CTRL_COUNT) {
                        controlAction = ctrlKeys[kbIndex - 1];
                    }
                } 

                if (charToInsert != 0) {
                   if (g_gemFocus == GEM_FOCUS_CREATE_DIFF || g_gemFocus == GEM_FOCUS_CREATE_DESIRE) {
                       if (charToInsert >= '0' && charToInsert <= '7' && focusedEditor->lines[0].length() < 1) { 
                           // Allow 0
                           if (focusedEditor->insertChar(tft, charToInsert)) needsFullRedraw = true;
                       }
                   } else if (g_gemFocus == GEM_FOCUS_CREATE_TIME) { 
                       if (isDigit(charToInsert)) {
                           String newNum = focusedEditor->lines[0] + charToInsert;
                           if (newNum.length() <= 3 && newNum.toInt() <= 100) { // 0-100 hours
                               if (focusedEditor->insertChar(tft, charToInsert)) needsFullRedraw = true;
                           }
                       }
                   } else if (g_gemFocus == GEM_FOCUS_CREATE_MINS) { 
                       if (isDigit(charToInsert)) {
                           String newNum = focusedEditor->lines[0] + charToInsert;
                           if (newNum.length() <= 3 && newNum.toInt() <= 100) { // 0-100 minutes
                               if (focusedEditor->insertChar(tft, charToInsert)) needsFullRedraw = true;
                           }
                       }
                   } else if (g_gemFocus == GEM_FOCUS_CREATE_ADD_CYCLE) { 
                       if (isDigit(charToInsert)) {
                           String newNum = focusedEditor->lines[0] + charToInsert;
                           if (newNum.length() <= 3 && newNum.toInt() <= 100) { // 0-100 minutes
                               if (focusedEditor->insertChar(tft, charToInsert)) needsFullRedraw = true;
                           }
                       }
                   } else {
                       if (focusedEditor->insertChar(tft, charToInsert)) needsFullRedraw = true;
                   }
                   g_forceCursorRedraw = true;
                   if (g_gemFocus == GEM_FOCUS_CREATE_SKILLS) {
                       gem_update_autocomplete(focusedEditor);
                   }
                } 
                else if (controlAction.length() > 0) {
                    if (controlAction == "SPACE") {
                        if (g_gemFocus != GEM_FOCUS_CREATE_DIFF && 
                            g_gemFocus != GEM_FOCUS_CREATE_DESIRE && 
                            g_gemFocus != GEM_FOCUS_CREATE_TIME &&
                            g_gemFocus != GEM_FOCUS_CREATE_MINS && 
                            g_gemFocus != GEM_FOCUS_CREATE_ADD_CYCLE) { 
                            if (focusedEditor->insertChar(tft, ' ')) needsFullRedraw = true;
                        }
                        g_forceCursorRedraw = true;
                        if (g_gemFocus == GEM_FOCUS_CREATE_SKILLS) {
                            gem_hide_autocomplete(true);
                        }
                    }
                    else if (controlAction == "ENTER") {
                        if (g_gemFocus == GEM_FOCUS_CREATE_DESC || g_gemFocus == GEM_FOCUS_CREATE_SKILLS) {
                           if (focusedEditor->insertLine(tft)) needsFullRedraw = true;
                            gem_hide_autocomplete(true);
                        } 
                        else {
                             if (g_gemFocus == GEM_FOCUS_CREATE_TITLE) { g_gemFocus = GEM_FOCUS_CREATE_DESC; }
                            else if (g_gemFocus == GEM_FOCUS_CREATE_DIFF) { g_gemFocus = GEM_FOCUS_CREATE_DESIRE; }
                            else if (g_gemFocus == GEM_FOCUS_CREATE_DESIRE) { g_gemFocus = GEM_FOCUS_CREATE_TIME; } 
                            else if (g_gemFocus == GEM_FOCUS_CREATE_TIME) { g_gemFocus = GEM_FOCUS_CREATE_MINS; } 
                            else if (g_gemFocus == GEM_FOCUS_CREATE_MINS) { 
                                if (g_gem_editor_questType == QT_STANDARD) g_gemFocus = GEM_FOCUS_CREATE_REPEATABLE;
                                else g_gemFocus = GEM_FOCUS_CREATE_ADD_CYCLE;
                            }
                            else if (g_gemFocus == GEM_FOCUS_CREATE_ADD_CYCLE) { g_gemFocus = GEM_FOCUS_CREATE_IMAGE; } // FIX: To Image
                            else if (g_gemFocus == GEM_FOCUS_CREATE_REPEATABLE) { g_gemFocus = GEM_FOCUS_CREATE_IMAGE; } // FIX: To Image
                            else if (g_gemFocus == GEM_FOCUS_CREATE_IMAGE) { g_gemFocus = GEM_FOCUS_FOOTER; g_gem_footer_selection = 0; needsFooterRedraw = true; } // FIX
                            
                            needsFullRedraw = true;
                            g_forceCursorRedraw = true;
                            gem_hide_autocomplete(false);
                        }
                    }
                    else if (controlAction == "LEFT") { 
                        if (focusedEditor->moveCursor(-1, 0)) needsFullRedraw = true;
                        g_forceCursorRedraw = true;
                    }
                    else if (controlAction == "RIGHT") { 
                        if (focusedEditor->moveCursor(1, 0)) needsFullRedraw = true;
                        g_forceCursorRedraw = true;
                    }
                    else if (controlAction == "UP") {
                        if (g_gem_autocomplete_active) {
                            g_gem_autocomplete_selection--;
                            if (g_gem_autocomplete_selection < 0) g_gem_autocomplete_selection = g_gem_autocomplete_count - 1;
                            g_forceCursorRedraw = true;
                        } else {
                            if (g_gemFocus == GEM_FOCUS_CREATE_TITLE) { g_gemFocus = GEM_FOCUS_HEADER; }
                            else if (g_gemFocus == GEM_FOCUS_CREATE_DESC) { g_gemFocus = GEM_FOCUS_CREATE_TITLE; }
                            else if (g_gemFocus == GEM_FOCUS_CREATE_SKILLS) { g_gemFocus = GEM_FOCUS_CREATE_DESC; }
                            else if (g_gemFocus == GEM_FOCUS_CREATE_DIFF) { g_gemFocus = GEM_FOCUS_CREATE_SKILLS; }
                            else if (g_gemFocus == GEM_FOCUS_CREATE_DESIRE) { g_gemFocus = GEM_FOCUS_CREATE_DIFF; } 
                            else if (g_gemFocus == GEM_FOCUS_CREATE_TIME) { g_gemFocus = GEM_FOCUS_CREATE_DESIRE; }
                            else if (g_gemFocus == GEM_FOCUS_CREATE_MINS) { g_gemFocus = GEM_FOCUS_CREATE_TIME; } 
                            else if (g_gemFocus == GEM_FOCUS_CREATE_REPEATABLE) { g_gemFocus = GEM_FOCUS_CREATE_MINS; } 
                            else if (g_gemFocus == GEM_FOCUS_CREATE_ADD_CYCLE) { g_gemFocus = GEM_FOCUS_CREATE_MINS; } 
                            else if (g_gemFocus == GEM_FOCUS_CREATE_IMAGE) { 
                                if (g_gem_editor_questType == QT_STANDARD) g_gemFocus = GEM_FOCUS_CREATE_REPEATABLE;
                                else g_gemFocus = GEM_FOCUS_CREATE_ADD_CYCLE;
                            }
                            
                            needsFullRedraw = true;
                            g_forceCursorRedraw = true;
                            gem_hide_autocomplete(false);
                        }
                    } 
                    else if (controlAction == "DOWN") {
                        if (g_gem_autocomplete_active) {
                            g_gem_autocomplete_selection++;
                            if (g_gem_autocomplete_selection >= g_gem_autocomplete_count) g_gem_autocomplete_selection = 0;
                            g_forceCursorRedraw = true;
                        } else {
                            if (g_gemFocus == GEM_FOCUS_CREATE_TITLE) { g_gemFocus = GEM_FOCUS_CREATE_DESC; }
                            else if (g_gemFocus == GEM_FOCUS_CREATE_DESC) { g_gemFocus = GEM_FOCUS_CREATE_SKILLS; }
                            else if (g_gemFocus == GEM_FOCUS_CREATE_SKILLS) { g_gemFocus = GEM_FOCUS_CREATE_DIFF; }
                            else if (g_gemFocus == GEM_FOCUS_CREATE_DIFF) { g_gemFocus = GEM_FOCUS_CREATE_DESIRE; }
                            else if (g_gemFocus == GEM_FOCUS_CREATE_DESIRE) { g_gemFocus = GEM_FOCUS_CREATE_TIME; } 
                            else if (g_gemFocus == GEM_FOCUS_CREATE_TIME) { g_gemFocus = GEM_FOCUS_CREATE_MINS; } 
                            else if (g_gemFocus == GEM_FOCUS_CREATE_MINS) { 
                                if (g_gem_editor_questType == QT_STANDARD) g_gemFocus = GEM_FOCUS_CREATE_REPEATABLE;
                                else g_gemFocus = GEM_FOCUS_CREATE_ADD_CYCLE;
                            }
                            else if (g_gemFocus == GEM_FOCUS_CREATE_REPEATABLE) { g_gemFocus = GEM_FOCUS_CREATE_IMAGE; } // FIX: To Image
                            else if (g_gemFocus == GEM_FOCUS_CREATE_ADD_CYCLE) { g_gemFocus = GEM_FOCUS_CREATE_IMAGE; } // FIX: To Image
                            else if (g_gemFocus == GEM_FOCUS_CREATE_IMAGE) { g_gemFocus = GEM_FOCUS_FOOTER; g_gem_footer_selection = 0; needsFooterRedraw = true; } // FIX

                            needsFullRedraw = true;
                            g_forceCursorRedraw = true;
                            gem_hide_autocomplete(false);
                        }
                    }
                }
            }
            break;
            
        case GEM_FOCUS_CREATE_REPEATABLE: 
            if (buttonIndex == IDX_PREV) { // Button 0: UP
                g_gemFocus = GEM_FOCUS_CREATE_MINS; 
                needsFullRedraw = true;
                g_forceCursorRedraw = true;
            }
            else if (buttonIndex == IDX_NEXT) { // Button 1: DOWN
                g_gemFocus = GEM_FOCUS_CREATE_IMAGE; // FIX: Go to Image
                needsFullRedraw = true; // Image requires full refresh
                g_forceCursorRedraw = true;
            }
            else if (buttonIndex == IDX_BACK) { // Button 3: BACK (acts as UP)
                g_gemFocus = GEM_FOCUS_CREATE_MINS;
                needsFullRedraw = true;
                g_forceCursorRedraw = true;
            }
            else if (buttonIndex == IDX_SELECT) { // Button 2: CONFIRM
                g_gem_editor_isRepeatable = !g_gem_editor_isRepeatable; 
                needsFullRedraw = true; 
                g_forceCursorRedraw = true;
            }
            break;
            
        // --- NEW: Image Selection Focus ---
        case GEM_FOCUS_CREATE_IMAGE:
            if (buttonIndex == IDX_PREV) { 
                if (g_gem_editor_questType == QT_STANDARD) g_gemFocus = GEM_FOCUS_CREATE_REPEATABLE;
                else g_gemFocus = GEM_FOCUS_CREATE_ADD_CYCLE;
                needsFullRedraw = true;
            }
            else if (buttonIndex == IDX_NEXT) { g_gemFocus = GEM_FOCUS_FOOTER; g_gem_footer_selection = 0; needsFooterRedraw = true; }
            else if (buttonIndex == IDX_SELECT) { 
                tft.endWrite();
                g_gemState = GEM_STATE_IMAGE_SELECT;
                gem_buildBMPList();
                gem_drawImageSelect();
                return;
            }
            break;
            
        // --- Merchant Toggle ---
        case GEM_FOCUS_MERCHANT_CREATE_CONSUMABLE:
            if (buttonIndex == IDX_PREV) { g_gemFocus = GEM_FOCUS_MERCHANT_CREATE_DESIRE; needsFullRedraw = true; }
            else if (buttonIndex == IDX_NEXT) { g_gemFocus = GEM_FOCUS_MERCHANT_CREATE_IMAGE; needsFullRedraw = true; }
            else if (buttonIndex == IDX_SELECT) { g_gem_editor_isConsumable = !g_gem_editor_isConsumable; needsFullRedraw = true; }
            break;

        // --- Merchant Image ---
        case GEM_FOCUS_MERCHANT_CREATE_IMAGE:
             if (buttonIndex == IDX_PREV) { g_gemFocus = GEM_FOCUS_MERCHANT_CREATE_CONSUMABLE; needsFullRedraw = true; }
             else if (buttonIndex == IDX_NEXT) { g_gemFocus = GEM_FOCUS_FOOTER; g_gem_footer_selection = 0; needsFooterRedraw = true; }
             else if (buttonIndex == IDX_SELECT) {
                 tft.endWrite();
                 g_gemState = GEM_STATE_IMAGE_SELECT;
                 gem_buildBMPList();
                 gem_drawImageSelect();
                 return;
             }
             break;

        case GEM_FOCUS_FOOTER:
            { 
                int max_selection = 1;
                if ((g_gemState == GEM_STATE_CREATE_QUEST && g_gem_currentQuestID != -1) || 
                    (g_gemState == GEM_STATE_INVENTORY_VIEW) ||
                    (g_gemState == GEM_STATE_MERCHANT_EDIT && g_merchant_currentItemID != -1)) {
                    max_selection = 2;
                }
                // Stats has 2 options (0, 1, 2) -> Inventory, Settings, Exit
                if (g_gemState == GEM_STATE_STATS) max_selection = 2;
                // Quest List has 2 options (0, 1, 2) -> Stats, Merchant, Exit
                if (g_gemState == GEM_STATE_QUEST_LIST) max_selection = 2;

                if (buttonIndex == IDX_PREV) { // Button 0: LEFT
                    if (g_gem_footer_selection == 0) {
                        // Move UP out of footer
                        if (g_gemState == GEM_STATE_QUEST_LIST) {
                            g_gemFocus = GEM_FOCUS_LIST;
                            g_gem_list_selection = g_questCount; 
                            g_gem_marquee_active = true;
                            needsFooterRedraw = true; 
                            gem_drawListItem(g_gem_list_selection, false);
                        }
                        else if (g_gemState == GEM_STATE_CREATE_QUEST) {
                            g_gemFocus = GEM_FOCUS_CREATE_IMAGE; // FIX: Go to Image selector
                            needsFullRedraw = true;
                            needsFooterRedraw = true;
                        }
                        else if (g_gemState == GEM_STATE_TYPE_SELECT) { 
                            g_gemFocus = GEM_FOCUS_TYPE_SELECT_LIST;
                            g_gem_type_select_selection = 1; // Select last item
                            needsFullRedraw = true;
                            needsFooterRedraw = true;
                        }
                        else if (g_gemState == GEM_STATE_STATS) {
                            g_gemFocus = GEM_FOCUS_STATS_LIST;
                            const int visible_count = 6;
                            int lastPossibleIndex = g_playerSkillCount - 1;
                            if (lastPossibleIndex < 0) lastPossibleIndex = 0;
                            if (g_playerSkillCount <= visible_count) {
                                g_gem_stats_top_skill = 0;
                            } else {
                                g_gem_stats_top_skill = g_playerSkillCount - visible_count;
                            }
                            g_gem_stats_selection = lastPossibleIndex;
                            needsFullRedraw = true;
                            needsFooterRedraw = true; 
                        }
                        else if (g_gemState == GEM_STATE_MERCHANT) {
                            g_gemFocus = GEM_FOCUS_MERCHANT_LIST;
                            g_merchant_list_selection = g_merchantItemCount - 1;
                            if (g_merchant_list_selection < 0) g_merchant_list_selection = 0;
                            // Check visibility and scroll if needed
                            if (gem_getMerchantItemY(g_merchant_list_selection) == -1) {
                                while(gem_getMerchantItemY(g_merchant_list_selection) == -1 && g_merchant_list_top_item < g_merchantItemCount) {
                                    g_merchant_list_top_item++;
                                }
                            }
                            needsFullRedraw = true; // Full redraw needed for scrolling
                        }
                        else if (g_gemState == GEM_STATE_MERCHANT_EDIT) {
                            g_gemFocus = GEM_FOCUS_MERCHANT_CREATE_IMAGE;
                            needsFullRedraw = true;
                            needsFooterRedraw = true;
                        }
                        else if (g_gemState == GEM_STATE_INVENTORY) {
                            g_gemFocus = GEM_FOCUS_INVENTORY_LIST;
                            g_inventory_list_selection = g_inventoryItemCount - 1;
                            if (g_inventory_list_selection < 0) g_inventory_list_selection = 0;
                            needsFullRedraw = true; // Full redraw for Inventory List
                        }
                        g_gem_marquee_offset = 0;
                        g_forceCursorRedraw = true; 
                    } else {
                        // We are NOT at the left-most item, so move LEFT
                        g_gem_footer_selection--;
                        needsFooterRedraw = true;
                    }
                }
                else if (buttonIndex == IDX_NEXT) { // Button 1: RIGHT
                    if (g_gem_footer_selection < max_selection) {
                        g_gem_footer_selection++;
                        needsFooterRedraw = true;
                    }
                }
                else if (buttonIndex == IDX_SELECT) { // Button 2: CONFIRM (Action)
                    String action;
                    // Define actions based on state
                    if (g_gemState == GEM_STATE_QUEST_LIST) {
                        if (g_gem_footer_selection == 0) action = "Stats";
                        else if (g_gem_footer_selection == 1) action = "Merchant";
                        else action = "Exit";
                    }
                    else if (g_gemState == GEM_STATE_STATS) {
                        if (g_gem_footer_selection == 0) action = "Inventory";
                        else if (g_gem_footer_selection == 1) action = "Settings";
                        else action = "Exit";
                    }
                    else if (g_gemState == GEM_STATE_TYPE_SELECT) {
                         if (g_gem_footer_selection == 0) action = "NewType";
                         else action = "Back";
                    }
                    else if (g_gemState == GEM_STATE_MERCHANT) {
                        if (g_gem_footer_selection == 0) action = "NewItem";
                        else action = "Exit";
                    }
                    else if (g_gemState == GEM_STATE_INVENTORY) {
                        if (g_gem_footer_selection == 0) action = "Back";
                        else action = "Exit";
                    }
                    else if (g_gemState == GEM_STATE_INVENTORY_VIEW) {
                        if (g_gem_footer_selection == 0) action = "Sell";
                        else if (g_gem_footer_selection == 1) action = "Back";
                        else action = "Delete";
                    }
                    else if (g_gemState == GEM_STATE_MERCHANT_EDIT) {
                        if (g_gem_footer_selection == 0) action = "SaveItem";
                        else if (g_gem_footer_selection == 1) action = "Back";
                        else action = "DeleteItem";
                    }
                    else { // CREATE_QUEST
                        if (g_gem_footer_selection == 0) action = "Save";
                        else if (g_gem_footer_selection == 1) action = "Back";
                        else action = "Delete";
                    }

                    // Execute Actions
                    if (action == "Exit") { tft.endWrite(); gem_stop(); return; }
                    else if (action == "Back") { 
                        tft.endWrite(); 
                        if(g_gemState == GEM_STATE_MERCHANT_EDIT) gem_returnToMerchantList();
                        else if(g_gemState == GEM_STATE_INVENTORY_VIEW) {
                            g_gemState = GEM_STATE_INVENTORY;
                            g_gemFocus = GEM_FOCUS_INVENTORY_LIST;
                            tft.startWrite(); gem_drawHeader(true); gem_drawInventoryList(); gem_drawFooter(true); tft.endWrite();
                        }
                        else gem_start(); 
                        return; 
                    }
                    else if (action == "Stats") {
                        tft.endWrite();
                        g_gemFocus = GEM_FOCUS_FOOTER; g_gem_footer_selection=0;
                        gem_drawStatisticsPage();
                        return;
                    }
                    else if (action == "Merchant") {
                        tft.endWrite();
                        g_gemState = GEM_STATE_MERCHANT;
                        g_gemFocus = GEM_FOCUS_MERCHANT_LIST;
                        g_merchant_list_selection = 0;
                        tft.startWrite(); gem_drawHeader(true); gem_drawMerchantList(); gem_drawFooter(true); tft.endWrite();
                        return;
                    }
                    else if (action == "Inventory") {
                        tft.endWrite();
                        g_gemState = GEM_STATE_INVENTORY;
                        g_gemFocus = GEM_FOCUS_INVENTORY_LIST;
                        g_inventory_list_selection = 0;
                        tft.startWrite(); gem_drawHeader(true); gem_drawInventoryList(); gem_drawFooter(true); tft.endWrite();
                        return;
                    }
                    else if (action == "NewItem") { // Merchant New Item
                        g_merchant_currentItemID = -1;
                        g_gemState = GEM_STATE_MERCHANT_EDIT;
                        g_gemFocus = GEM_FOCUS_MERCHANT_CREATE_TITLE;
                        gem_drawMerchantEditor();
                        g_forceCursorRedraw = true;
                        return;
                    }
                    else if (action == "SaveItem") { // Merchant Save
                        if (g_gemState == GEM_STATE_MERCHANT_EDIT) {
                             String title = gemCreateItemTitleEditor->lines[0];
                             String desc = gem_join_notes_from_editor(gemCreateItemDescEditor);
                             int r = gemCreateItemRarityEditor->lines[0].toInt();
                             int q = gemCreateItemQualityEditor->lines[0].toInt();
                             int n = gemCreateItemNeedEditor->lines[0].toInt();
                             int d = gemCreateItemDesireEditor->lines[0].toInt();
                             bool isConsumable = g_gem_editor_isConsumable;
                             
                             if (title.length() == 0) { pushSystemMessage("Title cannot be empty."); }
                             else {
                                 if (g_merchant_currentItemID == -1) {
                                     addMerchantItem(title, desc, r, q, n, d, isConsumable);
                                     g_merchantDB[g_merchantItemCount - 1].iconFile = g_gem_temp_icon_file;
                                     pushSystemMessage("New item created!");
                                 } else {
                                     MerchantItem& item = g_merchantDB[g_merchant_currentItemID];
                                     item.title = title; item.description = desc; item.rarity = r;
                                     item.quality = q; item.need = n; item.desire = d;
                                     item.isConsumable = isConsumable;
                                     item.price = gem_calculate_item_price(r, q, n, d, isConsumable);
                                     item.iconFile = g_gem_temp_icon_file;
                                     pushSystemMessage("Item updated!");
                                 }
                                 gem_save_database(g_gem_db_filename);
                                 tft.endWrite();
                                 gem_returnToMerchantList();
                                 return;
                             }
                        }
                    }
                    else if (action == "DeleteItem") { // Merchant Delete
                        if (g_gemState == GEM_STATE_MERCHANT_EDIT && g_merchant_currentItemID != -1) {
                            tft.endWrite();
                            gem_deleteMerchantItem(g_merchant_currentItemID);
                            gem_returnToMerchantList();
                            return;
                        }
                    }
                    
                    else if (action == "Delete") {
                        // 1. Quest Delete
                        if (g_gemState == GEM_STATE_CREATE_QUEST && g_gem_currentQuestID != -1) {
                            tft.endWrite();
                            gem_deleteQuest(g_gem_currentQuestID);
                            gem_start();
                            return;
                        }
                        // 2. Inventory Item Delete (FIXED LOGIC)
                        else if (g_gemState == GEM_STATE_INVENTORY_VIEW && g_inventory_currentItemID != -1) {
                            tft.endWrite();
                            
                            gem_deleteInventoryItem(g_inventory_currentItemID);
                            g_inventory_currentItemID = -1; // Reset View ID

                            // Adjust list pointers
                            if (g_inventoryItemCount == 0) {
                                g_inventory_list_selection = 0;
                                g_inventory_list_top_item = 0;
                            } else {
                                if (g_inventory_list_selection >= g_inventoryItemCount) g_inventory_list_selection = g_inventoryItemCount - 1;
                                if (g_inventory_list_top_item > g_inventory_list_selection) g_inventory_list_top_item = g_inventory_list_selection;
                            }

                            g_gemState = GEM_STATE_INVENTORY;
                            g_gemFocus = GEM_FOCUS_INVENTORY_LIST;

                            tft.startWrite();
                            gem_drawHeader(true);
                            gem_drawInventoryList();
                            gem_drawFooter(true);
                            tft.endWrite();
                            g_forceCursorRedraw = true;
                            return;
                        }
                    }

                    else if (action == "Save") { // Save Quest (Updated with Icon)
                        if (g_gemState == GEM_STATE_CREATE_QUEST) {
                            String title = gemCreateTitleEditor->lines[0];
                            String desc = gemCreateDescEditor->lines[0]; 
                            String notes = gem_join_notes_from_editor(gemCreateDescEditor);
                            String skills = gem_join_notes_from_editor(gemCreateSkillsEditor);
                            skills.replace("\n", "");
                            int diff = gemCreateDiffEditor->lines[0].toInt();
                            int desire = gemCreateDesireEditor->lines[0].toInt();
                            int totalTimeMins = (gemCreateTimeEditor->lines[0].toInt() * 60) + gemCreateMinsEditor->lines[0].toInt();
                            int addCycleMins = (g_gem_editor_questType == QT_CRUSADE) ? gemCreateAddCycleEditor->lines[0].toInt() : 0;
                            
                            gem_add_skills_to_database(skills);

                            if (g_gem_currentQuestID == -1) {
                                // FIX: PASS g_gem_temp_quest_icon
                                addQuest(title, desc, notes, skills, g_gem_editor_isRepeatable, diff, desire, totalTimeMins, g_gem_editor_questType, addCycleMins, g_gem_temp_quest_icon);
                                pushSystemMessage("New quest created!");
                            } else {
                                Quest& quest = g_questDB[g_gem_currentQuestID];
                                quest.title = title;
                                quest.description = desc;
                                quest.notes = notes; 
                                quest.skills = skills;
                                quest.difficulty = diff;
                                quest.desire = desire;
                                quest.timerDurationMinutes = totalTimeMins;
                                quest.addCycleMinutes = addCycleMins;
                                quest.iconFile = g_gem_temp_quest_icon; // FIX: SAVE ICON
                                
                                // Recalculate rewards
                                float timeHours = (float)totalTimeMins / 60.0f;
                                float base_reward = (diff * 10) + (timeHours * 2.0f);
                                float desire_multiplier = 1.0 + ( (7.0 - (float)desire) / 6.0 );
                                quest.gemReward = (int)(base_reward * desire_multiplier);
                                quest.xpReward = (quest.difficulty * 5) + (int)(desire_multiplier * 5) + (int)(timeHours * 2.0f);
                                
                                pushSystemMessage("Quest updated!");
                            }
                            gem_save_database(g_gem_db_filename);
                            tft.endWrite();
                            gem_start();
                            return;
                        }
                    }
                    
                    else if (action == "Sell") {
                        if (g_gemState == GEM_STATE_INVENTORY_VIEW && g_inventory_currentItemID != -1) {
                             MerchantItem& item = g_inventoryDB[g_inventory_currentItemID];
                             int gemsGained = max(1, (int)(item.price * 0.75));
                             g_gem_count += gemsGained;
                             // Move back to merchant
                             addMerchantItem(item.title, item.description, item.rarity, item.quality, item.need, item.desire, item.isConsumable);
                             g_merchantDB[g_merchantItemCount-1].iconFile = item.iconFile; 
                             
                             gem_deleteInventoryItem(g_inventory_currentItemID);
                             
                             // Handle pointers
                             g_inventory_currentItemID = -1;
                             if (g_inventoryItemCount == 0) {
                                g_inventory_list_selection = 0;
                                g_inventory_list_top_item = 0;
                             } else {
                                if (g_inventory_list_selection >= g_inventoryItemCount) g_inventory_list_selection = g_inventoryItemCount - 1;
                                if (g_inventory_list_top_item > g_inventory_list_selection) g_inventory_list_top_item = g_inventory_list_selection;
                             }
                             
                             pushSystemMessage("Sold " + item.title + " for " + String(gemsGained) + " G.");
                             g_gemState = GEM_STATE_INVENTORY;
                             g_gemFocus = GEM_FOCUS_INVENTORY_LIST;
                             tft.startWrite(); gem_drawHeader(true); gem_drawInventoryList(); gem_drawFooter(true); tft.endWrite();
                             g_forceCursorRedraw = true;
                             return;
                        }
                    }
                }
            }
            break;
    } 

    if (needsFullRedraw) {
        if (g_gemState == GEM_STATE_QUEST_LIST) {
            g_gem_marquee_offset = 0;
            gem_drawQuestList();
        } 
        else if (g_gemState == GEM_STATE_TYPE_SELECT) { 
            gem_drawTypeSelectPage();
        }
        else if (g_gemState == GEM_STATE_CREATE_QUEST) {
            gem_refreshCreateQuestView();
            if (needsFooterRedraw) gem_drawFooter(true);
        }
        else if (g_gemState == GEM_STATE_STATS) {
            tft.endWrite();
            gem_drawStatisticsPage();
            g_forceCursorRedraw = true;
            return;
        }
        else if (g_gemState == GEM_STATE_MERCHANT) {
            tft.endWrite();
            gem_drawMerchantList();
            g_forceCursorRedraw = true;
            return;
        }
        else if (g_gemState == GEM_STATE_MERCHANT_EDIT) {
             gem_refreshMerchantEditorView();
             if (needsFooterRedraw) gem_drawFooter(true);
        }
         else if (g_gemState == GEM_STATE_INVENTORY) {
            tft.endWrite();
            gem_drawInventoryList();
            g_forceCursorRedraw = true;
            return;
        }
    } else if (needsFooterRedraw) {
        gem_drawFooter(true);
    }

    tft.endWrite();
}
void gem_handleSerialInput(String cmdLine) {
    const int MAX_GEM_TOKENS = 3;
    String tokens[MAX_GEM_TOKENS];
    int count = 0;
    tokenizeLine(cmdLine, tokens, count, MAX_GEM_TOKENS);

    String gemCmd = "";
    if (count > 0) {
        gemCmd = tokens[0];
        gemCmd.toLowerCase();
    }

    auto isNumeric = [](String s) {
        if (s.length() == 0) return false;
        for (unsigned int i = 0; i < s.length(); i++) {
            if (!isDigit(s.charAt(i))) return false;
        }
        return true;
    };

    bool isCommand = true;
    bool needsFooterRedraw = false;

    // --- 1. LIST VIEW LOGIC ---
    if (g_gemState == GEM_STATE_QUEST_LIST) {
        
        g_gem_marquee_active = false;

        if (gemCmd == "up") {
            tft.startWrite();
            int old_selection = g_gem_list_selection;
            GemAppFocus old_focus = g_gemFocus; 
            g_gem_marquee_offset = 0;

            if (g_gemFocus == GEM_FOCUS_FOOTER) {
                g_gemFocus = GEM_FOCUS_LIST;
                g_gem_list_selection = g_questCount; 
                g_gem_marquee_active = true;
                gem_drawFooter(true);
                gem_drawListItem(g_gem_list_selection, false); 
            } 
            else {
                g_gem_list_selection--;
                if (g_gem_list_selection < 0) {
                     g_gem_list_selection = 0;
                }
            }
            
            if (old_focus != g_gemFocus) {
                if (old_focus == GEM_FOCUS_FOOTER) {
                   // gem_drawListItem(g_gem_list_selection); // Already drawn
                } else if (old_focus == GEM_FOCUS_LIST) {
                    gem_drawListItem(old_selection, false); // Pass false for drawBar
                }
            } else if (old_selection != g_gem_list_selection) {
                gem_drawListItem(old_selection, false);
                gem_drawListItem(g_gem_list_selection, false); 
            }

            g_forceCursorRedraw = true;
            tft.endWrite();
        } 
        else if (gemCmd == "down") {
            tft.startWrite();
            int old_selection = g_gem_list_selection;
            GemAppFocus old_focus = g_gemFocus; 
            g_gem_marquee_offset = 0;

            if (g_gemFocus == GEM_FOCUS_HEADER) {
                g_gemFocus = GEM_FOCUS_LIST;
                gem_drawHeader(false); 
            } else {
                g_gem_list_selection++;
                if (g_gem_list_selection >= (g_questCount + 1)) {
                    g_gem_list_selection = g_questCount;
                    g_gemFocus = GEM_FOCUS_FOOTER;    
                    g_gem_footer_selection = 0;
                    gem_drawFooter(true); 
                }
            }
            
            if (old_focus != g_gemFocus) {
                gem_drawListItem(old_selection, false);
                if (g_gemFocus == GEM_FOCUS_LIST) {
                    gem_drawListItem(g_gem_list_selection, false);
                }
            } else if (old_selection != g_gem_list_selection) {
                gem_drawListItem(old_selection, false);
                gem_drawListItem(g_gem_list_selection, false); 
            }

            g_forceCursorRedraw = true;
            tft.endWrite();
        } 
        
        else if (gemCmd == "enter" || gemCmd == "open") {
            if (g_gemFocus == GEM_FOCUS_FOOTER) {
                if (g_gem_footer_selection == 0) { // Stats
                    g_gemFocus = GEM_FOCUS_FOOTER;
                    g_gem_footer_selection = 0;
                    gem_drawStatisticsPage();
                    g_forceCursorRedraw = true;
                    return;
                } else { // Exit
                    gem_stop();
                    return;
                }
            }
            
            if (g_gem_list_selection == 0) {
                // "New Quest" selected
                g_gem_currentQuestID = -1;
                g_gemState = GEM_STATE_TYPE_SELECT;
                g_gemFocus = GEM_FOCUS_TYPE_SELECT_LIST;
                g_gem_type_select_selection = 0;
                gem_drawTypeSelectPage();
            } else {
                // Existing quest selected
                g_gem_currentQuestID = g_gem_list_selection - 1;
                g_gem_editor_questType = g_questDB[g_gem_currentQuestID].type;
                g_gemState = GEM_STATE_CREATE_QUEST;
                g_gemFocus = GEM_FOCUS_CREATE_TITLE;
                gem_drawQuestEditor();
            }
            g_forceCursorRedraw = true;
            return;
        } 
        else if (gemCmd == "stats") {
            g_gemFocus = GEM_FOCUS_FOOTER;
            g_gem_footer_selection = 0;
            gem_drawStatisticsPage();
            g_forceCursorRedraw = true;
            return;
        }
        else if (gemCmd == "focus") {
            if (count > 1) {
                if (tokens[1] == "footer") g_gemFocus = GEM_FOCUS_FOOTER;
            }
            g_gem_marquee_offset = 0;
            gem_drawQuestList();
            g_forceCursorRedraw = true;
        } else if (gemCmd == "left" || gemCmd == "right") {
            if (g_gemFocus == GEM_FOCUS_FOOTER) {
                if (gemCmd == "left") {
                    if (g_gem_footer_selection != 0) {
                        g_gem_footer_selection = 0;
                        tft.startWrite();
                        gem_drawFooter(true); tft.endWrite();
                    }
                } else { 
                    if (g_gem_footer_selection != 1) {
                        g_gem_footer_selection = 1;
                        tft.startWrite();
                        gem_drawFooter(true); tft.endWrite();
                    }
                }
                g_forceCursorRedraw = true;
            } else if (g_gemFocus == GEM_FOCUS_LIST && g_gem_list_selection > 0) {
                int db_index = g_gem_list_selection - 1;
                QuestState oldState = g_questDB[db_index].questState;
                
                bool success = gem_cycle_quest_state(db_index); 
                
                if (success) {
                    if (oldState == STATE_STARTED) { 
                        g_gem_list_selection = g_questCount;
                    } else { 
                        g_gem_list_selection = 1;
                    }
                    g_gem_marquee_offset = 0;
                    gem_drawQuestList(); 
                } else {
                    g_gem_marquee_active = true;
                }
                g_forceCursorRedraw = true;
            } else {
                g_forceCursorRedraw = true;
            }
        } 
        // --- THIS IS THE FIX ---
        else if (gemCmd == "delete") {
            if (g_gemFocus == GEM_FOCUS_LIST && g_gem_list_selection > 0) {
                // This is a valid quest to delete (not "[New Quest]")
                int db_index = g_gem_list_selection - 1;
                
                gem_deleteQuest(db_index); // This function handles the deletion and saves
                
                // Reset selection to the top ("New Quest")
                g_gem_list_selection = 0;
                g_gem_marquee_offset = 0;
                
                // Redraw the entire list to reflect the change
                tft.startWrite();
                gem_drawQuestList();
                tft.endWrite();
                g_forceCursorRedraw = true; // Force cursor to update to the new selection
            }
            // If they try to delete "[New Quest]" or focus is elsewhere, do nothing.
        }
        // --- END OF FIX ---
        else {
            isCommand = false;
        }

    }
    // --- 2. NEW: TYPE SELECT PAGE LOGIC ---
    else if (g_gemState == GEM_STATE_TYPE_SELECT) {
        bool needsFullRedraw = false;
        if (gemCmd == "up" || gemCmd == "back") {
            if (g_gemFocus == GEM_FOCUS_FOOTER) {
                g_gemFocus = GEM_FOCUS_TYPE_SELECT_LIST;
                g_gem_type_select_selection = 1; // Select last item
                needsFullRedraw = true;
            } else if (g_gemFocus == GEM_FOCUS_TYPE_SELECT_LIST) {
                g_gem_type_select_selection--;
                if (g_gem_type_select_selection < 0) {
                    g_gem_type_select_selection = 0;
                    g_gemFocus = GEM_FOCUS_HEADER;
                }
                needsFullRedraw = true;
            }
        } else if (gemCmd == "down") {
            if (g_gemFocus == GEM_FOCUS_HEADER) {
                g_gemFocus = GEM_FOCUS_TYPE_SELECT_LIST;
                g_gem_type_select_selection = 0;
                needsFullRedraw = true;
            } else if (g_gemFocus == GEM_FOCUS_TYPE_SELECT_LIST) {
                g_gem_type_select_selection++;
                if (g_gem_type_select_selection > 1) { // 2 items max
                    g_gem_type_select_selection = 1;
                    g_gemFocus = GEM_FOCUS_FOOTER;
                    g_gem_footer_selection = 0;
                    needsFooterRedraw = true;
                } else {
                    needsFullRedraw = true;
                }
            }
        } else if (gemCmd == "left") {
            if (g_gemFocus == GEM_FOCUS_FOOTER && g_gem_footer_selection != 0) {
                g_gem_footer_selection = 0;
                needsFooterRedraw = true;
            }
        } else if (gemCmd == "right") {
            if (g_gemFocus == GEM_FOCUS_FOOTER && g_gem_footer_selection != 1) {
                g_gem_footer_selection = 1;
                needsFooterRedraw = true;
            }
        } else if (gemCmd == "enter") {
            if (g_gemFocus == GEM_FOCUS_TYPE_SELECT_LIST) {
                 if (g_gem_type_select_selection == 0) g_gem_editor_questType = QT_STANDARD;
                 else g_gem_editor_questType = QT_CRUSADE;
                 
                 g_gemState = GEM_STATE_CREATE_QUEST;
                 g_gemFocus = GEM_FOCUS_CREATE_TITLE;
                 gem_drawQuestEditor();
                 g_forceCursorRedraw = true;
                 return;
            } else if (g_gemFocus == GEM_FOCUS_FOOTER) {
                String action = (g_gem_footer_selection == 0) ?
                                "NewType" : "Back";
                if (action == "NewType") {
                     pushSystemMessage("New quest type editor not yet implemented.");
                     needsFullRedraw = true;
                } else { // Back
                     gem_start();
                     return;
                }
            }
        } else if (gemCmd == "exit") {
             gem_start();
             return;
        }

        if (needsFullRedraw) {
            tft.startWrite();
            gem_drawTypeSelectPage();
            tft.endWrite();
            g_forceCursorRedraw = true;
        } else if (needsFooterRedraw) {
            tft.startWrite();
            gem_drawFooter(true);
            tft.endWrite();
            g_forceCursorRedraw = true;
        }
    }
    // --- 3. UNIFIED EDITOR LOGIC ---
    else if (g_gemState == GEM_STATE_CREATE_QUEST) {
        
        TextArea* focusedEditor = nullptr;
        if (g_gemFocus == GEM_FOCUS_CREATE_TITLE) focusedEditor = gemCreateTitleEditor;
        else if (g_gemFocus == GEM_FOCUS_CREATE_DESC) focusedEditor = gemCreateDescEditor;
        else if (g_gemFocus == GEM_FOCUS_CREATE_SKILLS) focusedEditor = gemCreateSkillsEditor;
        else if (g_gemFocus == GEM_FOCUS_CREATE_DIFF) focusedEditor = gemCreateDiffEditor;
        else if (g_gemFocus == GEM_FOCUS_CREATE_DESIRE) focusedEditor = gemCreateDesireEditor;
        else if (g_gemFocus == GEM_FOCUS_CREATE_TIME) focusedEditor = gemCreateTimeEditor;
        else if (g_gemFocus == GEM_FOCUS_CREATE_MINS) focusedEditor = gemCreateMinsEditor;
        else if (g_gemFocus == GEM_FOCUS_CREATE_ADD_CYCLE) focusedEditor = gemCreateAddCycleEditor; 

        bool needsFullRedraw = false;

        if (gemCmd == "up") {
            if (g_gem_autocomplete_active) {
                g_gem_autocomplete_selection--;
                if (g_gem_autocomplete_selection < 0) g_gem_autocomplete_selection = g_gem_autocomplete_count - 1;
            }
            else if (focusedEditor && focusedEditor->cursorLine > 0) {
                tft.startWrite();
                focusedEditor->moveCursor(0, -1);
                tft.endWrite();
            } else {
                GemAppFocus oldFocus = g_gemFocus;

                if (g_gemFocus == GEM_FOCUS_FOOTER) {
                    if (g_gem_editor_questType == QT_STANDARD) g_gemFocus = GEM_FOCUS_CREATE_REPEATABLE;
                    else g_gemFocus = GEM_FOCUS_CREATE_ADD_CYCLE;
                }
                else if (g_gemFocus == GEM_FOCUS_CREATE_REPEATABLE) { g_gemFocus = GEM_FOCUS_CREATE_MINS;
                } 
                else if (g_gemFocus == GEM_FOCUS_CREATE_ADD_CYCLE) { g_gemFocus = GEM_FOCUS_CREATE_MINS;
                } 
                else if (g_gemFocus == GEM_FOCUS_CREATE_MINS) { g_gemFocus = GEM_FOCUS_CREATE_TIME;
                } 
                else if (g_gemFocus == GEM_FOCUS_CREATE_TIME) { g_gemFocus = GEM_FOCUS_CREATE_DESIRE;
                } 
                else if (g_gemFocus == GEM_FOCUS_CREATE_DESIRE) { g_gemFocus = GEM_FOCUS_CREATE_DIFF;
                } 
                else if (g_gemFocus == GEM_FOCUS_CREATE_DIFF) { g_gemFocus = GEM_FOCUS_CREATE_SKILLS;
                }
                else if (g_gemFocus == GEM_FOCUS_CREATE_SKILLS) { g_gemFocus = GEM_FOCUS_CREATE_DESC;
                }
                else if (g_gemFocus == GEM_FOCUS_CREATE_DESC) { g_gemFocus = GEM_FOCUS_CREATE_TITLE;
                }
                else if (g_gemFocus == GEM_FOCUS_CREATE_TITLE) { g_gemFocus = GEM_FOCUS_HEADER;
                } 
                
                if (oldFocus != g_gemFocus) {
                    needsFullRedraw = true;
                    if (oldFocus == GEM_FOCUS_FOOTER) needsFooterRedraw = true; 
                    gem_hide_autocomplete(false); 
                }
            }
            g_forceCursorRedraw = true;

        } else if (gemCmd == "down") {
            if (g_gem_autocomplete_active) {
                g_gem_autocomplete_selection++;
                if (g_gem_autocomplete_selection >= g_gem_autocomplete_count) g_gem_autocomplete_selection = 0;
            }
            else if (g_gemFocus == GEM_FOCUS_HEADER) {
                g_gemFocus = GEM_FOCUS_CREATE_TITLE;
                needsFullRedraw = true;
            }
            else if (focusedEditor && focusedEditor->cursorLine < focusedEditor->lineCount - 1) {
                tft.startWrite();
                focusedEditor->moveCursor(0, 1);
                tft.endWrite();
            } else {
                GemAppFocus oldFocus = g_gemFocus;

                if (g_gemFocus == GEM_FOCUS_CREATE_TITLE) { g_gemFocus = GEM_FOCUS_CREATE_DESC; }
                else if (g_gemFocus == GEM_FOCUS_CREATE_DESC) { g_gemFocus = GEM_FOCUS_CREATE_SKILLS;
                }
                else if (g_gemFocus == GEM_FOCUS_CREATE_SKILLS) { g_gemFocus = GEM_FOCUS_CREATE_DIFF;
                }
                else if (g_gemFocus == GEM_FOCUS_CREATE_DIFF) { g_gemFocus = GEM_FOCUS_CREATE_DESIRE;
                }
                else if (g_gemFocus == GEM_FOCUS_CREATE_DESIRE) { g_gemFocus = GEM_FOCUS_CREATE_TIME;
                } 
                else if (g_gemFocus == GEM_FOCUS_CREATE_TIME) { g_gemFocus = GEM_FOCUS_CREATE_MINS;
                }
                else if (g_gemFocus == GEM_FOCUS_CREATE_MINS) {
                    if (g_gem_editor_questType == QT_STANDARD) g_gemFocus = GEM_FOCUS_CREATE_REPEATABLE;
                    else g_gemFocus = GEM_FOCUS_CREATE_ADD_CYCLE;
                }
                else if (g_gemFocus == GEM_FOCUS_CREATE_REPEATABLE) { g_gemFocus = GEM_FOCUS_FOOTER;
                }
                else if (g_gemFocus == GEM_FOCUS_CREATE_ADD_CYCLE) { g_gemFocus = GEM_FOCUS_FOOTER;
                }

                if (oldFocus != g_gemFocus) {
                    needsFullRedraw = true;
                    if (g_gemFocus == GEM_FOCUS_FOOTER) {
                        needsFooterRedraw = true;
                        g_gem_footer_selection = 0; 
                    }
                    gem_hide_autocomplete(false);
                }
            }
            g_forceCursorRedraw = true;

        } else if (gemCmd == "left") {
            if (g_gemFocus == GEM_FOCUS_FOOTER) {
                g_gem_footer_selection--;
                if (g_gem_footer_selection < 0) g_gem_footer_selection = 0;
                tft.startWrite(); gem_drawFooter(true); tft.endWrite();
            } else if (g_gemFocus == GEM_FOCUS_CREATE_DESIRE) { 
                g_gemFocus = GEM_FOCUS_CREATE_DIFF;
                needsFullRedraw = true;
                gem_hide_autocomplete(false);
            } else if (g_gemFocus == GEM_FOCUS_CREATE_MINS) { 
                g_gemFocus = GEM_FOCUS_CREATE_TIME;
                needsFullRedraw = true;
                gem_hide_autocomplete(false);
            } else if (g_gemFocus == GEM_FOCUS_HEADER) {
                // Do nothing
            } else if (focusedEditor) {
                int numToRun = 1;
                if (count == 2 && isNumeric(tokens[1])) numToRun = tokens[1].toInt();
                if (numToRun == 0) numToRun = 1;
                for (int i = 0; i < numToRun; i++) focusedEditor->moveCursor(-1, 0);
            }
            g_forceCursorRedraw = true;

        } else if (gemCmd == "right") {
            if (g_gemFocus == GEM_FOCUS_FOOTER) {
                int max_selection = (g_gem_currentQuestID != -1) ?
                                    2 : 1;
                g_gem_footer_selection++;
                if (g_gem_footer_selection > max_selection) g_gem_footer_selection = max_selection;
                tft.startWrite(); gem_drawFooter(true); tft.endWrite();
            } else if (g_gemFocus == GEM_FOCUS_CREATE_DIFF) { 
                g_gemFocus = GEM_FOCUS_CREATE_DESIRE;
                needsFullRedraw = true;
                gem_hide_autocomplete(false);
            } else if (g_gemFocus == GEM_FOCUS_CREATE_TIME) {
                g_gemFocus = GEM_FOCUS_CREATE_MINS;
                needsFullRedraw = true;
                gem_hide_autocomplete(false);
            } else if (g_gemFocus == GEM_FOCUS_HEADER) {
                // Do nothing
            } else if (focusedEditor) {
                int numToRun = 1;
                if (count == 2 && isNumeric(tokens[1])) numToRun = tokens[1].toInt();
                if (numToRun == 0) numToRun = 1;
                for (int i = 0; i < numToRun; i++) focusedEditor->moveCursor(1, 0);
            }
            g_forceCursorRedraw = true;

        } else if (gemCmd == "enter") {
            if (g_gem_autocomplete_active) {
                gem_select_autocomplete(focusedEditor);
                needsFullRedraw = true;
            }
            else if (g_gemFocus == GEM_FOCUS_FOOTER) {
                String action;
                if (g_gem_footer_selection == 0) action = "Save";
                else if (g_gem_footer_selection == 1) action = "Back";
                else if (g_gem_footer_selection == 2) action = "Delete";

                if (action == "Save") {
                    String title = gemCreateTitleEditor->lines[0];
                    String desc = gemCreateDescEditor->lines[0]; 
                    String notes = gem_join_notes_from_editor(gemCreateDescEditor);
                    String skills = gem_join_notes_from_editor(gemCreateSkillsEditor);
                    skills.replace("\n", "");
                    int diff = gemCreateDiffEditor->lines[0].toInt();
                    int desire = gemCreateDesireEditor->lines[0].toInt();
                    
                    int timeEstHours = gemCreateTimeEditor->lines[0].toInt();
                    int timeEstMins_field = gemCreateMinsEditor->lines[0].toInt(); 
                    int totalTimeMins = (timeEstHours * 60) + timeEstMins_field;
                    int addCycleMins = 0;
                    
                    bool isRepeatable;
                    if (g_gem_editor_questType == QT_CRUSADE) {
                        isRepeatable = true;
                        addCycleMins = gemCreateAddCycleEditor->lines[0].toInt();
                    } else {
                        isRepeatable = g_gem_editor_isRepeatable;
                    }
                    
                    // Validation
                    if (title.length() == 0) { pushSystemMessage("Title cannot be empty.");
                    }
                    else if (diff < 0 || diff > 7) { pushSystemMessage("Difficulty must be 0-7.");
                    }
                    else if (desire < 0 || desire > 7) { pushSystemMessage("Desire must be 0-7.");
                    }
                    else if (timeEstHours < 0 || timeEstHours > 100) { pushSystemMessage("Hours must be 0-100.");
                    }
                    else if (timeEstMins_field < 0 || timeEstMins_field > 100) { pushSystemMessage("Mins must be 0-100.");
                    }
                    else if (totalTimeMins == 0) { pushSystemMessage("Total time cannot be zero.");
                    }
                    else if (g_gem_editor_questType == QT_CRUSADE && (addCycleMins < 0 || addCycleMins > 100)) { 
                        pushSystemMessage("Add Cycle must be 0-100.");
                    }
                    else {
                        gem_add_skills_to_database(skills);
                        if (g_gem_currentQuestID == -1) {
                            // --- FIX: Pass EMPTY string for icon (Serial creation doesn't support icons yet) ---
                            addQuest(title, desc, notes, skills, isRepeatable, diff, desire, totalTimeMins, g_gem_editor_questType, addCycleMins, ""); 
                            // --- END FIX ---
                            pushSystemMessage("New quest created!");
                        } else {
                            Quest& quest = g_questDB[g_gem_currentQuestID];
                            quest.title = title;
                            quest.description = desc;
                            quest.notes = notes; 
                            quest.skills = skills;
                            quest.isRepeatable = isRepeatable;
                            quest.difficulty = diff;
                            quest.desire = desire;
                            quest.timerDurationMinutes = (unsigned long)totalTimeMins; // Save TOTAL MINS
                            quest.type = g_gem_editor_questType;
                            quest.addCycleMinutes = addCycleMins;
                            
                            float timeHours = (float)totalTimeMins / 60.0f;
                            float base_reward = (diff * 10) + (timeHours * 2.0f);
                            float desire_multiplier = 1.0 + ( (7.0 - (float)desire) / 6.0 );
                            quest.gemReward = (int)(base_reward * desire_multiplier);
                            quest.xpReward = (quest.difficulty * 5) + (int)(desire_multiplier * 5) + (int)(timeHours * 2.0f);
                            
                            pushSystemMessage("Quest updated!");
                        }
                        
                        gem_save_database(g_gem_db_filename);
                        gem_start(); 
                        return;
                    }
                    needsFullRedraw = true;
                } else if (action == "Back") {
                    gem_start();
                    return;
                } else if (action == "Delete") {
                    gem_deleteQuest(g_gem_currentQuestID);
                    gem_start();
                    return;
                }
            }
            else if (g_gemFocus == GEM_FOCUS_CREATE_REPEATABLE) {
                g_gem_editor_isRepeatable = !g_gem_editor_isRepeatable;
                needsFullRedraw = true;
            }
            else if (g_gemFocus == GEM_FOCUS_CREATE_DESC || g_gemFocus == GEM_FOCUS_CREATE_SKILLS) {
                tft.startWrite();
                if (focusedEditor) focusedEditor->insertLine(tft);
                if (focusedEditor) focusedEditor->draw(tft);
                tft.endWrite();
                gem_hide_autocomplete(true); 
            } 
            else {
                if (g_gemFocus == GEM_FOCUS_CREATE_TITLE) { g_gemFocus = GEM_FOCUS_CREATE_DESC;
                }
                else if (g_gemFocus == GEM_FOCUS_CREATE_DIFF) { g_gemFocus = GEM_FOCUS_CREATE_DESIRE;
                }
                else if (g_gemFocus == GEM_FOCUS_CREATE_DESIRE) { g_gemFocus = GEM_FOCUS_CREATE_TIME;
                } 
                else if (g_gemFocus == GEM_FOCUS_CREATE_TIME) { g_gemFocus = GEM_FOCUS_CREATE_MINS;
                } 
                else if (g_gemFocus == GEM_FOCUS_CREATE_MINS) {
                    if (g_gem_editor_questType == QT_STANDARD) g_gemFocus = GEM_FOCUS_CREATE_REPEATABLE;
                    else g_gemFocus = GEM_FOCUS_CREATE_ADD_CYCLE;
                }
                else if (g_gemFocus == GEM_FOCUS_CREATE_REPEATABLE) { g_gemFocus = GEM_FOCUS_FOOTER;
                g_gem_footer_selection = 0; needsFooterRedraw = true; } 
                else if (g_gemFocus == GEM_FOCUS_CREATE_ADD_CYCLE) { g_gemFocus = GEM_FOCUS_FOOTER;
                g_gem_footer_selection = 0; needsFooterRedraw = true; }
                needsFullRedraw = true;
                gem_hide_autocomplete(false);
            }
            g_forceCursorRedraw = true;

        } else if (gemCmd == "backspace" || gemCmd == "bs") {
            gem_hide_autocomplete(false);
            int numToRun = 1;
            if (count == 2 && isNumeric(tokens[1])) numToRun = tokens[1].toInt();
            tft.startWrite();
            if (focusedEditor) for (int i = 0; i < numToRun; i++) {
                if (focusedEditor->backspace(tft)) needsFullRedraw = true;
            }
            if (!needsFullRedraw && focusedEditor) focusedEditor->draw(tft);
            tft.endWrite();
            g_forceCursorRedraw = true;
            if (g_gemFocus == GEM_FOCUS_CREATE_SKILLS) gem_update_autocomplete(focusedEditor); 

        } else if (gemCmd == "space" || gemCmd == "sp") {
            int numToRun = 1;
            if (count == 2 && isNumeric(tokens[1])) numToRun = tokens[1].toInt();
            tft.startWrite();
            if (focusedEditor && g_gemFocus != GEM_FOCUS_CREATE_DIFF &&
                g_gemFocus != GEM_FOCUS_CREATE_DESIRE && g_gemFocus != GEM_FOCUS_CREATE_TIME &&
                g_gemFocus != GEM_FOCUS_CREATE_MINS &&
                g_gemFocus != GEM_FOCUS_CREATE_ADD_CYCLE) {
                for (int i = 0; i < numToRun; i++) {
                    if (focusedEditor->insertChar(tft, ' ')) needsFullRedraw = true;
                }
            }
            if (!needsFullRedraw && focusedEditor) focusedEditor->draw(tft);
            tft.endWrite();
            g_forceCursorRedraw = true;
            gem_hide_autocomplete(true); 

        } else if (gemCmd == "tab") {
            if (g_gem_autocomplete_active) {
                gem_select_autocomplete(focusedEditor);
                needsFullRedraw = true;
            } else {
                int numToRun = 4;
                if (count == 2 && isNumeric(tokens[1])) numToRun = tokens[1].toInt();
                tft.startWrite();
                if (focusedEditor && g_gemFocus != GEM_FOCUS_CREATE_DIFF &&
                    g_gemFocus != GEM_FOCUS_CREATE_DESIRE && g_gemFocus != GEM_FOCUS_CREATE_TIME &&
                    g_gemFocus != GEM_FOCUS_CREATE_MINS &&
                    g_gemFocus != GEM_FOCUS_CREATE_ADD_CYCLE) {
                    for 
                    (int i = 0; i < numToRun; i++) {
                        if (focusedEditor->insertChar(tft, ' ')) needsFullRedraw = true;
                    }
                }
                if (!needsFullRedraw && focusedEditor) focusedEditor->draw(tft);
                tft.endWrite();
                gem_hide_autocomplete(true); 
            }
            g_forceCursorRedraw = true;
        } else if (gemCmd == "focus") {
             if (count > 1) {
                if (tokens[1] == "footer") g_gemFocus = GEM_FOCUS_FOOTER;
            }
            needsFullRedraw = true;
            g_forceCursorRedraw = true;
            gem_hide_autocomplete(false);
        } else if (gemCmd == "save") {
            gem_hide_autocomplete(false);
            if (g_gemState == GEM_STATE_CREATE_QUEST) {
                // --- 5. UPDATED SAVE LOGIC ---
                String title = gemCreateTitleEditor->lines[0];
                String desc = gemCreateDescEditor->lines[0]; 
                String notes = gem_join_notes_from_editor(gemCreateDescEditor);
                String skills = gem_join_notes_from_editor(gemCreateSkillsEditor);
                skills.replace("\n", "");
                int diff = gemCreateDiffEditor->lines[0].toInt();
                int desire = gemCreateDesireEditor->lines[0].toInt();
                
                int timeEstHours = gemCreateTimeEditor->lines[0].toInt();
                int timeEstMins_field = gemCreateMinsEditor->lines[0].toInt(); 
                int totalTimeMins = (timeEstHours * 60) + timeEstMins_field;
                int addCycleMins = 0;
                
                bool isRepeatable;
                if (g_gem_editor_questType == QT_CRUSADE) {
                    isRepeatable = true;
                    addCycleMins = gemCreateAddCycleEditor->lines[0].toInt();
                } else {
                    isRepeatable = g_gem_editor_isRepeatable;
                }
                
                // Validation
                if (title.length() == 0) { pushSystemMessage("Title cannot be empty.");
                }
                else if (diff < 0 || diff > 7) { pushSystemMessage("Difficulty must be 0-7.");
                }
                else if (desire < 0 || desire > 7) { pushSystemMessage("Desire must be 0-7.");
                }
                else if (timeEstHours < 0 || timeEstHours > 100) { pushSystemMessage("Hours must be 0-100.");
                }
                else if (timeEstMins_field < 0 || timeEstMins_field > 100) { pushSystemMessage("Mins must be 0-100.");
                }
                else if (totalTimeMins == 0) { pushSystemMessage("Total time cannot be zero.");
                }
                else if (g_gem_editor_questType == QT_CRUSADE && (addCycleMins < 0 || addCycleMins > 100)) { 
                    pushSystemMessage("Add Cycle must be 0-100.");
                }
                else {
                    gem_add_skills_to_database(skills);
                    if (g_gem_currentQuestID == -1) {
                        // --- FIX: Pass empty string for icon ---
                        addQuest(title, desc, notes, skills, isRepeatable, diff, desire, totalTimeMins, g_gem_editor_questType, addCycleMins, "");
                        // --- END FIX ---
                        pushSystemMessage("New quest created!");
                    } else {
                        Quest& quest = g_questDB[g_gem_currentQuestID];
                        quest.title = title;
                        quest.description = desc;
                        quest.notes = notes; 
                        quest.skills = skills;
                        quest.isRepeatable = isRepeatable;
                        quest.difficulty = diff;
                        quest.desire = desire;
                        quest.timerDurationMinutes = (unsigned long)totalTimeMins; // Save TOTAL MINS
                        quest.type = g_gem_editor_questType;
                        quest.addCycleMinutes = addCycleMins;
                        
                        float timeHours = (float)totalTimeMins / 60.0f;
                        float base_reward = (diff * 10) + (timeHours * 2.0f);
                        float desire_multiplier = 1.0 + ( (7.0 - (float)desire) / 6.0 );
                        quest.gemReward = (int)(base_reward * desire_multiplier);
                        quest.xpReward = (quest.difficulty * 5) + (int)(desire_multiplier * 5) + (int)(timeHours * 2.0f);
                        
                        pushSystemMessage("Quest updated!");
                    }
                    
                    gem_save_database(g_gem_db_filename);
                    gem_start(); 
                    return;
                }
            }
            needsFullRedraw = true;
            g_forceCursorRedraw = true;

        } else if (gemCmd == "back") {
            gem_hide_autocomplete(false);
            if (g_gemState == GEM_STATE_CREATE_QUEST) gem_start();
            return;
        
        } else {
            isCommand = false;
        }

        // --- 6. UPDATED TEXT INPUT LOGIC ---
        if (!isCommand) { // DEFAULT: Type the text
            tft.startWrite();
            if (g_gemFocus == GEM_FOCUS_CREATE_REPEATABLE) {
                // Do nothing
            }
            else if (focusedEditor) {
                // 1. Insert the text
                for (int i = 0; i < cmdLine.length(); i++) {
                     char c = cmdLine.charAt(i);
                     if (g_gemFocus == GEM_FOCUS_CREATE_DIFF || g_gemFocus == GEM_FOCUS_CREATE_DESIRE) {
                       if (c >= '0' && c <= '7' && focusedEditor->lines[0].length() == 0) { // Allow 0
                           if(focusedEditor->insertChar(tft, c)) needsFullRedraw = true;
                       }
                    } else if (g_gemFocus == GEM_FOCUS_CREATE_TIME) { // <-- TIME (HOURS)
                        if (isDigit(c)) {
                            String newNum = focusedEditor->lines[0] + c;
                            if (newNum.length() <= 3 && newNum.toInt() <= 100) { // 0-100 HOURS
                                if(focusedEditor->insertChar(tft, c)) needsFullRedraw = true;
                            }
                        }
                    } else if (g_gemFocus == GEM_FOCUS_CREATE_MINS) { // <-- NEW
                        if (isDigit(c)) {
                           String newNum = focusedEditor->lines[0] + c;
                           if (newNum.length() <= 3 && newNum.toInt() <= 100) { // 0-100 MINUTES
                                if(focusedEditor->insertChar(tft, c)) needsFullRedraw = true;
                           }
                        }
                    } else if (g_gemFocus == GEM_FOCUS_CREATE_ADD_CYCLE) {
                        if (isDigit(c)) {
                            String newNum = focusedEditor->lines[0] + c;
                            if (newNum.length() <= 3 && newNum.toInt() <= 100) { // 0-100 MINUTES
                                if(focusedEditor->insertChar(tft, c)) needsFullRedraw = true;
                            }
                        }
                    } else {
                        // Title, Desc, Skills
                        if(focusedEditor->insertChar(tft, c)) needsFullRedraw = true;
                    }
                }
                
                if (g_gemFocus == GEM_FOCUS_CREATE_SKILLS) {
                    gem_update_autocomplete(focusedEditor);
                }

                // Auto-advance on single-line fields
                if (g_gemFocus == GEM_FOCUS_CREATE_TITLE ||
                    g_gemFocus == GEM_FOCUS_CREATE_DIFF  ||
                    g_gemFocus == GEM_FOCUS_CREATE_DESIRE ||
                    g_gemFocus == GEM_FOCUS_CREATE_TIME ||
                    g_gemFocus == GEM_FOCUS_CREATE_MINS || // <-- NEW
                    g_gemFocus == GEM_FOCUS_CREATE_ADD_CYCLE) 
                {
                    switch (g_gemFocus) {
                        case GEM_FOCUS_CREATE_TITLE:    g_gemFocus = GEM_FOCUS_CREATE_DESC;
                        break;
                        case GEM_FOCUS_CREATE_DIFF:     g_gemFocus = GEM_FOCUS_CREATE_DESIRE; break;
                        case GEM_FOCUS_CREATE_DESIRE:   g_gemFocus = GEM_FOCUS_CREATE_TIME; break;
                        case GEM_FOCUS_CREATE_TIME:     g_gemFocus = GEM_FOCUS_CREATE_MINS; break;
                        // <-- CHANGED
                        case GEM_FOCUS_CREATE_MINS: // <-- NEW
                            if (g_gem_editor_questType == QT_STANDARD) g_gemFocus = GEM_FOCUS_CREATE_REPEATABLE;
                            else g_gemFocus = GEM_FOCUS_CREATE_ADD_CYCLE;
                            break;
                        case GEM_FOCUS_CREATE_REPEATABLE: 
                            g_gemFocus = GEM_FOCUS_FOOTER;
                            g_gem_footer_selection = 0; needsFooterRedraw = true;
                            break;
                        case GEM_FOCUS_CREATE_ADD_CYCLE: 
                            g_gemFocus = GEM_FOCUS_FOOTER;
                            g_gem_footer_selection = 0; needsFooterRedraw = true;
                            break;
                    }
                    needsFullRedraw = true;
                    // Force redraw for new focus
                } else {
                    // Multiline field (DESC or SKILLS)
                    if (!needsFullRedraw && focusedEditor) focusedEditor->draw(tft);
                }
            }
            
            tft.endWrite();
            g_forceCursorRedraw = true;
        }
        
        
        if (needsFullRedraw) {
            if (g_gemState == GEM_STATE_QUEST_LIST) {
                g_gem_marquee_offset = 0;
                gem_drawQuestList();
            }
            else if (g_gemState == GEM_STATE_TYPE_SELECT) { // <-- ADDED
                 gem_drawTypeSelectPage();
            }
            else if (g_gemState == GEM_STATE_CREATE_QUEST) {
                gem_refreshCreateQuestView();
                if (needsFooterRedraw) { 
                    tft.startWrite();
                    gem_drawFooter(true);
                    tft.endWrite();
                }
            }
        } else if (needsFooterRedraw) { 
            tft.startWrite();
            gem_drawFooter(true);
            tft.endWrite();
        }
    }
    // --- 7. STATS PAGE LOGIC ---
    else if (g_gemState == GEM_STATE_STATS) {
        
        const int visible_count = 6;
        int lastPossibleIndex = g_playerSkillCount - 1;

        if (gemCmd == "up") {
            if (g_gemFocus == GEM_FOCUS_FOOTER) {
                g_gemFocus = GEM_FOCUS_STATS_LIST;
                if (lastPossibleIndex < 0) lastPossibleIndex = 0;
                if (g_playerSkillCount <= visible_count) g_gem_stats_top_skill = 0;
                else g_gem_stats_top_skill = g_playerSkillCount - visible_count;
                g_gem_stats_selection = lastPossibleIndex; 
            } else if (g_gemFocus == GEM_FOCUS_STATS_LIST) {
                g_gem_stats_selection--;
                if (g_gem_stats_selection < 0) g_gem_stats_selection = 0; 
                else if (g_gem_stats_selection < g_gem_stats_top_skill) g_gem_stats_top_skill--;
            }
            tft.startWrite();
            gem_drawStatisticsPage(); 
            tft.endWrite();
            g_forceCursorRedraw = true;
        }
        else if (gemCmd == "back" || gemCmd == "exit") {
             gem_start();
             return;
        }
        else if (gemCmd == "down") {
            if (g_gemFocus == GEM_FOCUS_HEADER) {
                g_gemFocus = GEM_FOCUS_STATS_LIST;
                g_gem_stats_selection = 0; 
                g_gem_stats_top_skill = 0; 
            } else if (g_gemFocus == GEM_FOCUS_STATS_LIST) {
                g_gem_stats_selection++;
                if (g_gem_stats_selection > lastPossibleIndex) {
                    g_gem_stats_selection = lastPossibleIndex;
                    g_gemFocus = GEM_FOCUS_FOOTER;
                    g_gem_footer_selection = 0;
                } else {
                    int lastVisibleIndex = g_gem_stats_top_skill + visible_count - 1;
                    if (g_gem_stats_selection > lastVisibleIndex) {
                        g_gem_stats_top_skill++;
                    }
                }
            }
            tft.startWrite();
            gem_drawStatisticsPage(); 
            tft.endWrite();
            g_forceCursorRedraw = true;
        }
        else if (gemCmd == "left") {
            if (g_gemFocus == GEM_FOCUS_FOOTER && g_gem_footer_selection != 0) {
                g_gem_footer_selection = 0;
                tft.startWrite(); gem_drawFooter(true); tft.endWrite();
            }
            g_forceCursorRedraw = true;
        } 
        else if (gemCmd == "right") {
            if (g_gemFocus == GEM_FOCUS_FOOTER && g_gem_footer_selection != 1) {
                g_gem_footer_selection = 1;
                tft.startWrite(); gem_drawFooter(true); tft.endWrite();
            }
            g_forceCursorRedraw = true;
        } 
        else if (gemCmd == "enter") {
            if (g_gemFocus == GEM_FOCUS_FOOTER) {
                String action = (g_gem_footer_selection == 0) ?
                                "Settings" : "Exit";
                if (action == "Settings") {
                    pushSystemMessage("Settings page not yet implemented.");
                    gem_drawStatisticsPage();
                    g_forceCursorRedraw = true;
                } else { // "Exit"
                    gem_start();
                    return;
                }
            }
        }
    }
    // --- 8. FALLBACK ---
    else {
        if (g_gemState == GEM_STATE_QUEST_LIST) gem_drawQuestList();
    }
}
bool gem_load_database(String filename) {
    if (!fsReady) return false;

    File db = LittleFS.open(filename, "r");
    if (!db) {
        return false;
    }

    unsigned long now = millis();
    
    // Reset counts to ensure a clean load
    // (Optional safety step, though strictly the caller should handle resets)
    g_gem_count = 0;
    g_gem_level = 1;
    g_gem_total_xp = 0;
    g_skillCount = 0;
    g_playerSkillCount = 0;
    g_merchantItemCount = 0;
    g_inventoryItemCount = 0;
    g_questCount = 0;

    while (db.available()) {
        String line = db.readStringUntil('\n');
        line.trim();
        if (line.length() == 0) continue;
        
        String key = "";
        String value = "";
        int separator = line.indexOf(':');
        
        if (separator == -1) {
            continue;
        } else {
            key = line.substring(0, separator);
            value = line.substring(separator + 1);
        }

        // --- 1. Global Stats ---
        if (key == "inventory_gems") {
            g_gem_count = value.toInt();
        } else if (key == "inventory_level") {
            g_gem_level = value.toInt();
        } else if (key == "inventory_xp") {
            g_gem_total_xp = (unsigned long)value.toInt();
        }
        
        // --- 2. Global Skills (Autocomplete) ---
        else if (key == "skill" && g_skillCount < MAX_SKILLS) {
            g_skillDB[g_skillCount++] = value;
        }
        
        // --- 3. Player Skills ---
        else if (key == "pskill" && g_playerSkillCount < MAX_PLAYER_SKILLS) {
            int s_end = value.indexOf('|');
            int l_end = value.indexOf('|', s_end + 1);

            if (s_end != -1 && l_end != -1) {
                PlayerSkill& skill = g_playerSkills[g_playerSkillCount];
                skill.name = value.substring(0, s_end);
                skill.level = value.substring(s_end + 1, l_end).toInt();
                skill.xp = (unsigned long)value.substring(l_end + 1).toInt();
                g_playerSkillCount++;
            }
        }
        
        // --- 4. Merchant Items ---
        else if (key == "item" && g_merchantItemCount < MAX_MERCHANT_ITEMS) {
            MerchantItem& item = g_merchantDB[g_merchantItemCount];
            int t_end = value.indexOf('|');
            int d_end = value.indexOf('|', t_end + 1);
            
            if (t_end != -1 && d_end != -1) {
                item.title = value.substring(0, t_end);
                item.description = gem_unescape_notes(value.substring(t_end + 1, d_end));
                
                // Check for price|icon separator
                int p_end = value.indexOf('|', d_end + 1);
                
                if (p_end != -1) {
                    // Format: ...|price|iconFile
                    item.price = value.substring(d_end + 1, p_end).toInt();
                    item.iconFile = value.substring(p_end + 1);
                } else {
                    // Legacy Format: ...|price
                    item.price = value.substring(d_end + 1).toInt();
                    item.iconFile = "";
                }
                g_merchantItemCount++;
            }
        }
        
        // --- 5. Inventory Items ---
        else if (key == "inv_item" && g_inventoryItemCount < MAX_INVENTORY_ITEMS) {
            MerchantItem& item = g_inventoryDB[g_inventoryItemCount];
            int t_end = value.indexOf('|');
            int d_end = value.indexOf('|', t_end + 1);
            
            if (t_end != -1 && d_end != -1) {
                item.title = value.substring(0, t_end);
                item.description = gem_unescape_notes(value.substring(t_end + 1, d_end));
                
                // Check for price|icon separator
                int p_end = value.indexOf('|', d_end + 1);
                
                if (p_end != -1) {
                    item.price = value.substring(d_end + 1, p_end).toInt();
                    item.iconFile = value.substring(p_end + 1);
                } else {
                    item.price = value.substring(d_end + 1).toInt();
                    item.iconFile = "";
                }
                g_inventoryItemCount++;
            }
        }
        
        // --- 6. QUESTS (The Fix is Here) ---
        else if (key == "quest" && g_questCount < MAX_QUESTS) {
            Quest& q = g_questDB[g_questCount];
            
            // Calculate all pipe positions
            int t_end = value.indexOf('|');
            int d_end = value.indexOf('|', t_end + 1);
            int f_end = value.indexOf('|', d_end + 1);
            int c_end = value.indexOf('|', f_end + 1);
            int ds_end = value.indexOf('|', c_end + 1);
            int te_end = value.indexOf('|', ds_end + 1);
            int tp_end = value.indexOf('|', te_end + 1);
            int b_end = value.indexOf('|', tp_end + 1);
            int s_end = value.indexOf('|', b_end + 1);
            int r_end = value.indexOf('|', s_end + 1);
            
            // New fields (Version 2+)
            int ty_end = value.indexOf('|', r_end + 1);
            int ac_end = value.indexOf('|', ty_end + 1);
            int rem_end = value.indexOf('|', ac_end + 1);
            int ic_end = value.indexOf('|', rem_end + 1); // <--- Icon End Index
            
            // Basic integrity check
            if (t_end == -1 || d_end == -1 || f_end == -1 || c_end == -1 || 
                ds_end == -1 || te_end == -1 || tp_end == -1 || b_end == -1) {
                continue;
            }

            // Load Basic Fields
            q.title = value.substring(0, t_end);
            q.description = value.substring(t_end + 1, d_end);
            q.difficulty = value.substring(d_end + 1, f_end).toInt();
            q.questState = (QuestState)value.substring(f_end + 1, c_end).toInt();
            q.desire = value.substring(c_end + 1, ds_end).toInt();
            q.timerMinutesElapsed = value.substring(te_end + 1, tp_end).toInt();
            q.gemBonusAwarded = value.substring(tp_end + 1, b_end).toInt();
            
            // --- Determine Format Version ---

            if (ty_end != -1 && ac_end != -1 && rem_end != -1) {
                // === MODERN FORMAT (Has Type, AddCycle, Remainder) ===
                q.timerDurationMinutes = (unsigned long)value.substring(ds_end + 1, te_end).toInt();
                q.skills = gem_unescape_notes(value.substring(b_end + 1, s_end));
                q.isRepeatable = value.substring(s_end + 1, r_end).toInt() == 1;
                q.type = (QuestType)value.substring(r_end + 1, ty_end).toInt();
                q.addCycleMinutes = value.substring(ty_end + 1, ac_end).toInt();
                q.timerRemainderMs = (unsigned long)value.substring(ac_end + 1, rem_end).toInt();
                
                // --- THE FIX: Check for Icon Separator ---
                if (ic_end != -1) {
                    // Format: ...|remainder|icon|notes
                    q.iconFile = value.substring(rem_end + 1, ic_end);
                    q.notes = gem_unescape_notes(value.substring(ic_end + 1));
                } else {
                    // Format: ...|remainder|notes (Older version)
                    q.iconFile = "";
                    q.notes = gem_unescape_notes(value.substring(rem_end + 1));
                }
                // -----------------------------------------
                
            } 
            else if (ty_end != -1 && ac_end != -1) {
                // === INTERMEDIATE FORMAT (Has Type, AddCycle, Notes) ===
                q.timerDurationMinutes = (unsigned long)value.substring(ds_end + 1, te_end).toInt();
                q.skills = gem_unescape_notes(value.substring(b_end + 1, s_end));
                q.isRepeatable = value.substring(s_end + 1, r_end).toInt() == 1;
                q.type = (QuestType)value.substring(r_end + 1, ty_end).toInt();
                q.addCycleMinutes = value.substring(ty_end + 1, ac_end).toInt();
                
                // The rest is notes (no remainder, no icon in this version)
                q.notes = gem_unescape_notes(value.substring(ac_end + 1));
                q.timerRemainderMs = 0;
                q.iconFile = "";
            } 
            else {
                // === LEGACY FORMAT (Base) ===
                // Handling old save files safely
                int timeEstHours = value.substring(ds_end + 1, te_end).toInt();
                q.timerDurationMinutes = (unsigned long)timeEstHours * 60;
                q.type = QT_STANDARD;
                q.addCycleMinutes = 0;
                q.timerRemainderMs = 0;
                q.iconFile = "";

                if (s_end == -1) { 
                    q.skills = "";
                    q.isRepeatable = false;
                    q.notes = gem_unescape_notes(value.substring(b_end + 1));
                } else if (r_end == -1) {
                    q.skills = gem_unescape_notes(value.substring(b_end + 1, s_end));
                    q.notes = gem_unescape_notes(value.substring(s_end + 1));
                    q.isRepeatable = false;
                } else {
                    q.skills = gem_unescape_notes(value.substring(b_end + 1, s_end));
                    q.isRepeatable = value.substring(s_end + 1, r_end).toInt() == 1;
                    q.notes = gem_unescape_notes(value.substring(r_end + 1));
                }
            }
            
            // --- Recalculate Rewards (in case balance formula changed) ---
            float timeHours = (float)q.timerDurationMinutes / 60.0f;
            float base_reward = (q.difficulty * 10) + (timeHours * 2.0f);
            float desire_multiplier = 1.0 + ( (7.0 - (float)q.desire) / 6.0 );
            q.gemReward = (int)(base_reward * desire_multiplier);
            q.xpReward = (q.difficulty * 5) + (int)(desire_multiplier * 5) + (int)(timeHours * 2.0f);

            // --- Restore Timer State ---
            if (q.questState == STATE_STARTED) {
                q.timerStartTime = now - q.timerRemainderMs;
            } else {
                q.timerStartTime = 0;
            }
            
            g_questCount++;
        }
    }
    
    db.close();
    
    // --- Post-Load Logic ---
    // Re-populate global skills from the loaded quests
    for (int i = 0; i < g_questCount; i++) {
        gem_add_skills_to_database(g_questDB[i].skills);
    }
    
    gem_sortQuests();
    gem_recalculateLevel();
    
    return true;
}
void gem_save_database(String filename) {
    if (!fsReady) return;
    gem_update_all_timers();
    
    File db = LittleFS.open(filename, "w");
    if (!db) {
        pushSystemMessage("FATAL: Can't save " + filename + "!");
        return;
    }
    
    // --- 1. Write the inventory ---
    db.println("inventory_gems:" + String(g_gem_count));
    db.println("inventory_level:" + String(g_gem_level));
    db.println("inventory_xp:" + String(g_gem_total_xp));

    // --- 2. Write the Global Skill DB ---
    for (int i = 0; i < g_skillCount; i++) {
        db.println("skill:" + g_skillDB[i]);
    }

    // --- 3. Write the Player Skill DB ---
    for (int i = 0; i < g_playerSkillCount; i++) {
        PlayerSkill& skill = g_playerSkills[i];
        db.println("pskill:" + skill.name + "|" + String(skill.level) + "|" + String(skill.xp));
    }

    unsigned long now_for_save = millis();
    for (int i = 0; i < g_questCount; i++) {
        Quest& q = g_questDB[i];
        String notes_str = gem_escape_notes(q.notes);
        String skills_str = gem_escape_notes(q.skills);
        
        unsigned long remainderMs = 0;
        if (q.questState == STATE_STARTED && q.timerStartTime != 0) {
            remainderMs = now_for_save - q.timerStartTime;
            if (remainderMs >= 60000) remainderMs = 0;
        }

        String quest_data = q.title + "|"
                            + q.description + "|"
                            + String(q.difficulty) + "|"
                            + String((int)q.questState) + "|"
                            + String(q.desire) + "|"
                            + String(q.timerDurationMinutes) + "|"
                            + String(q.timerMinutesElapsed) + "|"
                            + String(q.gemBonusAwarded) + "|"
                            + skills_str + "|"
                            + String(q.isRepeatable ? 1 : 0) + "|"
                            + String((int)q.type) + "|"
                            + String(q.addCycleMinutes) + "|"
                            + String(remainderMs) + "|"
                            + q.iconFile + "|"
                            + notes_str;
        db.println("quest:" + quest_data);
    }

    // --- 4. Write Merchant Items (Updated with iconFile) ---
    for (int i = 0; i < g_merchantItemCount; i++) {
        MerchantItem& item = g_merchantDB[i];
        String desc_str = gem_escape_notes(item.description);
        String item_data = item.title + "|"
                         + desc_str + "|"
                         + String(item.price) + "|"
                         + item.iconFile; // Add icon
        db.println("item:" + item_data);
    }

    // --- 5. Write Inventory Items (Updated with iconFile) ---
    for (int i = 0; i < g_inventoryItemCount; i++) {
        MerchantItem& item = g_inventoryDB[i];
        String desc_str = gem_escape_notes(item.description);
        String item_data = item.title + "|"
                         + desc_str + "|"
                         + String(item.price) + "|"
                         + item.iconFile; // Add icon
        db.println("inv_item:" + item_data);
    }
    db.close();
}
void drawItemBitmap(String filename, int x, int y, int maxW, int maxH) {
    if (!fsReady || filename.length() == 0) return;
    
    // Auto-append extension if missing
    if (!filename.endsWith(".bmp")) filename += ".bmp";
    
    // Case-insensitive lookup
    filename = findFileCaseInsensitive(filename);
    if (!LittleFS.exists(filename)) return;

    File bmpFile = LittleFS.open(filename, "r");
    if (!bmpFile) return;

    if (read16(bmpFile) != 0x4D42) { bmpFile.close(); return; } // Not BMP
    read32(bmpFile); // filesize
    read32(bmpFile); // reserved
    uint32_t bmpImageoffset = read32(bmpFile);
    read32(bmpFile); // header size
    int32_t bmpWidth = read32(bmpFile);
    int32_t bmpHeight = read32(bmpFile);
    
    if (read16(bmpFile) != 1) { bmpFile.close(); return; } // planes
    if (read16(bmpFile) != 24) { bmpFile.close(); return; } // bit depth
    if (read32(bmpFile) != 0) { bmpFile.close(); return; } // compression

    // --- NEW SCALING LOGIC ---
    // Calculate integer scale factor to fit within max dimensions
    int scaleW = maxW / bmpWidth;
    int scaleH = maxH / bmpHeight;
    int scale = min(scaleW, scaleH); // Maintain aspect ratio
    if (scale < 1) scale = 1;

    // Calculate centering offsets
    int actualDrawW = bmpWidth * scale;
    int actualDrawH = bmpHeight * scale;
    int offsetX = (maxW - actualDrawW) / 2;
    int offsetY = (maxH - actualDrawH) / 2;

    // Calculate row padding (BMP rows are padded to 4 bytes)
    uint32_t rowSize = (bmpWidth * 3 + 3) & ~3;
    uint8_t rowBuffer[bmpWidth * 3]; 

    // Iterate through the Source Image height
    for (int row = 0; row < bmpHeight; row++) {
        // BMPs are stored bottom-up
        int bmpRow = (bmpHeight - 1) - row;
        
        bmpFile.seek(bmpImageoffset + (bmpRow * rowSize));
        bmpFile.read(rowBuffer, bmpWidth * 3);

        for (int col = 0; col < bmpWidth; col++) {
            int bufIdx = col * 3;
            uint8_t b = rowBuffer[bufIdx];
            uint8_t g = rowBuffer[bufIdx+1];
            uint8_t r = rowBuffer[bufIdx+2];
            
            // Transparent color check (Magic Pink: 255, 0, 255)
            // Note: We treat anything very close to magic pink as transparent
            bool isTransparent = (r > 250 && g < 5 && b > 250);

            if (!isTransparent) {
                uint16_t color = tft.color565(r, g, b);
                
                // DRAW SCALED PIXEL (Rectangle)
                // We apply the offsets to center it in the box
                tft.fillRect(
                    x + offsetX + (col * scale), 
                    y + offsetY + (row * scale), 
                    scale, 
                    scale, 
                    color
                );
            }
        }
    }
    bmpFile.close();
}
void gem_buildBMPList() {
    g_gem_file_count = 0;
    g_gem_file_selection = 0;
    g_gem_file_top_index = 0;

    if (!fsReady) return;
    File root = LittleFS.open("/", "r");
    if (!root) return;

    File file = root.openNextFile();
    while (file && g_gem_file_count < SCROLLBACK_SIZE) {
        String fname = file.name();
        // Filter for BMPs
        if (fname.endsWith(".bmp")) {
            g_gem_file_list[g_gem_file_count++] = fname;
        }
        file.close();
        file = root.openNextFile();
    }
    root.close();
}
void gem_drawImageSelect() {
    tft.startWrite();
    tft.fillScreen(ST77XX_DARKGREY);
    
    // Header
    tft.fillRect(0, 0, SCREEN_WIDTH, LINE_HEIGHT + 2, ST77XX_MAGENTA);
    tft.setTextColor(ST77XX_BLACK, ST77XX_MAGENTA);
    tft.setCursor(5, 2);
    tft.print("SELECT ICON");

    if (g_gem_file_count == 0) {
        tft.setTextColor(ST77XX_WHITE, ST77XX_DARKGREY);
        tft.setCursor(10, 30);
        tft.print("No .bmp files found.");
        tft.endWrite();
        return;
    }

    int startY = LINE_HEIGHT + 6;
    int colWidth = SCREEN_WIDTH / 2;
    int itemsPerCol = (SCREEN_HEIGHT - startY - LINE_HEIGHT) / LINE_HEIGHT;
    int maxPerPage = itemsPerCol * 2; // 2 Columns

    int startIndex = (g_gem_file_top_index / maxPerPage) * maxPerPage;

    for (int i = 0; i < maxPerPage; i++) {
        int fileIdx = startIndex + i;
        if (fileIdx >= g_gem_file_count) break;

        int col = (i >= itemsPerCol) ? 1 : 0;
        int row = i % itemsPerCol;
        
        int xPos = (col * colWidth) + 5;
        int yPos = startY + (row * LINE_HEIGHT);

        bool isSelected = (fileIdx == g_gem_file_selection);
        uint16_t bg = isSelected ? ST77XX_CYAN : ST77XX_DARKGREY;
        uint16_t fg = isSelected ? ST77XX_BLACK : ST77XX_WHITE;

        tft.fillRect(xPos - 2, yPos, colWidth - 4, LINE_HEIGHT, bg);
        tft.setTextColor(fg, bg);
        tft.setCursor(xPos, yPos);
        
        // Strip extension for display to save space
        String dispName = g_gem_file_list[fileIdx];
        dispName.replace(".bmp", ""); 
        if(dispName.length() > 12) dispName = dispName.substring(0,12);
        
        tft.print(dispName);
    }
    
    // Footer
    tft.fillRect(0, SCREEN_HEIGHT - LINE_HEIGHT, SCREEN_WIDTH, LINE_HEIGHT, ST77XX_MAGENTA);
    tft.setTextColor(ST77XX_BLACK, ST77XX_MAGENTA);
    tft.setCursor(5, SCREEN_HEIGHT - LINE_HEIGHT + 1);
    tft.print("ENTER: Select   ESC: Cancel");
    
    tft.endWrite();
}
void gem_update_all_timers() {
    unsigned long now = millis();
    for (int i = 0; i < g_questCount; i++) {
        Quest& quest = g_questDB[i];
        
        // --- CHANGED ---
        // Only update quests that are STARTED
        if (quest.questState == STATE_STARTED) {
            // Get elapsed milliseconds since last update
            unsigned long elapsedMs = now - quest.timerStartTime;
            
            // Add any *whole* minutes to the progress
            quest.timerMinutesElapsed += (elapsedMs / 60000); 
            
            // Reset the start time, but keep the sub-minute remainder
            quest.timerStartTime = now - (elapsedMs % 60000); 
            
            // Clamp progress to the max duration
            if (quest.timerMinutesElapsed > quest.timerDurationMinutes) {
                quest.timerMinutesElapsed = quest.timerDurationMinutes;
            }
        }
    }
}
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
void run_cli_update(unsigned long now) {
    // --- Timer Finished Check ---
    if (timerEndTime > 0 && now >= timerEndTime) {
        timerEndTime = 0;
        lastTimerActiveBlink = 0;
        pushSystemMessage(">>> Timer Finished! <<<");
        timerFinishedBlinkEndTime = now + 10000;
        lastFinishedBlinkToggle = now;
        digitalWrite(STATUS_LED_PIN, HIGH);
        
        // --- FIX: Wrap draw call ---
        tft.startWrite();
        drawFullTerminal(); // Call raw function
        drawCursorAndPreview(); // We need to show the cursor after redraw
        tft.endWrite();
    }

    // --- Timer Finished Blinking Pattern Handler ---
    if (timerFinishedBlinkEndTime > 0) {
        if (now >= timerFinishedBlinkEndTime) {
            timerFinishedBlinkEndTime = 0;
            digitalWrite(STATUS_LED_PIN, LOW);
        } else {
            if (now - lastFinishedBlinkToggle >= FINISHED_BLINK_INTERVAL_MS) {
                digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN));
                lastFinishedBlinkToggle = now;
            }
        }
    }

    // --- Periodic Blink While Timer Active ---
    if (timerEndTime > 0) {
        if (now - lastTimerActiveBlink >= TIMER_ACTIVE_BLINK_INTERVAL_MS) {
            lastTimerActiveBlink = now;
            ledBlinkEndTime = now + LED_BLINK_DURATION_MS;
            digitalWrite(STATUS_LED_PIN, HIGH);
        }
    }

    // --- Cursor Blinking and Redraw Logic ---
    
    bool blink_fired = false; // <-- NEW: Declare the variable here
    
    if (now - lastBlink >= BLINK_MS) {
        lastBlink = now;
        cursorVisible = !cursorVisible;
        blink_fired = true; // <-- NEW: Set it to true when the blink timer fires
    }

    // --- THE FIX ---
    // This 'if' statement now works because blink_fired exists
    if (g_cli_needs_redraw || blink_fired) { // <-- NEW: Check both flags
        tft.startWrite(); 
        drawCursorAndPreview(); // Call the "raw" cursor drawer
        tft.endWrite();
        
        g_cli_needs_redraw = false; // <-- NEW: Reset the dirty flag
    }

}
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
            if (g_headless_mode) {
                Serial.println(line);
            }

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
void pushSystemMessage(const String &s) {
    pushScrollback(SYS_PROMPT + s, ST77XX_GREEN);  // <-- CRITICAL: Set color to GREEN
}
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
void drawFullTerminal() {
    // 1. Calculate all visual line segments
    String input = String(cmdBuf).substring(0, cmdLen);
    const int MAX_CHUNKS = 16;
    String fwdSegments[MAX_CHUNKS];
    int fwdCount = 0;
    calculateFullWrapSegments(input, fwdSegments, fwdCount, MAX_CHUNKS, false);
    
    if (fwdCount == 0) fwdCount = 1;

    // 2. Determine screen layout
    const int INPUT_LINES_TO_DRAW = min(fwdCount, MAX_LINES);
    int availableOutputRows = MAX_LINES - INPUT_LINES_TO_DRAW; 
    
    // 3. Draw the two main areas (these are also raw)
    drawScrollbackArea(availableOutputRows);
    drawInputArea();
    
    // 4. We NO LONGER draw the cursor here.
    // The "controller" function is responsible for that.
}
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
    
    // --- 1. Calculate state (NO DRAWING) ---
    const int MAX_CHUNKS = 16;
    String preSegments[MAX_CHUNKS];
    int preCount = 0;
    calculateFullWrapSegments(String(cmdBuf).substring(0, cmdLen), preSegments, preCount, MAX_CHUNKS, false);
    if (preCount == 0) preCount = 1;
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
    cursorPos++;
    cmdLen++;
    cmdBuf[cmdLen] = 0;

    // --- 3. Calculate state (NO DRAWING) ---
    String postSegments[MAX_CHUNKS];
    int postCount = 0;
    calculateFullWrapSegments(String(cmdBuf).substring(0, cmdLen), postSegments, postCount, MAX_CHUNKS, false);
    if (postCount == 0) postCount = 1;
    int postCursorRow = 0;
    int tempPostPos = cursorPos;
    if (tempPostPos >= LINE_1_CAPACITY) {
        tempPostPos -= LINE_1_CAPACITY;
        postCursorRow = 1 + (tempPostPos / LINE_N_CAPACITY);
    }

    // --- 4. START TRANSACTION ---
    tft.startWrite();
    
    // --- 5. Conditional Redraw Logic ---
    if (postCount > preCount || postCursorRow > preCursorRow) {
        // Full redraw
        bool oldVisibility = cursorVisible;
        cursorVisible = false;
        drawCursorAndPreview(); // Raw
        cursorVisible = oldVisibility;
        
        drawFullTerminal();     // Raw
        drawCursorAndPreview(); // Raw
    } else {
        // Flicker-free redraw
        redrawTrailingText();   // Raw (this already calls drawCursorAndPreview)
    }
    
    // --- 6. END TRANSACTION ---
    tft.endWrite();

    f1_copy_index = 0;
}
void insertStringAtCursor(const String& s) {
    for(int i = 0; i < s.length(); ++i) {
        insertCharAtCursor(s.charAt(i));
    }
}
void backspaceAtCursor() {
    if (cursorPos == 0 || cmdLen == 0) return;

    // --- 1. Get pre-state (NO DRAWING) ---
    const int MAX_CHUNKS = 16;
    String preSegments[MAX_CHUNKS];
    int preCount = 0;
    calculateFullWrapSegments(String(cmdBuf).substring(0, cmdLen), preSegments, preCount, MAX_CHUNKS, false);
    
    // --- 2. START TRANSACTION & Hide cursor ---
    tft.startWrite();
    bool oldVisibility = cursorVisible;
    cursorVisible = false;
    drawCursorAndPreview(); // Raw
    cursorVisible = oldVisibility;

    // --- 3. Perform the deletion in memory ---
    for (int i = cursorPos - 1; i < cmdLen - 1; ++i) {
        cmdBuf[i] = cmdBuf[i + 1];
    }
    cmdLen--;
    cursorPos--;
    cmdBuf[cmdLen] = 0;

    // --- 4. Get post-state (NO DRAWING) ---
    String postSegments[MAX_CHUNKS];
    int postCount = 0;
    calculateFullWrapSegments(String(cmdBuf).substring(0, cmdLen), postSegments, postCount, MAX_CHUNKS, false);
    if (cmdLen > 0 && postCount == 0) postCount = 1;
    bool didUnwrap = (postCount < preCount);

    // --- 5. HYBRID Conditional Redraw Logic ---
    if (didUnwrap) {
        drawFullTerminal(); // Raw
        
        // --- 6. Surgical prompt fix ---
        const int LINE_1_CAPACITY = COLS - PROMPT.length();
        if (cursorPos < LINE_1_CAPACITY) {
            int linesToDrawAfter = min(postCount > 0 ? postCount : 1, MAX_LINES);
            int firstInputLineY = (MAX_LINES - linesToDrawAfter) * LINE_HEIGHT;

            tft.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
            tft.setCursor(0, firstInputLineY);
            tft.print(PROMPT); 
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
        redrawTrailingText(); // Raw (which calls drawCursorAndPreview)
    }

    // --- 7. Restore cursor visibility ---
    cursorVisible = oldVisibility;
    drawCursorAndPreview(); // Raw
    
    // --- 8. TARGETED PATCH ---
    if (didUnwrap) {
        const int LINE_1_CAPACITY = COLS - PROMPT.length();
        if (cursorPos >= LINE_1_CAPACITY) {
            String segments[16];
            int segmentCount = 0;
            calculateFullWrapSegments(String(cmdBuf).substring(0, cmdLen), segments, segmentCount, 16, false);
            if (segmentCount == 0) segmentCount = 1;
            int linesToDraw = min(segmentCount, MAX_LINES);
            int startRow = MAX_LINES - linesToDraw;
            int tempCursorPos = cursorPos;
            int currentSegmentIndex = -1;
            for (int i = 0; i < segmentCount; ++i) {
                 if (tempCursorPos <= segments[i].length()) {
                     currentSegmentIndex = i;
                     break;
                 }
                 tempCursorPos -= segments[i].length();
            }
            if (currentSegmentIndex > 0) {
                int currentScreenY = (startRow + (currentSegmentIndex - (segmentCount - linesToDraw))) * LINE_HEIGHT;
                String lineText = segments[currentSegmentIndex];
                int charsToPatch = PROMPT.length() + 1;
                String patchText = lineText.substring(0, min(charsToPatch, lineText.length()));
                tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
                tft.setCursor(0, currentScreenY);
                tft.print(patchText);
            }
        }
    }
    
    // --- 9. END TRANSACTION ---
    tft.endWrite();
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
void kbPrev() {
    if (fkeyState == F_AWAIT_FORMAT_CONFIRM) {
        // Use the dedicated formatIndex instead of kbIndex
        formatIndex--;
        // Constrain the index to wrap back to 1 ('N') if it falls below 0 ('Y')
        if (formatIndex < 0) { 
            formatIndex = 1; 
        }
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
}
void kbConfirm() {
    char charToInsert = 0;
    String controlAction = "";
    // --- 1. Handle F-KEY Awaited Input ---
    if (fkeyState != F_INACTIVE) {
        char inputChar = 0;
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
            if (fkeyState == F7_AWAIT_INDEX) {
                 insertCharAtCursor(inputChar);
            } else {
                 insertCharAtCursor(inputChar);
                 handleFKeyInput(inputChar);
            }
            return;
        } 
    }

    // --- 2. Handle Format Confirmation ---
    if (fkeyState == F_AWAIT_FORMAT_CONFIRM) {
        int confirmedIndex = formatIndex;
        executeFormatConfirmation(confirmedIndex == 0);
        return;
    }

    // --- 3. Mode Switch Logic ---
    if (kbIndex == 0) {
        if (fkeyState != F_INACTIVE && kmode == FUNC_VIEW) {
        } else if (kmode == FUNC_VIEW) {
            kmode = CTRL;
        } else {
            if (kmode == ALPHA || kmode == ALPHA_LOWER) { kmode = NUM; }
            else if (kmode == NUM) { kmode = SYM; }
            else if (kmode == SYM) { kmode = CTRL; }
            else if (kmode == CTRL) { kmode = ALPHA; }
        }
        f1_copy_index = 0;
        kbIndex = 0;
        
        tft.startWrite();
        drawCursorAndPreview();
        tft.endWrite();
        return;
    }

    // --- 4. Determine Action ---
    const int ALPHA_CASE_KEY_INDEX = (int)strlen(alphaChars) + 1;
    const int NUM_KEY_INDEX_SPACE = (int)strlen(numberChars) + 1; 
    const int SYM_KEY_INDEX_SPACE = (int)strlen(symbolChars) + 1;
    if (kmode == ALPHA) {
        if (kbIndex <= (int)strlen(alphaChars)) { charToInsert = alphaChars[kbIndex - 1]; } 
        else if (kbIndex == ALPHA_CASE_KEY_INDEX) { controlAction = "SPACE"; }
        else if (kbIndex == ALPHA_CASE_KEY_INDEX + 1) { controlAction = "ENTER"; }
        else if (kbIndex == ALPHA_CASE_KEY_INDEX + 2) { 
            kmode = ALPHA_LOWER;
            kbIndex = ALPHA_CASE_KEY_INDEX + 2; 
            tft.startWrite(); drawCursorAndPreview(); tft.endWrite(); 
            return;
        }
    }
    else if (kmode == ALPHA_LOWER) {
        if (kbIndex <= (int)strlen(alphaLowerChars)) { charToInsert = alphaLowerChars[kbIndex - 1]; }
        else if (kbIndex == ALPHA_CASE_KEY_INDEX) { controlAction = "SPACE"; }
        else if (kbIndex == ALPHA_CASE_KEY_INDEX + 1) { controlAction = "ENTER"; }
        else if (kbIndex == ALPHA_CASE_KEY_INDEX + 2) {
            kmode = ALPHA;
            kbIndex = ALPHA_CASE_KEY_INDEX + 2;
            tft.startWrite(); drawCursorAndPreview(); tft.endWrite(); 
            return;
        }
    }
    else if (kmode == NUM) {
        if (kbIndex <= (int)strlen(numberChars)) { charToInsert = numberChars[kbIndex - 1]; } 
        else if (kbIndex == NUM_KEY_INDEX_SPACE) { controlAction = "SPACE"; } 
        else if (kbIndex == NUM_KEY_INDEX_SPACE + 1) { controlAction = "ENTER"; } 
    }
    else if (kmode == SYM) {
        if (kbIndex <= (int)strlen(symbolChars)) { charToInsert = symbolChars[kbIndex - 1]; } 
        else if (kbIndex == SYM_KEY_INDEX_SPACE) { controlAction = "SPACE"; } 
        else if (kbIndex == SYM_KEY_INDEX_SPACE + 1) { controlAction = "ENTER"; }
    }
    else if (kmode == CTRL) {
        if (kbIndex <= CTRL_COUNT) { controlAction = ctrlKeys[kbIndex - 1]; } 
        else if (kbIndex == CTRL_COUNT + 1) { 
            kmode = FUNC_VIEW;
            f1_copy_index = 0; 
            kbIndex = 0;
            tft.startWrite(); drawCursorAndPreview(); tft.endWrite(); 
            return;
        }
    }
    else if (kmode == FUNC_VIEW) {
        if (kbIndex <= FUNC_COUNT) {
            handleFKeyAction(kbIndex);
            if (kbIndex != 1 && fkeyState != F7_AWAIT_INDEX && fkeyState != F9_AWAIT_INDEX) {
                kmode = CTRL;
            }
            kbIndex = 0;
            tft.startWrite();
            drawCursorAndPreview();
            tft.endWrite();
            return;
        }
    }

    // --- 5. Execute Action ---
    if (charToInsert != 0) {
        insertCharAtCursor(charToInsert);
        f1_copy_index = 0;
    } 
    else if (controlAction.length() > 0) {
        if (controlAction == "SPACE") {
            insertCharAtCursor(' ');
            f1_copy_index = 0;
        } 
        
        else if (controlAction == "ENTER" && fkeyState == F7_AWAIT_INDEX) {
            String fullInput = String(cmdBuf).substring(0, cmdLen);
            int lastSpace = fullInput.lastIndexOf(' ');
            String indexInput = fullInput.substring(lastSpace + 1);
            indexInput.trim(); 
            int index = -1;
            if (indexInput.length() > 0) { index = indexInput.toInt(); } 
            
            if (index >= 0 && index < historyCount) {
                String commandToInsert = history[index % HISTORY_SIZE];
                int deleteCount = indexInput.length() + 1; 
                if (lastSpace == -1) { deleteCount = indexInput.length(); }
                
                for (int i = 0; i < deleteCount; ++i) { backspaceAtCursor(); } 
                insertStringAtCursor(commandToInsert);
                
                tft.startWrite();
                pushSystemMessage("Inserted history item " + String(index) + ".");
                drawFullTerminal();
                drawCursorAndPreview();
                tft.endWrite();

            } else if (indexInput.length() == 0) {
                tft.startWrite();
                pushSystemMessage("History selection canceled.");
                drawFullTerminal();
                drawCursorAndPreview();
                tft.endWrite();
            } else {
                pushSystemMessage("Invalid index " + indexInput + ". Must be 0-" + String(historyCount - 1) + ".");
                int deleteCount = indexInput.length() + 1; 
                if (lastSpace == -1) deleteCount = indexInput.length();
                for (int i = 0; i < deleteCount; ++i) { backspaceAtCursor(); }

                tft.startWrite();
                drawFullTerminal();
                drawCursorAndPreview();
                tft.endWrite();
            }
            fkeyState = F_INACTIVE; 
            kmode = ALPHA;
            kbIndex = 0;
            return;
        }
        
        // --- THIS IS THE MODIFIED BLOCK ---
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
            String trimmedCommand = trimStr(fullCommand); 
            if (trimmedCommand.length() == 0) {
                tft.startWrite();
                clearCmdBuffer(); 
                drawFullTerminal(); 
                drawCursorAndPreview();
                tft.endWrite();
                return; 
            }
            
            tft.startWrite();
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
            tft.endWrite(); 
            
            // --- 1. Get the command token ---
            String cmdToken = "";
            int firstSpace = trimmedCommand.indexOf(' ');
            if (firstSpace == -1) {
                cmdToken = trimmedCommand;
            } else {
                cmdToken = trimmedCommand.substring(0, firstSpace);
            }
            cmdToken.toLowerCase();

            // --- 2. Execute the command ---
            executeCommandLine(fullCommand);
            
            // --- 3. ONLY clear buffer if it was NOT an app-launching command ---
            if (cmdToken != "nano" && cmdToken != "cube" && 
                cmdToken != "mood" && cmdToken != "moon" && 
                cmdToken != "pic" && cmdToken != "gem" && // <-- "pic" is now in this list
                cmdToken != "part") 
            {
                clearCmdBuffer(); 
            }
            
            return;
            // --- END OF MODIFIED BLOCK ---
        } 
        else if (controlAction == "DELETE") { 
            tft.startWrite();
            if (cursorPos < cmdLen) {
                for (int i = cursorPos; i < cmdLen - 1; ++i) cmdBuf[i] = cmdBuf[i + 1];
                cmdLen--;
                cmdBuf[cmdLen] = 0;
                drawFullTerminal(); 
                drawCursorAndPreview();
            }
            tft.endWrite();
        } 
        else if (controlAction == "LEFT") {
            tft.startWrite();
            if (cursorPos > 0) {
                cursorPos--;
                drawCursorAndPreview(); 
            }
            tft.endWrite();
        }
        else if (controlAction == "RIGHT") {
            tft.startWrite();
            if (cursorPos < cmdLen) cursorPos++;
            drawCursorAndPreview(); 
            tft.endWrite();
        }
        else if (controlAction == "UP") {
            tft.startWrite();
            historyRecallUp(); 
            drawCursorAndPreview();
            tft.endWrite();
            return;
        }
        else if (controlAction == "DOWN") {
            tft.startWrite();
            historyRecallDown(); 
            drawCursorAndPreview();
            tft.endWrite();
            return;
        }
    }
}
void handleButtonPresses(unsigned long now) {
    for (int i = 0; i < NUM_BUTTONS; ++i) {
        
        // --- THIS IS THE NEW LOGIC ---
        // 1. Determine the correct cooldown for this button
        unsigned long currentCooldown;
        if (i == IDX_PREV || i == IDX_NEXT) {
            currentCooldown = cooldown_nav;
        } else { // IDX_SELECT or IDX_BACK
            currentCooldown = cooldown_action;
        }

        // 2. Check the button press with its specific cooldown
        if (digitalRead(buttonPins[i]) == HIGH && (now - lastPressTime[i] > currentCooldown)) {
        // --- END OF NEW LOGIC ---
            
            lastPressTime[i] = now;

            switch (g_currentApp) {
                case APP_STATE_CLI:
                    switch (i) {
                        case IDX_PREV:
                            kbPrev();
                            g_cli_needs_redraw = true; // Set the flag
                            break;
                        case IDX_NEXT:
                            kbNext();
                            g_cli_needs_redraw = true; // Set the flag
                            break;
                        case IDX_SELECT:
                            kbConfirm();
                            break;
                        case IDX_BACK:
                            backspaceAtCursor();
                            break;
                    }
                    break;
                case APP_STATE_NANO:
                    nano_handleInput(i);
                    break;
                case APP_STATE_GEM: 
                    gem_handleInput(i);
                    break;
                case APP_STATE_CUBE:
                    if (i == IDX_BACK) {
                        cube_stop();
                    }
                    break;
                case APP_STATE_MOOD:
                    if (i == IDX_BACK) {
                        mood_stop();
                    }
                    break;
                case APP_STATE_MOON:
                    moon_handleInput(i);
                    break;
                case APP_STATE_PIC:
                    if (i == IDX_BACK) {
                        pic_stop();
                    }
                    break;
                case APP_STATE_PIX: // <-- ADD THIS CASE
                    pix_handleInput(i);
                    break;
            }
        }
    }
}
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
String trimStr(const String &s) {
    int i = 0;
    while (i < s.length() && isspace(s[i])) i++;
    int j = s.length() - 1; while (j >= 0 && isspace(s[j])) j--;
    if (j < i) return "";
    return s.substring(i, j + 1);
}
String evalCalc(const String &expr) {
    String s = expr;
    String t = "";
    // Remove spaces
    for (unsigned int i = 0; i < s.length(); ++i) if (s[i] != ' ') t += s[i];
    s = t;

    // --- Fixed-size structures ---
    const int MAX_TOKENS_OUT = 32;
    const int MAX_OPS_STACK = 16;
    const int MAX_EVAL_STACK = 16;

    // *** Modified Tok struct ***
    struct Tok {
        String v; // Only used for numbers ('n') now
        char type; // 'n' or 'o'
        char op_char; // Stores the operator directly if type is 'o'
    };

    Tok out[MAX_TOKENS_OUT]; int out_sz = 0;
    char op[MAX_OPS_STACK]; int op_sz = 0; // Operator stack remains char
    double st[MAX_EVAL_STACK]; int st_sz = 0;

    // --- Helper lambdas ---
    auto push_out = [&](Tok tk) {
        if (out_sz >= MAX_TOKENS_OUT) return false;
        out[out_sz++] = tk;
        return true;
    };
    auto push_op = [&](char o) {
        if (op_sz >= MAX_OPS_STACK) return false;
        op[op_sz++] = o;
        return true;
    };
    auto pop_op = [&]()->char {
        if (op_sz == 0) return '\0';
        return op[--op_sz];
    };

    auto isOp = [&](char c){ return c=='+'||c=='-'||c=='*'||c=='/'; };
    auto prec = [&](char c)->int { if (c=='+'||c=='-') return 1; if (c=='*'||c=='/') return 2; return 0; };

    // --- Shunting-Yard Algorithm ---
    for (unsigned int i = 0; i < s.length();) {
        char c = s[i];
        if ((c >= '0' && c <= '9') || c=='.') { // Number
            String num="";
            while (i<s.length() && ((s[i]>='0'&&s[i]<='9') || s[i]=='.')) { num+=s[i]; i++; }
            // Store number in v, type 'n', op_char irrelevant
            Tok num_token = {num, 'n', '\0'};
            if (!push_out(num_token)) return "ERR: Expression Too Complex (Output)";
        } else if (isOp(c)) { // Operator
            char opch = c;
            while (op_sz > 0 && op[op_sz-1] != '(' && prec(op[op_sz-1]) >= prec(opch)) {
                 char popped_op_char = pop_op();
                 if (popped_op_char == '\0') return "ERR: Syntax (Pop Op)";
                 // *** Store operator char in op_char, type 'o', v irrelevant ***
                 Tok operator_token = {"", 'o', popped_op_char};
                 if (!push_out(operator_token)) return "ERR: Expression Too Complex (Output)";
            }
            if (!push_op(opch)) return "ERR: Expression Too Complex (Operator Stack)";
            i++;
        } else if (c=='(') { // Left parenthesis
            if (!push_op('(')) return "ERR: Expression Too Complex (Operator Stack)";
            i++;
        } else if (c==')') { // Right parenthesis
            bool found=false;
            while (op_sz > 0) {
                char top_char = pop_op();
                if (top_char == '\0') return "ERR: Syntax (Pop Op)";
                if (top_char == '(') { found=true; break; }
                 // *** Store operator char in op_char, type 'o', v irrelevant ***
                 Tok operator_token = {"", 'o', top_char};
                if (!push_out(operator_token)) return "ERR: Expression Too Complex (Output)";
            }
            if (!found) return "ERR: Mismatched Parentheses";
            i++;
        } else { // Invalid character
             return "ERR: Invalid Character";
        }
    }

    while (op_sz > 0) { // Pop remaining operators
        char top_char = pop_op();
        if (top_char == '\0') return "ERR: Syntax (Pop Op)";
        if (top_char == '(' || top_char == ')') return "ERR: Mismatched Parentheses";
         // *** Store operator char in op_char, type 'o', v irrelevant ***
         Tok operator_token = {"", 'o', top_char};
        if (!push_out(operator_token)) return "ERR: Expression Too Complex (Output)";
    }

    // --- RPN Evaluation ---
    st_sz = 0;
    for (int i = 0; i < out_sz; ++i) {
        Tok &tk = out[i];
        if (tk.type=='n') { // Number
            if (st_sz >= MAX_EVAL_STACK) return "ERR: Evaluation Stack Overflow";
            st[st_sz] = atof(tk.v.c_str()); // Get number from v
            st_sz++;
        } else if (tk.type=='o') { // Operator
            if (st_sz < 2) return "ERR: Syntax (Eval Stack Underflow)";
            st_sz--; double b = st[st_sz];
            st_sz--; double a = st[st_sz];

            // *** Get operator directly from op_char ***
            char opch = tk.op_char;
            double res = 0;
            if (opch=='+') res = a + b;
            else if (opch=='-') res = a - b;
            else if (opch=='*') res = a * b;
            else if (opch=='/') {
                if (b == 0.0) return "INF";
                res = a / b;
            } else {
                 // Check if op_char is somehow null/invalid
                 if (opch == '\0') return "ERR: Internal Null Operator";
                 else return "ERR: Unknown Operator"; // Should not happen
            }
            st[st_sz] = res;
            st_sz++;
        }
    }

    // --- Final Result ---
    if (st_sz != 1) return "ERR: Syntax (Final Eval Stack)";

    // Format result
    char buf[64];
    dtostrf(st[0], 0, 6, buf);
    String outstr = String(buf);
    while (outstr.length()>1 && outstr.indexOf('.')>=0 && (outstr.endsWith("0") || outstr.endsWith("."))) {
        if (outstr.endsWith("0")) outstr.remove(outstr.length()-1);
        else if (outstr.endsWith(".")) { outstr.remove(outstr.length()-1); break; }
    }

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
bool executeCommandLine(const String &raw) {
    g_commandSerialError = false;
    
    String line = trimStr(raw);
    if (line.length() == 0) {
        tft.startWrite();
        drawFullTerminal(); 
        clearCurrentCommand();
        drawCursorAndPreview();
        tft.endWrite();
        return true;
    }

    addHistory(line); 

    const int MAX_TOKENS = 8;
    String tokens[MAX_TOKENS];
    int count = 0;
    tokenizeLine(line, tokens, count, MAX_TOKENS);
    if (count == 0) return true;
    
    String cmd = tokens[0];
    cmd.toLowerCase();
    
    // --- SECTION 1: FULL-SCREEN APP LAUNCHERS ---
    if (cmd == "cube") {
        cube_start();
        return true;
    } else if (cmd == "mood") {
        mood_start();
        return true;
    } else if (cmd == "moon") { 
        moon_start();
        return true;
    } else if (cmd == "gem") { 
        gem_start();
        return true;
    } else if (cmd == "pix") {
        pix_start();
        return true;
    } else if (cmd == "nano" && count >= 2) {
        if (!fsReady) {
             // Defer error to Section 2
        } else {
            String filename_typed = tokens[1];
            String filename_actual = findFileCaseInsensitive(filename_typed);
            nano_start(filename_actual);
            return true; // Successful launch
        }
    } else if (cmd == "pic" && count >= 2) {
        if (!fsReady) {
            // Defer error to Section 2
        } else {
            String filename_typed = tokens[1];
            String filename_actual = findFileCaseInsensitive(filename_typed);
            String filename_lower = filename_actual;
            filename_lower.toLowerCase();
            if (!filename_lower.endsWith(".bmp")) {
                 // Defer error to Section 2
            } else if (!LittleFS.exists(filename_actual)) { 
                 // Defer error to Section 2
            } else {
                
                pic_start(filename_actual);
                return true; // Successful launch
            }
        }
    } else if (cmd == "part" && count >= 2) {
        if (!fsReady) {
            // Defer error to Section 2
        } else {
            String filename_typed = tokens[1];
            splitAndWrapFile(filename_typed);
            clearCurrentCommand();
            return true; // Successful launch
        }
    }
    
    // --- SECTION 2: SIMPLE CLI COMMANDS & APP LAUNCH ERRORS ---
    tft.startWrite();

    // --- App Launch Errors (now handled as simple commands) ---
    if (cmd == "nano") {
        if (count < 2) {
            pushSystemMessage("Usage: nano <filename>");
        } else if (!fsReady) { 
            pushSystemMessage("Error: LittleFS not available.");
        }
    } else if (cmd == "pic") {
        if (count < 2) {
            pushSystemMessage("Usage: pic <filename.bmp>");
        } else if (!fsReady) {
            pushSystemMessage("Error: LittleFS not available.");
        } else {
            String filename_typed = tokens[1];
            String filename_actual = findFileCaseInsensitive(filename_typed);
            String filename_lower = filename_actual;
            filename_lower.toLowerCase();
            if (!filename_lower.endsWith(".bmp")) {
                pushSystemMessage("Error: Only .bmp files supported.");
            } else if (!LittleFS.exists(filename_actual)) { 
                pushSystemMessage("Error: File not found: " + filename_actual);
            }
        }
    } else if (cmd == "part") {
        if (count < 2) {
            pushSystemMessage("Usage: part <filename.txt>");
        } else if (!fsReady) {
            pushSystemMessage("Error: LittleFS not available.");
        }
    }
    // --- Simple CLI Commands (Original) ---
    else if (cmd == "help") {
        pushSystemMessage("Available commands:");
        pushScrollback("help         - Show this message.");
        pushScrollback("reboot       - Reboot RP2040");
        pushScrollback("clear        - Clear terminal history.");
        pushScrollback("calc <expr>  - Evaluate simple math.");
        pushScrollback("pi           - Display Rainbow Pi");
        pushScrollback("timer <min>  - Start timer (minutes).");
        pushScrollback("ls           - List files on LittleFS.");
        pushScrollback("cat <file>   - Display file content.");
        pushScrollback("echo <text> >/>> <file> - Write file.");
        pushScrollback("nano <file>  - Simple text editor.");
        pushScrollback("rm <file>    - Delete a file.");
        pushScrollback("send <file>  - Send file to PC via USB.");
        pushScrollback("part <file>  - TXT into ~100KB parts.");
        pushScrollback("format       - Format LT-FS partition.");
        pushScrollback("df           - Disk usage information.");
        pushScrollback("ver          - Display version info.");
        pushScrollback("time         - Show uptime since boot.");
        pushScrollback("rgb          - Toggle NeoPixel RGB.");
        pushScrollback("bl <0-255>   - Set Backlight brightness.");
        pushScrollback("fkey         - Show F-key functions.");
        pushScrollback("pic <f.bmp>  - Display BMP picture.");
        pushScrollback("pix <f.bmp>  - Make pixel art!");
        pushScrollback("cube         - 3D CUBE, back to exit.");
        pushScrollback("mood         - Cycle through RGB colors.");
        pushScrollback("moon         - Moon phases.");
        pushScrollback("gem          - Micromanager.");
        pushScrollback("startup <OS/app> [file] - Start OS/APP.");
    } else if (cmd == "fkey") {
        pushScrollback("--- F-Key functionality: ---");
        pushScrollback("F1: Print last command, char by char.");
        pushScrollback("F2: Copy last cmd up to char.");
        pushScrollback("F3: Repeat last cmd.");
        pushScrollback("F4: Delete current cmd up to char.");
        pushScrollback("F5: Recall last cmd. F6: Insert ^Z.");
        pushScrollback("F7: Show history.");
        pushScrollback("F8: Cycle back history.");  
        pushScrollback("F9: Recall by history index.");    
    } else if (cmd == "reboot") {
        pushSystemMessage("Rebooting device...");
        drawFullTerminal();
        tft.endWrite(); // <-- IMPORTANT: End the transaction
        delay(100);
        rp2040.reboot();
    } 
    // --- UPDATED STARTUP COMMAND (Preserves Backlight) ---
    else if (cmd == "startup") {
        if (!fsReady) {
            pushSystemMessage("Error: LittleFS not available.");
        } else if (count < 2) {
            pushSystemMessage("Usage: startup <app|cli> [filename]");
            pushSystemMessage(" e.g.: startup gem");
            pushSystemMessage(" e.g.: startup nano notes.txt");
        } else {
            String appName = tokens[1];
            appName.toLowerCase();
            
            // 1. Read CURRENT settings to preserve them
            bool currentRgb = false;
            int currentBl = 255; // Default if not found
            
            File f_read = LittleFS.open("settings.cfg", "r");
            if (f_read) {
                while (f_read.available()) {
                    String line = f_read.readStringUntil('\n');
                    line.trim();
                    if (line.startsWith("rgb_enabled=1")) {
                        currentRgb = true;
                    } else if (line.startsWith("backlight=")) {
                        currentBl = line.substring(10).toInt();
                    }
                }
                f_read.close();
            }

            String newStartupApp = ""; 
            bool error = false;
            
            if (appName == "picos" || appName == "default" || appName == "none" || appName == "os") {
                newStartupApp = "";
            }
            else if (appName == "nano" || appName == "pic") {
                if (count < 3) {
                    pushSystemMessage("Error: Usage: startup " + appName + " <filename>");
                    error = true;
                } else {
                    newStartupApp = appName + " " + tokens[2]; // e.g., "nano notes.txt"
                }
            }
            else if (appName == "gem" || appName == "cube" || appName == "mood" || appName == "moon" || appName == "pix") {
                newStartupApp = appName;
            }
            else {
                pushSystemMessage("Error: Unknown startup app '" + appName + "'");
                error = true;
            }

            // 2. Write ALL settings back, only if no error
            if (!error) {
                File f_write = LittleFS.open("settings.cfg", "w");
                if (!f_write) {
                    pushSystemMessage("Error: Could not write settings.cfg");
                } else {
                    if (newStartupApp.length() > 0) {
                        f_write.println("startup_app=" + newStartupApp);
                    }
                    if (currentRgb) { // Preserve RGB setting
                        f_write.println("rgb_enabled=1");
                    }
                    // Preserve Backlight setting
                    f_write.println("backlight=" + String(currentBl)); 
                    
                    f_write.close();

                    if (newStartupApp.length() == 0) {
                        pushSystemMessage("Startup reset to PICOS> OS.");
                    } else {
                        pushSystemMessage("Startup app set to: " + newStartupApp);
                    }
                    pushSystemMessage("Reboot to apply.");
                }
            }
        }
    }
    // --- END UPDATED STARTUP COMMAND ---
    else if (cmd == "clear") {
        scrollbackCount = 0;
        scrollbackHead = 0;
    } else if (cmd == "ver") {
        pushSystemMessage(deviceVersion);
    } else if (cmd == "echo") {
        if (count < 2) {
            pushSystemMessage("Usage: echo <text> >/>> <file>");
        } 
        else {
            String output = "";
            String filePath = "";
            bool appendMode = false;
            bool redirection = false;
            for (int i = 1; i < count; ++i) {
                if (tokens[i] == ">" || tokens[i] == ">>") {
                    redirection = true;
                    appendMode = (tokens[i] == ">>");
                    if (i + 1 < count) filePath = tokens[i + 1];
                    else { 
                        pushSystemMessage("Error: Missing filename write!");
                        break; 
                    }
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
        }
    } else if (cmd == "time") {
        unsigned long uptime = (millis() - startMillis) / 1000;
        unsigned long seconds = uptime % 60;
        unsigned long minutes = (uptime / 60) % 60;
        unsigned long hours = (uptime / 3600);
        char buf[32];
        sprintf(buf, "Uptime: %lu:%02lu:%02lu", hours, minutes, seconds);
        pushSystemMessage(String(buf));
    
    // --- UPDATED RGB COMMAND (Preserves Backlight) ---
    } else if (cmd == "rgb") {
        if (!fsReady) {
            pushSystemMessage("Error: LittleFS not available.");
        } else {
            // 1. Read current settings
            String currentStartupApp = "";
            bool currentRgb = false;
            int currentBl = 255;

            File f_read = LittleFS.open("settings.cfg", "r");
            if (f_read) {
                while (f_read.available()) {
                    String line = f_read.readStringUntil('\n');
                    line.trim();
                    if (line.startsWith("startup_app=")) {
                        currentStartupApp = line.substring(12);
                    } else if (line.startsWith("rgb_enabled=1")) {
                        currentRgb = true;
                    } else if (line.startsWith("backlight=")) {
                        currentBl = line.substring(10).toInt();
                    }
                }
                f_read.close();
            }

            // 2. Toggle the RGB state
            g_rgb_enabled = !currentRgb; // This is the new state

            // 3. Write all settings back
            File f_write = LittleFS.open("settings.cfg", "w");
            if (!f_write) {
                pushSystemMessage("Error: Could not write settings.cfg");
            } else {
                if (currentStartupApp.length() > 0) {
                    f_write.println("startup_app=" + currentStartupApp);
                }
                if (g_rgb_enabled) { // Write if new state is true
                    f_write.println("rgb_enabled=1");
                }
                // Preserve Backlight setting
                f_write.println("backlight=" + String(currentBl));
                
                f_write.close();
            }

            // 4. Give feedback and update hardware
            if (g_rgb_enabled) {
                pushSystemMessage("RGB enabled!");
                g_rgb_hue = 0;
                g_rgb_last_rgb_update = millis();
                strip.setPixelColor(0, strip.ColorHSV(g_rgb_hue));
                strip.show();
            } else {
                pushSystemMessage("RGB disabled!");
                strip.clear();
                strip.show();
            }
        }
    // --- NEW BACKLIGHT COMMAND (Saves Everything) ---
    } else if (cmd == "bl") {
        if (count < 2) {
            pushSystemMessage("Usage: bl <0-255>");
        } else {
            int brightness = tokens[1].toInt();
            // Safety Clamps
            if (brightness < 0) brightness = 0;
            if (brightness > 255) brightness = 255;
            
            // 1. Update Hardware & Global Variable
            analogWrite(TFT_BL, brightness);
            g_bl_brightness = brightness;
            
            // 2. Read OTHER settings to preserve them
            if (fsReady) {
                String currentStartupApp = "";
                bool currentRgb = false;
                
                File f_read = LittleFS.open("settings.cfg", "r");
                if (f_read) {
                    while (f_read.available()) {
                        String line = f_read.readStringUntil('\n');
                        line.trim();
                        if (line.startsWith("startup_app=")) {
                            currentStartupApp = line.substring(12);
                        } else if (line.startsWith("rgb_enabled=1")) {
                            currentRgb = true;
                        }
                        // Note: We don't need to read 'backlight=' because we are overwriting it
                    }
                    f_read.close();
                }

                // 3. Write ALL settings back (App + RGB + New Backlight)
                File f_write = LittleFS.open("settings.cfg", "w");
                if (f_write) {
                    if (currentStartupApp.length() > 0) {
                        f_write.println("startup_app=" + currentStartupApp);
                    }
                    if (currentRgb) {
                        f_write.println("rgb_enabled=1");
                    }
                    // Write the NEW backlight value
                    f_write.println("backlight=" + String(brightness)); 
                    
                    f_write.close();
                    pushSystemMessage("Backlight saved: " + String(brightness));
                } else {
                    pushSystemMessage("Backlight set (Save Failed)");
                }
            } else {
                pushSystemMessage("Backlight set (No FS)");
            }
        }
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
                pushScrollback(expr + " = " + result);
        }
    } else if (cmd == "timer") {
        if (count < 2) {
            pushSystemMessage("Usage: timer <minutes>");
        } else {
            long minutes = tokens[1].toInt();
            if (minutes <= 0) {
                pushSystemMessage("Error: Minutes must be a positive number.");
            } else if (timerEndTime > 0) {
                 pushSystemMessage("Error: Another timer is already running.");
            } else {
                timerEndTime = millis() + (unsigned long)minutes * 60000UL;
                pushSystemMessage("Timer: " + String(minutes) + " min(s), Blink = 10s");
            }
        }
    } else if (cmd == "ls") {
        pushScrollback(listFiles());
    } else if (cmd == "cat") {
        if (count < 2) {
            pushSystemMessage("Usage: cat <filename>");
        } else {
            String filename_typed = tokens[1];
            String filename_actual = findFileCaseInsensitive(filename_typed);
            pushScrollback(readFile(filename_actual));
        }
    } else if (cmd == "rm") {
        if (count < 2) pushSystemMessage("Usage: rm <filename>");
        else {
            String filename_typed = tokens[1];
            String filename_actual = findFileCaseInsensitive(filename_typed);
            if (removeFile(filename_actual))
                pushSystemMessage("Deleted " + filename_actual + ".");
            else pushSystemMessage("Error: File not found or couldn't be deleted.");
        }
    } else if (cmd == "format") { 
        if (!fsReady) {
            pushSystemMessage("Error: LittleFS not available. Terminating...");
        } else if (fkeyState != F_INACTIVE) {
            pushSystemMessage("Error: Already in special input mode.");
        } else {
            pushSystemMessage("WARNING: Select Y/N to confirm FORMAT!");
            fkeyState = F_AWAIT_FORMAT_CONFIRM; 
            kbIndex = 0; 
            drawFullTerminal(); 
            drawCursorAndPreview();
        }
    } else if (cmd == "df") {
        if (!fsReady) { 
            pushSystemMessage("Error: LittleFS not available.");
        } else {
            #ifdef ESP32 
                size_t totalBytes = LittleFS.totalBytes();
                size_t usedBytes = LittleFS.usedBytes();
            #else // RP2040
                FSInfo fs_info;
                if (!LittleFS.info(fs_info)) {
                     pushSystemMessage("Error: Could not get FS info.");
                     drawFullTerminal(); 
                     clearCurrentCommand();
                     drawCursorAndPreview();
                     tft.endWrite();
                     return true;
                }
                size_t totalBytes = fs_info.totalBytes;
                size_t usedBytes = fs_info.usedBytes;
            #endif
            size_t freeBytes = totalBytes - usedBytes;
            String totalStr = String(totalBytes / 1024) + "KB";
            String usedStr = String(usedBytes / 1024) + "KB";
            String freeStr = String(freeBytes / 1024) + "KB";
            String header = "Filesystem   Size   Used  Available";
            String data   = "/           " + totalStr + "  " + usedStr + "   " + freeStr;
            pushScrollback(header);
            pushScrollback(data);
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
                    tft.endWrite();
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
                    tft.startWrite();
                }
            }
        }
    } 
    
    // --- SECTION 3: UNKNOWN COMMAND (FAILURE) ---
    else {
        pushSystemMessage("Error: Unknown command '" + cmd + "'. Type 'help'.");
        g_commandSerialError = true;
        
        drawFullTerminal(); 
        clearCurrentCommand();
        drawCursorAndPreview();
        tft.endWrite();
        return false; // Report FAILURE
    }

    // --- SECTION 4: SUCCESS CASE ---
    // All simple commands (and app launch errors) fall through to here.
    drawFullTerminal();
    clearCurrentCommand();
    drawCursorAndPreview();
    tft.endWrite();
    return true; // Report SUCCESS
}
void drawMultiColorString(const String &text, int lineNum, int x_start) {
    // MAX_LINES, LINE_HEIGHT, CHAR_WIDTH, and SCREEN_WIDTH are assumed to be defined
    if (lineNum < 0 || lineNum >= MAX_LINES) return;

    // Assumes 'tft' is the global Adafruit_ST7789 object
    tft.setTextWrap(false);
    //tft.setFont(NULL);
    tft.setTextFont(1);
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
void executeFormatConfirmation(bool didConfirm) {
    // This logic is copied directly from your kbConfirm() [cite: 956-970]
    
    // 1. STATE RESET (CRITICAL)
    fkeyState = F_INACTIVE;
    kmode = ALPHA; 
    kbIndex = 0;
    cmdLen = 0;
    cursorPos = 0; 
    cmdBuf[0] = '\0';
    cursorVisible = true;
    
    // 2. EXECUTE the action
    if (didConfirm) { 
        pushSystemMessage("Format selection confirmed. Executing format...");
        drawFullTerminal(); // Draw message before blocking call.
        formatFilesystem();
    } else { 
        pushSystemMessage("Format cancelled.");
    }
    
    // 3. PERFORM FINAL STATE CLEANUP AND REDRAW
    terminalScrollOffset = 0;
    inputWrapped = false; 
    historyIndex = historyCount; 

    // 4a. Clear artifacts from the last line
    const int y_pos_start = (MAX_LINES - 1) * LINE_HEIGHT;
    const int start_y = y_pos_start - 1;
    const int y_height = LINE_HEIGHT + 1;
    tft.fillRect(0, start_y, SCREEN_WIDTH, y_height, ST77XX_BLACK);
    
    // 4b. Redraw the full terminal (Draws the clean PICOS> prompt)
    drawFullTerminal();
    
    // 4c. Redraw the new [ALPHA] mode label and cursor.
    drawCursorAndPreview();
}
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
void executeDownload(String filename) {
    if (!fsReady) {
        pushSystemMessage("Error: LittleFS not available.");
        Serial.println("DOWNLOAD_ERROR LittleFS not available."); 
        return;
    }

    if (filename.startsWith("/")) {
        filename = filename.substring(1);
    }

    File file = LittleFS.open(filename, "r"); 
    
    if (!file) {
        Serial.print("DOWNLOAD_ERROR File not found: "); 
        Serial.println(filename);
        pushSystemMessage("Error: File not found: " + filename);
        return;
    }

    size_t fileSize = file.size();
    
    // 1. Send START Marker and file information
    // Serial.print("DOWNLOAD_START "); // <-- OLD BUG 
    Serial.print("SEND "); // <-- *** FIX ***
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
    // Serial.println("DOWNLOAD_END"); // <-- OLD BUG 
    Serial.println("END"); // <-- *** FIX ***
    
    pushSystemMessage("Streaming complete: " + filename);
}
void handleSerialCommands() {
    static char buffer[512];
    static size_t commandBufLen = 0;
    static bool sawCarriageReturn = false; 

    while (Serial.available()) {
        char c = Serial.read();

        if (c == '\n' || c == '\r') {
            if (commandBufLen > 0) {
                buffer[commandBufLen] = '\0';
                String cmdLine = String(buffer);
                commandBufLen = 0;
                cmdLine.trim();
                
                if (cmdLine.length() == 0) continue;

                // --- 1. Global "Master" Override ---
                if (cmdLine.equalsIgnoreCase("exit") && g_currentApp != APP_STATE_CLI) {
            
                    // Stop whatever app is running and go back to the CLI
                    if (g_currentApp == APP_STATE_CUBE) cube_stop();
                    if (g_currentApp == APP_STATE_NANO) nano_stop();
                    if (g_currentApp == APP_STATE_MOOD) mood_stop();
                    if (g_currentApp == APP_STATE_MOON) moon_stop();
                    if (g_currentApp == APP_STATE_PIC) pic_stop();
                    if (g_currentApp == APP_STATE_GEM) gem_stop();
                    
                    commandBufLen = 0; 
                    continue;
                }

                // --- 2. "Modal" Input (Format Y/N) ---
                if (fkeyState == F_AWAIT_FORMAT_CONFIRM) {
                    cmdLine.toUpperCase(); 
                    if (cmdLine == "Y") executeFormatConfirmation(true);
                    else if (cmdLine == "N") executeFormatConfirmation(false);
                    else {
                        pushSystemMessage("Invalid input. Please enter Y or N.");
                        Serial.println("Invalid input. Please enter Y or N.");
                        drawFullTerminal();
                    }
                    continue; 
                }
                
                // --- 3. Protocol Commands (UPLOAD/CAT) ---
                int firstSpace = cmdLine.indexOf(' ');
                String command = (firstSpace == -1) ? cmdLine : cmdLine.substring(0, firstSpace);
                command.toUpperCase();

                if (command == "UPLOAD") {
                    String args = cmdLine.substring(firstSpace + 1);
                    int secondSpace = args.indexOf(' ');
                    if (secondSpace > 0) {
                        String filename = args.substring(0, secondSpace);
                        String sizeStr = args.substring(secondSpace + 1);
                        size_t fileSize = (size_t)sizeStr.toInt();
                        while (Serial.available()) Serial.read(); 
                        executeUpload(filename, fileSize);
                    } else {
                        pushSystemMessage("Error: UPLOAD command malformed (needs file & size).");
                        Serial.println("FATAL ERROR: UPLOAD syntax error."); 
                    }
                } 
                else if (command == "DOWNLOAD") {
                    if (firstSpace != -1) {
                        String filename = cmdLine.substring(firstSpace + 1);
                        filename.trim();
                        executeDownload(filename); // <-- Renamed function
                    } else {
                        pushSystemMessage("Error: DOWNLOAD command requires a filename.");
                        Serial.println("ERROR: DOWNLOAD requires filename.");
                    }
                }
                
                // --- 4. Application-Specific Command Routing ---
                else if (g_currentApp == APP_STATE_CLI) {
                    // ROUTE TO: CLI Manager
                    executeCommandLine(cmdLine);
                }
                else if (g_currentApp == APP_STATE_GEM) {
                    // ROUTE TO: Gem Manager
                    gem_handleSerialInput(cmdLine);
                }
                // --- [THIS IS THE NEW, UPGRADED NANO HANDLER] ---
                else if (g_currentApp == APP_STATE_NANO) {
                    // ROUTE TO: Nano Manager (as smart, simulated typing)
                    
                    // --- FIX: Increase token limit to 3 to detect "bs 5 hello" ---
                    const int MAX_NANO_TOKENS = 3;
                    String tokens[MAX_NANO_TOKENS];
                    int count = 0;
                    tokenizeLine(cmdLine, tokens, count, MAX_NANO_TOKENS);

                    String nanoCmd = "";
                    if (count > 0) {
                        nanoCmd = tokens[0];
                        nanoCmd.toLowerCase();
                    }

                    // --- Helper function to check if a string is purely numeric ---
                    auto isNumeric = [](String s) {
                        if (s.length() == 0) return false;
                        for (unsigned int i = 0; i < s.length(); i++) {
                            if (!isDigit(s.charAt(i))) return false;
                        }
                        return true;
                    };

                    tft.startWrite(); // Start one transaction
                    bool isCommand = true; // Assume it's a command until proven otherwise

                    // --- NEW, ROBUST PARSING LOGIC ---
                    if (nanoCmd == "enter") {
                        if (count > 1) isCommand = false; // "enter the tavern" is text
                        else nano_insertLine();
                    }
                    else if (nanoCmd == "backspace" || nanoCmd == "bs") {
                        int numToRun = 1;
                        if (count == 1) { // "bs"
                            numToRun = 1;
                        } else if (count == 2) { // "bs 5" or "bs hello"
                            if (isNumeric(tokens[1])) {
                                numToRun = tokens[1].toInt();
                                if (numToRun == 0) numToRun = 1;
                            } else {
                                isCommand = false; // "bs hello"
                            }
                        } else { // "bs 5 hello" or more
                            isCommand = false;
                        }
                        if (isCommand) for (int i = 0; i < numToRun; i++) nano_backspace();
                    } 
                    else if (nanoCmd == "space" || nanoCmd == "sp") {
                        int numToRun = 1;
                        if (count == 1) { // "sp"
                            numToRun = 1;
                        } else if (count == 2) { // "sp 5" or "sp gameboy"
                            if (isNumeric(tokens[1])) {
                                numToRun = tokens[1].toInt();
                                if (numToRun == 0) numToRun = 1;
                            } else {
                                isCommand = false; // "sp gameboy"
                            }
                        } else { // "sp 5 hello"
                            isCommand = false;
                        }
                        if (isCommand) for (int i = 0; i < numToRun; i++) nano_insertChar(' ');
                    }
                    else if (nanoCmd == "tab") {
                        const int TAB_SIZE = 4;
                        int numToRun = 1;
                        if (count == 1) { // "tab"
                            numToRun = 1;
                        } else if (count == 2) { // "tab 3" or "tab something"
                            if (isNumeric(tokens[1])) {
                                numToRun = tokens[1].toInt();
                                if (numToRun == 0) numToRun = 1;
                            } else {
                                isCommand = false; // "tab something"
                            }
                        } else { // "tab 3 test"
                            isCommand = false;
                        }
                        if (isCommand) {
                            for (int t = 0; t < numToRun; t++) {
                                for (int i = 0; i < TAB_SIZE; i++) nano_insertChar(' ');
                            }
                        }
                    }
                    else if (nanoCmd == "up") {
                        int numToRun = 1;
                        if (count == 1) { // "up"
                            numToRun = 1;
                        } else if (count == 2) { // "up 5" or "up hello"
                            if (isNumeric(tokens[1])) {
                                numToRun = tokens[1].toInt();
                                if (numToRun == 0) numToRun = 1;
                            } else {
                                isCommand = false; // "up hello"
                            }
                        } else { // "up 5 hello"
                            isCommand = false;
                        }
                        if (isCommand) for (int i = 0; i < numToRun; i++) nano_moveCursor(0, -1);
                    }
                    else if (nanoCmd == "down") {
                        int numToRun = 1;
                        if (count == 1) { numToRun = 1; }
                        else if (count == 2 && isNumeric(tokens[1])) {
                            numToRun = tokens[1].toInt();
                            if (numToRun == 0) numToRun = 1;
                        } else { isCommand = false; }
                        if (isCommand) for (int i = 0; i < numToRun; i++) nano_moveCursor(0, 1);
                    }
                    else if (nanoCmd == "left") {
                        int numToRun = 1;
                        if (count == 1) { numToRun = 1; }
                        else if (count == 2 && isNumeric(tokens[1])) {
                            numToRun = tokens[1].toInt();
                            if (numToRun == 0) numToRun = 1;
                        } else { isCommand = false; }
                        if (isCommand) for (int i = 0; i < numToRun; i++) nano_moveCursor(-1, 0);
                    }
                    else if (nanoCmd == "right") {
                        int numToRun = 1;
                        if (count == 1) { numToRun = 1; }
                        else if (count == 2 && isNumeric(tokens[1])) {
                            numToRun = tokens[1].toInt();
                            if (numToRun == 0) numToRun = 1;
                        } else { isCommand = false; }
                        if (isCommand) for (int i = 0; i < numToRun; i++) nano_moveCursor(1, 0);
                    }
                    else if (nanoCmd == "save") {
                        if (nano_saveFile()) { 
                            pushSystemMessage("File Saved: " + nano_filename); 
                        } else {
                            pushSystemMessage("Error Saving File!");
                        }
                        nano_drawHeader(); 
                    }

                    else {
                        // The command was not recognized
                        isCommand = false;
                    }
                    // --- DEFAULT: If it wasn't a valid command, type the text ---
                    if (!isCommand) {
                        for (int i = 0; i < cmdLine.length(); i++) {
                            nano_insertChar(cmdLine.charAt(i));
                        }
                    }
                    
                    // Manually redraw nano's UI after the action
                    nano_drawUI();
                    nano_drawEditorCursor();
                    tft.endWrite(); // End the transaction
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
void pollCardKB() {
    // 1. Non-blocking poll timer
    if (millis() - g_kb_last_poll_time < KB_POLL_INTERVAL_MS) {
        return;
    }
    g_kb_last_poll_time = millis();

    // 2. Request one byte from the keyboard
    Wire.requestFrom(CARDKB_I2C_ADDR, 1);
    if (!Wire.available()) {
        return;
    }

    uint8_t c = Wire.read();
    if (c == 0x00) {
        return;
    }

    // 3. Route keypress based on active app
    switch (g_currentApp) {

        // --- CLI Input Handling ---
        case APP_STATE_CLI:
            if (c == 0x08) { // Backspace
                backspaceAtCursor();
            }
            else if (c == KEY_ESCAPE) { // Escape
                tft.startWrite();
                clearCurrentCommand();
                drawFullTerminal();
                drawCursorAndPreview();
                tft.endWrite();
            }
            else if (c == 0x0D) { // Enter
                String fullCommand = String(cmdBuf).substring(0, cmdLen);
                String trimmedCommand = trimStr(fullCommand);
                
                if (trimmedCommand.length() == 0) {
                    tft.startWrite();
                    clearCmdBuffer();
                    drawFullTerminal();
                    drawCursorAndPreview();
                    tft.endWrite();
                    return; 
                }

                tft.startWrite();
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
                tft.endWrite();
                
                executeCommandLine(fullCommand);
                if (g_currentApp == APP_STATE_CLI) {
                    clearCmdBuffer();
                }
            }
            else if (c == KEY_UP_ARROW) {
                tft.startWrite();
                historyRecallUp();
                drawCursorAndPreview();
                tft.endWrite();
            }
            else if (c == KEY_DOWN_ARROW) {
                tft.startWrite();
                historyRecallDown();
                drawCursorAndPreview();
                tft.endWrite();
            }
            else if (c == KEY_LEFT_ARROW) {
                tft.startWrite();
                if (cursorPos > 0) {
                    cursorPos--;
                    drawCursorAndPreview();
                }
                tft.endWrite();
            }
            else if (c == KEY_RIGHT_ARROW) {
                tft.startWrite();
                if (cursorPos < cmdLen) {
                    cursorPos++;
                }
                drawCursorAndPreview();
                tft.endWrite();
            }
            else if (c >= 0x20 && c <= 0x7E) { // Printable ASCII
                insertCharAtCursor(c);
            }
            break;

        // --- Nano Editor Input Handling ---
        case APP_STATE_NANO: {
            tft.startWrite();
            
            if (c == 0x08) { // Backspace
                nano_backspace();
            }
            else if (c == KEY_ESCAPE) { // Escape
                if (nano_focus == NANO_AWAIT_SAVE_CONFIRM) {
                    // Go back from "Save Y/N" to the editor
                    nano_focus = FOCUS_TEXT;
                    nano_drawUI(); 
                } else if (nano_isModified) {
                    // In editor, prompt to save
                    nano_focus = NANO_AWAIT_SAVE_CONFIRM;
                    nano_saveConfirmSelection = 0;
                    nano_drawFooter();
                } else {
                    // Not modified, just exit
                    nano_stop();
                    tft.endWrite(); 
                    return;
                }
            }
            else if (c == 0x0D) { // Enter
                if (nano_focus == FOCUS_TEXT) {
                    nano_insertLine();
                } else if (nano_focus == FOCUS_FOOTER) {
                    if (nano_footerSelection == 0) { // Action: SAVE
                        if (nano_saveFile()) {
                             nano_isModified = false;
                             pushSystemMessage("File Saved: " + nano_filename);
                        } else {
                             pushSystemMessage("Error Saving File!");
                        }
                        nano_focus = FOCUS_TEXT;
                        nano_drawUI();
                    } else if (nano_footerSelection == 1) { // Action: EXIT
                         if (nano_isModified) {
                             nano_focus = NANO_AWAIT_SAVE_CONFIRM;
                             nano_saveConfirmSelection = 0;
                             nano_drawFooter();
                         } else {
                             nano_stop();
                             tft.endWrite(); 
                             return;
                         }
                    }
                } else if (nano_focus == NANO_AWAIT_SAVE_CONFIRM) {
                    if (nano_saveConfirmSelection == 0) { // 'Y'
                         if (!nano_saveFile()) {
                            pushSystemMessage("Error Saving File! Exit aborted.");
                            nano_focus = FOCUS_TEXT; 
                            nano_drawUI();
                        } else {
                            nano_isModified = false;
                            pushSystemMessage("File Saved. Exiting.");
                            nano_stop(); 
                            tft.endWrite(); 
                            return;
                        }
                    } else { // 'N'
                        pushSystemMessage("Exiting without saving.");
                        nano_stop(); 
                        tft.endWrite(); 
                        return;
                    }
                }
            }
            else if (c == KEY_UP_ARROW) {
                if (nano_focus == FOCUS_TEXT) {
                    nano_moveCursor(0, -1);
                } else if (nano_focus == FOCUS_FOOTER) {
                    if (nano_footerSelection == 0) { 
                        nano_focus = FOCUS_TEXT;
                        nano_drawUI(); 
                    } else {
                        nano_footerSelection--;
                        nano_drawFooter();
                    }
                }
            }
            else if (c == KEY_DOWN_ARROW) {
                if (nano_focus == FOCUS_TEXT) {
                    if (nano_cursorLine == nano_lineCount - 1) {
                         nano_focus = FOCUS_FOOTER;
                         nano_footerSelection = 0;
                         nano_drawUI();
                     } else {
                         nano_moveCursor(0, 1);
                     }
                } else if (nano_focus == FOCUS_FOOTER) {
                    if (nano_footerSelection < 1) { 
                        nano_footerSelection++;
                    }
                    nano_drawFooter();
                }
            }
            else if (c == KEY_LEFT_ARROW) {
                if (nano_focus == FOCUS_TEXT) {
                    nano_moveCursor(-1, 0);
                } else if (nano_focus == FOCUS_FOOTER) {
                    if (nano_footerSelection > 0) {
                        nano_footerSelection--;
                    }
                    nano_drawFooter();
                }
            }
            else if (c == KEY_RIGHT_ARROW) {
                if (nano_focus == FOCUS_TEXT) {
                    nano_moveCursor(1, 0);
                } else if (nano_focus == FOCUS_FOOTER) {
                    if (nano_footerSelection < 1) { 
                        nano_footerSelection++;
                    }
                    nano_drawFooter();
                }
            }
            else if (c == ' ') { // Space
                if (nano_focus == FOCUS_TEXT) {
                    int screenLine = nano_cursorLine - nano_topLine;
                    int cursorScreenX = nano_cursorCol * CHAR_WIDTH;
                    int cursorScreenY = (NANO_HEADER_LINES + screenLine) * LINE_HEIGHT;
                    bool wrapOccurred = nano_insertChar(' ');
                    if (!wrapOccurred && screenLine >= 0 && screenLine < NANO_TEXT_AREA_LINES) {
                        tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
                        tft.setCursor(cursorScreenX, cursorScreenY);
                        tft.print(' ');
                        nano_redrawTrailingText();
                    }
                }
            }
            else if (c >= 0x21 && c <= 0x7E) { // Other Printable ASCII
                if (nano_focus == FOCUS_TEXT) {
                    int screenLine = nano_cursorLine - nano_topLine;
                    int cursorScreenX = nano_cursorCol * CHAR_WIDTH;
                    int cursorScreenY = (NANO_HEADER_LINES + screenLine) * LINE_HEIGHT;
                    bool wrapOccurred = nano_insertChar(c);
                    // if (!wasModified) nano_drawHeader(); // Optional optimization
                    if (!wrapOccurred && screenLine >= 0 && screenLine < NANO_TEXT_AREA_LINES) {
                        tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
                        tft.setCursor(cursorScreenX, cursorScreenY);
                        tft.print(c);
                        nano_redrawTrailingText();
                    }
                } else if (nano_focus == NANO_AWAIT_SAVE_CONFIRM) {
                    char upperC = toupper(c);
                    if (upperC == 'Y') {
                        nano_saveConfirmSelection = 0;
                        nano_drawFooter();
                    } else if (upperC == 'N') {
                        nano_saveConfirmSelection = 1;
                        nano_drawFooter();
                    }
                }
            }
            
            if (nano_focus == FOCUS_TEXT) {
                nano_drawEditorCursor();
            }
            tft.endWrite();
            break;
        }

        // --- Pix Editor Input Handling ---
        case APP_STATE_PIX:
            pix_handleKey(c);
            break;

        // --- Gem App Input Handling (Refactored) ---
        case APP_STATE_GEM:
            gem_handleKey(c);
            break;

        // --- Minor Apps ---
        case APP_STATE_MOON:
             if (c == KEY_ESCAPE) moon_stop();
             else if (c == KEY_LEFT_ARROW || c == KEY_UP_ARROW) moon_handleInput(IDX_PREV);
             else if (c == KEY_RIGHT_ARROW || c == KEY_DOWN_ARROW) moon_handleInput(IDX_NEXT);
             break;
        case APP_STATE_CUBE:
            if (c == KEY_ESCAPE) cube_stop();
            break;
        case APP_STATE_MOOD:
            if (c == KEY_ESCAPE) mood_stop();
            break;
        case APP_STATE_PIC:
            if (c == KEY_ESCAPE) pic_stop();
            break;
    }
}
void setup() {
    // Setup serial for debugging (optional)
    Serial.begin(115200);
    Wire.setSDA(0); // Use GPIO 0 for SDA
    Wire.setSCL(1); // Use GPIO 1 for SCL
    Wire.begin();
    randomSeed(analogRead(0)); 
    // TFT initialization
    tft.begin(); 
    tft.setRotation(3);
    tft.fillScreen(ST77XX_BLACK);
    tft.setTextWrap(false);
    pinMode(TFT_BL, OUTPUT);
    
    // Attempt to load settings from file
    if (LittleFS.begin()) {
        if (LittleFS.exists("settings.cfg")) {
            File f = LittleFS.open("settings.cfg", "r");
            if (f) {
                while (f.available()) {
                    String line = f.readStringUntil('\n');
                    line.trim();
                    if (line.startsWith("backlight=")) {
                        g_bl_brightness = line.substring(10).toInt();
                    }
                }
                f.close();
            }
        }
    }
    
    // Apply the loaded (or default) brightness
    if (g_bl_brightness < 0) g_bl_brightness = 0;
    if (g_bl_brightness > 255) g_bl_brightness = 255;
    analogWrite(TFT_BL, g_bl_brightness);
    //tft.setFont(NULL); 
    tft.setTextFont(1);
    tft.setTextSize(1);
    tft.setTextColor(ST77XX_WHITE);
    tft.setCursor(0, 0);
    // LED Initialization
    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, LOW);
    // Initialize button pins
    for (int i = 0; i < NUM_BUTTONS; ++i) {
        pinMode(buttonPins[i], INPUT_PULLDOWN);
        lastPressTime[i] = 0;
    }

    // --- NEW: Initialize NeoPixel ---
    strip.begin();
    strip.setBrightness(20); // Low brightness
    strip.show(); // Initialize to 'off'
    // --- END NEW ---

    if (fsBegin()) { 
        pushSystemMessage("LittleFS mounted.");
        fsReady = true; // Set the flag to true
        
    } else {
        pushSystemMessage("Warning: LittleFS mount/format failed. File commands disabled.");
        fsReady = false; // Ensure flag is false
    }

    // --- THIS IS THE NEW STARTUP LOGIC ---
    bool appLaunched = false;
    if (fsReady) {
        if (LittleFS.exists("settings.cfg")) {
            File f = LittleFS.open("settings.cfg", "r");
            if (f) {
                String startupAppLine = "";

                // Read the whole file
                while (f.available()) {
                    String line = f.readStringUntil('\n');
                    line.trim();
                    
                    if (line.startsWith("startup_app=")) {
                        startupAppLine = line.substring(12); // length of "startup_app="
                    }
                    else if (line.startsWith("rgb_enabled=1")) {
                        // Enable RGB if found in config
                        g_rgb_enabled = true;
                        g_rgb_hue = 0;
                        g_rgb_last_rgb_update = millis();
                        strip.setPixelColor(0, strip.ColorHSV(g_rgb_hue));
                        strip.show();
                    }
                }
                f.close();
                
                // Now, process the startup app
                if (startupAppLine.length() > 0) {
                    const int MAX_TOKENS = 2;
                    String tokens[MAX_TOKENS];
                    int count = 0;
                    tokenizeLine(startupAppLine, tokens, count, MAX_TOKENS);
                    if (count > 0) {
                        String appName = tokens[0];
                        appName.toLowerCase();
                        
                        if (appName == "gem") {
                            gem_start();
                            appLaunched = true;
                        } else if (appName == "cube") {
                            cube_start();
                            appLaunched = true;
                        } else if (appName == "mood") {
                            mood_start();
                            appLaunched = true;
                        } else if (appName == "moon") {
                            moon_start();
                            appLaunched = true;
                        } else if (appName == "nano" && count == 2) {
                            nano_start(tokens[1]);
                            appLaunched = true;
                        } else if (appName == "pic" && count == 2) {
                            pic_start(tokens[1]);
                            appLaunched = true;
                        }
                        else if (appName == "pix") {
                            pix_start();
                            appLaunched = true;
                        }
                    }
                }
            }
        }
    }
    // --- END OF NEW LOGIC ---


    // --- Fallback to CLI if no startup app was launched ---
    if (!appLaunched) {
        pushSystemMessage("Welcome to PICOS!");
        pushSystemMessage("Type 'help' for commands.");
        drawFullTerminal(); // Only draw terminal if we're starting in the CLI
    }
    // --- END OF NEW LOGIC ---

    historyIndex = historyCount;
    //multicore_launch_core1(core1_entry);
    handleSerialCommands();
}
void loop() {
    unsigned long now = millis();
    
    // --- 1. ALWAYS Handle Global Timers & Input ---
    
    // --- LED Timeout ---
    if (ledBlinkEndTime != 0 && now > ledBlinkEndTime && timerFinishedBlinkEndTime == 0) {
        digitalWrite(STATUS_LED_PIN, LOW);
        ledBlinkEndTime = 0;
    }

    // --- Global CLI Timer Finished Check ---
    if (timerEndTime > 0 && now >= timerEndTime) {
        timerEndTime = 0;
        lastTimerActiveBlink = 0;
        pushSystemMessage(">>> Timer Finished! <<<");
        timerFinishedBlinkEndTime = now + 10000;
        lastFinishedBlinkToggle = now;
        digitalWrite(STATUS_LED_PIN, HIGH);
        
        if (g_currentApp == APP_STATE_CLI) { // Only draw if in CLI
            tft.startWrite();
            drawFullTerminal();
            drawCursorAndPreview();
            tft.endWrite();
        }
    }

    // --- Global CLI Timer Finished Blinking Pattern Handler ---
    if (timerFinishedBlinkEndTime > 0) {
        if (now >= timerFinishedBlinkEndTime) {
            timerFinishedBlinkEndTime = 0;
            digitalWrite(STATUS_LED_PIN, LOW);
        } else {
            if (now - lastFinishedBlinkToggle >= FINISHED_BLINK_INTERVAL_MS) {
                digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN));
                lastFinishedBlinkToggle = now;
            }
        }
    }

    // --- Global CLI Periodic Blink While Timer Active ---
    if (g_rgb_enabled) {
        
        // Update every 20 milliseconds (50 FPS)
        if (now - g_rgb_last_rgb_update > 20UL) { 
            g_rgb_last_rgb_update = now;
            
            // Advance the hue by a tiny amount.
            // A step of 10 every 20ms will take ~2 minutes
            // for a full, smooth cycle (0-65535).
            g_rgb_hue += 10; 
            
            // strip.ColorHSV() automatically handles the wrap-around
            // when g_rgb_hue goes past 65535.
            strip.setPixelColor(0, strip.ColorHSV(g_rgb_hue));
            strip.show();
        }
    }
    // --- END NEW ---

    handleButtonPresses(now);
    pollCardKB();
    handleSerialCommands(); // <-- MOVED UP to run always

    // --- 2. Run the "update" function for the active app ---
    switch (g_currentApp) {
        case APP_STATE_CLI:
            run_cli_update(now); // Run CLI cursor blinks
            break;
        case APP_STATE_CUBE:
            cube_update(); // Run one frame of the cube animation
            break;
        case APP_STATE_NANO:
            nano_update(now); // Run nano's blink/update logic
            break;
        case APP_STATE_MOOD:
            mood_update(); // Run one frame of the mood light
            break;
        case APP_STATE_MOON:
            moon_update(); // This app is idle
            break;
        case APP_STATE_PIC:
            pic_update(); // This app is idle
            break;
        case APP_STATE_GEM:
            gem_update(now); // Run Gem's blink/update logic
            break;
        case APP_STATE_PIX:
            pix_update(now);
            break;
    }
}