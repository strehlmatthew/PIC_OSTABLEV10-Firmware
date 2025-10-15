// PICOLINKV1.cpp : Defines the entry point for the application.
//

#include "framework.h"
#include "PICOLINKV1.h"
#include <windows.h>
#include <commdlg.h>
#include <shlobj.h>
#include <string>
#include <sstream>
#include <fstream>
#include <vector>
#include <filesystem>
#include <thread>
#include <chrono>
#include <mutex>
#include <atomic>

// GLOBAL SYNCHRONIZATION OBJECTS
std::mutex serialMutex; // Protects hSerial and related serial calls (e.g., PurgeComm, ReadFile, WriteFile)
std::atomic<bool> picoReady(false); // Not used in this version, but preserved for future use
std::mutex msgMutex;    // Protects serialMessages
std::string serialMessages; // Shared buffer for incoming serial data

#undef min
#define MAX_LOADSTRING 100
#define IDC_UPLOAD       101
#define IDC_DOWNLOAD     102
#define IDC_DEBUG        103

// --- PROTOCOL CONSTANTS ---
const std::string ACK_MSG = "ACK";
const std::string READY_MSG = "READY";
const std::string UPLOAD_OK_MSG = "UPLOAD_OK";
const size_t BLOCK_SIZE = 512;
// --- TIMEOUT CONFIG ---
const int READY_TIMEOUT_SECONDS = 10;
const int ACK_TIMEOUT_SECONDS = 5;
const int UPLOAD_OK_TIMEOUT_SECONDS = 10;


// Global Variables:
HINSTANCE hInst;                    // current instance
WCHAR szTitle[MAX_LOADSTRING];      // The title bar text
WCHAR szWindowClass[MAX_LOADSTRING];  // the main window class name
HWND hDebug;
HANDLE hSerial = INVALID_HANDLE_VALUE; // Serial port handle

// Forward declarations
ATOM MyRegisterClass(HINSTANCE hInstance);
BOOL InitInstance(HINSTANCE, int);
LRESULT CALLBACK WndProc(HWND, UINT, WPARAM, LPARAM);
INT_PTR CALLBACK About(HWND, UINT, WPARAM, LPARAM);
void CreateUI(HWND hWnd);
bool OpenSerialPort(const std::wstring& portName);
void Log(const std::string& msg);
void FlushSerialMessagesToLog(); // NEW: Function to display raw serial output
void UploadFile();
std::wstring GetPicoLinkFolder();
void SendData(const char* data, DWORD size);
void PicoListenerThread();

// ----------------------------------------------------
// HELPER: WString to String Conversion (Fixes C4244 warning)
// ----------------------------------------------------
std::string wstring_to_string(const std::wstring& wstr)
{
    if (wstr.empty()) return std::string();

    int size_needed = WideCharToMultiByte(
        CP_UTF8,
        0,
        wstr.data(),
        (int)wstr.size(),
        NULL,
        0,
        NULL,
        NULL
    );

    std::string strTo(size_needed, 0);
    WideCharToMultiByte(
        CP_UTF8,
        0,
        wstr.data(),
        (int)wstr.size(),
        &strTo[0],
        size_needed,
        NULL,
        NULL
    );

    return strTo;
}


int APIENTRY wWinMain(_In_ HINSTANCE hInstance,
    _In_opt_ HINSTANCE hPrevInstance,
    _In_ LPWSTR       lpCmdLine,
    _In_ int          nCmdShow)
{
    UNREFERENCED_PARAMETER(hPrevInstance);
    UNREFERENCED_PARAMETER(lpCmdLine);

    LoadStringW(hInstance, IDS_APP_TITLE, szTitle, MAX_LOADSTRING);
    LoadStringW(hInstance, IDC_PICOLINKV1, szWindowClass, MAX_LOADSTRING);
    MyRegisterClass(hInstance);

    if (!InitInstance(hInstance, nCmdShow))
        return FALSE;

    HACCEL hAccelTable = LoadAccelerators(hInstance, MAKEINTRESOURCE(IDC_PICOLINKV1));
    MSG msg;

    while (GetMessage(&msg, nullptr, 0, 0))
    {
        if (!TranslateAccelerator(msg.hwnd, hAccelTable, hAccelTable))
        {
            TranslateMessage(&msg);
            DispatchMessage(&msg);
        }
    }

    return (int)msg.wParam;
}

ATOM MyRegisterClass(HINSTANCE hInstance)
{
    WNDCLASSEXW wcex;
    wcex.cbSize = sizeof(WNDCLASSEX);
    wcex.style = CS_HREDRAW | CS_VREDRAW;
    wcex.lpfnWndProc = WndProc;
    wcex.cbClsExtra = 0;
    wcex.cbWndExtra = 0;
    wcex.hInstance = hInstance;
    wcex.hIcon = LoadIcon(hInstance, MAKEINTRESOURCE(IDI_PICOLINKV1));
    wcex.hCursor = LoadCursor(nullptr, IDC_ARROW);
    wcex.hbrBackground = (HBRUSH)(COLOR_WINDOW + 1);
    wcex.lpszMenuName = MAKEINTRESOURCEW(IDC_PICOLINKV1);
    wcex.lpszClassName = szWindowClass;
    wcex.hIconSm = LoadIcon(wcex.hInstance, MAKEINTRESOURCE(IDI_SMALL));
    return RegisterClassExW(&wcex);
}

BOOL InitInstance(HINSTANCE hInstance, int nCmdShow)
{
    hInst = hInstance;

    const int btnX = 20;
    const int btnY = 20;
    const int btnWidth = 160;
    const int margin = 20;

    // Initial client width matches button right edge + margin
    const int clientWidth = btnX + btnWidth + margin;
    const int clientHeight = 220;

    RECT rc = { 0, 0, clientWidth, clientHeight };
    AdjustWindowRect(&rc, WS_OVERLAPPEDWINDOW, FALSE);

    HWND hWnd = CreateWindowW(
        szWindowClass,
        szTitle,
        WS_OVERLAPPEDWINDOW,
        CW_USEDEFAULT, CW_USEDEFAULT,
        rc.right - rc.left,
        rc.bottom - rc.top,
        nullptr, nullptr, hInstance, nullptr
    );

    if (!hWnd) return FALSE;

    ShowWindow(hWnd, nCmdShow);
    UpdateWindow(hWnd);

    CreateUI(hWnd);  // Creates upload button + debug window

    // Open serial port...
    if (!OpenSerialPort(L"COM4")) {
        Log("Failed to open serial port. Will retry automatically.");
    }

    return TRUE;
}

void Log(const std::string& msg)
{
    // The Log function must be thread-safe as it is called from both the GUI thread and the worker threads.
    // However, SendMessage is generally safe for cross-thread calls to the main thread's windows.
    std::stringstream ss;
    SYSTEMTIME st;
    GetLocalTime(&st);
    ss << "[" << st.wHour << ":" << st.wMinute << ":" << st.wSecond << "] " << msg << "\r\n";

    // These calls target the hDebug Edit control created on the main thread
    SendMessageA(hDebug, EM_SETSEL, -1, -1);
    SendMessageA(hDebug, EM_REPLACESEL, FALSE, (LPARAM)ss.str().c_str());
    SendMessageA(hDebug, EM_SCROLLCARET, 0, 0);
}

// ----------------------------------------------------
// NEW: Flushes the serialMessages buffer to the debug log
// ----------------------------------------------------
void FlushSerialMessagesToLog()
{
    std::lock_guard<std::mutex> lock(msgMutex);
    if (!serialMessages.empty())
    {
        // Log the raw content of the buffer
        Log("[PICO RAW] " + serialMessages);
        // Clear the buffer after logging
        serialMessages.clear();
    }
}

bool OpenSerialPort(const std::wstring& portName)
{
    std::lock_guard<std::mutex> lock(serialMutex); // lock while opening
    hSerial = CreateFileW(portName.c_str(), GENERIC_READ | GENERIC_WRITE,
        0, 0, OPEN_EXISTING, 0, 0);

    if (hSerial == INVALID_HANDLE_VALUE)
    {
        return false;
    }

    DCB dcb = { 0 };
    dcb.DCBlength = sizeof(DCB);
    if (!GetCommState(hSerial, &dcb)) {
        Log("ERROR: GetCommState failed.");
        CloseHandle(hSerial);
        hSerial = INVALID_HANDLE_VALUE;
        return false;
    }

    dcb.BaudRate = CBR_115200;
    dcb.ByteSize = 8;
    dcb.StopBits = ONESTOPBIT;
    dcb.Parity = NOPARITY;

    if (!SetCommState(hSerial, &dcb)) {
        Log("ERROR: SetCommState failed.");
        CloseHandle(hSerial);
        hSerial = INVALID_HANDLE_VALUE;
        return false;
    }

    // Set Timeouts (needed for non-blocking ReadFile in listener)
    COMMTIMEOUTS timeouts = { 0 };
    timeouts.ReadIntervalTimeout = 50;
    timeouts.ReadTotalTimeoutConstant = 50;
    timeouts.ReadTotalTimeoutMultiplier = 10;
    timeouts.WriteTotalTimeoutConstant = 50;
    timeouts.WriteTotalTimeoutMultiplier = 10;
    SetCommTimeouts(hSerial, &timeouts);

    Log("INIT: Connected to Pico on " + wstring_to_string(portName));
    return true;
}


std::wstring GetPicoLinkFolder()
{
    wchar_t path[MAX_PATH];
    if (SUCCEEDED(SHGetFolderPathW(NULL, CSIDL_PERSONAL, NULL, SHGFP_TYPE_CURRENT, path)))
    {
        std::wstring folder = std::wstring(path) + L"\\PICOLINK\\";
        CreateDirectoryW(folder.c_str(), NULL);
        return folder;
    }
    return L"C:\\PICOLINK\\";
}

void SendData(const char* data, DWORD size)
{
    // WriteFile must be protected by the serialMutex
    std::lock_guard<std::mutex> lock(serialMutex);
    if (hSerial != INVALID_HANDLE_VALUE)
    {
        DWORD bytesWritten;
        WriteFile(hSerial, data, size, &bytesWritten, NULL);
    }
    else
    {
        Log("ERROR: Cannot send data, serial not connected.");
    }
}


// Helper function to wait for a specific response from the Pico
bool WaitForPicoResponse(const std::string& expectedMsg, const std::string& errorMsg, int timeoutSeconds)
{
    auto start = std::chrono::steady_clock::now();
    while (true) {
        {
            std::lock_guard<std::mutex> lock(msgMutex);
            // Check for both the message WITH or WITHOUT the full line ending
            size_t pos = serialMessages.find(expectedMsg);

            if (pos != std::string::npos) {
                // Consume the message by erasing everything up to and including the newline
                size_t endOfLine = serialMessages.find_first_of('\n', pos);

                if (endOfLine != std::string::npos) {
                    // Erase everything up to and including the line ending
                    serialMessages.erase(0, endOfLine + 1);
                }
                else {
                    // Fallback to clear the message if no newline is found
                    serialMessages.erase(0, pos + expectedMsg.length());
                }
                return true;
            }
        }

        // Timeout check
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(now - start).count() > timeoutSeconds) {
            Log(errorMsg + " (Timeout). Current buffer: " + serialMessages);
            return false;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}


// *** CRITICAL UPLOAD FUNCTION WITH ACK FLOW CONTROL ***
void UploadFile()
{
    OPENFILENAMEA ofn;
    char szFile[MAX_PATH] = { 0 };
    ZeroMemory(&ofn, sizeof(ofn));
    ofn.lStructSize = sizeof(ofn);
    ofn.lpstrFile = szFile;
    ofn.nMaxFile = sizeof(szFile);
    ofn.lpstrFilter = "All Files\0*.*\0";
    ofn.lpstrTitle = "Select File to Upload";
    ofn.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST;

    if (!GetOpenFileNameA(&ofn)) {
        Log("Upload canceled.");
        return;
    }

    std::ifstream file(szFile, std::ios::binary | std::ios::ate);
    if (!file) {
        Log("ERROR: Failed to open file.");
        return;
    }

    size_t filesize_s = (size_t)file.tellg();
    file.seekg(0, std::ios::beg);
    std::string filename = std::filesystem::path(szFile).filename().string();

    // Step 1: Send UPLOAD header
    std::stringstream cmd;
    cmd << "UPLOAD " << filename << " " << filesize_s << "\r\n";
    SendData(cmd.str().c_str(), static_cast<DWORD>(cmd.str().size()));

    // Flush transmit buffer and pause
    {
        std::lock_guard<std::mutex> lock(serialMutex);
        if (hSerial != INVALID_HANDLE_VALUE) {
            // CRITICAL: Clear both the transmit buffer (TX) and the receive buffer (RX)
            PurgeComm(hSerial, PURGE_TXCLEAR | PURGE_TXABORT | PURGE_RXCLEAR | PURGE_RXABORT);
        }
    }

    // Clear the application's shared message buffer now that the hardware buffer is clear
    {
        std::lock_guard<std::mutex> msgLock(msgMutex);
        serialMessages.clear();
    }


    Log("Sent upload command for " + filename);

    // Wait for a moment for the Pico to start parsing the command
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Step 2: Wait for Pico READY
    if (!WaitForPicoResponse(READY_MSG, "ERROR: Pico did not respond with READY", READY_TIMEOUT_SECONDS)) {
        return; // Abort upload on timeout or error
    }

    Log("Pico READY received, sending file with ACK flow control...");

    std::vector<char> buffer(BLOCK_SIZE);
    size_t sent = 0;
    bool success = true;
    size_t block_counter = 0;

    // Step 3: Send file in 512-byte chunks and wait for ACK after each
    while ((file.read(buffer.data(), BLOCK_SIZE) || file.gcount() > 0) && success) {
        size_t bytes = file.gcount();
        block_counter++;

        if (bytes > 0) {
            SendData(buffer.data(), static_cast<DWORD>(bytes));
            sent += bytes;

            // Only wait for ACK if we know more data is coming.
            if (sent < filesize_s) {
                // CRITICAL: Wait for ACK after sending a block
                if (!WaitForPicoResponse(ACK_MSG, "ERROR: Pico did not acknowledge block #" + std::to_string(block_counter), ACK_TIMEOUT_SECONDS)) {
                    success = false;
                }
            }
        }
    }
    file.close();

    if (!success) {
        Log("Upload aborted due to missing ACK.");
        return;
    }

    // Step 4: Wait for UPLOAD_OK from Pico 
    if (!WaitForPicoResponse(UPLOAD_OK_MSG, "ERROR: Pico did not confirm UPLOAD_OK", UPLOAD_OK_TIMEOUT_SECONDS)) {
        return;
    }

    Log("SUCCESS: File transfer complete. Total bytes sent: " + std::to_string(sent));
}

void PicoListenerThread()
{
    char buffer[512];
    DWORD bytesRead;
    bool wasConnected = false;
    const std::wstring portToFind = L"COM4"; // Constant for the target port

    while (true)
    {
        bool connectedNow = false;

        {
            std::lock_guard<std::mutex> lock(serialMutex);
            if (hSerial != INVALID_HANDLE_VALUE)
            {
                DWORD errors;
                COMSTAT status;
                // Check if the port is still alive/valid
                if (ClearCommError(hSerial, &errors, &status))
                {
                    connectedNow = true;
                }
                else
                {
                    CloseHandle(hSerial);
                    hSerial = INVALID_HANDLE_VALUE;
                }
            }
        }

        // Handle connection state change (Disconnected -> Log)
        if (!connectedNow && wasConnected)
        {
            Log("Pico disconnected on " + wstring_to_string(portToFind));
        }

        // Attempt reconnection
        if (!connectedNow)
        {
            if (OpenSerialPort(portToFind)) { // Uses the improved OpenSerialPort logic
                connectedNow = true;
            }
        }

        // Handle connection state change (Connected -> Log)
        if (connectedNow && !wasConnected)
        {
            Log("Pico connected on " + wstring_to_string(portToFind));
        }

        wasConnected = connectedNow;

        // Read incoming data and append to shared buffer
        if (connectedNow)
        {
            std::lock_guard<std::mutex> lock(serialMutex);

            if (hSerial != INVALID_HANDLE_VALUE)
            {
                if (ReadFile(hSerial, buffer, sizeof(buffer), &bytesRead, NULL) && bytesRead > 0)
                {
                    std::lock_guard<std::mutex> msgLock(msgMutex);
                    // CRITICAL: Ensure we only append the bytes read, not the whole buffer
                    serialMessages.append(buffer, bytesRead);
                }
            }
        }

        // --- NEW: Display the raw messages in the log ---
        FlushSerialMessagesToLog();

        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
}


void CreateUI(HWND hWnd)
{
    const int btnX = 20;
    const int btnY = 20;
    const int btnWidth = 160;
    const int btnHeight = 30;
    const int margin = 20;

    // Upload button
    CreateWindowW(L"BUTTON", L"Upload File to Pico",
        WS_VISIBLE | WS_CHILD | BS_DEFPUSHBUTTON,
        btnX, btnY, btnWidth, btnHeight,
        hWnd, (HMENU)IDC_UPLOAD, hInst, NULL);

    // Debug window
    hDebug = CreateWindowExW(WS_EX_CLIENTEDGE, L"EDIT", L"",
        WS_CHILD | WS_VISIBLE | WS_VSCROLL | ES_MULTILINE | ES_AUTOVSCROLL | ES_READONLY, // Added ES_READONLY
        btnX, btnY + btnHeight + 10, btnWidth, 130,
        hWnd, (HMENU)IDC_DEBUG, hInst, NULL);

    // Initial resize call (will be handled better by WM_SIZE later)
    RECT rcClient;
    GetClientRect(hWnd, &rcClient);
    int dbgX = btnX;
    int dbgY = btnY + btnHeight + 10;
    int dbgWidth = rcClient.right - dbgX - margin;
    int dbgHeight = rcClient.bottom - dbgY - margin;
    MoveWindow(hDebug, dbgX, dbgY, dbgWidth, dbgHeight, TRUE);

    // Start the Pico listener thread
    std::thread(PicoListenerThread).detach();
}

LRESULT CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
    switch (message)
    {
    case WM_SIZE:
    {
        // Resize the debug window when the main window size changes
        int winWidth = LOWORD(lParam);
        int winHeight = HIWORD(lParam);

        const int dbgX = 20;    // same left margin as button
        const int dbgY = 60;    // below the button
        const int dbgMargin = 20;
        const int dbgWidth = winWidth - dbgX - dbgMargin;
        const int dbgHeight = winHeight - dbgY - dbgMargin;

        if (hDebug) {
            MoveWindow(hDebug, dbgX, dbgY, dbgWidth, dbgHeight, TRUE);
        }
    }
    break;
    case WM_COMMAND:
    {
        int wmId = LOWORD(wParam);
        switch (wmId)
        {
        case IDM_ABOUT:
            DialogBox(hInst, MAKEINTRESOURCE(IDD_ABOUTBOX), hWnd, About);
            break;

        case IDM_EXIT:
            DestroyWindow(hWnd);
            break;

        case IDC_UPLOAD:
            // Run the upload in a separate thread to avoid freezing GUI
            std::thread([]() {
                UploadFile();
                }).detach();
            break;
        }
    }
    break;

    case WM_PAINT:
    {
        PAINTSTRUCT ps;
        HDC hdc = BeginPaint(hWnd, &ps);
        EndPaint(hWnd, &ps);
    }
    break;

    case WM_DESTROY:
        PostQuitMessage(0);
        break;

    default:
        return DefWindowProc(hWnd, message, wParam, lParam);
    }
    return 0;
}

INT_PTR CALLBACK About(HWND hDlg, UINT message, WPARAM wParam, LPARAM lParam)
{
    UNREFERENCED_PARAMETER(lParam);
    switch (message)
    {
    case WM_INITDIALOG: return (INT_PTR)TRUE;
    case WM_COMMAND:
        if (LOWORD(wParam) == IDOK || LOWORD(wParam) == IDCANCEL)
        {
            EndDialog(hDlg, LOWORD(wParam));
            return (INT_PTR)TRUE;
        }
        break;
    }
    return (INT_PTR)FALSE;
}
