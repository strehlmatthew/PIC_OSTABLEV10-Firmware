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
#include <algorithm>
#include <cctype>
#include <SetupAPI.h>
#pragma comment(lib, "setupapi.lib")

// GLOBAL SYNCHRONIZATION OBJECTS
std::mutex serialMutex; // Protects hSerial and related serial calls (e.g., PurgeComm, ReadFile, WriteFile)
std::atomic<bool> protocolActive(false); // Flag: True when an upload/protocol sequence is in progress
std::mutex msgMutex;    // Protects serialMessages
std::string serialMessages; // Shared buffer for incoming serial data


#undef min
#define MAX_LOADSTRING 100
#define IDC_UPLOAD       101
#define IDC_DOWNLOAD     102
#define IDC_DEBUG        103

// --- PROTOCOL CONSTANTS (Exact search strings) ---
// Note: These must match the Pico's output EXACTLY, including \r\n if Pico adds it.
const std::string ACK_MSG = "ACK\r\n"; // Changed
const std::string READY_MSG = "READY\r\n"; // Changed
const std::string UPLOAD_OK_MSG = "UPLOAD_OK\r\n"; // Changed
const size_t BLOCK_SIZE = 512;
// --- TIMEOUT CONFIG ---
const int READY_TIMEOUT_SECONDS = 10;
const int ACK_TIMEOUT_SECONDS = 5;
const int UPLOAD_OK_TIMEOUT_SECONDS = 100;


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
void FlushSerialMessagesToLog(); // Function to display raw serial output
void UploadFile();
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

std::wstring GetDocumentsFolder()
{
    PWSTR pszPath = NULL;
    // Uses SHGetKnownFolderPath to get the path to the user's Documents folder
    if (SUCCEEDED(SHGetKnownFolderPath(FOLDERID_Documents, 0, NULL, &pszPath))) {
        std::wstring path = pszPath;
        CoTaskMemFree(pszPath);
        return path;
    }
    return L""; // Return empty string on failure
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
        // Correct usage of TranslateAccelerator
        if (!TranslateAccelerator(msg.hwnd, hAccelTable, &msg))
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
    // CRITICAL FIX: CS_VSCROLL removed here, as it caused a compile error.
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
    std::stringstream ss;
    SYSTEMTIME st;
    GetLocalTime(&st);
    ss << "[" << st.wHour << ":" << st.wMinute << ":" << st.wSecond << "] " << msg << "\r\n";

    // SendMessage is safe for cross-thread calls to the main thread's windows.
    SendMessageA(hDebug, EM_SETSEL, -1, -1);
    SendMessageA(hDebug, EM_REPLACESEL, FALSE, (LPARAM)ss.str().c_str());
    SendMessageA(hDebug, EM_SCROLLCARET, 0, 0);
}

// ----------------------------------------------------
// Flushes incoming serial messages to the log window line by line for clean display.
// ----------------------------------------------------
void FlushSerialMessagesToLog()
{
    // Only log if we are NOT in the middle of a file transfer protocol.
    if (protocolActive.load()) {
        return;
    }

    std::lock_guard<std::mutex> lock(msgMutex);

    size_t pos = 0;
    // Extract complete lines separated by '\n'
    while ((pos = serialMessages.find('\n')) != std::string::npos)
    {
        // Extract the line content (excluding the '\n')
        std::string line = serialMessages.substr(0, pos);

        // Remove preceding \r if present (typical Pico EOL is \r\n)
        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }

        // Use Log function to apply timestamp and final \r\n wrapper
        Log("[PICO] " + line);

        // Remove the processed line and the delimiter '\n' from the buffer
        serialMessages.erase(0, pos + 1);
    }
}
// ----------------------------------------------------
// Serial Port Setup: Implements Immediate Return Polling for reliable reading.
// ----------------------------------------------------
bool OpenSerialPort(const std::wstring& portName)
{
    std::lock_guard<std::mutex> lock(serialMutex);

    // Close the port if it was previously open but invalid
    if (hSerial != INVALID_HANDLE_VALUE) {
        CloseHandle(hSerial);
        hSerial = INVALID_HANDLE_VALUE;
    }

    // --- 1. Open the Port ---
    hSerial = CreateFileW(
        portName.c_str(),
        GENERIC_READ | GENERIC_WRITE,
        0, 0, OPEN_EXISTING,
        FILE_ATTRIBUTE_NORMAL, // Synchronous mode (Non-Overlapped)
        0
    );

    if (hSerial == INVALID_HANDLE_VALUE)
    {
        // Suppress repeated error logging if the port is just not present
        return false;
    }

    // --- 2. Configure DCB (Baud Rate, Data Format, and Flow Control) ---
    DCB dcb = { 0 };
    dcb.DCBlength = sizeof(DCB);

    if (!GetCommState(hSerial, &dcb)) {
        Log("ERROR: GetCommState failed.");
        CloseHandle(hSerial);
        hSerial = INVALID_HANDLE_VALUE;
        return false;
    }

    // Core Settings
    dcb.BaudRate = CBR_115200;
    dcb.ByteSize = 8;
    dcb.StopBits = ONESTOPBIT;
    dcb.Parity = NOPARITY;

    // CRITICAL FIX: Explicitly Enable DTR and RTS (recommended for Pico/CDC)
    dcb.fDtrControl = DTR_CONTROL_ENABLE;
    dcb.fRtsControl = RTS_CONTROL_ENABLE;

    // Disable ALL Flow Control Logic
    dcb.fOutxCtsFlow = FALSE;
    dcb.fOutxDsrFlow = FALSE;
    dcb.fInX = FALSE;
    dcb.fOutX = FALSE;

    // Other Flags
    dcb.fErrorChar = FALSE;
    dcb.fAbortOnError = FALSE;
    dcb.fBinary = TRUE;

    if (!SetCommState(hSerial, &dcb)) {
        Log("ERROR: SetCommState failed (Baud/Flow Control).");
        CloseHandle(hSerial);
        hSerial = INVALID_HANDLE_VALUE;
        return false;
    }

    // --- 3. Configure Timeouts ---
    // CRITICAL: Immediate Return Polling Mode
    COMMTIMEOUTS timeouts = { 0 }; // Initialization is safe here

    // ReadIntervalTimeout = MAXDWORD forces ReadFile to return immediately 
    // with whatever data is currently in the buffer (even if 0 bytes).
    timeouts.ReadIntervalTimeout = MAXDWORD;
    timeouts.ReadTotalTimeoutConstant = 0;
    timeouts.ReadTotalTimeoutMultiplier = 0;

    // Writes return quickly.
    timeouts.WriteTotalTimeoutConstant = 50;
    timeouts.WriteTotalTimeoutMultiplier = 0;

    if (!SetCommTimeouts(hSerial, &timeouts)) {
        Log("ERROR: SetCommTimeouts failed.");
        // This is not fatal, so we continue.
    }

    // --- 4. Final Cleanup ---
    // Clear any residual data in the buffers after opening.
    PurgeComm(hSerial, PURGE_RXCLEAR | PURGE_TXCLEAR);

    Log("INIT: Connected to Pico on " + wstring_to_string(portName));
    return true;
}

// ----------------------------------------------------
// Robust WriteFile error checking
// ----------------------------------------------------
void SendData(const char* data, DWORD size)
{
    // WriteFile must be protected by the serialMutex
    std::lock_guard<std::mutex> lock(serialMutex);
    if (hSerial != INVALID_HANDLE_VALUE)
    {
        DWORD bytesWritten = 0;
        // Explicitly check the result of WriteFile
        if (!WriteFile(hSerial, data, size, &bytesWritten, NULL))
        {
            DWORD error = GetLastError();
            // Log the failure to the main window
            Log("FATAL WRITE ERROR: WriteFile failed (Error " + std::to_string(error) + "). Cannot send data.");
        }
        else if (bytesWritten != size)
        {
            // Log a warning if a partial write occurred
            Log("WARNING: Sent only " + std::to_string(bytesWritten) + " of " + std::to_string(size) + " bytes. Potential buffer overflow.");
        }
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
            size_t pos = serialMessages.find(expectedMsg);

            if (pos != std::string::npos) {
                // Consume the message and anything leading up to it, ensuring the buffer is clear for the next message.
                // Erase from start of buffer (0) up to the end of the expected message.
                serialMessages.erase(0, pos + expectedMsg.length());
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
    // 1. Immediately SET the flag. This tells the listener thread to STOP logging data.
    protocolActive.store(true);

    // Use a try/finally or RAII style scope guard to ensure the flag is cleared on exit
    class ProtocolScopeGuard {
    public:
        ~ProtocolScopeGuard() {
            protocolActive.store(false);
            // After protocol ends, immediately flush any delayed messages
            FlushSerialMessagesToLog();
        }
    } guard;
    protocolActive.store(true);
    OPENFILENAMEA ofn;
    char szFile[MAX_PATH] = { 0 };
    // ... [File selection code remains the same] ...
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
    std::string command_str = cmd.str(); // Contains the essential trailing "\r\n"

    // 2. CRITICAL: Clear the buffer now, BEFORE the command is sent.
    {
        std::lock_guard<std::mutex> msgLock(msgMutex);
        serialMessages.clear();
    }

    // Attempt to send the command
    SendData(command_str.c_str(), static_cast<DWORD>(command_str.size()));

    // --- FIX FOR BLANK LINE ---
    // Create a copy of the command string to trim for logging only.
    std::string commandToLog = command_str;

    // Trim trailing whitespace/newlines from the log string
    commandToLog.erase(std::find_if(commandToLog.rbegin(), commandToLog.rend(), [](unsigned char ch) {
        return !std::isspace(ch);
        }).base(), commandToLog.end());

    // Log the trimmed command
    Log("Sent upload command: " + commandToLog);
    // --- END FIX ---

    // Clear the buffer again, just in case the Pico echo'd the command immediately
    {
        std::lock_guard<std::mutex> msgLock(msgMutex);
        serialMessages.clear();
    }

    // 3. CRITICAL: Wait immediately for the READY response.
    if (!WaitForPicoResponse(READY_MSG, "ERROR: Pico did not send READY", READY_TIMEOUT_SECONDS)) {
        Log("Upload aborted due to missing READY message.");
        return;
    }

    Log("Pico READY received, sending file with ACK flow control...");

    // ... [rest of the upload logic remains the same] ...
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

// Helper: Retrieves the Documents path, creates the "PicoLink Files" subfolder, and returns the path.
// This definition replaces any previous incomplete definition you may have had.
std::wstring GetPicoLinkFolder()
{
    // Ensure you have these includes at the top of your file:
    // #include <ShlObj.h> 
    // #include <filesystem> 

    PWSTR pszPath = NULL;
    std::wstring basePath;

    if (SUCCEEDED(SHGetKnownFolderPath(FOLDERID_Documents, 0, NULL, &pszPath))) {
        basePath = pszPath;
        CoTaskMemFree(pszPath);
    }
    else {
        Log("ERROR: Could not find Windows Documents folder path.");
        return L"";
    }

    std::wstring picoLinkPath = basePath + L"\\PicoLink Files";

    try {
        if (!std::filesystem::exists(picoLinkPath)) {
            std::filesystem::create_directories(picoLinkPath);
            Log("Created folder: " + wstring_to_string(picoLinkPath));
        }
    }
    catch (const std::filesystem::filesystem_error& e) {
        // FIX: Using 'e' to resolve the 'unreferenced local variable' warning
        Log("ERROR: Failed to create PicoLink folder: " + wstring_to_string(picoLinkPath) + ". Reason: " + e.what());
        return L"";
    }

    return picoLinkPath;
}

// ----------------------------------------------------
// CRITICAL: File Download Protocol Handler (Fixed Path and Data Types)
// ----------------------------------------------------
void HandleIncomingSend(const std::string& commandLine)
{
    // 1. Immediately SET the flag. This stops the listener thread from logging the file data.
    protocolActive.store(true);
    Log("Download initiated from Pico.");

    // Use a scope guard to ensure the flag is cleared on exit
    class ProtocolScopeGuard {
    public:
        ~ProtocolScopeGuard() {
            protocolActive.store(false);
            FlushSerialMessagesToLog();
            Log("Download session concluded.");
        }
    } guard;

    // --- Parse Command ---
    std::stringstream ss(commandLine);
    std::string command, filename_str;
    // FIX: Use size_t for file size to match standard C++ practice and remove warning
    size_t filesize_s = 0;

    // Example commandLine: "SEND A.TXT 1\r\n"
    if (!(ss >> command >> filename_str >> filesize_s) || command != "SEND") {
        Log("ERROR: Invalid SEND command format received.");
        return;
    }

    // --- Open Output File ---
    // CRITICAL FIX: Use the new GetPicoLinkFolder function
    std::wstring pico_folder_w = GetPicoLinkFolder();
    if (pico_folder_w.empty()) {
        Log("FATAL ERROR: Could not find or create PicoLink folder for saving.");
        return;
    }

    std::wstring filename_w = std::wstring(filename_str.begin(), filename_str.end());
    // Construct the full path: Documents\PicoLink Files\A.TXT
    std::wstring full_path_w = pico_folder_w + L"\\" + filename_w;

    std::ofstream outfile(full_path_w, std::ios::binary | std::ios::trunc);
    if (!outfile.is_open()) {
        Log("FATAL ERROR: Failed to open file for writing: " + wstring_to_string(full_path_w));
        return;
    }

    Log("Receiving file: " + filename_str + " (" + std::to_string(filesize_s) + " bytes) to: " + wstring_to_string(full_path_w));

    // --- Receive Data Loop ---
    // FIX: Use size_t for byte counter
    size_t bytesReceived = 0;

    // Safety timeout (e.g., 5 seconds for a small file)
    auto startTime = std::chrono::high_resolution_clock::now();
    const auto timeout = std::chrono::seconds(30);

    while (bytesReceived < filesize_s) {
        if (std::chrono::high_resolution_clock::now() - startTime > timeout) {
            Log("TIMEOUT: Download timed out before receiving all expected bytes.");
            break;
        }

        // Yield to listener thread to ensure the buffer is fed
        std::this_thread::sleep_for(std::chrono::milliseconds(5));

        std::lock_guard<std::mutex> lock(msgMutex);

        size_t available_data_in_buffer = serialMessages.length();
        size_t remaining_file_size = filesize_s - bytesReceived;

        size_t bytes_to_process = std::min(available_data_in_buffer, remaining_file_size);

        if (bytes_to_process > 0) {
            // Write the data chunk
            outfile.write(serialMessages.data(), bytes_to_process);
            bytesReceived += bytes_to_process;

            // Remove processed data from the buffer
            serialMessages.erase(0, bytes_to_process);
        }
    }

    outfile.close();

    // After receiving all expected bytes, consume the final END marker if present
    {
        std::lock_guard<std::mutex> lock(msgMutex);
        size_t end_pos = serialMessages.find("END\r\n");
        if (end_pos != std::string::npos) {
            serialMessages.erase(end_pos, 5); // 5 is for "END\r\n"
        }
    }

    if (bytesReceived == filesize_s) {
        Log("SUCCESS: File transfer complete. Bytes received: " + std::to_string(bytesReceived) + " to " + wstring_to_string(full_path_w));
    }
    else {
        Log("ERROR: File transfer incomplete. Expected " + std::to_string(filesize_s) + " bytes, got " + std::to_string(bytesReceived) + " bytes.");
    }
}
std::wstring FindPicoCOMPort()
{
    HDEVINFO hDevInfo = SetupDiGetClassDevs(NULL, L"USB", NULL, DIGCF_ALLCLASSES | DIGCF_PRESENT);
    if (hDevInfo == INVALID_HANDLE_VALUE) {
        return L"";
    }

    SP_DEVINFO_DATA devInfoData;
    devInfoData.cbSize = sizeof(SP_DEVINFO_DATA);
    DWORD i = 0;

    while (SetupDiEnumDeviceInfo(hDevInfo, i++, &devInfoData))
    {
        wchar_t buffer[512];
        if (SetupDiGetDeviceRegistryPropertyW(hDevInfo, &devInfoData, SPDRP_HARDWAREID, NULL, (PBYTE)buffer, sizeof(buffer), NULL))
        {
            // The Pico's hardware ID is "VID_2E8A&PID_000A"
            if (wcsstr(buffer, L"VID_2E8A&PID_000A") != NULL)
            {
                HKEY hKey = SetupDiOpenDevRegKey(hDevInfo, &devInfoData, DICS_FLAG_GLOBAL, 0, DIREG_DEV, KEY_READ);
                if (hKey != INVALID_HANDLE_VALUE)
                {
                    wchar_t portName[256];
                    DWORD size = sizeof(portName);
                    if (RegQueryValueExW(hKey, L"PortName", NULL, NULL, (LPBYTE)portName, &size) == ERROR_SUCCESS)
                    {
                        RegCloseKey(hKey);
                        SetupDiDestroyDeviceInfoList(hDevInfo);
                        return std::wstring(portName); // Found it! (e.g., L"COM3")
                    }
                    RegCloseKey(hKey);
                }
            }
        }
    }

    SetupDiDestroyDeviceInfoList(hDevInfo);
    return L""; // Pico not found
}
void PicoListenerThread()
{
    char buffer[512];
    DWORD bytesRead;
    bool wasConnected = false;
    std::wstring currentPortName = L"";

    while (true)
    {
        bool connectedNow = false;

        // --- Connection and Reconnection Logic (Now with Auto-Detection) ---
        {
            std::lock_guard<std::mutex> lock(serialMutex);
            if (hSerial != INVALID_HANDLE_VALUE)
            {
                DWORD errors;
                COMSTAT status;
                if (ClearCommError(hSerial, &errors, &status)) {
                    connectedNow = true;
                }
                else {
                    CloseHandle(hSerial);
                    hSerial = INVALID_HANDLE_VALUE;
                }
            }
        }

        if (!connectedNow) {
            // If disconnected, try to find the Pico's COM port
            std::wstring foundPort = FindPicoCOMPort();
            if (!foundPort.empty()) {
                if (OpenSerialPort(foundPort)) {
                    connectedNow = true;
                    if (currentPortName != foundPort) {
                        Log("Pico detected on " + wstring_to_string(foundPort));
                        currentPortName = foundPort;
                    }
                }
            }
            else {
                if (!currentPortName.empty()) {
                    Log("Pico disconnected.");
                    currentPortName = L"";
                }
            }
        }

        bool protocolInitiatedThisCycle = false;

        // Read incoming data and append to shared buffer
        if (connectedNow)
        {
            std::lock_guard<std::mutex> lock(serialMutex);
            if (hSerial != INVALID_HANDLE_VALUE)
            {
                DWORD errors;
                COMSTAT status;
                ClearCommError(hSerial, &errors, &status);

                BOOL readOK = ReadFile(hSerial, buffer, sizeof(buffer), &bytesRead, NULL);

                if (readOK && bytesRead > 0)
                {
                    std::string raw(buffer, bytesRead);

                    // --- SUPPRESS ACK/NACK RAW LOGGING ---
                    bool shouldLogRaw = true;
                    if (protocolActive.load()) {
                        if (bytesRead == 5) {
                            if (raw == "ACK\r\n" || raw == "NACK\r\n") {
                                shouldLogRaw = false;
                            }
                        }
                    }

                    if (shouldLogRaw) {
                        std::string rawLog = raw;
                        rawLog.erase(std::find_if(rawLog.rbegin(), rawLog.rend(), [](unsigned char ch) {
                            return !std::isspace(ch);
                            }).base(), rawLog.end());
                        rawLog.erase(rawLog.begin(), std::find_if(rawLog.begin(), rawLog.end(), [](unsigned char ch) {
                            return !std::isspace(ch);
                            }));
                        Log("[RAW RECV: " + std::to_string(bytesRead) + " bytes] " + rawLog);
                    }

                    std::lock_guard<std::mutex> msgLock(msgMutex);
                    serialMessages.append(buffer, bytesRead);
                }
                else if (!readOK && GetLastError() != ERROR_IO_PENDING)
                {
                    Log("FATAL SERIAL READ ERROR (Error: " + std::to_string(GetLastError()) + "). Forcing reconnection.");
                    CloseHandle(hSerial);
                    hSerial = INVALID_HANDLE_VALUE;
                }
            }
        }

        // --- Download Initiation Block ---
        {
            std::lock_guard<std::mutex> msgLock(msgMutex);
            if (!protocolActive.load()) {
                size_t send_pos = serialMessages.find("SEND ");
                size_t eol_pos = serialMessages.find('\n', send_pos);
                if (send_pos != std::string::npos && eol_pos != std::string::npos) {
                    std::string command_line = serialMessages.substr(send_pos, eol_pos - send_pos + 1);
                    serialMessages.erase(send_pos, eol_pos - send_pos + 1);
                    Log("[PICO COMMAND] " + command_line);
                    std::thread(HandleIncomingSend, command_line).detach();
                    protocolInitiatedThisCycle = true;
                }
            }
        }

        if (!protocolInitiatedThisCycle) {
            FlushSerialMessagesToLog();
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(20)); // Polling delay
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
        WS_CHILD | WS_VISIBLE | WS_VSCROLL | ES_MULTILINE | ES_AUTOVSCROLL | ES_READONLY,
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
        const int dbgMargin = 20; // Define the constant for resizing margins
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
