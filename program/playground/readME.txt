To build files in vscode

1. these parameters must be in the cMake file

cmake_minimum_required(VERSION 3.13)
set(PICO_BOARD pico_w)
set(WIFI_SSID "Your Network")
set(WIFI_PASSWORD "Your Password")
set(TEST_TCP_SERVER_IP "192.168.0.1")

2. in Vscode, go to CMake, Settings and add the PICO_SDK_PATH  as ..\..\pico-sdk

3. in vscode, CMake, Project Outline, click Configure on the CMakeLists.txt 
4. Then, click the .c filename (Executable)
5. Then go configure Build and selec the .c file name (NOT "all")
6. Then hit Build