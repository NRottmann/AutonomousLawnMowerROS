# ROS Autonomous Lawn Mower (under current development)
ROS Stack for the self-designed autonomous lawn mower of the University of Lübeck

## Voraussetzungen
* ROS Kinetic
  
## Requirements
* In `local.properties` den Pfad zum NDK eintragen, z.B. `ndk.dir=C\:\\Loomo\\android-ndk-r12b`
* In `app\CMakeLists.txt` den Pfad zu ROS C++ anpassen, z.B. `set(ROS_DIR C:/loomo/roscpp_android_ndk)` 
* IP-Adressen des ROS Core und des verwendeten Geräts in `native_library.cpp` (Anfang der Funktion `ROSMain`) setzen

## What does this project do:
* It enables the autonomous lawn mower to naviagte completely autonomously

## Testen (zB mit MATLAB R2018b)
```
roscore ...
```

