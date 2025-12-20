# Quick Start: Installing Blynk Library

## Install Blynk Library via Arduino IDE

1. **Open Arduino IDE**
2. **Go to Library Manager**:
   - Click `Sketch` → `Include Library` → `Manage Libraries...`
   - Or use shortcut: `Ctrl+Shift+I` (Windows) / `Cmd+Shift+I` (Mac)

3. **Search for Blynk**:
   - In the search box, type: `Blynk`
   - Look for **"Blynk"** by Volodymyr Shymanskyy

4. **Install**:
   - Click on the library entry
   - Click the **Install** button
   - Wait for installation to complete

5. **Verify Installation**:
   - Go to `Sketch` → `Include Library`
   - You should see "Blynk" in the list

## Alternative: Manual Installation

If Library Manager doesn't work:

1. Download from GitHub: https://github.com/blynkkk/blynk-library/releases
2. Download the latest `.zip` file
3. In Arduino IDE: `Sketch` → `Include Library` → `Add .ZIP Library...`
4. Select the downloaded ZIP file

## After Installation

Once installed, try compiling your sketch again. The `BlynkSimpleWiFi.h` error should be resolved.

## Still Getting Errors?

If you still see library errors after installation, you may need to check:
1. Arduino IDE version (use 1.8.19 or newer, or Arduino IDE 2.x)
2. Board package for Arduino UNO Q is properly installed
3. Restart Arduino IDE after installing libraries
