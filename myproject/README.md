# Raspberry Pi Pico - asm330lhh Project

This README explains how to set up your development environment on macOS and how to flash firmware to your Raspberry Pi Pico.

⸻

# 1. Environment Setup on macOS

Follow these steps to install the required tools, import your project into VS Code, and build it.

## 1.1 Install picotool

brew install picotool

picotool provides command-line utilities for interacting with the Pico’s BOOTSEL and UF2 interfaces.

## 1.2 Install the Pico Extension for VS Code
	1.	Launch Visual Studio Code.
	2.	Open the Extensions view (⇧⌘X).
	3.	Search for “Raspberry Pi Pico” (or “Pico”) and install the official RP2040 extension.

## 1.3 Import the Project
	1.	In VS Code, open the Command Palette (⇧⌘P).
	2.	Run “Pico: Import CMake Project…”
	3.	Select the root folder of this repository.

## 1.4 Clean and Configure with CMake
	1.	Open the Command Palette (⇧⌘P).
	2.	Run “CMake: Clean Reconfigure” (or “Pico: Clean Workspace” if provided).
	3.	Wait for CMake to finish configuring the build files.

## 1.5 Build the Project

In the lower status bar or in the Command Palette, click “Compile Project” (provided by the Pico extension). The extension will invoke CMake and Ninja to produce the firmware ELF and UF2 files.

At this point, your build/ directory should contain the compiled .elf and .uf2 files.

⸻

# 2. Flashing Firmware to the Pico

Once the project is built, you can flash the generated firmware onto the Pico:
	1.	Reboot into BOOTSEL mode

picotool reboot -u -f

This forces the Pico into BOOTSEL mode over USB.

	2.	Load the ELF file

picotool load path/to/build/your_project.elf

Replace path/to/build/your_project.elf with the actual path to your .elf file.

	3.	Reboot to Application Space

picotool reboot

The board will exit BOOTSEL mode and start running your application.

⸻

You’re now ready to develop and test code on your Raspberry Pi Pico directly from VS Code!