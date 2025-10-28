# STM32F412ZG_Bootloader
# 🧠 STM32F412ZG UART Bootloader + User Application

A custom UART bootloader and user application built for the **STM32F412ZG (NUCLEO-F412ZG)** board.  
The project demonstrates safe in-field firmware updates, flash programming, and reliable app handover using UART.

---

## 🔹 Overview
- **Bootloader_UART** – Handles firmware update over UART3 (PD8/PD9 @115200 bps)  
- **UserApp_UART** – Simple application that blinks LED (PB0) and sends UART status messages  
- Implements flash erase/write, vector table relocation, and secure jump to user app  
- Includes range checking, idle timeout detection, and startup validation of stack pointer (SP) and program counter (PC)

---

## ⚙️ Hardware
| Component | Specification |
|------------|----------------|
| **Board** | NUCLEO-F412ZG |
| **MCU** | STM32F412ZGT6 (ARM Cortex-M4) |
| **Flash** | 1 MB |
| **SRAM** | 256 KB |
| **UART** | USART3 – PD8 (TX), PD9 (RX) |

---

## 🧩 Memory Map
| Section | Start Address | Size | Description |
|----------|----------------|------|-------------|
| **Bootloader** | 0x08000000 | 32 KB | Handles UART update & jump |
| **User App** | 0x08008000 | 960 KB | Runs main application |

---

## 🛠 How It Works
1. Bootloader initializes UART and prints startup message.  
2. Waits 10 seconds for `'U'` over UART.  
3. If `'U'` received → erases app sectors (2–11), receives binary file, writes to flash.  
4. On timeout → validates existing app and jumps to it.  
5. UserApp configures clocks, VTOR, GPIO, and UART.  
6. LED PB0 blinks and UART prints confirm successful execution.

---

## 💻 Example Log Output
[BL] Bootloader UART Hello World
[BL] Firmware receive complete. Total: 9084 bytes
[BL] Vector[0] (SP) = 0x20040000
[BL] Vector[1] (PC) = 0x08008A69
[BL] Jumping to user app...
[APP] User Application Running!
[APP] Alive


---

## 🧰 Tools & Technologies
- STM32CubeIDE  
- STM32 HAL Libraries (C)  
- Tera Term / Serial Terminal  
- ARM Cortex-M4  
- GitHub for Version Control  

---

## 📁 Repository Structure
Bootloader_UART/ # STM32CubeIDE bootloader project
UserApp_UART/ # STM32CubeIDE user application
firmware/ # Pre-compiled .bin files
docs/ # Screenshots, serial logs, diagrams
README.md # This file
LICENSE # MIT License


---

## 📄 License
This project is licensed under the **MIT License**.  
© 2025 Hariharan Ashok  

---

## 🔗 Connect
**LinkedIn:** [linkedin.com/in/hariharan-ashok](https://linkedin.com/in/hariharan-ashok)  
**Email:** hariiharan443@gmail.com
