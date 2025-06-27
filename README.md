# 🚚 LoRoute – Encrypted LoRa-Based Smart Sensor Network Protocol

**LoRoute** is a lightweight, custom-built communication protocol for LoRa, developed as part of a semester project to replace standard LoRaWAN in constrained setups.

We originally planned to use **LoRaWAN** with ChirpStack, but practical issues during development — especially the limitations of single-channel gateways — led us to create our own workaround protocol.

The result is a lightweight, minimalist imitation of LoRaWAN, tailored specifically to our use case.

---

## 🧰 Project Components

- **Endnode** (`Endnode.ino`)  
  This script is used on each unit placed inside the underground containers we worked with.  
  Each unit includes an Arduino with a Dragino LoRa Shield connected to sensors: weight, time-of-flight (ToF), and ultrasonic.  
  The device continuously gathers data but only transmits every 10 minutes to a gateway using our custom protocol.

  > **Note:**  
  > If an ACK (acknowledgement) is not received, it retries up to 3 times in total.

- **Gateway** (`Gateway.ino`)  
  This script runs on every gateway placed throughout the city.  
  It listens for packets from nearby Endnodes, decrypts them, validates them using CRC, and responds with either an ACK or a command (e.g. to change sensor mode).

- **SerialDB** (`SerialDB.py`)  
  Since our Arduino-based hardware had no working Wi-Fi or Ethernet, we needed a workaround.  
  We used a USB serial connection to an Internet-connected device (in our case, a laptop — though a Raspberry Pi or ESP32 would work too).  
  The script listens for specific serial output from the Gateway, parses the decrypted packet, rewrites the data to match our schema, and uploads it to **InfluxDB** over HTTPS.

---

## ▶️ How to Use

1. Flash and run the **Gateway** on an Arduino-compatible device.
2. Start the **SerialDB** script on an Internet-connected system to begin logging.
3. Power on the **Endnode** — and be happy!

---
