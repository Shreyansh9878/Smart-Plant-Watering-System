# 🌿 Smart Plant Monitoring & Watering System

An **IoT-based smart agriculture solution** designed to monitor plant health and automate irrigation using real-time sensor data. Built using affordable components, this system is ideal for home gardeners, small-scale farmers, and IoT learners.

## 🎯 Objective

To automate plant health monitoring and irrigation using low-cost IoT components, enabling **remote monitoring and control** through MQTT protocol.

---

## 🌟 Key Features

- **Automatic & Manual Motor Control** via MQTT  
- **Soil Moisture Monitoring** with intelligent irrigation  
- **Temperature & Humidity Tracking** using DHT11  
- **Nutrient Measurement (NPK)** via RS485/Modbus  
- **Motion Detection** with PIR sensors  
- **Event-Based Data Publishing** for power-efficient IoT design  
- **Remote Access** through MQTT-enabled client (e.g., mobile app, dashboard)  

---

## 🧰 Technologies Used

### Hardware
- ESP8266 NodeMCU
- Soil Moisture Sensors
- DHT11 (Temp & Humidity)
- PIR Motion Sensors
- RS485-based NPK Sensor
- Water Pump & Motor Driver (e.g., relay/MOSFET)
- Power Supply (for pump and NodeMCU)

### Software
- Arduino IDE
- MQTT Protocol
- PubSubClient (MQTT Library)
- ModbusMaster (for NPK sensor)
- Wi-Fi Connectivity

---

## 🔄 System Workflow

1. **Initialization**:  
   - ESP8266 connects to Wi-Fi and MQTT broker  
   - Publishes initial sensor states  

2. **Sensor Polling**:  
   - Periodically reads soil moisture, PIR, DHT11, and NPK sensor  

3. **Watering Logic**:  
   - Waters plant if soil moisture < threshold  
   - Stops watering otherwise  
   - Supports **manual control** via MQTT  

4. **Data Publishing**:  
   - Publishes only updated sensor data to reduce network load  
   - Sends MQTT notifications for events (watering, motion detected)  

5. **Fault Tolerance**:  
   - Automatic Wi-Fi and MQTT reconnection logic  

---

## 🧠 Architecture Overview

```
[ Sensors ] → ESP8266 NodeMCU → MQTT Broker ← Mobile App / Dashboard  
              ↓        ↑
          Motor Control Logic
```

**Sensors Include**:
- Moisture Sensors (Analog/Digital)  
- DHT11 (Temperature/Humidity)  
- NPK Sensor (Modbus RS485)  
- PIR Motion Sensor  

---

## ✅ Results & Learning

- Fully functional system with real-time data and automated irrigation  
- Gained experience with embedded IoT systems and wireless communication  
- Learned to implement MQTT protocols and sensor calibration in a modular design  

---

## 🚀 Future Scope

- **ML for Plant Health**: Image-based plant disease detection  
- **Precision Agriculture**: Auto-dosing of water/nutrients  
- **Cloud Dashboard**: Web/mobile app with analytics and reminders  
- **Battery + Solar Power** for remote, low-power operation  

---

## 📸 Demo / Images


