# Quadcopter Control y Monitoreo con ESP32

Este proyecto implementa un sistema de control y monitoreo para un quadcopter utilizando un **ESP32**. Utiliza tecnologías como WiFi, MQTT y sensores MPU para comunicar y transmitir datos en tiempo real.

## Características

- **Control de LED**: Enciende y apaga un LED desde comandos MQTT.
- **Monitoreo de Sensores**: Publica información de sensores como ángulos, tasas, aceleración y giroscopio.
- **Soporte para JSON**: Recibe mensajes de control en formato JSON o texto.
- **Almacenamiento Persistente**: Guarda y restaura estados utilizando la librería `Preferences`.
- **Conexión WiFi**: Se conecta automáticamente a la red WiFi especificada.
- **Reconexion a MQTT**: Reintenta conectarse al servidor MQTT automáticamente si la conexión se pierde.

## Requisitos

- **Hardware**:
  - ESP32
  - Módulo MPU6050 u otro compatible
  - LED (opcional)
- **Software**:
  - Arduino IDE (o plataforma equivalente)
  - Librerías de Arduino:
    - `WiFi.h`
    - `PubSubClient.h`
    - `ArduinoJson.h`
    - `Preferences.h`

## Configuración

1. **Clonar este repositorio**:
   ```bash
   git clone https://github.com/Diegui3o/mqtt_dron.git
   cd repositorio
