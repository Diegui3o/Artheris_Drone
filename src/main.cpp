#include <WiFi.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include "motores.h"
#include "variables.h"
#include "mpu.h"
#include <Wire.h>
#include "piloto_mode.h"
#include "manual_mode.h"
#include <esp_task_wdt.h>

// ================= CONFIGURACIÓN =================
const char *ssid = "FAMILIAMYM";
const char *password = "mm221418";
const char *websocket_server = "192.168.1.56"; // IP de tu servidor Node.js
const int websocket_port = 3002;               // Puerto debe coincidir con el servidor
const char *websocket_path = "/esp32";

// Configuración IP fija
IPAddress local_IP(192, 168, 1, 200);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress primaryDNS(8, 8, 8, 8);
IPAddress secondaryDNS(8, 8, 4, 4);

// ================= VARIABLES =================
volatile bool ledState = false;
volatile bool motorState = false;
volatile int modoActual = 1;
volatile bool modoCambiado = false;

Preferences preferences;
TaskHandle_t TaskControl;
TaskHandle_t TaskComunicacion;

WebSocketsClient webSocket;
unsigned long lastConnectionAttempt = 0;
const long connectionInterval = 5000; // Intentar reconectar cada 5 segundos
unsigned long lastSendTime = 0;
const int sendInterval = 20; // ms

// ================= FUNCIONES =================
bool setup_wifi();
void webSocketEvent(WStype_t type, uint8_t *payload, size_t length);
void TaskControlCode(void *pvParameters);
void TaskComunicacionCode(void *pvParameters);
void prepareAndSendMessage();
void connectToWebSocket();
void changeMode(int newMode);
void processWebSocketMessage(const char *message);

bool setup_wifi()
{
    if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS))
    {
        Serial.println("Error al configurar IP estática");
        return false;
    }

    Serial.print("Conectando a ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);
    WiFi.setSleep(false); // Mejor estabilidad sin sleep

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 15)
    {
        delay(500);
        Serial.print(".");
        attempts++;
    }

    if (WiFi.status() != WL_CONNECTED)
    {
        Serial.println("\n❌ Fallo al conectar WiFi");
        return false;
    }

    Serial.println("\n✅ WiFi conectado. IP: " + WiFi.localIP().toString());
    return true;
}

void webSocketEvent(WStype_t type, uint8_t *payload, size_t length)
{
    switch (type)
    {
    case WStype_DISCONNECTED:
        Serial.println("❌ Desconectado del WebSocket");
        break;
    case WStype_CONNECTED:
        Serial.println("✅ Conectado al WebSocket");
        // Enviar mensaje de identificación
        webSocket.sendTXT("{\"type\":\"identify\",\"device\":\"esp32\"}");
        break;
    case WStype_TEXT:
        Serial.printf("📩 Mensaje recibido: %s\n", payload);
        // Procesar comandos recibidos
        processWebSocketMessage((const char *)payload);
        break;
    case WStype_ERROR:
        Serial.printf("❌ Error WebSocket: %s\n", payload);
        break;
    case WStype_PING:
        Serial.println("🛜 Ping recibido");
        break;
    case WStype_PONG:
        Serial.println("🛜 Pong recibido");
        break;
    }
}

void processWebSocketMessage(const char *message)
{
    DynamicJsonDocument doc(256);
    DeserializationError error = deserializeJson(doc, message);

    if (error)
    {
        Serial.print("❌ Error al parsear JSON: ");
        Serial.println(error.c_str());
        return;
    }

    // Procesar comando de modo
    if (doc.containsKey("mode"))
    {
        int newMode = doc["mode"];
        changeMode(newMode);
    }

    // Procesar comandos de control (nuevo formato)
    if (doc.containsKey("type") && strcmp(doc["type"], "command") == 0 && doc.containsKey("payload"))
    {
        JsonObject payload = doc["payload"];
        if (payload.containsKey("mode"))
        {
            int newMode = payload["mode"];
            if (newMode == 0)
                Serial.println("Modo piloto activado");
            else if (newMode == 2)
                Serial.println("Modo manual activado");
            else if (newMode == 1)
                Serial.println("Modo espera activado");
            changeMode(newMode);
        }
        if (payload.containsKey("led"))
        {
            ledState = payload["led"];
        }
        if (payload.containsKey("motors"))
        {
            motorState = payload["motors"];
        }
    }

    // Procesar comandos de control (formato antiguo)
    if (doc.containsKey("command"))
    {
        const char *command = doc["command"];
        if (strcmp(command, "ON_LED") == 0)
        {
            ledState = true;
        }
        else if (strcmp(command, "OFF_LED") == 0)
        {
            ledState = false;
        }
        else if (strcmp(command, "ON_MOTORS") == 0)
        {
            motorState = true;
        }
        else if (strcmp(command, "OFF_MOTORS") == 0)
        {
            motorState = false;
        }
    }
}

void connectToWebSocket()
{
    Serial.println("Intentando conectar WebSocket...");
    webSocket.begin(websocket_server, websocket_port, websocket_path);
    webSocket.onEvent(webSocketEvent);
    webSocket.setReconnectInterval(3000);
    webSocket.enableHeartbeat(15000, 3000, 2);
}

void TaskControlCode(void *pvParameters)
{
    esp_task_wdt_add(NULL);
    int lastMode = -1;
    for (;;)
    {
        esp_task_wdt_reset();

        digitalWrite(pinLed, ledState ? HIGH : LOW);
        motorState ? encenderMotores(1500) : apagarMotores();

        // Ejecutar setup solo si el modo cambió
        if (modoActual != lastMode)
        {
            lastMode = modoActual;
            switch (modoActual)
            {
            case 0: // Modo piloto
                Serial.println("Modo piloto activado");
                setup_pilote_mode();
                break;
            case 2: // Modo manual
                Serial.println("Modo manual activado");
                setup_manual_mode();
                break;
            case 1: // Modo espera
                Serial.println("Modo espera activado");
                // Aquí puedes agregar cualquier configuración específica para el modo espera
                break;
            default:
                // No hacer nada especial
                break;
            }
        }

        // Ejecutar loop solo si el modo es piloto o manual
        if (modoActual == 0 || modoActual == 2)
        {
            static uint32_t last_time = 0;
            float dt = (micros() - last_time) / 1e6;
            if (dt >= 0.002)
            {
                if (modoActual == 0)
                    loop_pilote_mode(dt);
                else if (modoActual == 2)
                    loop_manual_mode(dt);
                last_time = micros();
            }
        }
        // Si modoActual == 1, no se ejecuta ningún loop

        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

void TaskComunicacionCode(void *pvParameters)
{
    esp_task_wdt_add(NULL);
    for (;;)
    {
        esp_task_wdt_reset();

        // Manejar conexión WebSocket
        webSocket.loop();

        // Reconexión si es necesario
        if (!webSocket.isConnected() && millis() - lastConnectionAttempt > connectionInterval)
        {
            lastConnectionAttempt = millis();
            connectToWebSocket();
        }

        // Lectura de sensores
        gyro_signals();
        loop_yaw();

        // Envío periódico de datos
        unsigned long currentTime = millis();
        if (currentTime - lastSendTime >= sendInterval && webSocket.isConnected())
        {
            lastSendTime = currentTime;
            prepareAndSendMessage();
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void prepareAndSendMessage()
{
    DynamicJsonDocument doc(1024);
    doc["type"] = "telemetria";
    JsonObject payload = doc.createNestedObject("payload");

    payload["AngleRoll"] = AngleRoll_est;
    payload["AnglePitch"] = AnglePitch_est;
    payload["AngleYaw"] = AngleYaw;
    payload["RateRoll"] = gyroRateRoll;
    payload["RatePitch"] = gyroRatePitch;
    payload["RateYaw"] = RateYaw;
    payload["AccX"] = AccX;
    payload["AccY"] = AccY;
    payload["AccZ"] = AccZ;
    payload["tau_x"] = tau_x;
    payload["tau_y"] = tau_y;
    payload["tau_z"] = tau_z;
    payload["KalmanAngleRoll"] = AngleRoll;
    payload["KalmanAnglePitch"] = AnglePitch;
    payload["error_phi"] = error_phi;
    payload["error_theta"] = error_theta;
    payload["InputThrottle"] = InputThrottle;
    payload["InputRoll"] = DesiredAngleRoll;
    payload["InputPitch"] = DesiredAnglePitch;
    payload["InputYaw"] = DesiredRateYaw;
    payload["MotorInput1"] = MotorInput1;
    payload["MotorInput2"] = MotorInput2;
    payload["MotorInput3"] = MotorInput3;
    payload["MotorInput4"] = MotorInput4;
    payload["Altura"] = T;
    payload["modo"] = modoActual;

    String jsonString;
    serializeJson(doc, jsonString);
    webSocket.sendTXT(jsonString);
}

void changeMode(int newMode)
{
    if (newMode != modoActual)
    {
        modoActual = newMode;
        modoCambiado = true;
        preferences.putInt("modo", modoActual);
        Serial.printf("Modo cambiado a: %d\n", modoActual);
        if (modoActual == 0)
        {
            Serial.println("Modo piloto activado");
        }
        else if (modoActual == 2)
        {
            Serial.println("Modo manual activado");
        }
        else if (modoActual == 1)
        {
            Serial.println("Modo espera activado");
        }
    }
}

void setup()
{
    Serial.begin(115200);
    pinMode(pinLed, OUTPUT);
    digitalWrite(pinLed, HIGH); // Indicador de inicio

    preferences.begin("dronData", false);
    modoActual = preferences.getInt("modo", 1);
    ledState = preferences.getBool("ledState", false);

    setCpuFrequencyMhz(160);
    setupMPU();

    if (!setup_wifi())
    {
        Serial.println("Reiniciando por fallo WiFi...");
        delay(1000);
        ESP.restart();
    }

    btStop();
    connectToWebSocket();

    esp_task_wdt_init(10, true);

    xTaskCreatePinnedToCore(
        TaskControlCode,
        "TaskControl",
        10000,
        NULL,
        2,
        &TaskControl,
        0);

    xTaskCreatePinnedToCore(
        TaskComunicacionCode,
        "TaskComunicacion",
        10000,
        NULL,
        1,
        &TaskComunicacion,
        1);

    digitalWrite(pinLed, LOW);
    Serial.println("✅ Sistema inicializado correctamente");
}

void loop()
{
    delay(10); // El loop principal está vacío
}