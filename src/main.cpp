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

SemaphoreHandle_t sensorMutex = NULL;

// ================= CONFIGURACIÓN =================
const char *ssid = "FAMILIAMYM";
const char *password = "mm221418";
const char *websocket_server = "192.168.1.19";
const int websocket_port = 3003;
const char *websocket_path = "/esp32";

// Configuración IP fija
IPAddress local_IP(192, 168, 1, 200); // IP fija para el ESP32 en la red Wi-Fi
IPAddress gateway(192, 168, 1, 1);    // Puerta de enlace de tu red Wi-Fi
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
const int sendInterval = 6;
char txBuffer[400];
#define BUFFER_SIZE 80
struct SensorData
{
    float roll, pitch, yaw;
} sensorBuffer[BUFFER_SIZE];
int bufferIndex = 0;
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
    case WStype_TEXT:
    {
        // Asumimos que los comandos siguen en JSON
        char *message = (char *)payload;
        if (message[0] == '{')
        { // Es JSON
            processWebSocketMessage(message);
        }
        break;
    }
    case WStype_ERROR:
        Serial.printf("❌ Error WebSocket: %s\n", payload);
        break;
    case WStype_PING:
        Serial.println("🛜 Ping recibido");
        break;
    case WStype_PONG:
        Serial.println("🛜 Pong recibido");
        break;
    default:
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
        if (modoActual != lastMode)
        {
            lastMode = modoActual;
            switch (modoActual)
            {
            case 0:                         // Modo piloto
                digitalWrite(pinLed, HIGH); // Indicador de inicio
                Serial.println("Modo piloto activado");
                setup_pilote_mode();
                digitalWrite(pinLed, LOW); // Indicador de inicio
                break;
            case 2:                         // Modo manual
                digitalWrite(pinLed, HIGH); // Indicador de inicio
                Serial.println("Modo manual activado");
                channelInterrupHandler();
                setup_manual_mode();
                digitalWrite(pinLed, LOW); // Indicador de inicio
                break;
            case 1: // Modo espera
                Serial.println("Modo espera activado");
                apagarMotores();
                break;
            default:
                break;
            }
        }
        // Lectura de sensores SIEMPRE ACTIVA a 25Hz independiente del modo
        {
            static uint32_t sensor_last_time = 0;
            float sensor_dt = (micros() - sensor_last_time) / 1e6;
            if (sensor_dt >= 0.001)
            {
                if (xSemaphoreTake(sensorMutex, 0) == pdTRUE) // No bloquear
                {
                    gyro_signals(); // Leer sensores siempre a 25Hz
                    xSemaphoreGive(sensorMutex);
                }
                sensor_last_time = micros();
            }
        }

        // Ejecutar loop solo si el modo es piloto o manual
        if (modoActual == 0 || modoActual == 2)
        {
            static uint32_t control_last_time = 0;
            float control_dt = (micros() - control_last_time) / 1e6;
            if (control_dt >= 0.001)
            {
                if (xSemaphoreTake(sensorMutex, 0) == pdTRUE) // No bloquear
                {
                    if (modoActual == 0)
                        loop_pilote_mode(control_dt);
                    else if (modoActual == 2)
                        loop_manual_mode(control_dt);
                    xSemaphoreGive(sensorMutex);
                }
                control_last_time = micros();
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void TaskComunicacionCode(void *pvParameters)
{
    esp_task_wdt_add(NULL);
    for (;;)
    {
        esp_task_wdt_reset();
        webSocket.loop();

        // Reconexión si es necesario
        if (!webSocket.isConnected() && millis() - lastConnectionAttempt > connectionInterval)
        {
            lastConnectionAttempt = millis();
            connectToWebSocket();
        }

        // Envío de datos por WebSocket (periódico, independiente)
        if (webSocket.isConnected() && millis() - lastSendTime >= sendInterval)
        {
            prepareAndSendMessage();
            lastSendTime = millis();
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void prepareAndSendMessage()
{
    String msg = String(millis()) + "," +
                 String(AngleRoll_est, 3) + "," +
                 String(AnglePitch_est, 3) + "," +
                 String(AngleYaw, 3) + "," +
                 String(gyroRateRoll, 3) + "," +
                 String(gyroRatePitch, 3) + "," +
                 String(RateYaw, 3) + "," +
                 String(AccX, 3) + "," +
                 String(AccY, 3) + "," +
                 String(AccZ, 3) + "," +
                 String(tau_x, 3) + "," +
                 String(tau_y, 3) + "," +
                 String(tau_z, 3) + "," +
                 String(AngleRoll, 3) + "," +
                 String(AnglePitch, 3) + "," +
                 String(error_phi, 3) + "," +
                 String(error_theta, 3) + "," +
                 String(InputThrottle) + "," +
                 String(DesiredAngleRoll, 3) + "," +
                 String(DesiredAnglePitch, 3) + "," +
                 String(DesiredRateYaw, 3) + "," +
                 String(MotorInput1, 3) + "," +
                 String(MotorInput2, 3) + "," +
                 String(MotorInput3, 3) + "," +
                 String(MotorInput4, 3) + "," +
                 String(T, 3) + "," +
                 String(modoActual);
    webSocket.sendTXT(msg);
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
    modoActual = 1;                         // Siempre iniciar en modo espera
    preferences.putInt("modo", modoActual); // Guardar modo espera en la memoria
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

    esp_task_wdt_init(30, true); // Aumentar tiempo del watchdog a 30 segundos

    // Crear el mutex antes de iniciar las tareas
    sensorMutex = xSemaphoreCreateMutex();
    setupMotores();
    xTaskCreatePinnedToCore(
        TaskControlCode,
        "TaskControl",
        10000,
        NULL,
        2, // Prioridad más alta para el control
        &TaskControl,
        0); // Núcleo 0 para control en tiempo real

    xTaskCreatePinnedToCore(
        TaskComunicacionCode,
        "TaskComunicacion",
        15000,
        NULL,
        1, // Prioridad menor para comunicación
        &TaskComunicacion,
        1); // Núcleo 1 para comunicación

    digitalWrite(pinLed, LOW);
    Serial.println("✅ Sistema inicializado correctamente");
}

void loop()
{
}