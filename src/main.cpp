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
const char *websocket_server = "192.168.1.9";
const int websocket_port = 3003;
const char *websocket_path = "/esp32";

// Configuración IP fija
IPAddress local_IP(192, 168, 0, 200); // IP fija para el ESP32 en la red Wi-Fi
IPAddress gateway(192, 168, 0, 1);    // Puerta de enlace de tu red Wi-Fi
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
const int sendInterval = 50; // 50ms (20Hz) - Reducir frecuencia para evitar congestión
char txBuffer[308];          // Buffer para mensajes CSV;
#define BUFFER_SIZE 50       // Reducir buffer para mejor manejo de memoria
struct SensorData
{
    float roll, pitch, yaw;
    unsigned long timestamp;
} sensorBuffer[BUFFER_SIZE];
int bufferIndex = 0;
bool bufferFull = false;
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
    webSocket.setReconnectInterval(5000);      // Aumentar intervalo de reconexión
    webSocket.enableHeartbeat(30000, 5000, 3); // Heartbeat menos frecuente

    // Configurar buffer más pequeño para envíos
    webSocket.enableHeartbeat(30000, 5000, 2);
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
        // Ejecutar loop solo si el modo es piloto o manual
        if (modoActual == 0 || modoActual == 2)
        {
            static uint32_t last_time = 0;
            float dt = (micros() - last_time) / 1e6;
            if (dt >= 0.01) // 100Hz para control - más frecuente que comunicación
            {
                if (xSemaphoreTake(sensorMutex, 5 / portTICK_PERIOD_MS) == pdTRUE) // Timeout más corto
                {
                    if (modoActual == 0)
                        loop_pilote_mode(dt);
                    else if (modoActual == 2)
                        loop_manual_mode(dt);
                    xSemaphoreGive(sensorMutex);
                }
                last_time = micros();
            }
        }
        vTaskDelay(2 / portTICK_PERIOD_MS); // Aumentar delay para dar más tiempo a otras tareas
    }
}

void TaskComunicacionCode(void *pvParameters)
{
    esp_task_wdt_add(NULL);
    unsigned long lastSensorRead = 0;
    const int sensorReadInterval = 5; // Leer sensores cada 5ms

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

        // Lectura de sensores con menor frecuencia y timeout
        unsigned long currentTime = millis();
        if (currentTime - lastSensorRead >= sensorReadInterval)
        {
            lastSensorRead = currentTime;
            if (xSemaphoreTake(sensorMutex, 3 / portTICK_PERIOD_MS) == pdTRUE) // Timeout más corto
            {
                loop_yaw();
                xSemaphoreGive(sensorMutex);
            }
        }

        // Envío de datos a intervalo fijo más bajo
        if (currentTime - lastSendTime >= sendInterval)
        {
            lastSendTime = currentTime;
            if (webSocket.isConnected())
            {
                prepareAndSendMessage();
            }
        }

        vTaskDelay(3 / portTICK_PERIOD_MS); // Aumentar delay para mejor distribución de CPU
    }
}

void prepareAndSendMessage()
{
    // Verificar si hay suficiente memoria antes de enviar
    if (ESP.getFreeHeap() < 10000)
    {
        Serial.println("⚠️ Memoria baja, saltando envío");
        return;
    }

    // Usar snprintf para mejor control de memoria
    int length = snprintf(txBuffer, sizeof(txBuffer),
                          "%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d",
                          millis(),
                          AngleRoll_est,
                          AnglePitch_est,
                          AngleYaw,
                          gyroRateRoll,
                          gyroRatePitch,
                          RateYaw,
                          AccX,
                          AccY,
                          AccZ,
                          tau_x,
                          tau_y,
                          tau_z,
                          AngleRoll,
                          AnglePitch,
                          error_phi,
                          error_theta,
                          InputThrottle,
                          DesiredAngleRoll,
                          DesiredAnglePitch,
                          DesiredRateYaw,
                          MotorInput1,
                          MotorInput2,
                          MotorInput3,
                          MotorInput4,
                          T,
                          modoActual);

    if (length > 0 && length < sizeof(txBuffer))
    {
        webSocket.sendTXT(txBuffer, length);
    }
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

    setCpuFrequencyMhz(240); // Aumentar frecuencia de CPU para mejor rendimiento
    setupMPU();
    if (!setup_wifi())
    {
        Serial.println("Reiniciando por fallo WiFi...");
        delay(1000);
        ESP.restart();
    }
    // Test TCP solo después de WiFi OK
    WiFiClient testClient;
    if (testClient.connect(websocket_server, websocket_port))
    {
        Serial.println("✅ Conexión TCP exitosa al backend");
        testClient.stop();
    }
    else
    {
        Serial.println("❌ No se pudo conectar por TCP al backend");
    }
    btStop();

    // Configurar WebSocket antes de crear tareas
    connectToWebSocket();

    // Configurar watchdog con más tiempo
    esp_task_wdt_init(30, true); // Aumentar tiempo del watchdog a 30 segundos

    // Crear el mutex antes de iniciar las tareas
    sensorMutex = xSemaphoreCreateMutex();
    setupMotores();

    xTaskCreatePinnedToCore(
        TaskControlCode,
        "TaskControl",
        12000, // Aumentar stack size
        NULL,
        3, // Prioridad más alta para el control
        &TaskControl,
        0); // Núcleo 0 para control en tiempo real

    xTaskCreatePinnedToCore(
        TaskComunicacionCode,
        "TaskComunicacion",
        18000, // Aumentar stack size
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