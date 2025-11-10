idf_component_register(
    SRCS
        "src/ESP32PWM.cpp"
        "src/ESP32Servo.cpp"
    INCLUDE_DIRS
        "src"
    REQUIRES
        espressif__arduino-esp32
)

set_source_files_properties(
    ${COMPONENT_DIR}/src/ESP32PWM.cpp
    ${COMPONENT_DIR}/src/ESP32Servo.cpp
    PROPERTIES LANGUAGE CXX
)