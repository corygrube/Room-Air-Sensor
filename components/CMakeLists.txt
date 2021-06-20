set(SOURCES ./esp32-DHT11/dht11.c)

idf_component_register(
    SRCS ${SOURCES}
    INCLUDE_DIRS ./esp32-DHT11/include
)
