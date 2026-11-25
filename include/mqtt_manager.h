#pragma once
#include <Arduino.h>

void mqttPublish(const String& topic, const String& payload, bool retained = false);
void publishBridgeTelemetry(unsigned long now);
void publishExtractorTelemetry(unsigned long now);
void publishDevicesTelemetry(unsigned long now);
void publishAllMqttTelemetry(unsigned long now);
