#include "Bluetooth.h"
#include <SoftwareSerial.h>

SoftwareSerial BTSerial(2, 3); // Pines RX, TX

void Bluetooth::iniciarBluetooth() {
    BTSerial.begin(9600); // Velocidad de comunicación Bluetooth
}

String Bluetooth::recibirComando() {
    if (BTSerial.available()) {
        return BTSerial.readStringUntil('\n'); // Leer hasta salto de línea
    }
    return "";
}
