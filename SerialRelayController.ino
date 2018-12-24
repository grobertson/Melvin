//Serial controlled 8-relay bank

#define R1_PIN    2  //Pins 2-10 are relays 1-8
#define R2_PIN    3  //Pins 2-10 are relays 1-8
#define R3_PIN    4  //Pins 2-10 are relays 1-8
#define R4_PIN    5  //Pins 2-10 are relays 1-8
#define R5_PIN    6  //Pins 2-10 are relays 1-8
#define R6_PIN    7  //Pins 2-10 are relays 1-8
#define R7_PIN    8  //Pins 2-10 are relays 1-8
#define R8_PIN    9  //Pins 2-10 are relays 1-8


const byte numChars = 64;
char receivedChars[numChars];   // an array to store the received data

boolean newData = false;

void setup() {
    Serial.begin(9600);
}

void loop() {
    accumulateMessage();
    setRelayState();
}

void accumulateMessage() {
    static byte ndx = 0;
    char endMarker = ';';
    char rc;
    
    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (rc != endMarker) {
            receivedChars[ndx] = rc;
            ndx++;
            if (ndx >= numChars) {
                ndx = numChars - 1;
            }
        }
        else {
            receivedChars[ndx] = '\0'; // terminate the string
            ndx = 0;
            newData = true;
        }
    }
}

void setRelayState() {
    if (newData == true) {
        Serial.println(receivedChars);
        newData = false;
    }
}
