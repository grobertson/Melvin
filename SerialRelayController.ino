//Serial controlled 8-relay bank

#define R1_PIN    2  //Pins 2-9 are relays 1-8
#define R2_PIN    3  //Pins 2-9 are relays 1-8
#define R3_PIN    4  //Pins 2-9 are relays 1-8
#define R4_PIN    5  //Pins 2-9 are relays 1-8
#define R5_PIN    6  //Pins 2-9 are relays 1-8
#define R6_PIN    7  //Pins 2-9 are relays 1-8
#define R7_PIN    8  //Pins 2-9 are relays 1-8
#define R8_PIN    9  //Pins 2-9 are relays 1-8


const byte numChars = 64;
char receivedChars[numChars];   // an array to store the received data

boolean newData = false;

void setup() {
    Serial.begin(9600);
    
    pinMode(R1_PIN, OUTPUT);
    pinMode(R2_PIN, OUTPUT);
    pinMode(R3_PIN, OUTPUT);
    pinMode(R4_PIN, OUTPUT);
    pinMode(R5_PIN, OUTPUT);
    pinMode(R6_PIN, OUTPUT);
    pinMode(R7_PIN, OUTPUT);
    pinMode(R8_PIN, OUTPUT);
}

void loop() {
    accumulateMessage();
    if (newData == true) {
        parseMessage();
    }
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

void parseMessage() {
    newData = false; //reset the flag
    if (receivedChars[3] == ':') { //Device class is three digits, so if this isn't a : the message is unparseable.
    
    }
    Serial.println(receivedChars);
}
