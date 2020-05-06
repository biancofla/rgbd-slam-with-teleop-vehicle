const int IN1 = 2;
const int IN2 = 3;
const int IN3 = 5;
const int IN4 = 4;

void forward();
void backward();
void right();
void left();
void stop();

void setup() {
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    Serial.begin(9600);
}

void loop() {
    if (Serial.available()) {
        char c = Serial.read();

        Serial.print(c);
        
        switch (c) {
            case 'w': 
                forward();
                break;
            case 'a': 
                left();
                break;
            case 's': 
                backward();
                break;
            case 'd': 
                right();
                break;
            case 'x': 
                stop();
                break;
        }
    }

}

void forward() {
    stop();

    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
}

void backward() {
    stop();

    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
}

void right() {
    stop();

    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
}

void left() {
    stop();

    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
}

void stop() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
}