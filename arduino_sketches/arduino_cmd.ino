/* Input pins */
const int IN1 = 2;
const int IN2 = 3;
const int IN3 = 5;
const int IN4 = 4;
/* Encoder pins */
const int EN1 = 9;
const int EN2 = 10;

void forward();
void backward();
void right();
void left();
void stop();
void increaseVel();
void decreaseVel();

int currentVel;
float velPer = 1;

void setup() {
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    pinMode(EN1, OUTPUT);
    pinMode(EN2, OUTPUT);

    currentVel = map(velPer, 0, 1, 205, 255);

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
            case 'q':
                decreaseVel();
                break;
            case 'e':
                increaseVel();
                break;
        }
    }
}

/* Forward action. */
void forward() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(EN1, currentVel);

    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(EN2, currentVel);
}

/* Left action. */
void left() {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(EN1, currentVel);

    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(EN2, currentVel);
}

/* Backward action. */
void backward() {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(EN1, currentVel);

    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(EN2, currentVel);
}

/* Right action. */
void right() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(EN1, currentVel);

    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(EN2, currentVel);
}

/* Stop action. */
void stop() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);

    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
}

/* Increase velocities. */
void increaseVel() {
    if (velPer < 1) {
        velPer += 0.1;

        currentVel = map(velPer, 0, 1, 205, 255);
    }
}

/* Decrease velocities. */
void decreaseVel() {
    if (velPer > 0) {
        velPer -= 0.1;

        currentVel = map(velPer, 0, 1, 205, 255);
    }
}