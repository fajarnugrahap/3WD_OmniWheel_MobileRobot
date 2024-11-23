#include <Ps3Controller.h>
#include <HardwareSerial.h>

int cro, squ, tri, cir;
int xl, yl, xr, yr;
int str, slc, l1, l2, l3, r1, r2, r3, up, down, left, right, bat;

unsigned long interval = 4000;  // the time we need to wait
unsigned long previousMillis = 0, currentMillis;

char rx;

void notify() {
  if (Ps3.data.button.cross) {
    cro = 1;
    Serial.println("Pressing the cross button");
  } else cro = 0;

  if (Ps3.data.button.square) {
    squ = 1;
    Serial.println("Pressing the square button");
  } else squ = 0;

  if (Ps3.data.button.triangle) {
    tri = 1;
    Serial.println("Pressing the triangle button");
  } else tri = 0;

  if (Ps3.data.button.circle) {
    cir = 1;
    Serial.println("Pressing the circle button");
  } else cir = 0;

  xl = Ps3.data.analog.stick.lx;
  yl = Ps3.data.analog.stick.ly;
  xr = Ps3.data.analog.stick.rx;
  yr = Ps3.data.analog.stick.ry;

  if (Ps3.data.button.up) {
    up = 1;
    Serial.println("Pressing the D-Up button");
  } else up = 0;

  if (Ps3.data.button.down) {
    down = 1;
    Serial.println("Pressing the D-Down button");
  } else down = 0;

  if (Ps3.data.button.left) {
    left = 1;
    Serial.println("Pressing the D-Left button");
  } else left = 0;

  if (Ps3.data.button.right) {
    right = 1;
    Serial.println("Pressing the D-Right button");
  } else right = 0;

  if (Ps3.data.button.l1) {
    l1 = 1;
    Serial.println("Pressing the L1 button");
  } else l1 = 0;

  if (Ps3.data.button.l2) {
    l2 = 1;
    Serial.println("Pressing the L2 button");
  } else l2 = 0;

  if (Ps3.data.button.l3) {
    l3 = 1;
    Serial.println("Pressing the L3 button");
  } else l3 = 0;

  if (Ps3.data.button.r1) {
    r1 = 1;
    Serial.println("Pressing the R1 button");
  } else r1 = 0;

  if (Ps3.data.button.r2) {
    r2 = 1;
    Serial.println("Pressing the R2 button");
  } else r2 = 0;

  if (Ps3.data.button.r3) {
    r3 = 1;
    Serial.println("Pressing the R3 button");
  } else r3 = 0;

  if (Ps3.data.button.start) {
    str = 1;
    Serial.println("Pressing the Start button");
  } else str = 0;

  if (Ps3.data.button.select) {
    slc = 1;
    Serial.println("Pressing the Select button");
  } else slc = 0;
}

void onConnect() {
  Serial.println("Connected!.");
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(2000000, SERIAL_8N1, 16, 17);
  pinMode(2, OUTPUT);
  Ps3.attach(notify);
  Ps3.attachOnConnect(onConnect);
  Ps3.begin("12:07:02:00:00:00");
  Serial.println("Ready.");
}

void loop() {
  // if (cro == 1) digitalWrite(2, HIGH);
  // else digitalWrite(2, LOW);
  Serial2.write(12);
  Serial2.write(24);
  Serial2.write(36);
  Serial2.write(xl);
  Serial2.write(yl);
  Serial2.write(xr);
  Serial2.write(yr);
  Serial2.write(cro);
  Serial2.write(squ);
  Serial2.write(cir);
  Serial2.write(tri);
  delay(1);
}
