#include <AccelStepper.h>
#include <Encoder.h>
#include <U8g2lib.h>
#include <Wire.h>

/*
 * Kode for Bargellomaskinen
 * 
 * Denne koden demonstrerer grunnleggende funksjonalitet i maskinen, inkludert manuell mating av tekstil, 
 * justering av ønsket størrelse og antall kutt, og visning av valgt konfigurasjon på skjerm.
 * 
 * Følgende funksjoner er ennå ikke implementert:
 * - Automatisk beregning av hvor mange omdreininger tekstilrullen skal ha for å oppnå riktig stofflengde til brukeren
 * - Fullt automatisert kuttesekvens med flere kutt som utføres etter brukerens valg av størrelse og antall
 * 
 * Koden er laget som et funksjonelt testgrunnlag for videre utvikling av maskinens automatiserte prosesser.
 */

// === Skjerm ===
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0);

// Konfigurer knottene som styrer antall lapper som skal kuttes og størrelsen på hver lapp
Encoder numPatchesKnobEncoder(22, 24);
Encoder cutSizeKnobEncoder(28, 30);

const int selectNumPatchesButtonPin = 26;
const int selectCutSizeButtonPin = 32;

// === Knapper A8–A12 ===
#define ENABLE_PIN 8
#define startPin A8
#define stopPin A9
#define cutTextilePin A10
#define feedInPin A11
#define feedOutPin A12

// Konfigurer motorer og definer nyttige motorkonstanter
AccelStepper feederMotor(AccelStepper::DRIVER, 4, 7);  	// Mater tekstilet inn i maskinen
AccelStepper cutterMotor(AccelStepper::DRIVER, 2, 5);  	// Flytter kniven over tekstilet
AccelStepper holderMotor(AccelStepper::DRIVER, 3, 6);   // Flytter en arm som holder tekstilet på plass under kutting

const int numCutterSteps = 10800;  // Number of motor steps for moving the cutter from one end to the other
const int numHolderSteps = 750;    // Number of motor steps for moving the holder arm up and down

const float motorAcceleration = 1000.0;
const float motorMaxSpeed = 2000.0;

// === Quilte-meldinger når skjermen er i Standby-modus ===
const char* standbyMessages[] = {"Hello Bargello",       "In Stitches We Trust", "Ready to Quilt?",
                                 "Keep Calm & Quilt On", "Sewciopath at Work",   "I Came, I Sewed, I Conquered",
                                 "Snip Happens",         "Quilty Pleasure",      "Measure Twice, Cut Once",
                                 "Stitchin’ Time", "Snitches get Stitches"};
const uint8_t* fonts[] = {u8g2_font_ncenB24_tr, u8g2_font_ncenB14_tr, u8g2_font_courB18_tr,
                          u8g2_font_9x15B_tr,   u8g2_font_7x13B_tr,   u8g2_font_6x10_tr};
const int numMessages = sizeof(standbyMessages) / sizeof(standbyMessages[0]);
const int numFonts = sizeof(fonts) / sizeof(fonts[0]);

// === Tilstands-konfigurasjon ===
// Maskinen kan enten være i standby-modus, hvor brukeren kan konfigurere kuttemaskinen, eller aktiv
// Hvor maskinen kutter tekstil ihht konfigurasjonen
enum MainState { STANDBY, CONFIGURING_CUT_SIZE, CONFIGURING_NUM_PATCHES, READY, CUTTING };
MainState mainState = STANDBY;

enum HolderState { HOLDER_LIFTED, HOLDER_LOWERED };
HolderState holderState = HOLDER_LIFTED;

enum CutterState { CUTTER_LEFT, CUTTER_RIGHT };
CutterState cutterState = CUTTER_RIGHT;

long selectedNumPatches = 0;
long selectedCutSize = 0;
long selectedCutSizeIntegerPart = 0;
long selectedCutSizeFractionalPart = 0;

long previousNumPatchesEncoderValue = 0;
long previousCutSizeEncoderValue = 0;

long savedSelectedNumPatches = 0;
long savedCutSizeIntegerPart = 0;
long savedCutSizeFractionalPart = 0;
int previousNumPatchesButtonVoltage = HIGH;
int previousCutSizeButtonVoltage = HIGH;

int offset = 128, messageIndex = 0, fontIndex = 0;

void setup() {
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW);

  pinMode(startPin, INPUT_PULLUP);
  pinMode(feedInPin, INPUT_PULLUP);
  pinMode(feedOutPin, INPUT_PULLUP);
  pinMode(cutTextilePin, INPUT_PULLUP);
  pinMode(stopPin, INPUT_PULLUP);

  feederMotor.setMaxSpeed(motorMaxSpeed);
  feederMotor.setAcceleration(motorAcceleration);

  cutterMotor.setMaxSpeed(motorMaxSpeed);
  cutterMotor.setAcceleration(motorAcceleration);
  cutterMotor.setCurrentPosition(0);

  holderMotor.setMaxSpeed(motorMaxSpeed);
  holderMotor.setAcceleration(motorAcceleration);
  holderMotor.setCurrentPosition(0);  // Sett tekstilholder i oppe-posisjon for at det ikke krasjer i knivbladet

  u8g2.begin();
  pinMode(selectNumPatchesButtonPin, INPUT_PULLUP);
  pinMode(selectCutSizeButtonPin, INPUT_PULLUP);

  numPatchesKnobEncoder.write(0);
  cutSizeKnobEncoder.write(0);
  previousNumPatchesEncoderValue = numPatchesKnobEncoder.read();
  previousCutSizeEncoderValue = cutSizeKnobEncoder.read();
}

void showStandbyMessage() {
  u8g2.clearBuffer();
  u8g2.drawFrame(0, 0, 128, 64);
  const char* text = standbyMessages[messageIndex];
  const uint8_t* font = fonts[fontIndex];
  u8g2.setFont(font);
  int textWidth = u8g2.getStrWidth(text);
  int textHeight = u8g2.getMaxCharHeight();
  u8g2.setCursor(offset, (64 + textHeight) / 2 - 2);
  u8g2.print(text);
  u8g2.sendBuffer();
  offset -= 3;
  if (offset < -textWidth) {
    offset = 128;
    messageIndex = (messageIndex + 1) % numMessages;
    fontIndex = (fontIndex + 1) % numFonts;
  }
}

long updateValueFromEncoder(Encoder& encoder, long& previousEncoderReading, long currentValue) {
  long newReading = encoder.read();
  long change = newReading - previousEncoderReading;

// Ignorer små endringer som kan skyldes støy
  if (abs(change) >= 4) {
    // Normaliser endringen ved å kun øke/minske med 1.
    if (change > 0) {
      currentValue += 1;
    } else {
      currentValue -= 1;
    }

    // Sørg for at verdien ikke går under null
    if (currentValue < 0) {
      currentValue = 0;
    }
  }

  // Oppdater referansen til forrige avlesning fra encoder.
  previousEncoderReading = newReading;
  return currentValue;
}

bool encoderInputchanged(Encoder& encoder, long& previousEncoderReading, long currentValue) {
  /// Returner true hvis encoder-verdien har endret seg siden forrige oppdatering. 
  // //(Merk at currentValue ikke oppdateres, det sjekkes bare om den "ville" ha endret seg.)
  long newValue = updateValueFromEncoder(encoder, previousEncoderReading, currentValue);
  return newValue != currentValue;
}

bool buttonClicked(int buttonPin, int& previousVoltage) {
  // Returner true KUN HVIS knappen er trykket nå, men ikke var trykket forrige gang.
  // Oppdater også forrige spenningsnivå for knappen
  bool newVoltage = digitalRead(buttonPin);
  bool wasClicked = false;
  if (newVoltage == LOW && previousVoltage == HIGH) {
    wasClicked = true;
  }
  previousVoltage = newVoltage;
  return wasClicked;
}

void handleConfigureNumPatches() {
  long newSelectedNumPatches =
      updateValueFromEncoder(numPatchesKnobEncoder, previousNumPatchesEncoderValue, selectedNumPatches);
  if (selectedNumPatches != newSelectedNumPatches) {
    showSelectedConfiguration();
  }
  selectedNumPatches = newSelectedNumPatches;

  if (buttonClicked(selectNumPatchesButtonPin, previousNumPatchesButtonVoltage)) {
    savedSelectedNumPatches = selectedNumPatches;
    mainState = READY;
  }
}

void handleConfigureCutSize() {
  long newSelectedCutSize = updateValueFromEncoder(cutSizeKnobEncoder, previousCutSizeEncoderValue, selectedCutSize);
  if (selectedCutSize != newSelectedCutSize) {
    showSelectedConfiguration();
  }
  selectedCutSize = newSelectedCutSize;
  selectedCutSizeIntegerPart = selectedCutSize / 8;
  selectedCutSizeFractionalPart = selectedCutSize % 8;

  if (buttonClicked(selectCutSizeButtonPin, previousCutSizeButtonVoltage)) {
    savedCutSizeIntegerPart = selectedCutSizeIntegerPart;
    savedCutSizeFractionalPart = selectedCutSizeFractionalPart;
    mainState = READY;
  }
}

bool handleManualFeeding() {
  // Håndter manuell mating av tekstil inn/ut av maskinen.

  // MERK: Lav spenning (LOW) på knapp-pinnene betyr at knappen er trykket ned (koblet til jord)
  bool feedIn = digitalRead(feedInPin) == LOW;
  bool feedOut = digitalRead(feedOutPin) == LOW;

  if (feedIn && feedOut) {
    // Begge knapper trykket samtidig skal ikke gjøre noe
  } else if (feedIn) {
    feederMotor.setSpeed(motorMaxSpeed); 	// Mat tekstilet inn
    feederMotor.runSpeed();
  } else if (feedOut) {
    feederMotor.setSpeed(-motorMaxSpeed);  // Mat tekstilet ut
    feederMotor.runSpeed();
  }

  // Returner true hvis noen av knappene ble trykket
  return feedIn || feedOut;
}

void handleStartCutting() {
  bool pressedCutButton = digitalRead(cutTextilePin) == LOW;
  if (!pressedCutButton) {
    return;  // Knappen ble ikke trykket
  }

  // Det er egentlig ikke nødvendig å oppdatere mainState her, fordi kuttingen blokkerer all annen input fra brukeren.
  // Vi setter mainState til ACTIVE før kontroll returneres til brukeren.
  mainState = CUTTING;

  // Utfør kuttesekvens:
  // 1. Senk tekstilholderen.
  // 2. Kutt fra nåværende side til motsatt side (basert på knivens posisjon).
  // 3. Hev tekstilholderen.

  // Senk tekstilholderen hvis den er hevet (det bør den være nå!)
  if (holderState == HOLDER_LIFTED) {
    holderMotor.move(-numHolderSteps);
    while (holderMotor.distanceToGo() != 0) holderMotor.run();
    holderState = HOLDER_LOWERED;
  }

// Kjør kniven over tekstilet
  if (cutterState == CUTTER_RIGHT) {
    cutterMotor.move(-numCutterSteps);
    cutterState = CUTTER_LEFT;
  } else if (cutterState == CUTTER_LEFT) {
    cutterMotor.move(numCutterSteps);
    cutterState = CUTTER_RIGHT;
  }
  while (cutterMotor.distanceToGo() != 0) cutterMotor.run();

// Hev holderen hvis den er senket (det bør den være nå!)
  if (holderState == HOLDER_LOWERED) {
    holderMotor.move(numHolderSteps);
    while (holderMotor.distanceToGo() != 0) holderMotor.run();
    holderState = HOLDER_LIFTED;
  }

  mainState = READY;
}

void showSelectedConfiguration() {
  u8g2.clearBuffer();
  u8g2.drawFrame(0, 0, 128, 64);
  u8g2.setFont(u8g2_font_9x15_tr);

  u8g2.setCursor(5, 25);
  u8g2.print("Antall: ");
  u8g2.print(selectedNumPatches);
  u8g2.setCursor(5, 50);
  u8g2.print("Str: ");
  u8g2.print(selectedCutSizeIntegerPart);
  if (selectedCutSizeFractionalPart > 0) {
    u8g2.print(" ");
    const char* fractionStrings[] = {"", "1/8", "1/4", "3/8", "1/2", "5/8", "3/4", "7/8"};
    u8g2.print(fractionStrings[selectedCutSizeFractionalPart]);
  }
  u8g2.print("\"");

  u8g2.sendBuffer();
}

void loop() {
  switch (mainState) {
    case STANDBY:
      showStandbyMessage();
      // Sjekk om brukeren begynner å stille inn antall kutt eller størrelse.
      if (encoderInputchanged(numPatchesKnobEncoder, previousNumPatchesEncoderValue, selectedNumPatches)) {
        mainState = CONFIGURING_NUM_PATCHES;
      } else if (encoderInputchanged(cutSizeKnobEncoder, previousCutSizeEncoderValue, selectedCutSize)) {
        mainState = CONFIGURING_CUT_SIZE;
      }
      break;

    case CONFIGURING_NUM_PATCHES:
      handleConfigureNumPatches();
      break;

    case CONFIGURING_CUT_SIZE:
      handleConfigureCutSize();
      break;

    case READY:
      // Sjekk om brukeren begynner å stille inn antall kutt eller størrelse.
      if (encoderInputchanged(numPatchesKnobEncoder, previousNumPatchesEncoderValue, selectedNumPatches)) {
        mainState = CONFIGURING_NUM_PATCHES;
      } else if (encoderInputchanged(cutSizeKnobEncoder, previousCutSizeEncoderValue, selectedCutSize)) {
        mainState = CONFIGURING_CUT_SIZE;
        // ellers håndter alt annet
      } else {
        handleManualFeeding();
        handleStartCutting();
        showSelectedConfiguration();
      }
      break;

    case CUTTING:
      // CUTTING-tilstanden, slik den er implementert nå, er kun aktiv i handleStartCutting, og kan ikke stoppes før den er ferdig.
      // For å kunne stoppe midt i kuttingen må vi flytte noe av logikken fra handleStartCutting til denne delen.
      break;
  }
  delay(5);
}