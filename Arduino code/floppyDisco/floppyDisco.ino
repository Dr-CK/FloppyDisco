/*
 * FLOPPY DISCO
 * 
 * Christoffer Karlsson 2021
 * 
 * Read MIDI messages with an Arduino and play the notes on a floppy drive.
 * 
 * Depending on the current note, a timer is triggered performing a motor step 
 * every half wavelength, creating the correct note. (Every note is transposed
 * by one octave to sound a bit better on the floppy drive.)
 * 
 * Only the latest pressed key will be played, ignoring any others pressed
 * simultaneously.
 * 
 * Pitch bending is also implemented, but no other MIDI messages. Also the
 * velocity of the key press is ignored (except for 0 = note off).
 * 
 * The current read head position is tracked and the step direction changed
 * accordingly, trying to minimize the amount of direction switching necessary.
 * 
 */

#include "TimerOne.h" //Timer library

//Pins controlling floppy drive, connect to the corresponding floopy drive pin
#define SELECTPIN 6
#define DIRPIN 7
#define STEPPIN 8

#define TRACKS 160 //Standard number of tracks on a floppy disk

//MIDI messages
#define NOTE_ON 0B10010000
#define NOTE_OFF 0B10000000
#define PITCH_BEND 0B11100000
#define MASK_CHANNEL 0B11110000 //Status message bit mask
#define PITCH_BEND_OFFSET 0x2000 //Center = no pitch bend

//Keeps track of current status
int track, //Current track number along the floppy disk
  currentNote = 0, //Current note being played (0=none)
  pitchBendValue = PITCH_BEND_OFFSET; //Current pitch bend value
//STEP and DIR values written to the floppy drive controller
bool stepVal = LOW, dir; //Alternating STEP performs a step in the direction DIR

byte statusByte = 0; //Current MIDI status message

//Wavelength for each note divided by two
//int frequency[128] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 31, 33, 35, 37, 39, 41, 44, 46, 49, 52, 55, 58, 62, 65, 69, 73, 78, 82, 87, 93, 98, 104, 110, 117, 123, 131, 139, 147, 156, 165, 175, 185, 196, 208, 220, 233, 247, 262, 277, 294, 311, 330, 349, 370, 392, 415, 440, 466, 494, 523, 554, 587, 622, 659, 698, 740, 784, 831, 880, 932, 988, 1047, 1109, 1175, 1245, 1319, 1397, 1480, 1568, 1661, 1760, 1865, 1976, 2093, 2217, 2349, 2489, 2637, 2794, 2960, 3136, 3322, 3520, 3729, 3951, 4186, 4435, 4699, 4978, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
unsigned int halfwavelength[128] = {0, 57724, 54484, 51426, 48540, 45815, 43244, 40817, 38526, 36364, 34323, 32396, 30578, 28862, 27242, 25713, 24270, 22908, 21622, 20408, 19263, 18182, 17161, 16198, 15289, 14431, 13621, 12856, 12135, 11454, 10811, 10204, 9631, 9091, 8581, 8099, 7645, 7215, 6810, 6428, 6067, 5727, 5405, 5102, 4816, 4545, 4290, 4050, 3822, 3608, 3405, 3214, 3034, 2863, 2703, 2551, 2408, 2273, 2145, 2025, 1911, 1804, 1703, 1607, 1517, 1432, 1351, 1276, 1204, 1136, 1073, 1012, 956, 902, 851, 804, 758, 716, 676, 638, 602, 568, 536, 506, 478, 451, 426, 402, 379, 358, 338, 319, 301, 284, 268, 253, 239, 225, 213, 201, 190, 179, 169, 159, 150, 142, 134, 127, 119, 113, 106, 100, 95, 89, 84, 80, 75, 71, 67, 63, 60, 56, 53, 50, 47, 45, 42, 40};

//Serial communication for MIDI interface
//SerialPC can be used for debugging
HardwareSerial //*SerialPC = &Serial,
               *SerialMIDI = &Serial; //MIDI serial interface on pins 0 and 1
               //Serial3; //On an Arduino Mega you can use multiple serial interfaces at once

//Flag when the timer has triggered that another step should be performed
volatile bool timerTriggered = false;

//Setup
void setup() {
  Timer1.initialize();
  
  ////SerialPC->begin(115200); //PC communication
  SerialMIDI->begin(31250); //MIDI in

  /////SerialPC->println("Setting up...");

  //Floppy disk control pins
  pinMode(SELECTPIN, OUTPUT);
  pinMode(DIRPIN, OUTPUT);
  pinMode(STEPPIN, OUTPUT);

  digitalWrite(SELECTPIN, LOW); //Select current floppy drive
  
  resetTrack(); //Reset the read head to a known state
  
  ////SerialPC->println("Ready!");
}

//Loop
void loop() {
  //Try to read MIDI message when available
  if (SerialMIDI->available())
    readMidiMessage();

  //If timer is triggered: execute a step
  if (timerTriggered)
    doStep();
}

//Read message from MIDI port
void readMidiMessage() {
  byte message = SerialMIDI->read(); //readMidiAndPassThrough();

  if (isStatus(message)) //Check if it is a status message and if so update
    statusByte = message & MASK_CHANNEL;
  else switch (statusByte) { //Perform action according to the current status message
    case NOTE_ON: { //Note on (key was pressed)
      byte note = message;
      while (SerialMIDI->available() < 1) {} //Wait until data available
      byte velocity = SerialMIDI->read(); //readMidiAndPassThrough(); //Read velocity
      if (velocity == 0) //Velocity == 0 indicates key was lifted
        noteOff(note, velocity);
      else noteOn(note, velocity); //Otherwise start playing note
      break;
      }
    case NOTE_OFF: { //Note off (key was lifted)
      byte note = message;
      while (SerialMIDI->available() < 1) {} //Wait until data available
      byte velocity = SerialMIDI->read(); //readMidiAndPassThrough(); //Read velocity
      noteOff(note, velocity); //Stop playing note
      break;
      }
    case PITCH_BEND: { //Pitch bend
      //int lowerByte = message;
      while (SerialMIDI->available() < 1) {} //Wait until data available
      int upperByte = SerialMIDI->read(); //readMidiAndPassThrough(); //Read pitch bend data
      pitchBend((upperByte << 7) | message); //Perform pitch bend
      break;
      }
    default: { //No other MIDI messages are implemented here...
      ////SerialPC->print("UNHANDLED: ");
      ////SerialPC->println(message);
      break;
      }
  }
}

//Perform note on action
void noteOn(byte note, byte velocity) {
  note -= 12; //Transpose one octave
  startplayingNote(note);

  ////SerialPC->print("Note ON: ");
  ////SerialPC->print(note);
  ////SerialPC->print(", ");
  ////SerialPC->println(velocity);
}

//Perform note off action
void noteOff(byte note, byte velocity) {
  note -= 12; //Transpose one octave
  stopPlayingNote(note);
  
  //SerialPC->print("Note OFF: ");
  //SerialPC->print(note);
  //SerialPC->print(", ");
  //SerialPC->println(velocity);
}

//Perform pitch bend action
void pitchBend(int value) {
  pitchBendNote(value);
  pitchBendValue = value; //Save current pitch bend value
  
  //SerialPC->print("Pitch bend: ");
  //SerialPC->println(value, HEX);
}

//If MIDI message is a status message
bool isStatus(byte message) {
  return bitRead(message, 7);
}

//Start playing a note
void startplayingNote(int note) {
  //if (frequency[note] == 0) return;
  currentNote = note; //Save current being played
  //unsigned long halfwavelength = 1000000 / frequency[note] / 2;

  //If not using pitch bend: start timer to play note
  if (pitchBendValue == PITCH_BEND_OFFSET)
    Timer1.attachInterrupt(timerTrigger, halfwavelength[note]);
  //Otherwise: apply pitch bending first
  else pitchBendNote(pitchBendValue);
}

//Stop playing a note
void stopPlayingNote(int note) {
  if (currentNote != note) return; //If previous key was lifted: do nothing
  currentNote = 0; //No current note playing
  Timer1.detachInterrupt(); //Stop timer
  checkBestDir(); //Check in which direction to start for next note
}

//Start playing a note with pitch bend
void pitchBendNote(int pitchbend) {
  if (currentNote == 0) return; //If no current note: do nothing
  //Calculate the difference between desired pitch and A4
  float noteOffsetA4 = (currentNote-69) + float(pitchbend - PITCH_BEND_OFFSET) / float(PITCH_BEND_OFFSET / 2);
  //Calculate the frequency for that pitch
  float frequency = 440 * pow(2, noteOffsetA4/12);
  //Wavelength for that pitch divided by two
  int halfWavelength = 1000000 / frequency / 2;
  //Start playing note by starting timer
  Timer1.attachInterrupt(timerTrigger, halfWavelength);
}

//Trigger the timer, queueing taking a step
void timerTrigger() {
  timerTriggered = true;
}

//Perform a step on the floppy drive
void doStep() {
  timerTriggered = false; //Reset timer trigger
  stepVal = !stepVal; //Invert the STEP value
  digitalWrite(STEPPIN, stepVal); //Write the new STEP value to the floppy drive
  incrementTrack(); //Increment current track number
}

//Increment the current track number
void incrementTrack() {
  if (dir) { //If DIR is forward: increment
    track++;
    if (track == TRACKS) //If reached the end: switch direction
      setDirection(false);
  } else { //If DIR is backward: decrement
    track--;
    if (track == 0) //If reached the beginning: switch direction
      setDirection(true);
  }
}

//Set the direction to whichever has the longest available path
void checkBestDir() {
  setDirection(track < (TRACKS / 2));
  //This will minimize having to switch direction in the middle of a note
}

//Set the direction of the floppy drive DIR
void setDirection(bool newdir) {
  dir = newdir;
  digitalWrite(DIRPIN, dir);
}

//Move the floppy drive read head to the start of the disk
void resetTrack() {
  //SerialPC->print("Resetting track... ");
  //Move back the full number of tracks to ensure it is at the start
  setDirection(false);
  for (int i=0; i<TRACKS; i++) {
    digitalWrite(STEPPIN, LOW);
    delay(3);
    digitalWrite(STEPPIN, HIGH);
    delay(3);
  }
  track = 0; //Initialize current track number
  setDirection(true); //Set direction forward
  //SerialPC->println("Done!");
}

//Can be used to pass through MIDI message to other devices
/*byte readMidiAndPassThrough() {
  byte message = SerialMIDI->read();
  SerialMIDI->write(message);
  return message;
}*/

//Convenience method to read from the MIDI interface
/*byte readMidi() {
  return SerialMIDI->read();
}*/

//For debugging
/*void print//SerialPC(String text, bool newline = false) {
  if (!//SerialPC) return;
  //SerialPC->print(text);
  if (newline) //SerialPC->println();
}*/

//Play a note for a certain time
/*void playNoteWithDelay(unsigned long notefreq, unsigned long notelength) {
  unsigned long halfWavelength = 1000000 / notefreq / 2,
    numSteps = notefreq * notelength / 1000;

  for (unsigned long i=0; i<numSteps; i++)
    doStepWithDelay(halfWavelength);
    
  checkBestDir();
}*/

//Take two steps for a full wavelength
/*void doStepWithDelay(int steplength) {
  incrementTrack();
  digitalWrite(STEPPIN, LOW);
  delayMicroseconds(steplength);
  digitalWrite(STEPPIN, HIGH);
  delayMicroseconds(steplength);
}*/
