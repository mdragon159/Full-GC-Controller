// STATUS: Currently focusing on reading only
//   (reading between an active, separate controller and "console")


// Library from https://github.com/ivanseidel/DueTimer/releases
// Last used v1.4.7
#include <DueTimer.h>

/****** DEFINITIONS & VARIABLES: Pins *******/
// NOTE: Both should be connected to same line and 
//    output should be set as open drain
#define GC_DATA_OUT_PIN 3 // Pin used to output data from console
#define GC_DATA_IN_PIN 2  // Pin used to read in data from console

/****** DEFINITIONS & VARIABLES: Reading *******/
#define READ_SIZE 500  // Number of bits for a single read process to complete with
// TODO: Make this binary booleans for speed & space
int val[READ_SIZE];   // Read bits are stored here
int curReadPos;       // Counter that keeps track of current position of the read

enum ReadState {
  none,       // Currently not reading
  readWait1,  // Waiting for line to be high
  readWait0,  // Waiting for line to drop to low before beginning read
  timerWait,  // Waiting for timer interrupt
  done        // Read done, should process now
};
ReadState curReadState;

/****** INLINE HELPER FUNCS *******/
inline void digitalWriteDirect(int pin, boolean val){
  if(val) g_APinDescription[pin].pPort -> PIO_SODR = g_APinDescription[pin].ulPin;
  else    g_APinDescription[pin].pPort -> PIO_CODR = g_APinDescription[pin].ulPin;
}

inline int digitalReadDirect(int pin){
  return !!(g_APinDescription[pin].pPort -> PIO_PDSR & g_APinDescription[pin].ulPin);
}


/****** INTERRUPT CODE *******/
void timerReadInterrupt() {
  // Safety net!
  if(curReadState != timerWait) {
    return;  
  }
  
  // Read from the data line into the appropriate pos
  val[curReadPos] = digitalReadDirect(GC_DATA_IN_PIN);
  
  // Increment position read
  curReadPos++;

  // If finished reading, set the state accordingly
  if(curReadPos >= READ_SIZE) {
    curReadState = done;  
  }
  // Otherwise, go back to the first in-process read state to prepare for the next bit
  else {
    curReadState = readWait1;
  }

  // Stop the timer
    // TODO: See if this is necessary
  Timer1.stop();
}

/****** ARDUINO FUNCTIONS *******/
void setup() {
  // Enable serial output for debugging
  Serial.begin(115200); // High baud rate
  
  // Initialize base variables
  curReadState = none;

  // Setup input pin to read from
  pinMode(GC_DATA_IN_PIN, INPUT);

  // Setup output open drain pin
  pinMode(GC_DATA_OUT_PIN, OUTPUT);
  // Declare output pin as open drain (and then can connect safely to console)
    // Note: PIO_MDER is the register from the spec that enables open drain
  g_APinDescription[GC_DATA_OUT_PIN].pPort -> PIO_MDER = g_APinDescription[GC_DATA_OUT_PIN].ulPin;
  // Set out pin to high so it doesn't drive anything for sure by default
  digitalWriteDirect(GC_DATA_OUT_PIN, true);
}

void loop() {
  // Act based off of the current read state
  int holder;
  switch(curReadState) {
    // Default behavior when not reading yet
    case none:
      // Setup a read
      curReadPos = 0;
      curReadState = readWait1;
      
      break;

    // Wait until line goes high for sure to continue with read
    case readWait1:
      // If data line is high at all, then ready for a data bit to be sent
      holder = digitalReadDirect(GC_DATA_IN_PIN);
      if(holder == 1) {
        curReadState = readWait0;
      }
      
      break;

    // Wait until line goes low which signifies a transaction has begun
    case readWait0:
      // If data line is low, transaction has begun!
      holder = digitalReadDirect(GC_DATA_IN_PIN);
      if(holder == 0) {
        // Setup a timer interrupt to read the data line in 2us
        Timer1.attachInterrupt(timerReadInterrupt).start(1);

        // Change state to signify waiting for timer to finish
        curReadState = timerWait;
      }
      
      break;

    // Currently waiting for 2us timer interrupt to occur
    case timerWait:
      // Do nothing, just wait
      
      break;

    // Complete read is done
    case done:
      // Output the results
      for(int i=0; i < READ_SIZE; ++i) {
        Serial.print(val[i]);

        // Space outputs by each byte
        if(i!=0 && i%8 == 0) {
          Serial.print(" ");
        }
      }
      // Print a double newline at the end
      Serial.print("\r\n\r\n");

      // Clear the state
      curReadState = none;

      break;

    default:
      // SHOULD NEVER REACH HERE
      break;
  }

}
