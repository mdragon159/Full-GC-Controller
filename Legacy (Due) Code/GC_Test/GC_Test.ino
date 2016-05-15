// NOTE: Both should be connected to same line and 
//    output should be set as open drain (with a pull-up resistor)
#define GC_DATA_OUT_PIN 3 // Pin used to output data from console
#define GC_DATA_IN_PIN 2  // Pin used to read in data from console

#define NOP_FIVE nop\n nop\n nop\n nop\n nop\n
#define NOP_TWENTY NOP_FIVE NOP_FIVE NOP_FIVE NOP_FIVE
#define NOP_SIXTY NOP_TWENTY NOP_TWENTY


inline void digitalWriteDirect(int pin, boolean val){
  if(val) g_APinDescription[pin].pPort -> PIO_SODR = g_APinDescription[pin].ulPin;
  else    g_APinDescription[pin].pPort -> PIO_CODR = g_APinDescription[pin].ulPin;
}

inline int digitalReadDirect(int pin){
  return !!(g_APinDescription[pin].pPort -> PIO_PDSR & g_APinDescription[pin].ulPin);
}

inline void delayOneMicro() {
  // 100 nops = little more than one microsecond
  asm volatile(
    "nop\nnop\nnop\nnop\nnop\n"
    "nop\nnop\nnop\nnop\nnop\n"
    "nop\nnop\nnop\nnop\nnop\n"
    "nop\nnop\nnop\nnop\nnop\n"
    "nop\nnop\nnop\nnop\nnop\n"
    "nop\nnop\nnop\nnop\nnop\n"
    "nop\nnop\nnop\nnop\nnop\n"
    "nop\nnop\nnop\nnop\nnop\n"
    "nop\nnop\nnop\nnop\nnop\n"
    "nop\nnop\nnop\nnop\nnop\n"
    "nop\nnop\nnop\nnop\nnop\n"
    "nop\nnop\nnop\nnop\nnop\n"
    "nop\nnop\nnop\nnop\nnop\n"
    "nop\nnop\nnop\nnop\nnop\n"
    "nop\nnop\nnop\nnop\nnop\n"
    "nop\nnop\nnop\nnop\nnop\n"
    "nop\nnop\nnop\nnop\nnop\n"
    "nop\nnop\nnop\nnop\nnop\n"
    "nop\nnop\nnop\nnop\nnop\n"
    "nop\nnop\nnop\nnop\nnop\n"
  );
}

inline void delayOneMicroHalf() {
  // Arduino Due has 84MHz clock cycle, so 84 clock cycles = 1 microsecond
  // 115 nops = good enough for 1.5 us (Due can deal with the slight timing difference)
  asm volatile(
    "nop\nnop\nnop\nnop\nnop\n"
    "nop\nnop\nnop\nnop\nnop\n"
    "nop\nnop\nnop\nnop\nnop\n"
    "nop\nnop\nnop\nnop\nnop\n"
    "nop\nnop\nnop\nnop\nnop\n"
    
    "nop\nnop\nnop\nnop\nnop\n"
    "nop\nnop\nnop\nnop\nnop\n"
    "nop\nnop\nnop\nnop\nnop\n"
    "nop\nnop\nnop\nnop\nnop\n"
    "nop\nnop\nnop\nnop\nnop\n"
    
    "nop\nnop\nnop\nnop\nnop\n"
    "nop\nnop\nnop\nnop\nnop\n"
    "nop\nnop\nnop\nnop\nnop\n"
    "nop\nnop\nnop\nnop\nnop\n"
    "nop\nnop\nnop\nnop\nnop\n"
    
    "nop\nnop\nnop\nnop\nnop\n"
    "nop\nnop\nnop\nnop\nnop\n"
    "nop\nnop\nnop\nnop\nnop\n"
    "nop\nnop\nnop\nnop\nnop\n"
    "nop\nnop\nnop\nnop\nnop\n"
  );
}

// IMPORTANT: Do NOT connect to console until after setup runs
// TODO: Check if above is actually an issue! (has issues if unplug Arduino while connected for sure)
void setup() {
  // Enable serial output for debugging
  Serial.begin(115200);
  
  // Initialize GC data input and output pins
  pinMode(GC_DATA_IN_PIN, INPUT);
  pinMode(GC_DATA_OUT_PIN, OUTPUT); 

  // Declare output pin as open drain (and then can connect safely to console)
    // Note: PIO_MDER is the register from the spec that enables open drain
  g_APinDescription[GC_DATA_OUT_PIN].pPort -> PIO_MDER = g_APinDescription[GC_DATA_OUT_PIN].ulPin;

  // Set out pin to high so it doesn't drive anything > for sure < by default
  digitalWriteDirect(GC_DATA_OUT_PIN, true);
}

void testBasicWrite() {
  int val;
  
  // Set data to "high"
  digitalWriteDirect(GC_DATA_OUT_PIN, true);

  // Read- should be high
  val = digitalReadDirect(GC_DATA_IN_PIN);
  Serial.print("Should be high: ");
  Serial.println(val);

  delay(1000);

  // Set to low
  digitalWriteDirect(GC_DATA_OUT_PIN, false);

  // Read- should be low
  val = digitalReadDirect(GC_DATA_IN_PIN);
  Serial.print("Should be low: ");
  Serial.println(val);  

  delay(1000);
}

// Reads in one bit gamecube serial communication style then returns that value
// TODO: Change to returning a bool
// Note: Does NOT return until reads a value successfully
// Note: Interrupts SHOULD be disabled before calling function, else may have timing issues!
int readGCBit() {
  int holder;
  int data;
  while(true) {
    // Keep reading until get a 1 for sure (line should be high else data is already being written)
    holder = -1;
    while(holder != 1) {
      holder = digitalReadDirect(GC_DATA_IN_PIN);
    }

    // Keep reading until get a 0 for sure (a data bit write has begun)
    holder = -1;
    while(holder != 0) {
      holder = digitalReadDirect(GC_DATA_IN_PIN);
    }

    // Wait a microsecond and half (so data is for sure on line and stable)
      // TODO: Either update the comment above or the 100 noop function below
    delayOneMicro();
    // Read in data bit (whatever's on the line SHOULD be the data that was being sent)
    data = digitalReadDirect(GC_DATA_IN_PIN);

    // Return data bit read earlier!
    return data;
  }
}

void testReadTrue() {
  const int MAX_BUFFER = 88;
  int pos = 0;
  int val[MAX_BUFFER];

  noInterrupts();
  while(true) {
    // Read and save data
    val[pos] = readGCBit();

    pos++; // For next round
    if(pos != MAX_BUFFER) {
      continue; // Simply repeat
    }
    // If reached max buffer, output the info
      // TODO: Remove or somehow deal with losing timing again?
    else {
      interrupts();
      
      // Print all normally via serial except last one  
      for(int i=0; i < MAX_BUFFER; ++i) {
        Serial.print(val[i]);

        if(i!=0 && i%8 == 0) {
          Serial.print(" ");
        }
      }
      // Print a newline at the end
      Serial.print("\r\n\r\n");

      pos = 0; // Restart the buffer from the beginning
      noInterrupts();
    }
  }
}

void testContinuousRead() {
  const int MAX_BUFFER = 10000;
  int pos = 0;
  int val[MAX_BUFFER];

  noInterrupts();
  while(true) {
    // Read and save data
    asm volatile(
    "nop\nnop\nnop\nnop\nnop\n"
    "nop\nnop\nnop\nnop\nnop\n"
    "nop\nnop\nnop\nnop\nnop\n"
    "nop\nnop\nnop\nnop\nnop\n"
    );
    val[pos] = digitalReadDirect(GC_DATA_IN_PIN);

    pos++; // For next round
    if(pos != MAX_BUFFER) {
      continue; // Simply repeat
    }
    // If reached max buffer, output the info
      // TODO: Remove or somehow deal with losing timing again?
    else {
      interrupts();
      
      // Print all normally via serial except last one  
      for(int i=0; i < MAX_BUFFER; ++i) {
        Serial.print(val[i]);

        if(i!=0 && i%8 == 0) {
          Serial.print(" ");
        }
      }
      // Print a newline at the end
      Serial.print("\r\n\r\n");

      pos = 0; // Restart the buffer from the beginning
      noInterrupts();
    }
  }
}

void loop() {
  Serial.println("Beginning read system");
  testContinuousRead();
}
