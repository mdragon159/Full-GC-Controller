/**
 * Originally adapted from Gamecube controller to Nintendo 64 adapter
 * by Andrew Brown
 *  https://github.com/brownan/Gamecube-N64-Controller
 */
// TODO: Take license from that repo above and use here as applicable

#include "pins_arduino.h"


// Easily switch the mode between Arduino acting as a console, controller, or third party
  // Precursor work for separating behavior within a single library
enum GC_Mode {
  As_Console,     // Act as the console and read data from the GC controller
  As_Controller,  // TODO: Act as a controller and respond to console commands
  As_ThirdParty,  // Simply read transactions on the data line
  As_Test         // Extra test mode
};
const GC_Mode CUR_MODE = As_Test; // Change this line to change the mode

#define GC_PIN 2
#define GC_PIN_DIR DDRD
// these two macros set arduino pin 2 to input or output, which with an
// external 1K pull-up resistor to the 3.3V rail, is like pulling it high or
// low.  These operations translate to 1 op code, which takes 2 cycles
#define GC_HIGH DDRD &= ~0x04
#define GC_LOW DDRD |= 0x04
#define GC_QUERY (PIND & 0x04)

// 8 bytes of data that we get from the controller. This is a global
// variable (not a struct definition)
static struct {
    // bits: 0, 0, 0, start, y, x, b, a
    unsigned char data1;
    // bits: 1, L, R, Z, Dup, Ddown, Dright, Dleft
    unsigned char data2;
    unsigned char stick_x;
    unsigned char stick_y;
    unsigned char cstick_x;
    unsigned char cstick_y;
    unsigned char left;
    unsigned char right;
} gc_status;

// Zero points for the GC controller stick
static unsigned char zero_x;
static unsigned char zero_y;

static void gc_send(unsigned char *buffer, char length);
static int gc_get();
static void init_gc_controller();
static void print_gc_status();


/****** FUNCTIONS: Base GC functions *******/

static void init_gc_controller()
{
  // Initialize the gamecube controller by sending it a null byte.
  // This is unnecessary for a standard controller, but is required for the
  // Wavebird.
  unsigned char initialize = 0x00;
  gc_send(&initialize, 1);

  // Stupid routine to wait for the gamecube controller to stop
  // sending its response. We don't care what it is, but we
  // can't start asking for status if it's still responding
  int x;
  for (x=0; x<64; x++) {
      // make sure the line is idle for 64 iterations, should
      // be plenty.
      if (!GC_QUERY)
          x = 0;
  }
}

/**
 * This sends the given byte sequence to the controller
 * length must be at least 1
 * hardcoded for Arduino DIO 2 and external pull-up resistor
 */
static void gc_send(unsigned char *buffer, char length)
{
    asm volatile (
            "; Start of gc_send assembly\n"

            // passed in to this block are:
            // the Z register (r31:r30) is the buffer pointer
            // %[length] is the register holding the length of the buffer in bytes

            // Instruction cycles are noted in parentheses
            // branch instructions have two values, one if the branch isn't
            // taken and one if it is

            // r25 will be the current buffer byte loaded from memory
            // r26 will be the bit counter for the current byte. when this
            // reaches 0, we need to decrement the length counter, load
            // the next buffer byte, and loop. (if the length counter becomes
            // 0, that's our exit condition)
            
            "ld r25, Z\n" // load the first byte

            // This label starts the outer loop, which sends a single byte
            ".L%=_byte_loop:\n"
            "ldi r26,lo8(8)\n" // (1)

            // This label starts the inner loop, which sends a single bit
            ".L%=_bit_loop:\n"
            "sbi 0xa,2\n" // (2) pull the line low

            // line needs to stay low for 1µs for a 1 bit, 3µs for a 0 bit
            // this block figures out if the next bit is a 0 or a 1
            // the strategy here is to shift the register left, then test and
            // branch on the carry flag
            "lsl r25\n" // (1) shift left. MSB goes into carry bit of status reg
            "brcc .L%=_zero_bit\n" // (1/2) branch if carry is cleared

            
            // this block is the timing for a 1 bit (1µs low, 3µs high)
            // Stay low for 16 - 2 (above lsl,brcc) - 2 (below cbi) = 12 cycles
            "nop\nnop\nnop\nnop\nnop\n" // (5)
            "nop\nnop\nnop\nnop\nnop\n" // (5)
            "nop\nnop\n" // (2)
            "cbi 0xa,2\n" // (2) set the line high again
            // Now stay high for 2µs of the 3µs to sync up with the branch below
            // 2*16 - 2 (for the rjmp) = 30 cycles
            "nop\nnop\nnop\nnop\nnop\n" // (5)
            "nop\nnop\nnop\nnop\nnop\n" // (5)
            "nop\nnop\nnop\nnop\nnop\n" // (5)
            "nop\nnop\nnop\nnop\nnop\n" // (5)
            "nop\nnop\nnop\nnop\nnop\n" // (5)
            "nop\nnop\nnop\nnop\nnop\n" // (5)
            "rjmp .L%=_finish_bit\n" // (2)


            // this block is the timing for a 0 bit (3µs low, 1µs high)
            // Need to go high in 3*16 - 3 (above lsl,brcc) - 2 (below cbi) = 43 cycles
            ".L%=_zero_bit:\n"
            "nop\nnop\nnop\nnop\nnop\n" // (5)
            "nop\nnop\nnop\nnop\nnop\n" // (5)
            "nop\nnop\nnop\nnop\nnop\n" // (5)
            "nop\nnop\nnop\nnop\nnop\n" // (5)
            "nop\nnop\nnop\nnop\nnop\n" // (5)
            "nop\nnop\nnop\nnop\nnop\n" // (5)
            "nop\nnop\nnop\nnop\nnop\n" // (5)
            "nop\nnop\nnop\nnop\nnop\n" // (5)
            "nop\nnop\nnop\n" // (3)
            "cbi 0xa,2\n" // (2) set the line high again


            // The two branches meet up here.
            // We are now *exactly* 3µs into the sending of a bit, and the line
            // is high again. We have 1µs to do the looping and iteration
            // logic.
            ".L%=_finish_bit:\n"
            "subi r26,1\n" // (1) subtract 1 from our bit counter
            "breq .L%=_load_next_byte\n" // (1/2) branch if we've sent all the bits of this byte

            // At this point, we have more bits to send in this byte, but the
            // line must remain high for another 1µs (minus the above
            // instructions and the jump below and the sbi instruction at the
            // top of the loop)
            // 16 - 2(above) - 2 (rjmp below) - 2 (sbi after jump) = 10
            "nop\nnop\nnop\nnop\nnop\n" // (5)
            "nop\nnop\nnop\nnop\nnop\n" // (5)
            "rjmp .L%=_bit_loop\n"


            // This block starts 3 cycles into the last 1µs of the line being high
            // We need to decrement the byte counter. If it's 0, that's our exit condition.
            // If not we need to load the next byte and go to the top of the byte loop
            ".L%=_load_next_byte:\n"
            "subi %[length], 1\n" // (1)
            "breq .L%=_loop_exit\n" // (1/2) if the byte counter is 0, exit
            "adiw r30,1\n" // (2) increment byte pointer
            "ld r25, Z\n" // (2) load the next byte
            // delay block:
            // needs to go high after 1µs or 16 cycles
            // 16 - 9 (above) - 2 (the jump itself) - 3 (after jump) = 2
            "nop\nnop\n" // (2)
            "rjmp .L%=_byte_loop\n" // (2)


            // Loop exit
            ".L%=_loop_exit:\n"

            // final task: send the stop bit, which is a 1 (1µs low 3µs high)
            // the line goes low in:
            // 16 - 6 (above since line went high) - 2 (sbi instruction below) = 8 cycles
            "nop\nnop\nnop\nnop\nnop\n" // (5)
            "nop\nnop\nnop\n" // (3)
            "sbi 0xa,2\n" // (2) pull the line low
            // stay low for 1µs
            // 16 - 2 (below cbi) = 14
            "nop\nnop\nnop\nnop\nnop\n" // (5)
            "nop\nnop\nnop\nnop\nnop\n" // (5)
            "nop\nnop\nnop\nnop\n" // (4)
            "cbi 0xa,2\n" // (2) set the line high again

            // just stay high. no need to wait 3µs before returning

            :
            // outputs:
            "+z" (buffer) // (read and write)
            :
            // inputs:
            [length] "r" (length)
            :
            // clobbers:
                "r25", "r26"
            );

}

// Read 8 bytes from the gamecube controller
// hardwired to read from Arduino DIO2 with external pullup resistor
static int gc_get()
{
    // listen for the expected 8 bytes of data back from the controller and
    // and pack it into the gc_status struct.
    asm volatile (";Starting to listen");

    // treat the 8 byte struct gc_status as a raw char array.
    unsigned char *bitbin = (unsigned char*) &gc_status;

    unsigned char retval;

    asm volatile (
            "; START OF MANUAL ASSEMBLY BLOCK\n"
            // r25 is our bit counter. We read 64 bits and increment the byte
            // pointer every 8 bits
            "ldi r25,lo8(0)\n"
            // read in the first byte of the gc_status struct
            "ld r23,Z\n"
            // default exit value is 1 (success)
            "ldi %[retval],lo8(1)\n"

            // Top of the main read loop label
            "L%=_read_loop:\n"

            // This first spinloop waits for the line to go low. It loops 64
            // times before it gives up and returns
            "ldi r24,lo8(64)\n" // r24 is the timeout counter
            "L%=_1:\n"
            "sbis 0x9,2\n" // reg 9 bit 2 is PIND2, or arduino I/O 2
            "rjmp L%=_2\n" // line is low. jump to below
            // the following happens if the line is still high
            "subi r24,lo8(1)\n"
            "brne L%=_1\n" // loop if the counter isn't 0
            // timeout? set output to 0 indicating failure and jump to
            // the end
            "ldi %[retval],lo8(0)\n"
            "rjmp L%=_exit\n"
            "L%=_2:\n"

            // Next block. The line has just gone low. Wait approx 2µs
            // each cycle is 1/16 µs on a 16Mhz processor
            "nop\nnop\nnop\nnop\nnop\n"
            "nop\nnop\nnop\nnop\nnop\n"
            "nop\nnop\nnop\nnop\nnop\n"
            "nop\nnop\nnop\nnop\nnop\n"
            "nop\nnop\nnop\nnop\nnop\n"
            "nop\nnop\nnop\nnop\nnop\n"

            // This block left shifts the current gc_status byte in r23,
            // and adds the current line state as the LSB
            "lsl r23\n" // left shift
            "sbic 0x9,2\n" // read PIND2
            "sbr r23,lo8(1)\n" // set bit 1 in r23 if PIND2 is high
            "st Z,r23\n" // save r23 back to memory. We technically only have
            // to do this every 8 bits but this simplifies the branches below

            // This block increments the bitcount(r25). If bitcount is 64, exit
            // with success. If bitcount is a multiple of 8, then increment Z
            // and load the next byte.
            "subi r25,lo8(-1)\n" // increment bitcount
            "cpi r25,lo8(64)\n" // == 64?
            "breq L%=_exit\n" // jump to exit
            "mov r24,r25\n" // copy bitcounter(r25) to r24 for tmp
            "andi r24,lo8(7)\n" // get lower 3 bits
            "brne L%=_3\n" // branch if not 0 (is not divisble by 8)
            "adiw r30,1\n" // if divisible by 8, increment pointer
            "ld r23,Z\n" // ...and load the new byte into r23
            "L%=_3:\n"

            // This next block waits for the line to go high again. again, it
            // sets a timeout counter of 64 iterations
            "ldi r24,lo8(64)\n" // r24 is the timeout counter
            "L%=_4:\n"
            "sbic 0x9,2\n" // checks PIND2
            "rjmp L%=_read_loop\n" // line is high. ready for next loop
            // the following happens if the line is still low
            "subi r24,lo8(1)\n"
            "brne L%=_4\n" // loop if the counter isn't 0
            // timeout? set output to 0 indicating failure and fall through to
            // the end
            "ldi %[retval],lo8(0)\n"


            "L%=_exit:\n"
            ";END OF MANUAL ASSEMBLY BLOCK\n"
            // ----------
            // outputs:
            : [retval] "=r" (retval),
            // About the bitbin pointer: The "z" constraint tells the
            // compiler to put the pointer in the Z register pair (r31:r30)
            // The + tells the compiler that we are both reading and writing
            // this pointer. This is important because otherwise it will
            // allocate the same register for retval (r30).
            "+z" (bitbin)
            // clobbers (registers we use in the assembly for the compiler to
            // avoid):
            :: "r25", "r24", "r23"
            );

    return retval;
}

// Reads variable length of data on data line into a data buffer
// Returns number of bits successfully read
// Note: Uncertain behavior if bitbuffer does not contain length_in_bits bits
static uint8_t gc_read(uint8_t* bitbuffer, uint8_t length_in_bits) {
  // listen for the expected 8 bytes of data back from the controller and
    // and pack it into the gc_status struct.
    asm volatile (";Starting to listen");
    
    // Return value is number of bits read
    uint8_t retval;

    asm volatile (
            "; START OF MANUAL ASSEMBLY BLOCK\n"            
            // r25 is our bit counter. We read 64 bits and increment the byte
            // pointer every 8 bits
            "ldi r25,lo8(0)\n"
            // read in the first byte of the gc_status struct
            "ld r23,Z\n"
            
            // default exit value is 0 (no bits read)
            "ldi %[retval],lo8(0)\n"

            // Top of the main read loop label
            "L%=_read_loop:\n"

            // This first spinloop waits for the line to go low. It loops 64
            // times before it gives up and returns
            "ldi r24,lo8(64)\n" // r24 is the timeout counter
            "L%=_1:\n"
            "sbis 0x9,2\n" // reg 9 bit 2 is PIND2, or arduino I/O 2
            "rjmp L%=_2\n" // line is low. jump to below
            // the following happens if the line is still high
            "subi r24,lo8(1)\n"
            "brne L%=_1\n" // loop if the counter isn't 0
            
            // Timeout: Simply exit
            "rjmp L%=_early_exit\n"
            
            "L%=_2:\n"
            // Increment return value by one as valid bit is about to be read
            "subi %[retval],lo8(-1)\n"           
            // Next block. The line has just gone low. Wait APPROX 2µs
            // each cycle is 1/16 µs on a 16Mhz processor
            "nop\nnop\nnop\nnop\nnop\n"
            "nop\nnop\nnop\nnop\nnop\n"
            "nop\nnop\nnop\nnop\nnop\n"
            "nop\nnop\nnop\nnop\nnop\n"
            "nop\nnop\nnop\nnop\nnop\n"
            "nop\nnop\nnop\nnop\nnop\n"

            // This block left shifts the current gc_status byte in r23,
            // and adds the current line state as the LSB
            "lsl r23\n" // left shift
            "sbic 0x9,2\n" // read PIND2
            "sbr r23,lo8(1)\n" // set bit 1 in r23 if PIND2 is high
            
            // This block increments the bitcount(r25). If bitcount is 64, exit
            // with success. If bitcount is a multiple of 8, then increment Z
            // and load the next byte.
            "subi r25,lo8(-1)\n" // increment bitcount
            "cp r25, %[length_in_bits]\n" // == read length?
            "breq L%=_exit\n" // jump to exit
            "mov r24,r25\n" // copy bitcounter(r25) to r24 for tmp
            "andi r24,lo8(7)\n" // get lower 3 bits
            "brne L%=_3\n" // branch if not 0 (is not divisble by 8)
            "st Z,r23\n" // if divisible by 8, save r23 back to memory
            "adiw r30,1\n" // increment pointer
            "ld r23,Z\n" // ...and load the new byte into r23
            "L%=_3:\n"

            // This next block waits for the line to go high again. again, it
            // sets a timeout counter of 64 iterations
            "ldi r24,lo8(64)\n" // r24 is the timeout counter
            "L%=_4:\n"
            "sbic 0x9,2\n" // checks PIND2
            "rjmp L%=_read_loop\n" // line is high. ready for next loop
            // the following happens if the line is still low
            "subi r24,lo8(1)\n"
            "brne L%=_4\n" // loop if the counter isn't 0
            // Timeout: Simply fall to early exit

            // Early exit: Left shift as necessary so first read bit is at the MSB
            "L%=_early_exit:\n"
            // If divisible by 8, save and branch to exit
            // else, left shift, increment by 1, and repeat
            "mov r24,r25\n" // copy bitcounter(r25) to r24 for tmp
            "andi r24,lo8(7)\n" // get lower 3 bits
            "breq L%=_finish_early_exit\n" // branch if 0 (is divisble by 8)
            "subi r25,lo8(-1)\n" // if not divisible by 8, increment bitcount
            "lsl r23\n" // left shift
            "rjmp L%=_early_exit\n"

            "L%=_finish_early_exit:\n"
            "st Z,r23\n" // save properly shifted r23 back to memory

            "L%=_exit:\n"
            
            ";END OF MANUAL ASSEMBLY BLOCK\n"
            // ----------
            // outputs:
            : [retval] "=r" (retval),
            // About the bitbin pointer: The "z" constraint tells the
            // compiler to put the pointer in the Z register pair (r31:r30)
            // The + tells the compiler that we are both reading and writing
            // this pointer. This is important because otherwise it will
            // allocate the same register for retval (r30).
            "+z" (bitbuffer)
            // inputs:
            : [length_in_bits] "r" (length_in_bits)
            // clobbers (registers we use in the assembly for the compiler to
            // avoid):
            : "r25", "r24", "r23"
            );

    return retval;
}

// Responds to commands in the data line
static int gc_respond() {
    // listen for the expected 8 bytes of data back from the controller and
    // and pack it into the gc_status struct.
    asm volatile (";Starting to respond");
    
    // Return value is number of bits read
    uint8_t retval;

    asm volatile (
            "; START OF MANUAL ASSEMBLY BLOCK\n"
            // default exit value is 0 (nothing valuable read)
            "ldi %[retval],lo8(0)\n"

            // Check first bit- should be a 0 for both probe commands
            "L%=_check_A1:\n"
            // Wait for data to be ready on the line
            //------------------------------------------> LINE LOW PART
                // This first spinloop waits for the line to go low. It loops 64
                // times before it gives up and returns
                "ldi r24,lo8(64)\n" // r24 is the timeout counter
                "L%=_A1_read_loop:\n"
                "sbis 0x9,2\n" // reg 9 bit 2 is PIND2, or arduino I/O 2
                "rjmp L%=_A1_read_loop2\n" // line is low. jump to below
                // the following happens if the line is still high
                "subi r24,lo8(1)\n"
                "brne L%=_A1_read_loop\n" // loop if the counter isn't 0
                // TODO: Revisit timeout part
                "rjmp L%=_exit\n"
    
                "L%=_A1_read_loop2:\n"
                // Wait approx 2us (16MHz so 16 cycles = 1us, 30 nops here)
                "nop\nnop\nnop\nnop\nnop\n"
                "nop\nnop\nnop\nnop\nnop\n"
                "nop\nnop\nnop\nnop\nnop\n"
                "nop\nnop\nnop\nnop\nnop\n"
                "nop\nnop\nnop\nnop\nnop\n"
                "nop\nnop\nnop\nnop\nnop\n"
            //<------------------------------------------

            // Line better be 0 at first, else not start of either probing command!
            "sbic 0x9,2\n" // reg 9 bit 2 is PIND2, or arduino I/O 2
            "rjmp L%=_exit\n" // line is high/1. Command read failure 
            // Wait for data line to be high before continuing with read
            //------------------------------------------> LINE HIGHPART
                "ldi r24,lo8(64)\n" // r24 is the timeout counter
                "L%=_A1_high_wait:\n"
                "sbic 0x9,2\n" // checks PIND2
                "rjmp L%=_check_A2\n" // line is high. ready for next loop
                // the following happens if the line is still low
                "subi r24,lo8(1)\n"
                "brne L%=_A1_high_wait\n" // loop if the counter isn't 0
                // Timeout: Exit
                // TODO: Different action? Different return value?
                "rjmp L%=_exit\n"
            //<------------------------------------------

            // Check for a 1 or 0, depending on potentially which probe command
            "L%=_check_A2:\n"
            // Wait for data to be ready on the line
            //------------------------------------------> LINE LOW PART
                // This first spinloop waits for the line to go low. It loops 64
                // times before it gives up and returns
                "ldi r24,lo8(64)\n" // r24 is the timeout counter
                "L%=_A2_read_loop:\n"
                "sbis 0x9,2\n" // reg 9 bit 2 is PIND2, or arduino I/O 2
                "rjmp L%=_A2_read_loop2\n" // line is low. jump to below
                // the following happens if the line is still high
                "subi r24,lo8(1)\n"
                "brne L%=_A2_read_loop\n" // loop if the counter isn't 0
                // TODO: Revisit timeout part
                "rjmp L%=_exit\n"
    
                "L%=_A2_read_loop2:\n"
                // Wait approx 2us (16MHz so 16 cycles = 1us, 30 nops here)
                "nop\nnop\nnop\nnop\nnop\n"
                "nop\nnop\nnop\nnop\nnop\n"
                "nop\nnop\nnop\nnop\nnop\n"
                "nop\nnop\nnop\nnop\nnop\n"
                "nop\nnop\nnop\nnop\nnop\n"
                "nop\nnop\nnop\nnop\nnop\n"
            //<------------------------------------------
            // Check second bit- 0 = initialization probe command, 1 = read probe command
            "sbis 0x9,2\n" // skip next instruction if line is high
            "rjmp L%=_setup_B3\n" // Low, so possibly initialization probe command
            // Otherwise, a 1 so need 12 0's in a row so get ready to read that
            "ldi r25, lo8(12)\n" // Down counter
            // Wait for data line to be high before continuing with read
            //------------------------------------------> LINE HIGHPART
                "ldi r24,lo8(64)\n" // r24 is the timeout counter
                "L%=_A2_high_wait:\n"
                "sbic 0x9,2\n" // checks PIND2
                "rjmp L%=_check_A3\n" // line is high. ready for next loop
                // the following happens if the line is still low
                "subi r24,lo8(1)\n"
                "brne L%=_A2_high_wait\n" // loop if the counter isn't 0
                // Timeout: Exit
                // TODO: Different action? Different return value?
                "rjmp L%=_exit\n"
            //<------------------------------------------

            // Read 0's till counter reaches 0 (total 12)
            "L%=_check_A3:\n"
            // Wait for data to be ready on the line
            //------------------------------------------> LINE LOW PART
                // This first spinloop waits for the line to go low. It loops 64
                // times before it gives up and returns
                "ldi r24,lo8(64)\n" // r24 is the timeout counter
                "L%=_A3_read_loop:\n"
                "sbis 0x9,2\n" // reg 9 bit 2 is PIND2, or arduino I/O 2
                "rjmp L%=_A3_read_loop2\n" // line is low. jump to below
                // the following happens if the line is still high
                "subi r24,lo8(1)\n"
                "brne L%=_A3_read_loop\n" // loop if the counter isn't 0
                // TODO: Revisit timeout part
                "rjmp L%=_exit\n"
    
                "L%=_A3_read_loop2:\n"
                // Wait approx 2us (16MHz so 16 cycles = 1us, 30 nops here)
                "nop\nnop\nnop\nnop\nnop\n"
                "nop\nnop\nnop\nnop\nnop\n"
                "nop\nnop\nnop\nnop\nnop\n"
                "nop\nnop\nnop\nnop\nnop\n"
                "nop\nnop\nnop\nnop\nnop\n"
                "nop\nnop\nnop\nnop\nnop\n"
            //<------------------------------------------
            // Better be a 0- else read issue
            "sbic 0x9,2\n" // skip next instruction if line is low
            "rjmp L%=_exit\n" // line is high/1. Command read failure 
            "subi r25,lo8(1)\n" // line is low, decrement the 0s counter
            "breq L%=_setup_A4\n" // if result is 0, then branch to next check setup
            // Otherwise, wait for line to be high then recheck for more 0's
            //------------------------------------------> LINE HIGHPART
                "ldi r24,lo8(64)\n" // r24 is the timeout counter
                "L%=_A3_high_wait:\n"
                "sbic 0x9,2\n" // checks PIND2
                "rjmp L%=_check_A3\n" // line is high. ready for next loop
                // the following happens if the line is still low
                "subi r24,lo8(1)\n"
                "brne L%=_A3_high_wait\n" // loop if the counter isn't 0
                // Timeout: Exit
                // TODO: Different action? Different return value?
                "rjmp L%=_exit\n"
            //<------------------------------------------

            "L%=_setup_A4:\n"
            "ldi r25, lo8(2)\n" // Need to check for 2 1's so setup counter
            // Wait for line to be high again then jump to check_A4
            //------------------------------------------> LINE HIGHPART
                "ldi r24,lo8(64)\n" // r24 is the timeout counter
                "L%=_A4s_high_wait:\n"
                "sbic 0x9,2\n" // checks PIND2
                "rjmp L%=_check_A4\n" // line is high. ready for next loop
                // the following happens if the line is still low
                "subi r24,lo8(1)\n"
                "brne L%=_A4s_high_wait\n" // loop if the counter isn't 0
                // Timeout: Exit
                // TODO: Different action? Different return value?
                "rjmp L%=_exit\n"
            //<------------------------------------------

            // Read 1's until counter is 0 (total 2)
            "L%=_check_A4:\n"
            // Wait for data to be ready on the line
            //------------------------------------------> LINE LOW PART
                // This first spinloop waits for the line to go low. It loops 64
                // times before it gives up and returns
                "ldi r24,lo8(64)\n" // r24 is the timeout counter
                "L%=_A4_read_loop:\n"
                "sbis 0x9,2\n" // reg 9 bit 2 is PIND2, or arduino I/O 2
                "rjmp L%=_A4_read_loop2\n" // line is low. jump to below
                // the following happens if the line is still high
                "subi r24,lo8(1)\n"
                "brne L%=_A4_read_loop\n" // loop if the counter isn't 0
                // TODO: Revisit timeout part
                "rjmp L%=_exit\n"
    
                "L%=_A4_read_loop2:\n"
                // Wait approx 2us (16MHz so 16 cycles = 1us, 30 nops here)
                "nop\nnop\nnop\nnop\nnop\n"
                "nop\nnop\nnop\nnop\nnop\n"
                "nop\nnop\nnop\nnop\nnop\n"
                "nop\nnop\nnop\nnop\nnop\n"
                "nop\nnop\nnop\nnop\nnop\n"
                "nop\nnop\nnop\nnop\nnop\n"
            //<------------------------------------------
            "sbis 0x9,2\n" // skip next instruction if line is high
            "rjmp L%=_exit\n" // line is low/0. Command read failure 
            "subi r25,lo8(1)\n" // line is high, decrement the 1s counter
            "breq L%=_setup_A5\n" // if result is 0, then branch to next check setup
            // Otherwise, wait for line to be high then recheck for another 1
            //------------------------------------------> LINE HIGHPART
                "ldi r24,lo8(64)\n" // r24 is the timeout counter
                "L%=_A4_high_wait:\n"
                "sbic 0x9,2\n" // checks PIND2
                "rjmp L%=_check_A4\n" // line is high. ready for next loop
                // the following happens if the line is still low
                "subi r24,lo8(1)\n"
                "brne L%=_A4_high_wait\n" // loop if the counter isn't 0
                // Timeout: Exit
                // TODO: Different action? Different return value?
                "rjmp L%=_exit\n"
            //<------------------------------------------


            "L%=_setup_A5:\n"
            "ldi r25, lo8(8)\n" // Need to check for 8 0's so setup counter
            // Wait for line to be high again then jump to check_A4
            //------------------------------------------> LINE HIGHPART
                "ldi r24,lo8(64)\n" // r24 is the timeout counter
                "L%=_A5s_high_wait:\n"
                "sbic 0x9,2\n" // checks PIND2
                "rjmp L%=_check_A5\n" // line is high. ready for next loop
                // the following happens if the line is still low
                "subi r24,lo8(1)\n"
                "brne L%=_A5s_high_wait\n" // loop if the counter isn't 0
                // Timeout: Exit
                // TODO: Different action? Different return value?
                "rjmp L%=_exit\n"
            //<------------------------------------------

            // Read 0's until counter is 0 (total 8)
            "L%=_check_A5:\n"
            // Wait for data to be ready on the line
            //------------------------------------------> LINE LOW PART
                // This first spinloop waits for the line to go low. It loops 64
                // times before it gives up and returns
                "ldi r24,lo8(64)\n" // r24 is the timeout counter
                "L%=_A5_read_loop:\n"
                "sbis 0x9,2\n" // reg 9 bit 2 is PIND2, or arduino I/O 2
                "rjmp L%=_A5_read_loop2\n" // line is low. jump to below
                // the following happens if the line is still high
                "subi r24,lo8(1)\n"
                "brne L%=_A5_read_loop\n" // loop if the counter isn't 0
                // TODO: Revisit timeout part
                "rjmp L%=_exit\n"
    
                "L%=_A5_read_loop2:\n"
                // Wait approx 2us (16MHz so 16 cycles = 1us, 30 nops here)
                "nop\nnop\nnop\nnop\nnop\n"
                "nop\nnop\nnop\nnop\nnop\n"
                "nop\nnop\nnop\nnop\nnop\n"
                "nop\nnop\nnop\nnop\nnop\n"
                "nop\nnop\nnop\nnop\nnop\n"
                "nop\nnop\nnop\nnop\nnop\n"
            //<------------------------------------------
            "sbic 0x9,2\n" // skip next instruction if line is low
            "rjmp L%=_exit\n" // line is high/1. Command read failure 
            "subi r25,lo8(1)\n" // line is low, decrement the 0s counter
            "breq L%=_check_A6\n" // if result is 0, then branch to next check
            // Otherwise, wait for line to be high then recheck for more 0's
            //------------------------------------------> LINE HIGHPART
                "ldi r24,lo8(64)\n" // r24 is the timeout counter
                "L%=_A5_high_wait:\n"
                "sbic 0x9,2\n" // checks PIND2
                "rjmp L%=_check_A5\n" // line is high. ready for next loop
                // the following happens if the line is still low
                "subi r24,lo8(1)\n"
                "brne L%=_A5_high_wait\n" // loop if the counter isn't 0
                // Timeout: Exit
                // TODO: Different action? Different return value?
                "rjmp L%=_exit\n"
            //<------------------------------------------

            // Simply check for a 1- if so, then this is the read probe command!
            "L%=_check_A6:\n"
             // Wait for data to be ready on the line
            //------------------------------------------> LINE LOW PART
                // This first spinloop waits for the line to go low. It loops 64
                // times before it gives up and returns
                "ldi r24,lo8(64)\n" // r24 is the timeout counter
                "L%=_A6_read_loop:\n"
                "sbis 0x9,2\n" // reg 9 bit 2 is PIND2, or arduino I/O 2
                "rjmp L%=_A6_read_loop2\n" // line is low. jump to below
                // the following happens if the line is still high
                "subi r24,lo8(1)\n"
                "brne L%=_A6_read_loop\n" // loop if the counter isn't 0
                // TODO: Revisit timeout part
                "rjmp L%=_exit\n"
    
                "L%=_A6_read_loop2:\n"
                // Wait approx 2us (16MHz so 16 cycles = 1us, 30 nops here)
                "nop\nnop\nnop\nnop\nnop\n"
                "nop\nnop\nnop\nnop\nnop\n"
                "nop\nnop\nnop\nnop\nnop\n"
                "nop\nnop\nnop\nnop\nnop\n"
                "nop\nnop\nnop\nnop\nnop\n"
                "nop\nnop\nnop\nnop\nnop\n"
            //<------------------------------------------
            "sbis 0x9,2\n" // skip next instruction if line is high
            "rjmp L%=_exit\n" // line is low/0. Command read failure 
            // Wait for data line to be high before continuing for safety
            //------------------------------------------> LINE HIGHPART
                "ldi r24,lo8(64)\n" // r24 is the timeout counter
                "L%=_A6_high_wait:\n"
                "sbic 0x9,2\n" // checks PIND2
                "rjmp L%=_respond_A\n" // line is high. ready for next loop
                // the following happens if the line is still low
                "subi r24,lo8(1)\n"
                "brne L%=_A6_high_wait\n" // loop if the counter isn't 0
                // Timeout: Exit
                // TODO: Different action? Different return value?
                "rjmp L%=_exit\n"
            //<------------------------------------------

            // TODO: Success, found read probe command! Now, respond!
            "L%=_respond_A:\n"
            // For now, just return a 1 for success
            "ldi %[retval],lo8(1)\n"
            "rjmp L%=_exit\n" 

            // *************************************************************
            "L%=_setup_B3:\n"
            "ldi r25, lo8(6)\n" // Need to check for 6 more 0's so setup counter
            // Wait for line to be high again then jump to check_A4
            //------------------------------------------> LINE HIGHPART
                "ldi r24,lo8(64)\n" // r24 is the timeout counter
                "L%=_B3s_high_wait:\n"
                "sbic 0x9,2\n" // checks PIND2
                "rjmp L%=_check_B3\n" // line is high. ready for next loop
                // the following happens if the line is still low
                "subi r24,lo8(1)\n"
                "brne L%=_B3s_high_wait\n" // loop if the counter isn't 0
                // Timeout: Exit
                // TODO: Different action? Different return value?
                "rjmp L%=_exit\n"
            //<------------------------------------------v

            // Read 0's until counter is 0 (total 6)
            "L%=_check_B3:\n"
            // Wait for data to be ready on the line
            //------------------------------------------> LINE LOW PART
                // This first spinloop waits for the line to go low. It loops 64
                // times before it gives up and returns
                "ldi r24,lo8(64)\n" // r24 is the timeout counter
                "L%=_B3_read_loop:\n"
                "sbis 0x9,2\n" // reg 9 bit 2 is PIND2, or arduino I/O 2
                "rjmp L%=_B3_read_loop2\n" // line is low. jump to below
                // the following happens if the line is still high
                "subi r24,lo8(1)\n"
                "brne L%=_B3_read_loop\n" // loop if the counter isn't 0
                // TODO: Revisit timeout part
                "rjmp L%=_exit\n"
    
                "L%=_B3_read_loop2:\n"
                // Wait approx 2us (16MHz so 16 cycles = 1us, 30 nops here)
                "nop\nnop\nnop\nnop\nnop\n"
                "nop\nnop\nnop\nnop\nnop\n"
                "nop\nnop\nnop\nnop\nnop\n"
                "nop\nnop\nnop\nnop\nnop\n"
                "nop\nnop\nnop\nnop\nnop\n"
                "nop\nnop\nnop\nnop\nnop\n"
            //<------------------------------------------
            "sbic 0x9,2\n" // skip next instruction if line is low
            "rjmp L%=_exit\n" // line is high/1. Command read failure 
            "subi r25,lo8(1)\n" // line is low, decrement the 0s counter
            "breq L%=_check_B4\n" // if result is 0, then branch to next check
            // Otherwise, wait for line to be high then recheck for more 0's
            //------------------------------------------> LINE HIGHPART
                "ldi r24,lo8(64)\n" // r24 is the timeout counter
                "L%=_B3_high_wait:\n"
                "sbic 0x9,2\n" // checks PIND2
                "rjmp L%=_check_B3\n" // line is high. ready for next loop
                // the following happens if the line is still low
                "subi r24,lo8(1)\n"
                "brne L%=_B3_high_wait\n" // loop if the counter isn't 0
                // Timeout: Exit
                // TODO: Different action? Different return value?
                "rjmp L%=_exit\n"
            //<------------------------------------------

            // Simply check for a 1- if so, then command reading success!
            "L%=_check_B4:\n"
             // Wait for data to be ready on the line
            //------------------------------------------> LINE LOW PART
                // This first spinloop waits for the line to go low. It loops 64
                // times before it gives up and returns
                "ldi r24,lo8(64)\n" // r24 is the timeout counter
                "L%=_B4_read_loop:\n"
                "sbis 0x9,2\n" // reg 9 bit 2 is PIND2, or arduino I/O 2
                "rjmp L%=_B4_read_loop2\n" // line is low. jump to below
                // the following happens if the line is still high
                "subi r24,lo8(1)\n"
                "brne L%=_B4_read_loop\n" // loop if the counter isn't 0
                // TODO: Revisit timeout part
                "rjmp L%=_exit\n"
    
                "L%=_B4_read_loop2:\n"
                // Wait approx 2us (16MHz so 16 cycles = 1us, 30 nops here)
                "nop\nnop\nnop\nnop\nnop\n"
                "nop\nnop\nnop\nnop\nnop\n"
                "nop\nnop\nnop\nnop\nnop\n"
                "nop\nnop\nnop\nnop\nnop\n"
                "nop\nnop\nnop\nnop\nnop\n"
                "nop\nnop\nnop\nnop\nnop\n"
            //<------------------------------------------
            "sbis 0x9,2\n" // skip next instruction if line is high
            "rjmp L%=_exit\n" // line is low/0. Command read failure 
            // Wait for data line to be high before continuing for safety
            //------------------------------------------> LINE HIGHPART
                "ldi r24,lo8(64)\n" // r24 is the timeout counter
                "L%=_B4_high_wait:\n"
                "sbic 0x9,2\n" // checks PIND2
                "rjmp L%=_respond_B\n" // line is high. ready for next loop
                // the following happens if the line is still low
                "subi r24,lo8(1)\n"
                "brne L%=_B4_high_wait\n" // loop if the counter isn't 0
                // Timeout: Exit
                // TODO: Different action? Different return value?
                "rjmp L%=_exit\n"
            //<------------------------------------------

            // TODO: Success, found initialization probe command! Now, respond!
            "L%=_respond_B:\n"
            // For now, just return a 2 for success
            "ldi %[retval],lo8(2)\n"
            "rjmp L%=_exit\n" 


 
            "L%=_exit:\n"
            
            ";END OF MANUAL ASSEMBLY BLOCK\n"
            // ----------
            // outputs:
            : [retval] "=r" (retval)
            // clobbers (registers we use in the assembly for the compiler to
            // avoid):
              // r24 = scratch register (ex: timeout down counter)
              // r25 = down counter for reading sequence of bits
            :: "r25", "r24"         
            );

    return retval;
}

static void print_gc_status()
{
    Serial.println();
    Serial.print("Start: ");
    Serial.println(gc_status.data1 & 0x10 ? 1:0);

    Serial.print("Y:     ");
    Serial.println(gc_status.data1 & 0x08 ? 1:0);

    Serial.print("X:     ");
    Serial.println(gc_status.data1 & 0x04 ? 1:0);

    Serial.print("B:     ");
    Serial.println(gc_status.data1 & 0x02 ? 1:0);

    Serial.print("A:     ");
    Serial.println(gc_status.data1 & 0x01 ? 1:0);

    Serial.print("L:     ");
    Serial.println(gc_status.data2 & 0x40 ? 1:0);
    Serial.print("R:     ");
    Serial.println(gc_status.data2 & 0x20 ? 1:0);
    Serial.print("Z:     ");
    Serial.println(gc_status.data2 & 0x10 ? 1:0);

    Serial.print("Dup:   ");
    Serial.println(gc_status.data2 & 0x08 ? 1:0);
    Serial.print("Ddown: ");
    Serial.println(gc_status.data2 & 0x04 ? 1:0);
    Serial.print("Dright:");
    Serial.println(gc_status.data2 & 0x02 ? 1:0);
    Serial.print("Dleft: ");
    Serial.println(gc_status.data2 & 0x01 ? 1:0);

    Serial.print("Stick X:");
    Serial.println(gc_status.stick_x, DEC);
    Serial.print("Stick Y:");
    Serial.println(gc_status.stick_y, DEC);

    Serial.print("cStick X:");
    Serial.println(gc_status.cstick_x, DEC);
    Serial.print("cStick Y:");
    Serial.println(gc_status.cstick_y, DEC);

    Serial.print("L:     ");
    Serial.println(gc_status.left, DEC);
    Serial.print("R:     ");
    Serial.println(gc_status.right, DEC);
    Serial.flush();
}

static void print_bits_read(const uint8_t* data, uint8_t data_length, uint8_t bitsRead) {
  Serial.print("Bits read: ");
  Serial.println(bitsRead);

  // Output the full 8 bytes of data
  for(int i = 0; i < data_length; i++) {
    // Output each bit in the uint8_t in MSB order (which is read order)
    uint8_t bitPlace = 0x80; // 128
    for(int j = 7; j >= 0; j--) {
      Serial.print(*(data+i) & bitPlace ? 1:0); // print out current bit
      bitPlace /= 2; // 128 -> 64 -> 32 -> 16 -> 8 -> 4 -> 2 -> 1
    }
    Serial.print(" "); // Byte spacer
  }
  Serial.println();
}


/****** FUNCTIONS: Act As functions *******/

static bool rumble = false;
// Act as a console and read from an attached GC controller
// NOTE: Must have a pull-up resistor between data line and 3.3V!
inline void act_console() {
  int status;
  unsigned char data, addr;
  
  // clear out incomming raw data buffer
  // this should be unnecessary
  //memset(gc_raw_dump, 0, sizeof(gc_raw_dump));
  
  // Command to send to the gamecube
  // The last bit is rumble, flip it to rumble
  unsigned char command[] = {0x40, 0x03, 0x00};
  if (rumble) {
      command[2] = 0x01;
  }
  
  // turn on the led, so we can visually see things are happening
  digitalWrite(13, LOW);
  // don't want interrupts getting in the way
  noInterrupts();
  // send those 3 bytes
  gc_send(command, 3);
  // read in data and dump it to gc_raw_dump
  status = gc_get();
  // end of time sensitive code
  interrupts();
  digitalWrite(13, HIGH);
  
  if (status == 0) {
    // problem with getting the gamecube controller status. Maybe it's unplugged?
    // set a neutral N64 string
    Serial.print(millis(), DEC);
    Serial.println(" | GC controller read error. Trying to re-initialize");
    Serial.flush();
   // memset(n64_buffer, 0, sizeof(n64_buffer));
    memset(&gc_status, 0, sizeof(gc_status));
    gc_status.stick_x = zero_x;
    gc_status.stick_y = zero_y;
    // this may not work if the controller isn't plugged in, but if it
    // fails we'll try again next loop
    noInterrupts();
    init_gc_controller();
    interrupts();
  } else {
      // translate the data to the n64 byte string
        // gc_to_64();
  }
  
  // Wait for incomming 64 command
  // this will block until the N64 sends us a command
  noInterrupts();
  // get_n64_command();
  
  interrupts();
  
  // DEBUG: print it
  print_gc_status();
}

// Act as a controller and respond with data according to commands coming in
inline void act_controller() {
  // TODO: Everything
}

// Be a third party and simply observe transactions on the data line
inline void act_thirdparty() {
  // Initialize 8 bytes worth of space to read
  const uint8_t SIZE = 16;
  uint8_t data[SIZE] = {0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0};

  // Read in 16 bytes max
  noInterrupts();
  int bitsRead = 0;
  while(!bitsRead)
    bitsRead = gc_read(data, 128);
  interrupts();

  // Output read data
  print_bits_read(data, SIZE, bitsRead);
  
}

inline void act_test() {
  // Initialize 8 bytes worth of space to read
  int response = 0;

  // Keep reading until read more than 9 bits and first 9 bits is the initial probe command
  noInterrupts();
  do {
    response = gc_respond();
  }
  while(response == 0);
  interrupts();

  Serial.println(response);
}

/****** FUNCTIONS: Base Arduino functions *******/

void setup()
{
  Serial.begin(115200);

  Serial.println();
  Serial.println("Code has started!");
  Serial.flush();

  // Status LED
  digitalWrite(13, LOW);
  pinMode(13, OUTPUT);

  // Communication with gamecube controller on this pin
  // Don't remove these lines, we don't want to push +5V to the controller
  digitalWrite(GC_PIN, LOW);  
  pinMode(GC_PIN, INPUT);

  if(CUR_MODE == As_Console) {
    noInterrupts();
    init_gc_controller();
    interrupts();
  
    do {
      // Query for the gamecube controller's status. We do this
      // to get the 0 point for the control stick.
      unsigned char command[] = {0x40, 0x03, 0x00};
      noInterrupts();
      gc_send(command, 3);
      // read in data and dump it to gc_raw_dump
      gc_get();
      interrupts();
      zero_x = gc_status.stick_x;
      zero_y = gc_status.stick_y;
      Serial.print("GC zero point read: ");
      Serial.print(zero_x, DEC);
      Serial.print(", ");
      Serial.println(zero_y, DEC);
      Serial.flush();
      
      // some crappy/broken controllers seem to give bad readings
      // occasionally. This is a cheap hack to keep reading the
      // controller until we get a reading that is less erroneous.
    } while (zero_x == 0 || zero_y == 0);
  }  
  // Otherwise, initialize the gc status for default data to send
  else {
    gc_status.data1 = 0x00;
    gc_status.data2 = 0x80;
    gc_status.stick_x = 0x7A;
    gc_status.stick_y = 0x7B;
    gc_status.cstick_x = 0x7E;
    gc_status.cstick_y = 0x83;
    gc_status.left = 0x74;
    gc_status.right = 0x17;
  }
}

void loop()
{
  // Call the appropriate function according to the pre-set mode
    // Trying to encapsulate the behavior properly to make it easier to separate into a library
  switch(CUR_MODE) {
    case As_Console:
      act_console();
      break;

    case As_Controller:
      act_controller;
      break;

    case As_ThirdParty:
      act_thirdparty();
      break;

    case As_Test:
      act_test();
      
      break;

    default:
      // Should never reach here!
      Serial.println("\r\nERROR: GC_Mode CUR_MODE undefined!\r\m");
      break;
  }
}

