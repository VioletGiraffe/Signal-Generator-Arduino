// =============================================================================
//  Arduino Due -- Sine Wave Generator
//  Output : DAC0 (pin DAC0 / A13), 0-3.3 V, 12-bit
//  Method : 4096-point LUT + linear interpolation, TC0 timer ISR
// =============================================================================
//
//  Wiring
//  ------
//  DAC0 (A13)  ->  amplifier input (via DC-blocking cap if needed)
//  GND         ->  amplifier GND
//
//  The DAC mid-scale sits at ~1.65 V (code 2048).
//  Full-scale swing is 0 V - 3.3 V (codes 0 - 4095).
//  Use a series 10 uF electrolytic (or 100 uF for low frequencies) to remove
//  the DC offset before feeding a real amplifier input.
//
//  Usage
//  ------
//  * Set SAMPLE_RATE and FREQUENCY below.
//  * Or send a new frequency over Serial (115200 baud):
//        "f 440\n"   -> sets 440 Hz
//        "s 96000\n" -> sets 96 000 Hz sample rate (restarts timer)
//        "?\n"       -> prints current settings
//
//  ------
//  Practical reliable sample rate ~170-200 kHz with this ISR approach (see bottom of file for details).
// =============================================================================

#include <Arduino.h>
#include <tm1638-library/TM1638.h>

TM1638 module(A7, A6, A5);

// -- User-configurable defaults ------------------------------------------------
static uint32_t SAMPLE_RATE = 200000UL; // Hz
static double FREQUENCY = 20000.0f;		// Hz

// -- Constants -----------------------------------------------------------------
inline constexpr uint LUT_SIZE = 4096u;
inline constexpr uint LUT_MASK = LUT_SIZE - 1; // 0x0FFF  (LUT_SIZE must stay power-of-2)

// -- Sine LUT (built once in setup) --------------------------------------------
static uint16_t sineLUT[LUT_SIZE];

static void buildLUT()
{
	for (uint32_t i = 0; i < LUT_SIZE; i++)
	{
		// Full-range: 0 ... 4095, centred at 2047.5
		// Using double for precision at low frequencies
		static constexpr double midpoint = 2047.5;
		sineLUT[i] = (uint16_t)::lround(sin(2.0 * M_PI * (double)i / (double)LUT_SIZE) * midpoint + midpoint);
	}
}

// -- Phase accumulator ---------------------------------------------------------
//  32-bit accumulator, bits 31..20 - LUT index (0-4095), 19..0 - interpolation fraction (0 = 0 %, 0xFFFFF ~ 100%)
//  phaseIncrement = round( frequency / sampleRate * 2^32 )
// -----------------------------------------------------------------------------
static volatile uint32_t phaseAccum = 0;
static volatile uint32_t phaseIncrement = 0;

static void setFrequency(double freq)
{
	// Use double for precision at low frequencies
	FREQUENCY = freq;
	phaseIncrement = ::lround(freq / (double)SAMPLE_RATE * 4294967296.0);

	module.setDisplayToDecNumber((int)FREQUENCY, 0);
}

// -- Timer ISR -- TC0 channel 0 ------------------------------------------------
void TC0_Handler()
{
	// Reading TC_SR acknowledges and clears the interrupt flags
	(void)TC0->TC_CHANNEL[0].TC_SR;

	// Advance phase
	uint32_t phase = phaseAccum;
	phaseAccum = phase + phaseIncrement;

	// Split into integer index and fractional part
	uint32_t idx = phase >> 20;			  // top 12 bits  -> 0 ... 4095
	uint32_t frac = phase & 0x000FFFFFUL; // bottom 20 bits -> 0 ... 1 048 575

	// Linear interpolation between adjacent LUT samples
	int32_t a = sineLUT[idx];
	int32_t b = sineLUT[(idx + 1u) & LUT_MASK];
	int32_t value = a + (((b - a) * (int32_t)frac) >> 20);

	// Write directly to DAC conversion register (fastest path, no API overhead)
	DACC->DACC_CDR = (uint32_t)value;
}

// -- Timer setup ---------------------------------------------------------------
static void startTimer(uint32_t sampleRate)
{
	// TC0, channel 0  ->  IRQ TC0_IRQn
	pmc_enable_periph_clk(ID_TC0);

	TC_Configure(TC0, 0,
				 TC_CMR_WAVE |					// Waveform mode
				 TC_CMR_WAVSEL_UP_RC |		// Count up; reset on RC compare
				 TC_CMR_TCCLKS_TIMER_CLOCK1 // Clock = MCK / 2 = 42 MHz
	);

	// RC value that fires exactly at sampleRate
	// Actual rate = 42 000 000 / RC  (small rounding error, <0.1 % typically)
	uint32_t rc = (VARIANT_MCK / 2u) / sampleRate;
	if (rc < 2u)
		rc = 2u; // hardware minimum
	TC_SetRC(TC0, 0, rc);
	TC_Start(TC0, 0);

	// Enable RC compare match interrupt only
	TC0->TC_CHANNEL[0].TC_IER = TC_IER_CPCS;
	TC0->TC_CHANNEL[0].TC_IDR = ~TC_IER_CPCS;

	NVIC_SetPriority(TC0_IRQn, 0); // highest priority -> lowest jitter
	NVIC_EnableIRQ(TC0_IRQn);
}

static void stopTimer()
{
	NVIC_DisableIRQ(TC0_IRQn);
	TC_Stop(TC0, 0);
}

// -- DAC initialisation --------------------------------------------------------
static void initDAC()
{
	pmc_enable_periph_clk(ID_DACC);

	DACC->DACC_CR = DACC_CR_SWRST; // software reset

	DACC->DACC_MR = DACC_MR_TRGEN_DIS			// software trigger (write to CDR starts conversion)
					| DACC_MR_USER_SEL_CHANNEL0 // fixed channel 0 (DAC0 pin)
					| DACC_MR_WORD_HALF			// 12-bit half-word mode
					| DACC_MR_REFRESH(1)		// refresh every ~23 us (min value)
					| DACC_MR_STARTUP_8;		// 8-period start-up (fastest)

	DACC->DACC_CHER = DACC_CHER_CH0; // enable channel 0
}

// -- Serial command parser -----------------------------------------------------
static void printStatus()
{
	uint32_t rc = TC0->TC_CHANNEL[0].TC_RC;
	float actualRate = (float)(VARIANT_MCK / 2u) / (float)rc;
	float actualFreq = (float)phaseIncrement / 4294967296.0f * actualRate;

	Serial.println("-----------------------------");
	Serial.print("  Target sample rate : ");
	Serial.print(SAMPLE_RATE);
	Serial.println(" Hz");
	Serial.print("  Actual sample rate : ");
	Serial.print(actualRate, 1);
	Serial.println(" Hz");
	Serial.print("  Target frequency   : ");
	Serial.print(FREQUENCY, 3);
	Serial.println(" Hz");
	Serial.print("  Actual frequency   : ");
	Serial.print(actualFreq, 3);
	Serial.println(" Hz");
	Serial.print("  Phase increment    : 0x");
	Serial.println(phaseIncrement, HEX);
	Serial.println("-----------------------------");
}

static void handleSerial()
{
	static String cmdBuf;

	while (Serial.available())
	{
		char c = (char)Serial.read();
		if (c == '\n' || c == '\r')
		{
			cmdBuf.trim();
			if (cmdBuf.length() == 0)
			{
				cmdBuf = "";
				return;
			}

			if (cmdBuf == "?")
			{
				printStatus();
			}
			else if (cmdBuf.startsWith("f ") || cmdBuf.startsWith("F "))
			{
				double f = cmdBuf.substring(2).toDouble();
				if (f > 0.0 && f < SAMPLE_RATE / 2.0)
				{
					FREQUENCY = f;
					setFrequency(FREQUENCY);
					Serial.print("Frequency -> ");
					Serial.print(FREQUENCY, 3);
					Serial.println(" Hz");
					printStatus();
				}
				else
				{
					Serial.println("Error: frequency out of range (0 < f < Fs/2)");
				}
			}
			else if (cmdBuf.startsWith("s ") || cmdBuf.startsWith("S "))
			{
				uint32_t sr = (uint32_t)cmdBuf.substring(2).toInt();
				if (sr >= 8000UL && sr <= 400000UL)
				{
					stopTimer();
					SAMPLE_RATE = sr;
					setFrequency(FREQUENCY);
					startTimer(SAMPLE_RATE);
					Serial.print("Sample rate -> ");
					Serial.print(SAMPLE_RATE);
					Serial.println(" Hz");
					printStatus();
				}
				else
				{
					Serial.println("Error: sample rate out of range (8000 - 400000)");
				}
			}
			else
			{
				Serial.println("Commands:  f <Hz>   s <Hz>   ?");
			}
			cmdBuf = "";
		}
		else
		{
			cmdBuf += c;
		}
	}
}

// -- Arduino entry points ------------------------------------------------------
void setup()
{
	Serial.begin(9600);
	while (!Serial && millis() < 500); // wait for Serial Monitor

	buildLUT();
	setFrequency(FREQUENCY);
	initDAC();
	startTimer(SAMPLE_RATE);

	Serial.println("\nArduino Due Sine Generator -- ready");
	Serial.println("Commands:  f <Hz>   s <Hz>   ?");
	printStatus();

	module.setupDisplay(true, 7);
	module.setDisplayToDecNumber((int)FREQUENCY, 0);
}

void loop()
{
	handleSerial(); // non-blocking serial command handler

	//module.setLED(module.isButtonPressed(1) ? TM1638_COLOR_RED :  TM1638_COLOR_GREEN, 0);
	module.setLEDs(module.getButtons());
	module.setDisplayToStringWithDots("1.23.45  8.");
}

// =============================================================================
//  Practical maximum reliable sample rate -- explained
// =============================================================================
//
//  Clock chain
//  -----------
//  MCK = 84 MHz  ->  TC TIMER_CLOCK1 = MCK/2 = 42 MHz
//  Minimum RC register value = 2  ->  absolute ceiling = 21 MHz ISR rate.
//  In practice the ISR itself takes CPU time:
//
//  TC0_Handler body (worst-case ARM Cortex-M3 cycle count estimate)
//  ----------------------------------------------------------------
//   ISR entry / exit (stacking, unstacking)    ~  12 cycles
//   TC_SR read (32-bit peripheral read)         ~   4 cycles
//   32-bit add (phase accumulator)              ~   1 cycle
//   32-bit shift + mask (index / frac)          ~   2 cycles
//   Two 16-bit LUT reads                        ~   6 cycles (cache hit)
//   32-bit multiply (interpolation)             ~   3-5 cycles (M3 1-cycle MUL)
//   Arithmetic shift right 20                   ~   1 cycle
//   DACC CDR write (32-bit peripheral write)    ~   4 cycles
//   -------------------------------------------------------------
//   Total                                       ~ 33-40 cycles
//
//  At 84 MHz that implies a theoretical maximum of ~2.1-2.5 MHz ISR rate.
//  However, peripheral bus latency, cache misses on the first LUT access, and
//  the NVIC overhead in the real silicon are higher than ideal estimates.