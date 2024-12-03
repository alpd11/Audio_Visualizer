#include <Arduino.h>
#include <DFRobot_RGBPanel.h>
#include <arduinoFFT.h>
#include <Wire.h>
#include <KY040.h>

// Define the microphone input pin and FFT parameters
const int MIC_PIN = A0; // Analog pin connected to the microphone
const int SAMPLES = 128; // Number of FFT samples, must be a power of 2 and ≤ 128
const double samplingFrequency = 40000.0; // Sampling frequency in Hz

// Define rotary encoder pins
const int CLK = 9; // Clock pin for rotary encoder
const int DT = 8;  // Data pin for rotary encoder

// Initialize FFT and LED Panel instances
double vReal[SAMPLES]; // Real component of FFT input
double vImag[SAMPLES]; // Imaginary component of FFT input
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, SAMPLES, samplingFrequency); // FFT object
DFRobot_RGBPanel matrix; // LED matrix panel object
KY040 g_rotaryEncoder(CLK, DT); // Rotary encoder object

// Variable to hold rotary encoder value
volatile int v_value = 1; // Current mode value, modified by rotary encoder

// Interrupt Service Routine (ISR) for rotary encoder
void ISR_rotaryEncoder() {
  switch (g_rotaryEncoder.getRotation()) {
    case KY040::CLOCKWISE:
      v_value++; // Increment mode on clockwise rotation
      break;
    case KY040::COUNTERCLOCKWISE:
      v_value--; // Decrement mode on counterclockwise rotation
      break;
  }
}

// Array to track the height of the previous column for the LED display
int previousHeights[16] = {0};

void setup() {
  Wire.begin(); // Initialize I2C communication
  Serial.begin(115200); // Start serial communication for debugging

  matrix.clear(); // Clear the LED matrix
  delay(100);

  // Set rotary encoder pins as input and attach interrupts
  pinMode(CLK, INPUT);
  pinMode(DT, INPUT);
  attachInterrupt(digitalPinToInterrupt(CLK), ISR_rotaryEncoder, CHANGE); // Interrupt for CLK pin
  attachInterrupt(digitalPinToInterrupt(DT), ISR_rotaryEncoder, CHANGE); // Interrupt for DT pin

  Serial.println("Setup complete. Starting audio visualizer...");
}

void loop() {
  static int lastValue = 0; // To track changes in rotary encoder value
  int value;

  // Disable interrupts to safely read the rotary encoder value
  cli();
  value = v_value;
  sei();

  // If rotary encoder value changes, update the display
  if (lastValue != value) {
    matrix.clear(); // Clear the matrix display
    delay(200);     // Debounce delay
    Serial.println(value); // Debug print the new value
    lastValue = value;
  }

  // Constrain rotary encoder value to valid range (1 to 7)
  if (v_value > 7) v_value = 1;
  if (v_value < 1) v_value = 7;

  // Collect audio samples from the microphone
  for (int i = 0; i < SAMPLES; i++) {
    vReal[i] = analogRead(MIC_PIN) - 512; // Center signal around 0
    vImag[i] = 0; // Initialize imaginary part to 0
    Serial.println(vReal[i]); // Debug print raw audio data
    delayMicroseconds(1000000 / samplingFrequency); // Control sampling interval
  }

  // Perform FFT on collected samples
  FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD); // Apply Hamming window
  FFT.compute(FFT_FORWARD); // Compute FFT
  FFT.complexToMagnitude(); // Convert FFT results to magnitudes

  // Map FFT results to frequency bands for LED visualization
  double frequencyResolution = samplingFrequency / SAMPLES; // Resolution of FFT bins
  double maxFrequency = samplingFrequency / 2; // Nyquist frequency
  double bandWidth = maxFrequency / 16; // Divide into 16 frequency bands

  for (int x = 0; x < 16; x++) {
    // Calculate FFT bins for the current frequency band
    int startBin = (int)((x * bandWidth) / frequencyResolution);
    int endBin = (int)(((x + 1) * bandWidth) / frequencyResolution);

    // Compute average amplitude for the band
    double bandAmplitude = 0;
    for (int k = startBin; k < endBin && k < (SAMPLES / 2); k++) {
      bandAmplitude += vReal[k];
    }
    bandAmplitude /= (endBin - startBin);

    // Convert amplitude to decibels and map to LED column height
    double dB = 20 * log10(bandAmplitude + 1); // Avoid log(0) by adding 1
    int yPos = map(dB, 0, 50, 0, 7); // Map dB value to LED rows
    yPos = constrain(yPos, 0, 7); // Ensure within valid range

    // Update LED matrix only if the column height changes
    if (yPos != previousHeights[x]) {
      // Clear the column
      for (int y = 0; y <= 7; y++) {
        matrix.pixel(x, y, QUENCH); // Turn off pixels
      }

      // Set LED color based on current mode
      unsigned char color = RED; // Default color
      switch (v_value) {
        case 1: color = RED; break;
        case 2: color = GREEN; break;
        case 3: color = BLUE; break;
        case 4: color = YELLOW; break;
        case 5: color = CYAN; break;
        case 6: color = PURPLE; break;
        case 7: color = WHITE; break;
      }

      // Light up the column to the computed height
      for (int y = 0; y <= yPos; y++) {
        matrix.pixel(x, 7 - y, color); // Draw pixels from bottom up
      }

      previousHeights[x] = yPos; // Update height tracking
    }
  }

  delay(1); // Short delay for smoother operation
}
