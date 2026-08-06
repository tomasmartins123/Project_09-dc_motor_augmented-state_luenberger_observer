// Project CRIA 09 - Real-Time State-Space Disturbance Observer
// Runs the physical motors together with a discrete state-space disturbance
// observer to estimate motor speed and equivalent input disturbances in real time.

// -----------------------------------------------------------------------------
// Hardware Pin Assignments
// -----------------------------------------------------------------------------
const int PWM_LEFT_PIN  = 5;   // Left Motor PWM (PWM1)
const int DIR_LEFT_PIN  = A0;  // Left Motor Direction (DIR1)
const int PWM_RIGHT_PIN = 6;   // Right Motor PWM (PWM2)
const int DIR_RIGHT_PIN = A2;  // Right Motor Direction (DIR2)

const int ENC_LEFT_PIN  = 2;   // Left Encoder Interrupt (D2)
const int ENC_RIGHT_PIN = 3;   // Right Encoder Interrupt (D3);

// -----------------------------------------------------------------------------
// Encoder Interrupt Variables
// M/T method:
//  - sum_dt stores the accumulated time between encoder transitions.
//  - count_dt stores the number of measured periods.
//  - last_time stores the timestamp of the last valid transition.
// -----------------------------------------------------------------------------
volatile unsigned long sum_dt_left    = 0;
volatile unsigned int  count_dt_left  = 0;
volatile unsigned long last_time_left = 0;

volatile unsigned long sum_dt_right    = 0;
volatile unsigned int  count_dt_right  = 0;
volatile unsigned long last_time_right = 0;

// -----------------------------------------------------------------------------
// Last Valid RPM Values
// Used for Zero-Order Hold when no encoder transitions are detected.
// -----------------------------------------------------------------------------
float last_rpm_left = 0.0f;
float last_rpm_right = 0.0f;


// -----------------------------------------------------------------------------
// First-order exponential low-pass filter coefficient.
//
// Encoder measurements contain quantization noise and small fluctuations.
// Before entering the observer, the measured RPM is smoothed according to
//
//      y_f[k] = α·y[k] + (1-α)·y_f[k-1]
//
// where α controls the trade-off between responsiveness and noise reduction.
// -----------------------------------------------------------------------------
const float ALPHA = 0.30f;   
float rpm_L_filtered = 0.0f;
float rpm_R_filtered = 0.0f;


// -----------------------------------------------------------------------------
// Disturbance Leakage Factor
//
// Prevents long-term drift of the disturbance estimate by slowly pulling it
// back toward zero when no persistent disturbance is present.
// -----------------------------------------------------------------------------
const float DISTURBANCE_LEAKAGE = 0.97f;

// -----------------------------------------------------------------------------
// Experiment Parameters
// -----------------------------------------------------------------------------
const unsigned long SAMPLE_TIME_MS = 40;     // Sampling period (25 Hz, Ts = 0.04 s)
const unsigned long TEST_DURATION_MS = 18000; // Total experiment duration (18 s)

unsigned long start_time = 0;
unsigned long last_sample_time = 0;

bool test_active = true;

// -----------------------------------------------------------------------------
// Observer Parameters
// Identified discrete state-space matrices (Ts = 40 ms)
// x[k+1] = A·x[k] + B·u[k]
// The values for each matrix correspond to the identified discrete motor models.
// -----------------------------------------------------------------------------
const float A_L = 0.810f; 
const float B_L = 0.255f;

const float A_R = 0.833f;
const float B_R = 0.225f;

// -----------------------------------------------------------------------------
// Luenberger Disturbance Observer Gains
// -----------------------------------------------------------------------------
const float L1_L = 0.550f;
const float L2_L = 0.110f;

const float L1_R = 0.573f;
const float L2_R = 0.124f;

// -----------------------------------------------------------------------------
// Estimated States
// -----------------------------------------------------------------------------
float omega_hat_L = 0.0f;
float d_hat_L = 0.0f;

float omega_hat_R = 0.0f;
float d_hat_R = 0.0f;

// -----------------------------------------------------------------------------
// Applied Motor Commands
// -----------------------------------------------------------------------------
float u_applied_L = 0.0f;
float u_applied_R = 0.0f;

// -----------------------------------------------------------------------------
// Left Encoder Interrupt Service Routine
// Triggered on every encoder edge (CHANGE).
// Measures the time between two consecutive valid transitions.
// Transitions separated by less than 1 ms are ignored because they are most
// likely caused by electrical noise or signal bouncing.
// -----------------------------------------------------------------------------
void ISR_count_left() {

  unsigned long now = micros();
  unsigned long dt = now - last_time_left;

  if (dt >= 1000) {

    sum_dt_left += dt;
    count_dt_left++;
    last_time_left = now;

  }
}

// -----------------------------------------------------------------------------
// Right Encoder Interrupt Service Routine
// Same logic as the left encoder.
// -----------------------------------------------------------------------------
void ISR_count_right() {

  unsigned long now = micros();
  unsigned long dt = now - last_time_right;

  if (dt >= 1000) {

    sum_dt_right += dt;
    count_dt_right++;
    last_time_right = now;

  }
}
void setup() {

  Serial.begin(115200);

  // Configure Motor Control Pins
  pinMode(PWM_LEFT_PIN, OUTPUT);
  pinMode(DIR_LEFT_PIN, OUTPUT);
  pinMode(PWM_RIGHT_PIN, OUTPUT);
  pinMode(DIR_RIGHT_PIN, OUTPUT);

  // Set Motor Direction for forward motion
  digitalWrite(DIR_LEFT_PIN, LOW);
  digitalWrite(DIR_RIGHT_PIN, HIGH);

  // Configure Encoder Pins
  pinMode(ENC_LEFT_PIN, INPUT_PULLUP);
  pinMode(ENC_RIGHT_PIN, INPUT_PULLUP);

  // Interrupt on both rising and falling edges to double the number of
  // measured transitions and improve RPM resolution.
  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_PIN), ISR_count_left, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_PIN), ISR_count_right, CHANGE);

  // CSV Column Headers used in MATLAB
  Serial.println("time_ms,rpm_L_real,omega_hat_L,d_hat_L,u_L,rpm_R_real,omega_hat_R,d_hat_R,u_R");

  start_time = millis();
  last_sample_time = start_time;
}

void loop() {

  unsigned long current_time = millis();

  if (test_active && (current_time - last_sample_time >= SAMPLE_TIME_MS)) {

    last_sample_time = current_time;
    unsigned long elapsed_ms = current_time - start_time;

    // -------------------------------------------------------------------------
    // Stop the experiment after the complete observer response has been recorded.
    // -------------------------------------------------------------------------
    if (elapsed_ms >= TEST_DURATION_MS) {

      analogWrite(PWM_LEFT_PIN, 0);
      analogWrite(PWM_RIGHT_PIN, 0);

      test_active = false;
      return;

    }

    // -------------------------------------------------------------------------
    // Copy interrupt variables atomically.
    // The accumulators are reset so that each sampling instant processes only
    // the encoder information acquired during the current 40 ms window.
    // -------------------------------------------------------------------------
    noInterrupts();

    unsigned long sum_L   = sum_dt_left;
    unsigned int  count_L = count_dt_left;
    sum_dt_left   = 0;
    count_dt_left = 0;
    unsigned long time_since_L = micros() - last_time_left;

    unsigned long sum_R   = sum_dt_right;
    unsigned int  count_R = count_dt_right;
    sum_dt_right   = 0;
    count_dt_right = 0;
    unsigned long time_since_R = micros() - last_time_right;

    interrupts();

    // -------------------------------------------------------------------------
    // 1. Measure Physical Speeds (RPM)
    // -------------------------------------------------------------------------

    // -------------------------------------------------------------------------
    // LEFT MOTOR
    //
    // Average encoder period:
    //
    //      avg_dt = Σ(dt)/N
    //
    // Instantaneous speed:
    //
    //      RPM = 1 500 000 / avg_dt
    //
    // where:
    //   60×10^6 converts microseconds into minutes.
    //   40 encoder transitions correspond to one wheel revolution.
    //   Using CHANGE interrupts doubles the number of detected transitions,
    //   giving a constant of 1 500 000.
    // -------------------------------------------------------------------------
    float rpm_L_avg = 0.0f;

    if (count_L > 0) {

      float avg_dt_L = (float)sum_L / (float)count_L;

      rpm_L_avg = 1500000.0f / avg_dt_L;

    }
    else if (time_since_L > 200000) {

      // No transitions for 200 ms -> motor assumed stopped
      rpm_L_avg = 0.0f;

    }
    else {

      // Zero-Order Hold
      rpm_L_avg = last_rpm_left;

    }


    // -------------------------------------------------------------------------
    // First-order exponential low-pass filtering.
    //
    // The raw encoder measurement is filtered before being used by the observer.
    // This reduces high-frequency measurement noise while preserving the overall
    // motor dynamics.
    // -------------------------------------------------------------------------
    rpm_L_filtered =
     ALPHA * rpm_L_avg +
      (1.0f - ALPHA) * rpm_L_filtered;

    float rpm_L_real = rpm_L_filtered;
    last_rpm_left = rpm_L_real;

    // -------------------------------------------------------------------------
    // RIGHT MOTOR
    // Same processing chain as the left motor.
    // -------------------------------------------------------------------------
    float rpm_R_avg = 0.0f;

    if (count_R > 0) {

      float avg_dt_R = (float)sum_R / (float)count_R;

      rpm_R_avg = 1500000.0f / avg_dt_R;

    }
    else if (time_since_R > 200000) {

      // No transitions for 200 ms -> motor assumed stopped
      rpm_R_avg = 0.0f;

    }
    else {

      // Zero-Order Hold
      rpm_R_avg = last_rpm_right;

    }

    // Same filtering procedure as the left motor.
    rpm_R_filtered =
      ALPHA * rpm_R_avg +
      (1.0f - ALPHA) * rpm_R_filtered;

    float rpm_R_real = rpm_R_filtered;
    last_rpm_right = rpm_R_real;
       
  // -------------------------------------------------------------------------
  // Generate the observer validation input.
  //
  // 0 - 2 s  : Motor stopped
  // 2 - 18 s  : Constant PWM step
  // After 18 s: Motor stopped and experiment finished
  // -------------------------------------------------------------------------
    int u_ref = 0;

    if (elapsed_ms >= 2000) {

      u_ref = 100;

    }

    // 3. Left Disturbance Observer
    //
    // Observer equations:
    //
    //      x̂[k+1] = A·x̂[k] + B·(u+d̂) + L₁·e
    //      d̂[k+1] = λ·d̂[k] + L₂·e
    // where λ is the disturbance leakage factor.

    float error_L = rpm_L_real - omega_hat_L;

    float omega_next_L =
      A_L * omega_hat_L +
      B_L * (u_applied_L + d_hat_L) +
      L1_L * error_L;


    // -------------------------------------------------------------------------
    // Disturbance state update.
    //
    // A small leakage factor (<1) is applied to the disturbance estimate:
    //
    //      d̂[k+1] = λ·d̂[k] + L₂·e
    //
    // instead of
    //
    //      d̂[k+1] = d̂[k] + L₂·e
    //
    // This prevents long-term drift caused by small persistent modelling errors
    // or measurement offsets, allowing the disturbance estimate to slowly return
    // toward zero when no external disturbance is present.
    // -------------------------------------------------------------------------
    float d_next_L =
      DISTURBANCE_LEAKAGE* d_hat_L +
      L2_L * error_L;

    omega_hat_L = omega_next_L;
    d_hat_L = d_next_L;

    // -------------------------------------------------------------------------
    // 4. Right Disturbance Observer
    // Same observer structure as the left motor.
    // -------------------------------------------------------------------------
    float error_R = rpm_R_real - omega_hat_R;

    float omega_next_R =
      A_R * omega_hat_R +
      B_R * (u_applied_R + d_hat_R) +
      L1_R * error_R;

    // Same disturbance observer update as the left motor.
    float d_next_R =
      DISTURBANCE_LEAKAGE * d_hat_R +
      L2_R * error_R;

    omega_hat_R = omega_next_R;
    d_hat_R = d_next_R;

    // -------------------------------------------------------------------------
    // 5. Disturbance Compensation
    //
    // The estimated disturbance is compensated through feedforward action:
    //
    //      u = u_ref − d̂
    //
    // The PWM command is saturated to the valid Arduino range.
    // -------------------------------------------------------------------------
    u_applied_L =
      constrain((float)u_ref - d_hat_L, 0.0f, 255.0f);

    u_applied_R =
      constrain((float)u_ref - d_hat_R, 0.0f, 255.0f);

    // Apply PWM to the physical motors
    analogWrite(PWM_LEFT_PIN, (int)u_applied_L);
    analogWrite(PWM_RIGHT_PIN, (int)u_applied_R);

    // -------------------------------------------------------------------------
    // Stream experimental and observer data to the PC
    // -------------------------------------------------------------------------
    Serial.print(elapsed_ms);
    Serial.print(",");
    Serial.print(rpm_L_real);
    Serial.print(",");
    Serial.print(omega_hat_L);
    Serial.print(",");
    Serial.print(d_hat_L);
    Serial.print(",");
    Serial.print(u_applied_L);
    Serial.print(",");
    Serial.print(rpm_R_real);
    Serial.print(",");
    Serial.print(omega_hat_R);
    Serial.print(",");
    Serial.print(d_hat_R);
    Serial.print(",");
    Serial.println(u_applied_R);

  }
    // -------------------------------------------------------------------------
  // After the experiment finishes, keep both motors disabled.
  // -------------------------------------------------------------------------
  if (!test_active) {

    analogWrite(PWM_LEFT_PIN, 0);
    analogWrite(PWM_RIGHT_PIN, 0);

  }

}