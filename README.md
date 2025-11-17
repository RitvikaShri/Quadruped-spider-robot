# 🕷️ Arduino Quadruped Spider Robot

An Arduino-based 4-legged “spider” robot with 12 DOF (3 servos per leg), inverse kinematics–based leg control, and basic autonomous obstacle avoidance using an ultrasonic sensor.

The robot can walk forward and backward, turn on the spot, stand, sit, and perform fun gestures like hand-waving and hand-shaking.

---

## 📸 Demo & Screenshots


### Robot Overview  
![Robot Overview](https://github.com/RitvikaShri/Quadruped-spider-robot/blob/main/img_3.jpeg)

### Walking Gait  
![Walking Gait](https://github.com/RitvikaShri/Quadruped-spider-robot/blob/main/WhatsApp%20Video%202025-11-17%20at%2021.16.59%20(1).gif)

### Obstacle Avoidance in Action  
![Obstacle Avoidance](images/obstacle-avoidance.jpg)

### Hardware Setup  
![Hardware Setup](https://github.com/RitvikaShri/Quadruped-spider-robot/raw/main/img_2.jpeg)


---

## ✨ Features

- 🦵 **12 DOF Quadruped**
  - 4 legs × 3 servos each, controlled via a 2D servo array `servo[4][3]`.
- 🧠 **Inverse Kinematics**
  - Each leg is controlled in 3D space using Cartesian coordinates `(x, y, z)`.
  - Coordinates are converted to joint angles (`alpha`, `beta`, `gamma`) and then to servo positions.
- 🔁 **Smooth, Timer-Based Motion**
  - Uses `FlexiTimer2` to run a servo service routine at ~50 Hz.
  - Legs move from `site_now` to `site_expect` in straight lines with calculated speeds.
- 🚶 **Gaits & Poses**
  - `stand()` / `sit()` – smooth transitions between rest and standing height.
  - `step_forward(step)` – walk forward.
  - `step_back(step)` – walk backward.
  - `turn_left(step)` / `turn_right(step)` – spot turning using precomputed turning positions.
- 📡 **Autonomous Obstacle Avoidance**
  - Ultrasonic sensor constantly checks distance to obstacles.
  - If an object is detected closer than a threshold:
    - Robot steps back.
    - Randomly turns left or right to avoid the obstacle.
  - Otherwise, it keeps walking forward.
- 🤝 **Interactive Gestures**
  - `hand_wave(i)` – friendly waving motion using a front leg.
  - `hand_shake(i)` – up-and-down handshake motion.
  - `body_left(i)` / `body_right(i)` – small lateral body shifts.

---

## 🧩 Hardware

- Arduino Uno / Nano (or compatible board)
- 12× Micro servos (e.g., SG90 / MG90S) – 3 per leg
- 1× Ultrasonic distance sensor (HC-SR04 or compatible)
- External power supply for servos (recommended)
- Quadruped chassis (3D-printed, laser-cut, or kit)
- Jumper wires, breadboard / PCB, and mounting hardware

---

## 📚 Libraries Used

Make sure these libraries are installed in your Arduino IDE:

- `Servo` – standard Arduino servo library
- [`FlexiTimer2`](https://github.com/wimleers/FlexiTimer2) – timer interrupt for periodic servo updates
- [`NewPing`](https://bitbucket.org/teckel12/arduino-new-ping/wiki/Home) – easier ultrasonic distance measurement

You can install them via:

1. **Sketch → Include Library → Manage Libraries…**
2. Search for the library name.
3. Click **Install**.

For `FlexiTimer2` / `NewPing`, you may need to:
- Download the ZIP from the link.
- In Arduino IDE: **Sketch → Include Library → Add .ZIP Library…**

---

## 🧠 How It Works (High-Level)

### 1. Leg Coordinate System

Each leg’s foot position is represented as:

```cpp
volatile float site_now[4][3];    // current (x, y, z) for each leg
volatile float site_expect[4][3]; // target (x, y, z) for each leg

When you call set_site(leg, x, y, z), the code:
```
Computes the distance from current to target.

Calculates a per-axis speed for that leg using move_speed and speed_multiple.

Updates site_expect so the servo service can move towards it.

### 2. Smooth Motion via Timer Interrupt

```cpp
FlexiTimer2 runs servo_service() periodically (every 20 ms):

FlexiTimer2::set(20, servo_service);
FlexiTimer2::start();


Inside servo_service():

For each leg and axis, it moves site_now closer to site_expect using temp_speed.

Converts (x, y, z) to joint angles with cartesian_to_polar().

Converts those angles to servo commands using polar_to_servo().

Writes the final angle to each servo with servo[leg][j].write(angle);
```
This gives continuous, smooth motion.

### 3. Inverse Kinematics
```cpp
void cartesian_to_polar(volatile float &alpha,
                        volatile float &beta,
                        volatile float &gamma,
                        volatile float x,
                        volatile float y,
                        volatile float z);


Computes intermediate lengths based on leg geometry (length_a, length_b, length_c).

Uses trigonometry (atan2, acos) to calculate joint angles in radians.

Converts radians to degrees and returns alpha, beta, gamma.

polar_to_servo(leg, alpha, beta, gamma) then:

Adjusts angles depending on which leg (0–3) to account for orientation.

Offsets angles so they match the physical servo mounting.

Sends the final angles to servo[leg][0..2].
```

### 4. Obstacle Avoidance Logic
```cpp
In loop():

int uS = sonar.ping_cm();

if (uS < thresh && uS > 1) {
    step_back(3);
    delay(500);

    randNumber = random(1, 3);

    if (randNumber == 1) {
        turn_right(5);
    } else if (randNumber == 2) {
        turn_left(5);
    }
} else {
    step_forward(1);
}
```

If the measured distance is below thresh (e.g. 20 cm) and valid:

Robot backs up a bit.

Randomly decides to turn left or right.

If the path is clear:

Robot continues walking forward one step at a time.

# 🚀 Getting Started
### 1. Clone This Repository
git clone https://github.com/<your-username>/<your-repo-name>.git
cd <your-repo-name>

### 2. Open the Project in Arduino IDE

Open the .ino file that contains the main code.

Select your board and port under:

Tools → Board

Tools → Port

### 3. Install Required Libraries

Install the libraries listed in the Libraries Used section.

### 4. Wiring

Connect the 12 servos to digital pins:
2–13 according to:

const int servo_pin[4][3] = {
  {2, 3, 4},
  {5, 6, 7},
  {8, 9, 10},
  {11, 12, 13}
};


Connect the ultrasonic sensor:

#define TRIGGER_PIN  A1
#define ECHO_PIN     A2


Power the servos from a separate power supply (shared ground with Arduino).

### 5. Upload & Run

Click Upload in the Arduino IDE.

Once uploaded, the robot will:

Initialize servos.

Stand up.

Perform a small wave (hand_wave(3)).

Then enter the main loop and start walking / avoiding obstacles.

🧪 Example Behaviours

You can modify setup() to try different behaviors:

void setup() {
  // ... initialization code ...

  stand();
  delay(500);

  // Try different gestures:
  // hand_wave(3);
  // hand_shake(3);

  // Or start by walking:
  // step_forward(5);
}

# 🛠️ Customization Ideas

Adjust walking speed: tweak leg_move_speed, body_move_speed, spot_turn_speed.

Change step size: modify y_step, x_default, etc.

Tune obstacle threshold: change thresh (in cm).

Add more sensors (IR, IMU) or a Bluetooth/ESP8266 module for remote control.

Implement different gaits (crawl, trot, bound) by changing leg sequences.

# 📝 License

Add your preferred license here, for example:

MIT License – feel free to use, modify, and build on this project.

# 💬 Acknowledgements

Based on classic Arduino quadruped/spider robot designs and inverse kinematics examples.

Uses open-source libraries: Servo, FlexiTimer2, and NewPing.
