#Particle Photon Robot MK1

Mk1 is a behavior-based robotic rover that implements a simplified [subsumption architecture](https://en.wikipedia.org/wiki/Subsumption_architecture).

![Robot](https://raw.githubusercontent.com/travisterwilligar/Particle-Photon-Robot-MK1/master/mk1.png)

## Behaviors ##

MK1 has the following behaviors:

 1. **Manually start autopilot**: the user presses a button for the robot to start autopilot in the currently selected mode.
 2. **Manually select mode**: the user presses a button to cycle among the available autopilot modes.
 3. **Remote control**: using a web app, the user commands the robot to begin autopilot, change autopilot modes, or manually controls the robot to move in a specific direction.
 4. **Avoid cliffs**: using a time of flight sensor, the robot detects the presence of a drop and will automatically reverse and change direction to avoid falling off the cliff.
 5. **Obstacle sweep**: if the robot goes straight for a prolonged period of time and then encounters an obstacle ahead, it takes a sweep of the room, choosing the direction that has the longest length of open space and will change to that direction.
 6. **Avoid Obstacles**: the robot will avoid obstacles ahead of it, turning right to avoid them, and turning left after a prolonged number of right turns to prevent the robot from canyoning.
 7. **Detect impact**: occasionally, the robot's ultrasonic range finders won't respond to objects made of cloth (couches, curtains etc) and will not detect the presence of an obstacle. If the robot makes contact with another object as registered by the accelerometer, the detect impact command forces the robot to reverse and then change direction.
 8. **Light control**: uses one of three photoresistors to allow the user to control the robot using a flashlight (or other bright light source).
 9. **Follow walls**: uses an additional left-facing ultrasonic range finder to help the robot follow the perimeter of the room.
 10. **Autopilot (cruise)**: commands the robot to drive straight unless any higher-level behavior is triggered.
 11. **Headlights**: turns the headlights on/off or changes color depending on selected mode.
 12. **Sirens**: plays a beeping sound and changes the headlights an alternating red/white color when an obstacle is detected.

## Control Modes ##
Modes are higher-level actions that implement a subset of the available behaviors.

 1. Avoid obstacles: robot roams around avoiding obstacles, cliffs, and tries to find the best direction to travel based on avoiding the fewest number of obstacles. Behaviors:

	 - Manually start autopilot
	 - Manually select mode
	 - Remote control
	 - Avoid cliffs
	 - Obstacle sweep
	 - Avoid obstacles
	 - Detect Impact
	 - Autopilot (cruise)
	 - Headlights
	 - Sirens

2. Follow Light: robot follows the direction of the highest light intensity. This mode has been tuned to work best with flashlights as diffused ambient room light will generally not cause the robot to move. Behaviors:

	 - Manually start autopilot
	 - Manually select mode
	 - Remote control
	 - Detect Impact
	 - Light control
	 - Headlights
	 - Sirens

3. Follow Walls: the robot starts by turning left until it finds an obstacle (presumably a wall) on it's left. Using its sensors, the robot attempts to follow the perimeter of the room, making left turns to locate walls to its side, and making right turns to avoid walls ahead. Behaviors:
	 - Manually start autopilot
	 - Manually select mode
	 - Remote control
	 - Detect Impact
	 - Follow walls
	 - Headlights
	 - Sirens

##Hardware##
The robot implements the following hardware:

 - Particle photon: all robot controls and logic
 - Adafruit Pro Trinket 3v: used for additional analog inputs. Communicates basic sensor values to the photon over i2c.
 - Junior runt rover chassis
 - Sparkfun TB6612FNG H-bridge motor controller
 - Piezo speaker
 - 2 Momentary push buttons
 - SparkFun Triple Axis Accelerometer Breakout - MMA8452Q
 - HC-SR04 ultrasonic rangefinder: used for detecting objects to the left
 - MaxBotix LV-MaxSonar EZ: used for detecting front facing objects (more reliable than the HC-SR04).
 - 3 photoresistors
 - HiTec HS-322HD servo: used when sweeping the front facing sonar
 - 2 WS2812B Neopixel breakouts: used for the robot's headlights
 - Battery conainer for 4AA batteries with on/off switch
