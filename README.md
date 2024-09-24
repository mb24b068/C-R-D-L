# C-R-D-L
Ethanol/N2O rocket 
Arduino
Giblingsystem

Startröhren 

Two – Space Team				  RT-01 [Rocket Engine-Guided Landing] 

  

 

Idea 

 

 

1. Plans 

  

Structure 

 

Trunk Carbon fiber tube with a diameter of about 5-10 cm and a length 		of about 1.5-2 meters  

Stabilizers PLA for the fins (stabilizers), stable enough to hold the rocket 		stabilize during flight 

Engine Combined thrust engine, which produces 1.5 – 2 kW 

  

Landing system 

 

Engine-controlled landing Similar to SpaceX, small 	Return engine 	that the rocket slows down during landing. 

Reinforcement Structure of the rocket the loads during landing 			defy. 

  

2. Control Code 

 

This code controls a [servo Motor_01] that could move the rocket's fins to change the direction of flight. 

 

   

  #include <Servo.h> // For servo motors to control the fins 

   

  servo servofins; Define a servo for fin control 

   

  void setup() { 

    servofins.attach(9); Connect the servo motor to pin 9 

  } 

   

  void loop() { 

    Change fin position 

    for (int pos = 0; pos <= 180; pos += 1) { // Move fins from 0° to 180° 

      servoFlossen.write(pos); 

      delay(15); Wait 15ms for servo positioning 

    } 

    for (int pos = 180; pos >= 0; pos -= 1) { // Move fins from 180° to 0° 

      servoFlossen.write(pos); 

      delay(15); 

    } 

  }  

 

  

3. BOM 

  

Trunk 		Carbon fiber tube (1.5-2m long, Ø5-10 cm) 

Fins 		PLA for the stabilizers 

Engine 		Solid rocket engine or combined thruster engine 

Control 		Arduino Board, Servo Motors 

Battery 		For Arduino and servo motors 

Sensors 		Gyroscope and accelerometers for stabilization 

Landing system	Return engine or special brackets 

  

4. Cost list 

  

Carbon Fiber Tube 						€ 100 

PLA-Filament Flossen					€ 20-30  

Engine 							€ 200-300 

Arduino Board 						€ 20-30  

Servo motors 						€ 10-20 / Piece 

Battery 							€ 10-20  

Sensors 							€ 30-50  

Other accessories (cables, fasteners, etc.) 	€ 20-30  

  

Total costs:						€ 400-600  

  

5. Innovation	 

  

Engine-controlled landing The return engine should be precisely controlled. 		to enable a soft landing. A combination 		of GPS and IMU (Inertial Measurement Unit) could help you to make the landing 		. 

Modular design Design the rocket so that you can easily create different modules 		. 

Energy efficiency Think about how you can optimize energy supply and use		to minimize the weight of the rocket. 

  

 
The weight of your combined thrust rocket (i.e. a rocket engine that provides both thrust and control for landing) is affected by several factors.  

Factors that affect weight 

1. Rumpfmaterial Carbon fiber: Very light and strong. The fuselage of a rocket with 		a diameter of about 5-10 cm and a length of 1.5-2 meters 		could weigh between 200-500g, depending on the wall thickness and 		Construction. 

  

2. Fins (stabilizers) PLA: These are relatively light. Fins for a rocket 		this 	Size could weigh about 50-100g. 

  

 

3. Engine 	Combined thrust drive:The weight of such an engine 		can vary, but typically a small combined 			thrust engine suitable for this rocket, about 200-500 g. This is 		heavier than a solid propellant engine, as it has additional components 		for steering and landing. 

  

4. Electronics  Arduino and sensors: Electronic control included 			Arduino, servo motors and sensors will weigh about 100-200 g. 

  

5. Power supply 50-100g, depending on the capacity and type of batteries. 

  

6. Other components mounting material, fasteners, etc.: 50-100 g. 

  

Estimated total weight	[grams] 

  

Pluck					200-500  

Fins				50-100   

Engine 				200-500  

Electrical engineering			100-200  

Batteries  				50-100  

Other components 		50-100  

  

Total				650 - 1500   

  

Additional Considerations 

  

Safety Safety in the operation of a rocket motor is crucial. All 	Securely attach components and structurally stabilize the rocket. 

Landing system The return engine for landing will reduce the weight		but also to reduce the weight of the rocket, as the rocket 		may lose part of its take-off mass during flight. [Not 		possible] 

The exact weight composition depends on the specific components and their designs. It might be helpful to perform a detailed calculation and simulation to determine the final weight more accurately. 

 
The design of a combined thruster engine for a rocket is a challenging task, as it must combine several functions: thrust for ascent, control for stability and, if necessary, a return function for landing. Here are the basic steps and considerations for building such an engine: 

1. Concept and design 

Purpose 

Start		Generates enough thrust to propel the rocket to 1 km altitude. 

Stability	 Allows you to control the rocket during flight. 

Landing	 Controls the landing without a parachute. 

Key features: 

Thrust control  The main engine should be powerful enough to get the rocket into the air. 

Gimbaling: A way to control the direction of thrust (e.g. by adjusting the thrust vector). 

Landing engine: A smaller engine or nozzle to intercept the rocket as it falls. 

2. Components 

Main Drive: 

Type: A solid rocket engine or a liquid rocket engine could be used, with a liquid propulsion system usually being more complex. 

Size: For a rocket of this size, a thrust range of 1-2 kN (kilonewtons) could be suitable. 

Thrust control system: 

Gimbaling mechanism: This allows the engine to tilt to change the direction of flight. You could use servo motors or mechanical controls. 

Fuel pump and control: Liquid drives require a fuel pump and precision controls. 

Landing engine: 

Type: A small return engine (e.g. a small solid fuel engine) or an adjustable nozzle. 

Position: Integrate it so that it can be precisely controlled during the landing process. 

3. Design and construction 

Thruster engine: 

Design: Calculate the size and thrust profile based on the weight of your rocket and the thrust you want. 

Material choice: Use heat-resistant materials for the engine, such as special alloys or composites. 

Crafting: Build or buy an engine that meets the requirements. You can modify or adapt existing model rocket engines. 

Gimbaling-System: 

Mechanism: Design a mechanism that can tilt the engine in different directions. 

Control: Integrate servo motors or an electric drive for movement. 

Landing engine: 

Design: Design a separate landing engine or nozzle that activates during the descent. 

Integration: Make sure it works safely and reliably during flight and landing. 

4. Control and integration 

Control system: 

Arduino: Program the Arduino board to control the gimbal system and the landing engine. You'll need sensors like gyroscopes and accelerometers to monitor the stability of the rocket. 

Software: Develop software that regulates the control of the gimbal and the activation of the landing engine based on the current attitude and altitude. 

Tests and adjustments: 

Prototyping: Build and test the prototype of your engine and control system. 

Fine-tuning: Make sure all components work together reliably and adjust the design according to the test results. 

5. Security Considerations 

Safety: Since this is a complex system, pay attention to safety precautions when handling and testing the engine. Consider protective measures for yourself and others. 

Building a combined thruster engine is a challenging task and requires in-depth knowledge of rocket science and engineering. It might be helpful to engage with experts or experienced rocket builders and, if possible, seek professional advice or support. 

 
 

++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 

++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 

++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 

 

 

 1. Calculation of the required thrust 

  

Necessary thrust 1.5 x of mass. (Weight of the object to be transported + "static friction" x 1.5 (safety coificcient). 

   

2. Basic rocket equation (Tsiolkovsky equation) 

 

Delta v is the relationship between the initial velocity and the logarithmic dependence on the mass at the start up to point z=1000m. The fuel [ethanol / N2O (30% / 70%) is delivered linearly through each nozzle, per chamber (reason: pressure adjustment between ethanol 5-7 bar and N2O 70-80 bar)  

 

 

To calculate the speed that the rocket needs to reach the desired altitude of 1000 m, we use the basic rocket equation. 

Rocket basic equation: 

  

  

The final mass [mf] is the mass of the rocket without fuel, while [m0] the launch mass of the rocket containing the fuel.  

 

Fuel distribution (30% ethanol and 70% N2O) 

 

## **5. Arduino Code for Valve Control and Thrust Vector Control** 

  

Now the Arduino is used to control the valves for the fuel and the oxidizer as well as to enable thrust vector control via servos. 

  

### Arduino-Code: 

  

  

# 

 

++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 

++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 

 

 

. Basic Components 

Fuel tank 

 (Ethanol 30%)	3-5 bar 

Material: Aluminum 7075 or titanium 

Oxidator-Tank (N₂O 70%)	50-70 bar 

Material: High-strength aluminum or carbon fiber reinforced plastics 

 
 

Brennkammer  
Materialwahl [Leistungsfähige Kühlung; leicht; sabil; hitzebeständig] i/ Metal 316L 
Was ist Material 316L? 

Alloy 1.4404 or 316L is an austenitic AISI 316 stainless steel that has very good corrosion resistance due to its high chromium and molybdenum content combined with a low carbon content. 

 

 

 

  

 

?? Length.. nozzle??  
 

impulsparameter (Isp) 

??????????????????????????????????????????????????????????????????????????? 

The maximum combustion temperature of ethanol (C₂H₅OH) and nitrous oxide (N₂O) depends on the mixing ratios, pressure, and environment. If we assume a mixture of 30% ethanol and 70% N₂O, we can calculate a theoretical combustion temperature. 

  

1. **Reaction equation**: 

   The theoretical combustion of ethanol with N₂O can be described by the following equation: 

  

   \[ 

   C_2H_5OH + 3N_2O \rightarrow 2CO_2 + 3H_2O + 3N_2 

C2 H5 OH+3N2 O→2CO2 +3H2 O+3N2   

   \] 

  

2. **Calculation of adiabate flame temperature**: 

   In order to calculate the maximum flame temperature (adiabatic flame temperature), the specific heats of the reaction products and the energy content of the reactants must be taken into account. For ethanol and N₂O, the combustion temperature under standard conditions (1 atm pressure) is about **2980 K** to **3100 K**. These values are based on ideal assumptions with no heat losses. 

  

3. **Influence of Mixing Ratio**: 

   With a mixing ratio of 30% ethanol and 70% N₂O, the temperature could be slightly lower due to the lower ethanol content compared to pure oxygen (such as in combustion with pure O₂), but still in the range of **2500 K to 2900 K**. 

  

### Influencing factors: 

- **Pressure**: At higher pressures, the combustion temperature can rise. 

- **Efficiency of the mixture**: Incomplete combustion would reduce the temperature. 

  

The exact temperature also depends on the conditions in the combustion chamber (e.g. pressure, heat losses). 

  

### Sources: 

- Turns, S. R. *An Introduction to Combustion: Concepts and Applications*, 3rd edition, McGraw-Hill, 2012. 

- NASA CEA (Chemical Equilibrium with Applications) 
 
 
The term "atmospheric pressure" (abbreviated "atm") refers to the pressure generated by the weight of the air in the Earth's atmosphere at sea level. This pressure under normal conditions is: 

  

- **1 atm** = 101,325 Pa (Pascal) = 1,01325 bar = 760 mmHg (Millimeter Quecksilbersäule). 

  

Atmospheric pressure varies with altitude. At sea level, it is about 1 atm, but it decreases with increasing altitude because the air density and thus the weight of the air column decreases above a point. 

  

When rocket applications speak of combustion temperatures under "standard conditions", this refers to an environment at 1 atm (sea level). However, significantly higher pressures can prevail in the combustion chamber of a rocket, which increases the combustion temperature. 

 

 

 

 

 

Engine Mixture  
Sources  

 

 

 

************************************************************************* 

************************************************************************** 

  

  

 Choosing the material for the combustion chamber of a bi-liquid rocket engine: 
 
Critical role: mechanical stresses, thermal loads due to high temperatures, chemical compatibility with the propellants, and the material's weight. 

Inconel (Nickel-Chromium Superalloys, e.g., Inconel 625 or 718): 

   

  

  

  

?? Length.. nozzle??   

  

  

impulsparameter (Isp)  

  

???????????????????????????????????????????????????????????????????????????  

  

The maximum combustion temperature of ethanol (C₂H₅OH) and nitrous oxide (N₂O) depends on the mixing ratios, pressure, and environment. If we assume a mixture of 30% ethanol and 70% N₂O, we can calculate a theoretical combustion temperature.  

  

   

  

1. **Reaction equation**:  

  

   The theoretical combustion of ethanol with N₂O can be described by the following equation:  

  

   

  

   \[  

  

   C_2H_5OH + 3N_2O \rightarrow 2CO_2 + 3H_2O + 3N_2  

  

C2 H5 OH+3N2 O→2CO2 +3H2 O+3N2    

  

   \]  

  

   

  

2. **Calculation of adiabate flame temperature**:  

  

   In order to calculate the maximum flame temperature (adiabatic flame temperature), the specific heats of the reaction products and the energy content of the reactants must be taken into account. For ethanol and N₂O, the combustion temperature under standard conditions (1 atm pressure) is about **2980 K** to **3100 K**. These values are based on ideal assumptions with no heat losses.  

  

   

  

3. **Influence of Mixing Ratio**:  

  

   With a mixing ratio of 30% ethanol and 70% N₂O, the temperature could be slightly lower due to the lower ethanol content compared to pure oxygen (such as in combustion with pure O₂), but still in the range of **2500 K to 2900 K**.  

  

   

  

### Influencing factors:  

  

- **Pressure**: At higher pressures, the combustion temperature can rise.  

  

- **Efficiency of the mixture**: Incomplete combustion would reduce the temperature.  

  

   

  

The exact temperature also depends on the conditions in the combustion chamber (e.g. pressure, heat losses).  

  

   

  

### Sources:  

  

- Turns, S. R. *An Introduction to Combustion: Concepts and Applications*, 3rd edition, McGraw-Hill, 2012.  

  

- NASA CEA (Chemical Equilibrium with Applications)  

  

  

The term "atmospheric pressure" (abbreviated "atm") refers to the pressure generated by the weight of the air in the Earth's atmosphere at sea level. This pressure under normal conditions is:  

  

   

  

- **1 atm** = 101,325 Pa (Pascal) = 1,01325 bar = 760 mmHg (Millimeter Quecksilbersäule).  

  

   

  

Atmospheric pressure varies with altitude. At sea level, it is about 1 atm, but it decreases with increasing altitude because the air density and thus the weight of the air column decreases above a point.  

  

   

  

When rocket applications speak of combustion temperatures under "standard conditions", this refers to an environment at 1 atm (sea level). However, significantly higher pressures can prevail in the combustion chamber of a rocket, which increases the combustion temperature.  

  

  

  

  

  

  

  

  

  

  

  

Engine Mixture   

Sources   

 
 
 
