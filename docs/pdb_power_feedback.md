# **Guide to Modifying PDB Power Feedback** 

This guide explains the parameters you can modify in the PDB code to change the rates of sending and calculating chassis power feedback, in addition to the physical setup and debugging methods. 

Chassis power feedback can be accessed from the [Dev C repository](https://gitlab.com/nyu-robomaster/controls_spectre/-/blob/main/docs/freertos-architecture.md?ref_type=heads). Specifically, it is the variable s\_last\_power in chassis\_controller.c, and the actual power reading can be accessed via s\_last\_power.power. 

## **PDB Code**

The Power Distribution Board (PDB) code lives in the [“Embedded\_STM32” repository](https://gitlab.com/nyu-robomaster/embedded_stm32). To modify the PDB code and flash it onto a physical PDB using **VS Code**, follow these steps:  
 

1. Clone the [“Embedded\_STM32” repository](https://gitlab.com/nyu-robomaster/embedded_stm32)  
2. Open and build the PDB folder/project in VS Code

**To increase the send rate of the chassis power feedback, lower the POWER\_PERIOD in the main.c**  
**To increase the rate of power calculation, lower the last parameter of the function INA226\_setConfig, also found in main.c**

- Options are INA226\_AVG\_64, INA226\_AVG\_128, INA226\_AVG\_256, etc, which can be found in F7\_INA226.h  
- Lowering this parameter reduces the number of data samples used to calculate the chassis power, which can make the calculated power less reliable 

## **Physical Setup**

Connect the following together:

- Battery to Center Board over XT60  
- Center Board to Chassis PDB over XT30   
- Chassis PDB to Dev C over CAN   
- Chasis PDB to computer via SEGGER J-Link   
- Dev C to computer via USB

For reference, here are how PDB (left) and J-Link (right) look like when they are connected:  
![pdb(left), j-link(right)](./assets/pdb_j-link_connection.png)

## **How to Flash Code** (for VS Code)

Open STM32CubeProgrammer

To flash code to PDB: 

- Make sure the battery is turned on, and the two green lights on PDB are both turned on  
- Select J-link/Flasher and hit “Connect;” choose path to the elf file for the PDB project and start programming  
- Make sure to press the reset button on PDB after flashing  
- If one of the green lights on PDB turns off, restart the battery

You do not have to re-flash Dev C code when you flash PDB code and vice versa

## **Debugging Using Print Statements**

Debugging for power feedback is done on the Dev C side. Under the [Dev C repository](https://gitlab.com/nyu-robomaster/controls_spectre/-/blob/main/docs/freertos-architecture.md?ref_type=heads), go to modules/logger/logger\_config.h, and set LOG\_ENABLE\_CAN to 1 and everything else to 0\. Doing so will block unnecessary information from being printed on the serial monitor during testing. 

Furthermore, under chassis\_controller.c file under the same repository, there is a function called on\_chassis\_power\_update that prints power readings evertime chassis power feedback occurs. To find how many PID loops it takes for one chassis power feedback to occur, you can put a print statement inside a function that is called every PID loop and count how many PID loops are between one power reading print to the next power reading print on the serial monitor. 

To find the time it takes for chassis power feedback to occur, multiply the number of PID loops per power feedback by 0.005 sec. Another way is to print (unsigned long)HAL\_GetTick() inside the function on\_chassis\_power\_update. 

## **Why Increase the Chassis Power Feedback Rate**

During competition, if the robot’s chassis power exceeds a certain limit (75 watts), the robot will be forced to shut down for a short amount of time. To avoid this shutdown from happening, there are function(s) inside chassis\_controller.c that calculate by what factor the current output should be reduced, so that the chassis power of the robot never exceeds the power limit. 

Those functions reference the chassis power feedback via s\_last\_power.power; if those power feedbacks are too slow, the calculated power-limiting factor will poorly reflect the robot's actual state, which could result in unexpected behavior, such as the wheels having enough time accelerating above the power limit between two consecutive power feedbacks. 

Furthermore, it was discovered empirically that directly calculating the chassis power using the motors’ current and angular velocity leads to unexpected results. Hence, the most reliable way to implement the power-limiting functions is to reference chassis power feedback via s\_last\_power.power.   
