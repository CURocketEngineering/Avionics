# Flight Test History

## Flight Number Key
(rocket type)-(flight compute type)-(month)-(day)-(year)-(flight number)

### Rocket Types
- **L1** - Level 1 Certification
- **L2** - Level 2 Certification
- **L3** - Level 3 Certification
- **AA** - Active Aero
- **CV** - Competition Vehicle

### Flight Computer Types
- **MA** - MARTHA (Miniaturized Avionics for Rapid Testing Handling and Assessment)
- **JM** - JEM (Just an Expanded MARTHA)
- **AA** - Active Aero flight computer (responsible for brake deployment)

## Flight Test History (Most recent first)

- **AA-AA-04-13-2025-0 [Active Aero FC Apr 13th 2025 Active Aero Launch](https://github.com/CURocketEngineering/Active-Aero/releases/tag/1.0):**
  Flew on the same flight as **AA-MA-04-13-2025-0** and was connected to the air brakes. It successfully detected launch, engine burnout, and apogee (based on post-fight data analysis). The brakes were accurately deployed at 1 second after engine burnout and retracted 3 seconds before apogee (according to the software). There is no way of knowing if the fins actually deployed and then retracted or never deployed in the first place. Utilized the new SD card data saver which worked well. 


- **AA-MA-04-13-2025-0 [MARTHA 1.3 Apr 13th 2025 Active Aero Launch](https://github.com/CURocketEngineering/MARTHA-1.3/releases/tag/1.1.0):**
  MARTHA 1.3 Board 1 (B1) was flown the Active Aero vehicle to around 3800 feet. It detected launch and apogee successfully. It also performed live apogee prediction which had poor performance (tended to underestimate throughout the entire flight). All data was recovered successfully. Only the drogue chute deployed but MARTHA survived the hard landing. 

- **L1-MA-04-12-2025-0 [MARTHA 1.3 Apr 12th 2025 L1 Cert Launch](https://github.com/CURocketEngineering/MARTHA-1.3/releases/tag/1.0.2-no-alt):**  
  Flight **L1-MA-04-12-2025-0**.  
  MARTHA 1.3 Board 3 (B3) flew on an L1 cert. It successfully collected all data metrics except for altimeter data due to B3 having a malfunctioning altimeter. It ran a custom version of MARTHA software with the altimeter collection calls commented out. The parachute deployed partially, but B3 survived the hard landing. Launch detection was successful; apogee detection failed due to lack of altitude data.

- **L1-MA-03-08-2025-0 [MARTHA 1.3 Mar 8th 2025 L1 Cert Launches](https://github.com/CURocketEngineering/MARTHA-1.3/releases/tag/1.0.1):**  
  Flight **L1-MA-03-08-2025-0**.  
  The MARTHA 1.3 system successfully completed its first altitude data collection and apogee detection. Two separate MARTHA 1.3 boards, B1 and B2, were flown on different rockets, each accurately detecting both launch and apogee. MARTHA was used solely for data collection; parachute deployment was managed by a Stratologger. Notably, B1’s rocket did not experience a deployment event, whereas B2’s did, demonstrating that MARTHA’s apogee detection is independent of Stratologger deployment.

- **AA-MA-02-08-2025-0 [MARTHA 1.3 Feb 8th 2025 Test Vehicle Launch Attempt](https://github.com/CURocketEngineering/MARTHA-1.3/releases/tag/1.0.0):**  
  Flight **AA-MA-02-08-2025-0**.  
  MARTHA was mounted to the Active Aero test vehicle, but the vehicle never launched due to incomplete assembly. MARTHA collected data for 14.18 hours before the 9V battery expired. Data was successfully dumped, and the last hour before shutdown was recorded. A consistent 100Hz loop speed was achieved, and despite large accelerations during rough handling, the LaunchDetector showed no false positives.

- **L1-MA-11-09-2024-0 [MARTHA 1.1 Nov 9th 2024 L1 Cert Launch](https://github.com/CURocketEngineering/MARTHA-1.1/releases/tag/v1.1.0):**  
  Flight **L1-MA-11-09-2024-0**.  
  Launch detection and SD card data logging performed as expected. No altitude data was recorded, likely due to issues with sensor drivers or hardware damage.

- **CV-MA-06-22-2024-0 [MARTHA 1.1 Spaceport 2024](https://github.com/CURocketEngineering/MARTHA-1.1/releases/tag/v1.0.0):**  
  Flight **CV-MA-06-22-2024-0**.  
  Although data was captured via the serial logger, a battery failure on the pad resulted in no launch data being recorded. The log from being the pad for 6 hours was only 30 minutes long.

