# Flight Test History

## Flight-Number Format  
`<rocket type>-<flight computer type>-<year>-<month>-<day>-<index>`

The trailing **index** (`0`, `1`, …) distinguishes multiple flights of the **same rocket / flight-computer pair on the same day**.

### Rocket Types
| Code | Description                |
|------|----------------------------|
| **L1** | Level 1 Certification |
| **L2** | Level 2 Certification |
| **L3** | Level 3 Certification |
| **AA** | Active Aero            |
| **CV** | Competition Vehicle    |

### Flight-Computer Types
| Code | Description |
|------|-------------|
| **MA** | MARTHA (Miniaturized Avionics for Rapid Testing Handling and Assessment) |
| **JM** | JEM (Just an Expanded MARTHA) |
| **AA** | Active Aero flight computer (air-brake deployment) |

---

## Flight Test History (most recent first)

* **L2-MA-2025-11-09-0 [MARTHA 1.3 B1 Sam's L2 Live Telemetry Test](https://github.com/CURocketEngineering/MARTHA-1.3/releases/tag/1.2.0):**
  Flew on Sam's L2 test vehicle to about 1,300 feet. Live telemetry worked at 2 HZ with a live updating ground station dashboard and we could tell apogee and
  all the parachute events prior to recovery. No data was saved on the flash chip due to a hardware issue.
  The little blue spider wires on B1's flash chip likely got broken when placing the board in the 
  radio-MARTHA assembly. Used a custom mount to hold the radio and MARTHA together and only one support 
  struct sheared. Main chute did not come out, landed under drogue. 

* **CV-MA-2025-06-11-0 [MARTHA 1.3 B1 IREC 2025 Launch](https://github.com/CURocketEngineering/MARTHA-1.3/releases/tag/1.1.0):**
  Flew on the competition vehicle for IREC 2025 to about 9900 feet. It lost power at drogue deployment. It flew the same software as
  **AA-MA-2025-04-13-0**, including apogee detection, which performed poorly in the same way as before (underestimating throughout the flight).
  The main chute was deployed at apogee; drogue was never deployed.
  Launch was detected 0.19 seconds late, and apogee was detected 0.55 seconds late. Reported apogee was 9917 feet.
  Both boards were powered at 3:00 AM and then launched at around 1:30 PM, so the battery was on for
  about 10.5 hours. This was a 9V alkaline battery.

* **CV-MA-2025-06-11-1 [MARTHA 1.3 B2 IREC 2025 Launch](https://github.com/CURocketEngineering/MARTHA-1.3/releases/tag/1.0.1):**
  Flew on the same rocket as **CV-MA-2025-06-11-0** to about 9900 feet. It also lost power at drogue deployment. It flew older software, the same as
  **L1-MA-2025-03-08-0**. Launch was detected 0.19 seconds late, and apogee was detected 0.70 seconds late. Reported apogee was 9904 feet.

- **AA-AA-2025-04-13-0 [Active Aero FC Apr 13th 2025 Active Aero Launch](https://github.com/CURocketEngineering/Active-Aero/releases/tag/1.0):**  
  Flew on the same flight as **AA-MA-04-13-2025-0** and was connected to the air brakes. It successfully detected launch, engine burnout, and apogee (based on post-fight data analysis). The brakes were accurately deployed at 1 second after engine burnout and retracted 3 seconds before apogee (according to the software). There is no way of knowing if the fins actually deployed and then retracted or never deployed in the first place. Utilized the [new SD card data saver](../include/data_handling/DataSaverBigSD.h) which worked well. 

- **AA-MA-2025-04-13-0 [MARTHA 1.3 Apr 13th 2025 Active Aero Launch](https://github.com/CURocketEngineering/MARTHA-1.3/releases/tag/1.1.0):**  
  MARTHA 1.3 Board 1 (B1) was flown the Active Aero vehicle to around 3800 feet. It detected launch and apogee successfully. It also performed live apogee prediction which had poor performance (tended to underestimate throughout the entire flight). All data was recovered successfully. Only the drogue chute deployed but MARTHA survived the hard landing. 

- **L1-MA-2025-04-12-0 [MARTHA 1.3 Apr 12th 2025 L1 Cert Launch](https://github.com/CURocketEngineering/MARTHA-1.3/releases/tag/1.0.2-no-alt):**  
  MARTHA 1.3 Board 3 (B3) flew on an L1 cert. It successfully collected all data metrics except for altimeter data due to B3 having a malfunctioning altimeter. It ran a custom version of MARTHA software with the altimeter collection calls commented out. The parachute deployed partially, but B3 survived the hard landing. Launch detection was successful; apogee detection failed due to lack of altitude data.

- **L1-MA-2025-03-08-0 [MARTHA 1.3 Mar 8th 2025 L1 Cert Launches](https://github.com/CURocketEngineering/MARTHA-1.3/releases/tag/1.0.1):**  
  The MARTHA 1.3 system successfully completed its first altitude data collection and apogee detection. Two separate MARTHA 1.3 boards, B1 and B2, were flown on different rockets, each accurately detecting both launch and apogee. MARTHA was used solely for data collection; parachute deployment was managed by a Stratologger. Notably, B1’s rocket did not experience a deployment event, whereas B2’s did, demonstrating that MARTHA’s apogee detection is independent of Stratologger deployment.

- **AA-MA-2025-02-08-0 [MARTHA 1.3 Feb 8th 2025 Test Vehicle Launch Attempt](https://github.com/CURocketEngineering/MARTHA-1.3/releases/tag/1.0.0):**  
  MARTHA was mounted to the Active Aero test vehicle, but the vehicle never launched due to incomplete assembly. MARTHA collected data for 14.18 hours before the 9V battery expired. Data was successfully dumped, and the last hour before shutdown was recorded. A consistent 100Hz loop speed was achieved, and despite large accelerations during rough handling, the LaunchDetector showed no false positives.

- **L1-MA-2024-11-09-0 [MARTHA 1.1 Nov 9th 2024 L1 Cert Launch](https://github.com/CURocketEngineering/MARTHA-1.1/releases/tag/v1.1.0):**  
  Launch detection and SD card data logging performed as expected. No altitude data was recorded, likely due to issues with sensor drivers or hardware damage.

- **CV-MA-2024-06-22-0 [MARTHA 1.1 Spaceport 2024](https://github.com/CURocketEngineering/MARTHA-1.1/releases/tag/v1.0.0):**  
  Although data was captured via the serial logger, a battery failure on the pad resulted in no launch data being recorded. The log from being on the pad for 6 hours was only 30 minutes long.
