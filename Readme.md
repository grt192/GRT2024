# Contoller IDs
|  Controller Name | ID |
|:----------------:|:--:|
| Cyborg V.1 | 0 |
| Logitech Attack 3| 1 |
| Controller (Xbox One For | 2 |
| eStop Robotics HID | 3 |
# Motor IDs
|  Motor  | ID |
|:-------:|:--:|
| FL_Steer | 1 |
| FR_Drive | 2 |
| FR_Steer | 3 |
| BL_Drive | 4 |
| BL_Steer | 5 |
| BR_Drive | 6 |
| BR_Steer | 7 |
| FL_Drive | 8 |
| Climb Right Motor| 9 |
| Elevator Extention Motor | 10 |
| Elevator Follower Motor | 11 |
| Shooter Pivot Motor | 12 |
| Shooter Top Motor | 13 |
| Shooter Bottom Motor | 14 |
| Intake Pivot Motor | 16 |
| Intake Front Motor | 17 |
| Intake Back Motor | 18 |
| Integration Motor | 19 |
| Climb Left Motor | 20 |

# Network Tables
## Motors
| Entry Name | Type | Description |
|:----------:|:----:|:-----------:|
| 10Current | Double | |
| 10Voltage | Double | |
| 11Current | Double | |
| 11Voltage | Double | |
## Elevator
| Entry Name | Type | Description |
|:----------:|:----:|:-----------:|
| LimitSwitch | Boolean | True for triggered |
| ExtensionPercent | Double | Percent extended |
## FMS
| Entry Name | Type | Description |
|:----------:|:----:|:-----------:|
| AllianceColor | String | Blue or Red |
| StationNumber | Int | 1, 2, or 3. -1 as no connection. |
| MatchNumber | Int | |
| MatchType | String | |
| TimeLeft | Double | In seconds |
| IsAutnonmous | Boolean | |
| IsEStopped | Boolean | |
| IsEnabled | Boolean | |
| IsDSAttached | Boolean | |