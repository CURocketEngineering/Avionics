# Data Folder

## Data Preparation 

### (Required) Download pre-processed data 
1. You can download pre-processed data from [CURE/Engineering Divison/Flight Data/post-processed-test-data](https://clemson.sharepoint.com/:f:/t/ClemsonUniversityRocketEngineering/Ep--KpIhKqlCuFsy4GkQsskBGVB4WuLDC-54oEgmoQIoRg?e=s4G1eD)
2. Place those CSVs in this folder. 

### (Optional) Processing Flight Data 

1. Download CSV flight data from the Teams folder here: [CURE/Engineering Divison/Flight Data](https://clemson.sharepoint.com/:f:/r/teams/ClemsonUniversityRocketEngineering/Shared%20Documents/CURE/Engineering%20Division/Flight%20Data?csf=1&web=1&e=TvkbCZ)
2. Add CSV files to this `data` folder.  
3. Run the `format_converter.py` on the data. 
  -It renames the columns to our standard format 
  -It trims the data to only 30 seconds +- the peak altitude time  
4. The csv data provider tool in `test/CSVMockData.h` can then digest the processed data for use in the unit tests 

## Format 

Right now its expected the CSVs will have the following columns in this order:
`time,accelx,accely,accelz,gyrox,gyroy,gyroz,magx,magy,magz,altitude,pressure,temp`

### Example CSV
```
time,accelx,accely,accelz,gyrox,gyroy,gyroz,magx,magy,magz,altitude,pressure,temp
3674269.348155,9.84407,0.82313,0.0,-0.00244346,-0.00244346,0.00641408,-55.5539,-15.1564,17.2464,2.32248,1021.35,30.2456
3674350.158699,9.84407,0.842273,-0.100498,-0.00702495,-0.0175624,0.00366519,-55.6416,-15.1272,17.0418,2.66052,1021.37,30.2545
3674430.969242,9.84407,0.89013,-0.330209,-0.00962113,-0.0138972,0.00397062,-55.8755,-15.2002,17.9772,2.82953,1021.38,30.2545
```

### Units
- `time` is in miliseconds
- `accelx`, `accely`, `accelz` are in m/s^2
- `gyrox`, `gyroy`, `gyroz` are in rad/s
- `magx`, `magy`, `magz` are in uT
- `altitude` is in meters
- `pressure` is in hPa
- `temp` is in degrees Celsius
