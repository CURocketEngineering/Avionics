#include "flightstatus.h"
#include "SensorDataHandler.h"

//Error checking the desired frequency and the data's timing
void FlightStatus::frequencyCheck(SensorData someData){
    float dataPointsDif = theData[1].timestamp_ms - theData[0].timestamp_ms;
    float desiredFreq = someData.getinterval; //Need to define where the desired interval is

    //if frequencies do not match, throws warning
    if(1/dataPointsDif != desiredFreq){
        Serial.println('Frequency between given values does not match desired frequency');
    } 
}

//Set Flight Status value 
FlightStatus::FlightStatus(int sensorHz): altitudeDeque(128, 0), accelDeque(128,0) {
    flightStage = ARMED;
}

//Check Launch -> checks change in average from incoming values in specified frequency
bool FlightStatus::checkLaunch() {
    bool launch = false;
    float add = 0;
    float ave;
    float aveNew;

    ave = aveNew;

    for(int i = 0; i = size(theData); i++){
        add += theData[i].data;
    }
    aveNew = add/size(theData);

    //check if difference in new data points average of acceleration is greater than 11
    // 11 is based on previous data 
    if(ave - aveNew > 11){
        launch = true;
    }

return launch;
}


//Check coast with acceleration data
bool FlightStatus::checkCoast() {
    // If acceleration suddenly drops, then the engines have cut off
    // If average of last 2 seconds suddenly drops --> coast
    bool coast = false;
    float add = 0;
    float ave;
    float aveNew;

    ave = aveNew;

    for(int i = 0; i = size(theData); i++){
        add += theData[i].data;
    }
    aveNew = add/size(theData);

    if(ave > aveNew){
        coast = true;
    }

return coast;
    
}


//Check if rocket reached apogee using altitude
bool FlightStatus::checkApogee() {
    //if altitude starts decreasing -> apogee
    bool apogee = false;
    

        if(theDataAlt[0].data > theDataAlt[size(theData)-1].data){
        apogee = true;
        }

return apogee;
    
}



//Check if rocket is descending
bool FlightStatus::checkDescent() {
    // If average of last 2 seconds suddenly drops --> coast
    // also if rocket is not on ground and prior statement is true --> coast
    bool descent = false;
    float add = 0;
    float ave;
    float aveNew;

    ave = aveNew;

    for(int i = 0; i = size(theData); i++){
        add += theData[i].data;
    }
    aveNew = add/size(theData);

    if(ave < aveNew && checkLaunch){
        descent = true;
    }

return descent;
}

//Final check -> check if rocket reached the ground
bool FlightStatus::checkGround() {
    bool apogee = false;

    if(theDataAlt[0].data < 12){
        apogee = true;
    }

return apogee;
}


//Telemetry brings in new measurements
void FlightStatus::newTelemetry(SensorData* acceleration, SensorData* altitude) {
    //Creating altitude and acceleration deques
    //ascent -> coast -> apogee -> descent -> on ground must happen in order
    //do not make global
    std::vector<DataPoint> theData = acceleration -> getInOrderData; //alcz is acceleration
    std::vector<DataPoint> theDataAlt = altitude -> getInOrderData; //alt is altitude

    if(checkLaunch() && flightStage == ARMED) {
        flightStage = ASCENT;
    }
    if(checkCoast() && flightStage == ASCENT) {
        flightStage = COAST;
    }
    if(checkApogee() && flightStage == COAST)
    {
        flightStage = APOGEE;
    } 
    //adding checkdescent for time between apogee and desending
    if(checkDescent() && flightStage == APOGEE) {
        flightStage = DESCENT;
    }
    if(checkGround() && flightStage == DESCENT) {
        flightStage = ONGROUND;
    }
}

//returns what stage the rocket is in at time is ran
Stage FlightStatus::getStage() {
    return flightStage;
}