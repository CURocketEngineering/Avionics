#include "state_estimation/GroundLevelEstimator.h"

// Constructor
GroundLevelEstimator::GroundLevelEstimator()
: launched(false), estimatedGroundLevel_m(0.0F), sampleCount(0)
{}

// Update the ground level estimate or convert ASL to AGL - Altitude ABOVE ground level
float GroundLevelEstimator::update(float currentASL_m) {
    
    // Before launch: accumulate samples to estimate ground level
    if (!launched) {
        // Running average of ground level samples
        estimatedGroundLevel_m = ((estimatedGroundLevel_m * sampleCount) + currentASL_m) / (sampleCount + 1);
        sampleCount++;
        
        // Still on ground, so AGL is 0
        return 0.0F;
    }
    
    // After launch: convert ASL to AGL
    return currentASL_m - estimatedGroundLevel_m;
}

// Signal that launch has been detected
void GroundLevelEstimator::launchDeteched() {
    launched = true;
    // Ground level estimate is now frozen at estimatedGroundLevel_m
}

// Get the estimated ground level
float GroundLevelEstimator::getEGL() const {
    return estimatedGroundLevel_m;
}