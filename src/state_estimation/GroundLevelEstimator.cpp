#include "state_estimation/GroundLevelEstimator.h"

// Constructor
GroundLevelEstimator::GroundLevelEstimator()
: launched(false), estimatedGroundLevel_m(0.0F), sampleCount(0), alpha(0.1F)
{}

// Update the ground level estimate or convert ASL to AGL - Altitude ABOVE ground level
float GroundLevelEstimator::update(float currentASL_m) {
    // Before launch: accumulate samples to estimate ground level
    if (!launched) {
        if (sampleCount == 0) {
            // Initialize with first sample
            estimatedGroundLevel_m = currentASL_m;
        } else {
            // Exponential moving average: EMA = alpha * newValue + (1 - alpha) * oldEMA
            estimatedGroundLevel_m = (alpha * currentASL_m) + ((1.0F - alpha) * estimatedGroundLevel_m);
        }
        sampleCount++;
        
        // Still on ground, so AGL is 0
        return 0.0F;
    }
    
    // After launch: convert ASL to AGL
    return currentASL_m - estimatedGroundLevel_m;
}

// Signal that launch has been detected
void GroundLevelEstimator::launchDetected() {
    launched = true;
    // Ground level estimate is now frozen at estimatedGroundLevel_m
}

// Get the estimated ground level
float GroundLevelEstimator::getEGL() const {
    return estimatedGroundLevel_m;
}