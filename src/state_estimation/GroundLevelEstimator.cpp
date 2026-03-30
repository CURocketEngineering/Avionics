#include "state_estimation/GroundLevelEstimator.h"

// Constructor
GroundLevelEstimator::GroundLevelEstimator(float alpha)
: alpha_(alpha)
{}

// Update the ground level estimate or convert ASL to AGL - Altitude ABOVE ground level
float GroundLevelEstimator::update(float currentASL_m) {
    // Before launch: accumulate samples to estimate ground level
    if (!launched_) {
        if (sampleCount_ == 0) {
            // Initialize with first sample
            estimatedGroundLevel_m_ = currentASL_m;
        } else {
            // Exponential moving average: EMA = alpha * newValue + (1 - alpha) * oldEMA
            estimatedGroundLevel_m_ = (alpha_ * currentASL_m) + ((1.0F - alpha_) * estimatedGroundLevel_m_);
        }
        sampleCount_++;
        
        // Still on ground, so AGL is 0
        return 0.0F;
    }
    
    // After launch: convert ASL to AGL
    return currentASL_m - estimatedGroundLevel_m_;
}

// Signal that launch has been detected
void GroundLevelEstimator::launchDetected() {
    launched_ = true;
    // Ground level estimate is now frozen at estimatedGroundLevel_m
}

// Get the estimated ground level
float GroundLevelEstimator::getEGL() const {
    return estimatedGroundLevel_m_;
}
