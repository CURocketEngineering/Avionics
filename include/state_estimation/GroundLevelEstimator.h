#ifndef AGL_DETECTOR_H
#define AGL_DETECTOR_H
#include <cstdint>
/*
TWO RULES - 2 input functions, 1 output:

input
Update function: current ASL in meters as a DataPoint type
Launch Dectcted: Call this when launch has happened (MUST BE SURE)

output
GetEGL (estimated ground level), returns a float that represents how many meters above sealevel the rocket was before launch
*/

class GroundLevelEstimator{
    public:
        /**
         * @brief Constructs a GroundLevelEstimator.
         */
        GroundLevelEstimator();

        /**
         * @brief Updates the ground level estimate or converts ASL to AGL.
         * 
         * Before launch: Records altitude samples to estimate ground level.
         * After launch: Converts the provided ASL altitude to AGL.             * 
         * @param currentASL_m Current altitude above sea level in meters.
         * @return Current altitude above ground level in meters.
         */
        float update(float currentASL_m);

        /**
         * @brief Signals that launch has been detected.
         * 
         * Stops recording ground level measurements and freezes the EGL.
         * Should be called once when launch is confirmed.
         */
        void launchDetected();

        /**
         * @brief Gets the estimated ground level.
         * 
         * @return Altitude above sea level at the launch site in meters.
         */
        float getEGL() const;

    private:

        bool launched = false; //Turned true if launch is detected
        float estimatedGroundLevel_m = 0.0F; //EGL in meters
        uint32_t sampleCount = 0; //Number of samples used for ground level estimate
        float alpha; //Determines how much weight the most recent number added has on the current EGL

};

#endif