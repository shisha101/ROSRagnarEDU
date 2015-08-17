#ifndef RAGNARSOLUTION_H
#define RAGNARSOLUTION_H
#include "libs/Module.h"
#include "BaseSolution.h"

class Config;

enum PlatformSolution {
	FIXED,
	X_PLATFORM,
	HASH
};

class RagnarSolution : public BaseSolution {
    public:
        RagnarSolution(Config*);
        void cartesian_to_actuator( float[], float[] );
        void actuator_to_cartesian( float[], float[] );
        bool set_optional(const arm_options_t& options);
        bool get_optional(arm_options_t& options);

    private:
        void init();
        bool isValid;
		float parameter[32];
		float cp[32];
		float sp[32];
		float rad2deg;
		float pi12;
		float theta_min;
		float theta_max;
		PlatformSolution platform;
};
#endif // RAGNARSOLUTION_H
