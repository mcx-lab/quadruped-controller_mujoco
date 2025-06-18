# StateEstimate msg definition for Quadruped Robot

 
float32[4] contactEstimate        # Feet Contact Estimate

Cartesian comPosition             # CoM Position 
Cartesian comVelocity             # CoM Velocity

Cartesian[4] feetPosition2Body    # Feet Position w.r.t to CoM 
Cartesian[4] feetVelocity2Body    # Feet Velocity w.r.t to CoM 

float32[4] orientation            # Body Orientation - Quaternion
float32[3] rpy                    # Body rpy 
float32[3] omegaBody              # Body angular vel
