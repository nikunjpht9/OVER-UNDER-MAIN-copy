#include "main.h"
#include "roboconfig.h"

#include "Programs/skills.h"
#include "Programs/autonomous.h"

#include "Software/lcd.h" 
#include "Software/doctor.h"
#include "Software/imu.h"
#include "Software/odometry.h"
#include "Software/pid.h"
#include "Software/mathfunctions.h"

#include "Robot/drivetrain.h"
#include "Robot/intake.h"
#include "Robot/puncher.h"
#include "Robot/pneumatics.h"




double Math::PI = 3.141592653589793238462643383279; 

double Math::degreesToInches(double degrees, double wheelDiameter) {
  return degrees * wheelDiameter * Math::PI / 360.0; 
}


double Math::inchesToDegrees(double inches, double wheelDiameter) {
  return 360.0 * inches / (Math::PI * wheelDiameter); 
}



double Math::sine(double degrees) {
  return sin(degrees * Math::PI / 180.0); 
}

double Math::cosine(double degrees) {
  return cos(degrees * Math::PI / 180.0); 
}

double Math::tangent(double degrees) {
  return tan(degrees * Math::PI / 180.0); 
}

double Math::arcsine(double val) {
  return asin(val) * 180.0 / Math::PI; 
}

double Math::arccosine(double val) {
  return acos(val) * 180.0 / Math::PI; 
}

double Math::arctangent(double val) {
  return atan(val) * 180.0 / Math::PI; 
}

int Math::sign(double val) {
  if(val > 0) {
    return 1; 
  }
  else if (val < 0) {
    return -1; 
  }
  return 0; 
}

double Math::curve(double val, double alpha) {
  return Math::sign(val) * pow(fabs(val), alpha) * pow(100, 1 - alpha); 
}

double Math::headingToCartesian(double heading_) {
  if(heading_ > 90) {
    return 450 - heading_; 
  }
  else {
    return 90 - heading_; 
  }
}

double Math::cartesianToHeading(double cartesian) {
  if(cartesian <= 90) {
    return 90 - cartesian; 
  }
  else {
    return 450 - cartesian; 
  }
}

double Math::convertAngle(double angle_) {
  if(angle_ < 0) {
    while(true) {
      angle_ += 360; 
      if(angle_ >= 0 && angle_ < 360) {
        return angle_; 
      }
    }
  }
  else if (angle_ >= 360) {
    while(true) {
      angle_ -= 360; 
      if(angle_ >= 0 && angle_ < 360) {
        return angle_; 
      }      
    }
  }
  else {
    return angle_;
  }
}

double Math::abs(double val) {
  if(val < 0) {
    return -1 * val; 
  }
  else if (val > 0) {
    return val; 
  }
  return 0; 
}

int Math::abs(int val) {
  if(val < 0) {
    return -1 * val; 
  }
  else if (val > 0) {
    return val; 
  }
  return 0; 
}

double Math::hypotenuse(double x, double y) {
  return pow(x * x + y * y, 0.5); 
}


double Math::wheelDistance(double wheelDiameter, double rotationAngle) {
  return Math::PI * wheelDiameter * rotationAngle / 360.0; 
} 