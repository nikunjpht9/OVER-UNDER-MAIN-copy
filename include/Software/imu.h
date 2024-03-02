class IMUSensor {
    public: 
        static void calibrate(); 
        static double cartesian(); 
        static void updateHeading();
        static double adjustedHeading(); 
        static double currentHeading; 
        static double globalHeading; 
        static void reset();
}; 