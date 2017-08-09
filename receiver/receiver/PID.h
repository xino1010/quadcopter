class PID {
	private:
		double I = 0;
		float kP, kI, kD;
		double lowerLimit = 0;
		double upperLimit = 0;
		double desiredPoint = 0;
		double currentPoint = 0;
		long _time;
		long lastDt;
		double lastError = 0;

	public:
		PID(float kP, float kI, float kD, double lowerLimit, double upperLimit);
		void setKp(float kP);
    float getKp();
		void setKi(float kI);
    float getKi();
		void setKd(float kD);
    float getKd();
		void setDesiredPoint(double desiredPoint);
		void setCurrentPoint(double currentPoint);
		double calculate();
		void reset();

};
