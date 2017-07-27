class PID {
	private:
		double P = 0;
		double I = 0;
		double D = 0;
		double kP, kI, kD;
		double lowerLimit = 0;
		double upperLimit = 0;
		double desiredPoint = 0;
		double currentPoint = 0;
		long lastDt = millis();
		double lastError = 0;

	public:
		PID(double kP, double kI, double kD, double lowerLimit, double upperLimit);
		void setKp(double kP);
		void setKi(double kI);
		void setKd(double kD);
		void setDesiredPoint(double desiredPoint);
		void setCurrentPoint(double currentPoint);
		double calculate();
		void reset();

};
