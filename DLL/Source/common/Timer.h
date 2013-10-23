const float TICKS2SEC = 1.0e-3f;

class CHiResTimer
{
	public:
	CHiResTimer() 
	{
		m_prevTime = GetTimeInMiliseconds();
	}

	~CHiResTimer() {}

	
	float GetElapsedSeconds()
	{
		float timeStep;
		float dt;
		unsigned time;
		
		
		time = GetTimeInMiliseconds();
		dt = float (time - m_prevTime);
		m_prevTime = time;

		timeStep = dt * TICKS2SEC;
		if (timeStep > 0.1f) {
			timeStep = 0.1f;
		}
		if (timeStep < 0.005f) {
			timeStep = 0.005f;
		}
		
		return timeStep;
	} 

	unsigned GetTimeInMiliseconds() const
	{
		return timeGetTime();
	}
	
	private:
	unsigned m_prevTime;
	unsigned m_totalFrames;
};
