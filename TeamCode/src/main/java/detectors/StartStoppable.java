package detectors;

public abstract class StartStoppable {
	volatile boolean activated = false;

	Thread loop = new Thread(new Runnable()
	{
		@Override
		public void run()
		{
			while (activated)
			{
				StartStoppable.this.loop();
			}
		}
	});

	public final void start() {
		begin();
		activated = true;
		loop.start();
	}

	public final void stop() {
		end();
		//yes stop() is deprecated but there is no big penalty for premature death
		//and suspend is useless as well because each cycle is independent
		activated = false;
		loop.interrupt();
	}

	public abstract void loop();
	public abstract void begin();
	public abstract void end();

}
