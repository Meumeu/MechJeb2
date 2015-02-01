using System;

namespace MuMech
{
	public class Gravity : ReentrySimulator.IForceProvider
	{
		public Gravity ()
		{
		}

		public dState ComputeForce (ReentrySimulatorState st, CelestialBody mainBody)
		{
			return new dState(- st.pos.normalized * st.mass * mainBody.gravParameter / st.pos.sqrMagnitude);
		}
	}
}

