using System;
using System.Linq;

namespace MuMech
{
	public class TimedBurn : ReentrySimulator.IForceProvider
	{
		public double startUT;
		double thrust;

		public TimedBurn (Vessel vessel, double startUT)
		{
			this.startUT = startUT;
			this.thrust = 0;
			foreach(Part p in vessel.parts)
			{
				thrust += p.Modules.OfType<ModuleEngines>().Sum(
					e => e.maxThrust * e.thrustPercentage/100)
					+ p.Modules.OfType<ModuleEnginesFX>().Sum(
						e => e.maxThrust * e.thrustPercentage/100);
			}
		}

		public dState ComputeForce (ReentrySimulatorState st, CelestialBody mainBody)
		{
			if (st.t < startUT)
				return new dState(Vector3d.zero);
			// FIXME: maybe we should actually use fuel when we turn on the engines
			Vector3d svel = ReentrySimulator.SurfaceVelocity(st.pos, st.vel, mainBody).normalized;
			if (Vector3d.Dot(svel, st.pos) < 0 )
				return new dState( -svel.normalized * thrust);
			// Well, we are going up... just panic and keep going up
			return new dState(st.pos.normalized * thrust);
		}
	}
}

