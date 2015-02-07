using System;
using System.Collections.Generic;
using System.Linq;

namespace MuMech
{
	public class TimedBurn : ReentrySimulator.IForceProvider
	{
		public double startUT;
		public float throttle;
		private List<EngineSummary> engines = new List<EngineSummary>();

		// True if last computeForce finished fuel
		public bool depleted;

		public TimedBurn (Vessel vessel, double startUT, float throttle = 1)
		{
			this.startUT = startUT;
			this.throttle = throttle;
			this.depleted = false;
			//FIXME: have engines per stage
			foreach(Part p in vessel.parts)
			{
				foreach(var e in p.Modules.OfType<ModuleEngines>())
					engines.Add(new EngineSummary(e));
				foreach(var e in p.Modules.OfType<ModuleEnginesFX>())
					engines.Add(new EngineSummary(e));
			}
		}

		double Pressure(Vector3d pos, CelestialBody mainBody)
		{
			return FlightGlobals.getStaticPressure(pos.magnitude - mainBody.Radius, mainBody);
		}

		public void forceEngineUpdate(ReentrySimulatorState st)
		{
			foreach (var e in engines)
			{
				e.updateTanks(st.resources, true);
			}
		}

		public dState ComputeForce(ReentrySimulatorState st, CelestialBody mainBody)
		{
			depleted = false;
			var res = new dState(Vector3d.zero);
			if (st.t < startUT)
				return res;

			// Compute thrust direction
			Vector3d svel = ReentrySimulator.SurfaceVelocity(st.pos, st.vel, mainBody);
			Vector3d dir = -svel.normalized;
			if (Vector3d.Dot(svel, st.pos) >= 0 || svel.sqrMagnitude < 1)
			// Well, we are going up... just panic and keep going up
				dir = st.pos.normalized;

			float pressure = (float) Pressure(st.pos, mainBody);
			depleted = true;
			foreach (var engine in engines)
			{
				engine.updateTanks(st.resources);
				var flow = engine.evaluateFuelFlow(pressure, throttle);
				if (flow != null)
					depleted = false;
				res += new dState(dir * engine.thrust(throttle), flow);
			}
			return res;
		}
	}
}
