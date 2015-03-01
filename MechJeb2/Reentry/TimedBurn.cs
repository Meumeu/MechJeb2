﻿using System;
using System.Collections.Generic;
using System.Linq;

namespace MuMech
{
	public class TimedBurn : ReentrySimulator.IForceProvider
	{
		public double startUT;
		public float throttle;

		// True if last computeForce finished fuel
		public bool depleted;

		public TimedBurn(double startUT, float throttle = 1)
		{
			this.startUT = startUT;
			this.throttle = throttle;
			this.depleted = false;
		}

		double Pressure(Vector3d pos, CelestialBody mainBody)
		{
			return FlightGlobals.getStaticPressure(pos.magnitude - mainBody.Radius, mainBody);
		}

		public dState ComputeForce(ReentrySimulatorState st, CelestialBody mainBody)
		{
			depleted = false;
			var res = new dState(Vector3d.zero);
			if (st.t < startUT)
				return res;

			// Compute thrust direction
			Vector3d svel = ReentrySimulator.SurfaceVelocity(st.pos, st.vel, mainBody);
			Vector3d dir = MechJebModuleLandingAutopilot.ThrustDirection(st.pos, svel);

			float pressure = (float) Pressure(st.pos, mainBody);
			depleted = true;
			foreach (var engine in st.vesselSummary.activeEngines)
			{
				var flow = engine.evaluateFuelFlow(pressure, throttle);
				if (flow != null)
					depleted = false;
				res += new dState(dir * engine.thrust(throttle), flow);
			}
			return res;
		}
	}
}
