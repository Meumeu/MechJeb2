using System;

namespace MuMech
{
	public class BacktrackingReentrySimulator : ReentrySimulator
	{
		public BacktrackingReentrySimulator(Vessel vessel, Orbit orbit, double targetTouchDownSpeed) : base(vessel, orbit)
		{
			engineForce = new TimedBurn(vessel, double.MaxValue);
			forces.Add(engineForce);
			this.targetTouchDownSpeed = targetTouchDownSpeed;
			minUt = orbit.StartUT;
		}

		public readonly double targetTouchDownSpeed;
		private TimedBurn engineForce;

		private double minUt;
		public double burnUt { get { return engineForce.startUT;}}

		protected override ReentryResult postStep ()
		{
			if (state.t > engineForce.startUT && Vector3d.Dot(state.pos, state.vel) >= 0)
			{
				var pos = referenceFrame.ToAbsolute(state.pos, state.t);
				return new LandedReentryResult(pos, state.t, SurfaceVelocity(state.pos, state.vel).magnitude, referenceFrame);
			}
			return base.postStep ();
		}

		protected override void postProcessResult (object result)
		{
			var abs = referenceFrame.ToAbsolute(state.pos, state.t);
			double alt = mainBody.TerrainAltitude(abs.latitude, abs.longitude);
			if (abs.radius > mainBody.Radius + alt)
			{
				int startidx = (int)Math.Max((engineForce.startUT - states[0].t)/dt, 1);
				states.RemoveRange(startidx, states.Count - startidx);
				minUt = state.t;
				engineForce.startUT += (abs.radius - mainBody.Radius - alt) / SurfaceVelocity(state.pos, state.vel).magnitude;
				StartSimulation();
				return;
			}
			removeUnderground();
			var svel = SurfaceVelocity(state.pos, state.vel);
			double newStartDelay = (Math.Min(state.t, engineForce.startUT) - minUt) / 2 + minUt - states[0].t;
			int idx = (int) (newStartDelay / dt);
			if (svel.sqrMagnitude < targetTouchDownSpeed * targetTouchDownSpeed || idx <= 0)
			{
				var pos = referenceFrame.ToAbsolute(state.pos, state.t);
				this.result = new LandedReentryResult(pos, state.t, svel.magnitude, referenceFrame);
				return;
			}
			engineForce.startUT = states[0].t + newStartDelay;
			states.RemoveRange(idx, states.Count - idx);
			StartSimulation();

		}
	}
}

