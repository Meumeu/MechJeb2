using System;

namespace MuMech
{
	public class BacktrackingReentrySimulator : ReentrySimulator
	{
		private const double maxDuration = 3600;

		public BacktrackingReentrySimulator(Vessel vessel, Orbit orbit, double targetTouchDownSpeed) : base(vessel, orbit)
		{
			engineForce = new TimedBurn(vessel, double.MaxValue, 0.95f);
			forces.Add(engineForce);
			this.targetTouchDownSpeed = targetTouchDownSpeed;
			minUt = orbit.StartUT;
		}

		public readonly double targetTouchDownSpeed;
		private TimedBurn engineForce;

		private double minUt;
		private double maxUt = double.MaxValue;
		public double burnUt {
			get { return engineForce.startUT;}
			set
			{
				if (value > burnUt)
				{
					states.RemoveAll(st => st.t >= burnUt);
					engineForce.startUT = value;
				}
				else
				{
					engineForce.startUT = value;
					states.RemoveAll(st => st.t >= burnUt);
				}
			}
		}

		protected override ReentryResult postStep ()
		{
			if (engineForce.depleted)
				return new FailedReentryResult("Fuel finished");
			if (state.t > engineForce.startUT && Vector3d.Dot(state.pos, state.vel) >= 0)
			{
				var pos = referenceFrame.ToAbsolute(state.pos, state.t);
                return new LandedReentryResult(states, referenceFrame, burnUt);
			}

			if (state.t > minUt + maxDuration)
				throw new Exception("Reentry timeout, altitude=" + (state.pos.magnitude - mainBody.Radius).ToString("F2") + "m");
				//return new FailedReentryResult(new Exception("Reentry timeout, altitude=" + (state.pos.magnitude - mainBody.Radius).ToString("SI") + "m"));

			return base.postStep ();
		}

		protected override void postProcessResult (object result)
		{
			var abs = referenceFrame.ToAbsolute(state.pos, state.t);
			double alt = mainBody.TerrainAltitude(abs.latitude, abs.longitude);

			// Burn later if fuel is finished before touchdown
			if (engineForce.depleted && abs.radius > mainBody.Radius + alt)
			{
				minUt = burnUt;
				if (maxUt - minUt <= dt)
				{
					var pos = referenceFrame.ToAbsolute(state.pos, state.t);
                    this.result = new LandedReentryResult(states, referenceFrame, burnUt);
					return;
				}
				burnUt = (maxUt + minUt)/2;
				engineForce.depleted = false;
				StartSimulation();
				return;
			}

			if (abs.radius > mainBody.Radius + alt)
			{
				minUt = burnUt;
				var fireState = states.Find(st => st.t >= burnUt);
				double delay = (abs.radius - mainBody.Radius - alt) / SurfaceVelocity(fireState.pos, fireState.vel).magnitude;
				if (double.IsNaN(delay))
					delay = (maxUt - minUt) / 2;
				if (delay <= dt)
				{
					var pos = referenceFrame.ToAbsolute(state.pos, state.t);
                    this.result = new LandedReentryResult(states, referenceFrame, burnUt);
					return;
				}
				burnUt += delay;
				StartSimulation();
				return;
			}

			removeUnderground();
			maxUt = Math.Min(maxUt, burnUt);

			var svel = SurfaceVelocity(state.pos, state.vel);
			if (svel.sqrMagnitude < targetTouchDownSpeed * targetTouchDownSpeed || maxUt - minUt <= dt)
			{
				this.result = new LandedReentryResult(states, referenceFrame, burnUt);
				//var pos = referenceFrame.ToAbsolute(state.pos, state.t);
				//this.result = new LandedReentryResult(pos, state.t, svel.magnitude, referenceFrame);
				return;
			}
			burnUt = (maxUt + minUt)/2;
			StartSimulation();

		}
	}
}

