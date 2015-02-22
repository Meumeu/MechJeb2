using System;
using System.Collections.Generic;
using System.Linq;

namespace MuMech
{
	public struct ReentrySimulatorState
	{
		// mass of propellants of each part
		public readonly FuelContainer.VesselSummary vesselSummary;

		public readonly double t;

		public double mass { get { return vesselSummary.mass;}}

		public readonly Vector3d pos;
		public readonly Vector3d vel;

		public ReentrySimulatorState(Vector3d pos, Vector3d vel, double t, FuelContainer.VesselSummary vesselSummary)
		{
			this.pos = pos;
			this.vel = vel;
			this.t = t;
			this.vesselSummary = vesselSummary;
		}

		public ReentrySimulatorState increment(dState delta, double dt) { return this.increment(delta, dt, this); }

		// The velSt parameter is required to use the correct velocity as it is not stored in the dState
		public ReentrySimulatorState increment(dState delta, double dt, ReentrySimulatorState velSt)
		{
			return new ReentrySimulatorState(
				pos:this.pos + velSt.vel * dt,
				vel:this.vel + delta.force * dt / mass,
				t:this.t + dt,
				vesselSummary:this.vesselSummary.ApplyConsumption(delta.propellant, dt)
			);
		}
	}

	public struct dState
	{
		public Vector3d force;
		// mass flow rate in tonne/s for each propellant
		public Dictionary<ResourceIndex, double> propellant;

		public dState(Vector3d acc, Dictionary<ResourceIndex, double> propellant = null)
		{
			this.force = acc;
			this.propellant = propellant;
		}

		public static dState operator+(dState left, dState right)
		{
			return new dState(left.force + right.force, EngineSummary.mergeRates(left.propellant, right.propellant));
		}
	}
}

