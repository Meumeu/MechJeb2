using System;
using System.Collections.Generic;
using System.Linq;

namespace MuMech
{
	public class ResourceIndex
	{
		public readonly int resourceId;
		public readonly int partId;
		public ResourceIndex(int resourceId, int partId)
		{
			this.resourceId = resourceId;
			this.partId = partId;
		}
		public override bool Equals(object other)
		{
			var o = other as ResourceIndex;
			if (o == null)
				return false;
			return resourceId == o.resourceId && partId == o.partId;
		}
		public override int GetHashCode ()
		{
			return resourceId ^ partId;
		}
	}

	public class ReentrySimulatorState
	{
		// mass of propellants of each part
		public FuelContainer.VesselSummary vesselSummary;
		public int currentStage;

		public double t;

		public double mass { get { return vesselSummary.mass;}}

		public Vector3d pos;
		public Vector3d vel;

		public ReentrySimulatorState(Vessel vessel, Vector3d pos, Vector3d vel, double startUT)
		{
			this.pos = pos;
			this.vel = vel;
			this.t = startUT;
			this.vesselSummary = new FuelContainer.VesselSummary(vessel);
		}

		private ReentrySimulatorState() {}

		public ReentrySimulatorState increment(dState delta, double dt)
		{
			ReentrySimulatorState res = (ReentrySimulatorState)this.MemberwiseClone();
			res.pos += vel * dt;
			res.vel += delta.force * dt / mass;
			res.t += dt;
			if (delta.propellant != null)
			{
				res.vesselSummary = res.vesselSummary.ApplyConsumption(delta.propellant, dt);
			}
			return res;
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

