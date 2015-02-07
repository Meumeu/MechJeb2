using System;
using System.Collections.Generic;
using System.Linq;

namespace MuMech
{
	public class ResourceIndex
	{
		public readonly int resourceId;
		public readonly Part part;
		public ResourceIndex(int resourceId, Part part)
		{
			this.resourceId = resourceId;
			this.part = part;
		}
		public override bool Equals(object other)
		{
			var o = other as ResourceIndex;
			if (o == null)
				return false;
			return resourceId == o.resourceId && part == o.part;
		}
		public override int GetHashCode ()
		{
			return resourceId.GetHashCode() ^ part.GetHashCode();
		}
	}

	public class ReentrySimulatorState
	{
		// dry mass of each stage
		// index n is the full vessel
		// index n contains the dry mass of the previous stages as well
		private readonly double[] baseMass;
		// mass of propellants of each part
		public FuelContainer.FuelSummary resources;
		public int currentStage;

		public double t;

		public double mass { get ; private set;}

		public Vector3d pos;
		public Vector3d vel;

		public ReentrySimulatorState(Vessel vessel, Vector3d pos, Vector3d vel, double startUT)
		{
			this.pos = pos;
			this.vel = vel;
			this.t = startUT;
			this.resources = new FuelContainer.FuelSummary(vessel);

			Queue<KeyValuePair<Part,int>> pending = new Queue<KeyValuePair<Part,int>>();
			List<double> baseMass = new List<double>();

			pending.Enqueue(new KeyValuePair<Part, int>(vessel.rootPart, 0));

			while (pending.Count != 0)
			{
				var current = pending.Dequeue();
				var part = current.Key;
				int stage = current.Value;
				currentStage = Math.Max(currentStage, stage);
				{
					var decoupler = part.Modules.OfType<ModuleDecouple>().FirstOrDefault();
					var aDecoupler = part.Modules.OfType<ModuleAnchoredDecoupler>().FirstOrDefault();
					if (decoupler != null || aDecoupler != null)
					{
						//FIXME: if the decoupler is attached upside down, the mass and possible propellant should be added to the parent's stage
						stage = Math.Max(part.inverseStage+1, current.Value);
					}
				}

				while (baseMass.Count < stage + 1)
					baseMass.Add(0);

				if (part.IsPhysicallySignificant())
					baseMass[stage] += part.mass;

				foreach(var child in part.children)
					pending.Enqueue(new KeyValuePair<Part, int>(child, stage));
			}
			this.baseMass = baseMass.ToArray();
			for (int i = baseMass.Count - 1; i > 0 ; i--)
				this.baseMass[i-1] += this.baseMass[i];

			updateMass();
		}

		private void updateMass()
		{
			mass = baseMass[currentStage] + resources.fuelMass;
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
				res.resources = res.resources.ApplyConsumption(delta.propellant, dt);
				updateMass();
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
			Dictionary<ResourceIndex, double> propellant;
			if (left.propellant != null && right.propellant != null)
			{
				propellant = new Dictionary<ResourceIndex, double>(left.propellant);
				foreach(var kv in right.propellant)
				{
					double l ;
					propellant.TryGetValue(kv.Key, out l);
					propellant[kv.Key] = l + kv.Value;
				}
			}
			else if (left.propellant != null)
			{
				propellant = left.propellant;
			}
			else
			{
				// could be null
				propellant = right.propellant;
			}
			return new dState(left.force + right.force, propellant);
		}
	}
}

