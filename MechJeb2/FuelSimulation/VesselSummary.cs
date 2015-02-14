using System;
using System.Collections.Generic;
using System.Linq;

namespace MuMech
{
	public partial class FuelContainer
	{
		public class VesselSummary
		{
			public Dictionary<int, FuelContainer> nodes;
			private List<EngineSummary> engines;

			public double fuelMass { get { return nodes.Sum(kv => kv.Value.fuelMass);}}

			public double dryMass { get { return baseMass[stage];}}

			public double mass { get ; private set;}
			private void computeMass() { mass = dryMass + fuelMass;}

			public int stage { get ; private set;}

			// dry mass of each stage
			// index n is the full vessel
			// index n contains the dry mass of the previous stages as well
			private readonly double[] baseMass;

			public IEnumerable<EngineSummary> activeEngines { get { return engines.Where(e => e.activeInStage(stage));}}

			public VesselSummary(Vessel vessel)
			{
				stage = Staging.lastStage;
				Dictionary<Part, FuelContainer> nodes = vessel.parts.ToDictionary(p => p, p => new FuelContainer(p));
				this.nodes = new Dictionary<int, FuelContainer>();
				foreach (CompoundPart p in vessel.parts.OfType<CompoundPart>())
				{
					if (p.Modules.OfType<CompoundParts.CModuleFuelLine>().Count() > 0
						&& nodes.ContainsKey(p.target))
						nodes[p.target].linkedParts.Add(p.parent.GetInstanceID());
				}
				// Keep useful parts
				foreach (var p in nodes)
				{
					// Engines are useful
					if (p.Key.Modules.OfType<ModuleEngines>().Count() > 0
						|| p.Key.Modules.OfType<ModuleEnginesFX>().Count() > 0)
						this.nodes[p.Key.GetInstanceID()] = p.Value;
					// Fuel is useful
					if (p.Value.fuelMass > 0)
						this.nodes[p.Key.GetInstanceID()] = p.Value;
					// Things attached to other things could be useful
					if (p.Value.stacked.Count > 0 || p.Value.radialParent != null)
						this.nodes[p.Key.GetInstanceID()] = p.Value;
				}
				// Setup stage number in each FuelContainer
				nodes[vessel.rootPart].setupStage(nodes, vessel.rootPart, -1);

				engines = new List<EngineSummary>();
				foreach(var p in nodes)
				{
					// Ignore sepratrons
					if (p.Key.inverseStage == p.Value.decoupledInStage)
						continue;
					foreach(var e in p.Key.Modules.OfType<ModuleEngines>())
						engines.Add(new EngineSummary(e, this));
					foreach(var e in p.Key.Modules.OfType<ModuleEnginesFX>())
						engines.Add(new EngineSummary(e, this));
				};

				// Fill dry mass info
				baseMass = new double[stage+1];
				foreach (var p in nodes)
				{
					// Compute total mass - fuelMass to count deactivated resources (not in fuelMass)
					baseMass[p.Value.decoupledInStage + 1] += p.Key.TotalMass() - p.Value.fuelMass;
				}
				for (int i = stage - 1; i > 0 ; i--)
					baseMass[i-1] += baseMass[i];
				computeMass();
			}

			public VesselSummary ApplyConsumption(Dictionary<ResourceIndex, double> rates, double time)
			{
				if (rates == null || rates.Count == 0)
				{
					return this;
				}
				var res = (VesselSummary) this.MemberwiseClone();
				res.nodes = new Dictionary<int, FuelContainer>(res.nodes);
				bool stateChanged = false;
				foreach (var rate in rates)
				{
					var node = (FuelContainer)res.nodes[rate.Key.partId].MemberwiseClone();
					res.nodes[rate.Key.partId] = node;
					node.resourceMass = new Dictionary<int, double>(node.resourceMass);
					double newMass = node.resourceMass[rate.Key.resourceId] - rate.Value * time;
					node.resourceMass[rate.Key.resourceId] = Math.Max(0, newMass);
					if (newMass <= 0)
						stateChanged = true;
				}
				computeMass();
				if (stateChanged)
					res.engines = engines.Select(e => e.updateTanks(this)).ToList();
				return res;
			}

			public VesselSummary ApplyConsumption(Dictionary<ResourceIndex, double> rates, out double time)
			{
				time = rates.Min(rate => nodes[rate.Key.partId].resourceMass[rate.Key.resourceId] / rate.Value);
				return this.ApplyConsumption(rates, time);
			}

			public VesselSummary Stage()
			{
				if (stage == 0)
					return this;
				stage--;
				var res = (VesselSummary) this.MemberwiseClone();
				// Remove decoupled FuelContainer objects
				res.nodes = nodes.Where(kv => kv.Value.decoupledInStage < stage).ToDictionary(kv => kv.Key, kv => kv.Value);
				// Remove decoupled EngineSummary objects and update the tanks they take resources from
				res.engines = engines.Where(e => res.nodes.ContainsKey(e.partId)).Select(e => e.updateTanks(this)).ToList();
				res.computeMass();
				return res;
			}

			public bool AllowedToStage()
			{
				float pressure = 0;
				if (stage == 0)
					return false;

				var engines = activeEngines;
				// Allowed to stage if no engine is active, or none has fuel
				if (engines.Count() == 0 || engines.All(e => e.depleted))
					return true;

				// Not allowed to decouple an active engine
				if (engines.Any(e => nodes[e.partId].decoupledInStage == stage -1))
					return false;

				// Not allowed to decouple a tank containing a resource being used
				var rates = engines.Aggregate((Dictionary<ResourceIndex, double>)null,
					(r,e) => EngineSummary.mergeRates(r, e.evaluateFuelFlow(pressure, 1)));
				var fuels = rates.Keys.Select(idx => idx.resourceId).Distinct();
				foreach (var fc in nodes.Values)
				{
					if (fc.decoupledInStage != stage - 1)
						continue;
					if (fc.resourceMass.Any(kv => kv.Value > 0.05 && fuels.Contains(kv.Key)))
						return false;
				}

				// Do not stage if the only effect is to activate more parts
				if (nodes.Values.All(fc => fc.decoupledInStage != stage -1))
					return false;

				return true;
			}
		}
	}
}

