using System;
using System.Collections.Generic;
using System.Linq;

namespace MuMech
{
	public class EngineSummary
	{
		bool throttleLocked;
		public readonly bool depleted;
		float minThrust;
		float maxThrust;
		float thrustPercentage;
		FloatCurve atmosphereCurve;
		List<Propellant> propellants;
		Dictionary<Propellant, List<FuelContainer>> resources;
		public readonly int partId ;
		int activationStage;

		public EngineSummary (ModuleEngines e, FuelContainer.VesselSummary vessel)
		{
			throttleLocked = e.throttleLocked;
			minThrust = e.minThrust;
			maxThrust = e.maxThrust;
			thrustPercentage = e.thrustPercentage;
			propellants = e.propellants;
			partId = e.part.GetInstanceID();
			// Duplicate the isp curve for thread safety
			ConfigNode save = new ConfigNode();
			e.atmosphereCurve.Save(save);
			atmosphereCurve = new FloatCurve();
			atmosphereCurve.Load(save);
			activationStage = e.staged ? int.MaxValue : e.part.inverseStage;
			depleted = setupTanks(vessel);
		}

		public EngineSummary (ModuleEnginesFX e, FuelContainer.VesselSummary vessel)
		{
			throttleLocked = e.throttleLocked;
			minThrust = e.minThrust;
			maxThrust = e.maxThrust;
			thrustPercentage = e.thrustPercentage;
			propellants = e.propellants;
			partId = e.part.GetInstanceID();
			// Duplicate the isp curve for thread safety
			ConfigNode save = new ConfigNode();
			e.atmosphereCurve.Save(save);
			atmosphereCurve = new FloatCurve();
			atmosphereCurve.Load(save);
			activationStage = e.staged ? int.MaxValue : e.part.inverseStage;
			depleted = setupTanks(vessel);
		}

		public bool activeInStage(int stage) { return stage <= activationStage;}

		// result in tonne per second
		public Dictionary<ResourceIndex, double> evaluateFuelFlow (float pressure, float throttle)
		{
			// Check if we have all required propellants
			if (depleted)
				return null;

			double thrust = this.thrust(throttle);
			var flow = new Dictionary<ResourceIndex, double>();
			double ratioSum = resources.Sum (pair => pair.Key.ratio);
			foreach (KeyValuePair<Propellant, List<FuelContainer>> pair in resources) {
				// KSP gravity is 9.82 m/s²
				float isp = atmosphereCurve.Evaluate (pressure);
				double rate = thrust * pair.Key.ratio / (9.82 * isp * pair.Value.Count * ratioSum);
				foreach (var part in pair.Value)
				{
					flow[new ResourceIndex(pair.Key.id, part.partId)] = rate;
				}
			}
			return flow;
		}

		public EngineSummary updateTanks(FuelContainer.VesselSummary fuels)
		{
			var res = (EngineSummary)this.MemberwiseClone();
			res.updateTanks(fuels);
			return res;
		}

		private bool setupTanks(FuelContainer.VesselSummary fuels)
		{
			// For each relevant propellant, get the list of tanks the engine will drain resources
			resources = propellants.FindAll (
				prop => PartResourceLibrary.Instance.GetDefinition (prop.id).density > 0 && prop.name != "IntakeAir")
					.ToDictionary (
				prop => prop,
					prop => fuels.nodes[partId].GetTanks(prop.id, fuels.nodes, new HashSet<int> ()));

			return resources.Any(p => p.Value.Count == 0);
		}

		public float thrust (float throttle)
		{
			if (depleted)
				return 0;
			if (throttleLocked)
				throttle = 1;
			return ((maxThrust - minThrust) * throttle * thrustPercentage / 100 + minThrust);
		}

		public static Dictionary<ResourceIndex, double> mergeRates(Dictionary<ResourceIndex, double> left, Dictionary<ResourceIndex, double> right)
		{
			if (left == null)
				return right;
			if (right == null)
				return left;
			Dictionary<ResourceIndex, double> res =  new Dictionary<ResourceIndex, double>(left);
			foreach(var kv in right)
			{
				double l ;
				res.TryGetValue(kv.Key, out l);
				res[kv.Key] = l + kv.Value;
			}
			return res;
		}
	}
}
