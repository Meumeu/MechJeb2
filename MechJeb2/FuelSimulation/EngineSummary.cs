using System;
using System.Collections.Generic;
using System.Linq;

namespace MuMech
{
	public class EngineSummary
	{
		bool throttleLocked;
		public bool depleted { get ; private set;}
		float minThrust;
		float maxThrust;
		float thrustPercentage;
		float vacuumIsp;
		float seaLevelIsp;
		List<Propellant> propellants;
		Dictionary<Propellant, List<int>> resources;
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
			vacuumIsp = e.atmosphereCurve.Evaluate(0);
			seaLevelIsp = e.atmosphereCurve.Evaluate(1);
			activationStage = e.staged ? int.MaxValue : e.part.inverseStage;
			setupTanks(vessel);
		}

		public EngineSummary (ModuleEnginesFX e, FuelContainer.VesselSummary vessel)
		{
			throttleLocked = e.throttleLocked;
			minThrust = e.minThrust;
			maxThrust = e.maxThrust;
			thrustPercentage = e.thrustPercentage;
			propellants = e.propellants;
			partId = e.part.GetInstanceID();
			vacuumIsp = e.atmosphereCurve.Evaluate(0);
			seaLevelIsp = e.atmosphereCurve.Evaluate(1);
			activationStage = e.staged ? int.MaxValue : e.part.inverseStage;
			setupTanks(vessel);
		}

		public float isp(float atmospheres)
		{
			return vacuumIsp + (seaLevelIsp - vacuumIsp) * Math.Min(atmospheres, 1);
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
			foreach (var pair in resources) {
				// KSP gravity is 9.82 m/s²
				double rate = thrust * pair.Key.ratio / (9.82 * isp(pressure) * pair.Value.Count * ratioSum);
				foreach (var partId in pair.Value)
				{
					flow[new ResourceIndex(pair.Key.id, partId)] = rate;
				}
			}
			return flow;
		}

		public EngineSummary updateTanks(FuelContainer.VesselSummary fuels)
		{
			var res = (EngineSummary)this.MemberwiseClone();
			res.setupTanks(fuels);
			return res;
		}

		private void setupTanks(FuelContainer.VesselSummary fuels)
		{
			// For each relevant propellant, get the list of tanks the engine will drain resources
			resources = propellants.FindAll (
				prop => PartResourceLibrary.Instance.GetDefinition (prop.id).density > 0 && prop.name != "IntakeAir")
					.ToDictionary (
				prop => prop,
					prop => fuels.GetTanks(prop.id, partId));

			depleted = resources.Any(p => p.Value.Count == 0);
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
