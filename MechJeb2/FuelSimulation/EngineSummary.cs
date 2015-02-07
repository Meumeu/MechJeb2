using System;
using System.Collections.Generic;
using System.Linq;

namespace MuMech
{
	public class EngineSummary
	{
		bool throttleLocked;
		bool depleted = false;
		float minThrust;
		float maxThrust;
		float thrustPercentage;
		FloatCurve atmosphereCurve;
		List<Propellant> propellants;
		Dictionary<Propellant, List<FuelContainer>> resources;
		Part part;

		public EngineSummary (ModuleEngines e)
		{
			throttleLocked = e.throttleLocked;
			minThrust = e.minThrust;
			maxThrust = e.maxThrust;
			thrustPercentage = e.thrustPercentage;
			atmosphereCurve = e.atmosphereCurve;
			propellants = e.propellants;
			part = e.part;
		}

		public EngineSummary (ModuleEnginesFX e)
		{
			throttleLocked = e.throttleLocked;
			minThrust = e.minThrust;
			maxThrust = e.maxThrust;
			thrustPercentage = e.thrustPercentage;
			atmosphereCurve = e.atmosphereCurve;
			propellants = e.propellants;
			part = e.part;
		}

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
					flow[new ResourceIndex(pair.Key.id, part.part)] = rate;
				}
			}
			return flow;
		}

		public void updateTanks (Dictionary<Part,FuelContainer> availableNodes)
		{
			// For each relevant propellant, get the list of tanks the engine will drain resources
			resources = propellants.FindAll (
				prop => PartResourceLibrary.Instance.GetDefinition (prop.id).density > 0 && prop.name != "IntakeAir")
					.ToDictionary (
				prop => prop,
				prop => availableNodes[part].GetTanks (prop.id, availableNodes, new HashSet<Part> ()));

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
	}
}
