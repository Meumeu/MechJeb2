using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using UnityEngine;

namespace MuMech
{
	public class FuelFlowSimulator
	{
		public List<Stats> result;

		private FuelContainer.VesselSummary vesselSummary;

		private readonly float pressure;

		public FuelFlowSimulator(Vessel vessel, float pressure) : this(new FuelContainer.VesselSummary(vessel), pressure) {}

		public FuelFlowSimulator(FuelContainer.VesselSummary vesselSummary, float pressure)
		{
			this.vesselSummary = vesselSummary;
			this.pressure = pressure;
			StartSimulation();
		}

		public void StartSimulation()
		{
			ThreadPool.QueueUserWorkItem(Run);
		}

		private static List<Stats> ComputeStaging(FuelContainer.VesselSummary vesselSummary, float pressure)
		{
			var result = new List<Stats>();
			var subStages = new List<FuelContainer.VesselSummary>{};
			var durations = new List<double>();
			while (true)
			{
				while (vesselSummary.AllowedToStage())
				{
					subStages.Add(vesselSummary);
					result.Add(new Stats(subStages, durations));
					subStages.Clear();
					durations.Clear();
					if (vesselSummary.stage == 0)
					{
						result.Reverse();
						return result;
					}
					vesselSummary = vesselSummary.Stage();
				}
				subStages.Add(vesselSummary);

				var rates = vesselSummary.activeEngines.Aggregate((Dictionary<ResourceIndex, double>)null,
					(r,e) => EngineSummary.mergeRates(r, e.evaluateFuelFlow(pressure, 1)));
				double stageTime;
				vesselSummary = vesselSummary.ApplyConsumption(rates, out stageTime);
				durations.Add(stageTime);
			}
		}

		private void Run(object unused)
		{
			try
			{
				result = ComputeStaging(vesselSummary, pressure);
			}
			catch (Exception e)
			{
				result = new List<Stats>();
				Debug.LogException(e);
			}
		}

		//A Stats struct describes the result of the simulation over a certain interval of time (e.g., one stage)
		public struct Stats
		{
			public double startMass;
			public double endMass;
			public float startThrust;
			public double maxAccel;
			public double duration;
			public double deltaV;

			public double StartTWR(double geeASL) { return startThrust / (9.81 * geeASL * startMass); }
			public double MaxTWR(double geeASL) { return maxAccel / (9.81 * geeASL); }

			public Stats(List<FuelContainer.VesselSummary> states, List<double> durations)
			{
				startMass = states[0].mass;
				startThrust = states[0].activeEngines.Sum(e => e.thrust(1));
				maxAccel = 0;
				endMass = states.Last().mass;
				deltaV = 0;
				duration = durations.Sum();
				for (int i = 0 ; i < states.Count - 1 ; i++)
				{
					//FIXME: include cosine loss
					var thrust = states[i].activeEngines.Sum(e => e.thrust(1));
					if (states[i+1].mass > 0)
						maxAccel = Math.Max(maxAccel, thrust / states[i+1].mass);
					if (durations[i] > 0 && states[i].mass > states[i+1].mass && states[i+1].mass > 0)
						deltaV += thrust * durations[i] * Math.Log(states[i].mass / states[i+1].mass) / (states[i].mass - states[i+1].mass);
				}
			}
		}
	}
}

