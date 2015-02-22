﻿using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace MuMech
{
	public class BacktrackingReentrySimulator : ReentrySimulator
	{
		public BacktrackingReentrySimulator(Vessel vessel, Orbit orbit, double targetTouchDownSpeed) : base(vessel, orbit)
		{
			engineForce = new TimedBurn(double.MaxValue, 0.95f);
			forces.Add(engineForce);
			this.targetTouchDownSpeed = targetTouchDownSpeed;
			minUt = state.t;
		}

		private List<ReentrySimulatorState> noBurnStates;

		public readonly double targetTouchDownSpeed;
		private TimedBurn engineForce;

		private double minUt;
		private double maxUt = double.MaxValue;
		public double burnUt {
			get { return engineForce.startUT;}
			private set
			{
				#if DEBUG
				Debug.Log(string.Format("New backtracking time: {0}",value));
				#endif
				engineForce.startUT = value;
				int startIdx = findIdxForUt(value - 5);
				int endIdx = findIdxForUt(value, startIdx);
				states = noBurnStates.GetRange(startIdx, Math.Max(endIdx - startIdx, 1));
			}
		}

		private int findIdxForUt(double ut, int beforeIdx = 0)
		{
			int afterIdx = noBurnStates.Count - 1;
			while (afterIdx - beforeIdx > 1)
			{
				int middle = (afterIdx + beforeIdx)/2;
				if (noBurnStates[middle].t >= ut)
					afterIdx = middle;
				else
					beforeIdx = middle;
			}
			return beforeIdx;
		}

		protected override void initialize()
		{
		}

		protected override ReentryResult postStep ()
		{
			if (engineForce.depleted)
				return new FailedReentryResult("Fuel finished");
			if (state.t > engineForce.startUT && Vector3d.Dot(state.pos, state.vel) >= 0)
			{
				states.RemoveAt(states.Count - 1);
				// The result is ignored, as we make a new one in postProcessResult
				return new FailedReentryResult("");
			}

			return base.postStep ();
		}

		protected override void postProcessResult (object result)
		{
			// First simulation result are stored so we can start the next simulation without recalculating freefall
			if (burnUt == double.MaxValue)
			{
				noBurnStates = states;
				// In case we are going up, do not burn before we have reached "apoapsis"
				minUt = Math.Max(states.FirstOrDefault(st => Vector3d.Dot(st.pos, st.vel) <0).t, minUt);
			}

			var abs = referenceFrame.ToAbsolute(state.pos, state.t);
			double alt = Math.Max(mainBody.TerrainAltitude(abs.latitude, abs.longitude), 0);

			// Burn later if fuel is finished before touchdown
			if (engineForce.depleted && abs.radius > mainBody.Radius + alt)
			{
				minUt = burnUt;
				if (maxUt - minUt <= dt)
				{
					LandedReentryResult landed = new LandedReentryResult(states, referenceFrame);
					landed.fixLandingRadius();
					this.result = landed;
					return;
				}
				burnUt = (maxUt + minUt)/2;
				engineForce.depleted = false;
				StartSimulation();
				return;
			}

			// Burn later if we started going up before touching the ground
			if (abs.radius > mainBody.Radius + alt)
			{
				minUt = burnUt;
				var fireState = states.Find(st => st.t >= burnUt);
				double delay = (abs.radius - mainBody.Radius - alt) / SurfaceVelocity(fireState.pos, fireState.vel).magnitude;
				if (double.IsNaN(delay) || maxUt - burnUt - delay < dt)
					delay = (maxUt - minUt) / 2;
				if (delay <= dt)
				{
					LandedReentryResult landed = new LandedReentryResult(states, referenceFrame);
					landed.fixLandingRadius();
					this.result = landed;
					return;
				}
				burnUt = Math.Min(burnUt + delay, maxUt);
				StartSimulation();
				return;
			}

			removeUnderground();
			maxUt = Math.Min(state.t, burnUt);

			var svel = SurfaceVelocity(state.pos, state.vel);
			// Termination condition: we touch the ground at most at requested speed
			// If we have less than one time step to select burn UT, return the best value we have
			if (svel.sqrMagnitude < targetTouchDownSpeed * targetTouchDownSpeed || maxUt - minUt <= dt)
			{
				LandedReentryResult landed = new LandedReentryResult(states, referenceFrame);
				landed.fixLandingRadius();
				this.result = landed;
				return;
			}
			// Force engine activation, for engineForce to work in initial state
			engineForce.startUT = 0;
			double thrust = engineForce.ComputeForce(states[0], mainBody).force.magnitude;
			double gravity = new Gravity().ComputeForce(state, mainBody).force.magnitude;
			double time = svel.magnitude * state.mass / (thrust - gravity);
			if (time > dt && maxUt - time > minUt + dt)
				burnUt = Math.Max(maxUt - time, minUt);
			else
				burnUt = (minUt + maxUt) / 2;
			StartSimulation();
		}
	}
}

