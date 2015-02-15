using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using UnityEngine;

namespace MuMech
{
	public class ReentrySimulator
	{
		public interface IForceProvider
		{
			dState ComputeForce(ReentrySimulatorState st, CelestialBody mainBody);
		}

		public ReentrySimulator(Vessel vessel, Orbit orbit)
		{
			this.mainBody = orbit.referenceBody;
			referenceFrame = new ReferenceFrame(mainBody);
			double startUt = Math.Max(orbit.StartUT, Planetarium.GetUniversalTime());
			var pos = orbit.SwappedRelativePositionAtUT(startUt);
			var vel = orbit.SwappedOrbitalVelocityAtUT(startUt);
			states = new List<ReentrySimulatorState>();
			states.Add(new ReentrySimulatorState(vessel, pos, vel, startUt));

			// default forces
			forces.Add(new Gravity());
			if (mainBody.atmosphere)
				forces.Add(new StockDrag(vessel));

			minSimulationUt = orbit.NextPeriapsisTime(startUt);

		}

		public void StartSimulation()
		{
			ThreadPool.QueueUserWorkItem(Run);
		}

		private void Run(object unused)
		{
			try
			{
				ReentryResult result = null;
				initialize();
				while (result == null && --iterationsLeft > 0)
				{
					integrationStep();
					result = postStep();
				}
				if (iterationsLeft == 0)
				{
					this.result = new FailedReentryResult("Reentry timeout");
					return;
				}

				MechJebCore.QueueUserWorkItem(postProcessResultWrapper, result);
			}
			catch(System.Exception e)
			{
				result = new FailedReentryResult(e);
			}
		}

		protected List<IForceProvider> forces = new List<IForceProvider>();

		protected List<ReentrySimulatorState> states;
		protected double minSimulationUt;
		protected ReentrySimulatorState state { get { return states.Last(); } }

		protected double dt = 0.02;

		protected int iterationsLeft = 100000;

		protected CelestialBody mainBody;
		protected readonly ReferenceFrame referenceFrame;

		public ReentryResult result;

		private void integrationStep()
		{
			// Runge Kutta step
			dState dx1 = forces.Aggregate(new dState(Vector3d.zero), (ds, f) => ds + f.ComputeForce(state, mainBody));

			ReentrySimulatorState st1 = state.increment(dx1, dt/2);
			dState dx2 = forces.Aggregate(new dState(Vector3d.zero), (ds, f) => ds + f.ComputeForce(st1, mainBody));

			ReentrySimulatorState st2 = state.increment(dx2, dt/2);
			dState dx3 = forces.Aggregate(new dState(Vector3d.zero), (ds, f) => ds + f.ComputeForce(st2, mainBody));

			ReentrySimulatorState st3 = state.increment(dx3, dt);
			dState dx4 = forces.Aggregate(new dState(Vector3d.zero), (ds, f) => ds + f.ComputeForce(st3, mainBody));

			states.Add(state.increment(dx1, dt/6)
				.increment(dx2, dt/3)
				.increment(dx3, dt/3)
				.increment(dx4, dt/6));
		}

		protected virtual ReentryResult postStep()
		{
			// Check if we have landed
			var pos = referenceFrame.ToAbsolute(state.pos, state.t);
			// Compute until altitude is under sea level, postprocessing will only keep valid values
			if (pos.radius < mainBody.Radius)
			{
				return new LandedReentryResult(pos, state.t, SurfaceVelocity(state.pos, state.vel).magnitude, referenceFrame);
			}
			if (state.t < minSimulationUt)
				return null;
			// Check if we are leaving the atmosphere
			if (pos.radius > mainBody.Radius + mainBody.RealMaxAtmosphereAltitude() && Vector3d.Dot(state.pos, state.vel) > 0)
			{
				var result = new AerobrakedReentryResult(pos, referenceFrame.ToAbsolute(state.vel, state.t), referenceFrame, state.t);
				if (result.orbit.PeA > mainBody.RealMaxAtmosphereAltitude())
					return new NoReentryResult();
				else
				{
					return result;
				}
			}
			return null;
		}

		protected virtual void initialize(){}

		// Must be called in main thread
		protected void removeUnderground()
		{
			while (states.Count > 1)
			{
				var abs = referenceFrame.ToAbsolute(state.pos, state.t);
				double alt = Math.Max(mainBody.TerrainAltitude(abs.latitude, abs.longitude), 0);
				if (abs.radius < mainBody.Radius + alt)
				{
					states.RemoveAt(states.Count - 1);
				}
				else
				{
					return;
				}
			}
		}

		private void postProcessResultWrapper(object result)
		{
			try
			{
				postProcessResult(result);
			}
			catch (System.Exception e)
			{
				Debug.LogException(e);
				this.result = new FailedReentryResult(e);
			}
		}

		protected virtual void postProcessResult(object result)
		{
			if (result is LandedReentryResult)
			{
				removeUnderground();
				var abs = referenceFrame.ToAbsolute(state.pos, state.t);
				LandedReentryResult landed = new LandedReentryResult(abs, state.t, SurfaceVelocity(state.pos, state.vel).magnitude, referenceFrame);
				landed.fixLandingRadius();
				this.result = landed;
			}
			else
			{
				this.result = (ReentryResult)result;
			}
		}

		public static Vector3d SurfaceVelocity(Vector3d pos, Vector3d vel, CelestialBody mainBody)
		{
			return vel - Vector3d.Cross(mainBody.angularVelocity, pos);
		}

		public Vector3d SurfaceVelocity(Vector3d pos, Vector3d vel)
		{
			return SurfaceVelocity(pos, vel, mainBody);
		}
	}
}

