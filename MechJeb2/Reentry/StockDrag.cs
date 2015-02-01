using System;
using System.Linq;

namespace MuMech
{
	public class StockDrag : ReentrySimulator.IForceProvider
	{
		private double dragCoeff;

		public StockDrag(Vessel vessel)
		{
			dragCoeff = vessel.parts.Sum(p => p.IsPhysicallySignificant() ? p.TotalMass() * p.maximum_drag : 0);
		}

		double Pressure(Vector3d pos, CelestialBody mainBody)
		{
			return FlightGlobals.getStaticPressure(pos.magnitude - mainBody.Radius, mainBody);
		}

		double AirDensity(Vector3d pos, CelestialBody mainBody)
		{
			return FlightGlobals.getAtmDensity(Pressure(pos, mainBody));
		}

		public dState ComputeForce(ReentrySimulatorState st, CelestialBody mainBody)
		{
			Vector3d airVel = ReentrySimulator.SurfaceVelocity(st.pos, st.vel, mainBody);

			// FIXME: this is not totally accurate, in stock ksp the coefficient changes when fuel is consumed
			double realDragCoefficient = dragCoeff;

			return new dState(-0.5 * FlightGlobals.DragMultiplier * realDragCoefficient * AirDensity(st.pos, mainBody) * airVel.sqrMagnitude * airVel.normalized);
		}
	}
}

