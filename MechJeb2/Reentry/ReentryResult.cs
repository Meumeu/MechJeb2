using System;
using System.Collections.Generic;

namespace MuMech
{
	public class ReentryResult {}

	public struct TrajectoryPoint
	{
		public AbsoluteVector pos;
		public AbsoluteVector svel;
	}

	public class LandedReentryResult : ReentryResult
	{
		public LandedReentryResult(AbsoluteVector pos, double endUt, double speed, ReferenceFrame frame)
		{
			this.landingSite = pos;
			this.touchdownTime = endUt;
			this.touchdownSpeed = speed;
			this.frame = frame;
			this.burnUt = double.NaN;
		}

		public LandedReentryResult(List<ReentrySimulatorState> states, ReferenceFrame frame, double burnUt)
		{
			var lastState = states[states.Count - 1];
			this.landingSite = frame.ToAbsolute(lastState.pos, lastState.t);
			this.touchdownTime = lastState.t;
			this.touchdownSpeed = ReentrySimulator.SurfaceVelocity(lastState.pos, lastState.vel, frame.referenceBody).magnitude;
			this.frame = frame;
			this.burnUt = burnUt;

			trajectory = new List<TrajectoryPoint>();
			foreach (ReentrySimulatorState i in states)
			{
				if (i.t >= burnUt)
				{
					TrajectoryPoint item;
					item.pos = frame.ToAbsolute(i.pos, i.t);
					item.svel = frame.ToAbsolute(ReentrySimulator.SurfaceVelocity(i.pos, i.vel, frame.referenceBody), i.t);
					trajectory.Add(item);
				}
			}
		}

		public AbsoluteVector landingSite;
		public double touchdownTime;
		public double burnUt;
		public double touchdownSpeed;
		public Vector3d WorldPosition { get {
				return frame.WorldPositionAtCurrentTime(landingSite);
			}
		}

		public readonly ReferenceFrame frame;
		public List<TrajectoryPoint> trajectory;

		public override string ToString()
		{
			return string.Format("LandedReentryResult: {0}", Coordinates.ToStringDMS(landingSite.latitude, landingSite.longitude));
		}
	}

	public class AerobrakedReentryResult : ReentryResult
	{
		public AerobrakedReentryResult(AbsoluteVector pos, AbsoluteVector vel, ReferenceFrame f, double endUt)
		{
			this.frame = f;
			this.pos = pos;
			this.vel = vel;
			this.endUt = endUt;
		}
		private ReferenceFrame frame;
		private AbsoluteVector pos;
		private AbsoluteVector vel;
		public Orbit orbit { get
			{
				return MuUtils.OrbitFromStateVectors(frame.WorldPositionAtCurrentTime(pos), frame.WorldVelocityAtCurrentTime(vel), pos.body, endUt);
			}
		}
		// Time when we leave the atmosphere
		public double endUt;
	}

	public class FailedReentryResult : ReentryResult
	{
		public FailedReentryResult(System.Exception e)
		{
			this.message = e.Message;// + "\n" + e.StackTrace;
		}
		public string message;

		public override string ToString()
		{
			return string.Format("FailedReentryResult: {0}", message);
		}
	}

	public class NoReentryResult : ReentryResult
	{
	}

	//Why do AbsoluteVector and ReferenceFrame exist? What problem are they trying to solve? Here is the problem.
	//
	//The reentry simulation runs in a separate thread from the rest of the game. In principle, the reentry simulation
	//could take quite a while to complete. Meanwhile, some time has elapsed in the game. One annoying that that happens
	//as time progresses is that the origin of the world coordinate system shifts (due to the floating origin feature).
	//Furthermore, the axes of the world coordinate system rotate when you are near the surface of a rotating celestial body.
	//
	//So, one thing we do in the reentry simulation is be careful not to refer to external objects that may change
	//with time. Once the constructor finishes, the ReentrySimulation stores no reference to any CelestialBody, or Vessel,
	//or Orbit. It just stores the numbers that it needs to crunch. Then it crunches them, and comes out with a deterministic answer
	//that will never be affected by what happened in the game while it was crunching.
	//
	//However, this is not enough. What does the answer that the simulation produces mean? Suppose the reentry
	//simulation chugs through its calculations and determines that the vessel is going to land at the position
	//(400, 500, 600). That's fine, but where is that, exactly? The origin of the world coordinate system may have shifted,
	//and its axes may have rotated, since the simulation began. So (400, 500, 600) now refers to a different place
	//than it did when the simulation started.
	//
	//To deal with this, any vectors (that is, positions and velocities) that the reentry simulation produces as output need to
	//be provided in some unambiguous format, so that we can interpret these positions and velocities correctly at a later
	//time, regardless of what sort of origin shifts and axis rotations have occurred.
	//
	//Now, it doesn't particularly matter what unambiguous format we use, as long as it is in fact unambiguous. We choose to 
	//represent positions unambiguously via a latitude, a longitude, a radius, and a time. If we record these four data points
	//for an event, we can unambiguously reconstruct the position of the event at a later time. We just have to account for the
	//fact that the rotation of the planet means that the same position will have a different longitude.

	//An AbsoluteVector stores the information needed to unambiguously reconstruct a position or velocity at a later time.
	public struct AbsoluteVector
	{
		public double latitude;
		public double longitude;
		public double radius;
		public double UT;
		public CelestialBody body;
		public double ASL { get { return radius - body.Radius;}}
	}

	//A ReferenceFrame is a scheme for converting Vector3d positions and velocities into AbsoluteVectors, and vice versa
	public class ReferenceFrame
	{
		private readonly double epoch;
		private readonly Vector3d lat0lon0AtStart;
		private readonly Vector3d lat0lon90AtStart;
		private readonly Vector3d lat90AtStart;
		public readonly CelestialBody referenceBody;

		public ReferenceFrame(CelestialBody referenceBody)
		{
			lat0lon0AtStart = referenceBody.GetSurfaceNVector(0, 0);
			lat0lon90AtStart = referenceBody.GetSurfaceNVector(0, 90);
			lat90AtStart = referenceBody.GetSurfaceNVector(90, 0);
			epoch = Planetarium.GetUniversalTime();
			this.referenceBody = referenceBody;
		}

		//Vector3d must be either a position RELATIVE to referenceBody, or a velocity
		public AbsoluteVector ToAbsolute(Vector3d vector3d, double UT)
		{
			AbsoluteVector absolute = new AbsoluteVector();

			absolute.latitude = 180 / Math.PI * Math.Asin(Vector3d.Dot(vector3d.normalized, lat90AtStart));

			double longitude = 180 / Math.PI * Math.Atan2(Vector3d.Dot(vector3d.normalized, lat0lon90AtStart), Vector3d.Dot(vector3d.normalized, lat0lon0AtStart));
			longitude -= 360 * (UT - epoch) / referenceBody.rotationPeriod;
			absolute.longitude = MuUtils.ClampDegrees180(longitude);

			absolute.radius = vector3d.magnitude;
			absolute.body = referenceBody;

			absolute.UT = UT;

			return absolute;
		}

		//Interprets a given AbsoluteVector as a position, and returns the corresponding Vector3d position
		//in world coordinates.
		public Vector3d WorldPositionAtCurrentTime(AbsoluteVector absolute)
		{
			return referenceBody.position + WorldVelocityAtCurrentTime(absolute);
		}

		//Interprets a given AbsoluteVector as a velocity, and returns the corresponding Vector3d velocity
		//in world coordinates.
		public Vector3d WorldVelocityAtCurrentTime(AbsoluteVector absolute)
		{
			double now = Planetarium.GetUniversalTime();
			double unrotatedLongitude = MuUtils.ClampDegrees360(absolute.longitude - 360 * (now - absolute.UT) / referenceBody.rotationPeriod);
			return absolute.radius * referenceBody.GetSurfaceNVector(absolute.latitude, unrotatedLongitude);
		}
	}
}

