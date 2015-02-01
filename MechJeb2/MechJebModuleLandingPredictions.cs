using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using UnityEngine;

namespace MuMech
{
    public class MechJebModuleLandingPredictions : ComputerModule
    {
        //inputs:
        [Persistent(pass = (int)Pass.Global)]
        public bool makeAerobrakeNodes = false;

        public bool deployChutes = false;        
        public int limitChutesStage = 0;

        //simulation inputs:
        public double decelEndAltitudeASL = 0; // The altitude at which we need to have killed velocity - NOTE that this is not the same as the height of the predicted landing site.imulate this descent speed policy
        public double parachuteSemiDeployMultiplier = 3; // this will get updated by the autopilot.

        //internal data:
      
		protected bool simulationRunning { get { return simulator != null && simulator.result == null;}}
        protected long millisecondsBetweenSimulations;

		protected ReentrySimulator simulator = null;

		public ReentryResult result = null;

        public ManeuverNode aerobrakeNode = null;

        protected const int interationsPerSecond = 5; // the number of times that we want to try to run the simulation each second.
        protected double dt = 0.2; // the suggested dt for each timestep in the simulations. This will be adjusted depending on how long the simulations take to run.

        public override void OnStart(PartModule.StartState state)
        {
            if (state != PartModule.StartState.None && state != PartModule.StartState.Editor)
            {
                RenderingManager.AddToPostDrawQueue(1, DoMapView);
            }
        }

        public override void OnModuleDisabled()
        {
        }

        public override void OnFixedUpdate()
        {
            try
            {
                if (vessel.isActiveVessel)
                {
					if (simulator != null && simulator.result != null)
						result = simulator.result;
                    // We should be running simulations periodically. If one is not running right now,
                    // check if enough time has passed since the last one to start a new one:
                    if (!simulationRunning)
                    {
						Orbit o = vessel.orbit;
						if (vessel.patchedConicSolver.maneuverNodes.Count > 0)
						{
							var node = vessel.patchedConicSolver.maneuverNodes.Last(n => n != aerobrakeNode);
							if (node != null && node.nextPatch != null)
								o = node.nextPatch;
						}
						simulator = new ReentrySimulator(vessel, o);
						simulator.StartSimulation();
                    }
                }
            }
            catch (Exception ex)
            {
                Debug.LogException(ex);
            }
        }

        public override void OnUpdate()
        {
            if (vessel.isActiveVessel)
            {
                MaintainAerobrakeNode();
            }
        }

        protected Orbit GetReenteringPatch()
        {
            Orbit patch = orbit;

            int i = 0;

            do
            {
                i++;
                double reentryRadius = patch.referenceBody.Radius + patch.referenceBody.RealMaxAtmosphereAltitude();
                Orbit nextPatch = vessel.GetNextPatch(patch, aerobrakeNode);
                if (patch.PeR < reentryRadius)
                {
                    if (patch.Radius(patch.StartUT) < reentryRadius) return patch;

                    double reentryTime = patch.NextTimeOfRadius(patch.StartUT, reentryRadius);
                    if (patch.StartUT < reentryTime && (nextPatch == null || reentryTime < nextPatch.StartUT))
                    {
                        return patch;
                    }
                }

                patch = nextPatch;
            }
            while (patch != null);

            return null;
        }

        protected void MaintainAerobrakeNode()
        {
            if (makeAerobrakeNodes)
            {
                //Remove node after finishing aerobraking:
                if (aerobrakeNode != null && vessel.patchedConicSolver.maneuverNodes.Contains(aerobrakeNode))
                {
                    if (aerobrakeNode.UT < vesselState.time && vesselState.altitudeASL > mainBody.RealMaxAtmosphereAltitude())
                    {
                        vessel.patchedConicSolver.RemoveManeuverNode(aerobrakeNode);
                        aerobrakeNode = null;
                    }
                }

                //Update or create node if necessary:
				AerobrakedReentryResult r = result as AerobrakedReentryResult;
                if (r != null)
                {
                    //Compute the node dV:
                    Orbit preAerobrakeOrbit = GetReenteringPatch();

                    //Put the node at periapsis, unless we're past periapsis. In that case put the node at the current time.
                    double UT;
                    if (preAerobrakeOrbit == orbit &&
                        vesselState.altitudeASL < mainBody.RealMaxAtmosphereAltitude() && vesselState.speedVertical > 0)
                    {
                        UT = vesselState.time;
                    }
                    else
                    {
                        UT = preAerobrakeOrbit.NextPeriapsisTime(preAerobrakeOrbit.StartUT);
                    }

					Orbit postAerobrakeOrbit = r.orbit;

                    Vector3d dV = OrbitalManeuverCalculator.DeltaVToChangeApoapsis(preAerobrakeOrbit, UT, postAerobrakeOrbit.ApR);

                    if (aerobrakeNode != null && vessel.patchedConicSolver.maneuverNodes.Contains(aerobrakeNode))
                    {
                        //update the existing node
                        Vector3d nodeDV = preAerobrakeOrbit.DeltaVToManeuverNodeCoordinates(UT, dV);
                        aerobrakeNode.OnGizmoUpdated(nodeDV, UT);
                    }
                    else
                    {
                        //place a new node
                        aerobrakeNode = vessel.PlaceManeuverNode(preAerobrakeOrbit, dV, UT);
                    }
                }
                else
                {
                    //no aerobraking, remove the node:
                    if (aerobrakeNode != null && vessel.patchedConicSolver.maneuverNodes.Contains(aerobrakeNode))
                    {
                        vessel.patchedConicSolver.RemoveManeuverNode(aerobrakeNode);
                    }
                }
            }
            else
            {
                //Remove aerobrake node when it is turned off:
                if (aerobrakeNode != null && vessel.patchedConicSolver.maneuverNodes.Contains(aerobrakeNode))
                {
                    vessel.patchedConicSolver.RemoveManeuverNode(aerobrakeNode);
                }
            }
        }

        void DoMapView()
        {
            if (MapView.MapIsEnabled && vessel.isActiveVessel && this.enabled)
            {
				LandedReentryResult landed = result as LandedReentryResult;
				if (landed != null)
                {
					GLUtils.DrawMapViewGroundMarker(landed.landingSite.body, landed.landingSite.latitude, landed.landingSite.longitude, Color.blue, 60);
                }
            }
        }

        public MechJebModuleLandingPredictions(MechJebCore core) : base(core) { }
    }
}
