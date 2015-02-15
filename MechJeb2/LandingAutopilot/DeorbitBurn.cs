using System; 
using System.Linq;
using UnityEngine;

namespace MuMech
{
    namespace Landing
    {
        public class DeorbitBurn : AutopilotStep
        {
            BacktrackingReentrySimulator simulator;
            Vector3d ManeuverDeltaV;
            double ManeuverDate;
            bool placedNode;

            public DeorbitBurn(MechJebCore core) : base(core)
            {
                // If maneuvers nodes are locked, don't compute a maneuver
                if (!vessel.patchedConicsUnlocked())
                {
                    return;
                }

                //if we don't want to deorbit but we're already on a reentry trajectory, we can't wait until the ideal point 
                //in the orbit to deorbt; we already have deorbited.
                if (orbit.ApA < mainBody.RealMaxAtmosphereAltitude())
                {
                    return;
                }

                ComputeManeuverNode();
            }

            void ComputeManeuverNode()
            {
                // FIXME: doesn't work for synchronous orbit
                double synodicPeriod = 1 / Math.Abs(1 / orbit.period - 1 / mainBody.rotationPeriod);
                int nb = 100;

                placedNode = false;

                for (int i = 0; i < nb; i++)
                {
                    double deltaT = 30 + synodicPeriod * i / nb;
                    Vector3d DV;
                    bool ok = ComputeManeuver(deltaT, out DV);

                    if (ok)
                    {
                        ManeuverDate = Planetarium.GetUniversalTime() + deltaT;
                        ManeuverDeltaV = orbit.DeltaVToManeuverNodeCoordinates(ManeuverDate, DV);
                        Debug.Log("DeorbitBurn: UT=" + ManeuverDate + ", DV=" + ManeuverDeltaV);
                        return;
                    }
                }
            }

            void StartSimulation()
            {
                simulator = new BacktrackingReentrySimulator(
                    vessel,
                    OrbitExtensions.PerturbedOrbit(orbit, ManeuverDate, orbit.ManeuverNodeToDeltaVCoordinates(ManeuverDate, ManeuverDeltaV)),
                    core.landing.touchdownSpeed);

                simulator.StartSimulation();
            }

            int nbCorrections = 0;
            void AddCourseCorrection()
            {
                if (simulator == null)
                {
                    StartSimulation();
                    return;
                }
                else if (simulator.result == null)
                {
                    return;
                }
                else if (!(simulator.result is LandedReentryResult))
                {
                    StartSimulation();
                    return;
                }

                Orbit o = OrbitExtensions.PerturbedOrbit(orbit, ManeuverDate, orbit.ManeuverNodeToDeltaVCoordinates(ManeuverDate, ManeuverDeltaV));
                Vector3d correction = core.landing.ComputeCourseCorrection(true, o, ManeuverDate, simulator.result as LandedReentryResult) * 0.5;

                ManeuverDeltaV += correction;
                nbCorrections++;

                if (correction.magnitude < 0.5 || ManeuverDate < Planetarium.GetUniversalTime() + 5 || nbCorrections > 5)
                {
                    vessel.PlaceManeuverNode(orbit, orbit.ManeuverNodeToDeltaVCoordinates(ManeuverDate, ManeuverDeltaV), ManeuverDate);
                    placedNode = true;
                    core.node.ExecuteOneNode(core.landing);
                }

                StartSimulation();
            }

            bool ComputeManeuver(double deltaT, out Vector3d DV)
            {
                double t = vesselState.time + deltaT;

                //We aim for a trajectory that 
                // a) has the same vertical speed as our current trajectory
                // b) has a horizontal speed that will give it a periapsis of -10% of the body's radius
                // c) has a heading that points toward where the target will be at the end of free-fall, accounting for planetary rotation

                //Imagine we are going to deorbit now. Find the burn that would lower our periapsis to -10% of the planet's radius
                Vector3d horizontalDV = OrbitalManeuverCalculator.DeltaVToChangePeriapsis(orbit, t, 0.9 * mainBody.Radius);
                //Compute the orbit that would put us on
                Orbit forwardDeorbitTrajectory = orbit.PerturbedOrbit(t, horizontalDV);

                Vector3d pos = forwardDeorbitTrajectory.SwappedAbsolutePositionAtUT(t);
                Vector3d vel = forwardDeorbitTrajectory.SwappedOrbitalVelocityAtUT(t);
                Vector3d up = (pos - mainBody.position).normalized;

                //Find how long that orbit would take to impact the ground
                double freefallTime = forwardDeorbitTrajectory.NextTimeOfRadius(t, mainBody.Radius) - vesselState.time;
                //Find how many degrees the planet will rotate during that time
                double planetRotationDuringFreefall = 360 * freefallTime / mainBody.rotationPeriod;
                //Find the current vector from the planet center to the target landing site
                Vector3d currentTargetRadialVector = mainBody.GetRelSurfacePosition(core.target.targetLatitude, core.target.targetLongitude, 0);
                //Construct a quaternion representing the rotation of the planet found above
                Quaternion freefallPlanetRotation = Quaternion.AngleAxis((float)planetRotationDuringFreefall, mainBody.angularVelocity);
                //Use this quaternion to find what the vector from the planet center to the target will be when we hit the ground
                Vector3d freefallEndTargetRadialVector = freefallPlanetRotation * currentTargetRadialVector;
                //Then find the actual position of the target at that time
                Vector3d freefallEndTargetPosition = mainBody.position + freefallEndTargetRadialVector;
                //Find a horizontal unit vector that points toward where the target will be when we hit the ground
                Vector3d freefallEndHorizontalToTarget = Vector3d.Exclude(up, freefallEndTargetPosition - pos).normalized;
                //Find our current horizontal velocity
                Vector3d currentHorizontalVelocity = Vector3d.Exclude(up, vel);
                //Find the desired horizontal speed after the deorbit burn
                double finalHorizontalSpeed = (currentHorizontalVelocity + horizontalDV).magnitude;
                //Combine the desired speed and direction to get the desired velocity after the deorbit burn
                Vector3d finalHorizontalVelocity = finalHorizontalSpeed * freefallEndHorizontalToTarget;

                //Compute the angle between the location of the target at the end of freefall and the normal to our orbit:
                Vector3d currentRadialVector = pos - mainBody.position;
                double targetAngleToOrbitNormal = Vector3d.Angle(orbit.SwappedOrbitNormal(), freefallEndTargetRadialVector);
                targetAngleToOrbitNormal = Math.Min(targetAngleToOrbitNormal, 180 - targetAngleToOrbitNormal);

                double targetAheadAngle = Vector3d.Angle(currentRadialVector, freefallEndTargetRadialVector); //How far ahead the target is, in degrees
                double planeChangeAngle = Vector3d.Angle(currentHorizontalVelocity, freefallEndHorizontalToTarget); //The plane change required to get onto the deorbit trajectory, in degrees

                DV = finalHorizontalVelocity - currentHorizontalVelocity;

                //If the target is basically almost normal to our orbit, it doesn't matter when we deorbit; might as well do it now
                //Otherwise, wait until the target is ahead
                return (targetAngleToOrbitNormal < 10 || (targetAheadAngle < 90 && targetAheadAngle > 60 && planeChangeAngle < 90));
            }

            public override AutopilotStep Drive(FlightCtrlState s)
            {
                if (vessel.patchedConicsUnlocked())
                {
                    if (placedNode)
                    {
                        if (vessel.patchedConicSolver.maneuverNodes.Any())
                        {
                            if (core.node.burnTriggered)
                                status = "Doing high deorbit burn";
                            else
                                status = "Moving to high deorbit burn point";
                        }
                        else
                        {
                            core.landing.maxSimulationAge = MechJebModuleLandingAutopilot.defaultMaxSimulationAge;
                            return new CoastToDeceleration(core);
                        }
                    }
                    else
                    {
                        AddCourseCorrection();
                        status = "Computing deorbit burn";
                    }
                }
                else
                {
                    status = "Unable to place a maneuver node, please deorbit manually";

                    if (core.landing.prediction is LandedReentryResult)
                    {
                        return new CoastToDeceleration(core);
                    }
                }

                return this;
            }
        }
    }
}
