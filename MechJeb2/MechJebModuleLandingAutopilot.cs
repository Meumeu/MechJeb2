using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;

namespace MuMech
{
    public class MechJebModuleLandingAutopilot : AutopilotModule
    {
        bool deployedGears;
        public bool landAtTarget;

        [Persistent(pass = (int)(Pass.Local | Pass.Type | Pass.Global))]
        public EditableDouble touchdownSpeed = 0.5;

        [Persistent(pass = (int)(Pass.Local | Pass.Type | Pass.Global))]
        public bool deployGears = true;
        [Persistent(pass = (int)(Pass.Local | Pass.Type | Pass.Global))]
        public EditableInt limitGearsStage = 0;
        [Persistent(pass = (int)(Pass.Local | Pass.Type | Pass.Global))]
        public bool deployChutes = true;
        [Persistent(pass = (int)(Pass.Local | Pass.Type | Pass.Global))]
        public EditableInt limitChutesStage = 0;
        [Persistent(pass = (int)(Pass.Local | Pass.Type | Pass.Global))]
        public bool rcsCourseCorrection = false;

        //Landing prediction data:
        BacktrackingReentrySimulator simulator;
        double simulatorCreationUt = 0;
        public ReentryResult prediction;
        public double BurnUt = double.NaN;
        public List<KeyValuePair<AbsoluteVector, AbsoluteVector>> trajectory;

        MechJebModuleLandingPredictions predictor;

        public bool LandingBurnReady
        {
            get
            {
                return !double.IsNaN(BurnUt);
            }
        }

        public bool PredictionReady //We shouldn't do any autopilot stuff until this is true
        {
            get
            {
                return prediction is LandedReentryResult;
            }
        }

        double LandingAltitude // The altitude above sea level of the terrain at the landing site
        {
            get
            {
                LandedReentryResult prediction = this.prediction as LandedReentryResult;
                if (prediction != null)
                {
                    // Although we know the landingASL as it is in the prediction, we suspect that
                    // it might sometimes be incorrect. So to check we will calculate it again here,
                    // and if the two differ log an error. It seems that this terrain ASL calls when
                    // made from the simulatiuon thread are regularly incorrect, but are OK when made
                    // from this thread. At the time of writting (KSP0.23) there seem to be several
                    // other things going wrong with he terrain system, such as visual glitches as
                    // we as the occasional exceptions being thrown when calls to the CelestialBody
                    // object are made. I suspect a bug or some sort - for now this hack improves
                    // the landing results.
                    return prediction.landingSite.body.TerrainAltitude(prediction.landingSite.latitude, prediction.landingSite.longitude);
                }
                else
                {
                    return 0;
                }
            }
        }

        public Vector3d LandingSite // The current position of the landing site
        {
            get
            {
                LandedReentryResult prediction = this.prediction as LandedReentryResult;
                return mainBody.GetWorldSurfacePosition(prediction.landingSite.latitude,
                    prediction.landingSite.longitude, LandingAltitude);
            }
        }

        Vector3d RotatedLandingSite // The position where the landing site will be when we land at it
        {
            get { return (prediction as LandedReentryResult).WorldPosition; }
        }

        public MechJebModuleLandingAutopilot(MechJebCore core) : base(core)
        {
        }

        public override void OnStart(PartModule.StartState state)
        {
            predictor = core.GetComputerModule<MechJebModuleLandingPredictions>();
        }

        //public interface:
        public void LandAtPositionTarget(object controller)
        {
            landAtTarget = true;
            users.Add(controller);

            vessel.RemoveAllManeuverNodes(); // For the benefit of the landing predictions module

            deployedGears = false;

            if (orbit.PeA < 0)
                setStep(new Landing.CourseCorrection(core));
            else if (UseLowDeorbitStrategy())
                setStep(new Landing.PlaneChange(core));
            else
                setStep(new Landing.DeorbitBurn(core));
        }

        public void LandUntargeted(object controller)
        {
            landAtTarget = false;
            users.Add(controller);

            deployedGears = false;

            setStep(new Landing.UntargetedDeorbit(core));
        }

        public override void OnModuleEnabled()
        {
            core.attitude.users.Add(this);
            core.thrust.users.Add(this);
        }

        public void StopLanding()
        {
            this.users.Clear();
        }

        public override void OnModuleDisabled()
        {
            core.attitude.attitudeDeactivate();
            simulator = null;
            BurnUt = double.NaN;
            prediction = null;
            core.thrust.ThrustOff();
            core.thrust.users.Remove(this);
            core.rcs.enabled = false;
            setStep(null);
        }

        public override void Drive(FlightCtrlState s)
        {
            if (!active)
                return;

            // If the latest prediction is a landing, aerobrake or no-reentry prediciton then keep it.
            // However if it is any other sort or result it is not much use to us, so do not bother!

            if (simulator != null && simulator.result != null)
            {
                if (simulator.result is LandedReentryResult || simulator.result is AerobrakedReentryResult || simulator.result is NoReentryResult)
                {
                    prediction = simulator.result;
                    predictor.result = simulator.result;
                    BurnUt = simulator.burnUt;
                }

                Debug.Log("Simulation result available: " + simulator.result);
                simulator = null;
            }
            if (simulator == null && Planetarium.GetUniversalTime() - simulatorCreationUt > 2)
            {
                Debug.Log("Starting reentry simulation");

                Orbit o = orbit;
                if (vessel.patchedConicSolver.maneuverNodes.Count > 0)
                {
                    var node = vessel.patchedConicSolver.maneuverNodes.Last(n => n != core.GetComputerModule<MechJebModuleLandingPredictions>().aerobrakeNode);
                    if (node != null && node.nextPatch != null)
                        o = node.nextPatch;
                }
                simulator = new BacktrackingReentrySimulator(vessel, orbit, touchdownSpeed.val);
                simulator.StartSimulation();
                simulatorCreationUt = Planetarium.GetUniversalTime();
                Debug.Log("Reentry simulation started");
            }

            // Consider lowering the langing gear
            {
                double minalt = Math.Min(vesselState.altitudeBottom, Math.Min(vesselState.altitudeASL, vesselState.altitudeTrue));
                if (deployGears && !deployedGears && (minalt < 1000))
                    DeployLandingGears();
            }

            base.Drive(s);
        }

        public override void OnFixedUpdate()
        {
            base.OnFixedUpdate();
            DeployParachutes();
        }

        // Estimate the delta-V of the correction burn that would be required to put us on
        // course for the target
        public Vector3d ComputeCourseCorrection(bool allowPrograde)
        {
            // actualLandingPosition is the predicted actual landing position
            Vector3d actualLandingPosition = RotatedLandingSite - mainBody.position;

            // orbitLandingPosition is the point where our current orbit intersects the planet
            double endRadius = mainBody.Radius + DecelerationEndAltitude() - 100;
            Vector3d orbitLandingPosition;
            if (orbit.PeR < endRadius)
                orbitLandingPosition = orbit.SwappedRelativePositionAtUT(orbit.NextTimeOfRadius(vesselState.time, endRadius));
            else
                orbitLandingPosition = orbit.SwappedRelativePositionAtUT(orbit.NextPeriapsisTime(vesselState.time));

            // convertOrbitToActual is a rotation that rotates orbitLandingPosition on actualLandingPosition
            Quaternion convertOrbitToActual = Quaternion.FromToRotation(orbitLandingPosition, actualLandingPosition);

            // Consider the effect small changes in the velocity in each of these three directions
            Vector3d[] perturbationDirections = { vesselState.surfaceVelocity.normalized, vesselState.radialPlusSurface, vesselState.normalPlusSurface };

            // Compute the effect burns in these directions would
            // have on the landing position, where we approximate the landing position as the place
            // the perturbed orbit would intersect the planet.
            Vector3d[] deltas = new Vector3d[3];
            for (int i = 0; i < 3; i++)
            {
                const double perturbationDeltaV = 1; //warning: hard experience shows that setting this too low leads to bewildering bugs due to finite precision of Orbit functions
                Orbit perturbedOrbit = orbit.PerturbedOrbit(vesselState.time, perturbationDeltaV * perturbationDirections[i]); //compute the perturbed orbit
                double perturbedLandingTime;
                if (perturbedOrbit.PeR < endRadius) perturbedLandingTime = perturbedOrbit.NextTimeOfRadius(vesselState.time, endRadius);
                else perturbedLandingTime = perturbedOrbit.NextPeriapsisTime(vesselState.time);
                Vector3d perturbedLandingPosition = perturbedOrbit.SwappedRelativePositionAtUT(perturbedLandingTime); //find where it hits the planet
                Vector3d landingDelta = perturbedLandingPosition - orbitLandingPosition; //find the difference between that and the original orbit's intersection point
                landingDelta = convertOrbitToActual * landingDelta; //rotate that difference vector so that we can now think of it as starting at the actual landing position
                landingDelta = Vector3d.Exclude(actualLandingPosition, landingDelta); //project the difference vector onto the plane tangent to the actual landing position
                deltas[i] = landingDelta / perturbationDeltaV; //normalize by the delta-V considered, so that deltas now has units of meters per (meter/second) [i.e., seconds]
            }

            // Now deltas stores the predicted offsets in landing position produced by each of the three perturbations. 
            // We now figure out the offset we actually want

            // First we compute the target landing position. We have to convert the latitude and longitude of the target
            // into a position. We can't just get the current position of those coordinates, because the planet will
            // rotate during the descent, so we have to account for that.
            Vector3d desiredLandingPosition = mainBody.GetRelSurfacePosition(core.target.targetLatitude, core.target.targetLongitude, 0);
            float bodyRotationAngleDuringDescent = (float)(360 * ((prediction as LandedReentryResult).touchdownTime - vesselState.time) / mainBody.rotationPeriod);
            Quaternion bodyRotationDuringFall = Quaternion.AngleAxis(bodyRotationAngleDuringDescent, mainBody.angularVelocity.normalized);
            desiredLandingPosition = bodyRotationDuringFall * desiredLandingPosition;

            Vector3d desiredDelta = desiredLandingPosition - actualLandingPosition;
            desiredDelta = Vector3d.Exclude(actualLandingPosition, desiredDelta);

            // Now desiredDelta gives the desired change in our actual landing position (projected onto a plane
            // tangent to the actual landing position).

            Vector3d downrangeDirection;
            Vector3d downrangeDelta;
            if (allowPrograde)
            {
                // Construct the linear combination of the prograde and radial+ perturbations 
                // that produces the largest effect on the landing position. The Math.Sign is to
                // detect and handle the case where radial+ burns actually bring the landing sign closer
                // (e.g. when we are traveling close to straight up)
                downrangeDirection = (deltas[0].magnitude * perturbationDirections[0]
                    + Math.Sign(Vector3d.Dot(deltas[0], deltas[1])) * deltas[1].magnitude * perturbationDirections[1]).normalized;

                downrangeDelta = Vector3d.Dot(downrangeDirection, perturbationDirections[0]) * deltas[0]
                    + Vector3d.Dot(downrangeDirection, perturbationDirections[1]) * deltas[1];
            }
            else
            {
                // If we aren't allowed to burn prograde, downrange component of the landing
                // position has to be controlled by radial+/- burns:
                downrangeDirection = perturbationDirections[1];
                downrangeDelta = deltas[1];
            }

            // Now solve a 2x2 system of linear equations to determine the linear combination
            // of perturbationDirection01 and normal+ that will give the desired offset in the
            // predicted landing position.
            Matrix2x2 A = new Matrix2x2(
                downrangeDelta.sqrMagnitude, Vector3d.Dot(downrangeDelta, deltas[2]),
                Vector3d.Dot(downrangeDelta, deltas[2]), deltas[2].sqrMagnitude
            );

            Vector2d b = new Vector2d(Vector3d.Dot(desiredDelta, downrangeDelta), Vector3d.Dot(desiredDelta, deltas[2]));

            Vector2d coeffs = A.inverse() * b;

            Vector3d courseCorrection = coeffs.x * downrangeDirection + coeffs.y * perturbationDirections[2];

            return courseCorrection;
        }

        void DeployParachutes()
        {
            if (vesselState.mainBody.atmosphere && deployChutes)
            {
                foreach (ModuleParachute p in vesselState.parachutes)
                {
                    // what is the ASL at which we should deploy this parachute? It is the actual deployment height above the surface + the ASL of the predicted landing point.
                    double LandingSiteASL = LandingAltitude;
                    double ParachuteDeployAboveGroundAtLandingSite = p.deployAltitude;

                    double ASLDeployAltitude = ParachuteDeployAboveGroundAtLandingSite + LandingSiteASL;

                    if (p.part.inverseStage >= limitChutesStage && p.deploymentState == ModuleParachute.deploymentStates.STOWED && ASLDeployAltitude > vesselState.altitudeASL)
                    {
                        p.Deploy();
                        // Debug.Log("Deploying parachute " + p.name + " at " + ASLDeployAltitude + ". (" + LandingSiteASL + " + " + ParachuteDeployAboveGroundAtLandingSite +")");
                    }
                }
            }
        }

        // This methods works out if there are any parachutes that are capable of being deployed
        public bool ParachutesDeployable()
        {
            if (!vesselState.mainBody.atmosphere) return false;
            if (!deployChutes) return false;

            foreach (ModuleParachute p in vesselState.parachutes)
            {
                if (p.part.inverseStage >= limitChutesStage && p.deploymentState == ModuleParachute.deploymentStates.STOWED)
                {
                    return true;
                }
            }

            return false;
        }

        // This methods works out if there are any parachutes that have already been deployed (or semi deployed)
        public bool ParachutesDeployed()
        {
            return vesselState.parachuteDeployed;
        }


        void DeployLandingGears()
        {
            //new-style landing legs are activated by an event:
            //vessel.rootPart.SendEvent("LowerLeg");

            //old-style landings legs are activated on part activation:
            foreach (Part p in vessel.parts)
            {
                if (p.HasModule<ModuleLandingLeg>()) 
                if ( p.inverseStage >= limitGearsStage )
                    foreach (ModuleLandingLeg l in p.FindModulesImplementing<ModuleLandingLeg>())
                        l.LowerLeg();

                if (p is LandingLeg)
                {
                    LandingLeg l = (LandingLeg)p;
                    if (l.legState == LandingLeg.LegStates.RETRACTED)
                    {
                        l.DeployOnActivate = true;
                        l.force_activate();
                    }
                }
            }
            deployedGears = true;
        }

        public double DecelerationEndAltitude()
        {
            if (UseAtmosphereToBrake())
            {
                // if the atmosphere is thick, deceleration (meaning freefall through the atmosphere)
                // should end a safe height above the landing site in order to allow braking from terminal velocity
                double landingSiteDragLength = mainBody.DragLength(LandingAltitude, (vesselState.massDrag + ParachuteAddedDragMass()) / vesselState.mass);

                return 2 * landingSiteDragLength + LandingAltitude;
            }
            else
            {
                //if the atmosphere is thin, the deceleration burn should end
                //500 meters above the landing site to allow for a controlled final descent
                return 500 + LandingAltitude;
            }
        }

        //On planets with thick enough atmospheres, we shouldn't do a deceleration burn. Rather,
        //we should let the atmosphere decelerate us and only burn during the final descent to
        //ensure a safe touchdown speed. How do we tell if the atmosphere is thick enough? We check
        //to see if there is an altitude within the atmosphere for which the characteristic distance
        //over which drag slows the ship is smaller than the altitude above the terrain. If so, we can
        //expect to get slowed to near terminal velocity before impacting the ground. 
        public bool UseAtmosphereToBrake()
        {
            //The air density goes like exp(-h/(scale height)), so the drag length goes like exp(+h/(scale height)).
            //Some math shows that if (scale height) > e * (surface drag length) then 
            //there is an altitude at which (altitude) > (drag length at that altitude).
            double seaLevelDragLength = mainBody.DragLength(0, (vesselState.massDrag + ParachuteAddedDragMass()) / vesselState.mass);
            return (1000 * mainBody.atmosphereScaleHeight > 2.71828 * seaLevelDragLength);
        }

        // This is not the exact number, but it's good enough for our use
        public double ParachuteAddedDragMass()
        {
            double addedDragMass = 0;
            if (vesselState.mainBody.atmosphere && deployChutes)
            {
                foreach (ModuleParachute p in vesselState.parachutes)
                {
                    if (p.part.inverseStage >= limitChutesStage)
                        switch (p.deploymentState)
                    {
                        case ModuleParachute.deploymentStates.STOWED:
                        case ModuleParachute.deploymentStates.ACTIVE:
                            addedDragMass += p.part.mass * p.fullyDeployedDrag - p.part.mass * p.stowedDrag;
                            break;
                        case ModuleParachute.deploymentStates.SEMIDEPLOYED:
                            addedDragMass += p.part.mass * p.fullyDeployedDrag - p.part.mass * p.semiDeployedDrag;
                            break;
                    }

                }
            }
            return addedDragMass;
        }

        bool UseLowDeorbitStrategy()
        {
            if (mainBody.atmosphere) return false;

            double periapsisSpeed = orbit.SwappedOrbitalVelocityAtUT(orbit.NextPeriapsisTime(vesselState.time)).magnitude;
            double stoppingDistance = Math.Pow(periapsisSpeed, 2) / (2 * vesselState.limitedMaxThrustAccel);

            return orbit.PeA < 2 * stoppingDistance + mainBody.Radius / 4;
        }

        public double MinAltitude()
        {
            return Math.Min(vesselState.altitudeBottom, Math.Min(vesselState.altitudeASL, vesselState.altitudeTrue));
        }
    }
}