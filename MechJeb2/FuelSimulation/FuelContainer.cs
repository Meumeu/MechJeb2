using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace MuMech
{
	public partial class FuelContainer
	{
		public int partId;

		private Dictionary<int, double> resourceMass;

		// List of nodes that point to the current node
		private List<int> linkedParts = new List<int>();

		private List<int> stacked;

		private Nullable<int> radialParent;
		private bool fuelCrossFeed;

		private int decoupledInStage;

		public double fuelMass { get {return resourceMass.Sum(r => r.Value);}}

		// Follows fuel flow for a given propellant (by name) and returns the list of parts from which resources
		// will be drained
		public List<FuelContainer> GetTanks(int propellantId, Dictionary<int,FuelContainer> availableNodes, HashSet<int> visitedTanks)
		{
			if (PartResourceLibrary.Instance.GetDefinition(propellantId).resourceFlowMode == ResourceFlowMode.NO_FLOW)
			{
				if (resourceMass.ContainsKey(propellantId) && resourceMass[propellantId] <= 0)
					return new List<FuelContainer>();
				return new List<FuelContainer> {this};
			}

			List<FuelContainer> result = new List<FuelContainer>();

			// Rule 1
			if (visitedTanks.Contains(partId))
				return result;

			visitedTanks.Add(partId);

			// Rule 2
			foreach (int p in linkedParts)
			{
				if (availableNodes.ContainsKey(p))
					result.AddRange(availableNodes[p].GetTanks(propellantId, availableNodes, visitedTanks));
			}

			if (result.Count > 0)
				return result;

			// Rule 3
			// There is no rule 3

			// Rule 4
			if (fuelCrossFeed)
			{
				foreach (int p in stacked.Where(availableNodes.ContainsKey))
				{
					result.AddRange(availableNodes[p].GetTanks(propellantId, availableNodes, visitedTanks));
				}
			}

			if (result.Count > 0)
				return result;

			// Rule 5
			if (resourceMass.ContainsKey(propellantId) && resourceMass[propellantId] > 0)
				return new List<FuelContainer> { this };

			// Rule 6
			if (resourceMass.ContainsKey(propellantId) && resourceMass[propellantId] <= 0)
				return new List<FuelContainer>();

			// Rule 7
			if (fuelCrossFeed && radialParent.HasValue && availableNodes.ContainsKey(radialParent.Value))
			{
				return availableNodes[radialParent.Value].GetTanks(propellantId, availableNodes, visitedTanks);
			}

			// Rule 8
			return new List<FuelContainer>();
		}

		private FuelContainer(Part part)
		{
			this.partId = part.GetInstanceID();
			resourceMass = part.Resources.list.ToDictionary(x => x.info.id, x => x.enabled ? x.info.density * x.amount : 0);
			fuelCrossFeed = part.fuelCrossFeed;

			stacked = new List<int>();
			foreach (AttachNode i in part.attachNodes)
			{
				if (i != null && i.attachedPart != null &&
					i.nodeType == AttachNode.NodeType.Stack &&
					i.id != "Strut" &&
					!(part.NoCrossFeedNodeKey.Length > 0 && i.id.Contains(part.NoCrossFeedNodeKey)))
				{
					stacked.Add(i.attachedPart.GetInstanceID());
				}
			}

			if (part.attachMode == AttachModes.SRF_ATTACH && part.parent != null)
				radialParent = part.parent.GetInstanceID();
		}

		private void setupStage(Dictionary<Part, FuelContainer> nodes, Part p, int currentStage)
		{
			if (p.IsUnfiredDecoupler() || p.IsLaunchClamp())
				decoupledInStage = p.inverseStage > currentStage ? p.inverseStage : currentStage;
			else
				decoupledInStage = currentStage;

			foreach (Part child in p.children)
				nodes[child].setupStage(nodes, child, decoupledInStage);
		}
	}

}

