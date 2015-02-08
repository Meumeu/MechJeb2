using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace MuMech
{
	public class FuelContainer
	{
		public class FuelSummary
		{
			public Dictionary<int, FuelContainer> nodes;

			public double fuelMass { get { return nodes.Sum(kv => kv.Value.fuelMass);}}

			// True if any of the containers changed state compared to parent
			public bool stateChanged;

			public FuelSummary(Vessel vessel)
			{
				stateChanged = true;
				Dictionary<Part, FuelContainer> nodes = vessel.parts.ToDictionary(p => p, p => new FuelContainer(p));
				this.nodes = new Dictionary<int, FuelContainer>();
				foreach (CompoundPart p in vessel.parts.OfType<CompoundPart>())
				{
					if (p.Modules.OfType<CompoundParts.CModuleFuelLine>().Count() > 0
						&& nodes.ContainsKey(p.target))
						nodes[p.target].linkedParts.Add(p.parent.GetInstanceID());
				}
				// Keep useful parts
				foreach (var p in nodes)
				{
					// Engines are useful
					if (p.Key.Modules.OfType<ModuleEngines>().Count() > 0
						|| p.Key.Modules.OfType<ModuleEnginesFX>().Count() > 0)
						this.nodes[p.Key.GetInstanceID()] = p.Value;
					// Fuel is useful
					if (p.Value.fuelMass > 0)
						this.nodes[p.Key.GetInstanceID()] = p.Value;
					// Things attached to other things could be useful
					if (p.Value.stacked.Count > 0 || p.Value.radialParent != null)
						this.nodes[p.Key.GetInstanceID()] = p.Value;
				}
			}

			public FuelSummary ApplyConsumption(Dictionary<ResourceIndex, double> rates, double time)
			{
				if (rates.Count == 0)
				{
					if (stateChanged)
					{
						var copy = (FuelSummary) this.MemberwiseClone();
						copy.stateChanged = false;
						return copy;
					}
					else
						return this;
				}
				var res = (FuelSummary) this.MemberwiseClone();
				res.nodes = new Dictionary<int, FuelContainer>(res.nodes);
				foreach (var rate in rates)
				{
					var node = (FuelContainer)res.nodes[rate.Key.partId].MemberwiseClone();
					res.nodes[rate.Key.partId] = node;
					node.resourceMass = new Dictionary<int, double>(node.resourceMass);
					double newMass = node.resourceMass[rate.Key.resourceId] - rate.Value * time;
					node.resourceMass[rate.Key.resourceId] = Math.Max(0, newMass);
					if (newMass <= 0)
						res.stateChanged = true;
				}
				return res;
			}
		}
		public int partId;

		private Dictionary<int, double> resourceMass;

		// List of nodes that point to the current node
		private List<int> linkedParts = new List<int>();

		private List<int> stacked;
		private Nullable<int> radialParent;
		private bool fuelCrossFeed;

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
				if (i != null && i.attachedPart != null &&
					i.attachedPart == part.parent &&
					i.nodeType == AttachNode.NodeType.Surface)
				{
					radialParent = i.attachedPart.GetInstanceID();
				}
			}
		}
	}

}

