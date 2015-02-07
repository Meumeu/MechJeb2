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
			public Dictionary<Part, FuelContainer> nodes;

			public double fuelMass { get { return nodes.Sum(kv => kv.Value.fuelMass);}}

			// True if any of the containers changed state compared to parent
			public bool stateChanged;

			public FuelSummary(Vessel vessel)
			{
				stateChanged = true;
				nodes = vessel.parts.ToDictionary(p => p, p => new FuelContainer(p));
				foreach (CompoundPart p in vessel.parts.OfType<CompoundPart>())
				{
					if (p.Modules.OfType<CompoundParts.CModuleFuelLine>().Count() > 0
						&& nodes.ContainsKey(p.target))
						nodes[p.target].linkedParts.Add(p.parent);
				}
				// Remove useless parts
				List<Part> useless = new List<Part>();
				foreach (var p in nodes)
				{
					// Engines are useful
					if (p.Key.Modules.OfType<ModuleEngines>().Count() > 0
						|| p.Key.Modules.OfType<ModuleEnginesFX>().Count() > 0)
						continue;
					// Fuel is useful
					if (p.Value.fuelMass > 0)
						continue;
					// Things attached to other things could be useful
					if (p.Value.stacked.Count > 0 || p.Value.radialParent != null)
						continue;
					useless.Add(p.Key);
				}
				foreach (var p in useless)
					nodes.Remove(p);
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
				res.nodes = new Dictionary<Part, FuelContainer>(res.nodes);
				foreach (var rate in rates)
				{
					var node = (FuelContainer)res.nodes[rate.Key.part].MemberwiseClone();
					res.nodes[rate.Key.part] = node;
					node.resourceMass = new Dictionary<int, double>(node.resourceMass);
					double newMass = node.resourceMass[rate.Key.resourceId] - rate.Value * time;
					node.resourceMass[rate.Key.resourceId] = Math.Max(0, newMass);
					if (newMass <= 0)
						res.stateChanged = true;
				}
				return res;
			}
		}
		public Part part;

		private Dictionary<int, double> resourceMass;

		// List of nodes that point to the current node
		private List<Part> linkedParts = new List<Part>();

		private List<Part> stacked;
		private Part radialParent;

		public double fuelMass { get {return resourceMass.Sum(r => r.Value);}}

		// Follows fuel flow for a given propellant (by name) and returns the list of parts from which resources
		// will be drained
		public List<FuelContainer> GetTanks(int propellantId, Dictionary<Part,FuelContainer> availableNodes, HashSet<Part> visitedTanks)
		{
			if (PartResourceLibrary.Instance.GetDefinition(propellantId).resourceFlowMode == ResourceFlowMode.NO_FLOW)
			{
				if (resourceMass.ContainsKey(propellantId) && resourceMass[propellantId] <= 0)
					return new List<FuelContainer>();
				return new List<FuelContainer> {this};
			}

			List<FuelContainer> result = new List<FuelContainer>();

			// Rule 1
			if (visitedTanks.Contains(part))
				return result;

			visitedTanks.Add(part);

			// Rule 2
			foreach (Part p in linkedParts)
			{
				if (availableNodes.ContainsKey(p))
					result.AddRange(availableNodes[p].GetTanks(propellantId, availableNodes, visitedTanks));
			}

			if (result.Count > 0)
				return result;

			// Rule 3
			// There is no rule 3

			// Rule 4
			if (part.fuelCrossFeed)
			{
				foreach (Part p in stacked.Where(availableNodes.ContainsKey))
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
			if (part.fuelCrossFeed && radialParent != null && availableNodes.ContainsKey(radialParent))
			{
				return availableNodes[radialParent].GetTanks(propellantId, availableNodes, visitedTanks);
			}

			// Rule 8
			return new List<FuelContainer>();
		}

		private FuelContainer(Part part)
		{
			this.part = part;
			resourceMass = part.Resources.list.ToDictionary(x => x.info.id, x => x.enabled ? x.info.density * x.amount : 0);

			stacked = new List<Part>();
			foreach (AttachNode i in part.attachNodes)
			{
				if (i != null && i.attachedPart != null &&
					i.nodeType == AttachNode.NodeType.Stack &&
					i.id != "Strut" &&
					!(part.NoCrossFeedNodeKey.Length > 0 && i.id.Contains(part.NoCrossFeedNodeKey)))
				{
					stacked.Add(i.attachedPart);
				}
				if (i != null && i.attachedPart != null &&
					i.attachedPart == part.parent &&
					i.nodeType == AttachNode.NodeType.Surface)
				{
					radialParent = i.attachedPart;
				}
			}
		}
	}

}

