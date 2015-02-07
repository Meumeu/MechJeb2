using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace MuMech
{
	public class FuelContainer
	{
		public Part part;

		private Dictionary<int, double> resourceMass;

		// List of nodes that point to the current node
		private List<Part> linkedParts = new List<Part>();

		private List<Part> stacked;
		private Part radialParent;

		private double baseMass;

		public static Dictionary<Part, FuelContainer> Build(Vessel vessel)
		{
			var res = vessel.parts.ToDictionary(p => p, p => new FuelContainer(p));
			foreach (CompoundPart p in vessel.parts.OfType<CompoundPart>())
			{
				if (p.Modules.OfType<CompoundParts.CModuleFuelLine>().Count() > 0
					&& res.ContainsKey(p.target))
					res[p.target].linkedParts.Add(p.parent);
			}
			return res;
		}

		public static Dictionary<Part, FuelContainer> ApplyConsumption(Dictionary<Part, FuelContainer> nodes, Dictionary<ResourceIndex, double> rates, double time)
		{
			if (rates.Count == 0)
				return nodes;
			var res = new Dictionary<Part, FuelContainer>(nodes);
			foreach (var rate in rates)
			{
				var node = (FuelContainer)res[rate.Key.part].MemberwiseClone();
				res[rate.Key.part] = node;
				node.resourceMass = new Dictionary<int, double>(node.resourceMass);
				node.resourceMass[rate.Key.resourceId] = Math.Max(0, node.resourceMass[rate.Key.resourceId] - rate.Value * time);
			}
			return res;
		}

		public double fuelMass { get {return resourceMass.Sum(r => r.Value);}}

		public double mass { get { return baseMass + fuelMass;}}

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
			if (part.IsPhysicallySignificant())
				baseMass = part.mass + part.Resources.list.Sum(x => x.enabled ? 0 : x.info.density * x.amount);
			else
				baseMass = 0;

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

