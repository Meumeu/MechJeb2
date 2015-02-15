using System;

namespace MuMech
{
	public class ResourceIndex
	{
		public readonly int resourceId;
		public readonly int partId;
		public ResourceIndex(int resourceId, int partId)
		{
			this.resourceId = resourceId;
			this.partId = partId;
		}
		public override bool Equals(object other)
		{
			var o = other as ResourceIndex;
			if (o == null)
				return false;
			return resourceId == o.resourceId && partId == o.partId;
		}
		public override int GetHashCode ()
		{
			return resourceId ^ partId;
		}
	}
	
}
