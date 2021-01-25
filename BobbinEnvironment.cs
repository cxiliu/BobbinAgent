using Rhino.Geometry;
using System.Collections.Generic;
using ICD.AbmFramework.Core.Environments;
using ICD.AbmFramework.Core.Agent;
using ICD.AbmFramework.Core.AgentSystem;
using ICD.AbmFramework.Core.Behavior;

namespace ICD.AbmFramework.Core.Environments
{
    public class BobbinEnvironment : EnvironmentBase
    {
        public List<Point3d> Bobbins;
        public List<int> BusyList;
        // not boundary corners

        public BobbinEnvironment(List<Point3d> bobbins)
        {
            this.Bobbins = bobbins;
            this.BusyList = new List<int>();
        }

        // find the first bobbin that is within range for picking
        public int FindBobbin(BobbinBot bot, double distance)
        {
            Point3d position = bot.Position;
            foreach (Point3d bobbinPos in Bobbins)
            {
                // don't be attracted to committed bobbins
                if (this.BusyList.Contains(Bobbins.IndexOf(bobbinPos)))
                    continue;
                if (position.DistanceTo(bobbinPos) < distance)
                {
                    if ((Bobbins.IndexOf(bobbinPos) == bot.leftIndex) || (Bobbins.IndexOf(bobbinPos) == bot.rightIndex))
                    {
                        continue;
                    }
                    else
                    {
                        return Bobbins.IndexOf(bobbinPos);
                    }
                }
            }
            return -1;
        }

        public void GrabBobbin(int bobbinIndex)
        {
            this.BusyList.Add(bobbinIndex);
        }

        public void FreeBobbin(int bobbinIndex)
        {
            this.BusyList.Remove(bobbinIndex);
        }

        // bobbins within this range is "visible" and attract bots
        public List<Point3d> FindPotentialBobbin(BobbinBot bot, double distance)
        {
            List<Point3d> neighbors = new List<Point3d>();
            
            foreach (Point3d bobbinPos in Bobbins)
            {
                // don't be attracted to committed bobbins
                if (this.BusyList.Contains(Bobbins.IndexOf(bobbinPos)))
                    continue;
                if (bot.Position.DistanceTo(bobbinPos) < distance)
                {
                    // assuming bot's eyes have 180 fov
                    if (System.Math.Abs(Vector3d.VectorAngle((bot.Position - bobbinPos), bot.Velocity)) < 90)
                        neighbors.Add(bobbinPos);
                }
            }
            return neighbors;
        }

        public override List<object> GetDisplayGeometry()
        {
            List<object> displayGeometries = new List<object>();
            foreach (Point3d pos in Bobbins)
            {
                displayGeometries.Add(pos);
            }
            return displayGeometries;
        }
    }
}