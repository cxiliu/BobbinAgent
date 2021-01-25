using Rhino.Geometry;
using System.Collections.Generic;
using ICD.AbmFramework.Core.Environments;
using ICD.AbmFramework.Core.Agent;
using ICD.AbmFramework.Core.AgentSystem;
using ICD.AbmFramework.Core.Behavior;


namespace ICD.AbmFramework.Core.AgentSystem
{
    /// <summary>
    /// Bobbin bot agent system
    /// </summary>
    public class BobbinBotSystem : AgentSystemBase
    {
        public BobbinEnvironment BobbinEnvironment;
        public double MaxSpeed = 2.0;
        public double MaxForce = 3.0;
        public double TimeStep = 0.05; //0.02

        public BobbinBotSystem(List<BobbinBot> agents, BobbinEnvironment bobbinEnvironment)
        {
            BobbinEnvironment = bobbinEnvironment as BobbinEnvironment;
            Agents = new List<AgentBase>();
            foreach (BobbinBot agent in agents)
            {
                Agents.Add(agent as BobbinBot);
                agent.AgentSystem = this;
            }
        }

        public override void Reset()
        {
            base.Reset();
        }

        public override void Execute()
        {
            foreach (BobbinBot agent in Agents)
                agent.Execute();
        }

        public override void PostExecute()
        {
            foreach (BobbinBot agent in Agents)
                agent.PostExecute();
        }

        public List<BobbinBot> FindBotNeighbors(BobbinBot agent, double distance)
        {
            List<BobbinBot> neighbors = new List<BobbinBot>();

            foreach (BobbinBot potentialNeighbor in Agents)
            {
                if (agent != potentialNeighbor && agent.Position.DistanceTo(potentialNeighbor.Position) < distance)
                    neighbors.Add(potentialNeighbor);
            }

            return neighbors;
        }

        public int FindClosestBotReady(BobbinBot agent)
        {
            int closestBotIndex = -1;
            double closestDistance = 100000; // an arbitrarily large number

            foreach (BobbinBot potentialNeighbor in Agents)
            {
                if (agent != potentialNeighbor)
                {
                    if (!(potentialNeighbor.leftActive && potentialNeighbor.rightActive) || potentialNeighbor.paired)
                        continue;

                    double distance = agent.Position.DistanceTo(potentialNeighbor.Position);
                    if (distance < closestDistance)
                    {
                        closestDistance = distance;
                        closestBotIndex = agent.AgentSystem.Agents.IndexOf(potentialNeighbor);
                    }
                }
            }
            return closestBotIndex;
        }

        public override List<object> GetDisplayGeometries()
        {
            List<object> displayGeometry = new List<object>();

            //foreach (BobbinBot agent in Agents)
            //    displayGeometry.AddRange(agent.GetDisplayGeometries());

            displayGeometry.AddRange(BobbinEnvironment.GetDisplayGeometry());
            return displayGeometry;
        }
    }
}