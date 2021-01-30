using System;
using System.Collections;
using System.Collections.Generic;

using Rhino;
using Rhino.Geometry;

using Grasshopper;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;

// <Custom "using" statements>
using ICD.AbmFramework.Core;
using ICD.AbmFramework.Core.Agent;
using ICD.AbmFramework.Core.AgentSystem;
using ICD.AbmFramework.Core.Behavior;
using ICD.AbmFramework.Core.Environments;
using System.Drawing;
using System.Runtime.CompilerServices;
using System.Collections.Specialized;
// </Custom "using" statements>


#region padding (this ensures the line number of this file match with those in the code editor of the C# Script component
















#endregion

public partial class MyExternalScript : GH_ScriptInstance
{
    #region Do_not_modify_this_region
    private void Print(string text) { }
    private void Print(string format, params object[] args) { }
    private void Reflect(object obj) { }
    private void Reflect(object obj, string methodName) { }
    public override void InvokeRunScript(IGH_Component owner, object rhinoDocument, int iteration, List<object> inputs, IGH_DataAccess DA) { }
    public RhinoDoc RhinoDocument;
    public GH_Document GrasshopperDocument;
    public IGH_Component Component;
    public int Iteration;
    #endregion







    private void RunScript(List<Point3d> P, List<Vector3d> V, List<Point3d> Bobbins, double MF, double MS, Rectangle3d Bounds, ref object oAgents, ref object oAgentSystem)
        {
        // <Custom code>

        // Problems with a purely behaviour based approach: 
        // how to avoid bobbins being "shortsighted"?
        // how to make sure crosses are occuring in a safe, collision-free way?        

        List<BehaviorBase> behaviours = new List<BehaviorBase>();
        
        behaviours.Add(new RectangleContainmentBehaviour(Bounds, 1.0)); // containment
        behaviours.Add(new AvoidanceBehaviour(1000.0, 20.0*2, true)); // go away when another bot is within 20mm

        behaviours.Add(new PickupBobbinBehaviour(10.0));  // pick up becomes possible when bobbin is within 10mm
        behaviours.Add(new DropOffBobbinBehaviour(10.0));

        behaviours.Add(new FindPartnerBehaviour(40.0)); // find partner
        behaviours.Add(new TwistBehaviour());
        behaviours.Add(new CrossBehaviour(20.0));

        behaviours.Add(new AttractionBehaviour(1.0, 1.0, 20.0)); // detect bobbins within 20*1 with the target of being 1mm away
        //behaviours.Add(new RandomWalkBehaviour(50.0, 10, 10.0)); // random walk every 10 iterations with 1000 velocity

        List<BobbinBot> bobbinBots = new List<BobbinBot>();
        for (int i = 0; i < P.Count; i++)
        {
            bobbinBots.Add(new BobbinBot(P[i], V[i], behaviours));
        }
        oAgents = bobbinBots;

        BobbinEnvironment bobbinEnvironment = new BobbinEnvironment(Bobbins);

        BobbinBotSystem bs = new BobbinBotSystem(bobbinBots, bobbinEnvironment);
        bs.MaxSpeed = MS;
        bs.MaxForce = MF;

        oAgentSystem = bs;

        // </Custom code>
    }

    // <Custom additional code>


    /// <summary>
    /// pick up bobbins when it's free
    /// </summary>
    public class PickupBobbinBehaviour : BehaviorBase
    {
        // when it is "empty", approach the first and closes bobbin for pickup (left first)
        // when one of left/right is active, approach the first and closest bobbin for pickup on the right
        public double PickupDistance;

        public PickupBobbinBehaviour(double pickupDistance)
        {
            PickupDistance = pickupDistance;
        }

        public override void Execute(AgentBase agent)
        {
            BobbinBot bot = (BobbinBot) agent;

            // if the bot is already full or it is not "free", ignore
            if ((bot.leftActive && bot.rightActive) || bot.State != BotState.Free)
                return;

            // if the bot has just finished some routine, do not pick up anything until things are reset
            if (bot.currentTwists + bot.currentCrosses >=1)
                return;

            // if the bot just finished a drop, do not pick up anything too close to the drop
            if (bot.secondDropPosition.DistanceTo(bot.Position) < 45.0)
                return;

            // find bobbin for pickup
            int target = bot.AgentSystem.BobbinEnvironment.FindBobbin(bot, PickupDistance);
            if (target == -1)
                return;

            // if both are empty, grip left one first
            // TODO: let bot decide which one to get
            if (!bot.leftActive && !bot.rightActive)
            {
                bot.GrabBobbin(target, true);
            }
            else
            {
                if (bot.leftActive)
                {
                    // grip right
                    bot.GrabBobbin(target, false);
                    bot.SetState(BotState.Twist);
                    bot.CustomData["current_state"] = "twisting";
                }
                else
                {
                    // grip left
                    bot.GrabBobbin(target, true);
                }
            }
        }
    }


    /// <summary>
    /// drop off bobbins when it's free
    /// </summary>
    public class DropOffBobbinBehaviour : BehaviorBase
    {
        // always drop off left first
        double DropoffDistance;
        bool lastLeft;

        public DropOffBobbinBehaviour(double dropoffDistance)
        {
            DropoffDistance = dropoffDistance;
        }

        public override void Execute(AgentBase agent)
        {
            BobbinBot bot = (BobbinBot)agent;

            // if the bot is not carrying anything, ignore
            if (!bot.leftActive && !bot.rightActive)
                return;

            // if bot has taken less than three actions, ignore
            // if bot is too close to pairing position, ignore
            // TTC, TCT, TCTC, CTT, CTC, CTCT
            if (bot.currentTwists + bot.currentCrosses < 3)
                return;

            // if both are gripping, drop off left first
            if (bot.leftActive &&  bot.rightActive)
            {
                // the bot has not gone far enough yet, do not drop the bobbin
                if (bot.Position.DistanceTo(bot.unpairPosition) < 30.0)
                {
                    bot.CustomData["current_state"] = "dumping 1st bobbin";
                    return;
                }

                // pick one to drop real quick
                //if (lastLeft)
                //{
                //    bot.ReleaseBobbin(false);
                //    bot.TwistCount += bot.currentTwists;
                //    bot.CrossCount += bot.currentCrosses;
                //    bot.AddForce( new Vector3d(-DropoffDistance * System.Math.Cos(bot.HeadingAngle), -DropoffDistance * System.Math.Sin(bot.HeadingAngle), 0));
                //    lastLeft = false;
                //}
                //else
                //{
                    bot.ReleaseBobbin(true);
                    bot.TwistCount += bot.currentTwists;
                    bot.CrossCount += bot.currentCrosses;
                    bot.AddForce( new Vector3d(DropoffDistance * System.Math.Cos(bot.HeadingAngle), DropoffDistance * System.Math.Sin(bot.HeadingAngle), 0));
                    lastLeft = true;
                //}
                bot.firstDropPosition = bot.Position;
            }
            else
            {
                // the bot has not gone far enough yet, do not drop the second bobbin
                if (bot.Position.DistanceTo(bot.firstDropPosition) < 45.0)
                {
                    bot.CustomData["current_state"] = "dumping 2nd bobbin";
                    return;
                }

                if (bot.leftActive)
                {
                    bot.ReleaseBobbin(true);
                    bot.AddForce( new Vector3d(-DropoffDistance * System.Math.Cos(bot.HeadingAngle), -DropoffDistance * System.Math.Sin(bot.HeadingAngle), 0));
                    lastLeft = false;
                }
                else
                {
                    bot.ReleaseBobbin(false);
                    bot.AddForce( new Vector3d(DropoffDistance * System.Math.Cos(bot.HeadingAngle), DropoffDistance * System.Math.Sin(bot.HeadingAngle), 0));
                    lastLeft = true;
                }

                bot.CustomData["current_state"] = "is free again! ";
                bot.secondDropPosition = bot.Position;

                // the bot starts with a clean slate
                bot.currentCrosses = 0;
                bot.currentTwists = 0;
            }
        }
    }


    /// <summary>
    /// do the shit
    /// </summary>
    public class TwistBehaviour : BehaviorBase
    {
        public bool Ready = true;
        private double startHeading;

        public TwistBehaviour()
        {
        }

        public override void Execute(AgentBase agent)
        {
            BobbinBot bot = (BobbinBot)agent;
            if (!(bot.leftActive && bot.rightActive) || (bot.State != BotState.Twist))
                return;
            
            if (Ready)
            {
                startHeading = bot.HeadingAngle;
                Ready = false;
            }

            bot.SetHeading(bot.HeadingAngle + 0.1);
            if ((bot.HeadingAngle - startHeading) >= System.Math.PI*2)
            {
                // create a release routine
                // release one side, move a little and release the other side
                // OR release one side, and keep looking for another one to braid with
                //bot.ReleaseBobbin(true); // release left first and keep going
                //bot.SetState(BotState.Free);
                //Ready = true;

                bot.currentTwists += 2;
                
                // assume after one twist, go find partner
                bot.SetState(BotState.FindPartner);
                bot.CustomData["current_state"] = "finding partner";
                Ready = true;
            }

        }
    }


    /// <summary>
    /// behaviour that helps bots decide on a partner
    /// </summary>
    public class FindPartnerBehaviour : BehaviorBase
    {
        public bool Ready;
        public bool Finished;
        public double PartnerDistance;

        public FindPartnerBehaviour(double partnerDistance)
        {
            PartnerDistance = partnerDistance;
        }

        public override void Execute(AgentBase agent)
        {
            BobbinBot bot = (BobbinBot)agent;
            if (!(bot.leftActive && bot.rightActive) || bot.paired || (bot.State != BotState.FindPartner))
                return;

            int closestBotIndex = bot.AgentSystem.FindClosestBotReady(bot);

            if (closestBotIndex == -1)
                return;

            int botIndex = bot.AgentSystem.Agents.IndexOf(bot);
            BobbinBot closestBot = (BobbinBot) bot.AgentSystem.Agents[closestBotIndex];

            // if the intended bot is already committed to someone
            if (closestBot.committedPartnerIndex != -1)
            {
                // and this "someone" is itself, then we continue
                if (closestBot.committedPartnerIndex == botIndex)
                {
                    if (closestBot.Position.DistanceTo(bot.Position) <= PartnerDistance)
                    {
                        bot.MakePair(closestBot);
                        bot.SetState(BotState.Cross);
                        closestBot.SetState(BotState.Cross);
                    }
                }
                // if the intended bot was comitted to someone else, ignore it
                else
                    return;
            }
            // if the intended bot is free or still in decision phase
            else
            {
                // and current bot has not yet registered intention
                if (bot.intendedPartnerIndex == -1)
                {
                    bot.intendedPartnerIndex = closestBotIndex;
                    bot.CustomData["current_state"] = "eyeing down " + closestBotIndex.ToString();
                }
                // or if intent is registered, try to make a decision
                else
                {
                    // if the interest is mutual, make a pair
                    if (closestBot.intendedPartnerIndex == botIndex)
                    {
                        bot.MakeAgreement(closestBot);
                    }
                    else
                    {
                        // no deal, keep moving and see what happens
                        bot.intendedPartnerIndex = -1;
                    }
                }

            }
        }
    }


    /// <summary>
    /// do a cross only after the bot has been paired
    /// </summary>
    public class CrossBehaviour : BehaviorBase
    {
        public int CrossCount;
        public double ExchangeDistance;

        public bool Ready = true;
        private int stage;
        private int counter = 0;

        private int leftIndex;
        private int rightIndex;
        private Point3d botPosition;
        private Point3d partnerPosition;

        public CrossBehaviour(double exchangeDistance)
        {
            ExchangeDistance = exchangeDistance;
        }

        public override void Execute(AgentBase agent)
        {
            BobbinBot bot = (BobbinBot)agent;

            if ((bot.State != BotState.Cross) || !bot.isLead || !bot.paired)
                return;
            
            BobbinBotSystem botSystem = (BobbinBotSystem)bot.AgentSystem;
            BobbinBot partner = (BobbinBot)botSystem.Agents[bot.committedPartnerIndex];

            // if bot in cross state
            // 0: store starting state

            if (Ready)
            {
                leftIndex = bot.rightIndex;
                rightIndex = partner.leftIndex;
                botPosition = new Point3d(bot.Position.X, bot.Position.Y, bot.Position.Z);
                partnerPosition = new Point3d(partner.Position.X, partner.Position.Y, partner.Position.Z);
                stage = 0;
                counter = 0;
                Ready = false;
                return;
            }

            Vector3d orientationVec = partnerPosition - botPosition;
            orientationVec.Unitize();
            double targetAngle = Vector3d.VectorAngle(orientationVec, Vector3d.YAxis) + System.Math.PI / 2.0;
            Point3d midPt = (partnerPosition + botPosition) / 2;
            Vector3d moveVec = new Vector3d(0, 1.0, 0);
            moveVec.Rotate(targetAngle, Vector3d.ZAxis);
            
            int interval = 10; // animation speed

            switch (stage)
            {
                case 0:
                    counter += 1;
                    if (counter > interval)
                    {
                        // 1: align
                        partner.SetHeading(targetAngle);
                        bot.SetHeading(targetAngle);
                        stage += 1;
                        counter = 0;
                    }
                    break;
                case 1:
                    counter += 1;
                    if (counter > interval)
                    {
                        // 2: move to diagonal position
                        bot.Position = midPt + moveVec * ExchangeDistance - orientationVec * 10.0;
                        partner.Position = midPt - moveVec * ExchangeDistance + orientationVec * 10.0;
                        stage += 1;
                        counter = 0;
                    }
                    break;
                case 2:
                    // 3: drop one bobbin
                    bot.ReleaseBobbin(false);
                    partner.ReleaseBobbin(true);
                    stage += 1;
                    break;
                case 3:
                    counter += 1;
                    if (counter > interval)
                    {
                        // 4: move back to starting position
                        bot.Position = botPosition;
                        partner.Position = partnerPosition;
                        stage += 1;
                        counter = 0;
                    }
                    break;
                case 4:
                    counter += 1;
                    if (counter > interval)
                    {
                        // 5: move to each other's position
                        bot.Position = midPt - moveVec * ExchangeDistance + orientationVec * 10.0;
                        partner.Position = midPt + moveVec * ExchangeDistance - orientationVec * 10.0;
                        stage += 1;
                        counter = 0;
                    }
                    break;
                case 5:
                    // 6: pick one bobbin
                    bot.GrabBobbin(rightIndex, false);
                    partner.GrabBobbin(leftIndex, true);
                    stage += 1;
                    break;
                case 6:
                    counter += 1;
                    if (counter > interval)
                    {
                        // 6: back to start position
                        bot.Position = botPosition;
                        partner.Position = partnerPosition;
                        stage += 1;
                        counter = 0;
                    }
                    break;
                case 7:
                    counter += 1;
                    if (counter > interval)
                    {
                        Vector3d disperse = bot.Position - partner.Position;
                        disperse.Unitize();
                        disperse *= 1.0;
                        bot.AddForce(disperse);
                        partner.AddForce(-disperse);
                        bot.Unpair(partner);
                        bot.currentCrosses += 1;
                        partner.currentCrosses += 1;

                        bot.SetState(BotState.Free);
                        partner.SetState(BotState.Free);
                        Ready = true;
                        counter = 0;
                    }
                    break;
            }
        }
    }


    /// <summary>
    /// find bobbins when it's free
    /// </summary>
    public class RandomWalkBehaviour : BehaviorBase
    {
        public double Craziness;
        public double VelocityRange;
        private int counter;
        private Random rdm; 
        // random walk around and avoid collisions
        // approach closest bobbin

        public RandomWalkBehaviour(double weight, double craziness, double velocityRange)
        {
            Weight = weight;
            Craziness = craziness;
            VelocityRange = velocityRange;
            rdm = new Random();
        }
        
        private Vector3d RandomVector()
        {
            return new Vector3d(
                VelocityRange * (rdm.NextDouble() * 2.0 - 1.0),
                VelocityRange * (rdm.NextDouble() * 2.0 - 1.0), 0);
        }

        public override void Execute(AgentBase agent)
        {
            BobbinBot bot = (BobbinBot)agent;


            // spice it up every now and again
            counter += 1;
            if (counter >= Craziness)
            {
                bot.AddForce(new Vector3d(
            VelocityRange * (rdm.NextDouble() * 2.0 - 1.0),
            VelocityRange * (rdm.NextDouble() * 2.0 - 1.0), 0));
                counter = 0;
            }
        }
    }


    /// <summary>
    /// bot move towards bobbins when it is free
    /// bot move towards partner when it reaches agreement
    /// </summary>
    public class AttractionBehaviour : BehaviorBase
    {
        public double TargetDistance;
        public double RangeFactor;
        public double Pickiness;

        public AttractionBehaviour(double weight, double distance, double rangeFactor)
        {
            Weight = weight;
            TargetDistance = distance;
            RangeFactor = rangeFactor;
        }

        public override void Execute(AgentBase agent)
        {
            BobbinBot bot = (BobbinBot)agent;
            List<Point3d> neighbors = new List<Point3d>();
            BobbinBotSystem botSystem = (BobbinBotSystem)bot.AgentSystem;

            // if the bot is finding a partner
            if (bot.State == BotState.FindPartner)
            {
                // and through FindPartnerBehaviour bot has found the right partner
                if (bot.committedPartnerIndex != -1)
                {
                    BobbinBot closestBot = botSystem.Agents[bot.committedPartnerIndex] as BobbinBot;
                    neighbors.Add(closestBot.Position);
                }
            }
            // if the bot is free, it's just attracted to other bobbins
            else if (bot.State == BotState.Free)
            {
                // unless it is already busy
                if (bot.leftActive && bot.rightActive)
                    return;
                // or a "disperse" action is in progress
                // bot is never attracted by bobbins in its previous activity area
                if ((bot.Position.DistanceTo(bot.firstDropPosition) < 30.0) || (bot.Position.DistanceTo(bot.secondDropPosition) < 45.0))
                    return;
                
                double rangeDistance = TargetDistance * RangeFactor;
                neighbors = botSystem.BobbinEnvironment.FindPotentialBobbin(bot, rangeDistance);

                // if the potential bobbins are too close to where it dropped off the last guy, remove this attraction
                List<Point3d> newN = new List<Point3d>();
                foreach (Point3d nei in neighbors)
                {
                    if (nei.DistanceTo(bot.secondDropPosition) >45.0){
                        newN.Add(nei);                    
                    }
                }
                neighbors = newN;
            }

            if (neighbors.Count == 0 || Weight == 0.0)
                return;

            // for all neighbours that are within range, calculate distance - target distance; apply this as vector
            Vector3d movementVector = new Vector3d();
            foreach (Point3d bobbin in neighbors)
            {
                Vector3d toThis = bobbin - bot.Position;
                double dist = toThis.Length;
                if (dist > TargetDistance) //dist < rangeDistance && 
                {
                    double strength = ((dist - TargetDistance) / TargetDistance);
                    toThis.Unitize();
                    toThis *= strength;
                    movementVector += toThis;
                }
            }

            // apply combined movement
            bot.AddForce(Weight * movementVector);
        }
    }


    /// <summary>
    /// bot moves away from each other unless they are paired
    /// bot avoids bobbins when its hands are full
    /// </summary>
    public class AvoidanceBehaviour : BehaviorBase
    {
        public double Distance;
        public double Power = 10.0;
        public bool AffectSelf;

        public AvoidanceBehaviour(double weight, double distance, bool affectSelf)
        {
            Weight = weight;
            Distance = distance;
            AffectSelf = affectSelf;
        }

        public override void Execute(AgentBase agent)
        {
            BobbinBot bot = (BobbinBot)agent;
            BobbinBotSystem botSystem = (BobbinBotSystem) bot.AgentSystem;
            Vector3d separation = Vector3d.Zero;
            
            // avoid other bobbin bots at all times
            List<BobbinBot> neighbors = botSystem.FindBotNeighbors(bot, this.Distance);
            bot.CustomData["collision_count"] = "";
            //foreach (BobbinBot nei in neighbors)
            //{
            //    bot.CustomData["collision_count"] += botSystem.Agents.IndexOf(nei).ToString() + ",";
            //}
            if (neighbors.Count > 0)
            {
                foreach (BobbinBot neighbor in neighbors)
                {
                    // avoid all neighbors except the one the bot is paired with
                    if (botSystem.Agents.IndexOf(neighbor) == bot.committedPartnerIndex)
                        continue;
                    else
                    {
                        Vector3d moveAway = bot.Position - neighbor.Position;
                        double len = moveAway.Length;

                        if (moveAway.IsZero) moveAway = Vector3d.XAxis;

                        if (len < Distance)
                        {
                            moveAway.Unitize();
                            Vector3d thisMove = Weight * moveAway * Math.Pow((Distance - len) / Distance, Power) * Distance;

                            if (AffectSelf)
                            {
                                separation += thisMove;
                            }
                            else // move neighbour
                            {
                                neighbor.AddForce(-thisMove);
                            }
                        }
                    }
                }
            }

            // if the bot is already carrying bobbins, then bot should avoid other bobbins too
            // (this bobbin list does not include the ones currently carried by other bots)
            // bot is always repulsed by bobbins in its previous activity area
            if ((bot.leftActive && bot.rightActive) || (bot.Position.DistanceTo(bot.firstDropPosition) < 30.0) || (bot.Position.DistanceTo(bot.secondDropPosition) < 45.0))
            {
                List<Point3d> neighborBobbins = botSystem.BobbinEnvironment.FindPotentialBobbin(bot, this.Distance);
                bot.CustomData["collision_count"] = neighborBobbins.Count.ToString();
                if (neighborBobbins.Count == 0)
                    return;

                foreach (Point3d neighbor in neighborBobbins)
                {
                    Vector3d moveAway = bot.Position - neighbor;
                    double len = moveAway.Length;

                    if (moveAway.IsZero) moveAway = Vector3d.XAxis;

                    if (len < Distance)
                    {
                        moveAway.Unitize();
                        Vector3d thisMove = Weight * moveAway * Math.Pow((Distance - len) / Distance, Power) * Distance;
                        separation += thisMove;
                    }
                }
            }


            bot.AddForce(separation * Weight);
        }
    }


    /// <summary>
    /// stay in line
    /// </summary>
    public class RectangleContainmentBehaviour : BehaviorBase
    {

        public Rectangle3d Rectangle = Rectangle3d.Unset;

        public RectangleContainmentBehaviour(Rectangle3d bounds, double weight)
        {
            Rectangle = bounds;
            Weight = weight;
        }

        public override void Execute(AgentBase agent)
        {
            BobbinBot bot = agent as BobbinBot;

            if (Rectangle.IsValid)
            {
                if (bot.Position.X < Rectangle.BoundingBox.Min.X)
                {
                    bot.Velocity.X *= -1;
                    bot.Position.X = Rectangle.BoundingBox.Min.X;
                }
                else if (bot.Position.X > Rectangle.BoundingBox.Max.X)
                {
                    bot.Velocity.X *= -1;
                    bot.Position.X = Rectangle.BoundingBox.Max.X;
                }
                if (bot.Position.Y < Rectangle.BoundingBox.Min.Y)
                {
                    bot.Velocity.Y *= -1;
                    bot.Position.Y = Rectangle.BoundingBox.Min.Y;
                }
                else if (bot.Position.Y > Rectangle.BoundingBox.Max.Y)
                {
                    bot.Velocity.Y *= -1;
                    bot.Position.Y = Rectangle.BoundingBox.Max.Y;
                }
            }
        }
    }

    // </Custom additional code>
}
