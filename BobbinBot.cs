using Rhino.Geometry;
using System.Collections.Generic;
using ICD.AbmFramework.Core.Environments;
using ICD.AbmFramework.Core.Agent;
using ICD.AbmFramework.Core.AgentSystem;
using ICD.AbmFramework.Core.Behavior;

namespace ICD.AbmFramework.Core.Agent
{
    public enum BotState
    {
        Free = 0,
        Twist = 1,
        Cross = 2,
        FindPartner = 3, 
        GoTo = 4,
        Stop = 5,
    }

    public class BobbinBot : AgentBase
    {
        // ABM parameters
        public new BobbinBotSystem AgentSystem;
        public Point3d Position;
        public double HeadingAngle;
        public Vector3d Velocity;
        public Vector3d Force;
        public Point3d StartPosition;
        protected Vector3d startVelocity;

        // bot state
        public bool paired;
        public bool leftActive = false;
        public bool rightActive = false;
        public BotState State = BotState.Free;
        public Point3d Target;

        // how much twists and crosses it likes
        public int twistCount;
        public int crossCount;
        public int totalTwists;
        public int totalCrosses;

        // partner information
        public int intendedPartnerIndex = -1;
        public int committedPartnerIndex = -1;
        public Point3d partnerStartPosition;

        // bobbin information
        public int leftIndex;
        public int rightIndex;
        private Vector3d leftOffset = new Vector3d(-10.0, 0, 0);
        private Vector3d rightOffset = new Vector3d(10.0, 0, 0);
        
        public BobbinBot(Point3d position, Vector3d velocity, List<BehaviorBase> behaviors)
        {
            StartPosition = Position = position;
            startVelocity = Velocity = velocity;
            Behaviors = behaviors;
            CustomData["current_target"] = "";
            CustomData["current_state"] = "free";
        }

        public override void Reset()
        {
            State = BotState.Free;
            paired = false;
            leftActive = false;
            rightActive = false;
            State = BotState.Free;
            Position = StartPosition;
            Velocity = startVelocity;
            Force = Vector3d.Zero;
            intendedPartnerIndex = -1;
            committedPartnerIndex = -1;
            CustomData["current_state"] = "free";
            CustomData["current_target"] = "";
        }

        public override void PreExecute()
        {
            this.Force = Vector3d.Zero;
        }

        public override void Execute()
        {
            foreach (BehaviorBase behavior in Behaviors)
                behavior.Execute(this);
        }

        public override void PostExecute()
        {
            if ((this.State == BotState.Twist) || (this.State == BotState.Cross) || (this.State == BotState.Stop))
            {
                // Twist: implemented in twist behaviour
                // Cross: implemented in cross behaviour                
            }
            else if (this.State == BotState.GoTo)
            {
            }
            else //FindPartner, Free
            {
                BobbinBotSystem thisAgentSystem = (BobbinBotSystem)this.AgentSystem;

                if (Force.Length > thisAgentSystem.MaxForce) Force *= thisAgentSystem.MaxForce / Force.Length;
                Velocity += Force * thisAgentSystem.TimeStep;

                if (Velocity.Length > thisAgentSystem.MaxSpeed) Velocity *= thisAgentSystem.MaxSpeed / Velocity.Length;
                Position += Velocity * thisAgentSystem.TimeStep;

                this.HeadingAngle = Vector3d.VectorAngle(Velocity, Vector3d.YAxis);
            }

            // carry the bobbins along
            if (leftActive)
            {
                this.AgentSystem.BobbinEnvironment.Bobbins[leftIndex] = Position + 
                    new Vector3d(leftOffset.X*System.Math.Cos(this.HeadingAngle), leftOffset.X * System.Math.Sin(this.HeadingAngle), leftOffset.Z);
            }
            if (rightActive)
            {
                this.AgentSystem.BobbinEnvironment.Bobbins[rightIndex] = Position +
                    new Vector3d(rightOffset.X * System.Math.Cos(this.HeadingAngle), rightOffset.X * System.Math.Sin(this.HeadingAngle), leftOffset.Z);
            }

            BehaviourArbitration();
        }

        protected void BehaviourArbitration()
        {
            // create some rules in the future for how behaviours transition to/from each other

            // if the bot is empty and free: 
            //      seek bobbins (AttractionBehaviour)
            //      *go to a specific bobbin (GoToBehaviour)

            // if the bot has one bobbin and free: 
            //      seek bobbins (AttractionBehaviour)
            //      *go to a specific bobbin (GoToBehaviour)

            // if the bot is full, but has not paired with another, the bot can: 
            //      do a twist (TwistBehaviour)
            //      find partner organically (FindPartnerBehaviour)
            //      *go to specific position before starting braid or rendezvous with a specific partner (GoToBehaviour)

            // if the bot is full and has paired with another: 
            //      do a twist (TwistBehaviour)
            //      do a cross (CrossBehaviour)
            //      *go to a specific position before starting braid (GoToBehaviour)

            // Avoidance behaviour is always active
        }

        public void AddForce(Vector3d force)
        {
            this.Force += force;
        }

        public void SetHeading(double headingAngle)
        {
            this.HeadingAngle = headingAngle;
        }

        public void SetState(BotState state)
        {
            this.State = state;
        }

        public void SetTarget(Point3d target)
        {
            this.Target = target;
        }

        public void GrabBobbin(int target, bool isLeft)
        {
            if (isLeft)
            {
                this.leftIndex = target;
                this.AgentSystem.BobbinEnvironment.GrabBobbin(target);
                this.leftActive = true;
                this.CustomData["current_target"] = target.ToString();
            }
            else
            {
                this.rightIndex = target;
                this.AgentSystem.BobbinEnvironment.GrabBobbin(target);
                this.rightActive = true;
                this.CustomData["current_target"] += "&" + target.ToString();
            }
        }

        public void ReleaseBobbin(bool isLeft)
        {
            if (isLeft)
            {
                this.AgentSystem.BobbinEnvironment.FreeBobbin(this.leftIndex);
                this.leftIndex = -1;
                this.leftActive = false;
            }
            else
            {
                this.AgentSystem.BobbinEnvironment.FreeBobbin(this.rightIndex);
                this.rightIndex = -1;
                this.rightActive = false;
            }
        }

        public override List<object> GetDisplayGeometries()
        {
            List<object> objects = new List<object>
            {
                this.Position
            };
            if (leftActive)
            {
                objects.Add(this.AgentSystem.BobbinEnvironment.Bobbins[leftIndex]);
            }
            if (rightActive)
            {
                objects.Add(this.AgentSystem.BobbinEnvironment.Bobbins[rightIndex]);
            }
            return new List<object>() { this.Position };
        }
    }

}