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
using System.IO;
using System.Linq;
using System.Data;
using System.Drawing;
using System.Reflection;
using System.Windows.Forms;
using System.Xml;
using System.Xml.Linq;
using System.Runtime.InteropServices;

using System.Text;
using System.Threading.Tasks;
using Rhino.DocObjects;
using Rhino.Collections;
using GH_IO;
using GH_IO.Serialization;
using KPlankton;
using KangarooSolver;
using KangarooSolver.Goals;


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


    private void RunScript(bool reset, bool iScatter, Point3d iAttractor, Curve iBoundary, int iBobCount, bool addParticles, ref object trails, ref object GoalListOut, ref object ParticleCountOut, ref object Debug1, ref object Debug2, ref object Debug3, ref object Debug4)
    {
        // <Custom code>
        
        
        //Initialiazie variables

        boundary = iBoundary;
        attractor = iAttractor;
        scatter = iScatter;



        if (reset || Bobbins == null)
        {
            //initialize the solver

            PS = new PhysicalSystem();
            PS.ClearParticles();
            GoalList.Clear();
            count = iBobCount;

            //reset the bobbin system
            Bobbins = new BobSystem(iBobCount);
            Bobbins.Reset();

            //reset variables
            //Spools = new List<Line>();
            //InitialRadii = new List<double>();

            List<Point3d> startingPoints = new List<Point3d>();
            SphereCollidePIndices = new List<int>();

            //LineIndices = new List<List<int>>();
            //LinePPos = new List<Point3d>();
            //LinePIndex = new List<int>();


            var tempList1 = new List<int>();
            var tempList2 = new List<Point3d>();

            foreach (Bobbin bob in Bobbins.bobs)
            {
                PS.AddParticle(bob.loc, .001);
                startingPoints.Add(bob.loc);
                SphereCollidePIndices.Add(bob.id);
                
                tempList1.Add(bob.id);
                tempList2.Add(bob.loc);

                //InitialRadii.Add(0.12);
                //Spools.Add(new Line(bob.loc, bob.loc + new Vector3d(0, 0, -.1)));

                //LineIndices.Add(new List<int> { bob.id, count + bob.id });
                //LinePPos.Add(bob.loc);
                //LinePPos.Add(bob.loc + new Vector3d(0, 0, .1));
                //LinePIndex.Add(bob.id);
                //LinePIndex.Add(count + bob.id);

            }

            
            SC = new SphereCollide(startingPoints, 0.12, 1);
            SC.PIndex = SphereCollidePIndices.ToArray();            
            GoalList.Add(SC);

            A = new Anchors(tempList1, tempList2, 10000000);
            GoalList.Add(A);

            //LC = new LineLineCollider(LineIndices, Spools, InitialRadii, 1.0);
            //PS.AssignPIndex(LC, .001);
            //GoalList.Add(LC);

     

            //goto Conclusion;
        }

        AnchorPoints = new List<Point3d>();
        AnchorPIndices = new List<int>();

        // When addParticles == false, no new particles are added but the system still relax.
        if (addParticles)
        {
            

            //Run Bob class
            Bobbins.Update();

            //var tempList = new List<int>();
            
            foreach (Bobbin bob in Bobbins.bobs)
            {
                int indexer = count + bob.id;

                //Add Particles
                PS.AddParticle(bob.trail[0], 0.001);
                
                //Add particle indices to the sphereCollide goal
                SphereCollidePIndices.Add(indexer);
                

                //Update the pINDEX of goal
                SC.PIndex = SphereCollidePIndices.ToArray();


                //Lock 1st ring of particles
                if (count == iBobCount)
                {
                    Anchor a = new Anchor(bob.id, bob.loc, 100000000);
                    GoalList.Add(a);

                }//end if



                //Unary Force

                Unary u = new Unary(indexer, forceVec);

                GoalList.Add(u);
                //PS.AssignPIndex(u, .001);

                //Horizontal Spring

                //Spring s = new Spring(indexer - iBobCount, indexer, 0.2, 1);
                Spring s = new Spring(indexer - iBobCount, indexer, 0.2, 50);
                s.Stiffness = 10;
                GoalList.Add(s);

                //create line
                //Line l = new Line(PS.GetPosition(indexer - iBobCount), PS.GetPosition(indexer));
                //Spools.Add(l);

                //InitialRadii.Add(.12);
                //LineIndices.Add(new List<int> { indexer - count, indexer });
                //LinePPos.Add(bob.trail[0]);
                //LinePPos.Add(bob.loc);
                //LinePIndex.Add(bob.id);
                //LinePIndex.Add(indexer);

                //LC.PPos = LinePPos.ToArray();
                //LC.PIndex = LinePIndex.ToArray();

                


                // endpoint as anchor

                AnchorPIndices.Add(indexer);
                AnchorPoints.Add(bob.trail[bob.trail.Count-1]);


                //Anchor b = new Anchor(indexer, bob.loc,100000000);
                //GoalList.Add(b);


            }

            //LC.Lines = Spools;


            A.Pts = AnchorPoints;
            //A.PPos = AnchorPoints.ToArray();
            A.PIndex = AnchorPIndices.ToArray();
            //A.PIndex = tempList.ToArray();

            // We also need to manually expand the Move and the Weighting arrays
            SC.Move = new Vector3d[count + iBobCount]; //CHECK THIS
            SC.Weighting = new double[count + iBobCount]; //CHECK THIS used to be c+2
            for (int i = 0; i < count + iBobCount; i++)
            {
                //Check
                SC.Weighting[i] = 1.0;
            }

            //LC.Radii = InitialRadii;
            //LC.Move = new Vector3d[(count + iBobCount)*2];
            //LC.Weighting = new double[(count + iBobCount) * 2];

        }


        Bobbins.Update();


        //Simulate
        PS.Step(GoalList, true, 1);


        //Output
        GoalListOut = PS.GetOutput(GoalList);
        ParticleCountOut = PS.ParticleCount();

        

        count += iBobCount;
        

        //visualize the data 

        //Conclusion:

        DataTree<Point3d> tree = new DataTree<Point3d>();
        List<Point3d> positions = new List<Point3d>();
        List<Vector3d> velocities = new List<Vector3d>();

        foreach (Bobbin bob in Bobbins.bobs)
        {
            positions.Add(bob.loc);
            velocities.Add(bob.Velocity);

            GH_Path gH_Path = new GH_Path(bob.id);

            foreach (Point3d point in bob.trail)
            {
                tree.Add(point, gH_Path);
            }
            

        }

        
        //List<Sphere> spheres = new List<Sphere>();
        //List<double> diams = new List<double>();

        //Point3d[] points = PS.GetPositionArray();
        //foreach (int i in SC.PIndex)

        //    spheres.Add(new Sphere(points[i], SC.Diam * 0.5));

        //diams.Add(SC.Diam);


        Debug1 = positions;
        Debug2 = velocities;

        //Debug3 = spheres;
        //Debug4 = diams;

        trails = tree;   


        // </Custom code>
    }


    // <Custom additional code>


    //Kangaroo Physics Engine
    public PhysicalSystem PS;
    public List<IGoal> GoalList = new List<IGoal>();
    public Vector3d forceVec = new Vector3d(0, 0, 0.0001);
    int count;


    SphereCollide SC = null;
    List<int> SphereCollidePIndices = null;

    Anchors A = null;
    List<Point3d> AnchorPoints = null;
    List<int> AnchorPIndices = null;

    //LineLineCollider LC = null;
    //List<Line> Spools = null;
    //List<double> InitialRadii = null;
    //List<List<int>> LineIndices = null;
    //List<Point3d> LinePPos = null;
    //List<int> LinePIndex = null;

    //initialize Bobbin system

    BobSystem Bobbins = new BobSystem();
    static Curve boundary;
    static Point3d attractor;
    static bool scatter;


    //Bob System
    class BobSystem
    {
        int bobCount;
        public List<Bobbin> bobs = new List<Bobbin>();
        public List<Point3d> initialLocations = new List<Point3d>();
        public int counter = 0;

        public BobSystem()
        {

        }
         
        public BobSystem(int _bobCount)
        {
            bobCount = _bobCount;
            


            for (int i = 0; i<_bobCount; i++)
            {
                var rand = new Random(i);
                Point3d randomInitialPosition = new Point3d( rand.NextDouble() * 30, rand.NextDouble() * 30, 0.0);
                initialLocations.Add(randomInitialPosition);
                
                Bobbin bob;

                bob = new Bobbin(randomInitialPosition,i);
                bob.system = this;
                
                bobs.Add(bob);
                
            }
        }


        public void Update()
        {
            foreach (Bobbin bob in bobs)
            {
                bob.ComputeDesiredVelocity();
            }
            foreach (Bobbin bob in bobs)
            {
                bob.Update();
            }
            counter++;
        }

        public void Reset()
        {
            foreach (Bobbin bob in bobs)
            {
                bob.Reset();
                counter = 0; 
            }
        }

    }
    
    
    //Bob
    class Bobbin
    {
        
        public int id;
        public Point3d loc;
        public Vector3d Velocity = new Vector3d(0.0,0.0,0.0);
        public double speed = 0.2;
        public List<Point3d> trail = new List<Point3d>();
        public BobSystem system;
        public Vector3d desiredVelocity;
        double z;


        public Bobbin(Point3d _loc, int _id)
        {
            loc = _loc;
            id = _id;

        }

        public void Update()
        {

            //Increment
            z += 0.1;
            //Velocity += desiredVelocity;
            Velocity = 0.9 * Velocity + 0.1 * desiredVelocity;

            if (Velocity.Length > 1)
            {
                Velocity.Unitize();
                Velocity *= 1;
            }


            //Point location
            loc += Velocity;
            //loc.Z = z;
            //All Points list

            int historyLength = 10;
            Point3d movedLocation = new Point3d(loc.X, loc.Y, loc.Z + z);

            if (trail.Count < historyLength)
            {
                
                trail.Add(movedLocation);
            }

            else
            {
                trail.RemoveAt(0);
                trail.Add(movedLocation);
            }

        }

        public void ComputeDesiredVelocity()
        {
            //Move toward an attractor
            if (!scatter)
            {
                Vector3d toAttractor = attractor - loc;
                toAttractor.Unitize();
                toAttractor *= 4.0; //replace with strength in the future
                desiredVelocity = toAttractor;
            }

            else
            {
                Vector3d toAttractor = loc - attractor;
                toAttractor.Unitize();
                toAttractor *= 1.0; //replace with strength in the future
                desiredVelocity = toAttractor;
            }
            


            //Avoid other agents

            foreach (Bobbin bobbin in system.bobs)
            {
                if (id == bobbin.id)
                {
                    continue;
                }

                Vector3d avoid = loc - bobbin.loc;
                double distance1 = avoid.Length;
                avoid.Unitize();
                avoid *= 1 / distance1; //replace with iSeparation strength/ distance
                desiredVelocity += avoid;
            }

            //Avoid Boundary

            double t;
            boundary.ClosestPoint(loc, out t, 10);
            Point3d pointOnCurve = boundary.PointAt(t);

            Vector3d getAway = loc - pointOnCurve;
            double distance2 = getAway.Length;
            getAway.Unitize();
            desiredVelocity += 1.0 * getAway/ (distance2 * distance2);


        }

        public void Reset()
        {
            
            trail.Clear();

        }


    }//End of Bob

   

    //Anchors goal
    public class Anchors : GoalObject
    {
        public double Strength;
        public List<Point3d> Pts;


        public Anchors()
        {

        }

        public Anchors (List<int> Ids, List<Point3d> Ps, double k)
        {
            int L = Ids.Count;
            PPos = Ps.ToArray();
            PIndex = Ids.ToArray();
            Pts = Ps;
            Move = new Vector3d[L];
            Weighting = new double[L];
            for (int i = 0; i < L; i++) 
            {
                Weighting[i] = k;
            }
            Strength = k;
            
            
        }
        public override void Calculate(List<KangarooSolver.Particle> p)
        {
            int L = PIndex.Length;
            

            for(int i = 0; i< L; i++)
            {
                Move[i] = Pts[i] - p[PIndex[i]].Position;
                Weighting[i] = Strength;
            }
        }
    }

    public class LineLineCollider : GoalObject
    {
        public double Strength;
        public List<Line> Lines;
        public List<double> Radii;
        public List<List<int>> ObjectIndices;
        
        public LineLineCollider()
        {

        }

        public LineLineCollider(List<Line> _Lines, List<double> _Radii, double k)
        {
            this.Lines = _Lines;
            this.Radii = _Radii;
            this.Strength = k;
            
            ObjectIndices = new List<List<int>>();
            var PPosList = new List<Point3d>();
            
            int IndexCounter = 0;

            for (int i = 0; i< Lines.Count; i++)
            {
                var L = Lines[i];
                PPosList.Add(L.From);
                PPosList.Add(L.To);
                ObjectIndices.Add(new List<int> { IndexCounter, IndexCounter + 1 });
                IndexCounter += 2;
            }

            PPos = PPosList.ToArray();
            Move = new Vector3d[PPos.Length];
            Weighting = new double[PPos.Length];

        }

        public LineLineCollider(List<List<int>> _Indices, List<Line> _Lines, List<double> _Radii, double k)
        {
            Lines = _Lines;
            Radii = _Radii;
            Strength = k;
            ObjectIndices = _Indices;

            var PPosList = new List<Point3d>();


            for (int i = 0; i < Lines.Count; i++)
            {
                var L = Lines[i];
                PPosList.Add(L.From);
                PPosList.Add(L.To);

            }

            PPos = PPosList.ToArray();
            Move = new Vector3d[PPos.Length];
            Weighting = new double[PPos.Length];

        }
        public override void Calculate(List<KangarooSolver.Particle> p)
        {
            // get all the boundingbox collisions
            // then calculate the actual collision response

            for (int i = 0; i< PPos.Length; i++)
            {
                Move[i] = Vector3d.Zero;
                Weighting[i] = 0;
            }

            var BoundingBoxes = new List<AABB>();

            for (int i = 0; i< Radii.Count; i++)
            {
                var Rad = Radii[i];

                var PtA = p[PIndex[ObjectIndices[i][0]]].Position;
                var PtB = p[PIndex[ObjectIndices[i][1]]].Position;
                var Low = new Point3d(
                      Math.Min(PtA.X, PtB.X) - Rad,
                      Math.Min(PtA.Y, PtB.Y) - Rad,
                      Math.Min(PtA.Z, PtB.Z) - Rad
                      );
                var High = new Point3d(
                  Math.Max(PtA.X, PtB.X) + Rad,
                  Math.Max(PtA.Y, PtB.Y) + Rad,
                  Math.Max(PtA.Z, PtB.Z) + Rad
                  );
                BoundingBoxes.Add(new AABB(Low, High, i));
            }

            var PotentialCollisions = SAPCollide(BoundingBoxes);

            for (int i = 0; i < PotentialCollisions.Count; i++)
            {
                int Ix0 = PotentialCollisions[i].Item1;
                int Ix1 = PotentialCollisions[i].Item2;

                int LAStartIndex = ObjectIndices[Ix0][0];
                int LAEndIndex = ObjectIndices[Ix0][1];
                int LBStartIndex = ObjectIndices[Ix1][0];
                int LBEndIndex = ObjectIndices[Ix1][1];

                var L1 = new Line(p[PIndex[LAStartIndex]].Position, p[PIndex[LAEndIndex]].Position);
                var L2 = new Line(p[PIndex[LBStartIndex]].Position, p[PIndex[LBEndIndex]].Position);

                Vector3d u = L1.To - L1.From;
                Vector3d v = L2.To - L2.From;
                Vector3d w = L1.From - L2.From;
                double a = u * u;
                double b = u * v;
                double c = v * v;
                double d = u * w;
                double e = v * w;
                double D = a * c - b * b;
                double sc, sN, sD = D;
                double tc, tN, tD = D;

                // compute the line parameters of the two closest points
                if (D < 1e-8)
                { // the lines are almost parallel
                    sN = 0.0;         // force using point P0 on segment S1
                    sD = 1.0;         // to prevent possible division by 0.0 later
                    tN = e;
                    tD = c;
                }
                else
                {
                    sN = b * e - c * d;
                    tN = a * e - b * d;

                    if (sN < 0.0)
                    {        // sc < 0 => the s=0 edge is visible
                        sN = 0.0;
                        tN = e;
                        tD = c;
                    }
                    else if (sN > sD)
                    {  // sc > 1  => the s=1 edge is visible
                        sN = sD;
                        tN = e + b;
                        tD = c;
                    }
                }

                if (tN < 0.0)
                {            // tc < 0 => the t=0 edge is visible
                    tN = 0.0;
                    // recompute sc for this edge
                    if (-d < 0.0)
                        sN = 0.0;
                    else if (-d > a)
                        sN = sD;
                    else
                    {
                        sN = -d;
                        sD = a;
                    }
                }
                else if (tN > tD)
                {      // tc > 1  => the t=1 edge is visible
                    tN = tD;
                    // recompute sc for this edge
                    if ((-d + b) < 0.0)
                        sN = 0;
                    else if ((-d + b) > a)
                        sN = sD;
                    else
                    {
                        sN = (-d + b);
                        sD = a;
                    }
                }

                // finally do the division to get sc and tc
                sc = (Math.Abs(sN) < 1e-8 ? 0.0 : sN / sD);
                tc = (Math.Abs(tN) < 1e-8 ? 0.0 : tN / tD);

                // get the difference of the two closest points
                Vector3d dP = w + (sc * u) - (tc * v);  // =  S1(sc) - S2(tc)


                var Separation = -dP;
                var Overlap = Radii[Ix0] + Radii[Ix1] - Separation.Length;

                if (Overlap > 0)
                {
                    Separation.Unitize();
                    var Push = 1 * Separation * Overlap;

                    Move[LAStartIndex] += (1 - sc) * -Push;
                    Move[LAEndIndex] += (sc) * -Push;

                    Move[LBStartIndex] += (1 - tc) * Push;
                    Move[LBEndIndex] += (tc) * Push;

                    Weighting[LAStartIndex] = Weighting[LAEndIndex] = Weighting[LBStartIndex] = Weighting[LBEndIndex] = Strength;
                }
            }

        }


        public static List<Tuple<int, int>> SAPCollide(List<AABB> boxes)
        {

            //this will send out the branch references for all the boxes that collide
            // List<int> CollideRef0 = new List<int>();
            // List<int> CollideRef1 = new List<int>();

            var CollideRefs = new List<Tuple<int, int>>();

            List<EndPoint> sortedInX = new List<EndPoint>();

            //add the endpoints in x
            foreach (AABB boxForPoint in boxes)
            {
                sortedInX.Add(boxForPoint.min[0]);//change this 0 to 1 or 2 to sort in Y or Z - will also need to change the second half of the test function
                sortedInX.Add(boxForPoint.max[0]);//change this 0 to 1 or 2 to sort in Y or Z
            }

            //sort by the num value of each EndPoint
            sortedInX.Sort(EndPoint.compareEndPoints);

            //this could be a list of intgers?
            List<int> openBoxes = new List<int>();

            foreach (EndPoint endPoint in sortedInX)
            {
                if (endPoint.isMin)
                {
                    AABB thisPointOwner = endPoint.owner;
                    //check against all in openBoxes
                    foreach (int openBoxRef in openBoxes)
                    {
                        //if it collides output the integers of the branches that collide
                        //do they collide in y?
                        if (thisPointOwner.max[1].num > boxes[openBoxRef].min[1].num && thisPointOwner.min[1].num < boxes[openBoxRef].max[1].num)
                        {
                            //they collide in y, do they collide in z?
                            if (thisPointOwner.max[2].num > boxes[openBoxRef].min[2].num && thisPointOwner.min[2].num < boxes[openBoxRef].max[2].num)
                            {
                                //they collide in z
                                //therefore they collide! Add to the list of collide refs
                                //make sure the lowest index comes first to make searching easier later
                                int CollideA = endPoint.owner.branchRef;
                                int CollideB = boxes[openBoxRef].branchRef;
                                CollideRefs.Add(new Tuple<int, int>(Math.Min(CollideA, CollideB), Math.Max(CollideA, CollideB)));
                            }
                        }
                    }
                    //add corresponding box to openBoxes
                    openBoxes.Add(thisPointOwner.branchRef);

                }
                else
                {
                    //it must be an max point
                    //remove corresponding box from openBoxes
                    openBoxes.Remove(endPoint.owner.branchRef);
                }
            }

            return CollideRefs;
        }

        public class AABB
        {
            public EndPoint[] min;//an array of size 3 with the x,y,z value for the AABB min
            public EndPoint[] max;//an array of size 3 with the x,y,z value for the AABB max
            public int branchRef;

            //constructor
            public AABB(Point3d tMin, Point3d tMax, int tBranchRef)
            {
                min = new EndPoint[] { new EndPoint(tMin.X, true, this), new EndPoint(tMin.Y, true, this), new EndPoint(tMin.Z, true, this) };
                max = new EndPoint[] { new EndPoint(tMax.X, false, this), new EndPoint(tMax.Y, false, this), new EndPoint(tMax.Z, false, this) };
                branchRef = tBranchRef;
            }
        }

        public class EndPoint
        {
            public AABB owner;
            public double num;//its actual value - corresponds to the x,y or z value
            public bool isMin;//to distinguish whether this is a min point or a max point

            //constructor
            public EndPoint(double tNum, bool tIsMin, AABB tOwner)
            {
                num = tNum;
                isMin = tIsMin;
                owner = tOwner;
            }

            //used as a comparison to sort the endpoints
            public static int compareEndPoints(EndPoint x, EndPoint y)
            {
                return x.num.CompareTo(y.num);
            }
        }
    }

    // </Custom additional code>
}
